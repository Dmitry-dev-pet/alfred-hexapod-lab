"""Evidence-first v0.4 physical reconstruction of Freenove FNK0052.

This module does NOT build a calibrated MuJoCo model. It prevents an incomplete
mass budget or a kinematic reference being silently treated as physical truth.
Pure Python, independent of MuJoCo.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

DEFAULT_MANIFEST = Path(__file__).resolve().parents[1] / "data" / "fnk0052_v04_manifest.json"


def load_manifest(path: str | Path = DEFAULT_MANIFEST) -> dict:
    """Read the authoritative v0.4 evidence manifest."""
    with Path(path).open(encoding="utf-8") as stream:
        result = json.load(stream)
    validate_manifest(result)
    return result


def validate_manifest(doc: dict) -> None:
    """Fail closed on missing provenance, duplicate items or invented mass."""
    if doc.get("schema_version") != "0.4-draft-1":
        raise ValueError("Unknown v0.4 manifest version")
    sources = doc["sources"]
    if "vendor_control" not in sources or not sources["vendor_control"].get("url"):
        raise ValueError("Official control code provenance is required")
    for name, datum in doc["geometry"].items():
        if datum["source"] is not None and datum["source"] not in sources:
            raise ValueError(f"Unknown source for {name}")
        kind, value = datum["kind"], datum["value"]
        if kind == "unverified":
            if value is not None:
                raise ValueError(f"Unverified {name} must have null value")
        elif kind == "documented_in_vendor_control_code":
            if value is None or datum["source"] != "vendor_control":
                raise ValueError(f"Invalid documented datum {name}")
        else:
            raise ValueError(f"Unsupported evidence kind: {kind}")
    geo = doc["geometry"]
    links = geo["effective_leg_link_lengths_mm"]["value"]
    if len(links) != 3 or any(not isinstance(v, (float, int)) or v <= 0 for v in links):
        raise ValueError("Three positive kinematic link lengths required")
    for name in ("default_foot_reference_points_mm", "body_structure_reference_points_mm"):
        points = geo[name]["value"]
        if len(points) != 6 or any(len(point) != 3 for point in points):
            raise ValueError(f"Six 3D reference points required: {name}")
    for name in ("leg_frame_transform_angles_deg", "leg_radial_transform_offsets_mm"):
        if len(geo[name]["value"]) != 6:
            raise ValueError(f"Six leg entries required: {name}")
    seen = set()
    for part in doc["mass_ledger"]:
        part_id = part["id"]
        if part_id in seen:
            raise ValueError(f"Duplicate mass part: {part_id}")
        seen.add(part_id)
        if not isinstance(part["count"], int) or isinstance(part["count"], bool) or part["count"] < 1:
            raise ValueError(f"Invalid count: {part_id}")
        if part["source"] is not None and part["source"] not in sources:
            raise ValueError(f"Unknown mass source for {part_id}")
        mass = part["mass_each_kg"]
        if mass is not None:
            if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0:
                raise ValueError(f"Invalid mass for {part_id}")
            if part["source"] is None:
                raise ValueError(f"Mass needs its own provenance: {part_id}")
    for name, config in doc["configurations"].items():
        if not config["components"] or len(set(config["components"])) != len(config["components"]):
            raise ValueError(f"Invalid component list in {name}")
        if set(config["components"]) - seen:
            raise ValueError(f"Unknown component(s) in {name}")
    if set(doc["evidence_gates"].values()) - {True, False}:
        raise ValueError("Evidence gates must be boolean")


def mass_report(doc: dict, configuration: str = "stock_pi") -> dict:
    """Account only sourced measured/estimated masses; never invent a remainder.

    The subtotal of known parts is not a robot mass. total_mass_kg stays None
    until every included component has a mass with provenance.
    """
    validate_manifest(doc)
    config = doc["configurations"][configuration]
    parts = {item["id"]: item for item in doc["mass_ledger"]}
    known = 0.0
    missing = []
    for name in config["components"]:
        p = parts[name]
        if p["mass_each_kg"] is None:
            missing.append(name)
        else:
            known += p["count"] * p["mass_each_kg"]
    complete = not missing
    return {
        "configuration": configuration,
        "component_count": len(config["components"]),
        "known_subtotal_kg": round(known, 6),
        "unknown_components": missing,
        "mass_budget_complete": complete,
        "total_mass_kg": round(known, 6) if complete else None,
    }


def ready_for_physics_comparison(doc: dict, configuration: str = "stock_pi") -> bool:
    """A strict acceptance gate: v0.4 is NOT yet a validated digital twin."""
    return all(doc["evidence_gates"].values()) and mass_report(doc, configuration)["mass_budget_complete"]


def manufacturer_leg_fk_mm(
    angle_a_deg: float, angle_b_deg: float, angle_c_deg: float,
    lengths_mm: tuple[float, float, float] = (33.0, 90.0, 110.0),
) -> tuple[float, float, float]:
    """Original Freenove control.py angle_to_coordinate() convention.

    Kinematic *reference* only: joint physical poses and mass distribution
    are unverified. Returns (x, y, z) in Freenove leg-local coordinates, mm.
    """
    a, b, c = map(math.radians, (angle_a_deg, angle_b_deg, angle_c_deg))
    l1, l2, l3 = lengths_mm
    radius = l1 + l2 * math.cos(b) + l3 * math.cos(b + c)
    return (
        l2 * math.sin(b) + l3 * math.sin(b + c),
        math.sin(a) * radius,
        math.cos(a) * radius,
    )


if __name__ == "__main__":
    reference = load_manifest()
    print(json.dumps({
        "state": reference["state"],
        "mass_budget": {name: mass_report(reference, name) for name in reference["configurations"]},
        "ready_for_physics_comparison": ready_for_physics_comparison(reference),
    }, indent=2, ensure_ascii=False))
