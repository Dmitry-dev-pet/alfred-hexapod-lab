"""Reproducible sensitivity estimates from attributed community FNK0052 CAD.

CAUTION: This is NOT an assembled mass measurement or calibrated inertia model.
The community URDF already includes 18 large and 2 small servo meshes.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REF_PATH = ROOT / "data" / "fnk0052_community_urdf_reference.json"
SCENARIOS_PATH = ROOT / "data" / "fnk0052_community_mass_scenarios.json"


def load(path: str | Path) -> dict:
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def validate(ref: dict, scenario: dict) -> None:
    """Verify topology/provenance, check mass table and no double-counting."""
    if ref["schema_version"] != "fnk0052_community_urdf_reference/1":
        raise ValueError("Wrong CAD reference")
    if scenario["schema_version"] != "fnk0052_community_mass_scenarios/1":
        raise ValueError("Wrong mass scenario schema")
    if len(ref["links"]) != 25 or len(ref["joints"]) != 24:
        raise ValueError("Unexpected number of CAD joints/links")
    if sum(x["type"] == "revolute" for x in ref["joints"]) != 20:
        raise ValueError("Expected 18 leg + 2 head revolute joints")
    s = {
        "big": sum(x["servo_mesh_instances"]["big"] for x in ref["links"]),
        "small": sum(x["servo_mesh_instances"]["small"] for x in ref["links"]),
    }
    if s != {"big": 18, "small": 2}:
        raise ValueError(f"Unexpected servo visual count: {s}")
    source_sum = sum(x["mass_kg"] for x in ref["links"])
    if not math.isclose(source_sum, scenario["reference_cad_urdf_mass_kg"], abs_tol=1e-6):
        raise ValueError("URDF mass vs scenario source mismatch")
    if any(
        x["mass_kg"] is None or x["inertia_kg_m2"] is None
        for x in ref["links"]
    ):
        raise ValueError("Missing CAD source inertial entry")
    source_keys = set(scenario["sources"])
    seen = set()
    for row in scenario["calculations"]["rows"]:
        name = row["id"]
        if name in seen or row["count"] < 1:
            raise ValueError(f"Invalid or duplicated group {name}")
        seen.add(name)
        if not set(row["source"]).issubset(source_keys):
            raise ValueError(f"Unknown evidence key for {name}")
        masses = [row[f"{case}_kg"] for case in ("lower", "nominal", "upper")]
        if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in masses):
            raise ValueError(f"Invalid mass scenario value {name}")
        if not (0 < masses[0] <= masses[1] <= masses[2]):
            raise ValueError(f"Unordered range for {name}")
    names = {
        "cad_static_chassis_and_holders", "coxa_group_per_leg",
        "femur_group_per_leg", "tibia_group_per_leg", "head_pan_tilt_assembly",
        "18650_high_current_cells", "raspberry_pi_4_board",
        "not_in_CAD_control_electronics_wires_screws",
    }
    if seen != names:
        raise ValueError("Unexpected mass budget parts")
    expected_counts = {
        "cad_static_chassis_and_holders": 1, "coxa_group_per_leg": 6,
        "femur_group_per_leg": 6, "tibia_group_per_leg": 6,
        "head_pan_tilt_assembly": 1, "18650_high_current_cells": 4,
        "raspberry_pi_4_board": 1,
        "not_in_CAD_control_electronics_wires_screws": 1,
    }
    for row in scenario["calculations"]["rows"]:
        if row["count"] != expected_counts[row["id"]]:
            raise ValueError(f"Wrong count for {row['id']}")
    actual = totals(scenario)
    reported = scenario["calculations"]["scenario_totals_kg"]
    for key, value in actual.items():
        if not math.isclose(value, reported[key], abs_tol=1e-5):
            raise ValueError(f"Non-reproducible {key} total {value} != {reported[key]}")
    if scenario["is_measured_mass"] is not False:
        raise ValueError("Community scenario may not be marked measured")


def totals(scenario: dict) -> dict:
    return {
        key: round(sum(x["count"] * x[f"{key}_kg"]
                       for x in scenario["calculations"]["rows"]), 6)
        for key in ("lower", "nominal", "upper")
    }


def reference_breakdown(ref: dict) -> dict:
    """CAD-derived masses, including servos once, before any correction."""
    breakdown = {"base": 0.0, "coxa": 0.0, "femur": 0.0, "tibia": 0.0, "head": 0.0, "virtual": 0.0}
    for x in ref["links"]:
        name = x["name"]
        group = ("base" if name == "base_link_2" else
                 "coxa" if name.startswith("coxa") else
                 "femur" if name.startswith("femur") else
                 "tibia" if name.startswith("tibia") else
                 "head" if name in ("neck", "face") else "virtual")
        breakdown[group] += x["mass_kg"]
    return {k: round(v, 9) for k, v in breakdown.items()}


if __name__ == "__main__":
    ref, scenario = load(REF_PATH), load(SCENARIOS_PATH)
    validate(ref, scenario)
    print(json.dumps({
        "source_cad_mass_kg": scenario["reference_cad_urdf_mass_kg"],
        "source_cad_breakdown_kg": reference_breakdown(ref),
        "full_assembly_mass_scenarios_kg": totals(scenario),
        "measured": scenario["is_measured_mass"],
        "physical_com_and_inertias_calibrated": False,
        "warnings": scenario["warnings"],
    }, indent=2, ensure_ascii=False))
