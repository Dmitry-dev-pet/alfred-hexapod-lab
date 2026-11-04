"""Freenove FNK0052 official servo channel map (code-level reference only).

Maps the vendor's six logical legs to the 18 channel numbers and applies the
vendor's right/left servo-angle conventions. This is NOT a mapping from
CAD MuJoCo qpos to PWM: the physical zero and sign of each MJCF joint remain
unverified. Never send these outputs directly to a real robot.

Source:
https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi/blob/b7d228cc870b0a802d3768bfcf742f85fa8695f6/Code/Server/control.py
"""
from __future__ import annotations

from math import isfinite
from typing import Iterable

# Official vendor leg order: front right, middle right, rear right,
# rear left, middle left, front left. The shortened names correspond to
# the original onshape-derived CAD bodies coxa*, femur*, tibia*.
VENDOR_LEGS = ("fr", "mr", "br", "bl", "ml", "fl")
SERVO_CHANNELS = {
    "fr": (15, 14, 13),
    "mr": (12, 11, 10),
    "br": (9, 8, 31),
    "bl": (22, 23, 27),
    "ml": (19, 20, 21),
    "fl": (16, 17, 18),
}
# IMPORTANT: the upstream Onshape/URDF names are mirrored relative to
# Freenove control.py's vendor leg numbering. This mapping is established
# by the motor joint names, not by guessing from "left" or "right".
# E.g. CAD front-left ("fl") has channels (15,14,13): vendor front-right.
CAD_TO_VENDOR_LEG = {
    "fr": "fl", "mr": "ml", "br": "bl",
    "bl": "br", "ml": "mr", "fl": "fr",
}
CHANNEL_KINDS = ("coxa", "femur", "tibia")
SOURCE_REF = "Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi@b7d228cc870b0a802d3768bfcf742f85fa8695f6"


def channel_map() -> dict[str, dict[str, int]]:
    assert len(SERVO_CHANNELS) == 6
    values = [n for channels in SERVO_CHANNELS.values() for n in channels]
    if len(values) != 18 or len(set(values)) != 18:
        raise ValueError("Duplicate or missing Freenove servo channel")
    return {
        leg: dict(zip(CHANNEL_KINDS, SERVO_CHANNELS[leg]))
        for leg in VENDOR_LEGS
    }


def vendor_angles_to_servo_degrees(
    leg: str, angles_deg: Iterable[float],
    calibration_deg: Iterable[float] = (0., 0., 0.),
) -> tuple[float, float, float]:
    """Apply set_leg_angles() angle convention from Freenove control.py.

    Here angles_deg are the VENDOR inverse-kinematics angles (a, b, c);
    they are NOT MuJoCo qpos. Per-unit calibration data are intentionally
    required before any hardware use; default zeros are only for tests.
    """
    if leg not in SERVO_CHANNELS:
        raise ValueError("Unknown Freenove leg")
    angles, calibration = tuple(angles_deg), tuple(calibration_deg)
    if len(angles) != 3 or len(calibration) != 3:
        raise ValueError("Expected exactly 3 joint angles and 3 calibration values")
    if any(not isfinite(v) for v in angles + calibration):
        raise ValueError("Non-finite servo input")
    a, b, c = angles
    ca, cb, cc = calibration
    if leg in VENDOR_LEGS[:3]:
        raw = (a + ca, 90 - (b + cb), c + cc)
    else:
        raw = (a + ca, 90 + b + cb, 180 - (c + cc))
    return tuple(max(0., min(180., v)) for v in raw)


def vendor_leg_for_cad(cad_suffix: str) -> str:
    """Find vendor wiring equivalent for a community CAD body suffix."""
    if set(CAD_TO_VENDOR_LEG) != set(VENDOR_LEGS) or set(CAD_TO_VENDOR_LEG.values()) != set(VENDOR_LEGS):
        raise ValueError("Incomplete CAD/vendor wiring remap")
    try:
        return CAD_TO_VENDOR_LEG[cad_suffix]
    except KeyError as exc:
        raise ValueError(f"Unknown CAD leg suffix: {cad_suffix}") from exc


def cad_joint_names(leg: str) -> tuple[str, str, str]:
    mapping = channel_map()[leg]
    return tuple(f"motor{mapping[k]}" for k in CHANNEL_KINDS)


if __name__ == "__main__":
    import json
    print(json.dumps(channel_map(), indent=2))
