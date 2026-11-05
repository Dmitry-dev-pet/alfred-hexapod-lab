"""Non-blocking six-leg velocity gait for the FNK0052 CAD30 simulation.

The pure-Python controller never sleeps, simulates physics, commands PWM, or
writes root qpos. It produces *body-frame foot offsets* relative to a recorded
six-leg stance. A separate adapter supplies IK, limited motor torques and actual
MuJoCo contacts. This is a classical open-loop gait generator with a command
watchdog, not an RL policy or a proven stable physical controller.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi, sin

# Source CAD naming, NOT the mirrored vendor left/right names.
LEGS = ("ml", "bl", "br", "mr", "fr", "fl")


@dataclass(frozen=True)
class Twist:
    """Body coordinates: x forward, y left, yaw positive CCW (SI units)."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def __post_init__(self) -> None:
        if not all(isfinite(float(v)) for v in (self.vx, self.vy, self.wz)):
            raise ValueError("Velocity commands must be finite")

    @property
    def moving(self) -> bool:
        return any(abs(v) > 1e-9 for v in (self.vx, self.vy, self.wz))


@dataclass(frozen=True)
class GaitFrame:
    time: float
    phase: float
    body_velocity: Twist
    foot_offsets: dict[str, tuple[float, float, float]]
    scheduled_swing: tuple[str, ...]
    mode: str


def _clamp(x: float, limit: float) -> float:
    return max(-limit, min(limit, x))


def _smoothstep(u: float) -> float:
    u = max(0.0, min(1.0, u))
    return u * u * (3 - 2 * u)


class LiveGait:
    """Tick-based tripod or wave scheduler with mid-cycle command replacement.

    ``anchors_xy`` are nominal foot-tip coordinates in the *body* frame in
    metres, measured at the already physically settled six-leg stance.
    Limits are conservative research defaults, not vendor motor ratings.
    """

    def __init__(
        self,
        anchors_xy: dict[str, tuple[float, float]],
        *,
        gait: str = "wave",
        period_s: float = 2.4,
        foot_lift_m: float = 0.012,
        max_stride_m: float = 0.035,
        max_foot_speed_m_s: float = 0.10,
        command_timeout_s: float = 0.5,
    ) -> None:
        if gait not in ("wave", "tripod"):
            raise ValueError("Expected wave or tripod gait")
        if set(anchors_xy) != set(LEGS):
            raise ValueError("Expected exactly six CAD leg anchors")
        self.anchors = {name: tuple(map(float, anchors_xy[name])) for name in LEGS}
        if not all(isfinite(v) for pair in self.anchors.values() for v in pair):
            raise ValueError("Nonfinite anchor")
        if not (0 < period_s and 0 < foot_lift_m <= .04 and
                0 < max_stride_m <= .08 and max_foot_speed_m_s > 0 and
                command_timeout_s > 0):
            raise ValueError("Invalid gait parameters")
        self.gait = gait
        self.period_s = period_s
        self.foot_lift = foot_lift_m
        self.max_stride = max_stride_m
        self.max_foot_speed = max_foot_speed_m_s
        self.timeout = command_timeout_s
        self.time = 0.0
        self.phase = 0.0
        self.velocity = Twist()
        self._command = Twist()
        self._last_command_time = float("-inf")
        self._active = False
        self._stopping = False
        self._finishing = False
        self._offsets = {name: (0.0, 0.0, 0.0) for name in LEGS}

    def set_command(self, command: Twist) -> None:
        """Replace desired speed immediately; never wait for a gait boundary."""
        if not isinstance(command, Twist):
            raise TypeError("command must be Twist")
        self._command = command
        self._last_command_time = self.time
        if command.moving:
            self._active = True
            self._stopping = False
            self._finishing = False
        elif self._active:
            self._stopping = True

    def tick(self, dt: float) -> GaitFrame:
        if not isfinite(dt) or dt <= 0 or dt > .1:
            raise ValueError("Expected finite controller dt in (0, 0.1]")
        self.time += dt
        fresh = self.time - self._last_command_time <= self.timeout
        raw = self._command if fresh else Twist()
        # Control *acceleration* instead of discontinuously changing stride.
        target = Twist(_clamp(raw.vx, .04), _clamp(raw.vy, .04),
                       _clamp(raw.wz, .35))
        old = self.velocity
        self.velocity = Twist(
            old.vx + _clamp(target.vx - old.vx, .08 * dt),
            old.vy + _clamp(target.vy - old.vy, .08 * dt),
            old.wz + _clamp(target.wz - old.wz, .6 * dt),
        )
        if not target.moving and self._active:
            self._stopping = True
        if self._active and not self._finishing:
            old_phase = self.phase
            self.phase = (self.phase + dt / self.period_s) % 1.0
            # Complete current cycle before settling all legs to ground.
            if self._stopping and self.phase < old_phase:
                self._finishing = True
                self.phase = 0.0

        swing = []
        output = {}
        for i, name in enumerate(LEGS):
            x, y = self.anchors[name]
            if self._active and not self._finishing:
                # Local foot velocity induced by a body planar twist.
                local_vx = self.velocity.vx - self.velocity.wz * y
                local_vy = self.velocity.vy + self.velocity.wz * x
                step_x = _clamp(local_vx * self.period_s, self.max_stride)
                step_y = _clamp(local_vy * self.period_s, self.max_stride)
                duty = 1 / 6 if self.gait == "wave" else .5
                offset = i / 6 if self.gait == "wave" else (i % 2) * .5
                leg_phase = (self.phase + offset) % 1.0
                if leg_phase < duty:
                    u = leg_phase / duty
                    horizontal = _smoothstep(u) - .5
                    lift = self.foot_lift * sin(pi * u)
                    swing.append(name)
                else:
                    u = (leg_phase - duty) / (1.0 - duty)
                    horizontal = .5 - u
                    lift = 0.0
                ideal = (step_x * horizontal, step_y * horizontal, lift)
            else:
                ideal = (0.0, 0.0, 0.0)
            prior = self._offsets[name]
            # Per-axis slew limiting prevents a foot position discontinuity
            # when the joystick changes direction in the middle of swing.
            cap = self.max_foot_speed * dt
            nxt = tuple(p + _clamp(v - p, cap) for p, v in zip(prior, ideal))
            self._offsets[name] = nxt
            output[name] = nxt
        if self._finishing and all(max(map(abs, p)) < 1e-5 for p in output.values()):
            self._active = False
            self._stopping = False
            self._finishing = False
            self._offsets = {name: (0., 0., 0.) for name in LEGS}
            output = self._offsets.copy()
        return GaitFrame(
            self.time, self.phase, self.velocity, output, tuple(swing),
            "settling" if self._finishing else "walking" if self._active else "stand",
        )
