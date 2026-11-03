"""A 2-D static support-polygon proxy for Freenove FNK0052.

IMPORTANT: This is NOT a rigid-body/servo/ground-contact simulation.
The horizontal motion of the body is prescribed; the simulator cannot predict
whether the motors can actually generate it. Outputs are support-margins only.

Coordinates/leg lengths follow Freenove's Code/Server/control.py (2026-11-03).
The base foot positions are a simplification of body_points in that file.
No Freenove source code is copied here.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import hypot
from typing import Iterable

# Nominal foot locations (metres), around the body, order follows Freenove body_points.
FOOT_XY = (
    (0.1371, 0.1894), (0.2250, 0.0), (0.1371, -0.1894),
    (-0.1371, -0.1894), (-0.2250, 0.0), (-0.1371, 0.1894),
)
LINK_LENGTH_M = (0.033, 0.090, 0.110)
TRIPOD_GROUP_A = frozenset((0, 2, 4))
TRIPOD_GROUP_B = frozenset((1, 3, 5))


def cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def convex_hull(points: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_margin(com: tuple[float, float], feet: Iterable[tuple[float, float]]) -> float:
    """Minimum signed distance (m) from centre of mass to support polygon edges.

    Positive means inside polygon; negative means outside; -inf if <3 supports.
    This is a *geometric* proxy, ignoring forces and dynamics.
    """
    hull = convex_hull(feet)
    if len(hull) < 3:
        return float('-inf')
    margins = []
    for i, a in enumerate(hull):
        b = hull[(i+1) % len(hull)]
        length = hypot(b[0]-a[0], b[1]-a[1])
        margins.append(cross(a,b,com)/length)
    return min(margins)


@dataclass
class Leg:
    index: int
    planted_xy: tuple[float, float]
    swinging: bool = False
    swing_elapsed: float = 0.0
    stance_elapsed: float = 0.0
    disabled: bool = False
    swing_count: int = 0


class Walker:
    def __init__(self, strategy: str, velocity: float = 0.020,
                 step_ahead: float = 0.018):
        if strategy not in ('tripod', 'decentralized'):
            raise ValueError(strategy)
        self.strategy = strategy
        self.velocity = velocity  # m/s: imposed body progress, not a prediction
        self.step_ahead = step_ahead
        self.legs = [Leg(i, p) for i, p in enumerate(FOOT_XY)]
        self.body_x = 0.0
        self.t = 0.0
        self.previous_phase = None
        self.touchdown_events = []

    def lift(self, leg: Leg):
        if leg.disabled or leg.swinging:
            return
        leg.swinging = True
        leg.swing_elapsed = 0.0
        leg.swing_count += 1

    def land(self, leg: Leg):
        leg.swinging = False
        leg.swing_elapsed = 0.0
        leg.stance_elapsed = 0.0
        leg.planted_xy = (self.body_x + FOOT_XY[leg.index][0] + self.step_ahead,
                          FOOT_XY[leg.index][1])
        self.touchdown_events.append((self.t, leg.index))

    def support_feet(self, omit=()):
        return [leg.planted_xy for leg in self.legs
                if not leg.disabled and not leg.swinging and leg.index not in omit]

    def margin(self):
        return support_margin((self.body_x, 0.0), self.support_feet())

    def advance(self, dt: float, broken_leg: int | None = None):
        self.t += dt
        self.body_x += self.velocity * dt
        if broken_leg is not None:
            leg = self.legs[broken_leg]
            leg.disabled = True
            leg.swinging = False
        for leg in self.legs:
            if leg.disabled:
                continue
            if leg.swinging:
                leg.swing_elapsed += dt
            else:
                leg.stance_elapsed += dt
        if self.strategy == 'tripod':
            self._fixed_tripod()
        else:
            self._local_controller()

    def _fixed_tripod(self):
        """Reference: centrally imposed alternating tripod, no feedback."""
        period = 1.20
        phase = (self.t % period) / period
        # Each group swings for 30% of the cycle; they do not overlap.
        swing = TRIPOD_GROUP_A if phase < 0.30 else TRIPOD_GROUP_B if 0.50 <= phase < 0.80 else frozenset()
        for leg in self.legs:
            if leg.disabled:
                continue
            if leg.index in swing and not leg.swinging:
                self.lift(leg)
            elif leg.index not in swing and leg.swinging:
                self.land(leg)

    def _local_controller(self):
        """Minimal LOCAL rule-inspired controller, *not* an implementation of neuroWalknet.

        - Each leg has its own stance/swing state.
        - A swinging neighbor inhibits lift-off (analogy to Cruse rule 1).
        - Touchdown releases the leg for later cycles; stance age triggers readiness.
        - Static support polygon guards against removing unsafe support legs.
        - No globally imposed tripod or wave phases.
        """
        for leg in self.legs:
            if leg.swinging and leg.swing_elapsed >= 0.24:
                self.land(leg)
        candidate_legs = sorted(
            (leg for leg in self.legs if not leg.disabled and not leg.swinging),
            key=lambda l: (-l.stance_elapsed, l.index),
        )
        for leg in candidate_legs:
            if leg.stance_elapsed < 0.72:
                continue
            neighbors = ((leg.index - 1) % 6, (leg.index + 1) % 6)
            if any(self.legs[i].swinging for i in neighbors):
                continue
            if sum(not l.disabled and l.swinging for l in self.legs) >= 2:
                continue
            # Minimum 1 cm static stability margin if this leg lifts.
            if support_margin((self.body_x, 0.0), self.support_feet(omit=(leg.index,))) < 0.01:
                continue
            self.lift(leg)


def simulate(strategy: str, *, duration: float = 12.0, dt: float = 0.02,
             speed: float = 0.02, fail_leg_at: float | None = None,
             fail_leg_index: int = 0):
    walker = Walker(strategy, velocity=speed)
    margins = []
    trace = []
    n = round(duration/dt)
    broken = False
    for k in range(n):
        broken = broken or (fail_leg_at is not None and k*dt >= fail_leg_at)
        walker.advance(dt, broken_leg=fail_leg_index if broken else None)
        margin = walker.margin()
        margins.append(margin)
        if k % 10 == 0:
            trace.append({
                't': round(walker.t, 3),
                'margin_m': round(margin, 4) if margin != float('-inf') else None,
                'contacts': [l.index for l in walker.legs if not l.swinging and not l.disabled],
                'body_x_m': round(walker.body_x, 4),
            })
    return {
        'strategy': strategy,
        'duration_s': duration,
        'speed_m_s_prescribed': speed,
        'failed_leg': fail_leg_index if fail_leg_at is not None else None,
        'failure_time_s': fail_leg_at,
        'min_margin_mm': round(min(margins)*1000, 2) if all(m != float('-inf') for m in margins) else None,
        'fraction_geometrically_unstable': round(sum(m <= 0 for m in margins)/len(margins), 4),
        'fraction_geometrically_safe_10mm': round(sum(m >= 0.01 for m in margins)/len(margins), 4),
        'foot_swing_count': sum(l.swing_count for l in walker.legs),
        'trace': trace,
    }