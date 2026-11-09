"""Position legs first, then a single bounded wheel pulse; no overlapping preparation."""
from dataclasses import dataclass


@dataclass
class Sequence:
    target: float
    sign: int
    pulse_s: float
    ready_since: float | None = None
    pulse_at: float | None = None
    timed_out: bool = False

    def command(self, t, q, v):
        """Clock is relative to the end of the 0.75 s gravity settling period."""
        target = .1 + (self.target-.1)*min(1., max(0.,t)/.3)
        if self.pulse_at is None and not self.timed_out:
            ready = t >= .3 and max(abs(x-self.target) for x in q) <= .015 and max(abs(x) for x in v) <= .1
            self.ready_since = (self.ready_since if self.ready_since is not None else t) if ready else None
            if self.ready_since is not None and t-self.ready_since >= .1-1e-9:
                self.pulse_at = t
            elif t >= 1.25:
                self.timed_out = True
        if self.pulse_at is None:
            return [target]*2, [0.,0.], 'position-timeout' if self.timed_out else 'position-legs'
        active = t < self.pulse_at+self.pulse_s-1e-9
        return [self.target]*2, [self.sign*.03 if active else 0.]*2, 'wheel-impulse' if active and self.sign else 'hold-legs'

