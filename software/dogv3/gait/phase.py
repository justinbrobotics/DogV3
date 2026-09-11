"""Phase clock for periodic gaits.

The clock holds a normalized phase in [0, 1). Each leg has a fixed phase offset;
for a trot the diagonal pairs FL+BR and FR+BL move together, 180 deg apart. A
crawl instead lifts one leg at a time in a fixed, statically-stable order.
"""
from __future__ import annotations

# Trot: diagonal pairs in phase, pairs offset by half a cycle.
TROT_OFFSETS: dict[str, float] = {
    "FL": 0.0,
    "BR": 0.0,
    "FR": 0.5,
    "BL": 0.5,
}

# Crawl (statically-stable wave gait): exactly one foot leaves the ground at a
# time, in the maximum-stability-margin order rear-left, front-left, rear-right,
# front-right. The value is the global phase at which each leg lifts (begins
# swing). With one quarter-cycle between lifts and a stance fraction >= 0.75,
# the swing windows tile without overlap, so three feet are always planted.
CRAWL_LIFT_ORDER: dict[str, float] = {
    "BL": 0.0,
    "FL": 0.25,
    "BR": 0.5,
    "FR": 0.75,
}

# A crawl is only statically stable while at most one foot is airborne, which
# requires the stance fraction to be at least 3/4. The generator floors the
# seed's duty_factor to this in crawl mode regardless of the trot tuning value.
CRAWL_MIN_DUTY: float = 0.75


class PhaseClock:
    """Advances a normalized phase by ``dt / cycle_period`` each tick."""

    def __init__(self, cycle_period: float = 0.6):
        if cycle_period <= 0:
            raise ValueError("cycle_period must be > 0")
        self.cycle_period = cycle_period
        self._phase = 0.0

    @property
    def phase(self) -> float:
        return self._phase

    def reset(self, phase: float = 0.0) -> None:
        self._phase = phase % 1.0

    def advance(self, dt: float) -> float:
        """Advance by *dt* seconds; returns the new global phase."""
        self._phase = (self._phase + dt / self.cycle_period) % 1.0
        return self._phase

    def leg_phase(self, leg: str) -> float:
        """Phase for *leg* including its trot offset, in [0, 1)."""
        return (self._phase + TROT_OFFSETS.get(leg, 0.0)) % 1.0
