"""STS3215 goal-trajectory model: how fast the servo is even *told* to move.

The real servo does not jump to a written goal. Its internal profile generator
ramps toward the goal at the commanded SPEED with the commanded ACC — the same
``DEFAULT_SPEED`` / ``DEFAULT_ACC`` the D2 hot loop writes with every
``SyncWritePosEx``. The physics sim needs that layer between the 60 Hz control
goals and the position actuator, otherwise the sim leg teleports and every
gait looks feasible.

Torque-side reality (can the loaded leg keep up with even this profile) is the
MuJoCo position actuator's job (kp/kv/forcerange); this module is purely the
electrical command ceiling.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..kinematics.leg import COUNTS_PER_RAD
from ..runtime.control import DEFAULT_ACC, DEFAULT_SPEED

# WritePosEx units: SPEED is counts/s, ACC is x100 counts/s^2.
VMAX_RAD_S = DEFAULT_SPEED / COUNTS_PER_RAD
AMAX_RAD_S2 = DEFAULT_ACC * 100.0 / COUNTS_PER_RAD


@dataclass
class ServoProfile:
    """Trapezoidal tracker for one joint: position chases the latest goal at
    <= vmax with |accel| <= amax. ``limited`` counts sim steps where the
    velocity ceiling (not the goal) dictated the motion — the signature of a
    gait demanding more than the servo command channel can deliver."""

    pos: float
    vmax: float = VMAX_RAD_S
    amax: float = AMAX_RAD_S2
    vel: float = 0.0
    steps: int = field(default=0)
    limited_steps: int = field(default=0)

    def step(self, dt: float, goal: float) -> float:
        err = goal - self.pos
        # Velocity that still allows stopping exactly on the goal at amax.
        v_des = math.copysign(min(self.vmax, math.sqrt(2.0 * self.amax * abs(err))), err)
        if abs(err) < 1e-9:
            v_des = 0.0
        dv = max(-self.amax * dt, min(self.amax * dt, v_des - self.vel))
        self.vel += dv
        self.pos += self.vel * dt
        self.steps += 1
        if abs(self.vel) >= self.vmax * 0.999:
            self.limited_steps += 1
        return self.pos

    @property
    def limited_frac(self) -> float:
        return self.limited_steps / self.steps if self.steps else 0.0
