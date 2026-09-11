"""Locked 3-DOF leg kinematics reference.

Coordinate frame (per labeled CAD):
  X = lateral (left<->right), Y = longitudinal (+Y = forward/toward head),
  Z = vertical (foot below hip -> negative z).

Origin at the hip-abduction joint. ``side`` is the geometric left/right mirror,
applied **inside the kinematics by flipping both the lateral X (the hip ``L1``
link points outward: +X right, -X left) and the forward Y**. It is kept entirely
separate from a servo's ``invert`` (encoder direction) and from ``knee_sign``
(IK branch). This matches the MuJoCo twin, which builds each hip with ``sx*L1``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# 4096 counts / 2*pi rad = 651.9 counts per radian (matches the spec constant).
COUNTS_PER_RAD = 4096.0 / (2.0 * math.pi)  # ~= 651.9
NEUTRAL_COUNT = 2048


@dataclass(frozen=True)
class LegGeometry:
    """Link lengths plus the per-leg geometric constants.

    side: "left" or "right" — only mirrors Y inside the kinematics.
    knee_sign: +1 / -1 — selects the IK knee branch (fore/aft bend).
    """

    L1: float
    L2: float
    L3: float
    side: str = "right"
    knee_sign: int = 1

    @property
    def side_sign(self) -> int:
        # right -> +1, left -> -1. Mirrors both the lateral hip-link offset (L1)
        # and the forward Y axis, so a left leg is a true mirror of the right.
        return -1 if self.side == "left" else 1


def forward_kinematics(geom: LegGeometry, t1: float, t2: float, t3: float) -> tuple[float, float, float]:
    """Joint angles (radians) -> foot position (x, y, z) in the leg frame.

    Serves as the test oracle for the IK; equations are copied verbatim from the
    spec, with the lateral (X) and forward (Y) mirror folded in via ``side_sign``."""
    lx = geom.side_sign  # lateral mirror: hip L1 link points outward (+X right, -X left)
    P_y = geom.L2 * math.sin(t2) + geom.L3 * math.sin(t2 + t3)
    P_z = -(geom.L2 * math.cos(t2) + geom.L3 * math.cos(t2 + t3))
    x = lx * geom.L1 * math.cos(t1) + P_z * math.sin(t1)   # lateral (mirrored)
    y = lx * P_y                                            # forward (mirrored)
    z = -lx * geom.L1 * math.sin(t1) + P_z * math.cos(t1)  # up (negative below hip)
    return x, y, z


def inverse_kinematics(geom: LegGeometry, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Foot position (x, y, z) -> joint angles (t1, t2, t3) in radians.

    The lateral (X) and forward (Y) mirror is undone via ``side_sign`` so the same
    branch math serves left and right legs."""
    lx = geom.side_sign  # lateral mirror (matches forward_kinematics and the MuJoCo twin)
    y_local = lx * y

    d = math.sqrt(max(x * x + z * z - geom.L1 * geom.L1, 0.0))
    t1 = math.atan2(-d, lx * geom.L1) - math.atan2(z, x)
    r = math.hypot(y_local, d)
    c3 = (r * r - geom.L2 * geom.L2 - geom.L3 * geom.L3) / (2.0 * geom.L2 * geom.L3)
    c3 = max(-1.0, min(1.0, c3))
    t3 = geom.knee_sign * math.acos(c3)
    t2 = math.atan2(y_local, d) - math.atan2(geom.L3 * math.sin(t3), geom.L2 + geom.L3 * math.cos(t3))
    return t1, t2, t3


def angle_to_count(
    theta: float,
    *,
    invert: bool = False,
    offset: float = 0.0,
    min_raw: int = 0,
    max_raw: int = 4095,
) -> int:
    """Joint angle (radians, canonical frame) -> clamped servo count.

    ``count = clamp(round(2048 + dir*(theta - offset)*651.9), min_raw, max_raw)``
    where ``dir`` is +1 normally, -1 when the servo's encoder is inverted.
    ``invert`` is the *only* place a servo's encoder direction enters — never
    ``side`` or ``knee_sign``."""
    direction = -1 if invert else 1
    count = round(NEUTRAL_COUNT + direction * (theta - offset) * COUNTS_PER_RAD)
    return max(min_raw, min(max_raw, count))


def count_to_angle(count: int, *, invert: bool = False, offset: float = 0.0) -> float:
    """Inverse of :func:`angle_to_count` (no clamping)."""
    direction = -1 if invert else 1
    return offset + direction * (count - NEUTRAL_COUNT) / COUNTS_PER_RAD


def _with_links(geom: "LegGeometry", *, l2: float | None = None, l3: float | None = None) -> "LegGeometry":
    return LegGeometry(
        L1=geom.L1,
        L2=geom.L2 if l2 is None else l2,
        L3=geom.L3 if l3 is None else l3,
        side=geom.side,
        knee_sign=geom.knee_sign,
    )


def joint_positions(
    geom: LegGeometry, t1: float, t2: float, t3: float
) -> tuple[tuple[float, float, float], ...]:
    """Leg-frame 3D positions of the kinematic chain, for the live skeleton:

    ``(hip_abduction, femur_pitch_joint, knee_joint, foot)``.

    Reuses :func:`forward_kinematics` with truncated link lengths so the exact
    same mirror/branch convention drives the visualizer and the controller — the
    foot returned here is identical to ``forward_kinematics(geom, t1, t2, t3)``."""
    hip = (0.0, 0.0, 0.0)
    femur = forward_kinematics(_with_links(geom, l2=0.0, l3=0.0), t1, t2, t3)
    knee = forward_kinematics(_with_links(geom, l3=0.0), t1, t2, t3)
    foot = forward_kinematics(geom, t1, t2, t3)
    return hip, femur, knee, foot
