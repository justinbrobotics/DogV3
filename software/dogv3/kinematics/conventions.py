"""Sign and direction conventions — the single source of truth for D1/D2/sim.

Everything about "which way is positive" lives here so the setup program, the
runtime, and the simulator can never quietly disagree. Read this before touching
any sign. It is deliberately small and pure (no hardware, no config objects) so
it is trivially testable — see ``tests/test_regressions.py``.

Coordinate frame (right-handed, body-fixed, origin at each hip):
    +X = lateral, toward the robot's RIGHT
    +Y = longitudinal, FORWARD (toward the head)
    +Z = up; a hanging foot sits below the hip at -Z

Per-joint PHYSICAL convention — the SAME physical question for all four legs,
which is what makes the robot understandable:
    hip    :  + = foot swings OUTWARD, away from the body centerline
    femur  :  + = leg swings FORWARD, toward the head
    tibia  :  + = knee FLEXES, foot drawn under the body

Three INDEPENDENT signs, never folded into one another (a hard invariant):
    side       (per leg, "left"/"right"): the geometric body mirror, applied
               INSIDE the kinematics (``LegGeometry.side_sign`` flips both the
               lateral X and forward Y axes). It is NOT a servo property.
    invert     (per servo, bool): the encoder direction — whether a servo's raw
               count increases or decreases for +canonical joint angle. The ONLY
               place a servo's electrical direction enters. Fixed in D1 Direction.
    knee_sign  (per leg, +1/-1): which IK branch the knee folds into. Chosen so
               the stance is knees-back (knee toward the tail). Independent of
               invert; changing it never touches captured ranges.

DERIVED, and the key to a symmetric, understandable robot:
    physical_sign(side, invert, joint): +1 if a servo's raw INCREASES when its
    joint moves physical-positive (outward / forward / flex), else -1. This is
    what lets a mirror move every leg the SAME physical way — "outward on the
    left => outward on the right" (opposite in body-X, but symmetric about the
    centerline) — even though a given canonical IK angle means opposite body
    directions on left vs right. It is also the sign used when raising/lowering
    the stance: all legs actuate by an equal physical amount, left and right raw
    counts moving equal-and-opposite as needed.

    The hip carries an EXTRA flip vs femur/tibia. Reason: the kinematics mirror
    the lateral (X) axis differently from the fore/aft (Y) axis, so at neutral
    d(foot_x)/d(hip_angle) has no side factor while d(foot_y)/d(femur_angle)
    does. The formula below is verified against the real robot's hand-posed
    ground truth in the regression tests — do not "simplify" it away.
"""
from __future__ import annotations

# Joint order everywhere in the stack.
JOINTS: tuple[str, str, str] = ("hip", "femur", "tibia")

# Human-facing physical-positive description per joint (used by the D1 GUI).
PHYSICAL_POSITIVE: dict[str, str] = {
    "hip": "foot swings OUTWARD from the centerline",
    "femur": "leg swings FORWARD, toward the head",
    "tibia": "knee FLEXES, foot drawn under the body",
}

NEUTRAL_COUNT = 2048  # servo raw at joint neutral; must match kinematics.leg


def side_sign(side: str) -> int:
    """-1 for a left leg, +1 for a right leg. Matches ``LegGeometry.side_sign``."""
    return -1 if side == "left" else 1


def encoder_dir(invert: bool) -> int:
    """-1 if the servo encoder is inverted, else +1. Matches angle<->count."""
    return -1 if invert else 1


def physical_sign(side: str, invert: bool, joint: str) -> int:
    """+1 if the servo's raw count increases when its joint moves in its physical-
    positive sense (hip OUTWARD, femur FORWARD, tibia FLEX), else -1.

    The hip flips relative to femur/tibia (see module docstring)."""
    if joint not in JOINTS:
        raise ValueError(f"unknown joint {joint!r}")
    ss = side_sign(side)
    return (-ss if joint == "hip" else ss) * encoder_dir(invert)


def mirror_raw(master_raw: int, master_sign: int, follower_sign: int,
               neutral: int = NEUTRAL_COUNT) -> int:
    """Raw count that places a follower joint at the SAME physical offset from
    neutral as the master — same physical direction, re-signed through each axis.

    ``master_sign`` / ``follower_sign`` are :func:`physical_sign` values. This is
    an involution: ``mirror_raw(mirror_raw(r, a, b), b, a) == r``."""
    physical_offset = master_sign * (master_raw - neutral)  # counts, + = out/fwd/flex
    return neutral + follower_sign * physical_offset
