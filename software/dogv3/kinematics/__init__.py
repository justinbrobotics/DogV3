"""Per-leg kinematics (FK/IK) and joint-angle <-> servo-count conversion."""

from .leg import (
    LegGeometry,
    forward_kinematics,
    inverse_kinematics,
    angle_to_count,
    count_to_angle,
    COUNTS_PER_RAD,
)

__all__ = [
    "LegGeometry",
    "forward_kinematics",
    "inverse_kinematics",
    "angle_to_count",
    "count_to_angle",
    "COUNTS_PER_RAD",
]
