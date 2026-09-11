"""FK(IK(p)) ~= p across the workspace — the required hardware-free test."""
import math

import pytest

from dogv3.kinematics import (
    LegGeometry,
    forward_kinematics,
    inverse_kinematics,
    angle_to_count,
    count_to_angle,
    COUNTS_PER_RAD,
)

GEOM_PARAMS = [
    ("right", 1),
    ("left", 1),
    ("right", -1),
    ("left", -1),
]


def _reachable_points(geom):
    pts = []
    for x in (-30, 0, 40):
        for y in (-40, 0, 30):
            for z in (-150, -200, -230):
                pts.append((x, y, z))
    return pts


@pytest.mark.parametrize("side,knee", GEOM_PARAMS)
def test_fk_ik_roundtrip(side, knee):
    geom = LegGeometry(L1=50, L2=110, L3=130, side=side, knee_sign=knee)
    for p in _reachable_points(geom):
        t1, t2, t3 = inverse_kinematics(geom, *p)
        q = forward_kinematics(geom, t1, t2, t3)
        for a, b in zip(p, q):
            assert math.isclose(a, b, abs_tol=1e-6), f"{side}/{knee}: {p} -> {q}"


def test_counts_per_rad_constant():
    assert math.isclose(COUNTS_PER_RAD, 651.9, abs_tol=0.1)


def test_angle_to_count_neutral():
    assert angle_to_count(0.0) == 2048


def test_angle_to_count_invert_is_mirror():
    theta = 0.5
    normal = angle_to_count(theta)
    inverted = angle_to_count(theta, invert=True)
    # Symmetric about neutral.
    assert (normal - 2048) == -(inverted - 2048)


def test_angle_count_roundtrip():
    for theta in (-1.0, -0.3, 0.0, 0.4, 1.2):
        for invert in (False, True):
            count = angle_to_count(theta, invert=invert)
            back = count_to_angle(count, invert=invert)
            # within one count of resolution
            assert abs(back - theta) < 1.0 / COUNTS_PER_RAD + 1e-9


def test_angle_to_count_clamps():
    assert angle_to_count(10.0, min_raw=100, max_raw=3000) == 3000
    assert angle_to_count(-10.0, min_raw=100, max_raw=3000) == 100


def test_offset_shifts_neutral():
    # With an offset equal to the angle, count returns to neutral.
    assert angle_to_count(0.7, offset=0.7) == 2048
