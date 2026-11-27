import math

import pytest

from spider_sim.arm_kinematics import forward_kinematics, inverse_kinematics


@pytest.mark.parametrize('x,z', [(0.20, -0.12), (0.16, 0.10), (0.22, 0.0), (0.18, 0.05)])
def test_ik_round_trip_reaches_target(x, z):
    shoulder, elbow = inverse_kinematics(x, z)
    fx, fz = forward_kinematics(shoulder, elbow)
    assert math.hypot(fx - x, fz - z) < 1e-3


def test_target_above_lifts_arm_up():
    # positive joint angle tips the link down, so "up" needs a negative shoulder
    shoulder_up, _ = inverse_kinematics(0.16, 0.10)
    shoulder_down, _ = inverse_kinematics(0.20, -0.12)
    assert shoulder_up < shoulder_down


def test_angles_stay_inside_urdf_limits():
    for x, z in [(0.3, 0.3), (0.01, -0.3), (-0.2, 0.1)]:
        shoulder, elbow = inverse_kinematics(x, z)
        assert -1.5 <= shoulder <= 1.5
        assert 0.05 <= elbow <= 2.8
