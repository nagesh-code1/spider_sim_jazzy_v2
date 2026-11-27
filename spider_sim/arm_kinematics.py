"""2-link planar IK for the arm (shoulder + elbow pitch).

Target (x, z) is relative to the shoulder pivot: x forward, z up.
Joint convention in the URDF: both pitch joints turn about +y, so a POSITIVE
joint angle tips the link DOWN. The maths below works in the usual
"up is positive" convention and flips the sign at the end.
"""
import math

A1 = 0.14  # upper arm length (m)
A2 = 0.14  # forearm length (m)

SHOULDER_LIMIT = (-1.5, 1.5)
ELBOW_LIMIT = (0.05, 2.8)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def inverse_kinematics(x, z):
    """Return (shoulder_joint, elbow_joint) in URDF joint angles (rad)."""
    r = math.hypot(x, z)
    r = _clamp(r, 0.05, A1 + A2 - 0.005)
    c2 = _clamp((r * r - A1 * A1 - A2 * A2) / (2 * A1 * A2), -1.0, 1.0)
    # elbow-up solution: forearm bends down relative to the upper arm
    theta2 = -math.acos(c2)
    theta1 = math.atan2(z, x) - math.atan2(A2 * math.sin(theta2), A1 + A2 * math.cos(theta2))
    shoulder = _clamp(-theta1, *SHOULDER_LIMIT)
    elbow = _clamp(-theta2, *ELBOW_LIMIT)
    return shoulder, elbow


def forward_kinematics(shoulder, elbow):
    """Inverse of the above, used by the self-check: joint angles -> (x, z)."""
    t1 = -shoulder
    t2 = -elbow
    x = A1 * math.cos(t1) + A2 * math.cos(t1 + t2)
    z = A1 * math.sin(t1) + A2 * math.sin(t1 + t2)
    return x, z
