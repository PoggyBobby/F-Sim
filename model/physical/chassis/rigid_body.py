"""6-DOF rigid-body kinematics for the full plant: quaternion attitude,
rotation matrices, Euler's equations with a roll-yaw product of inertia.

Pure functions on tuples of floats. No numpy: the vectors are three long and
the per-call overhead of an ndarray is larger than the arithmetic.

Conventions (ISO 8855): body x forward, y LEFT, z up. The ground frame is
X, Y, Z with Z up. The quaternion q = (q0, q1, q2, q3), scalar first, rotates
BODY vectors into the GROUND frame: v_ground = R(q) · v_body. Body angular
velocity ω = (p, q, r) = (roll, pitch, yaw rates). With this choice

    dq/dt = ½ · q ⊗ (0, p, q, r)

and the unit-norm constraint |q| = 1 is what makes four numbers hold three
angles (the DOF doc's reason for a quaternion: no gimbal lock at 90°).

Euler angles are ZYX (yaw ψ about Z, then pitch θ about Y, then roll φ about
X): R = Rz(ψ) Ry(θ) Rx(φ). Positive roll = left side UP (the body leans to
the right, the normal left-turn roll); positive pitch = nose DOWN.
"""

import math


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def q_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def q_to_R(q):
    """Rotation matrix (body -> ground) as a tuple of three row tuples."""
    q0, q1, q2, q3 = q
    return (
        (1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)),
        (2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)),
        (2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)),
    )


def rot(R, v):
    """R · v  (body -> ground)."""
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


def rot_T(R, v):
    """Rᵀ · v  (ground -> body)."""
    return (R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
            R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
            R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2])


def q_dot(q, w):
    """dq/dt = ½ q ⊗ (0, p, q, r)."""
    q0, q1, q2, q3 = q
    p, qq, r = w
    return (0.5 * (-q1 * p - q2 * qq - q3 * r),
            0.5 * (q0 * p + q2 * r - q3 * qq),
            0.5 * (q0 * qq - q1 * r + q3 * p),
            0.5 * (q0 * r + q1 * qq - q2 * p))


def q_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def euler_from_q(q):
    """(roll, pitch, yaw) in the ZYX convention above."""
    R = q_to_R(q)
    sp = -R[2][0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return roll, pitch, yaw


def omega_dot(I, w, M):
    """Euler's equations I·ω̇ + ω × (I·ω) = M for an inertia tensor with a
    roll-yaw product term: I = (Ixx, Iyy, Izz, Ixz), the matrix being
    [[Ixx, 0, -Ixz], [0, Iyy, 0], [-Ixz, 0, Izz]]. Returns (ṗ, q̇, ṙ)."""
    Ixx, Iyy, Izz, Ixz = I
    p, qq, r = w
    h = (Ixx * p - Ixz * r, Iyy * qq, -Ixz * p + Izz * r)      # angular momentum
    wxh = cross(w, h)
    rx = M[0] - wxh[0]
    ry = M[1] - wxh[1]
    rz = M[2] - wxh[2]
    det = Ixx * Izz - Ixz * Ixz
    return ((Izz * rx + Ixz * rz) / det,
            ry / Iyy,
            (Ixz * rx + Ixx * rz) / det)
