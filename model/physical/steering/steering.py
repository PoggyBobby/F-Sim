"""Steering: the rack and the two front knuckles as three DOFs (6 states).

The driver's handwheel angle is the input. It reaches the wheels through
two compliances and three inertias:

    handwheel ──column k_col──▶ rack (m_r, q_r) ──tie rods k_δ──▶ knuckles (I_k, δ_i)

    m_r q̈_r = k_col/r_p² (q_cmd − q_r) − c_r q̇_r − F_fric tanh(q̇_r/v_sm)
              − Σ_i k_δ (δ_kin,i(q_r) − δ_i) · dδ_kin,i/dq_r
    I_k δ̈_i = k_δ (δ_kin,i(q_r) − δ_i) − c_k δ̇_i + M_kp,i

q_cmd = handwheel · c_factor / 2π is where the rack would be with a rigid
column; r_p = c_factor / 2π is the pinion radius that turns the column's
torsional stiffness into a linear one. δ_kin,i(q_r) is the KINEMATIC wheel
angle the rack position asks for: the handwheel-equivalent of q_r pushed
through the team's steering chart (sensors/steering_angle) and split
inner/outer by the Ackermann fraction (vehicle.front_steer_angles). The
tie-rod term on the rack is the reaction of the knuckle springs, by virtual
work. M_kp,i is the tire's moment about the kingpin (aligning moment,
mechanical trail, scrub radius), supplied by the plant.

Why this matters for TV (the DOF doc's point): the controller should be
able to tell "the car did not yaw because the torque split was wrong" from
"the front wheels never reached the angle the driver asked for".

Values: model/physical/steering/params.yaml.
"""

import math

from model.physical.vehicle import front_steer_angles
from model.sensors.steering_angle.sas import steer_map_deg

TWO_PI = 2.0 * math.pi


class Steering:
    def __init__(self, sp, vp, v_smooth):
        self.p = sp
        self.vp = vp
        self.v_sm = v_smooth
        self.r_p = sp.c_factor / TWO_PI                  # pinion radius [m/rad]
        self.k_col_lin = sp.k_col / (self.r_p * self.r_p)  # column as a linear spring on the rack
        self.dq = 1e-5                                   # finite-difference step for dδ/dq [m]

    def rack_target(self, handwheel):
        """Rack position [m] a rigid column would give for handwheel [rad],
        clipped at the rack stops."""
        q = handwheel * self.r_p
        return max(-self.p.q_max, min(self.p.q_max, q))

    def kinematic_angles(self, q_r):
        """(δ_kin_FL, δ_kin_FR) [rad] for rack position q_r."""
        hw_deg = math.degrees(q_r / self.r_p)
        delta = math.radians(steer_map_deg(hw_deg))
        return front_steer_angles(self.vp, delta)

    def derivatives(self, q_r, q_rd, d_FL, d_FLd, d_FR, d_FRd, handwheel, M_kp):
        """Returns (q̈_r, δ̈_FL, δ̈_FR) and the kinematic angles used."""
        p = self.p
        kin = self.kinematic_angles(q_r)
        kin2 = self.kinematic_angles(q_r + self.dq)
        dkin_dq = ((kin2[0] - kin[0]) / self.dq, (kin2[1] - kin[1]) / self.dq)

        tau_FL = p.k_delta * (kin[0] - d_FL)
        tau_FR = p.k_delta * (kin[1] - d_FR)

        q_cmd = self.rack_target(handwheel)
        F_rack = (self.k_col_lin * (q_cmd - q_r) - p.c_rack * q_rd
                  - p.F_fric * math.tanh(q_rd / self.v_sm)
                  - tau_FL * dkin_dq[0] - tau_FR * dkin_dq[1])
        q_rdd = F_rack / p.m_rack

        d_FLdd = (tau_FL - p.c_k * d_FLd + M_kp[0]) / p.I_k
        d_FRdd = (tau_FR - p.c_k * d_FRd + M_kp[1]) / p.I_k
        return (q_rdd, d_FLdd, d_FRdd), kin

    def kingpin_moment(self, Fy_w, Fx_w, Mz_tire, sgn):
        """Tire moment about the kingpin for one front wheel [N·m], positive
        = steers the wheel LEFT: the pneumatic aligning moment (already
        signed to reduce the slip angle), the lateral force on the mechanical
        trail (same sense), and the longitudinal force on the scrub radius.
        sgn = +1 for the left wheel, −1 for the right: the contact patch sits
        outboard of the kingpin by r_scrub, so a drive force steers a left
        wheel RIGHT (M = −y·Fx with y = +r_scrub) and a right wheel LEFT —
        the torque steer a left/right split produces."""
        return Mz_tire - Fy_w * self.p.t_mech - sgn * Fx_w * self.p.r_scrub
