"""Suspension forces and kinematics for the four jounce DOFs of the full plant.

Per corner i the plant carries the unsprung mass's vertical position and
velocity. The suspension deflection s_i (compression from STATIC, positive =
bump) is algebraic from the chassis corner height and the unsprung height,
and the force the suspension pushes the body up with is

    F_susp,i = F_static,i + k_w,i s_i + F_arb,i + F_stop,i + F_damper,i

F_static,i is the sprung corner load at rest (so s = 0 is the static ride
height and the spring preload never has to be entered), k_w the wheel rate
(spring rate × motion ratio², from suspension/params.yaml), F_arb the
anti-roll bar as a spring across the axle, F_stop the bump stop past its gap
and the droop stop past the rebound travel. The damper is dampers/damper.py.

Kinematics: camber and toe as functions of jounce and body roll, with the
sign per side so one static value serves both wheels of an axle:

    γ_i = sgn_i·(γ0 + dγ/ds·s_i) + dγ/dφ·φ        sgn = +1 left, −1 right
    δ_toe,i = −sgn_i·(toe0 + dtoe/ds·s_i)

γ is the wheel's inclination about the body x axis (top leaning to +y =
positive), so a negative static camber leans both tops inward.

Load-transfer geometry: lateral force on an axle is reacted partly through
the links at the roll-centre height (instantly, onto the unsprung masses)
and partly through the springs via the body's roll. Likewise longitudinal
force with anti-dive / anti-squat. geometric_transfer() returns the link
part as a vertical force pair; the plant applies it down on the unsprung
mass and up on the body, which leaves exactly Fy·(h_s − h_rc) as the elastic
roll moment (verify.py K checks the closed form).
"""


class Suspension:
    def __init__(self, sp, vp, F_static):
        """sp: SuspensionParams, vp: VehicleParams, F_static: the four static
        sprung corner loads [N] (FL FR RL RR)."""
        self.p = sp
        self.F_static = tuple(F_static)
        pw = sp.per_wheel
        self.k_w = pw(sp.k_wheel_f, sp.k_wheel_r)
        self.bs_gap = pw(sp.bs_gap_f, sp.bs_gap_r)
        self.s_jounce = pw(sp.s_jounce_f, sp.s_jounce_r)
        self.s_rebound = pw(sp.s_rebound_f, sp.s_rebound_r)
        self.camber0 = pw(sp.camber0_f, sp.camber0_r)
        self.toe0 = pw(sp.toe0_f, sp.toe0_r)
        self.camber_bump = pw(sp.camber_bump_f, sp.camber_bump_r)
        self.camber_roll = pw(sp.camber_roll_f, sp.camber_roll_r)
        self.bump_steer = pw(sp.bump_steer_f, sp.bump_steer_r)
        self.sgn = (1.0, -1.0, 1.0, -1.0)            # left +, right −
        self.mate = (1, 0, 3, 2)                      # the other wheel on the axle
        # ARB as a spring across each axle: moment per rad of axle roll,
        # referred to the two wheels (F = k_arb (s_i − s_j) / t²)
        self.k_arb_wheel = (sp.k_arb_f / vp.track_f ** 2, sp.k_arb_f / vp.track_f ** 2,
                            sp.k_arb_r / vp.track_r ** 2, sp.k_arb_r / vp.track_r ** 2)
        self.h_rc = (sp.h_rc_f, sp.h_rc_f, sp.h_rc_r, sp.h_rc_r)
        self.anti = (sp.anti_dive, sp.anti_dive, sp.anti_squat, sp.anti_squat)
        self.track = (vp.track_f, vp.track_f, vp.track_r, vp.track_r)
        self.L = vp.wheelbase

    # ---------------------------------------------------------------- forces
    def spring_force(self, i, s):
        """Spring + ARB-free static part: F_static + k_w s."""
        return self.F_static[i] + self.k_w[i] * s

    def arb_force(self, i, s_all):
        """Anti-roll-bar force on wheel i (positive = pushes the body up on
        this side), from the axle's jounce difference."""
        return self.k_arb_wheel[i] * (s_all[i] - s_all[self.mate[i]])

    def stop_force(self, i, s):
        """Bump stop past its gap, a second stiffness past the jounce travel,
        the droop stop past the rebound travel — all at k_bs."""
        k = self.p.k_bs
        F = 0.0
        if s > self.bs_gap[i]:
            F += k * (s - self.bs_gap[i])
        if s > self.s_jounce[i]:
            F += k * (s - self.s_jounce[i])
        if s < -self.s_rebound[i]:
            F += k * (s + self.s_rebound[i])
        return F

    # ------------------------------------------------------------ kinematics
    def camber(self, i, s, roll):
        """Wheel inclination about body x [rad] at jounce s and body roll φ."""
        return (self.sgn[i] * (self.camber0[i] + self.camber_bump[i] * s)
                + self.camber_roll[i] * roll)

    def toe_steer(self, i, s):
        """Steer angle [rad] the toe setting and bump steer add to wheel i."""
        return -self.sgn[i] * (self.toe0[i] + self.bump_steer[i] * s)

    # ------------------------------------------------------- load transfer
    def geometric_transfer(self, Fy_wheel, Fx_wheel, h_s, ay, m_u):
        """The part of load transfer that bypasses the springs, as two force
        sets: (on_unsprung, on_body), both per wheel [N].

          on_unsprung: positive DOWN on the unsprung mass = more tire load
          on_body:     positive UP on the body corner (the reaction)

        Lateral, per axle (Milliken's three-way split, the elastic part
        being what the springs then see through the body's roll):
          link part      ±(Fy_axle − m_u,axle·a_y)·h_rc / t   on both sets
          unsprung part  ±m_u,axle·a_y·h_u / t                on the tires only
        (the lateral force the links pass to the body is the tire force
        minus what accelerates the unsprung mass; the unsprung mass's own
        inertia at h_u is reacted straight into the tire loads.)
        Longitudinal: sign·Fx_axle·anti·h_s/L on both sets, rear +, front −
        (drive at the rear with anti-squat pushes the rear axle down; braking
        at the front with anti-dive pushes the front axle down).

        Fy_wheel, Fx_wheel: body-frame tire forces per wheel; ay: the body's
        lateral acceleration; m_u: unsprung mass per wheel."""
        on_u = [0.0] * 4
        on_b = [0.0] * 4
        h_u = self.p.h_unsprung
        for i in range(4):
            j = self.mate[i]
            Fy_ax = Fy_wheel[i] + Fy_wheel[j]
            Fx_ax = Fx_wheel[i] + Fx_wheel[j]
            mu_ax = m_u[i] + m_u[j]
            # a +y (leftward) force loads the RIGHT (outer) wheel
            link = -self.sgn[i] * (Fy_ax - mu_ax * ay) * self.h_rc[i] / self.track[i]
            unsp = -self.sgn[i] * mu_ax * ay * h_u / self.track[i]
            sign = 1.0 if i >= 2 else -1.0
            lon = 0.5 * sign * Fx_ax * self.anti[i] * h_s / self.L
            on_u[i] = link + unsp + lon
            on_b[i] = link + lon
        return tuple(on_u), tuple(on_b)
