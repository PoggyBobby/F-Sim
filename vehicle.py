"""Planar two-track (4-wheel) vehicle model with rear wheel-speed dynamics.

This is deliberately the MINIMUM physics for a software differential and
torque vectoring to be meaningful:

  * 3-DOF rigid body in the plane: vx, vy, r  (+ X, Y, psi for plotting)
  * 4 tire contact patches with individual vertical loads
    (static + aero + quasi-static longitudinal & lateral load transfer)
  * rear wheel rotational dynamics  I_w * dω/dt = T_wheel - r_w * Fx
    -> wheel slip ratios exist, so an inner wheel CAN actually spin up,
       which is the entire problem the s-diff solves
  * Magic-Formula tires with load sensitivity and a friction-circle cap

Deliberately EXCLUDED (the other team's full model owns these): suspension
kinematics / roll & pitch DOFs, rolling resistance, motor electrical &
thermal dynamics, sensor noise / state estimation, driver model, banking
and grade.

States (indices below):
    X, Y, psi : position [m] and heading [rad] in the ground frame
    vx, vy    : body-frame velocities [m/s] (x forward, y left)
    r         : yaw rate [rad/s], CCW (left turn) positive
    wRL, wRR  : rear wheel angular speeds [rad/s]

Equations of motion (body frame):
    m (dvx/dt - r vy) = ΣFx - F_drag
    m (dvy/dt + r vx) = ΣFy
    I_z dr/dt         = ΣM_z = Σ ( x_i F_y,i - y_i F_x,i )

Front wheels are undriven free-rollers: they generate lateral force only
(their spin state is not tracked). Rear wheels generate combined Fx/Fy from
slip ratio and slip angle.
"""

import math
from params import VehicleParams, G, RHO_AIR
from tire import MagicFormulaTire

# state vector indices
IX, IY, IPSI, IVX, IVY, IR, IWRL, IWRR = range(8)
NSTATES = 8

WHEEL_NAMES = ("FL", "FR", "RL", "RR")


def front_steer_angles(p: VehicleParams, delta: float):
    """Per-wheel front road-wheel angles (FL, FR) from the single-track
    command delta, with Ackermann geometry.

    The maneuver scripts command ONE steer angle (the single-track/bicycle
    angle). The steering linkage turns the two front wheels by different
    amounts: for zero low-speed scrub the inner wheel must point at a
    tighter radius than the outer (100% Ackermann):

        R        = L / tan(|delta|)          turn radius the command implies
        delta_in = atan(L / (R - t_f/2))     inner wheel
        delta_out= atan(L / (R + t_f/2))     outer wheel

    ackermann_frac blends between parallel steer (0: both wheels get delta,
    the pre-2026-08-30 behavior) and full geometric Ackermann (1); negative
    values give anti-Ackermann. In a LEFT turn (delta > 0, ISO) the LEFT
    (FL) wheel is the inner one.
    """
    fA = p.ackermann_frac
    ad = abs(delta)
    if fA == 0.0 or ad < 1e-9:
        return delta, delta
    R = p.wheelbase / math.tan(ad)
    # guard: at absurd commands the inner-radius term could cross zero
    Ri = max(R - p.track_f / 2.0, 0.05)
    d_in = math.atan(p.wheelbase / Ri)
    d_out = math.atan(p.wheelbase / (R + p.track_f / 2.0))
    dFL_m = ad + fA * ((d_in if delta > 0 else d_out) - ad)
    dFR_m = ad + fA * ((d_out if delta > 0 else d_in) - ad)
    sgn = 1.0 if delta > 0 else -1.0
    return sgn * dFL_m, sgn * dFR_m


class VehicleModel:
    def __init__(self, p: VehicleParams, tire_front: MagicFormulaTire,
                 tire_rear: MagicFormulaTire):
        self.p = p
        self.tires = (tire_front, tire_front, tire_rear, tire_rear)
        # wheel positions relative to CG, ISO frame (x fwd, y left): FL FR RL RR
        self.wheel_xy = (
            ( p.a,  p.track_f / 2.0),
            ( p.a, -p.track_f / 2.0),
            (-p.b,  p.track_r / 2.0),
            (-p.b, -p.track_r / 2.0),
        )

    # ------------------------------------------------------------------ loads
    def wheel_loads(self, vx: float, ax: float, ay: float):
        """Per-wheel vertical loads from static weight, aero downforce, and
        quasi-static longitudinal / lateral load transfer.

        ax, ay are the specific forces (ΣF/m) currently acting on the body —
        positive ax = accelerating forward (load moves rearward), positive
        ay = accelerating left, i.e. a LEFT turn (load moves to the RIGHT,
        outer, wheels).
        """
        p = self.p
        m, L, h = p.m_total, p.wheelbase, p.h_cg

        downforce = 0.5 * RHO_AIR * p.ClA * vx * vx
        down_f = p.aero_balance_front * downforce
        down_r = downforce - down_f

        # axle loads with longitudinal transfer (moment balance about contacts)
        Fz_axle_f = m * (G * p.b - ax * h) / L + down_f
        Fz_axle_r = m * (G * p.a + ax * h) / L + down_r

        # lateral transfer per axle, split by roll-stiffness fraction.
        # dF = load ADDED to the right wheel and REMOVED from the left wheel.
        dF_f = p.lat_transfer_frac_front * m * ay * h / p.track_f
        dF_r = (1.0 - p.lat_transfer_frac_front) * m * ay * h / p.track_r

        Fz = (
            Fz_axle_f / 2.0 - dF_f,   # FL
            Fz_axle_f / 2.0 + dF_f,   # FR
            Fz_axle_r / 2.0 - dF_r,   # RL
            Fz_axle_r / 2.0 + dF_r,   # RR
        )
        # clamp: a wheel in the air carries no (and never negative) load
        return tuple(max(f, 0.0) for f in Fz)

    # ----------------------------------------------------------------- forces
    def tire_forces(self, s, delta: float, Fz):
        """Slips and tire forces for all four wheels at state s.

        Returns dict with per-wheel lists (order FL FR RL RR):
            alpha [rad], kappa [-] (0 for fronts), Fx_w/Fy_w (wheel frame),
            Fx_b/Fy_b (body frame).
        """
        p = self.p
        vx, vy, r = s[IVX], s[IVY], s[IR]
        omegas = (None, None, s[IWRL], s[IWRR])
        dFL, dFR = front_steer_angles(p, delta)
        steers = (dFL, dFR, 0.0, 0.0)

        alpha = [0.0] * 4
        kappa = [0.0] * 4
        Fx_w = [0.0] * 4
        Fy_w = [0.0] * 4
        Fx_b = [0.0] * 4
        Fy_b = [0.0] * 4

        for i, (x, y) in enumerate(self.wheel_xy):
            # contact-patch velocity in the body frame
            vxi = vx - r * y
            vyi = vy + r * x
            # rotate into the wheel frame (front wheels steered by delta)
            cd, sd = math.cos(steers[i]), math.sin(steers[i])
            vcx = vxi * cd + vyi * sd
            vcy = -vxi * sd + vyi * cd

            # slip angle: velocity vector angle relative to wheel heading,
            # signed so that positive alpha -> positive (leftward) Fy
            alpha[i] = -math.atan2(vcy, max(vcx, p.v_eps))

            if omegas[i] is None:
                # undriven front wheel: free rolling, lateral force only
                Fy_w[i] = self.tires[i].lateral(alpha[i], Fz[i])
                Fx_w[i] = 0.0
            else:
                # driven rear wheel: slip ratio from wheel speed state
                kappa[i] = (omegas[i] * p.r_wheel - vcx) / max(abs(vcx), p.v_eps)
                Fx_w[i], Fy_w[i] = self.tires[i].combined(kappa[i], alpha[i], Fz[i])

            # back to the body frame
            Fx_b[i] = Fx_w[i] * cd - Fy_w[i] * sd
            Fy_b[i] = Fx_w[i] * sd + Fy_w[i] * cd

        return {"alpha": alpha, "kappa": kappa,
                "Fx_w": Fx_w, "Fy_w": Fy_w, "Fx_b": Fx_b, "Fy_b": Fy_b}

    # ------------------------------------------------------------ derivatives
    def derivatives(self, s, delta: float, T_RL: float, T_RR: float):
        """Time derivatives of the state + an info dict for logging.

        Load transfer depends on accelerations, which depend on tire forces,
        which depend on loads — a small algebraic loop. Solved here with 3
        fixed-point iterations starting from zero transfer (converges fast
        because transfer is a moderate correction to static+aero load).
        """
        p = self.p
        m = p.m_total
        vx, vy, r, psi = s[IVX], s[IVY], s[IR], s[IPSI]

        F_drag = 0.5 * RHO_AIR * p.CdA * vx * vx

        ax = ay = 0.0
        Fz = w = None
        for _ in range(3):
            Fz = self.wheel_loads(vx, ax, ay)
            w = self.tire_forces(s, delta, Fz)
            ax = (sum(w["Fx_b"]) - F_drag) / m
            ay = sum(w["Fy_b"]) / m

        # yaw moment about the CG from all contact-patch forces
        Mz = sum(x * fy - y * fx
                 for (x, y), fx, fy in zip(self.wheel_xy, w["Fx_b"], w["Fy_b"]))

        ds = [0.0] * NSTATES
        ds[IX] = vx * math.cos(psi) - vy * math.sin(psi)
        ds[IY] = vx * math.sin(psi) + vy * math.cos(psi)
        ds[IPSI] = r
        ds[IVX] = ax + r * vy
        ds[IVY] = ay - r * vx
        ds[IR] = Mz / p.I_z
        # rear wheel spin dynamics: drive torque vs. tire reaction torque
        ds[IWRL] = (T_RL - p.r_wheel * w["Fx_w"][2]) / p.I_wheel
        ds[IWRR] = (T_RR - p.r_wheel * w["Fx_w"][3]) / p.I_wheel

        info = {"Fz": Fz, "alpha": w["alpha"], "kappa": w["kappa"],
                "Fx_w": w["Fx_w"], "Fy_w": w["Fy_w"],
                "ax": ax, "ay": ay, "Mz": Mz}
        return ds, info
