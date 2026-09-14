"""Planar two-track (4-wheel) vehicle model with four wheel-speed dynamics.

This is deliberately the MINIMUM physics for a software differential and
torque vectoring to be meaningful:

  * 3-DOF rigid body in the plane: vx, vy, r  (+ X, Y, psi for plotting)
  * 4 tire contact patches with individual vertical loads
    (static + aero + quasi-static longitudinal & lateral load transfer)
  * wheel rotational dynamics at all four corners,
        I_w,i * dω_i/dt = T_i - r_w * Fx_i
    -> every wheel has a slip ratio, so any wheel CAN actually spin up,
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
    wFL, wFR,
    wRL, wRR  : wheel angular speeds [rad/s], order FL FR RL RR

Equations of motion (body frame):
    m (dvx/dt - r vy) = ΣFx - F_drag
    m (dvy/dt + r vx) = ΣFy
    I_z dr/dt         = ΣM_z = Σ ( x_i F_y,i - y_i F_x,i )

EVERY wheel is driven: derivatives() takes four wheel torques in FL, FR, RL,
RR order, and a rear-drive car is simply T_FL = T_FR = 0. There is no
drive-layout flag in the plant.

That is not the same as the old free-roller front, which was implicitly
MASSLESS — it matched ground speed instantaneously and made no longitudinal
force at all. A front wheel with a real spin state lags its contact-patch
speed and therefore makes force in every transient, which is the car
accelerating its own front wheels: 2*I_w/r_w^2 = 13.4 kg of apparent mass,
5.3% of this car. verify.py D1 measures exactly that against a closed form.

The front wheels are also STEERED, so two things differ from the rears:
their slip ratio is built from the contact-patch speed resolved along the
wheel's own heading (v_cx, not vx - r*y), and their drive force rotates into
BOTH body axes, adding an x*Fy yaw-moment term the rear axle has no
equivalent of. Neither needs special-casing — both fall out of the existing
rotation once front Fx is nonzero.
"""

import math
from model.params import VehicleParams, G, RHO_AIR
from model.physical.tires.tire import MagicFormulaTire

# state vector indices. Wheel spins are in the repo-wide FL, FR, RL, RR order,
# so IW[i] lines up with index i of wheel_xy, Fz, kappa, Fx_w and self.tires.
IX, IY, IPSI, IVX, IVY, IR, IWFL, IWFR, IWRL, IWRR = range(10)
NSTATES = 10
NWHEELS = 4

WHEEL_NAMES = ("FL", "FR", "RL", "RR")
IW = (IWFL, IWFR, IWRL, IWRR)      # state index of each wheel's spin speed


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


def wheel_steer_angles(p: VehicleParams, delta: float):
    """Per-wheel road-wheel angles [rad], order FL FR RL RR. Rears are
    unsteered; this exists so per-wheel loops never special-case an axle."""
    dFL, dFR = front_steer_angles(p, delta)
    return (dFL, dFR, 0.0, 0.0)


def contact_speeds(model, s, delta: float):
    """Each contact patch's speed ALONG ITS OWN WHEEL HEADING [m/s], FL FR RL RR.

    Deliberately a SECOND implementation of the projection that tire_forces()
    does inline: verify.py compares the two, and a check that re-ran the sim's
    own expression would prove nothing.
    """
    p = model.p
    vx, vy, r = s[IVX], s[IVY], s[IR]
    steers = wheel_steer_angles(p, delta)
    out = []
    for i, (x, y) in enumerate(model.wheel_xy):
        vxi = vx - r * y
        vyi = vy + r * x
        out.append(vxi * math.cos(steers[i]) + vyi * math.sin(steers[i]))
    return tuple(out)


def free_rolling_omegas(model, s, delta: float):
    """The four wheel speeds that make kappa exactly zero at this state.

    Use this for every initial condition. `vx0 / r_wheel` is only correct when
    delta = vy = r = 0 — true of every maneuver TODAY, but not a property
    anyone should have to remember: at 23 deg of steer it would plant +0.086
    of slip ratio on each front wheel at t = 0, worth ~800 N of spurious drive
    force per wheel once the fronts are driven.
    """
    return tuple(v / model.p.r_wheel for v in contact_speeds(model, s, delta))


class VehicleModel:
    def __init__(self, p: VehicleParams, tire_front: MagicFormulaTire,
                 tire_rear: MagicFormulaTire, tires=None):
        self.p = p
        # per-wheel tires (FL, FR, RL, RR); `tires` overrides the default
        # front/rear pairing — used for split-µ tests (tracks.py)
        self.tires = (tuple(tires) if tires is not None
                      else (tire_front, tire_front, tire_rear, tire_rear))
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
            alpha [rad], kappa [-], Fx_w/Fy_w (wheel frame),
            Fx_b/Fy_b (body frame).
        """
        p = self.p
        vx, vy, r = s[IVX], s[IVY], s[IR]
        omegas = tuple(s[j] for j in IW)
        steers = wheel_steer_angles(p, delta)

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

            # every wheel is driven: slip ratio from its own spin state.
            # vcx is already resolved along this wheel's heading, so a steered
            # front wheel needs no special case. At kappa = 0 combined() returns
            # bit-identically (0, lateral(alpha, Fz)), so a zero-torque wheel
            # reproduces the old free-roller branch exactly.
            kappa[i] = (omegas[i] * p.r_wheel - vcx) / max(abs(vcx), p.v_eps)
            Fx_w[i], Fy_w[i] = self.tires[i].combined(kappa[i], alpha[i], Fz[i])

            # back to the body frame
            Fx_b[i] = Fx_w[i] * cd - Fy_w[i] * sd
            Fy_b[i] = Fx_w[i] * sd + Fy_w[i] * cd

        return {"alpha": alpha, "kappa": kappa,
                "Fx_w": Fx_w, "Fy_w": Fy_w, "Fx_b": Fx_b, "Fy_b": Fy_b}

    # ------------------------------------------------------------ derivatives
    def derivatives(self, s, delta: float, T_wheel):
        """Time derivatives of the state + an info dict for logging.

        T_wheel is a sequence of FOUR wheel torques [N·m] in FL, FR, RL, RR
        order. A rear-drive car passes (0, 0, T_RL, T_RR) — the plant has no
        drive-layout knob and does not need one.

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
        # wheel spin dynamics at all four corners: drive torque vs. the tire's
        # reaction. Fx_w is the WHEEL-frame force — the reaction acts about the
        # spin axis, not the body axis.
        I_corner = p.I_wheel_corner
        for i in range(NWHEELS):
            ds[IW[i]] = (T_wheel[i] - p.r_wheel * w["Fx_w"][i]) / I_corner[i]

        info = {"Fz": Fz, "alpha": w["alpha"], "kappa": w["kappa"],
                "Fx_w": w["Fx_w"], "Fy_w": w["Fy_w"],
                "ax": ax, "ay": ay, "Mz": Mz}
        return ds, info
