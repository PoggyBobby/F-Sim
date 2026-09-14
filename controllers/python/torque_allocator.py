"""Four-corner torque allocator — the default controller with a motor per upright.

WHY THIS EXISTS RATHER THAN "THE S-DIFF, TWICE". Longitudinal load transfer
takes the front axle from 43.4% of the car's weight at rest to 28.3% at 1 g.
Because per-wheel longitudinal capacity is mu_x(Fz)*Fz, at 0.8 g a front tire
saturates at 162 N*m while a rear takes 311 — against 273 N*m available at
every motor. So a front motor can overwhelm its own tire at 60% of its own
peak while the rears cannot saturate at all, and an even four-way split spins
the fronts and wastes nearly half the rear axle. (verify.py B6 pins those
numbers; the ratio grows 1.255 -> 2.315 from 0 to 1.1 g.)

The allocator therefore decides FRONT/REAR from estimated load before it
decides anything left/right.

    stage 1   axle split      frac_front follows the estimated normal-load split
    stage 2   left/right      each axle shared by estimated corner load
    stage 2b  spin guard      per-corner cut once |kappa| passes kappa_lim
    stage 3   limits          _apply_limits4() — see its own ordering comment

BOTH ACCELERATION ESTIMATES ARE DERIVED, NEVER MEASURED. ax comes from the
torque REQUEST, ay from yaw rate plus a kinematic steer feedforward. Three
reasons, all load-bearing:

  * the perfect-state path has no ax in the state vector, so a measured ax
    would make the two update paths structurally different and verify I4 would
    stop measuring exactly one thing (the vx estimate);
  * an accelerometer is a cycle stale and carries noise, which would land
    straight in the torque split;
  * nothing measured feeds back into the allocation, so it CANNOT oscillate.

The cost is honesty about saturation: when the tires are past the limit, or a
power cap is clipping, the request over-predicts ax. That over-predicts
rearward transfer and so UNDER-drives the front — erring away from the
front-spin failure mode rather than into it. ax_est_max bounds the rest.

Constants: controllers/python/awd/params.yaml (namespace controllers.awd).
These are a design output of this sim, not a mirror of the firmware — see that
file's `about:` block, and do not put them in controllers/python/params.yaml.
"""

import math

from model.params import VehicleParams, AllocParams, G, RHO_AIR
from model.physical.vehicle import IVX, IR, IW, NWHEELS
from controllers.python.debug import ControllerDebug
from controllers.python.torque_split import clampf, slew, TorqueSplitController

NAME_OPEN = "open 4WD"
NAME_AWD = "4-corner AWD"
NAME_RWD = "s-diff (RWD ref)"
CONFIG_NAMES = (NAME_OPEN, NAME_AWD, NAME_RWD)



# ───────────────────────────────────────────────────────────── estimators
def ax_feedforward(vp: VehicleParams, ap: AllocParams, T_req_total, vx):
    """Longitudinal acceleration the REQUEST implies [m/s²].

    Clamped to what the drivetrain can actually deliver at this wheel speed
    before differentiating, so a request the power cap will clip does not
    inflate the estimate. Still pure feedforward — no plant state, no loop."""
    w = max(abs(vx) / vp.r_wheel, vp.w_eps)
    deliverable = min(abs(T_req_total),
                      vp.driven_wheels * vp.motor_P_peak / w,
                      vp.T_drive_max)
    T = math.copysign(deliverable, T_req_total)
    F_drag = 0.5 * RHO_AIR * vp.CdA * vx * vx
    ax = (T / vp.r_wheel - F_drag) / vp.m_total
    return clampf(ax, -ap.ax_est_max, ap.ax_est_max)


def ay_estimate(vp: VehicleParams, ap: AllocParams, vx, r, delta):
    """Lateral acceleration [m/s²] from yaw rate, with a kinematic steer
    feedforward for phase lead at turn-in."""
    ay = ((1.0 - ap.ay_ff_frac) * r * vx
          + ap.ay_ff_frac * vx * vx * delta / vp.wheelbase)
    return clampf(ay, -ap.ay_est_max, ap.ay_est_max)


def axle_load_fraction(vp: VehicleParams, vx, ax):
    """Front axle's share of total vertical load [-].

    Mirrors VehicleModel.wheel_loads()'s longitudinal + aero terms. The AERO
    TERM IS NOT OPTIONAL: 485 N of downforce at 15 m/s, 218 N of it on the
    front against a ~1075 N static front axle. Dropping it says the front
    carries 24.7% at 1.24 g when the truth is 28.0% — a 3.3-point error, all
    in the direction of starving the front."""
    down = 0.5 * RHO_AIR * vp.ClA * vx * vx
    Fz_f = (vp.m_total * (G * vp.b - ax * vp.h_cg) / vp.wheelbase
            + vp.aero_balance_front * down)
    return Fz_f / (vp.m_total * G + down)


def corner_loads(vp: VehicleParams, vx, ax, ay):
    """Estimated per-corner vertical load (FL, FR, RL, RR) [N].

    Same sign convention as the plant: dF is ADDED to the right wheel and
    REMOVED from the left, since ay > 0 is a left turn."""
    down = 0.5 * RHO_AIR * vp.ClA * vx * vx
    down_f = vp.aero_balance_front * down
    Fz_axle_f = (vp.m_total * (G * vp.b - ax * vp.h_cg) / vp.wheelbase + down_f)
    Fz_axle_r = (vp.m_total * (G * vp.a + ax * vp.h_cg) / vp.wheelbase
                 + down - down_f)
    dF_f = vp.lat_transfer_frac_front * vp.m_total * ay * vp.h_cg / vp.track_f
    dF_r = ((1.0 - vp.lat_transfer_frac_front) * vp.m_total * ay * vp.h_cg
            / vp.track_r)
    return (max(Fz_axle_f / 2.0 - dF_f, 0.0), max(Fz_axle_f / 2.0 + dF_f, 0.0),
            max(Fz_axle_r / 2.0 - dF_r, 0.0), max(Fz_axle_r / 2.0 + dF_r, 0.0))


# ──────────────────────────────────────────────────────────── the controller
class FourCornerAllocator:
    """One class, two configurations. `allocate=False` switches every stage off
    and gives an exact even split — which is precisely an open differential on
    four wheels, and makes the baseline share this code path rather than being
    a separate class that might differ for uninteresting reasons."""

    def __init__(self, vp: VehicleParams, ap: AllocParams, name: str,
                 allocate: bool = True):
        self.vp, self.ap = vp, ap
        self.allocate = allocate
        self.name = name
        self.plaus_cut = False
        self.reset()

    def reset(self):
        self.plaus_cut = False
        self.frac_front = self.ap.frac_front_base if self.allocate else 0.5
        self.spin = [1.0] * NWHEELS

    # ------------------------------------------------------------- stage 1
    def _axle_split(self, T_req_total, vx, dt):
        ap = self.ap
        if not self.allocate:
            self.frac_front = 0.5
            return 0.0, 0.5
        ax = ax_feedforward(self.vp, ap, T_req_total, vx)
        frac_load = axle_load_fraction(self.vp, vx, ax)
        target = clampf(ap.frac_front_base
                        + ap.k_load * (frac_load - self.vp.weight_frac_front),
                        ap.frac_front_min, ap.frac_front_max)
        self.frac_front = slew(self.frac_front, target, ap.frac_rate, dt)
        return ax, frac_load

    # ------------------------------------------------------------- stage 2
    def _lateral_weights(self, vx, r, delta):
        """Per-corner share of ITS OWN axle's torque (FL, FR, RL, RR)."""
        ap = self.ap
        if not self.allocate:
            return (0.5, 0.5, 0.5, 0.5), 0.0
        ay = ay_estimate(self.vp, ap, vx, r, delta)
        Fz = corner_loads(self.vp, vx, 0.0, ay)   # lateral only: ax cannot
        wgt = []                                  # change an axle's TOTAL
        for lo, hi in ((0, 1), (2, 3)):
            a = max(Fz[lo], 0.0) ** ap.load_exponent
            b = max(Fz[hi], 0.0) ** ap.load_exponent
            # BOTH shares by division from the same denominator, never one as
            # `1 - other`. Float addition is commutative, so a/(a+b) and
            # b/(a+b) map onto each other exactly when the corners swap — which
            # is what keeps the mirrored-steer identity (verify D2) EXACT
            # rather than exact-to-1e-9. The clamp branches mirror too.
            # a + b only as a ZERO guard, never as a permanent epsilon in the
            # denominator: (a+b)/(a+b+eps) is short of 1 by ~8e-11, which is
            # 3e-8 N·m of torque quietly lost per axle (verify F13 catches it).
            den = a + b
            if den <= 0.0:
                sl = sr = 0.5
            else:
                sl, sr = a / den, b / den
            if sl < ap.share_min:
                sl, sr = ap.share_min, 1.0 - ap.share_min
            elif sr < ap.share_min:
                sl, sr = 1.0 - ap.share_min, ap.share_min
            wgt += [sl, sr]
        return tuple(wgt), ay

    # ------------------------------------------------------------ stage 2b
    def _spin_guard(self, T, w, vx, dt):
        """Per-corner proportional cut. Cut instantly, restore rate-limited —
        a traction-control shape, and P-only so no gain needs retuning at the
        VCU rate (verify section H's standing finding)."""
        ap = self.ap
        kap = [0.0] * NWHEELS
        if not self.allocate:
            return tuple(T), tuple(kap)
        v_ref = max(abs(vx), self.vp.v_eps)
        out = []
        for i in range(NWHEELS):
            kap[i] = (w[i] * self.vp.r_wheel - v_ref) / v_ref
            over = max(abs(kap[i]) - ap.kappa_lim, 0.0)
            h = clampf(1.0 - ap.k_spin * over / ap.kappa_lim, ap.spin_floor, 1.0)
            # asymmetric: drop immediately, recover only at the release rate
            self.spin[i] = h if h < self.spin[i] else slew(
                self.spin[i], h, ap.spin_release_rate, dt)
            out.append(T[i] * self.spin[i])
        return tuple(out), tuple(kap)

    # ------------------------------------------------------------- stage 3
    def _apply_limits4(self, T, w, vx):
        """Physical limits, four wheels. ORDER MATTERS and each step's
        invariant is a verify check:

          1  regen floor
          2  per-AXLE difference-preserving shift into +-T_max
          3  per-wheel POWER headroom, applied to the axle DIFFERENCE first
          4  total rules cap, as an axle-mean shift
          5  uniform scale as a last resort, flagged

        Step 3 is the subtle one. The per-motor power clip is |T| <= P/omega,
        so it always bites the FASTER wheel first — which in a corner is the
        outer wheel, the one a yaw split has just given MORE torque to. Clipping
        each wheel independently after a difference-preserving shift therefore
        shrinks the split and can invert its sign: 200/300 at omega 100/140
        comes back as 173/143, a requested +100 delivered as -30. (verify F7
        records the two-motor version of this as finding #3.) So the difference
        is capped against the TIGHTER of the axle's two headrooms first, and
        the mean is fitted afterwards.
        """
        vp = self.vp
        T_max = vp.T_wheel_max
        T_min = -T_max if vx > vp.regen_speed_cutoff else 0.0
        T = list(T)
        cap_fallback = 0.0

        for lo, hi in ((0, 1), (2, 3)):
            # 1-2: split into mean + difference, clip the difference to what
            # the torque range can produce, then translate the pair to fit.
            mean = 0.5 * (T[lo] + T[hi])
            diff = clampf(T[hi] - T[lo], -(T_max - T_min), T_max - T_min)
            # 3: ... and to what each motor's POWER allows, tighter of the two
            head = min(vp.motor_P_peak / max(abs(w[lo]), vp.w_eps),
                       vp.motor_P_peak / max(abs(w[hi]), vp.w_eps))
            head = min(head, T_max)
            diff = clampf(diff, -2.0 * head, 2.0 * head)
            a, b = mean - diff / 2.0, mean + diff / 2.0
            over = max(a, b) - head
            if over > 0.0:
                a -= over
                b -= over
            under = T_min - min(a, b)
            if under > 0.0:
                a += under
                b += under
            T[lo], T[hi] = clampf(a, T_min, head), clampf(b, T_min, head)

        # 4: the 80 kW rules cap. At today's numbers driven_wheels*P_peak is
        # EXACTLY power_cap_total, so step 3 already implies this and it never
        # fires — see verify F6's info line. It stays because motor_power_peak
        # is a DERIVED estimate that a dyno could move.
        P = [T[i] * w[i] for i in range(NWHEELS)]
        if sum(P) > vp.P_total_max:
            excess = sum(P) - vp.P_total_max
            P_f, P_r = max(P[0] + P[1], 0.0), max(P[2] + P[3], 0.0)
            share_f = P_f / (P_f + P_r) if (P_f + P_r) > 0.0 else 0.5
            for (lo, hi), sh in (((0, 1), share_f), ((2, 3), 1.0 - share_f)):
                cut = sh * excess / max(abs(w[lo]) + abs(w[hi]), vp.w_eps)
                cut = min(cut, min(T[lo], T[hi]) - T_min)   # never below floor
                T[lo] -= cut
                T[hi] -= cut
            P = [T[i] * w[i] for i in range(NWHEELS)]
            if sum(P) > vp.P_total_max:                     # 5: last resort
                scale = vp.P_total_max / sum(P)
                T = [t * scale for t in T]
                cap_fallback = 1.0
        return tuple(T), cap_fallback

    # ---------------------------------------------------------------- update
    def update(self, s, delta: float, T_req_total: float, dt: float):
        vp = self.vp
        vx, r = s[IVX], s[IR]
        w = tuple(s[j] for j in IW)

        ax_est, frac_load = self._axle_split(T_req_total, vx, dt)
        wgt, ay_est = self._lateral_weights(vx, r, delta)

        T_axle = (self.frac_front * T_req_total,
                  (1.0 - self.frac_front) * T_req_total)
        T = tuple(T_axle[i // 2] * wgt[i] for i in range(NWHEELS))
        T, kap = self._spin_guard(T, w, vx, dt)
        T, cap_fallback = self._apply_limits4(T, w, vx)

        dbg = ControllerDebug(T=T)
        dbg.dw_target = r * vp.track_r / vp.r_wheel
        dbg.dT_sdiff = T[3] - T[2]
        dbg.f_applied = self.frac_front
        dbg.g_left_appl, dbg.g_right_appl = wgt[2], wgt[3]
        dbg.delta_norm = frac_load
        self.last = (ax_est, ay_est, frac_load, wgt, kap, tuple(self.spin),
                     cap_fallback, T_req_total)
        return dbg

    def update_from_sensors(self, sr, dt: float):
        """The real path: SensorReadings only, exactly as the VCU sees them."""
        from model.config import cfg
        from model.physical.vehicle import NSTATES

        braking = sr.bps_bar > cfg.sensors.brake_pressure_sens.actuated_bar
        if sr.apps_pct > cfg.sensors.vcu.plaus_apps_cut and braking:
            self.plaus_cut = True
        elif self.plaus_cut and sr.apps_pct < cfg.sensors.vcu.plaus_apps_restore:
            self.plaus_cut = False

        if self.plaus_cut:
            T_req = 0.0
        elif braking:
            bps = cfg.sensors.brake_pressure_sens
            T_req = -bps.t_regen_max * min(sr.bps_bar / bps.range_bar, 1.0)
        else:
            T_req = self.vp.T_drive_max * sr.apps_pct / 100.0

        ps = [0.0] * NSTATES
        ps[IVX] = sr.vx_est
        ps[IR] = sr.yaw_rate
        for j, nm in zip(IW, ("FL", "FR", "RL", "RR")):
            ps[j] = getattr(sr, "wheel_speed_" + nm)
        return self.update(ps, sr.steer_est, T_req, dt)


# ────────────────────────────────────────────────────────── configurations
def make_configs(vp, tp_front, tp_rear, cp):
    """The configurations compared in every maneuver.

    The baseline is this same allocator with its law switched OFF, not a
    separate class, so open-vs-AWD isolates the allocation law rather than the
    code path. The rear-drive s-diff is kept as a reference — it is the only
    way to attribute a metric change to AWD rather than to the allocator, and
    it is what the SIL is diffed against.

    Order matters: verify.py indexes this list positionally ([0] and [1]), so
    the reference is APPENDED, never inserted."""
    return [
        FourCornerAllocator(vp, cp.awd, NAME_OPEN, allocate=False),
        FourCornerAllocator(vp, cp.awd, NAME_AWD, allocate=True),
        TorqueSplitController(vp, tp_front, tp_rear, cp, True, NAME_RWD),
    ]
