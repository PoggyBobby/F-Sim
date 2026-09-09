"""Physics verification suite — assume the sim is wrong, then try to prove it.

Every check here compares the sim against something INDEPENDENT of the sim's
own code path: a closed-form solution, an algebraic identity, a symmetry the
physics must obey, or a hand-derived textbook result. A check that merely
re-runs the sim's own formula would prove nothing.

    .venv/bin/python verify.py

Sections
  A  tire model        stiffness construction, peak force, friction circle,
                       load sensitivity, where the placeholder curve peaks
  B  vertical loads    total-load identity, static split, transfer signs
  C  slip kinematics   contact-patch velocities & slip angles vs independent
                       textbook formulas; wheel-speed-difference target sign
  D  rigid-body EOM    yaw moment sum, coast-down vs closed form, mirror
                       symmetry, steady-state force balance, RK4 dt-refine
  E  steady cornering  two-track sim vs independent linear bicycle model
  F  controller limits torque caps, yaw-split preservation, regen floor,
                       power caps — unit-tested against hand values
  G  in-run audit      invariants recorded DURING the standard maneuvers:
                       friction-circle utilization, wheel lift, mu floor,
                       torque/power/speed limits, wheel-speed kinematics
  H  robustness        controller at realistic VCU rates (100 Hz / 1 kHz)
                       instead of the physics rate (4 kHz)

Exit code 0 = every hard check passed. INFO lines are findings/quantified
observations, not pass/fail.
"""

import math
import sys

import numpy as np

from model.params import VehicleParams, TireParams, ControlParams, default_setup, G, RHO_AIR
from model.physical.tires.tire import MagicFormulaTire
from model.physical.vehicle import (VehicleModel, front_steer_angles, NSTATES, NWHEELS,
                                    IW, WHEEL_NAMES, free_rolling_omegas,
                                    wheel_steer_angles,
                                    contact_speeds, IX, IY, IPSI,
                     IVX, IVY, IR, IWRL, IWRR)
from controllers.python.torque_split import TorqueSplitController
from controllers.python.torque_allocator import (make_configs, FourCornerAllocator,
                                                 axle_load_fraction, ax_feedforward)
from model.maneuvers.maneuvers import Maneuver, step_steer, corner_exit, slalom
from model.sim import simulate, metrics

FAIL = []
RESULTS = []


def check(section, name, ok, detail=""):
    tag = "PASS" if ok else "FAIL"
    line = f"[{tag}] {section}: {name}" + (f" — {detail}" if detail else "")
    RESULTS.append(line)
    print(line)
    if not ok:
        FAIL.append(line)


def info(section, name, detail):
    line = f"[info] {section}: {name} — {detail}"
    RESULTS.append(line)
    print(line)


# ═══════════════════════════════════════════════════════ A. tire model
def section_a():
    vp, tp_f, tp_r, cp = default_setup()
    tire = MagicFormulaTire(tp_r)

    # A1: small-slip stiffness. The MF construction B = c/(C*mu) claims the
    # slope at zero slip is exactly c_alpha*Fz. Differentiate numerically.
    for Fz in (300.0, 593.0, 1200.0):
        h = 1e-6
        slope = (tire.lateral(h, Fz) - tire.lateral(-h, Fz)) / (2 * h)
        want = tp_r.c_alpha * Fz
        check("A1", f"lateral stiffness at Fz={Fz:.0f} N",
              abs(slope / want - 1) < 1e-3,
              f"dFy/dα = {slope:.1f} N/rad vs c_alpha·Fz = {want:.1f}")
    h = 1e-6
    Fz = 593.0
    slope = (tire.longitudinal(h, Fz) - tire.longitudinal(-h, Fz)) / (2 * h)
    check("A1", "longitudinal stiffness",
          abs(slope / (tp_r.c_kappa * Fz) - 1) < 1e-3,
          f"dFx/dκ = {slope:.1f} vs c_kappa·Fz = {tp_r.c_kappa * Fz:.1f}")

    # A2: the peak of each pure curve must be exactly its D = µ·Fz — the
    # Magic Formula's sin(...) has max 1 only if C*atan(...) reaches pi/2.
    Fz = 593.0
    alphas = np.linspace(0, 0.6, 4001)
    Fy = np.array([tire.lateral(a, Fz) for a in alphas])
    D = tire.mu(Fz) * Fz
    check("A2", "peak lateral force equals mu_y·Fz",
          abs(Fy.max() / D - 1) < 1e-3,
          f"max Fy = {Fy.max():.1f} N vs mu·Fz = {D:.1f} N")
    kaps = np.linspace(0, 0.8, 4001)
    Fx = np.array([tire.longitudinal(k, Fz) for k in kaps])
    Dx = tire.mu_x(Fz) * Fz
    check("A2", "peak longitudinal force equals mu_x·Fz (separate µx)",
          abs(Fx.max() / Dx - 1) < 1e-3,
          f"max Fx = {Fx.max():.1f} N vs mu_x·Fz = {Dx:.1f} N "
          f"(µx/µy = {tire.mu_x(Fz) / tire.mu(Fz):.2f}, TTC-fitted)")
    a_pk = alphas[Fy.argmax()]
    info("A2", "fitted curve peaks",
         f"lateral at {math.degrees(a_pk):.1f}° slip (TTC sweeps reach ±12° "
         "and the R20 plateaus there — beyond is extrapolated plateau), "
         f"longitudinal at κ = {kaps[Fx.argmax()]:.3f} (the replay's spin "
         "threshold)")

    # A3: friction ELLIPSE — combined output may never leave the ellipse
    # with semi-axes µx·Fz and µy·Fz, for any slip combination and load.
    worst = 0.0
    for Fz in (150.0, 593.0, 1500.0, 3000.0):
        for k in np.linspace(-1.2, 1.2, 25):
            for a in np.linspace(-0.5, 0.5, 25):
                fx, fy = tire.combined(k, a, Fz)
                worst = max(worst, math.hypot(fx / (tire.mu_x(Fz) * Fz),
                                              fy / (tire.mu(Fz) * Fz)))
    check("A3", "friction ellipse never exceeded (grid of 2500 slip states)",
          worst <= 1.0 + 1e-9, f"max utilization {worst:.6f}")

    # A4: load sensitivity — the grip COEFFICIENT must fall as load rises,
    # and total force must still RISE with load in the working range.
    mus = [tire.mu(Fz) for Fz in (300, 600, 1200)]
    musx = [tire.mu_x(Fz) for Fz in (300, 600, 1200)]
    check("A4", "mu falls with load (both directions)",
          mus[0] > mus[1] > mus[2] and musx[0] > musx[1] > musx[2],
          f"mu_y(300/600/1200 N) = {mus[0]:.3f}/{mus[1]:.3f}/{mus[2]:.3f}, "
          f"mu_x = {musx[0]:.3f}/{musx[1]:.3f}/{musx[2]:.3f}")
    peaks = [max(tire.lateral(a, Fz) for a in np.linspace(0, 0.6, 600))
             for Fz in (300, 600, 1200)]
    check("A4", "peak force still rises with load", peaks[0] < peaks[1] < peaks[2],
          f"peak Fy = {peaks[0]:.0f}/{peaks[1]:.0f}/{peaks[2]:.0f} N")
    info("A4", "mu floor (0.5·mu0) engages at",
         f"Fz = {tp_r.Fz_nom * (1 + 0.5 / tp_r.s_mu):.0f} N per tire "
         f"(nominal static is {tp_r.Fz_nom:.0f} N) — section G checks whether "
         "any maneuver actually gets there")

    # A5: zero force at zero slip and at zero load.
    ok = (tire.lateral(0.0, 600.0) == 0.0 and tire.longitudinal(0.0, 600.0) == 0.0
          and tire.combined(0.3, 0.2, 0.0) == (0.0, 0.0)
          and tire.combined(0.3, 0.2, -50.0) == (0.0, 0.0))
    check("A5", "zero slip / zero load → zero force (incl. negative load)", ok)

    # A6: combined(0, α, Fz) is BIT-IDENTICAL to (0, lateral(α, Fz)). This is
    # the identity the whole four-wheel change rests on — it is why a
    # zero-torque wheel reproduces the old free-roller branch exactly, and why
    # the plant needs no `if undriven` branch. Asserted with ==, not a
    # tolerance: _mf(0) = 0 kills Fx, and pure lateral force can never exceed
    # the ellipse because D = µ·Fz is the curve's own peak, so the cap never
    # fires.
    worst_fx = worst_fy = 0.0
    for Fz_ in (0.0, -50.0, 200.0, 537.0, 900.0, 1600.0):
        for adeg in np.linspace(-20.0, 20.0, 121):
            fx, fy = tire.combined(0.0, math.radians(adeg), Fz_)
            worst_fx = max(worst_fx, abs(fx))
            worst_fy = max(worst_fy, abs(fy - tire.lateral(math.radians(adeg), Fz_)))
    check("A6", "combined(0, α, Fz) ≡ (0, lateral(α, Fz)) exactly",
          worst_fx == 0.0 and worst_fy == 0.0,
          f"over 6 loads × 121 slip angles: max |Fx| = {worst_fx}, "
          f"max |Fy − lateral| = {worst_fy} (exact zeros, not tolerances)")


# ═══════════════════════════════════════════════ B. vertical loads
def section_b():
    vp, tp_f, tp_r, cp = default_setup()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))

    # B1: identity — for ANY (vx, ax, ay) the four loads must sum to
    # m·g + downforce(vx) exactly (as long as no wheel-lift clamp fires).
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(500):
        vx = rng.uniform(0, 30)
        ax = rng.uniform(-15, 15)
        ay = rng.uniform(-20, 20)
        Fz = model.wheel_loads(vx, ax, ay)
        if min(Fz) == 0.0:
            continue                       # clamp fired — identity breaks by design
        want = vp.m_total * G + 0.5 * RHO_AIR * vp.ClA * vx * vx
        worst = max(worst, abs(sum(Fz) / want - 1))
    check("B1", "ΣFz = m·g + downforce for 500 random states", worst < 1e-12,
          f"worst relative error {worst:.2e}")

    # B2: static split — front fraction and left/right symmetry at rest.
    Fz = model.wheel_loads(0.0, 0.0, 0.0)
    front_frac = (Fz[0] + Fz[1]) / sum(Fz)
    check("B2", "static front-axle share equals WEIGHT_FRACTION_FRONT",
          abs(front_frac - vp.weight_frac_front) < 1e-12,
          f"{front_frac:.4f} vs {vp.weight_frac_front:.4f}")
    check("B2", "static left/right symmetric",
          abs(Fz[0] - Fz[1]) < 1e-12 and abs(Fz[2] - Fz[3]) < 1e-12)

    # B3: transfer directions. Braking (ax<0) must load the FRONT; a left
    # turn (ay>0) must load the RIGHT (outer) wheels.
    Fz0 = model.wheel_loads(10.0, 0.0, 0.0)
    Fzb = model.wheel_loads(10.0, -8.0, 0.0)
    check("B3", "braking shifts load forward",
          Fzb[0] > Fz0[0] and Fzb[2] < Fz0[2],
          f"FL {Fz0[0]:.0f}→{Fzb[0]:.0f} N, RL {Fz0[2]:.0f}→{Fzb[2]:.0f} N")
    Fzl = model.wheel_loads(10.0, 0.0, +8.0)
    check("B3", "left turn loads the right (outer) side",
          Fzl[1] > Fzl[0] and Fzl[3] > Fzl[2],
          f"front L/R {Fzl[0]:.0f}/{Fzl[1]:.0f} N, rear L/R {Fzl[2]:.0f}/{Fzl[3]:.0f} N")
    # total lateral transfer magnitude = m·ay·h/track (equal tracks)
    dF_tot = (Fzl[1] + Fzl[3]) - (Fzl[0] + Fzl[2])
    want = 2 * vp.m_total * 8.0 * vp.h_cg / vp.track_f
    check("B3", "total lateral transfer = m·ay·h/track",
          abs(dF_tot / want - 1) < 1e-12, f"{dF_tot:.1f} vs {want:.1f} N")

    # B5: axle-load statics under longitudinal acceleration, against the
    # textbook moment balance derived here independently. This is the term the
    # four-corner allocator has to estimate, so it is worth pinning exactly.
    worst = 0.0
    for ax in (0.0, 4.0, 8.0, 11.0):
        Fz_ = VehicleModel(VehicleParams(ClA=0.0), MagicFormulaTire(tp_f),
                           MagicFormulaTire(tp_r)).wheel_loads(0.0, ax, 0.0)
        got = (Fz_[0] + Fz_[1]) / (Fz_[2] + Fz_[3])
        want = ((G * vp.b - ax * vp.h_cg) / (G * vp.a + ax * vp.h_cg))
        worst = max(worst, abs(got / want - 1))
    check("B5", "front/rear axle load ratio matches the moment balance under ax",
          worst < 1e-12, f"worst relative error {worst:.1e} over ax = 0…11 m/s²")

    # B6: the coupling AWD introduces. Longitudinal transfer unloads the front,
    # so per-wheel LONGITUDINAL capacity µx(Fz)·Fz diverges between the axles —
    # the front tire saturates at a torque the rear cannot even reach.
    ratios = []
    for ax in (0.0, 4.0, 8.0, 11.0):
        Fz_ = model.wheel_loads(10.0, ax, 0.0)
        cap_f = model.tires[0].mu_x(Fz_[0]) * Fz_[0]
        cap_r = model.tires[2].mu_x(Fz_[2]) * Fz_[2]
        ratios.append(cap_r / cap_f)
    rising = all(b > a for a, b in zip(ratios, ratios[1:]))
    check("B6", "rear/front longitudinal capacity grows with acceleration",
          ratios[0] > 1.0 and rising,
          "ratio " + " → ".join(f"{r:.3f}" for r in ratios) + " at ax = 0/4/8/11 m/s²")
    Fz8 = model.wheel_loads(10.0, 8.0, 0.0)
    info("B6", "per-wheel saturating torque at 0.8 g",
         f"front {model.tires[0].mu_x(Fz8[0]) * Fz8[0] * vp.r_wheel:.1f} N·m vs rear "
         f"{model.tires[2].mu_x(Fz8[2]) * Fz8[2] * vp.r_wheel:.1f} N·m, against "
         f"{vp.T_wheel_max:.0f} N·m per motor — a front motor can overwhelm its "
         "own tire at 60% of its peak while the rears cannot saturate at all")

    # B4: wheel lift — clamp engages, no negative load ever escapes.
    Fzx = model.wheel_loads(0.0, 0.0, 60.0)     # absurd ay to force lift
    check("B4", "extreme ay lifts inner wheels to exactly 0 (never negative)",
          Fzx[0] == 0.0 and Fzx[2] == 0.0 and min(Fzx) >= 0.0,
          f"loads {tuple(round(f, 1) for f in Fzx)}")


# ═══════════════════════════════════════ C. slip kinematics
def section_c():
    vp, tp_f, tp_r, cp = default_setup()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))

    # C1: slip angles vs the independent textbook small-angle formulas
    #   α_f = δ − (vy + a·r)/vx ,  α_r = −(vy − b·r)/vx
    vx, vy, r, delta = 15.0, 0.4, 0.5, 0.06
    s = [0.0] * NSTATES
    s[IVX], s[IVY], s[IR] = vx, vy, r
    for j, w0 in zip(IW, free_rolling_omegas(model, s, delta)):
        s[j] = w0                      # all four rolling, kappa = 0 by construction
    Fz = model.wheel_loads(vx, 0.0, 0.0)
    w = model.tire_forces(s, delta, Fz)
    a_f_txt = delta - (vy + vp.a * r) / vx
    a_r_txt = -(vy - vp.b * r) / vx
    # average L/R (the exact per-wheel values differ via r·track/2 terms)
    a_f_sim = 0.5 * (w["alpha"][0] + w["alpha"][1])
    a_r_sim = 0.5 * (w["alpha"][2] + w["alpha"][3])
    check("C1", "front slip angle matches textbook formula (small angles)",
          abs(a_f_sim - a_f_txt) < 2e-3,
          f"sim {a_f_sim:.5f} vs textbook {a_f_txt:.5f} rad")
    check("C1", "rear slip angle matches textbook formula",
          abs(a_r_sim - a_r_txt) < 2e-3,
          f"sim {a_r_sim:.5f} vs textbook {a_r_txt:.5f} rad")

    # C2: slip ratio. "Rolling" in a YAWING car means each wheel matches its
    # OWN contact-patch speed (vx − r·y), not the CG speed — first attempt at
    # this check used the CG speed and wrongly failed the sim. Set each rear
    # wheel to its own contact speed: κ must be ~0; then overspeed one by 10%.
    s3 = list(s)
    for j, w0 in zip(IW, free_rolling_omegas(model, s, delta)):
        s3[j] = w0
    w3 = model.tire_forces(s3, delta, Fz)
    k_roll = max(abs(k) for k in w3["kappa"])
    s3[IW[0]] *= 1.10                       # overspeed FL — the STEERED wheel
    w4 = model.tire_forces(s3, delta, Fz)
    check("C2", "per-wheel rolling → κ≈0 on all four; +10% overspeed → κ≈0.10",
          k_roll < 1e-6 and abs(w4["kappa"][0] - 0.10) < 2e-3,
          f"κ_roll = {k_roll:.2e} (worst of 4), κ_overspeed = {w4['kappa'][0]:.4f} "
          "(per-wheel contact speed incl. the r·y term, resolved along the "
          "wheel's own heading)")

    # C2b: the front rolling speed is a WHEEL-FRAME quantity. Building it the
    # rear way — (vx − r·y)/r_w, ignoring the steer projection — must FAIL.
    s_naive = list(s3)
    s_naive[IW[0]] = (vx - r * model.wheel_xy[0][1]) / vp.r_wheel
    s_naive[IW[1]] = (vx - r * model.wheel_xy[1][1]) / vp.r_wheel
    w_naive = model.tire_forces(s_naive, delta, Fz)
    k_naive = max(abs(w_naive["kappa"][0]), abs(w_naive["kappa"][1]))
    vcx_f = contact_speeds(model, s, delta)[0]
    check("C2b", "steered front rolls at v_cx/r_w, not (vx−r·y)/r_w",
          k_naive > 1e-4,
          f"the naive rear-style speed leaves κ = {k_naive:.2e} on the fronts "
          f"at δ = {math.degrees(delta):.1f}° (v_cx = {vcx_f:.4f} vs "
          f"vx − r·y = {vx - r * model.wheel_xy[0][1]:.4f} m/s)")

    # C3: the s-diff target. In a steady left turn the RIGHT (outer) wheel
    # must spin faster; target Δω = r·track/r_wheel from wheel path speeds
    # vx ± r·track/2 — re-derived here from the wheel positions directly.
    y_RL, y_RR = model.wheel_xy[2][1], model.wheel_xy[3][1]
    v_RL = vx - r * y_RL
    v_RR = vx - r * y_RR
    dw_indep = (v_RR - v_RL) / vp.r_wheel
    dw_ctrl = r * vp.track_r / vp.r_wheel
    check("C3", "Δω target matches independent wheel-path derivation, outer=faster",
          abs(dw_indep - dw_ctrl) < 1e-12 and (dw_ctrl > 0) == (r > 0),
          f"both {dw_ctrl:.4f} rad/s for r={r} (right wheel faster in left turn)")

    # C5: Ackermann steering geometry. Independent references: at fA=0 the
    # wheels must be exactly parallel (the original behavior); at fA=1 the
    # angles must match the textbook atan(L/(R ∓ t/2)) values computed here
    # from scratch; a right turn must mirror a left turn exactly.
    d23 = math.radians(23.0)
    pl = VehicleParams(ackermann_frac=0.0)
    check("C5", "fA=0 keeps parallel steer (pre-Ackermann behavior)",
          front_steer_angles(pl, d23) == (d23, d23))
    pa = VehicleParams(ackermann_frac=1.0)
    dFL, dFR = front_steer_angles(pa, d23)
    R = pa.wheelbase / math.tan(d23)
    want_in = math.atan(pa.wheelbase / (R - pa.track_f / 2))
    want_out = math.atan(pa.wheelbase / (R + pa.track_f / 2))
    check("C5", "fA=1 at 23°: inner/outer match textbook Ackermann exactly",
          abs(dFL - want_in) < 1e-12 and abs(dFR - want_out) < 1e-12
          and dFL > d23 > dFR,
          f"FL(inner) {math.degrees(dFL):.2f}° / FR(outer) "
          f"{math.degrees(dFR):.2f}° for a 23° left-turn command")
    dFL_r, dFR_r = front_steer_angles(pa, -d23)
    check("C5", "right turn mirrors left turn exactly (FR becomes inner)",
          abs(dFL_r + dFR) < 1e-12 and abs(dFR_r + dFL) < 1e-12)
    d5 = math.radians(5.0)
    a5 = front_steer_angles(pa, d5)
    info("C5", "Ackermann split magnitude",
         f"±{math.degrees((a5[0] - a5[1]) / 2):.2f}° at 5° steer, "
         f"±{math.degrees((dFL - dFR) / 2):.2f}° at 23° — negligible in "
         "normal driving, real at full lock. ACKERMANN_FRACTION is still "
         "0.0 (parallel) pending the steering team's curve interpretation")

    # C8: the zero-front-torque identity — the regression lever for the whole
    # four-wheel change. With no front torque the front wheels must sit at
    # kappa = 0 and make NO longitudinal force, i.e. reproduce the old
    # free-roller branch exactly. (A steady-trim identity, not a trajectory
    # claim: in a TRANSIENT the fronts legitimately make force — that is their
    # rotational inertia, and D1 measures it against a closed form.)
    s8 = list(s)
    for j, w0 in zip(IW, free_rolling_omegas(model, s, delta)):
        s8[j] = w0
    w8 = model.tire_forces(s8, delta, Fz)
    worst_fx = max(abs(w8["Fx_w"][0]), abs(w8["Fx_w"][1]))
    worst_fy = max(abs(w8["Fy_w"][i] - model.tires[i].lateral(w8["alpha"][i], Fz[i]))
                   for i in (0, 1))
    check("C8", "zero front torque → fronts make no Fx and pure lateral Fy",
          worst_fx < 1e-9 and worst_fy < 1e-9,
          f"front |Fx_w| = {worst_fx:.1e} N, |Fy_w − lateral()| = {worst_fy:.1e} N "
          "— combined(0, α, Fz) is bit-identically (0, lateral(α, Fz))")

    # C6: the yaw moment a FRONT left/right torque split makes. Until the
    # fronts were driven, front Fx_w was identically 0, so half of the
    # body-frame rotation at the bottom of tire_forces() was dead code.
    #
    # At delta = 0, with Fx on the fronts only:
    #     Mz = Σ(x·Fy_b − y·Fx_b) = −y_FL·Fx_FL − y_FR·Fx_FR
    #        = −(t_f/2)·Fx_FL + (t_f/2)·Fx_FR = +(t_f/2)·ΔFx
    # with ΔFx = Fx_FR − Fx_FL. Note the HALF: the couple arm is the
    # half-track, not the track.
    dFx = 600.0
    fx = [-dFx / 2.0, +dFx / 2.0, 0.0, 0.0]
    Mz_front = sum(x * 0.0 - y * f for (x, y), f in zip(model.wheel_xy, fx))
    check("C6", "front L/R force split: Mz = +(track_f/2)·ΔFx",
          abs(Mz_front - 0.5 * vp.track_f * dFx) < 1e-9,
          f"Mz {Mz_front:+.3f} vs hand-derived {0.5 * vp.track_f * dFx:+.3f} N·m "
          f"for ΔFx = {dFx:.0f} N (positive ΔFx → nose LEFT)")

    # C6b: a DRIVEN AND STEERED front wheel. Its drive force rotates into BOTH
    # body axes, so it adds an x·Fy yaw term the rear axle has no equivalent
    # of. Check the rotation per wheel straight out of the plant.
    s6 = list(s)
    for j, w0 in zip(IW, free_rolling_omegas(model, s, delta)):
        s6[j] = w0
    s6[IW[0]] *= 1.05                      # spin FL up: real front Fx at last
    w6 = model.tire_forces(s6, delta, Fz)
    st6 = wheel_steer_angles(vp, delta)
    worst_rot = 0.0
    for i in range(NWHEELS):
        cd, sd = math.cos(st6[i]), math.sin(st6[i])
        worst_rot = max(worst_rot,
                        abs(w6["Fx_b"][i] - (w6["Fx_w"][i] * cd - w6["Fy_w"][i] * sd)),
                        abs(w6["Fy_b"][i] - (w6["Fx_w"][i] * sd + w6["Fy_w"][i] * cd)))
    check("C6b", "driven+steered front: body-frame rotation exact per wheel",
          worst_rot < 1e-12 and abs(w6["Fx_w"][0]) > 1.0,
          f"worst rotation error {worst_rot:.1e} N with front Fx_w = "
          f"{w6['Fx_w'][0]:.1f} N at δ = {math.degrees(delta):.1f}°")
    info("C6b", "the steer-projection yaw arm the rear axle does not have",
         "a·sin δ vs (t_f/2)·cos δ = " + ", ".join(
             f"{math.degrees(d):.0f}°: {100 * vp.a * math.sin(d) / (0.5 * vp.track_f * math.cos(d)):.0f}%"
             for d in (math.radians(5), math.radians(8), math.radians(23))))

    # C7: at delta = 0 the two axles buy identical yaw per newton — but only
    # because track_f == track_r on this car. The allocator needs to know that.
    fx_r = [0.0, 0.0, -dFx / 2.0, +dFx / 2.0]
    Mz_rear = sum(x * 0.0 - y * f for (x, y), f in zip(model.wheel_xy, fx_r))
    check("C7", "front and rear L/R splits give the same Mz at δ = 0",
          abs(Mz_front - Mz_rear) < 1e-12,
          f"{Mz_front:+.3f} vs {Mz_rear:+.3f} N·m — equal ONLY because "
          f"track_f = track_r = {vp.track_f:.2f} m today")

    # C4: yaw moment sum — synthetic forces with a hand-computed answer.
    Mz_hand = 0.0
    forces = [(120.0, 800.0), (-40.0, 900.0), (300.0, 400.0), (250.0, 350.0)]
    for (x, y), (fx, fy) in zip(model.wheel_xy, forces):
        Mz_hand += x * fy - y * fx
    # independent recomputation with explicit numbers
    a_, b_, tf, tr = vp.a, vp.b, vp.track_f / 2, vp.track_r / 2
    Mz_indep = (a_ * 800 - tf * 120) + (a_ * 900 - tf * 40) + \
               (-b_ * 400 - tr * 300) + (-b_ * 350 + tr * 250)
    # (FR term: −y·Fx = −(−tf)·(−40) = −tf·40 — the first version of this
    # check had +tf·40, a double-negative slip in the CHECK, not the sim.)
    check("C4", "yaw-moment arm signs (hand-expanded per wheel)",
          abs(Mz_hand - Mz_indep) < 1e-9, f"Mz = {Mz_hand:.1f} N·m")


# ═══════════════════════════════════ D. rigid-body EOM & integration
def section_d():
    vp, tp_f, tp_r, cp = default_setup()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))

    # D1: coast-down. δ=0, T=0 → only drag decelerates. Closed form for
    # dv/dt = −k·v²/m_eff with m_eff = m + 2·(I_wf + I_wr)/r_w² — ALL FOUR
    # wheels store rotational KE now that the fronts have a spin state.
    coast = Maneuver("coast", "coast", 3.0, 15.0, lambda t: (0.0, 0.0))
    ctrl = make_configs(vp, tp_f, tp_r, cp)[0]
    log = simulate(model, ctrl, coast, dt=2.5e-4)
    k = 0.5 * RHO_AIR * vp.CdA
    m_eff = vp.m_total + 2 * (vp.I_wheel_f + vp.I_wheel_r) / vp.r_wheel ** 2
    v_pred = 15.0 / (1 + k * 15.0 * 3.0 / m_eff)
    v_sim = log["vx"][-1]
    # 0.1%, not 1%: with 4 spinning wheels the WRONG (2-wheel) m_eff is only
    # 0.63% away, so a 1% tolerance cannot tell the two apart and would pass
    # either. At 0.1% this is the strongest single proof that the two new front
    # spin states integrate correctly.
    check("D1", "coast-down matches closed-form drag solution (with wheel KE)",
          abs(v_sim / v_pred - 1) < 1e-3,
          f"vx(3 s): sim {v_sim:.3f} vs closed form {v_pred:.3f} m/s")
    check("D1", "coast stays perfectly straight",
          abs(log["r"]).max() < 1e-10 and abs(log["Y"]).max() < 1e-8,
          f"max |r| = {abs(log['r']).max():.1e} rad/s")
    m_nowheel = 15.0 / (1 + k * 15.0 * 3.0 / vp.m_total)
    info("D1", "wheel-inertia effect on coast",
         f"{abs(m_nowheel - v_pred) * 1000:.0f} mm/s over 3 s — I_WHEEL is "
         "visible but small here; it matters most in spin-up, not coasting")

    # D5: the pointwise mechanism whose integral D1 checks. Coasting with zero
    # torque, the front wheels are DECELERATING the car's forward motion into
    # their own rotation, so they must sit at a small POSITIVE slip ratio and
    # push the car along — the free-roller model made this force identically 0.
    kf = np.concatenate([log["kFL"], log["kFR"]])
    check("D5", "coasting fronts carry a small positive slip ratio (their KE)",
          kf.min() >= 0.0 and kf.max() < 5e-3,
          f"front κ over the coast: {kf.min():.2e} … {kf.max():.2e} — the "
          f"{2 * vp.I_wheel_f / vp.r_wheel ** 2:.1f} kg of front rotational "
          "inertia the old massless free-roller ignored")

    # D2: mirror symmetry. Same maneuver with steer sign flipped must give
    # the exactly mirrored trajectory — catches ANY left/right sign error.
    man_l = step_steer(delta_deg=+5.0)
    man_r = step_steer(delta_deg=-5.0)
    ctrl_l = make_configs(vp, tp_f, tp_r, cp)[1]    # s-diff
    ctrl_r = make_configs(vp, tp_f, tp_r, cp)[1]
    log_l = simulate(model, ctrl_l, man_l, dt=2.5e-4)
    log_r = simulate(model, ctrl_r, man_r, dt=2.5e-4)
    errs = {
        "r": np.abs(log_l["r"] + log_r["r"]).max(),
        "Y": np.abs(log_l["Y"] + log_r["Y"]).max(),
        "beta": np.abs(log_l["beta"] + log_r["beta"]).max(),
        "T_RL vs T_RR": np.abs(log_l["T_RL"] - log_r["T_RR"]).max(),
    }
    check("D2", "mirrored steer gives exactly mirrored car (all controllers on)",
          max(errs.values()) < 1e-9,
          "max asymmetry " + ", ".join(f"{k} {v:.1e}" for k, v in errs.items()))

    # D3: steady-state force balance. Late in the step steer the car is in a
    # steady circle: measured ay must equal r·vx (centripetal identity) and
    # yaw acceleration must be ~0.
    i0 = int(0.8 * len(log_l["t"]))
    ay_meas = log_l["ay"][i0:]
    rv = log_l["r"][i0:] * log_l["vx"][i0:]
    check("D3", "steady cornering: ay = r·vx (centripetal balance)",
          np.abs(ay_meas - rv).max() / np.abs(rv).mean() < 0.02,
          f"worst |ay − r·vx| = {np.abs(ay_meas - rv).max():.3f} m/s² at "
          f"ay ≈ {rv.mean():.2f} m/s²")

    # D4: integration convergence — halving dt twice must not move the
    # answer. If it does, dt is too coarse or the integrator is broken.
    man = corner_exit()
    outs = []
    for dt in (2.5e-4, 1.25e-4, 6.25e-5):
        c = make_configs(vp, tp_f, tp_r, cp)[1]
        lg = simulate(model, c, man, dt=dt, log_every=max(1, int(1e-3 / dt)))
        outs.append((lg["r"][-1], lg["X"][-1], lg["Y"][-1],
                     metrics(lg, vp)["dw RMSE [rad/s]"]))
    d12 = max(abs((outs[0][i] - outs[1][i])) for i in range(4))
    d23 = max(abs((outs[1][i] - outs[2][i])) for i in range(4))
    check("D4", "RK4 dt-refinement converged (dt, dt/2, dt/4)",
          d12 < 5e-3 and d23 <= d12 + 1e-12,
          f"max state drift dt→dt/2: {d12:.2e}, dt/2→dt/4: {d23:.2e}")


# ═══════════════════════ E. steady cornering vs independent bicycle model
def section_e():
    # Aero off, tiny steer → the two-track sim must reproduce the LINEAR
    # single-track (bicycle) model, which we solve here from scratch:
    #     r_ss = vx·δ / (L + K_us·vx²),  K_us = m/L·(b/C_f − a/C_r)
    # with C_axle = c_alpha·Fz_axle. This formula is derived independently
    # (steady-state moment + force balance), not taken from the code.
    vp = VehicleParams(ClA=0.0, CdA=0.0)
    tp_f = TireParams(c_alpha=15.0)
    tp_r = TireParams(c_alpha=16.0)
    cp = ControlParams()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))

    m, L = vp.m_total, vp.wheelbase
    C_f = tp_f.c_alpha * m * G * vp.b / L
    C_r = tp_r.c_alpha * m * G * vp.a / L
    K_us = m / L * (vp.b / C_f - vp.a / C_r)
    info("E1", "understeer gradient (independent derivation)",
         f"K_us = {K_us * 1e3:.3f} ×10⁻³ s²/m "
         f"({'understeer' if K_us > 0 else 'OVERSTEER'}) — "
         f"C_f {C_f:.0f}, C_r {C_r:.0f} N/rad")

    for vx0, ddeg in ((10.0, 1.0), (18.0, 1.0), (14.0, 2.0)):
        d = math.radians(ddeg)
        man = Maneuver("ss", "ss", 6.0, vx0,
                       lambda t, d=d: (d if t > 0.5 else 0.0, 0.0))
        ctrl = make_configs(vp, tp_f, tp_r, cp)[0]      # open — no controller
        log = simulate(model, ctrl, man, dt=2.5e-4)
        i0 = int(0.9 * len(log["t"]))
        r_sim = log["r"][i0:].mean()
        vx_now = log["vx"][i0:].mean()                  # tire losses shave a bit
        r_bike = vx_now * d / (L + K_us * vx_now ** 2)
        ay_g = abs(r_sim * vx_now) / G
        check("E1", f"two-track matches bicycle model ({vx0:.0f} m/s, {ddeg:.0f}°, "
              f"ay={ay_g:.2f} g)",
              abs(r_sim / r_bike - 1) < 0.03,
              f"r_sim {r_sim:.4f} vs bicycle {r_bike:.4f} rad/s "
              f"({(r_sim / r_bike - 1) * 100:+.1f}%)")


# ═══════════════════════════════════ F. controller limit unit tests
def section_f():
    vp, tp_f, tp_r, cp = default_setup()
    # index 2 is the rear-drive s-diff reference: F8-F11 test the sdiff.c LAW,
    # not the four-corner allocator, so they must hold onto that controller.
    ctrl = make_configs(vp, tp_f, tp_r, cp)[2]
    Tmax = vp.T_wheel_max

    # F1: no limits active — pure split.
    TL, TR = ctrl._apply_limits(100.0, 60.0, 15.0, 65.0, 65.0)
    check("F1", "unsaturated split: base±dT/2 exactly",
          abs(TL - 70) < 1e-12 and abs(TR - 130) < 1e-12, f"{TL:.0f}/{TR:.0f} N·m")

    # F2: upper clip — the yaw split must survive, thrust must give way.
    TL, TR = ctrl._apply_limits(250.0, 100.0, 15.0, 65.0, 65.0)
    check("F2", "upper torque clip preserves ΔT (thrust sacrificed)",
          abs((TR - TL) - 100.0) < 1e-9 and TR <= Tmax + 1e-9,
          f"{TL:.1f}/{TR:.1f}, ΔT = {TR - TL:.1f} N·m, cap {Tmax:.0f}")

    # F3: regen floor below the cutoff speed — no negative torque at all.
    TL, TR = ctrl._apply_limits(5.0, -80.0, 1.0, 5.0, 5.0)
    check("F3", "below regen cutoff no wheel torque is negative",
          TL >= -1e-9 and TR >= -1e-9, f"{TL:.1f}/{TR:.1f} N·m at vx=1.0")
    info("F3", "base-shift direction at the floor",
         f"request was base 5, ΔT −80 → delivered {TL:.0f}/{TR:.0f}: the shift "
         f"ADDS {TL + TR - 10:.0f} N·m of thrust the driver didn't ask for to "
         "protect the yaw split — policy says sacrifice thrust, but at the "
         "LOWER bound it manufactures thrust instead (finding #2)")

    # F4: an absurd ΔT request gets clipped into the physical range.
    TL, TR = ctrl._apply_limits(0.0, 5000.0, 15.0, 65.0, 65.0)
    check("F4", "ΔT request clipped to physical torque range",
          TR <= Tmax + 1e-9 and TL >= -Tmax - 1e-9 and TR - TL <= 2 * Tmax + 1e-9,
          f"{TL:.0f}/{TR:.0f} N·m")

    # F5: per-motor power cap at high wheel speed.
    w = 140.0                                    # rad/s ≈ 32 m/s ground speed
    TL, TR = ctrl._apply_limits(250.0, 0.0, 32.0, w, w)
    check("F5", "per-motor power cap enforced",
          TL * w <= vp.motor_P_peak + 1e-6,
          f"P = {TL * w / 1e3:.1f} kW vs cap {vp.motor_P_peak / 1e3:.0f} kW")

    # F6: 80 kW total cap.
    TL, TR = ctrl._apply_limits(220.0, 0.0, 34.0, 150.0, 150.0)
    P = TL * 150 + TR * 150
    check("F6", "80 kW total cap enforced", P <= vp.P_total_max + 1e-6,
          f"P_total = {P / 1e3:.1f} kW")

    # F7: power clamps CAN break the preserved yaw split — quantify.
    TL, TR = ctrl._apply_limits(200.0, 120.0, 32.0, 140.0, 140.0)
    info("F7", "yaw split vs power caps",
         f"requested ΔT 120 → delivered {TR - TL:.1f} N·m at 32 m/s: the "
         "per-motor/total power clamps clip each side independently, quietly "
         "shrinking yaw authority at high speed (finding #3). Real motors do "
         "this too — but the sim should REPORT it, not hide it")

    # F8: open-loop s-diff mirrors sdiff.c. Settled multipliers at full
    # LEFT lock, computed by hand from the YAML constants:
    #   f = 1 - k_derate (floored at f_min),  g_in = 1 - k_inner,  g_out = 1
    #   RL (inner) = T_base*f*g_in,  RR (outer) = T_base*f
    s = [0.0] * NSTATES
    s[IVX] = 10.0
    for j in IW:
        s[j] = 10.0 / vp.r_wheel
    ctrl.reset()
    for _ in range(200):                     # 2 s at 10 ms — slew fully settled
        d = ctrl.update(s, +cp.delta_max, 200.0, 0.01)
    f_exp = max(1.0 - cp.k_derate, cp.f_min)
    RL_exp = 100.0 * f_exp * (1.0 - cp.k_inner)
    RR_exp = 100.0 * f_exp
    check("F8", "s-diff full LEFT lock: inner=RL derated, outer=RR budget only",
          abs(d.T_RL - RL_exp) < 1e-9 and abs(d.T_RR - RR_exp) < 1e-9,
          f"RL {d.T_RL:.2f} / RR {d.T_RR:.2f} vs expected {RL_exp:.2f} / {RR_exp:.2f} N·m")

    # F9: mirror — full RIGHT lock swaps the sides exactly.
    ctrl.reset()
    for _ in range(200):
        d = ctrl.update(s, -cp.delta_max, 200.0, 0.01)
    check("F9", "s-diff full RIGHT lock: mirror of F8",
          abs(d.T_RR - RL_exp) < 1e-9 and abs(d.T_RL - RR_exp) < 1e-9,
          f"RL {d.T_RL:.2f} / RR {d.T_RR:.2f}")

    # F10: inside the deadband there is NO inner/outer split — but the
    # friction budget f still applies (sdiff.c gates only g_left/g_right on
    # DEADBAND, not f). Both sides equal, both = T_base * f.
    ctrl.reset()
    for _ in range(200):
        d = ctrl.update(s, 0.5 * cp.deadband, 200.0, 0.01)
    # (with k_derate = 0 the budget term is inert and f_db is exactly 1.0 —
    # what F10 still proves is that the two sides stay equal.)
    f_db = max(1.0 - cp.k_derate * (0.5 * cp.deadband / cp.delta_max), cp.f_min)
    check("F10", "s-diff inside deadband: equal sides, budget-only derate",
          abs(d.T_RL - d.T_RR) < 1e-12 and abs(d.T_RL - 100.0 * f_db) < 1e-9,
          f"RL {d.T_RL:.2f} / RR {d.T_RR:.2f} (f = {f_db:.4f}, no L/R split)")

    # F11: slew actually limits — one 10 ms step from straight toward full
    # lock may move a multiplier by at most rate*dt. Probed on g_left, the
    # INNER multiplier: with k_derate = 0 (the current sdiff.c) f's target is
    # 1.0, so f never leaves its 1.0 start and cannot exercise the slew.
    ctrl.reset()
    d = ctrl.update(s, +cp.delta_max, 200.0, 0.01)
    check("F11", "s-diff slew: one step moves g_left by <= rate*dt",
          abs(d.g_left_appl - (1.0 - cp.rate * 0.01)) < 1e-9,
          f"g_left after one step {d.g_left_appl:.4f} "
          f"(limit {1.0 - cp.rate * 0.01:.4f})")


def section_f2():
    """The four-corner allocator's limit chain — F12..F16."""
    vp, tp_f, tp_r, cp = default_setup()
    T_max = vp.T_wheel_max
    alloc = make_configs(vp, tp_f, tp_r, cp)[1]
    openc = make_configs(vp, tp_f, tp_r, cp)[0]
    s = [0.0] * NSTATES
    s[IVX] = 15.0
    for j in IW:
        s[j] = 15.0 / vp.r_wheel

    # F12: with the law switched off the allocator IS an open differential —
    # exactly T_req/4 at every corner. This is why the baseline config shares
    # this code path instead of being a separate class.
    d = openc.update(s, 0.1, 400.0, 0.01)
    check("F12", "law off → exact even four-way split (an open diff)",
          all(abs(t - 100.0) < 1e-12 for t in d.T),
          " / ".join(f"{t:.6f}" for t in d.T) + " N·m for a 400 N·m request")

    # F13: unsaturated, the allocation is conserved — nothing is invented or
    # lost between the request and the four commands.
    d = alloc.update(s, 0.0, 400.0, 0.01)
    check("F13", "unsaturated: the four commands sum to the request",
          abs(sum(d.T) - 400.0) < 1e-9,
          f"Σ T = {sum(d.T):.6f} vs 400 N·m requested, split "
          f"{d.T_FL + d.T_FR:.1f} front / {d.T_RL + d.T_RR:.1f} rear")

    # F14: THE invariant the limit chain exists for. The per-motor power clip
    # is |T| <= P/omega, so it always bites the FASTER wheel first — which in a
    # corner is the OUTER wheel, the one a yaw split just gave more torque to.
    # Clipping each wheel independently after a difference-preserving shift
    # shrinks the split and can INVERT it. Check the axle difference keeps its
    # sign through the whole chain at a speed where the clip is active.
    w_fast = (100.0, 140.0, 100.0, 140.0)
    T_req4 = (200.0, 300.0, 200.0, 300.0)      # outer wheel asked for more
    T_out, _ = alloc._apply_limits4(T_req4, w_fast, 32.0)
    d_in = T_req4[1] - T_req4[0]
    d_out_f = T_out[1] - T_out[0]
    d_out_r = T_out[3] - T_out[2]
    check("F14", "per-motor power clip cannot invert an axle's torque split",
          d_out_f > 0.0 and d_out_r > 0.0,
          f"requested ΔT {d_in:+.0f} → delivered {d_out_f:+.1f} front / "
          f"{d_out_r:+.1f} rear at ω = 100/140 rad/s (a naive per-wheel clip "
          f"gives −30.1 here — sign inverted)")

    # F15: every hard limit is respected simultaneously.
    ok_T = all(-T_max - 1e-6 <= t <= T_max + 1e-6 for t in T_out)
    ok_P = all(abs(t * w) <= vp.motor_P_peak + 1e-6
               for t, w in zip(T_out, w_fast))
    ok_tot = sum(t * w for t, w in zip(T_out, w_fast)) <= vp.P_total_max + 1e-6
    check("F15", "four-wheel limits: per-wheel torque, per-motor power, total cap",
          ok_T and ok_P and ok_tot,
          f"peak |T| {max(abs(t) for t in T_out):.1f} ≤ {T_max:.0f} N·m; peak "
          f"per-motor {max(abs(t * w) for t, w in zip(T_out, w_fast)) / 1e3:.1f} ≤ "
          f"{vp.motor_P_peak / 1e3:.0f} kW; total "
          f"{sum(t * w for t, w in zip(T_out, w_fast)) / 1e3:.1f} ≤ "
          f"{vp.P_total_max / 1e3:.0f} kW")

    # F16: the 80 kW rules cap is STRUCTURALLY unreachable, and saying so is
    # more useful than a check that cannot fail.
    head = vp.driven_wheels * vp.motor_P_peak - vp.P_total_max
    info("F16", "the total power cap has zero headroom and cannot fire",
         f"driven_wheels × motor_power_peak = "
         f"{vp.driven_wheels * vp.motor_P_peak / 1e3:.0f} kW vs a "
         f"{vp.P_total_max / 1e3:.0f} kW cap — headroom {head / 1e3:.0f} kW. The "
         "per-motor clip already implies the total, so the rules branch is dead "
         "code at these numbers. It stays because motor_power_peak is a DERIVED "
         "estimate (kit curves scaled to 380 V, three methods spanning "
         "18.1–20.3 kW) — a dyno above 20 kW makes the cap live.")

    # F17: the axle split follows the estimated load split, and that estimate
    # tracks the plant's own wheel_loads() including the AERO term.
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    worst = 0.0
    for vx in (5.0, 15.0, 25.0):
        for ax in (-5.0, 0.0, 5.0, 10.0):
            Fz = model.wheel_loads(vx, ax, 0.0)
            want = (Fz[0] + Fz[1]) / sum(Fz)
            worst = max(worst, abs(axle_load_fraction(vp, vx, ax) - want))
    check("F17", "allocator's load estimate matches the plant's wheel_loads",
          worst < 1e-12,
          f"worst front-share error {worst:.1e} over vx = 5…25 m/s, "
          "ax = −5…10 m/s² (aero included — dropping it costs 3.3 points at 1.2 g)")


# ═══════════════════════ G. in-run invariant audit (standard maneuvers)
class AuditModel(VehicleModel):
    """Same physics; records invariants at every derivative evaluation."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.reset_audit()

    def reset_audit(self):
        self.min_Fz = float("inf")
        self.max_util = 0.0
        self.mu_floor_hits = 0
        self.max_w = 0.0
        self.max_kappa = 0.0
        self.load_id_err = 0.0

    def derivatives(self, s, delta, T_wheel):
        ds, inf_ = super().derivatives(s, delta, T_wheel)
        Fz = inf_["Fz"]
        self.min_Fz = min(self.min_Fz, min(Fz))
        if min(Fz) > 0.0:
            want = self.p.m_total * G + 0.5 * RHO_AIR * self.p.ClA * s[IVX] ** 2
            self.load_id_err = max(self.load_id_err, abs(sum(Fz) / want - 1))
        for i in range(4):
            if Fz[i] <= 0:
                continue
            t = self.tires[i]
            self.max_util = max(self.max_util, math.hypot(
                inf_["Fx_w"][i] / (t.mu_x(Fz[i]) * Fz[i]),
                inf_["Fy_w"][i] / (t.mu(Fz[i]) * Fz[i])))
            if t.mu(Fz[i]) <= 0.5 * t.p.mu0 + 1e-12:
                self.mu_floor_hits += 1
        self.max_w = max([self.max_w] + [abs(s[j]) for j in IW])
        self.max_kappa = max([self.max_kappa] + [abs(k) for k in inf_["kappa"]])
        return ds, inf_


def section_g():
    vp, tp_f, tp_r, cp = default_setup()
    audit = AuditModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    Tmax = vp.T_wheel_max

    for man in (step_steer(), corner_exit(), slalom()):
        for ci in (0, 1):                       # open & s-diff
            audit.reset_audit()
            ctrl = make_configs(vp, tp_f, tp_r, cp)[ci]
            log = simulate(audit, ctrl, man, dt=2.5e-4)
            nm = f"{man.slug}/{ctrl.name}"
            check("G1", f"{nm}: load identity holds throughout",
                  audit.load_id_err < 1e-9, f"worst {audit.load_id_err:.1e}")
            check("G2", f"{nm}: friction circle respected throughout",
                  audit.max_util <= 1.0 + 1e-9, f"peak util {audit.max_util:.3f}")
            peak_T = max(np.abs(log["T_" + w]).max() for w in WHEEL_NAMES)
            check("G3", f"{nm}: torque commands inside ±T_wheel_max",
                  peak_T <= Tmax + 1e-6, f"peak |T| {peak_T:.0f} N·m (worst of 4)")
            check("G4", f"{nm}: total power under the 80 kW cap",
                  log["P_total"].max() <= vp.P_total_max * 1.001,
                  f"peak {log['P_total'].max() / 1e3:.1f} kW")
            mot_rpm = audit.max_w * vp.gear_ratio / (math.pi / 30)
            check("G5", f"{nm}: motor speed under 20 krpm",
                  mot_rpm < 20000, f"peak {mot_rpm:.0f} rpm")
            if (man.slug, ci) == ("corner_exit", 0):
                info("G6", f"{nm}: closest approach to wheel lift",
                     f"min Fz = {audit.min_Fz:.0f} N (0 = airborne wheel)")
                info("G6", f"{nm}: mu floor engaged",
                     f"{audit.mu_floor_hits} times (0 = load sens. stayed in "
                     "its valid linear range)")
                info("G6", f"{nm}: peak tire utilization / slip ratio",
                     f"{audit.max_util:.2f} / {audit.max_kappa:.3f}")

    # G7: wheel-speed kinematic consistency, end-to-end through the
    # integrator: in the steady part of the step steer the actual rear
    # wheel-speed difference must land near r·track/r_w on its own.
    ctrl = make_configs(vp, tp_f, tp_r, cp)[0]       # OPEN — nobody forcing it
    log = simulate(audit, ctrl, step_steer(), dt=2.5e-4)
    i0 = int(0.8 * len(log["t"]))
    dw_act = (log["wRR"] - log["wRL"])[i0:].mean()
    dw_kin = (log["r"][i0:] * vp.track_r / vp.r_wheel).mean()
    check("G7", "open-diff wheels settle to the kinematic Δω by themselves",
          abs(dw_act - dw_kin) / abs(dw_kin) < 0.25,
          f"actual {dw_act:.3f} vs kinematic {dw_kin:.3f} rad/s (gap = real "
          "slip-ratio difference from the torque split, not an error)")


# ═══════════════════ H. controller at a realistic VCU rate
def section_h():
    vp, tp_f, tp_r, cp = default_setup()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    man = corner_exit()
    rows = []
    for hz, every in (("4 kHz (as simulated)", 1), ("1 kHz", 4), ("100 Hz", 40)):
        ctrl = make_configs(vp, tp_f, tp_r, cp)[1]
        log = simulate(model, ctrl, man, dt=2.5e-4, ctrl_every=every)
        dw_rmse = float(np.sqrt(np.mean(
            (log["dw_target"] - (log["wRR"] - log["wRL"])) ** 2)))
        finished = log["t"][-1] >= man.duration - 0.01
        rows.append((hz, dw_rmse, finished))
    base = rows[0]
    for hz, dr, fin in rows:
        info("H1", f"s-diff at {hz}",
             f"Δω RMSE {dr:.4f} ({(dr / base[1] - 1) * 100:+.0f}%)"
             + ("" if fin else "  ← SPUN"))
    degraded = rows[2][1] > 3.0 * base[1] or not rows[2][2]
    check("H1", "gains survive a realistic 100 Hz VCU rate without instability",
          rows[2][2],
          "did not spin at 100 Hz" if rows[2][2] else "SPINS at 100 Hz — gains "
          "are tuned to the unrealistic 4 kHz update rate")
    if degraded and rows[2][2]:
        info("H1", "degradation at 100 Hz",
             "stable but markedly worse — retune gains at the real VCU rate "
             "before quoting controller performance (finding #1)")


# ═══════════════════════ I. sensor stack (sensors.py)
def section_i():
    from model.config import cfg
    from model.sensors import (SensorSuite, DriverAdapter, steer_map_deg,
                         steer_map_inv_deg)
    vp, tp_f, tp_r, cp = default_setup()

    # I1: steering map — centered, odd, and the driver-adapter inverse is
    # an exact round trip over the full ±23° road-wheel range.
    worst = max(abs(steer_map_deg(steer_map_inv_deg(y)) - y)
                for y in np.linspace(-23.0, 23.0, 201))
    check("I1", "steer map: inverse→forward is identity over ±23°",
          worst < 1e-9 and steer_map_deg(0.0) == 0.0
          and abs(steer_map_deg(-40.0) + steer_map_deg(40.0)) < 1e-12,
          f"worst round-trip error {worst:.1e} deg; map(0)=0; odd-symmetric")
    info("I1", "handwheel at full lock",
         f"{steer_map_inv_deg(23.0):.0f}° of steering wheel for 23° road wheel")

    # I2: WSS chain — motor rpm ÷ planetary ratio reproduces wheel speed
    # exactly with quantization off, and within one LSB with it on.
    sen = SensorSuite(vp, noise=False)
    s = [0.0] * NSTATES
    s[IVX] = 15.0
    for j, w0 in zip(IW, (40.0, 50.0, 60.0, 70.0)):
        s[j] = w0
    from model.sensors import DriverInputs
    sr = sen.measure(s, DriverInputs(), {"ax": 0, "ay": 0}, 0.01, False)
    got = [getattr(sr, "wheel_speed_" + nm) for nm in WHEEL_NAMES]
    want = [40.0, 50.0, 60.0, 70.0]
    check("I2", "WSS: motor rpm → wheel speed chain exact on all four corners",
          max(abs(g - w) for g, w in zip(got, want)) < 1e-12,
          " ".join(f"{nm} {g:.4f}" for nm, g in zip(WHEEL_NAMES, got)) + " rad/s")
    lsb = cfg.sensors.wheel_speed.quant_rpm * (math.pi / 30) / vp.gear_ratio
    info("I2", "WSS quantization at the wheel",
         f"1 motor-rpm LSB = {lsb * vp.r_wheel * 1000:.1f} mm/s of ground speed "
         "— the resolver-over-CAN path is effectively noise-free")

    # I3: EV.4.7 plausibility latch — unit-tested against the rule text.
    ctrl = make_configs(vp, tp_f, tp_r, cp)[0]
    ctrl.reset()
    from model.sensors import SensorReadings
    def step(apps, bps):
        sr = SensorReadings(apps_pct=apps, bps_bar=bps, vx_est=10.0)
        sr.wheel_speed_RL = sr.wheel_speed_RR = 10.0 / vp.r_wheel
        return ctrl.update_from_sensors(sr, 0.01)
    d1 = step(60.0, 0.0)                    # drive
    d2 = step(60.0, 20.0)                   # illegal overlap → cut
    d3 = step(15.0, 0.0)                    # released but still >5% → latched
    d4 = step(3.0, 0.0)                     # below 5% → restored
    d5 = step(60.0, 0.0)                    # drives again
    check("I3", "EV.4.7 plausibility: cut at >25%+brake, latched until <5%",
          d1.T_RL > 50 and d2.T_RL == 0.0 and d3.T_RL == 0.0
          and d5.T_RL > 50 and not ctrl.plaus_cut,
          f"T through the sequence: {d1.T_RL:.0f} → {d2.T_RL:.0f} → "
          f"{d3.T_RL:.0f} → {d4.T_RL:.0f} → {d5.T_RL:.0f} N·m")

    # I4: clean sensors ≈ perfect state. Noise/quantization OFF, VCU at the
    # physics rate: the only remaining difference is the vx ESTIMATE (from
    # wheel speeds, which slip). Results must stay close — this bounds what
    # the estimator itself costs.
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    man = corner_exit()
    lp = simulate(model, make_configs(vp, tp_f, tp_r, cp)[1], man)
    ls = simulate(model, make_configs(vp, tp_f, tp_r, cp)[1], man,
                  sensors=SensorSuite(vp, noise=False), ctrl_every=1)
    yp = metrics(lp, vp)["dw RMSE [rad/s]"]
    ys = metrics(ls, vp)["dw RMSE [rad/s]"]
    check("I4", "clean sensors ≈ perfect state (vx estimation is the only gap)",
          ls["t"][-1] >= man.duration - 0.01 and 0.5 < ys / yp < 1.5,
          f"dw RMSE perfect {yp:.4f} vs clean-sensor {ys:.4f} "
          f"({(ys / yp - 1) * 100:+.0f}%)")

    # I5: determinism — same seed → bit-identical noisy runs.
    la = simulate(model, make_configs(vp, tp_f, tp_r, cp)[1], man,
                  sensors=SensorSuite(vp), ctrl_every=40)
    lb = simulate(model, make_configs(vp, tp_f, tp_r, cp)[1], man,
                  sensors=SensorSuite(vp), ctrl_every=40)
    check("I5", "seeded noise → exactly repeatable runs",
          float(np.abs(la["r"] - lb["r"]).max()) == 0.0 and
          float(np.abs(la["T_RL"] - lb["T_RL"]).max()) == 0.0)
    info("I5", "full noisy stack at 100 Hz (corner exit, s-diff)",
         f"dw RMSE {metrics(la, vp)['dw RMSE [rad/s]']:.4f} vs perfect "
         f"{yp:.4f} — noise + VCU rate + estimation all included")

    # I7: the ground-speed estimate must beat the naive wheel-only pick, and
    # must stay bounded through a traction event. With four driven wheels
    # min(w)*r_w is no longer a lower bound on ground speed, so this is the
    # check that catches a revert to it.
    li = simulate(model, make_configs(vp, tp_f, tp_r, cp)[1], corner_exit(),
                  sensors=SensorSuite(vp), ctrl_every=40)
    err_est = float(np.abs(li["vx_est"] - li["vx"]).max())
    wheel_only = np.minimum(li["wRL"], li["wRR"]) * vp.r_wheel
    err_wss = float(np.abs(wheel_only - li["vx"]).max())
    check("I7", "vx estimate is bounded and beats the wheel-only pick",
          err_est < 1.0 and err_est < err_wss,
          f"worst |vx_est − vx| = {err_est:.3f} m/s vs {err_wss:.3f} m/s for "
          f"min(driven wheels)·r_w over corner_exit")

    # I6: pedal map round trip. DriverAdapter turns the maneuver's requested
    # torque into an APPS percentage and the controller turns it back; the two
    # are exact inverses ONLY while both scale by the same driven-wheel count.
    # If driver.py and the controller ever disagree (2 vs 4 motors), every
    # sensor-vs-perfect-state comparison is silently confounded — this is the
    # check that catches it.
    adapter = DriverAdapter(vp)
    worst = 0.0
    for frac in np.linspace(0.0, 1.0, 21):
        T_in = frac * vp.T_drive_max
        apps = adapter.inputs(0.0, T_in).apps_pct
        T_back = vp.T_drive_max * apps / 100.0     # the controller's pedal map
        worst = max(worst, abs(T_back - T_in))
    check("I6", "pedal map: driver adapter and controller are exact inverses",
          worst < 1e-9,
          f"worst round-trip error {worst:.1e} N·m over 0..{vp.T_drive_max:.0f} "
          f"N·m ({vp.driven_wheels} driven wheels)")


def main():
    print("FSAE-Sim physics verification — independent cross-checks")
    print("=" * 64)
    for name, fn in (("A. tire model", section_a),
                     ("B. vertical loads", section_b),
                     ("C. slip kinematics", section_c),
                     ("D. EOM & integration", section_d),
                     ("E. vs bicycle model", section_e),
                     ("F. controller limits (s-diff)", section_f),
                     ("F2. four-corner allocator limits", section_f2),
                     ("G. in-run audit", section_g),
                     ("H. VCU-rate robustness", section_h),
                     ("I. sensor stack", section_i)):
        print(f"\n── {name} " + "─" * (60 - len(name)))
        fn()
    print("\n" + "=" * 64)
    n_pass = sum(1 for r in RESULTS if r.startswith("[PASS]"))
    print(f"{n_pass} checks passed, {len(FAIL)} failed")
    if FAIL:
        print("\nFAILED:")
        for f in FAIL:
            print("  " + f)
    sys.exit(1 if FAIL else 0)


if __name__ == "__main__":
    main()
