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
  F  VCU firmware      the compiled firmware driven directly, cycle by
                       cycle, against the #defines in its own sdiff.c: the
                       open-diff build splits evenly, the s-diff build cuts
                       the inner wheel by K_INNER, deadband, slew rate
  G  in-run audit      invariants recorded DURING the standard maneuvers,
                       with the firmware in the loop: friction-circle
                       utilization, wheel lift, torque/power/speed limits,
                       wheel-speed kinematics
  I  sensor stack      steer map, WSS chain, determinism, vx estimate, and
                       the pedal map round trip through the firmware
  J  full plant        6-DOF chassis: quaternion norm, static equilibrium,
                       aero static deflection, gravity on a grade, the
                       whole-vehicle load-transfer identity
  K  full plant        suspension & dampers: damper curve, link/unsprung
                       transfer identities, two-axle roll closed form,
                       bump stop, ARB monotonicity
  L  full plant        tire transients & steering: relaxation, aligning
                       moment, camber thrust, rack/knuckle settling, scrub
  M  full plant        motors, battery, thermal, brakes: torque ceiling,
                       rotor equation with hand-summed losses, I·V = P,
                       coulomb count, thermal network, derate, hydraulics
  N  full plant        collapses to the planar plant with its extra effects
                       off; firmware in the loop; bit-exact repeatability

Sections D and E drive the plant with EqualTorque, a fixture defined below
that reads the true state — closed-form answers need exact, known torques,
and the firmware can only be reached through the noisy, quantized sensor
stack. It is test scaffolding, not a sim configuration. Sections F, G and I
need the firmware built:  make -C sil

Exit code 0 = every hard check passed. INFO lines are findings/quantified
observations, not pass/fail.
"""

import math
import sys

import numpy as np

from model.params import VehicleParams, TireParams, default_setup, G, RHO_AIR
from model.physical.tires.tire import MagicFormulaTire
from model.physical.vehicle import (VehicleModel, front_steer_angles, NSTATES, NWHEELS,
                                    IW, WHEEL_NAMES, free_rolling_omegas,
                                    wheel_steer_angles,
                                    contact_speeds, IX, IY, IPSI,
                     IVX, IVY, IR, IWRL, IWRR)
from sil.vcu_sil import (make_configs, ControllerDebug, VcuProcess,
                         SIL_EXE, SIL_OPEN_EXE)
from model.maneuvers.maneuvers import Maneuver, step_steer, corner_exit, slalom
from model.sim import simulate, metrics, run_matrix

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


class EqualTorque:
    """Physics fixture, not a sim configuration: the scripted request split
    evenly over the driven wheels, read from the TRUE state. Sections D and E
    use it to drive the plant with exact, known torques; their closed-form
    answers cannot tolerate the sensor noise and quantization that sit between
    the plant and the firmware."""
    name = "equal-torque fixture"
    plaus_cut = False

    def __init__(self, vp):
        self.vp = vp

    def reset(self):
        pass

    def update(self, s, delta, T_req, dt):
        n = self.vp.driven_wheels
        each = T_req / n
        T = (each, each, each, each) if n == 4 else (0.0, 0.0, each, each)
        dbg = ControllerDebug(T=T)
        dbg.dw_target = s[IR] * self.vp.track_r / self.vp.r_wheel
        return dbg


# ═══════════════════════════════════════════════════════ A. tire model
def section_a():
    vp, tp_f, tp_r = default_setup()
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
    vp, tp_f, tp_r = default_setup()
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
    vp, tp_f, tp_r = default_setup()
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
    vp, tp_f, tp_r = default_setup()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))

    # D1: coast-down. δ=0, T=0 → only drag decelerates. Closed form for
    # dv/dt = −k·v²/m_eff with m_eff = m + 2·(I_wf + I_wr)/r_w² — ALL FOUR
    # wheels store rotational KE now that the fronts have a spin state.
    coast = Maneuver("coast", "coast", 3.0, 15.0, lambda t: (0.0, 0.0))
    ctrl = EqualTorque(vp)
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
    ctrl_l = EqualTorque(vp)
    ctrl_r = EqualTorque(vp)
    log_l = simulate(model, ctrl_l, man_l, dt=2.5e-4)
    log_r = simulate(model, ctrl_r, man_r, dt=2.5e-4)
    errs = {
        "r": np.abs(log_l["r"] + log_r["r"]).max(),
        "Y": np.abs(log_l["Y"] + log_r["Y"]).max(),
        "beta": np.abs(log_l["beta"] + log_r["beta"]).max(),
        "T_RL vs T_RR": np.abs(log_l["T_RL"] - log_r["T_RR"]).max(),
    }
    check("D2", "mirrored steer gives exactly mirrored car",
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
        c = EqualTorque(vp)
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
        ctrl = EqualTorque(vp)
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


# ═══════════════════════════════════ F. the VCU firmware against its own source
def _firmware_defines():
    """The #defines the binaries were compiled from — read from the PATCHED
    copy in sil/build/src, so the checks below test the code that actually
    ran, against numbers nobody retyped."""
    import os, re
    src = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "sil", "build", "src")
    want = ("STEERING_RATIO", "DELTA_MAX", "K_DERATE", "K_INNER", "F_MIN",
            "DEADBAND", "RATE", "SDIFF_LOOP_PERIOD", "MAX_MOTOR_CURRENT_MA")
    found = {}
    for fn in ("sdiff.c", "powertrainControl.h"):
        with open(os.path.join(src, fn)) as fh:
            for line in fh:
                m = re.match(r"\s*#define\s+(\w+)\s+([-+0-9.eE]+)f?\b", line)
                if m and m.group(1) in want:
                    found[m.group(1)] = float(m.group(2))
    missing = [k for k in want if k not in found]
    if missing:
        raise RuntimeError(f"firmware no longer defines {missing} — update "
                           "verify.py section F")
    return found


def _drive(exe, handwheel_deg, n, apps=50.0, vx=10.0):
    """Boot a fresh firmware process and feed it n identical VCU cycles.
    Returns [(cmd_RL, cmd_RR), ...] in mA, one per cycle."""
    vp = VehicleParams()
    rpm = vx / vp.r_wheel * vp.gear_ratio * 30.0 / math.pi
    vcu = VcuProcess(exe)
    vcu.start()
    out, t = [], 0.0
    for _ in range(n):
        t += 0.01
        _, _, rl, rr = vcu.exchange(t, apps, 0.0, handwheel_deg, rpm, rpm,
                                    0.0, 0.0, 0.0)
        out.append((rl, rr))
    vcu.stop()
    return out


def section_f():
    import os
    if not (os.path.exists(SIL_EXE) and os.path.exists(SIL_OPEN_EXE)):
        check("F0", "firmware binaries built", False,
              "run  make -C sil  first")
        return
    d = _firmware_defines()
    ratio = d["STEERING_RATIO"]
    lock_hw = 3.0 * d["DELTA_MAX"] * ratio      # road wheel well past DELTA_MAX
    db_hw = 0.5 * d["DEADBAND"] * ratio          # road wheel inside DEADBAND
    f_exp = max(1.0 - d["K_DERATE"], d["F_MIN"])
    g_in = 1.0 - d["K_INNER"]
    base = 0.5 * d["MAX_MOTOR_CURRENT_MA"]      # 50% pedal
    settle = int(round(d["K_INNER"] / (d["RATE"] * d["SDIFF_LOOP_PERIOD"])))
    N = settle + 60
    info("F", "firmware constants (read from sdiff.c / powertrainControl.h)",
         ", ".join(f"{k} {v:g}" for k, v in d.items()))

    # F1: the open-diff build — the s-diff switch off — must send the two
    # rear motors the SAME command at full lock, equal to the pedal map.
    rl, rr = _drive(SIL_OPEN_EXE, +lock_hw, N)[-1]
    check("F1", "open-diff firmware: equal commands at full lock",
          rl == rr and abs(rl / base - 1) < 0.02,
          f"RL {rl} mA, RR {rr} mA at 50% pedal (pedal map {base:.0f} mA)")

    # F2/F3: the s-diff build at full left / right lock. Inner = K_INNER cut,
    # outer = the shared budget only (f, which K_DERATE = 0 pins at 1).
    rl, rr = _drive(SIL_EXE, +lock_hw, N)[-1]
    check("F2", "s-diff firmware, LEFT lock: inner RL = (1 − K_INNER)·outer",
          abs(rl / rr - g_in) < 0.005 and abs(rr / (f_exp * base) - 1) < 0.02,
          f"RL/RR {rl / rr:.4f} vs {g_in:.4f}; RR {rr} mA vs {f_exp * base:.0f}")
    rl2, rr2 = _drive(SIL_EXE, -lock_hw, N)[-1]
    check("F3", "s-diff firmware, RIGHT lock: mirror of F2",
          rl2 == rr and rr2 == rl,
          f"left lock RL/RR {rl}/{rr} mA, right lock {rl2}/{rr2} mA")

    # F4: inside the deadband there is no left/right split.
    rl, rr = _drive(SIL_EXE, +db_hw, N)[-1]
    check("F4", "s-diff firmware inside DEADBAND: equal sides",
          rl == rr,
          f"RL {rl} mA, RR {rr} mA at {db_hw / ratio:.2f}° road wheel")

    # F5: the slew. The inner multiplier may fall by RATE·SDIFF_LOOP_PERIOD per
    # cycle, so the full K_INNER cut takes K_INNER / (RATE·period) cycles.
    cmds = _drive(SIL_EXE, +lock_hw, N)
    final = cmds[-1][0] / cmds[-1][1]
    n_hit = next(k for k, (l, r) in enumerate(cmds, 1)
                 if l / r <= final + 0.002)
    check("F5", "s-diff firmware slew: full cut takes K_INNER/(RATE·period) cycles",
          abs(n_hit - settle) <= 3,
          f"{n_hit} cycles to settle vs {settle} predicted")


# ═══════════════════════ G. in-run invariant audit (standard maneuvers)
class AuditModel(VehicleModel):
    """Same physics; records invariants at every derivative evaluation."""

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        # the front tire's own peak-force slip at its static load — G8's
        # reference, computed from the tire model so it moves when the fit does
        self.k_peak_front = float(self.tires[0].kappa_at_peak(
            self.p.m_total * G * self.p.weight_frac_front / 2.0))
        self.reset_audit()

    def reset_audit(self):
        self.min_Fz = float("inf")
        self.max_util = 0.0
        self.mu_floor_hits = 0
        self.max_w = 0.0
        self.max_kappa = 0.0
        self.load_id_err = 0.0
        self.airborne_driven = 0
        self.min_vcx = float("inf")

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
        for i in range(4):
            if Fz[i] <= 0.0 and abs(T_wheel[i]) > 1e-9:
                self.airborne_driven += 1
        self.min_vcx = min([self.min_vcx]
                           + [abs(v) for v in contact_speeds(self, s, delta)])
        return ds, inf_


def section_g():
    from model.sensors import SensorSuite
    vp, tp_f, tp_r = default_setup()
    audit = AuditModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    Tmax = vp.T_wheel_max

    for man in (step_steer(), corner_exit(), slalom()):
        for ci in (0, 1):                       # both configs, firmware in the loop
            audit.reset_audit()
            ctrl = make_configs(vp)[ci]
            log = simulate(audit, ctrl, man, dt=2.5e-4,
                           sensors=SensorSuite(vp), ctrl_every=40)
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
            # G8: the fronts must not run away. Their peak-force slip is
            # kappa_at_peak at the front static load; twice that is deep on the
            # falling side of the curve, where more wheel speed buys LESS force.
            k_f = max(np.abs(log["kFL"]).max(), np.abs(log["kFR"]).max())
            check("G8", f"{nm}: front slip stays off the falling side",
                  k_f < 2.0 * audit.k_peak_front,
                  f"peak front |κ| {k_f:.4f} vs tire peak "
                  f"{audit.k_peak_front:.4f}")
            # G9: nothing may spin up while its tire is off the ground — a
            # lifted wheel makes no reaction force, so dω/dt = T/I unopposed.
            check("G9", f"{nm}: no wheel driven while airborne",
                  audit.airborne_driven == 0,
                  f"{audit.airborne_driven} evaluations with Fz = 0 and "
                  "torque still commanded")
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
    ctrl = make_configs(vp)[0]                       # OPEN — nobody forcing it
    log = simulate(audit, ctrl, step_steer(), dt=2.5e-4,
                   sensors=SensorSuite(vp), ctrl_every=40)
    i0 = int(0.8 * len(log["t"]))
    dw_act = (log["wRR"] - log["wRL"])[i0:].mean()
    dw_kin = (log["r"][i0:] * vp.track_r / vp.r_wheel).mean()
    check("G7", "open-diff REAR wheels settle to the kinematic Δω by themselves",
          abs(dw_act - dw_kin) / abs(dw_kin) < 0.25,
          f"actual {dw_act:.3f} vs kinematic {dw_kin:.3f} rad/s (gap = real "
          "slip-ratio difference from the torque split, not an error)")
    # the FRONT axle's target carries a cos(delta): both front wheels share
    # x = a, so the sin(delta) terms cancel in the difference. This is the
    # stronger of the two — it exercises the steered projection, which the rear
    # axle never touches.
    dwf_act = (log["wFR"] - log["wFL"])[i0:].mean()
    dwf_kin = (log["r"][i0:] * vp.track_f * np.cos(log["delta"][i0:])
               / vp.r_wheel).mean()
    check("G7", "open-diff FRONT wheels settle to r·track_f·cos δ / r_w",
          abs(dwf_act - dwf_kin) / abs(dwf_kin) < 0.25,
          f"actual {dwf_act:.3f} vs kinematic {dwf_kin:.3f} rad/s "
          f"(cos δ correction {100 * (1 - np.cos(log['delta'][i0:]).mean()):.2f}%)")
    info("G7", "closest approach to the low-speed slip guard",
         f"min |v_cx| over the run = {audit.min_vcx:.2f} m/s against "
         f"v_eps = {vp.v_eps:.2f} — below that the slip-ratio denominator is "
         "floored and the wheel-spin states stiffen sharply")


# ═══════════════════════ I. sensor stack (sensors.py)
def section_i():
    from model.config import cfg
    from model.sensors import (SensorSuite, DriverAdapter, steer_map_deg,
                         steer_map_inv_deg)
    vp, tp_f, tp_r = default_setup()

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
    model_i2 = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    sr = sen.measure(model_i2, s, DriverInputs(), {"ax": 0, "ay": 0}, 0.01, False)
    got = [getattr(sr, "wheel_speed_" + nm) for nm in WHEEL_NAMES]
    want = [40.0, 50.0, 60.0, 70.0]
    check("I2", "WSS: motor rpm → wheel speed chain exact on all four corners",
          max(abs(g - w) for g, w in zip(got, want)) < 1e-12,
          " ".join(f"{nm} {g:.4f}" for nm, g in zip(WHEEL_NAMES, got)) + " rad/s")
    lsb = cfg.sensors.wheel_speed.quant_rpm * (math.pi / 30) / vp.gear_ratio
    info("I2", "WSS quantization at the wheel",
         f"1 motor-rpm LSB = {lsb * vp.r_wheel * 1000:.1f} mm/s of ground speed "
         "— the resolver-over-CAN path is effectively noise-free")

    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    man = corner_exit()

    # I5: determinism — same seed → bit-identical noisy runs.
    la = simulate(model, make_configs(vp)[1], man,
                  sensors=SensorSuite(vp), ctrl_every=40)
    lb = simulate(model, make_configs(vp)[1], man,
                  sensors=SensorSuite(vp), ctrl_every=40)
    check("I5", "seeded noise + firmware → exactly repeatable runs",
          float(np.abs(la["r"] - lb["r"]).max()) == 0.0 and
          float(np.abs(la["T_RL"] - lb["T_RL"]).max()) == 0.0)
    info("I5", "full noisy stack at 100 Hz (corner exit, s-diff firmware)",
         f"dw RMSE {metrics(la, vp)['dw RMSE [rad/s]']:.4f}")

    # I7: the ground-speed estimate must beat the naive wheel-only pick, and
    # must stay bounded through a traction event. With four driven wheels
    # min(w)*r_w is no longer a lower bound on ground speed, so this is the
    # check that catches a revert to it.
    li = simulate(model, make_configs(vp)[1], corner_exit(),
                  sensors=SensorSuite(vp), ctrl_every=40)
    err_est = float(np.abs(li["vx_est"] - li["vx"]).max())
    wheel_only = np.minimum(li["wRL"], li["wRR"]) * vp.r_wheel
    err_wss = float(np.abs(wheel_only - li["vx"]).max())
    check("I7", "vx estimate is bounded and beats the wheel-only pick",
          err_est < 1.0 and err_est < err_wss,
          f"worst |vx_est − vx| = {err_est:.3f} m/s vs {err_wss:.3f} m/s for "
          f"min(driven wheels)·r_w over corner_exit")

    # I6: pedal map round trip, through the real firmware. DriverAdapter turns
    # a scripted torque into an APPS percentage; the firmware turns that into
    # a motor current; Kt × gear × driven wheels turns it back into torque. If
    # the adapter and the firmware ever disagree (a changed current cap, a
    # different driven-wheel count), every maneuver's torque is silently wrong.
    adapter = DriverAdapter(vp)
    k = vp.motor_kt * vp.gear_ratio / 1000.0 * vp.driven_wheels
    worst = 0.0
    for frac in (0.1, 0.3, 0.5, 0.7, 0.9):
        T_in = frac * vp.T_drive_max
        apps = adapter.inputs(0.0, T_in).apps_pct
        rl, rr = _drive(SIL_OPEN_EXE, 0.0, 5, apps=apps)[-1]
        worst = max(worst, abs(0.5 * (rl + rr) * k - T_in) / vp.T_drive_max)
    check("I6", "pedal map: driver adapter and firmware are inverses",
          worst < 0.01,
          f"worst round-trip error {worst * 100:.2f}% of {vp.T_drive_max:.0f} N·m "
          f"({vp.driven_wheels} driven wheels, ADC quantization included)")


# ═══════════════════════════════════════════ J–N. the full (17-DOF) plant
# Every check below is against a closed form, an algebraic identity or a
# module-level unit test — never the plant's own assembled formula. The
# EqualTorqueFull fixture is the same scaffolding as EqualTorque, reading
# the full plant's yaw-rate slot for its diagnostic.
from dataclasses import replace as _replace
from model.params import full_setup, FullParams
from model.physical import vehicle17 as V17
from model.physical.vehicle17 import FullVehicleModel


class EqualTorqueFull(EqualTorque):
    def update(self, s, delta, T_req, dt):
        n = self.vp.driven_wheels
        each = T_req / n
        T = (each, each, each, each) if n == 4 else (0.0, 0.0, each, each)
        dbg = ControllerDebug(T=T)
        dbg.dw_target = s[V17.IR] * self.vp.track_r / self.vp.r_wheel
        return dbg


def _full_model(fp=None, tp_f=None, tp_r=None):
    vp, fp0, tpf0, tpr0 = full_setup()
    fp = fp0 if fp is None else fp
    tp_f = tpf0 if tp_f is None else tp_f
    tp_r = tpr0 if tp_r is None else tp_r
    return vp, fp, FullVehicleModel(vp, fp, MagicFormulaTire(tp_f),
                                    MagicFormulaTire(tp_r))


def section_j():
    """6-DOF chassis: attitude integration, static equilibrium, gravity on a
    grade, and the whole-vehicle lateral load-transfer identity."""
    vp, fp, model = _full_model()
    c = fp.chassis

    # J1: the attitude quaternion stays on the unit sphere through a slalom
    log = simulate(model, EqualTorqueFull(vp), slalom(), dt=2.5e-4, ctrl_every=40)
    # the log has roll/pitch; re-derive |q| by integrating the plant directly
    s = model.initial_state(15.0, 0.0)
    model.reset_run(0.01)
    from model.sim import rk4_step
    u = model.zero_input()
    worst = 0.0
    for k in range(4000):
        delta = math.radians(4.0) * math.sin(2 * math.pi * 0.5 * k * 2.5e-4)
        s = rk4_step(model, s, delta, u, 2.5e-4)
        n = math.sqrt(sum(s[j] ** 2 for j in (V17.IQ0, V17.IQ1, V17.IQ2, V17.IQ3)))
        worst = max(worst, abs(n - 1.0))
    check("J1", "quaternion norm stays 1 through 1 s of sine steer",
          worst < 1e-9, f"max |q|−1 = {worst:.1e}")
    check("J1", "slalom on the full plant runs to completion",
          log["t"][-1] >= 5.99, f"t_end {log['t'][-1]:.2f} s, max |β| "
          f"{math.degrees(np.max(np.abs(log['beta']))):.2f}°")

    # J2: at rest every derivative is zero — the static corner loads, tire
    # deflections and spring preloads were set up by hand, the plant must
    # agree
    s = model.initial_state(0.0, 0.0)
    ds, inf_ = model.derivatives(s, 0.0, model.zero_input())
    worst = max(abs(x) for x in ds)
    check("J2", "car at rest: all 80 derivatives zero (static equilibrium)",
          worst < 1e-6, f"max |ds/dt| = {worst:.1e}")
    check("J2", "at rest ΣF_z = m·g on the four tires",
          abs(sum(inf_["Fz"]) - vp.m_total * G) < 1e-6,
          f"ΣFz {sum(inf_['Fz']):.3f} N vs m·g {vp.m_total * G:.3f} N")
    # the sprung CG height was derived so the two agree — check it
    h_eff = (c.m_sprung * c.h_sprung + sum(m * fp.susp.h_unsprung
                                           for m in c.m_unsprung)) / vp.m_total
    check("J2", "sprung + unsprung CG heights recombine to h_cg",
          abs(h_eff - vp.h_cg) < 1e-12, f"{h_eff:.6f} vs {vp.h_cg:.6f} m")

    # J3: running at 15 m/s with drag and rolling resistance switched off
    # (so nothing decelerates the car and no longitudinal transfer appears),
    # the suspension settles to the aero load's static deflection,
    # s_i = ΔF_i / (k_w + k0) — the dynamics must land on the static answer
    _, tpf0, tpr0 = default_setup()
    fp_j3 = FullParams()
    fp_j3.motor = _replace(fp_j3.motor, T_fric=0.0, b_visc=0.0, eta_gear=1.0)
    _, _, m_j3 = _full_model(fp_j3, _replace(tpf0, f_rr=0.0), _replace(tpr0, f_rr=0.0))
    m_j3.p = _replace(vp, CdA=0.0)
    log = simulate(m_j3, EqualTorqueFull(vp),
                   Maneuver("coast", "coast", 2.0, 15.0, lambda t: (0.0, 0.0)),
                   dt=2.5e-4, ctrl_every=40)
    v_end = log["vx"][-1]
    down = 0.5 * fp.env.rho * vp.ClA * v_end ** 2
    dF = (0.5 * vp.aero_balance_front * down,) * 2 + \
         (0.5 * (1 - vp.aero_balance_front) * down,) * 2
    want = [dF[i] / (m_j3.susp.k_w[i] + fp.damp.k0) for i in range(4)]
    got = [log["s" + nm][-1] for nm in WHEEL_NAMES]
    err = max(abs(g - w) for g, w in zip(got, want))
    check("J3", "constant speed: suspension travel settles to ΔF_aero/(k_w+k0)",
          err < 0.02e-3 and abs(v_end - 15.0) < 5e-3,
          f"max error {err * 1e6:.1f} µm (front {got[0] * 1e3:.2f} mm, rear "
          f"{got[2] * 1e3:.2f} mm); speed held at {v_end:.4f} m/s")

    # J4: gravity on a grade — a car at rest on a 5° slope starts to roll
    # back at exactly g·sin(5°) (no aero, no rolling resistance, no slip:
    # nothing else can touch du/dt at t = 0)
    fp_g = FullParams()
    fp_g.env = _replace(fp_g.env, grade=math.radians(5.0))
    _, _, m_g = _full_model(fp_g)
    s = m_g.initial_state(0.0, 0.0)
    m_g.reset_run(0.01)
    ds, _ = m_g.derivatives(s, 0.0, m_g.zero_input())
    want = -G * math.sin(math.radians(5.0))
    check("J4", "at rest on a 5° uphill grade: du/dt = −g·sin(5°) exactly",
          abs(ds[V17.IU] - want) < 1e-9, f"{ds[V17.IU]:.6f} vs {want:.6f} m/s²")

    # J5: steady-state cornering — the whole-vehicle load-transfer identity.
    # Moments about the ground contact line: the per-axle vertical load
    # differences times half-tracks equal m_t·a_y·h_cg plus the overturning
    # term of the rolled sprung mass, m_s·g·(h_s − h_rc)·φ. Independent of
    # how the plant splits the transfer between axles, links and springs.
    log = simulate(model, EqualTorqueFull(vp), step_steer(), dt=2.5e-4, ctrl_every=40)
    i = -1
    M_t = ((log["FzFR"][i] - log["FzFL"][i]) * vp.track_f / 2.0
           + (log["FzRR"][i] - log["FzRL"][i]) * vp.track_r / 2.0)
    h_rc_eff = (fp.susp.h_rc_f * model.b_s + fp.susp.h_rc_r * model.a_s) / vp.wheelbase
    phi = log["roll"][i]
    # ΣF_y is the body-frame tire force, which already carries gravity's
    # g·sin φ when the body is rolled — the textbook's separate overturning
    # term m_s g (h_s − h_rc) φ is inside it (INFO line quantifies it)
    Fy_sum = log["Fy_f"][i] + log["Fy_r"][i]
    want = abs(Fy_sum) * vp.h_cg
    check("J5", "steady cornering: Σ ΔFz·t/2 = ΣF_y·h_cg (the whole-vehicle moment balance, 0.5%)",
          abs(M_t / want - 1.0) < 0.005,
          f"{M_t:.1f} vs {want:.1f} N·m ({(M_t / want - 1) * 100:+.2f}%), "
          f"a_y {Fy_sum / vp.m_total / G:.3f} g, roll {math.degrees(phi):.2f}°")
    info("J5", "gravity overturning share of that transfer",
         f"{c.m_sprung * G * (c.h_sprung - h_rc_eff) * abs(phi):.1f} N·m "
         f"({c.m_sprung * G * (c.h_sprung - h_rc_eff) * abs(phi) / M_t:.1%}) — carried "
         "inside ΣF_y as g·sin φ in the body frame")
    info("J5", "full plant 5° step steer, steady state",
         f"r {log['r'][i]:.3f} rad/s, a_y {log['ay'][i] / G:.3f} g, "
         f"knuckles {math.degrees(log['dFL'][i]):.2f}°/{math.degrees(log['dFR'][i]):.2f}° "
         f"for a {math.degrees(log['delta'][i]):.1f}° command — the rest is "
         "steering compliance (PLACEHOLDER column/tie-rod stiffness)")


def section_k():
    """Suspension and dampers: the damper curve, the geometric/elastic split
    identity, the roll-angle closed form, the bump stop."""
    vp, fp, model = _full_model()
    c = fp.chassis

    # K1: with alpha = 0 the damper IS the bilinear curve; the curve is
    # continuous at the knee and asymmetric bump/rebound
    d = model.damp
    knee = fp.damp.v_knee
    ok = True
    for v in np.linspace(-0.5, 0.5, 201):
        F = d.force(0, v, 0.0, 0.7)         # z = 0.7 must not matter at alpha = 0
        if v >= 0:
            lo, hi = fp.damp.c_bump_low_f, fp.damp.c_bump_high_f
        else:
            lo, hi = fp.damp.c_reb_low_f, fp.damp.c_reb_high_f
        want = lo * v if abs(v) <= knee else math.copysign(lo * knee + hi * (abs(v) - knee), v)
        ok &= abs(F - want) < 1e-9
    jump = abs(d.curve_force(0, knee + 1e-9) - d.curve_force(0, knee - 1e-9))
    check("K1", "damper = bilinear dyno curve (hysteresis off), continuous at the knee",
          ok and jump < 1e-4, f"knee jump {jump:.1e} N; rebound/bump low-speed ratio "
          f"{fp.damp.c_reb_low_f / fp.damp.c_bump_low_f:.2f}")
    # Bouc-Wen state is bounded at |z| <= 1 for beta + gamma = 1, A = 1
    z = 0.0
    for k in range(20000):
        v = 0.3 * math.sin(2 * math.pi * 3.0 * k * 1e-4)
        z += 1e-4 * d.z_dot(v, z)
    check("K1", "Bouc-Wen state stays bounded (|z| ≤ 1) under a 3 Hz sweep",
          abs(z) <= 1.0 + 1e-9, f"|z| end {abs(z):.3f}")

    # K2: the geometric pair's roll moment equals −Fy·h_rc exactly, so the
    # body is left with Fy·(h_s − h_rc) of elastic roll moment
    Fy = (300.0, 250.0, 0.0, 0.0)
    dFu, dFb = model.susp.geometric_transfer(Fy, (0.0,) * 4, c.h_sprung, 0.0, c.m_unsprung)
    M_pair = sum(model.wheel_xy[i][1] * dFb[i] for i in range(4))
    check("K2", "link reaction on the body: Σ y_i·ΔF_i = −Fy_axle·h_rc exactly",
          abs(M_pair + 550.0 * fp.susp.h_rc_f) < 1e-9,
          f"{M_pair:.4f} vs {-550.0 * fp.susp.h_rc_f:.4f} N·m")
    check("K2", "the link reaction is a pure couple (Σ ΔF_i = 0) and equals the tire-side force",
          abs(sum(dFb)) < 1e-9 and all(abs(a - b) < 1e-12 for a, b in zip(dFu, dFb)))
    # with lateral acceleration the unsprung inertia adds m_u·a_y·h_u/t to
    # the TIRE loads only (the body never sees it)
    # a front-axle tire force that exactly accelerates its own unsprung
    # masses passes nothing through the links: the body sees no reaction,
    # the tires see m_u,axle·a_y·h_u/t
    ay = 5.0
    Fy_u = (c.m_unsprung_f * ay, c.m_unsprung_f * ay,
            c.m_unsprung_r * ay, c.m_unsprung_r * ay)
    dFu2, dFb2 = model.susp.geometric_transfer(Fy_u, (0.0,) * 4, c.h_sprung, ay, c.m_unsprung)
    want = 2 * c.m_unsprung_f * ay * fp.susp.h_unsprung / vp.track_f
    check("K2", "unsprung transfer: m_u,axle·a_y·h_u/t onto the outer tire, nothing onto the body",
          abs(dFu2[1] - want) < 1e-9 and dFu2[0] == -dFu2[1]
          and max(abs(x) for x in dFb2) < 1e-12,
          f"{dFu2[1]:.3f} N per front tire at {ay:.0f} m/s²")

    # K3: steady-state roll angle vs the closed form. The body's roll over
    # the GROUND is the spring deflection plus the tire deflection; the
    # springs carry only the elastic moment, the tires carry the whole
    # transfer (elastic + link + unsprung) — and the rolled sprung mass
    # adds its own overturning moment:
    #     φ = [m_s a_y h_arm / K_s + m_t a_y h_cg / K_t]
    #         / [1 − m_s g h_arm (1/K_s + 1/K_t)]
    #     h_arm = h_s − h_rc(at the sprung CG's station)
    #     K_s = Σ_axles (k_w + k0) t²/2 + ARBs,   K_t = Σ_axles k_t t²/2
    # The body is rigid, so each axle's spring moment M_a is whatever makes
    # both axles roll by the same φ:
    #     φ = M_a/K_s,a + (M_a + G_a)/K_t,a          (springs, then tires)
    #     Σ_a M_a = Σ_a (F_y,a − m_u,a a)(h_s − h_rc,a) + m_s g h_arm φ
    # with G_a the link + unsprung moment of axle a and a = ΣF_y/m. Two
    # equations, solved for φ. The axle forces come from the log; the
    # stiffnesses and heights from the YAML.
    log = simulate(model, EqualTorqueFull(vp), step_steer(), dt=2.5e-4, ctrl_every=40)
    i = -1
    a_t = (log["Fy_f"][i] + log["Fy_r"][i]) / vp.m_total
    h_rc_eff = (fp.susp.h_rc_f * model.b_s + fp.susp.h_rc_r * model.a_s) / vp.wheelbase
    arm = c.h_sprung - h_rc_eff
    M0, sumG, sumC = 0.0, 0.0, 0.0
    for Fy_a, k_w, k_arb, k_t, t, h_rc, m_ua in (
            (log["Fy_f"][i], model.susp.k_w[0], fp.susp.k_arb_f, model.tires[0].p.k_vertical,
             vp.track_f, fp.susp.h_rc_f, 2 * c.m_unsprung_f),
            (log["Fy_r"][i], model.susp.k_w[2], fp.susp.k_arb_r, model.tires[2].p.k_vertical,
             vp.track_r, fp.susp.h_rc_r, 2 * c.m_unsprung_r)):
        K_s = (k_w + fp.damp.k0) * t * t / 2.0 + k_arb
        K_t = k_t * t * t / 2.0
        C = 1.0 / K_s + 1.0 / K_t
        G_a = (Fy_a - m_ua * a_t) * h_rc + m_ua * a_t * fp.susp.h_unsprung
        M0 += (Fy_a - m_ua * a_t) * (c.h_sprung - h_rc)
        sumG += G_a / (K_t * C)
        sumC += 1.0 / C
    phi_want = (M0 + sumG) / (sumC - c.m_sprung * G * arm)
    phi = log["roll"][i]
    check("K3", "steady-state roll angle matches the two-axle roll-stiffness closed form (2%)",
          abs(phi / phi_want - 1.0) < 0.02,
          f"{math.degrees(phi):.3f}° vs {math.degrees(phi_want):.3f}° "
          f"({(phi / phi_want - 1) * 100:+.2f}%) at {a_t / G:.3f} g")
    info("K3", "roll gradient", f"{math.degrees(phi) / (a_t / G):.2f} °/g")

    # K4: the bump stop engages exactly at its gap, at k_bs
    gap = fp.susp.bs_gap_f
    F0 = model.susp.stop_force(0, gap - 1e-6)
    F1 = model.susp.stop_force(0, gap + 0.005)
    check("K4", "bump stop: zero below the gap, k_bs·(s − gap) above it",
          F0 == 0.0 and abs(F1 - fp.susp.k_bs * 0.005) < 1e-9,
          f"5 mm past the gap → {F1:.0f} N")

    # K5: an ARB on one axle moves that axle's share of load transfer up
    def front_share(fp_):
        _, _, m_ = _full_model(fp_)
        lg = simulate(m_, EqualTorqueFull(vp), step_steer(), dt=2.5e-4, ctrl_every=40)
        df = lg["FzFR"][-1] - lg["FzFL"][-1]
        dr = lg["FzRR"][-1] - lg["FzRL"][-1]
        return df / (df + dr)
    fp_arb = FullParams()
    fp_arb.susp = _replace(fp_arb.susp, k_arb_f=300.0 / math.radians(1.0))   # 300 N·m/deg
    base, with_bar = front_share(fp), front_share(fp_arb)
    check("K5", "a 300 N·m/deg FRONT bar raises the front share of lateral transfer",
          with_bar > base + 0.02, f"front share {base:.3f} → {with_bar:.3f}")


def section_l():
    """Tire transients, aligning moment, and the steering DOFs."""
    vp, fp, model = _full_model()

    # L1: relaxation — a step in slip ratio at constant speed reaches 63.2%
    # after rolling one relaxation length
    t = model.tires[0]
    v = 12.0
    sig = t.p.sigma_kappa
    kt = 0.0
    dt = 1e-5
    n = int(round(sig / v / dt))
    for _ in range(n):
        kd, _, _ = t.slip_rates(0.1, 0.0, 0.0, kt, 0.0, 0.0, v)
        kt += dt * kd
    check("L1", "slip-ratio relaxation: 63.2% of a step after rolling σ_κ",
          abs(kt / 0.1 - (1 - math.exp(-1))) < 1e-3,
          f"{kt / 0.1:.4f} vs {1 - math.exp(-1):.4f} after {sig:.2f} m")

    # L2: aligning moment opposes the slip angle and vanishes at the trail's
    # slip angle
    Fz = t.p.Fz_nom
    a = math.radians(2.0)
    Fy = t.mf.lateral(a, Fz)
    Mz = t.aligning_moment(a, Fy, Fz)
    want = -t.p.trail_0 * (1 - a / t.p.trail_alpha) * Fy
    check("L2", "aligning moment = −t_p(α)·F_y, signed to reduce |α|",
          Fy > 0 and Mz < 0 and abs(Mz - want) < 1e-9,
          f"α 2°: Fy {Fy:.0f} N, Mz {Mz:.2f} N·m")
    check("L2", "pneumatic trail is zero beyond trail_slip_angle",
          t.aligning_moment(t.p.trail_alpha * 1.5, 500.0, Fz) == 0.0)

    # L3: camber thrust pushes toward the side the top leans, scales with
    # load, and is switched off on a parked (non-rolling) tire
    Fx0, Fy0 = t.forces(0.0, 0.0, Fz, 0.0)
    Fxp, Fyp = t.forces(0.0, 0.0, Fz, math.radians(1.0))
    Fxs, Fys = t.forces(0.0, 0.0, Fz, math.radians(1.0), rolling=0.0)
    check("L3", "camber thrust: +γ → +Fy, none at γ = 0, none when not rolling",
          Fy0 == 0.0 and Fyp > 0 and Fys == 0.0 and abs(Fyp - t.p.camber_stiffness * math.radians(1.0)) < 1e-9,
          f"1° at Fz_nom → {Fyp:.1f} N")

    # L4: steering DOFs alone (no tire moment): the rack settles exactly at
    # the handwheel's rack target and the knuckles at the kinematic angles
    st = model.steer
    hw = math.radians(45.0)
    x = [0.0] * 6                     # q_r q̇_r δ_FL δ̇_FL δ_FR δ̇_FR

    def f(x):
        (qdd, dFLdd, dFRdd), _ = st.derivatives(x[0], x[1], x[2], x[3], x[4], x[5], hw, (0.0, 0.0))
        return [x[1], qdd, x[3], dFLdd, x[5], dFRdd]
    dt = 2e-5
    for _ in range(int(1.5 / dt)):
        k1 = f(x)
        k2 = f([x[i] + 0.5 * dt * k1[i] for i in range(6)])
        k3 = f([x[i] + 0.5 * dt * k2[i] for i in range(6)])
        k4 = f([x[i] + dt * k3[i] for i in range(6)])
        x = [x[i] + dt / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(6)]
    q_want = st.rack_target(hw)
    kin = st.kinematic_angles(q_want)
    check("L4", "steering DOFs settle: rack at c_factor·θ/2π, knuckles at the kinematic angles",
          abs(x[0] - q_want) < 1e-7 and abs(x[2] - kin[0]) < 1e-6 and abs(x[4] - kin[1]) < 1e-6,
          f"rack {x[0] * 1e3:.3f} mm (want {q_want * 1e3:.3f}), FL {math.degrees(x[2]):.3f}° "
          f"FR {math.degrees(x[4]):.3f}° (want {math.degrees(kin[0]):.3f}/{math.degrees(kin[1]):.3f})")
    # L5: a left/right drive-torque split steers the wheels through the scrub
    # radius: the left wheel right, the right wheel left (torque steer)
    M_L = st.kingpin_moment(0.0, 500.0, 0.0, +1.0)
    M_R = st.kingpin_moment(0.0, 500.0, 0.0, -1.0)
    check("L5", "scrub radius: a drive force steers the left wheel right and the right wheel left",
          M_L < 0 < M_R and abs(M_L + M_R) < 1e-12,
          f"±{abs(M_L):.1f} N·m per 500 N")


def section_m():
    """Motors, shafts, battery, thermal, brakes."""
    vp, fp, model = _full_model()
    mp, bp = fp.motor, fp.batt
    md = model.motor

    # M1: torque ceiling — peak torque below base speed, P/ω above, scaled by bus voltage
    w_base = mp.P_peak / mp.T_peak
    check("M1", "torque ceiling: T_peak below base speed, P_peak/ω above",
          md.torque_ceiling(0.5 * w_base, mp.V_nom) == mp.T_peak
          and abs(md.torque_ceiling(2 * w_base, mp.V_nom) - 0.5 * mp.T_peak) < 1e-12
          and abs(md.torque_ceiling(2 * w_base, 0.5 * mp.V_nom) - 0.25 * mp.T_peak) < 1e-12,
          f"base speed {w_base * 30 / math.pi:.0f} rpm; half the bus voltage halves the ceiling above it")
    check("M1", "no regen below the rules cut-off speed; regen capped at T_regen_max above it",
          md.electromagnetic_torque(-200.0, 500.0, mp.V_nom, False) == 0.0
          and md.electromagnetic_torque(-200.0, 500.0, mp.V_nom, True) == -mp.T_regen_max)

    # M2: the rotor equation with the losses summed BY HAND from the YAML:
    # drive the plant 2 s at a constant current, then at that state
    #     T_s = G·(T_e − T_fric − b·ω_m − (1−η_g)|T_s|/G − J_m·ω̇_m)
    # must hold to round-off (T_s from the shaft twist, ω̇_m from the
    # plant's derivative, T_e from the inverter model)
    from model.sim import rk4_step
    s = model.initial_state(12.0, 0.0)
    model.reset_run(0.01)
    u = (0.0, (6.0, 6.0, 6.0, 6.0), 0.0)        # 6 A on each motor
    for _ in range(8000):
        s = rk4_step(model, s, 0.0, u, 2.5e-4)
    ds, inf_ = model.derivatives(s, 0.0, u)
    worst = 0.0
    for i in range(4):
        Te, Ts, wm = inf_["T_e"][i], inf_["T_s"][i], s[V17.IOMM[i]]
        loss = mp.T_fric + mp.b_visc * wm + (1 - mp.eta_gear) * abs(Ts) / mp.G
        worst = max(worst, abs(Ts - mp.G * (Te - loss - mp.J_m * ds[V17.IOMM[i]])))
    check("M2", "rotor equation: T_s = G·(T_e − friction − windage − gear loss − J_m·ω̇_m), losses summed by hand",
          worst < 1e-9, f"worst mismatch {worst:.1e} N·m; T_e {inf_['T_e'][2]:.3f} → "
          f"T_s {inf_['T_s'][2]:.2f} N·m at the wheel (= {inf_['T_s'][2] / (mp.G * inf_['T_e'][2]):.1%} of G·T_e)")
    check("M2", "Kt: T_e = Kt·I below the ceiling",
          abs(inf_["T_e"][2] - mp.kt * 6.0) < 1e-12)
    log = simulate(model, EqualTorqueFull(vp), step_steer(), dt=2.5e-4, ctrl_every=40)

    # M3: pack current solves the power identity I·V_bus = P_elec at every sample
    I, V, P = log["I_pack"], log["V_bus"], log["P_elec"]
    mask = np.abs(P) > 100.0
    rel = np.max(np.abs(I[mask] * V[mask] - P[mask]) / np.abs(P[mask]))
    check("M3", "pack current: I·V_bus = Σ motor electrical power at every sample",
          rel < 1e-9, f"worst {rel:.1e}; I {np.max(I):.1f} A, V_bus {np.min(V):.1f}–{np.max(V):.1f} V")

    # M4: coulomb counting — ΔSOC = −η_c ∫I dt / (n_p Q_cap)
    dsoc = log["SOC"][-1] - log["SOC"][0]
    want = -bp.eta_c * np.trapezoid(I, log["t"]) / (bp.n_p * bp.Q_cap)
    check("M4", "SOC change = −η_c·∫I dt / (n_p·Q_cap)",
          abs(dsoc / want - 1.0) < 1e-3, f"ΔSOC {dsoc:.5e} vs {want:.5e} "
          f"({(dsoc / want - 1) * 100:+.3f}%, trapezoid on the 1 kHz log)")

    # M5: thermal network steady state — integrate the battery alone at a
    # constant current until settled, then the algebraic identities
    #   T_core − T_surf = Q R_cs,  T_surf − T_cool = Q R_sc,j,
    #   T_cool − T_amb = ΣQ / UA
    b = model.batt
    I_c = 150.0
    socs, v1s, v2s = (0.9,) * 3, (0.0,) * 3, (0.0,) * 3
    Tc, Ts, Tcool = [bp.T_0] * 3, [bp.T_0] * 3, bp.T_0
    dt = 0.5
    for _ in range(int(6000 / dt)):
        _, _, _, dtc, dts, dtcool = b.derivatives(I_c, socs, v1s, v2s, Tc, Ts, Tcool, fp.env.T_amb)
        Tc = [Tc[j] + dt * dtc[j] for j in range(3)]
        Ts = [Ts[j] + dt * dts[j] for j in range(3)]
        Tcool += dt * dtcool
    Q = [b.n_cells_zone * (I_c / bp.n_p) ** 2 * b.r0(Tc[j]) for j in range(3)]
    e1 = max(abs((Tc[j] - Ts[j]) - Q[j] * bp.R_cs) for j in range(3))
    e2 = max(abs((Ts[j] - Tcool) - Q[j] * b.R_sc[j]) for j in range(3))
    e3 = abs((Tcool - fp.env.T_amb) - sum(Q) / bp.UA)
    check("M5", "thermal steady state: ΔT = Q·R across each stage, ΣQ/UA to ambient",
          max(e1, e2, e3) < 0.05,
          f"core {max(Tc):.1f} °C (middle zone hottest: {Tc[1] > Tc[0]}), coolant {Tcool:.1f} °C "
          f"at {I_c:.0f} A; worst identity error {max(e1, e2, e3):.3f} K")
    check("M5", "the middle zone runs hottest (hot-spot factor)", Tc[1] > Tc[0] and Tc[1] > Tc[2])
    # derate: the current window shrinks linearly between derate_start and derate_end
    I_dis_cold, _ = b.current_window(socs, v1s, v2s, (25.0,) * 3)
    I_dis_mid, _ = b.current_window(socs, v1s, v2s, (0.5 * (bp.T_derate_0 + bp.T_derate_1),) * 3)
    I_dis_hot, _ = b.current_window(socs, v1s, v2s, (bp.T_derate_1 + 5.0,) * 3)
    check("M5", "BMS derate: full current cold, half way at the midpoint, zero past derate_end",
          abs(I_dis_mid / I_dis_cold - 0.5) < 1e-9 and I_dis_hot == 0.0,
          f"{I_dis_cold:.0f} A → {I_dis_mid:.0f} A → {I_dis_hot:.0f} A")

    # M6: brakes — first-order pressure step and the hand-computed torque gain
    br = model.brakes
    P = 0.0
    dt = 1e-5
    for _ in range(int(round(fp.brake.tau_f / dt))):
        P += dt * br.pressure_rates(P, 0.0, 40.0)[0]
    check("M6", "front line pressure reaches 63.2% of the pedal after τ_f",
          abs(P / 40.0 - (1 - math.exp(-1))) < 1e-3, f"{P:.2f} bar of 40 after {fp.brake.tau_f * 1e3:.0f} ms")
    gain = 1e5 * fp.brake.A_p_f * fp.brake.mu_pad * fp.brake.R_eff_f * fp.brake.N_surf
    T = br.torques(40.0, 0.0, (50.0, 50.0, 50.0, 50.0))
    check("M6", "brake torque = P·A_p·μ·R_eff·N, opposing rotation",
          abs(T[0] + 40.0 * gain) < 1e-6 and T[2] == 0.0,
          f"40 bar front → {-T[0]:.0f} N·m per wheel ({gain:.2f} N·m/bar)")


def section_n():
    """Cross-plant and firmware-in-the-loop on the full plant."""
    vp, fp, model = _full_model()
    from model.sensors import SensorSuite

    # N1: with every effect the planar plant lacks switched off (steering
    # compliance, relaxation, trail, camber, rolling resistance, motor
    # losses, roll centres, anti-geometry, gas spring, CoP height), the full
    # plant's steady-state cornering must reproduce the planar plant's
    fp_r = FullParams()
    # steering compliance is removed by removing what LOADS it — mechanical
    # trail, scrub radius (and pneumatic trail, below) — not by stiffening:
    # the tie-rod stiffness reflects through the steering ratio squared
    # onto the rack, and a rigid value puts that mode past RK4's stability
    fp_r.steer = _replace(fp_r.steer, t_mech=0.0, r_scrub=0.0, F_fric=0.0)
    fp_r.damp = _replace(fp_r.damp, k0=0.0)
    fp_r.motor = _replace(fp_r.motor, T_fric=0.0, b_visc=0.0, eta_gear=1.0)
    fp_r.susp = _replace(fp_r.susp, h_rc_f=0.0, h_rc_r=0.0, anti_dive=0.0,
                         anti_squat=0.0, camber_bump_f=0.0, camber_bump_r=0.0,
                         camber_roll_f=0.0, camber_roll_r=0.0,
                         camber0_f=0.0, camber0_r=0.0)
    fp_r.chassis = _replace(fp_r.chassis, cop_height=fp.chassis.h_sprung)
    _, tp_f, tp_r = default_setup()
    rigid = dict(sigma_kappa=0.01, sigma_alpha=0.01, sigma_mz=0.01, trail_0=0.0,
                 camber_stiffness=0.0, f_rr=0.0)
    tp_f_r, tp_r_r = _replace(tp_f, **rigid), _replace(tp_r, **rigid)
    _, _, m_rigid = _full_model(fp_r, tp_f_r, tp_r_r)
    m_plan = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    # below the grip limit (~0.5 g): at the limit a 1% difference in how
    # the two plants split load between the wheels moves the yaw rate 3–4%,
    # which says nothing about whether the physics collapses correctly
    man = step_steer(delta_deg=3.0, vx0=12.0, T_hold=50.0)
    lf = simulate(m_rigid, EqualTorqueFull(vp), man, dt=2.5e-4, ctrl_every=40)
    lp = simulate(m_plan, EqualTorque(vp), man, dt=2.5e-4, ctrl_every=40)
    r_f, r_p = lf["r"][-1], lp["r"][-1]
    ay_f, ay_p = lf["ay"][-1], lp["ay"][-1]
    v_f, v_p = lf["vx"][-1], lp["vx"][-1]
    check("N1", "rigid-ised full plant reproduces the planar plant's steady yaw rate (2%, 3° at 12 m/s)",
          abs(r_f / r_p - 1.0) < 0.02, f"r {r_f:.4f} vs {r_p:.4f} rad/s ({(r_f / r_p - 1) * 100:+.2f}%)")
    check("N1", "… and its lateral acceleration and speed",
          abs(ay_f / ay_p - 1.0) < 0.02 and abs(v_f / v_p - 1.0) < 0.01,
          f"a_y {ay_f / G:.3f} vs {ay_p / G:.3f} g; v {v_f:.2f} vs {v_p:.2f} m/s")
    man5 = step_steer()
    lf5 = simulate(m_rigid, EqualTorqueFull(vp), man5, dt=2.5e-4, ctrl_every=40)
    lp5 = simulate(m_plan, EqualTorque(vp), man5, dt=2.5e-4, ctrl_every=40)
    full = simulate(model, EqualTorqueFull(vp), man5, dt=2.5e-4, ctrl_every=40)
    info("N1", "at the grip limit (5° at 15 m/s): rigid-ised full vs planar",
         f"r {lf5['r'][-1]:.4f} vs {lp5['r'][-1]:.4f} rad/s ({(lf5['r'][-1] / lp5['r'][-1] - 1) * 100:+.1f}%), "
         f"a_y {lf5['ay'][-1] / G:.3f} vs {lp5['ay'][-1] / G:.3f} g — load-split differences at the limit")
    info("N1", "full plant with every placeholder effect ON vs planar, same 5° step",
         f"r {full['r'][-1]:.4f} vs {lp5['r'][-1]:.4f} rad/s ({(full['r'][-1] / lp5['r'][-1] - 1) * 100:+.1f}%): "
         "the gap is steering compliance + transients + camber — all PLACEHOLDER-driven")

    # N2: the firmware in the loop on the full plant — both configs complete
    # the step steer and the corner exit, torques stay inside the motor's
    # envelope, and the s-diff's cut shows up in the rear electromagnetic
    # torques when the wheel is past DELTA_MAX
    sensors = SensorSuite(vp)
    for man in (step_steer(), corner_exit()):
        model.reset_run(0.01)
        res = run_matrix(model, make_configs(vp), man, dt=2.5e-4,
                         sensors=sensors, ctrl_every=40)
        for nm, r in res.items():
            lg = r["log"]
            Te_max = max(np.max(np.abs(lg["Te_" + w])) for w in WHEEL_NAMES)
            check("N2", f"{man.slug} / {nm}: completes on the full plant, |T_e| ≤ T_peak",
                  r["metrics"]["finished"] and Te_max <= fp.motor.T_peak + 1e-9,
                  f"max |T_e| {Te_max:.2f} N·m, max |β| {r['metrics']['max |beta| [deg]']:.2f}°, "
                  f"max |κ| {r['metrics']['max |kappa| [-]']:.3f}")
        if man.slug == "step_steer":
            sd = res["s-diff"]["log"]
            i = -1
            check("N2", "s-diff at 5° left: inner-rear T_e below outer-rear (the firmware's cut reaches the motors)",
                  sd["Te_RL"][i] < sd["Te_RR"][i] - 1e-6,
                  f"T_e RL {sd['Te_RL'][i]:.3f} vs RR {sd['Te_RR'][i]:.3f} N·m")
            op = res["open diff"]["log"]
            check("N2", "open diff at 5° left: equal rear T_e",
                  abs(op["Te_RL"][i] - op["Te_RR"][i]) < 1e-6)

    # N3: seeded noise + firmware + bias walks → bit-exact repeatability
    man = step_steer()
    a = simulate(model, make_configs(vp)[1], man, dt=2.5e-4, sensors=SensorSuite(vp), ctrl_every=40)
    b = simulate(model, make_configs(vp)[1], man, dt=2.5e-4, sensors=SensorSuite(vp), ctrl_every=40)
    same = all(np.array_equal(a[k], b[k]) for k in ("r", "Te_RL", "b_ax", "b_gps1", "vx_est"))
    check("N3", "full plant + firmware + GPS/accel bias walks: two runs bit-identical", same,
          f"bias walked to b_ax {a['b_ax'][-1]:+.2e} m/s², b_gps1 {a['b_gps1'][-1]:+.3e} m/s")
    moved = abs(a["b_ax"][-1]) > 0.0 or abs(a["b_gps1"][-1]) > 0.0
    check("N3", "the bias states actually walk (not stuck at zero)", moved)


def main():
    print("FSAE-Sim physics verification — independent cross-checks")
    print("=" * 64)
    from sil.vcu_sil import ensure_firmware, FirmwareError
    try:
        ensure_firmware()
        fw_ok = True
    except FirmwareError as e:
        check("F0", "VCU firmware found and compiled", False, str(e))
        fw_ok = False
    needs_fw = {"F", "G", "I", "N"}
    for name, fn in (("A. tire model", section_a),
                     ("B. vertical loads", section_b),
                     ("C. slip kinematics", section_c),
                     ("D. EOM & integration", section_d),
                     ("E. vs bicycle model", section_e),
                     ("F. VCU firmware vs its own source", section_f),
                     ("G. in-run audit (firmware in the loop)", section_g),
                     ("I. sensor stack", section_i),
                     ("J. full plant: 6-DOF chassis", section_j),
                     ("K. full plant: suspension & dampers", section_k),
                     ("L. full plant: tire transients & steering", section_l),
                     ("M. full plant: motors, battery, thermal, brakes", section_m),
                     ("N. full plant: vs planar, firmware in the loop", section_n)):
        print(f"\n── {name} " + "─" * (60 - len(name)))
        if name[0] in needs_fw and not fw_ok:
            print("   skipped — the firmware did not build (see F0)")
            continue
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
