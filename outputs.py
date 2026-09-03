"""Outputs — what "optimal" means, measured (plan Step 4), the ranking of
the torque-allocation modes, and the IDEAL input/output sheet per corner.

Three things live here:

1. STEP-4 METRICS from a run log (added to every metrics table / CSV):
     exit ax [g]           mean longitudinal accel over the EXIT WINDOW —
                           the 1.5 s after the throttle step (higher = the
                           mode put more power down)
     peak inner kappa [-]  peak slip ratio of the INNER rear from the
                           throttle step on (lower = less wasted spin;
                           anything past the tire peak ≈ 0.10 is waste)
     yaw RMSE [rad/s]      tracking error vs r_ref (already in sim.metrics)
     inner regen [kJ]      energy taken OUT of the inner wheel (negative
                           wheel power integrated) — large means the tune
                           is dragging the inner wheel instead of capping it
   The exit window is found from the data (first time the torque request
   reaches 90 % of its rise), the inner side from the steer sign — so it
   works for any maneuver, not just the track tests.

2. RANKING: modes ranked 1..n on each of the four numbers, summed; a run
   that spun is ranked last whatever its numbers.

3. IDEAL INPUT/OUTPUT SHEET for a corner (R, angle) from car_data.py only:
   entry speed, ideal steer (kinematic and with the understeer gradient),
   handwheel angle, yaw rate, per-wheel ground and wheel speeds (motor
   rpm), rear loads with lateral transfer, the tire's peak slip ratio at
   each load, the torque each rear wheel can take before it spins, the
   resulting throttle ceilings (open 50/50 vs a perfect allocator), the
   throttle that holds speed, and the brake pressure / decel picture for
   the entry. Written to REPORT.md + ideal.csv in the run folder.

Usage from a finished run folder (recomputes from the saved CSVs):
    python outputs.py runs/latest
"""

import csv
import json
import math
import os
import sys

import numpy as np

import car_data as cd
from params import VehicleParams, TireParams, G, RHO_AIR, default_setup
from tire import MagicFormulaTire

# the three numbers this module adds to sim.metrics (yaw RMSE is already there)
STEP4_EXTRA = ["exit ax [g]", "peak inner kappa [-]", "inner regen [kJ]"]
STEP4_COLS = ["exit ax [g]", "peak inner kappa [-]", "yaw RMSE [rad/s]",
              "inner regen [kJ]"]
# +1 = higher is better, −1 = lower is better
BETTER = {"exit ax [g]": +1, "peak inner kappa [-]": -1,
          "yaw RMSE [rad/s]": -1, "inner regen [kJ]": -1}
# differences smaller than this are a TIE (same rank) — the sim's own
# repeatability is far finer, but nothing this small is a design result
TIE_TOL = {"exit ax [g]": 0.01, "peak inner kappa [-]": 0.005,
           "yaw RMSE [rad/s]": 0.002, "inner regen [kJ]": 0.01}

EXIT_WINDOW_S = 1.5         # length of the exit window after the throttle step
V_TOP_MPS = 40.0 * cd.MPH   # endurance top speed (team, maneuvers.py) — the
                            # "from" speed of the entry-braking picture


# ───────────────────────────────────────────────────────── step-4 metrics
def exit_window(log, hold_s=EXIT_WINDOW_S):
    """(i0, i1) index window of the corner exit. Starts at the throttle
    event — the first sample where T_req has covered 90 % of its rise; if
    the torque never rises (constant-torque tests) at the point the steer
    reaches 90 % of its peak. Ends hold_s later or at the end of the log."""
    t, T = log["t"], log["T_req"]
    n = len(t)
    if n < 2:
        return 0, n
    rise = T.max() - T[0]
    if rise > 0.1 * max(abs(T.max()), 1.0):
        i0 = int(np.argmax(T >= T[0] + 0.9 * rise))
    else:
        d = np.abs(log["delta"])
        i0 = int(np.argmax(d >= 0.9 * d.max())) if d.max() > 0 else 0
    i1 = int(np.searchsorted(t, t[i0] + hold_s, side="right"))
    return i0, min(max(i1, i0 + 1), n)


def inner_side(log, i0, i1):
    """'RL' in a left turn (δ > 0), 'RR' in a right turn; on a straight,
    whichever rear slipped more."""
    d = float(np.mean(log["delta"][i0:i1])) if i1 > i0 else 0.0
    if abs(d) < 1e-4:
        return "RL" if np.max(np.abs(log["kRL"])) >= np.max(np.abs(log["kRR"])) else "RR"
    return "RL" if d > 0 else "RR"


def step4_metrics(log, hold_s=EXIT_WINDOW_S):
    """The plan's Step-4 numbers from one run log (dict of arrays)."""
    t = log["t"]
    if len(t) < 2:
        return {c: float("nan") for c in STEP4_EXTRA} | {"inner side": "RL"}
    i0, i1 = exit_window(log, hold_s)
    side = inner_side(log, i0, i1)
    P_in = log["T_" + side] * log["w" + side]
    E_regen = max(-float(np.sum(np.minimum(P_in[1:], 0.0) * np.diff(t))), 0.0)
    return {
        "exit ax [g]": float(np.mean(log["ax"][i0:i1])) / G,
        "peak inner kappa [-]": float(np.max(np.abs(log["k" + side][i0:]))),
        "inner regen [kJ]": E_regen / 1e3,
        "inner side": side,
        "exit window [s]": (float(t[i0]), float(t[min(i1, len(t)) - 1])),
    }


def rank_configs(results):
    """results: {config: {"metrics": {...}}} → [(config, score, {col: rank})]
    best first. Score = sum of per-column ranks (1 = best, ties within
    TIE_TOL share the better rank); a config that did not finish (spun) is
    pushed to the bottom."""
    names = list(results)
    ranks = {n: {} for n in names}
    for col in STEP4_COLS:
        vals = {n: results[n]["metrics"].get(col, float("nan")) for n in names}
        key = lambda n: (-BETTER[col] * vals[n] if np.isfinite(vals[n]) else np.inf)
        order = sorted(names, key=key)
        rank, prev = 0, None
        for i, n in enumerate(order):
            if prev is None or abs(vals[n] - vals[prev]) > TIE_TOL[col]:
                rank, prev = i + 1, n
            ranks[n][col] = rank
    score = {n: sum(ranks[n].values())
             + (0 if results[n]["metrics"].get("finished", True) else 100)
             for n in names}
    return sorted(((n, score[n], ranks[n]) for n in names), key=lambda x: x[1])


# ────────────────────────────────────────────────────── ideal corner sheet
def understeer_gradient(vp: VehicleParams, tp_f: TireParams, tp_r: TireParams):
    """K_us [s²/m] from the linearized tire stiffnesses at static axle
    loads — the same expression the TV reference uses."""
    m, L = vp.m_total, vp.wheelbase
    C_f = tp_f.c_alpha * m * G * vp.b / L
    C_r = tp_r.c_alpha * m * G * vp.a / L
    return m / L * (vp.b / C_f - vp.a / C_r)


def ideal_corner(vp: VehicleParams, tp_f: TireParams, tp_r: TireParams,
                 radius: float, corner_deg: float = None,
                 entry_frac: float = 0.9, mu: float = None) -> dict:
    """Ideal inputs and outputs for a steady corner of radius R, entered at
    entry_frac·√(µgR). Everything comes from car_data.py through the
    parameter containers; the tire is asked (not assumed) for its peaks."""
    from sensors import steer_map_inv_deg
    tire = MagicFormulaTire(tp_r)
    m, L, h = vp.m_total, vp.wheelbase, vp.h_cg
    mu_y = tp_r.mu0 if mu is None else mu

    v = entry_frac * math.sqrt(mu_y * G * radius)
    r = v / radius
    ay = v * v / radius
    K_us = understeer_gradient(vp, tp_f, tp_r)
    d_kin = math.atan(L / radius)
    d_ss = d_kin + K_us * ay
    downforce = 0.5 * RHO_AIR * vp.ClA * v * v
    drag = 0.5 * RHO_AIR * vp.CdA * v * v

    # rear loads: static + rear aero share + lateral transfer (rear share)
    Fz_r_axle = m * G * vp.a / L + (1.0 - vp.aero_balance_front) * downforce
    dF_r = (1.0 - vp.lat_transfer_frac_front) * m * ay * h / vp.track_r
    Fz_in = max(Fz_r_axle / 2.0 - dF_r, 0.0)
    Fz_out = Fz_r_axle / 2.0 + dF_r

    # lateral demand on the rear axle (moment balance), shared by load;
    # what is left for drive on the friction ellipse
    Fy_r = m * ay * vp.a / L
    def fx_avail(Fz):
        if Fz <= 0:
            return 0.0
        fy = Fy_r * Fz / Fz_r_axle
        util = min(fy / (tire.mu(Fz) * Fz), 1.0)
        return tire.mu_x(Fz) * Fz * math.sqrt(max(1.0 - util * util, 0.0))
    Fx_in, Fx_out = fx_avail(Fz_in), fx_avail(Fz_out)
    T_in_max, T_out_max = Fx_in * vp.r_wheel, Fx_out * vp.r_wheel
    T_axle_peak = 2.0 * vp.T_wheel_max

    # torque that just holds speed: aero drag + tire induced drag (linear α)
    Fy_f = m * ay * vp.b / L
    Fz_f_axle = m * G * vp.b / L + vp.aero_balance_front * downforce
    alpha_f = Fy_f / (tp_f.c_alpha * Fz_f_axle) if Fz_f_axle > 0 else 0.0
    alpha_r = Fy_r / (tp_r.c_alpha * Fz_r_axle) if Fz_r_axle > 0 else 0.0
    F_induced = Fy_f * math.sin(alpha_f) + Fy_r * math.sin(alpha_r)
    T_hold = vp.r_wheel * (drag + F_induced)

    # wheel speeds with no slip: each rear at v ∓ r·t/2
    v_in, v_out = v - r * vp.track_r / 2.0, v + r * vp.track_r / 2.0
    w_in, w_out = v_in / vp.r_wheel, v_out / vp.r_wheel
    rpm = lambda w: w * vp.gear_ratio / cd.RPM

    # entry braking picture: limit decel from the top speed down to v.
    # BPS commands regen only in this sim; the rest is mechanical brakes.
    mu_x = tire.mu_x(tp_r.Fz_nom)
    a_brake = mu_x * (G + downforce / m)
    T_brake_total = m * a_brake * vp.r_wheel
    T_regen = min(T_brake_total, cd.T_REGEN_MAX)
    bps_bar = cd.BPS_RANGE_BAR * T_regen / cd.T_REGEN_MAX
    dist = max(V_TOP_MPS ** 2 - v * v, 0.0) / (2.0 * a_brake)

    return {
        "corner [deg]": corner_deg if corner_deg is not None else float("nan"),
        "radius [m]": radius,
        "entry speed [m/s]": v,
        "lateral accel [g]": ay / G,
        "yaw rate [rad/s]": r,
        "steer kinematic [deg]": math.degrees(d_kin),
        "steer w/ understeer [deg]": math.degrees(d_ss),
        "handwheel [deg]": steer_map_inv_deg(math.degrees(d_ss)),
        "inner ground speed [m/s]": v_in,
        "outer ground speed [m/s]": v_out,
        "inner wheel [rad/s]": w_in,
        "outer wheel [rad/s]": w_out,
        "inner motor [rpm]": rpm(w_in),
        "outer motor [rpm]": rpm(w_out),
        "ideal dw RR-RL [rad/s]": (w_out - w_in),
        "Fz inner rear [N]": Fz_in,
        "Fz outer rear [N]": Fz_out,
        "tire peak kappa inner [-]": tire.kappa_at_peak(Fz_in) if Fz_in > 0 else 0.0,
        "tire peak kappa outer [-]": tire.kappa_at_peak(Fz_out),
        "T inner before spin [Nm]": T_in_max,
        "T outer before spin [Nm]": T_out_max,
        "throttle ceiling open 50/50 [%]": min(2.0 * T_in_max / T_axle_peak, 1.0) * 100.0,
        "throttle ceiling ideal split [%]": min((T_in_max + T_out_max) / T_axle_peak, 1.0) * 100.0,
        "hold-speed torque [Nm]": T_hold,
        "hold-speed throttle [%]": T_hold / T_axle_peak * 100.0,
        "entry brake decel limit [g]": a_brake / G,
        "entry brake torque total [Nm]": T_brake_total,
        "entry regen share [Nm]": T_regen,
        "entry mechanical brake remainder [Nm]": T_brake_total - T_regen,
        "entry brake pressure (regen map) [bar]": bps_bar,
        "entry braking distance from top speed [m]": dist,
    }
