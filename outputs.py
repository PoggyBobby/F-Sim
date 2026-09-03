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
