"""Simulation loop (fixed-step RK4) + logging + summary metrics.

Structure of one step, mirroring a real VCU running on a zero-order hold:
    1. controller reads the current (perfectly measured) state -> four torques
    2. torques and steer are held constant while the vehicle physics is
       integrated one dt with classic RK4

dt defaults to 0.25 ms: the stiffest dynamics are the wheel-spin states
(time constant ≈ I_w*vx/(C_kappa*r_w²) ≈ 1–2 ms with placeholder numbers),
and RK4 needs a few steps per time constant to stay accurate.
"""

import math
import numpy as np
from model.physical.vehicle import (VehicleModel, NSTATES, NWHEELS, IX, IY, IPSI,
                                    IVX, IVY, IR, IW, WHEEL_NAMES,
                                    free_rolling_omegas)


def rk4_step(model, s, delta, T_wheel, dt, k1=None):
    """One RK4 step with inputs held constant (zero-order hold).

    T_wheel: four wheel torques [N·m], FL FR RL RR."""
    if k1 is None:
        k1, _ = model.derivatives(s, delta, T_wheel)
    s2 = [s[i] + 0.5 * dt * k1[i] for i in range(NSTATES)]
    k2, _ = model.derivatives(s2, delta, T_wheel)
    s3 = [s[i] + 0.5 * dt * k2[i] for i in range(NSTATES)]
    k3, _ = model.derivatives(s3, delta, T_wheel)
    s4 = [s[i] + dt * k3[i] for i in range(NSTATES)]
    k4, _ = model.derivatives(s4, delta, T_wheel)
    return [s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
            for i in range(NSTATES)]


def simulate(model: VehicleModel, controller, maneuver, dt=2.5e-4, log_every=4,
             sensors=None, ctrl_every=1):
    """Run one maneuver with one controller. Returns a dict of numpy arrays
    (logged every `log_every` steps => 1 kHz logs at the default dt).

    sensors=None  → controller reads the true state every physics step (the
                    original perfect-feedback mode, used by verify.py).
    sensors=SensorSuite → the controller reads ONLY sensor measurements
                    (WSS/IMU/SAS/APPS/BPS), sampled every `ctrl_every`
                    physics steps = the VCU rate, held (ZOH) in between.
    The PLANT inputs are identical in both modes — only what the controller
    KNOWS changes."""
    from model.config import cfg
    p = model.p
    controller.reset()

    # closed-loop "driver replacement" (tracks.py): inputs from (t, state);
    # scripted maneuvers keep their open-loop inputs(t). run_matrix reuses ONE
    # maneuver object across every config, and TrackDriver carries PI state, so
    # reset it here rather than relying on its t < t_last auto-reset.
    driver = getattr(maneuver, "driver", None)
    if driver is not None and hasattr(driver, "reset"):
        driver.reset()

    adapter = None
    if sensors is not None:
        from model.sensors import DriverAdapter
        sensors.reset()
        adapter = DriverAdapter(p)

    n_steps = int(round(maneuver.duration / dt))
    s = [0.0] * NSTATES
    s[IVX] = maneuver.vx0
    # rolling, no initial slip — from each wheel's OWN contact-patch speed
    # resolved along its own heading, so a maneuver that starts steered would
    # still begin at kappa = 0 rather than with a step of spurious drive force.
    delta0 = (driver(0.0, s) if driver is not None else maneuver.inputs(0.0))[0]
    for j, w0 in zip(IW, free_rolling_omegas(model, s, delta0)):
        s[j] = w0

    log = {k: [] for k in
           ("t", "X", "Y", "psi", "vx", "vy", "r",
            "wFL", "wFR", "wRL", "wRR",
            "delta", "T_req", "T_FL", "T_FR", "T_RL", "T_RR", "dw_target",
            "dT_sdiff", "delta_norm", "f_applied", "g_left_appl", "g_right_appl",
            "beta", "ay", "ax",
            "kFL", "kFR", "kRL", "kRR",
            "FzFL", "FzFR", "FzRL", "FzRR", "P_total",
            "vx_est", "r_meas", "apps", "bps", "handwheel", "plaus_cut")}

    dbg = None
    last_info = {"ax": 0.0, "ay": 0.0}
    sr = None
    for k in range(n_steps):
        t = k * dt
        delta, T_req = (driver(t, s) if driver is not None
                        else maneuver.inputs(t))
        if k % ctrl_every == 0 or dbg is None:
            if sensors is None:
                dbg = controller.update(s, delta, T_req, dt * ctrl_every)
            else:
                pedals = maneuver.pedals(t) if maneuver.pedals else None
                drv = adapter.inputs(delta, T_req, pedals)
                braking = drv.bps_bar > cfg.sensors.brake_pressure_sens.actuated_bar
                sr = sensors.measure(s, drv, last_info, dt * ctrl_every,
                                     braking)
                dbg = controller.update_from_sensors(sr, dt * ctrl_every)

        # evaluate derivatives once for logging, reuse as RK4's k1
        k1, info = model.derivatives(s, delta, dbg.T)
        last_info = info

        if k % log_every == 0:
            log["t"].append(t)
            log["X"].append(s[IX]);   log["Y"].append(s[IY])
            log["psi"].append(s[IPSI])
            log["vx"].append(s[IVX]); log["vy"].append(s[IVY])
            log["r"].append(s[IR])
            for nm, j in zip(WHEEL_NAMES, IW):
                log["w" + nm].append(s[j])
            for i, nm in enumerate(WHEEL_NAMES):
                log["T" + "_" + nm].append(dbg.T[i])
            log["delta"].append(delta); log["T_req"].append(T_req)
            log["dw_target"].append(dbg.dw_target)
            log["dT_sdiff"].append(dbg.dT_sdiff)
            log["delta_norm"].append(dbg.delta_norm); log["f_applied"].append(dbg.f_applied)
            log["g_left_appl"].append(dbg.g_left_appl); log["g_right_appl"].append(dbg.g_right_appl)
            log["beta"].append(math.atan2(s[IVY], max(s[IVX], 0.5)))
            log["ay"].append(info["ay"]); log["ax"].append(info["ax"])
            for i, nm in enumerate(WHEEL_NAMES):
                log["k" + nm].append(info["kappa"][i])
                log["Fz" + nm].append(info["Fz"][i])
            log["P_total"].append(sum(dbg.T[i] * s[IW[i]]
                                      for i in range(NWHEELS)))
            if sr is not None:
                log["vx_est"].append(sr.vx_est)
                log["r_meas"].append(sr.yaw_rate)
                log["apps"].append(sr.apps_pct)
                log["bps"].append(sr.bps_bar)
                log["handwheel"].append(sr.handwheel_deg)
                log["plaus_cut"].append(1.0 if controller.plaus_cut else 0.0)
            else:
                for nm in ("vx_est", "r_meas", "apps", "bps", "handwheel",
                           "plaus_cut"):
                    log[nm].append(0.0)

        s = rk4_step(model, s, delta, dbg.T, dt, k1=k1)

        # bail out if the car has physically spun / diverged — logged data
        # up to here is still useful, and the run is flagged in the table
        beta_now = abs(math.atan2(s[IVY], max(s[IVX], 0.5)))
        if abs(s[IR]) > 8.0 or abs(s[IVY]) > 15.0 or beta_now > 1.0:
            break

    return {k: np.asarray(v) for k, v in log.items()}


# ─────────────────────────────────────────────────────────────── metrics
def metrics(log, p):
    """Scalar summary of one run — used for the comparison table."""
    dw_act = log["wRR"] - log["wRL"]
    out = {
        # body sideslip: big values = the rear stepping out
        "max |beta| [deg]": float(np.degrees(np.max(np.abs(log["beta"])))),
        # how well the rear wheel-speed difference matched corner geometry
        "dw RMSE [rad/s]": float(np.sqrt(np.mean((log["dw_target"] - dw_act) ** 2))),
        # worst wheel slip — inner-wheel spin shows up here
        # worst wheel — now genuinely any of four, not just the driven rears
        "max |kappa| [-]": float(np.max(np.abs(np.stack(
            [log["k" + nm] for nm in WHEEL_NAMES])))),
        "max |ay| [g]": float(np.max(np.abs(log["ay"])) / 9.81),
    }
    return out


def run_matrix(model, controllers, maneuver, dt=2.5e-4, sensors=None,
               ctrl_every=1):
    """Run one maneuver across all controller configs. With sensors, every
    config gets the SAME reset sensor suite (identical noise — fair fight)."""
    results = {}
    for ctrl in controllers:
        log = simulate(model, ctrl, maneuver, dt=dt, sensors=sensors,
                       ctrl_every=ctrl_every)
        m = metrics(log, model.p)
        m["finished"] = bool(log["t"][-1] >= maneuver.duration - 0.01)
        results[ctrl.name] = {"log": log, "metrics": m}
    return results


def print_table(maneuver, results):
    cols = ["max |beta| [deg]", "dw RMSE [rad/s]",
            "max |kappa| [-]", "max |ay| [g]"]
    name_w = max(len(n) for n in results) + 2
    print(f"\n=== {maneuver.name}  ({maneuver.description}) ===")
    header = "config".ljust(name_w) + "".join(c.rjust(19) for c in cols) + "  diverged?"
    print(header)
    print("-" * len(header))
    for name, res in results.items():
        m = res["metrics"]
        row = name.ljust(name_w)
        row += "".join(f"{m[c]:19.3f}" for c in cols)
        row += "     no" if m["finished"] else "    YES (spun)"
        print(row)
