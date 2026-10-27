"""Simulation loop (fixed-step RK4) + logging + summary metrics.

Structure of one step, mirroring a real VCU running on a zero-order hold:
    1. every `ctrl_every` steps the controller — the VCU firmware, via
       sil/vcu_sil.py — reads the sensor frame and returns its command; the
       plant turns that into its own input (plant_input: four wheel torques
       for the planar plant, four motor currents through the inverter
       command path for the full plant)
    2. that input and the steer command are held constant while the
       vehicle physics is integrated one dt with classic RK4

dt defaults to 0.25 ms: the stiffest dynamics are the wheel-spin states
(time constant ≈ I_w*vx/(C_kappa*r_w²) ≈ 1–2 ms with placeholder numbers),
and RK4 needs a few steps per time constant to stay accurate. The full
plant's shaft compliance (~100 Hz) and wheel hop (~16 Hz) sit inside that.

THE LOOP IS PLANT-AGNOSTIC. It never indexes a state vector; it asks the
model for nstates, initial_state(), derivatives(), plant_input(),
state_view(), sensor_truth(), diverged() and the extra log channels. Both
model/physical/vehicle.py (planar, 10 states) and vehicle17.py (full, 80
states) provide them.
"""

import math
import numpy as np
from model.physical.vehicle import WHEEL_NAMES, NWHEELS


def rk4_step(model, s, delta, u, dt, k1=None):
    """One RK4 step with inputs held constant (zero-order hold)."""
    n = len(s)
    if k1 is None:
        k1, _ = model.derivatives(s, delta, u)
    s2 = [s[i] + 0.5 * dt * k1[i] for i in range(n)]
    k2, _ = model.derivatives(s2, delta, u)
    s3 = [s[i] + 0.5 * dt * k2[i] for i in range(n)]
    k3, _ = model.derivatives(s3, delta, u)
    s4 = [s[i] + dt * k3[i] for i in range(n)]
    k4, _ = model.derivatives(s4, delta, u)
    return [s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
            for i in range(n)]


BASE_LOG = ("t", "X", "Y", "psi", "vx", "vy", "r",
            "wFL", "wFR", "wRL", "wRR",
            "delta", "T_req", "T_FL", "T_FR", "T_RL", "T_RR", "dw_target",
            "beta", "ay", "ax",
            "kFL", "kFR", "kRL", "kRR",
            "FzFL", "FzFR", "FzRL", "FzRR", "P_total",
            "vx_est", "r_meas", "apps", "bps", "handwheel", "plaus_cut")


def simulate(model, controller, maneuver, dt=2.5e-4, log_every=4,
             sensors=None, ctrl_every=1):
    """Run one maneuver with one controller. Returns a dict of numpy arrays
    (logged every `log_every` steps => 1 kHz logs at the default dt).

    sensors=SensorSuite → the controller reads ONLY sensor measurements
                    (WSS/IMU/SAS/APPS/BPS/GPS), sampled every `ctrl_every`
                    physics steps = the VCU rate, held (ZOH) in between.
                    Every real run; the firmware cannot read anything else.
    sensors=None  → the controller's update(s, delta, T_req, dt) is called
                    with the true state. Only verify.py's physics fixtures
                    use this, to drive the plant with exact known torques."""
    from model.config import cfg
    p = model.p
    controller.reset()
    dt_vcu = dt * ctrl_every

    # closed-loop "driver replacement" (tracks.py): inputs from (t, state);
    # scripted maneuvers keep their open-loop inputs(t). run_matrix reuses ONE
    # maneuver object across every config, and TrackDriver carries PI state, so
    # reset it here rather than relying on its t < t_last auto-reset.
    driver = getattr(maneuver, "driver", None)
    if driver is not None and hasattr(driver, "reset"):
        driver.reset()
    if driver is not None and hasattr(driver, "bind"):
        driver.bind(model)

    adapter = None
    if sensors is not None:
        from model.sensors import DriverAdapter
        sensors.reset()
        adapter = DriverAdapter(p)

    n_steps = int(round(maneuver.duration / dt))
    # rolling, no initial slip — from each wheel's OWN contact-patch speed
    # resolved along its own heading, so a maneuver that starts steered would
    # still begin at kappa = 0 rather than with a step of spurious drive force.
    s0 = [0.0] * model.nstates
    delta0 = (driver(0.0, s0) if driver is not None else maneuver.inputs(0.0))[0]
    s = model.initial_state(maneuver.vx0, delta0)
    if hasattr(model, "reset_run"):
        model.reset_run(dt_vcu)

    extra_keys = tuple(getattr(model, "EXTRA_LOG", ()))
    log = {k: [] for k in BASE_LOG + extra_keys}

    dbg = None
    u = None
    # the first sensor frame needs a force evaluation at the initial state
    _, last_info = model.derivatives(s, delta0, model.zero_input())
    sr = None
    for k in range(n_steps):
        t = k * dt
        delta, T_req = (driver(t, s) if driver is not None
                        else maneuver.inputs(t))
        if k % ctrl_every == 0 or dbg is None:
            if sensors is None:
                dbg = controller.update(s, delta, T_req, dt_vcu)
                drv = None
            else:
                pedals = maneuver.pedals(t) if maneuver.pedals else None
                drv = adapter.inputs(delta, T_req, pedals)
                braking = drv.bps_bar > cfg.sensors.brake_pressure_sens.actuated_bar
                sr = sensors.measure(model, s, drv, last_info, dt_vcu, braking)
                dbg = controller.update_from_sensors(sr, dt_vcu)
            u = model.plant_input(dbg, drv, dt_vcu)

        # evaluate derivatives once for logging, reuse as RK4's k1
        k1, info = model.derivatives(s, delta, u)
        last_info = info

        if k % log_every == 0:
            X, Y, psi, vx, vy, r, omegas = model.state_view(s)
            log["t"].append(t)
            log["X"].append(X);   log["Y"].append(Y)
            log["psi"].append(psi)
            log["vx"].append(vx); log["vy"].append(vy)
            log["r"].append(r)
            for nm, w in zip(WHEEL_NAMES, omegas):
                log["w" + nm].append(w)
            for i, nm in enumerate(WHEEL_NAMES):
                log["T" + "_" + nm].append(dbg.T[i])
            log["delta"].append(delta); log["T_req"].append(T_req)
            log["dw_target"].append(dbg.dw_target)
            log["beta"].append(math.atan2(vy, max(vx, 0.5)))
            log["ay"].append(info["ay"]); log["ax"].append(info["ax"])
            for i, nm in enumerate(WHEEL_NAMES):
                log["k" + nm].append(info["kappa"][i])
                log["Fz" + nm].append(info["Fz"][i])
            log["P_total"].append(sum(dbg.T[i] * omegas[i]
                                      for i in range(NWHEELS)))
            if sr is not None:
                log["vx_est"].append(sr.vx_est)
                log["r_meas"].append(sr.yaw_rate)
                log["apps"].append(sr.apps_pct)
                log["bps"].append(sr.bps_bar)
                log["handwheel"].append(sr.handwheel_deg)
                log["plaus_cut"].append(1.0 if getattr(controller, "plaus_cut", False) else 0.0)
            else:
                for nm in ("vx_est", "r_meas", "apps", "bps", "handwheel",
                           "plaus_cut"):
                    log[nm].append(0.0)
            if extra_keys:
                ex = model.log_extra(s, info)
                for nm in extra_keys:
                    log[nm].append(ex[nm])

        s = rk4_step(model, s, delta, u, dt, k1=k1)

        # bail out if the car has physically spun / diverged — logged data
        # up to here is still useful, and the run is flagged in the table
        if model.diverged(s):
            break

    return {k: np.asarray(v) for k, v in log.items()}


# ─────────────────────────────────────────────────────────────── metrics
def metrics(log, p):
    """Scalar summary of one run — used for the comparison table."""
    dw_act = log["wRR"] - log["wRL"]
    dwf_act = log["wFR"] - log["wFL"]
    out = {
        # body sideslip: big values = the rear stepping out
        "max |beta| [deg]": float(np.degrees(np.max(np.abs(log["beta"])))),
        # how well the rear wheel-speed difference matched corner geometry
        "dw RMSE [rad/s]": float(np.sqrt(np.mean((log["dw_target"] - dw_act) ** 2))),
        # the front axle's own kinematic target carries a cos(delta): both front
        # wheels share x = a, so the sin(delta) terms cancel in the difference
        "dw_f RMSE [rad/s]": float(np.sqrt(np.mean(
            (log["dw_target"] * (p.track_f / p.track_r) * np.cos(log["delta"])
             - dwf_act) ** 2))),
        # worst wheel slip — inner-wheel spin shows up here
        # worst wheel — now genuinely any of four, not just the driven rears
        "max |kappa| [-]": float(np.max(np.abs(np.stack(
            [log["k" + nm] for nm in WHEEL_NAMES])))),
        "max |ay| [g]": float(np.max(np.abs(log["ay"])) / 9.81),
        # how close the run came to the rules cap — only meaningful once every
        # wheel is driven
        "max P [kW]": float(np.max(log["P_total"]) / 1e3),
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


# the comparison table's columns. run_sim.py imports this rather than keeping
# its own copy — they were two hand-maintained lists of the same thing.
METRIC_COLS = ["max |beta| [deg]", "dw RMSE [rad/s]", "dw_f RMSE [rad/s]",
               "max |kappa| [-]", "max |ay| [g]", "max P [kW]"]


def print_table(maneuver, results):
    cols = METRIC_COLS
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
