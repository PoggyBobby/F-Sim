"""Entry point: run all maneuvers across all four controller configurations,
print comparison tables, save plots + animated replays, and record the run.

EVERY run is recorded. `runs/<NNN>__<date>__<label>/` gets the time-series
data, the plots, the replay videos, the complete parameter set with its
provenance tags, and an explicit CHANGES.md saying what moved since the
previous run and what did not. See runlog.py.

Usage:
    python run_sim.py                          # everything, auto-labeled
    python run_sim.py -l "ttc-tires" -n "first real tire fit"
    python run_sim.py --maneuver corner_exit   # one maneuver
    python run_sim.py --no-animate             # skip the videos (faster)
    python run_sim.py --show                   # open plot windows too
"""

import argparse
import inspect
import os
import sys
import time

import matplotlib

from model.params import default_setup
from model.physical.tires.tire import MagicFormulaTire
from model.physical.vehicle import VehicleModel
from controllers.python.torque_split import make_configs
from model.maneuvers.maneuvers import step_steer, corner_exit, slalom, pedal_check
from model.maneuvers.tracks import track_maneuvers, model_for, CORNER_TYPES
import model.maneuvers.tracks as _tracks
from model.sim import run_matrix, print_table
from style import RC, CONFIG_COLORS, REF_COLOR, config_lw, config_z
import runlog
from runlog import RunRecorder

METRIC_COLS = ["max |beta| [deg]", "dw RMSE [rad/s]",
               "max |kappa| [-]", "max |ay| [g]"]


def _plot_series(ax, results, xkey, ykey, transform=None):
    """One line per controller config, fixed colors, combined drawn on top."""
    for name, res in results.items():
        log = res["log"]
        y = log[ykey] if transform is None else transform(log)
        ax.plot(log[xkey], y, color=CONFIG_COLORS[name], lw=config_lw(name),
                zorder=config_z(name), label=name)


def _footer(fig, maneuver, tag):
    fig.text(0.01, 0.005,
             f"{tag} · {maneuver.name}: {maneuver.description} · all four "
             "configs receive identical driver inputs — differences are the "
             "torque split alone",
             fontsize=7.5, color="#898781")


def plot_response(plt, maneuver, results, outdir, tag=""):
    fig, axs = plt.subplots(2, 2, figsize=(11, 7.8))
    fig.suptitle(f"Vehicle response — {maneuver.name}  "
                 f"({maneuver.description})", fontweight="bold")

    ax = axs[0, 0]
    _plot_series(ax, results, "t", "r")
    ax.set_title("Yaw rate — how the car rotated", fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("yaw rate  r  [rad/s]")

    ax = axs[0, 1]
    for name, res in results.items():
        log = res["log"]
        ax.plot(log["X"], log["Y"], color=CONFIG_COLORS[name],
                lw=config_lw(name), zorder=config_z(name))
    ax.set_title("Path over the ground (plan view)\n"
                 "(wider arc = the car yawed less than asked)", fontsize=10)
    ax.set_xlabel("position  X  [m]")
    ax.set_ylabel("position  Y  [m]")
    ax.set_aspect("equal", adjustable="datalim")

    ax = axs[1, 0]
    _plot_series(ax, results, "t", "ay", transform=lambda l: l["ay"] / 9.81)
    ax.set_title("Lateral acceleration — grip in use\n"
                 "(tire-limited ceiling ≈ µ·(1+downforce/weight))",
                 fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("lateral acceleration  a_y  [g]")

    ax = axs[1, 1]
    import numpy as _np
    _plot_series(ax, results, "t", "beta",
                 transform=lambda l: _np.degrees(l["beta"]))
    ax.set_title("Body sideslip — how sideways the car is\n"
                 "(small = planted; growing = sliding toward a spin)",
                 fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("body sideslip  β  [deg]")

    handles, labels = axs[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, fontsize=8.5,
               bbox_to_anchor=(0.5, 0.012))
    _footer(fig, maneuver, tag)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    path = os.path.join(outdir, f"{maneuver.slug}_response.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def plot_wheels(plt, maneuver, results, outdir, tag="", k_spin=None,
                p_ceiling_kw=None):
    fig, axs = plt.subplots(2, 2, figsize=(11, 7.8))
    fig.suptitle(f"Wheels & torque split — {maneuver.name}  "
                 f"({maneuver.description})", fontweight="bold")

    ax = axs[0, 0]
    _plot_series(ax, results, "t", "r",
                 transform=lambda l: l["dw_target"] - (l["wRR"] - l["wRL"]))
    ax.axhline(0.0, ls="--", color=REF_COLOR, lw=1.6, zorder=1,
               label="zero error = wheels exactly match corner geometry")
    ax.set_title("s-diff objective: wheel-speed-difference error\n"
                 "(s-diff should pin this to zero)",
                 fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("Δω error = target − actual  [rad/s]")

    ax = axs[0, 1]
    _plot_series(ax, results, "t", "kRL")
    if k_spin:
        ax.axhline(k_spin, ls=":", color="#d03b3b", lw=1.6, zorder=1)
        ax.text(0.99, k_spin, f" tire peak κ ≈ {k_spin:.2f} — WHEELSPIN above ",
                transform=ax.get_yaxis_transform(), ha="right", va="bottom",
                fontsize=8, color="#d03b3b")
    ax.set_title("Inner-rear slip ratio — the wheelspin watch\n"
                 "(above the red line more spin makes LESS force)",
                 fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("slip ratio  κ_RL  [–]")

    ax = axs[1, 0]
    _plot_series(ax, results, "t", "r",
                 transform=lambda l: l["T_RR"] - l["T_RL"])
    ax.set_title("Commanded torque split — the actuator itself\n"
                 "(positive = more torque to the RIGHT rear → yaws LEFT)",
                 fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("torque split  ΔT = T_RR − T_RL  [N·m]")

    ax = axs[1, 1]
    _plot_series(ax, results, "t", "P_total",
                 transform=lambda l: l["P_total"] / 1e3)
    ax.axhline(80.0, ls=":", color=REF_COLOR, lw=1.8, zorder=1,
               label="80 kW rules cap (EV.4.2)")
    if p_ceiling_kw and p_ceiling_kw < 79:
        ax.axhline(p_ceiling_kw, ls="--", color=REF_COLOR, lw=1.4, zorder=1,
                   label=f"2-motor ceiling 2×P_peak = {p_ceiling_kw:.0f} kW "
                         "@ 380 V")
    ax.set_title("Total mechanical drive power\n"
                 "(the 2-rear-motor build cannot reach the rules cap)",
                 fontsize=10)
    ax.set_xlabel("time  t  [s]")
    ax.set_ylabel("drive power  P  [kW]")

    handles, labels = axs[0, 0].get_legend_handles_labels()
    h2, l2 = axs[1, 1].get_legend_handles_labels()
    for h, l in zip(h2, l2):
        if l not in labels:
            handles.append(h); labels.append(l)
    fig.legend(handles, labels, loc="lower center", ncol=4, fontsize=8.5,
               bbox_to_anchor=(0.5, 0.012))
    _footer(fig, maneuver, tag)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    path = os.path.join(outdir, f"{maneuver.slug}_wheels.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return path


def summary_table(maneuver, results):
    """The printed comparison table, in a form summary.md can render."""
    rows = []
    for name, res in results.items():
        m = res["metrics"]
        rows.append([name] + [f"{m[c]:.3f}" for c in METRIC_COLS] +
                    ["no" if m["finished"] else "**YES**"])
    return {"title": f"{maneuver.name} — {maneuver.description}",
            "cols": METRIC_COLS, "rows": rows}


def _defaults(fn):
    """The default knob values straight out of maneuvers.py — the single
    source of truth; nothing here hardcodes a second copy of them."""
    return {k: p.default for k, p in inspect.signature(fn).parameters.items()
            if p.default is not inspect.Parameter.empty}


def _ask(label, unit, default, lo=None, hi=None):
    """One interactive question. Enter keeps the default."""
    while True:
        raw = input(f"    {label} [{unit}]  ({default:g}): ").strip()
        if not raw:
            return default
        try:
            v = float(raw)
        except ValueError:
            print("      not a number — try again (Enter keeps the default)")
            continue
        if lo is not None and (v < lo or (hi is not None and v > hi)):
            print(f"      out of range ({lo:g}–{hi:g}) — try again")
            continue
        return v


def build_maneuvers(args, vp, interactive):
    """Build the selected maneuvers from (in priority order): interactive
    answers, command-line flags, maneuvers.py defaults."""
    sd, cd_, ld = _defaults(step_steer), _defaults(corner_exit), _defaults(slalom)
    # corner-exit throttle is stored as N·m in maneuvers.py (None = 45% of
    # peak); the user-facing knob is % of peak wheel torque
    peak = 2.0 * vp.T_wheel_max
    cd_throttle_pct = 45.0

    want = lambda slug: args.maneuver in ("all", slug)
    v = {   # knob -> (flag value or maneuvers.py default)
        "st_v": args.step_speed or sd["vx0"],
        "st_d": args.step_deg or sd["delta_deg"],
        "st_T": args.step_torque or sd["T_hold"],
        "sl_v": args.slalom_speed or ld["vx0"],
        "sl_d": args.slalom_deg or ld["delta_deg"],
        "sl_f": args.slalom_hz or ld["freq_hz"],
        "sl_T": args.slalom_torque or ld["T_hold"],
        "co_v": args.corner_speed or cd_["vx0"],
        "co_d": args.corner_deg or cd_["delta_deg"],
        "co_p": args.corner_throttle or cd_throttle_pct,
    }

    if interactive:
        print("── maneuver setup — Enter keeps the (default) ──────────────")
        if want("step_steer"):
            print("  step steer:")
            v["st_v"] = _ask("speed", "m/s", v["st_v"], 3, 35)
            v["st_d"] = _ask("steer step", "deg", v["st_d"], 0.5, 25)
            v["st_T"] = _ask("hold torque, total", "N·m", v["st_T"], 0, peak)
        if want("corner_exit"):
            print("  corner exit:")
            v["co_v"] = _ask("entry speed", "m/s", v["co_v"], 3, 35)
            v["co_d"] = _ask("corner steer", "deg", v["co_d"], 0.5, 25)
            v["co_p"] = _ask("exit throttle", "% of peak", v["co_p"], 0, 100)
        if want("slalom"):
            print("  slalom (sine steer):")
            v["sl_v"] = _ask("speed", "m/s", v["sl_v"], 3, 35)
            v["sl_d"] = _ask("steer amplitude", "±deg", v["sl_d"], 0.5, 25)
            v["sl_f"] = _ask("frequency", "Hz", v["sl_f"], 0.1, 3)
            v["sl_T"] = _ask("hold torque, total", "N·m", v["sl_T"], 0, peak)
        print()

    mans = []
    if want("step_steer"):
        mans.append(step_steer(delta_deg=v["st_d"], vx0=v["st_v"],
                               T_hold=v["st_T"]))
    if want("corner_exit"):
        mans.append(corner_exit(delta_deg=v["co_d"], vx0=v["co_v"],
                                T_max_total=v["co_p"] / 100.0 * peak))
    if want("slalom"):
        mans.append(slalom(delta_deg=v["sl_d"], freq_hz=v["sl_f"],
                           vx0=v["sl_v"], T_hold=v["sl_T"]))
    if want("pedal_check"):
        mans.append(pedal_check())

    # track tests (tracks.py): --maneuver tracks; not part of "all" so the
    # default run stays the four quick maneuvers
    if args.maneuver == "tracks":
        thr = (args.track_throttle if args.track_throttle is not None
               else _tracks.DEFAULT_THROTTLE_PCT)
        frac = (args.track_entry_frac if args.track_entry_frac is not None
                else _tracks.DEFAULT_ENTRY_FRAC)
        if interactive:
            print("── track tests — Enter keeps the (default) ──────────────")
            thr = _ask("apex throttle step", "% of peak", thr, 0, 100)
            frac = _ask("entry speed fraction of √(µgR)", "-", frac, 0.3, 1.0)
            print()
        mans += track_maneuvers(vp, types=args.track, radius=args.track_radius,
                                throttle_pct=thr, entry_frac=frac,
                                direction=-1 if args.track_right else 1)
    return mans


def main():
    ap = argparse.ArgumentParser(description="SR s-diff basic sim")
    ap.add_argument("--maneuver", default="all",
                    choices=["all", "step_steer", "corner_exit", "slalom",
                             "pedal_check", "tracks"],
                    help="'all' = the four scripted maneuvers; 'tracks' = "
                         "the corner test matrix (tracks.py)")
    tg = ap.add_argument_group("track tests (--maneuver tracks)")
    tg.add_argument("--track", nargs="*", default=["all"],
                    metavar="TYPE",
                    help="corner types to run: " +
                         " ".join(list(CORNER_TYPES) + ["split_mu", "all"]) +
                         " (default all)")
    tg.add_argument("--track-radius", type=float,
                    help="run every selected type at this one radius [m] "
                         "instead of its default radii")
    tg.add_argument("--track-throttle", type=float,
                    help="apex throttle step [%% of peak axle torque] "
                         f"(default {_tracks.DEFAULT_THROTTLE_PCT:g})")
    tg.add_argument("--track-entry-frac", type=float,
                    help="entry speed as a fraction of √(µgR) "
                         f"(default {_tracks.DEFAULT_ENTRY_FRAC:g})")
    tg.add_argument("--track-right", action="store_true",
                    help="right-hand corners instead of left-hand")
    ap.add_argument("--sil", action="store_true",
                    help="add the real VCU firmware as a fifth config, "
                         "software-in-the-loop (build it first: make -C sil)")
    ap.add_argument("--perfect-state", action="store_true",
                    help="bypass the sensor stack: controller reads the "
                         "sim's true state (pre-2026-08-31 behavior)")
    ap.add_argument("--ask", action="store_true",
                    help="interactively ask for the maneuver test values "
                         "(speed, steer, throttle) — Enter keeps defaults")
    g = ap.add_argument_group("maneuver values (default: maneuvers.py)")
    g.add_argument("--step-speed", type=float, help="step steer: speed [m/s]")
    g.add_argument("--step-deg", type=float, help="step steer: angle [deg]")
    g.add_argument("--step-torque", type=float, help="step steer: hold torque [N·m]")
    g.add_argument("--slalom-speed", type=float, help="slalom: speed [m/s]")
    g.add_argument("--slalom-deg", type=float, help="slalom: amplitude [±deg]")
    g.add_argument("--slalom-hz", type=float, help="slalom: frequency [Hz]")
    g.add_argument("--slalom-torque", type=float, help="slalom: hold torque [N·m]")
    g.add_argument("--corner-speed", type=float, help="corner exit: entry speed [m/s]")
    g.add_argument("--corner-deg", type=float, help="corner exit: steer [deg]")
    g.add_argument("--corner-throttle", type=float,
                   help="corner exit: exit throttle [%% of peak torque]")
    ap.add_argument("-l", "--label", default=None,
                    help="what this run IS (e.g. 'ttc-tires', 'kp-sweep-hi'). "
                         "Auto-derived from what changed if omitted.")
    ap.add_argument("-n", "--note", default="",
                    help="free-text note stored with the run")
    ap.add_argument("--no-animate", dest="animate", action="store_false",
                    help="skip the replay videos (a few seconds faster)")
    ap.add_argument("--fps", type=int, default=30, help="replay frame rate")
    ap.add_argument("--csv-hz", type=float, default=200.0,
                    help="sample rate of the saved time-series CSVs")
    ap.add_argument("--runs-dir", default="runs")
    ap.add_argument("--show", action="store_true", help="open plot windows")
    args = ap.parse_args()

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update(RC)

    # all numbers come from the params.yaml files (via model/config.py)
    vp, tp_f, tp_r, cp = default_setup()
    model = VehicleModel(vp, MagicFormulaTire(tp_f), MagicFormulaTire(tp_r))
    maneuvers = build_maneuvers(args, vp,
                                interactive=args.ask and sys.stdin.isatty())
    config_names = [c.name for c in make_configs(vp, tp_f, tp_r, cp)]
    if args.sil:
        if args.perfect_state:
            ap.error("--sil needs the sensor stack; drop --perfect-state")
        from sil.vcu_sil import SilController, SIL_NAME
        config_names.append(SIL_NAME)

    # sensor stack: the controller reads WSS/IMU/SAS/APPS/BPS at the VCU
    # rate — the default since 2026-08-31 (--perfect-state to bypass)
    from model.config import cfg
    dt = 2.5e-4
    if args.perfect_state:
        sensors, ctrl_every = None, 1
    else:
        from model.sensors import SensorSuite
        sensors = SensorSuite(vp)
        ctrl_every = max(1, int(round(1.0 / (dt * cfg.sensors.vcu.rate_hz))))

    rec = RunRecorder(label=args.label, note=args.note, csv_hz=args.csv_hz,
                      runs_dir=args.runs_dir,
                      sim_settings={"dt_s": dt, "maneuver_filter":
                                    args.maneuver, "replay_fps": args.fps,
                                    "animated": bool(args.animate),
                                    "sensor_stack": not args.perfect_state,
                                    "sil": bool(args.sil),
                                    "vcu_rate_hz": (cfg.sensors.vcu.rate_hz
                                                    if not args.perfect_state
                                                    else "physics rate")})
    run_dir = rec.begin(maneuvers, config_names)

    print("SJSU Spartan Racing — software-diff basic sim")
    print(f"  total mass {vp.m_total:.0f} kg, wheelbase {vp.wheelbase:.2f} m, "
          f"peak wheel torque {vp.T_wheel_max:.0f} N·m/side"
          "  (sources/tags: the params.yaml files)")
    print(f"  configs: {' | '.join(config_names)}")
    print("  feedback: " + ("PERFECT STATE (sensor stack bypassed)"
                            if args.perfect_state else
                            f"sensor stack (WSS/IMU/SAS/APPS/BPS) at "
                            f"{cfg.sensors.vcu.rate_hz:g} Hz VCU rate"))
    print(f"  run {rec.run_number:03d} '{rec.label}' -> {run_dir}\n")

    plots, replays, tables = [], [], {}
    for man in maneuvers:
        t0 = time.time()
        controllers = make_configs(vp, tp_f, tp_r, cp)
        if args.sil:
            controllers.append(SilController(vp))
        # split-µ track tests run on a patched plant (tracks.model_for);
        # every other maneuver gets the base model back unchanged
        results = run_matrix(model_for(man, model), controllers, man, dt=dt,
                             sensors=sensors, ctrl_every=ctrl_every)
        results = run_matrix(model, controllers, man, dt=dt, sensors=sensors,
                             ctrl_every=ctrl_every)
        print_table(man, results)
        if args.sil:
            print("    " + controllers[-1].summary())
        print(f"    ({time.time() - t0:.1f} s of compute)")

        rec.add_results(man, results)
        tables[man.slug] = summary_table(man, results)
        from animate import spin_threshold
        tag = f"run {rec.run_number:03d} · {rec.label}"
        plots.append(plot_response(plt, man, results,
                                   os.path.join(run_dir, "plots"), tag=tag))
        plots.append(plot_wheels(plt, man, results,
                                 os.path.join(run_dir, "plots"), tag=tag,
                                 k_spin=spin_threshold(vp, MagicFormulaTire(tp_r)),
                                 p_ceiling_kw=2 * vp.motor_P_peak / 1e3))

        if args.animate:
            from animate import animate_maneuver
            t0 = time.time()
            out = os.path.join(run_dir, "replay", f"{man.slug}.mp4")
            path = animate_maneuver(plt, man, results, vp,
                                    MagicFormulaTire(tp_r), out,
                                    fps=args.fps, show=args.show)
            if path:
                replays.append(path)
                print(f"    replay: {os.path.relpath(path)} "
                      f"({time.time() - t0:.1f} s to render)")

    rec.finish(plots, replays, tables)

    # what this run was, and how it differs from the one before it
    d, c = rec.param_diff, rec.code_diff
    print(f"\nRun {rec.run_number:03d} '{rec.label}' saved to {run_dir}")
    if rec.prev_id is None:
        print("  first recorded run — nothing to compare against")
    else:
        print(f"  vs run {rec.prev_id.split('__')[0]}:")
        if d["changed"]:
            for n in sorted(d["changed"]):
                ch = d["changed"][n]
                print(f"    CHANGED  {n}: {ch['from']:g} -> {ch['to']:g}")
        else:
            print("    CHANGED  (no car-data parameter changed)")
        for n in sorted(d["added"]):
            e = d["added"][n]
            print(f"    ADDED    {n} = {e['value']:g}  ({e['tag']})")
        for slug in sorted(rec.man_diff["changed"]):
            for k, ch in sorted(rec.man_diff["changed"][slug].items()):
                print(f"    TEST     {slug} {k}: {ch['from']:g} -> {ch['to']:g}"
                      "  (maneuver value, metrics not comparable)")
        print(f"    SAME     {len(d['unchanged'])} parameters unchanged")
        code_moved = [f for f in c["changed"] + c["added"]
                      if f not in runlog.DATA_FILES]
        print("    CODE     " + (", ".join(code_moved) if code_moved
                                 else "unchanged (model source identical)"))
    print(f"  read:  {os.path.join(run_dir, 'summary.md')}"
          f"  |  {os.path.join(run_dir, 'CHANGES.md')}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
