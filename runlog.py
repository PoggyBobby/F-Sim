"""Run recording — every sim run is labeled, saved, and diffed against the
run before it.

WHY: results from this sim get quoted in design reviews. A number is only
worth quoting if you can say which car it came from. So every run writes its
own folder containing the FULL parameter set it ran on (each value with its
provenance tag straight out of its params.yaml), the raw time-series data, the
summary metrics, and an explicit statement of WHAT CHANGED and WHAT STAYED
THE SAME versus the previous run.

    runs/
      index.csv                     one row per run, newest last
      all_metrics.csv               every metric of every run, tidy/long form
      latest -> 007__.../           symlink to the most recent run
      007__2026-08-30_1815__mu0-1.40-to-1.60/
        summary.md                  read this first: label, headline table
        CHANGES.md                  changed vs unchanged, versus run 006
        PARAMETERS.md               all parameters, values, tags, change marks
        manifest.json               machine-readable everything (incl. code hashes)
        metrics.csv                 this run's metrics, tidy/long form
        data/<maneuver>__<config>.csv     time series, one file per run
        plots/*.png
        replay/*.mp4

The label is not decoration: an unlabeled run gets an automatic one derived
from what actually changed (e.g. `mu0-1.40-to-1.60`, `code-change`, `repeat`),
so a folder name always says what the run WAS.
"""

import csv
import hashlib
import json
import os
import platform
import re
import shutil
import sys
from datetime import datetime

import numpy as np

from model.config import cfg

RUNS_DIR = "runs"

# Manifest schema version. Bumped to 2 when the parameters moved out of
# car_data.py into the per-component params.yaml files: parameter keys are now
# dotted config paths (`tires.mu0`) instead of module constants (`TIRE_MU0`),
# so a v1 manifest's parameter block cannot be diffed against a v2 one.
SCHEMA = 2

# Parameters excluded from the diff: physical constants that cannot change.
# (The unit conversions that used to be excluded here no longer exist as
# parameters — model/config.py applies them during loading.)
EXCLUDED_TAGS = {"CONSTANT"}

# The params.yaml files are the DATA files: an edit to one shows up in the
# parameter diff, so it must not ALSO be reported as a code change (that
# warning is about the model itself moving under the numbers).
DATA_FILES = tuple(sorted(cfg.sections().values()))

CODE_FILES = ("model/config.py", "model/params.py", "model/sim.py",
              "model/physical/vehicle.py", "model/physical/tires/tire.py",
              "model/sensors/suite.py", "model/sensors/driver.py",
              "model/sensors/readings.py", "model/sensors/quantize.py",
              "model/sensors/imu_6axis/imu.py",
              "model/sensors/wheel_speed/wss.py",
              "model/sensors/steering_angle/sas.py",
              "model/sensors/throttle_pos/apps.py",
              "model/sensors/brake_pressure_sens/bps.py",
              "model/maneuvers/maneuvers.py", "model/maneuvers/tracks.py",
              "controllers/python/torque_split.py",
              "run_sim.py", "runlog.py", "animate.py", "style.py",
              "verify.py", "tire_fit.py") + DATA_FILES


# ───────────────────────────────────────────────────────── parameter snapshot
def param_snapshot():
    """Every car-data number the sim can see, with its value and provenance.

    Keys are the dotted config paths (`tires.mu0`, `sensors.imu_6axis.lpf_hz`),
    so a name in a run manifest is the same string you grep for in the code and
    in the params.yaml files.

    Provenance comes from each entry's `status:` field rather than from a
    trailing source comment, which is the whole reason the YAML entries carry
    one: the tag is data now, not something scraped back out of a comment.
    """
    snap = {}
    for path, entry in cfg.params().items():
        tag = cfg.tag_of(path)
        if tag in EXCLUDED_TAGS:
            continue
        val = entry["si"]
        if not isinstance(val, (int, float)) or isinstance(val, bool):
            continue
        snap[path] = {"value": float(val), "tag": tag,
                      "note": str(entry.get("status", "")).strip()}
    return snap


def _sha(path):
    with open(path, "rb") as fh:
        return hashlib.sha256(fh.read()).hexdigest()[:12]


def code_snapshot():
    """SHA of every file that can change what the sim computes.

    Includes the params.yaml files: they are data, but an edit to one is still
    something a later run needs to know about. DATA_FILES separates them out
    again when the "the code moved under the numbers" warning is written.
    """
    return {f: _sha(f) for f in CODE_FILES if os.path.exists(f)}


def maneuver_snapshot(maneuvers):
    return {m.slug: {"name": m.name, "description": m.description,
                     "duration_s": m.duration, "vx0_mps": m.vx0,
                     "params": dict(getattr(m, "params", {}))}
            for m in maneuvers}


def diff_maneuvers(prev, cur):
    """Changes in the maneuver TEST VALUES (speed/steer/throttle) between
    runs. Scenario settings, not car data — but a run at 20 m/s must never
    be compared to one at 15 m/s without the difference being labeled."""
    changed = {}
    comparable = False
    for slug, m in cur.items():
        pm = prev.get(slug)
        if not pm:
            continue
        pp = pm.get("params") or {}
        if pp:
            comparable = True
        for k, val in (m.get("params") or {}).items():
            if k in pp and abs(pp[k] - val) > 1e-9:
                changed.setdefault(slug, {})[k] = {"from": pp[k], "to": val}
    return {"changed": changed,
            "added": sorted(x for x in cur if x not in prev),
            "removed": sorted(x for x in prev if x not in cur),
            "comparable": comparable}


# ─────────────────────────────────────────────────────────────────── diffing
def diff_params(prev, cur, comparable=True):
    """CHANGED / ADDED / REMOVED / UNCHANGED between two parameter snapshots.

    `comparable=False` (a previous run recorded under an older manifest schema)
    returns an explicitly incomparable diff. The alternative — diffing dotted
    paths against the old module constants — would report every parameter as
    removed and re-added, which reads like a catastrophic change and is really
    just a rename.
    """
    if not comparable:
        return {"changed": {}, "added": {}, "removed": {}, "unchanged": {},
                "incomparable": "previous run used an older manifest schema "
                                "(parameters were named as car_data.py "
                                "constants); parameter diff skipped"}
    changed, added, removed, unchanged = {}, {}, {}, {}
    for name, entry in cur.items():
        if name not in prev:
            added[name] = entry
        elif abs(prev[name]["value"] - entry["value"]) > 1e-12:
            changed[name] = {"from": prev[name]["value"], "to": entry["value"],
                             "tag_from": prev[name]["tag"], "tag_to": entry["tag"],
                             "note": entry["note"]}
        else:
            unchanged[name] = entry
            if prev[name]["tag"] != entry["tag"]:
                # same number, new provenance — worth seeing (e.g. PLACEHOLDER
                # promoted to MEASURED). Recorded as a retag, not a change.
                unchanged[name] = dict(entry, retagged_from=prev[name]["tag"])
    for name, entry in prev.items():
        if name not in cur:
            removed[name] = entry
    return {"changed": changed, "added": added, "removed": removed,
            "unchanged": unchanged}


def diff_code(prev, cur):
    changed = sorted(f for f in cur if f in prev and prev[f] != cur[f])
    added = sorted(f for f in cur if f not in prev)
    removed = sorted(f for f in prev if f not in cur)
    return {"changed": changed, "added": added, "removed": removed}


def _auto_label(pdiff, cdiff, mdiff, first_run):
    """A folder name that says what the run was, when nobody supplied one."""
    if first_run:
        return "baseline"
    ch = pdiff["changed"]
    if ch:
        bits = []
        for name in sorted(ch)[:2]:
            d = ch[name]
            bits.append(f"{name.lower()}-{d['from']:g}-to-{d['to']:g}")
        extra = "" if len(ch) <= 2 else f"-and-{len(ch) - 2}-more"
        return "__".join(bits) + extra
    if pdiff["added"] or pdiff["removed"]:
        return "params-added"
    if mdiff["changed"]:
        slug = sorted(mdiff["changed"])[0]
        k = sorted(mdiff["changed"][slug])[0]
        d = mdiff["changed"][slug][k]
        return f"{slug}-{k}-{d['from']:g}-to-{d['to']:g}"
    if cdiff["changed"] or cdiff["added"]:
        return "code-change"
    return "repeat"


def _slug(text):
    s = re.sub(r"[^a-zA-Z0-9]+", "-", str(text)).strip("-").lower()
    return s[:60] or "run"


# ───────────────────────────────────────────────────────────────── recorder
class RunRecorder:
    """Creates one run folder and writes everything about that run into it."""

    def __init__(self, label=None, note="", sim_settings=None, csv_hz=200.0,
                 runs_dir=RUNS_DIR):
        self.runs_dir = runs_dir
        self.user_label = label
        self.note = note or ""
        self.sim_settings = dict(sim_settings or {})
        self.csv_hz = float(csv_hz)
        self.params = param_snapshot()
        self.code = code_snapshot()
        self.metric_rows = []
        self.started = datetime.now()

    # -- setup ------------------------------------------------------------
    def begin(self, maneuvers, config_names):
        os.makedirs(self.runs_dir, exist_ok=True)
        self.maneuvers = maneuver_snapshot(maneuvers)
        self.config_names = list(config_names)

        prev = self._load_previous()
        self.prev_id = prev["run_id"] if prev else None
        same_schema = bool(prev) and prev.get("schema", 1) == SCHEMA
        self.param_diff = diff_params(prev["parameters"] if prev else {},
                                      self.params,
                                      comparable=(not prev) or same_schema)
        self.code_diff = diff_code(prev["code_sha256"] if prev else {}, self.code)
        self.man_diff = diff_maneuvers(prev.get("maneuvers", {}) if prev else {},
                                       self.maneuvers)

        self.run_number = (int(self.prev_id.split("__")[0]) + 1) if self.prev_id else 1
        label = self.user_label or _auto_label(self.param_diff, self.code_diff,
                                               self.man_diff, prev is None)
        self.label = label
        stamp = self.started.strftime("%Y-%m-%d_%H%M")
        self.run_id = f"{self.run_number:03d}__{stamp}__{_slug(label)}"
        self.dir = os.path.join(self.runs_dir, self.run_id)
        os.makedirs(os.path.join(self.dir, "data"), exist_ok=True)
        os.makedirs(os.path.join(self.dir, "plots"), exist_ok=True)
        os.makedirs(os.path.join(self.dir, "replay"), exist_ok=True)

        self._write_manifest()
        self._write_changes()
        self._write_parameters()
        return self.dir

    def _load_previous(self):
        """Manifest of the most recent completed run, or None."""
        if not os.path.isdir(self.runs_dir):
            return None
        candidates = sorted(d for d in os.listdir(self.runs_dir)
                            if re.match(r"^\d{3}__", d)
                            and os.path.exists(os.path.join(self.runs_dir, d,
                                                            "manifest.json")))
        if not candidates:
            return None
        with open(os.path.join(self.runs_dir, candidates[-1], "manifest.json"),
                  encoding="utf-8") as fh:
            return json.load(fh)

    # -- during the run ---------------------------------------------------
    def add_results(self, maneuver, results):
        """Save one maneuver's time series (one CSV per config) + metrics."""
        for cfg, res in results.items():
            log, met = res["log"], res["metrics"]
            self._write_log_csv(maneuver, cfg, log)
            for key, val in met.items():
                if key == "finished":
                    continue
                self.metric_rows.append({
                    "run_id": self.run_id, "label": self.label,
                    "maneuver": maneuver.slug, "config": cfg,
                    "metric": key, "value": round(float(val), 6),
                })
            self.metric_rows.append({
                "run_id": self.run_id, "label": self.label,
                "maneuver": maneuver.slug, "config": cfg,
                "metric": "spun (did not finish)",
                "value": 0.0 if met["finished"] else 1.0,
            })

    def _write_log_csv(self, maneuver, cfg, log):
        """Time series, decimated to csv_hz — 1 kHz internal logs are more
        rows than anyone opens in a spreadsheet, and nothing in these
        channels moves faster than a few tens of Hz."""
        t = log["t"]
        if len(t) < 2:
            return
        dt_log = float(t[1] - t[0])
        step = max(1, int(round(1.0 / (self.csv_hz * dt_log))))
        keys = list(log.keys())
        path = os.path.join(self.dir, "data",
                            f"{maneuver.slug}__{_slug(cfg)}.csv")
        cols = np.column_stack([log[k][::step] for k in keys])
        header = ",".join(keys)
        np.savetxt(path, cols, delimiter=",", header=header, comments="",
                   fmt="%.6g")

    # -- teardown ---------------------------------------------------------
    def finish(self, plot_paths=(), replay_paths=(), tables=None):
        self._write_metrics_csv()
        self._write_summary(plot_paths, replay_paths, tables or {})
        self._append_index()
        self._point_latest_at_this_run()
        return self.dir

    # -- writers ----------------------------------------------------------
    def _write_manifest(self):
        manifest = {
            "schema": SCHEMA,
            "run_id": self.run_id,
            "run_number": self.run_number,
            "label": self.label,
            "note": self.note,
            "timestamp": self.started.isoformat(timespec="seconds"),
            "previous_run": self.prev_id,
            "sim_settings": self.sim_settings,
            "csv_log_hz": self.csv_hz,
            "maneuvers": self.maneuvers,
            "controller_configs": self.config_names,
            "parameters": self.params,
            "code_sha256": self.code,
            "parameter_diff_vs_previous": self.param_diff,
            "maneuver_diff_vs_previous": self.man_diff,
            "code_diff_vs_previous": self.code_diff,
            "environment": {
                "python": sys.version.split()[0],
                "numpy": np.__version__,
                "platform": platform.platform(),
            },
        }
        with open(os.path.join(self.dir, "manifest.json"), "w",
                  encoding="utf-8") as fh:
            json.dump(manifest, fh, indent=2)

    def _write_changes(self):
        d, cdf = self.param_diff, self.code_diff
        L = [f"# What changed in run {self.run_id}", ""]
        L.append(f"**Label:** {self.label}  ")
        if self.note:
            L.append(f"**Note:** {self.note}  ")
        L.append(f"**Compared against:** "
                 f"{self.prev_id or '— (first recorded run, nothing to compare)'}")
        L.append("")

        L.append("## CHANGED parameters")
        L.append("")
        if d.get("incomparable"):
            L.append(f"_Not comparable — {d['incomparable']}._")
            L.append("")
            L.append("The parameters themselves are listed in `PARAMETERS.md`. "
                     "From the next run on, diffs work normally again.")
            L.append("")
        elif d["changed"]:
            L.append("| parameter | was | now | provenance |")
            L.append("|---|---|---|---|")
            for n in sorted(d["changed"]):
                c = d["changed"][n]
                tag = (c["tag_to"] if c["tag_from"] == c["tag_to"]
                       else f"{c['tag_from']} → {c['tag_to']}")
                L.append(f"| `{n}` | {c['from']:g} | **{c['to']:g}** | {tag} |")
        else:
            L.append("_None — every car-data number is identical to the "
                     "previous run._")
        L.append("")

        for key, title in (("added", "ADDED parameters"),
                           ("removed", "REMOVED parameters")):
            if d[key]:
                L.append(f"## {title}")
                L.append("")
                for n in sorted(d[key]):
                    e = d[key][n]
                    val = e["value"] if isinstance(e, dict) else e
                    L.append(f"- `{n}` = {val:g}")
                L.append("")

        retagged = {n: e for n, e in d["unchanged"].items() if "retagged_from" in e}
        if retagged:
            L.append("## RETAGGED (same number, new provenance)")
            L.append("")
            for n in sorted(retagged):
                e = retagged[n]
                L.append(f"- `{n}` = {e['value']:g} — {e['retagged_from']} → "
                         f"**{e['tag']}**")
            L.append("")

        if not d.get("incomparable"):
            L.append("## UNCHANGED parameters")
            L.append("")
            L.append(f"{len(d['unchanged'])} parameters are byte-identical to "
                     f"run {self.prev_id or '(n/a)'}, grouped by provenance:")
            L.append("")
            by_tag = {}
            for n, e in sorted(d["unchanged"].items()):
                by_tag.setdefault(e["tag"], []).append(f"`{n}`={e['value']:g}")
            for tag in sorted(by_tag):
                L.append(f"- **{tag}** ({len(by_tag[tag])}): "
                         + ", ".join(by_tag[tag]))
            L.append("")

        md = self.man_diff
        L.append("## Maneuver test values (scenario settings, not car data)")
        L.append("")
        if md["changed"]:
            L.append("| maneuver | knob | was | now |")
            L.append("|---|---|---|---|")
            for slug in sorted(md["changed"]):
                for k in sorted(md["changed"][slug]):
                    c = md["changed"][slug][k]
                    L.append(f"| {slug} | `{k}` | {c['from']:g} | **{c['to']:g}** |")
            L.append("")
            L.append("⚠️ Metrics are NOT comparable to the previous run's for "
                     "these maneuvers — the test itself changed, not (only) "
                     "the car.")
        elif not md["comparable"]:
            L.append("_Previous run predates maneuver-value recording — first "
                     "comparable run from here on._")
        else:
            L.append("_Unchanged — same speeds, steer angles, and torques as "
                     "the previous run._")
        for key, word in (("added", "run this time but not last time"),
                          ("removed", "run last time but not this time")):
            if md[key]:
                L.append("")
                L.append(f"Maneuvers {word}: " +
                         ", ".join(f"`{x}`" for x in md[key]))
        L.append("")

        L.append("## Code")
        L.append("")
        changed_code = [f for f in cdf["changed"] if f not in DATA_FILES]
        if changed_code or cdf["added"] or cdf["removed"]:
            for f in changed_code:
                L.append(f"- edited: `{f}`")
            for f in cdf["added"]:
                L.append(f"- new: `{f}`")
            for f in cdf["removed"]:
                L.append(f"- deleted: `{f}`")
            L.append("")
            L.append("⚠️ The model code changed as well as (or instead of) the "
                     "data — a metric difference versus the previous run is "
                     "not necessarily a vehicle difference.")
        else:
            edited_data = [f for f in cdf["changed"] if f in DATA_FILES]
            L.append("_Unchanged — same model source, byte for byte."
                     + (" (" + ", ".join(f"`{f}`" for f in edited_data)
                        + " was edited; those edits are the parameter changes "
                          "listed above.)"
                        if edited_data else "") + "_")
        L.append("")
        self._write("CHANGES.md", L)

    def _write_parameters(self):
        d = self.param_diff
        L = [f"# Parameters used by run {self.run_id}", ""]
        L.append("Every number the sim ran on, straight out of the "
                 "`params.yaml` files, with its provenance tag and whether it "
                 f"moved since run {self.prev_id or '(n/a)'}.")
        L.append("")
        L.append("| parameter | value | provenance | vs. previous run |")
        L.append("|---|---|---|---|")
        for n in sorted(self.params):
            e = self.params[n]
            if n in d["changed"]:
                status = f"**CHANGED** from {d['changed'][n]['from']:g}"
            elif n in d["added"]:
                status = "**NEW**"
            elif "retagged_from" in d["unchanged"].get(n, {}):
                status = f"same value, retagged from {d['unchanged'][n]['retagged_from']}"
            else:
                status = "same"
            L.append(f"| `{n}` | {e['value']:g} | {e['tag']} | {status} |")
        L.append("")
        L.append("Provenance tags are the `status:` field of each entry in the `params.yaml` files. "
                 "`PLACEHOLDER` means no source at all yet — results that "
                 "depend on those are provisional, and the tire block is "
                 "entirely placeholder.")
        self._write("PARAMETERS.md", L)

    def _write_metrics_csv(self):
        path = os.path.join(self.dir, "metrics.csv")
        fields = ["run_id", "label", "maneuver", "config", "metric", "value"]
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(self.metric_rows)

        # append to the across-runs file so trends are one read away
        all_path = os.path.join(self.runs_dir, "all_metrics.csv")
        new = not os.path.exists(all_path)
        with open(all_path, "a", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            if new:
                w.writeheader()
            w.writerows(self.metric_rows)

    def _write_summary(self, plot_paths, replay_paths, tables):
        d = self.param_diff
        L = [f"# Run {self.run_number:03d} — {self.label}", ""]
        L.append(f"- **when:** {self.started.strftime('%Y-%m-%d %H:%M')}")
        L.append(f"- **run id:** `{self.run_id}`")
        L.append(f"- **previous run:** `{self.prev_id or '— none'}`")
        if self.note:
            L.append(f"- **note:** {self.note}")
        L.append(f"- **car:** {cfg.mass.total:.1f} kg total (car+driver), "
                 f"{cfg.geometry.wheelbase:.2f} m wheelbase, "
                 f"tire µ₀ {cfg.tires.mu0:.2f} "
                 f"({cfg.tag_of('tires.mu0')})")
        n_ph = sum(1 for e in self.params.values() if e["tag"] == "PLACEHOLDER")
        L.append(f"- **provenance:** {n_ph} of {len(self.params)} parameters are "
                 f"still PLACEHOLDER (see `PARAMETERS.md`)")
        L.append("")

        man_ch = self.man_diff["changed"]
        if d["changed"] or man_ch:
            L.append("## Changed since the previous run")
            L.append("")
            for n in sorted(d["changed"]):
                c = d["changed"][n]
                L.append(f"- `{n}`: {c['from']:g} → **{c['to']:g}**")
            for slug in sorted(man_ch):
                for k in sorted(man_ch[slug]):
                    c = man_ch[slug][k]
                    L.append(f"- {slug} `{k}`: {c['from']:g} → **{c['to']:g}** "
                             "(test value, not car data)")
            L.append("")
            L.append(f"Everything else ({len(d['unchanged'])} car parameters) "
                     "is unchanged — see `CHANGES.md` for the full list.")
        elif self.prev_id is None:
            L.append("## Changed since the previous run")
            L.append("")
            L.append("**This is the first recorded run** — there is nothing to "
                     "compare it against. Every later run is diffed against "
                     "the one before it.")
        else:
            L.append("## Changed since the previous run")
            L.append("")
            L.append("**Nothing.** Every car-data value and every maneuver "
                     "test value matches the previous run; see `CHANGES.md` "
                     "for whether the code moved.")
        L.append("")

        esc = lambda x: str(x).replace("|", r"\|")
        if tables:
            L.append("## How to read the tables")
            L.append("")
            L.append("All configs get IDENTICAL inputs — differences are "
                     "the torque split alone. "
                     "**max \\|beta\\|** = peak sideslip (small = planted; "
                     "~57° = spun) · **dw RMSE** = wheel-speed diff vs corner "
                     "geometry (the s-diff's job) "
                     "· **max \\|kappa\\|** = worst wheel slip (above the "
                     "tire peak ≈ 0.10 = wheelspin, red wheel in the replay) "
                     "· **max \\|ay\\|** = grip actually used. Full test "
                     "catalog: `BREAKDOWN.md` §6.")
            L.append("")
        if self.maneuvers:
            L.append("## Test points")
            L.append("")
            for slug, m in self.maneuvers.items():
                if m.get("params"):
                    kv = ", ".join(f"{k} = {v:g}" for k, v in m["params"].items())
                    L.append(f"- **{slug}**: {kv}")
            L.append("")

        for slug, table in tables.items():
            L.append(f"## {table['title']}")
            L.append("")
            L.append("| config | " + " | ".join(esc(c) for c in table["cols"])
                     + " | spun? |")
            L.append("|---" * (len(table["cols"]) + 2) + "|")
            for row in table["rows"]:
                L.append("| " + " | ".join(esc(v) for v in row) + " |")
            L.append("")

        if plot_paths or replay_paths:
            L.append("## Files")
            L.append("")
            for p in replay_paths:
                L.append(f"- replay: `{os.path.relpath(p, self.dir)}`")
            for p in plot_paths:
                L.append(f"- plot: `{os.path.relpath(p, self.dir)}`")
            L.append("- data: `data/<maneuver>__<config>.csv` "
                     f"(time series at {self.csv_hz:g} Hz)")
            L.append("- full parameter set: `PARAMETERS.md` / `manifest.json`")
            L.append("")
        self._write("summary.md", L)

    def _append_index(self):
        path = os.path.join(self.runs_dir, "index.csv")
        fields = ["run_id", "timestamp", "label", "note", "params_changed",
                  "changed_names", "code_changed", "total_mass_kg", "tire_mu0",
                  "placeholders", "maneuvers", "configs_spun"]
        spun = sorted({f"{r['maneuver']}:{r['config']}" for r in self.metric_rows
                       if r["metric"].startswith("spun") and r["value"] > 0.5})
        row = {
            "run_id": self.run_id,
            "timestamp": self.started.isoformat(timespec="seconds"),
            "label": self.label,
            "note": self.note,
            "params_changed": len(self.param_diff["changed"]),
            "changed_names": " ".join(sorted(self.param_diff["changed"])),
            "code_changed": " ".join(self.code_diff["changed"]),
            "total_mass_kg": round(
                self.params.get("CAR_MASS_NO_DRIVER", {}).get("value", float("nan"))
                + self.params.get("DRIVER_MASS", {}).get("value", 0.0), 2),
            "tire_mu0": self.params.get("TIRE_MU0", {}).get("value", ""),
            "placeholders": sum(1 for e in self.params.values()
                                if e["tag"] == "PLACEHOLDER"),
            "maneuvers": " ".join(sorted(self.maneuvers)),
            "configs_spun": " ".join(spun),
        }
        new = not os.path.exists(path)
        with open(path, "a", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            if new:
                w.writeheader()
            w.writerow(row)

    def _point_latest_at_this_run(self):
        link = os.path.join(self.runs_dir, "latest")
        try:
            if os.path.islink(link) or os.path.exists(link):
                (os.unlink if os.path.islink(link) else shutil.rmtree)(link)
            os.symlink(self.run_id, link)
        except OSError:
            pass       # symlinks are a convenience, never worth failing a run

    def _write(self, name, lines):
        with open(os.path.join(self.dir, name), "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
