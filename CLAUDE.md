# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A purpose-built **software-differential (s-diff) simulator** for Spartan Racing's FSAE EV — the minimum physics needed for a left/right rear torque split to mean something, so VCU control code can be sanity-checked before it runs on the car. It is deliberately **not** the full vehicle model (that is `reference/VehicleSim`, another team's repo, vendored as a submodule): no suspension kinematics, no roll/pitch states, no motor thermal/electrical, no battery, no mechanical brakes, no driver model.

The repo is **private** and cannot be made public as-is — see `docs/PUBLISHING.md` (TTC-restricted tire data, third-party PDFs, no license).

## Commands

```bash
# first-time setup
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# the physics audit — the check count and runtime are printed at the end
# (130 checks, ~3 min at the time of writing); exit 1 on any failure
.venv/bin/python verify.py

# the sim (defaults to all four scripted maneuvers, all three configs)
.venv/bin/python run_sim.py --no-animate          # skip video (~25 s/maneuver)
.venv/bin/python run_sim.py --maneuver step_steer --no-animate
.venv/bin/python run_sim.py --perfect-state --no-animate   # bypass sensors
.venv/bin/python run_sim.py --maneuver tracks --track 90deg --no-animate

# full workflow: verify → ask for test points → run → open results
./run.sh --defaults

# regenerate the team spreadsheet after changing any number
.venv/bin/python param_sheet.py

# regenerate the generated table in docs/guide/parameters.md (same trigger)
.venv/bin/python param_table.py
.venv/bin/python param_table.py --check    # exit 1 if the doc is stale

# read one value or its full provenance entry
.venv/bin/python -c "from model.config import cfg; print(cfg.tires.mu0)"
.venv/bin/python -c "import json; from model.config import cfg; print(json.dumps(cfg.meta('mass.car_no_driver'), indent=2))"

# build the real VCU firmware and run it as an extra config
git submodule update --init sil/SRE-VCU && make -C sil
.venv/bin/python run_sim.py --sil --maneuver step_steer --no-animate
```

`README.md` has the full flag list. There is no pytest suite and no linter — `verify.py` **is** the test suite.

### Keeping docs/guide/parameters.md in sync

`docs/guide/parameters.md` holds a generated table (parameter_name, file_name,
parameter_type) between `<!-- BEGIN PARAM TABLE -->` / `<!-- END PARAM TABLE -->`
markers. `param_table.py` rewrites everything between them from `cfg`; the prose
outside them is hand-written and must not be touched by the generator or by hand-
editing the rows.

**Check it on every parameter change.** Whenever `git status` / `git diff` shows a
`params.yaml` touched — including a `status:` tag edit, or a `params.yaml` added or
deleted — the table is potentially stale:

```bash
git diff --name-only HEAD -- '*params.yaml'   # did any parameter data move?
.venv/bin/python param_table.py --check       # exit 1 = stale
.venv/bin/python param_table.py               # regenerate, then read the diff
```

Never commit a `params.yaml` change with a stale table. The `param-table` skill
(`.claude/skills/param-table/SKILL.md`) walks the full check → regenerate → verify
sequence, including the two other artifacts the same edit invalidates
(`param_sheet.py`'s spreadsheet and `verify.py`).

### Running one verification section

`verify.py` has no `-k` filter or section selector; `main()` runs A–I and exits. To run a single section during development:

```bash
.venv/bin/python -c "import verify; verify.section_a()"
```

Sections: A tire model · B vertical loads · C slip kinematics · D EOM & integration · E vs bicycle model · F controller limits · G in-run audit · H VCU-rate robustness · I sensor stack.

## Architecture

### The parameter system is the spine — read `model/config.py`'s docstring first

**No number is ever hardcoded in Python.** Every value lives in a `params.yaml` next to the component it describes (`model/physical/*/params.yaml`, `model/sensors/*/params.yaml`, `controllers/python/params.yaml`, `sil/params.yaml`), is loaded and unit-converted by `model/config.py`, and is read as `cfg.<namespace>.<name>`. Consequences:

- **Enter values in the unit you measured in** (`unit: lb`) — the loader converts; the code always sees SI. Never hand-convert.
- Derived entries carry a `derived:` formula string referencing other dotted paths, evaluated in dependency order. The formula stays in the data file so the sheet and docs can show *how* a number is computed.
- Every entry carries a `status:` provenance tag from a fixed vocabulary (`MEASURED` / `TTC FIT` / `FROM REPORT` / `CURRENT CAR` / `DERIVED` / `PLACEHOLDER` / `RULES VALUE` / `TUNED (sim)` / `NUMERICAL GUARD` / `CONSTANT` / `SUSPECT`) plus `label / symbol / what / need / how / why`. `param_sheet.py` and `runlog.py` generate the team spreadsheet and per-run provenance snapshot purely from these fields — adding a parameter to the sheet is adding it to a YAML, nothing else.
- **When you change a value, update its `status`.** If vehicle or tire numbers move meaningfully, controller gains in `controllers/python/params.yaml` need retuning.
- Maneuver settings (speeds, steer angles, throttle) are *scenario* definitions, not car data — they live in `model/maneuvers/`, not in YAML.

### Conventions used repo-wide

- **ISO 8855 / SAE J670**: x forward, **y LEFT**, z up. Positive yaw rate = nose swings left. Positive steer = left turn.
- Wheel order is always **FL, FR, RL, RR**. Left wheels at y = +track/2.
- State vector is 10 wide, indices exported from `model/physical/vehicle.py`: `IX, IY, IPSI, IVX, IVY, IR, IWFL, IWFR, IWRL, IWRR = range(10)`, plus `IW = (IWFL, IWFR, IWRL, IWRR)` so `IW[i]` lines up with wheel index `i` everywhere. Import those constants; never index by literal.
- Slip ratio κ uses the **SAE** form `(ω·r_w − v)/v`, unbounded — say which convention you're quoting when talking to the full-model team (κ = 67 here reads ~0.985 under the textbook's bounded form).

### The plant

`model/physical/vehicle.py` — planar two-track, 10 states, **all four wheels driven**: `derivatives(s, delta, T_wheel)` takes four wheel torques in FL, FR, RL, RR order and a rear-drive car is simply zero on the fronts. There is no drive-layout flag in the plant. Mechanical braking is still not modelled — the BPS commands regen only, through the motors, so what AWD added is regen on four wheels rather than braking.

  A front wheel is no longer massless. The old free-roller matched ground speed instantly and made no longitudinal force; a real spin state lags its contact-patch speed, so the fronts now make force in every transient — 2·I_w/r_w² = 13.4 kg of apparent mass, 5.3% of the car. `verify.py` D1 measures exactly that against a closed-form coast-down at 0.1% (the *wrong*, two-wheel `m_eff` is 0.638% away, so the old 1% tolerance could not tell them apart). Vertical loads depend on accelerations which depend on tire forces which depend on loads; the algebraic loop is closed by **3 fixed-point iterations** per force evaluation. `model/physical/tires/tire.py` is a simplified Pacejka with load-sensitive µ and a friction **ellipse** (separate µx/µy, fitted µx/µy = 0.91 from TTC Round 9). `model/sim.py` is fixed-step RK4 at dt = 0.25 ms with zero-order-hold inputs; the small step is set by the wheel-spin time constant (~6 ms).

### Controller path — two update entry points

Both `controllers/python/torque_allocator.py` (the default) and
`controllers/python/torque_split.py` (the firmware mirror) expose:
- `update(s, delta, T_req_total, dt)` — perfect-state feedback, used by `verify.py` and `--perfect-state`.
- `update_from_sensors(sr, dt)` — **the default path.** Consumes `SensorReadings` only: wheel speeds from motor resolvers ÷ planetary, yaw rate from the filtered IMU gyro, road-wheel angle estimated through the SAS steer map, vx *estimated* from wheel speeds (no ground-speed sensor exists), torque request through the pedal map gated by the FSAE EV.4.7 APPS/BPS plausibility cut.

`model/sensors/suite.py` owns the single seeded RNG, so every config sees identical noise — a fair fight, and runs are bit-exact repeatable (checked by verify I5).

Both paths end in a limit chain — `_apply_limits()` in the two-motor mirror, `_apply_limits4()` in the allocator: per-wheel peak torque (where the **base** torque shifts so the left/right *difference* survives — thrust is sacrificed before yaw authority), regen speed cutoff, per-motor power, then the 80 kW total rules cap.

`vx` is estimated in `model/sensors/vcu/vx_estimator.py`. With four driven wheels "the slowest driven wheel is closest to the truth" collapses — every wheel over-reads at once — so it refers each wheel to the CG (`v_i = w_i·r_w + r·y_i`, which alone removed most of the error) and gates the wheel correction on the residual spread, carrying the gap on the accelerometer. Worst error over `corner_exit` is 0.042 m/s against 0.605 for the naive pick.

### What the s-diff currently is

The live s-diff is a **line-for-line Python port of `sdiff.c`** from the SRE-VCU **`S-diff`** branch (the one the gitlink pins and the SIL builds; `sdiff-sil` is a stale fork — see below) — deliberately open-loop on steering angle only, never reading wheel speeds. Same variable names as the C so the two can be diffed by eye.

**`sdiff.c` works in road-wheel DEGREES.** `controllers/python/params.yaml` carries the same numbers as the C `#define`s under `unit: deg`, so the loader hands the controller radians and `delta / delta_max` matches the C's `delta_deg / DELTA_MAX` exactly. Don't "fix" this by adding a degrees conversion in `torque_split.py`. The constants mirror the firmware — when the C changes, they change; they are not a design output of this sim. Torque vectoring was removed on 2026-09-04 and parked in `controllers/python/torque_vectoring.py`, which **nothing imports**; that file carries its own re-enable instructions. `make_configs()` now lives in `controllers/python/torque_allocator.py` and returns three: `open 4WD`, `4-corner AWD` and `s-diff (RWD ref)`. **Append, never insert** — `verify.py` indexes it positionally.

### The four-corner allocator — a design output, not a firmware mirror

`controllers/python/torque_allocator.py` is the default controller. Its constants live in `controllers/python/awd/params.yaml` under `namespace: controllers.awd`, deliberately separate from `controllers/python/params.yaml` so the `sdiff.c` mirror stays uncontaminated. Nothing in the allocator exists in the firmware; nothing in it may be tagged `CURRENT CAR`.

Three stages: axle split following the estimated normal-load split; left/right by estimated corner load with exponent `load_exponent` (0 is exactly an open diff, which is how the baseline config is built); a per-corner spin guard. Both acceleration estimates are **derived, never measured** — `ax` feed-forward from the request, `ay` from yaw rate plus a kinematic steer term — so the perfect-state and sensor paths stay structurally identical and nothing measured can feed back into the split and oscillate.

**`load_exponent` sits below a stability cliff, and the cliff moves with torque.** More torque to the loaded outer wheel makes a yaw moment into the corner; past a point that is oversteer, not vectoring. The parameter's `note:` carries the sweep with a stability column at three request levels — the objective (Δω RMSE) improves monotonically *right across* the cliff, so tuning on it alone walks straight off.

**The real limiter is the per-motor power clip, not the 80 kW cap.** The clip bites the faster (outer) wheel first, so applying it per-wheel after a difference-preserving shift inverts the split: 200/300 at ω 100/140 comes back as 173/143. The axle difference is therefore capped against the tighter of its two headrooms first (`verify.py` F14). Meanwhile `driven_wheels × motor_power_peak` equals `power_cap_total` exactly, so the rules cap is dead code and F16 says so as an info line rather than pretending to test it.

### SIL — the real firmware in the loop

`make -C sil` compiles the actual SRE-VCU firmware for the host: XC2000 types replaced by `sil/host/host_types.h` (force-included), the TTTech IO library replaced by `sil/host/io_stubs.c`, which serves pedal/brake/steering/LV ADC channels and a fake BMS on CAN from the sim's sensor frame. **The submodule is never modified** — sources are copied to `sil/build/src` and the patches in `sil/patches/` applied there; each patch is a firmware bug the host build can't live with, and the goal is zero patches. `sil/vcu_sil.py` runs the binary as a child process, one text line per VCU cycle over stdin/stdout (protocol in `sil/host/sil_link.h`), and converts the firmware's milliamp command to wheel torque via `motor_kt × gear_ratio`.

### Run artifacts

`run.sh` gates the sim on `verify.py` — **if any check fails the sim does not run.** `runlog.py` records each run under `runs/NNN__date__label/` with a full parameter snapshot (values + provenance), test points, time-series CSVs, plots, replay videos, a CHANGED/UNCHANGED diff against the previous run, and source-file hashes. Nothing is overwritten; `runs/index.csv` and `runs/all_metrics.csv` accumulate. `runs/` is gitignored, so runs referenced in the docs (007, 008) no longer exist as artifacts.

## Known defects — check before trusting results

The three `docs/problems.txt` §C defects were **fixed on 2026-09-09** (they were outside `verify.py`'s coverage, which is why they survived):

1. `model/sim.py` overwrote the closed-loop `driver(t, s)` call on the next line, so `TrackDriver` was dead code and every `--maneuver tracks` run used the drag-only fallback. **Every track and split-µ result recorded before that commit is invalid.**
2. `run_sim.py` discarded the `model_for(man, model)` result, so the split-µ test ran on a uniform-µ plant at double the compute.
3. `runlog.py` read pre-restructure parameter keys, so `total_mass_kg` was `nan` and `tire_mu0` empty in every `runs/index.csv` row.

The rad/deg divergence in `controllers/python/params.yaml` was **fixed on 2026-09-08**; so were the three stale `sdiff-sil` assumptions in the SIL host layer (SAS calibration, RL/RR CAN IDs, uncalibrated brake).

Two live findings came out of that work, both **in the firmware, neither fixed**:

1. **No safety condition reduces torque on `S-diff`.** `SafetyChecker_reduceTorque()` computes `multiplier` correctly — including `multiplier = 0` for any fault, HVIL loss, or the EV.4.7 APPS/BPS implausibility — and then discards it: the four `powertrain->motor_* = ... * multiplier` lines at the end of the function are commented out (upstream `4c164a7`, "commenting out cutting off motors", still commented at `5df1baf`). Confirmed in the SIL: at 50% APPS with the brakes at 40 bar the fault flag `F_tpsbpsImplausible` sets and both rear motors stay at full 35000 mA. This is rules-critical (EV.4.7 / EV.5.7) — and **under AWD it gets worse**, since those four commented-out lines name all four motors, so the failure now fails to cut four.
2. **`DELTA_MAX` is probably a typo.** It went `25 → 2` in `ba13d61`, a commit whose message is "refactor comments and formatting in sdiff.h for clarity". `25` ≈ the 23.2° full-lock angle and would make the derate progressive; `2` saturates it at ~10.8° of handwheel, so the s-diff is close to on/off. It survived PNR runs #1–#3. `params.yaml` mirrors the current value and tags it `SUSPECT`.

## Documentation drift

`README.md` and the `docs/guide/*` stubs are thin: **eight of the ten guide files are 0 bytes** (`controller.md`, `conventions.md`, `file-map.md`, `glossary.md`, `interop-vehiclesim.md`, `known-defects.md`, `sil.md`, `what-this-is.md`), and `docs/BREAKDOWN.md`, `docs/FINDINGS.md` and `PROGRESS.md` — all still cited by `docs/problems.txt` §A/§D — **no longer exist**. Treat this file and `docs/problems.txt` as the live documentation; filling the guide stubs is separate, unstarted work.

`run.sh` no longer quotes a check count: `verify.py` prints its own, so the number cannot go stale again.
