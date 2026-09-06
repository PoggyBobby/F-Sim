# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A purpose-built **software-differential (s-diff) simulator** for Spartan Racing's FSAE EV — the minimum physics needed for a left/right rear torque split to mean something, so VCU control code can be sanity-checked before it runs on the car. It is deliberately **not** the full vehicle model (that is `reference/VehicleSim`, another team's repo, vendored as a submodule): no suspension kinematics, no roll/pitch states, no motor thermal/electrical, no battery, no mechanical brakes, no driver model.

The repo is **private** and cannot be made public as-is — see `docs/PUBLISHING.md` (TTC-restricted tire data, third-party PDFs, no license).

## Commands

```bash
# first-time setup
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# the physics audit — 80 independent cross-checks, ~90 s, exit 1 on any failure
.venv/bin/python verify.py

# the sim (defaults to all four scripted maneuvers, both configs)
.venv/bin/python run_sim.py --no-animate          # skip video (~25 s/maneuver)
.venv/bin/python run_sim.py --maneuver step_steer --no-animate
.venv/bin/python run_sim.py --perfect-state --no-animate   # bypass sensors
.venv/bin/python run_sim.py --maneuver tracks --track 90deg --no-animate

# full workflow: verify → ask for test points → run → open results
./run.sh --defaults

# regenerate the team spreadsheet after changing any number
.venv/bin/python param_sheet.py

# read one value or its full provenance entry
.venv/bin/python -c "from model.config import cfg; print(cfg.tires.mu0)"
.venv/bin/python -c "import json; from model.config import cfg; print(json.dumps(cfg.meta('mass.car_no_driver'), indent=2))"

# build the real VCU firmware and run it as an extra config
git submodule update --init sil/SRE-VCU && make -C sil
.venv/bin/python run_sim.py --sil --maneuver step_steer --no-animate
```

`README.md` has the full flag list. There is no pytest suite and no linter — `verify.py` **is** the test suite.

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
- State vector is 8 wide, indices exported from `model/physical/vehicle.py`: `IX, IY, IPSI, IVX, IVY, IR, IWRL, IWRR = range(8)`. Import those constants; never index by literal.
- Slip ratio κ uses the **SAE** form `(ω·r_w − v)/v`, unbounded — say which convention you're quoting when talking to the full-model team (κ = 67 here reads ~0.985 under the textbook's bounded form).

### The plant

`model/physical/vehicle.py` — planar two-track, 8 states, front wheels are undriven free-rollers (which is why this sim cannot do braking or 4WD). Vertical loads depend on accelerations which depend on tire forces which depend on loads; the algebraic loop is closed by **3 fixed-point iterations** per force evaluation. `model/physical/tires/tire.py` is a simplified Pacejka with load-sensitive µ and a friction **ellipse** (separate µx/µy, fitted µx/µy = 0.91 from TTC Round 9). `model/sim.py` is fixed-step RK4 at dt = 0.25 ms with zero-order-hold inputs; the small step is set by the wheel-spin time constant (~6 ms).

### Controller path — two update entry points

`controllers/python/torque_split.py` exposes:
- `update(s, delta, T_req_total, dt)` — perfect-state feedback, used by `verify.py` and `--perfect-state`.
- `update_from_sensors(sr, dt)` — **the default path.** Consumes `SensorReadings` only: wheel speeds from motor resolvers ÷ planetary, yaw rate from the filtered IMU gyro, road-wheel angle estimated through the SAS steer map, vx *estimated* from wheel speeds (no ground-speed sensor exists), torque request through the pedal map gated by the FSAE EV.4.7 APPS/BPS plausibility cut.

`model/sensors/suite.py` owns the single seeded RNG, so every config sees identical noise — a fair fight, and runs are bit-exact repeatable (checked by verify I5).

Both paths end in `_apply_limits()`: per-wheel peak torque (where the **base** torque shifts so the left/right *difference* survives — thrust is sacrificed before yaw authority), regen speed cutoff, per-motor power, then the 80 kW total rules cap.

### What the s-diff currently is

The live s-diff is a **line-for-line Python port of `sdiff.c`** from the SRE-VCU `sdiff-sil` branch — deliberately open-loop on steering angle only, never reading wheel speeds. Same variable names as the C so the two can be diffed by eye. Torque vectoring was removed on 2026-09-04 and parked in `controllers/python/torque_vectoring.py`, which **nothing imports**; that file carries its own re-enable instructions. `make_configs()` returns two configs: `open (50/50)` and `s-diff`.

### SIL — the real firmware in the loop

`make -C sil` compiles the actual SRE-VCU firmware for the host: XC2000 types replaced by `sil/host/host_types.h` (force-included), the TTTech IO library replaced by `sil/host/io_stubs.c`, which serves pedal/brake/steering/LV ADC channels and a fake BMS on CAN from the sim's sensor frame. **The submodule is never modified** — sources are copied to `sil/build/src` and the patches in `sil/patches/` applied there; each patch is a firmware bug the host build can't live with, and the goal is zero patches. `sil/vcu_sil.py` runs the binary as a child process, one text line per VCU cycle over stdin/stdout (protocol in `sil/host/sil_link.h`), and converts the firmware's milliamp command to wheel torque via `motor_kt × gear_ratio`.

### Run artifacts

`run.sh` gates the sim on `verify.py` — **if any check fails the sim does not run.** `runlog.py` records each run under `runs/NNN__date__label/` with a full parameter snapshot (values + provenance), test points, time-series CSVs, plots, replay videos, a CHANGED/UNCHANGED diff against the previous run, and source-file hashes. Nothing is overwritten; `runs/index.csv` and `runs/all_metrics.csv` accumulate. `runs/` is gitignored, so runs referenced in the docs (007, 008) no longer exist as artifacts.

## Known live defects — check before trusting results

`docs/problems.txt` §C documents three bugs that are **still unfixed in `main`**:

1. `model/sim.py:76` unconditionally overwrites the closed-loop `driver(t, s)` call two lines above it. `TrackDriver` is dead code — every `--maneuver tracks` run uses the drag-only open-loop fallback, so all corner results are meaningless.
2. `run_sim.py:412` overwrites the `model_for(man, model)` call at line 405, discarding the split-µ plant. The split-µ test runs on a uniform-µ car, at double the compute.
3. `runlog.py:625-627` reads pre-restructure parameter keys (`CAR_MASS_NO_DRIVER`, `TIRE_MU0`) instead of dotted config paths, so `total_mass_kg` is `nan` and `tire_mu0` is empty in every `runs/index.csv` row.

Also: `DELTA_MAX` (2) and `DEADBAND` (1.0) are consumed as **radians** in both `sdiff.h` and `controllers/python/params.yaml`, but full road-wheel lock is 0.405 rad. The inner/outer split therefore never engages at any steering angle, and the s-diff reduces to a flat ~5% torque cut.

## Documentation drift

`README.md` and `docs/BREAKDOWN.md` predate the TV removal and the s-diff rewrite. They describe **four** configs (open / s-diff / TV / s-diff+TV), a PI wheel-speed s-diff with an 80 N·m `DT_SDIFF_MAX` clamp, and a TV gain table — none of which exist in the code. They also cite `car_data.py`, `controllers.py` and `sensors.py`, which the restructure replaced with `model/config.py`, `controllers/python/` and `model/sensors/`, and quote 69 or 78 verification checks where the real count is 80. `sil/vcu_sil.py`'s docstring still claims the SIL config feeds zero torque to the plant; it has fed torque since the `motor_kt` commit. Treat `docs/BREAKDOWN.md` as authoritative on **physics and parameter provenance**, and the code as authoritative on **controllers and configs**.
