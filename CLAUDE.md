# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A purpose-built **software-differential (s-diff) simulator** for Spartan Racing's FSAE EV — the minimum physics needed for a left/right rear torque split to mean something, so VCU control code can be sanity-checked before it runs on the car. It is deliberately **not** the full vehicle model (that is `reference/VehicleSim`, another team's repo, vendored as a submodule): no suspension kinematics, no roll/pitch states, no motor thermal/electrical, no battery, no mechanical brakes, no driver model.

The repo is **private** and cannot be made public as-is — see `docs/PUBLISHING.md` (TTC-restricted tire data, third-party PDFs, no license).

## Commands

```bash
# first-time setup
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
git submodule update --init sil/SRE-VCU   # REQUIRED: the firmware is the controller (built automatically)

# the audit — physics (both plants) plus the firmware against its own sdiff.c;
# the check count is printed at the end (139 checks, ~2 min at the time of
# writing); exit 1 on any failure
.venv/bin/python verify.py

# the sim (defaults to all four scripted maneuvers, both firmware configs,
# on the full 17-DOF plant)
.venv/bin/python run_sim.py --no-animate          # skip video
.venv/bin/python run_sim.py --plant planar --no-animate   # the 10-state reference plant
.venv/bin/python run_sim.py --maneuver step_steer --no-animate
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

Sections: A tire model · B vertical loads · C slip kinematics · D EOM & integration · E vs bicycle model · F VCU firmware vs its own `sdiff.c` · G in-run audit (firmware in the loop) · I sensor stack · J–N the full plant (chassis · suspension & dampers · tire transients & steering · motors, battery, thermal, brakes · vs planar + firmware in the loop). D, E and J–N drive the plant through `EqualTorque` / `EqualTorqueFull`, fixtures inside `verify.py` that read the true state — closed-form answers need exact torques, and the firmware is only reachable through the noisy sensor stack. They are test scaffolding, not configurations.

## Architecture

### The parameter system is the spine — read `model/config.py`'s docstring first

**No number is ever hardcoded in Python.** Every value lives in a `params.yaml` next to the component it describes (`model/physical/*/params.yaml`, `model/sensors/*/params.yaml`, `sil/params.yaml`), is loaded and unit-converted by `model/config.py`, and is read as `cfg.<namespace>.<name>`. Consequences:

- **Enter values in the unit you measured in** (`unit: lb`) — the loader converts; the code always sees SI. Never hand-convert.
- Derived entries carry a `derived:` formula string referencing other dotted paths, evaluated in dependency order. The formula stays in the data file so the sheet and docs can show *how* a number is computed.
- Every entry carries a `status:` provenance tag from a fixed vocabulary (`MEASURED` / `TTC FIT` / `FROM REPORT` / `CURRENT CAR` / `DERIVED` / `PLACEHOLDER` / `RULES VALUE` / `TUNED (sim)` / `NUMERICAL GUARD` / `CONSTANT` / `SUSPECT`) plus `label / symbol / what / need / how / why`. `param_sheet.py` and `runlog.py` generate the team spreadsheet and per-run provenance snapshot purely from these fields — adding a parameter to the sheet is adding it to a YAML, nothing else.
- **When you change a value, update its `status`.** Controller constants are not in any YAML: the controller is the VCU firmware and its constants live in `sdiff.c`. `drivetrain.max_motor_current` mirrors the firmware's `MAX_MOTOR_CURRENT_MA` and must track it.
- Maneuver settings (speeds, steer angles, throttle) are *scenario* definitions, not car data — they live in `model/maneuvers/`, not in YAML.

### Conventions used repo-wide

- **ISO 8855 / SAE J670**: x forward, **y LEFT**, z up. Positive yaw rate = nose swings left. Positive steer = left turn.
- Wheel order is always **FL, FR, RL, RR**. Left wheels at y = +track/2.
- State vector is 10 wide, indices exported from `model/physical/vehicle.py`: `IX, IY, IPSI, IVX, IVY, IR, IWFL, IWFR, IWRL, IWRR = range(10)`, plus `IW = (IWFL, IWFR, IWRL, IWRR)` so `IW[i]` lines up with wheel index `i` everywhere. Import those constants; never index by literal.
- Slip ratio κ uses the **SAE** form `(ω·r_w − v)/v`, unbounded — say which convention you're quoting when talking to the full-model team (κ = 67 here reads ~0.985 under the textbook's bounded form).

### The plant

`model/physical/vehicle.py` — the **planar reference plant** (`run_sim.py --plant planar`): two-track, 10 states, **all four wheels driven**: `derivatives(s, delta, T_wheel)` takes four wheel torques in FL, FR, RL, RR order and a rear-drive car is simply zero on the fronts. There is no drive-layout flag in the plant. Mechanical braking is not modelled on THIS plant — the BPS commands regen only, through the motors (the full plant below has hydraulic brakes).

  A front wheel is no longer massless. The old free-roller matched ground speed instantly and made no longitudinal force; a real spin state lags its contact-patch speed, so the fronts now make force in every transient — 2·I_w/r_w² = 13.4 kg of apparent mass, 5.3% of the car. `verify.py` D1 measures exactly that against a closed-form coast-down at 0.1% (the *wrong*, two-wheel `m_eff` is 0.638% away, so the old 1% tolerance could not tell them apart). Vertical loads depend on accelerations which depend on tire forces which depend on loads; the algebraic loop is closed by **3 fixed-point iterations** per force evaluation. `model/physical/tires/tire.py` is a simplified Pacejka with load-sensitive µ and a friction **ellipse** (separate µx/µy, fitted µx/µy = 0.91 from TTC Round 9). `model/sim.py` is fixed-step RK4 at dt = 0.25 ms with zero-order-hold inputs; the small step is set by the wheel-spin time constant (~6 ms).

### The full plant — `model/physical/vehicle17.py` (default since 2026-10-22)

The DOF-expansion plan built to its ledger: **17 DOF, 80 states**, `run_sim.py`'s default (`--plant planar` gets the 10-state plant above, which stays as the verified reference). State blocks and the module that owns each one's physics, each next to its `params.yaml`:

| states | block | module |
|---|---|---|
| 13 | 6-DOF sprung body: X Y Z, u v w, quaternion, p q r | `chassis/rigid_body.py` (pure quaternion/Euler math) |
| 8 | 4 unsprung masses: height above static + rate | `suspension/suspension.py` — springs, ARBs, bump/droop stops, camber/toe kinematics, the geometric (link) vs elastic load-transfer split |
| 8 | 4 wheel angles + spins | in `vehicle17.py` |
| 6 | rack + 2 knuckles (angle + rate each) | `steering/steering.py` — column and tie-rod compliance, kingpin moment from trail and scrub |
| 4 | Bouc-Wen damper hysteresis | `dampers/damper.py` — bilinear digressive curve, α = 0 switches hysteresis off |
| 12 | κ̃, α̃, M̃z per tire | `tires/transient.py` — relaxation lengths, aligning moment, camber thrust; wraps the Magic Formula tire |
| 8 | rotor speed + shaft twist per corner | `motors/motors.py` — Kt, torque-speed ceiling scaled by bus voltage, regen limit, losses, backlash; `CommandPath` is the inverter's latency + ramp at the VCU rate (discrete, no states — the DOF doc's argument) |
| 9 + 7 | 3-zone battery: SOC, V1, V2; T_core, T_surf, T_cool | `battery/battery.py` — 2-RC cell model, pack current solved as a quadratic, BMS current/voltage/thermal window, hot-spot factor |
| 2 | brake line pressures | `brakes/brakes.py` — first-order hydraulics, P·A·µ·R·N torque. **Mechanical brakes exist on this plant** (the BPS still also requests regen through the firmware) |
| 3 | accel bias, 2 GPS speed errors | random walks advanced by the sensor suite at the VCU rate (noise is not an ODE); `sensors/gps/gps.py` |

How it is coupled (the usual 14-DOF split): horizontal equations use the total mass, vertical ones the sprung mass; suspension deflection is algebraic (body corner height − unsprung height); spring forces are `F_static + k·s` so s = 0 is static ride height and no preload is entered; tire vertical force is the carcass spring and IS the Fz the force law sees. Lateral tire force goes to the body at the contact patch, the link part (`(F_y − m_u a)·h_rc/t`, plus `m_u a h_u/t` onto the tires only) as an equal-and-opposite vertical pair — verify K2/K3/J5 check the textbook results that fall out. The sprung CG is the body origin; moment arms are from it (`a_s`, `b_s`), not from the whole-vehicle CG.

The firmware's **current** command (`ControllerDebug.I_cmd`, amps) drives the motor model inside the plant; `T` is only what the planar plant consumes. `plant_input()` builds the per-cycle input; `sim.py` is plant-agnostic (asks the model for `nstates`, `initial_state`, `derivatives`, `state_view`, `sensor_truth`, `diverged`, `EXTRA_LOG`). The sensor suite reads a `SensorTruth` from the plant, never a state index; the planar path is bit-identical to before the refactor.

**Placeholders.** 154 of 223 parameters are `PLACEHOLDER` (every one says which measurement replaces it; `param_sheet.py` lists them). Two to know about: steering compliance (`column_stiffness`, `tierod_stiffness`) costs ~1.7° of a 5° command at 0.9 g, so the full plant yaws ~35% less than the planar on the same input (verify N1 INFO); and `knuckle_damping` is set to 6 N·m·s/rad because at the first guess (1) the steering shimmied at 16 Hz through the trail/relaxation feedback loop — the sim found a real mechanism, the number is still a guess. Values the team sent on 2026-10-22 (sheet of unconfirmed car: I_xx, I_yy, unsprung masses, k_t, damper slopes) went in as placeholders tagged "team sheet (which car is unconfirmed)"; the pre-existing conflicting values (h_cg, tracks, wheelbase, tire size) were **not** changed.

**Verification** sections J–N (`verify.py`): J chassis (quaternion norm, static equilibrium to 1e-15, aero static deflection, gravity on a grade, the whole-vehicle load-transfer identity), K suspension (damper curve, link/unsprung transfer identities, the two-axle roll-angle closed form to 0.1%, bump stop, ARB monotonic), L tires/steering (relaxation 63.2%, aligning-moment sign, camber thrust, rack/knuckle settling, scrub torque-steer sign), M drivetrain/battery/thermal/brakes (torque ceiling, rotor equation with hand-summed losses to 1e-15, I·V = P, coulomb count, thermal network identities, derate, hydraulic τ, brake gain), N the full plant with every placeholder effect switched off reproduces the planar plant below the grip limit (−1.5%), firmware in the loop completes with the s-diff's cut visible in T_e, bit-exact repeatability with the bias walks. A derivative evaluation costs ~37 µs; a 5 s maneuver runs in ~3 s per config.

### The controller is the VCU firmware — there is no Python controller

`sil/vcu_sil.py`'s `make_configs(vp)` returns the two configurations every run compares, in fixed order (verify.py indexes it):

- `open diff` — `sil/build/vcu_sil_open`: the firmware with `main.c`'s `SDiff_new(TRUE)` flipped to `FALSE` at build time, so `sdiff.c` passes the driver's command through unchanged to both motors.
- `s-diff` — `sil/build/vcu_sil`: the firmware as flashed.

The firmware only sees the sensor stack: `model/sim.py` calls `update_from_sensors(sr, dt)` every `ctrl_every` physics steps (100 Hz). `sim.simulate(sensors=None)` still calls `controller.update(s, ...)` with the true state, but only `verify.py`'s `EqualTorque` fixture uses it.

The firmware's milliamp command becomes wheel torque through `motor_kt × gear_ratio` (Kt is a PLACEHOLDER), then a per-wheel **motor** clip: peak torque, and peak power at that wheel's measured speed. That clip is motor physics, not control — the firmware has no power limiting, and since `driven_wheels × motor_power_peak` equals `power_cap_total`, it also bounds the total at the 80 kW rules value (held torque overshoots by ~0.2% between VCU cycles). With `driven_wheels: 4` each rear command is mirrored onto the same-side front — the firmware commands two motors; this is a sim-side assumption and every run's summary says so.

100% pedal is the firmware's: `drivetrain.max_motor_current` (70 A) × Kt × gear per wheel — `VehicleParams.T_drive_max`. `DriverAdapter` inverts that map, so a maneuver's scripted torque is what the firmware commands; scripted torque above full pedal is capped at full pedal (verify I6 round-trips it through the firmware).

`model/sensors/suite.py` owns the single seeded RNG, so both configs see identical noise and runs are bit-exact repeatable, firmware included (verify I5). `vx` is estimated in `model/sensors/vcu/vx_estimator.py`, referring each wheel to the CG and gating on the residual spread.

### What the s-diff is

`sdiff.c` on the SRE-VCU **`S-diff`** branch (the gitlink pins `5df1baf`; `sdiff-sil` is a stale fork) — open-loop on steering angle only, in road-wheel **degrees**: past `DELTA_MAX` the inner wheel gets `1 − K_INNER` of the driver's command, the outer is untouched (`K_DERATE` = 0), split suppressed inside `DEADBAND`, slew-limited at `RATE`. `verify.py` section F reads those `#define`s from the compiled copy and checks the binary obeys them. Torque vectoring and the four-corner allocator were Python controllers and were removed on 2026-10-14 with the rest of `controllers/`; they are in git history.

### SIL — the real firmware in the loop

`make -C sil` compiles the actual SRE-VCU firmware for the host, twice (`vcu_sil` and `vcu_sil_open`, see above): XC2000 types replaced by `sil/host/host_types.h` (force-included), the TTTech IO library replaced by `sil/host/io_stubs.c`, which serves pedal/brake/steering/LV ADC channels and a fake BMS on CAN from the sim's sensor frame. **The submodule is never modified** — sources are copied to `sil/build/src` and the patches in `sil/patches/` applied there; each patch is a firmware bug the host build can't live with, and the goal is zero patches. `sil/vcu_sil.py` runs each binary as a child process, one text line per VCU cycle over stdin/stdout (protocol in `sil/host/sil_link.h`). `ensure_firmware()` runs before every `run_sim.py` and `verify.py`: it fetches an empty submodule once, rebuilds when the binaries are missing or older than their source, rebuilds clean when the source tree changes (`sil/build/.vcu_source`), and raises `FirmwareError` with the fix otherwise — `run_sim.py` exits, `verify.py` fails F0 and skips F, G, I. `$SRE_VCU_DIR` points the build at any other SRE-VCU checkout without touching the submodule. Setup steps for people: README.md, "VCU firmware (SIL) setup".

### Run artifacts

`run.sh` gates the sim on `verify.py` — **if any check fails the sim does not run.** `runlog.py` records each run under `runs/NNN__date__label/` with a full parameter snapshot (values + provenance), test points, time-series CSVs, plots, replay videos, a CHANGED/UNCHANGED diff against the previous run, and source-file hashes. Nothing is overwritten; `runs/index.csv` and `runs/all_metrics.csv` accumulate. `runs/` is gitignored, so runs referenced in the docs (007, 008) no longer exist as artifacts.

## Known defects — check before trusting results

The three `docs/problems.txt` §C defects were **fixed on 2026-09-09** (they were outside `verify.py`'s coverage, which is why they survived):

1. `model/sim.py` overwrote the closed-loop `driver(t, s)` call on the next line, so `TrackDriver` was dead code and every `--maneuver tracks` run used the drag-only fallback. **Every track and split-µ result recorded before that commit is invalid.**
2. `run_sim.py` discarded the `model_for(man, model)` result, so the split-µ test ran on a uniform-µ plant at double the compute.
3. `runlog.py` read pre-restructure parameter keys, so `total_mass_kg` was `nan` and `tire_mu0` empty in every `runs/index.csv` row.

The rad/deg divergence in the old Python s-diff port was **fixed on 2026-09-08**; so were the three stale `sdiff-sil` assumptions in the SIL host layer (SAS calibration, RL/RR CAN IDs, uncalibrated brake).

Two live findings came out of that work, both **in the firmware, neither fixed**:

1. **No safety condition reduces torque on `S-diff`.** `SafetyChecker_reduceTorque()` computes `multiplier` correctly — including `multiplier = 0` for any fault, HVIL loss, or the EV.4.7 APPS/BPS implausibility — and then discards it: the four `powertrain->motor_* = ... * multiplier` lines at the end of the function are commented out (upstream `4c164a7`, "commenting out cutting off motors", still commented at `5df1baf`). Confirmed in the SIL: at 50% APPS with the brakes at 40 bar the fault flag `F_tpsbpsImplausible` sets and both rear motors stay at full 35000 mA. This is rules-critical (EV.4.7 / EV.5.7) — and **under AWD it gets worse**, since those four commented-out lines name all four motors, so the failure now fails to cut four.
2. **`DELTA_MAX` is probably a typo.** It went `25 → 2` in `ba13d61`, a commit whose message is "refactor comments and formatting in sdiff.h for clarity". `25` ≈ the 23.2° full-lock angle and would make the derate progressive; `2` saturates it at ~10.8° of handwheel, so the s-diff is close to on/off. It survived PNR runs #1–#3. `params.yaml` mirrors the current value and tags it `SUSPECT`.

## Documentation drift

`README.md` is short and current. `docs/guide/` holds only `getting-started.md` and the generated `parameters.md`; the empty stubs were deleted. `docs/problems.txt` predates the firmware-only switch and still refers to the Python controllers and to `docs/BREAKDOWN.md`, `docs/FINDINGS.md` and `PROGRESS.md`, which no longer exist — treat its §A/§B physics notes as live and its controller references as history. This file is the live documentation.

`run.sh` no longer quotes a check count: `verify.py` prints its own, so the number cannot go stale again.
