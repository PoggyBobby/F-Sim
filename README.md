# FSim: Software Differential + Torque Vectoring Simulator

SJSU Spartan Racing's simulation for developing and comparing the
**software differential (s-diff)** and **torque vectoring (TV)** logic for a
dual-rear-motor FSAE EV. It models just enough real physics (a two-track
vehicle, rear wheel spin, load-sensitive tires, the powertrain limits, and
the car's actual sensor set) for those two controllers to be judged
honestly. The full vehicle model is another team's job.

Every run drives four controller configurations through the same maneuvers
with the same driver inputs, so the only difference on screen is the torque
split:

| Config | What it does |
|---|---|
| `open (50/50)` | Equal torque to both motors. Behaves like an open diff, including the inner-wheel spin-up on corner exit. |
| `s-diff` | PI control of the rear **wheel-speed difference** to what corner geometry requires (an ideal software LSD). |
| `TV` | PI control of body **yaw rate** to a bicycle-model reference, applied as a left/right torque bias. |
| `s-diff + TV` | Both terms summed: wheel speeds managed by the s-diff, handling balance by TV. |

> **Private repo.** The tire coefficients are derived from Tire Test
> Consortium (TTC) data and two third-party PDFs are included. Read
> `PUBLISHING.md` before making this public. The `ttc/` folder is
> gitignored and must never be committed.

## Setup

Requires Python 3.10 or newer (developed on 3.14). `ffmpeg` is optional:
with it the replays are MP4, without it they are GIFs.

```bash
git clone https://github.com/PoggyBobby/F-Sim.git
cd F-Sim
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

That is the whole install. Check that it works:

```bash
.venv/bin/python verify.py     # 78 physics checks, about 45 s, ends with "0 failed"
```

## Run it

```bash
./run.sh                # verify -> ask for test values -> simulate -> open results
./run.sh --defaults     # same, but skip the questions
```

`run.sh` does three things:

1. **Verifies the physics** with `verify.py`. If any check fails, the sim
   does not run and the failures are printed.
2. **Runs the simulation.** It asks which values to test at (speeds, steer
   angles, slalom frequency, corner-exit throttle); Enter keeps each
   default. Every maneuver is run in all four configs and recorded as a
   labeled run under `runs/`.
3. **Opens the results** (macOS): the plots in Preview, the replay videos
   in QuickTime, and the run folder in Finder. On other systems, open
   `runs/latest/` yourself.

Anything else you pass to `run.sh` goes straight to `run_sim.py`:

```bash
./run.sh -l "ttc-tires" -n "first real tire fit"       # label and note the run
./run.sh --maneuver corner_exit --corner-throttle 65    # one maneuver, custom value
./run.sh --defaults --no-animate                        # fastest: no replay videos
```

To run the pieces by hand:

```bash
.venv/bin/python run_sim.py           # sim only, default values, nothing opened
.venv/bin/python run_sim.py --ask     # the same questions run.sh asks
.venv/bin/python run_sim.py --help    # every flag
.venv/bin/python verify.py            # physics audit only
```

### Flags

| Flag | Meaning |
|---|---|
| `--maneuver {all,step_steer,corner_exit,slalom,pedal_check}` | Which maneuver to run (default: all) |
| `-l LABEL`, `-n NOTE` | What this run is, plus a free-text note. Without a label, one is derived from what changed (`tire_mu0-1.4-to-1.6`, `code-change`, `repeat`). |
| `--step-speed`, `--step-deg`, `--step-torque` | Step steer: speed [m/s], angle [deg], hold torque [N·m] |
| `--slalom-speed`, `--slalom-deg`, `--slalom-hz`, `--slalom-torque` | Slalom: speed, amplitude, frequency, hold torque |
| `--corner-speed`, `--corner-deg`, `--corner-throttle` | Corner exit: entry speed, steer angle, exit throttle [% of peak torque] |
| `--no-animate`, `--fps N` | Skip the replay videos, or set their frame rate |
| `--csv-hz N` | Sample rate of the saved time-series CSVs (default 200 Hz) |
| `--perfect-state` | Bypass the sensor stack so the controller reads the sim's true state |
| `--show` | Also open plot windows |

## Where the results go

Every run gets its own folder, so any number can be traced back to the car
and the test it came from. `runs/` is gitignored; everything in it is local.

```
runs/
  index.csv            one row per run: label, what changed, key numbers
  all_metrics.csv      every metric of every run, tidy/long, for trend plots
  latest -> 018__...   symlink to the most recent run
  018__2026-08-31_1319__corner-exit-corner-steer-deg-8-to-10/
    summary.md         read this first: label, note, headline tables, what changed
    CHANGES.md         CHANGED / RETAGGED / UNCHANGED parameters vs the previous run
    PARAMETERS.md      every parameter with its value and provenance tag
    manifest.json      machine-readable everything, including source-file hashes
    metrics.csv        this run's metrics
    data/*.csv         time series, one file per maneuver x config
    plots/*.png        <maneuver>_response.png and <maneuver>_wheels.png
    replay/*.mp4       one video per maneuver, all four configs at once
```

Each maneuver prints a metrics table (yaw-rate tracking RMSE, max sideslip,
wheel-speed-difference RMSE, max slip ratio, peak lateral g). The
`_response` plot shows yaw rate vs reference, trajectory, lateral
acceleration and sideslip. The `_wheels` plot shows the wheel-speed
tracking error, inner-wheel slip ratio, torque split and power.

`CHANGES.md` also diffs the test values against the previous run. A run at
22 m/s is never quietly compared with one at 15 m/s: the diff flags
"metrics not comparable, the test itself changed."

### The replay videos

All four configs drive the maneuver at once from the same start with the
same scripted inputs. The ground view is true scale (body, track, wheelbase
and wheel size are the real numbers), the camera follows the pack, and each
car leaves a trail, so diverging configs are visibly diverging paths. Front
wheels turn with the steer input. A rear wheel turns **red** once its slip
ratio passes the tire's peak-force slip, the point where more wheel speed
makes less force. Side panels show the whole trajectory, yaw rate vs
reference and inner-rear slip, with a time cursor tracking the replay.

## Changing the test values

Three ways, in priority order:

1. **Answer the questions.** `./run.sh` asks for each maneuver's values.
2. **Flags.** Every knob is a `run_sim.py` flag (table above).
3. **Edit the defaults.** They are the factory arguments in `maneuvers.py`,
   e.g. `step_steer(delta_deg=5.0, vx0=15.0, ...)`. The prompts and flags
   read their defaults from there, so the numbers live in exactly one place.

## Changing the car

**`car_data.py` is the only file to edit for car data.** Every number the
sim uses is defined there with what it is, the SI unit the code expects,
how to measure it, and how to enter imperial values (`507.0 * LB`,
`11.0 * INCH`). Masses are entered as car-without-driver and driver
separately; the total is computed.

Every value carries a provenance tag, so you always know what is real:

| Tag | Meaning |
|---|---|
| `FROM REPORT` | Table 1 of the ME295B project report (2023 car), adopted as the working baseline |
| `DERIVED` | Computed from report values, formula shown |
| `CURRENT CAR` | Reported by the team for the car being built now |
| `MEASURED <date>` | Measured on the current car. Use this tag when you replace a value |
| `TTC FIT R9` | Fitted from TTC Round 9 tire data with `tire_fit.py` |
| `TUNED (sim)` | Controller gain tuned in the sim. Retune when real data lands |
| `PLACEHOLDER` | A guess with no source yet |
| `RULES VALUE` | Fixed by the FSAE EV rulebook |
| `SUSPECT` | Entered as printed, but conflicts with other numbers. Confirm before trusting |

After editing numbers:

1. Update the tag on each value you changed, e.g. `MEASURED 2026-09-01 (corner scales)`.
2. Retune the controller gains at the bottom of `car_data.py` if vehicle or
   tire values moved meaningfully.
3. Run `./run.sh`. `verify.py` catches physics that no longer adds up, and
   the new run's `CHANGES.md` lists exactly what moved.
4. Regenerate the team parameter sheet: `.venv/bin/python param_sheet.py`
   rewrites `FSAE-Sim Parameters.xlsx` from `car_data.py`. Drag the file
   into Google Drive to get a Google Sheet.

The open data items (mass split, front weight fraction, gear ratio, aero
balance, belt-to-road tire scale) are flagged in `car_data.py` and ranked
in `FINDINGS.md`.

### Fitting tires from TTC data

`tire_fit.py` turns Calspan raw tire data into the nine Magic Formula
coefficients the sim uses and prints a ready-to-paste `car_data.py` block:

```bash
.venv/bin/python tire_fit.py --cornering ttc/run31.mat ttc/run32.mat \
    --drivebrake ttc/run72.mat --pressure 12 --out ttc/fit_hoosier_r20
```

The fitter reports raw belt values. The test belt grips harder than
asphalt, so `car_data.py` applies a road scale (`TIRE_MU_ROAD_SCALE`,
currently 0.67, to be validated on the skidpad). TTC data is restricted to
member teams: keep it in `ttc/` on local disks only.

## What the sim models

A planar two-track (four-wheel) model, the minimum for left/right torque
control to exist, integrated with fixed-step RK4 at 0.25 ms. The full
equations, every parameter with its provenance, and the test catalog are in
`BREAKDOWN.md`.

- **Body**: 3 DOF (longitudinal, lateral, yaw) in ISO axes. A left/right
  drive-force difference becomes a yaw moment through `Mz = (track/2)·ΔFx`.
- **Rear wheel spin states**, giving real slip ratios, so an unloaded inner
  wheel genuinely spins up under power. The fronts are undriven free-rollers.
- **Tires**: simplified Magic Formula in slip angle and slip ratio, load
  sensitivity, separate longitudinal and lateral peak friction, and a
  friction ellipse for combined slip. Coefficients fitted from TTC Round 9
  data for surrogate tires of the same outer diameter.
- **Vertical loads**: static, aero downforce, and quasi-static longitudinal
  and lateral load transfer with a roll-stiffness split.
- **Powertrain limits**: per-motor peak torque and power (AMK Racing Kit
  A2370DD motors with KW26-S5 inverters), regen speed cutoff, and the 80 kW
  FSAE EV cap. When a wheel command saturates, the base torque shifts so
  the left/right difference survives before total thrust does.
- **Sensors, not truth**: the controller runs at the VCU rate (100 Hz) on
  the real car's sensor set from `sensors.py`. APPS and BPS with the FSAE
  EV.4.7 plausibility check, wheel speeds from motor rpm over CAN, an IMU
  with noise, bias and low-pass filtering, and a steering-angle sensor read
  through the team's steering map. There is no vehicle-speed sensor, so
  speed is estimated from the wheel speeds. Noise is seeded: runs are
  repeatable and every config sees identical noise.

Deliberately excluded: suspension kinematics and roll/pitch DOFs, rolling
resistance, motor electrical and thermal dynamics, a driver model, banking
and grade. `FINDINGS.md` ranks the known gaps and what still needs
real-world validation before numbers are quoted as absolutes.

## The controllers

All in `controllers.py`.

- **S-diff.** A wheel offset from the CG must roll at `vx − r·y`, so the
  rear pair should differ by `Δω_target = r·track_r / r_wheel`. A PI loop on
  that error shifts torque from the wheel spinning faster than geometry
  allows toward the other one: it permits the kinematic speed difference
  and fights spin beyond it.
- **TV.** The reference yaw rate comes from the steady-state bicycle model,
  `r_ref = vx·δ / (L + K_us·vx²)`, with the understeer gradient from the
  linearized tire stiffnesses and a friction cap. A PI loop on `r_ref − r`
  commands a yaw moment, converted to a torque split by
  `ΔT = 2·Mz·r_wheel / track_r`.
- **Combined.** `T_RL,RR = T_request/2 ∓ (ΔT_sdiff + ΔT_tv)/2`, then the
  limits chain.

## What the maneuvers show

| Maneuver | What to look for |
|---|---|
| `step_steer` | TV configs settle on the yaw-rate reference. The open car understeers below it and runs a wider arc. |
| `corner_exit` | The money plot. At the default exit throttle the open split lets the unloaded inner wheel run away and degrades yaw tracking. The controlled configs hold it. |
| `slalom` | Transient tracking through direction changes. Same pattern. |
| `pedal_check` | The EV.4.7 APPS/BPS plausibility check: torque holds at zero through a throttle-plus-brake overlap until the pedal is released. |

A limit worth knowing: raise the corner-exit throttle past roughly 55% and
the whole rear axle goes past its grip limit. No left/right split can save
that, and the s-diff alone makes it worse by loading the outer wheel until
the car snaps into power oversteer. That regime needs traction control,
which is out of scope here, and it is why s-diff/TV must eventually be
integrated with a slip limiter on the real car.

## Extending toward the real car

1. Measured parameters and a fit of the actual tire in `car_data.py`, then
   retune the gains.
2. **4WD** (the goal, with the same AMK kit): add front wheel-speed states
   and torques in `vehicle.py`, run the same two controllers per axle plus a
   front/rear distribution law.
3. **Traction control**: a per-wheel slip-ratio limiter on top of the split.
   TV is not TC.
4. Feed-forward TV, a steering-geometry Δω target, and gain scheduling with
   speed.

## Project layout

| File | Contents |
|---|---|
| `run.sh` | The workflow: verify, simulate, open the new run's results |
| `run_sim.py` | Entry point: runs the maneuver x config matrix, plots, replays, records the run |
| `verify.py` | Physics verification suite, 78 independent checks |
| `car_data.py` | **Master data file. Every number, documented. The only file to edit for car data** |
| `params.py` | Parameter containers and derived quantities, pulled from `car_data.py` |
| `vehicle.py` | Two-track 3-DOF body, rear wheel spin dynamics, vertical loads |
| `tire.py` | Magic Formula, load sensitivity, friction ellipse |
| `controllers.py` | s-diff, TV, combined split, torque and power limits |
| `sensors.py` | APPS/BPS/WSS/IMU/SAS models, driver adapter, steering map |
| `maneuvers.py` | Step steer, corner exit, slalom and pedal-check input scripts, and their default values |
| `sim.py` | RK4 loop, logging, metrics, comparison table |
| `runlog.py` | Run folders, parameter snapshots with provenance, run-to-run diffs |
| `animate.py` | The replay video: top-down ground view plus live traces |
| `style.py` | One visual system: config colors, ink, status red |
| `tire_fit.py` | TTC raw data to the nine Magic Formula coefficients |
| `param_sheet.py` | Regenerates `FSAE-Sim Parameters.xlsx` from `car_data.py` |
| `BREAKDOWN.md` | Full technical breakdown: every equation, every number with provenance, the test catalog |
| `FINDINGS.md` | Scrutiny report: what is proven, known gaps ranked, validation ladder |
| `PUBLISHING.md` | Checklist before this repo can go public |
| `runs/` | One folder per run (local only, gitignored) |
