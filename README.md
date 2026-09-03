# SJSU Spartan Racing — Software Differential + Torque Vectoring Sim

A deliberately **basic** Python simulation for developing and comparing the
**software differential (s-diff)** and **torque vectoring (TV)** logic for a
dual-rear-motor FSAE EV. It contains just enough real physics for those two
controllers to be meaningful — the full vehicle model is another team's job.

Every controller run is compared in four configurations:

| config | what it is |
|---|---|
| `open (50/50)` | equal torque to both motors — behaves exactly like an open diff, including the inner-wheel spin-up failure |
| `s-diff` | PI control of the **wheel-speed difference** to the value corner geometry requires (an ideal software LSD) |
| `TV` | PI control of the body **yaw rate** to a bicycle-model reference, actuated as a left/right torque bias |
| `s-diff + TV` | both terms summed — speeds managed by the s-diff, handling balance by TV |

## ⚠️ `car_data.py` is the master data file

**Every number the sim uses is set in `car_data.py`** and pulled from there
by the rest of the code. Each entry documents what the quantity is, the SI
unit the code expects, how to measure it, and how to enter imperial values
(`507.0 * LB`, `11.0 * INCH`, …). Masses are entered as *car without driver*
and *driver* separately — the with-driver total is computed, never entered.

Values are tagged by provenance: `FROM REPORT` (the ME295B project report's
Table 1, `1ME295B Project Report_FINALJW-2.pdf`), `DERIVED` (computed from
report values), `PLACEHOLDER` (the report doesn't provide it — still a
guess), `RULES VALUE`, and `SUSPECT` (entered as printed but inconsistent
with other report numbers — confirm before trusting).

**As of 2026-08-30 the `FROM REPORT` numbers are the working baseline, not
placeholders.** The report describes the 2023 car, but the team was told the
new car's numbers will come out very similar, so the sim now runs on them
directly; they get *verified* against the new car when it's measured rather
than replaced wholesale. Three things that "similar numbers" does **not**
settle, all flagged in `car_data.py`:

- the 241.7 kg total is read as **car + driver** (team, 2026-08-30), split
  166.7 + 75.0 in `car_data.py`. The sim only uses the sum, so this is
  bookkeeping — but it deserves a pass over the corner scales, because it
  leaves a 167 kg car where FSAE EVs run ~220–280 kg dry, and the report's
  mass rows miss by exactly the unsprung total (180.9 + 2 × 30.4 = 241.7).
  If 241.7 turns out to be car-only, the running total is ~317 kg (+31%)
  and every grip / load-transfer result moves;
- the front weight split — the report's own two CG rows disagree (58% vs
  46% front); the plausible 46% is used;
- the gear ratio — a powertrain design choice, not inherited.

Still missing entirely: aero balance, and **all tire grip data** (the
report's "tire stiffness" is the vertical spring rate, not grip — a TTC fit
is still required). The tire numbers are the ones the s-diff/TV results are
most sensitive to. When new data arrives, edit `car_data.py` **only** and
update the tags, then retune the controller gains at the bottom of that
file.

Drive layout (team-confirmed): **two independent rear motors** for the
current build — AMK Racing Kit A2370DD motors with KW26-S5 inverters (kit
manual: `AMK Racing Kit Datasheet.pdf`, motor data tagged `CURRENT CAR` in
`car_data.py`). **4WD with the same kit is the goal**; the same controller
math applies per axle (see *Extending*, below).

## Quick start

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

./run.sh                                       # THE workflow: scrutiny → sim → opens results
./run.sh -l "ttc-tires" -n "first real tire fit"
./run.sh --maneuver corner_exit --no-animate   # extra args pass through to run_sim.py
```

`run.sh` first runs the 69-check physics audit (`verify.py`); **if any check
fails the sim does not run** and the failures are printed. It then **asks
which values to test at** — speeds, steer angles, slalom frequency, corner
throttle — where Enter keeps each default (`--defaults` skips the questions
entirely). Finally it records a labeled run and opens that run's plots
(Preview), replay videos (QuickTime), and folder (Finder) automatically.

## Changing the test values (speeds, steer, throttle)

Three ways, in priority order:

1. **Answer the questions** — `./run.sh` asks for each selected maneuver's
   values before running.
2. **Flags** — every knob is a `run_sim.py` flag, e.g.
   `./run.sh --slalom-speed 18 --slalom-hz 0.8 --corner-throttle 65`
   (full list: `run_sim.py --help`).
3. **Edit the defaults** — they live in `maneuvers.py` as the factory
   arguments (`step_steer(delta_deg=5.0, vx0=15.0, T_hold=80.0)`, …). The
   prompts and flags read their defaults from there, so there is exactly one
   place the numbers live.

The chosen values are recorded in every run's `manifest.json` and
`summary.md` (*Test points*), and `CHANGES.md` diffs them against the
previous run — a run at 22 m/s can never quietly be compared against one at
15 m/s: the diff flags **"metrics not comparable — the test itself
changed."** Changing only a test value auto-labels the run accordingly
(e.g. `slalom-speed_mps-18-to-22`).

To run pieces by hand:

```bash
.venv/bin/python run_sim.py                    # sim only, defaults, nothing opened
.venv/bin/python run_sim.py --ask              # same questions as run.sh
.venv/bin/python verify.py                     # audit only
```

Each maneuver prints a metrics table (yaw-rate tracking RMSE, max sideslip,
wheel-speed-difference RMSE, max slip ratio, peak lateral g), saves two
figures — `*_response.png` (yaw rate vs reference, trajectory, a_y, sideslip)
and `*_wheels.png` (Δω tracking error, inner-wheel slip ratio, torque split,
power) — and renders a replay video. Everything lands in that run's own
folder under `runs/`.

## The controller reads sensors, not truth (since 2026-08-31)

The controller consumes the real car's sensor set (`sensors.py`), sampled at
the VCU rate (`VCU_RATE_HZ`, default 100 Hz), never the sim's perfect state:

| sensor | what the VCU gets |
|---|---|
| **APPS** | pedal % → torque request (linear map, 100% = full axle torque) |
| **BPS** | brake pressure → regen request (mechanical brakes not modeled) |
| **WSS** | = MOTOR rpm from the AMK resolver over CAN ÷ upright planetary ratio |
| **IMU** | yaw rate (noise + bias + VCU low-pass) and ax/ay |
| **SAS** | handwheel angle → road-wheel estimate through the team's steering-chart map |

Two things the VCU must live without, exactly like the real one: there is
**no vx sensor** (speed is estimated from wheel speeds — the slower rear
while driving, the faster while braking), and steering is only known through
the map. The **FSAE EV.4.7 APPS/BPS plausibility check** is implemented
(>25% throttle while braking cuts power until APPS < 5%) and demonstrated by
the `pedal_check` maneuver — watch the torque hold zero through the overlap.
Sensor noise is seeded: runs are repeatable and every config sees identical
noise. `--perfect-state` bypasses the whole stack (verify.py tests physics
that way).

## Watching it — `runs/<run>/replay/<maneuver>.mp4`

One video per maneuver, all four controller configs driving it at once from
the same start with the same scripted driver inputs, so every difference on
screen is the torque split and nothing else (`animate.py`).

- **Ground view, true scale** — body, track, wheelbase and wheel size are the
  real `car_data.py` numbers; the camera follows the pack and the trail is
  where each car has been, so configs diverging *is* the paths separating.
- **Front wheels turn** with the actual steer input.
- **A rear wheel turns red** once its slip ratio passes the tire model's
  peak-force slip — past that point more wheel speed makes *less* force, so
  the wheel is running away. That threshold is asked of the tire
  (`MagicFormulaTire.kappa_at_peak`), not hardcoded, so it moves when the
  tire data does. At the default 45% corner-exit throttle nothing reaches it;
  raise the throttle in `maneuvers.py` and the inner rear goes red.
- Side panels: whole trajectory, yaw rate vs reference, inner-rear slip, all
  with a time cursor tracking the replay.

`--no-animate` skips rendering (~25 s per maneuver); `--fps` sets the frame
rate. Without `ffmpeg` on PATH it writes a GIF instead of an MP4.

## Every run is recorded — `runs/`

A number is only worth quoting in a design review if you can say which car it
came from. So `run_sim.py` never just overwrites the last set of plots: each
run gets its own labeled folder (`runlog.py`).

```
runs/
  index.csv          one row per run: label, what changed, µ₀, mass, spins
  all_metrics.csv    every metric of every run, tidy/long — plot trends
  latest -> 003__…   symlink to the most recent run
  003__2026-09-02_1140__tire_mu0-1.4-to-1.6/
    summary.md       read first: label, note, headline tables, what changed
    CHANGES.md       CHANGED / RETAGGED / UNCHANGED vs the previous run
    PARAMETERS.md    every parameter, value, provenance tag, change mark
    manifest.json    machine-readable everything (incl. source-file hashes)
    metrics.csv      this run's metrics
    data/*.csv       time series, one file per maneuver × config
    plots/*.png
    replay/*.mp4
```

- **`-l/--label` says what the run *is*** (`ttc-tires`, `kp-sweep-hi`). Leave
  it off and the label is derived from what actually changed —
  `tire_mu0-1.4-to-1.6`, `code-change`, `repeat` — so a folder name is never
  meaningless. `-n/--note` stores free text alongside it.
- **What changed and what didn't are both stated explicitly.** `CHANGES.md`
  lists changed parameters (old → new, with provenance), parameters that kept
  their value but changed tag (`PLACEHOLDER` → `MEASURED`), and every
  unchanged parameter grouped by provenance. It also hashes the model source
  files, so "the number moved because the *code* moved" can't hide as a
  vehicle result.
- **The data is saved**, not just the pictures: `data/<maneuver>__<config>.csv`
  at 200 Hz by default (`--csv-hz`), every logged channel.

## What "optimal" means, measured — `outputs.py`

Every metrics table, `metrics.csv` and `summary.md` now also carries the
plan's **Step 4** numbers, and each run gets a `REPORT.md` ranking the four
modes on them (plus `ideal.csv`):

| number | meaning | better |
|---|---|---|
| **exit ax [g]** | mean longitudinal accel in the 1.5 s after the throttle step (found from the data: first sample where the torque request has covered 90% of its rise) | higher — the mode put more power down |
| **peak inner kappa [-]** | peak slip ratio of the *inner* rear from the throttle step on (inner = from the steer sign) | lower — past the tire peak ≈ 0.10 is wasted spin |
| **yaw RMSE [rad/s]** | tracking error against the driver's yaw-rate reference | lower |
| **inner regen [kJ]** | energy pulled *out* of the inner wheel (negative wheel power, integrated) | lower — a large number means the tune drags the inner wheel instead of capping it |

Rank = per-column ranks summed; a run that spun is ranked last regardless.
The console prints the ranking under every table.

`REPORT.md` also holds the **ideal input/output sheet** for any test that
carries corner geometry (`radius_m` in its recorded params): entry speed,
ideal steer (kinematic and with the understeer gradient) and handwheel
angle, yaw rate, per-wheel ground/wheel speeds and motor rpm, rear loads
with lateral transfer, the tire's peak slip ratio at each load, the torque
each rear can take before spinning, the resulting **throttle ceilings —
open 50/50 (2× the unloaded inner wheel) vs an ideal split (both wheels)**,
the throttle that just holds speed, and the entry-braking picture (limit
decel, regen share the BPS map can command, mechanical remainder). All of
it comes from `car_data.py` through the same containers the sim uses.

```bash
.venv/bin/python outputs.py runs/latest     # rebuild REPORT.md from a run's CSVs
```

## Is the math right? — `verify.py`

```bash
.venv/bin/python verify.py     # ~35 s, exit 0 = all checks pass
```

69 machine-checked comparisons against things independent of the sim's own
code: closed-form solutions (coast-down ODE), algebraic identities (ΣFz,
friction circle), symmetries (mirrored steer → exactly mirrored car), and
textbook models (linear bicycle model, slip-angle formulas). Run it after
any physics change. `FINDINGS.md` is the standing scrutiny report — what the
checks prove, the known gaps ranked, and what still needs real-world
validation before numbers get quoted as absolutes.

## What the physics actually is

Planar **two-track (4-wheel) model** — the minimum for left/right torque
control to exist — integrated with fixed-step RK4 at 0.25 ms:

- Body (ISO axes: x forward, y left, yaw CCW-positive):
  `m(v̇x − r·vy) = ΣFx − F_drag`, `m(v̇y + r·vx) = ΣFy`,
  `Iz·ṙ = Σ(x_i·Fy_i − y_i·Fx_i)` — the last term is where a left/right
  drive-force difference becomes a yaw moment: `Mz = (track/2)·ΔFx`.
- **Rear wheel spin states**: `I_w·ω̇ = T_wheel − r_w·Fx`. This gives real
  slip ratios, so an unloaded inner wheel genuinely spins up under power —
  the failure the s-diff exists to stop. Front wheels are undriven
  free-rollers (lateral force only).
- **Tires**: simplified Magic Formula in slip angle and slip ratio, with
  load sensitivity (grip *coefficient* falls with load — why load transfer
  costs grip) and a friction-circle cap for combined slip (`tire.py`).
- **Vertical loads**: static + aero downforce + quasi-static longitudinal
  and lateral load transfer (roll-stiffness split as a parameter); solved
  with a short fixed-point iteration each step.
- **Powertrain limits**: per-motor peak torque and power, regen speed
  cutoff, and the 80 kW FSAE EV power cap. When a wheel command saturates,
  the base torque is shifted so the left/right *difference* (the yaw
  moment) survives before total thrust does.

Deliberately **excluded** (full-model territory): suspension kinematics and
roll/pitch DOFs, rolling resistance, motor electrical/thermal dynamics,
sensor noise and state estimation (controllers get perfect state), driver
model, banking/grade.

## The controllers (`controllers.py`)

**S-diff.** A wheel at lateral offset from the CG must roll at `vx − r·y`,
so the rear pair should differ by `Δω_target = r·track_r / r_wheel`. PI on
`e = Δω_target − (ω_RR − ω_RL)` shifts torque from the wheel spinning faster
than geometry allows toward the other — permits exactly the kinematic speed
difference, fights spin beyond it.

**TV.** Reference yaw rate from the steady-state bicycle model,
`r_ref = vx·δ / (L + K_us·vx²)` with the understeer gradient computed from
the linearized tire stiffnesses, capped by friction
(`|r| ≤ 0.95·μ(g + F_downforce/m)/vx`). PI on `r_ref − r` commands a yaw
moment, converted to a split by `ΔT = 2·Mz·r_wheel/track_r`.

**Combined.** `T_RL,RR = T_request/2 ∓ (ΔT_sdiff + ΔT_tv)/2`, then limits.

## What the included maneuvers demonstrate

- **step steer** — TV configs settle on the yaw-rate reference; the open car
  understeers below it and runs a wider arc.
- **corner exit** (45% throttle by default) — the open split lets the
  unloaded inner wheel run away and degrades yaw tracking; the controlled
  configs hold it. This is the money plot.
- **slalom** — transient tracking through direction changes; same pattern.
- **A limit worth knowing** (raise corner-exit throttle past ~55% to see
  it): once the WHOLE rear axle is past its grip limit, no left/right split
  can save it — and the s-diff alone actually makes it worse, because
  shoving torque onto the loaded outer wheel burns its lateral grip and
  snaps the car into power oversteer, while the open diff "harmlessly"
  spins the light inner wheel. That regime needs traction control (out of
  scope here) — and it's why s-diff/TV must eventually be integrated with a
  slip limiter on the real car.

## Track tests — the corner matrix (`tracks.py`)

The ranking test matrix from the s-diff plan: **30°, 45°, 90°, 120° and
U-turn (180°)** corners at 2–3 radii each, plus one **split-µ** case (the
90° corner with the inner rear on a µ × 0.6 patch). Entry is at 90% of
√(µgR), the throttle **steps to full at the apex**, and the wheel unwinds
once the car's heading has swept the corner angle.

```bash
.venv/bin/python run_sim.py --maneuver tracks                    # all 13 tests
.venv/bin/python run_sim.py --maneuver tracks --track u_turn 90deg
.venv/bin/python run_sim.py --maneuver tracks --track-throttle 55  # apex step, % of peak
.venv/bin/python run_sim.py --maneuver tracks --track-radius 8 --track-entry-frac 0.8
.venv/bin/python run_sim.py --maneuver tracks --track split_mu --track-right
```

Unlike the scripted maneuvers these are driven **closed-loop** by a minimal
driver replacement: fixed kinematic steer δ = atan(L/R), a PI speed hold at
the entry speed until the apex (heading = half the corner angle), then the
throttle step — so a 90° corner is a 90° corner regardless of how fast each
config exits. Every config drives the same driver, reset per run. The
split-µ plant is built by `tracks.model_for` (scaled tire µ on the inner
rear only — the controller is never told).

What to expect on the current car data: at the plan's **100% apex step the
whole rear axle is past its grip limit on every corner from 90° up** — open,
s-diff and s-diff+TV spin, only TV survives (by yawing instead of
accelerating). That is the same finding as the corner-exit test: no
left/right split rescues an axle past its total budget, which is what the
per-wheel slip limiter (Stage 3 of the plan) is for. **55%** is the window
where the four configs separate; the 30°/45° corners separate at any
throttle. Corner angle, radius, entry speed and apex throttle are recorded in
every run's `manifest.json` / `summary.md` like any other test value.

## Extending toward the real car

1. **Real parameters + TTC tire fit** in `car_data.py`, then retune the
   controller gains (bottom of the same file).
2. **4WD**: add front wheel-speed states and torques in `vehicle.py`, run
   the same two controllers per axle plus a front/rear distribution law.
3. **Traction control**: per-wheel slip-ratio limiter on top of the split —
   TV is *not* TC (see the corner-exit TV-only run).
4. **Estimation**: replace perfect state with yaw gyro + wheel speeds + a
   vx estimator before any of this goes on the car.
5. Feed-forward TV (aero/load-based), Δω target from steering geometry
   instead of measured yaw rate, gain scheduling with speed.

## Files

| file | contents |
|---|---|
| `car_data.py` | **MASTER DATA FILE — every number, documented; the only file to edit** |
| `params.py` | parameter containers + derived quantities (pulls from `car_data.py`) |
| `tire.py` | Magic Formula + load sensitivity + friction circle |
| `vehicle.py` | two-track 3-DOF body + rear wheel spin dynamics + loads |
| `controllers.py` | s-diff, TV, combined split + torque/power limits |
| `maneuvers.py` | step steer, corner exit, slalom, pedal-check input scripts |
| `sensors.py` | APPS/BPS/WSS/IMU/SAS simulation + driver adapter + steering map |
| `sim.py` | RK4 loop, logging, metrics, comparison table |
| `verify.py` | physics verification suite — 69 independent cross-checks |
| `FINDINGS.md` | scrutiny report: what's proven, known gaps, validation ladder |
| `BREAKDOWN.md` | **full technical breakdown**: every equation, every number + provenance, and the list of numbers still needed (by owner) |
| `run.sh` | **the workflow**: verify → sim → auto-open the new run's results |
| `run_sim.py` | entry point: runs the matrix, plots, replays, records the run |
| `runlog.py` | run folders, parameter snapshots + provenance, run-to-run diffs |
| `animate.py` | the replay video (top-down ground view + live traces) |
| `style.py` | one visual system — config colors, ink, status red |
| `tire_fit.py` | TTC raw data → the nine Magic Formula coefficients (fit + plots + paste block) |
| `param_sheet.py` | regenerates `FSAE-Sim Parameters.xlsx` (team parameter sheet) from `car_data.py` — rerun after editing numbers; drag the .xlsx into Google Drive to get the Google Sheet |
| `runs/` | one folder per run: data, plots, replays, parameters, changes |
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
