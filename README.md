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
