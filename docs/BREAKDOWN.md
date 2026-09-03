# FSAE-Sim — Full Technical Breakdown

*Spartan Racing software-differential / torque-vectoring sim · 2026-08-30*
*Companion documents: `FINDINGS.md` (audit results), `README.md` (how to run).*

This is the no-BS version: every equation the sim solves, every number it
uses with where that number came from, every modeling choice and what it
costs, and at the bottom the list of numbers still needed and who owns them.

---

## 1. What this sim is — and is not

**Is:** the minimum physics for left/right torque control to mean anything.
A planar (top-view) 4-wheel vehicle with individual tire loads and rear
wheel-spin states, driven by scripted maneuvers, comparing four controller
configurations under identical inputs: open (50/50) · s-diff · TV ·
s-diff + TV.

**Is not:** the full vehicle model (another sub-team owns that). Deliberately
excluded: suspension kinematics, roll/pitch as states, ride, rolling
resistance, motor electrical/thermal behavior, battery, driver model,
banking/grade, mechanical brakes (BPS commands regen only).

**Sensors (added 2026-08-31):** the controller reads the real sensor set —
APPS, BPS, WSS (= motor rpm ÷ planetary), IMU (noisy, filtered), SAS
(through the steering-chart map) — at the VCU rate, with vx *estimated*
from wheel speeds, and the EV.4.7 APPS/BPS plausibility cut implemented.
Perfect-state feedback remains available (`--perfect-state`) and is what
the physics checks use.

**Trust level (updated 2026-08-31):** the *math* is machine-verified — 78
independent cross-checks (`verify.py`); zero errors found. The *tires* are
now **TTC Round 9 fits** (real Calspan data, R20 compound) via size
surrogates — the last wholesale guess is gone. Absolute numbers are now
*provisional* rather than fictional: the remaining unknowns are the
belt→asphalt scale (×0.67 placeholder — skidpad measures it), the assumed
12 psi, and the size-surrogate approximation. Still zero on-car validation.

---

## 2. Conventions (ISO 8855 — say this up front in any discussion)

- Axes: **x forward, y LEFT, z up**. Positive yaw rate r = nose swings left
  (counter-clockwise from above). Positive steer δ = left turn.
- Wheel order everywhere: **FL, FR, RL, RR**. Left wheels at y = +track/2.
- All internal units SI: kg, m, s, N, N·m, W, rad. `car_data.py` provides
  `LB / INCH / LBFT / MPH / RPM / DEG` constants so imperial values are
  entered as e.g. `507.0 * LB`, never hand-converted.

---

## 3. The vehicle model (`vehicle.py`, `tire.py`)

### 3.1 States (8)

| state | meaning |
|---|---|
| X, Y, ψ | position and heading in the ground frame (for trajectories/replay) |
| vx, vy | body-frame velocities |
| r | yaw rate |
| ω_RL, ω_RR | **rear wheel spin speeds** — these make wheelspin real |

Front wheels are undriven free-rollers: lateral force only, no spin state.
That's why this sim can't do braking or 4WD yet (extension notes in README).

### 3.2 Rigid-body equations of motion (body frame)

```
m (v̇x − r·vy) = ΣFx − F_drag          F_drag = ½ρ·CdA·vx²
m (v̇y + r·vx) = ΣFy
Iz · ṙ        = ΣMz = Σᵢ (xᵢ·Fy,ᵢ − yᵢ·Fx,ᵢ)
```

The yaw equation's last line is the whole point of the project: a left/right
drive-force difference ΔFx at the rear axle makes a yaw moment
**Mz = (track_r/2)·ΔFx** — steering the car with the motors.

### 3.3 Vertical loads on each tire

Per axle (moment balance about the contact patches):

```
Fz_front_axle = m(g·b − ax·h)/L + aero_balance·downforce
Fz_rear_axle  = m(g·a + ax·h)/L + (1−aero_balance)·downforce
downforce     = ½ρ·ClA·vx²
```

where a, b are CG-to-front/rear-axle distances (b = weight_frac_front · L),
h is CG height, ax the longitudinal specific force. Lateral transfer moves
load `ΔF = frac·m·ay·h/track` from inner to outer wheels per axle, split
front/rear by the roll-stiffness fraction. Wheels clamp at 0 N (a wheel in
the air carries nothing, never negative).

Known simplifications: drag is treated as acting at CG height (no
center-of-pressure height — ~2% axle-load error, FINDINGS #6); the
roll-stiffness split is spring-rate ratio only (no ARBs, motion ratios, roll
centers — FINDINGS list).

Loads depend on accelerations which depend on tire forces which depend on
loads — a small algebraic loop, solved by 3 fixed-point iterations per
evaluation. Verified: ΣFz = m·g + downforce holds to 3×10⁻¹⁶ everywhere.

### 3.4 Tires — simplified Pacejka "Magic Formula"

Definitions your tire person will use:
- **Slip angle α**: angle between where the wheel points and where its
  contact patch actually travels. Generates lateral force.
- **Slip ratio κ**: (ω·r_wheel − v)/v — how much faster the wheel surface
  moves than the ground. Generates drive/brake force. κ > 0 = wheelspin.

Pure-slip force, same form both directions:

```
F(s) = D·sin( C·atan( B·s − E·(B·s − atan(B·s)) ) )

D = µ(Fz)·Fz                      peak force
B = c / (C·µ(Fz))                 chosen so small-slip stiffness = c·Fz exactly
µ(Fz) = µ₀·(1 − s_µ·(Fz/Fz_nom − 1))    load sensitivity
```

**Load sensitivity is the physics that makes torque placement matter**: the
grip *coefficient* falls as load rises, so load transfer always costs total
grip, and the outer tire — though more loaded — is less efficient per newton.

Combined slip: a friction-circle cap — if √(Fx²+Fy²) > µFz, both scale down
onto the circle. (Real combined behavior is subtler; the TTC fit replaces
this in the full team model.)

**Fitted from TTC Round 9 (2026-08-31):** all nine coefficients now come
from Calspan data for the R20 compound (57k lateral + 26k longitudinal
samples, RMS 87/199 N), pooled across two size surrogates of our untested
18.0×7.5-10. The fitted curve peaks at 9.8° slip — inside the real-slick
band. The data also forced a model upgrade: the R20's longitudinal peak is
~9% below lateral, so the tire now carries **separate µx/µy and a friction
ELLIPSE** instead of a circle. Peak µ is belt-scaled ×0.67 for asphalt
(placeholder until skidpad); validity |α| ≤ 12° (the R20 plateaus at the
sweep edge — beyond is extrapolated plateau), camber ≈ 0.

### 3.5 Rear wheel spin dynamics

```
I_wheel · ω̇ = T_wheel − r_wheel · Fx
```

Drive torque spins the wheel up; the tire's reaction torque holds it back.
When the tire can't react what the motor sends (unloaded inner wheel), ω
runs away — that IS wheelspin, and it's the failure the s-diff exists to
stop. I_wheel includes the motor rotor's inertia multiplied by gear ratio²
(≈ 0.05 of the 0.35 kg·m² total) — reflected inertia matters.

### 3.6 Steering geometry — parallel vs Ackermann (current status)

In a corner the inner front wheel tracks a tighter radius than the outer.
**100% Ackermann** geometry steers the wheels to their own radii:

```
R = L/tan(δ)      δ_inner = atan(L/(R − t_f/2))     δ_outer = atan(L/(R + t_f/2))
```

**Parallel steer** gives both wheels the same angle. Race cars deliberately
run somewhere between (even anti-Ackermann), because the loaded outer tire
wants the larger slip angle.

- The sim currently runs **parallel** (`ACKERMANN_FRACTION = 0`), with a
  verified blend function ready (`vehicle.front_steer_angles`): 0 = parallel,
  1 = full geometric, negative = anti.
- Sized: irrelevant below ~10° steer (±0.2° split at 5°); **±4° per wheel at
  the 23° full lock**.
- Sensitivity runs at the team's real envelope (23° / 40 mph): parallel
  (run 007) vs full Ackermann (run 008) **flips a result** — s-diff-only
  survives the hairpin with parallel steer and spins with full Ackermann
  (more front bite → more rotation → power-oversteer). The real steering
  curve decides which is true.
- The team's chart gives the left wheel only:
  `LWheel(deg) = −0.0797 + 0.31x − 8.44E-04·x²`, x almost certainly the
  steering-wheel angle in deg (hits the stated 23° max at x ≈ 105°).
  **The RWheel curve from the same chart is needed** — the L−R difference
  IS the Ackermann, and it drops straight into the ready hook.

### 3.7 Numerical integration

Classic RK4, fixed step **dt = 0.25 ms**, inputs held constant within a step
(zero-order hold). Why so small: the stiffest dynamics are the wheel-spin
states (time constant ≈ I_w·v/(c_κ·Fz·r_w²) ≈ 6 ms) and RK4 wants several
steps per time constant. Verified converged: halving dt twice moves the
answer by < 10⁻³. The controller also updates every physics step (4 kHz);
verified the tune survives a realistic 100 Hz VCU rate unchanged.

---

## 4. The controllers (`controllers.py`)

With two independent rear motors there is no mechanical differential — the
"diff" is whatever the software decides. Everything reduces to:

```
T_RL = T_base − dT/2         T_RR = T_base + dT/2
dT   = dT_sdiff + dT_tv
```

### 4.1 Open (50/50) — the baseline

dT = 0. Exactly reproduces an open differential, including its failure mode:
the unloaded inner wheel is free to spin up and burn its grip.

### 4.2 Software differential (s-diff) — regulates wheel SPEEDS

Corner geometry demands the rear wheels differ by

```
Δω_target = ω_RR − ω_RL = r · track_r / r_wheel
```

(outer wheel travels a longer path). A PI controller on the error
`e = Δω_target − Δω_actual` shifts torque from the wheel spinning faster
than geometry allows to the other one — an ideal limited-slip: it *permits*
the kinematic speed difference and *fights* spin beyond it.

Critical clamp: **|dT_sdiff| ≤ 80 N·m** (`DT_SDIFF_MAX`). Without it, near
the rear axle's total grip limit the PI dumps torque onto the loaded outer
tire, burns its lateral grip through the friction circle, and
power-oversteers the car — the sim demonstrates the spin on demand if you
raise the clamp. A speed controller must not be allowed large torque moves.

### 4.3 Torque vectoring (TV) — regulates the body YAW RATE

Reference from the steady-state single-track (bicycle) model of what the
driver's inputs imply:

```
r_ref = vx·δ / (L + K_us·vx²)
K_us  = (m/L)·(b/C_f − a/C_r)          understeer gradient [s²/m]
C_axle = c_α · Fz_axle                  linearized axle cornering stiffness
```

capped by friction: |r_ref| ≤ 0.95·µ(g + downforce/m)/vx. PI on
`e = r_ref − r` commands a yaw moment Mz, converted to a torque split by

```
dT_tv = 2·Mz·r_wheel / track_r
```

(exact round trip, verified). Current K_us ≈ +4.3×10⁻⁴ s²/m — mildly
understeering on placeholder tires.

**Known design gap (found by the envelope run):** the reference is yaw-rate
only. At full lock beyond grip, TV "achieves" the capped yaw target by
carrying 22° of body sideslip — a drift. Production TV pairs the yaw term
with a sideslip (β) term or limiter. On the fix-later list (FINDINGS 8-bis).

### 4.4 The limits chain (applied in this order, every update)

1. **dT pre-clip** to the physically producible range.
2. **Per-wheel peak torque** ±273 N·m (21 N·m motor × 13:1). If one wheel
   would exceed it, the BASE torque shifts so the *difference* (yaw moment)
   survives — thrust is sacrificed before yaw authority. Quirk: at the
   *lower* bound this shift *adds* thrust instead (FINDINGS #2).
3. **Regen floor**: no negative torque below 1.4 m/s (rules).
4. **Per-motor power** 30 kW, **total 80 kW** (rules EV.4.2) — note these
   clip each side independently and can quietly shrink the yaw split at
   high speed (FINDINGS #3); they never engage below ~22 kW in the current
   maneuvers. Cap is applied to *mechanical* power — no drivetrain
   efficiency modeled yet (FINDINGS #4).

### 4.5 s-diff + TV

The two dT terms simply sum. They are complementary — s-diff manages
*traction* (wheel speeds), TV manages *handling balance* (yaw) — and at
corner exit both push torque away from the unloaded inner wheel. Sim
finding: TV is the most spin-robust single system (yaw feedback backs off
the bias); s-diff-only is the most spin-prone near the limit.

---

## 5. Every number the sim uses (45 parameters, `car_data.py`)

Provenance tags: **FROM REPORT** = 2023-car value from the ME295B report,
adopted as working baseline on the team's "new car will be very similar"
call · **CURRENT CAR** = confirmed for the car being built · **DERIVED** =
computed, formula shown in `car_data.py` · **PLACEHOLDER** = a guess, no
source · **RULES VALUE** · **TUNED** = controller gain tuned in-sim.

### Mass & geometry

| value | number | tag | notes |
|---|---|---|---|
| Car mass, no driver | 181.5 kg | **MEASURED** | corner scales 2026-08-31 (Intercomp SW500): 87.5/86.3/114.8/111.5 lb = 400.1 lb |
| Driver mass | 70.8 kg | CURRENT CAR | team: "156 lb" — confirm heaviest driver + suited |
| → total (car+driver) | 252.2 kg | computed | runs ≤010 used the report-derived 241.7 — not comparable across that line |
| Wheelbase | 1.52 m | FROM REPORT | |
| Track F / R | 1.34 / 1.34 m | FROM REPORT | |
| Front weight fraction | 0.4344 | **MEASURED** (no driver) | scales 2026-08-31; with driver seated expect ~1–2% lower — redo seated |
| CG height | 0.23 m | FROM REPORT | sprung ≈ whole-vehicle here |
| Yaw inertia I_z | 70.38 kg·m² | FROM REPORT | |

### Wheels & powertrain

| value | number | tag | notes |
|---|---|---|---|
| Tire radius | 0.2286 m | CURRENT CAR | 18" OD on 10" rim; loaded radius a few mm less — measure |
| Gear ratio | 13:1 | FROM REPORT | design choice, NOT inherited — confirm |
| Motor peak torque | 21 N·m | CURRENT CAR | AMK A2370DD kit curves → 273 N·m/wheel |
| Pack voltage | 380 V | CURRENT CAR | design decision 2026-08-30; sizes 4WD at exactly the 80 kW cap |
| Motor peak power | 20 kW | DERIVED | kit curves scaled to 380 VDC (19.8/20.3/18.1 kW by three methods) — 2-motor build tops out at ~40 kW |
| Motor max speed | 20,000 rpm | CURRENT CAR | ≈ 36 m/s ground speed — not the limiter |
| Wheel spin inertia | 0.35 kg·m² | DERIVED | 0.30 guess + rotor J×13² ≈ 0.05 — replace with CAD; report's 3.033 rejected (~10× too big) |
| Power cap / regen cutoff | 80 kW / 1.4 m/s | RULES | mechanical simplification, see 4.4 |

### Aero & load transfer

| value | number | tag | notes |
|---|---|---|---|
| C_L / C_D / area | 3.18 / 1.36 / 1.106 m² | FROM REPORT | old car's aero package |
| → ClA / CdA | 3.52 / 1.50 m² | DERIVED | |
| Aero balance front | 0.45 | **PLACEHOLDER** | pure guess — CFD center of pressure needed |
| Lateral-transfer front frac | 0.438 | FROM REPORT | spring-ratio only; crude |

### Tires — TTC ROUND 9 FIT (R20 surrogates, 2026-08-31)

| value | number | notes |
|---|---|---|
| µ₀ lateral (road) | 1.74 | belt 2.593 × 0.67 road scale (scale = placeholder, skidpad validates) |
| µ₀ longitudinal (road) | 1.58 | belt 2.359 × 0.67 — µx/µy = 0.91 fitted → friction ELLIPSE model |
| Load sensitivity s_µ | 0.112 | shared by both directions |
| Nominal load Fz_nom | 619 N | = m·g/4, follows the mass entries |
| Cornering stiffness c_α | 37.0 /rad | same tire all round → linear K_us exactly 0 (neutral) |
| Longitudinal stiffness c_κ | 41.9 | |
| Shape C lat / long | 1.397 / 1.705 | fitted; lateral peak lands at 9.8° slip |
| Curvature E lat / long | 0.365 / 0.389 | fitted |

### Steering

| value | number | tag | notes |
|---|---|---|---|
| Ackermann fraction | 0.0 (parallel) | PLACEHOLDER | machinery ready; LWheel curve on file; **RWheel curve needed** |
| Max road-wheel angle | 23° | CURRENT CAR (team) | envelope, used in run 007/008 |

### Controller gains (all TUNED in-sim 2026-08-29 — retune when real data lands)

| gain | value | | gain | value |
|---|---|---|---|---|
| s-diff Kp / Ki | 15 / 60 | | TV Kp / Ki | 2500 / 1000 |
| s-diff integral clamp | 60 N·m | | TV integral clamp | 200 N·m |
| **s-diff dT clamp** | **80 N·m** | | Mz clamp | 600 N·m |
| | | | ay fraction (ref cap) | 0.95 |

(Plus V_EPS = 0.5 m/s, a numerical guard in slip denominators — below
~0.5 m/s wheel-slip dynamics aren't trustworthy; launches are out of scope.)

---

## 6. The test catalog — what each maneuver measures, how, and the numbers we want

Scripted open-loop inputs — no driver model — so runs are exactly repeatable
across configs: **all four controllers get the identical input, so any
difference in the table is the torque split and nothing else.** Test points
are adjustable per run (the script asks, or flags) and recorded; changing
one flags "metrics not comparable" in the run diff.

### First: how to read the metrics table (the five numbers every test reports)

| column | what it measures | what good looks like |
|---|---|---|
| **yaw RMSE [rad/s]** | how faithfully the car's rotation tracked what the steering asked for (the yaw-rate reference) | lower = more precise handling. TV exists to minimize this — expect TV configs to win it |
| **max \|β\| [deg]** | peak body sideslip — how sideways the car got | small = planted. Watch for TV winning yaw *while* β grows: that's a drift, not control (found at full lock). 57° = spun, flagged |
| **dw RMSE [rad/s]** | how well the rear wheel-speed difference matched corner geometry — the s-diff's own objective | s-diff should win it. ⚠ TV *deliberately* forces Δω off-kinematic to make yaw moment — its higher number here is the design working, not failing |
| **max \|κ\| [-]** | worst wheel slip ratio — the wheelspin detector | below the tire's fitted peak (κ ≈ 0.10) = working; above = spinning. The replay paints that wheel red |
| **max \|ay\| [g]** | peak lateral grip actually used | compare to the theoretical cap µ·(1 + downforce/W) ≈ 1.8+ g at speed — how close each config dares to run to the limit |

### Step steer — the yaw-response test
- **Measures:** how the car answers a sudden steering input — rise time,
  overshoot, steady-state accuracy. The classic handling-precision test and
  THE test for tuning TV gains (Kp sets response speed, Ki kills the
  steady-state error).
- **How:** straight running at constant speed, then a fast (~0.1 s) ramp to
  a fixed steer angle, held; drive torque holds speed. Default 5° @ 15 m/s.
- **Numbers we want:** steady-state yaw rate ON the reference (yaw RMSE —
  run 015: TV 0.011 vs open 0.034 rad/s, a 3× precision gain); no
  overshoot/oscillation in the yaw trace (that's the gain-tuning signal —
  oscillation = gains too hot); β under ~1° at this test point.

### Corner exit (power on) — the traction test, the s-diff's reason to exist
- **Measures:** can the config put power down mid-corner without lighting up
  the unloaded inner wheel or spinning the car.
- **How:** turn in, hold a steady corner, then ramp throttle while unwinding
  the wheel — the moment the inner rear is lightest. Default 8° @ 10 m/s,
  throttle to 45% of peak.
- **Numbers we want:** **max κ on the inner rear** — the headline. Open diff
  lets it spin; the s-diff must contain it below the κ ≈ 0.10 peak (at the
  23° envelope test: open hit κ 67, s-diff held 0.07 — that one number IS
  the s-diff's justification). Plus: no spin flag, yaw still tracked while
  power goes down, dw RMSE low for s-diff configs.
- ⚠ **Test-point map on the real tires** (runs 015/017): **45%** throttle —
  too easy, every config clean (open κ 0.029). **~55%** — the
  config-separating window; use it for the showcase comparison. **65%** —
  past the rear axle's TOTAL grip: open, s-diff, and s-diff+TV all spin,
  only TV-only survives (β 29°). No left/right split can save an axle past
  its whole budget — that run is the cleanest argument for why TRACTION
  CONTROL follows the s-diff on the roadmap (deliberately out of scope
  here).

### Slalom (sine steer) — the transient-handling test
- **Measures:** behavior through repeated direction changes: phase lag
  between steering and response, left/right symmetry, and whether errors
  GROW cycle to cycle (the spin precursor — at the full-lock slalom, open
  and s-diff-only diverged exactly this way).
- **How:** sine-wave steering, amplitude faded in, constant drive torque.
  Default ±4° @ 0.5 Hz @ 15 m/s.
- **Numbers we want:** yaw RMSE through the reversals (transient tracking);
  β bounded and symmetric left/right; cycle-to-cycle amplitude NOT growing;
  for s-diff, Δω flipping sign cleanly with each direction change.

### Pedal check (APPS/BPS) — the rules-compliance test (pass/fail, not comparative)
- **Measures:** the pedal→torque chain and the FSAE **EV.4.7** APPS/BPS
  plausibility cut. Nothing about handling — it runs straight.
- **How:** throttle to 60% APPS → brake pressure applied WHILE still on
  throttle (the illegal overlap) → both released → brake alone (regen).
- **Numbers we want:** wheel torque **exactly 0** through the whole overlap
  window (run 015: 0.000 ✓); the cut stays latched until APPS < 5%; regen
  torque negative and inside T_REGEN_MAX; vx estimate tracking through it.

### The real envelope (team, 2026-08-30)
**Max steer 23°, endurance top speed ~40 mph (17.9 m/s).** At 40 mph the
friction cap saturates the yaw reference at ~5° of steer — full lock at top
speed is a robustness test, not an operating point (and it found the
TV-drifts-when-saturated design gap). Still missing: a real minimum-corner
speed for corner-exit entry.

---

## 7. How results are produced and recorded

`./run.sh` = the workflow: ① 72-check physics audit — **sim refuses to run
if any check fails** ② asks for test values ③ records the run under
`runs/NNN__date__label/` (full parameter snapshot with provenance, test
points, time-series CSVs, plots, replay videos, CHANGED/UNCHANGED diff vs
the previous run, source-file hashes) ④ opens the results. Nothing is ever
overwritten; `runs/index.csv` and `all_metrics.csv` accumulate history.

---

## 8. Verification status (details: `FINDINGS.md`)

**Proven** (independent cross-checks, all passing): tire stiffness/peak/
friction-circle exact; ΣFz identity to machine precision; slip kinematics
match textbook formulas; coast-down matches the closed-form drag ODE incl.
wheel rotational KE; mirrored steer → exactly mirrored car (no sign errors
anywhere); two-track reproduces the linear bicycle model to 0.1%; all
limits enforced in-run; RK4 converged; gains stable at 100 Hz; Ackermann
blend matches atan geometry exactly.

**Not established:** any agreement with the physical car. Validation ladder:
TTC tires → corner scales → skidpad/accel anchor → logged laps vs replay.

---

## 9. THE LIST — numbers to finalize, who owns them, what each unlocks

Priority order. "Impact" = what's wrong with the sim until it lands.

### ✅ 1. Tire data — FITTED from TTC Round 9 (2026-08-31)
Done: pooled surrogate fit (16×7.5-10 @8" + 18.0×6.0-10 @7", both R20 —
our exact 18.0×7.5-10 was never TTC-tested), 57k lateral + 26k
longitudinal samples, RMS 87/199 N, entered in `car_data.py` tagged
`TTC FIT R9`; friction-ellipse model added (µx/µy = 0.91). Run 015 is the
first real-tire baseline; gains verified stable, 78 checks pass.
**Still open from this item:** confirm running pressure (12 psi assumed)
and rim width; skidpad measurement of the belt→road scale (×0.67
placeholder — it multiplies every grip number); size-surrogate caveat when
quoting (surrogates agreed ±6% on µ, ±1% on stiffness — that bounds it).

### 🔴 2. Masses & weight split — corner scales *(team, ~30 min with the car)*
**RESOLVED 2026-08-31 (corner scales, Intercomp SW500, no driver):**
LF 87.5 / RF 86.3 / LR 114.8 / RR 111.5 lb = 400.1 lb; front 43.44%; left
50.56% (sim assumes symmetric — 1 kg asymmetry is noise); cross 50.26%
(setup number, unused here). Car mass and front fraction are now MEASURED.
Last step: **repeat the weighing with the driver seated** — the seated
driver pulls the front share down ~1–2% (est. 41.5–43%), and confirm 156 lb
is the heaviest driver, suited.

### 🟠 3. RWheel steering curve *(steering/suspension — same chart the LWheel curve came from)*
The other polynomial (or 3–4 point triples of handwheel°, LWheel°,
RWheel°). L−R difference = the car's real Ackermann; drops into a ready,
verified hook. Proven to matter: parallel vs full Ackermann flips whether
s-diff-only survives the 23° hairpin (runs 007 vs 008). Also confirm the
chart's x-axis is steering-wheel degrees and the max handwheel angle.

### 🟠 4. Upright planetary ratio *(powertrain)*
One motor per wheel still has a reduction: the planetary gear stage packaged
in each upright (the 21 N·m / 20,000 rpm A2370DD cannot drive a wheel
directly — 1:1 would give 0.08 g and a 479 m/s speed match). Ask: "what is
the reduction ratio of the planetary in our uprights?" (AMK-kit cars
typically run 12–14.5:1; the sim assumes the old car's 13:1). Scales peak
wheel torque (×gr) and reflected rotor inertia (×gr²) — both matter to the
s-diff. Drive layout is CONFIRMED (2026-08-30): one motor per wheel, four
fitted, the two REARS active — the sim's rear-drive model matches the car.

### ✅ 5. Pack voltage — RESOLVED (380 V, 2026-08-30)
Motor peak power set to 20 kW/motor (kit curves scaled to 380 VDC; three
methods agree within ±1 kW). Note the design coherence: 4 × 20 kW = 80 kW =
the rules cap exactly; the current 2-rear build tops out at ~40 kW. Still
open from this item: the exact motor sheet "Motor_data_sheet_A2370DD" for
rotor inertia (the ~0.05 kg·m² reflected term is a typical value), and a
curve-read or dyno confirmation of the 20 kW once the pack exists.

### 🟡 6. Aero: current-car ClA, CdA, aero balance, CP height *(aero/CFD)*
Coefficients are the old car's; balance (0.45) is a pure guess. Balance
moves front/rear grip with speed — it shapes high-speed handling directly.
CP height fixes the drag-at-CG simplification.

### 🟡 7. Wheel/drivetrain inertia from CAD *(design/CAD owner)*
Tire+rim+hub+rotor spin inertia (current 0.30 is a guess; controller gains
are sensitive to it — retune whenever it changes).

### 🟡 8. Roll-stiffness split *(suspension)*
Real front/rear roll rates including ARBs and motion ratios (current 0.438
is bare spring-rate ratio). Sets which axle eats lateral load transfer —
directly trades understeer/oversteer at the limit.

### 🟡 9. Envelope: minimum corner speed *(driver/track data)*
For a realistic corner-exit entry speed (10 m/s is a guess). Plus loaded
tire radius (measure axle-center height, car on ground).

### ⚪ 10. VCU & sensor specs *(software/electronics — the stack is BUILT, the specs are placeholders)*
The sensor stack exists and is the default path (2026-08-31). Now needed to
make its numbers real: actual VCU loop rate (100 Hz assumed), the chosen
IMU's gyro noise/bias spec, SAS resolution, APPS/BPS sensor specs and brake
line pressures (what bar = braking), max regen torque from the battery
charge limit, CAN latency and inverter torque-ramp limits (still
unmodeled), drivetrain efficiency, and current-year rulebook confirmation
of 80 kW / regen cutoff / EV.4.7 thresholds.

**When any of these land:** edit `car_data.py` only (units + provenance tag),
run `./run.sh` — the audit re-verifies, the run diff labels exactly what
changed, and the gains at the bottom of `car_data.py` get retuned if
vehicle/tire numbers moved meaningfully.
