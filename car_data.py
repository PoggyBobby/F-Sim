"""
╔═══════════════════════════════════════════════════════════════════════════╗
║                    MASTER CAR DATA FILE  —  EDIT ME                       ║
║                                                                           ║
║  Every number the simulation uses is set HERE and pulled from here by     ║
║  the rest of the code. This is the ONLY file you change for car data.     ║
╚═══════════════════════════════════════════════════════════════════════════╝

DATA SOURCES / TAGS
───────────────────
  STATUS 2026-08-30 — everything tagged FROM REPORT / DERIVED comes from
  "1ME295B Project Report_FINALJW-2.pdf" (Table 1, SJSU MS project on the
  OLD 2023 car). The team has been told the NEW car's numbers will come out
  very similar, so these are now the sim's WORKING BASELINE — no longer
  throwaway placeholders, and the sim's results can be quoted as real
  (subject to the tire caveat below).

  What "the new car is similar" does NOT do:
    • it does not fix the report's own internal contradictions — see the
      MASS box and WEIGHT_FRACTION_FRONT, both still open;
    • it does not supply what the report never had — tire GRIP data and
      aero balance are still PLACEHOLDER guesses, and the tire numbers are
      what the s-diff/TV results are most sensitive to;
    • it does not settle drivetrain design choices (GEAR_RATIO) that the
      new car picks fresh rather than inherits.
  When the new car is actually measured, these values get VERIFIED against
  it, and whatever comes back different gets replaced.
  Exception: entries tagged CURRENT CAR are for the car being built now.

  ⚠️ TEAM DECISION 2026-08-30: this parameter set is declared the FINAL
  DESIGN set (TTC access not available; pack voltage fixed at 380 V).
  "Final" freezes the values — it does not upgrade their provenance: the
  tire block remains unvalidated estimates, so absolute outputs remain
  indicative and config-vs-config comparisons remain the solid product.

  FROM REPORT   — Table 1 value (2023 car), adopted as our working number
                  on the team's "the new car will be very similar" call.
  DERIVED       — computed from report values (formula shown).
  CURRENT CAR   — reported by the team for the car being built now.
  PLACEHOLDER   — a guess; no source at all yet.
  RULES VALUE   — fixed by the FSAE EV rulebook.
  SUSPECT       — entered from the report, but the value conflicts with
                  other numbers in the same table or with physical sanity.

HOW TO EDIT THIS FILE
─────────────────────
1. Everything in the code runs in SI units: kg, m, s, N, N·m, W, rad.
   If you measured something in imperial, DON'T convert by hand — enter it
   using the conversion constants below, e.g.:

        CAR_MASS_NO_DRIVER = 507.0 * LB       # weighed 507 lb on the scales
        H_CG               = 11.0  * INCH     # 11 inches
        MOTOR_TORQUE_PEAK  = 23.6  * LBFT     # 23.6 lb-ft

2. When you replace a value, update its tag to e.g.
   `MEASURED 2026-09-01 (corner scales)` so we know which numbers are real.

3. Each parameter is documented as:
        What:  plain-language definition
        Unit:  SI unit the code expects (+ imperial entry hint)
        From:  where/how the team gets the real value

4. After editing, just rerun:  .venv/bin/python run_sim.py
   NOTE: if vehicle/tire values change meaningfully, the controller gains
   at the bottom of this file must be retuned.

(Test-maneuver settings — speeds, steer angles, throttle profiles — are
scenario definitions, not car data; those live in maneuvers.py.)
"""

import math

# ─────────────────────────────────────────────────────────────────────────
# UNIT CONVERSIONS — multiply your measured value by these to get SI
# ─────────────────────────────────────────────────────────────────────────
LB   = 0.45359237      # kg per pound-mass        (weight/mass)
INCH = 0.0254          # m per inch               (lengths)
FT   = 0.3048          # m per foot
LBFT = 1.3558179       # N·m per lb-ft            (torque)
MPH  = 0.44704         # m/s per mph              (speed)
HP   = 745.699872      # W per horsepower         (power)
DEG  = math.pi / 180.0 # rad per degree           (angles)
RPM  = math.pi / 30.0  # rad/s per rev-per-minute (rotational speed)


# ─────────────────────────────────────────────────────────────────────────
# ENVIRONMENT
# ─────────────────────────────────────────────────────────────────────────
# What:  standard gravity. A physical constant — never edit.
# Unit:  m/s²
G = 9.81                                # FROM REPORT (same as standard)

# What:  air density, used for downforce and drag.
# Unit:  kg/m³
RHO_AIR = 1.204                         # FROM REPORT


# ─────────────────────────────────────────────────────────────────────────
# MASS  — read this box before entering anything
# ─────────────────────────────────────────────────────────────────────────
# The sim wants mass WITHOUT driver and DRIVER mass as two separate entries;
# total-with-driver is computed (m_total = car + driver), never entered.
#
# HISTORY (kept because it explains why earlier runs used 241.7 kg): until
# 2026-08-30 the total came from the report's single m_t = 241.7 kg, read
# as car+driver, split 166.7 + 75. The report's own mass rows never added
# up (sprung 180.9 + 2 × unsprung 30.4 = 241.7 — one row double-counts),
# and 166.7 kg was flagged as implausibly light. The team's real number
# below supersedes all of that. Runs 001–010 ran at 241.7 kg total; runs
# from here run at ~256 kg (+6%) — metrics across that boundary are not
# directly comparable.

# What:  complete ready-to-run vehicle mass, NO driver seated.
# Unit:  kg (weighed in lbs — entered exactly as reported, in lbs)
# From:  CORNER SCALES 2026-08-31 (E-Z Weight Intercomp SW500), no driver:
#            LF  87.5 lb    RF  86.3 lb      front pair 173.8 lb (43.44%)
#            LR 114.8 lb    RR 111.5 lb      rear pair  226.3 lb
#            total 400.1 lb   left 50.56%   cross (RF+LR) 50.26%
#        Left/right is 0.56% asymmetric (≈1 kg heavier left) — the sim
#        assumes perfect L/R symmetry; at that size the error is noise.
#        Cross weight is a setup number this sim doesn't use; recorded for
#        the suspension crew.
CAR_MASS_NO_DRIVER = 400.1 * LB         # MEASURED 2026-08-31 (corner scales) = 181.5 kg

# What:  driver mass in full gear (suit, helmet, shoes). Convention: use
#        the HEAVIEST regular driver for conservative tuning — if 156 lb
#        is not the heaviest, bump this when the roster is known. Confirm
#        whether 156 was weighed suited (gear adds ~5-8 lb).
# Unit:  kg   (entered in lbs, exactly as reported)
# From:  team, 2026-08-30: "the driver is 156 lbs".
DRIVER_MASS = 156.0 * LB                # CURRENT CAR (team) = 70.8 kg


# ─────────────────────────────────────────────────────────────────────────
# GEOMETRY & CG (whole-vehicle values — the sim is a single rigid body)
# ─────────────────────────────────────────────────────────────────────────
# What:  distance between front and rear axle centerlines.
# Unit:  m   (inches?  write e.g.  61.0 * INCH)
# From:  report L_f + L_r = 0.64 + 0.88. (The report's sprung-CG distances
#        sum to 1.524 — 4 mm off; the vehicle-CG pair is used here.)
WHEELBASE = 1.52                        # DERIVED from report (L_f + L_r)

# What:  front track width — distance between the CENTERS of the two front
#        tire contact patches (NOT outside-to-outside of the tires).
# Unit:  m
TRACK_FRONT = 1.34                      # FROM REPORT

# What:  rear track width, same definition as front.
# Unit:  m
TRACK_REAR = 1.34                       # FROM REPORT

# What:  static front weight fraction = front axle weight / total weight.
#        By statics this equals (CG-to-REAR-axle distance)/wheelbase.
# Unit:  dimensionless, 0..1
# From:  CORNER SCALES 2026-08-31, NO driver: (87.5+86.3)/400.1 = 43.44%
#        front — finally replaces the old report's self-contradictory
#        46%/58%. ⚠️ ONE STEP LEFT: the sim runs WITH a driver, and a
#        seated driver's CG sits aft of mid-wheelbase, so the true
#        with-driver front share is ~1–2% LOWER than this car-only number
#        (estimate ≈ 41.5–43%). Using the car-only measurement until the
#        team repeats the weighing with the driver seated — one more
#        minute on the same scales.
WEIGHT_FRACTION_FRONT = 0.4344          # MEASURED 2026-08-31 (scales, NO driver — redo seated)

# What:  height of the CG above the ground. Drives ALL load transfer.
# Unit:  m   (inches?  write e.g.  11.0 * INCH)
# From:  report h = 0.23 m — strictly the SPRUNG-mass CG height, but with
#        unsprung CGs at wheel-center height (~0.20 m) the whole-vehicle
#        value works out to ≈0.226 m, so 0.23 is fine as-is.
H_CG = 0.23                             # FROM REPORT (sprung ≈ total)

# What:  yaw moment of inertia of the whole vehicle about the vertical
#        axis through the CG. Sets how fast yaw rate responds to the TV
#        yaw moment.
# Unit:  kg·m²
I_Z = 70.38                             # FROM REPORT (J_z)


# ─────────────────────────────────────────────────────────────────────────
# WHEELS & DRIVETRAIN
# Layout (team-confirmed 2026-08-30): FOUR motors fitted, one per wheel
# (hub drive, planetary reduction in each upright) — the TWO REAR ones are
# active in the current build, 4WD is the goal. This sim models the active
# 2-motor rear axle; the 4WD extension is described in README.md.
# Motors: AMK "Racing Kit" A2370DD (DD5-type synchronous servo) with
# KW26-S5-FSE-4Q inverters — see "AMK Racing Kit Datasheet.pdf" in this
# folder (kit manual; exact motor data sheet: Motor_data_sheet_A2370DD_DD5).
# ─────────────────────────────────────────────────────────────────────────
# What:  LOADED tire radius — ground to axle center with the car's weight
#        on the tire. Converts wheel torque to drive force (F = T/r) and
#        wheel speed to ground speed.
# Unit:  m
# From:  CURRENT CAR runs 18-inch outer-diameter tires on 10-inch rims
#        (team-reported). Radius = 18"/2 = 0.2286 m. The rim size doesn't
#        enter the sim — only the tire's rolling radius does. The LOADED
#        radius is a few mm less than free radius (tire squish); measure
#        axle-center height with the car on the ground to refine.
#        (The old report's "0.406 m radius" was that car's tire diameter.)
WHEEL_RADIUS = 18.0 * INCH / 2.0        # CURRENT CAR (18in tire OD, 10in rim)

# What:  total reduction between motor shaft and wheel (motor revs per
#        wheel rev). With one motor PER WHEEL this is the PLANETARY gear
#        stage packaged in each upright with the motor — not a chain or
#        gearbox. It must exist: the A2370DD makes 21 N·m at up to
#        20,000 rpm; direct drive would give 92 N of drive force per wheel
#        (0.08 g car) and a 479 m/s speed match. Wheel torque = motor × gr,
#        reflected rotor inertia = rotor J × gr².
# Unit:  dimensionless
# From:  old report gr = 13 (typical AMK-kit planetaries run 12–14.5:1).
#        DESIGN CHOICE, not inherited — ask powertrain: "what is the
#        reduction ratio of the planetary in our uprights?"
GEAR_RATIO = 13.0                       # FROM REPORT — confirm upright planetary ratio

# What:  peak torque of ONE motor at its shaft (before the gear reduction).
#        → peak WHEEL torque = 21 × 13 = 273 N·m per side.
# Unit:  N·m   (lb-ft?  write e.g.  23.6 * LBFT)
# From:  AMK kit manual §6.5.3 torque curve: flat 21 N·m up to
#        ~13,000 rpm at 600 VDC (motor A2370DD). Matches the old report.
MOTOR_TORQUE_PEAK = 21.0                # CURRENT CAR (AMK kit datasheet)

# What:  accumulator (HV pack) nominal voltage. Not used directly by the
#        physics — it SETS the motor peak power below, and it is the
#        number that sizes the car at the rules cap: 4 motors × ~20 kW at
#        380 V = 80 kW exactly (the 4WD end state); the current 2-rear
#        build tops out at ~40 kW, so the 80 kW cap is unreachable until
#        4WD.
# Unit:  V
PACK_VOLTAGE = 380.0                    # CURRENT CAR (design decision 2026-08-30)

# What:  peak mechanical power of ONE motor at OUR pack voltage.
# Unit:  W   (kW: write e.g. 20e3;  hp?  write e.g.  27.0 * HP)
# From:  AMK kit manual §6.5.3 — peak power rides on HV voltage ("The
#        maximum motor power dependents on the available HV voltage"),
#        curves give ~26 kW @ 500 VDC and ~32 kW @ 600 VDC. Scaled to the
#        380 V pack three ways: 26·(380/500) = 19.8 kW, 32·(380/600) =
#        20.3 kW, base-speed method 21 N·m × (13 krpm·380/600) = 18.1 kW
#        → 20 kW adopted. Refine by reading the 380 V curve directly or
#        from a dyno pull once the pack exists.
MOTOR_POWER_PEAK = 20e3                 # DERIVED from kit curves at PACK_VOLTAGE

# What:  maximum motor speed. NOT enforced by this basic sim (with 13:1
#        and 0.229 m tires it corresponds to ~36 m/s ≈ 130 km/h, far above
#        the test maneuvers) — recorded for completeness / future top-speed
#        studies. The inverter caps speed setpoints at 30,000 rpm.
# Unit:  rad/s   (entered in rpm via the RPM constant)
# From:  AMK kit manual §6.5.3 curves (characteristics end at 20,000 rpm).
MOTOR_MAX_SPEED = 20000 * RPM           # CURRENT CAR (AMK kit datasheet)

# What:  spin inertia of ONE rear corner about the axle: tire + rim +
#        brake rotor + hub + (motor rotor inertia × gear_ratio²). Sets how
#        fast an unloaded wheel spins up — matters a lot to the s-diff.
# Unit:  kg·m²
# From:  estimate = tire+rim+hub ≈ 0.30 (PLACEHOLDER guess for an 18"
#        tire on a 10" rim) + rotor inertia × gear² ≈ 2.74e-4 × 13² ≈ 0.05
#        (rotor J is the typical A2370DD value — confirm from the motor
#        data sheet, it is not in the kit manual). Replace with CAD.
#        The old report's I_w = 3.033 was rejected: ~10× too large for one
#        corner (possibly a 4-wheel total, and it's the old car anyway).
I_WHEEL = 0.35                          # DERIVED estimate — replace with CAD

# What:  total drive power cap from the FSAE EV rules (EV.4.2: 80 kW at
#        the accumulator). Applied here to mechanical power — drivetrain
#        efficiency is ignored in this basic sim.
# Unit:  W
POWER_CAP_TOTAL = 80e3                  # RULES VALUE (simplified)

# What:  below this speed the controllers command no negative (regen)
#        torque — the rules restrict regen near standstill (≈ 5 km/h).
# Unit:  m/s   (5 km/h = 1.39 m/s)
REGEN_SPEED_CUTOFF = 1.4                # RULES VALUE (approx.)


# ─────────────────────────────────────────────────────────────────────────
# AERO — entered the way the report gives it: coefficients + frontal area.
# The sim uses the products Cl·A and Cd·A, computed below.
# ─────────────────────────────────────────────────────────────────────────
# What:  lift (downforce) coefficient, referenced to FRONTAL_AREA.
# Unit:  dimensionless
CL_COEFF = 3.18                         # FROM REPORT (C_L)

# What:  drag coefficient, referenced to FRONTAL_AREA.
# Unit:  dimensionless
CD_COEFF = 1.36                         # FROM REPORT (C_D)

# What:  aerodynamic frontal reference area.
# Unit:  m²
FRONTAL_AREA = 1.106                    # FROM REPORT (A_F)

# Computed products the sim actually uses — don't edit these two lines,
# edit the three entries above. Downforce = 0.5·rho·CLA·v², same for drag.
CLA = CL_COEFF * FRONTAL_AREA           # DERIVED = 3.52 m²
CDA = CD_COEFF * FRONTAL_AREA           # DERIVED = 1.50 m²

# What:  fraction of total downforce landing on the FRONT axle (aero
#        balance). 0.45 = 45% front / 55% rear.
# Unit:  dimensionless, 0..1
# From:  NOT in the report — get from aero team (CFD center of pressure).
AERO_BALANCE_FRONT = 0.45               # PLACEHOLDER


# ─────────────────────────────────────────────────────────────────────────
# STEERING GEOMETRY
# ─────────────────────────────────────────────────────────────────────────
# What:  Ackermann fraction — how much of the geometric (100%) Ackermann
#        inner/outer steer split the steering actually delivers.
#          0.0 = parallel steer (both fronts get the same angle)
#          1.0 = full geometric Ackermann (inner = atan(L/(R − t/2)), etc.)
#          negative = anti-Ackermann
#        Only matters at large steer angles: at 5° the inner/outer split is
#        ~±0.2°; at the 23° full-lock hairpin it is ~±4° per wheel.
# Unit:  dimensionless (fraction of geometric Ackermann)
# From:  steering/suspension team. Team chart curve (2026-08-30):
#            LWheel(Deg) = -0.0797 + 0.31·x - 8.44E-04·x²
#        Confirmed y = LEFT road-wheel angle [deg]. x is presumed the
#        STEERING-WHEEL angle [deg]: the curve reaches 23.2° (the team's
#        stated max road-wheel angle) at x ≈ 105°, a typical FSAE lock.
#        (Quadratic coefficient read as -8.44E-04; the literal -0.0844
#        makes the curve peak at x≈1.8° — not a wheel-angle chart.)
#        Implied steering ratio: ~3.2:1 near center, ~7.5:1 at lock.
#
#        ⚠️ ONE wheel's curve cannot give the Ackermann SPLIT — that is the
#        DIFFERENCE between the two wheels, and the fit is not valid
#        through x=0, so the right wheel can't be mirror-derived. NEEDED
#        from the same chart: the RWheel(Deg) polynomial (or a few x,
#        LWheel, RWheel point triples). Then LWheel−RWheel vs x wires
#        straight into vehicle.front_steer_angles. Until then the sim
#        stays parallel-steer, as before.
ACKERMANN_FRACTION = 0.0                # PLACEHOLDER — parallel steer


# ─────────────────────────────────────────────────────────────────────────
# SENSORS & VCU — what the controller is allowed to know (sensors.py).
# The controller no longer reads the simulation's perfect truth: it reads
# these sensors, sampled at the VCU rate, with quantization and noise.
# ─────────────────────────────────────────────────────────────────────────
# What:  VCU control-loop rate — how often sensors are sampled and torque
#        commands are recomputed (physics still integrates at 4 kHz).
# Unit:  Hz
# From:  software team — set to their real loop rate. 100 Hz proven stable
#        for the current gains (verify.py section H).
VCU_RATE_HZ = 100.0                     # PLACEHOLDER — ask software team

# What:  steering map — handwheel angle x [deg] to road-wheel angle [deg],
#        the team chart's LWheel curve: y = A0 + A1·x + A2·x².
#        Used odd-symmetrically (sign(x)·map(|x|)) and centered so
#        map(0) = 0 (the −0.0797° intercept is fit noise — an offset that
#        size would pull the car left with the wheel straight).
# Unit:  deg → deg
# From:  team steering chart 2026-08-30 ("LWheel(Deg)").
STEER_MAP_A0 = -0.0797                  # CURRENT CAR (team chart)
STEER_MAP_A1 = 0.31                     # CURRENT CAR (team chart)
STEER_MAP_A2 = -8.44e-4                 # CURRENT CAR (team chart; -844E-04 read as -8.44E-04)

# What:  steering-angle sensor (SAS) quantization, at the HANDWHEEL.
# Unit:  deg per LSB
SAS_QUANT_DEG = 0.5                     # PLACEHOLDER — sensor spec

# What:  wheel-speed sensing = MOTOR shaft speed from the AMK resolver
#        over CAN (there is no separate wheel sensor); VCU divides by the
#        planetary ratio. Quantization of the reported motor speed.
# Unit:  rpm per LSB (motor side)
WSS_QUANT_RPM = 1.0                     # PLACEHOLDER — AMK CAN resolution

# What:  IMU yaw-gyro 1-sigma noise and constant bias, and the first-order
#        low-pass the VCU applies to the gyro before feedback.
# Unit:  rad/s (entered in deg/s), Hz
IMU_GYRO_NOISE_STD = 0.3 * DEG          # PLACEHOLDER — IMU spec (deg/s 1σ)
IMU_GYRO_BIAS = 0.1 * DEG               # PLACEHOLDER — uncalibrated bias
IMU_LPF_HZ = 20.0                       # PLACEHOLDER — VCU filter choice

# What:  IMU accelerometer 1-sigma noise (ax, ay channels) and constant
#        bias on the longitudinal channel (what the fused speed estimate
#        below has to live with — it is why that filter leaks back to the
#        wheel speeds instead of integrating forever).
# Unit:  m/s²
IMU_ACCEL_NOISE_STD = 0.05              # PLACEHOLDER — IMU spec
IMU_ACCEL_BIAS = 0.03                   # PLACEHOLDER — uncalibrated bias

# What:  VCU ground-speed ESTIMATOR (sensors.py) — there is no speed
#        sensor. VX_EST_USE_IMU=False: wheel-only (slower rear driving,
#        faster rear braking). True: complementary filter — integrate the
#        accelerometers with the gyro (v̇x = ax + r·vy, v̇y = ay − r·vx; vy
#        leaks to zero over VX_EST_VY_TAU since nothing measures it), pull
#        vx toward the yaw-corrected wheel-based value with time constant
#        VX_EST_TAU; if wheel and IMU disagree by more than VX_EST_GATE_MPS
#        the wheels are taken to be slipping together and the pull slows
#        to VX_EST_GATE_TAU (bias-bounded, never open-loop). Stand-in for
#        the real car's EKF; tune against launch and full-throttle
#        corner-exit logs once they exist.
# Unit:  bool, s, m/s, s, s
VX_EST_USE_IMU = True                   # PLACEHOLDER — estimator design choice
VX_EST_TAU = 0.3                        # PLACEHOLDER — estimator tuning
VX_EST_GATE_MPS = 1.0                   # PLACEHOLDER — estimator tuning
VX_EST_GATE_TAU = 3.0                   # PLACEHOLDER — estimator tuning
VX_EST_VY_TAU = 1.0                     # PLACEHOLDER — estimator tuning
# What:  IMU accelerometer 1-sigma noise (ax, ay channels).
# Unit:  m/s²
IMU_ACCEL_NOISE_STD = 0.05              # PLACEHOLDER — IMU spec

# What:  accelerator pedal position sensor (APPS) quantization. The pedal
#        map is linear: APPS % → torque request, 100% = 2×T_wheel_max.
# Unit:  percent per LSB
APPS_QUANT_PCT = 0.5                    # PLACEHOLDER — sensor spec

# What:  brake pressure sensor (BPS) full range, quantization, and the
#        pressure above which the brake counts as "actuated" for the rules
#        plausibility check.
# Unit:  bar
BPS_RANGE_BAR = 100.0                   # PLACEHOLDER — sensor spec
BPS_QUANT_BAR = 0.5                     # PLACEHOLDER — sensor spec
BPS_ACTUATED_BAR = 3.0                  # PLACEHOLDER — calibration choice

# What:  total rear-axle REGEN torque at full brake pressure (mechanical
#        brakes are NOT modeled — BPS commands regen only in this sim).
# Unit:  N·m (total, both motors)
T_REGEN_MAX = 250.0                     # PLACEHOLDER — battery charge limit sets this

# What:  FSAE EV.4.7 APPS/BPS plausibility: >25% APPS while braking cuts
#        motor power until APPS falls below 5%.
# Unit:  percent
PLAUS_APPS_CUT = 25.0                   # RULES VALUE (EV.4.7)
PLAUS_APPS_RESTORE = 5.0                # RULES VALUE (EV.4.7)

# What:  seed for the sensor-noise generator — runs are exactly repeatable
#        and all four controller configs see IDENTICAL noise (fair fight).
SENSOR_SEED = 2026                      # NUMERICAL GUARD (repeatability)


# ─────────────────────────────────────────────────────────────────────────
# LOAD TRANSFER
# ─────────────────────────────────────────────────────────────────────────
# What:  fraction of total LATERAL load transfer reacted by the front
#        axle, set by the front/rear roll-stiffness split.
# Unit:  dimensionless, 0..1
# From:  DERIVED from the report's spring rates: with equal tracks the
#        split ≈ ks_front/(ks_front + ks_rear) = 19.3/(19.3+24.8) = 0.438.
#        Crude — ignores ARBs, motion ratios, and roll-center heights;
#        refine with the suspension team's real roll-stiffness numbers.
#        (Report spring units say "N/m"; clearly N/mm — the RATIO is
#        unit-independent, which is all that's used here.)
LAT_TRANSFER_FRAC_FRONT = 19.3 / (19.3 + 24.8)   # DERIVED from report


# ─────────────────────────────────────────────────────────────────────────
# TIRES — ⚠️ the report contains NO tire GRIP data. Its "tire stiffness"
# (113 832 N/m) is the VERTICAL spring rate of the carcass — useless for
# grip. ✅ FITTED 2026-08-31 from TTC ROUND 9 (program B2356, Calspan
# Jan 2022) — tire_fit.py, RunData Matlab SI files in ttc/.
#
# Our tire (Hoosier 18.0x7.5-10 R20, pattern FT28) was never TTC-tested,
# so this is a POOLED SURROGATE FIT, same R20 compound:
#     lateral:  43075 16x7.5-10 @8" (same width) + 43100 18.0x6.0-10 @7"
#               (same OD) — 57,294 samples, RMS 87 N; the two tires
#               individually agree within ±6% on µ₀, ±1% on stiffness
#     longitudinal: 43100 18.0x6.0-10 (both rims) — 25,807 samples
# Fit conditions & VALIDITY:
#     12 psi (ASSUMED running pressure — confirm with team), |camber|<1°,
#     loads 210–1150 N (covers our 430–620 N corners: interpolation ✓),
#     slip angle sweeps ±12° — the R20 PLATEAUS there, it does not fall
#     off; beyond ±12° the model extrapolates a flat plateau. No camber
#     effects modeled (fit at ~0°).
# Belt→asphalt: peak µ scaled by TIRE_MU_ROAD_SCALE below; stiffnesses
# and shapes carried over unscaled (standard practice).
# Model: simplified Magic Formula + friction ellipse (separate µx).
# ─────────────────────────────────────────────────────────────────────────
# What:  belt→asphalt grip scaling. Calspan's sandpaper belt grips harder
#        than track asphalt; common practice scales the fitted peak µ by
#        ~0.65–0.70 for on-road prediction (stiffnesses/shapes carry over
#        unscaled). 0.67 chosen; VALIDATE on skidpad — that measurement
#        replaces this guess with the truth.
# Unit:  dimensionless
TIRE_MU_ROAD_SCALE = 0.67                        # PLACEHOLDER — skidpad validates

# What:  peak LATERAL friction coefficient at the nominal load.
# Unit:  dimensionless
# From:  TTC Round 9 pooled fit (belt µ₀ = 2.593, surrogate spread ±6%)
#        × road scale.
TIRE_MU0 = TIRE_MU_ROAD_SCALE * 2.593            # TTC FIT R9 = 1.74 road

# What:  peak LONGITUDINAL friction coefficient (drive/brake). The R20
#        measures ~9% below its lateral peak; combined slip caps on the
#        friction ELLIPSE in tire.py.
# Unit:  dimensionless
# From:  TTC Round 9 pooled drive/brake fit (belt 2.359) × road scale.
TIRE_MU0_LONG = TIRE_MU_ROAD_SCALE * 2.359       # TTC FIT R9 = 1.58 road

# What:  load sensitivity — how much the friction COEFFICIENT drops as
#        vertical load rises: mu = MU0 * (1 - S_MU*(Fz/FZ_NOM - 1)).
#        Shared by the lateral and longitudinal peaks.
# Unit:  dimensionless
TIRE_S_MU = 0.112                                # TTC FIT R9

# What:  the "nominal" vertical load at which MU0 is defined — pick ≈ the
#        static load on one tire.
# Unit:  N
TIRE_FZ_NOM = (CAR_MASS_NO_DRIVER + DRIVER_MASS) * G / 4.0   # DERIVED ≈ 593 N
                                                 # (m·g/4 — follows the mass
                                                 # entries above; was a
                                                 # hardcoded 241.7)

# What:  normalized cornering stiffness of a FRONT tire: lateral force per
#        radian of slip angle, per newton of load (C_alpha = this * Fz).
#        FSAE slicks: ~14–25 1/rad from TTC.
# Unit:  1/rad
TIRE_C_ALPHA_FRONT = 37.0                        # TTC FIT R9 (same tire all round)

# What:  same, for a REAR tire.
# Unit:  1/rad
TIRE_C_ALPHA_REAR = 37.0                         # TTC FIT R9 (same tire all round)
# NOTE: identical normalized stiffness front/rear makes the LINEAR
# understeer gradient exactly zero (neutral) — balance now comes entirely
# from load transfer, aero split, and tire load sensitivity.

# What:  normalized longitudinal slip stiffness: drive/brake force per
#        unit slip ratio per newton (C_kappa = this * Fz). TTC: ~20–40.
# Unit:  1/unit-slip
TIRE_C_KAPPA = 41.9                              # TTC FIT R9

# What:  Magic Formula shape (C) and curvature (E) factors — how sharply
#        force peaks and falls off past the peak. From the TTC fit.
# Unit:  dimensionless
TIRE_SHAPE_C_LAT = 1.397                         # TTC FIT R9  (C_y)
TIRE_CURV_E_LAT = 0.365                          # TTC FIT R9  (E_y)
TIRE_SHAPE_C_LONG = 1.705                        # TTC FIT R9  (C_x)
TIRE_CURV_E_LONG = 0.389                         # TTC FIT R9  (E_x)


# ─────────────────────────────────────────────────────────────────────────
# CONTROLLER GAINS — retuned 2026-08-29 against the report-based car
# above (heavier wheel inertia, 273 N·m wheel torque, I_z = 70). Whenever
# vehicle/tire numbers change meaningfully, retune again (run the three
# maneuvers; oscillation = too hot, sluggish tracking = too cold).
# ─────────────────────────────────────────────────────────────────────────
# Software differential: PI on wheel-speed-difference error.
KP_SDIFF = 15.0        # N·m per rad/s of wheel-speed error    TUNED (sim)
KI_SDIFF = 60.0        # N·m per rad of accumulated error      TUNED (sim)
I_SDIFF_MAX = 60.0     # N·m anti-windup clamp on the integral   TUNED (sim)

# What:  hard cap on how much torque the s-diff term may transfer between
#        the wheels. Without it, an unbounded speed-difference PI dumps
#        torque onto the loaded outer tire near the limit, burns its
#        lateral grip (friction circle), and power-oversteers the car —
#        the sim demonstrates exactly this if you raise the cap. The
#        s-diff's job is trimming wheel speeds, not large torque moves.
# Unit:  N·m
DT_SDIFF_MAX = 80.0    # TUNED (sim)

# Torque vectoring: PI on yaw-rate error -> yaw moment.
KP_TV = 2500.0         # N·m per rad/s of yaw-rate error       TUNED (sim)
KI_TV = 1000.0         # N·m per rad of accumulated error      TUNED (sim)
I_TV_MAX = 200.0       # N·m anti-windup clamp on the integral   TUNED (sim)
MZ_MAX = 600.0         # N·m clamp on commanded yaw moment       TUNED (sim)

# What:  the yaw-rate reference is capped at this fraction of the friction-
#        limited lateral acceleration.
# Unit:  dimensionless, 0..1
AY_FRAC = 0.95         # TUNED (sim)


# ─────────────────────────────────────────────────────────────────────────
# NUMERICAL GUARDS (not car data — leave alone unless the sim misbehaves)
# ─────────────────────────────────────────────────────────────────────────
# Minimum speed used in slip-angle/slip-ratio denominators. Unit: m/s.
V_EPS = 0.5            # NUMERICAL GUARD


# ─────────────────────────────────────────────────────────────────────────
# REPORT VALUES **NOT USED** BY THIS SIM (documented so nobody hunts for
# them). This basic sim has no suspension/ride DOFs — the full-model team
# needs these, we don't:
#   sprung mass (180.9 kg), unsprung masses (7.4/7.4/7.8/7.8 kg),
#   sprung-CG-to-axle distances (0.817/0.707 m), pitch inertia (64.46),
#   roll inertia (15.34), suspension damping (1437.6/1644.4 N·s/m),
#   tire VERTICAL stiffness (113 832 N/m), rolling-resistance coeff (0.02
#   — rolling drag is deliberately omitted here).
# Suspension spring rates (19.3/24.8) are used ONLY via their ratio, for
# LAT_TRANSFER_FRAC_FRONT above.
# ─────────────────────────────────────────────────────────────────────────
