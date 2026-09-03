"""Generate "FSAE-Sim Parameters.xlsx" — the team-facing parameter sheet.

Three tabs:
  1. Car parameters   — every number in car_data.py: what it is, its value
                        (pulled LIVE from car_data.py so the sheet can never
                        disagree with the sim), provenance, and how/why to
                        measure it.
  2. Sim signals      — the quantities the sim COMPUTES each step (slip
                        angle, slip ratio, loads, yaw rate, ...): what they
                        are, how the sim gets them, and which sensor sees
                        them on the real car.
  3. Test values      — the maneuver knobs and the real envelope numbers.

Color rule (team convention): values confirmed for the CURRENT CAR or
DRIVER are GREEN; everything else (report car, guesses, rules, tuned gains)
is black. When a value is later MEASURED it stays green.

Rebuild after editing car_data.py:
    .venv/bin/python param_sheet.py        # writes + reopens the file

Import to Google Sheets: drag the .xlsx into drive.google.com — formatting
and colors carry over.
"""

import math

from openpyxl import Workbook
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

from model.config import cfg  # was: import car_data as cd

OUT = "FSAE-Sim Parameters.xlsx"

GREEN = "FF1A7A1A"          # current-car / driver / measured values
BLACK = "FF1A1A1A"
HDR_FILL = PatternFill("solid", fgColor="FF223038")
SEC_FILL = PatternFill("solid", fgColor="FFE8EAE4")
WARN = "FF9A6A00"           # placeholder annotation text

# tags that count as "for the current car / driver" → green
GREEN_TAGS = {"CURRENT CAR", "MEASURED", "TTC FIT"}

THIN = Side(style="thin", color="FFD9DBD4")
BORDER = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)


def fmt(v):
    """Human-friendly number."""
    if isinstance(v, float):
        if v == 0:
            return "0"
        if abs(v) >= 1000:
            return f"{v:,.0f}"
        return f"{v:.4g}"
    return str(v)


# ──────────────────────────────────────────────────────────────────────
# Tab 1 — car parameters
# columns: Parameter | Symbol | Code name | What it is / what it simulates |
#          Value now | Units | Status (source) | What we measure |
#          How we measure it | Why it matters
# value=None means "pull getattr(cd, code_name)"
# ──────────────────────────────────────────────────────────────────────
P = []  # (section, rows)

P.append(("ENVIRONMENT", [
 ("Gravity", "g", "G", "Standard gravitational acceleration — sets every weight and load.",
  None, "m/s²", "constant", "Nothing", "Physical constant", "Everything vertical scales from it."),
 ("Air density", "ρ", "RHO_AIR", "Density of the air the car drives through — sets downforce and drag.",
  None, "kg/m³", "FROM REPORT", "Ambient conditions on event days (optional refinement)",
  "Weather data / altitude correction", "Downforce and drag are ½ρ·(coeff·A)·v² — 5% density = 5% aero."),
]))

P.append(("MASS", [
 ("Car mass, no driver", "m_car", "CAR_MASS_NO_DRIVER",
  "Complete ready-to-run vehicle, nobody in the seat.",
  None, "kg (400.1 lb)", "MEASURED 2026-08-31 (corner scales)",
  "Total car weight", "Corner scales (Intercomp SW500): 87.5/86.3/114.8/111.5 lb",
  "With driver mass it sets EVERY inertial and grip force in the sim."),
 ("Driver mass", "m_driver", "DRIVER_MASS",
  "Driver in full gear (suit, helmet, shoes). Convention: use the HEAVIEST regular driver.",
  None, "kg (entered 156 lb)", "CURRENT CAR (team)",
  "Suited driver weight", "Bathroom scale, fully suited",
  "The car never runs without one. Confirm 156 lb is heaviest + includes gear (~5–8 lb)."),
]))

P.append(("GEOMETRY & CG", [
 ("Wheelbase", "L", "WHEELBASE", "Front-axle to rear-axle distance.",
  None, "m", "FROM REPORT (2023 car)", "Axle-center to axle-center distance",
  "Tape measure / CAD", "Sets yaw geometry, the bicycle-model reference, and CG position math."),
 ("Front track", "t_f", "TRACK_FRONT", "Distance between the two FRONT contact-patch centers.",
  None, "m", "FROM REPORT", "Center-of-tire to center-of-tire, front",
  "Tape measure / CAD", "Lever arm for front lateral load transfer."),
 ("Rear track", "t_r", "TRACK_REAR", "Same, rear axle.",
  None, "m", "FROM REPORT", "Center-of-tire to center-of-tire, rear",
  "Tape measure / CAD", "THE torque-vectoring lever arm: Mz = (t_r/2)·ΔFx. Bigger track = more yaw per N·m of split."),
 ("Front weight fraction", "%front", "WEIGHT_FRACTION_FRONT",
  "Share of total weight on the front axle, driver seated.",
  None, "0–1", "MEASURED 2026-08-31 (scales, NO driver)",
  "Front axle share WITH DRIVER SEATED (expect ~1–2% lower than the 43.44% car-only)",
  "Repeat the corner-scale session with the driver in the seat",
  "Decides how much grip each axle owns — understeer/oversteer balance starts here."),
 ("CG height", "h", "H_CG", "Height of the center of gravity above ground.",
  None, "m", "FROM REPORT", "CG height",
  "Tilt test (weigh one axle while lifting the other) or CAD",
  "Drives ALL load transfer — every grip shift in braking, accel, and cornering is proportional to h."),
 ("Yaw inertia", "I_z", "I_Z", "Resistance of the whole car to changing its yaw (spin) rate.",
  None, "kg·m²", "FROM REPORT", "Yaw moment of inertia",
  "CAD mass model (practical) or trifilar pendulum test",
  "Sets how fast the car responds to the TV yaw moment — directly in Iz·ṙ = ΣMz."),
]))

P.append(("WHEELS & DRIVETRAIN", [
 ("Tire radius (loaded)", "r_w", "WHEEL_RADIUS", "Ground-to-axle-center distance with the car's weight on the tire.",
  None, "m (18\" OD / 2)", "CURRENT CAR",
  "Axle-center height, car on the ground", "Ruler to the axle center",
  "Converts wheel torque to drive force (F = T/r) and wheel speed to ground speed. Loaded is a few mm less than free."),
 ("Upright planetary ratio", "gr", "GEAR_RATIO",
  "Reduction of the planetary gear stage in each upright, motor shaft → wheel.",
  None, "—", "FROM REPORT ⚠ confirm with powertrain",
  "The planetary's reduction ratio", "Ask powertrain / count gear teeth in CAD",
  "Wheel torque = motor torque × gr; reflected rotor inertia = rotor J × gr². The 21 N·m motor is useless without it."),
 ("Motor peak torque", "T_pk", "MOTOR_TORQUE_PEAK", "Peak torque of ONE motor at its shaft.",
  None, "N·m", "CURRENT CAR (AMK kit datasheet)",
  "Nothing (datasheet value)", "AMK kit manual §6.5.3; dyno confirms someday",
  "×13 gear = 273 N·m per wheel — the ceiling every torque command lives under."),
 ("Pack voltage", "V_DC", "PACK_VOLTAGE", "Accumulator nominal voltage.",
  None, "V", "CURRENT CAR (design decision)",
  "Nothing (design value)", "Accumulator design",
  "Sets motor peak power. 4 motors × ~20 kW @ 380 V = 80 kW = the rules cap exactly — sized for the 4WD end state."),
 ("Motor peak power", "P_pk", "MOTOR_POWER_PEAK", "Peak mechanical power of ONE motor at OUR pack voltage.",
  None, "W", "DERIVED (kit curves scaled to 380 V)",
  "Actual peak power at 380 V", "Read the 380 V curve in the AMK manual; dyno pull once the pack exists",
  "Caps torque above ~17 m/s (37 mph): at 40 mph only 256 of 273 N·m is available. 2-rear build tops out ~40 kW."),
 ("Motor max speed", "n_max", "MOTOR_MAX_SPEED", "Maximum motor shaft speed.",
  None, "rad/s (20,000 rpm)", "CURRENT CAR (AMK kit datasheet)",
  "Nothing (datasheet value)", "AMK kit manual",
  "= ~36 m/s (80 mph) ground speed at 13:1 — comfortably above the 40 mph envelope, not the limiter."),
 ("Wheel spin inertia", "I_w", "I_WHEEL",
  "Spin inertia of one rear corner: tire + rim + hub + brake rotor + motor rotor × gr².",
  None, "kg·m²", "DERIVED estimate ⚠ replace with CAD",
  "Rotating inertia of one corner", "CAD mass properties + rotor J from the exact AMK motor sheet",
  "Sets how fast an unloaded wheel spins up — the s-diff's whole problem. Controller gains are sensitive to it: retune when it changes."),
 ("Total power cap", "P_max", "POWER_CAP_TOTAL", "FSAE EV rules limit on total drive power (EV.4.2).",
  None, "W", "RULES VALUE",
  "Nothing (rulebook)", "Confirm in the current-year rulebook",
  "Hard ceiling. Sim applies it to mechanical power (no drivetrain losses modeled) — real car hits it ~10–15% earlier."),
 ("Regen speed cutoff", "—", "REGEN_SPEED_CUTOFF", "Below this speed no negative (regen) torque is commanded.",
  None, "m/s (≈5 km/h)", "RULES VALUE",
  "Nothing (rulebook)", "Confirm in the current-year rulebook",
  "Rules restrict regen near standstill; also keeps the sim's low-speed slip math out of its untrustworthy zone."),
]))

P.append(("AERO", [
 ("Lift coefficient", "C_L", "CL_COEFF", "Downforce coefficient, referenced to the frontal area.",
  None, "—", "FROM REPORT (old car's aero)",
  "Current car's downforce coefficient", "CFD, then straight-line coast-down / load-cell validation",
  "Downforce = ½ρ·C_L·A·v² — free grip that grows with speed²."),
 ("Drag coefficient", "C_D", "CD_COEFF", "Aerodynamic drag coefficient, same reference area.",
  None, "—", "FROM REPORT (old car's aero)",
  "Current car's drag coefficient", "CFD + coast-down test",
  "Sets top-speed force budget and the torque needed to hold speed in every maneuver."),
 ("Frontal area", "A_F", "FRONTAL_AREA", "Aerodynamic reference area.",
  None, "m²", "FROM REPORT",
  "Projected frontal area", "CAD projection",
  "Only the products C·A enter the physics — keep coefficient and area consistent with each other."),
 ("ClA (computed)", "C_L·A", "CLA", "The downforce product the sim actually uses.",
  None, "m²", "DERIVED", "—", "= C_L × A_F, never edited directly", "—"),
 ("CdA (computed)", "C_D·A", "CDA", "The drag product the sim actually uses.",
  None, "m²", "DERIVED", "—", "= C_D × A_F, never edited directly", "—"),
 ("Aero balance", "%aero_f", "AERO_BALANCE_FRONT", "Fraction of total downforce landing on the FRONT axle.",
  None, "0–1", "PLACEHOLDER ⚠ pure guess",
  "Center-of-pressure position", "CFD (aero team) — ask for CP height too",
  "Moves grip between axles as speed rises — shapes high-speed balance directly. One of the last pure guesses outside tires."),
]))

P.append(("STEERING GEOMETRY", [
 ("Ackermann fraction", "%A", "ACKERMANN_FRACTION",
  "How much of the geometric inner/outer steer split the linkage delivers (0 = parallel, 1 = full Ackermann).",
  None, "—", "PLACEHOLDER (parallel) ⚠ RWheel curve needed",
  "The steering curve: both wheels' angles vs handwheel angle",
  "From the steering chart: LWheel curve is on file, need the RWheel polynomial from the same chart",
  "±4°/wheel at the 23° full lock; proven to flip whether s-diff-only survives the hairpin (runs 007 vs 008)."),
]))

P.append(("SENSORS & VCU — what the controller is allowed to know (sensors.py)", [
 ("VCU loop rate", "—", "VCU_RATE_HZ", "How often sensors are sampled and torque is recomputed (physics runs at 4 kHz regardless).",
  None, "Hz", "PLACEHOLDER ⚠ ask software team", "The real VCU loop rate", "Software team",
  "Gains proven stable at 100 Hz; make it match reality before quoting controller performance."),
 ("Steering map A0/A1/A2", "y=A0+A1x+A2x²", "STEER_MAP_A0/A1/A2",
  "Handwheel angle → road-wheel angle, the team chart's LWheel curve (centered, odd-extended).",
  "−0.0797 / 0.31 / −8.44E-04", "deg→deg", "CURRENT CAR (team chart)",
  "The steering curve incl. RWheel", "Steering chart / kinematics sweep",
  "How the VCU turns the SAS reading into a road-wheel angle. Full lock 23° ≈ 103° handwheel."),
 ("SAS quantization", "—", "SAS_QUANT_DEG", "Steering-angle sensor resolution at the handwheel.",
  None, "deg/LSB", "PLACEHOLDER", "Sensor spec", "Datasheet of the chosen SAS", "Steering resolution the VCU actually sees."),
 ("WSS quantization", "—", "WSS_QUANT_RPM", "Motor-speed resolution over CAN (WSS = motor resolver ÷ planetary ratio).",
  None, "rpm/LSB", "PLACEHOLDER", "AMK CAN speed resolution", "AMK CAN spec",
  "1 motor rpm ≈ 1.8 mm/s ground speed — effectively noise-free wheel speed."),
 ("IMU gyro noise / bias", "σ, b", "IMU_GYRO_NOISE_STD / _BIAS", "Yaw-gyro 1σ noise and constant bias.",
  "0.3 / 0.1 °/s", "rad/s", "PLACEHOLDER", "IMU datasheet numbers", "Chosen IMU's spec sheet",
  "Goes straight into the TV feedback — sets how clean the yaw control can be."),
 ("IMU accel noise", "σ", "IMU_ACCEL_NOISE_STD", "Accelerometer 1σ noise (ax, ay).",
  None, "m/s²", "PLACEHOLDER", "IMU datasheet", "Spec sheet", "Feeds future estimation (vx observer, validation logging)."),
 ("VCU gyro low-pass", "f_c", "IMU_LPF_HZ", "First-order filter the VCU applies to the gyro before feedback.",
  None, "Hz", "PLACEHOLDER", "—", "Software choice; tune against noise",
  "Trade: more filtering = less dither but more phase lag in the yaw loop."),
 ("APPS quantization", "—", "APPS_QUANT_PCT", "Accelerator pedal position resolution. Pedal map: 100% = full axle torque.",
  None, "%/LSB", "PLACEHOLDER", "APPS sensor spec", "Datasheet", "Torque request granularity."),
 ("BPS range / quant / actuated", "—", "BPS_RANGE_BAR / _QUANT_BAR / _ACTUATED_BAR",
  "Brake pressure sensor range, resolution, and the pressure that counts as 'braking' for the rules check.",
  "100 / 0.5 / 3 bar", "bar", "PLACEHOLDER", "BPS spec + brake system pressures",
  "Brake team: line pressure at threshold/full braking", "Defines braking for regen mapping and EV.4.7."),
 ("Max regen torque", "—", "T_REGEN_MAX", "Total rear-axle regen torque at full brake pressure (mechanical brakes NOT modeled).",
  None, "N·m", "PLACEHOLDER ⚠ battery charge limit sets this", "Allowed charge current → torque",
  "Accumulator team: max charge C-rate", "Caps how much 'braking' the sim's BPS can command."),
 ("Plausibility thresholds", "—", "PLAUS_APPS_CUT / _RESTORE", "FSAE EV.4.7: >25% APPS while braking cuts power until APPS <5%.",
  "25 / 5", "%", "RULES VALUE", "—", "Rulebook (confirm current year)",
  "Implemented and unit-tested (verify I3); pedal_check maneuver demonstrates it."),
 ("Sensor noise seed", "—", "SENSOR_SEED", "Seed for the noise generator — runs repeatable, all configs see identical noise.",
  None, "—", "NUMERICAL GUARD", "—", "—", "Fair comparisons and reproducible runs."),
]))

P.append(("LOAD TRANSFER", [
 ("Front lateral-transfer fraction", "—", "LAT_TRANSFER_FRAC_FRONT",
  "Share of total lateral load transfer reacted by the FRONT axle (roll-stiffness split).",
  None, "0–1", "FROM REPORT (spring-rate ratio only) ⚠ crude",
  "Front vs rear roll stiffness", "Suspension team: roll rates incl. ARBs, motion ratios, roll-center heights",
  "Which axle eats the load transfer = which axle loses grip first — trades understeer/oversteer at the limit."),
]))

P.append(("TIRES — TTC ROUND 9 FIT (R20 compound, 2026-08-31) — surrogate sizes; road µ = belt × 0.67 pending skidpad", [
 ("Belt→road grip scale", "—", "TIRE_MU_ROAD_SCALE",
  "Calspan's belt grips harder than asphalt; peak µ scaled by this for road prediction.",
  None, "—", "PLACEHOLDER ⚠ skidpad measures this", "Steady-state skidpad lateral g",
  "Skidpad test vs sim prediction", "Multiplies EVERY grip number — the single biggest absolute-accuracy lever left."),
 ("Peak longitudinal friction", "µ₀x", "TIRE_MU0_LONG",
  "Drive/brake grip peak — fitted 9% below lateral; combined slip caps on the friction ELLIPSE.",
  None, "—", "TTC FIT R9", "—", "tire_fit.py on Round 9 drive/brake runs",
  "Sets wheelspin threshold and traction capability."),
 ("Peak friction coefficient", "µ₀", "TIRE_MU0", "Peak grip per newton of load, at the nominal load.",
  None, "—", "TTC FIT R9",
  "Peak µ at several loads", "TTC data fit (if access ever appears) or skidpad testing",
  "THE grip number. Max lateral g, spin thresholds, everything — the single most important number in the sim."),
 ("Load sensitivity", "s_µ", "TIRE_S_MU", "How fast the grip COEFFICIENT falls as load rises.",
  None, "—/100% load", "TTC FIT R9",
  "µ at low vs high load", "Same TTC fit",
  "The reason load transfer costs grip and torque placement matters at all. Zero would make TV pointless."),
 ("Nominal tire load", "Fz_nom", "TIRE_FZ_NOM", "The load at which µ₀ is defined (≈ one static corner weight).",
  None, "N", "DERIVED (follows the masses automatically)",
  "—", "= (m_car + m_driver)·g / 4", "Reference point for the load-sensitivity line."),
 ("Front cornering stiffness", "c_α,f", "TIRE_C_ALPHA_FRONT",
  "Lateral force per radian of slip angle, per newton of load (front tire).",
  None, "1/rad", "TTC FIT R9",
  "Slope of Fy vs slip angle at small angles", "TTC cornering sweep",
  "Front-axle responsiveness; with rear stiffness sets the understeer gradient K_us the TV reference uses."),
 ("Rear cornering stiffness", "c_α,r", "TIRE_C_ALPHA_REAR", "Same, rear tire.",
  None, "1/rad", "TTC FIT R9", "Same, rear", "TTC cornering sweep",
  "Rear grip slope — stability side of the understeer gradient."),
 ("Longitudinal slip stiffness", "c_κ", "TIRE_C_KAPPA",
  "Drive/brake force per unit slip ratio, per newton of load.",
  None, "1/unit slip", "TTC FIT R9",
  "Slope of Fx vs slip ratio", "TTC drive/brake sweep",
  "Sets wheel-spin dynamics speed — the s-diff's plant. Gains retune if it moves."),
 ("Lateral shape factor", "C_y", "TIRE_SHAPE_C_LAT", "Magic Formula: how sharply lateral force peaks.",
  None, "—", "TTC FIT R9", "Curve shape near/past the peak", "TTC fit",
  "With E_y, sets where the curve peaks (currently 11.6° — real slicks ~6–10°) and how it lets go."),
 ("Lateral curvature factor", "E_y", "TIRE_CURV_E_LAT", "Magic Formula: fall-off past the lateral peak.",
  None, "—", "TTC FIT R9", "Post-peak behavior", "TTC fit",
  "Gentle vs snappy breakaway — currently more forgiving than a real slick."),
 ("Longitudinal shape factor", "C_x", "TIRE_SHAPE_C_LONG", "Same idea, drive/brake direction.",
  None, "—", "TTC FIT R9", "Curve shape", "TTC fit",
  "Sets the peak slip ratio (~0.10) — also the replay's red-wheel spin threshold."),
 ("Longitudinal curvature factor", "E_x", "TIRE_CURV_E_LONG", "Fall-off past the longitudinal peak.",
  None, "—", "TTC FIT R9", "Post-peak behavior", "TTC fit",
  "How violently a spinning wheel loses force — feeds the wheelspin runaway."),
]))

P.append(("CONTROLLER GAINS — tuned in-sim; retune whenever vehicle/tire numbers move", [
 ("s-diff proportional gain", "Kp", "KP_SDIFF", "Torque per rad/s of wheel-speed-difference error.",
  None, "N·m/(rad/s)", "TUNED (sim)", "—", "Re-tune in sim: oscillation = too hot, sluggish = too cold",
  "How hard the s-diff fights wheel-speed error."),
 ("s-diff integral gain", "Ki", "KI_SDIFF", "Torque per accumulated rad of error.",
  None, "N·m/rad", "TUNED (sim)", "—", "Same", "Removes steady error (e.g. constant-radius corners)."),
 ("s-diff integral clamp", "—", "I_SDIFF_MAX", "Anti-windup limit on the integral term.",
  None, "N·m", "TUNED (sim)", "—", "Same", "Stops the integral charging up during saturation and overshooting after."),
 ("s-diff torque-split clamp", "ΔT_max", "DT_SDIFF_MAX",
  "Hard cap on how much torque the s-diff may move between wheels.",
  None, "N·m", "TUNED (sim) — SAFETY-CRITICAL",
  "—", "Sized in sim against the spin scenario",
  "Without it the s-diff dumps torque onto the loaded outer tire near the limit and power-oversteers the car. Proven in sim."),
 ("TV proportional gain", "Kp", "KP_TV", "Yaw moment per rad/s of yaw-rate error.",
  None, "N·m/(rad/s)", "TUNED (sim)", "—", "Re-tune in sim", "How hard TV chases the yaw reference."),
 ("TV integral gain", "Ki", "KI_TV", "Yaw moment per accumulated rad of error.",
  None, "N·m/rad", "TUNED (sim)", "—", "Same", "Steady-state yaw accuracy in long corners."),
 ("TV integral clamp", "—", "I_TV_MAX", "Anti-windup limit on the TV integral.",
  None, "N·m", "TUNED (sim)", "—", "Same", "Bounds windup when the reference is unreachable (e.g. full lock at speed)."),
 ("Yaw-moment clamp", "Mz_max", "MZ_MAX", "Cap on total commanded yaw moment.",
  None, "N·m", "TUNED (sim)", "—", "Same", "Keeps TV authority bounded."),
 ("Reference aggressiveness", "—", "AY_FRAC",
  "The yaw-rate reference is capped at this fraction of friction-limited lateral accel.",
  None, "0–1", "TUNED (sim)", "—", "Same",
  "0.95 = ask for 95% of what grip allows — margin against an optimistic reference."),
]))

P.append(("NUMERICAL", [
 ("Low-speed guard", "v_ε", "V_EPS", "Minimum speed used in slip-angle/slip-ratio denominators.",
  None, "m/s", "NUMERICAL GUARD", "—", "—",
  "Below ~0.5 m/s wheel-slip math is untrustworthy — launches are out of scope until reworked."),
]))


# ──────────────────────────────────────────────────────────────────────
# Tab 2 — computed signals
# ──────────────────────────────────────────────────────────────────────
S = [
 ("Slip angle", "α", "alpha (vehicle.py)",
  "Angle between where a wheel POINTS and where its contact patch actually TRAVELS. The input that generates lateral force.",
  "Computed every step per wheel: α = −atan2(v_lateral, v_forward) at the contact patch",
  "Not directly measurable cheaply — estimated from IMU + steering angle; optical sensors (Kistler) exist but cost more than the car",
  "Every tire always runs some slip angle — that IS how it grips. Past ~11.6° (placeholder peak) more angle = less force."),
 ("Slip ratio", "κ", "kappa (vehicle.py)",
  "How much faster the wheel SURFACE moves than the ground: κ = (ω·r_w − v)/v. Positive = wheelspin side.",
  "Computed per driven wheel from its spin state and contact-patch speed",
  "Motor resolver speed (AMK reports it over CAN) ÷ gear ratio vs estimated ground speed",
  "The s-diff's whole world. Past the peak (~0.10) the wheel runs away — that's wheelspin, and it emerges from the ODE, no if-statement."),
 ("Wheel spin speeds", "ω_RL, ω_RR", "states 7–8",
  "Rear wheel rotational speeds — real states with their own differential equation I·ω̇ = T − r·Fx.",
  "Integrated by RK4 like everything else",
  "AMK motor resolvers over CAN (÷ gear ratio) — free, accurate, fast",
  "The signal the real s-diff will actually control. Sensor already exists in the motors."),
 ("Yaw rate", "r", "state 6",
  "How fast the car rotates about vertical. Left turn positive.",
  "Integrated from Iz·ṙ = ΣMz",
  "IMU gyro (any automotive-grade IMU; the VCU needs one anyway)",
  "The signal TV controls. Its sensor is the one non-negotiable hardware item for TV."),
 ("Body velocities", "vx, vy", "states 4–5",
  "Forward and sideways speed of the CG in the car's own frame.",
  "Integrated from the force balance",
  "vx: wheel speeds + IMU fusion (or GPS); vy: estimated (observer) — no cheap direct sensor",
  "vx feeds the yaw reference and slip math; vy is why sideslip needs estimation on the real car."),
 ("Body sideslip", "β", "beta (logged)",
  "Angle between where the car POINTS and where it TRAVELS: β = atan(vy/vx). Big β = the rear stepping out.",
  "Computed from vy/vx",
  "Estimated (IMU + model observer); optical ground-speed sensors measure it directly but are exotic",
  "The honest 'is the car sliding' number. TV holding yaw while β grows to 22° = a drift (found in run 007)."),
 ("Vertical loads", "Fz (×4)", "wheel_loads() (vehicle.py)",
  "Weight on each tire right now: static + downforce + longitudinal & lateral load transfer.",
  "Algebraic from ax, ay, speed each step (quasi-static, 3-iteration loop)",
  "Suspension load cells or pushrod strain gauges (nice-to-have); corner scales give the static part",
  "Grip is proportional to load (minus load sensitivity) — every force the tire model produces starts from Fz."),
 ("Tire forces", "Fx, Fy (×4)", "tire_forces() (vehicle.py)",
  "What each contact patch pushes on the ground: drive/brake (Fx) and cornering (Fy).",
  "Magic Formula from (α, κ, Fz), capped by the friction circle √(Fx²+Fy²) ≤ µFz",
  "Not directly measurable — inferred from IMU accelerations and wheel torques",
  "The only place the car touches the world. Everything else is bookkeeping around these eight numbers."),
 ("Friction utilization", "—", "verify.py audit",
  "How much of a tire's total grip budget is in use: √(Fx²+Fy²)/(µFz). 1.00 = on the limit.",
  "Computed in the audit/replay from the forces",
  "Inferred (same as forces)",
  "The corner-exit test runs the rear axle at 1.00 — meaning results live on the guessed part of the tire curve."),
 ("Accelerations", "ax, ay", "logged",
  "Specific forces on the body (what an accelerometer strapped to the car would read).",
  "ΣF/m each step",
  "IMU accelerometer — cheap, already needed",
  "ay·vx-vs-yaw-rate cross-checks (ay = r·vx steady state) are how sim vs real-car validation will start."),
 ("Yaw moment", "Mz", "derivatives() (vehicle.py)",
  "Net twisting moment about the CG from all eight tire force components.",
  "Mz = Σ(x·Fy − y·Fx) over the four wheels",
  "Not measurable directly — its EFFECT (yaw acceleration) is, via the gyro",
  "The quantity TV commands. (track/2)·ΔFx of it comes from the torque split."),
 ("Wheel-speed-difference target", "Δω_target", "controllers.py",
  "The rear-wheel speed difference corner geometry requires: r·track/r_wheel.",
  "Computed from measured yaw rate each controller update",
  "Derived from the gyro on the real car",
  "The s-diff's setpoint: allow exactly this difference, fight spin beyond it."),
 ("Yaw-rate reference", "r_ref", "controllers.py",
  "What the driver's inputs IMPLY they want: bicycle model of steer + speed, capped by friction.",
  "r_ref = vx·δ/(L + K_us·vx²), |r_ref·vx| ≤ 0.95·µ·g_effective",
  "Computed in the VCU from steering sensor + speed estimate",
  "TV steers the car toward this. Known gap: it has no sideslip term — at full lock it will happily command a drift."),
 ("Torque split", "ΔT", "controllers.py",
  "The left/right torque difference: ΔT = ΔT_sdiff + ΔT_tv, then limits applied.",
  "Controller output every update",
  "Commanded over CAN to the AMK inverters; they report actual torque back",
  "The single actuator this whole project exists to command."),
 ("Wheel torques", "T_RL, T_RR", "controllers.py output",
  "Final per-wheel torque commands after every limit (peak, regen, power caps).",
  "Base ± ΔT/2, then the limits chain",
  "AMK inverters accept torque setpoints and report actuals over CAN",
  "What actually gets sent. The limits chain can silently shrink ΔT at high speed — log requested vs delivered on the car."),
 ("Total drive power", "P", "logged",
  "Mechanical power at the wheels: T_RL·ω_RL + T_RR·ω_RR.",
  "Computed from torques × speeds",
  "Rules energy meter measures the electrical version at the accumulator",
  "The 80 kW rules cap lives here (sim: mechanical; real: electrical — real car hits it ~10–15% sooner)."),
]

# ──────────────────────────────────────────────────────────────────────
# Tab 3 — the tests: what each measures, how, and the numbers we want
# ──────────────────────────────────────────────────────────────────────
T = [
 ("Step steer", "5° step at 15 m/s, torque holds speed",
  "How the car answers a sudden steering input: rise time, overshoot, steady-state accuracy. THE test for tuning TV gains (Kp = response speed, Ki = steady error).",
  "Straight running, then a fast (~0.1 s) ramp to a fixed steer angle, held to the end.",
  "Steady-state yaw rate ON the reference → yaw RMSE (run 015: TV 0.011 vs open 0.034 rad/s = 3× precision). NO overshoot/oscillation in the yaw trace (oscillation = gains too hot). Sideslip β < ~1°.",
  "TV configs must win yaw RMSE; all configs stable."),
 ("Corner exit (power on)", "8° corner at 10 m/s, throttle ramp to 45% of peak, unwind",
  "Traction management — the s-diff's reason to exist: put power down mid-corner without spinning the unloaded inner wheel or the car.",
  "Turn in, steady corner, then throttle ramps in exactly when the inner rear is lightest, wheel unwinding on exit.",
  "MAX SLIP RATIO κ on the inner rear — the headline. Must stay under the tire peak κ≈0.10. At the 23° envelope test: open diff κ=67 (violent spin) vs s-diff κ=0.07 — that single pair of numbers IS the s-diff justification. Also: no spin flag, yaw tracked while power is down.",
  "s-diff configs contain κ; open shows the failure. NOTE: with real R20 grip the 45% default barely stresses the axle — run --corner-throttle 65 (or the 23° envelope) to separate configs."),
 ("Slalom (sine steer)", "±4° at 0.5 Hz, 15 m/s",
  "Transient handling through repeated direction changes: phase lag, left/right symmetry, and whether errors GROW cycle-to-cycle (the spin precursor).",
  "Sine-wave steering, amplitude faded in over 1 s, constant drive torque.",
  "Yaw RMSE through reversals; β bounded and symmetric; amplitude NOT growing cycle to cycle (growth = impending spin — exactly how open & s-diff-only died at full lock); Δω flips sign cleanly each reversal.",
  "TV configs track through reversals; nothing diverges at the default point."),
 ("Pedal check (APPS/BPS)", "APPS 60% → overlap with brake → release → regen",
  "The pedal→torque chain and the FSAE EV.4.7 plausibility cut. Rules compliance, not handling — pass/fail.",
  "Straight line: throttle, then brake pressure WHILE throttle held (illegal overlap), release both, then brake alone.",
  "Wheel torque EXACTLY 0 through the whole overlap (run 015: 0.000 ✓); cut latched until APPS < 5%; regen negative and inside T_REGEN_MAX; vx estimate tracks through it.",
  "Hard pass/fail — any nonzero torque in the overlap is a rules violation."),
 ("Envelope: max steer", "23° road-wheel (CURRENT CAR)",
  "The physical steering lock — robustness test point, not an operating point (at 40 mph the friction cap saturates the yaw reference at ~5°).",
  "Run any maneuver at 23° via ./run.sh prompts or flags.",
  "Full-lock hairpin: the s-diff κ-containment headline. Full-lock slalom at 40 mph: which configs survive (TV yes / open & s-diff-only spin). Found the TV-drifts-when-saturated design gap (FINDINGS 8-bis).",
  "TV configs survive; β stays bounded."),
 ("Envelope: top speed", "40 mph = 17.9 m/s (CURRENT CAR)",
  "Endurance top expected speed.",
  "Speed test point for step/slalom.",
  "Above ~17 m/s the 20 kW motor power cap trims torque (256 of 273 N·m at 40 mph) — check commands respect it.",
  "—"),
 ("Changing test points", "./run.sh asks · flags · maneuvers.py defaults",
  "Every run RECORDS its test values; changing one auto-labels the run and flags 'metrics not comparable' vs the previous run.",
  "—", "—", "—"),
]


# ──────────────────────────────────────────────────────────────────────
# build the workbook
# ──────────────────────────────────────────────────────────────────────
def style_header(ws, row, cols):
    for c in range(1, cols + 1):
        cell = ws.cell(row=row, column=c)
        cell.font = Font(bold=True, color="FFFFFFFF", size=11)
        cell.fill = HDR_FILL
        cell.alignment = Alignment(vertical="center", wrap_text=True)
        cell.border = BORDER


def add_sheet(wb, title, headers, widths):
    ws = wb.create_sheet(title)
    for i, (h, w) in enumerate(zip(headers, widths), start=1):
        ws.cell(row=1, column=i, value=h)
        ws.column_dimensions[get_column_letter(i)].width = w
    style_header(ws, 1, len(headers))
    ws.freeze_panes = "A2"
    ws.auto_filter.ref = f"A1:{get_column_letter(len(headers))}1"
    return ws


wb = Workbook()
wb.remove(wb.active)

# ---- tab 1
H1 = ["Parameter", "Symbol", "Name in the code", "What it is / what it simulates",
      "Value now", "Units", "Status (where the number comes from)",
      "What we measure", "How we measure it", "Why it matters"]
W1 = [26, 9, 26, 46, 13, 16, 34, 30, 38, 48]
ws = add_sheet(wb, "Car parameters", H1, W1)

r = 2
for section, rows in P:
    ws.cell(row=r, column=1, value=section)
    ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=len(H1))
    c = ws.cell(row=r, column=1)
    c.font = Font(bold=True, size=10.5, color=BLACK)
    c.fill = SEC_FILL
    r += 1
    for (name, sym, code, what, val, units, status, meas, how, why) in rows:
        if val is None:
            val = fmt(getattr(cd, code))
        green = any(t in status for t in GREEN_TAGS)
        vals = [name, sym, code, what, val, units, status, meas, how, why]
        for ci, v in enumerate(vals, start=1):
            cell = ws.cell(row=r, column=ci, value=v)
            cell.alignment = Alignment(vertical="top", wrap_text=True)
            cell.border = BORDER
            color = BLACK
            if ci == 5:                     # the value column
                color = GREEN if green else BLACK
                cell.font = Font(color=color, bold=green)
            elif ci == 7:
                color = GREEN if green else (WARN if "PLACEHOLDER" in status or "⚠" in status else BLACK)
                cell.font = Font(color=color, size=10)
            else:
                cell.font = Font(color=BLACK, size=10)
        r += 1

# legend
r += 1
ws.cell(row=r, column=1, value="Legend:").font = Font(bold=True)
ws.cell(row=r, column=2, value="GREEN value = confirmed for the CURRENT car/driver (or MEASURED). "
        "Black = report car, guess, rules, or sim-tuned. Orange status = placeholder/needs attention.")
ws.cell(row=r, column=2).font = Font(color=BLACK, size=10)
ws.merge_cells(start_row=r, start_column=2, end_row=r, end_column=len(H1))

# ---- tab 2
H2 = ["Signal", "Symbol", "Where in the code", "What it is",
      "How the sim gets it", "How we'd measure it on the real car", "Why it matters"]
W2 = [24, 12, 24, 48, 42, 44, 52]
ws2 = add_sheet(wb, "Sim signals", H2, W2)
r = 2
for row in S:
    for ci, v in enumerate(row, start=1):
        cell = ws2.cell(row=r, column=ci, value=v)
        cell.alignment = Alignment(vertical="top", wrap_text=True)
        cell.border = BORDER
        cell.font = Font(color=BLACK, size=10)
    r += 1

# ---- tab 3
H3 = ["Test", "Current test point", "What it measures", "How it works",
      "The numbers we want (and what good looks like)", "Expected outcome"]
W3 = [22, 30, 44, 40, 60, 40]
ws3 = add_sheet(wb, "Tests", H3, W3)
r = 2
for row in T:
    green = "CURRENT CAR" in row[1]
    for ci, v in enumerate(row, start=1):
        cell = ws3.cell(row=r, column=ci, value=v)
        cell.alignment = Alignment(vertical="top", wrap_text=True)
        cell.border = BORDER
        if ci == 2:
            cell.font = Font(color=GREEN if green else BLACK, bold=green, size=10)
        else:
            cell.font = Font(color=BLACK, size=10)
    r += 1

wb.save(OUT)
print(f"wrote {OUT}")
print("Import to Google Sheets: drag the file into drive.google.com — "
      "colors and layout carry over.")
