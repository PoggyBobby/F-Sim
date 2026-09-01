# Scrutiny report — 2026-08-30

The brief: assume everything is wrong — the math, the physics — and try to
prove it isn't. The instrument is `verify.py`: 69 machine-checked comparisons
against things *independent* of the sim's own code path (closed-form
solutions, algebraic identities, symmetries, textbook models). Re-run it any
time with:

    .venv/bin/python verify.py        # exit 0 = all hard checks pass

**Result: 69/69 pass.** Two checks failed on the first run — both turned out
to be errors in the *checks* (one used the CG speed where the sim correctly
uses per-wheel contact-patch speed; one had a double-negative in the
hand-expanded yaw moment). The fixes are commented in `verify.py` so the
process is auditable. No error was found in the sim's math or physics.

That does NOT mean the sim is "right" — it means it is internally consistent
and agrees with the standard models it claims to implement. What it can and
cannot claim, and the real issues found, are below.

---

## What is now PROVEN (with the check that proves it)

| Claim | Evidence |
|---|---|
| Tire small-slip stiffness is exactly `c·Fz` (the B = c/(C·µ) construction) | A1: numeric derivative matches to <0.1% at 3 loads |
| Tire peak force is exactly `µ(Fz)·Fz`; friction circle never exceeded | A2, A3: 2 500-state slip grid, max utilization 1.000000 |
| Load sensitivity: µ falls with load, total force still rises | A4 |
| ΣFz = m·g + downforce **exactly**, for any (vx, ax, ay) and throughout every maneuver | B1 (500 random states, err 3×10⁻¹⁶), G1 (in-run) |
| Static split = `WEIGHT_FRACTION_FRONT`; braking loads front; left turn loads outer; total lateral transfer = m·ay·h/track | B2, B3 |
| Wheel lift clamps to 0 N, never negative | B4 |
| Slip angles match the textbook `α_f = δ − (v_y + a·r)/v_x` (small angles) | C1 |
| Slip ratios use per-wheel contact speed incl. the r·y term | C2 |
| s-diff Δω target = independent wheel-path derivation; outer wheel faster | C3 |
| Yaw-moment arms have the right signs | C4 (hand-expanded) |
| Coast-down matches the closed-form drag ODE **including wheel rotational KE** (m_eff = m + 2I_w/r²) | D1: 12.934 vs 12.934 m/s after 3 s |
| No left/right sign error anywhere in the chain: mirrored steer gives an exactly mirrored car, controllers on | D2: asymmetry 0.0 (to machine precision) |
| Steady cornering satisfies ay = r·vx | D3: worst 0.037 m/s² at 11.3 m/s² |
| RK4 at dt = 0.25 ms is converged (halving dt twice moves nothing) | D4: drift 1.1×10⁻³ → 5.6×10⁻⁴, halving as O(dt) input-resolution predicts |
| The two-track model reproduces an independently derived linear bicycle model to +0.1% at low g | E1 (3 speed/steer points) |
| The TV yaw-rate reference is within +0.5% of the true steady state at 0.5 g | E2 |
| Torque clip preserves the yaw split; regen floor holds; per-motor and 80 kW caps enforce; Mz↔ΔT is an exact round trip; r_ref honors the friction cap | F1–F9 |
| In every standard maneuver: friction circle, torque caps, power cap, motor speed all respected; µ floor never engages; no wheel lifts (min Fz = 368 N) | G2–G6 |
| Open-diff wheel speeds settle near the kinematic Δω on their own (end-to-end integrator sanity) | G7 |
| Gains are stable at a realistic 100 Hz VCU rate, not just the simulated 4 kHz | H1 (yaw RMSE actually −8% at 100 Hz) |

---

## Findings — work for later, ranked

### 1. ✅ RESOLVED 2026-08-31 — tires fitted from TTC Round 9 (surrogates)
Real Calspan R20 data via `tire_fit.py`; friction ellipse + separate µx
added (fitted µx/µy = 0.91); lateral peak lands at 9.8° slip. Residual
unknowns: belt→road scale ×0.67 (skidpad validates), 12 psi assumed, size
surrogates (±6% µ spread bounds the effect). Original text kept below.

### 1-old. Tire data 100% placeholder (superseded)
Known, but the scrutiny quantified a new angle: the placeholder lateral curve
peaks at **11.6° slip angle** (A2); real FSAE slicks peak nearer 6–10°. The
placeholder tire is more progressive/forgiving than reality, so breakaway in
the sim is gentler than the real car's. The corner-exit comparison runs the
rear axle at **utilization 1.00** (G6) — i.e. the headline results live
exactly on the part of the tire curve we know least. **No controller
conclusion beyond direction-of-effect should be quoted until the TTC fit.**
(Peak slip ratio 0.104 is plausible; it is also the replay's spin threshold.)

### 2. At the torque floor, the limiter *manufactures* thrust (behavior, MEDIUM)
`controllers._apply_limits` preserves the yaw split by shifting base torque.
At the **upper** cap that sacrifices thrust — correct, and F2 proves it. But
at the **lower** bound (regen floor, low/zero throttle) the same shift adds
thrust the driver didn't request: F3 shows a request of base 5 N·m with
ΔT −80 delivered as 80/0 N·m — 70 N·m of self-throttle. Window today: TV
active below the 1.4 m/s regen cutoff (cranked-wheel launches). Fix later:
shrink |ΔT| at the floor instead of shifting the base up.

### 3. Power caps silently eat yaw authority at speed (behavior, MEDIUM)
Per-motor and 80 kW clamps clip each side independently, after the
split-preserving logic: F7 shows a ΔT request of 120 N·m delivered as
74 N·m at 32 m/s. Real motors do this too — the issue is the sim doesn't
*report* the lost authority (and the TV integral can wind against it;
`I_TV_MAX` bounds the damage). Irrelevant at current test speeds — G4 shows
peak 22 kW — but it will matter for accel/high-speed work. Fix later: log
"ΔT requested vs delivered" and derate symmetrically.

### 4. The 80 kW cap is mechanical, not electrical (scope, MEDIUM for energy work)
The rules cap accumulator power. The sim caps wheel mechanical power with no
drivetrain/inverter efficiency (~0.85–0.92 real). The real car hits the cap
~10–15% earlier than this sim says. Also regen bookkeeping: one wheel
regenerating offsets the other's draw in `P_tot`, which the rules would not.
Harmless for the three handling maneuvers (≤22 kW); wrong tool for
accel-event or energy studies until fixed.

### 5. Rolling resistance omitted (scope, LOW-MEDIUM)
Deliberate scope cut, but sized now: at 15 m/s it would be ~47 N vs ~204 N of
aero drag — a 23% understatement of total road load. Irrelevant to the
left/right-split comparisons (identical in all configs), visible the moment
anyone uses this sim for speed traces or energy. The report even supplies the
coefficient (0.02).

### 6. Drag acts at CG height; no aero pitch moment (scope, LOW)
Load transfer from drag is computed as if drag acts at the CG (no
center-of-pressure height parameter). Sized: ~204 N of drag with a CP 0.2 m
above the CG would shift ~27 N between axles (~2% of an axle load).
`AERO_BALANCE_FRONT` is itself still a placeholder — get CP height from the
aero team along with the balance.

### 7. Controller-rate and actuator realism (scope, LOW — now quantified)
The controller currently updates every physics step (4 kHz). H1 proves the
tune is *robust* down to 100 Hz (metrics essentially unchanged, no spin), so
this is no longer a threat to current conclusions — but the real loop has CAN
latency, inverter torque ramps, and sensor filters, none modeled. When the
VCU team fixes the real rate, make it an explicit parameter (the harness in
`verify.py section H` already does this).

### 8. Reference-model idealizations (model, LOW)
- `K_us` uses static axle loads; downforce raises both axle stiffnesses.
  Quantified: r_ref within +0.5% of true steady state at 0.5 g (E2) — fine.
- The friction cap in `yaw_rate_ref` uses µ₀ average, ignoring load
  sensitivity → slightly optimistic near the limit; `AY_FRAC = 0.95` is
  currently doing the compensating. Revisit with real tire data.

### 8-bis. Yaw-rate-only TV drifts the car when its reference caps (design, MEDIUM — found by the envelope run)
Run 007 (23° full lock at 40 mph, the team's real envelope): TV achieves the
best yaw RMSE of all configs while carrying **22° of body sideslip** — it
hits the capped yaw target by rotating the car past front grip. A yaw-rate
controller has no concept of sideslip; production TV pairs the yaw term with
a beta (stability) term or a beta limiter. Candidate fix when gains are
retuned on real tires. (Same run: at a 23° hairpin the s-diff contains
inner-wheel spin to κ 0.07 where open/TV-only reach κ 67/8 — its clearest
demonstration yet; and at full-lock slalom at 40 mph, configs without yaw
feedback spin while TV configs survive.)

### 8-ter. No Ackermann steering geometry (scope, LOW — matters only at full lock)
Both fronts get the same steer angle (`vehicle.py:105`). Per-wheel slip
angles are still individually correct from the kinematics (r·y terms,
verified C1/C2); what's missing is the rack's geometric toe difference.
Sized: at 5° steer the inner/outer difference is ~±0.2° (negligible); at the
23° full-lock hairpin it is ~±4° per wheel — front-axle detail in run 007 is
approximate (rear-axle conclusions unaffected). Note race cars often run
reduced/anti-Ackermann deliberately. Fix when the suspension team supplies
the real %-Ackermann: one per-wheel steer function + one car_data.py entry.

### 9. Small code smells (cosmetic)
- `controllers._apply_limits`: `for _ in range(1):` is a dead loop.
- `alpha` guard `max(vcx, v_eps)` is asymmetric for reversing (never reached
  in these maneuvers).
- Below ~0.5 m/s the κ denominator floor makes wheel-spin dynamics
  untrustworthy — launches are out of scope; keep them out until this is
  reworked.

### 10. Metric interpretation guardrails (documentation)
- `dw RMSE` *by construction* penalizes TV-only: TV deliberately drives Δω
  away from the kinematic value to make yaw moment. Don't read column-wise
  "TV is worse at diff-ing" — that's the design working.
- A spun run's metrics cover only the survived window; the table flags it,
  but cross-config comparison of a spun row is not apples-to-apples.

### 11. Data risks already on file (unchanged, listed for completeness)
Mass bookkeeping doesn't close (241.7 = sprung + 2×unsprung — see
`car_data.py`); 46% vs 58% front-split contradiction; `I_WHEEL` needs CAD;
gear ratio unconfirmed; aero balance placeholder. All flagged in
`car_data.py` with measurement actions.

---

## What this scrutiny does NOT establish

Internal consistency and textbook agreement — which is what was checked — is
necessary, not sufficient. The sim has **zero experimental validation**: no
skidpad, no accel run, no data logger trace has been compared against it.
The validation ladder from here:

1. TTC tire fit → replaces the entire placeholder tire block (finding #1).
2. Corner scales + driver weigh-in → closes the mass/split contradictions.
3. Skidpad steady-state ay + accel-event time vs sim prediction → first
   real-world anchor for the vehicle model.
4. Logged wheel speeds + yaw rate through a real corner vs replay → first
   anchor for the controller-relevant dynamics.

Until step 3–4, quote the sim for *comparisons and trends* (config A vs
config B under identical assumptions), not absolute numbers.
