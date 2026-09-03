"""Scripted open-loop test maneuvers (steer angle + total torque request).

Each maneuver returns (delta [rad], T_req_total [N·m at the wheels]) as a
function of time. No driver model — inputs are scripted so that runs are
exactly repeatable between controller configurations.

WHAT EACH TEST MEASURES, HOW, AND THE NUMBERS WE WANT: BREAKDOWN.md §6
(the test catalog) — also the "Tests" tab of FSAE-Sim Parameters.xlsx, and
every run's summary.md carries a metrics legend.

REAL ENVELOPE (team, 2026-08-30) — the first real test-envelope numbers:
  * max road-wheel steer angle: 23 deg
  * expected top speed in endurance: ~40 mph = 17.88 m/s
Physics note before testing at that corner of the envelope: at 17.9 m/s the
yaw-rate reference saturates at ~5 deg of steer (friction cap) — full lock
at top speed is a ROBUSTNESS test deep past the grip limit, not an operating
point. The car's answer will be front-axle saturation (push), not more yaw.
"""

import math
from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Maneuver:
    name: str
    slug: str            # filesystem-friendly name
    duration: float      # s
    vx0: float           # m/s initial speed
    inputs: Callable[[float], tuple]   # t -> (delta, T_req_total)
    description: str = ""
    # the knob values this maneuver was built with — recorded into every
    # run's manifest so a run at 20 m/s can never masquerade as one at 15
    params: dict = field(default_factory=dict)
    # optional pedal script t -> (APPS %, BPS bar). When set, the driver's
    # pedals come from here instead of being back-computed from T_req —
    # used by maneuvers that exercise the APPS/BPS chain directly.
    pedals: Callable = None


def _ramp(t, t0, t1, y0, y1):
    """Linear ramp from y0 at t0 to y1 at t1, clamped outside."""
    if t <= t0:
        return y0
    if t >= t1:
        return y1
    return y0 + (y1 - y0) * (t - t0) / (t1 - t0)


def step_steer(delta_deg=5.0, vx0=15.0, t_step=0.5, T_hold=80.0, duration=5.0):
    """Straight running, then a fast steer ramp to a constant angle.
    Classic yaw-response test: shows rise time, overshoot, and steady-state
    yaw rate vs. the reference. T_hold ≈ aero drag + tire induced drag at
    vx0 and ~1.1 g lateral, so speed stays near constant mid-corner."""
    d = math.radians(delta_deg)

    def f(t):
        delta = _ramp(t, t_step, t_step + 0.1, 0.0, d)  # ~fast hand input
        return delta, T_hold

    return Maneuver("step steer", "step_steer", duration, vx0, f,
                    f"{delta_deg:.0f}° step at {vx0:.0f} m/s, torque holds speed",
                    params={"speed_mps": vx0, "steer_deg": delta_deg,
                            "hold_torque_Nm": T_hold, "duration_s": duration})


def corner_exit(delta_deg=8.0, vx0=10.0, t_ramp0=1.5, t_ramp1=3.0,
                T_max_total=None, duration=6.0):
    """Steady cornering, then the driver feeds in power and unwinds the wheel
    on the way out — the s-diff money shot. During the power ramp the
    unloaded inner rear wheel is asked for more force than it has grip; with
    an open (50/50) split it spins up and the rear axle loses lateral grip.

    T_max_total defaults to 45% of the car's peak torque. That sits in the
    window that separates the controllers: enough to overwhelm the UNLOADED
    INNER wheel, not the whole rear axle. Flooring 100% mid-corner spins
    every config alike — this rear-drive car is over-motored relative to
    rear grip, and no left/right split fixes an axle that is past its total
    limit (that needs traction control, deliberately out of scope here)."""
    if T_max_total is None:
        from params import VehicleParams
        T_max_total = 0.45 * 2.0 * VehicleParams().T_wheel_max
    d = math.radians(delta_deg)

    def f(t):
        delta = _ramp(t, 0.1, 0.6, 0.0, d)          # turn in
        delta = _ramp(t, 3.5, 5.5, delta, math.radians(1.0))  # unwind on exit
        T = _ramp(t, t_ramp0, t_ramp1, 20.0, T_max_total)
        return delta, T

    return Maneuver("corner exit (power on)", "corner_exit", duration, vx0, f,
                    f"{delta_deg:.0f}° corner, throttle to {T_max_total:.0f} N·m, unwind",
                    params={"entry_speed_mps": vx0, "corner_steer_deg": delta_deg,
                            "throttle_Nm_total": T_max_total,
                            "duration_s": duration})


def slalom(delta_deg=4.0, freq_hz=0.5, vx0=15.0, T_hold=55.0, duration=6.0):
    """Sinusoidal steering — transient handling / TV tracking through
    repeated direction changes."""
    d = math.radians(delta_deg)

    def f(t):
        amp = _ramp(t, 0.5, 1.5, 0.0, d)   # fade the sine in
        delta = amp * math.sin(2.0 * math.pi * freq_hz * (t - 0.5))
        return delta, T_hold

    return Maneuver("slalom (sine steer)", "slalom", duration, vx0, f,
                    f"±{delta_deg:.0f}° at {freq_hz:.1f} Hz, {vx0:.0f} m/s",
                    params={"speed_mps": vx0, "steer_amplitude_deg": delta_deg,
                            "freq_hz": freq_hz, "hold_torque_Nm": T_hold,
                            "duration_s": duration})


def pedal_check(vx0=10.0, apps_pct=60.0, bps_bar=30.0, duration=6.0):
    """Straight-line APPS/BPS exercise — proves the pedal chain and the
    FSAE EV.4.7 plausibility cut work:
        1.0–2.5 s  throttle (APPS 60%) — car accelerates
        2.5–3.5 s  throttle STILL PRESSED + brake pressure  → plausibility
                   cut must zero the torque (>25% APPS while braking)
        3.5–4.0 s  both released → cut latches until APPS < 5%, then clears
        4.0–5.5 s  brake alone → clean regen deceleration
    No steering. Watch T_RL/T_RR in the run data: torque must be ZERO
    during the overlap window even though APPS commands 60%."""
    def ped(t):
        if 1.0 <= t < 2.5:
            return (apps_pct, 0.0)
        if 2.5 <= t < 3.5:
            return (apps_pct, 20.0)          # the illegal overlap
        if 4.0 <= t < 5.5:
            return (0.0, bps_bar)
        return (0.0, 0.0)

    def f(t):
        return 0.0, 0.0                      # steering straight; torque
                                             # comes from the pedals
    return Maneuver("pedal check (APPS/BPS)", "pedal_check", duration, vx0, f,
                    f"APPS {apps_pct:.0f}% / overlap cut / regen {bps_bar:.0f} bar",
                    params={"speed_mps": vx0, "apps_pct": apps_pct,
                            "bps_bar": bps_bar, "duration_s": duration},
                    pedals=ped)


def all_maneuvers():
    return [step_steer(), corner_exit(), slalom(), pedal_check()]
