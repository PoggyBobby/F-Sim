"""Torque-split controllers: open diff and software differential.

With two independent rear motors there is no mechanical differential — the
"differential" is whatever the software decides the left/right torque split
is. Everything below reduces to choosing T_RL and T_RR from the driver's
per-wheel request T_base = T_req_total / 2.

1) OPEN-DIFF BASELINE (s-diff off): T_RL = T_RR = T_base. This exactly
   reproduces an open differential (equal torque, wheels free to spin at
   different speeds) — including its failure mode: an unloaded inner wheel
   can spin up and dump grip.

2) SOFTWARE DIFFERENTIAL (s-diff): a line-for-line port of sdiff.c from the
   SRE-VCU `S-diff` branch (Andy Van, Sellab Ahmadzai). OPEN LOOP — it reads
   the steering angle only, never the wheel speeds:
        delta_norm = |delta| / delta_max                     (0..1)
        f          = 1 - k_derate * delta_norm, floored at f_min   (shared budget)
        g_in       = 1 - k_inner  * delta_norm,  g_out = 1         (inner extra cut)
        inner side = left if delta > deadband, right if delta < -deadband,
                     neither inside the deadband (f still applies)
        f, g_left, g_right are slew-limited at `rate` per second, then
        T_RL = T_base * clamp(f * g_left,  0, 1),
        T_RR = T_base * clamp(f * g_right, 0, 1).
   Same variable names as the C so the two can be diffed by eye. Two
   deliberate differences: no handwheel→road-wheel conversion (our delta is
   already a road-wheel angle in rad), and slew() takes dt instead of
   assuming the VCU's 10 ms loop. Constants: controllers/python/params.yaml.

   ANGLE UNITS: sdiff.c works in road-wheel DEGREES. The YAML carries the
   same numbers as the C #defines under `unit: deg`, so the loader hands this
   code radians and delta/delta_max matches the C's delta_deg/DELTA_MAX
   exactly. Do not reintroduce a degrees conversion here.

Torque vectoring is parked in torque_vectoring.py (not imported).

After the split, physical limits are enforced: per-motor peak torque, the
regen speed cutoff, per-motor peak power, and the 80 kW total (FSAE EV
rules) cap. When one wheel's command would exceed its torque limit, the
mean torque is shifted so the left/right DIFFERENCE is preserved — total
thrust is sacrificed before the split.

Two update paths exist:
  update(...)               — perfect-state feedback (physics testing)
  update_from_sensors(...)  — the REAL path: consumes sensors.py readings
                              (WSS/IMU/SAS/APPS/BPS at the VCU rate),
                              including the vx estimate and the EV.4.7
                              APPS/BPS plausibility cut. run_sim.py uses
                              this path by default.
"""

import math
from model.params import VehicleParams, TireParams, ControlParams
from model.physical.vehicle import IVX, IR, IWRL, IWRR
from controllers.python.debug import ControllerDebug


def clampf(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def slew(x, target, rate, dt):
    step = rate * dt
    if (target > x):
        return x+step if (x + step < target) else target
    if (target < x):
        return x-step if (x - step > target) else target
    return x


class TorqueSplitController:
    """One controller class; the s-diff term is switched on/off to get the
    two configurations (open / s-diff)."""

    def __init__(self, vp: VehicleParams, tp_front: TireParams,
                 tp_rear: TireParams, cp: ControlParams,
                 sdiff_on: bool, name: str):
        self.vp, self.cp = vp, cp
        self.sdiff_on = sdiff_on
        self.name = name

        self.plaus_cut = False   # EV.4.7 APPS/BPS plausibility latch
        self.f_applied = 1.0     # slewed multipliers — SDiff_new() in sdiff.c
        self.g_left_appl = 1.0
        self.g_right_appl = 1.0

    def reset(self):
        self.plaus_cut = False
        self.f_applied = 1.0
        self.g_left_appl = 1.0
        self.g_right_appl = 1.0

    # ------------------------------------------- update (s_diff_control)
    def update(self, s, delta: float, T_req_total: float, dt: float) -> ControllerDebug:
        vp, cp = self.vp, self.cp
        vx, r = s[IVX], s[IR]
        wRL, wRR = s[IWRL], s[IWRR]
        dbg = ControllerDebug()
        dbg.dw_target = r * vp.track_r / vp.r_wheel

        T_base = T_req_total / 2.0
        if self.sdiff_on:
            delta_norm = clampf(abs(delta / cp.delta_max), 0.0, 1.0)  # delta is a road-wheel angle [rad]
            dbg.delta_norm = delta_norm
            f = clampf(1.0 - cp.k_derate * delta_norm, cp.f_min, 1.0)
            g_in = 1.0 - cp.k_inner * delta_norm
            g_out = 1.0
            if delta > cp.deadband:
                g_left, g_right = g_in, g_out
            elif delta < -cp.deadband:
                g_left, g_right = g_out, g_in
            else:
                g_left = g_right = 1.0
            self.f_applied = slew(self.f_applied, f, cp.rate, dt)
            self.g_left_appl = slew(self.g_left_appl, g_left, cp.rate, dt)
            self.g_right_appl = slew(self.g_right_appl, g_right, cp.rate, dt)
            dbg.f_applied, dbg.g_left_appl, dbg.g_right_appl = self.f_applied, self.g_left_appl, self.g_right_appl
            mult_left  = clampf(self.f_applied * self.g_left_appl,  0.0, 1.0)
            mult_right = clampf(self.f_applied * self.g_right_appl, 0.0, 1.0)
            T_RL = T_base * mult_left
            T_RR = T_base * mult_right
        else:
            T_RL = T_RR = T_base

        T_RL, T_RR = self._apply_limits((T_RL + T_RR) / 2.0, T_RR - T_RL, vx, wRL, wRR)
        dbg.T = (0.0, 0.0, T_RL, T_RR)   # rear drive: the fronts get nothing
        dbg.dT_sdiff = T_RR - T_RL
        return dbg

    # -------------------------------------------------- sensor-driven update
    def update_from_sensors(self, sr, dt: float) -> ControllerDebug:
        """The REAL update path: consumes SensorReadings only (sensors.py).
        The controller knows nothing the VCU wouldn't know:
          * wheel speeds from motor resolvers (÷ planetary ratio)
          * yaw rate from the filtered IMU gyro
          * road-wheel angle estimated from the SAS through the steer map
          * vx ESTIMATED from wheel speeds (no ground-speed sensor exists)
          * torque request from APPS/BPS through the pedal map, gated by
            the FSAE EV.4.7 plausibility check.
        """
        from model.config import cfg

        # pedal map + rules plausibility (EV.4.7): >25% APPS while braking
        # cuts motor power; restored only when APPS falls below 5%.
        braking = sr.bps_bar > cfg.sensors.brake_pressure_sens.actuated_bar
        if sr.apps_pct > cfg.sensors.vcu.plaus_apps_cut and braking:
            self.plaus_cut = True
        elif self.plaus_cut and sr.apps_pct < cfg.sensors.vcu.plaus_apps_restore:
            self.plaus_cut = False

        if self.plaus_cut:
            T_req = 0.0
        elif braking:
            T_req = -cfg.sensors.brake_pressure_sens.t_regen_max * min(sr.bps_bar / cfg.sensors.brake_pressure_sens.range_bar, 1.0)
        else:
            T_req = 2.0 * self.vp.T_wheel_max * sr.apps_pct / 100.0

        # pseudo-state holding ONLY what the sensors gave us
        from model.physical.vehicle import NSTATES, IVX, IR, IWRL, IWRR
        ps = [0.0] * NSTATES
        ps[IVX] = sr.vx_est
        ps[IR] = sr.yaw_rate
        ps[IWRL] = sr.wheel_speed_RL
        ps[IWRR] = sr.wheel_speed_RR
        return self.update(ps, sr.steer_est, T_req, dt)

    # ---------------------------------------------------------- constraints
    def _apply_limits(self, T_base, dT, vx, wRL, wRR):
        vp = self.vp
        T_max = vp.T_wheel_max
        # regen: no negative torque below the rules speed cutoff
        T_min = -T_max if vx > vp.regen_speed_cutoff else 0.0

        # clip the split itself to what the torque range can ever produce
        dT = max(-(T_max - T_min), min(T_max - T_min, dT))
        T_RL = T_base - dT / 2.0
        T_RR = T_base + dT / 2.0

        # shift the base torque to keep the DIFFERENCE (yaw moment) intact
        hi, lo = max(T_RL, T_RR), min(T_RL, T_RR)
        if hi > T_max:
            T_RL -= hi - T_max
            T_RR -= hi - T_max
        elif lo < T_min:
            T_RL += T_min - lo
            T_RR += T_min - lo

        # per-motor peak power (at the wheel: P = T*ω)
        for _ in range(1):
            P_lim = vp.motor_P_peak
            wL = max(abs(wRL), 5.0)
            wR = max(abs(wRR), 5.0)
            T_RL = max(-P_lim / wL, min(P_lim / wL, T_RL))
            T_RR = max(-P_lim / wR, min(P_lim / wR, T_RR))

        # total 80 kW rules cap: scale both if exceeded (simplification —
        # a real VCU would derate while trying to preserve the split)
        P_tot = T_RL * wRL + T_RR * wRR
        if P_tot > vp.P_total_max:
            scale = vp.P_total_max / P_tot
            T_RL *= scale
            T_RR *= scale

        return T_RL, T_RR


# ────────────────────────────────────────────────────────── configurations
def make_configs(vp, tp_front, tp_rear, cp):
    """The two configurations compared in every maneuver."""
    return [
        TorqueSplitController(vp, tp_front, tp_rear, cp, False, "open (50/50)"),
        TorqueSplitController(vp, tp_front, tp_rear, cp, True,  "s-diff"),
    ]
