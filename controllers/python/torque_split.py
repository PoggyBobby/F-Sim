"""Torque-split controllers: software differential, torque vectoring, both.

With two independent rear motors there is no mechanical differential — the
"differential" is whatever the software decides the left/right torque split
is. Everything below reduces to choosing:

    T_RL = T_base - dT/2
    T_RR = T_base + dT/2        with  dT = dT_sdiff + dT_tv

1) OPEN-DIFF BASELINE (both controllers off): equal torque to both motors.
   This exactly reproduces an open differential (equal torque, wheels free
   to spin at different speeds) — including its failure mode: an unloaded
   inner wheel can spin up and dump grip.

2) SOFTWARE DIFFERENTIAL (s-diff): makes the wheel-speed DIFFERENCE track
   the value the corner geometry requires. A wheel at lateral offset y from
   the CG must roll at ground speed vx - r*y, so the rear pair should differ
   by:
        Δω_target = ω_RR - ω_RL = r * track_r / r_wheel
   A PI controller on  e = Δω_target - (ω_RR - ω_RL)  shifts torque from the
   wheel spinning faster than geometry allows to the other one. This acts
   like an ideal limited-slip diff: it permits exactly the kinematic speed
   difference and fights inner-wheel spin-up beyond it.

3) TORQUE VECTORING (TV): makes the YAW RATE track a reference computed
   from what the driver is asking for (steering angle + speed), using the
   steady-state single-track (bicycle) model:

        r_ref = vx * delta / (L + K_us * vx^2)
        K_us  = m/L * ( b/C_f - a/C_r )        (understeer gradient)

   capped by the friction-limited lateral acceleration  |r| <= ay_max/vx,
   with ay_max = mu*(g + downforce/m). A PI controller on e = r_ref - r
   commands a yaw moment M_z, produced by a left/right longitudinal force
   difference at the rear axle:

        M_z = (track_r/2) * (Fx_RR - Fx_RL) = (track_r/2) * dT/r_wheel
        =>  dT_tv = 2 * M_z * r_wheel / track_r

4) COMBINED: dT contributions simply sum. They are complementary — the
   s-diff regulates wheel SPEEDS (traction management), TV regulates the
   body YAW RATE (handling balance) — and at corner exit they push the same
   direction: both move torque away from the unloaded inner wheel.

After the split, physical limits are enforced: per-motor peak torque, the
regen speed cutoff, per-motor peak power, and the 80 kW total (FSAE EV
rules) cap. When one wheel's command would exceed its torque limit, the
BASE torque is shifted so the left/right DIFFERENCE (i.e. the yaw moment)
is preserved — total thrust is sacrificed before yaw authority.

Two update paths exist:
  update(...)               — perfect-state feedback (physics testing)
  update_from_sensors(...)  — the REAL path: consumes sensors.py readings
                              (WSS/IMU/SAS/APPS/BPS at the VCU rate),
                              including the vx estimate and the EV.4.7
                              APPS/BPS plausibility cut. run_sim.py uses
                              this path by default.
"""

import math
from dataclasses import dataclass, field
from model.params import VehicleParams, TireParams, ControlParams, G, RHO_AIR
from model.physical.vehicle import IVX, IR, IWRL, IWRR


@dataclass
class ControllerDebug:
    r_ref: float = 0.0        # yaw-rate reference [rad/s]
    dw_target: float = 0.0    # target wheel-speed difference wRR-wRL [rad/s]
    dT_sdiff: float = 0.0     # s-diff torque-split contribution [N·m]
    dT_tv: float = 0.0        # TV torque-split contribution [N·m]
    Mz_cmd: float = 0.0       # commanded yaw moment [N·m]
    T_RL: float = 0.0
    T_RR: float = 0.0


class TorqueSplitController:
    """One controller class; s-diff and TV terms are switched on/off to get
    the four configurations (open / s-diff / TV / combined)."""

    def __init__(self, vp: VehicleParams, tp_front: TireParams,
                 tp_rear: TireParams, cp: ControlParams,
                 sdiff_on: bool, tv_on: bool, name: str):
        self.vp, self.cp = vp, cp
        self.sdiff_on, self.tv_on = sdiff_on, tv_on
        self.name = name
        self.mu_ref = 0.5 * (tp_front.mu0 + tp_rear.mu0)  # for the ay cap

        # understeer gradient from the linearized tire stiffnesses at static
        # axle loads:  C_axle = c_alpha * Fz_axle  (per-axle, both tires)
        m, L = vp.m_total, vp.wheelbase
        C_f = tp_front.c_alpha * m * G * vp.b / L
        C_r = tp_rear.c_alpha * m * G * vp.a / L
        self.K_us = m / L * (vp.b / C_f - vp.a / C_r)   # s^2/m, >0 = understeer

        self.i_sdiff = 0.0   # integral terms (stored as torque / moment)
        self.i_tv = 0.0
        self.plaus_cut = False

    def reset(self):
        self.i_sdiff = 0.0
        self.i_tv = 0.0
        self.plaus_cut = False   # EV.4.7 APPS/BPS plausibility latch

    # ------------------------------------------------------- reference model
    def yaw_rate_ref(self, vx: float, delta: float) -> float:
        vp, L = self.vp, self.vp.wheelbase
        vx_s = max(vx, 1.0)
        r_ref = vx_s * delta / (L + self.K_us * vx_s * vx_s)
        # friction-limited cap: |ay| = |r*vx| <= mu*(g + downforce/m)
        downforce = 0.5 * RHO_AIR * vp.ClA * vx * vx
        ay_max = self.mu_ref * (G + downforce / vp.m_total)
        r_cap = self.cp.ay_frac * ay_max / vx_s
        return max(-r_cap, min(r_cap, r_ref))

    # --------------------------------------------------------------- update
    def update(self, s, delta: float, T_req_total: float, dt: float) -> ControllerDebug:
        vp, cp = self.vp, self.cp
        vx, r = s[IVX], s[IR]
        wRL, wRR = s[IWRL], s[IWRR]

        dbg = ControllerDebug()
        dbg.r_ref = self.yaw_rate_ref(vx, delta)
        dbg.dw_target = r * vp.track_r / vp.r_wheel

        T_base = T_req_total / 2.0
        dT = 0.0

        if self.tv_on:
            e = dbg.r_ref - r
            self.i_tv += cp.ki_tv * e * dt
            self.i_tv = max(-cp.i_tv_max, min(cp.i_tv_max, self.i_tv))
            Mz = cp.kp_tv * e + self.i_tv
            Mz = max(-cp.Mz_max, min(cp.Mz_max, Mz))
            dbg.Mz_cmd = Mz
            dbg.dT_tv = 2.0 * Mz * vp.r_wheel / vp.track_r
            dT += dbg.dT_tv

        if self.sdiff_on:
            e = dbg.dw_target - (wRR - wRL)
            self.i_sdiff += cp.ki_sdiff * e * dt
            self.i_sdiff = max(-cp.i_sdiff_max, min(cp.i_sdiff_max, self.i_sdiff))
            # cap the transfer: the s-diff trims wheel speeds, it must not
            # dump torque onto the loaded outer tire (power-oversteer risk)
            dbg.dT_sdiff = max(-cp.dT_sdiff_max, min(cp.dT_sdiff_max,
                               cp.kp_sdiff * e + self.i_sdiff))
            dT += dbg.dT_sdiff

        T_RL, T_RR = self._apply_limits(T_base, dT, vx, wRL, wRR)
        dbg.T_RL, dbg.T_RR = T_RL, T_RR
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
        from model.config import cfg  # was: import car_data as cd

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
    """The four configurations compared in every maneuver."""
    return [
        TorqueSplitController(vp, tp_front, tp_rear, cp, False, False, "open (50/50)"),
        TorqueSplitController(vp, tp_front, tp_rear, cp, True,  False, "s-diff"),
        TorqueSplitController(vp, tp_front, tp_rear, cp, False, True,  "TV"),
        TorqueSplitController(vp, tp_front, tp_rear, cp, True,  True,  "s-diff + TV"),
    ]
