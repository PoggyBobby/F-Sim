"""Sensor simulation — the controller sees THESE, never the sim's truth.

Real VCU sensor set (team, 2026-08-31):
    APPS  accelerator pedal position sensor  → torque request (pedal map)
    BPS   brake pressure sensor              → regen request + rules check
    WSS   "wheel speed" = MOTOR shaft rpm from the AMK resolver over CAN,
          divided by the upright planetary ratio (no separate wheel sensor)
    IMU   yaw-rate gyro (noise + bias + VCU low-pass) and accelerometers
    SAS   steering-angle sensor at the HANDWHEEL, converted to a road-wheel
          angle through the team's steering-chart map

Two layers live here:

  DriverAdapter — converts a maneuver's scripted (road-wheel angle, total
      torque) into what the DRIVER's hardware actually does: a handwheel
      angle (inverse steering map) and pedal positions. The PLANT then uses
      the forward map of that handwheel angle, so the physics inputs are
      bit-identical to the pre-sensor sim — only the CONTROLLER'S KNOWLEDGE
      degrades.

  SensorSuite — samples the true state at the VCU rate and returns
      quantized/noisy readings. Noise is seeded: repeatable runs, and every
      controller config sees identical noise (fair comparison).

What the VCU must ESTIMATE (and the real one will too):
  vx — there is no vehicle-speed sensor. Wheel-based, YAW-CORRECTED
      estimate: each rear wheel is first referred to
         the CG with the gyro (v_from_RL = ω_RL·r_w + r·t/2, v_from_RR =
         ω_RR·r_w − r·t/2 — the inner wheel genuinely runs slower in a
         corner, and without this the pick below reads ~r·t/2 low in
         every turn), then min(...) while driving (a spinning wheel reads
         too fast, so take the slower one), max(...) while braking (a
         locking wheel reads too slow) — `vx_wheel_est` (= `vx_est` here).
      Measured on the corner-exit runs (before any spin): wheel-only
      0.4–0.7 m/s rms error, yaw-corrected 0.14. Both rears spinning
      together still fools it — the IMU fusion comes next.
  per-wheel ground speed and slip ratio — each rear contact patch moves at
      vx ∓ r·track/2 (left wheel slower in a left turn). Combined with the
      wheel speed that gives a per-wheel slip-ratio ESTIMATE
      (`kappa_est_RL/RR`) — the input a per-wheel slip limiter needs.
  dw_geo — the rear wheel-speed difference steering geometry explains,
      v·t·tan(δ)/(L·r_w): what an LSD-style law must subtract from the
      measured difference before acting (zero when the wheel is straight).
"""

import math
from dataclasses import dataclass

import numpy as np

import car_data as cd
from params import VehicleParams


# ── steering map: handwheel deg ↔ road-wheel rad ─────────────────────────
def steer_map_deg(x_deg: float) -> float:
    """Handwheel angle [deg] → road-wheel angle [deg]. Team chart LWheel
    curve, centered so map(0)=0 and extended odd-symmetrically."""
    a0, a1, a2 = cd.STEER_MAP_A0, cd.STEER_MAP_A1, cd.STEER_MAP_A2
    ax = abs(x_deg)
    y = (a0 + a1 * ax + a2 * ax * ax) - a0     # centered: subtract map(0)
    return math.copysign(y, x_deg)


def steer_map_inv_deg(y_deg: float) -> float:
    """Road-wheel angle [deg] → handwheel angle [deg] (exact quadratic
    inverse of the centered map, on the physical branch)."""
    a1, a2 = cd.STEER_MAP_A1, cd.STEER_MAP_A2
    ay = abs(y_deg)
    if abs(a2) < 1e-12:
        x = ay / a1
    else:
        # a2·x² + a1·x − ay = 0 → physical (smaller-|x|) root
        disc = a1 * a1 + 4.0 * a2 * ay
        x = (-a1 + math.sqrt(max(disc, 0.0))) / (2.0 * a2)
    return math.copysign(x, y_deg)


# ── data containers ──────────────────────────────────────────────────────
@dataclass
class DriverInputs:
    """What the driver's hardware is physically doing (the truth)."""
    handwheel_deg: float = 0.0
    apps_pct: float = 0.0
    bps_bar: float = 0.0


@dataclass
class SensorReadings:
    """What the VCU receives — quantized, noisy, filtered."""
    apps_pct: float = 0.0
    bps_bar: float = 0.0
    motor_rpm_RL: float = 0.0        # WSS: motor-side speeds
    motor_rpm_RR: float = 0.0
    yaw_rate: float = 0.0            # IMU gyro after VCU low-pass [rad/s]
    ax: float = 0.0                  # IMU accelerometers [m/s²]
    ay: float = 0.0
    handwheel_deg: float = 0.0       # SAS
    # ---- VCU-derived (computed from the raw readings above) ----
    wheel_speed_RL: float = 0.0      # rad/s at the wheel (÷ gear ratio)
    wheel_speed_RR: float = 0.0
    vx_est: float = 0.0              # estimated ground speed [m/s]
    vx_wheel_est: float = 0.0        # wheel-only ground speed estimate [m/s]
    steer_est: float = 0.0           # estimated road-wheel angle [rad]
    v_ground_RL: float = 0.0         # est. ground speed under each rear
    v_ground_RR: float = 0.0         #   contact patch [m/s] (vx ∓ r·t/2)
    kappa_est_RL: float = 0.0        # est. slip ratio per rear wheel [-]
    kappa_est_RR: float = 0.0
    dw_geo: float = 0.0              # Δω (RR−RL) steering geometry explains


class DriverAdapter:
    """Maneuver (road-wheel rad, total N·m) → driver hardware (handwheel,
    pedals). Exact-inverse, so the plant sees the maneuver unchanged."""

    def __init__(self, vp: VehicleParams):
        self.T_axle_max = 2.0 * vp.T_wheel_max

    def inputs(self, delta_rad: float, T_req: float,
               pedals=None) -> DriverInputs:
        d = DriverInputs()
        d.handwheel_deg = steer_map_inv_deg(math.degrees(delta_rad))
        if pedals is not None:                    # maneuver scripts pedals
            d.apps_pct, d.bps_bar = pedals
            return d
        if T_req >= 0.0:
            d.apps_pct = 100.0 * min(T_req / self.T_axle_max, 1.0)
        else:
            d.bps_bar = cd.BPS_RANGE_BAR * min(-T_req / cd.T_REGEN_MAX, 1.0)
        return d


def _quant(v, step):
    return step * round(v / step) if step > 0 else v


def expected_dw(vx: float, delta: float, vp: VehicleParams) -> float:
    """Rear wheel-speed difference (ω_RR − ω_RL) that steering geometry
    alone produces: kinematic 1/R = tan(δ)/L, each rear contact patch runs
    at v·(1 ± t/(2R)), so Δω = v·t·tan(δ)/(L·r_w). Positive = right wheel
    faster = left turn. Exactly 0 with the wheel straight — no divide by
    zero, no 'are we turning' flag."""
    return vx * vp.track_r * math.tan(delta) / (vp.wheelbase * vp.r_wheel)


class SensorSuite:
    """Samples truth → SensorReadings, at the VCU rate."""

    def __init__(self, vp: VehicleParams, seed=None, noise=True):
        self.vp = vp
        self.noise = noise
        self.rng = np.random.default_rng(cd.SENSOR_SEED if seed is None else seed)
        self.gyro_bias = cd.IMU_GYRO_BIAS if noise else 0.0
        self.accel_bias = cd.IMU_ACCEL_BIAS if noise else 0.0
        self._gyro_lpf = None

    def measure(self, s, driver: DriverInputs, info, dt_vcu: float,
                braking: bool) -> SensorReadings:
        """One VCU sample. `s` is the true state, `info` the latest force
        evaluation (for the accelerometer channels)."""
        from vehicle import IVX, IR, IWRL, IWRR
        vp, r = self.vp, SensorReadings()
        n = (lambda std: self.rng.normal(0.0, std)) if self.noise else (lambda std: 0.0)

        # pedals & steering — quantization only (they are digital senders)
        r.apps_pct = _quant(driver.apps_pct, cd.APPS_QUANT_PCT if self.noise else 0)
        r.bps_bar = _quant(driver.bps_bar, cd.BPS_QUANT_BAR if self.noise else 0)
        r.handwheel_deg = _quant(driver.handwheel_deg,
                                 cd.SAS_QUANT_DEG if self.noise else 0)

        # WSS: motor rpm over CAN (÷ planetary back to wheel speed)
        RPM = math.pi / 30.0
        for name, idx in (("RL", IWRR - 1), ("RR", IWRR)):
            rpm = s[idx] * vp.gear_ratio / RPM
            rpm = _quant(rpm, cd.WSS_QUANT_RPM if self.noise else 0)
            setattr(r, f"motor_rpm_{name}", rpm)
            setattr(r, f"wheel_speed_{name}", rpm * RPM / vp.gear_ratio)

        # IMU: gyro noise + bias, then the VCU's first-order low-pass
        gyro = s[IR] + self.gyro_bias + n(cd.IMU_GYRO_NOISE_STD)
        if self._gyro_lpf is None:
            self._gyro_lpf = gyro
        alpha = dt_vcu / (dt_vcu + 1.0 / (2.0 * math.pi * cd.IMU_LPF_HZ))
        self._gyro_lpf += alpha * (gyro - self._gyro_lpf)
        r.yaw_rate = self._gyro_lpf
        r.ax = info["ax"] + self.accel_bias + n(cd.IMU_ACCEL_NOISE_STD)
        r.ay = info["ay"] + n(cd.IMU_ACCEL_NOISE_STD)

        # ── VCU estimates (computed from the readings above) ──────────
        # road-wheel angle via the steering map
        r.steer_est = math.radians(steer_map_deg(r.handwheel_deg))

        # ground speed: yaw-corrected wheel pick
        wl, wr = r.wheel_speed_RL, r.wheel_speed_RR
        half_t = 0.5 * vp.track_r
        v_from_L = wl * vp.r_wheel + r.yaw_rate * half_t   # y = +t/2
        v_from_R = wr * vp.r_wheel - r.yaw_rate * half_t   # y = −t/2
        v_pick = max(v_from_L, v_from_R) if braking else min(v_from_L, v_from_R)
        r.vx_wheel_est = max(v_pick, 0.0)
        r.vx_est = r.vx_wheel_est

        # per-wheel ground speed and slip ratio (left wheel at y = +t/2
        # moves at vx − r·t/2; the only place the turn enters)
        r.v_ground_RL = r.vx_est - r.yaw_rate * half_t
        r.v_ground_RR = r.vx_est + r.yaw_rate * half_t
        r.kappa_est_RL = ((wl * vp.r_wheel - r.v_ground_RL)
                          / max(abs(r.v_ground_RL), vp.v_eps))
        r.kappa_est_RR = ((wr * vp.r_wheel - r.v_ground_RR)
                          / max(abs(r.v_ground_RR), vp.v_eps))

        # the speed difference steering explains (LSD geometry correction)
        r.dw_geo = expected_dw(r.vx_est, r.steer_est, vp)
        return r

    def reset(self):
        self._gyro_lpf = None
        self.rng = np.random.default_rng(cd.SENSOR_SEED)
