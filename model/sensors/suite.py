"""SensorSuite — samples the true state at the VCU rate, returns what the
controller is allowed to know.

The controller never reads the simulation's truth. It reads THIS: quantized,
noisy, filtered, and sampled at the VCU loop rate.

The suite owns the one seeded random generator and hands it to the IMU (the
only sensor with a noise model), which is what makes runs exactly repeatable
and makes every controller configuration see identical noise — a fair fight.

WHAT THE VCU MUST ESTIMATE (and the real one will too)
──────────────────────────────────────────────────────
vx — there is no vehicle-speed sensor. Estimated from the rear wheel speeds:
    min(ωL,ωR)·r_w while driving (a spinning wheel reads too fast, so take the
    slower one), max(...) while braking (a locking wheel reads too slow). Both
    rears spinning together still fools it — that is a REAL limitation the real
    car inherits, not a sim bug.

steer — there is no road-wheel angle sensor either. Estimated by pushing the
    SAS handwheel reading through the steering map.
"""

import math

from model.config import cfg
from model.params import VehicleParams
from model.sensors.readings import DriverInputs, SensorReadings
from model.sensors.brake_pressure_sens.bps import BrakePressureSensor
from model.sensors.imu_6axis.imu import Imu6Axis
from model.sensors.steering_angle.sas import SteeringAngleSensor, steer_map_deg
from model.sensors.throttle_pos.apps import ThrottlePositionSensor
from model.sensors.wheel_speed.wss import WheelSpeedSensor

import numpy as np


class SensorSuite:
    """Samples truth → SensorReadings, at the VCU rate."""

    def __init__(self, vp: VehicleParams, seed=None, noise=True):
        self.vp = vp
        self.noise = noise
        self.rng = np.random.default_rng(
            cfg.sensors.vcu.seed if seed is None else seed)
        self.apps = ThrottlePositionSensor(noise=noise)
        self.bps = BrakePressureSensor(noise=noise)
        self.sas = SteeringAngleSensor(noise=noise)
        self.wss_RL = WheelSpeedSensor(vp.gear_ratio, noise=noise)
        self.wss_RR = WheelSpeedSensor(vp.gear_ratio, noise=noise)
        self.imu = Imu6Axis(self.rng, noise=noise)

    def measure(self, s, driver: DriverInputs, info, dt_vcu: float,
                braking: bool) -> SensorReadings:
        """One VCU sample. `s` is the true state, `info` the latest force
        evaluation (for the accelerometer channels)."""
        from model.physical.vehicle import IR, IWRL, IWRR
        vp, r = self.vp, SensorReadings()

        # pedals & steering — quantization only (they are digital senders)
        r.apps_pct = self.apps.read(driver.apps_pct)
        r.bps_bar = self.bps.read(driver.bps_bar)
        r.handwheel_deg = self.sas.read(driver.handwheel_deg)

        # WSS: motor rpm over CAN (÷ planetary back to wheel speed)
        r.motor_rpm_RL, r.wheel_speed_RL = self.wss_RL.read(s[IWRL])
        r.motor_rpm_RR, r.wheel_speed_RR = self.wss_RR.read(s[IWRR])

        # IMU: gyro noise + bias, then the VCU's first-order low-pass
        r.yaw_rate, r.ax, r.ay = self.imu.read(
            s[IR], info["ax"], info["ay"], dt_vcu)

        # VCU estimates: road-wheel angle via the map; vx from wheel speeds
        r.steer_est = math.radians(steer_map_deg(r.handwheel_deg))
        wl, wr = r.wheel_speed_RL, r.wheel_speed_RR
        w_pick = max(wl, wr) if braking else min(wl, wr)
        r.vx_est = max(w_pick * vp.r_wheel, 0.0)
        return r

    def reset(self):
        self.imu.reset()
        self.rng = np.random.default_rng(cfg.sensors.vcu.seed)
        self.imu.rng = self.rng
