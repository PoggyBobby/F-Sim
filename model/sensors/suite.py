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
from model.sensors.vcu.vx_estimator import VxEstimator

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
        # one per corner: same hardware everywhere (a resolver on the motor,
        # a planetary in the upright). These draw NOTHING from the RNG — only
        # the IMU does — so adding two cannot perturb the noise stream, which
        # is what keeps verify I5's bit-exactness intact.
        self.wss = tuple(WheelSpeedSensor(vp.gear_ratio, noise=noise)
                         for _ in range(4))
        self.imu = Imu6Axis(self.rng, noise=noise)
        self.vx_est = VxEstimator()

    def measure(self, s, driver: DriverInputs, info, dt_vcu: float,
                braking: bool) -> SensorReadings:
        """One VCU sample. `s` is the true state, `info` the latest force
        evaluation (for the accelerometer channels)."""
        from model.physical.vehicle import IR, IW, WHEEL_NAMES
        vp, r = self.vp, SensorReadings()

        # pedals & steering — quantization only (they are digital senders)
        r.apps_pct = self.apps.read(driver.apps_pct)
        r.bps_bar = self.bps.read(driver.bps_bar)
        r.handwheel_deg = self.sas.read(driver.handwheel_deg)

        # WSS: motor rpm over CAN (÷ planetary back to wheel speed)
        for i, nm in enumerate(WHEEL_NAMES):
            rpm, w = self.wss[i].read(s[IW[i]])
            setattr(r, "motor_rpm_" + nm, rpm)
            setattr(r, "wheel_speed_" + nm, w)

        # IMU: gyro noise + bias, then the VCU's first-order low-pass
        r.yaw_rate, r.ax, r.ay = self.imu.read(
            s[IR], info["ax"], info["ay"], dt_vcu)

        # VCU estimates: road-wheel angle via the map; ground speed from the
        # four wheel speeds, gated on how far they disagree and carried through
        # the disagreement by the accelerometer (see vx_estimator.py).
        r.steer_est = math.radians(steer_map_deg(r.handwheel_deg))
        r.vx_est = self.vx_est.update(
            [getattr(r, "wheel_speed_" + nm) for nm in WHEEL_NAMES],
            r.ax, dt_vcu, braking, vp.r_wheel, r.yaw_rate,
            (0.5 * vp.track_f, -0.5 * vp.track_f,
             0.5 * vp.track_r, -0.5 * vp.track_r))
        return r

    def reset(self):
        self.imu.reset()
        self.vx_est.reset()
        self.rng = np.random.default_rng(cfg.sensors.vcu.seed)
        self.imu.rng = self.rng
