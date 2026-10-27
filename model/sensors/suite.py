"""SensorSuite — samples the true state at the VCU rate, returns what the
controller is allowed to know.

The controller never reads the simulation's truth. It reads THIS: quantized,
noisy, filtered, and sampled at the VCU loop rate.

The suite is PLANT-AGNOSTIC: it asks the model for a SensorTruth
(model.sensor_truth) and never indexes a state vector. The planar plant
hands over ΣF/m and ω·gear; the full plant the specific force at the IMU
position, the rotor speed the resolver sees, and its sensor-bias states.

The suite owns the one seeded random generator. Draw order per cycle:
gyro, ax, ay (the IMU, unchanged since the planar sim) — then, only when the
plant carries bias states (the full plant): GPS 1 noise, GPS 2 noise, and
the three random-walk increments (accel bias, GPS 1, GPS 2) which the suite
applies to the plant's states. The planar plant draws nothing extra, so its
runs are bit-identical to the pre-GPS suite. Either way every controller
configuration sees identical noise — a fair fight — and runs repeat exactly
(verify I5).

WHAT THE VCU MUST ESTIMATE (and the real one will too)
──────────────────────────────────────────────────────
vx — there is no vehicle-speed sensor the s-diff firmware reads. Estimated
    from the wheel speeds, gated on their spread, carried by the
    accelerometer (vcu/vx_estimator.py). GPS speed is reported alongside
    for the slip-controller work; nothing in sdiff.c consumes it yet.

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
from model.sensors.gps.gps import GpsSpeed
from model.sensors.vcu.vx_estimator import VxEstimator
from model.physical.vehicle import WHEEL_NAMES

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
        # the IMU and GPS do — so adding them cannot perturb the noise stream.
        self.wss = tuple(WheelSpeedSensor(vp.gear_ratio, noise=noise)
                         for _ in range(4))
        self.imu = Imu6Axis(self.rng, noise=noise)
        self.gps = (GpsSpeed(self.rng, noise=noise), GpsSpeed(self.rng, noise=noise))
        self.vx_est = VxEstimator()
        self.accel_walk = cfg.sensors.imu_6axis.accel_bias_walk
        self.gps_walk = cfg.sensors.gps.bias_walk

    def measure(self, model, s, driver: DriverInputs, info, dt_vcu: float,
                braking: bool) -> SensorReadings:
        """One VCU sample. `s` is the true state, `info` the latest force
        evaluation; both go to model.sensor_truth()."""
        vp, r = self.vp, SensorReadings()
        tr = model.sensor_truth(s, info)

        # pedals & steering — quantization only (they are digital senders)
        r.apps_pct = self.apps.read(driver.apps_pct)
        r.bps_bar = self.bps.read(driver.bps_bar)
        r.handwheel_deg = self.sas.read(driver.handwheel_deg)

        # WSS: motor rpm over CAN (÷ planetary back to wheel speed)
        for i, nm in enumerate(WHEEL_NAMES):
            rpm, w = self.wss[i].read_motor(tr.omega_motor[i])
            setattr(r, "motor_rpm_" + nm, rpm)
            setattr(r, "wheel_speed_" + nm, w)

        # IMU: gyro noise + bias, then the VCU's first-order low-pass; the
        # accelerometer carries the plant's slowly wandering bias
        r.yaw_rate, r.ax, r.ay = self.imu.read(
            tr.yaw_rate, tr.ax + tr.b_ax, tr.ay, dt_vcu)
        r.roll_rate, r.pitch_rate, r.az = tr.p, tr.q, tr.az

        # GPS: two receivers; noise and the bias walks only on a plant that
        # carries the bias states
        if tr.has_bias:
            r.gps_speed_1 = self.gps[0].read(tr.v_ground, tr.b_gps[0], dt_vcu)
            r.gps_speed_2 = self.gps[1].read(tr.v_ground, tr.b_gps[1], dt_vcu)
            if self.noise:
                sq = math.sqrt(dt_vcu)
                d_ax = self.rng.normal(0.0, self.accel_walk * sq)
                d_g = (self.rng.normal(0.0, self.gps_walk * sq),
                       self.rng.normal(0.0, self.gps_walk * sq))
                model.apply_bias_walk(s, d_ax, d_g)
        else:
            r.gps_speed_1 = r.gps_speed_2 = tr.v_ground

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
        for g in self.gps:
            g.reset()
        self.rng = np.random.default_rng(cfg.sensors.vcu.seed)
        self.imu.rng = self.rng
        for g in self.gps:
            g.rng = self.rng
