"""IMU — yaw-rate gyro and the ax/ay accelerometer channels.

The only sensor on the car with a real noise model, and therefore the only one
that draws from the random generator. It draws in a fixed order — gyro, then
ax, then ay — and the suite hands it the single seeded generator so that every
controller configuration sees an IDENTICAL noise sequence. Changing that draw
order changes every noisy result in the repo, so don't.

With noise off, nothing is drawn at all (the generator is not advanced), the
bias is zero, and the low-pass still runs: that is the "clean sensors" mode
verify.py check I4 uses to isolate estimation error from measurement error.

Noise, bias and the VCU filter cutoff: model/sensors/imu_6axis/params.yaml
"""

import math

from model.config import cfg


class Imu6Axis:
    """Yaw gyro (noise + bias + the VCU's first-order low-pass) and ax/ay."""

    def __init__(self, rng, noise=True):
        self.rng = rng
        self.noise = noise
        self.gyro_bias = cfg.sensors.imu_6axis.gyro_bias if noise else 0.0
        self.gyro_noise_std = cfg.sensors.imu_6axis.gyro_noise_std
        self.accel_noise_std = cfg.sensors.imu_6axis.accel_noise_std
        self.lpf_hz = cfg.sensors.imu_6axis.lpf_hz
        self._gyro_lpf = None

    def _n(self, std):
        """One noise draw. With noise off the generator is NOT advanced."""
        return self.rng.normal(0.0, std) if self.noise else 0.0

    def read(self, yaw_rate_true: float, ax_true: float, ay_true: float,
             dt_vcu: float):
        """True yaw rate and accelerations → (yaw_rate, ax, ay) as measured.

        Draw order is gyro, ax, ay — see the module docstring.
        """
        gyro = yaw_rate_true + self.gyro_bias + self._n(self.gyro_noise_std)
        if self._gyro_lpf is None:
            self._gyro_lpf = gyro
        alpha = dt_vcu / (dt_vcu + 1.0 / (2.0 * math.pi * self.lpf_hz))
        self._gyro_lpf += alpha * (gyro - self._gyro_lpf)
        ax = ax_true + self._n(self.accel_noise_std)
        ay = ay_true + self._n(self.accel_noise_std)
        return self._gyro_lpf, ax, ay

    def reset(self):
        self._gyro_lpf = None
