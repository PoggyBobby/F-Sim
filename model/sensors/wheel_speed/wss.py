"""WSS — "wheel speed", which on this car is not a wheel sensor at all.

There is no separate wheel-speed sensor: the AMK resolver reports MOTOR shaft
speed over CAN, and the VCU divides by the upright planetary ratio to get a
wheel speed. So the quantization that matters happens in MOTOR rpm, before the
divide — which is why this module quantizes in rpm and converts afterwards.

Resolution: model/sensors/wheel_speed/params.yaml
"""

import math

from model.config import cfg
from model.sensors.quantize import quant

RPM = math.pi / 30.0        # rad/s per rev-per-minute


class WheelSpeedSensor:
    """One corner's speed, as the VCU sees it."""

    def __init__(self, gear_ratio: float, noise=True):
        self.gear_ratio = gear_ratio
        self.quant_rpm = cfg.sensors.wheel_speed.quant_rpm if noise else 0.0

    def read(self, omega_wheel: float):
        """True wheel speed [rad/s] → (motor rpm reported, wheel speed [rad/s]).

        The wheel speed returned is the one the VCU computes back from the
        QUANTIZED motor rpm — not the truth it started from.
        """
        return self.read_motor(omega_wheel * self.gear_ratio)

    def read_motor(self, omega_motor: float):
        """True MOTOR shaft speed [rad/s] → (motor rpm reported, wheel speed
        [rad/s]). What the resolver measures; the full plant has a rotor
        state, the planar plant hands over ω_wheel × gear."""
        rpm = omega_motor / RPM
        rpm = quant(rpm, self.quant_rpm)
        return rpm, rpm * RPM / self.gear_ratio
