"""The two data containers that cross the sensor boundary.

They live in their own module so every sensor and the suite can import them
without importing each other.
"""

from dataclasses import dataclass


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
    steer_est: float = 0.0           # estimated road-wheel angle [rad]
