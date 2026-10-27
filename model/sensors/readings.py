"""The data containers that cross the sensor boundary.

They live in their own module so every sensor, the suite and both plants
can import them without importing each other.
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
    motor_rpm_FL: float = 0.0        # WSS: motor-side speeds, FL FR RL RR
    motor_rpm_FR: float = 0.0
    motor_rpm_RL: float = 0.0
    motor_rpm_RR: float = 0.0
    yaw_rate: float = 0.0            # IMU gyro after VCU low-pass [rad/s]
    ax: float = 0.0                  # IMU accelerometers [m/s²]
    ay: float = 0.0
    az: float = 0.0                  # the other three IMU channels (full plant)
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    handwheel_deg: float = 0.0       # SAS
    gps_speed_1: float = 0.0         # GPS receivers, ground speed [m/s]
    gps_speed_2: float = 0.0
    # ---- VCU-derived (computed from the raw readings above) ----
    wheel_speed_FL: float = 0.0      # rad/s at the wheel (÷ gear ratio)
    wheel_speed_FR: float = 0.0
    wheel_speed_RL: float = 0.0
    wheel_speed_RR: float = 0.0
    vx_est: float = 0.0              # estimated ground speed [m/s]
    steer_est: float = 0.0           # estimated road-wheel angle [rad]


class SensorTruth:
    """What a plant exposes to the sensor suite, plant-agnostic: the suite
    never indexes a state vector. ax/ay/az are the SPECIFIC FORCE at the IMU
    (what an accelerometer reads); omega_motor is the motor-shaft speed the
    resolver sees; v_ground the ground speed at the GPS antenna. b_ax and
    b_gps are the plant's sensor-bias states (zero, has_bias=False, on the
    planar plant, which carries none)."""
    __slots__ = ("yaw_rate", "p", "q", "ax", "ay", "az", "omega_wheel",
                 "omega_motor", "v_ground", "b_ax", "b_gps", "has_bias")

    def __init__(self, yaw_rate, p, q, ax, ay, az, omega_wheel, omega_motor,
                 v_ground, b_ax, b_gps, has_bias):
        self.yaw_rate = yaw_rate
        self.p, self.q = p, q
        self.ax, self.ay, self.az = ax, ay, az
        self.omega_wheel = omega_wheel
        self.omega_motor = omega_motor
        self.v_ground = v_ground
        self.b_ax = b_ax
        self.b_gps = b_gps
        self.has_bias = has_bias
