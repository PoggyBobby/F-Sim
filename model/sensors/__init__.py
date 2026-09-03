"""The VCU sensor stack — what the controller is allowed to know.

Real VCU sensor set (team, 2026-08-31), one directory each:

    throttle_pos/         APPS  accelerator pedal position  → torque request
    brake_pressure_sens/  BPS   brake pressure              → regen request
    wheel_speed/          WSS   MOTOR shaft rpm from the AMK resolver over
                                CAN, ÷ the upright planetary ratio (there is
                                no separate wheel sensor)
    imu_6axis/            IMU   yaw-rate gyro (noise + bias + VCU low-pass)
                                and the ax/ay accelerometer channels
    steering_angle/       SAS   handwheel angle → road-wheel estimate through
                                the team's steering-chart map
    gps/                        not modeled yet
    vcu/                        the VCU itself: loop rate, rules thresholds,
                                noise seed

Two layers live here:

  DriverAdapter (driver.py) — converts a maneuver's scripted (road-wheel angle,
      total torque) into what the DRIVER's hardware actually does.

  SensorSuite (suite.py) — samples the true state at the VCU rate and returns
      quantized/noisy readings.

Import the public names straight from this package:

    from model.sensors import SensorSuite, DriverAdapter
"""

from model.sensors.quantize import quant
from model.sensors.readings import DriverInputs, SensorReadings
from model.sensors.driver import DriverAdapter
from model.sensors.suite import SensorSuite
from model.sensors.steering_angle.sas import steer_map_deg, steer_map_inv_deg

__all__ = ["SensorSuite", "DriverAdapter", "SensorReadings", "DriverInputs",
           "steer_map_deg", "steer_map_inv_deg", "quant"]
