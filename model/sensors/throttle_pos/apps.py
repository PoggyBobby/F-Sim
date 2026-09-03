"""APPS — accelerator pedal position sensor.

A digital sender: quantization only, no noise model. The pedal map that turns
a percentage into a torque request is linear and lives in the controller
(100% = full axle torque).

Resolution: model/sensors/throttle_pos/params.yaml
"""

from model.config import cfg
from model.sensors.quantize import quant


class ThrottlePositionSensor:
    def __init__(self, noise=True):
        self.quant_pct = cfg.sensors.throttle_pos.quant_pct if noise else 0.0

    def read(self, apps_pct: float) -> float:
        """True pedal position [%] → what the VCU receives [%]."""
        return quant(apps_pct, self.quant_pct)
