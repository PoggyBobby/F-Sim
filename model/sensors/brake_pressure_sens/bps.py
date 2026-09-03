"""BPS — brake pressure sensor.

Mechanical brakes are NOT modeled in this sim: brake pressure commands REGEN
only. The sensor itself is a digital sender — quantization, no noise.

Range, resolution, the "actuated" threshold for the EV.4.7 plausibility check,
and the regen torque at full pressure: model/sensors/brake_pressure_sens/params.yaml
"""

from model.config import cfg
from model.sensors.quantize import quant


class BrakePressureSensor:
    def __init__(self, noise=True):
        self.quant_bar = cfg.sensors.brake_pressure_sens.quant_bar if noise else 0.0

    def read(self, bps_bar: float) -> float:
        """True line pressure [bar] → what the VCU receives [bar]."""
        return quant(bps_bar, self.quant_bar)
