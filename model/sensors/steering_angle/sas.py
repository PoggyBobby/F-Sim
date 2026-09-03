"""SAS — steering-angle sensor, at the HANDWHEEL.

The VCU never measures a road-wheel angle. It reads the handwheel and converts
through the team's steering chart:

    y = A0 + A1*x + A2*x²

used odd-symmetrically (sign(x)·map(|x|)) and centered so map(0) = 0. This
module owns both directions of that map: the forward one the VCU applies to a
sensor reading, and the exact inverse the DriverAdapter uses to work out what
handwheel angle a scripted road-wheel angle corresponds to.

Coefficients and resolution: model/sensors/steering_angle/params.yaml
"""

import math

from model.config import cfg
from model.sensors.quantize import quant


def steer_map_deg(x_deg: float) -> float:
    """Handwheel angle [deg] → road-wheel angle [deg]. Team chart LWheel
    curve, centered so map(0)=0 and extended odd-symmetrically."""
    a0 = cfg.sensors.steering_angle.map_a0
    a1 = cfg.sensors.steering_angle.map_a1
    a2 = cfg.sensors.steering_angle.map_a2
    ax = abs(x_deg)
    y = (a0 + a1 * ax + a2 * ax * ax) - a0     # centered: subtract map(0)
    return math.copysign(y, x_deg)


def steer_map_inv_deg(y_deg: float) -> float:
    """Road-wheel angle [deg] → handwheel angle [deg] (exact quadratic
    inverse of the centered map, on the physical branch)."""
    a1 = cfg.sensors.steering_angle.map_a1
    a2 = cfg.sensors.steering_angle.map_a2
    ay = abs(y_deg)
    if abs(a2) < 1e-12:
        x = ay / a1
    else:
        # a2·x² + a1·x − ay = 0 → physical (smaller-|x|) root
        disc = a1 * a1 + 4.0 * a2 * ay
        x = (-a1 + math.sqrt(max(disc, 0.0))) / (2.0 * a2)
    return math.copysign(x, y_deg)


class SteeringAngleSensor:
    """Reads the handwheel angle. Quantization only — a digital sender."""

    def __init__(self, noise=True):
        self.quant_deg = cfg.sensors.steering_angle.quant_deg if noise else 0.0

    def read(self, handwheel_deg: float) -> float:
        """True handwheel angle [deg] → what the VCU receives [deg]."""
        return quant(handwheel_deg, self.quant_deg)
