"""The driver's hardware — the layer ABOVE the sensors.

A maneuver script commands a road-wheel angle and a total torque. A real driver
cannot do either directly: they turn a handwheel and press pedals. DriverAdapter
converts the script into those physical actions, using the EXACT INVERSE of the
steering map.

That exactness is the point. The plant then applies the forward map to the
handwheel angle, so the physics inputs are bit-identical to the pre-sensor sim
— only the CONTROLLER'S KNOWLEDGE degrades. Any error here would show up as a
change in the car's motion, which would confound every sensor comparison.
"""

import math

from model.config import cfg
from model.params import VehicleParams
from model.sensors.readings import DriverInputs
from model.sensors.steering_angle.sas import steer_map_inv_deg


class DriverAdapter:
    """Maneuver (road-wheel rad, total N·m) → driver hardware (handwheel,
    pedals). Exact-inverse, so the plant sees the maneuver unchanged."""

    def __init__(self, vp: VehicleParams):
        self.T_axle_max = 2.0 * vp.T_wheel_max

    def inputs(self, delta_rad: float, T_req: float,
               pedals=None) -> DriverInputs:
        d = DriverInputs()
        d.handwheel_deg = steer_map_inv_deg(math.degrees(delta_rad))
        if pedals is not None:                    # maneuver scripts pedals
            d.apps_pct, d.bps_bar = pedals
            return d
        if T_req >= 0.0:
            d.apps_pct = 100.0 * min(T_req / self.T_axle_max, 1.0)
        else:
            bps = cfg.sensors.brake_pressure_sens
            d.bps_bar = bps.range_bar * min(-T_req / bps.t_regen_max, 1.0)
        return d
