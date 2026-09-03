"""Parameter containers and derived quantities.

⚠️  THE NUMBERS DO NOT LIVE HERE. Every value is pulled from the per-component
`params.yaml` files through `model/config.py`, which documents what each
quantity means, its units, and how to measure it. To change anything about the
car, edit the YAML file next to the component:

    model/physical/mass/params.yaml        car and driver mass
    model/physical/geometry/params.yaml    wheelbase, track, CG, yaw inertia
    model/physical/drivetrain/params.yaml  wheels, gearing, motors, limits
    model/physical/aero/params.yaml        C_L, C_D, area, balance
    model/physical/tires/params.yaml       Magic Formula coefficients
    controllers/python/params.yaml         the tuned gains

This file only defines:
  * the dataclass containers the rest of the code passes around,
  * derived quantities (total mass, CG position, wheel torque limit),
  * default_setup(), which builds the standard parameter set.

Conventions (ISO 8855):
    x forward, y LEFT, z up.  Positive yaw rate r = nose swings left (CCW
    from above).  Positive steer angle delta = left turn.
    Wheel order everywhere in the code: [FL, FR, RL, RR].
    Left-side wheels sit at y = +track/2, right-side at y = -track/2.

Drive layout (team-confirmed 2026-08-30): four motors fitted, one per wheel;
the two REAR ones are active in the current build, 4WD is the goal. The same
controller math applies per axle — extension noted in README.md.
"""

from dataclasses import dataclass

from model.config import cfg
from model.config import G, RHO_AIR   # re-exported; the sim imports these here

__all__ = ["VehicleParams", "TireParams", "ControlParams", "default_setup",
           "G", "RHO_AIR"]


# ─────────────────────────────────────────────────────────────────────────
# Vehicle (chassis + powertrain) — values documented in model/physical/*.yaml
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class VehicleParams:
    # mass & geometry
    m_car: float = cfg.mass.car_no_driver
    m_driver: float = cfg.mass.driver
    wheelbase: float = cfg.geometry.wheelbase
    track_f: float = cfg.geometry.track_front
    track_r: float = cfg.geometry.track_rear
    weight_frac_front: float = cfg.geometry.weight_fraction_front
    h_cg: float = cfg.geometry.h_cg
    I_z: float = cfg.geometry.I_z
    # wheels / drivetrain
    r_wheel: float = cfg.drivetrain.wheel_radius
    I_wheel: float = cfg.drivetrain.I_wheel
    gear_ratio: float = cfg.drivetrain.gear_ratio
    motor_T_peak: float = cfg.drivetrain.motor_torque_peak
    motor_P_peak: float = cfg.drivetrain.motor_power_peak
    P_total_max: float = cfg.drivetrain.power_cap_total
    regen_speed_cutoff: float = cfg.drivetrain.regen_speed_cutoff
    # aero
    ClA: float = cfg.aero.ClA
    CdA: float = cfg.aero.CdA
    aero_balance_front: float = cfg.aero.balance_front
    # steering geometry
    ackermann_frac: float = cfg.steering.ackermann_fraction
    # load transfer
    lat_transfer_frac_front: float = cfg.loads.lat_transfer_frac_front
    # numerical guard
    v_eps: float = cfg.numerical.v_eps

    # ---- derived quantities (computed, never entered) ----------------------
    @property
    def m_total(self) -> float:
        """Total mass WITH driver = car + driver (see the `about:` box in
        model/physical/mass/params.yaml — never enter a with-driver total
        directly)."""
        return self.m_car + self.m_driver

    @property
    def b(self) -> float:
        """CG to REAR axle distance. From statics F_z_front/W = b/L,
        so b = weight_frac_front * wheelbase."""
        return self.weight_frac_front * self.wheelbase

    @property
    def a(self) -> float:
        """CG to FRONT axle distance."""
        return self.wheelbase - self.b

    @property
    def T_wheel_max(self) -> float:
        """Peak torque available at ONE wheel (motor peak * gear ratio)."""
        return self.motor_T_peak * self.gear_ratio


# ─────────────────────────────────────────────────────────────────────────
# Tire (simplified Magic Formula) — model/physical/tires/params.yaml
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class TireParams:
    mu0: float = cfg.tires.mu0
    mu0x: float = cfg.tires.mu0_long
    s_mu: float = cfg.tires.s_mu
    Fz_nom: float = cfg.tires.Fz_nom
    c_alpha: float = cfg.tires.c_alpha_rear   # per-axle value set in default_setup()
    C_y: float = cfg.tires.shape_c_lat
    E_y: float = cfg.tires.curv_e_lat
    c_kappa: float = cfg.tires.c_kappa
    C_x: float = cfg.tires.shape_c_long
    E_x: float = cfg.tires.curv_e_long


# ─────────────────────────────────────────────────────────────────────────
# Controller gains — controllers/python/params.yaml; retune on real data
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class ControlParams:
    kp_sdiff: float = cfg.controllers.kp_sdiff
    ki_sdiff: float = cfg.controllers.ki_sdiff
    i_sdiff_max: float = cfg.controllers.i_sdiff_max
    dT_sdiff_max: float = cfg.controllers.dt_sdiff_max
    kp_tv: float = cfg.controllers.kp_tv
    ki_tv: float = cfg.controllers.ki_tv
    i_tv_max: float = cfg.controllers.i_tv_max
    Mz_max: float = cfg.controllers.mz_max
    ay_frac: float = cfg.controllers.ay_frac


def default_setup():
    """The standard parameter set, built entirely from the params.yaml files.

    Returns (vehicle, tire_front, tire_rear, control)."""
    vehicle = VehicleParams()
    tire_front = TireParams(c_alpha=cfg.tires.c_alpha_front)
    tire_rear = TireParams(c_alpha=cfg.tires.c_alpha_rear)
    control = ControlParams()
    return vehicle, tire_front, tire_rear, control
