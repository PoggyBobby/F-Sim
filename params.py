"""Parameter containers and derived quantities.

⚠️  THE NUMBERS DO NOT LIVE HERE. Every value is pulled from car_data.py —
the master data file — which documents what each quantity means, its units,
and how to measure it. To change anything about the car, edit car_data.py.

This file only defines:
  * the dataclass containers the rest of the code passes around,
  * derived quantities (total mass, CG position, wheel torque limit),
  * default_setup(), which builds the standard parameter set.

Conventions (ISO 8855):
    x forward, y LEFT, z up.  Positive yaw rate r = nose swings left (CCW
    from above).  Positive steer angle delta = left turn.
    Wheel order everywhere in the code: [FL, FR, RL, RR].
    Left-side wheels sit at y = +track/2, right-side at y = -track/2.

Drive layout (team-confirmed 2026-08-30): four motors fitted, one per
wheel; the two REAR ones are active in the current build, 4WD is the goal.
The same controller math applies per axle — extension noted in README.md.
"""

from dataclasses import dataclass

import car_data as cd
from car_data import G, RHO_AIR   # re-exported; the sim imports these from here


# ─────────────────────────────────────────────────────────────────────────
# Vehicle (chassis + powertrain) — values documented in car_data.py
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class VehicleParams:
    # mass & geometry
    m_car: float = cd.CAR_MASS_NO_DRIVER
    m_driver: float = cd.DRIVER_MASS
    wheelbase: float = cd.WHEELBASE
    track_f: float = cd.TRACK_FRONT
    track_r: float = cd.TRACK_REAR
    weight_frac_front: float = cd.WEIGHT_FRACTION_FRONT
    h_cg: float = cd.H_CG
    I_z: float = cd.I_Z
    # wheels / drivetrain
    r_wheel: float = cd.WHEEL_RADIUS
    I_wheel: float = cd.I_WHEEL
    gear_ratio: float = cd.GEAR_RATIO
    motor_T_peak: float = cd.MOTOR_TORQUE_PEAK
    motor_P_peak: float = cd.MOTOR_POWER_PEAK
    P_total_max: float = cd.POWER_CAP_TOTAL
    regen_speed_cutoff: float = cd.REGEN_SPEED_CUTOFF
    # aero
    ClA: float = cd.CLA
    CdA: float = cd.CDA
    aero_balance_front: float = cd.AERO_BALANCE_FRONT
    # steering geometry
    ackermann_frac: float = cd.ACKERMANN_FRACTION
    # load transfer
    lat_transfer_frac_front: float = cd.LAT_TRANSFER_FRAC_FRONT
    # numerical guard
    v_eps: float = cd.V_EPS

    # ---- derived quantities (computed, never entered) ----------------------
    @property
    def m_total(self) -> float:
        """Total mass WITH driver = car + driver (see the MASS box in
        car_data.py — never enter a with-driver total directly)."""
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
# Tire (simplified Magic Formula) — values documented in car_data.py
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class TireParams:
    mu0: float = cd.TIRE_MU0
    mu0x: float = cd.TIRE_MU0_LONG
    s_mu: float = cd.TIRE_S_MU
    Fz_nom: float = cd.TIRE_FZ_NOM
    c_alpha: float = cd.TIRE_C_ALPHA_REAR   # per-axle value set in default_setup()
    C_y: float = cd.TIRE_SHAPE_C_LAT
    E_y: float = cd.TIRE_CURV_E_LAT
    c_kappa: float = cd.TIRE_C_KAPPA
    C_x: float = cd.TIRE_SHAPE_C_LONG
    E_x: float = cd.TIRE_CURV_E_LONG


# ─────────────────────────────────────────────────────────────────────────
# Controller gains — values documented in car_data.py; retune on real data
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class ControlParams:
    kp_sdiff: float = cd.KP_SDIFF
    ki_sdiff: float = cd.KI_SDIFF
    i_sdiff_max: float = cd.I_SDIFF_MAX
    dT_sdiff_max: float = cd.DT_SDIFF_MAX
    kp_tv: float = cd.KP_TV
    ki_tv: float = cd.KI_TV
    i_tv_max: float = cd.I_TV_MAX
    Mz_max: float = cd.MZ_MAX
    ay_frac: float = cd.AY_FRAC


def default_setup():
    """The standard parameter set, built entirely from car_data.py.

    Returns (vehicle, tire_front, tire_rear, control)."""
    vehicle = VehicleParams()
    tire_front = TireParams(c_alpha=cd.TIRE_C_ALPHA_FRONT)
    tire_rear = TireParams(c_alpha=cd.TIRE_C_ALPHA_REAR)
    control = ControlParams()
    return vehicle, tire_front, tire_rear, control
