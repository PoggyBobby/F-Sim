"""Parameter containers and derived quantities.

⚠️  THE NUMBERS DO NOT LIVE HERE. Every value is pulled from the per-component
`params.yaml` files through `model/config.py`, which documents what each
quantity means, its units, and how to measure it. To change anything about the
car, edit the YAML file next to the component:

    model/physical/mass/params.yaml        car and driver mass, sprung/unsprung
    model/physical/geometry/params.yaml    wheelbase, track, CG, inertias
    model/physical/drivetrain/params.yaml  wheels, gearing, motors, limits
    model/physical/aero/params.yaml        C_L, C_D, area, balance, CoP
    model/physical/tires/params.yaml       Magic Formula + carcass/transients
    model/physical/suspension/params.yaml  jounce DOFs, ARBs, kinematics
    model/physical/dampers/params.yaml     damper curves + Bouc-Wen
    model/physical/steering/params.yaml    Ackermann; rack + knuckle DOFs
    model/physical/motors/params.yaml      rotor, shaft, inverter command path
    model/physical/battery/params.yaml     3-zone equivalent circuit
    model/physical/battery_thermal/params.yaml   core/surface/coolant
    model/physical/brakes/params.yaml      hydraulic lag, friction torque

The controller constants are not here: the only controller is the VCU
firmware (sil/), and its s-diff constants live in sdiff.c.

This file only defines:
  * the dataclass containers the rest of the code passes around,
  * derived quantities (total mass, CG position, wheel torque limit),
  * default_setup(), which builds the planar plant's parameter set, and
    full_setup(), which adds the groups the 17-DOF plant needs.

Conventions (ISO 8855):
    x forward, y LEFT, z up.  Positive yaw rate r = nose swings left (CCW
    from above).  Positive steer angle delta = left turn.
    Wheel order everywhere in the code: [FL, FR, RL, RR].
    Left-side wheels sit at y = +track/2, right-side at y = -track/2.

Drive layout (team-confirmed 2026-08-30): four motors fitted, one per wheel.
The sim drives all four as of 2026-09-09 — the plant takes four wheel torques
and a rear-drive car is simply zero on the fronts. `driven_wheels` and the
firmware's full-pedal current set what 100% throttle MEANS (T_drive_max); the
plant does not read them.
"""

from dataclasses import dataclass, field

from model.config import cfg
from model.config import G, RHO_AIR, BAR   # re-exported; the sim imports these here

__all__ = ["VehicleParams", "TireParams", "FullParams", "default_setup",
           "full_setup", "G", "RHO_AIR", "BAR"]


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
    I_wheel_f: float = cfg.drivetrain.I_wheel_front
    I_wheel_r: float = cfg.drivetrain.I_wheel_rear
    gear_ratio: float = cfg.drivetrain.gear_ratio
    motor_T_peak: float = cfg.drivetrain.motor_torque_peak
    motor_kt: float = cfg.drivetrain.motor_kt
    motor_I_max: float = cfg.drivetrain.max_motor_current
    motor_P_peak: float = cfg.drivetrain.motor_power_peak
    motor_w_max: float = cfg.drivetrain.motor_max_speed
    P_total_max: float = cfg.drivetrain.power_cap_total
    regen_speed_cutoff: float = cfg.drivetrain.regen_speed_cutoff
    driven_wheels: int = cfg.drivetrain.driven_wheels
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
    w_eps: float = cfg.numerical.w_eps

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

    @property
    def I_wheel_corner(self) -> tuple:
        """Spin inertia per corner, wheel order FL, FR, RL, RR."""
        return (self.I_wheel_f, self.I_wheel_f, self.I_wheel_r, self.I_wheel_r)

    @property
    def T_pedal_wheel(self) -> float:
        """Torque at ONE wheel at 100% pedal: the firmware's full-pedal current
        through Kt and the gearbox, never more than the motor's peak."""
        return min(self.motor_I_max * self.motor_kt * self.gear_ratio,
                   self.T_wheel_max)

    @property
    def T_drive_max(self) -> float:
        """Total torque at 100% pedal: one wheel's full-pedal torque times the
        number of DRIVEN wheels. This is what 100% throttle means — the
        firmware's pedal map, the driver adapter's exact inverse of it, and
        every maneuver's torque budget all scale by it, so they must all read
        this one property and never spell the motor count out again."""
        return self.driven_wheels * self.T_pedal_wheel


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
    # ---- full plant only (vehicle17.py); the planar tire ignores these ----
    k_vertical: float = cfg.tires.k_vertical
    c_vertical: float = cfg.tires.c_vertical
    trail_0: float = cfg.tires.pneumatic_trail_0
    trail_alpha: float = cfg.tires.trail_slip_angle
    camber_stiffness: float = cfg.tires.camber_stiffness
    f_rr: float = cfg.tires.rolling_resistance
    sigma_kappa: float = cfg.tires.sigma_kappa
    sigma_alpha: float = cfg.tires.sigma_alpha
    sigma_mz: float = cfg.tires.sigma_mz


# ─────────────────────────────────────────────────────────────────────────
# The groups only the full (17-DOF) plant reads. One dataclass per
# params.yaml so a field name is always the YAML key with the namespace
# dropped; nothing is renamed on the way through.
# ─────────────────────────────────────────────────────────────────────────
@dataclass
class ChassisParams:
    """mass + geometry entries the 6-DOF body needs beyond VehicleParams."""
    m_unsprung_f: float = cfg.mass.unsprung_front
    m_unsprung_r: float = cfg.mass.unsprung_rear
    m_sprung: float = cfg.mass.sprung
    h_sprung: float = cfg.geometry.h_cg_sprung
    I_xx: float = cfg.geometry.I_xx
    I_yy: float = cfg.geometry.I_yy
    I_xz: float = cfg.geometry.I_xz
    y_cg: float = cfg.geometry.cg_lateral_offset
    ride_height_f: float = cfg.geometry.ride_height_front
    ride_height_r: float = cfg.geometry.ride_height_rear
    cop_height: float = cfg.aero.cop_height
    dCl_dh: float = cfg.aero.ride_height_sensitivity

    @property
    def m_unsprung(self) -> tuple:
        return (self.m_unsprung_f, self.m_unsprung_f,
                self.m_unsprung_r, self.m_unsprung_r)


@dataclass
class SuspensionParams:
    k_wheel_f: float = cfg.suspension.wheel_rate_front
    k_wheel_r: float = cfg.suspension.wheel_rate_rear
    k_arb_f: float = cfg.suspension.arb_rate_front
    k_arb_r: float = cfg.suspension.arb_rate_rear
    h_rc_f: float = cfg.suspension.roll_center_height_front
    h_rc_r: float = cfg.suspension.roll_center_height_rear
    anti_dive: float = cfg.suspension.anti_dive
    anti_squat: float = cfg.suspension.anti_squat
    s_jounce_f: float = cfg.suspension.travel_jounce_front
    s_jounce_r: float = cfg.suspension.travel_jounce_rear
    s_rebound_f: float = cfg.suspension.travel_rebound_front
    s_rebound_r: float = cfg.suspension.travel_rebound_rear
    bs_gap_f: float = cfg.suspension.bump_stop_gap_front
    bs_gap_r: float = cfg.suspension.bump_stop_gap_rear
    k_bs: float = cfg.suspension.bump_stop_rate
    camber0_f: float = cfg.suspension.camber_static_front
    camber0_r: float = cfg.suspension.camber_static_rear
    toe0_f: float = cfg.suspension.toe_static_front
    toe0_r: float = cfg.suspension.toe_static_rear
    camber_bump_f: float = cfg.suspension.camber_gain_bump_front
    camber_bump_r: float = cfg.suspension.camber_gain_bump_rear
    camber_roll_f: float = cfg.suspension.camber_gain_roll_front
    camber_roll_r: float = cfg.suspension.camber_gain_roll_rear
    bump_steer_f: float = cfg.suspension.bump_steer_front
    bump_steer_r: float = cfg.suspension.bump_steer_rear
    h_unsprung: float = cfg.suspension.unsprung_cg_height

    def per_wheel(self, f, r):
        return (f, f, r, r)


@dataclass
class DamperParams:
    c_bump_low_f: float = cfg.dampers.c_bump_low_front
    c_bump_high_f: float = cfg.dampers.c_bump_high_front
    c_reb_low_f: float = cfg.dampers.c_rebound_low_front
    c_reb_high_f: float = cfg.dampers.c_rebound_high_front
    c_bump_low_r: float = cfg.dampers.c_bump_low_rear
    c_bump_high_r: float = cfg.dampers.c_bump_high_rear
    c_reb_low_r: float = cfg.dampers.c_rebound_low_rear
    c_reb_high_r: float = cfg.dampers.c_rebound_high_rear
    v_knee: float = cfg.dampers.knee_velocity
    k0: float = cfg.dampers.gas_spring_rate
    alpha: float = cfg.dampers.hysteresis_gain
    bw_A: float = cfg.dampers.bw_A
    bw_beta: float = cfg.dampers.bw_beta
    bw_gamma: float = cfg.dampers.bw_gamma
    bw_n: float = cfg.dampers.bw_n


@dataclass
class SteeringParams:
    ackermann_frac: float = cfg.steering.ackermann_fraction
    c_factor: float = cfg.steering.c_factor
    q_max: float = cfg.steering.rack_travel_max
    m_rack: float = cfg.steering.rack_mass
    F_fric: float = cfg.steering.rack_friction
    c_rack: float = cfg.steering.rack_damping
    k_col: float = cfg.steering.column_stiffness
    k_delta: float = cfg.steering.tierod_stiffness
    I_k: float = cfg.steering.knuckle_inertia
    c_k: float = cfg.steering.knuckle_damping
    kpi: float = cfg.steering.kpi
    caster: float = cfg.steering.caster
    r_scrub: float = cfg.steering.scrub_radius
    t_mech: float = cfg.steering.mechanical_trail


@dataclass
class MotorParams:
    """One motor + its inverter and the shaft to its wheel (all four alike)."""
    kt: float = cfg.drivetrain.motor_kt
    G: float = cfg.drivetrain.gear_ratio
    T_peak: float = cfg.drivetrain.motor_torque_peak
    P_peak: float = cfg.drivetrain.motor_power_peak
    w_max: float = cfg.drivetrain.motor_max_speed
    I_max: float = cfg.drivetrain.max_motor_current
    P_cap_total: float = cfg.drivetrain.power_cap_total
    regen_v_cut: float = cfg.drivetrain.regen_speed_cutoff
    pole_pairs: int = cfg.motors.pole_pairs
    J_m: float = cfg.motors.rotor_inertia
    J_gear: float = cfg.motors.gearbox_inertia_wheel_side
    eta_gear: float = cfg.motors.gearbox_efficiency
    k_s: float = cfg.motors.shaft_stiffness
    c_s: float = cfg.motors.shaft_damping
    backlash: float = cfg.motors.backlash
    T_fric: float = cfg.motors.loss_coulomb
    b_visc: float = cfg.motors.loss_viscous
    eta: float = cfg.motors.efficiency
    T_regen_max: float = cfg.motors.regen_torque_max
    I_ramp: float = cfg.motors.current_ramp
    t_latency: float = cfg.motors.command_latency
    V_nom: float = cfg.motors.v_bus_nominal


@dataclass
class BatteryParams:
    """Electrical (battery/) and thermal (battery_thermal/) together — the
    thermal block has no meaning without the heat the electrical one makes."""
    n_s: int = cfg.battery.cells_series
    n_p: int = cfg.battery.cells_parallel
    n_zones: int = cfg.battery.zones
    Q_cap: float = cfg.battery.capacity_cell          # coulombs (loader: Ah -> C)
    eta_c: float = cfg.battery.coulombic_efficiency
    ocv_0: float = cfg.battery.ocv_at_empty
    ocv_1: float = cfg.battery.ocv_at_full
    R0_ref: float = cfg.battery.r0_ref
    R0_tc: float = cfg.battery.r0_tempco
    R1: float = cfg.battery.r1
    C1: float = cfg.battery.c1
    R2: float = cfg.battery.r2
    C2: float = cfg.battery.c2
    V_min: float = cfg.battery.v_cell_min
    V_max: float = cfg.battery.v_cell_max
    I_dis_cell: float = cfg.battery.i_discharge_max_cell
    I_chg_cell: float = cfg.battery.i_charge_max_cell
    soc_0: float = cfg.battery.soc_initial
    # thermal
    C_core: float = cfg.battery_thermal.c_core
    C_surf: float = cfg.battery_thermal.c_surface
    R_cs: float = cfg.battery_thermal.r_core_surface
    R_sc: float = cfg.battery_thermal.r_surface_coolant
    f_hot: float = cfg.battery_thermal.hot_spot_factor
    C_cool: float = cfg.battery_thermal.c_coolant
    UA: float = cfg.battery_thermal.ua_radiator
    T_derate_0: float = cfg.battery_thermal.derate_start
    T_derate_1: float = cfg.battery_thermal.derate_end
    T_0: float = cfg.battery_thermal.t_initial

    @property
    def n_s_zone(self) -> int:
        """Series cells per zone (the pack is split evenly)."""
        return self.n_s // self.n_zones


@dataclass
class BrakeParams:
    bias_f: float = cfg.brakes.bias_front
    A_p_f: float = cfg.brakes.piston_area_front
    A_p_r: float = cfg.brakes.piston_area_rear
    mu_pad: float = cfg.brakes.pad_mu
    R_eff_f: float = cfg.brakes.r_eff_front
    R_eff_r: float = cfg.brakes.r_eff_rear
    N_surf: int = cfg.brakes.n_surfaces
    tau_f: float = cfg.brakes.tau_front
    tau_r: float = cfg.brakes.tau_rear
    P_max: float = cfg.brakes.p_max

    @property
    def gain_f(self) -> float:
        """Brake torque per bar of FRONT line pressure, one wheel [N·m/bar]."""
        return BAR * self.A_p_f * self.mu_pad * self.R_eff_f * self.N_surf

    @property
    def gain_r(self) -> float:
        """Brake torque per bar of REAR line pressure, one wheel [N·m/bar]."""
        return BAR * self.A_p_r * self.mu_pad * self.R_eff_r * self.N_surf

    @property
    def rear_over_front(self) -> float:
        """Rear line pressure / front line pressure from the balance bar."""
        return (1.0 - self.bias_f) / self.bias_f


@dataclass
class EnvParams:
    g: float = cfg.environment.g
    rho: float = cfg.environment.rho_air
    T_amb: float = cfg.environment.t_ambient
    v_wind: float = cfg.environment.wind_speed
    psi_wind: float = cfg.environment.wind_direction
    grade: float = cfg.environment.road_grade
    bank: float = cfg.environment.road_bank
    v_smooth: float = cfg.numerical.v_smooth
    w_smooth: float = cfg.numerical.w_smooth


@dataclass
class FullParams:
    """Everything the 17-DOF plant reads beyond VehicleParams, one group per
    params.yaml. Built by full_setup()."""
    chassis: ChassisParams = field(default_factory=ChassisParams)
    susp: SuspensionParams = field(default_factory=SuspensionParams)
    damp: DamperParams = field(default_factory=DamperParams)
    steer: SteeringParams = field(default_factory=SteeringParams)
    motor: MotorParams = field(default_factory=MotorParams)
    batt: BatteryParams = field(default_factory=BatteryParams)
    brake: BrakeParams = field(default_factory=BrakeParams)
    env: EnvParams = field(default_factory=EnvParams)


def default_setup():
    """The standard parameter set, built entirely from the params.yaml files.

    Returns (vehicle, tire_front, tire_rear)."""
    vehicle = VehicleParams()
    tire_front = TireParams(c_alpha=cfg.tires.c_alpha_front)
    tire_rear = TireParams(c_alpha=cfg.tires.c_alpha_rear)
    return vehicle, tire_front, tire_rear


def full_setup():
    """default_setup() plus the full plant's groups.

    Returns (vehicle, full, tire_front, tire_rear)."""
    vehicle, tire_front, tire_rear = default_setup()
    return vehicle, FullParams(), tire_front, tire_rear
