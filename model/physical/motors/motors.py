"""Motors, inverters and the shaft to each wheel (8 states: ω_m,i and θ_s,i).

Per corner:

    θ̇_s = ω_m / G − ω_w                          shaft twist rate
    T_s  = k_s θ_s + c_s θ̇_s                      torque delivered to the wheel
    J_m ω̇_m = T_e − T_s / G − T_loss(ω_m, T_s)   rotor

T_e is the electromagnetic torque the inverter makes from the commanded
q-axis current, Kt·I_q, clamped to what the motor can deliver at this speed
and bus voltage:

    T_max(ω, V) = min(T_peak, P_peak · (V/V_nom) / |ω|)

plus the regen limit, the rules' regen speed cut-off, and the motor's speed
limiter. The pack-level limits (BMS current, cell voltage, the 80 kW rule
at the accumulator) are applied by the plant as one scale on all four T_e
once the battery has solved its current — see vehicle17.py.

Losses: Coulomb + viscous drag at the rotor and the gearbox's (1 − η) of
the transmitted torque, all opposing rotation.

Backlash: a dead zone of ±backlash/2 in θ_s carries no torque.

The COMMAND PATH (current ramp, CAN latency) is discrete at the VCU rate,
not a continuous state — the DOF doc's argument: a 1–3 ms current loop is
invisible to a 10 ms controller, so the only things worth keeping are the
dead time and the ramp, both of which live naturally at the cycle level.
CommandPath does that; the sim calls it once per VCU cycle.

Values: drivetrain/params.yaml (Kt, G, T_peak, P_peak, limits) and
motors/params.yaml (everything else).
"""

import math


class MotorDrive:
    def __init__(self, mp, w_eps, w_smooth):
        self.p = mp
        self.w_eps = w_eps
        self.w_sm = w_smooth

    # ------------------------------------------------------------ shaft
    def shaft_torque(self, theta_s, theta_s_dot):
        """Torque at the WHEEL from the shaft twist, with backlash."""
        p = self.p
        half = 0.5 * p.backlash
        if half > 0.0:
            if theta_s > half:
                theta_s -= half
            elif theta_s < -half:
                theta_s += half
            else:
                return 0.0
        return p.k_s * theta_s + p.c_s * theta_s_dot

    # ------------------------------------------------------------ inverter
    def torque_ceiling(self, w_m, V_bus):
        """Largest |T_e| the motor can make at rotor speed w_m [rad/s] and
        bus voltage V_bus: peak torque, then peak power scaled with voltage."""
        p = self.p
        T = p.T_peak
        aw = abs(w_m)
        if aw > self.w_eps:
            T = min(T, p.P_peak * (V_bus / p.V_nom) / aw)
        return max(T, 0.0)

    def electromagnetic_torque(self, I_q, w_m, V_bus, regen_allowed):
        """T_e from the q-axis current [A], clamped."""
        p = self.p
        T = p.kt * I_q
        T_max = self.torque_ceiling(w_m, V_bus)
        if T > T_max:
            T = T_max
        if T < 0.0:
            if not regen_allowed:
                T = 0.0
            else:
                T = max(T, -p.T_regen_max, -T_max)
        # speed limiter: no torque that drives the rotor past w_max
        if (w_m >= p.w_max and T > 0.0) or (w_m <= -p.w_max and T < 0.0):
            T = 0.0
        return T

    def loss_torque(self, w_m, T_s):
        """Drag at the rotor: friction + windage + gearbox loss on the
        transmitted torque, all opposing rotation."""
        p = self.p
        s = math.tanh(w_m / self.w_sm)
        return (p.T_fric * s + p.b_visc * w_m
                + (1.0 - p.eta_gear) * abs(T_s) / p.G * s)

    def rotor_accel(self, T_e, T_s, w_m):
        p = self.p
        return (T_e - T_s / p.G - self.loss_torque(w_m, T_s)) / p.J_m

    def electrical_power(self, T_e, w_m):
        """Power drawn from the pack by one motor [W] (negative = regen)."""
        P_mech = T_e * w_m
        return P_mech / self.p.eta if P_mech >= 0.0 else P_mech * self.p.eta


class CommandPath:
    """The inverter's command path at the VCU rate: a whole-cycle dead time
    and a current ramp limit. Four of them, one per motor."""

    def __init__(self, mp, dt_vcu):
        self.p = mp
        self.dt = dt_vcu
        self.n_delay = int(round(mp.t_latency / dt_vcu))
        self.reset()

    def reset(self):
        self.buf = [0.0] * (self.n_delay + 1)
        self.I_applied = 0.0

    def step(self, I_cmd):
        """Push this cycle's commanded current [A], return the current the
        motor actually gets this cycle."""
        self.buf.append(I_cmd)
        target = self.buf.pop(0)
        dI = self.p.I_ramp * self.dt
        if target > self.I_applied + dI:
            self.I_applied += dI
        elif target < self.I_applied - dI:
            self.I_applied -= dI
        else:
            self.I_applied = target
        return self.I_applied
