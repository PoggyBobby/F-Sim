"""The full plant: 17 DOF, 80 states — the DOF-expansion plan, built to its
ledger.

    DOF                              states  indices
    6-DOF sprung body                  13    X Y Z, u v w, q0..q3, p q r
    4 suspension jounce                 8    z_u (unsprung height), ż_u
    4 wheel rotation                    8    θ_w, ω_w
    rack + 2 knuckles                   6    q_r q̇_r, δ_FL δ̇_FL, δ_FR δ̇_FR
    damper hysteresis                   4    z_d (Bouc-Wen)
    tire transients                    12    κ̃, α̃, M̃_z
    motor rotor + shaft                 8    ω_m, θ_s
    battery electrical (3 zones)        9    SOC, V1, V2
    battery thermal                     7    T_core, T_surf, T_cool
    brake hydraulics                    2    P_f, P_r
    sensor bias                         3    b_ax, b_gps1, b_gps2
                                       80

Each block's physics lives next to its params.yaml and is documented there:
chassis/rigid_body.py, suspension/, dampers/, steering/, tires/transient.py,
motors/, battery/, brakes/. This file only assembles forces and moments on
the sprung body and the unsprung masses and hands each block its states.

HOW THE BODY AND THE WHEELS ARE COUPLED
──────────────────────────────────────
The sprung body (m_s, I) carries the 6 DOF. Each unsprung mass m_u,i moves
VERTICALLY (ground z) on its own coordinate z_u,i and is dragged along
horizontally with the body — the usual 14-DOF split. So:

  * horizontal body equations use the TOTAL mass (the unsprung masses ride
    along), vertical ones the sprung mass with the four suspension forces;
  * the unsprung masses' own horizontal inertia reaches the body as a
    moment at wheel-centre height (the "unsprung load transfer" term);
  * tire horizontal forces are applied to the body at the contact patch,
    and the part that the links carry instantly (roll centre, anti-dive /
    anti-squat) is added as an equal-and-opposite vertical pair — down on
    the unsprung mass, up on the body corner. Net on the body: exactly
    F_y (h_s − h_rc) of elastic roll moment, the textbook result (verify K).

The suspension deflection s_i is ALGEBRAIC: the body corner's height minus
the unsprung height, both measured from static. Spring forces are written
F_static + k·s so s = 0 is the static ride height and no preload is entered.

Tire vertical force is the carcass spring: F_z = k_t (δ_t0 − z_u) − c_t ż_u,
never negative. That F_z is what the force law and the load checks see.

INPUTS. derivatives(s, u) with u = (handwheel [rad], four motor currents
[A] FL FR RL RR, brake line pressure [bar]). The sim loop builds u from the
firmware's command through plant_input(); the steering map and Ackermann
split live in steering.py; Kt, the torque-speed ceiling and the pack limits
in motors.py / battery.py. Nothing in this file turns a current into a
torque by itself.

Sign conventions as everywhere in the repo (ISO 8855): x forward, y LEFT,
z up, wheel order FL FR RL RR, positive steer = left.
"""

import math

from model.params import VehicleParams, FullParams, G
from model.physical.vehicle import front_steer_angles
from model.physical.chassis.rigid_body import (q_normalize, q_to_R, rot, rot_T,
                                               q_dot, q_from_euler,
                                               euler_from_q, omega_dot, cross)
from model.physical.suspension.suspension import Suspension
from model.physical.dampers.damper import Damper
from model.physical.steering.steering import Steering
from model.physical.tires.transient import TransientTire
from model.physical.motors.motors import MotorDrive, CommandPath
from model.physical.battery.battery import Battery
from model.physical.brakes.brakes import Brakes
from model.sensors.steering_angle.sas import steer_map_inv_deg
from model.sensors.readings import SensorTruth

# ── state layout ──────────────────────────────────────────────────────────
IX, IY, IZ = 0, 1, 2                 # sprung CG position, ground frame [m]
IU, IV, IWZ = 3, 4, 5                # body velocity u v w [m/s]
IQ0, IQ1, IQ2, IQ3 = 6, 7, 8, 9      # attitude quaternion (body -> ground)
IP, IQ, IR = 10, 11, 12              # body rates p q r [rad/s]
IZU = (13, 14, 15, 16)               # unsprung height above static [m]
IZUD = (17, 18, 19, 20)              # its rate
ITH = (21, 22, 23, 24)               # wheel rotation angle [rad]
IOM = (25, 26, 27, 28)               # wheel spin speed [rad/s]
IQR, IQRD = 29, 30                   # rack position [m], rate
IDFL, IDFLD, IDFR, IDFRD = 31, 32, 33, 34   # knuckle angles [rad], rates
IZD = (35, 36, 37, 38)               # Bouc-Wen damper state
IKT = (39, 40, 41, 42)               # effective slip ratio
IAT = (43, 44, 45, 46)               # effective slip angle [rad]
IMT = (47, 48, 49, 50)               # effective aligning moment [N·m]
IOMM = (51, 52, 53, 54)              # rotor speed [rad/s]
ITHS = (55, 56, 57, 58)              # shaft twist [rad]
ISOC = (59, 60, 61)                  # zone SOC
IV1 = (62, 63, 64)                   # zone fast RC voltage [V]
IV2 = (65, 66, 67)                   # zone slow RC voltage [V]
ITC = (68, 69, 70)                   # zone core temperature [°C]
ITS = (71, 72, 73)                   # zone surface temperature [°C]
ITCOOL = 74                          # coolant temperature [°C]
IPF, IPR = 75, 76                    # brake line pressures [bar]
IBAX = 77                            # accelerometer bias [m/s²]
IBGPS = (78, 79)                     # GPS speed error, receivers 1 and 2 [m/s]
NSTATES17 = 80
NWHEELS = 4
WHEEL_NAMES = ("FL", "FR", "RL", "RR")
SGN = (1.0, -1.0, 1.0, -1.0)         # left +, right −

# what each state block is, for the log and the docs
STATE_BLOCKS = (
    ("chassis", 13), ("suspension", 8), ("wheels", 8), ("steering", 6),
    ("dampers", 4), ("tire transients", 12), ("motors", 8),
    ("battery electrical", 9), ("battery thermal", 7), ("brakes", 2),
    ("sensor bias", 3),
)
assert sum(n for _, n in STATE_BLOCKS) == NSTATES17


class FullVehicleModel:
    """The 80-state plant. Same duck-type as the planar VehicleModel for
    the sim loop: nstates, initial_state(), derivatives(), state_view(),
    sensor_truth(), plant_input(), with_tires()."""

    nstates = NSTATES17
    is_full = True

    def __init__(self, vp: VehicleParams, fp: FullParams, tire_front,
                 tire_rear, tires=None):
        self.p = vp
        self.f = fp
        mf = (tuple(tires) if tires is not None
              else (tire_front, tire_front, tire_rear, tire_rear))
        self.mf_tires = mf                       # the MagicFormulaTire objects
        self.tires = tuple(TransientTire(t) for t in mf)
        c = fp.chassis
        # the body's origin is the SPRUNG CG: it sits where the whole
        # vehicle's CG and the unsprung masses leave it
        L = vp.wheelbase
        m_t, m_s = vp.m_total, c.m_sprung
        b_s = (m_t * vp.b - 2.0 * c.m_unsprung_f * L) / m_s
        a_s = L - b_s
        self.a_s, self.b_s = a_s, b_s
        # wheel attachment points on the sprung body (its CG height), the
        # contact patches at ground level, the unsprung CGs — all measured
        # from the sprung CG, body frame
        self.wheel_xy = (
            ( a_s,  vp.track_f / 2.0),
            ( a_s, -vp.track_f / 2.0),
            (-b_s,  vp.track_r / 2.0),
            (-b_s, -vp.track_r / 2.0),
        )
        self.r_att = tuple((x, y, 0.0) for x, y in self.wheel_xy)
        self.r_cp = tuple((x, y, -c.h_sprung) for x, y in self.wheel_xy)
        # the unsprung masses' inertia reaction enters the body like a
        # reduced tire force at ground level: the link pair (geometric
        # transfer) already carries its roll-centre lever, so the net roll
        # moment comes out as (F_y − m_u a)(h_s − h_rc) — verify.py J5/K3
        self.r_u = self.r_cp
        self.m_u = c.m_unsprung
        self.I = (c.I_xx, c.I_yy, vp.I_z, c.I_xz)

        # static sprung corner loads
        W_s = m_s * G
        F_f, F_r = W_s * b_s / L, W_s * a_s / L
        lf = 0.5 + c.y_cg / vp.track_f
        lr = 0.5 + c.y_cg / vp.track_r
        self.F_static = (F_f * lf, F_f * (1.0 - lf), F_r * lr, F_r * (1.0 - lr))
        # static tire deflection carries the sprung load plus the corner's
        # own unsprung weight
        self.delta_t0 = tuple((self.F_static[i] + self.m_u[i] * G)
                              / self.tires[i].p.k_vertical for i in range(4))

        self.susp = Suspension(fp.susp, vp, self.F_static)
        self.damp = Damper(fp.damp)
        self.steer = Steering(fp.steer, vp, fp.env.v_smooth)
        self.motor = MotorDrive(fp.motor, vp.w_eps, fp.env.w_smooth)
        self.batt = Battery(fp.batt)
        self.brakes = Brakes(fp.brake, fp.env.w_smooth)
        self.I_w = tuple(I + fp.motor.J_gear for I in vp.I_wheel_corner)
        self.cmd = None                          # CommandPath x4, built per run

        # gravity in the ground frame, tilted by road grade and bank
        e = fp.env
        self.g_ground = (-G * math.sin(e.grade), -G * math.sin(e.bank),
                         -G * math.cos(e.grade) * math.cos(e.bank))
        self.wind_ground = (-e.v_wind * math.cos(e.psi_wind),
                            -e.v_wind * math.sin(e.psi_wind), 0.0)

    def with_tires(self, mf_tires):
        """A copy of this plant on a different per-wheel tire set (split-µ)."""
        return FullVehicleModel(self.p, self.f, mf_tires[0], mf_tires[2],
                                tires=tuple(mf_tires))

    # ───────────────────────────────────────────────────── initial state
    def initial_state(self, vx0, delta0):
        """Static equilibrium at speed vx0 on flat ground with the aero load
        of that speed carried by the springs and tires (level L/R, pitched
        by the aero balance), rolling without slip, wheels pointed where the
        steering command puts them, pack at its initial SOC and temperature."""
        p, f = self.p, self.f
        c = f.chassis
        s = [0.0] * NSTATES17
        # aero at vx0 as an extra load per corner
        down = 0.5 * f.env.rho * p.ClA * vx0 * vx0
        dF = (0.5 * p.aero_balance_front * down,) * 2 + \
             (0.5 * (1.0 - p.aero_balance_front) * down,) * 2
        z_u = [-dF[i] / self.tires[i].p.k_vertical for i in range(4)]
        # the spring AND the damper's gas-spring term carry the extra load
        s_defl = [dF[i] / (self.susp.k_w[i] + f.damp.k0) for i in range(4)]
        z_c = [z_u[i] - s_defl[i] for i in range(4)]
        zc_f = 0.5 * (z_c[0] + z_c[1])
        zc_r = 0.5 * (z_c[2] + z_c[3])
        pitch = (zc_r - zc_f) / p.wheelbase          # nose down positive
        Z = c.h_sprung + (zc_f * self.b_s + zc_r * self.a_s) / p.wheelbase
        s[IZ] = Z
        q = q_from_euler(0.0, pitch, 0.0)
        s[IQ0], s[IQ1], s[IQ2], s[IQ3] = q
        # the car moves horizontally over the ground; on the pitched body
        # that is a (u, 0, w) pair, not u alone — otherwise the corners would
        # be climbing at vx0·sin(pitch) and the dampers would react at t = 0
        s[IU], _, s[IWZ] = rot_T(q_to_R(q), (vx0, 0.0, 0.0))
        for i in range(4):
            s[IZU[i]] = z_u[i]
        # steering: the rack where the handwheel puts it, knuckles at the
        # kinematic angles
        hw = math.radians(steer_map_inv_deg(math.degrees(delta0)))
        s[IQR] = self.steer.rack_target(hw)
        dFL, dFR = self.steer.kinematic_angles(s[IQR])
        s[IDFL], s[IDFR] = dFL, dFR
        # free rolling: each wheel at its own contact-patch speed along its
        # own heading, rotor at gear ratio x wheel, no shaft twist
        steers = self.wheel_steer(s)
        for i, (x, y) in enumerate(self.wheel_xy):
            vcx = vx0 * math.cos(steers[i])
            w = vcx / p.r_wheel
            s[IOM[i]] = w
            s[IOMM[i]] = w * f.motor.G
            s[IAT[i]] = -math.atan2(-vx0 * math.sin(steers[i]), max(vcx, p.v_eps))
        # battery
        for j in range(3):
            s[ISOC[j]] = f.batt.soc_0
            s[ITC[j]] = f.batt.T_0
            s[ITS[j]] = f.batt.T_0
        s[ITCOOL] = f.batt.T_0
        return s

    def reset_run(self, dt_vcu):
        """Per-run objects: the four inverter command paths."""
        self.cmd = tuple(CommandPath(self.f.motor, dt_vcu) for _ in range(4))

    # ───────────────────────────────────────────────────── small helpers
    def wheel_steer(self, s):
        """Actual road-wheel angles [rad] FL FR RL RR: knuckle states for the
        fronts plus toe / bump steer on all four."""
        zc = self._corner_heights(s)
        out = []
        for i in range(4):
            sd = s[IZU[i]] - zc[i]
            d = self.susp.toe_steer(i, sd)
            if i == 0:
                d += s[IDFL]
            elif i == 1:
                d += s[IDFR]
            out.append(d)
        return tuple(out)

    def _corner_heights(self, s):
        q = (s[IQ0], s[IQ1], s[IQ2], s[IQ3])
        R = q_to_R(q_normalize(q))
        h_s = self.f.chassis.h_sprung
        return tuple(s[IZ] + rot(R, r)[2] - h_s for r in self.r_att)

    def plant_input(self, dbg, drv, dt_vcu):
        """Build this VCU cycle's plant input from the controller's output.
        Uses the firmware's current command when the controller gives one
        (dbg.I_cmd, amps FL FR RL RR); otherwise (the physics fixtures)
        the equivalent current for the requested wheel torque, T/(Kt·G).
        Pushes it through the inverter command paths (latency + ramp)."""
        if self.cmd is None:
            self.reset_run(dt_vcu)
        m = self.f.motor
        I_req = getattr(dbg, "I_cmd", None)
        if I_req is None:
            I_req = tuple(T / (m.kt * m.G) for T in dbg.T)
        I_app = tuple(self.cmd[i].step(I_req[i]) for i in range(4))
        bps = drv.bps_bar if drv is not None else 0.0
        # the handwheel slot is filled per physics step from the commanded
        # steer angle (input_from_delta); 0 here is never used
        return (0.0, I_app, bps)

    def zero_input(self):
        return (0.0, (0.0, 0.0, 0.0, 0.0), 0.0)

    def input_from_delta(self, u, delta):
        """The sim loop commands the steer angle each step; the handwheel is
        its exact inverse through the steering map."""
        hw = math.radians(steer_map_inv_deg(math.degrees(delta)))
        return (hw, u[1], u[2])

    # ───────────────────────────────────────────────────── derivatives
    def derivatives(self, s, delta, u):
        """ds/dt and an info dict. `delta` is the commanded road-wheel angle
        (turned into the handwheel inside), `u` the (handwheel, currents,
        bps) tuple from plant_input()."""
        p, f = self.p, self.f
        c, e, m = f.chassis, f.env, f.motor
        u = self.input_from_delta(u, delta)
        hw, I_cmd, bps = u

        # ---- body kinematics ------------------------------------------------
        q = q_normalize((s[IQ0], s[IQ1], s[IQ2], s[IQ3]))
        R = q_to_R(q)
        Vb = (s[IU], s[IV], s[IWZ])
        w = (s[IP], s[IQ], s[IR])
        roll, pitch, yaw = euler_from_q(q)
        g_b = rot_T(R, self.g_ground)
        h_s = c.h_sprung
        m_t, m_s = p.m_total, c.m_sprung

        # ---- suspension geometry (algebraic) -------------------------------
        zc, zcd, sd, sdd = [0.0] * 4, [0.0] * 4, [0.0] * 4, [0.0] * 4
        for i in range(4):
            r = self.r_att[i]
            zc[i] = s[IZ] + rot(R, r)[2] - h_s
            vc = cross(w, r)
            zcd[i] = rot(R, (Vb[0] + vc[0], Vb[1] + vc[1], Vb[2] + vc[2]))[2]
            sd[i] = s[IZU[i]] - zc[i]            # compression from static
            sdd[i] = s[IZUD[i]] - zcd[i]

        # ---- tire vertical force (the carcass spring) ----------------------
        Fz = [0.0] * 4
        for i in range(4):
            tp = self.tires[i].p
            Fz[i] = max(0.0, tp.k_vertical * (self.delta_t0[i] - s[IZU[i]])
                        - tp.c_vertical * s[IZUD[i]])

        # ---- suspension force on the body (up) -----------------------------
        F_sp = [0.0] * 4
        for i in range(4):
            F_sp[i] = (self.susp.spring_force(i, sd[i]) + self.susp.arb_force(i, sd)
                       + self.susp.stop_force(i, sd[i])
                       + self.damp.force(i, sdd[i], sd[i], s[IZD[i]]))

        # ---- wheel-frame slips and forces ----------------------------------
        steers = []
        for i in range(4):
            d = self.susp.toe_steer(i, sd[i])
            if i == 0:
                d += s[IDFL]
            elif i == 1:
                d += s[IDFR]
            steers.append(d)
        alpha, kappa = [0.0] * 4, [0.0] * 4
        Fx_w, Fy_w, Fx_b, Fy_b = [0.0] * 4, [0.0] * 4, [0.0] * 4, [0.0] * 4
        Mz_ss, vcx_all, gamma = [0.0] * 4, [0.0] * 4, [0.0] * 4
        for i in range(4):
            vcp = cross(w, self.r_cp[i])
            vxi = Vb[0] + vcp[0]
            vyi = Vb[1] + vcp[1]
            cd, sn = math.cos(steers[i]), math.sin(steers[i])
            vcx = vxi * cd + vyi * sn
            vcy = -vxi * sn + vyi * cd
            vcx_all[i] = vcx
            alpha[i] = -math.atan2(vcy, max(vcx, p.v_eps))
            kappa[i] = (s[IOM[i]] * p.r_wheel - vcx) / max(abs(vcx), p.v_eps)
            gamma[i] = self.susp.camber(i, sd[i], roll)
            Fx_w[i], Fy_w[i] = self.tires[i].forces(
                s[IKT[i]], s[IAT[i]], Fz[i], gamma[i],
                math.tanh(abs(vcx) / e.v_smooth))
            Mz_ss[i] = self.tires[i].aligning_moment(s[IAT[i]], Fy_w[i], Fz[i])
            Fx_b[i] = Fx_w[i] * cd - Fy_w[i] * sn
            Fy_b[i] = Fx_w[i] * sn + Fy_w[i] * cd

        # ---- aero ------------------------------------------------------------
        v_air = Vb[0] - rot_T(R, self.wind_ground)[0]
        q_dyn = 0.5 * e.rho * v_air * abs(v_air)
        dh = 0.25 * (zc[0] + zc[1] + zc[2] + zc[3])
        ClA = p.ClA * (1.0 + c.dCl_dh * dh)
        F_down = q_dyn * ClA
        F_drag = -q_dyn * p.CdA
        down_f = p.aero_balance_front * F_down
        down_r = F_down - down_f

        # ---- body force and moment -------------------------------------------
        Fx = F_drag + m_t * g_b[0]
        Fy = m_t * g_b[1]
        for i in range(4):
            Fx += Fx_b[i]
            Fy += Fy_b[i]
        ax_b, ay_b = Fx / m_t, Fy / m_t           # body-frame accelerations (horizontal)
        # what the unsprung masses are pushed with: the tire (and drag)
        # forces per unit mass = inertial acceleration minus gravity's body
        # projection — the specific force an accelerometer reads
        a_lon = ax_b - g_b[0]
        a_lat = ay_b - g_b[1]
        # the link part of load transfer, now that a_y is known
        dF_u, dF_b = self.susp.geometric_transfer(Fy_b, Fx_b, h_s, a_lat, self.m_u)
        Fz_body = -F_down + m_s * g_b[2]
        for i in range(4):
            Fz_body += F_sp[i] + dF_b[i]

        Mx = My = Mz = 0.0
        for i in range(4):
            # tire horizontal forces at the contact patch
            rc = self.r_cp[i]
            Mx += rc[1] * 0.0 - rc[2] * Fy_b[i]
            My += rc[2] * Fx_b[i] - rc[0] * 0.0
            Mz += rc[0] * Fy_b[i] - rc[1] * Fx_b[i]
            # suspension + the link reaction, vertical at the corner
            Fv = F_sp[i] + dF_b[i]
            ra = self.r_att[i]
            Mx += ra[1] * Fv
            My += -ra[0] * Fv
            # unsprung inertia reaction, through the links at roll-centre height
            ru = self.r_u[i]
            Fu = (-self.m_u[i] * a_lon, -self.m_u[i] * a_lat, 0.0)
            Mx += -ru[2] * Fu[1]
            My += ru[2] * Fu[0]
            # aligning moment
            Mz += s[IMT[i]]
        # aero: drag at the CoP height, downforce at the axles
        My += (c.cop_height - h_s) * F_drag
        My += -(self.a_s * (-down_f)) - (-self.b_s) * (-down_r)
        # (the x-moment of a lateral force at ground level: r_z = −h_s)

        wdot = omega_dot(self.I, w, (Mx, My, Mz))
        wxV = cross(w, Vb)
        ds = [0.0] * NSTATES17
        Xd = rot(R, Vb)
        ds[IX], ds[IY], ds[IZ] = Xd
        ds[IU] = ax_b - wxV[0]
        ds[IV] = ay_b - wxV[1]
        ds[IWZ] = Fz_body / m_s - wxV[2]
        ds[IQ0], ds[IQ1], ds[IQ2], ds[IQ3] = q_dot(q, w)
        ds[IP], ds[IQ], ds[IR] = wdot

        # ---- unsprung masses (vertical) ----------------------------------------
        for i in range(4):
            ds[IZU[i]] = s[IZUD[i]]
            ds[IZUD[i]] = (Fz[i] - F_sp[i] - dF_u[i] + self.m_u[i] * g_b[2]) / self.m_u[i]

        # ---- motors: electromagnetic torque with the pack in the loop ----------
        regen_ok = s[IU] > m.regen_v_cut
        socs = tuple(s[j] for j in ISOC)
        v1s = tuple(s[j] for j in IV1)
        v2s = tuple(s[j] for j in IV2)
        Tcs = tuple(s[j] for j in ITC)
        V_oc, R_pack = self.batt.pack_state(socs, v1s, v2s, Tcs)
        V_bus = V_oc
        T_e = [0.0] * 4
        P_el = 0.0
        I_pack = 0.0
        for _ in range(2):
            P_el = 0.0
            for i in range(4):
                T_e[i] = self.motor.electromagnetic_torque(I_cmd[i], s[IOMM[i]], V_bus, regen_ok)
                P_el += self.motor.electrical_power(T_e[i], s[IOMM[i]])
            I_pack = self.batt.solve_current(P_el, V_oc, R_pack)
            V_bus = max(V_oc - I_pack * R_pack, 1.0)
        # pack / BMS / rules limits as one scale on the torques
        I_dis, I_chg = self.batt.current_window(socs, v1s, v2s, Tcs)
        scale = 1.0
        if P_el > 0.0:
            P_avail = min(self.batt.power_at(I_dis, V_oc, R_pack), m.P_cap_total)
            if P_el > P_avail:
                scale = max(P_avail, 0.0) / P_el
        elif P_el < 0.0:
            P_avail = self.batt.power_at(I_chg, V_oc, R_pack)
            if P_el < P_avail:
                scale = P_avail / P_el
        if scale < 1.0:
            P_el = 0.0
            for i in range(4):
                T_e[i] *= scale
                P_el += self.motor.electrical_power(T_e[i], s[IOMM[i]])
            I_pack = self.batt.solve_current(P_el, V_oc, R_pack)
            V_bus = max(V_oc - I_pack * R_pack, 1.0)

        # ---- shafts, wheels, rotors ---------------------------------------------
        T_s = [0.0] * 4
        T_b = self.brakes.torques(s[IPF], s[IPR], tuple(s[j] for j in IOM))
        for i in range(4):
            ths_dot = s[IOMM[i]] / m.G - s[IOM[i]]
            T_s[i] = self.motor.shaft_torque(s[ITHS[i]], ths_dot)
            T_rr = self.tires[i].rolling_resistance(Fz[i], p.r_wheel, s[IOM[i]], e.w_smooth)
            ds[ITH[i]] = s[IOM[i]]
            ds[IOM[i]] = (T_s[i] - p.r_wheel * Fx_w[i] + T_b[i] + T_rr) / self.I_w[i]
            ds[IOMM[i]] = self.motor.rotor_accel(T_e[i], T_s[i], s[IOMM[i]])
            ds[ITHS[i]] = ths_dot

        # ---- steering ---------------------------------------------------------------
        M_kp = (self.steer.kingpin_moment(Fy_w[0], Fx_w[0], s[IMT[0]], SGN[0]),
                self.steer.kingpin_moment(Fy_w[1], Fx_w[1], s[IMT[1]], SGN[1]))
        (qrdd, dFLdd, dFRdd), kin = self.steer.derivatives(
            s[IQR], s[IQRD], s[IDFL], s[IDFLD], s[IDFR], s[IDFRD], hw, M_kp)
        ds[IQR], ds[IQRD] = s[IQRD], qrdd
        ds[IDFL], ds[IDFLD] = s[IDFLD], dFLdd
        ds[IDFR], ds[IDFRD] = s[IDFRD], dFRdd

        # ---- dampers, tire transients --------------------------------------------
        for i in range(4):
            ds[IZD[i]] = self.damp.z_dot(sdd[i], s[IZD[i]])
            kd, ad, md = self.tires[i].slip_rates(
                kappa[i], alpha[i], Mz_ss[i], s[IKT[i]], s[IAT[i]], s[IMT[i]],
                max(abs(vcx_all[i]), p.v_eps))
            ds[IKT[i]], ds[IAT[i]], ds[IMT[i]] = kd, ad, md

        # ---- battery ----------------------------------------------------------------
        Tss = tuple(s[j] for j in ITS)
        dsoc, dv1, dv2, dtc, dts, dtcool = self.batt.derivatives(
            I_pack, socs, v1s, v2s, Tcs, Tss, s[ITCOOL], e.T_amb)
        for j in range(3):
            ds[ISOC[j]], ds[IV1[j]], ds[IV2[j]] = dsoc[j], dv1[j], dv2[j]
            ds[ITC[j]], ds[ITS[j]] = dtc[j], dts[j]
        ds[ITCOOL] = dtcool

        # ---- brakes ------------------------------------------------------------------
        ds[IPF], ds[IPR] = self.brakes.pressure_rates(s[IPF], s[IPR], bps)
        # sensor biases: random walks advanced by the sensor suite, not here

        info = {"Fz": tuple(Fz), "alpha": alpha, "kappa": kappa,
                "Fx_w": Fx_w, "Fy_w": Fy_w, "Fx_b": Fx_b, "Fy_b": Fy_b,
                "ax": ax_b, "ay": ay_b, "az": Fz_body / m_s,
                "Mz": Mz, "Mx": Mx, "My": My,
                "wdot": wdot, "g_b": g_b, "Vb": Vb, "w": w,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "s": tuple(sd), "sdot": tuple(sdd), "F_susp": tuple(F_sp),
                "dF_geo": dF_u, "dF_body": dF_b, "gamma": tuple(gamma),
                "steer": tuple(steers),
                "T_e": tuple(T_e), "T_s": tuple(T_s), "T_b": T_b,
                "I_cmd": tuple(I_cmd), "P_elec": P_el, "I_pack": I_pack,
                "V_bus": V_bus, "V_oc": V_oc, "F_down": F_down, "F_drag": F_drag,
                "Mz_ss": tuple(Mz_ss), "delta_kin": kin, "hw": hw}
        return ds, info

    # ───────────────────────────────────────────────────── views for sim/sensors
    def state_view(self, s):
        """(X, Y, psi, vx, vy, r, omegas) — what the generic loop logs."""
        _, _, yaw = euler_from_q((s[IQ0], s[IQ1], s[IQ2], s[IQ3]))
        return (s[IX], s[IY], yaw, s[IU], s[IV], s[IR],
                tuple(s[j] for j in IOM))

    def sensor_truth(self, s, info):
        """What the sensor suite samples. The accelerometer channels are the
        SPECIFIC FORCE at the IMU position (ω̇ × r and ω × (ω × r) included,
        gravity removed along the body axes — a tilted body reads g·sinθ)."""
        from model.config import cfg
        imu = cfg.sensors.imu_6axis
        r_imu = (imu.pos_x, imu.pos_y, imu.pos_z)
        w = info["w"]
        wd = info["wdot"]
        g_b = info["g_b"]
        # inertial acceleration of the CG in body axes (ΣF/m including
        # gravity's body projection), transported to the IMU, gravity removed:
        # the specific force an accelerometer reads. On a level body at
        # steady speed this is the planar plant's ax, ay and +g in z.
        a_cg = (info["ax"], info["ay"], info["az"])
        wxr = cross(w, r_imu)
        term = cross(wd, r_imu)
        cen = cross(w, wxr)
        f = tuple(a_cg[k] + term[k] + cen[k] - g_b[k] for k in range(3))
        gps = cfg.sensors.gps
        v_ant = (s[IU] - s[IR] * gps.antenna_y, s[IV] + s[IR] * gps.antenna_x)
        return SensorTruth(
            yaw_rate=s[IR], p=s[IP], q=s[IQ],
            ax=f[0], ay=f[1], az=f[2],
            omega_wheel=tuple(s[j] for j in IOM),
            omega_motor=tuple(s[j] for j in IOMM),
            v_ground=math.hypot(v_ant[0], v_ant[1]),
            b_ax=s[IBAX], b_gps=(s[IBGPS[0]], s[IBGPS[1]]), has_bias=True)

    def apply_bias_walk(self, s, d_ax, d_gps):
        s[IBAX] += d_ax
        s[IBGPS[0]] += d_gps[0]
        s[IBGPS[1]] += d_gps[1]

    def diverged(self, s):
        beta = abs(math.atan2(s[IV], max(s[IU], 0.5)))
        return (abs(s[IR]) > 8.0 or abs(s[IV]) > 15.0 or beta > 1.0
                or abs(s[IZ]) > 5.0 * self.f.chassis.h_sprung)

    # extra log channels beyond the generic set
    EXTRA_LOG = ("Z", "vz", "roll", "pitch", "p", "q",
                 "sFL", "sFR", "sRL", "sRR",
                 "dFL", "dFR", "rack",
                 "Te_FL", "Te_FR", "Te_RL", "Te_RR",
                 "Ts_FL", "Ts_FR", "Ts_RL", "Ts_RR",
                 "Tb_FL", "Tb_FR", "Tb_RL", "Tb_RR",
                 "I_FL", "I_FR", "I_RL", "I_RR",
                 "wm_FL", "wm_FR", "wm_RL", "wm_RR",
                 "ktFL", "ktFR", "ktRL", "ktRR",
                 "P_elec", "I_pack", "V_bus", "SOC", "T_core_max", "T_cool",
                 "P_f", "P_r", "F_down", "b_ax", "b_gps1", "b_gps2",
                 "Fy_f", "Fy_r", "Fx_f", "Fx_r")

    def log_extra(self, s, info):
        _, _, _ = info["roll"], info["pitch"], info["yaw"]
        out = {"Z": s[IZ], "vz": s[IWZ], "roll": info["roll"],
               "pitch": info["pitch"], "p": s[IP], "q": s[IQ],
               "dFL": s[IDFL], "dFR": s[IDFR], "rack": s[IQR],
               "P_elec": info["P_elec"], "I_pack": info["I_pack"],
               "V_bus": info["V_bus"],
               "SOC": (s[ISOC[0]] + s[ISOC[1]] + s[ISOC[2]]) / 3.0,
               "T_core_max": max(s[j] for j in ITC), "T_cool": s[ITCOOL],
               "P_f": s[IPF], "P_r": s[IPR], "F_down": info["F_down"],
               "b_ax": s[IBAX], "b_gps1": s[IBGPS[0]], "b_gps2": s[IBGPS[1]],
               "Fy_f": info["Fy_b"][0] + info["Fy_b"][1],
               "Fy_r": info["Fy_b"][2] + info["Fy_b"][3],
               "Fx_f": info["Fx_b"][0] + info["Fx_b"][1],
               "Fx_r": info["Fx_b"][2] + info["Fx_b"][3]}
        for i, nm in enumerate(WHEEL_NAMES):
            out["s" + nm] = info["s"][i]
            out["Te_" + nm] = info["T_e"][i]
            out["Ts_" + nm] = info["T_s"][i]
            out["Tb_" + nm] = info["T_b"][i]
            out["I_" + nm] = info["I_cmd"][i]
            out["wm_" + nm] = s[IOMM[i]]
            out["kt" + nm] = s[IKT[i]]
        return out


