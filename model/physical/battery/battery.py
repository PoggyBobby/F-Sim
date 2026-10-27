"""Battery: 3-zone equivalent circuit (9 states) + thermal (7 states).

Electrical, per cell in zone j (I_c = pack current / n_p):

    ṠOC_j = −η_c I_c / Q_cap
    V̇1_j  = −V1_j / (R1 C1) + I_c / C1          fast polarisation
    V̇2_j  = −V2_j / (R2 C2) + I_c / C2          slow polarisation
    V_cell,j = OCV(SOC_j) − I_c R0(T_j) − V1_j − V2_j

The pack is n_s cells in series split evenly over the zones, so the pack
open-circuit voltage and resistance are sums over zones; the current that
supplies the motors' electrical power P is the root of

    R_pack I² − V_oc I + P = 0

(the smaller root: the physical one). Pack voltage is V_oc − I R_pack.

Limits the BMS/cells impose, returned as a current window the plant uses to
scale the motors' torque: the cell discharge/charge current times n_p, the
thermal derate (linear from derate_start to derate_end on the hottest core),
and the cell voltage floor/ceiling converted exactly to a current:
I ≤ n_p (OCV − V1 − V2 − V_min) / R0 for every zone.

Thermal, per zone (heat = I_c² R0 + I_c (V1 + V2), times the zone's cells):

    C_core Ṫ_core = Q̇ − (T_core − T_surf) / R_cs
    C_surf Ṫ_surf = (T_core − T_surf) / R_cs − (T_surf − T_cool) / R_sc,j
    C_cool Ṫ_cool = Σ_j (T_surf,j − T_cool) / R_sc,j − UA (T_cool − T_amb)

R_sc of the middle zone is f_hot × the others, which is what makes a hot
spot (and therefore a derate) emerge from how hard the motors drive the pack
rather than being scripted.

Values: battery/params.yaml and battery_thermal/params.yaml. The OCV is a
straight line between ocv_at_empty and ocv_at_full until a table exists.
"""

import math


class Battery:
    def __init__(self, bp, T_ref=25.0):
        self.p = bp
        self.T_ref = T_ref                        # the temperature R0_ref is quoted at
        self.nz = bp.n_zones
        mid = self.nz // 2
        self.R_sc = tuple(bp.R_sc * (bp.f_hot if j == mid else 1.0)
                          for j in range(self.nz))
        self.n_cells_zone = bp.n_s_zone * bp.n_p

    # ------------------------------------------------------------ cell laws
    def ocv(self, soc):
        p = self.p
        return p.ocv_0 + (p.ocv_1 - p.ocv_0) * soc

    def r0(self, T):
        p = self.p
        return p.R0_ref * max(1.0 + p.R0_tc * (T - self.T_ref), 0.1)

    # ------------------------------------------------------------ pack
    def pack_state(self, socs, v1s, v2s, T_cores):
        """(V_oc_pack, R_pack) from the zone states."""
        p = self.p
        V_oc = 0.0
        R = 0.0
        for j in range(self.nz):
            V_oc += p.n_s_zone * (self.ocv(socs[j]) - v1s[j] - v2s[j])
            R += p.n_s_zone * self.r0(T_cores[j]) / p.n_p
        return V_oc, R

    def current_window(self, socs, v1s, v2s, T_cores):
        """(I_max_discharge, I_max_charge[negative]) the pack may carry now:
        cell current limits × n_p, the thermal derate on discharge, and the
        voltage floor/ceiling converted to currents, zone by zone."""
        p = self.p
        T_hot = max(T_cores)
        derate = (p.T_derate_1 - T_hot) / (p.T_derate_1 - p.T_derate_0)
        derate = max(0.0, min(1.0, derate))
        I_dis = p.n_p * p.I_dis_cell * derate
        I_chg = -p.n_p * p.I_chg_cell
        for j in range(self.nz):
            e = self.ocv(socs[j]) - v1s[j] - v2s[j]
            r = self.r0(T_cores[j])
            I_dis = min(I_dis, p.n_p * (e - p.V_min) / r)
            I_chg = max(I_chg, p.n_p * (e - p.V_max) / r)
        return max(I_dis, 0.0), min(I_chg, 0.0)

    @staticmethod
    def solve_current(P, V_oc, R):
        """Pack current that delivers electrical power P [W] (negative =
        regen) through resistance R from open-circuit voltage V_oc: the
        physical root of R I² − V_oc I + P = 0. Beyond the pack's maximum
        power V_oc²/(4R) the current saturates at the max-power point."""
        disc = V_oc * V_oc - 4.0 * R * P
        if disc < 0.0:
            return V_oc / (2.0 * R)
        return (V_oc - math.sqrt(disc)) / (2.0 * R)

    @staticmethod
    def power_at(I, V_oc, R):
        return I * (V_oc - I * R)

    # ------------------------------------------------------------ dynamics
    def derivatives(self, I, socs, v1s, v2s, T_cores, T_surfs, T_cool, T_amb):
        """Returns (dSOC[nz], dV1[nz], dV2[nz], dTcore[nz], dTsurf[nz], dTcool)."""
        p = self.p
        I_c = I / p.n_p
        dsoc, dv1, dv2, dtc, dts = [], [], [], [], []
        sum_to_cool = 0.0
        for j in range(self.nz):
            dsoc.append(-p.eta_c * I_c / p.Q_cap)
            dv1.append(-v1s[j] / (p.R1 * p.C1) + I_c / p.C1)
            dv2.append(-v2s[j] / (p.R2 * p.C2) + I_c / p.C2)
            Q = self.n_cells_zone * (I_c * I_c * self.r0(T_cores[j])
                                     + I_c * (v1s[j] + v2s[j]))
            q_cs = (T_cores[j] - T_surfs[j]) / p.R_cs
            q_sc = (T_surfs[j] - T_cool) / self.R_sc[j]
            dtc.append((Q - q_cs) / p.C_core)
            dts.append((q_cs - q_sc) / p.C_surf)
            sum_to_cool += q_sc
        dtcool = (sum_to_cool - p.UA * (T_cool - T_amb)) / p.C_cool
        return dsoc, dv1, dv2, dtc, dts, dtcool
