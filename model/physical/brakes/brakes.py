"""Brake hydraulics (2 states: front and rear line pressure) and friction torque.

    Ṗ_f = (P_cmd,f − P_f) / τ_f        P_cmd,f = min(bps, P_max)
    Ṗ_r = (P_cmd,r − P_r) / τ_r        P_cmd,r = P_cmd,f · (1 − bias) / bias
    T_b,i = P · A_p · μ_pad · R_eff · N_surfaces      (P in Pa)

The driver's pedal gives the FRONT line pressure (that is what the BPS
reads); the balance bar sets the rear. The torque opposes wheel rotation
through tanh(ω / w_smooth) so a braked wheel can stop instead of chattering
about zero.

Values: model/physical/brakes/params.yaml.
"""

import math


class Brakes:
    def __init__(self, bp, w_smooth):
        self.p = bp
        self.w_sm = w_smooth
        self.gain = (bp.gain_f, bp.gain_f, bp.gain_r, bp.gain_r)

    def commands(self, bps_bar):
        """(P_cmd_front, P_cmd_rear) [bar] from the pedal's line pressure."""
        Pf = max(0.0, min(bps_bar, self.p.P_max))
        return Pf, Pf * self.p.rear_over_front

    def pressure_rates(self, P_f, P_r, bps_bar):
        cf, cr = self.commands(bps_bar)
        return (cf - P_f) / self.p.tau_f, (cr - P_r) / self.p.tau_r

    def torques(self, P_f, P_r, omegas):
        """Four brake torques [N·m], signed to oppose each wheel's rotation."""
        P = (P_f, P_f, P_r, P_r)
        return tuple(-self.gain[i] * max(P[i], 0.0) * math.tanh(omegas[i] / self.w_sm)
                     for i in range(4))
