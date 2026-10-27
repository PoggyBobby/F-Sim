"""Dampers: bilinear digressive curve + Bouc-Wen hysteresis state.

    F_d = c(v)·v + k0·s + α·z          v = compression velocity (bump > 0)
    dz/dt = A·v − β·|v|·|z|^(n−1)·z − γ·v·|z|^n

c(v) is piecewise linear with a knee: slope c_low up to v_knee, c_high
beyond, continuous in force, separately for bump and rebound. With α = 0
the state z is decoupled (it still integrates, bounded at |z| ≤ 1 for
β + γ = 1) and the damper is exactly the curve — verify.py K1.

Values: model/physical/dampers/params.yaml (all at the wheel).
"""


class Damper:
    def __init__(self, dp):
        self.p = dp
        f = (dp.c_bump_low_f, dp.c_bump_high_f, dp.c_reb_low_f, dp.c_reb_high_f)
        r = (dp.c_bump_low_r, dp.c_bump_high_r, dp.c_reb_low_r, dp.c_reb_high_r)
        self.c = (f, f, r, r)

    def curve_force(self, i, v):
        """c(v)·v for corner i: the dyno curve alone."""
        bl, bh, rl, rh = self.c[i]
        knee = self.p.v_knee
        av = abs(v)
        if v >= 0.0:
            lo, hi = bl, bh
        else:
            lo, hi = rl, rh
        if av <= knee:
            return lo * v
        mag = lo * knee + hi * (av - knee)
        return mag if v > 0.0 else -mag

    def force(self, i, v, s, z):
        """Total damper force at the wheel, positive = pushes the body up."""
        return self.curve_force(i, v) + self.p.k0 * s + self.p.alpha * z

    def z_dot(self, v, z):
        p = self.p
        az = abs(z)
        azn1 = az ** (p.bw_n - 1.0) if p.bw_n != 1.0 else 1.0
        return p.bw_A * v - p.bw_beta * abs(v) * azn1 * z - p.bw_gamma * v * azn1 * az
