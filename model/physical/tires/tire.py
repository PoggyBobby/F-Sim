"""Simplified Pacejka "Magic Formula" tire with load sensitivity and a
friction-circle combined-slip cap.

Pure-slip force (same form for lateral and longitudinal):

    F(s) = D * sin( C * atan( B*s - E*(B*s - atan(B*s)) ) )

with  D = mu(Fz) * Fz            (peak force)
      B = c / (C * mu(Fz))       (stiffness factor, chosen so that the
                                  small-slip stiffness is  B*C*D = c*Fz)
      mu(Fz) = mu0 * (1 - s_mu*(Fz/Fz_nom - 1))   (tire load sensitivity:
                                  grip *coefficient* falls as load rises —
                                  this is why load transfer costs grip and
                                  why TV torque placement matters)

LONGITUDINAL vs LATERAL GRIP (upgrade 2026-08-31, driven by TTC data):
the fitted R20 makes ~20% more longitudinal than lateral grip — a shared µ
cannot represent that. The model carries separate µ₀ values (mu0 lateral,
mu0x longitudinal, shared load sensitivity), and combined slip is capped on
the friction ELLIPSE:

    (Fx/(µx·Fz))² + (Fy/(µy·Fz))² ≤ 1

(Real combined-slip behavior is still subtler — full MF combined model
belongs to the team's full-vehicle sim.)

Coefficients: TTC Round 9 fit (R20 surrogates) — see car_data.py tires
block for provenance and the belt→asphalt scaling.
"""

import math
from params import TireParams


class MagicFormulaTire:
    def __init__(self, p: TireParams):
        self.p = p

    def mu(self, Fz: float) -> float:
        """LATERAL friction coefficient at this load (load sensitivity),
        floored at half nominal so extrapolation can't go negative."""
        p = self.p
        mu = p.mu0 * (1.0 - p.s_mu * (Fz / p.Fz_nom - 1.0))
        return max(mu, 0.5 * p.mu0)

    def mu_x(self, Fz: float) -> float:
        """LONGITUDINAL friction coefficient — same load-sensitivity slope,
        its own peak (TTC: R20 drives/brakes ~20% harder than it corners)."""
        p = self.p
        mu = p.mu0x * (1.0 - p.s_mu * (Fz / p.Fz_nom - 1.0))
        return max(mu, 0.5 * p.mu0x)

    @staticmethod
    def _mf(slip: float, B: float, C: float, D: float, E: float) -> float:
        Bs = B * slip
        return D * math.sin(C * math.atan(Bs - E * (Bs - math.atan(Bs))))

    def lateral(self, alpha: float, Fz: float) -> float:
        """Pure lateral force from slip angle alpha [rad]."""
        if Fz <= 0.0:
            return 0.0          # wheel off the ground carries no force
        p = self.p
        mu = self.mu(Fz)
        D = mu * Fz
        B = p.c_alpha / (p.C_y * mu)
        return self._mf(alpha, B, p.C_y, D, p.E_y)

    def longitudinal(self, kappa: float, Fz: float) -> float:
        """Pure longitudinal force from slip ratio kappa [-]."""
        if Fz <= 0.0:
            return 0.0
        p = self.p
        mu = self.mu_x(Fz)
        D = mu * Fz
        B = p.c_kappa / (p.C_x * mu)
        return self._mf(kappa, B, p.C_x, D, p.E_x)

    def kappa_at_peak(self, Fz: float, k_max: float = 0.8, n: int = 321):
        """Slip ratio where pure longitudinal force PEAKS at this load.

        Below it a driven wheel is just working; above it the tire is on the
        falling side of the curve, so more wheel speed buys less force — the
        wheel is running away. That makes it the honest threshold for
        "this wheel is spinning", and because it is computed from the tire
        coefficients it moves on its own when real TTC data replaces the
        placeholders."""
        best_k, best_F = 0.0, -1.0
        for i in range(1, n + 1):
            k = k_max * i / n
            F = self.longitudinal(k, Fz)
            if F > best_F:
                best_k, best_F = k, F
        return best_k

    def combined(self, kappa: float, alpha: float, Fz: float):
        """(Fx, Fy) under combined slip: pure-slip forces capped onto the
        friction ELLIPSE (µx·Fz longitudinal semi-axis, µy·Fz lateral)."""
        if Fz <= 0.0:
            return 0.0, 0.0
        Fx = self.longitudinal(kappa, Fz)
        Fy = self.lateral(alpha, Fz)
        u = math.hypot(Fx / (self.mu_x(Fz) * Fz), Fy / (self.mu(Fz) * Fz))
        if u > 1.0:
            Fx /= u
            Fy /= u
        return Fx, Fy
