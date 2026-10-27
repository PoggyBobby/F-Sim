"""Tire transients, aligning moment and camber thrust for the full plant.

Wraps a MagicFormulaTire (the steady-state force law the planar plant uses)
with the three per-wheel states of the DOF doc:

    κ̃̇ = |v_cx| / σ_κ · (κ − κ̃)        effective slip ratio
    α̃̇ = |v_cx| / σ_α · (α − α̃)        effective slip angle
    M̃ż = |v_cx| / σ_M · (M_z − M̃_z)    effective aligning moment

The force law is evaluated at the EFFECTIVE slips, so a step in wheel torque
builds force over a rolling distance σ rather than instantly; at low speed
the time constant σ/|v| grows, which is the lag a slip controller overshoots
against. |v_cx| is floored at the plant's v_eps so a stopped tire still
relaxes (it never truly stops relaxing; the floor is a numerical guard).

Aligning moment (steady state): M_z = −t_p(α, Fz) · F_y with a pneumatic
trail t_p = t_p0 · (Fz/Fz_nom) · max(0, 1 − |α|/α_t) — the trail shrinks to
zero as the patch slides, which is where the steering goes light. Signed so
that it turns the wheel toward the velocity (reduces |α|).

Camber thrust: F_y,γ = C_γ · (Fz/Fz_nom) · γ, toward the side the top of the
wheel leans. Added to the lateral force and then the pair is re-capped on
the friction ellipse so camber can never buy grip the tire does not have.

Coefficients: model/physical/tires/params.yaml (the transient, trail and
camber entries are PLACEHOLDERS until tire_fit.py is run on the TTC Mz and
camber channels).
"""

import math

from model.physical.tires.tire import MagicFormulaTire


class TransientTire:
    def __init__(self, mf: MagicFormulaTire):
        self.mf = mf
        self.p = mf.p

    # ---------------------------------------------------------- relaxation
    def slip_rates(self, kappa, alpha, Mz_ss, kappa_t, alpha_t, Mz_t, v_cx):
        """Time derivatives of the three effective-slip states."""
        p = self.p
        v = abs(v_cx)
        return ((kappa - kappa_t) * v / p.sigma_kappa,
                (alpha - alpha_t) * v / p.sigma_alpha,
                (Mz_ss - Mz_t) * v / p.sigma_mz)

    # ------------------------------------------------------------- forces
    def forces(self, kappa_t, alpha_t, Fz, gamma, rolling=1.0):
        """(Fx, Fy) in the wheel frame at the effective slips, with camber
        thrust, capped on the friction ellipse. `rolling` (0..1) scales the
        camber thrust: a tire makes it by rolling, so the plant passes
        tanh(|v_cx| / v_smooth) and a parked car pushes on nothing."""
        if Fz <= 0.0:
            return 0.0, 0.0
        mf = self.mf
        Fx, Fy = mf.combined(kappa_t, alpha_t, Fz)
        Fy += rolling * self.camber_force(gamma, Fz)
        u = math.hypot(Fx / (mf.mu_x(Fz) * Fz), Fy / (mf.mu(Fz) * Fz))
        if u > 1.0:
            Fx /= u
            Fy /= u
        return Fx, Fy

    def camber_force(self, gamma, Fz):
        p = self.p
        return p.camber_stiffness * (Fz / p.Fz_nom) * gamma

    def aligning_moment(self, alpha_t, Fy, Fz):
        """Steady-state M_z = −t_p · F_y (the value the M̃_z state relaxes to)."""
        if Fz <= 0.0:
            return 0.0
        p = self.p
        t = p.trail_0 * (Fz / p.Fz_nom) * max(0.0, 1.0 - abs(alpha_t) / p.trail_alpha)
        return -t * Fy

    def rolling_resistance(self, Fz, r_w, omega, w_smooth):
        """Rolling-resistance torque on the wheel [N·m], opposing rotation."""
        return -self.p.f_rr * max(Fz, 0.0) * r_w * math.tanh(omega / w_smooth)
