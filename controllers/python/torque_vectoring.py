"""Torque vectoring (TV) — PARKED. Nothing in the sim imports this file.

Pulled out of torque_split.py on 2026-09-04: the sim is s-diff only for now.
Everything needed to plug TV back in lives here — the gains that used to sit
in controllers/python/params.yaml, the understeer-gradient / yaw-rate
reference model, and the PI yaw-moment step that produced dT_tv.

TV makes the YAW RATE track a reference computed from what the driver is
asking for (steering angle + speed), using the steady-state single-track
(bicycle) model:
    r_ref = vx * delta / (L + K_us * vx^2)
    K_us  = m/L * ( b/C_f - a/C_r )        (understeer gradient)
capped by the friction-limited lateral acceleration  |r| <= ay_max/vx,
with ay_max = mu*(g + downforce/m), mu = mean of the front/rear tire mu0.
A PI controller on e = r_ref - r commands a yaw moment M_z, produced by a
left/right longitudinal force difference at the rear axle:
    M_z = (track_r/2) * (Fx_RR - Fx_RL) = (track_r/2) * dT/r_wheel
    =>  dT_tv = 2 * M_z * r_wheel / track_r
Combined with the s-diff the dT contributions simply sum; they are
complementary — the s-diff regulates wheel SPEEDS, TV the body YAW RATE.

To re-enable: put the gains back in params.yaml / ControlParams, give the
controller a tv_on flag and an i_tv integrator, log r_ref / dT_tv again, and
add the tv_step() output to dT before _apply_limits().
"""
from model.params import G, RHO_AIR

# as last tuned in sim (2026-08-29) against the report-based car
TV_GAINS = {
    "kp_tv":    2500.0,  # N*m/(rad/s)  yaw moment per rad/s of yaw-rate error
    "ki_tv":    1000.0,  # N*m/rad      yaw moment per accumulated rad of error
    "i_tv_max":  200.0,  # N*m          anti-windup clamp on the integral
    "mz_max":    600.0,  # N*m          clamp on the commanded yaw moment
    "ay_frac":     0.95, # -            reference capped at this fraction of mu*g_eff
}


def understeer_gradient(vp, tp_front, tp_rear):
    """K_us [s^2/m] from the linearized tire stiffnesses at static axle
    loads:  C_axle = c_alpha * Fz_axle  (per axle, both tires). >0 = understeer."""
    m, L = vp.m_total, vp.wheelbase
    C_f = tp_front.c_alpha * m * G * vp.b / L
    C_r = tp_rear.c_alpha * m * G * vp.a / L
    return m / L * (vp.b / C_f - vp.a / C_r)


def yaw_rate_ref(vp, K_us, mu_ref, ay_frac, vx, delta):
    """Bicycle-model yaw-rate reference, friction-capped."""
    vx_s = max(vx, 1.0)
    r_ref = vx_s * delta / (vp.wheelbase + K_us * vx_s * vx_s)
    downforce = 0.5 * RHO_AIR * vp.ClA * vx * vx
    ay_max = mu_ref * (G + downforce / vp.m_total)
    r_cap = ay_frac * ay_max / vx_s
    return max(-r_cap, min(r_cap, r_ref))


def tv_step(vp, gains, i_tv, r_ref, r, dt):
    """One PI update on the yaw-rate error. Returns (dT_tv, Mz, i_tv)."""
    e = r_ref - r
    i_tv += gains["ki_tv"] * e * dt
    i_tv = max(-gains["i_tv_max"], min(gains["i_tv_max"], i_tv))
    Mz = gains["kp_tv"] * e + i_tv
    Mz = max(-gains["mz_max"], min(gains["mz_max"], Mz))
    return 2.0 * Mz * vp.r_wheel / vp.track_r, Mz, i_tv
