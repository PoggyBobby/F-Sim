"""Track (corner) test catalog — the test matrix for ranking the torque
allocation modes.

The five corner types from the s-diff plan, 2–3 radii each, plus one
split-µ case:

    30°, 45°, 90°, 120°, U-turn (180°)          entry at 90 % of √(µ g R)
    throttle STEP to full at the apex             then unwind onto a straight
    split-µ: the 90° corner with the INNER rear on a low-grip patch

Each test is one `TrackManeuver`. Unlike the scripted maneuvers in
maneuvers.py these are driven CLOSED-LOOP by a minimal "driver
replacement" (`TrackDriver`):

  * steer:   fixed kinematic angle  δ = atan(L / R)  (≈ L/R) after a fast
             turn-in; the wheel is unwound when the car's HEADING has swept
             the corner angle — so a 90° corner is a 90° corner whatever
             speed the car exits at;
  * speed:   PI speed hold at the entry speed until the apex (heading =
             half the corner angle), then a throttle step to the apex
             torque (default: full) held to the end of the run.

Every controller config drives the same TrackDriver (state reset per run),
so differences between configs are still the torque split alone — with the
one honest caveat that the driver reacts to the car, so a config that exits
faster reaches the unwind point earlier.

Nothing about the car is assumed here beyond what the params.yaml files provide:
wheelbase (steer), mass / wheel radius / drag (speed-hold gains and
feed-forward), peak torque (apex step), tire µ₀ (entry speed).

Recorded per test (Maneuver.params, so runs can never be confused):
    corner_deg, radius_m, entry_speed_mps, entry_frac, apex_throttle_pct,
    direction (+1 left, −1 right), mu_inner_scale (split-µ only)
"""

import math
from dataclasses import dataclass, replace

from model.config import cfg
from model.maneuvers.maneuvers import Maneuver, _ramp
from model.params import VehicleParams, G, RHO_AIR
from model.physical.tires.tire import MagicFormulaTire
from model.physical.vehicle import VehicleModel, IVX, IPSI

# ── the catalog ──────────────────────────────────────────────────────────
# slug -> (corner angle [deg], default radii [m]). Radii are chosen so the
# 90 %-of-limit entry speed stays inside the endurance envelope (~18 m/s,
# maneuvers.py) and the tightest steer stays inside the 23° road-wheel
# lock: atan(1.52 / 4.5) = 18.7°.
CORNER_TYPES = {
    "30deg":  (30.0,  (12.0, 22.0)),
    "45deg":  (45.0,  (9.0, 16.0)),
    "90deg":  (90.0,  (6.0, 9.0, 14.0)),
    "120deg": (120.0, (5.0, 8.0)),
    "u_turn": (180.0, (4.5, 6.5, 9.0)),
}

# split-µ case: (corner slug, radius, µ scale on the INNER rear tire).
# Where the LSD-style s-diff and per-wheel slip control differ most.
SPLIT_MU_CASE = ("90deg", 9.0, 0.6)

DEFAULT_ENTRY_FRAC = 0.9        # entry speed = frac · √(µ g R)   (plan §3)
DEFAULT_THROTTLE_PCT = 100.0    # apex step, % of peak axle torque (plan §3)

WHEEL_INDEX = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}


def entry_speed(radius, mu=None, frac=DEFAULT_ENTRY_FRAC):
    """Corner entry speed: frac · √(µ g R). µ defaults to the tire's
    nominal LATERAL peak from the tire params (the number the plan means)."""
    mu = cfg.tires.mu0 if mu is None else mu
    return frac * math.sqrt(mu * G * radius)


def kinematic_steer(vp: VehicleParams, radius):
    """Bicycle-model steer for radius R: δ = atan(L/R) (→ L/R for R ≫ L)."""
    return math.atan(vp.wheelbase / radius)


# ── the driver replacement ───────────────────────────────────────────────
class TrackDriver:
    """(t, state) → (road-wheel steer [rad], total torque request [N·m]).

    Phases (heading ψ measured in the turn direction):
        0 … t_in              straight, PI speed hold at v_entry
        t_in … +t_turnin      steer ramps 0 → δ_c
        arc, ψ < θ/2          hold δ_c, PI speed hold
        apex  ψ ≥ θ/2         throttle STEP to T_apex, held from here on
        ψ ≥ θ − lead          steer ramps δ_c → 0 over t_unwind (lead = the
                              heading still swept during the ramp itself)
        then                  straight, T_apex held until the run ends

    Auto-resets when called with a time earlier than the previous call, so
    the same driver object serves every controller config in turn.
    """

    def __init__(self, vp: VehicleParams, corner_deg, radius, v_entry,
                 T_apex, direction=1, t_in=0.5, t_turnin=0.25,
                 t_unwind=0.3, hold_frac=0.5):
        self.vp = vp
        self.theta = math.radians(corner_deg)
        self.R = radius
        self.dir = 1.0 if direction >= 0 else -1.0
        self.v_entry = v_entry
        self.T_apex = T_apex
        self.delta_c = kinematic_steer(vp, radius)
        self.t_in, self.t_turnin, self.t_unwind = t_in, t_turnin, t_unwind
        # speed hold: torque per m/s of error ≈ (4 m/s² per m/s) · m · r_w,
        # integral over ~0.5 s, capped so the hold itself can't be the
        # wheelspin event (it only has to cover drag + tire induced drag)
        m, r_w = vp.m_total, vp.r_wheel
        self.kp = 4.0 * m * r_w
        self.ki = self.kp / 0.5
        self.T_hold_max = hold_frac * 2.0 * vp.T_wheel_max
        # heading targets
        self.psi_apex = 0.5 * self.theta
        r_kin = v_entry / radius
        self.psi_unwind = self.theta - 0.5 * r_kin * t_unwind
        self.reset()

    def reset(self):
        self.t_apex = None
        self.t_unwind_start = None
        self.integ = 0.0
        self.t_last = None

    # feed-forward: aero drag at the hold speed
    def _T_ff(self, v):
        return self.vp.r_wheel * 0.5 * RHO_AIR * self.vp.CdA * v * v

    def steer_open_loop(self, t):
        """Turn-in only (no unwind) — what Maneuver.inputs reports."""
        return self.dir * _ramp(t, self.t_in, self.t_in + self.t_turnin,
                                0.0, self.delta_c)

    def __call__(self, t, s):
        if self.t_last is None or t < self.t_last:
            self.reset()
        dt = 0.0 if self.t_last is None else t - self.t_last
        self.t_last = t

        vx = s[IVX]
        psi = self.dir * s[IPSI]

        # steering
        delta = _ramp(t, self.t_in, self.t_in + self.t_turnin, 0.0, self.delta_c)
        if self.t_unwind_start is None and psi >= self.psi_unwind:
            self.t_unwind_start = t
        if self.t_unwind_start is not None:
            delta = _ramp(t, self.t_unwind_start,
                          self.t_unwind_start + self.t_unwind, self.delta_c, 0.0)

        # throttle
        if self.t_apex is None and psi >= self.psi_apex:
            self.t_apex = t
        if self.t_apex is None:
            e = self.v_entry - vx
            self.integ += self.ki * e * dt
            self.integ = max(0.0, min(self.T_hold_max, self.integ))
            T = self._T_ff(self.v_entry) + self.kp * e + self.integ
            T = max(0.0, min(self.T_hold_max, T))
        else:
            T = self.T_apex
        return self.dir * delta, T


# ── the maneuver object ──────────────────────────────────────────────────
@dataclass
class TrackManeuver(Maneuver):
    driver: TrackDriver = None       # closed-loop inputs (sim.py prefers it)
    mu_patch: dict = None            # wheel index -> µ scale (split-µ), or None
    corner_deg: float = 0.0
    radius_m: float = 0.0


def track_maneuver(vp: VehicleParams, corner_slug, radius, throttle_pct=None,
                   entry_frac=None, direction=1, mu_inner_scale=None,
                   t_exit=2.0):
    """One corner test. mu_inner_scale ≠ None makes it the split-µ case."""
    if corner_slug not in CORNER_TYPES:
        raise KeyError(f"unknown corner type {corner_slug!r}; "
                       f"known: {', '.join(CORNER_TYPES)}")
    throttle_pct = DEFAULT_THROTTLE_PCT if throttle_pct is None else throttle_pct
    entry_frac = DEFAULT_ENTRY_FRAC if entry_frac is None else entry_frac
    theta_deg = CORNER_TYPES[corner_slug][0]
    v0 = entry_speed(radius, frac=entry_frac)
    T_apex = throttle_pct / 100.0 * 2.0 * vp.T_wheel_max
    drv = TrackDriver(vp, theta_deg, radius, v0, T_apex, direction=direction)

    # run length: entry + turn-in + the arc at entry speed + unwind + exit
    # straight. The car exits faster than it entered, so the arc is shorter
    # in practice and the exit straight correspondingly longer.
    arc_time = math.radians(theta_deg) * radius / v0
    duration = drv.t_in + drv.t_turnin + arc_time + drv.t_unwind + t_exit

    def inputs(t):        # open-loop view for tools that only know inputs(t)
        return drv.steer_open_loop(t), drv._T_ff(v0)

    side = "L" if direction >= 0 else "R"
    slug = f"track_{corner_slug}_R{radius:g}{'' if direction >= 0 else '_rh'}"
    name = f"{corner_slug.replace('deg', '°').replace('u_turn', 'U-turn')} R={radius:g} m"
    desc = (f"entry {v0:.1f} m/s ({entry_frac:.0%} of √(µgR)), "
            f"δ={math.degrees(drv.delta_c):.1f}°, apex step to "
            f"{throttle_pct:.0f}% ({T_apex:.0f} N·m)")
    params = {"corner_deg": theta_deg, "radius_m": radius,
              "entry_speed_mps": v0, "entry_frac": entry_frac,
              "apex_throttle_pct": throttle_pct, "direction": float(direction),
              "duration_s": duration}

    mu_patch = None
    if mu_inner_scale is not None:
        inner = WHEEL_INDEX["RL" if direction >= 0 else "RR"]
        mu_patch = {inner: mu_inner_scale}
        slug += "_splitmu"
        name += f" split-µ (inner ×{mu_inner_scale:g})"
        desc += f", inner rear µ ×{mu_inner_scale:g}"
        params["mu_inner_scale"] = mu_inner_scale

    return TrackManeuver(name, slug, duration, v0, inputs, desc,
                         params=params, driver=drv, mu_patch=mu_patch,
                         corner_deg=theta_deg, radius_m=radius)


def track_maneuvers(vp: VehicleParams, types=("all",), radius=None,
                    throttle_pct=None, entry_frac=None, direction=1,
                    split_mu=True):
    """The test matrix. types: corner slugs, 'split_mu', or 'all'.
    radius: override every corner type's radius list with this one value."""
    types = list(types or ["all"])
    want_all = "all" in types
    mans = []
    for slug, (_, radii) in CORNER_TYPES.items():
        if not (want_all or slug in types):
            continue
        for R in ((radius,) if radius else radii):
            mans.append(track_maneuver(vp, slug, R, throttle_pct, entry_frac,
                                       direction))
    if split_mu and (want_all or "split_mu" in types):
        cs, R, scale = SPLIT_MU_CASE
        mans.append(track_maneuver(vp, cs, radius or R, throttle_pct,
                                   entry_frac, direction, mu_inner_scale=scale))
    unknown = [t for t in types if t not in CORNER_TYPES
               and t not in ("all", "split_mu")]
    if unknown:
        raise KeyError(f"unknown track selection {unknown}; known: "
                       f"{', '.join(list(CORNER_TYPES) + ['split_mu', 'all'])}")
    return mans


# ── split-µ plant ────────────────────────────────────────────────────────
def model_for(maneuver, base_model: VehicleModel) -> VehicleModel:
    """The plant to run this maneuver on. Plain maneuvers get the base
    model back unchanged; a split-µ TrackManeuver gets a copy whose patched
    wheel(s) carry a scaled tire µ (lateral AND longitudinal). The
    CONTROLLER is never told — the patch is a surprise, as on the road."""
    patch = getattr(maneuver, "mu_patch", None)
    if not patch:
        return base_model
    tires = list(base_model.tires)
    for idx, scale in patch.items():
        tp = tires[idx].p
        tires[idx] = MagicFormulaTire(replace(tp, mu0=tp.mu0 * scale,
                                              mu0x=tp.mu0x * scale))
    return VehicleModel(base_model.p, tires[0], tires[2], tires=tuple(tires))
