"""Animated replay — watch the four controller configurations drive the same
maneuver side by side, in the ground frame, at true scale.

One video per maneuver. All four cars start at the same point with the same
scripted driver inputs (steer + total torque), so every difference you see on
screen is the torque split and nothing else.

    ┌───────────────────────────┬──────────────────┐
    │                           │ whole trajectory │
    │   replay (follow camera)  ├──────────────────┤
    │   true scale, 4 cars      │ yaw rate vs ref  │
    │   red wheel = spinning    ├──────────────────┤
    │                           │ inner-rear slip  │
    └───────────────────────────┴──────────────────┘

Reading the picture:
  * body color = controller config (the same colors as every other figure);
  * front wheels turn with the actual steer input;
  * a rear wheel turns RED once its slip ratio passes the peak-force slip of
    the tire model — past that point more wheel speed makes LESS force, which
    is the inner-wheel spin-up the software diff exists to prevent. The
    threshold is asked of the tire, not hardcoded, so it tracks the tire data;
  * the trail is where that car has been, so the configs diverging is visible
    directly as the paths separating.

Nothing here is drawn to scale except the car: body, track, wheelbase and
wheel size are the real numbers out of the params.yaml files.
"""

import os

import numpy as np

from model.params import G
from model.physical.vehicle import front_steer_angles
from style import (CONFIG_COLORS, STATUS_CRITICAL, SPIN_KAPPA_FALLBACK,
                   REF_COLOR, INK, INK_2, MUTED, ASPHALT, SURFACE,
                   config_lw, config_z)

# body-frame channels resampled onto the animation's time grid
CHANNELS = ("X", "Y", "psi", "delta", "kRL", "kRR", "r", "r_ref", "vx",
            "T_RL", "T_RR")

# direct-label placement per config, in units of the camera half-window, so
# the four labels stay readable (and apart) at any zoom level
LABEL_OFFSETS = {
    "open (50/50)": (0.0, 0.20),
    "s-diff": (0.26, -0.16),
    "TV": (-0.26, -0.16),
    "s-diff + TV": (0.0, -0.30),
    "VCU (SIL)": (0.0, 0.34),
}
SHORT_NAMES = {"open (50/50)": "open", "s-diff": "s-diff", "TV": "TV",
               "s-diff + TV": "s-diff + TV", "VCU (SIL)": "VCU"}


def _rot(pts, ang):
    c, s = np.cos(ang), np.sin(ang)
    return pts @ np.array([[c, -s], [s, c]]).T


def _body_polygon(vp):
    """FSAE-ish plan-view silhouette in the body frame (x fwd, y left)."""
    a, b = vp.a, vp.b
    nose, tail = a + 0.42, -(b + 0.38)
    return np.array([
        (nose, 0.00), (nose - 0.22, 0.11), (a, 0.16), (0.15, 0.20),
        (-b + 0.05, 0.22), (tail + 0.10, 0.20), (tail, 0.12),
        (tail, -0.12), (tail + 0.10, -0.20), (-b + 0.05, -0.22),
        (0.15, -0.20), (a, -0.16), (nose - 0.22, -0.11),
    ])


def _wheel_polygon(r_wheel):
    return np.array([(-r_wheel, -0.11), (r_wheel, -0.11),
                     (r_wheel, 0.11), (-r_wheel, 0.11)])


def spin_threshold(vp, tire_rear):
    """Slip ratio at which a rear wheel is called 'spinning' — the tire
    model's own peak-force slip at the static rear wheel load."""
    try:
        Fz = vp.m_total * G * (1.0 - vp.weight_frac_front) / 2.0
        k = float(tire_rear.kappa_at_peak(Fz))
        return k if k > 1e-3 else SPIN_KAPPA_FALLBACK
    except Exception:
        return SPIN_KAPPA_FALLBACK


def _resample(results, tgrid):
    """Every config's log on one time grid. A config that spun and stopped
    early holds its last state (and is flagged) rather than vanishing."""
    out = {}
    for name, res in results.items():
        log = res["log"]
        t = log["t"]
        cfg = {k: np.interp(tgrid, t, log[k]) for k in CHANNELS if k in log}
        cfg["alive"] = tgrid <= t[-1] + 1e-9
        cfg["spun"] = not res["metrics"]["finished"]
        out[name] = cfg
    return out


def _smooth(v, k):
    """Moving average that keeps the ends put — the camera should glide, not
    jitter, and must not drift off the cars at the start or end."""
    if k < 2:
        return v
    pad = k // 2 + 1
    kern = np.ones(k) / k
    return np.convolve(np.pad(v, pad, mode="edge"), kern, mode="same")[pad:len(v) + pad]


def animate_maneuver(plt, maneuver, results, vp, tire_rear, outpath, fps=30,
                     dpi=110, show=False):
    """Render one maneuver's replay. Returns the written path (or None)."""
    from matplotlib import animation
    from matplotlib.lines import Line2D
    from matplotlib.patches import Polygon

    tgrid = np.arange(0.0, maneuver.duration + 1e-9, 1.0 / fps)
    data = _resample(results, tgrid)
    names = list(results.keys())
    k_spin = spin_threshold(vp, tire_rear)

    fig = plt.figure(figsize=(13.5, 7.6))
    gs = fig.add_gridspec(3, 3, width_ratios=[2.05, 0.02, 1.0],
                          height_ratios=[1, 1, 1], hspace=0.55, wspace=0.02)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_over = fig.add_subplot(gs[0, 2])
    ax_yaw = fig.add_subplot(gs[1, 2])
    ax_slip = fig.add_subplot(gs[2, 2])

    fig.suptitle(f"Replay — {maneuver.name}   ({maneuver.description})",
                 fontweight="bold", y=0.975)

    # ── replay panel ────────────────────────────────────────────────────
    ax_map.set_facecolor(ASPHALT)
    ax_map.grid(True, color="#e4e3de", lw=0.8)
    ax_map.set_xlabel("X [m]")
    ax_map.set_ylabel("Y [m]")
    ax_map.set_title("Ground view — true scale, camera follows the pack")

    body_tmpl = _body_polygon(vp)
    wheel_tmpl = _wheel_polygon(vp.r_wheel)
    wheel_xy = np.array([(vp.a, vp.track_f / 2), (vp.a, -vp.track_f / 2),
                         (-vp.b, vp.track_r / 2), (-vp.b, -vp.track_r / 2)])
    # where each suspension arm meets the body (keeps the wheels visually
    # attached — at FSAE track widths they sit well outside the bodywork)
    arm_root = np.array([(vp.a, 0.14), (vp.a, -0.14),
                         (-vp.b, 0.16), (-vp.b, -0.16)])

    art = {}
    for name in names:
        color = CONFIG_COLORS[name]
        z = config_z(name)
        trail, = ax_map.plot([], [], color=color, lw=1.6, alpha=0.45, zorder=z)
        arms = [ax_map.plot([], [], color=INK_2, lw=1.4, alpha=0.75,
                            solid_capstyle="round", zorder=z + 9)[0]
                for _ in range(4)]
        body = Polygon(body_tmpl, closed=True, facecolor=color,
                       edgecolor=SURFACE, lw=1.2, zorder=z + 10, alpha=0.95)
        ax_map.add_patch(body)
        wheels = []
        for _ in range(4):
            w = Polygon(wheel_tmpl, closed=True, facecolor=INK_2,
                        edgecolor=SURFACE, lw=0.8, zorder=z + 11)
            ax_map.add_patch(w)
            wheels.append(w)
        label = ax_map.text(0, 0, SHORT_NAMES.get(name, name), color=INK,
                            fontsize=8.5, ha="center", va="center",
                            zorder=z + 20,
                            bbox=dict(boxstyle="round,pad=0.22", fc=SURFACE,
                                      ec=color, lw=1.1, alpha=0.92))
        art[name] = {"trail": trail, "body": body, "wheels": wheels,
                     "arms": arms, "label": label,
                     "off": LABEL_OFFSETS.get(name, (0.0, 0.2))}

    # ── overview: the whole path, fixed limits ──────────────────────────
    ax_over.set_title("Whole trajectory", fontsize=9.5)
    ax_over.set_xlabel("X [m]", fontsize=8.5)
    ax_over.set_ylabel("Y [m]", fontsize=8.5)
    ax_over.tick_params(labelsize=8)
    for name in names:
        log = results[name]["log"]
        ax_over.plot(log["X"], log["Y"], color=CONFIG_COLORS[name],
                     lw=config_lw(name) - 0.7, zorder=config_z(name))
    dots = {n: ax_over.plot([], [], "o", color=CONFIG_COLORS[n], ms=5.5,
                            mec=SURFACE, mew=1.0, zorder=6)[0] for n in names}
    ax_over.set_aspect("equal", adjustable="datalim")

    # ── traces with a moving time cursor ────────────────────────────────
    for ax, title, ylab in ((ax_yaw, "Yaw rate vs reference", "r [rad/s]"),
                            (ax_slip, "Inner-rear slip ratio", "κ_RL [-]")):
        ax.set_title(title, fontsize=9.5)
        ax.set_ylabel(ylab, fontsize=8.5)
        ax.set_xlabel("time [s]", fontsize=8.5)
        ax.tick_params(labelsize=8)
        ax.set_xlim(0, maneuver.duration)

    k_peak_seen = 0.0
    for name in names:
        log = results[name]["log"]
        ax_yaw.plot(log["t"], log["r"], color=CONFIG_COLORS[name],
                    lw=config_lw(name) - 0.4, zorder=config_z(name))
        ax_slip.plot(log["t"], log["kRL"], color=CONFIG_COLORS[name],
                     lw=config_lw(name) - 0.4, zorder=config_z(name))
        k_peak_seen = max(k_peak_seen, float(np.max(np.abs(log["kRL"]))))
    ref = results[names[-1]]["log"]
    ax_yaw.plot(ref["t"], ref["r_ref"], "--", color=REF_COLOR, lw=1.5, zorder=1)

    # only stretch the slip axis up to the spin threshold when a wheel got
    # anywhere near it — otherwise the real trace would be squashed flat
    if k_peak_seen > 0.55 * k_spin:
        ax_slip.axhline(k_spin, ls=":", color=STATUS_CRITICAL, lw=1.5, zorder=1)
        ax_slip.text(maneuver.duration * 0.99, k_spin, " spinning ",
                     color=STATUS_CRITICAL, fontsize=7.5, ha="right",
                     va="bottom")
    else:
        ax_slip.set_ylim(min(0.0, -0.1 * k_peak_seen), k_peak_seen * 1.35)
        ax_slip.text(0.5, 0.93,
                     f"peak |κ| {k_peak_seen:.3f} — spin threshold "
                     f"{k_spin:.2f} not reached",
                     transform=ax_slip.transAxes, ha="center", va="top",
                     fontsize=7.5, color=MUTED)
    cursors = [ax.axvline(0.0, color=MUTED, lw=1.2, zorder=8)
               for ax in (ax_yaw, ax_slip)]

    # ── legend: identity by color, plus the one reserved status color ───
    handles = [Line2D([], [], color=CONFIG_COLORS[n], lw=3, label=n)
               for n in names]
    handles.append(Line2D([], [], color=STATUS_CRITICAL, lw=3,
                          label=f"rear wheel spinning (|κ| > {k_spin:.2f})"))
    handles.append(Line2D([], [], color=REF_COLOR, lw=1.6, ls="--",
                          label="yaw-rate reference"))
    fig.legend(handles=handles, loc="lower center", ncol=6, fontsize=9,
               bbox_to_anchor=(0.5, 0.0))

    clock = ax_map.text(0.015, 0.975, "", transform=ax_map.transAxes,
                        ha="left", va="top", fontsize=11, color=INK,
                        fontweight="bold")

    # camera: centered on the pack, wide enough to hold every car, smoothed
    cx = np.mean([data[n]["X"] for n in names], axis=0)
    cy = np.mean([data[n]["Y"] for n in names], axis=0)
    spread = np.max([np.hypot(data[n]["X"] - cx, data[n]["Y"] - cy)
                     for n in names], axis=0)
    k = max(1, int(0.4 * fps))
    cx, cy = _smooth(cx, k), _smooth(cy, k)
    win = _smooth(np.maximum(5.5, spread + 2.5), k)
    fig.canvas.draw()                      # positions are needed for the ratio
    bb = ax_map.get_position()
    ar = (bb.width * fig.get_figwidth()) / (bb.height * fig.get_figheight())

    def update(i):
        t = tgrid[i]
        hw = win[i]
        for name in names:
            d, A = data[name], art[name]
            X, Y, psi = d["X"][i], d["Y"][i], d["psi"][i]
            origin = np.array([X, Y])
            A["body"].set_xy(_rot(body_tmpl, psi) + origin)
            dFL, dFR = front_steer_angles(vp, d["delta"][i])
            steers = (dFL, dFR, 0.0, 0.0)
            kap = (0.0, 0.0, d["kRL"][i], d["kRR"][i])
            for j, w in enumerate(A["wheels"]):
                pts = _rot(wheel_tmpl, steers[j]) + wheel_xy[j]
                w.set_xy(_rot(pts, psi) + origin)
                spinning = abs(kap[j]) > k_spin
                w.set_facecolor(STATUS_CRITICAL if spinning else INK_2)
                w.set_linewidth(1.4 if spinning else 0.8)
                seg = _rot(np.array([arm_root[j], wheel_xy[j]]), psi) + origin
                A["arms"][j].set_data(seg[:, 0], seg[:, 1])
            A["trail"].set_data(d["X"][:i + 1], d["Y"][:i + 1])
            dx, dy = A["off"]
            A["label"].set_position((X + dx * hw, Y + dy * hw))
            A["label"].set_text(SHORT_NAMES.get(name, name) +
                                ("  ✕ spun" if d["spun"] and not d["alive"][i]
                                 else ""))
            dots[name].set_data([X], [Y])
        ax_map.set_xlim(cx[i] - hw * ar, cx[i] + hw * ar)
        ax_map.set_ylim(cy[i] - hw, cy[i] + hw)
        for c in cursors:
            c.set_xdata([t, t])
        clock.set_text(f"t = {t:4.2f} s")
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(tgrid),
                                   interval=1000 / fps, blit=False)

    written = None
    try:
        if animation.writers.is_available("ffmpeg"):
            writer = animation.FFMpegWriter(fps=fps, bitrate=2600,
                                            metadata={"title": maneuver.name})
            anim.save(outpath, writer=writer, dpi=dpi,
                      savefig_kwargs={"facecolor": SURFACE})
            written = outpath
        else:
            gif = os.path.splitext(outpath)[0] + ".gif"
            anim.save(gif, writer=animation.PillowWriter(fps=fps), dpi=90,
                      savefig_kwargs={"facecolor": SURFACE})
            written = gif
    finally:
        if show:
            plt.show()
        else:
            plt.close(fig)
    return written
