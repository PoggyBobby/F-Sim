"""Shared plot styling — one visual system for every figure and animation.

Colors are a validated colorblind-safe categorical palette. The slot order is
the safety mechanism, not decoration: it was checked with the palette
validator (worst adjacent CVD ΔE 9.1, normal-vision ΔE 22.9, both above the
floors). Two rules that keep it honest:

  * one color per CONTROLLER CONFIG, identical in every figure — color always
    follows the entity, never its rank in the current chart;
  * config colors carry IDENTITY only. Magnitudes (slip, load) never get a
    config color, and the reserved status red is only ever used for the one
    "this wheel is spinning" state, always with a label beside it.

Three of these hues sit below 3:1 contrast on the light surface, so every
figure carries a legend and the animation direct-labels each car.
"""

# ── categorical: one slot per controller config, fixed order ─────────────
CONFIG_COLORS = {
    "open (50/50)": "#2a78d6",   # blue
    "s-diff":       "#eb6834",   # orange
    "TV":           "#1baf7a",   # aqua
    "s-diff + TV":  "#eda100",   # yellow (drawn last + slightly thicker)
    "VCU (SIL)":    "#8a63d2",   # violet — the real firmware in the loop
}

# ── status: reserved, never used as a series color ───────────────────────
STATUS_CRITICAL = "#d03b3b"      # rear wheel spinning (|kappa| over threshold)
SPIN_KAPPA_FALLBACK = 0.12       # only if the tire model can't be asked; the
                                 # real threshold is MagicFormulaTire.
                                 # kappa_at_peak(), i.e. the slip where the
                                 # tire stops making more force

# ── chrome & ink ─────────────────────────────────────────────────────────
REF_COLOR = "#898781"            # muted ink for reference / target lines
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
ASPHALT = "#f0efec"              # neutral ground plane in the replay view

RC = {
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "text.color": INK, "axes.labelcolor": INK_2,
    "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.edgecolor": AXIS, "axes.linewidth": 1.0,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.axisbelow": True,
    "axes.spines.top": False, "axes.spines.right": False,
    "font.family": "sans-serif", "font.size": 10,
    "axes.titlesize": 11, "axes.titleweight": "bold",
    "legend.frameon": False,
    "lines.linewidth": 2.0,
}


def config_lw(name):
    """The combined config is drawn slightly thicker and on top."""
    return 2.2 if name == "s-diff + TV" else 2.0


def config_z(name):
    return 3 if name == "s-diff + TV" else 2
