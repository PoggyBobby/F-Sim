"""TTC → Magic Formula fitter: turns Calspan raw tire data into the nine
tire coefficients car_data.py needs.

    .venv/bin/python tire_fit.py --cornering ttc/…run31.mat ttc/…run32.mat \
        --drivebrake ttc/…run72.mat --pressure 12 --out ttc/fit_hoosier_r20

Reads the standard TTC raw-data channels (SA slip angle, SL slip ratio,
FX/FY/FZ forces, IA camber, P pressure), filters to the requested pressure
and near-zero camber, and fits THIS SIM'S tire model (tire.py):

    F(s)  = D·sin(C·atan(B·s − E·(B·s − atan(B·s))))          + shifts
    D     = µ(Fz)·Fz          µ(Fz) = µ₀·(1 − s_µ·(Fz/Fz_nom − 1))
    B     = c/(C·µ(Fz))       (so small-slip stiffness is exactly c·Fz)

Lateral fit → µ₀, s_µ, c_alpha, C_y, E_y  (global over all loads at once —
load sensitivity comes from the fit, not from per-load hand comparison).
Longitudinal fit → c_kappa, C_x, E_x with µ(Fz) FIXED from the lateral fit
(the sim's model shares one µ; the report states how far the true
longitudinal peak deviates — if it's large we extend the model).

Small horizontal/vertical shifts (plysteer/conicity/rolling offsets) are
fitted as nuisance parameters and then DROPPED — the sim's symmetric model
has none, and they belong to the specific test tire, not the model.

⚠️ BELT vs ASPHALT: Calspan's sandpaper belt grips harder than track
asphalt. Common practice scales the fitted µ₀ by ~0.65–0.70 for real-road
predictions. The fitter reports RAW BELT values; apply (and document) any
road scaling when entering car_data.py — never silently.

Output: fitted values + quality stats printed, comparison plots saved to
--out, and a ready-to-paste car_data.py block.
"""

import argparse
import glob
import math
import os
import sys

import numpy as np
from scipy.io import loadmat
from scipy.optimize import least_squares

LB = 4.4482216
DEG = math.pi / 180.0


# ── data loading ─────────────────────────────────────────────────────────
def load_run(path, units):
    m = loadmat(path)
    conv_f = LB if units == "uscs" else 1.0
    d = {
        "SA": m["SA"].ravel() * DEG,          # slip angle [rad]
        "IA": m["IA"].ravel(),                # camber [deg] (filter only)
        # pressure filter works in psi; SI files carry P in kPa
        "P": m["P"].ravel() / (6.89476 if units == "si" else 1.0),
        "FY": m["FY"].ravel() * conv_f,       # [N]
        "FX": m["FX"].ravel() * conv_f,       # [N]
        "FZ": -m["FZ"].ravel() * conv_f,      # [N], compression → positive
        "SL": m["SL"].ravel(),                # slip ratio [-]
        "ET": m["ET"].ravel(),
    }
    d["tireid"] = str(m.get("tireid", ["?"])[0])
    d["testid"] = str(m.get("testid", ["?"])[0])
    return d


def stack(paths, units):
    runs = [load_run(p, units) for p in paths]
    out = {k: np.concatenate([r[k] for r in runs]) for k in
           ("SA", "IA", "P", "FY", "FX", "FZ", "SL", "ET")}
    out["ids"] = [(r["tireid"], r["testid"]) for r in runs]
    return out


# ── the sim's tire model (must match tire.py exactly) ────────────────────
def mf(s, Fz, mu0, s_mu, c, C, E, Fz_nom):
    mu = mu0 * (1.0 - s_mu * (Fz / Fz_nom - 1.0))
    mu = np.maximum(mu, 0.5 * mu0)
    D = mu * Fz
    B = c / (C * mu)
    Bs = B * s
    return D * np.sin(C * np.arctan(Bs - E * (Bs - np.arctan(Bs))))


# ── fitting ──────────────────────────────────────────────────────────────
def fit_lateral(d, p_target, cam_max, Fz_nom, fz_min):
    m = ((np.abs(d["P"] - p_target) < 0.75) & (np.abs(d["IA"]) < cam_max)
         & (d["FZ"] > fz_min) & (np.abs(d["SA"]) < 15 * DEG))
    s, F, Fz = d["SA"][m], d["FY"][m], d["FZ"][m]
    if s.size < 500:
        sys.exit(f"lateral: only {s.size} samples survive the filters — "
                 "check --pressure / --camber-max against the run")
    # sign: the sim wants positive slope at the origin (ISO); TTC/SAE data
    # usually slopes the other way. Detect and flip the FORCE, not the fit.
    core = np.abs(s) < 2 * DEG
    sign = -1.0 if np.polyfit(s[core], F[core], 1)[0] < 0 else 1.0
    F = sign * F

    def resid(th):
        mu0, s_mu, c, C, E, Sh, Sv = th
        return mf(s + Sh, Fz, mu0, s_mu, c, C, E, Fz_nom) + Sv - F

    th0 = (2.0, 0.10, 18.0, 1.5, -0.5, 0.0, 0.0)
    lo = (0.5, 0.0, 5.0, 1.1, -2.0, -2 * DEG, -300.0)
    hi = (4.0, 0.6, 45.0, 1.99, 0.9, 2 * DEG, 300.0)
    r = least_squares(resid, th0, bounds=(lo, hi), loss="soft_l1",
                      f_scale=100.0)
    rms = float(np.sqrt(np.mean(resid(r.x) ** 2)))
    return r.x, {"n": int(s.size), "rms_N": rms, "sign": sign,
                 "loads": np.percentile(Fz, [5, 50, 95]).round(0)}


def fit_longitudinal(d, p_target, cam_max, Fz_nom, fz_min, mu0, s_mu,
                     free_mux=True):
    m = ((np.abs(d["P"] - p_target) < 0.75) & (np.abs(d["IA"]) < cam_max)
         & (d["FZ"] > fz_min) & (np.abs(d["SA"]) < 0.6 * DEG)
         & (np.abs(d["SL"]) < 0.35) & (np.abs(d["SL"]) > 1e-4))
    s, F, Fz = d["SL"][m], d["FX"][m], d["FZ"][m]
    if s.size < 300:
        sys.exit(f"longitudinal: only {s.size} samples survive — the run "
                 "may hold SA≠0 plateaus; loosen filters or check the run")
    core = np.abs(s) < 0.05
    sign = -1.0 if np.polyfit(s[core], F[core], 1)[0] < 0 else 1.0
    F = sign * F

    def resid(th):
        mux0, c, C, E, Sh, Sv = th
        return mf(s + Sh, Fz, mux0, s_mu, c, C, E, Fz_nom) + Sv - F

    th0 = (mu0 * 1.15, 25.0, 1.6, -0.5, 0.0, 0.0)
    lo = (mu0 if not free_mux else 0.5, 5.0, 1.1, -2.0, -0.02, -300.0)
    hi = (mu0 * (1 + 1e-9) if not free_mux else 4.5, 60.0, 1.99, 0.9,
          0.02, 300.0)
    r = least_squares(resid, th0, bounds=(lo, hi), loss="soft_l1",
                      f_scale=100.0)
    rms = float(np.sqrt(np.mean(resid(r.x) ** 2)))
    return r.x, {"n": int(s.size), "rms_N": rms, "sign": sign,
                 "mux_over_muy": r.x[0] / mu0}


# ── plots ────────────────────────────────────────────────────────────────
def plot_fit(outdir, d, kind, th, Fz_nom, p_target, cam_max, stats,
             mu_fixed=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    key_s, key_f = ("SA", "FY") if kind == "lateral" else ("SL", "FX")
    m = ((np.abs(d["P"] - p_target) < 0.75) & (np.abs(d["IA"]) < cam_max)
         & (d["FZ"] > 150))
    if kind == "longitudinal":
        m &= np.abs(d["SA"]) < 0.6 * DEG
    s, F, Fz = d[key_s][m], stats["sign"] * d[key_f][m], d["FZ"][m]
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.plot((s / DEG if kind == "lateral" else s), F, ".", ms=1.5,
            alpha=0.18, color="#2a78d6", label="TTC samples")
    bins = np.percentile(Fz, [15, 50, 85])
    ss = np.linspace(s.min(), s.max(), 300)
    for Fzb, colr in zip(bins, ("#1baf7a", "#eda100", "#eb6834")):
        if kind == "lateral":
            mu0, s_mu, c, C, E, Sh, Sv = th
            yy = mf(ss + Sh, Fzb, mu0, s_mu, c, C, E, Fz_nom) + Sv
        else:
            mux0, c, C, E, Sh, Sv = th
            _, s_mu = mu_fixed
            yy = mf(ss + Sh, Fzb, mux0, s_mu, c, C, E, Fz_nom) + Sv
        ax.plot(ss / DEG if kind == "lateral" else ss, yy, lw=2.4,
                color=colr, label=f"fit @ Fz = {Fzb:.0f} N")
    ax.set_xlabel("slip angle [deg]" if kind == "lateral" else "slip ratio [-]")
    ax.set_ylabel(("Fy" if kind == "lateral" else "Fx") + " [N]")
    ax.set_title(f"{kind} fit — RMS {stats['rms_N']:.0f} N over "
                 f"{stats['n']} samples (P≈{p_target} psi, |IA|<{cam_max}°)")
    ax.grid(True, alpha=0.4)
    ax.legend()
    path = os.path.join(outdir, f"fit_{kind}.png")
    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)
    return path


# ── main ─────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="TTC → Magic Formula fitter")
    ap.add_argument("--cornering", nargs="+", required=True,
                    help="cornering-run .mat files (slip-angle sweeps)")
    ap.add_argument("--drivebrake", nargs="*", default=[],
                    help="drive/brake-run .mat files (slip-ratio sweeps)")
    ap.add_argument("--units", choices=["uscs", "si"], default="uscs")
    ap.add_argument("--pressure", type=float, default=12.0, help="psi")
    ap.add_argument("--pressure-db", type=float, default=None,
                    help="psi for the drive/brake runs when they were "
                         "tested at a different pressure (default: same)")
    ap.add_argument("--camber-max", type=float, default=1.0, help="deg")
    ap.add_argument("--fz-min", type=float, default=150.0, help="N")
    ap.add_argument("--fznom", type=float, default=None,
                    help="reference load; default = the car's TIRE_FZ_NOM")
    ap.add_argument("--out", default="ttc/fit_report")
    a = ap.parse_args()

    if a.fznom is None:
        import car_data as cd
        a.fznom = cd.TIRE_FZ_NOM
    os.makedirs(a.out, exist_ok=True)

    lat = stack(sum([glob.glob(p) for p in a.cornering], []), a.units)
    print("── cornering data ──")
    for tid, test in lat["ids"]:
        print(f"   {tid}  |  {test}")
    th_l, st_l = fit_lateral(lat, a.pressure, a.camber_max, a.fznom, a.fz_min)
    mu0, s_mu, c_a, C_y, E_y, Sh, Sv = th_l
    print(f"   n={st_l['n']}  RMS {st_l['rms_N']:.0f} N   "
          f"loads 5/50/95%: {st_l['loads']} N   (shifts dropped: "
          f"Sh {Sh/DEG:+.2f}°, Sv {Sv:+.0f} N)")
    plot_fit(a.out, lat, "lateral", th_l, a.fznom, a.pressure, a.camber_max,
             st_l)

    th_x = st_x = None
    if a.drivebrake:
        lon = stack(sum([glob.glob(p) for p in a.drivebrake], []), a.units)
        print("── drive/brake data ──")
        for tid, test in lon["ids"]:
            print(f"   {tid}  |  {test}")
        p_db = a.pressure_db if a.pressure_db is not None else a.pressure
        th_x, st_x = fit_longitudinal(lon, p_db, a.camber_max, a.fznom,
                                      a.fz_min, mu0, s_mu)
        print(f"   n={st_x['n']}  RMS {st_x['rms_N']:.0f} N   "
              f"µx/µy = {st_x['mux_over_muy']:.2f}")
        plot_fit(a.out, lon, "longitudinal", th_x, a.fznom, p_db,
                 a.camber_max, st_x, mu_fixed=(mu0, s_mu))

    print("\n──── fitted values (RAW BELT — see road-scaling note) ────")
    print(f"TIRE_MU0            = {mu0:.3f}")
    print(f"TIRE_S_MU           = {s_mu:.3f}")
    print(f"TIRE_FZ_NOM         = {a.fznom:.1f}   (reference, kept from car)")
    print(f"TIRE_C_ALPHA (f=r)  = {c_a:.2f}    per rad per N")
    print(f"TIRE_SHAPE_C_LAT    = {C_y:.3f}")
    print(f"TIRE_CURV_E_LAT     = {E_y:.3f}")
    if th_x is not None:
        mux0, c_k, C_x, E_x, _, _ = th_x
        print(f"TIRE_MU0_LONG       = {mux0:.3f}")
        print(f"TIRE_C_KAPPA        = {c_k:.2f}")
        print(f"TIRE_SHAPE_C_LONG   = {C_x:.3f}")
        print(f"TIRE_CURV_E_LONG    = {E_x:.3f}")
    print(f"\nplots: {a.out}/fit_lateral.png"
          + (f", {a.out}/fit_longitudinal.png" if th_x is not None else ""))
    print("⚠️  belt→asphalt: common practice scales µ₀ by ~0.65–0.70 when "
          "predicting on-track absolutes; document the choice in car_data.")


if __name__ == "__main__":
    main()
