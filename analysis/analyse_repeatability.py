#!/usr/bin/env python3
"""
analyse_repeatability.py — return-to-origin pose repeatability analysis

Computes pose repeatability in the sense of ISO 18646-2 (Robotics — Performance
criteria and related test methods for service robots — Part 2: Navigation), which
is the applicable standard for mobile platforms. ISO 9283 covers manipulators.

WHAT THIS SCRIPT DOES AND DOES NOT REPORT
-----------------------------------------
REPORTS   pose repeatability: dispersion of attained poses about their own
          barycentre. Independent of where the commanded pose was, so the
          deliberate +0.10 m goal offset does not enter the computation at all.

DOES NOT  pose accuracy: deviation of the barycentre from the commanded pose.
          The commanded pose was offset to keep the fiducial array inside the
          camera's ~98 mm measurement window, so the barycentre is not an
          independent estimate of absolute capability. The offset is printed
          for completeness and explicitly flagged as not an accuracy figure.

VERIFY BEFORE PUBLICATION
-------------------------
The RP scalar below uses RP = mean(l) + 3*SD(l), where l is each trial's radial
distance from the barycentre. This is the conventional form, but the exact
expression MUST be checked against the standard text before it appears in the
manuscript. Do not take a formula for a cited standard on trust.

Usage:
    python3 analyse_repeatability.py results.csv --run-tag n30 --outdir ./out

Expected columns: trial, ex_m, ey_m, yaw_error_deg, all_ids_detected,
                  tag_id_used  (last two optional but used for validation)
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import stats


# ── statistics ───────────────────────────────────────────────────────────────

def circular_mean_deg(a_deg: np.ndarray) -> float:
    a = np.radians(a_deg)
    return float(np.degrees(np.arctan2(np.mean(np.sin(a)), np.mean(np.cos(a)))))


def circular_sd_deg(a_deg: np.ndarray) -> float:
    """Circular SD, degrees. Valid across the +/-180 branch cut."""
    a = np.radians(a_deg)
    R = np.hypot(np.mean(np.sin(a)), np.mean(np.cos(a)))
    R = min(max(R, 1e-15), 1.0)
    return float(np.degrees(np.sqrt(-2.0 * np.log(R))))


def circular_residuals_deg(a_deg: np.ndarray) -> np.ndarray:
    r = np.radians(a_deg - circular_mean_deg(a_deg))
    return np.degrees(np.angle(np.exp(1j * r)))


def sigma_ci(sigma: float, n: int, conf: float = 0.95) -> tuple[float, float]:
    """Confidence interval on a standard deviation (chi-square, normal data)."""
    if n < 3:
        return (float("nan"), float("nan"))
    df = n - 1
    lo = sigma * np.sqrt(df / stats.chi2.ppf(1 - (1 - conf) / 2, df))
    hi = sigma * np.sqrt(df / stats.chi2.ppf((1 - conf) / 2, df))
    return float(lo), float(hi)


# ── validation ───────────────────────────────────────────────────────────────

def validate(df: pd.DataFrame) -> tuple[pd.DataFrame, list[dict]]:
    """Flag trials that cannot be trusted. Exclusions are reported, never silent."""
    excl = []

    # (1) The reported tag must actually have been detected in that trial.
    if "tag_id_used" in df and "all_ids_detected" in df:
        for i, row in df.iterrows():
            ids = {int(s) for s in str(row["all_ids_detected"]).split(";") if s.strip()}
            if int(row["tag_id_used"]) not in ids:
                excl.append({
                    "trial": int(row["trial"]),
                    "reason": (f"tag_id_used={int(row['tag_id_used'])} not present in "
                               f"all_ids_detected={{{row['all_ids_detected']}}} — the "
                               f"reported pose cannot have come from this frame"),
                })

    # (2) Byte-identical position to another trial: a carried-forward stale value,
    #     not an independent measurement. Real measurements do not repeat exactly.
    dup = df.duplicated(subset=["ex_m", "ey_m"], keep="first")
    for i in df.index[dup]:
        first = df[(df.ex_m == df.loc[i, "ex_m"]) &
                   (df.ey_m == df.loc[i, "ey_m"])].trial.iloc[0]
        excl.append({
            "trial": int(df.loc[i, "trial"]),
            "reason": (f"position byte-identical to trial {int(first)} — stale value "
                       f"carried forward, not an independent measurement"),
        })

    bad = {e["trial"] for e in excl}
    return df[~df.trial.isin(bad)].copy(), excl


# ── analysis ─────────────────────────────────────────────────────────────────

def analyse(df: pd.DataFrame, goal_offset_x_m: float) -> dict:
    x = df.ex_m.values * 1000.0          # mm
    y = df.ey_m.values * 1000.0
    yaw = df.yaw_error_deg.values
    n = len(df)

    # Barycentre of attained poses. This, not the commanded pose, is the datum
    # for repeatability.
    xb, yb = float(np.mean(x)), float(np.mean(y))
    dx, dy = x - xb, y - yb
    l = np.hypot(dx, dy)                  # radial distance from barycentre

    l_mean, l_sd = float(np.mean(l)), float(np.std(l, ddof=1))
    rp = l_mean + 3.0 * l_sd             # VERIFY against standard text

    sx, sy = float(np.std(dx, ddof=1)), float(np.std(dy, ddof=1))

    yaw_mean = circular_mean_deg(yaw)
    yaw_sd = circular_sd_deg(yaw)
    yaw_res = circular_residuals_deg(yaw)

    # Session trend: is there a drift, and is it significant?
    t = df.trial.values.astype(float)
    tr = stats.linregress(t, l)
    tr_lo = tr.slope - 1.96 * tr.stderr
    tr_hi = tr.slope + 1.96 * tr.stderr

    return {
        "n": n,
        "barycentre_mm": {"x": xb, "y": yb},
        "repeatability": {
            "radial_mean_from_barycentre_mm": l_mean,
            "radial_sd_mm": l_sd,
            "radial_sd_95ci_mm": sigma_ci(l_sd, n),
            "radial_max_mm": float(np.max(l)),
            "RP_mean_plus_3sd_mm": rp,
            "sigma_x_mm": sx, "sigma_x_95ci_mm": sigma_ci(sx, n),
            "sigma_y_mm": sy, "sigma_y_95ci_mm": sigma_ci(sy, n),
            "xy_correlation": float(np.corrcoef(dx, dy)[0, 1]),
        },
        "heading": {
            "circular_mean_deg": yaw_mean,
            "circular_sd_deg": yaw_sd,
            "circular_sd_95ci_deg": sigma_ci(yaw_sd, n),
            "max_abs_residual_deg": float(np.max(np.abs(yaw_res))),
        },
        "session_trend": {
            "slope_mm_per_trial": float(tr.slope),
            "slope_95ci": [float(tr_lo), float(tr_hi)],
            "p_value": float(tr.pvalue),
            "significant_at_0.05": bool(tr.pvalue < 0.05),
        },
        "NOT_accuracy": {
            "note": ("The commanded pose was offset by the value below to keep the "
                     "fiducial array inside the camera measurement window. The "
                     "barycentre offset is therefore NOT a pose accuracy figure."),
            "commanded_goal_offset_x_mm": goal_offset_x_m * 1000.0,
            "barycentre_offset_from_measurement_datum_mm": float(np.hypot(xb, yb)),
        },
    }


# ── plotting ─────────────────────────────────────────────────────────────────

def plot(df: pd.DataFrame, res: dict, excluded: pd.DataFrame,
         run_tag: str, outdir: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse, Circle

    x = df.ex_m.values * 1000.0
    y = df.ey_m.values * 1000.0
    xb, yb = res["barycentre_mm"]["x"], res["barycentre_mm"]["y"]
    dx, dy = x - xb, y - yb
    l = np.hypot(dx, dy)
    r = res["repeatability"]
    n = res["n"]

    fig, ax = plt.subplots(2, 2, figsize=(12.5, 10.5))

    # (a) dispersion about the barycentre — NOT about the commanded pose
    a = ax[0][0]
    cov = np.cov(dx, dy)
    ev, evec = np.linalg.eigh(cov)
    order = ev.argsort()[::-1]
    ev, evec = ev[order], evec[:, order]
    angle = np.degrees(np.arctan2(evec[1, 0], evec[0, 0]))
    for k, ls, lab in ((1, "--", "1σ"), (2, ":", "2σ")):
        a.add_patch(Ellipse((0, 0), 2 * k * np.sqrt(ev[0]), 2 * k * np.sqrt(ev[1]),
                            angle=angle, fill=False, ls=ls, ec="#c0392b", lw=1.3,
                            label=f"{lab} ellipse"))
    a.add_patch(Circle((0, 0), r["RP_mean_plus_3sd_mm"], fill=False, ls="-",
                       ec="#7f8c8d", lw=1.2,
                       label=f"RP = {r['RP_mean_plus_3sd_mm']:.1f} mm"))
    a.scatter(dx, dy, s=46, c="#1a6faf", zorder=3, label=f"trials (n={n})")
    if len(excluded):
        ex = excluded.ex_m.values * 1000.0 - xb
        ey = excluded.ey_m.values * 1000.0 - yb
        a.scatter(ex, ey, s=80, facecolors="none", edgecolors="#e67e22", lw=1.8,
                  zorder=4, label="excluded (see caption)")
    a.plot(0, 0, "+", ms=14, mew=2.2, c="k", zorder=5, label="barycentre")
    lim = max(30.0, 1.25 * r["RP_mean_plus_3sd_mm"])
    a.set_xlim(-lim, lim); a.set_ylim(-lim, lim); a.set_aspect("equal")
    a.set_xlabel("x deviation from barycentre (mm)")
    a.set_ylabel("y deviation from barycentre (mm)")
    a.set_title("(a) Pose repeatability — dispersion about the barycentre")
    a.grid(lw=0.3, alpha=0.5); a.legend(fontsize=8, loc="lower left")

    # (b) radial deviation
    b = ax[0][1]
    b.hist(l, bins=max(6, int(np.sqrt(n) * 1.5)), color="#1a6faf", edgecolor="none",
           alpha=0.85)
    b.axvline(r["radial_mean_from_barycentre_mm"], color="#c0392b", ls="--", lw=1.4,
              label=f"mean {r['radial_mean_from_barycentre_mm']:.1f} mm")
    b.axvline(r["radial_max_mm"], color="k", ls=":", lw=1.4,
              label=f"max observed {r['radial_max_mm']:.1f} mm")
    b.set_xlabel("radial deviation from barycentre (mm)")
    b.set_ylabel("trials")
    b.set_title("(b) Radial deviation distribution")
    b.grid(axis="y", lw=0.3, alpha=0.5); b.legend(fontsize=8)

    # (c) heading — circular residuals
    c = ax[1][0]
    hres = circular_residuals_deg(df.yaw_error_deg.values)
    c.hist(hres, bins=max(6, int(np.sqrt(n) * 1.5)), color="#2c7a4b",
           edgecolor="none", alpha=0.85)
    c.axvline(0, color="#c0392b", ls="--", lw=1.4, label="circular mean")
    lo, hi = res["heading"]["circular_sd_95ci_deg"]
    c.set_xlabel("heading deviation from circular mean (deg)")
    c.set_ylabel("trials")
    c.set_title(f"(c) Heading deviation — circular SD "
                f"{res['heading']['circular_sd_deg']:.2f}° "
                f"[{lo:.2f}, {hi:.2f}]")
    c.grid(axis="y", lw=0.3, alpha=0.5); c.legend(fontsize=8)

    # (d) session trend
    d = ax[1][1]
    t = df.trial.values
    tr = res["session_trend"]
    d.plot(t, l, "o-", color="#1a6faf", label="radial deviation")
    fit = tr["slope_mm_per_trial"] * t + (np.mean(l) - tr["slope_mm_per_trial"] * np.mean(t))
    sig = "significant" if tr["significant_at_0.05"] else "not significant"
    d.plot(t, fit, ":", color="k", lw=1.5,
           label=(f"trend {tr['slope_mm_per_trial']:+.2f} mm/trial\n"
                  f"p = {tr['p_value']:.2f} ({sig})"))
    d.axhline(r["radial_mean_from_barycentre_mm"], color="#c0392b", ls="--", lw=1.2,
              label="mean")
    d.set_xlabel("trial number"); d.set_ylabel("radial deviation (mm)")
    d.set_title("(d) Deviation across the session — drift check")
    d.grid(lw=0.3, alpha=0.5); d.legend(fontsize=8)

    fig.suptitle(
        f"Return-to-origin pose repeatability — run={run_tag}, n={n}\n"
        f"Dispersion about the barycentre of attained poses (ISO 18646-2 sense). "
        f"Pose accuracy is not reported: see script header.",
        fontsize=11.5)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    p = outdir / f"fig_repeatability_{run_tag}.png"
    fig.savefig(p, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return p


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--run-tag", default="run")
    ap.add_argument("--outdir", default=".")
    ap.add_argument("--goal-offset-x", type=float, default=0.10,
                    help="commanded goal offset in x, metres (reporting only)")
    args = ap.parse_args()

    outdir = Path(args.outdir); outdir.mkdir(parents=True, exist_ok=True)
    raw = pd.read_csv(args.csv)

    kept, excl = validate(raw)
    excluded = raw[~raw.trial.isin(kept.trial)]

    res = analyse(kept, args.goal_offset_x)
    res["excluded_trials"] = excl
    res["trials_performed"] = int(len(raw))

    png = plot(kept, res, excluded, args.run_tag, outdir)
    js = outdir / f"repeatability_{args.run_tag}.json"
    js.write_text(json.dumps(res, indent=2))

    r, h, tr = res["repeatability"], res["heading"], res["session_trend"]
    print(f"\n{'='*74}\n  POSE REPEATABILITY — run={args.run_tag}\n{'='*74}")
    print(f"  trials performed / included : {res['trials_performed']} / {res['n']}")
    for e in excl:
        print(f"    EXCLUDED trial {e['trial']}: {e['reason']}")
    print(f"\n  barycentre of attained poses: "
          f"({res['barycentre_mm']['x']:+.2f}, {res['barycentre_mm']['y']:+.2f}) mm")
    print(f"\n  -- position, about the barycentre --")
    print(f"  mean radial deviation : {r['radial_mean_from_barycentre_mm']:7.2f} mm")
    print(f"  SD of radial deviation: {r['radial_sd_mm']:7.2f} mm  "
          f"95% CI [{r['radial_sd_95ci_mm'][0]:.2f}, {r['radial_sd_95ci_mm'][1]:.2f}]")
    print(f"  max observed          : {r['radial_max_mm']:7.2f} mm")
    print(f"  RP (mean + 3SD)       : {r['RP_mean_plus_3sd_mm']:7.2f} mm   "
          f"<-- VERIFY FORMULA against the standard")
    print(f"  sigma_x / sigma_y     : {r['sigma_x_mm']:7.2f} / {r['sigma_y_mm']:.2f} mm "
          f"(corr {r['xy_correlation']:+.2f})")
    print(f"\n  -- heading, circular statistics --")
    print(f"  circular mean         : {h['circular_mean_deg']:+7.2f} deg")
    print(f"  circular SD           : {h['circular_sd_deg']:7.2f} deg  "
          f"95% CI [{h['circular_sd_95ci_deg'][0]:.2f}, {h['circular_sd_95ci_deg'][1]:.2f}]")
    print(f"  max abs residual      : {h['max_abs_residual_deg']:7.2f} deg")
    print(f"\n  -- session trend --")
    print(f"  slope {tr['slope_mm_per_trial']:+.3f} mm/trial, 95% CI "
          f"[{tr['slope_95ci'][0]:+.3f}, {tr['slope_95ci'][1]:+.3f}], "
          f"p = {tr['p_value']:.3f} -> "
          f"{'significant' if tr['significant_at_0.05'] else 'NOT significant'}")
    print(f"\n  NOT REPORTED: pose accuracy. {res['NOT_accuracy']['note']}")
    print(f"{'='*74}")
    print(f"  figure -> {png}\n  stats  -> {js}\n")


if __name__ == "__main__":
    main()
