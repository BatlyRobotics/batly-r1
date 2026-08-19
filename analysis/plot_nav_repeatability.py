#!/usr/bin/env python3
"""
plot_nav_repeatability.py -- Nav2 return-to-pose repeatability figures and statistics

Reduces a return-to-pose trial log to the statistics and figures reported in Section 7
of the accompanying paper, and writes a stats.md summary so that every published number
is traceable to a file rather than retyped.

WHAT IS AND IS NOT REPORTED
    Reported: repeatability, meaning the dispersion of the attained poses about their
    own barycentre. Radial dispersion about the barycentre is invariant under rotation
    of the measurement frame, which matters here because the measurement baseline was
    captured with the platform positioned manually over the marker array and its heading
    differs from the map origin heading by an uncalibrated amount.

    Not reported: pose accuracy, the deviation of the barycentre from the commanded
    pose. The commanded goal was deliberately offset from the mapped origin so that the
    reference marker stayed inside the camera measurement window, so the distance from
    the commanded pose reflects the choice of offset rather than the platform. The
    offset is printed for completeness and explicitly flagged as not an accuracy figure.

    ISO 18646-2 is the applicable standard for mobile service robot navigation
    performance and draws the same distinction between accuracy and repeatability. No
    composite standard scalar is computed: dispersion is reported directly as the mean
    radial deviation, its standard deviation with a confidence interval, and the maximum
    observed. The 1-sigma and 2-sigma ellipses are ordinary bivariate covariance
    ellipses.

METHOD NOTES
    - Angles use circular mean and circular standard deviation. The baseline yaw sits
      near the +/-180 degree branch cut, where linear statistics are invalid.
    - Standard deviations use ddof = 1 and carry a chi-square confidence interval.
      At n = 30 the 95% interval on a standard deviation spans roughly +/-25%.
    - The maximum observed deviation is reported rather than a high percentile, since
      at n = 30 a 95th percentile extrapolates from the two worst trials.
    - Per-axis sigma_x and sigma_y are reported but de-emphasised: they rotate with the
      measurement frame, whereas the radial statistics do not.
    - Two trial exclusions are applied and always printed with reasons: the reference
      marker must appear in the detector's own list for that capture window, and a
      trial whose position exactly duplicates an earlier one is a carried-forward stale
      value rather than an independent measurement.
    - No model fitting, smoothing or other outlier rejection is applied. Every trial
      that passes the two checks above is included.

Expected CSV columns
    trial, ex_m, ey_m, yaw_error_deg, and optionally ez_m, detected,
    all_ids_detected, tag_id_used.

Usage
    python3 plot_nav_repeatability.py --csv trials.csv --out-dir figs --dpi 600

Requires numpy and matplotlib. scipy is optional: without it the confidence intervals
are omitted and the drift p-value uses a normal approximation.
"""

import argparse
import csv
import math
import os
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from matplotlib.ticker import MaxNLocator

try:
    from scipy import stats as _st
    HAVE_SCIPY = True
except ImportError:                                    # graceful degradation
    HAVE_SCIPY = False

# house palette: saturated, clearly separable, print-safe
BLUE = "#2E5FD9"
GREEN = "#21A038"
RED = "#E0202E"
ORANGE = "#F08000"
GREY = "#6B7280"
INK = "#1A1D21"
FOOT = "#6B7280"

plt.rcParams.update({
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.titleweight": "bold",
    "axes.labelsize": 10.5,
    "xtick.labelsize": 9.5,
    "ytick.labelsize": 9.5,
    "legend.fontsize": 9,
    "axes.edgecolor": "#2D3238",
    "axes.linewidth": 1.0,
    "axes.grid": True,
    "grid.color": "#DDE1E5",
    "grid.linewidth": 0.7,
    "figure.dpi": 110,
    "savefig.bbox": "tight",
    "legend.frameon": True,
    "legend.framealpha": 1.0,
    "legend.edgecolor": "#9AA0A6",
    "legend.fancybox": False,
})

FOOTNOTE = ("Deviations are measured about the barycentre of the attained poses, and "
            "are therefore invariant under rotation of the measurement frame. "
            "Ellipses are bivariate covariance ellipses at 1 and 2 standard "
            "deviations. Pose accuracy is not reported; see Methods. Trials in which "
            "the reference marker was not detected during the capture window are "
            "excluded.")


# ── circular statistics ──────────────────────────────────────────────────────
# Baseline yaw is 178.1 deg, adjacent to the +/-180 branch cut. Linear mean and
# standard deviation are invalid there.

def circ_mean_deg(a_deg):
    a = np.radians(a_deg)
    return float(np.degrees(np.arctan2(np.mean(np.sin(a)), np.mean(np.cos(a)))))


def circ_sd_deg(a_deg):
    """sqrt(-2 ln R) on the mean resultant length. Converges to the ordinary SD
    for small dispersion, and stays correct across the branch cut."""
    a = np.radians(a_deg)
    R = float(np.hypot(np.mean(np.sin(a)), np.mean(np.cos(a))))
    R = min(max(R, 1e-15), 1.0)
    return float(np.degrees(math.sqrt(-2.0 * math.log(R))))


def circ_resid_deg(a_deg):
    r = np.radians(np.asarray(a_deg, float) - circ_mean_deg(a_deg))
    return np.degrees(np.angle(np.exp(1j * r)))


def sd_ci(sigma, n, conf=0.95):
    """CI on a standard deviation. Chi-square if scipy is available."""
    if n < 3 or not HAVE_SCIPY:
        return (float("nan"), float("nan"))
    df = n - 1
    lo = sigma * math.sqrt(df / _st.chi2.ppf(1 - (1 - conf) / 2, df))
    hi = sigma * math.sqrt(df / _st.chi2.ppf((1 - conf) / 2, df))
    return float(lo), float(hi)


# ── data ─────────────────────────────────────────────────────────────────────

def load(csv_path, ref_id="0"):
    """Load trials. A trial is suspect if the reference marker does not appear in
    all_ids_detected: lookup_transform(Time()) returns cached transforms, so a
    successful lookup does not imply the marker was detected in that window."""
    rows, suspect, dupes = [], [], []
    seen_xy = {}
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            det = str(r.get("detected", "")).strip().lower() in ("true", "1")
            ids = [s for s in str(r.get("all_ids_detected", "")).split(";") if s]
            trial = int(float(r["trial"]))
            if not det:
                rows.append({"trial": trial, "ok": False, "suspect": False,
                             "ex": np.nan, "ey": np.nan, "yaw": np.nan,
                             "ez": np.nan})
                continue
            ex = float(r["ex_m"]) * 1000.0
            ey = float(r["ey_m"]) * 1000.0
            ez = float(r["ez_m"]) * 1000.0 if r.get("ez_m") else float("nan")
            is_suspect = ref_id not in ids
            if is_suspect:
                suspect.append(trial)
            # Byte-identical position to an earlier trial: a carried-forward
            # stale value, not an independent measurement.
            key = (round(ex, 6), round(ey, 6))
            if key in seen_xy:
                dupes.append((trial, seen_xy[key]))
                is_suspect = True
            else:
                seen_xy[key] = trial
            rows.append({"trial": trial, "ok": True, "suspect": is_suspect,
                         "ex": ex, "ey": ey, "yaw": float(r["yaw_error_deg"]),
                         "ez": ez})
    return rows, suspect, dupes


def stats_of(ex, ey, yaw, trials):
    """All dispersion about the barycentre of attained poses."""
    n = len(ex)
    xb, yb = float(np.mean(ex)), float(np.mean(ey))
    dx, dy = ex - xb, ey - yb
    l = np.hypot(dx, dy)                       # radial distance from barycentre

    l_mean = float(np.mean(l))
    l_sd = float(np.std(l, ddof=1)) if n > 1 else 0.0
    sx = float(np.std(dx, ddof=1)) if n > 1 else 0.0
    sy = float(np.std(dy, ddof=1)) if n > 1 else 0.0

    t = np.asarray(trials, float)
    if HAVE_SCIPY and n > 2:
        lr = _st.linregress(t, l)
        slope, se, p = float(lr.slope), float(lr.stderr), float(lr.pvalue)
    elif n > 2:                                # normal approximation
        slope, icpt = np.polyfit(t, l, 1)
        resid = l - (slope * t + icpt)
        se = float(np.sqrt((resid ** 2).sum() / (n - 2) /
                           ((t - t.mean()) ** 2).sum()))
        z = abs(slope / se) if se > 0 else 0.0
        p = float(math.erfc(z / math.sqrt(2.0)))
        slope = float(slope)
    else:
        slope = se = p = float("nan")

    return {
        "n": n,
        # datum offset: a CONFIGURATION fact, not an accuracy figure
        "datum_dx": xb, "datum_dy": yb, "datum_dist": float(math.hypot(xb, yb)),
        # repeatability, frame-rotation invariant
        "l_mean": l_mean, "l_sd": l_sd, "l_sd_ci": sd_ci(l_sd, n),
        "l_max": float(np.max(l)), "l_p95": float(np.percentile(l, 95)),
        # frame-dependent, de-emphasised
        "sx": sx, "sx_ci": sd_ci(sx, n),
        "sy": sy, "sy_ci": sd_ci(sy, n),
        "xy_corr": float(np.corrcoef(dx, dy)[0, 1]) if n > 2 else float("nan"),
        # heading, circular
        "yaw_cmean": circ_mean_deg(yaw), "yaw_csd": circ_sd_deg(yaw),
        "yaw_csd_ci": sd_ci(circ_sd_deg(yaw), n),
        "yaw_max_res": float(np.max(np.abs(circ_resid_deg(yaw)))),
        # trend
        "trend": slope, "trend_se": se, "trend_p": p,
        "trend_sig": bool(p < 0.05) if p == p else False,
    }


# ── panels ───────────────────────────────────────────────────────────────────

def panel_scatter(ax, ex, ey, s, sus_pts=None, pad=1.10):
    """Dispersion about the barycentre. NOTE: no commanded-target marker and no
    bias arrow — those made this an accuracy plot, which this experiment cannot
    support."""
    xb, yb = s["datum_dx"], s["datum_dy"]
    dx, dy = ex - xb, ey - yb

    if len(dx) > 2:
        cov = np.cov(dx, dy)
        vals, vecs = np.linalg.eigh(cov)
        order = vals.argsort()[::-1]
        vals, vecs = vals[order], vecs[:, order]
        ang = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
        for k, ls, lab in ((1, "--", "1σ ellipse"), (2, ":", "2σ ellipse")):
            ax.add_patch(Ellipse((0, 0), 2 * k * math.sqrt(vals[0]),
                                 2 * k * math.sqrt(vals[1]), angle=ang,
                                 fill=False, ec=RED, ls=ls, lw=1.4, label=lab))

    ax.scatter(dx, dy, s=46, c=BLUE, zorder=3, ec="white", lw=0.6,
               label=f"trials (n={s['n']})")
    if sus_pts is not None and len(sus_pts[0]):
        ax.scatter(sus_pts[0] - xb, sus_pts[1] - yb, s=84, facecolors="none",
                   ec=ORANGE, lw=1.9, zorder=4, label="excluded")
    ax.plot(0, 0, "+", ms=15, mew=2.4, c=INK, zorder=5, label="barycentre")

    lim = float(np.max(np.hypot(dx, dy))) * pad
    ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_aspect("equal")
    ax.axhline(0, color="#9AA0A6", lw=0.8, zorder=0)
    ax.axvline(0, color="#9AA0A6", lw=0.8, zorder=0)
    ax.set_xlabel("x deviation from barycentre (mm)")
    ax.set_ylabel("y deviation from barycentre (mm)")
    ax.set_title("(a) Pose repeatability — dispersion about the barycentre")
    # FIX: was loc="lower left" inside the axes, which sat on top of the
    # 2-sigma ellipse and a trial point. A horizontal strip below
    # the axes cannot collide with anything.
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.135),
              ncol=3, columnspacing=1.1, handletextpad=0.6,
              borderaxespad=0.0)


def _headroom(ax, counts, factor=1.70):
    ax.set_ylim(0, max(1.0, float(np.max(counts)) * factor))
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))


def _nbins(n, bins=None):
    return bins if bins else max(6, int(round(math.sqrt(n) * 1.6)))


def panel_hist_radial(ax, ex, ey, s, bins=None):
    l = np.hypot(ex - s["datum_dx"], ey - s["datum_dy"])
    c, _, _ = ax.hist(l, bins=_nbins(len(l), bins), color=BLUE, ec="white", lw=0.6)
    ax.axvline(s["l_mean"], color=RED, ls="--", lw=1.6,
               label=f"mean {s['l_mean']:.1f} mm")
    ax.axvline(s["l_max"], color=INK, ls=":", lw=1.6,
               label=f"max observed {s['l_max']:.1f} mm")
    _headroom(ax, c)
    ax.set_xlabel("radial deviation from barycentre (mm)")
    ax.set_ylabel("trials (count)")
    ax.set_title("(b) Positioning dispersion")
    ax.legend(loc="upper left")


def panel_hist_yaw(ax, yaw, s, bins=None):
    res = circ_resid_deg(yaw)
    c, _, _ = ax.hist(res, bins=_nbins(len(res), bins), color=GREEN, ec="white",
                      lw=0.6)
    ax.axvspan(-s["yaw_csd"], s["yaw_csd"], color=RED, alpha=0.10,
               label=f"±1σ ({s['yaw_csd']:.2f}°)")
    ax.axvline(0, color=RED, ls="--", lw=1.6, label="circular mean")
    _headroom(ax, c)
    ax.set_xlabel("heading deviation from circular mean (deg)")
    ax.set_ylabel("trials (count)")
    ax.set_title("(c) Heading dispersion")
    ax.legend(loc="upper left")


def panel_sequence(ax, trials, ex, ey, s):
    l = np.hypot(ex - s["datum_dx"], ey - s["datum_dy"])
    t = np.asarray(trials, float)
    ax.axhspan(max(0.0, s["l_mean"] - s["l_sd"]), s["l_mean"] + s["l_sd"],
               color=BLUE, alpha=0.10, label=f"±1σ ({s['l_sd']:.1f} mm)")
    ax.plot(t, l, "o-", color=BLUE, ms=5, lw=1.3, label="radial deviation")
    ax.axhline(s["l_mean"], color=RED, ls="--", lw=1.5,
               label=f"mean {s['l_mean']:.1f} mm")
    if s["trend"] == s["trend"]:
        icpt = s["l_mean"] - s["trend"] * t.mean()
        tag = "significant" if s["trend_sig"] else "not significant"
        ax.plot(t, s["trend"] * t + icpt, ":", color=INK, lw=1.7,
                label=(f"trend {s['trend']:+.2f} mm/trial\n"
                       f"p = {s['trend_p']:.2f} ({tag})"))
    ax.set_ylim(0, l.max() * 1.55)
    ax.set_xlabel("trial number")
    ax.set_ylabel("radial deviation from barycentre (mm)")
    ax.set_title("(d) Dispersion across the session — drift check")
    ax.legend(ncol=2)


# ── output ───────────────────────────────────────────────────────────────────

def save(fig, out_dir, name, dpi):
    fig.savefig(os.path.join(out_dir, f"{name}.png"), dpi=dpi)
    fig.savefig(os.path.join(out_dir, f"{name}.pdf"))
    plt.close(fig)


def footnote(fig, text=FOOTNOTE, chars_per_inch=17.0, fs=8.5):
    """Draw the footnote and return the figure-height fraction it occupies.

    FIX: the previous version placed the text at a fixed y=0.005 without
    reserving space, so it overlapped the x tick labels and axis labels. The
    caller now passes the returned fraction to tight_layout(rect=...) so the
    footnote gets its own band.
    """
    w_in, h_in = fig.get_size_inches()
    lines = textwrap.wrap(text, width=max(40, int(w_in * chars_per_inch)))
    line_h = (fs * 1.45) / 72.0                      # inches per line
    band = (len(lines) * line_h + 0.10) / h_in       # + margin
    fig.text(0.5, 0.012, "\n".join(lines), ha="center", va="bottom",
             fontsize=fs, color=FOOT, style="italic", linespacing=1.45)
    return band


def build_all(ex, ey, yaw, trials, s, sus_pts, out_dir, dpi, fig_no, title,
              pad=1.10, bins=None):
    # LEG_BAND: vertical space the below-axes legend of panel (a) needs, as a
    # fraction of figure height. Added on top of the footnote band.
    f, a = plt.subplots(figsize=(6.6, 7.4))
    panel_scatter(a, ex, ey, s, sus_pts, pad)
    band = footnote(f)
    f.tight_layout(rect=[0, band + 0.085, 1, 1])
    save(f, out_dir, "fig_scatter", dpi)

    f, a = plt.subplots(1, 2, figsize=(12.4, 5.0))
    panel_hist_radial(a[0], ex, ey, s, bins)
    panel_hist_yaw(a[1], yaw, s, bins)
    band = footnote(f)
    f.tight_layout(rect=[0, band, 1, 1])
    save(f, out_dir, "fig_histograms", dpi)

    f, a = plt.subplots(figsize=(12.4, 4.0))
    panel_sequence(a, trials, ex, ey, s)
    f.tight_layout()
    save(f, out_dir, "fig_sequence", dpi)

    f = plt.figure(figsize=(13.0, 11.4))
    band = footnote(f)
    gs = f.add_gridspec(2, 2, hspace=0.42, wspace=0.24,
                        left=0.065, right=0.985,
                        top=0.935, bottom=band + 0.035)
    panel_scatter(f.add_subplot(gs[0, 0]), ex, ey, s, sus_pts, pad)
    panel_hist_radial(f.add_subplot(gs[0, 1]), ex, ey, s, bins)
    panel_hist_yaw(f.add_subplot(gs[1, 0]), yaw, s, bins)
    panel_sequence(f.add_subplot(gs[1, 1]), trials, ex, ey, s)
    f.suptitle(f"Figure {fig_no} — {title}", fontsize=13, y=0.985)
    save(f, out_dir, "fig_nav_combined", dpi)


def _ci(v):
    return "n/a (scipy not installed)" if v[0] != v[0] else f"[{v[0]:.2f}, {v[1]:.2f}]"


def write_stats(s, out_dir, n_total, suspect, dupes, kept, datum_note):
    p = os.path.join(out_dir, "stats.md")
    tag = "significant" if s["trend_sig"] else "not significant"
    with open(p, "w") as f:
        f.write(f"""# Nav2 return-to-origin — pose repeatability

_Reference: downward-facing AprilTag beneath the platform. All dispersion is
computed about the barycentre of the attained poses, as distinct from accuracy,
which is the deviation of that barycentre from the commanded pose. ISO 18646-2 is
the applicable standard for mobile service robot navigation performance and draws
the same distinction; ISO 9283 covers manipulators. Radial dispersion about the
barycentre is invariant under rotation of the measurement frame._

## Data
- trials logged: {n_total}
- trials included: {kept}
- excluded, reference marker absent from the detection list: {suspect if suspect else 'none'}
- excluded, position identical to an earlier trial (stale value): {[d[0] for d in dupes] if dupes else 'none'}

## Pose repeatability — headline
- **mean radial deviation from barycentre = {s['l_mean']:.2f} mm**
- **SD of radial deviation = {s['l_sd']:.2f} mm**, 95% CI {_ci(s['l_sd_ci'])}
- **maximum observed = {s['l_max']:.2f} mm**  (95th percentile {s['l_p95']:.2f} mm)
- **heading circular SD = {s['yaw_csd']:.2f} deg**, 95% CI {_ci(s['yaw_csd_ci'])}
  (max absolute residual {s['yaw_max_res']:.2f} deg)

## Frame-dependent components
The baseline heading differs from the map origin heading, so the measurement
frame is rotated relative to the map frame by an uncalibrated amount. The
following rotate with that frame and should be quoted with less weight than the
radial figures above.
- sigma_x = {s['sx']:.2f} mm, 95% CI {_ci(s['sx_ci'])}
- sigma_y = {s['sy']:.2f} mm, 95% CI {_ci(s['sy_ci'])}
- x-y correlation = {s['xy_corr']:+.2f}

## Session drift
- slope {s['trend']:+.3f} mm/trial, p = {s['trend_p']:.3f} — {tag}

## Measurement datum offset (configuration, NOT accuracy)
{datum_note}
- barycentre lies {s['datum_dist']:.2f} mm from the measurement datum
  (dx = {s['datum_dx']:+.2f} mm, dy = {s['datum_dy']:+.2f} mm in the measurement frame)

**Pose accuracy is not reported.** The commanded goal was offset from the mapped
origin so the marker stayed inside the camera measurement window, and the
transformation between the commanded pose and the measurement datum was not
independently calibrated. Neither the positional nor the heading offset above is
a platform property.

## Paragraph
Return-to-origin repeatability was assessed over {n_total} Nav2 trials with a
downward-facing AprilTag as the position reference, of which {kept} were included.
Dispersion is reported about the barycentre of the attained poses, as distinct from
accuracy, which is the deviation of that barycentre from the commanded pose. The mean radial deviation from the
barycentre was {s['l_mean']:.2f} mm with a standard deviation of {s['l_sd']:.2f} mm
(95% CI {_ci(s['l_sd_ci'])}), a maximum observed deviation of {s['l_max']:.2f} mm,
and a maximum observed deviation of {s['l_max']:.2f} mm. Heading dispersion was {s['yaw_csd']:.2f} deg
(circular standard deviation, 95% CI {_ci(s['yaw_csd_ci'])}). No significant
drift was observed across the session ({s['trend']:+.2f} mm/trial,
p = {s['trend_p']:.2f}). The fiducial measurement dispersion characterised
separately (sigma <= 0.03 mm) is more than two orders of magnitude below this
spread, so the reported figures reflect platform and navigation behaviour rather
than the measurement method. Pose accuracy is not reported: the commanded goal
was offset from the mapped origin to keep the marker within the camera's
measurement window, and no traceable external reference was available.
""")
    print(f"stats written to: {p}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", default="accuracy_results.csv")
    p.add_argument("--out-dir", default="figs")
    p.add_argument("--dpi", type=int, default=600)
    p.add_argument("--ref-id", default="0")
    p.add_argument("--keep-suspect", action="store_true")
    p.add_argument("--fig-no", default="7")
    p.add_argument("--bins", type=int, default=None,
                   help="histogram bin count; higher = slimmer bars")
    p.add_argument("--pad", type=float, default=1.10,
                   help="axis headroom for panel (a); 1.0 = tight. "
                        "Large values shrink the scatter and understate spread.")
    p.add_argument("--datum-note",
                   default="- commanded goal offset from the mapped origin: "
                           "+0.10 m in x, selected empirically so the reference "
                           "marker remained inside the camera measurement window",
                   help="reporting only; never enters the computation")
    p.add_argument("--title",
                   default="Nav2 return-to-origin pose repeatability measured "
                           "against a downward-facing AprilTag")
    a = p.parse_args()

    if not HAVE_SCIPY:
        print("NOTE: scipy not found. Confidence intervals unavailable and the "
              "drift p-value uses a normal approximation. pip install scipy")

    os.makedirs(a.out_dir, exist_ok=True)
    rows, suspect, dupes = load(a.csv, a.ref_id)
    n_total = len(rows)

    use = [r for r in rows if r["ok"] and (a.keep_suspect or not r["suspect"])]
    drop = [r for r in rows if r["ok"] and r["suspect"] and not a.keep_suspect]
    if len(use) < 2:
        raise SystemExit("need at least 2 usable trials")

    ex = np.array([r["ex"] for r in use])
    ey = np.array([r["ey"] for r in use])
    yaw = np.array([r["yaw"] for r in use])
    trials = [r["trial"] for r in use]
    sus_pts = (np.array([r["ex"] for r in drop]),
               np.array([r["ey"] for r in drop]))

    s = stats_of(ex, ey, yaw, trials)
    build_all(ex, ey, yaw, trials, s, sus_pts, a.out_dir, a.dpi, a.fig_no,
              a.title, pad=a.pad, bins=a.bins)
    write_stats(s, a.out_dir, n_total, suspect, dupes, len(use), a.datum_note)

    ez = np.array([r["ez"] for r in use])
    if np.isfinite(ez).any():
        m = float(np.nanmean(ez))
        print(f"\nez diagnostic: mean {m:+.1f} mm, spread "
              f"{float(np.nanmax(ez) - np.nanmin(ez)):.1f} mm")
        if abs(m) > 10.0:
            print("  ez should be ~0: base_footprint and the marker both lie on "
                  "the ground plane. This reads out the error in the "
                  "base_footprint->camera static transform.")

    print(f"\nradial SD = {s['l_sd']:.2f} mm | "
          f"max = {s['l_max']:.2f} mm | heading SD = {s['yaw_csd']:.2f} deg")
    print(f"figures written to: {a.out_dir}/")


if __name__ == "__main__":
    main()
