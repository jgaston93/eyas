"""Plots and difference table for the V2 frequency responses of the M7c entry in
docs/VALIDATION.md (D-035 M7c amendment).

Reads the CSV written by eyas_m7c_fletcher (Eyas's averaged linear model at Fletcher's
conditions, with and without GenHel's primary servo) and Fletcher's digitized flight-data
responses (data/uh60a/validation/fletcher_fig3.csv, fig4.csv; human-reviewed, DATA_GAPS.md).
Writes one figure per condition (magnitude, phase and Fletcher's coherence for p and q per
inch of lateral and longitudinal cyclic) and m7c_fletcher_differences.csv: Eyas minus flight
at Fletcher's readings where the coherence is at least 0.6, up to 12 rad/s (mean, RMS,
largest; magnitude in dB, phase in deg). Fletcher's phase curves sit on their own 360 deg
branch, so Eyas's phase is moved by the whole turns that best align it over that band.

    cmake --build --preset release
    ./build/release/tests/eyas_m7c_fletcher build/m7c_fletcher
    .venv/bin/python tools/plot_m7c_fletcher.py build/m7c_fletcher docs/validation/m7c
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "data" / "uh60a" / "validation"
CONDITIONS = [("hover", "fig3", "hover"), ("80kt", "fig4", "80 kt")]
RESPONSES = [("p_lat", "p / δlat"), ("q_lon", "q / δlon"), ("p_lon", "p / δlon"),
             ("q_lat", "q / δlat")]
W_MAX = 12.0  # rad/s; Fletcher's lag-mode limit (Eyas has no lag, D-026)
COH_MIN = 0.6
GRID = 1.0 / 24.0  # the readings' grid step [decades]


def rows(path):
    with path.open(encoding="utf-8") as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def load_fletcher(fig):
    out = {}
    for r in rows(DATA / f"fletcher_{fig}.csv"):
        out.setdefault((r["response"], r["quantity"]), []).append(
            (float(r["frequency_rad_s"]), float(r["value"])))
    return {k: np.array(sorted(v)) for k, v in out.items()}


def load_eyas(path):
    out = {}
    for r in rows(path):
        out.setdefault((r["condition"], r["response"]), []).append(
            [float(r[k]) for k in ("frequency_rad_s", "magnitude_db", "phase_deg",
                                   "magnitude_db_servo", "phase_deg_servo")])
    return {k: np.array(v) for k, v in out.items()}


def coherence_at(coh, w):
    """Fletcher's coherence at w, or NaN where it has no reading within 0.75 of a grid step."""
    lw, lc = np.log10(coh[:, 0]), np.log10(w)
    c = np.interp(lc, lw, coh[:, 1])
    near = np.min(np.abs(lw[None, :] - lc[:, None]), axis=1) <= 0.75 * GRID
    return np.where(near, c, np.nan)


def used(coh, w):
    c = coherence_at(coh, w)
    return (w <= W_MAX) & (np.nan_to_num(c, nan=0.0) >= COH_MIN)


def turns(eyas_phase, flight_phase):
    """Whole turns [deg] to add to Eyas's phase to sit on the flight curve's branch."""
    return 360.0 * round(float(np.mean(flight_phase - eyas_phase)) / 360.0)


def stats(d):
    return (float(np.mean(d)), float(np.sqrt(np.mean(d * d))), float(d[np.argmax(np.abs(d))]))


def unused_spans(coh):
    """Frequency spans [rad/s] not compared, for shading."""
    w = np.logspace(-1, math.log10(20.0), 600)
    ok = used(coh, w)
    spans, start = [], None
    for wi, oi in zip(w, ok):
        if not oi and start is None:
            start = wi
        elif oi and start is not None:
            spans.append((start, wi))
            start = None
    if start is not None:
        spans.append((start, 20.0))
    return spans


def plot_condition(cond, fig_name, label, fl, ey, outdir, table):
    fig, axes = plt.subplots(3, 4, figsize=(15, 9), sharex=True,
                             gridspec_kw={"height_ratios": [3, 3, 1.4]})
    for col, (resp, title) in enumerate(RESPONSES):
        mag, ph, coh = (fl[(resp, q)] for q in ("magnitude_db", "phase_deg", "coherence"))
        e = ey[(cond, resp)]
        lw_e = np.log10(e[:, 0])
        # Eyas at Fletcher's readings, compared where used
        m_ok = used(coh, mag[:, 0])
        p_ok = used(coh, ph[:, 0])
        shift = {}
        for servo, (im, ip) in (("servo", (3, 4)), ("no_servo", (1, 2))):
            em = np.interp(np.log10(mag[:, 0]), lw_e, e[:, im])
            ep = np.interp(np.log10(ph[:, 0]), lw_e, e[:, ip])
            shift[servo] = turns(ep[p_ok], ph[p_ok, 1]) if p_ok.any() else 0.0
            for qty, d, n in (("magnitude_db", em[m_ok] - mag[m_ok, 1], m_ok.sum()),
                              ("phase_deg", ep[p_ok] + shift[servo] - ph[p_ok, 1], p_ok.sum())):
                if n:
                    lo = (mag if qty == "magnitude_db" else ph)[:, 0][
                        m_ok if qty == "magnitude_db" else p_ok]
                    table.append([cond, resp, servo, qty, n, f"{lo.min():.2f}", f"{lo.max():.2f}",
                                  *(f"{v:.2f}" for v in stats(d))])
        spans = unused_spans(coh)
        for row, (data, ie, ylab) in enumerate(((mag, (3, 1), "magnitude [dB, (rad/s)/in]"),
                                                (ph, (4, 2), "phase [deg]"),
                                                (coh, None, "coherence"))):
            ax = axes[row, col]
            for a, b in spans:
                ax.axvspan(a, b, color="0.88", lw=0, zorder=0)
            ax.plot(data[:, 0], data[:, 1], color="black", lw=1.4, label="flight (Fletcher)")
            if ie is not None:
                off = shift["servo"] if row == 1 else 0.0
                off_ns = shift["no_servo"] if row == 1 else 0.0
                ax.plot(e[:, 0], e[:, ie[0]] + off, color="tab:red", lw=1.6,
                        label="Eyas with GenHel servo")
                ax.plot(e[:, 0], e[:, ie[1]] + off_ns, color="tab:red", lw=1.0, ls="--",
                        label="Eyas, no servo")
            else:
                ax.axhline(COH_MIN, color="0.4", lw=0.8, ls=":")
                ax.set_ylim(0, 1.05)
            ax.axvline(W_MAX, color="tab:blue", lw=0.8, ls="--")
            ax.set_xscale("log")
            ax.set_xlim(0.1, 20)
            ax.grid(True, which="both", lw=0.3, alpha=0.5)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 0:
                ax.set_title(title)
            if row == 2:
                ax.set_xlabel("frequency [rad/s]")
    axes[0, 0].legend(fontsize=8, loc="lower left")
    fig.suptitle(f"UH-60A {label}: Eyas against Fletcher's flight data (21st ERF 1995 Paper "
                 f"VII-4, Fig. {fig_name[-1]}); grey: not compared (coherence < 0.6 or > 12 rad/s)")
    fig.tight_layout()
    fig.savefig(outdir / f"m7c_fletcher_{cond}.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    data, outdir = Path(sys.argv[1]), Path(sys.argv[2])
    outdir.mkdir(parents=True, exist_ok=True)
    ey = load_eyas(data / "m7c_fletcher.csv")
    table = []
    for cond, fig_name, label in CONDITIONS:
        plot_condition(cond, fig_name, label, load_fletcher(fig_name), ey, outdir, table)
    with (outdir / "m7c_fletcher_differences.csv").open("w", newline="", encoding="utf-8") as f:
        f.write("# Eyas minus Fletcher's flight data where coherence >= 0.6 and w <= 12 rad/s "
                "(tools/plot_m7c_fletcher.py)\n")
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["condition", "response", "eyas", "quantity", "n", "w_min", "w_max", "mean",
                    "rms", "largest"])
        w.writerows(table)
    for r in table:
        print(",".join(str(v) for v in r))
    return 0


if __name__ == "__main__":
    sys.exit(main())
