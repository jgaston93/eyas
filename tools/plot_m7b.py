"""Plots for the M7b entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m7b_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m7b_cases build/m7b_cases
    .venv/bin/python tools/plot_m7b.py build/m7b_cases docs/validation/m7
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# Mode labels in a fixed order, so each keeps its colour across plots.
LABELS = ["phugoid", "short period", "heave subsidence", "pitch subsidence",
          "speed subsidence", "Dutch roll", "lateral phugoid", "lateral oscillation",
          "roll subsidence", "yaw subsidence", "spiral"]


def load(path):
    """A CSV whose first line is a # comment and second the column names."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8",
                         skip_header=1)


def modes(data, out):
    d = load(data / "m7b_modes.csv")
    d = d[d["wn"] > 1e-6]
    fig, ax = plt.subplots(1, 3, figsize=(13.0, 4.2))
    for i, label in enumerate(LABELS):
        m = d[d["label"] == label]
        if len(m) == 0:
            continue
        c = f"C{i % 10}"
        ax[0].plot(m["re"], m["im"], "o", ms=4, color=c, label=label)
        ax[1].plot(m["knots"], m["wn"], "o", ms=4, color=c, label=label)
        ax[2].plot(m["knots"], m["zeta"], "o", ms=4, color=c, label=label)
    a = ax[0]
    a.axvline(0.0, color="0.7", lw=0.6)
    a.set_xlabel("Re λ [1/s]")
    a.set_ylabel("Im λ [rad/s]")
    a.set_title("roots, 0–150 kt", fontsize=9)
    a = ax[1]
    a.set_xlabel("true airspeed [kt]")
    a.set_ylabel("natural frequency ω_n [rad/s]")
    a.set_yscale("log")
    a = ax[2]
    a.axhline(0.0, color="0.7", lw=0.6)
    a.set_xlabel("true airspeed [kt]")
    a.set_ylabel("damping ratio ζ")
    a.legend(fontsize=7, loc="lower right")
    fig.suptitle("UH-60A condensed 6-DOF modes (labels are indicative where modes couple)",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "m7b_modes.png", dpi=120)
    plt.close(fig)


def derivatives(data, out):
    d = load(data / "m7b_derivatives.csv")
    v = d["knots"]
    names = [("X_u", "1/s"), ("Z_w", "1/s"), ("M_q", "1/s"), ("L_p", "1/s"),
             ("N_r", "1/s"), ("M_u", "rad/(m·s)"), ("M_w", "rad/(m·s)"), ("L_v", "rad/(m·s)"),
             ("N_v", "rad/(m·s)"), ("Y_v", "1/s"), ("Z_col", "m/s² per unit"),
             ("M_lon", "rad/s² per unit"), ("L_lat", "rad/s² per unit"),
             ("N_ped", "rad/s² per unit")]
    fig, ax = plt.subplots(4, 4, figsize=(12.0, 10.0), sharex=True)
    for a, (n, unit) in zip(ax.flat, names):
        a.plot(v, d[n], "o-", ms=3, color="C0")
        a.axhline(0.0, color="0.7", lw=0.6)
        a.set_title(f"{n} [{unit}]", fontsize=9)
    for a in ax.flat[len(names):]:
        a.axis("off")
    for a in ax[-1]:
        a.set_xlabel("true airspeed [kt]")
    fig.suptitle("UH-60A derivatives, condensed model (Padfield App. 4B form; L, N primed; "
                 "controls per unit of travel)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "m7b_derivatives.png", dpi=120)
    plt.close(fig)


def periodicity(data, out):
    f = load(data / "m7b_floquet.csv")
    p = load(data / "m7b_periodicity.csv")
    s = load(data / "m7b_sscaling.csv")
    fig, ax = plt.subplots(1, 3, figsize=(13.0, 4.2))
    a = ax[0]
    mus = np.unique(f["mu"])
    size = np.hypot(f["avg_re"], f["avg_im"])
    for lo, hi, c, mk in [(0.0, 1.0, "C0", "o"), (1.0, 3.0, "C9", "s")]:
        sel = (size >= lo) & (size < hi)
        gap = [f["gap"][sel & (f["mu"] == m)].max() if np.any(sel & (f["mu"] == m)) else np.nan
               for m in mus]
        a.semilogy(mus, np.maximum(gap, 1e-12), mk + "-", ms=3, color=c,
                   label=f"{lo:g} ≤ |λ| < {hi:g} rad/s")
    a.set_xlabel("advance ratio μ")
    a.set_ylabel("largest |Floquet − averaged| [1/s]")
    a.set_title("averaging error", fontsize=9)
    a.legend(fontsize=7)
    a = ax[1]
    a.semilogy(p["mu"], p["variation"], "o-", ms=3, color="C1", label="all columns")
    a.semilogy(p["mu"], p["variation_without_beta_d"], "s-", ms=3, color="C2",
               label="without β_d columns")
    a.set_xlabel("advance ratio μ")
    a.set_ylabel("max |A_M(ψ) − Ā| / |Ā|, body rows")
    a.set_title("periodic part of the multiblade Jacobian", fontsize=9)
    a.legend(fontsize=7)
    a = ax[2]
    for i, kt in enumerate(np.unique(s["knots"])):
        m = s[s["knots"] == kt]
        a.loglog(m["s"], m["gap"], "o-", ms=3, color=f"C{i + 3}", label=f"{kt:.0f} kt")
    ref = np.array([0.125, 1.0])
    top = s["gap"][s["s"] == 1.0].max()
    a.loglog(ref, top * ref ** 2, "--", color="0.5", lw=0.8, label="slope 2")
    a.set_xticks([0.125, 0.25, 0.5, 1.0], ["1/8", "1/4", "1/2", "1"])
    a.minorticks_off()
    a.set_xlabel("scale s of the periodic part")
    a.set_ylabel("largest |Floquet − averaged| [1/s]")
    a.set_title("second order in the periodic part", fontsize=9)
    a.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "m7b_floquet.png", dpi=120)
    plt.close(fig)


def l3(data, out):
    d = load(data / "m7b_l3.csv")
    v = np.unique(d["knots"])
    per = [np.max(d["c0_periodic"][d["knots"] == k]) for k in v]
    avg = [np.max(d["c0_averaged"][d["knots"] == k]) for k in v]
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.semilogy(v, np.maximum(per, 1e-7), "o-", ms=4, color="C0", label="periodic model")
    ax.semilogy(v, np.maximum(avg, 1e-7), "s-", ms=4, color="C1", label="averaged model")
    ax.step([0, 60, 60, 150], [1e-4, 1e-4, 1e-3, 1e-3], where="post", color="0.4", lw=1.0,
            ls="--", label="gate (periodic)")
    ax.annotate("30 kt: tail-rotor fin-blockage\ncorner (GenHel), not gated", xy=(30, per[3]),
                xytext=(45, 3e-2), fontsize=7, arrowprops={"arrowstyle": "->", "lw": 0.6})
    ax.set_xlabel("true airspeed [kt]")
    ax.set_ylabel("leftover c0 of the relative error")
    ax.set_title("L3: linear against nonlinear, largest leftover over 12 cases", fontsize=9)
    ax.legend(fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(out / "m7b_l3.png", dpi=120)
    plt.close(fig)


def stepsize(data, out):
    d = load(data / "m7b_stepsize.csv")
    keys = [q for q in np.unique(d["quantity"]) if not q.startswith("mode")]
    fig, ax = plt.subplots(1, 3, figsize=(13.0, 4.2), sharey=True)
    for a, kt in zip(ax, np.unique(d["knots"])):
        m = d[d["knots"] == kt]
        for i, q in enumerate(keys):
            r = m[m["quantity"] == q]
            ref = r["value"][r["factor"] == 1.0][0]
            rel = np.abs(r["value"] / ref - 1.0)
            a.loglog(r["factor"], np.maximum(rel, 1e-12), "o-", ms=3, lw=0.8,
                     color=f"C{i % 10}", label=q)
        a.set_title(f"{kt:.0f} kt", fontsize=9)
        a.set_xlabel("step scale factor")
    ax[0].set_ylabel("relative change from factor 1")
    ax[-1].legend(fontsize=6, ncol=2)
    fig.suptitle("Step-size study: key derivatives", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "m7b_stepsize.png", dpi=120)
    plt.close(fig)


def main():
    data = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    modes(data, out)
    derivatives(data, out)
    periodicity(data, out)
    l3(data, out)
    stepsize(data, out)


if __name__ == "__main__":
    main()
