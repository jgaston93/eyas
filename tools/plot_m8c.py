"""Plots for the M8c entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m8c_cases executable (M8c1, D-045) and
writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m8c_cases build/m8c_cases
    .venv/bin/python tools/plot_m8c.py build/m8c_cases docs/validation/m8
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# Two variants per panel: blue solid and orange dashed (identity by style too).
A = dict(color="C0", ls="-", marker="o", ms=4, lw=2)
B = dict(color="C1", ls="--", marker="s", ms=4, lw=2)


def load(path, skip=1):
    """A CSV with a header row, after `skip` comment lines."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8",
                         skip_header=skip)


def trims(data, out):
    d = load(data / "m8c_trims.csv")
    fig, ax = plt.subplots(2, 2, figsize=(9, 6.5), sharex=True)
    panels = (("stabilator_deg", "stabilator incidence [deg]"),
              ("pitch_deg", "pitch attitude [deg]"),
              ("lon_cyclic", "longitudinal cyclic [-1..1, fwd +]"),
              ("ay_sensor_mps2", "lateral accelerometer [m/s²]"))
    for a, (col, label) in zip(ax.flat, panels):
        for on, style, name in ((1, A, "lateral-acceleration feedback on (flying config)"),
                                (0, B, "feedback zeroed")):
            m = d["lateral_accel"] == on
            a.plot(d["knots"][m], d[col][m], label=name, **style)
        a.set_ylabel(label)
        a.grid(True, alpha=0.3)
    for a in ax[1]:
        a.set_xlabel("airspeed [kt]")
    ax[0, 0].legend(fontsize=8, loc="upper right")
    fig.suptitle("UH-60A level trims with the flight control system, ISA sea level (D-045)")
    fig.tight_layout()
    fig.savefig(out / "m8c_trims.png", dpi=110)
    plt.close(fig)


def modes(data, out):
    d = load(data / "m8c_modes.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 4), sharey=True)
    for a, kt in zip(ax, speeds):
        for kind, style, name in (("dynamic", A, "dynamic stabilator (D-045)"),
                                  ("held", B, "held at its trimmed incidence")):
            m = (d["knots"] == kt) & (d["stabilator"] == kind) & (np.hypot(d["re"], d["im"]) < 3)
            a.plot(d["re"][m], d["im"][m], ls="none", marker=style["marker"], ms=7,
                   color=style["color"], mfc="none" if kind == "held" else style["color"],
                   label=name)
        a.axvline(0.0, color="k", lw=0.8)
        a.axhline(0.0, color="k", lw=0.5, alpha=0.4)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("Re λ [1/s]")
        a.grid(True, alpha=0.3)
    ax[0].set_ylabel("Im λ [rad/s]")
    ax[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("UH-60A condensed 6-DOF modes, |λ| < 3 rad/s (right of the line: unstable)")
    fig.tight_layout()
    fig.savefig(out / "m8c_modes.png", dpi=110)
    plt.close(fig)


def start(data, out):
    d = load(data / "m8c_start.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 3.6), sharey=True)
    for a, kt in zip(ax, speeds):
        for kind, style, name in (("orbit", A, "sensors on their free-flight orbit (now)"),
                                  ("held", B, "sensors at held-body values (before)")):
            m = (d["knots"] == kt) & (d["start"] == kind)
            a.semilogy(d["time_s"][m], np.maximum(np.abs(d["stabilator_change_deg"][m]), 1e-7),
                       color=style["color"], ls=style["ls"], lw=2, label=name)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("time from trim [s]")
        a.set_ylim(1e-6, 0.2)
        a.grid(True, which="both", alpha=0.3)
    ax[0].set_ylabel("|stabilator change| [deg]")
    ax[0].legend(fontsize=8, loc="lower right")
    fig.suptitle("Free flight from the trim: the stabilator's start-up transient (D-045 amendment)")
    fig.tight_layout()
    fig.savefig(out / "m8c_start.png", dpi=110)
    plt.close(fig)


def kick(data, out):
    d = load(data / "m8c_kick.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 3.6), sharey=True)
    for a, kt in zip(ax, speeds):
        m = d["knots"] == kt
        a.plot(d["time_s"][m], d["stabilator_difference_deg"][m], color="C0", lw=2)
        a.axvspan(1.5, d["time_s"][m].max(), color="k", alpha=0.06)
        for y in (0.25, -0.25):
            a.axhline(y, color="k", ls="--", lw=1)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("time [s]")
        a.grid(True, alpha=0.3)
    ax[0].set_ylabel("kicked − unkicked stabilator [deg]")
    ax[0].text(1.55, 0.27, "gate after 1.5 s: ±0.25°", fontsize=8)
    fig.suptitle("A 0.5° stabilator-servo kick dies out (Trim.FcsKickDiesOut)")
    fig.tight_layout()
    fig.savefig(out / "m8c_kick.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    trims(data, out)
    modes(data, out)
    start(data, out)
    kick(data, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
