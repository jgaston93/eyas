"""Plots for the M6d entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m6d_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m6d_cases build/m6d_cases
    .venv/bin/python tools/plot_m6d.py build/m6d_cases docs/validation/m6
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

LBF = 4.4482216152605
DEG = np.pi / 180.0


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def drop(data, out):
    d = load(data / "m6d_drop.csv")
    weight = 16638.0
    fig, ax = plt.subplots(3, 1, figsize=(7.5, 8.5), sharex=True)
    ax[0].plot(d["t"], d["height"], label="CG height above the ground [m]")
    ax[0].legend(fontsize=8)
    ax[0].set_title("UH-60A dropped level from 0.3 m: collective down, brakes set, ground pedal")
    ax[1].plot(d["t"], d["pitch"] / DEG, label="pitch [deg]")
    ax[1].plot(d["t"], d["roll"] / DEG, label="roll [deg]")
    ax[1].legend(fontsize=8)
    for i, name in enumerate(("left main", "right main", "tail")):
        ax[2].plot(d["t"], d[f"n{i}"] / LBF / 1000.0, label=name)
    ax[2].plot(d["t"], (d["n0"] + d["n1"] + d["n2"]) / LBF / 1000.0, "k", label="sum")
    ax[2].plot(d["t"], weight / 1000.0 - d["lift"] / LBF / 1000.0, "k--",
               label="weight minus aerodynamic lift")
    ax[2].set_ylim(0, 1.6 * weight / 1000.0)
    ax[2].set_ylabel("ground normal load [klbf]")
    ax[2].set_xlabel("time [s]")
    ax[2].legend(fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(out / "m6d_drop.png", dpi=110)
    plt.close(fig)


def rocking(data, out):
    d = load(data / "m6d_drop.csv")
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    ax.semilogy(d["t"], np.abs(d["q"]) / DEG + 1e-6, lw=0.6, label="|pitch rate| [deg/s]")
    ax.semilogy(d["t"], np.abs(d["p"]) / DEG + 1e-6, lw=0.6, label="|roll rate| [deg/s]")
    ax.set_ylim(1e-3, 30)
    ax.set_xlabel("time [s]")
    ax.set_title("Residual rocking on the gear after the drop (D-032 \"to watch in M6d\")")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m6d_rocking.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    data = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    drop(data, out)
    rocking(data, out)


if __name__ == "__main__":
    main()
