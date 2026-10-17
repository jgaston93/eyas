"""Plots for the M8a entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m8a_cases executable (the D-042 long
runs) and, if present in the same directory, the eyas_bench timing CSVs
(bench_idle.csv, bench_load15.csv, bench_load16.csv), and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m8a_cases build/m8a_cases
    ./build/release/bench/eyas_bench --csv build/m8a_cases/bench_idle.csv
    ./build/release/bench/eyas_bench --load 15 --csv build/m8a_cases/bench_load15.csv
    ./build/release/bench/eyas_bench --load 16 --csv build/m8a_cases/bench_load16.csv
    .venv/bin/python tools/plot_m8a.py build/m8a_cases docs/validation/m8
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BUDGET_US = 0.5 * 1e6 / 60.0  # D-005: half a 60 Hz frame


def load(path, skip=1):
    """A CSV with a header row, after `skip` comment lines."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8",
                         skip_header=skip)


def held_orbit(data, out):
    d = load(data / "m8a_held_orbit.csv")
    fig, ax = plt.subplots(figsize=(7.5, 4))
    for kt, label in ((0.0, "hover"), (120.0, "120 kt")):
        m = d["speed_kt"] == kt
        ax.semilogy(d["time_s"][m] / 60.0, d["deviation"][m], label=label)
    ax.axhline(1e-8, color="k", ls="--", lw=1, label="trim periodicity tolerance (gate)")
    ax.set_xlabel("time [min]")
    ax.set_ylabel("max |x(nT) − x(0)|, component states")
    ax.set_title("UH-60A held periodic orbit, RK4 at T/256 (D-042 b)")
    ax.set_ylim(1e-11, 1e-7)
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m8a_held_orbit.png", dpi=110)
    plt.close(fig)


def parked(data, out):
    d = load(data / "m8a_parked.csv")
    t = d["time_s"] / 60.0
    fig, ax = plt.subplots(3, 1, figsize=(7.5, 8), sharex=True)
    ref = np.searchsorted(d["time_s"], 60.0)  # values at 60 s, after settling
    horiz = np.hypot(d["x_m"] - d["x_m"][ref], d["y_m"] - d["y_m"][ref])
    ax[0].semilogy(t, np.maximum(horiz, 1e-12) * 1e3, label="horizontal CG move from 60 s [mm]")
    ax[0].semilogy(t, np.maximum(np.abs(d["z_m"] - d["z_m"][ref]), 1e-12) * 1e3,
                   label="vertical CG move from 60 s [mm]")
    ax[0].set_title("UH-60A parked on its gear, brakes set, 30 min (D-042 d)")
    ax[0].legend(fontsize=8)
    for name, label in (("roll_rad", "roll"), ("pitch_rad", "pitch"), ("heading_rad", "heading")):
        ax[1].semilogy(t, np.maximum(np.abs(d[name] - d[name][ref]), 1e-15),
                       label=f"|Δ {label}| from 60 s")
    ax[1].set_ylabel("[rad]")
    ax[1].legend(fontsize=8)
    ax[2].semilogy(t, d["speed_mps"], label="|v_b| [m/s]")
    ax[2].semilogy(t, d["rate_radps"], label="|ω_b| [rad/s]")
    ax[2].semilogy(t, np.maximum(d["quat_norm_error"], 1e-17), label="quaternion norm error [-]")
    ax[2].set_xlabel("time [min]")
    ax[2].legend(fontsize=8)
    for a in ax:
        a.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "m8a_parked.png", dpi=110)
    plt.close(fig)


def bench(data, out):
    runs = [(n, data / f"bench_{n}.csv") for n in ("idle", "load15", "load16")]
    runs = [(n, p) for n, p in runs if p.exists()]
    if not runs:
        return
    titles = {"idle": "idle", "load15": "15 busy threads", "load16": "16 busy threads"}
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for i, kind in enumerate(("step", "advance")):
        for n, p in runs:
            d = load(p, skip=0)
            us = d["us"][d["kind"] == kind]
            ax[i].hist(us, bins=np.logspace(np.log10(max(us.min(), 1.0)), np.log10(us.max()) + 0.01, 80),
                       histtype="step", label=f"{titles[n]} (max {us.max():.0f} µs)")
        ax[i].set_xscale("log")
        ax[i].set_yscale("log")
        ax[i].set_xlabel(f"{kind}() time [µs]")
        ax[i].set_ylabel("calls")
        ax[i].grid(True, which="both", alpha=0.3)
        if kind == "advance":
            ax[i].axvline(BUDGET_US, color="k", ls="--", lw=1, label="D-005 budget, 60 Hz")
        ax[i].legend(fontsize=7)
    fig.suptitle("UH-60A timing, all cases (hover, 120 kt, ground), release build")
    fig.tight_layout()
    fig.savefig(out / "m8a_bench.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    held_orbit(data, out)
    parked(data, out)
    bench(data, out)


if __name__ == "__main__":
    main()
