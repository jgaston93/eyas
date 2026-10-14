"""Plots for the M6c entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m6c_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m6c_cases build/m6c_cases
    .venv/bin/python tools/plot_m6c.py build/m6c_cases docs/validation/m6
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

LBF = 4.4482216152605
INCH = 0.0254


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def drop(data, out):
    d = load(data / "m6c_drop.csv")
    fig, ax = plt.subplots(3, 1, figsize=(7.5, 8), sharex=True)
    ax[0].plot(d["t"], d["height"] - d["height"][-1], label="CG height above its final value [m]")
    ax[0].plot(d["t"], d["vz"], label="vertical speed (down +) [m/s]")
    ax[0].legend(fontsize=8)
    ax[0].set_title("Drop of the UH-60A gear from 0.3 m, 7000 kg airframe, damping on")
    for i, name in enumerate(("left main", "right main", "tail")):
        ax[1].plot(d["t"], d[f"s{i}"] / INCH, label=name)
        ax[2].plot(d["t"], d[f"n{i}"] / LBF / 1000.0, label=name)
    ax[1].set_ylabel("strut stroke [in]")
    ax[1].legend(fontsize=8)
    ax[2].set_ylabel("normal load [klbf]")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "m6c_drop.png", dpi=110)
    plt.close(fig)


def energy(data, out):
    d = load(data / "m6c_energy.csv")
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.loglog(d["step_ms"], d["relative_drift"], "o-", label="worst |E − E₀| / (W − T)·h")
    ax.axhline(1e-3, color="C1", ls="--", lw=1, label="gate at 1 kHz")
    ax.axhline(1e-5, color="C2", ls="--", lw=1, label="gate at 8 kHz")
    ax.set_xlabel("physics step [ms]")
    ax.set_ylabel("relative energy drift")
    ax.set_title("Drop with damping and friction off: energy against step")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m6c_energy.png", dpi=110)
    plt.close(fig)


def slope(data, out):
    d = load(data / "m6c_slope.csv")
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    for c, a in zip((0, 1), ax):
        r = d[d["case"] == c]
        a.plot(r["t"], r["x"], label="down-slope displacement [m]")
        if c == 1:
            t0 = 5.0
            i0 = np.argmin(abs(r["t"] - t0))
            acc = r["expected_accel"][0]
            tt = r["t"][i0:]
            a.plot(tt, r["x"][i0] + r["v"][i0] * (tt - t0) + 0.5 * acc * (tt - t0) ** 2, "--",
                   label=f"g(sin α − K_SLIP μ cos α) = {acc:.3f} m/s²")
        a.set_title(f"slope {r['angle_deg'][0]:.1f}°" + (" (hold)" if c == 0 else " (slide)"))
        a.set_xlabel("time [s]")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    fig.suptitle("Every wheel braked, μ = 0.6 (atan μ = 31.0°)")
    fig.tight_layout()
    fig.savefig(out / "m6c_slope.png", dpi=110)
    plt.close(fig)


def roll(data, out):
    d = load(data / "m6c_roll.csv")
    a = float(np.atleast_1d(load(data / "m6c_roll_expected.csv")["decel"])[0])
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.plot(d["t"], d["u"], label="forward speed")
    ax.plot(d["t"], d["u"][0] - a * d["t"], "--", label=f"K_SLIP k_off μ g = {a:.4f} m/s²")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed [m/s]")
    ax.set_title("Brakes off, rolling from 2 m/s on level ground")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m6c_roll.png", dpi=110)
    plt.close(fig)


def vs_sweep(data, out):
    d = load(data / "m6c_vs_sweep.csv")
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.semilogx(d["vs_ftps"], d["distance"] * 100.0, "o-")
    ax.axvline(0.1, color="C1", ls="--", lw=1, label="v_s = 0.1 ft/s (D-032)")
    ax.set_xlabel("sliding blend speed v_s [ft/s]")
    ax.set_ylabel("distance to rest [cm]")
    ax.set_title("Braked stop from 4 ft/s: sensitivity to v_s")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m6c_vs_sweep.png", dpi=110)
    plt.close(fig)


def main():
    data = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    drop(data, out)
    energy(data, out)
    slope(data, out)
    roll(data, out)
    vs_sweep(data, out)


if __name__ == "__main__":
    main()
