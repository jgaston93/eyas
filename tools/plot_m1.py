"""Plots for the M1 entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m1_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m1_cases build/m1_cases
    .venv/bin/python tools/plot_m1.py build/m1_cases docs/validation/m1
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

EPS = np.finfo(float).eps


def load(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


def seconds(d):
    return d["time_ns"] * 1e-9


def torque_free(src, out):
    d = load(src / "torque_free.csv")
    t = seconds(d)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.semilogy(t[1:], np.abs(d["dE_rel"][1:]) + EPS * 1e-3, label="|ΔE| / E₀")
    ax.semilogy(t[1:], d["dH_rel"][1:] + EPS * 1e-3, label="|ΔH_n| / |H₀|")
    ax.axhline(1e-8, color="k", ls="--", lw=1, label="M1 limit 1e-8")
    ax.axhline(EPS, color="gray", ls=":", lw=1, label="double eps")
    ax.set(xlabel="t [s]", ylabel="relative drift",
           title="Torque-free tumble, UH-60A airframe inertias with I_xz, RK4 at 1 kHz")
    ax.legend(loc="center right")
    fig.tight_layout()
    fig.savefig(out / "m1_torque_free.png", dpi=120)


def intermediate_axis(src, out):
    d = load(src / "intermediate_axis.csv")
    t = seconds(d)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    a1.plot(t, d["w1"], label="ω₁ (minor axis)")
    a1.plot(t, d["w2"], label="ω₂ (intermediate, spin axis)")
    a1.plot(t, d["w3"], label="ω₃ (major axis)")
    a1.set(ylabel="ω [rad/s]", title="Spin about the intermediate axis: the flip")
    a1.legend(loc="lower left")
    a2.semilogy(t, np.abs(d["w1"]) + 1e-30, label="|ω₁| simulated")
    a2.semilogy(t, d["w1_linear"], "k--", lw=1, label="ε e^{λt} (linearized Euler eqs.)")
    a2.set(xlabel="t [s]", ylabel="|ω₁| [rad/s]", ylim=(1e-9, 10))
    a2.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out / "m1_intermediate_axis.png", dpi=120)


def oscillator(src, out):
    d = load(src / "oscillator.csv")
    t = seconds(d)
    s0 = d["s_exact"][0]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    a1.plot(t, d["s"], label="s simulated (coupled solve)")
    a1.plot(t, d["s_exact"], "k--", lw=1, label="s₀ cos(ω_n t)")
    a1.set(ylabel="s [m]", title="Spring–mass on a rail: two-mass oscillator, ω_n = 10 rad/s")
    a1.legend(loc="upper right")
    a2.plot(t, (d["s"] - d["s_exact"]) / s0)
    a2.axhline(1e-7, color="k", ls="--", lw=1, label="test bound ±1e-7")
    a2.axhline(-1e-7, color="k", ls="--", lw=1)
    a2.set(xlabel="t [s]", ylabel="(s − s_exact) / s₀")
    a2.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(out / "m1_oscillator.png", dpi=120)


def quat_norm(src, out):
    d = np.genfromtxt(src / "quat_norm_sweep.csv", delimiter=",", names=True)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.loglog(d["omega_rad_s"], d["measured"] + 1e-20, "o", ms=4, label="measured (before renormalization)")
    ax.loglog(d["omega_rad_s"], d["predicted"], "k-", lw=1, label="RK4 prediction ≈ θ⁶/144, θ = |ω|h/2")
    ax.axhline(1e-12, color="r", ls="--", lw=1, label="M1 limit 1e-12")
    ax.axhline(EPS, color="gray", ls=":", lw=1, label="double eps")
    ax.set(xlabel="|ω| [rad/s]", ylabel="| ‖q‖ − 1 | per step", ylim=(1e-18, 1e-8),
           title="Quaternion norm drift per 1 ms step")
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(out / "m1_quat_norm.png", dpi=120)


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    torque_free(src, out)
    intermediate_axis(src, out)
    oscillator(src, out)
    quat_norm(src, out)


if __name__ == "__main__":
    main()
