"""Plots for the M2 entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m2_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m2_cases build/m2_cases
    .venv/bin/python tools/plot_m2.py build/m2_cases docs/validation/m2
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

G0 = 9.80665


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def isa_profile(src, out):
    d = load(src / "isa_profile.csv")
    z = d["Z"] / 1000.0
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(8, 4.5), sharey=True)
    for key, label, style in (("m20", "ISA − 20 K", "--"), ("0", "ISA", "-"), ("p20", "ISA + 20 K", ":")):
        a1.plot(d["T_" + key], z, style, label=label)
        a2.plot(d["rho_" + key], z, style, label=label)
    a1.axhline(11.0 * 6356.766 / (6356.766 - 11.0), color="gray", lw=0.8)
    a1.text(222, 11.3, "tropopause, H = 11 km′", fontsize=8, color="gray")
    a1.set(xlabel="temperature [K]", ylabel="geometric altitude Z [km]")
    a2.set(xlabel="density [kg/m³]")
    a1.legend()
    fig.suptitle("ISA (U.S. Standard Atmosphere 1976) with hydrostatic ΔT (D-015)")
    fig.tight_layout()
    fig.savefig(out / "m2_isa_profile.png", dpi=120)


def isa_table_check(src, out):
    d = load(src / "isa_table_check.csv")
    labels = [f"{'Z' if g else 'H'} = {a:g} m" for g, a in zip(d["geometric"], d["altitude"])]
    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(8, 4))
    w = 0.27
    ax.bar(x - w, d["dT_lsd"], w, label="T")
    ax.bar(x, d["dP_lsd"], w, label="P")
    ax.bar(x + w, d["drho_lsd"], w, label="ρ")
    ax.axhline(0.5, color="k", ls="--", lw=1, label="±½ unit (rounded: T, ρ)")
    ax.axhline(-0.5, color="k", ls="--", lw=1)
    ax.axhline(1.0, color="tab:orange", ls=":", lw=1, label="+1 unit (truncated: P)")
    ax.set_xticks(x, labels, rotation=20)
    ax.set(ylabel="Eyas − table [units of last printed digit]",
           title="Eyas vs USSA 1976 Table I (geometric Z and geopotential H tables)")
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    fig.savefig(out / "m2_isa_table.png", dpi=120)


def equilibrium(src, out):
    d = load(src / "equilibrium.csv")
    t = d["time_ns"] * 1e-9
    fig, ax = plt.subplots(figsize=(7, 4))
    floor = 1e-18
    ax.semilogy(t, d["tilted_pos"] + floor, label="tilted, translating: |Δp| [m]")
    ax.semilogy(t, d["tilted_vel"] + floor, label="tilted, translating: |Δv| [m/s] (exactly 0)")
    ax.semilogy(t, d["level_pos"] + floor, label="level, at rest: |Δp| (exactly 0)")
    ax.axhline(1e-8, color="k", ls="--", lw=1, label="round-off bound, 1e-8 m")
    ax.set(xlabel="t [s]", ylabel="drift from analytic", ylim=(1e-19, 1e-6),
           title="Equilibrium under a weight-balancing force, RK4 at 1 kHz")
    ax.legend(fontsize=8, loc="center right")
    fig.tight_layout()
    fig.savefig(out / "m2_equilibrium.png", dpi=120)


def collective_step(src, out):
    d = load(src / "collective_step.csv")
    t = d["time_ns"] * 1e-9
    a = -0.2 * G0
    ts = np.clip(t - 1.0, 0.0, None)
    w_ref = a * ts
    D_ref = 0.5 * a * ts**2
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    a1.plot(t, d["altitude"], label="Eyas altitude (Tier 1)")
    a1.plot(t, 10.0 - D_ref, "k--", lw=1, label="10 m − ½ a (t − 1)²")
    a1.set(ylabel="altitude [m MSL]", title="Collective 0.5 → 0.6 at t = 1 s: a = 0.2 g up")
    a1.legend()
    a2.semilogy(t, np.abs(d["w"] - w_ref) + 1e-18, label="|w − a(t−1)| [m/s]")
    a2.semilogy(t, np.abs(d["D"] - D_ref) + 1e-18, label="|D − ½a(t−1)²| [m]")
    a2.set(xlabel="t [s]", ylabel="error", ylim=(1e-19, 1e-10))
    a2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m2_collective_step.png", dpi=120)


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    isa_profile(src, out)
    isa_table_check(src, out)
    equilibrium(src, out)
    collective_step(src, out)


if __name__ == "__main__":
    main()
