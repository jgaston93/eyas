"""Plots for the M4 entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m4_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m4_cases build/m4_cases
    .venv/bin/python tools/plot_m4.py build/m4_cases docs/validation/m4
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

DEG = 180.0 / np.pi


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def flapping_mu(src, out):
    d = load(src / "flapping_mu.csv")
    titles = {
        "centre": "Centre-spring rotor (e = 0, λβ² = 1.12)\nvs Padfield 3rd ed. Eqs. 3.57–3.58, 3.64",
        "offset": "Offset hinge (e = 0.05R, no spring)\nvs Johnson 2013 Eqs. 6.277–6.279",
    }
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), sharey=True)
    for ax, (rotor, title) in zip(axes, titles.items()):
        m = d["rotor"] == rotor
        mu = d["mu"][m]
        for i, (k, label) in enumerate([("b0", "β₀"), ("b1c", "β₁c"), ("b1s", "β₁s")]):
            c = f"C{i}"
            ax.plot(mu, d[k][m] * DEG, "-o", color=c, ms=3, label=f"{label} Eyas")
            ax.plot(mu, d["cf_" + k][m] * DEG, "--", color=c, lw=1.2, label=f"{label} closed form")
        ax.axvspan(0.1, 0.2, color="0.9", zorder=0)
        ax.axvspan(0.2, 0.35, color="0.82", zorder=0)
        ax.text(0.003, 0.2, "gate 1.5% of tilt", fontsize=7)
        ax.text(0.103, 0.2, "gate 3% at μ = 0.2", fontsize=7)
        ax.text(0.21, 0.2, "reported only", fontsize=7)
        ax.set(xlabel="advance ratio μ", title=title)
        ax.grid(True, lw=0.3)
    axes[0].set_ylabel("flapping [deg]")
    axes[1].legend(fontsize=7, ncol=2, loc="upper right")
    fig.suptitle("Flapping against advance ratio: θ₀ = 8°, θ_tw = −8°, θ₁s = −2°, θ₁c = 1°, γ ≈ 8",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "m4_flapping_mu.png", dpi=120)


def coning(src, out):
    d = load(src / "coning.csv")
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.3))
    a1.plot(d["ct"], d["b0"] * DEG, "-o", ms=4, color="C0", label="Eyas")
    a1.plot(d["ct"], d["cf_b0"] * DEG, "--", color="C0", lw=1.2,
            label="Johnson 2013 Eq. 6.277 + weight (Eq. 6.208)")
    a1.set(xlabel="C_T", ylabel="coning β₀ [deg]",
           title="Hover coning against thrust (offset hinge e = 0.05R)")
    a1.legend(fontsize=8)
    a1.grid(True, lw=0.3)
    err = (d["b0"] - d["cf_b0"]) * DEG
    a2.plot(d["theta0_deg"], err, "-o", ms=4, color="C0")
    a2.set(xlabel="θ₀ at hub centre [deg]", ylabel="Eyas − closed form [deg]",
           title="Residual (small-angle approximations of the closed form)")
    a2.axhline(0.0, color="0.5", lw=0.8)
    a2.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m4_coning.png", dpi=120)


def cyclic_step(src, out):
    d = load(src / "cyclic_step.csv")
    t = d["t"]
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.3))
    a1.plot(t, d["b1c"] * DEG, color="C1", label="β₁c Eyas")
    a1.plot(t, d["b1s"] * DEG, color="C2", label="β₁s Eyas")
    a1.axhline(d["cf_b1c"][0] * DEG, color="C1", ls="--", lw=1.2, label="β₁c steady, Johnson")
    a1.axhline(d["cf_b1s"][0] * DEG, color="C2", ls="--", lw=1.2, label="β₁s steady, Johnson")
    a1.set(xlabel="time after step [s]", ylabel="disc tilt [deg]",
           title="Hover, θ₁s step 0 → −3° (offset hinge e = 0.05R)")
    a1.legend(fontsize=8)
    a1.grid(True, lw=0.3)
    lag = np.degrees(np.arctan2(d["b1s"], d["b1c"])) + 90.0
    m = np.hypot(d["b1c"], d["b1s"]) > 0.05 * np.hypot(d["cf_b1c"][0], d["cf_b1s"][0])
    a2.plot(t[m], lag[m], color="C0", label="Eyas")
    a2.axhline(d["lag_deg"][0], color="C0", ls="--", lw=1.2,
               label=f"90° − atan(N*/d₁) = {d['lag_deg'][0]:.2f}°")
    a2.set(xlabel="time after step [s]", ylabel="phase lag behind pitch [deg]",
           title="Flap response phase lag (Johnson 2013 Eqs. 6.278–6.279)")
    a2.legend(fontsize=8)
    a2.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m4_cyclic_step.png", dpi=120)


def conservation(src, out):
    d = load(src / "conservation.csv")
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.loglog(d["step_ms"], d["dH"], "-o", color="C0", label="angular momentum about system CG")
    ax.loglog(d["step_ms"], d["dP"], "-s", color="C1", label="linear momentum − weight × t")
    h = np.array([0.25, 4.0])
    ax.loglog(h, d["dH"][-1] * (h / 0.25) ** 4, ":", color="gray", label="slope 4 (RK4)")
    ax.set(xlabel="physics step [ms]", ylabel="relative drift after 1 s",
           title="Tumbling airframe + flapping blades in vacuum\n(production executive and coupled solve)")
    ax.legend(fontsize=8)
    ax.grid(True, which="both", lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m4_conservation.png", dpi=120)


def main():
    src, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    flapping_mu(src, out)
    coning(src, out)
    cyclic_step(src, out)
    conservation(src, out)


if __name__ == "__main__":
    main()
