"""Plots for the M6 entries in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m6_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m6_cases build/m6_cases
    .venv/bin/python tools/plot_m6.py build/m6_cases docs/validation/m6
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def anchor(data, out):
    mu = load(data / "m6a_anchor_mu.csv")
    lam = load(data / "m6a_anchor_lambda.csv")
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))

    ax = axes[0]
    ax.plot(mu["mu"], mu["ct_blade_element"], "k-", label="blade element (production M3 rotor)")
    ax.plot(mu["mu"], mu["ct_bailey"], "C0--", label="Bailey (production tail rotor)")
    ax.plot(mu["mu"], mu["ct_small_angle"], "C1:", label="small-angle integral, full reverse flow")
    ax.set_xlabel("advance ratio μ")
    ax.set_ylabel("C_T")
    ax.set_title("UH-60A tail rotor, θ₀ = 20°, λ_i = 0.05")
    ax.legend(fontsize=8)

    ax = axes[1]
    be = mu["ct_blade_element"]
    ax.plot(mu["mu"], 100 * (mu["ct_bailey"] / be - 1), "C0-", label="Bailey / blade element − 1")
    ax.plot(mu["mu"], 100 * (mu["ct_small_angle"] / be - 1), "C1-",
            label="small-angle integral / blade element − 1")
    ax.axhline(0, color="0.6", lw=0.8)
    ax.set_xlabel("advance ratio μ")
    ax.set_ylabel("difference [%]")
    ax.set_title("Bailey drops the O(μ³) reverse-flow loss")
    ax.legend(fontsize=8)

    ax = axes[2]
    ax.plot(lam["lambda_i"], 100 * (lam["ct_bailey"] - lam["ct_blade_element"]) / lam["ct_blade_element"][0],
            "C0-o", ms=3)
    ax.axhline(0, color="0.6", lw=0.8)
    ax.set_xlabel("induced inflow λ_i (hover)")
    ax.set_ylabel("(Bailey − blade element) / C_T(λ_i = 0.01) [%]")
    ax.set_title("Hover: small-angle error, grows with λ")

    fig.tight_layout()
    fig.savefig(out / "m6a_bailey_anchor.png", dpi=110)
    plt.close(fig)


def pedal_step(data, out):
    d = load(data / "m6a_pedal_step.csv")
    fig, ax = plt.subplots(figsize=(8, 4.5))
    styles = {
        "eyas": ("k-", "Eyas: inflow state (D-028), any frame rate"),
        "genhel_1khz": ("C0.-", "GenHel previous-pass scheme at 1 kHz"),
        "genhel_60hz": ("C3o-", "GenHel previous-pass scheme at 60 Hz"),
    }
    for model, (style, label) in styles.items():
        sel = d["model"] == model
        ax.plot(d["t"][sel] * 1000, d["thrust_lbf"][sel], style, ms=3, lw=1, label=label)
    ax.set_xlabel("time after the pedal step [ms]")
    ax.set_ylabel("tail-rotor thrust [lbf]")
    ax.set_title("UH-60A tail rotor in hover, commanded pitch 16° → 18°")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m6a_pedal_step.png", dpi=110)
    plt.close(fig)


def lag_vs_delay(data, out):
    d = load(data / "m6b_lag_vs_delay.csv")
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    speeds = [20, 40, 60, 100, 150]
    for i, kt in enumerate(speeds):
        sel = d["speed_kt"] == kt
        w = d["omega"][sel]
        c = f"C{i}"
        axes[0, 0].plot(w, d["wash_gain"][sel], c + "-", label=f"{kt} kt")
        axes[0, 0].plot(w, d["wash_gain_analytic"][sel], "k:", lw=0.8)
        axes[1, 0].plot(w, d["wash_phase_deg"][sel], c + "-")
        axes[1, 0].plot(w, d["wash_phase_deg_analytic"][sel], "k:", lw=0.8)
        axes[0, 1].plot(w, 100 * (d["lift_gain"][sel] - 1), c + "-", label=f"{kt} kt")
        axes[1, 1].plot(w, d["lift_phase_deg"][sel], c + "-")
    for ax in axes.flat:
        ax.set_xscale("log")
        ax.axvspan(1, 5, color="0.92", zorder=0)
        ax.grid(True, which="both", lw=0.3)
    axes[0, 0].set_title("delayed fuselage downwash at the stabilator: lag / exact delay")
    axes[0, 0].set_ylabel("gain ratio")
    axes[1, 0].set_ylabel("phase difference [deg]")
    axes[0, 1].set_title("resulting stabilator lift: lag / exact delay")
    axes[0, 1].set_ylabel("gain error [%]")
    axes[1, 1].set_ylabel("phase difference [deg]")
    for ax in axes[1]:
        ax.set_xlabel("heave frequency [rad/s] (shaded: piloted band)")
    axes[0, 0].legend(fontsize=8, title="dotted: e^{jωτ}/(1+jωτ)", title_fontsize=8)
    axes[0, 1].set_ylim(-5, 5)
    axes[1, 1].set_ylim(-5, 5)
    fig.suptitle("UH-60A stabilator, sinusoidal heave, collective 50 %, sea level")
    fig.tight_layout()
    fig.savefig(out / "m6b_lag_vs_delay.png", dpi=110)
    plt.close(fig)


def a1_sensitivity(data, out):
    d = load(data / "m6b_a1_derivatives.csv")
    lag = load(data / "m6b_a1_lag.csv")
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    ax = axes[0]
    ax.plot(d["speed_kt"], np.abs(d["ratio"]), "ko-")
    ax.set_xlabel("airspeed [kt]")
    ax.set_ylabel("|lift per deg a1| / |lift per deg angle of attack|")
    ax.set_title("how much the stabilator feels main-rotor flapping a1")
    ax.grid(True, lw=0.3)
    ax = axes[1]
    for i, tau in enumerate(np.unique(lag["tau"])):
        sel = lag["tau"] == tau
        ax.plot(lag["omega"][sel], -lag["phase_deg"][sel], f"C{i}o", label=f"τ = {tau:g} s (production path)")
        ax.plot(lag["omega"][sel], -lag["phase_deg_analytic"][sel], f"C{i}:", lw=1)
    ax.axhline(5, color="k", lw=0.8)
    ax.axvspan(1, 5, color="0.92", zorder=0)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("frequency [rad/s] (shaded: piloted band)")
    ax.set_ylabel("phase lag the a1 filter adds [deg]")
    ax.set_title("a lag on AA1FMR: 5° limit (line) holds for τ ≤ 17.5 ms")
    ax.legend(fontsize=8)
    ax.grid(True, which="both", lw=0.3)
    fig.tight_layout()
    fig.savefig(out / "m6b_a1_sensitivity.png", dpi=110)
    plt.close(fig)


def sideward_fade(data, out):
    st = load(data / "m6b_fade_steepness.csv")
    sw = load(data / "m6b_fade_sweep.csv")
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    ax = axes[0]
    fades = st["fade_deg"][1:].astype(float)
    ax.plot(fades, st["ratio_force"][1:].astype(float), "ko-", label="force")
    ax.plot(fades, st["ratio_moment"][1:].astype(float), "ks--", label="moment")
    ax.axhline(2, color="C3", lw=0.8)
    ax.set_xlabel("fade half-width c [deg]")
    ax.set_ylabel("steepest load change per deg, / same away from the line")
    ax.set_title("steepness within 20° of the line (red: 2×)")
    ax.legend(fontsize=8)
    ax.grid(True, lw=0.3)
    styles = {0.0: ":", 10.0: "--", 20.0: "-"}
    for ax, comp, name in ((axes[1], "L", "rolling"), (axes[2], "N", "yawing")):
        for i, eta in enumerate(np.unique(sw["eta_deg"])):
            for fade, ls in styles.items():
                sel = (sw["eta_deg"] == eta) & (sw["fade_deg"] == fade)
                label = f"{eta:g}° from the line" if fade == 20.0 else None
                ax.plot(sw["theta_deg"][sel], sw[comp][sel] / 1000.0, f"C{i}{ls}", lw=1, label=label)
        ax.set_xlabel("direction θ of the x–z flow component [deg] (0 forward, 90 down)")
        ax.set_ylabel(f"total {name} moment [kN·m]")
        ax.set_title(f"{name} moment (fade: dotted none, dashed 10°, solid 20°)", fontsize=10)
        ax.set_xticks(range(0, 361, 90))
        ax.grid(True, lw=0.3)
    axes[2].legend(fontsize=8)
    fig.suptitle("UH-60A airframe, 20 kt nearly sideward, no main-rotor wash, sea level (D-031)")
    fig.tight_layout()
    fig.savefig(out / "m6b_sideward_fade.png", dpi=110)
    plt.close(fig)


def main():
    data = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    anchor(data, out)
    pedal_step(data, out)
    lag_vs_delay(data, out)
    a1_sensitivity(data, out)
    sideward_fade(data, out)


if __name__ == "__main__":
    main()
