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


def main():
    data = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    anchor(data, out)
    pedal_step(data, out)


if __name__ == "__main__":
    main()
