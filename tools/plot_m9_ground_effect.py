"""Plots for the M9 ground-effect entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m9_ground_effect_cases executable (D-050) and
writes PNGs, with USAAEFA's UH-60A hover fits and Johnson's empirical curves.

    cmake --build --preset release
    ./build/release/tests/eyas_m9_ground_effect_cases build/m9_ground_effect_cases
    .venv/bin/python tools/plot_m9_ground_effect.py build/m9_ground_effect_cases docs/validation/m9
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

MODELS = (("genhel", "Eyas, GenHel K_GE", "C0"),
          ("cheeseman_bennett", "Eyas, Cheeseman-Bennett", "C1"))

# UH-60A tethered hover, USAAEFA ADA180181 App. E (S/N 82-23748, normal utility
# configuration, winds under 3 kt). C_P from the engines' shaft power. The
# fits' coefficients as printed on each figure (solid lines); C_T ranges the
# data cover, read from the plots.
IGE_RANGE = (0.0050, 0.0105)  # Fig. 2, 2 ft wheel height
OGE_RANGE = (0.0050, 0.0091)  # Fig. 3, 100 ft wheel height


def usaaefa_ige(ct):
    """Fig. 2 (2 ft wheel height): C_P = A0 + A1 C_T^1.5 + A2 C_T^3."""
    return 8.06470e-05 + 7.55353e-01 * ct ** 1.5 + 3.96385e01 * ct ** 3


def usaaefa_oge(ct):
    """Fig. 3 (100 ft wheel height), two segments meeting at C_T = 0.0070."""
    low = 8.06470e-05 + 1.02707 * ct ** 1.5
    high = 2.57292e-04 + 4.23840e-01 * ct ** 1.5 + 5.15000e02 * ct ** 3
    return np.where(ct < 0.0070, low, high)


def johnson_fits(z, ct_sigma):
    """Johnson 2013 Eqs. 4.73-4.76, T/T_inf = f_g at constant power."""
    zd = z / 2.0
    return {
        "Law (4.73)": 1.0 / ((1.0991 - 0.1042 / zd) / (1.0 + ct_sigma * (0.2894 - 0.3913 / zd))),
        "Hayden (4.74)": (0.9926 + 0.03794 / zd ** 2) ** (2.0 / 3.0),
        "Zbrozek (4.75)": 0.9122 + 0.0544 / (z * np.sqrt(ct_sigma)),
        "Schmaus et al. (4.76)": (0.146 + 2.090 * z - 2.068 * z ** 2 + 0.932 * z ** 3
                                  - 0.157 * z ** 4) ** (-2.0 / 3.0),
    }


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")


def factor(data, out):
    """κ_g in hover against z/R, with Johnson's fits as κ_g = f_g^(-3/2)."""
    d = load(data / "m9_ge_factor.csv")
    z = d["z_over_r"]
    fig, ax = plt.subplots(figsize=(7.5, 5))
    ax.plot(z, d["genhel"], "C0-", lw=2.2, label="GenHel K_GE (Eyas genhel)")
    ax.plot(z, d["cheeseman_bennett"], "C1-", lw=2.2,
            label="Cheeseman-Bennett, f_g^(-3/2) (Eyas cheeseman_bennett)")
    zz = np.linspace(0.3, 2.0, 200)
    for k, (name, f) in enumerate(johnson_fits(zz, 0.08).items()):
        ax.plot(zz, f ** -1.5, color=f"C{k + 2}", ls="--", lw=1.4, label=name)
    ax.axvline(14.0 / 26.83, color="k", lw=0.8, ls=":")
    ax.text(14.0 / 26.83 + 0.02, 0.42, "USAAEFA 2 ft\nwheel height", fontsize=8)
    ax.set_xlabel("hub height z/R")
    ax.set_ylabel("induced inflow ratio κ_g = v/v∞ at constant thrust")
    ax.set_title("Ground effect in hover (Johnson 2013 Eqs. 4.71-4.76; C_T/σ = 0.08)")
    ax.set_xlim(0.3, 2.0)
    ax.set_ylim(0.3, 1.02)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="lower right")
    fig.tight_layout()
    fig.savefig(out / "m9_ge_factor.png", dpi=110)


def hover_ratios(d, model):
    """Eyas's P(2 ft)/P(100 ft) at each altitude: from 2.5 and 3 ft, extrapolated
    linearly in wheel height to USAAEFA's 2 ft (at 2 ft Eyas's tail wheel grazes
    the ground; VALIDATION.md M9). Rows with a gear load or unconverged dropped."""
    rows = {w: d[(d["model"] == model) & (d["wheel_ft"] == w)] for w in (2.5, 3.0, 100.0)}
    ok = np.ones(len(rows[100.0]), dtype=bool)
    for r in rows.values():
        ok &= (r["converged"] == 1) & (r["gear_load_n"] == 0.0)
    p25, p3, p100 = (rows[w]["power_w"] for w in (2.5, 3.0, 100.0))
    p2 = 2.0 * p25 - p3
    return rows[100.0]["cw"][ok], (p2 / p100)[ok], (p25 / p100)[ok]


def hover(data, out):
    """C_P against C_W near the ground and at 100 ft wheel height, and their ratio."""
    d = load(data / "m9_ge_hover.csv")
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    ct = np.linspace(0.0050, 0.0105, 100)
    for wheel, rng, fit, ls in ((2.0, IGE_RANGE, usaaefa_ige, "-"),
                                (100.0, OGE_RANGE, usaaefa_oge, "--")):
        c = ct[(ct >= rng[0]) & (ct <= rng[1])]
        ax[0].plot(c * 1e4, fit(c) * 1e5, "k", ls=ls, lw=2.4,
                   label=f"USAAEFA fit, {wheel:g} ft (engine power)")
    for wheel, ls in ((2.5, "-"), (100.0, "--")):
        for model, label, color in MODELS:
            m = (d["model"] == model) & (d["wheel_ft"] == wheel) & (d["converged"] == 1)
            ax[0].plot(d["cw"][m] * 1e4, d["cp"][m] * 1e5, color=color, ls=ls, marker="o", ms=3,
                       label=f"{label}, {wheel:g} ft (main rotor)")
    ax[0].set_xlabel("C_W x 10^4")
    ax[0].set_ylabel("C_P x 10^5")
    ax[0].set_title("UH-60A hover power (USAAEFA ADA180181 App. E Figs. 2-3)")
    ax[0].grid(True, alpha=0.3)
    ax[0].legend(fontsize=7)

    c = ct[(ct >= OGE_RANGE[0]) & (ct <= OGE_RANGE[1])]
    ax[1].plot(c * 1e4, usaaefa_ige(c) / usaaefa_oge(c), "k-", lw=2.4,
               label="USAAEFA fits, 2 ft / 100 ft")
    for model, label, color in MODELS:
        cw, r2, r25 = hover_ratios(d, model)
        ax[1].plot(cw * 1e4, r2, color=color, marker="o", ms=3,
                   label=f"{label}, 2 ft (extrapolated)")
        ax[1].plot(cw * 1e4, r25, color=color, ls=":", lw=1.2, label=f"{label}, 2.5 ft")
    ax[1].set_xlabel("C_W x 10^4")
    ax[1].set_ylabel("P(2 ft) / P(100 ft) at the same weight")
    ax[1].set_title("Hover power in ground effect, hub z/R = 0.52")
    ax[1].grid(True, alpha=0.3)
    ax[1].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m9_ge_hover.png", dpi=110)


def height(data, out):
    """Main-rotor power against hub height, sea level, over the power with no ground."""
    d = load(data / "m9_ge_height.csv")
    fig, ax = plt.subplots(figsize=(7.5, 5))
    p_free = d["power_w"][(d["model"] == "none") & (d["wheel_ft"] == 100.0)][0]
    for model, label, color in MODELS:
        m = (d["model"] == model) & (d["converged"] == 1) & (d["gear_load_n"] == 0.0)
        ax.plot(d["hub_z_over_r"][m], d["power_w"][m] / p_free, color=color, marker="o", ms=3,
                label=label)
    ax.axvline(14.0 / 26.83, color="k", lw=0.8, ls=":")
    ax.set_xscale("log")
    ax.set_xlabel("hub height z/R")
    ax.set_ylabel("main-rotor power / power with no ground")
    ax.set_title("UH-60A hover, ISA sea level, 16 638 lbm")
    ax.grid(True, alpha=0.3, which="both")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "m9_ge_height.png", dpi=110)


def speed(data, out):
    """Power required against airspeed near the ground (cf. Johnson 2013 Fig. 5.12)."""
    d = load(data / "m9_ge_speed.csv")
    fig, ax = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for j, (model, label, _) in enumerate(MODELS):
        for k, target in enumerate((0.55, 0.65, 1.0, np.nan)):
            if np.isnan(target):
                m = (d["model"] == model) & np.isnan(d["hub_z_over_r_target"])
                name = "no ground"
            else:
                m = (d["model"] == model) & np.isclose(d["hub_z_over_r_target"], target)
                name = f"z/R = {target:g}"
            m &= (d["converged"] == 1) & (d["gear_load_n"] == 0.0)
            ax[j].plot(d["airspeed_kt"][m], d["power_w"][m] / 1000.0, color=f"C{k}", marker="o",
                       ms=3, label=name)
        ax[j].set_title(label)
        ax[j].set_xlabel("airspeed [kt]")
        ax[j].grid(True, alpha=0.3)
        ax[j].legend(fontsize=8)
    ax[0].set_ylabel("main-rotor power [kW]")
    fig.suptitle("UH-60A level flight near the ground, ISA sea level (cf. Johnson 2013 Fig. 5.12)")
    fig.tight_layout()
    fig.savefig(out / "m9_ge_speed.png", dpi=110)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    factor(data, out)
    hover(data, out)
    height(data, out)
    speed(data, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
