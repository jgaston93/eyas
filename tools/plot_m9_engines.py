"""Plots for the M9 Stage B (engines) entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m9_engine_cases executable (D-053) and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m9_engine_cases build/m9_engine_cases
    .venv/bin/python tools/plot_m9_engines.py build/m9_engine_cases docs/validation/m9
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

GOVERNED = 27.0  # [rad/s] the UH-60A's governed rotor speed (GenHel OMEGA)
ALTITUDES = ((0.0, "sea level", "C0"), (5250.0, "5250 ft", "C1"))


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def plot_trims(data, outdir):
    fig, ax = plt.subplots(3, 1, figsize=(7.5, 9.0), sharex=True)
    for feet, label, color in ALTITUDES:
        r = [x for x in data if float(x["altitude_ft"]) == feet and x["converged"] == "1"]
        kt = [float(x["airspeed_kt"]) for x in r]
        # The revolution's mean speed, 2π/T, not the 4/rev ripple at t = 0.
        nr = [100.0 * 2.0 * math.pi / (float(x["period_s"]) * GOVERNED) for x in r]
        ax[0].plot(kt, nr, "o-", color=color, label=label)
        ax[1].plot(kt, [float(x["n_gg_pct"]) for x in r], "o-", color=color,
                   label=f"N_GG, {label}")
        ax[1].plot(kt, [float(x["n_gg_lds_pct"]) for x in r], "--", color=color,
                   label=f"spindle's N_GG,LDS, {label}")
        ax[2].plot(kt, [float(x["vss_v"]) for x in r], "o-", color=color, label=label)
    ax[0].set_ylabel("rotor speed [% of governed]")
    ax[0].set_title("UH-60A level trims on GenHel's T700s (D-053)")
    ax[1].set_ylabel("gas-generator speed [%]")
    ax[2].set_ylabel("ECU integrator V_SS [V]")
    ax[2].axhline(0.0, color="grey", lw=0.8)
    ax[2].text(2, 0.03, "A_U = 0 V: the ECU can only trim the spindle down", fontsize=8)
    ax[2].set_xlabel("airspeed [kt]")
    for a in ax:
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "m9_engine_trims.png", dpi=110)
    plt.close(fig)


def plot_step(data, outdir):
    fields = (("rotor_speed_pct", "rotor speed [%]"),
              ("engine_torque_ftlbf", "engines' torque at the rotor [ft·lbf]"),
              ("n_gg_pct", "gas-generator speed [%]"),
              ("yaw_rate_deg_s", "yaw rate [deg/s]"))
    fig, ax = plt.subplots(len(fields), 2, figsize=(11.0, 10.0), sharex=True)
    for col, (feet, label, _) in enumerate(ALTITUDES):
        for drive, name, style in (("free", "on the engines", "-"),
                                   ("ideal", "rotor speed held", "--")):
            r = [x for x in data if float(x["altitude_ft"]) == feet and x["drive"] == drive]
            t = [float(x["time_s"]) for x in r]
            for i, (key, ylabel) in enumerate(fields):
                if drive == "ideal" and key != "yaw_rate_deg_s":
                    continue  # nothing turns the engines on the held shaft
                ax[i, col].plot(t, [float(x[key]) for x in r], style, label=name)
                ax[i, col].set_ylabel(ylabel)
        ax[0, col].set_title(f"0.05 collective step from hover, {label}")
        ax[-1, col].set_xlabel("time [s]")
        for a in ax[:, col]:
            a.grid(True, alpha=0.3)
        ax[-1, col].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "m9_engine_step.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_m9_engines.py <data-dir> <output-dir>")
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    plot_trims(rows(data / "m9_engine_trims.csv"), out)
    plot_step(rows(data / "m9_engine_step.csv"), out)


if __name__ == "__main__":
    main()
