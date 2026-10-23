"""Plots for the M9 Stage B1' and B2a entries (Ballin's T700, D-054, D-055) in
docs/VALIDATION.md.

Reads the CSVs written by the eyas_m9_ballin_engine_cases executable and writes PNGs: the
UH-60A on Ballin's engines at Table B.1's conditions against Table B.1 (NASA TM-100991 p. 53),
a collective step from hover on Ballin's engines, on GenHel's and with the rotor speed held,
and one engine's power lever to idle and back at 80 kt.

    cmake --build --preset release
    ./build/release/tests/eyas_m9_ballin_engine_cases build/m9_ballin
    .venv/bin/python tools/plot_m9_ballin.py build/m9_ballin docs/validation/m9
"""

import csv
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

CASES = (("hover", "hover"), ("80kt", "80 kt"), ("80kt_descent", "80 kt, -1000 ft/min"))
QUANTITIES = (("required_torque_hub_ftlbf", "rotor torque (hub)"),
              ("engine_torque_shaft_ftlbf", "engine torque"),
              ("hp_per_engine", "power per engine"),
              ("fuel_per_engine_lbm_hr", "fuel per engine"),
              ("ng_rpm", "NG"), ("ps3_psia", "Ps3"), ("p41_psia", "P41"), ("t41_degr", "T41"),
              ("p45_psia", "P45"), ("t45_degr", "T45"))
DRIVES = (("ballin", "Ballin's T700 (D-054)", "-", "C3"),
          ("genhel", "GenHel's T700 (D-053)", "--", "C0"),
          ("held", "rotor speed held", ":", "0.3"))


def rows(path):
    with open(path) as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def plot_table_b1(data, outdir):
    fig, ax = plt.subplots(figsize=(10.0, 4.6))
    width = 0.27
    for k, (case, label) in enumerate(CASES):
        diffs = []
        for q, _ in QUANTITIES:
            (r,) = [x for x in data if x["case"] == case and x["quantity"] == q]
            diffs.append(100.0 * (float(r["eyas"]) / float(r["ballin"]) - 1.0))
        xs = [i + (k - 1) * width for i in range(len(QUANTITIES))]
        ax.bar(xs, diffs, width, label=label)
    ax.axhline(0.0, color="black", lw=0.8)
    ax.set_xticks(range(len(QUANTITIES)))
    ax.set_xticklabels([q[1] for q in QUANTITIES], rotation=30, ha="right")
    ax.set_ylabel("Eyas minus Table B.1 [%]")
    ax.set_title("UH-60A on Ballin's T700s at Table B.1's loading and conditions "
                 "(NASA TM-100991 p. 53)")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "m9_ballin_table_b1.png", dpi=110)
    plt.close(fig)


def plot_step(data, outdir):
    fields = (("rotor_speed_pct", "rotor speed [%]"),
              ("engine_torque_ftlbf", "engines' torque at the rotor [ft·lbf]"),
              ("n_g_pct", "gas-generator speed [%]"),
              ("yaw_rate_deg_s", "yaw rate [deg/s]"))
    fig, ax = plt.subplots(len(fields), 2, figsize=(11.0, 10.0), sharex=True)
    for col, (feet, label) in enumerate(((0.0, "sea level"), (5250.0, "5250 ft"))):
        for drive, name, style, color in DRIVES:
            r = [x for x in data if float(x["altitude_ft"]) == feet and x["drive"] == drive]
            t = [float(x["time_s"]) for x in r]
            for i, (key, ylabel) in enumerate(fields):
                if drive == "held" and key != "yaw_rate_deg_s" and key != "rotor_speed_pct":
                    continue  # no engine on the held shaft
                ax[i, col].plot(t, [float(x[key]) for x in r], style, color=color, label=name)
                ax[i, col].set_ylabel(ylabel)
        ax[0, col].set_title(f"0.05 collective step from hover, {label}")
        ax[-1, col].set_xlabel("time [s]")
        for a in ax[:, col]:
            a.grid(True, alpha=0.3)
        ax[0, col].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "m9_ballin_step.png", dpi=110)
    plt.close(fig)


def plot_lever(data, outdir):
    engines = (("1", "engine 1 (lever at fly)", "C0"), ("2", "engine 2 (lever cut)", "C3"))
    fig, ax = plt.subplots(4, 1, figsize=(9.0, 11.0), sharex=True)
    for e, name, color in engines:
        r = [x for x in data if x["engine"] == e]
        t = [float(x["time_s"]) for x in r]
        if e == "1":
            ax[0].plot(t, [float(x["rotor_speed_pct"]) for x in r], color="black")
        ax[1].plot(t, [float(x["torque_ftlbf"]) for x in r], color=color, label=name)
        ax[2].plot(t, [float(x["n_g_pct"]) for x in r], color=color, label=name)
        if e == "2":
            ax[3].plot(t, [float(x["hmusel"]) for x in r], color=color, lw=2.0,
                       label="selected, HMUSEL")
            ax[3].plot(t, [float(x["wfpac"]) for x in r], "--", color="C2",
                       label="acceleration limit, WFPAC (Fig. C30)")
            ax[3].plot(t, [float(x["wfidm"]) for x in r], ":", color="C1",
                       label="idle floor, WFIDM (Fig. C20)")
            ax[3].plot(t, [float(x["wfpdc"]) for x in r], ":", color="C4",
                       label="deceleration floor, WFPDC (Fig. C22)")
            ax[3].plot(t, [float(x["wfpdm"]) for x in r], "-.", color="0.4",
                       label="demand, WFPDM")
    ax[0].set_ylabel("rotor speed [%]")
    ax[1].set_ylabel("power-turbine torque [ft·lbf]")
    ax[2].set_ylabel("gas-generator speed [%]")
    ax[3].set_ylabel("engine 2's fuel ratio [lbm·in²/(lbf·hr)]")
    ax[3].set_ylim(-1.0, 5.0)
    ax[-1].set_xlabel("time [s]")
    ax[0].set_title("UH-60A on Ballin's T700s, 80 kt, SAS and FPS on: engine 2's lever to idle "
                    "at 2 s, to fly at 22 s")
    for a in ax:
        a.grid(True, alpha=0.3)
        a.axvspan(2.0, 22.0, color="0.92", zorder=0)
    for a in ax[1:]:
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "m9_ballin_lever.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_m9_ballin.py <data-dir> <output-dir>")
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    table = rows(data / "m9_ballin_table_b1.csv")
    plot_table_b1(table, out)
    plot_step(rows(data / "m9_ballin_step.csv"), out)
    plot_lever(rows(data / "m9_ballin_lever.csv"), out)
    for case, _ in CASES:
        for q, label in QUANTITIES:
            (r,) = [x for x in table if x["case"] == case and x["quantity"] == q]
            d = 100.0 * (float(r["eyas"]) / float(r["ballin"]) - 1.0)
            if not math.isnan(d):
                print(f"{case:13s} {label:20s} {d:+6.2f} %")


if __name__ == "__main__":
    main()
