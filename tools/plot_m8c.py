"""Plots for the M8c entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m8c_cases executable (M8c1, M8c2 and
M8c3, D-045) and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m8c_cases build/m8c_cases
    .venv/bin/python tools/plot_m8c.py build/m8c_cases docs/validation/m8
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# Two variants per panel: blue solid and orange dashed (identity by style too);
# a third, where needed, grey dotted.
A = dict(color="C0", ls="-", marker="o", ms=4, lw=2)
B = dict(color="C1", ls="--", marker="s", ms=4, lw=2)
C = dict(color="0.45", ls=":", marker="^", ms=4, lw=2)


def load(path, skip=1):
    """A CSV with a header row, after `skip` comment lines."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8",
                         skip_header=skip)


def trims(data, out):
    d = load(data / "m8c_trims.csv")
    fig, ax = plt.subplots(2, 2, figsize=(9, 6.5), sharex=True)
    panels = (("stabilator_deg", "stabilator incidence [deg]"),
              ("pitch_deg", "pitch attitude [deg]"),
              ("lon_cyclic", "longitudinal cyclic [-1..1, fwd +]"),
              ("ay_sensor_mps2", "lateral accelerometer [m/s²]"))
    for a, (col, label) in zip(ax.flat, panels):
        for on, style, name in ((1, A, "lateral-acceleration feedback on (flying config)"),
                                (0, B, "feedback zeroed")):
            m = d["lateral_accel"] == on
            a.plot(d["knots"][m], d[col][m], label=name, **style)
        a.set_ylabel(label)
        a.grid(True, alpha=0.3)
    for a in ax[1]:
        a.set_xlabel("airspeed [kt]")
    ax[0, 0].legend(fontsize=8, loc="upper right")
    fig.suptitle("UH-60A level trims with the flight control system, ISA sea level (D-045)")
    fig.tight_layout()
    fig.savefig(out / "m8c_trims.png", dpi=110)
    plt.close(fig)


def modes(data, out):
    d = load(data / "m8c_modes.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 4), sharey=True)
    for a, kt in zip(ax, speeds):
        for kind, style, name in (("dynamic", A, "dynamic stabilator (D-045)"),
                                  ("held", B, "held at its trimmed incidence")):
            m = (d["knots"] == kt) & (d["stabilator"] == kind) & (np.hypot(d["re"], d["im"]) < 3)
            a.plot(d["re"][m], d["im"][m], ls="none", marker=style["marker"], ms=7,
                   color=style["color"], mfc="none" if kind == "held" else style["color"],
                   label=name)
        a.axvline(0.0, color="k", lw=0.8)
        a.axhline(0.0, color="k", lw=0.5, alpha=0.4)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("Re λ [1/s]")
        a.grid(True, alpha=0.3)
    ax[0].set_ylabel("Im λ [rad/s]")
    ax[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("UH-60A condensed 6-DOF modes, |λ| < 3 rad/s (right of the line: unstable)")
    fig.tight_layout()
    fig.savefig(out / "m8c_modes.png", dpi=110)
    plt.close(fig)


def start(data, out):
    d = load(data / "m8c_start.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 3.6), sharey=True)
    for a, kt in zip(ax, speeds):
        for kind, style, name in (("orbit", A, "sensors on their free-flight orbit (now)"),
                                  ("held", B, "sensors at held-body values (before)")):
            m = (d["knots"] == kt) & (d["start"] == kind)
            a.semilogy(d["time_s"][m], np.maximum(np.abs(d["stabilator_change_deg"][m]), 1e-7),
                       color=style["color"], ls=style["ls"], lw=2, label=name)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("time from trim [s]")
        a.set_ylim(1e-6, 0.2)
        a.grid(True, which="both", alpha=0.3)
    ax[0].set_ylabel("|stabilator change| [deg]")
    ax[0].legend(fontsize=8, loc="lower right")
    fig.suptitle("Free flight from the trim: the stabilator's start-up transient (D-045 amendment)")
    fig.tight_layout()
    fig.savefig(out / "m8c_start.png", dpi=110)
    plt.close(fig)


def kick(data, out):
    d = load(data / "m8c_kick.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 3.6), sharey=True)
    for a, kt in zip(ax, speeds):
        m = d["knots"] == kt
        a.plot(d["time_s"][m], d["stabilator_difference_deg"][m], color="C0", lw=2)
        a.axvspan(1.5, d["time_s"][m].max(), color="k", alpha=0.06)
        for y in (0.25, -0.25):
            a.axhline(y, color="k", ls="--", lw=1)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("time [s]")
        a.grid(True, alpha=0.3)
    ax[0].set_ylabel("kicked − unkicked stabilator [deg]")
    ax[0].text(1.55, 0.27, "gate after 1.5 s: ±0.25°", fontsize=8)
    fig.suptitle("A 0.5° stabilator-servo kick dies out (Trim.FcsKickDiesOut)")
    fig.tight_layout()
    fig.savefig(out / "m8c_kick.png", dpi=110)
    plt.close(fig)


def sas_trims(data, out):
    d = load(data / "m8c2_trims.csv")
    off = d[d["afcs"] == 0]
    on = d[d["afcs"] == 1]
    fig, ax = plt.subplots(1, 3, figsize=(12, 3.6), sharex=True)
    panels = (("lon_cyclic", "longitudinal cyclic [%, fwd +]"),
              ("lat_cyclic", "lateral cyclic [%, right +]"),
              ("pedal", "pedal [%, right +]"))
    for a, (col, label) in zip(ax, panels):
        a.plot(on["knots"], 100.0 * (on[col] - off[col]), **A)
        a.axhline(0.0, color="k", lw=0.8)
        a.set_ylabel(label)
        a.set_xlabel("airspeed [kt]")
        a.grid(True, alpha=0.3)
    fig.suptitle("Stick in trim, SAS and PBA on minus off (half travel = 100 %): "
                 "the aircraft's state is the same (Trim.SasAndPbaMoveOnlyTheStick)")
    fig.tight_layout()
    fig.savefig(out / "m8c2_trims.png", dpi=110)
    plt.close(fig)


def sas_modes(data, out):
    d = load(data / "m8c2_modes.csv")
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    variants = (("off_quasi_static", C, "SAS off, FCS quasi-static (as M8c1 reported)"),
                ("off", B, "SAS off, FCS dynamic"),
                ("on", A, "SAS and PBA on"))
    for a, label, title in ((ax[0], "Dutch roll", "Dutch roll"),
                            (ax[1], "phugoid", "phugoid")):
        for v, style, name in variants:
            # In forward flight only: at 30 kt two lateral oscillations both
            # take the heuristic's Dutch-roll label.
            m = (d["variant"] == v) & (d["label"] == label) & (d["knots"] >= 90)
            a.plot(d["knots"][m], d["zeta"][m], label=name, **style)
        a.axhline(0.0, color="k", lw=0.8)
        a.set_title(title)
        a.set_xlabel("airspeed [kt]")
        a.set_ylabel("damping ratio ζ [-]")
        a.grid(True, alpha=0.3)
    ax[0].legend(fontsize=8, loc="upper right")
    fig.suptitle("UH-60A modes, 90-150 kt; flight control system states kept dynamic "
                 "unless marked (below 0: unstable)")
    fig.tight_layout()
    fig.savefig(out / "m8c2_modes.png", dpi=110)
    plt.close(fig)


def sas_kick(data, out):
    d = load(data / "m8c2_kick.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(12, 3.6), sharey=True)
    for a, kt in zip(ax, speeds):
        for on, style, name in ((0, B, "SAS and PBA off"), (1, A, "SAS and PBA on")):
            m = (d["knots"] == kt) & (d["afcs"] == on)
            a.plot(d["time_s"][m], d["yaw_rate_difference_deg_s"][m], color=style["color"],
                   ls=style["ls"], lw=2, label=name)
        a.axhline(0.0, color="k", lw=0.5, alpha=0.5)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("time [s]")
        a.grid(True, alpha=0.3)
    ax[0].set_ylabel("kicked − unkicked yaw rate [deg/s]")
    ax[0].legend(fontsize=8, loc="upper right")
    fig.suptitle("Dutch roll after a 0.05 rad/s yaw-rate kick (Trim.SasDampsTheDutchRoll)")
    fig.tight_layout()
    fig.savefig(out / "m8c2_kick.png", dpi=110)
    plt.close(fig)


def fps_trims(data, out):
    d = load(data / "m8c3_trims.csv")
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.6), sharex=True)
    ax[0].plot(d["knots"], 100.0 * d["turn_pedal_change"], **A)
    ax[0].set_ylabel("pedal, turn switch on minus off [%, right +]")
    ax[1].plot(d["knots"], d["turn_held_drift_pct_s"], **A)
    ax[1].set_ylabel("held drift rate of the yaw integrator [%/s]")
    for a in ax:
        a.axhline(0.0, color="k", lw=0.8)
        a.set_xlabel("airspeed [kt]")
        a.grid(True, alpha=0.3)
    worst = np.max(d["fps_largest_difference"])
    same = "bit for bit" if worst == 0.0 else f"to {worst:.0e}"
    fig.suptitle(f"Trims with the FPS on equal the SAS-only trims ({same}); "
                 "the turn switch moves only the pedal")
    fig.tight_layout()
    fig.savefig(out / "m8c3_trims.png", dpi=110)
    plt.close(fig)


def fps_modes(data, out):
    d = load(data / "m8c3_modes.csv")
    speeds = sorted(set(d["knots"]))
    fig, ax = plt.subplots(1, len(speeds), figsize=(13, 4), sharey=True)
    for a, kt in zip(ax, speeds):
        for v, style, name in (("sas", B, "SAS and PBA"), ("fps", A, "with the FPS")):
            m = ((d["knots"] == kt) & (d["variant"] == v) & (d["body"] >= 0.5)
                 & (np.hypot(d["re"], d["im"]) < 3))
            a.plot(d["re"][m], d["im"][m], ls="none", marker=style["marker"], ms=7,
                   color=style["color"], mfc="none" if v == "sas" else style["color"],
                   label=name)
        a.axvline(0.0, color="k", lw=0.8)
        a.axhline(0.0, color="k", lw=0.5, alpha=0.4)
        a.set_title(f"{kt:.0f} kt")
        a.set_xlabel("Re λ [1/s]")
        a.grid(True, alpha=0.3)
    ax[0].set_ylabel("Im λ [rad/s]")
    ax[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("UH-60A body modes (body share ≥ ½), |λ| < 3 rad/s; flight control states "
                 "dynamic, rotor quasi-static (right of the line: unstable)")
    fig.tight_layout()
    fig.savefig(out / "m8c3_modes.png", dpi=110)
    plt.close(fig)


def _case(d, name, variant):
    m = (d["case"] == name) & (d["variant"] == variant)
    return d[m]


def fps_flights(data, out):
    d = load(data / "m8c3_flights.csv")
    # Attitude kicks.
    fig, ax = plt.subplots(2, 4, figsize=(14, 6), sharex=True)
    for j, name in enumerate(("60kt_pitch_kick", "60kt_roll_kick", "120kt_pitch_kick",
                              "120kt_roll_kick")):
        for i, col in enumerate(("pitch_deg", "roll_deg")):
            a = ax[i, j]
            for v, style, label in (("sas", B, "SAS and PBA"), ("fps", A, "with the FPS")):
                c = _case(d, name, v)
                a.plot(c["time_s"], c[col], color=style["color"], ls=style["ls"], lw=1.6,
                       label=label)
            a.axhline(0.0, color="k", lw=0.5)
            a.grid(True, alpha=0.3)
            if j == 0:
                a.set_ylabel(("pitch" if i == 0 else "roll") + " change [deg]")
            if i == 0:
                a.set_title(name.replace("_", " ").replace("kt", " kt"))
            else:
                a.set_xlabel("time [s]")
    ax[0, 0].legend(fontsize=8)
    fig.suptitle("Attitude hold: hands-off after a pitch-rate (0.05 rad/s) or roll-rate "
                 "(0.1 rad/s) kick (Uh60aFps.HoldsTheAttitude)")
    fig.tight_layout()
    fig.savefig(out / "m8c3_attitude.png", dpi=110)
    plt.close(fig)

    # Airspeed: stick push above 30 kt, gust at 20 kt.
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    for a, name in zip(ax[:2], ("60kt_stick_push", "100kt_stick_push")):
        for v, style, label in (("sas", B, "SAS and PBA"), ("fps", A, "with the FPS")):
            c = _case(d, name, v)
            a.plot(c["time_s"], c["airspeed_kt"], color=style["color"], ls=style["ls"], lw=1.6,
                   label=label)
        a.set_title(name.replace("_", " ").replace("kt", " kt") + " (stick 0.01 forward)")
    for v, style, label in (("sas", B, "SAS and PBA (diverges)"), ("fps", A, "with the FPS")):
        c = _case(d, "20kt_gust", v)
        ax[2].plot(c["time_s"], c["airspeed_kt"], color=style["color"], ls=style["ls"], lw=1.6,
                   label=label)
    ax[2].set_title("20 kt, 5 kt gust (not held below 30 kt)")
    ax[2].set_ylim(-2, 20)
    for a in ax:
        a.axhline(0.0, color="k", lw=0.5)
        a.set_xlabel("time [s]")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    ax[0].set_ylabel("airspeed change [kt]")
    fig.suptitle("Airspeed hold above 30 kt (Uh60aFps.HoldsTheAirspeedAbove30Knots, "
                 "DoesNotHoldTheAirspeedBelow30Knots)")
    fig.tight_layout()
    fig.savefig(out / "m8c3_airspeed.png", dpi=110)
    plt.close(fig)

    # Yaw: hover pedal push, turn, collective step.
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
    for v, style, label in (("sas", C, "SAS and PBA"),
                            ("fps_physical", A, "FPS, physical wiring"),
                            ("fps_as_printed", B, "FPS, as printed")):
        c = _case(d, "hover_pedal_push", v)
        ax[0].plot(c["time_s"], c["heading_deg"], color=style["color"], ls=style["ls"], lw=1.6,
                   label=label)
    ax[0].set_ylim(-45, 45)
    ax[0].axhspan(-1, 1, color="k", alpha=0.08)
    ax[0].set_title("hover, pedal 0.05 right (band: ±1° deadband)")
    ax[0].set_ylabel("heading change [deg]")
    for v, style, label in (("off_physical", C, "turn switch off"),
                            ("on_physical", A, "on, physical wiring"),
                            ("on_as_printed", B, "on, as printed")):
        c = _case(d, "100kt_turn", v)
        ax[1].plot(c["time_s"], c["ay_mps2"], color=style["color"], ls=style["ls"], lw=1.6,
                   label=label)
    ax[1].set_ylim(-1.0, 0.4)
    ax[1].set_title("100 kt turn, ~10° bank (trim release to 8 s)")
    ax[1].set_ylabel("lateral accelerometer [m/s²]")
    for v, style, label in (("sas", C, "SAS and PBA"), ("fps", A, "FPS"),
                            ("fps_no_ksxcyo", B, "FPS, KSXCYO zeroed")):
        c = _case(d, "20kt_collective_step", v)
        ax[2].plot(c["time_s"], c["heading_deg"], color=style["color"], ls=style["ls"], lw=1.6,
                   label=label)
    ax[2].set_title("20 kt, collective +0.05")
    ax[2].set_ylabel("heading change [deg]")
    for a in ax:
        a.axhline(0.0, color="k", lw=0.5)
        a.set_xlabel("time [s]")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    fig.suptitle("Heading hold, turn coordination and the collective-to-yaw path (Uh60aFps)")
    fig.tight_layout()
    fig.savefig(out / "m8c3_yaw.png", dpi=110)
    plt.close(fig)


def afcs_long_run(data, out):
    d = load(data / "m8c3_afcs_30min.csv")
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6), sharex=True)
    for kt, style in ((0.0, B), (120.0, A)):
        c = d[d["speed_kt"] == kt]
        t = c["time_s"] / 60.0
        lab = f"{kt:.0f} kt"
        ax[0].plot(t, np.degrees(c["pitch_rad"] - c["pitch_rad"][0]), color=style["color"],
                   ls=style["ls"], lw=1.6, label=lab)
        ax[1].plot(t, np.degrees(c["roll_rad"] - c["roll_rad"][0]), color=style["color"],
                   ls=style["ls"], lw=1.6, label=lab)
        ax[2].plot(t, (c["airspeed_mps"] - c["airspeed_mps"][0]) / 0.514444, color=style["color"],
                   ls=style["ls"], lw=1.6, label=lab)
    for a, label in zip(ax, ("pitch change [deg]", "roll change [deg]",
                             "airspeed change [kt]")):
        a.set_ylabel(label)
        a.set_xlabel("time [min]")
        a.grid(True, alpha=0.3)
        a.legend(fontsize=8)
    fig.suptitle("30 min hands-off with the SAS, PBA and FPS on, from level trims at 60 Hz "
                 "(D-042 a); the band is the 4/rev vibration sampled once a second")
    fig.tight_layout()
    fig.savefig(out / "m8c3_afcs_30min.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    trims(data, out)
    modes(data, out)
    start(data, out)
    kick(data, out)
    sas_trims(data, out)
    sas_modes(data, out)
    sas_kick(data, out)
    fps_trims(data, out)
    fps_modes(data, out)
    fps_flights(data, out)
    afcs_long_run(data, out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
