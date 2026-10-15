"""Plots for the M7a entry in docs/VALIDATION.md.

Reads the CSVs written by the eyas_m7_cases executable and writes PNGs.

    cmake --build --preset release
    ./build/release/tests/eyas_m7_cases build/m7_cases
    .venv/bin/python tools/plot_m7.py build/m7_cases docs/validation/m7
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    """A CSV whose first line is a # comment and second the column names."""
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8",
                         skip_header=1)


def trim(data, out):
    d = load(data / "m7_trim.csv")
    v = d["knots"]
    fig, ax = plt.subplots(3, 2, figsize=(9.0, 9.0), sharex=True)
    a = ax[0, 0]
    a.plot(v, 100.0 * d["collective"], "o-", ms=3, label="collective (0 = full down)")
    a.plot(v, 50.0 * (1.0 + d["lon_cyclic"]), "o-", ms=3, label="longitudinal (100 = full forward)")
    a.plot(v, 50.0 * (1.0 + d["lat_cyclic"]), "o-", ms=3, label="lateral (100 = full right)")
    a.plot(v, 50.0 * (1.0 + d["pedal"]), "o-", ms=3, label="pedal (100 = full right)")
    a.set_ylabel("control position [% of travel]")
    a.set_ylim(0, 100)
    a.legend(fontsize=7)
    a = ax[0, 1]
    a.plot(v, d["pitch_deg"], "o-", ms=3, label="pitch θ (+ nose up)")
    a.plot(v, d["roll_deg"], "o-", ms=3, label="roll φ (+ right wing down)")
    a.axhline(0.0, color="0.7", lw=0.6)
    a.set_ylabel("attitude [deg]")
    a.legend(fontsize=7)
    a = ax[1, 0]
    a.plot(v, d["theta0_deg"], "o-", ms=3, label="θ0 at the cuff")
    a.plot(v, d["theta1s_deg"], "o-", ms=3, label="θ1s")
    a.plot(v, d["theta1c_deg"], "o-", ms=3, label="θ1c")
    a.axhline(0.0, color="0.7", lw=0.6)
    a.set_ylabel("main-rotor blade pitch [deg]")
    a.legend(fontsize=7)
    a = ax[1, 1]
    a.plot(v, d["stabilator_deg"], "o-", ms=3, color="C4")
    a.set_ylabel("stabilator [deg, + trailing edge down]")
    a = ax[2, 0]
    a.plot(v, d["mr_power_hp"], "o-", ms=3, color="C0")
    a.set_ylabel("main-rotor power [hp]")
    a.set_xlabel("true airspeed [kt]")
    a = ax[2, 1]
    a.plot(v, d["tr_thrust_lbf"], "o-", ms=3, color="C1")
    a.set_ylabel("tail-rotor thrust [lbf]")
    a.set_xlabel("true airspeed [kt]")
    for a in ax.flat:
        a.grid(True, color="0.9", lw=0.6)
    fig.suptitle("UH-60A level-flight trim, ISA sea level, 16 638 lbm (Eyas, M7a)")
    fig.tight_layout()
    fig.savefig(out / "m7a_trim.png", dpi=110)
    plt.close(fig)


def drift(data, out):
    d = load(data / "m7_drift.csv")
    v = d["knots"]
    fig, ax = plt.subplots(1, 2, figsize=(9.0, 3.6), sharex=True)
    for a, key, unit, bound_lo, bound_hi in (
        (ax[0], "vdot", "m/s²", 5e-5, 2.5e-3),
        (ax[1], "wdot", "rad/s²", 2e-5, 1e-3),
    ):
        a.semilogy(v, d[f"model_{key}"], "o-", ms=4, label="Model::advance, 1 ms")
        a.semilogy(v, d[f"exact_{key}"], "s--", ms=4, label="RK4 at T/256")
        a.step([0, 60, 60, 150], [bound_lo, bound_lo, bound_hi, bound_hi], where="post",
               color="0.4", lw=1.0, label="test bound")
        a.set_ylabel(f"|mean acceleration| [{unit}]")
        a.set_xlabel("true airspeed [kt]")
        a.grid(True, color="0.9", lw=0.6, which="both")
    ax[0].set_title("linear", fontsize=9)
    ax[1].set_title("angular", fontsize=9)
    ax[0].legend(fontsize=7)
    fig.suptitle("Free flight from trim: body accelerations averaged over four revolutions")
    fig.tight_layout()
    fig.savefig(out / "m7a_drift.png", dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    data = Path(sys.argv[1])
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    trim(data, out)
    drift(data, out)


if __name__ == "__main__":
    main()
