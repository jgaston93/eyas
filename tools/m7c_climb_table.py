"""The M7c climb table per drive, for VALIDATION.md M7c and M9 (Ballin Figs. B2-B4).

For each run directory written by eyas_m7c_cases: the climbs that trim, the RMS of
Eyas - Sikorsky GenHel as the change from level flight (over the Sikorsky readings inside
the climbs that trim; stabilator held), the differences at the range's ends, the held and
scheduled RMS of longitudinal cyclic and pitch, and the main-rotor power slope over
-2000 to +2000 ft/min (a least-squares line through the trims there).

    .venv/bin/python tools/m7c_climb_table.py <run-dir> [<run-dir> ...]
"""

import sys
from pathlib import Path

import numpy as np

import plot_m7c as P

QTYS = ["collective", "longitudinal_cyclic", "lateral_cyclic", "pedal", "pitch_attitude",
        "main_rotor_power"]


def converged(eyas, kt, constraint):
    """Converged trims at one speed and constraint, by rate of climb."""
    return sorted((r for r in eyas if r["knots"] == kt and r["constraint"] == constraint
                   and r["converged"]), key=lambda r: r["rate_of_climb_fpm"])


def drive(run):
    eyas = P.load_eyas(run / "m7c_climb.csv")
    print(f"== {run}{P.aircraft(run / 'm7c_climb.csv')}")
    for fig_id, kt in (("b2", 60.0), ("b3", 100.0), ("b4", 137.5)):
        b = P.load_ballin(fig_id)
        both = []
        for constraint in ("fixed_stabilator", "genhel"):
            rows = converged(eyas, kt, constraint)
            x = np.array([r["rate_of_climb_fpm"] / 1000.0 for r in rows])
            held = constraint == "fixed_stabilator"
            if held:
                print(f"{kt:6g} kt, held: trims {x.min():+.1f} to {x.max():+.1f} "
                      "[1000 ft/min]")
            for q in QTYS:
                col = P.QTY[q][0]
                scale = 0.01 if q == "main_rotor_power" else 1.0  # Ballin plots hp/100
                e = P.from_level(np.column_stack([x, [r[col] for r in rows]]))
                s = P.from_level(P.series(b, q, "sikorsky_genhel") * [1.0, 1.0 / scale])
                inside = s[(s[:, 0] >= x.min()) & (s[:, 0] <= x.max())]
                d = np.interp(inside[:, 0], e[:, 0], e[:, 1]) - inside[:, 1]
                rms = np.sqrt((d ** 2).mean())
                if held:
                    lo, hi = (np.interp(v, e[:, 0], e[:, 1]) - np.interp(v, s[:, 0], s[:, 1])
                              for v in (x.min(), x.max()))
                    print(f"   {q:22s} rms {rms:6.2f}   ends {lo:+6.2f} at {x.min():+.1f}, "
                          f"{hi:+6.2f} at {x.max():+.1f}")
                if q in ("longitudinal_cyclic", "pitch_attitude"):
                    both.append(f"{q} {'held' if held else 'scheduled'} {rms:.2f}")
                if q == "main_rotor_power" and held:
                    m = (x >= -2.0) & (x <= 2.0)
                    eyas_slope = np.polyfit(x[m], np.array([r[col] for r in rows])[m], 1)[0]
                    sg = P.series(b, q, "sikorsky_genhel")
                    ms = (sg[:, 0] >= -2.0) & (sg[:, 0] <= 2.0)
                    sik_slope = np.polyfit(sg[ms, 0], sg[ms, 1] / scale, 1)[0]
                    print(f"   power slope over {x[m].min():+.1f} to {x[m].max():+.1f}: Eyas "
                          f"{eyas_slope:.0f}, Sikorsky {sik_slope:.0f} hp per 1000 ft/min")
        print("   " + "; ".join(both))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    for run in sys.argv[1:]:
        drive(Path(run))
    return 0


if __name__ == "__main__":
    sys.exit(main())
