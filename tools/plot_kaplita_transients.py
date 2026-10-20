"""Plot Eyas flown through Kaplita, NASA CR-177360 (1985), Fig. 4 against Sikorsky's GenHel and the
USAAEFA flight test, and tabulate the differences.

Reads kaplita_fig4.csv written by eyas_kaplita_transient_cases and the digitized, human-reviewed
traces (data/uh60a/validation/kaplita_transients/kaplita_fig4.csv), and writes
kaplita_fig4.png (six panels: yaw, roll and pitch rates and attitudes, sideslip) and
kaplita_fig4_differences.csv: per quantity, the RMS difference over WINDOW of Eyas (with and
without GenHel's downwash correction terms, D-048) from each, and of GenHel from the test.

    cmake --build --preset release
    ./build/release/tests/eyas_kaplita_transient_cases build/kaplita_transients
    .venv/bin/python tools/plot_kaplita_transients.py build/kaplita_transients docs/validation/kaplita

Attitudes are compared as changes from their values at the window's start. The CG waterline and
the density altitude are STAND-INS (DATA_GAPS.md); the thin lines are Eyas at the sensitivity
altitudes.
"""

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DIGITIZED = ROOT / "data" / "uh60a" / "validation" / "kaplita_transients" / "kaplita_fig4.csv"
STAND_IN_ALTITUDE = 5750.0  # [ft] as eyas_kaplita_transient_cases
T_END = 8.9  # [s] the traced lines end about 9 s; a few readings after that are stray ink
WINDOW = (1.5, 6.5)  # [s] the pedal step to where both models' large bank takes over
# (Kaplita quantity, Eyas column, label, attitude?)
PANELS = [
    ("yaw_rate", "r_dps", "yaw rate [deg/s]", False),
    ("sideslip", "sideslip_deg", "sideslip [deg]", False),
    ("roll_rate", "p_dps", "roll rate [deg/s]", False),
    ("roll_attitude", "roll_deg", "roll [deg]", True),
    ("pitch_rate", "q_dps", "pitch rate [deg/s]", False),
    ("pitch_attitude", "pitch_deg", "pitch [deg]", True),
]


def rows(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    eyas = rows(data / "kaplita_fig4.csv")
    kap = rows(DIGITIZED)

    def kaplita(q, series):
        d = sorted((float(r["t_s"]), float(r["value"])) for r in kap
                   if r["quantity"] == q and r["series"] == series and float(r["t_s"]) <= T_END)
        return np.array(d).T if d else None

    def run(variant, alt):
        r = [x for x in eyas if x["variant"] == variant and float(x["density_altitude_ft"]) == alt]
        return {c: np.array([float(x[c]) for x in r]) for c in r[0] if c != "variant"}

    runs = {("eyas", STAND_IN_ALTITUDE): run("eyas", STAND_IN_ALTITUDE),
            ("no_additions", STAND_IN_ALTITUDE): run("no_additions", STAND_IN_ALTITUDE)}
    others = sorted({float(x["density_altitude_ft"]) for x in eyas} - {STAND_IN_ALTITUDE})
    for alt in others:
        runs[("eyas", alt)] = run("eyas", alt)

    t = np.arange(WINDOW[0], WINDOW[1] + 1e-9, 0.025)
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    diffs = []
    for ax, (q, col, label, attitude) in zip(axes.ravel(), PANELS):
        ref = {}
        for series, style, name in (("aefa_test", "g--", "flight test"),
                                    ("sikorsky_genhel", "k-", "Sikorsky GenHel")):
            d = kaplita(q, series)
            if d is None:
                continue
            off = np.interp(WINDOW[0], *d) if attitude else 0.0
            ax.plot(d[0], d[1] - off, style, lw=1.6, label=name)
            ref[series] = np.interp(t, *d) - off
        for (variant, alt), r in runs.items():
            off = np.interp(WINDOW[0], r["t_s"], r[col]) if attitude else 0.0
            y = r[col] - off
            if alt != STAND_IN_ALTITUDE:
                ax.plot(r["t_s"], y, "r-", lw=0.6, alpha=0.4)
                continue
            style, name = (("r-", "Eyas (D-048)") if variant == "eyas"
                           else ("b:", "Eyas, no correction terms"))
            ax.plot(r["t_s"], y, style, lw=1.3, label=name)
            e = np.interp(t, r["t_s"], r[col]) - off
            for series, d in ref.items():
                diffs.append((q, variant, series, float(np.sqrt(np.mean((e - d) ** 2)))))
        if len(ref) == 2:
            diffs.append((q, "sikorsky_genhel", "aefa_test",
                          float(np.sqrt(np.mean((ref["sikorsky_genhel"] - ref["aefa_test"]) ** 2)))))
        ax.axvspan(*WINDOW, color="0.9", zorder=0)
        ax.set_title(label + (" (change from 1.5 s)" if attitude else ""))
        ax.grid(alpha=0.3)
        ax.set_xlim(0, 9)
        ax.set_xlabel("time [s]")
    axes[0][0].legend(fontsize=8)
    fig.suptitle("Kaplita Fig. 4: half-inch right pedal step, 144 kt, SAS off, stabilator held. "
                 "STAND-INS: CG waterline 231.8 in, density altitude 5750 ft "
                 f"(thin red: {', '.join(f'{a:g}' for a in others)} ft). Shaded: the compared window.",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "kaplita_fig4.png", dpi=90)
    with open(out / "kaplita_fig4_differences.csv", "w", encoding="utf-8") as f:
        f.write(f"# RMS difference over {WINDOW[0]:g}-{WINDOW[1]:g} s; attitudes as changes from "
                f"{WINDOW[0]:g} s. Eyas at the STAND-IN density altitude {STAND_IN_ALTITUDE:g} ft.\n")
        f.write("quantity,model,against,rms\n")
        for q, m, a, v in diffs:
            f.write(f"{q},{m},{a},{v:.2f}\n")
    print(f"wrote {out / 'kaplita_fig4.png'} and kaplita_fig4_differences.csv")


if __name__ == "__main__":
    main()
