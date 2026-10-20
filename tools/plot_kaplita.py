"""Plot Eyas's steady-heading sideslips against Kaplita, NASA CR-177360, Figs. 11-13 (D-048).

Reads kaplita_sideslip.csv written by eyas_kaplita_cases and the digitized figures
(data/uh60a/validation/kaplita_sideslip.csv, flight test; kaplita_sideslip_genhel.csv,
Sikorsky's GenHel), and writes kaplita_sideslip.png: one row per airspeed, one column per
quantity, and kaplita_gradients.csv: each series' least-squares gradient over +-12 deg.

    cmake --build --preset release
    ./build/release/tests/eyas_kaplita_cases build/kaplita
    .venv/bin/python tools/plot_kaplita.py build/kaplita docs/validation/kaplita

Eyas flies a stand-in aircraft (Ballin's weight and CG at 5250 ft), so levels aren't compared:
every series is drawn as its change from zero sideslip (the flight test's from its own
least-squares line at zero). Flight-test readings flagged beta_open (placeholders) are left out.
"""

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
VALIDATION = ROOT / "data" / "uh60a" / "validation"
SPEEDS = (60, 100, 140)
# panel -> (Eyas column, label)
PANELS = {
    "lon": ("lon_cyclic_pct", "longitudinal stick [%]"),
    "pitch": ("pitch_deg", "pitch [deg]"),
    "lat": ("lat_cyclic_pct", "lateral stick [%]"),
    "roll": ("roll_deg", "roll [deg]"),
    "pedal": ("pedal_pct", "pedal [%]"),
}
GRADIENT_RANGE = 12.0  # [deg] as KaplitaSideslip.GradientsAgainstSikorskyGenHel


def rows(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def gradient(x, y):
    x, y = np.asarray(x), np.asarray(y)
    m = np.abs(x) <= GRADIENT_RANGE + 1e-9
    return np.polyfit(x[m], y[m], 1)[0] if m.sum() >= 3 else float("nan")


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    data, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    eyas = rows(data / "kaplita_sideslip.csv")
    genhel = rows(VALIDATION / "kaplita_sideslip_genhel.csv")
    test = rows(VALIDATION / "kaplita_sideslip.csv")

    fig, axes = plt.subplots(len(SPEEDS), len(PANELS), figsize=(17, 10), squeeze=False)
    grads = []
    for i, kt in enumerate(SPEEDS):
        for j, (panel, (col, label)) in enumerate(PANELS.items()):
            ax = axes[i][j]
            g = sorted((float(r["sideslip_deg"]), float(r["value"])) for r in genhel
                       if r["panel"] == panel and int(r["airspeed_kt"]) == kt)
            gx, gy = np.array(g).T
            g0 = np.interp(0.0, gx, gy)
            ax.plot(gx, gy - g0, "k-", lw=1.5, label="Sikorsky GenHel (Kaplita)")
            grads.append((kt, panel, "sikorsky_genhel", gradient(gx, gy)))
            t = [(float(r["sideslip_deg"]), float(r["value"])) for r in test
                 if r["panel"] == panel and int(float(r["airspeed_kt"])) == kt
                 and "beta_open" not in r["flag"]]
            if t:
                tx, ty = np.array(t).T
                fit = np.polyfit(tx, ty, 1)
                ax.plot(tx, ty - fit[1], "o", mfc="none", mec="tab:green", label="flight test")
                grads.append((kt, panel, "flight_test", gradient(tx, ty)))
            for variant, style, name in (("eyas", "r-", "Eyas (D-048)"),
                                         ("no_additions", "r:", "Eyas, no correction terms")):
                e = sorted((float(r["sideslip_deg"]), float(r[col])) for r in eyas
                           if r["variant"] == variant and int(float(r["knots"])) == kt)
                ex, ey = np.array(e).T
                ax.plot(ex, ey - np.interp(0.0, ex, ey), style, lw=1.3, label=name)
                grads.append((kt, panel, variant, gradient(ex, ey)))
            ax.axhline(0, color="0.8", lw=0.6)
            ax.axvline(0, color="0.8", lw=0.6)
            ax.grid(alpha=0.3)
            if i == 0:
                ax.set_title(label)
            if j == 0:
                ax.set_ylabel(f"{kt} kt\nchange from zero sideslip")
            if i == len(SPEEDS) - 1:
                ax.set_xlabel("sideslip [deg] (+ wind from the right)")
    axes[0][0].legend(fontsize=8, loc="best")
    fig.suptitle("Steady-heading sideslips: Eyas against Kaplita, NASA CR-177360 Figs. 11-13 "
                 "(SAS on, stabilator held)")
    fig.tight_layout()
    fig.savefig(out / "kaplita_sideslip.png", dpi=110)
    with open(out / "kaplita_gradients.csv", "w", encoding="utf-8") as f:
        f.write(f"# least-squares gradient per degree of sideslip over +-{GRADIENT_RANGE:g} deg\n")
        f.write("knots,panel,series,gradient\n")
        for kt, panel, series, s in grads:
            f.write(f"{kt},{panel},{series},{s:.4f}\n")
    print(f"wrote {out / 'kaplita_sideslip.png'} and kaplita_gradients.csv")


if __name__ == "__main__":
    main()
