"""Plots and difference tables for the M7c entry in docs/VALIDATION.md (D-035 V1).

Reads the CSVs written by eyas_m7c_cases and Ballin's digitized App. B plots
(data/uh60a/validation/ballin_b*.csv, human-reviewed, DATA_GAPS.md), and writes
one figure per Ballin figure (Eyas over GenHel and flight test; for B2-B4 the change from
level flight) and m7c_differences.csv: Eyas minus Sikorsky GenHel at the curve samples, and
Eyas minus flight test at the test points, per quantity (mean, RMS, largest).

    cmake --build --preset release
    ./build/release/tests/eyas_m7c_cases build/m7c_cases
    .venv/bin/python tools/plot_m7c.py build/m7c_cases docs/validation/m7
"""

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
BALLIN = ROOT / "data" / "uh60a" / "validation"

# Ballin quantity -> (Eyas column, axis label)
QTY = {
    "longitudinal_cyclic": ("lon_cyclic_pct", "longitudinal cyclic [%] (0 = full fwd)"),
    "lateral_cyclic": ("lat_cyclic_pct", "lateral cyclic [%] (100 = full right)"),
    "collective": ("collective_pct", "collective [%] (100 = full up)"),
    "pitch_attitude": ("pitch_deg", "pitch attitude [deg] (+ nose up)"),
    "roll_angle": ("roll_deg", "roll angle [deg] (+ right)"),
    "sideslip": ("sideslip_deg", "sideslip [deg] (+ right)"),
    "pedal": ("pedal_pct", "pedal [%] (100 = full right)"),
    "stabilator_incidence": ("stabilator_deg", "stabilator [deg] (+ TE down)"),
    "main_rotor_power": ("mr_power_hp", "main-rotor power [hp]"),
}


def load_eyas(path):
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(line for line in f if not line.startswith("#")))
    for r in rows:
        for k, v in r.items():
            if k != "constraint":
                r[k] = float(v)
    return rows


def load_ballin(fig):
    with (BALLIN / f"ballin_{fig}.csv").open(encoding="utf-8") as f:
        rows = list(csv.DictReader(line for line in f if not line.startswith("#")))
    for r in rows:
        r["x"] = float(r["x"])
        r["y"] = float(r["y"])
    return rows


def series(rows, quantity, name):
    pts = sorted((r["x"], r["y"]) for r in rows
                 if r["quantity"] == quantity and r["series"] == name)
    return np.array(pts).reshape(-1, 2)


def diffs(x_e, y_e, ref, label, quantity, out, x_range):
    """Eyas (interpolated in x) minus a reference, inside the Eyas x range."""
    if not len(ref):
        return
    keep = (ref[:, 0] >= x_range[0]) & (ref[:, 0] <= x_range[1])
    ref = ref[keep]
    if not len(ref):
        return
    d = np.interp(ref[:, 0], x_e, y_e) - ref[:, 1]
    out.append(dict(quantity=quantity, against=label, n=len(d), mean=d.mean(),
                    rms=np.sqrt((d ** 2).mean()), largest=d[np.argmax(np.abs(d))],
                    at=ref[np.argmax(np.abs(d)), 0]))


def level(data, outdir, table):
    eyas = load_eyas(data / "m7c_level.csv")
    gen = [r for r in eyas if r["constraint"] == "genhel"]
    beta0 = [r for r in eyas if r["constraint"] == "zero_sideslip"]
    b1 = load_ballin("b1")
    fig, axs = plt.subplots(3, 3, figsize=(13.0, 10.0), sharex=True)
    for ax, (q, (col, label)) in zip(axs.flat, QTY.items()):
        v = np.array([r["knots"] for r in gen])
        y = np.array([r[col] for r in gen])
        for name, style, lab in (("sikorsky_genhel", "k-", "GenHel (Sikorsky)"),
                                 ("ames_genhel", "k--", "GenHel (Ames)")):
            s = series(b1, q, name)
            if len(s):
                ax.plot(s[:, 0], s[:, 1], style, lw=1.0, label=lab)
        t = series(b1, q, "aefa_test")
        if len(t):
            ax.plot(t[:, 0], t[:, 1], "o", mfc="none", mec="k", ms=6, label="flight test")
        ax.plot(v, y, "o-", color="tab:red", ms=3, lw=1.5, label="Eyas")
        if q in ("roll_angle", "sideslip", "pedal", "lateral_cyclic"):
            ax.plot([r["knots"] for r in beta0], [r[col] for r in beta0], ":",
                    color="tab:red", lw=1.0, label="Eyas, β = 0")
        ax.set_ylabel(label, fontsize=8)
        ax.grid(alpha=0.3)
        if q == "longitudinal_cyclic":
            ax.invert_yaxis()  # as Ballin plots it: forward up
        diffs(v, y, series(b1, q, "sikorsky_genhel"), "GenHel", q, table, (v.min(), v.max()))
        diffs(v, y, t, "flight test", q, table, (v.min(), v.max()))
    for ax in axs[-1]:
        ax.set_xlabel("true airspeed [kt]")
    axs[0, 0].legend(fontsize=7)
    axs[1, 2].legend(fontsize=7)
    fig.suptitle("Level-flight trim against Ballin Fig. B1 (16 000 lbf, 5250 ft; Eyas under "
                 "GenHel's lateral constraint, zero roll from 50 kt)", fontsize=10)
    fig.tight_layout()
    fig.savefig(outdir / "m7c_level.png", dpi=110)
    plt.close(fig)


def from_level(xy):
    """A climb series as the change from its own level-flight value (interpolated at 0)."""
    if not len(xy):
        return xy
    return np.column_stack([xy[:, 0], xy[:, 1] - np.interp(0.0, xy[:, 0], xy[:, 1])])


def climbs(data, outdir, table):
    """Change from level flight against climb rate (B2-B4 give no weight, CG or density
    altitude, so only the change is compared). Eyas at B1's conditions, with the stabilator held
    at its level-flight incidence as Sikorsky's GenHel ran (Ballin p. 10) and, dotted,
    scheduled; the differences are for the held stabilator."""
    eyas = load_eyas(data / "m7c_climb.csv")
    qs = ["collective", "main_rotor_power", "pedal", "pitch_attitude", "lateral_cyclic",
          "longitudinal_cyclic"]
    for fig_id, kt in (("b2", 60.0), ("b3", 100.0), ("b4", 137.5)):
        b = load_ballin(fig_id)
        fig, axs = plt.subplots(3, 2, figsize=(10.0, 10.0), sharex=True)
        for ax, q in zip(axs.flat, qs):
            col, label = QTY[q]
            scale = 0.01 if q == "main_rotor_power" else 1.0  # Ballin plots hp/100
            for name, style, lab in (("sikorsky_genhel", "k-", "GenHel (Sikorsky)"),
                                     ("ames_genhel", "k--", "GenHel (Ames)")):
                s = series(b, q, name)
                if len(s):
                    s = from_level(np.column_stack([s[:, 0], s[:, 1] / scale]))
                    ax.plot(s[:, 0], s[:, 1], style, lw=1.0, label=lab)
            t = series(b, q, "aefa_test")
            if len(t):
                t = from_level(np.column_stack([t[:, 0], t[:, 1] / scale]))
                ax.plot(t[:, 0], t[:, 1], "o", mfc="none", mec="k", ms=6,
                        label="flight test")
            for constraint, style, lab in (("fixed_stabilator", "o-", "Eyas, stabilator held"),
                                           ("genhel", ":", "Eyas, stabilator scheduled")):
                # Converged trims only: a point with no trim (past full
                # collective, or an autorotation GenHel's engines can't trim)
                # is the solver's last iterate, not a flight condition.
                rows = sorted((r for r in eyas if r["knots"] == kt
                               and r["constraint"] == constraint and r["converged"]),
                              key=lambda r: r["rate_of_climb_fpm"])
                e = from_level(np.array([[r["rate_of_climb_fpm"] / 1000.0, r[col]]
                                         for r in rows]))
                ax.plot(e[:, 0], e[:, 1], style, color="tab:red", ms=3,
                        lw=1.5 if constraint == "fixed_stabilator" else 1.0, label=lab)
                if constraint == "fixed_stabilator":
                    rng = (e[:, 0].min(), e[:, 0].max())
                    diffs(e[:, 0], e[:, 1], from_level(series(b, q, "sikorsky_genhel") *
                                                       [1.0, 1.0 / scale]),
                          f"GenHel {kt:g} kt", q, table, rng)
                    diffs(e[:, 0], e[:, 1], t, f"flight test {kt:g} kt", q, table, rng)
            ax.set_ylabel(f"change in {label}", fontsize=8)
            ax.grid(alpha=0.3)
            if q == "longitudinal_cyclic":
                ax.invert_yaxis()
        for ax in axs[-1]:
            ax.set_xlabel("rate of climb [1000 ft/min]")
        axs[0, 0].legend(fontsize=7)
        fig.suptitle(f"Change from level flight at {kt:g} kt against Ballin Fig. "
                     f"{fig_id.upper()}", fontsize=10)
        fig.tight_layout()
        fig.savefig(outdir / f"m7c_climb_{kt:g}kt.png", dpi=110)
        plt.close(fig)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    data, outdir = Path(sys.argv[1]), Path(sys.argv[2])
    outdir.mkdir(parents=True, exist_ok=True)
    table = []
    level(data, outdir, table)
    climbs(data, outdir, table)
    with (outdir / "m7c_differences.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(table[0]))
        w.writeheader()
        for r in table:
            w.writerow({k: (f"{v:.3f}" if isinstance(v, float) else v) for k, v in r.items()})
    print(f"{'quantity':22s} {'against':22s} {'n':>3s} {'mean':>8s} {'rms':>8s} "
          f"{'largest':>8s} {'at':>7s}")
    for r in table:
        print(f"{r['quantity']:22s} {r['against']:22s} {r['n']:3d} {r['mean']:8.2f} "
              f"{r['rms']:8.2f} {r['largest']:8.2f} {r['at']:7.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
