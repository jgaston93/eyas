"""Plots and difference tables for Ballin's App. D and E transient responses (D-046).

Reads the CSVs written by eyas_ballin_app_d_cases and the digitized App. D and E plots
(data/uh60a/validation/ballin_d/), and writes one figure per Ballin figure (Eyas, with GenHel's
and with Ames's primary servos, over the Ames GenHel, the flight test and, in App. E, the
Sikorsky GenHel) and ballin_app_d_differences.csv: per figure, quantity and Eyas variant, the
RMS and largest difference from each Ballin series over the response window, the input's start
to 2 s after it, and the same for Ballin's simulations against the flight test (eyas column
naming the simulation) as a reference.

    cmake --build --preset release
    ./build/release/tests/eyas_ballin_app_d_cases build/ballin_app_d
    .venv/bin/python tools/plot_ballin_app_d.py build/ballin_app_d docs/validation/ballin_app_d

Attitudes and rates are compared as changes from their value at t = 0 (each simulation starts
in its own trim, and the flight test where it was when the input came); heading always so.
A case with a stand-in config variant (File 210's waterline, Fig. E2) is plotted with each
variant dotted, its differences under eyas "<servo>_<variant>". Series the reviewer did not
accept (flag rejected) are left out. Where the Ames simulation's 4/rev swing was read as a band (the vertical accelerations), its
edges are shaded and its middle compared; stretches not read (no rows) break the lines and
are left out of the differences.
"""

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DIGITIZED = ROOT / "data" / "uh60a" / "validation" / "ballin_d"
TABLE_C1 = ROOT / "data" / "uh60a" / "validation" / "ballin_table_c1.csv"

# Ballin quantity -> (Eyas column, axis label)
QTY = {
    "pitch_rate": ("q_dps", "pitch rate [deg/s]"),
    "roll_rate": ("p_dps", "roll rate [deg/s]"),
    "yaw_rate": ("r_dps", "yaw rate [deg/s]"),
    "pitch_attitude": ("pitch_deg", "Δ pitch [deg]"),
    "roll_attitude": ("roll_deg", "Δ roll [deg]"),
    "heading": ("heading_deg", "Δ heading [deg]"),
    "vertical_acceleration_cg_sensor": ("az_sensor_fps2", "vertical accel., CG sensor [ft/s²]"),
}
INPUTS = ["lateral_cyclic", "longitudinal_cyclic", "collective", "pedals"]
SERIES = {"ames_genhel": ("Ames GenHel", "0.45", "-"),
          "sikorsky_genhel": ("Sikorsky GenHel", "0.45", "-."),
          "aefa_test": ("flight test", "k", "--")}
EYAS = {"genhel_servo": ("Eyas, GenHel servos", "tab:blue"),
        "ames_servo": ("Eyas, Ames servos", "tab:orange")}
WINDOW = 2.0  # [s] after the input starts
# changes from t = 0: everything but the acceleration, which keeps its level (1 g)
ABSOLUTE = {"vertical_acceleration_cg_sensor"}


def captions():
    """Figure -> its input, from the digitized CSVs' source lines ("File 201: 1-in. right
    cyclic input at hover")."""
    out = {}
    for p in DIGITIZED.glob("ballin_*.csv"):
        head = p.read_text(encoding="utf-8").splitlines()[0]
        fig = p.stem.split("_")[1].upper()
        out[fig] = head.split(": ", 2)[-1].split(". Digitized")[0]
    return out


CAPTIONS = captions()


def read_csv(path):
    with open(path, encoding="utf-8") as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def digitized(fig):
    out = {}
    for r in read_csv(DIGITIZED / f"ballin_{fig.lower()}.csv"):
        if r["flag"] == "rejected":  # not accepted by the reviewer
            continue
        out.setdefault((r["quantity"], r["series"]), []).append((float(r["t_s"]), float(r["value"])))
    return {k: np.array(v) for k, v in out.items()}


def stepped_control(caption):
    """The control a figure steps, from its caption (Table C1 doesn't say)."""
    c = caption.lower()
    if "collective" in c:
        return "collective"
    if "pedal" in c:
        return "pedals"
    if "forward" in c or "aft" in c:
        return "longitudinal_cyclic"
    return "lateral_cyclic"  # "right/left cyclic", "right lateral"


def input_start(dig, control):
    """When the stepped control starts to move: 0.1 s before it first leaves its starting
    value by 0.15 in (the smallest step is 0.5 in; the drift before it stays below 0.1 in)."""
    a = dig[(control, "ames_genhel")]
    moved = np.abs(a[:, 1] - a[0, 1]) > 0.15
    return max(0.0, a[np.argmax(moved), 0] - 0.1)


STEP = 0.025  # [s] the digitized readings' grid


def at(series, t):
    """The digitized series at t, linearly between readings; NaN outside them and across a
    stretch not read (off the scale, hidden or unreadable: no rows there)."""
    y = np.interp(t, series[:, 0], series[:, 1], left=np.nan, right=np.nan)
    i = np.clip(np.searchsorted(series[:, 0], t), 1, len(series) - 1)
    y[np.diff(series[:, 0])[i - 1] > 1.5 * STEP] = np.nan
    return y


def broken(t, y):
    """t, y with a NaN in each stretch not read, so a plot line breaks there."""
    cut = np.flatnonzero(np.diff(t) > 1.5 * STEP) + 1
    return np.insert(t, cut, np.nan), np.insert(y, cut, np.nan)


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    cases = {r["figure"]: r for r in read_csv(TABLE_C1) if r["figure"] != "none"}
    diffs = []
    for fig, case in cases.items():
        runs = {k: src / f"ballin_{fig.lower()}_{k}.csv" for k in EYAS}
        if not all(p.exists() for p in runs.values()):
            continue
        # stand-in config variants (tools/build_uh60a_config.py), flown for comparison
        variants = sorted({p.stem.split(f"_{k}_", 1)[1] for k in EYAS
                           for p in src.glob(f"ballin_{fig.lower()}_{k}_*.csv")})
        styles = {k: (name, colour, "-") for k, (name, colour) in EYAS.items()}
        for v in variants:
            for k, (name, colour) in EYAS.items():
                runs[f"{k}_{v}"] = src / f"ballin_{fig.lower()}_{k}_{v}.csv"
                styles[f"{k}_{v}"] = (f"{name}, {v} (stand-in)", colour, ":")
        eyas = {k: read_csv(p) for k, p in runs.items()}
        dig = digitized(fig)
        control = stepped_control(CAPTIONS[fig])
        t_in = input_start(dig, control)
        qtys = [q for q in QTY if (q, "ames_genhel") in dig]
        n = len(qtys)
        f, axes = plt.subplots((n + 1) // 2 + 1, 2, figsize=(11, 2.3 * ((n + 1) // 2 + 1)),
                               sharex=True)
        axes = axes.ravel()
        # inputs: change from the first sample, in inches
        ax = axes[0]
        for q, c in zip(INPUTS, ["tab:green", "tab:red", "tab:purple", "tab:brown"]):
            a = dig.get((q, "ames_genhel"))
            if a is not None:
                ax.plot(a[:, 0], a[:, 1] - a[0, 1], color=c, lw=1.2, label=q.replace("_", " "))
        ax.set_ylabel("Δ input [in]")
        ax.legend(fontsize=7, ncol=2)
        axes[1].axis("off")
        axes[1].text(0.0, 0.9, f"Ballin Fig. {fig}, file {case['file']}", fontsize=10,
                     transform=axes[1].transAxes, weight="bold")
        axes[1].text(0.0, 0.1, "\n".join([
            f"{case['gross_weight_lbm']} lbm, CG FS {case['cg_station_in']} in, "
            f"WL {case['cg_waterline_in']} in",
            f"EAS {case['equivalent_airspeed_kt']} kt, density alt. {case['density_altitude_ft']} ft",
            f"{CAPTIONS[fig]}",
            *([f"dotted Eyas: stand-in config {', '.join(variants)} (comparison only)"]
              if variants else []),
            f"{control.replace('_', ' ')} moves at {t_in:.2f} s; differences over "
            f"{t_in:.2f}-{t_in + WINDOW:.2f} s"]),
            fontsize=8, transform=axes[1].transAxes)
        for ax, q in zip(axes[2:], qtys):
            col, label = QTY[q]
            rel = q not in ABSOLUTE
            ref = {}
            for s, (name, colour, ls) in SERIES.items():
                a = dig.get((q, s))
                if a is None:
                    continue
                y = a[:, 1] - (a[0, 1] if rel else 0.0)
                if q == "heading":
                    y = np.degrees(np.unwrap(np.radians(a[:, 1]))) - a[0, 1]
                ref[s] = np.column_stack([a[:, 0], y])
                ax.plot(*broken(a[:, 0], y), color=colour, ls=ls, lw=1.2, label=name)
            # the edges of the Ames simulation's 4/rev swing, where it was read as a band
            lo, hi = dig.get((q, "ames_genhel_lower")), dig.get((q, "ames_genhel_upper"))
            if lo is not None and hi is not None:
                ax.fill_between(hi[:, 0], at(lo, hi[:, 0]), hi[:, 1], color="0.45", alpha=0.2,
                                lw=0, label="Ames GenHel, 4/rev swing")
            for k, (name, colour, ls) in styles.items():
                t = np.array([float(r["t_s"]) for r in eyas[k]])
                y = np.array([float(r[col]) for r in eyas[k]])
                if q == "heading":
                    y = np.degrees(np.unwrap(np.radians(y)))
                if rel:
                    y = y - y[0]
                ax.plot(t, y, color=colour, ls=ls, lw=1.4, label=name)
                win = (t >= t_in) & (t <= t_in + WINDOW)
                for s, a in ref.items():
                    d = y[win] - at(a, t[win])
                    d = d[np.isfinite(d)]
                    if len(d):
                        diffs.append([fig, q, k, s, f"{np.sqrt(np.mean(d ** 2)):.3f}",
                                      f"{d[np.argmax(np.abs(d))]:.3f}"])
            # the reference: how far the simulations Ballin plots are from the flight test
            test = ref.get("aefa_test")
            for s in ("ames_genhel", "sikorsky_genhel"):
                if s in ref and test is not None:
                    tt = ref[s][:, 0]
                    win = (tt >= t_in) & (tt <= t_in + WINDOW)
                    d = ref[s][win, 1] - at(test, tt[win])
                    d = d[np.isfinite(d)]
                    if len(d):
                        diffs.append([fig, q, s, "aefa_test", f"{np.sqrt(np.mean(d ** 2)):.3f}",
                                      f"{d[np.argmax(np.abs(d))]:.3f}"])
            ax.axvspan(t_in, t_in + WINDOW, color="0.9", zorder=0)
            ax.set_ylabel(label, fontsize=8)
            ax.grid(alpha=0.3)
        for ax in axes[2 + n:]:
            ax.axis("off")
        axes[2].legend(fontsize=7)
        for ax in axes[-2:]:
            ax.set_xlabel("time [s]")
        f.tight_layout()
        f.savefig(dst / f"ballin_{fig.lower()}.png", dpi=110)
        plt.close(f)
        print(f"wrote {fig}")
    with open(dst / "ballin_app_d_differences.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["figure", "quantity", "eyas", "against", "rms", "largest"])
        w.writerows(diffs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
