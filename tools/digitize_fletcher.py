"""Digitize Fletcher, 21st ERF (1995) Paper VII-4, Figs. 3 and 4 (flight-data frequency responses
of the UH-60 in hover and at 80 kt) for M7c (D-035 V2).

    .venv/bin/python -I tools/digitize_fletcher.py <ERF-1995-VII-4-Fletcher.pdf> <overlay dir>

Writes data/uh60a/validation/fletcher_fig3.csv and fletcher_fig4.csv and, in <overlay dir>, one
PNG per panel with the readings drawn over the scan, for human review. The readings were
reviewed by a person (DATA_GAPS.md); a re-run that changes them needs a new review.

Each figure has four responses (p and q to lateral and longitudinal cyclic), each in three panels:
magnitude [dB of (rad/s)/in], phase [deg] and coherence, against frequency [rad/s] on a log axis
from 0.1 (the y axis) to 20 (the end of the x axis). How a panel is read:
- the y axis and x axis are snapped near the rough positions in the panel table (300 dpi), the
  y ticks found near where the rough tick values put them, and a linear y map fitted to them
  (it stops if fewer than three ticks are found or one is further than 3 px off the fit); x is
  log, from the y axis (0.1) to the right end of the x axis (20), checked against the 1 and 10
  rad/s ticks;
- the axes, their tick stubs and the response label are masked, and the one curve is followed
  column by column: the ink run nearest the previous reading (its end nearest, for the long
  runs of a steep stretch), with the allowed jump growing while the curve has no ink;
- the ink is masked from 5 px above the x axis down (the axis line and the specks along it), and
  in a 7 px strip at each printed x tick up to 14 px above it;
- readings are reported at 24 per decade where the curve has ink, plus wherever the straight
  line between two readings strays more than REFINE_DEV px from the traced curve or the curve
  rises more than REFINE_STEP px between them (the steep stretches, peaks and notches); flagged
  `gap` where filled across a short break and `edge` within 12 px of the x axis;
- a coherence curve that runs into the x axis line (no ink above the mask between two readings
  within 12 px of the axis) is read as 0, flagged `axis`: it lies on the line, within its width.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

import fitz
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.patheffects as pe  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "uh60a" / "validation"
DPI = 300
DARK = 140
SOURCE = ("Fletcher, J. W., A Model Structure for Identification of Linear Models of the UH-60 "
          "Helicopter in Hover and Forward Flight, 21st European Rotorcraft Forum (1995), "
          "Paper VII-4")

# Rough positions at 150 dpi (doubled below): column (x of the y axis, x of the 20 rad/s end) and
# row (y of the top tick, y of the bottom tick); y values (top, bottom, step); label boxes (x0, y0,
# x1, y1) at 150 dpi.
FIG3 = dict(page=9, caption="Fig. 3, CIFER-identified frequency responses for UH-60 in hover",
            cols={"L": (211, 636), "R": (711, 1133)},
            rows=[(241, 383), (437, 578), (630, 772), (848, 992), (1046, 1189), (1240, 1385)])
FIG4 = dict(page=10, caption="Fig. 4, CIFER-identified frequency responses for UH-60 in 80 knots "
            "forward flight",
            cols={"L": (188, 612), "R": (692, 1115)},
            rows=[(198, 342), (392, 536), (588, 730), (806, 950), (1003, 1146), (1197, 1340)])

REFINE_DEV = 2.5  # px, about the curve's line width
REFINE_STEP = 12.0  # px
AXIS_BAND = 12  # px above the x axis counted as on it (flags `edge`, `axis`)
# printed x ticks [rad/s]
X_TICKS = [k * 10.0 ** e for e in (-1, 0) for k in range(1, 10)] + [10.0, 20.0]

MAG = (0.0, -60.0, 20.0)
COH = (1.0, 0.0, 0.5)
# (response, column, first row): the response's magnitude, phase and coherence rows
RESPONSES = [("p_lat", "L", 0), ("p_lon", "R", 0), ("q_lat", "L", 3), ("q_lon", "R", 3)]
PHASE = {
    "fig3": {"p_lat": (-180.0, -540.0, 180.0), "p_lon": (0.0, -720.0, 180.0),
             "q_lat": (-180.0, -540.0, 180.0), "q_lon": (90.0, -270.0, 180.0)},
    "fig4": {"p_lat": (90.0, -270.0, 180.0), "p_lon": (90.0, -270.0, 180.0),
             "q_lat": (-90.0, -450.0, 180.0), "q_lon": (90.0, -270.0, 180.0)},
}
LABELS = {  # response label boxes at 150 dpi, in the magnitude panel
    "fig3": {"p_lat": (222, 228, 268, 285), "p_lon": (722, 228, 768, 285),
             "q_lat": (222, 836, 268, 893), "q_lon": (722, 836, 768, 893)},
    "fig4": {"p_lat": (200, 250, 250, 305), "p_lon": (715, 190, 770, 250),
             "q_lat": (200, 865, 250, 930), "q_lon": (715, 865, 770, 930)},
}


def render(doc, page):
    z = DPI / 72.0
    pix = doc[page - 1].get_pixmap(matrix=fitz.Matrix(z, z), colorspace=fitz.csGRAY)
    return np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width).copy()


def runs(v):
    d = np.diff(np.concatenate(([0], v.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


class PanelError(Exception):
    pass


def calibrate(dark, xa, xr, yt, yb, yv):
    """Snap the axes; fit y on the ticks; x log from 0.1 (y axis) to 20 (x-axis end)."""
    r = 16
    x_ax = xa - r + int(np.argmax([dark[yt:yb, x].sum() for x in range(xa - r, xa + r)]))
    y_ax = yb - r + int(np.argmax([dark[y, xa:xr].sum() for y in range(yb - r, yb + r)]))
    # the x axis's right end: last dark column on the axis line near the rough end
    line = dark[y_ax - 1:y_ax + 2, xr - 40:xr + 40].any(axis=0)
    ends = np.flatnonzero(line)
    if len(ends) == 0:
        raise PanelError("x axis end not found")
    x_end = xr - 40 + int(ends[-1])
    top, bot, step = yv
    values = np.arange(top, bot - 1e-9, -step) if top > bot else np.arange(top, bot + 1e-9, step)
    ypix = np.linspace(yt, yb, len(values))
    found = []
    for v, py in zip(values, ypix):
        best = None
        for y in range(int(py) - 10, int(py) + 11):
            stub = dark[y, x_ax - 16:x_ax - 2].sum()
            if stub >= 8 and (best is None or stub > best[1]):
                best = (y, stub)
        if best is not None:
            found.append((v, float(best[0])))
    if len(found) < 3:
        raise PanelError(f"only {len(found)} y ticks found")
    v_, y_ = np.array(found).T
    c = np.polyfit(y_, v_, 1)
    res = np.abs(np.polyval(c, y_) - v_) / abs(c[0])
    if res.max() > 3.0:
        raise PanelError(f"y tick off the fit by {res.max():.1f} px")
    xcoef = (x_ax, x_end)  # log10(w) = -1 + (x - x_ax) / (x_end - x_ax) * (log10(20) + 1)
    return x_ax, y_ax, x_end, c, xcoef


def to_w(px, xcoef):
    x0, x1 = xcoef
    return 10.0 ** (-1.0 + (px - x0) / (x1 - x0) * (math.log10(20.0) + 1.0))


def to_px(w, xcoef):
    x0, x1 = xcoef
    return x0 + (math.log10(w) + 1.0) / (math.log10(20.0) + 1.0) * (x1 - x0)


def ink_runs(col):
    """The column's ink runs, joined across gaps of up to 3 px (the scan's hatching)."""
    out = []
    for s, e in runs(col):
        if out and s - out[-1][1] <= 3:
            out[-1] = (out[-1][0], e)
        else:
            out.append((s, e))
    return out


def dist_to(v, rs):
    """Distance [px] from row v to the nearest of the runs rs (0 inside one)."""
    return min((0.0 if s <= v <= e - 1 else min(abs(s - v), abs(e - 1 - v)) for s, e in rs),
               default=math.inf)


def follow(ink, x_from, x_to, max_jump=20):
    """Follow the one curve column by column, on the run connected to the previous column's
    (else the nearest within reach). A long run (a steep stretch, a spike or a notch) is read
    at the end where the curve leaves it, half a line width in: the end nearer the ink of the
    first of the next six columns that tells them apart, else the end further from the
    previous reading."""
    n = x_to - x_from
    y = np.full(n, np.nan)
    last = prev = None
    missing = 0
    for i in range(n):
        rs = ink_runs(ink[:, x_from + i])
        if not rs:
            missing += 1
            continue
        if last is None:
            run = max(rs, key=lambda r: r[1] - r[0])
        else:
            linked = [r for r in rs if r[0] <= prev[1] and r[1] >= prev[0]] if missing == 0 else []
            reach = max_jump + 2 * missing
            cands = linked or [r for r in rs if dist_to(last, [r]) <= reach]
            if not cands:
                missing += 1
                continue
            run = min(cands, key=lambda r: dist_to(last, [r]))
        s, e = run
        if e - s > 12 and last is not None:
            y[i] = s + 2 if abs(s - last) > abs(e - 1 - last) else e - 3
            for j in range(i + 1, min(i + 7, n)):
                ahead = ink_runs(ink[:, x_from + j])
                dt, db = dist_to(s, ahead), dist_to(e - 1, ahead)
                if abs(dt - db) > 2:
                    y[i] = s + 2 if dt < db else e - 3
                    break
        else:
            y[i] = (s + e - 1) / 2.0
        last, prev = y[i], run
        missing = 0
    return y


def refine(y, a, b, keep):
    """Add columns between kept columns a and b until the straight line between neighbours
    follows the traced curve (REFINE_DEV) and no step is steeper than REFINE_STEP."""
    if b - a < 2:
        return
    chord = np.linspace(y[a], y[b], b - a + 1)
    dev = np.abs(y[a:b + 1] - chord)
    k = int(np.argmax(dev))
    if dev[k] <= REFINE_DEV:
        if abs(y[b] - y[a]) <= REFINE_STEP:
            return
        k = 1 + int(np.argmin(np.abs(y[a + 1:b] - (y[a] + y[b]) / 2.0)))
    keep.add(a + k)
    refine(y, a, a + k, keep)
    refine(y, a + k, b, keep)


def digitize_panel(img, x_ax_r, x_r, yt, yb, yv, label, coherence=False):
    dark = img < DARK
    x_ax, y_ax, x_end, ycoef, xcoef = calibrate(dark, x_ax_r, x_r, yt, yb, yv)
    ink = np.zeros_like(dark)
    y0, y1 = int(yt - 14), int(y_ax - 5)
    ink[y0:y1, x_ax + 5:x_end + 2] = dark[y0:y1, x_ax + 5:x_end + 2]
    for w in X_TICKS:
        x = int(round(to_px(w, xcoef)))
        ink[y_ax - 14:y1, x - 3:x + 4] = False
    if label is not None:
        a, b, c, d = (2 * v for v in label)
        ink[b:d, a:c] = False
    y = follow(ink, x_ax + 5, x_end + 1)
    ok = ~np.isnan(y)
    idx = np.flatnonzero(ok)
    filled = np.zeros(len(y), dtype=bool)
    on_axis = np.zeros(len(y), dtype=bool)
    near = lambda i: y[i] > y_ax - AXIS_BAND  # noqa: E731
    for a, b in zip(idx[:-1], idx[1:]):
        if 1 < b - a <= 25:
            y[a + 1:b] = np.interp(np.arange(a + 1, b), [a, b], [y[a], y[b]])
            filled[a + 1:b] = True
        elif b - a > 25 and coherence and near(a) and near(b):
            y[a + 1:b] = y_ax
            on_axis[a + 1:b] = True
    n = len(y)
    keep = set()
    for k in range(-24, int(24 * math.log10(20.0)) + 1):
        i = int(round(to_px(10.0 ** (k / 24.0), xcoef) - (x_ax + 5)))
        if 0 <= i < n and not np.isnan(y[i]):
            keep.add(i)
    valid = ~np.isnan(y)
    starts = np.flatnonzero(valid & ~np.concatenate(([False], valid[:-1])))
    ends = np.flatnonzero(valid & ~np.concatenate((valid[1:], [False])))
    keep.update(starts.tolist() + ends.tolist())
    for s, e in zip(starts, ends):
        cols = sorted(i for i in keep if s <= i <= e)
        for a, b in zip(cols[:-1], cols[1:]):
            refine(y, a, b, keep)
    rows = []
    for i in sorted(keep):
        if on_axis[i]:
            flag = "axis"
        else:
            flag = "gap" if filled[i] else ("edge" if near(i) else "")
        rows.append((to_w(x_ax + 5 + i, xcoef), float(np.polyval(ycoef, y[i])), flag))
    return rows, (img, x_ax, y_ax, x_end, yt, ycoef, xcoef, yv)


def unused_spans(coherence):
    """Frequency spans [rad/s] left out of the comparison: all but the readings with coherence at
    least 0.6 up to 12 rad/s, each reading's span reaching halfway (in log frequency) to its
    neighbours, and no further than 0.75 of a grid step (readings snap to pixel columns, so
    neighbouring grid readings can sit a little more than a step apart)."""
    half = 10.0 ** (0.75 / 24.0)
    ws = [w for w, _, _ in coherence]
    used = []
    for k, (w, v, _) in enumerate(coherence):
        if v >= 0.6 and w <= 12.0:
            lo = max(w / half, math.sqrt(ws[k - 1] * w)) if k > 0 else w / half
            hi = min(w * half, math.sqrt(ws[k + 1] * w)) if k + 1 < len(ws) else w * half
            used.append((lo, hi))
    spans, start = [], 0.1
    for a, b in used:
        if a > start * 1.0001:
            spans.append((start, a))
        start = max(start, b)
    spans.append((start, 20.0))
    return spans


def draw_overlay(panel, rows, spans, overlay):
    """The readings over the scan, for review: the unused spans greyed, the frequency axis
    labelled (Fletcher labels only the bottom panels) and the 12 rad/s limit dashed."""
    img, x_ax, y_ax, x_end, yt, ycoef, xcoef, yv = panel
    x0, x1 = x_ax - 90, x_end + 30
    yy0, yy1 = int(yt - 40), int(y_ax + 62)
    fig = plt.figure(figsize=((x1 - x0) / 100, (yy1 - yy0) / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img[yy0:yy1, x0:x1], cmap="gray", extent=(x0, x1, yy1, yy0))
    ax.add_patch(plt.Rectangle((x0, y_ax + 14), x1 - x0, yy1 - y_ax - 14, color="white", zorder=2))
    for a, b in spans:
        ax.axvspan(to_px(a, xcoef), to_px(b, xcoef), ymin=(yy1 - y_ax) / (yy1 - yy0), color="0.55",
                   alpha=0.35, lw=0, zorder=1)
    inv_y = lambda v: (v - ycoef[1]) / ycoef[0]  # noqa: E731
    for w, v, flag in rows:
        ax.plot(to_px(w, xcoef), inv_y(v), "x", ms=7, mew=1.8, zorder=3,
                color="magenta" if flag else "tab:red",
                path_effects=[pe.withStroke(linewidth=4, foreground="white")])
    top, bot, step = yv
    for v in (np.arange(bot, top + 1e-9, step) if top > bot else np.arange(top, bot + 1e-9, step)):
        ax.axhline(inv_y(v), color="orange", lw=0.5, alpha=0.7)
    for w in (1.0, 10.0):
        ax.axvline(to_px(w, xcoef), color="orange", lw=0.5, alpha=0.7)
    ax.axvline(to_px(12.0, xcoef), ymax=(yy1 - y_ax) / (yy1 - yy0) + (y_ax - yy0) / (yy1 - yy0),
               color="tab:blue", lw=1.2, ls="--", zorder=2)
    for w in (0.1, 0.2, 0.5, 1, 2, 5, 10, 12, 20):
        ax.text(to_px(w, xcoef), y_ax + 22, f"{w:g}", ha="center", va="top", fontsize=13, zorder=4,
                color="tab:blue" if w == 12 else "black")
    ax.text(x0 + 4, y_ax + 22, "rad/s", ha="left", va="top", fontsize=11, color="0.35", zorder=4)
    ax.set_xlim(x0, x1)
    ax.set_ylim(yy1, yy0)
    ax.axis("off")
    fig.savefig(overlay)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pdf", type=Path)
    parser.add_argument("overlays", type=Path)
    args = parser.parse_args()
    args.overlays.mkdir(parents=True, exist_ok=True)
    doc = fitz.open(args.pdf)
    failed = 0
    for name, spec in (("fig3", FIG3), ("fig4", FIG4)):
        img = render(doc, spec["page"])
        out = []
        for resp, col, r0 in RESPONSES:
            xa, xr = (2 * v for v in spec["cols"][col])
            panels = {}
            for kind, row, yv in (("magnitude_db", r0, MAG), ("phase_deg", r0 + 1, PHASE[name][resp]),
                                  ("coherence", r0 + 2, COH)):
                yt, yb = (2 * v for v in spec["rows"][row])
                label = LABELS[name][resp] if kind == "magnitude_db" else None
                try:
                    rows, panel = digitize_panel(img, xa, xr, yt, yb, yv, label,
                                                 coherence=kind == "coherence")
                except PanelError as e:
                    print(f"{name} {resp} {kind}: FAILED: {e}", file=sys.stderr)
                    failed += 1
                    continue
                print(f"{name} {resp} {kind}: {len(rows)} readings")
                out += [(resp, kind, w, v, flag) for w, v, flag in rows]
                panels[kind] = (rows, panel)
            spans = (unused_spans(panels["coherence"][0]) if "coherence" in panels
                     else [(12.0, 20.0)])
            for kind, (rows, panel) in panels.items():
                path = args.overlays / f"fletcher_{name}_{resp}_{kind}.png"
                draw_overlay(panel, rows, spans, path)
        path = OUT / f"fletcher_{name}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            f.write(f"# source: {SOURCE}, {spec['caption']} (p. VII-4-{spec['page'] - 2}, PDF p. "
                    f"{spec['page']}); flight data. Digitized from the scan at {DPI} dpi by "
                    "tools/digitize_fletcher.py.\n")
            f.write("# status: VERIFIED by human review of the overlays (2026-10-16, "
                    "DATA_GAPS.md); a re-run changes readings and needs a new review\n")
            f.write("# Responses: p_lat = p/delta_lat etc.; magnitude in dB of (rad/s)/in, phase in "
                    "deg, frequency in rad/s.\n")
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["response", "quantity", "frequency_rad_s", "value", "flag"])
            for resp, kind, fw, v, flag in out:
                w.writerow([resp, kind, f"{fw:.4f}", f"{v:.3f}", flag])
        print(f"wrote {path.relative_to(ROOT)}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
