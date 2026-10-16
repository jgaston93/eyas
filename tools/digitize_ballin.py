"""Digitize Ballin, NASA TM-88360 (1987), App. B trim plots (Figs. B1-B4) for M7c (V1).

    .venv/bin/python -I tools/digitize_ballin.py <NASA-TM-88360.pdf> <overlay dir>

Writes data/uh60a/validation/ballin_b1.csv ... ballin_b4.csv and, in <overlay dir>, one PNG
per panel with the readings drawn over the scan, for human review (D-035 M7c amendment).
The readings are `UNVERIFIED` until a person has checked the overlays (DATA_GAPS.md).

How it reads a panel. The panel table below gives, per panel, rough pixel positions of the
axes (read off a gridded render at 300 dpi) and the tick values printed on the scan. The
tool then:
- snaps to the exact axis lines and finds every tick stub near where the rough positions
  put it, and fits a linear pixel -> value map to them (it stops if fewer than three ticks
  are found or a tick is further off the fitted line than the larger of 3 px and 1 % of
  the axis, after dropping at most one stray mark);
- finds the flight-test points as hollow circles (their centre dot is the reading);
- masks the circles, the axes, the long straight lines (zero lines) and the label boxes,
  then follows the one or two GenHel curves column by column: where two separate, the one
  with more gaps is the dashed Ames curve; where they touch, both read the same.
Every reading carries a flag, and the low-confidence ones are listed for review:
`merged` (the two curves coincide within a line width; both take the shared centre),
`gap` (interpolated across a mask, a dash gap or a stretch along a zero line), `end`
(extended to the y axis by at most 8 px), `overlap` (circles closer than a diameter),
`on_curve` (a circle with a curve through its centre), `weak` (a broken ring).
Then, per figure, a cross-panel check: each flight-test run appears in every panel at the
same x, so a reading far from its run's median, or a run missing from a panel, is listed
for review (`consistency()`).
"""

import argparse
import csv
import sys
from pathlib import Path

import fitz
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "uh60a" / "validation"
DPI = 300
DARK = 140  # gray level below which a pixel is ink

SOURCE = ("Ballin, M. G., Validation of a Real-Time Engineering Simulation of the UH-60A "
          "Helicopter, NASA TM-88360 (1987), App. B")

KT = ("true_airspeed", "kt")
ROC = ("rate_of_climb", "1000 ft/min")

# Figures: page (1-based PDF page), rotation, page offset of B2's rough positions, caption.
FIGURES = {
    "B1": {"caption": "Fig. B1, UH-60A level-flight static trim (pp. 28-29); 16000 lb, CG FS "
                      "351.0 in, WL 231.5 in, density altitude 5250 ft"},
    "B2": {"caption": "Fig. B2, steady-climb and descent trim, 60 knots (p. 30)"},
    "B3": {"caption": "Fig. B3, steady-climb and descent trim, 100 knots (p. 31)"},
    "B4": {"caption": "Fig. B4, steady-climb and descent trim, 137.5 knots (p. 32)"},
}


def b1(page, pid, qty, unit, x_ax, y_top, x_right, y_bot, yv, labels=()):
    return dict(fig="B1", page=page, rot=0, id=pid, quantity=qty, unit=unit, xq=KT,
                approx=(x_ax, y_top, x_right, y_bot), x=(0.0, 200.0, 50.0), y=yv,
                labels=list(labels))


# Rough positions: (x of the y axis, y of the top tick, x of the right-most tick, y of the x
# axis) at 300 dpi; y = (value at the top tick, value at the bottom, step).
# Label boxes are (x0, y0, x1, y1) in pixels, masked before the curves are followed.
PANELS = [
    b1(30, "a", "longitudinal_cyclic", "% (0 = full forward)", 327, 1083, 1157, 1495,
       (0.0, 100.0, 20.0), [(330, 1060, 470, 1120), (330, 1420, 400, 1490)]),
    b1(30, "b", "lateral_cyclic", "% (100 = full right)", 329, 1575, 1157, 1985,
       (100.0, 0.0, 20.0), [(330, 1550, 440, 1610), (330, 1910, 400, 1980)]),
    b1(30, "c", "collective", "% (100 = full up)", 333, 2063, 1160, 2475,
       (100.0, 0.0, 20.0), [(336, 2040, 430, 2100), (336, 2400, 400, 2470)]),
    b1(30, "d", "pitch_attitude", "deg (+ nose up)", 1430, 921, 2265, 1240,
       (10.0, -10.0, 5.0), [(1432, 900, 1540, 960), (1432, 1185, 1500, 1238),
                            (1780, 900, 2290, 1030)]),
    b1(30, "e", "roll_angle", "deg (+ right)", 1433, 1333, 2265, 1655,
       (10.0, -10.0, 5.0), [(1436, 1310, 1530, 1365), (1436, 1600, 1500, 1652)]),
    b1(30, "f", "sideslip", "deg (+ right)", 1435, 1733, 2265, 1980,
       (10.0, -5.0, 5.0), [(1438, 1710, 1530, 1765), (1438, 1925, 1500, 1977)]),
    b1(30, "g", "pedal", "% (100 = full right)", 1438, 2063, 2268, 2475,
       (100.0, 0.0, 20.0), [(1440, 2040, 1540, 2100), (1440, 2400, 1505, 2470)]),
    b1(31, "h", "stabilator_incidence", "deg (+ trailing edge down)", 855, 783, 1675, 1193,
       (50.0, 0.0, 10.0), [(858, 720, 1300, 770), (858, 1130, 930, 1188),
                           (1220, 770, 1680, 900)]),
    b1(31, "i", "main_rotor_power", "hp", 855, 1355, 1675, 1763,
       (2500.0, 0.0, 500.0), [(858, 1690, 930, 1755)]),
]


def climb(fig, page, dx, dy, overrides=None):
    """B2-B4 share one layout; dx, dy shift B2's rough positions to the page. overrides:
    {panel id: {"labels": [boxes], "leaders": [(x0, y0, x1, y1)]}} in page pixels, for the
    curve labels and their leader lines, which differ per figure."""
    o = overrides or {}
    rows = [  # id, quantity, unit, x_ax, y_top, x_right, y_bot, (top, bottom, step), labels
        ("collective", "collective", "% (100 = full up)", 621, 669, 1205, 1154,
         (100.0, 0.0, 20.0), [(624, 645, 720, 700)]),
        ("power", "main_rotor_power", "hp/100", 1398, 573, 1983, 1145,
         (25.0, -5.0, 5.0), []),
        ("pedal", "pedal", "% (100 = full right)", 621, 1245, 1205, 1720,
         (100.0, 0.0, 20.0), [(624, 1225, 720, 1280)]),
        ("pitch", "pitch_attitude", "deg (+ nose up)", 1398, 1340, 1983, 1715,
         (10.0, -10.0, 5.0), [(1400, 1320, 1600, 1375)]),
        ("lateral", "lateral_cyclic", "% (100 = full right)", 621, 1823, 1205, 2302,
         (100.0, 0.0, 20.0), [(624, 1800, 720, 1860)]),
        ("longitudinal", "longitudinal_cyclic", "% (0 = full forward)", 1394, 1823, 1983,
         2302, (0.0, 100.0, 20.0), [(1396, 1800, 1520, 1860)]),
    ]
    out = []
    for pid, qty, unit, xa, yt, xr, yb, yv, labels in rows:
        extra = o.get(pid, {})
        out.append(dict(fig=fig, page=page, rot=90, id=pid, quantity=qty, unit=unit, xq=ROC,
                        approx=(xa + dx, yt + dy, xr + dx, yb + dy), x=(-3.0, 3.0, 1.0), y=yv,
                        labels=[(a + dx, b + dy, c + dx, d + dy) for a, b, c, d in labels]
                        + extra.get("labels", []), leaders=extra.get("leaders", [])))
    return out


PANELS += climb("B2", 32, 0, 0, {
    "pedal": {"labels": [(955, 1415, 1205, 1450), (945, 1588, 1022, 1620),
                         (925, 1646, 1140, 1680)],
              "leaders": [(1076, 1452, 1086, 1515), (1060, 1640, 1110, 1588),
                          (985, 1588, 968, 1565)]}})
PANELS += climb("B3", 33, -4, 22, {
    "pedal": {"labels": [(950, 1340, 1202, 1382), (930, 1585, 1147, 1624)],
              "leaders": [(1074, 1383, 1038, 1468), (1004, 1586, 1049, 1530)]}})
PANELS += climb("B4", 34, -6, 94, {
    "pedal": {"labels": [(954, 1419, 1205, 1460), (940, 1639, 1158, 1679),
                         (650, 1626, 742, 1664)],
              "leaders": [(1080, 1461, 1058, 1515), (1065, 1638, 1112, 1583),
                          (712, 1624, 755, 1597)]}})


# ---- Image helpers -------------------------------------------------------------------------

def render(doc, page, rot):
    pg = doc[page - 1]
    z = DPI / 72.0
    pix = pg.get_pixmap(matrix=fitz.Matrix(z, z).prerotate(rot), colorspace=fitz.csGRAY)
    return np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width).copy()


def runs(v):
    """(start, end) of each run of True in a 1-D bool array, end exclusive."""
    d = np.diff(np.concatenate(([0], v.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


class PanelError(Exception):
    pass


def snap_axis(dark, approx):
    """Exact x of the y axis and y of the x axis near the rough positions."""
    xa, yt, xr, yb = approx
    r = 30
    cols = [dark[yt:yb, x].sum() for x in range(xa - r, xa + r)]
    x_ax = xa - r + int(np.argmax(cols))
    rows = [dark[y, xa:xr].sum() for y in range(yb - r, yb + r)]
    y_ax = yb - r + int(np.argmax(rows))
    return x_ax, y_ax


def stub_centres(profile_fn, lo, hi):
    """Centres of the positions in [lo, hi) where profile_fn says a tick stub is."""
    hits = np.array([profile_fn(i) for i in range(lo, hi)])
    return [lo + (s + e - 1) / 2.0 for s, e in runs(hits)]


def calibrate(dark, approx, x_ax, y_ax, xv, yv):
    """Linear pixel -> value maps from the tick stubs along both axes, and the pixel rows
    and columns of the stubs found."""
    xa, yt, xr, yb = approx
    stubs = []

    def is_ystub(y):
        left = dark[y, x_ax - 22:x_ax - 2][::-1]
        right = dark[y, x_ax + 3:x_ax + 27]
        for side in (left, right):
            n = int(np.argmin(side)) if not side.all() else len(side)
            if 5 <= n <= 18 and side[:n].all():
                return True
        return False

    def is_xstub(x):
        up = dark[y_ax - 26:y_ax - 3, x][::-1]
        down = dark[y_ax + 3:y_ax + 26, x]
        for side in (up, down):
            n = int(np.argmin(side)) if not side.all() else len(side)
            if 5 <= n <= 18 and side[:n].all():
                return True
        return False

    def fit(found, values, guess_px, what):
        # The plots were drawn by hand: ticks sit up to about 1 % of the axis off even
        # spacing (B2's collective panel: 6 px). The tolerance is the larger of 3 px and
        # 1 % of the axis length, and the residual is reported as read-off accuracy.
        tol = max(3.0, 0.01 * abs(guess_px[-1] - guess_px[0]))
        pairs = []
        for v, g in zip(values, guess_px):
            near = [f for f in found if abs(f - g) <= 18]
            if near:
                pairs.append((min(near, key=lambda f: abs(f - g)), v))
        if len(pairs) < 3:
            raise PanelError(f"{what}: only {len(pairs)} ticks found near the rough positions")
        def lsq(p):
            A = np.vstack([p[:, 0], np.ones(len(p))]).T
            coef, *_ = np.linalg.lstsq(A, p[:, 1], rcond=None)
            return coef, (A @ coef - p[:, 1]) / coef[0]

        p = np.array(pairs)
        coef, res_px = lsq(p)
        if np.abs(res_px).max() > tol and len(p) >= 5:
            # one stray mark taken for a tick: drop the worst and refit
            worst = int(np.argmax(np.abs(res_px)))
            print(f"    {what}: dropped the tick at {p[worst, 1]:g} "
                  f"({res_px[worst]:+.1f} px off)", file=sys.stderr)
            p = np.delete(p, worst, axis=0)
            coef, res_px = lsq(p)
        if np.abs(res_px).max() > tol:
            raise PanelError(f"{what}: tick {np.abs(res_px).max():.1f} px off the fitted line")
        if what == "y axis":
            stubs.extend(p[:, 0].tolist())
        return coef, len(p), float(np.abs(res_px).max())

    # y ticks: values from top to bottom, rough pixels by linear interpolation.
    top, bot, step = yv
    n = int(round(abs(top - bot) / step))
    yvals = [top + (bot - top) * k / n for k in range(n + 1)]
    yguess = [yt + (y_ax - yt) * k / n for k in range(n + 1)]
    yfound = stub_centres(is_ystub, yt - 25, y_ax + 1)
    # The x axis itself is the bottom tick, the top of the axis line the top one.
    yfound.append(float(y_ax))
    ycoef, ny, yres = fit(yfound, yvals, yguess, "y axis")

    left, right, step = xv
    n = int(round((right - left) / step))
    xvals = [left + step * k for k in range(n + 1)]
    xguess = [x_ax + (xr - x_ax) * k / n for k in range(n + 1)]
    xfound = stub_centres(is_xstub, x_ax - 5, xr + 25)
    xfound.append(float(x_ax))
    xcoef, nx, xres = fit(xfound, xvals, xguess, "x axis")
    return xcoef, ycoef, (nx, xres, ny, yres), stubs, xfound


def find_circles(dark, box, r_range=(10, 15)):
    """Centres (x, y) [px] of the hollow test circles in box, with ring scores and radii.

    A candidate needs a ring (at least 80 % of 48 samples inked at some radius r in
    r_range), white just inside and just outside it (a thin printed circle, not a crossing
    of curves), a white annulus at 6 px (hollow) and a centre dot (ink within 3 px), whose
    centroid is the reading. One reading per dot; overlapping symbols have their own dots.
    """
    x0, y0, x1, y1 = box
    sub = dark[y0:y1, x0:x1]
    h, w = sub.shape
    ang = np.linspace(0, 2 * np.pi, 48, endpoint=False)
    pad = r_range[1] + 6
    P = np.pad(sub, pad)

    def ring(r, tol):
        """Fraction of samples at radius r inked (within tol px)."""
        acc = np.zeros((h, w))
        for t in ang:
            dy, dx = int(round(r * np.sin(t))), int(round(r * np.cos(t)))
            hit = np.zeros((h, w), dtype=bool)
            for ey in range(-tol, tol + 1):
                for ex in range(-tol, tol + 1):
                    hit |= P[pad + dy + ey:pad + dy + ey + h, pad + dx + ex:pad + dx + ex + w]
            acc += hit
        return acc / len(ang)

    best = np.zeros((h, w))
    inside = np.ones((h, w))
    outside = np.ones((h, w))
    best_r = np.zeros((h, w), dtype=int)
    for r in range(r_range[0], r_range[1] + 1):
        acc = ring(r, 1)
        better = acc > best
        if better.any():
            ins = ring(r - 4, 0)
            out = ring(r + 4, 0)
            best[better] = acc[better]
            inside[better] = ins[better]
            outside[better] = out[better]
            best_r[better] = r
    hollow = ring(6, 0)
    # a full ring may have a steep curve running past it (B3's collective): more ink
    # just outside is allowed then
    cand = np.argwhere(((best >= 0.8) & (outside <= 0.4) | (best >= 0.95) & (outside <= 0.5))
                       & (inside <= 0.3) & (hollow <= 0.3))
    order = sorted(cand.tolist(), key=lambda p: -best[p[0], p[1]])
    found = []
    for y, x in order:
        win = sub[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
        ys, xs = np.nonzero(win)
        if not len(xs):
            continue
        cx = max(0, x - 3) + xs.mean()
        cy = max(0, y - 3) + ys.mean()
        if any((cx - ox) ** 2 + (cy - oy) ** 2 <= 9 ** 2 for ox, oy, *_ in found):
            continue
        # The printed dot is 8-16 px of ink with white around it (radius 4.5).
        dot = int(sub[int(cy) - 2:int(cy) + 3, int(cx) - 2:int(cx) + 3].sum())
        iso = np.mean([P[pad + int(round(cy + 4.5 * np.sin(t))),
                         pad + int(round(cx + 4.5 * np.cos(t)))] for t in ang[::2]])
        found.append((cx, cy, float(best[y, x]), int(best_r[y, x]), dot, float(iso)))
    # Clean symbols: a full ring and an isolated dot. A weaker candidate is kept only next
    # to a clean one (two test points printed overlapping, whose second dot is hidden), and
    # read as `overlap`; anything else is a crossing of curves, dashes or letters.
    clean = [f for f in found if f[2] >= 0.95 and f[5] <= 0.1 and f[4] <= 18]
    out = [(x, y, s, r) for x, y, s, r, _, _ in clean]
    # A full ring with a curve through its centre (no isolated dot): read as `on_curve`,
    # since the curve's ink pulls the dot centroid by up to 3 px.
    on_curve = [f for f in found if f not in clean and f[2] >= 0.95 and f[5] <= 0.35
                and f[4] <= 25]
    out += [(x, y, 0.94, r) for x, y, s, r, _, _ in on_curve]
    for x, y, s, r, dot, iso in found:
        # thresholds from the B1-B4 candidates: every real partner has s >= 0.85,
        # iso <= 0.33, dot <= 21; every crossing next to a symbol s <= 0.83
        if any(abs(x - ox) < 1e-9 and abs(y - oy) < 1e-9 for ox, oy, *_ in out) \
                or s < 0.85 or iso > 0.35 or dot > 21:
            continue
        if any((x - cx) ** 2 + (y - cy) ** 2 <= (2.2 * r) ** 2 for cx, cy, *_ in clean):
            out.append((x, y, min(s, 0.89), r))  # below 0.9: reported as weak or overlap
    return [(x0 + x, y0 + y, s, r) for x, y, s, r in out]


def straight_lines(img, max_slope=0.01, min_ink=0.6):
    """Near-horizontal straight lines across a bool image: (offset at the middle column
    [px], slope [px/px]) of each line whose path is at least min_ink inked (within 1 px).
    The scans are skewed by up to about 0.5 %, so a printed zero line isn't one pixel row."""
    h, w = img.shape
    xs = np.arange(w) - w / 2.0
    pad = np.pad(img, ((3, 3), (0, 0)))
    found = []
    for slope in np.linspace(-max_slope, max_slope, 21):
        rows = np.rint(xs * slope).astype(int)
        frac = np.zeros(h)
        for y0 in range(h):
            r = y0 + rows + 3
            ok = (r >= 1) & (r < h + 5)
            r = np.clip(r, 1, h + 4)
            hit = pad[r, np.arange(w)] | pad[r - 1, np.arange(w)] | pad[r + 1, np.arange(w)]
            frac[y0] = (hit & ok).mean()
        for y0 in np.flatnonzero(frac >= min_ink):
            found.append((float(frac[y0]), float(y0), float(slope)))
    # one line per cluster of offsets: the best-inked, centred on its ink (the tolerance
    # above makes neighbouring offsets look fully inked too)
    found.sort(reverse=True)
    lines = []
    cols = np.arange(w)
    for f, y0, slope in found:
        if any(abs(y0 - l[0]) <= 8 for l in lines):
            continue
        rows = np.rint(xs * slope).astype(int)
        offs = np.arange(int(y0) - 5, int(y0) + 6)
        wts = []
        for o in offs:
            r = np.clip(o + rows, 0, h - 1)
            wts.append(img[r, cols].mean())
        wts = np.array(wts)
        lines.append((float((offs * wts).sum() / wts.sum()), slope))
    return lines


def mask_line(img, y0, slope, half=4):
    """Clear a straight line (as found by straight_lines) and half px each side."""
    h, w = img.shape
    xs = np.arange(w)
    ys = np.rint(y0 + (xs - w / 2.0) * slope).astype(int)
    for d in range(-half, half + 1):
        r = ys + d
        ok = (r >= 0) & (r < h)
        img[r[ok], xs[ok]] = False


def follow_curves(ink, x_from, x_to, max_jump=25, end_gap=150):
    """Follow up to two curves column by column. Returns two arrays of y [px] (NaN where a
    track has no ink) and a per-column merged flag.

    Each column's ink runs are matched to the two tracks by nearness. The allowed jump
    grows by 2 px per column since a track last had ink, so a track picks its curve up
    again after a masked circle, label or zero line; after end_gap columns without ink
    the track has ended. While the tracks are within 6 px they
    move together: one run is both (merged), two runs split them.
    """
    n = x_to - x_from
    t = np.full((2, n), np.nan)
    merged = np.zeros(n, dtype=bool)
    last = [None, None]
    missing = [0, 0]
    sep = None  # (column, slopes, positions) where the tracks last came together

    def slope_before(y, i, span=20):
        idx = np.arange(max(0, i - span), i)
        idx = idx[~np.isnan(y[idx])]
        if len(idx) < 4:
            return 0.0
        return float(np.polyfit(idx, y[idx], 1)[0])

    for i in range(n):
        rs = [((s + e - 1) / 2.0, e - s) for s, e in runs(ink[:, x_from + i])]
        if last[0] is None:
            if rs:
                pick = sorted(rs, key=lambda r: -r[1])[:2]
                last = sorted([pick[0][0], pick[-1][0]])
                t[0, i], t[1, i] = last
                merged[i] = len(pick) == 1
            continue
        # a track with no ink for more than end_gap columns has ended (the curve stops)
        reach = [max_jump + 2 * m if m <= end_gap else -1.0 for m in missing]
        if abs(last[0] - last[1]) <= 6:
            if sep is None:
                # entering a shared stretch: remember each track's slope, so that when the
                # curves part again each is matched to its own (they may have crossed)
                sep = (i, [slope_before(t[k], i) for k in (0, 1)], list(last))
            mid = (last[0] + last[1]) / 2.0
            near = sorted((r for r in rs if abs(r[0] - mid) <= max(reach)),
                          key=lambda r: abs(r[0] - mid))[:2]
            if len(near) == 1:
                t[:, i] = near[0][0]
                merged[i] = True
            elif len(near) == 2:
                a, b = sorted(r[0] for r in near)
                i0, slopes, y0s = sep
                pred = [y0s[k] + slopes[k] * (i - i0) for k in (0, 1)]
                if abs(pred[0] - a) + abs(pred[1] - b) <= abs(pred[0] - b) + abs(pred[1] - a):
                    t[0, i], t[1, i] = a, b
                else:
                    t[0, i], t[1, i] = b, a
        else:
            sep = None
            # each run goes to at most one track, the nearer one; the other track takes its
            # next-nearest run within reach, if any
            pairs = sorted((abs(r[0] - last[k]), k, j) for k in (0, 1) for j, r in enumerate(rs)
                           if abs(r[0] - last[k]) <= reach[k])
            used = set()
            for _, k, j in pairs:
                if np.isnan(t[k, i]) and j not in used:
                    t[k, i] = rs[j][0]
                    used.add(j)
            if not np.isnan(t[0, i]) and not np.isnan(t[1, i]) and abs(t[0, i] - t[1, i]) <= 3:
                merged[i] = True
        for k in (0, 1):
            if np.isnan(t[k, i]):
                missing[k] += 1
            else:
                last[k] = t[k, i]
                missing[k] = 0
    return t, merged


def tidy_tracks(t, merged, dash_gap=30, stray=12):
    """Clean the two tracks: drop stray fragments (shorter than `stray` columns with no
    ink for dash_gap columns on both sides), and where one track has had no ink for longer
    than a dash gap while the other has, the curves coincide: it takes the other's value
    (merged)."""
    t = t.copy()
    merged = merged.copy()
    n = t.shape[1]
    for k in (0, 1):
        ok = ~np.isnan(t[k])
        for s_, e_ in runs(ok):
            before = ok[max(0, s_ - dash_gap):s_].any()
            after = ok[e_:e_ + dash_gap].any()
            if e_ - s_ < stray and not before and not after:
                t[k, s_:e_] = np.nan
    for k in (0, 1):
        o = 1 - k
        for s_, e_ in runs(np.isnan(t[k])):
            if e_ - s_ > dash_gap or s_ == 0 or e_ == n:
                idx = np.arange(s_, e_)
                have = idx[~np.isnan(t[o, idx])]
                t[k, have] = t[o, have]
                merged[have] = True
    return t, merged


def fill_gaps(y, limit=30):
    """Linear interpolation across NaN gaps of at most `limit` columns; returns the filled
    array and a bool array marking the filled columns."""
    y = y.copy()
    filled = np.zeros(len(y), dtype=bool)
    ok = ~np.isnan(y)
    idx = np.flatnonzero(ok)
    for a, b in zip(idx[:-1], idx[1:]):
        if 1 < b - a <= limit + 1:
            y[a + 1:b] = np.interp(np.arange(a + 1, b), [a, b], [y[a], y[b]])
            filled[a + 1:b] = True
    return y, filled


def digitize(img, spec, overlay_path):
    dark = img < DARK
    x_ax, y_ax = snap_axis(dark, spec["approx"])
    xcoef, ycoef, quality, ystubs, xstubs = calibrate(dark, spec["approx"], x_ax, y_ax,
                                                      spec["x"], spec["y"])
    _, yt, xr, _ = spec["approx"]
    box = (x_ax + 4, yt - 30, xr + 60, y_ax - 3)

    def in_label(x, y):
        return any(a <= x <= c and b <= y <= d for a, b, c, d in spec["labels"])

    circles = [c for c in find_circles(dark, box) if not in_label(c[0], c[1])]

    ink = dark.copy()
    keep = np.zeros_like(ink)
    keep[box[1]:box[3], box[0]:box[2]] = True
    ink &= keep
    for x, y, _, r in circles:
        yy, xx = np.ogrid[:ink.shape[0], :ink.shape[1]]
        ink[(xx - x) ** 2 + (yy - y) ** 2 <= (r + 4) ** 2] = False
    for a, b, c, d in spec["labels"]:
        ink[b:d, a:c] = False
    for xa_, ya_, xb_, yb_ in spec.get("leaders", []):
        # a label's leader line, 4 px each side, between the given end points
        for f in np.linspace(0.0, 1.0, 200):
            cx, cy = xa_ + f * (xb_ - xa_), ya_ + f * (yb_ - ya_)
            ink[int(cy) - 4:int(cy) + 5, int(cx) - 4:int(cx) + 5] = False
    # Long straight lines (zero lines, the climb-rate zero line), found before the circles
    # break them up. A curve lying on one can't be told from it; those columns are left
    # unread.
    # Only lines on the panel's zero value count: a nearly flat, straight curve (B3's
    # lateral cyclic) is otherwise just as straight and long.
    plot = dark[box[1]:box[3], box[0]:box[2]]
    y_zero = -ycoef[1] / ycoef[0] - box[1]
    x_zero = -xcoef[1] / xcoef[0] - box[0]
    hlines = [ln for ln in straight_lines(plot) if abs(ln[0] - y_zero) <= 8]
    vlines = [ln for ln in straight_lines(plot.T) if abs(ln[0] - x_zero) <= 8]
    masked_rows = [box[1] + int(round(y0)) for y0, _ in hlines]
    masked_cols = [box[0] + int(round(x0)) for x0, _ in vlines]
    sub = ink[box[1]:box[3], box[0]:box[2]]
    for y0, slope in hlines:
        mask_line(sub, y0, slope)
    for x0, slope in vlines:
        mask_line(sub.T, x0, slope)
    # the axes and the tick stubs next to them
    ink[:, :x_ax + 4] = False
    ink[y_ax - 6:, :] = False
    for y in ystubs:
        ink[int(y) - 4:int(y) + 5, x_ax:x_ax + 24] = False
    for x in xstubs:
        ink[y_ax - 24:y_ax, int(x) - 4:int(x) + 5] = False

    t, merged = follow_curves(ink, box[0], box[2])
    t, merged = tidy_tracks(t, merged)
    # Which track is dashed (Ames): more gaps where the two are apart.
    apart = ~merged & ~np.isnan(t[0]) | ~merged & ~np.isnan(t[1])
    gaps = [np.isnan(t[k][apart]).sum() for k in (0, 1)]
    ames = int(np.argmax(gaps)) if max(gaps) > 0 else 1
    sik = 1 - ames

    def to_x(px):
        return xcoef[0] * px + xcoef[1]

    def to_y(py):
        return ycoef[0] * py + ycoef[1]

    rows = []
    zero_px = [box[1] + y0 for y0, _ in hlines]
    for name, k in (("sikorsky_genhel", sik), ("ames_genhel", ames)):
        y, filled = fill_gaps(t[k])
        # across a masked zero line the gap may be long (a curve running along it):
        # interpolate up to 150 px when both ends are within 12 px of the line
        ok = np.flatnonzero(~np.isnan(y))
        for a_, b_ in zip(ok[:-1], ok[1:]):
            if 1 < b_ - a_ <= 151 and zero_px and all(
                    min(abs(y[j] + 0 - z) for z in zero_px) <= 12 for j in (a_, b_)):
                y[a_ + 1:b_] = np.interp(np.arange(a_ + 1, b_), [a_, b_], [y[a_], y[b_]])
                filled[a_ + 1:b_] = True
        # a curve starting within 8 px of the y axis is extended to it (its first point
        # is under the axis mask)
        ok = np.flatnonzero(~np.isnan(y))
        if len(ok) >= 6 and ok[0] <= 8:
            i0 = ok[0]
            slope = (y[ok[5]] - y[i0]) / (ok[5] - i0)
            ext = np.arange(-(box[0] - x_ax), i0)
            for i in ext:
                rows.append((name, box[0] + i, y[i0] + slope * (i - i0), "end"))
        for i in np.flatnonzero(~np.isnan(y)):
            px = box[0] + i
            flag = "merged" if merged[i] else ("gap" if filled[i] else "")
            if any(abs(py - y[i]) <= 5 for py in masked_rows) or px in masked_cols:
                flag = "gap"
            rows.append((name, px, y[i], flag))
    pts = []
    for x, y, s, r in sorted(circles):
        flag = ""
        if any(0 < (x - ox) ** 2 + (y - oy) ** 2 < (2 * r + 2) ** 2 for ox, oy, _, _ in circles):
            flag = "overlap"
        elif s == 0.94:
            flag = "on_curve"
        elif s < 0.9:
            flag = "weak"
        pts.append(("aefa_test", x, y, flag))

    # Curves are reported every few data units, test points as found.
    step = 5.0 if spec["xq"] == KT else 0.25
    out = []
    for name in ("sikorsky_genhel", "ames_genhel"):
        cr = [r for r in rows if r[0] == name]
        if not cr:
            continue
        xs = np.array([to_x(r[1]) for r in cr])
        ys = np.array([to_y(r[2]) for r in cr])
        fl = [r[3] for r in cr]
        lo = np.ceil(xs.min() / step) * step
        for xv in np.arange(lo, xs.max() + 1e-9, step):
            j = int(np.argmin(np.abs(xs - xv)))
            if abs(xs[j] - xv) > 2 * abs(xcoef[0]):
                continue  # no ink at this x
            yv = float(np.interp(xv, xs, ys))
            out.append((name, float(xv), yv, fl[j]))
    for name, x, y, flag in pts:
        out.append((name, to_x(x), to_y(y), flag))

    # Overlay for review.
    x0, y0 = max(0, x_ax - 140), max(0, yt - 70)
    x1, y1 = min(img.shape[1], xr + 60), min(img.shape[0], y_ax + 70)
    fig = plt.figure(figsize=((x1 - x0) / 100, (y1 - y0) / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img[y0:y1, x0:x1], cmap="gray", extent=(x0, x1, y1, y0))
    inv_x = lambda v: (v - xcoef[1]) / xcoef[0]  # noqa: E731
    inv_y = lambda v: (v - ycoef[1]) / ycoef[0]  # noqa: E731
    colours = {"sikorsky_genhel": "tab:red", "ames_genhel": "tab:blue", "aefa_test": "tab:green"}
    for name, x, y, flag in out:
        mk = "o" if name == "aefa_test" else ("x" if name == "ames_genhel" else "+")
        c = "magenta" if flag in ("overlap", "weak", "gap") else colours[name]
        ax.plot(inv_x(x), inv_y(y), mk, ms=9 if name == "aefa_test" else 6, mfc="none", mec=c,
                mew=1.4)
    for v in np.arange(spec["y"][1], spec["y"][0] + 1e-9, spec["y"][2]) if spec["y"][0] > \
            spec["y"][1] else np.arange(spec["y"][0], spec["y"][1] + 1e-9, spec["y"][2]):
        ax.axhline(inv_y(v), color="orange", lw=0.5, alpha=0.7)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.axis("off")
    fig.savefig(overlay_path)
    plt.close(fig)
    return out, quality, len(circles)


def consistency(fig, panels):
    """Cross-panel check of the test points: each flight-test run is plotted in every
    panel of a figure at the same airspeed (B1) or climb rate (B2-B4), so the spread of its
    x readings measures the read-off accuracy. Clusters runs by x; prints each cluster's
    spread; returns review lines for readings far from their cluster's median and for
    panels a run is missing from."""
    tol = 2.0 if fig == "B1" else 0.08  # cluster join distance [kt], [1000 ft/min]
    far = 1.5 if fig == "B1" else 0.1   # outlier distance from the median
    pts = [(x, spec["id"], y) for spec, rows in panels for name, x, y, _ in rows
           if name == "aefa_test"]
    pts.sort()
    clusters = []
    for p_ in pts:
        if clusters and p_[0] - clusters[-1][-1][0] <= tol:
            clusters[-1].append(p_)
        else:
            clusters.append([p_])
    out = []
    ids = [spec["id"] for spec, _ in panels]
    print(f"{fig}: test-point runs (x median, panels, spread sd):")
    for c in clusters:
        xs = np.array([q[0] for q in c])
        med = float(np.median(xs))
        print(f"    {med:8.3f}  {len(c):2d}  {xs.std():.3f}")
        for x, pid, y in c:
            if abs(x - med) > far:
                out.append(f"{fig}:{pid} aefa_test x={x:.2f} y={y:.2f} off its run's median "
                           f"x={med:.2f} (other panels)")
        have = {q[1] for q in c}
        if len(c) >= 3 and len(have) < len(ids):
            out.append(f"{fig} run at x={med:.2f}: no test point found in panel(s) "
                       f"{', '.join(i for i in ids if i not in have)}")
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pdf", type=Path, help="NASA-TM-88360.pdf")
    parser.add_argument("overlays", type=Path, help="directory for the review overlays")
    parser.add_argument("--only", help="digitize only panels whose 'fig:id' contains this")
    args = parser.parse_args()
    args.overlays.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    doc = fitz.open(args.pdf)
    pages = {}
    by_fig = {}
    failed = 0
    review = []
    for spec in PANELS:
        tag = f"{spec['fig']}:{spec['id']}"
        if args.only and args.only not in tag:
            continue
        key = (spec["page"], spec["rot"])
        if key not in pages:
            pages[key] = render(doc, *key)
        path = args.overlays / f"ballin_{spec['fig'].lower()}_{spec['id']}.png"
        try:
            rows, (nx, xres, ny, yres), nc = digitize(pages[key], spec, path)
        except PanelError as e:
            print(f"{tag}: FAILED: {e}", file=sys.stderr)
            failed += 1
            continue
        print(f"{tag}: ticks x {nx} (max {xres:.2f} px), y {ny} (max {yres:.2f} px); "
              f"{nc} test points; {sum(r[0] != 'aefa_test' for r in rows)} curve samples")
        for name, x, y, flag in rows:
            if flag in ("overlap", "weak", "on_curve"):
                review.append(f"{tag} {spec['quantity']} {name} x={x:.2f} y={y:.2f} {flag}")
        by_fig.setdefault(spec["fig"], []).append((spec, rows))
    for fig, panels in by_fig.items():
        review += consistency(fig, panels)
        path = OUT / f"ballin_{fig.lower()}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            f.write(f"# source: {SOURCE}, {FIGURES[fig]['caption']}. Digitized from the NASA "
                    f"scan at {DPI} dpi by tools/digitize_ballin.py.\n")
            f.write("# status: UNVERIFIED - needs human review of the overlays (DATA_GAPS.md)\n")
            f.write("# Series: sikorsky_genhel (solid), ames_genhel (dashed), aefa_test (circles,"
                    " USAAEFA flight test). Flags: merged, gap, overlap, weak (tool docstring).\n")
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["panel", "quantity", "unit", "x_quantity", "x_unit", "series", "x", "y",
                        "flag"])
            for spec, rows in panels:
                for name, x, y, flag in rows:
                    w.writerow([spec["id"], spec["quantity"], spec["unit"], spec["xq"][0],
                                spec["xq"][1], name, f"{x:.3f}", f"{y:.3f}", flag])
        print(f"wrote {path.relative_to(ROOT)}")
    (args.overlays / "review.txt").write_text("\n".join(review) + "\n", encoding="utf-8")
    print(f"{len(review)} low-confidence readings listed in {args.overlays / 'review.txt'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
