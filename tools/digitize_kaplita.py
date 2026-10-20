"""Digitize Kaplita, NASA CR-177360 (1985), Figs. 10-13: UH-60A steady-heading sideslip.

    .venv/bin/python -I tools/digitize_kaplita.py <19870008277.pdf> <overlay dir>

Writes data/uh60a/validation/kaplita_sideslip.csv and, in <overlay dir>, one PNG per panel
with the readings drawn over the scan (with the same crop unmarked, and a JSON of the crop
and the readings in it), for human review. Readings need a person's check of the overlays
before use (DATA_GAPS.md); a re-run needs a new review.

The figures are USAAEFA flight-test points (USAAEFA Project 79-24) plotted as hollow circles
on graph paper, with Sikorsky's GenHel curves; only the circles are read. Each page is a
600 dpi bilevel scan, printed rotated; it is turned upright and averaged to 300 dpi grey.

How it reads a panel:
- Axes: the panel table gives the rough pixel positions of the printed tick labels and
  their values (read off a 150 dpi render). The labels sit on evenly spaced major
  gridlines (every tenth line, slightly wider than the others), so their offset and
  spacing are fitted jointly to the line profile across the plot (calibrate()), each line
  refined to its peak, and a straight line fitted through them; it stops if one is more
  than 3 px off. The sideslip axis is calibrated the same way, once per page.
- Circles: the graph-paper grid is removed by notch filtering its line frequencies (and
  their harmonics) in the Fourier domain, with a 2 px Gaussian blur; then each rough
  centre (placed by eye on the 150 dpi render, in the panel table) is refined by fitting a
  ring (radius 7-15 px) that maximizes the ring's ink against just inside and just outside
  it, within 15 px of the rough centre.
Flags: `on_curve` (the rough centre was placed on a GenHel curve, so the ring is partly
hidden), `weak` (ring contrast below WEAK), `hand` (the reviewer's own centre, clicked on the
review page's 300 dpi scan, from kaplita_hand_edits.csv; a point the reviewer found no
test circle for is dropped), `beta_open` (the sideslip is in question: the 60 kt run near
zero sideslip is plotted about 1.3 deg apart on Fig. 11a and on Figs. 10 and 11b, and the
reviewer couldn't tell from the scan whether the plot or the reading is off; BETA_OPEN, a
placeholder until resolved). Then a cross-panel check: each run is plotted
at the same sideslip in every panel of a flight, so a reading more than 0.6 deg from its
run's median sideslip is listed for review.
"""

import argparse
import csv
import json
from pathlib import Path

import fitz
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "uh60a" / "validation" / "kaplita_sideslip.csv"
HAND = ROOT / "data" / "uh60a" / "validation" / "kaplita_hand_edits.csv"
CURVES = ROOT / "data" / "uh60a" / "validation" / "kaplita_sideslip_genhel.csv"
HAND_LINES = ROOT / "data" / "uh60a" / "validation" / "kaplita_genhel_hand_lines.csv"
# Panels whose GenHel curve is traced: all but the stabilator (Fig. 10's curves hold the
# incidence fixed) and the collective (a flat line, lost with the axis lines).
TRACED = ("roll", "lat", "pedal", "pitch", "lon")
# Sideslip range traced where it is narrower than the test points': a label's leader line
# meets the curve near the end (the tracer would follow the leader) [deg].
TRACE_LIMITS = {("13b", "pitch"): (-20, 10), ("11a", "lat"): (-20, 15),
                ("12a", "pedal"): (-15, 20), ("13a", "pedal"): (-13, 20)}
TRACE_STEP = 3  # [px] the most the curve may move between neighbouring columns
TRACE_BEND = 0.01  # penalty per px² of that move, against the line's darkness (about 0.5-1)
WEAK = 0.12  # ring contrast below which a circle is flagged `weak`
# The 60 kt run near zero sideslip, whose sideslip is in question (`beta_open`).
BETA_OPEN = {("10", "stab", 4), ("11a", "roll", 5), ("11a", "lat", 4), ("11a", "pedal", 4),
             ("11b", "pitch", 5), ("11b", "lon", 4), ("11b", "coll", 4)}
SCALE = 2548.0 / 1686.0  # rough positions were read on a 1686-px-wide render

SOURCE = ("Kaplita, T. T., UH-60 Black Hawk Engineering Simulation Model Validation and "
          "Proposed Modifications, NASA CR-177360 (USAAVSCOM TR 85-A-2, 1985), Figs. 10-13 "
          "(pp. 57-63, PDF pp. 71-77): USAAEFA Project 79-24 flight test, steady-heading "
          "sideslips")

# Conditions printed on each figure (low-confidence digits: see DATA_GAPS.md).
FLIGHTS = {
    "60": "60 kt; flight 13, runs 28-40; GW 15500 lb, CG FS 352.0 in, WL 231.9 in, "
          "Hd 5500 or 5900 ft (unclear)",
    "100": "100 kt; flight 17, runs 18-27; GW 16000 lb, CG FS 348.7 in, WL 231.6 in, Hd 5700 ft",
    "140": "140 kt; flight 19, runs 21-26; GW 15750 lb, CG FS 351.2 in, WL 231.8 or 231.9 in "
           "(unclear), Hd 5750 ft",
}

# Pages: PDF page, figure, airspeed, sideslip axis labels as (display x, deg).
PAGES = {
    71: ("10", "60", [(762, -20), (1065, 0), (1365, 20)]),
    72: ("11a", "60", [(762, -20), (1060, 0), (1365, 20)]),
    73: ("11b", "60", [(762, -20), (1065, 0), (1362, 20)]),
    74: ("12a", "100", [(755, -20), (1050, 0), (1358, 20)]),
    75: ("12b", "100", [(760, -20), (1058, 0), (1360, 20)]),
    76: ("13a", "140", [(760, -20), (1055, 0), (1360, 20)]),
    77: ("13b", "140", [(755, -20), (1053, 0), (1360, 20)]),
}

# Panels: (page, id, quantity, unit, y labels as (display y, value), rough circle centres
# as display (x, y), the indices of those placed on a curve). Display coordinates are
# those of a 1686 x 2184 render of the upright page; page px = display x SCALE.
P = [
    (71, "stab", "stabilator_incidence", "deg (+ trailing edge down)",
     [(1468, 40), (1543, 30), (1620, 20), (1695, 10), (1770, 0), (1845, -10)],
     [(770, 1535), (840, 1535), (918, 1535), (983, 1535), (1075, 1535), (1140, 1535),
      (1213, 1535), (1296, 1535), (1376, 1535)], ()),
    (72, "roll", "roll_attitude", "deg (+ right)",
     [(570, 4), (648, 2), (720, 0), (793, -2), (868, -4), (942, -6)],
     [(775, 802), (840, 768), (840, 802), (920, 720), (983, 738), (1060, 730), (1137, 720),
      (1213, 673), (1296, 650), (1378, 640)], ()),
    (72, "lat", "lateral_stick", "% (100 = full right)",
     [(1018, 100), (1095, 80), (1170, 60), (1245, 40), (1320, 20), (1395, 0)],
     [(770, 1240), (838, 1240), (918, 1232), (983, 1228), (1060, 1220), (1137, 1213),
      (1213, 1208), (1296, 1200), (1378, 1193)], (4, 5, 7)),
    (72, "pedal", "pedal", "% (100 = full right)",
     [(1468, 100), (1545, 80), (1620, 60), (1695, 40), (1770, 20), (1845, 0)],
     [(768, 1621), (834, 1632), (913, 1648), (976, 1661), (1052, 1684), (1133, 1702),
      (1206, 1717), (1289, 1732), (1370, 1755)], ()),
    (73, "pitch", "pitch_attitude", "deg (+ nose up)",
     [(568, 4), (643, 2), (716, 0), (790, -2), (866, -4), (940, -6)],
     [(775, 766), (840, 740), (840, 755), (920, 733), (983, 740), (1078, 748), (1140, 717),
      (1215, 712), (1298, 706), (1380, 724)], (5,)),
    (73, "lon", "longitudinal_stick", "% (0 = full forward)",
     [(1015, 0), (1092, 20), (1168, 40), (1242, 60), (1317, 80), (1392, 100)],
     [(770, 1232), (840, 1225), (920, 1215), (983, 1212), (1076, 1200), (1140, 1183),
      (1218, 1176), (1298, 1168), (1378, 1170)], ()),
    (73, "coll", "collective_stick", "% (100 = full up)",
     [(1462, 100), (1543, 80), (1620, 60), (1690, 40), (1770, 20), (1845, 0)],
     [(770, 1688), (840, 1688), (918, 1688), (983, 1688), (1070, 1688), (1138, 1688),
      (1213, 1688), (1295, 1688), (1375, 1688)], ()),
    (74, "roll", "roll_attitude", "deg (+ right)",
     [(503, 15), (575, 10), (650, 5), (722, 0), (797, -5), (872, -10), (948, -15)],
     [(770, 818), (828, 785), (898, 752), (983, 745), (1075, 697), (1143, 676),
      (1213, 655), (1290, 650)], (3,)),
    (74, "lat", "lateral_stick", "% (100 = full right)",
     [(1018, 100), (1092, 80), (1168, 60), (1245, 40), (1320, 20), (1397, 0)],
     [(770, 1220), (830, 1213), (898, 1208), (978, 1205), (1072, 1200), (1143, 1188),
      (1213, 1178), (1288, 1165)], (1, 4)),
    (74, "pedal", "pedal", "% (100 = full right)",
     [(1470, 100), (1545, 80), (1620, 60), (1695, 40), (1770, 20), (1845, 0)],
     [(770, 1545), (828, 1568), (893, 1595), (976, 1622), (1070, 1660), (1138, 1703),
      (1210, 1733), (1283, 1765)], (3, 4)),
    (75, "pitch", "pitch_attitude", "deg (+ nose up)",
     [(583, 2), (658, 0), (733, -2), (808, -4), (883, -6), (957, -8)],
     [(773, 717), (828, 725), (898, 722), (983, 670), (1062, 642), (1075, 635), (1148, 628),
      (1215, 583), (1290, 588)], (2, 6)),
    (75, "lon", "longitudinal_stick", "% (0 = full forward)",
     [(1035, 0), (1110, 20), (1185, 40), (1260, 60), (1335, 80), (1408, 100)],
     [(770, 1245), (828, 1240), (895, 1225), (983, 1210), (1060, 1190), (1075, 1178),
      (1148, 1155), (1213, 1145), (1290, 1138)], (0, 1, 2, 3, 4)),
    (75, "coll", "collective_stick", "% (100 = full up)",
     [(1485, 100), (1560, 80), (1635, 60), (1710, 40), (1785, 20), (1860, 0)],
     [(773, 1683), (828, 1683), (898, 1683), (983, 1683), (1062, 1683), (1075, 1683),
      (1143, 1683), (1213, 1683), (1290, 1683)], ()),
    (76, "roll", "roll_attitude", "deg (+ right)",
     [(498, 15), (573, 10), (648, 5), (722, 0), (797, -5), (870, -10), (945, -15)],
     [(840, 912), (920, 802), (990, 780), (1075, 700), (1150, 668), (1220, 605),
      (1298, 527)], (2, 3, 5, 6)),
    (76, "lat", "lateral_stick", "% (100 = full right)",
     [(1020, 100), (1095, 80), (1170, 60), (1245, 40), (1320, 20), (1397, 0)],
     [(843, 1195), (918, 1190), (988, 1188), (1070, 1178), (1150, 1170), (1218, 1158)],
     (3, 4)),
    (76, "pedal", "pedal", "% (100 = full right)",
     [(1470, 100), (1545, 80), (1620, 60), (1695, 40), (1770, 20), (1846, 0)],
     [(840, 1505), (918, 1548), (985, 1600), (1068, 1650), (1143, 1702), (1213, 1748)],
     (0, 1, 2, 3, 4, 5)),
    (77, "pitch", "pitch_attitude", "deg (+ nose up)",
     [(578, 2), (650, 0), (725, -2), (800, -4), (875, -6), (950, -8)],
     [(830, 808), (908, 870), (978, 825), (1062, 757), (1140, 730), (1210, 697),
      (1290, 718)], (1, 4)),
    (77, "lon", "longitudinal_stick", "% (0 = full forward)",
     [(1023, 0), (1100, 20), (1175, 40), (1250, 60), (1327, 80), (1402, 100)],
     [(830, 1232), (908, 1215), (980, 1190), (1062, 1157), (1140, 1127), (1210, 1115),
      (1288, 1100)], (1, 2, 3, 4, 6)),
    (77, "coll", "collective_stick", "% (100 = full up)",
     [(1472, 100), (1548, 80), (1625, 60), (1700, 40), (1775, 20), (1850, 0)],
     [(830, 1598), (905, 1598), (978, 1598), (1062, 1598), (1138, 1598), (1210, 1598),
      (1290, 1598)], ()),
]


def load(doc, n):
    """Page n (1-based), upright, 300 dpi grey in [0, 1] (1 = ink)."""
    xref = doc[n - 1].get_images(full=True)[0][0]
    pix = fitz.Pixmap(doc, xref)
    a = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width)
    a = np.rot90(a, k=-1).astype(float)
    h, w = (a.shape[0] // 2) * 2, (a.shape[1] // 2) * 2
    a = a[:h, :w]
    g = (a[0::2, 0::2] + a[1::2, 0::2] + a[0::2, 1::2] + a[1::2, 1::2]) / 4
    return 1.0 - g / 255.0


def grid_period(ink, axis):
    """Fine gridline spacing [px] along `axis` (0: rows, 1: columns), from the spectrum."""
    p = ink[600:2900, 1100:2100].mean(axis=1 - axis)
    s = np.abs(np.fft.rfft(p - p.mean()))
    f = np.fft.rfftfreq(len(p))
    band = (f > 1 / 14) & (f < 1 / 9)
    return 1.0 / f[band][np.argmax(s[band])]


def degrid(ink):
    """The page without the graph-paper grid, blurred (sigma 2 px)."""
    py, px = grid_period(ink, 0), grid_period(ink, 1)
    F = np.fft.fft2(ink)
    H, W = ink.shape
    fy = np.fft.fftfreq(H)[:, None]
    fx = np.fft.fftfreq(W)[None, :]
    m = np.ones(ink.shape)
    for (qy, qx), (w, band) in (((py, px), (0.006, 0.02)), ((10 * py, 10 * px), (0.0015, 0.004))):
        for k in range(1, 400):
            if k / qy <= 0.5:
                m[(np.abs(np.abs(fy) - k / qy) < w) & (np.abs(fx) < band)] = 0
            if k / qx <= 0.5:
                m[(np.abs(np.abs(fx) - k / qx) < w) & (np.abs(fy) < band)] = 0
    m *= np.exp(-2 * np.pi**2 * 4.0 * (fx**2 + fy**2))
    return np.real(np.fft.ifft2(F * m))


def box(p, n):
    return np.convolve(p, np.ones(n) / n, mode="same")


def calibrate(ink, labels, axis, lo, hi, what):
    """Linear pixel -> value map through the major gridlines at the labels. The labels sit
    on evenly spaced major lines, so the offset and spacing are fitted jointly: the pair
    (within 25 px and 4 % of the rough positions) whose lines are darkest on the 5-px
    smoothed line profile (averaged over [lo, hi) of the other coordinate), where a major
    line, slightly wider than the fine ones, scores highest. Each line is then refined to
    its profile peak within 3 px. Stops if one is then more than 3 px off the fit."""
    est = np.array([d * SCALE for d, _ in labels])
    v = np.array([val for _, val in labels], float)
    k = (v - v[0]) / (v[1] - v[0])  # label index steps
    step = np.polyfit(k, est, 1)[0]
    if axis == 0:
        prof = ink[:, lo:hi].mean(axis=1)
    else:
        prof = ink[lo:hi, :].mean(axis=0)
    s5 = box(prof, 5)
    best = None
    for d in np.arange(step * 0.96, step * 1.04, 0.1):
        for y0 in np.arange(est[0] - 25, est[0] + 25.5, 0.5):
            sc = s5[np.round(y0 + k * d).astype(int)].mean()
            if best is None or sc > best[0]:
                best = (sc, d, y0)
    _, d, y0 = best
    px = np.array([q - 3 + int(np.argmax(s5[q - 3:q + 4]))
                   for q in np.round(y0 + k * d).astype(int)], float)
    a, b = np.polyfit(px, v, 1)
    res = px - (v - b) / a
    if np.max(np.abs(res)) > 3.0:
        raise SystemExit(f"{what}: label lines {px} off a straight scale by {res} px")
    return a, b, px


TH = np.linspace(0, 2 * np.pi, 90, endpoint=False)


def ring(g, cx, cy, r):
    xs = np.round(cx + r * np.cos(TH)).astype(int)
    ys = np.round(cy + r * np.sin(TH)).astype(int)
    return g[ys, xs].mean()


def fit(g, x0, y0, search=15):
    best = None
    for cy in range(y0 - search, y0 + search + 1):
        for cx in range(x0 - search, x0 + search + 1):
            for r in range(7, 16):
                s = ring(g, cx, cy, r) - 0.5 * (ring(g, cx, cy, r - 3) + ring(g, cx, cy, r + 3))
                if best is None or s > best[3]:
                    best = (cx, cy, r, s)
    return best


def overlay(ink, xs_px, ys_px, pts, title, path, curve=None):
    """The panel at 300 dpi with the calibration lines and readings drawn (`path`), the
    same crop unmarked (`<path>_scan.png`) for comparison, and the crop's box and the
    readings in its pixels (`<path>.json`), for a review page."""
    xs = [int(c[0]) for c in pts]
    ys = [int(c[1]) for c in pts]
    x0, x1 = max(0, min(xs_px) - 60), max(xs_px) + 60
    y0 = max(0, min(min(ys), int(min(ys_px))) - 80)
    y1 = max(max(ys), int(max(ys_px))) + 80
    scan = Image.fromarray(((1.0 - ink[y0:y1, x0:x1]) * 255).astype(np.uint8)).convert("RGB")
    scan.save(path.with_name(path.stem + "_scan.png"))
    im = scan.copy()
    dr = ImageDraw.Draw(im)
    w, h = im.size
    for X in xs_px:
        dr.line([(X - x0, 0), (X - x0, h)], fill=(0, 170, 220), width=1)
    for Y in ys_px:
        dr.line([(0, Y - y0), (w, Y - y0)], fill=(0, 170, 220), width=1)
    for k, (cx, cy, r, _, _) in enumerate(pts):
        X, Y = cx - x0, cy - y0
        dr.ellipse([X - r - 4, Y - r - 4, X + r + 4, Y + r + 4], outline=(230, 0, 0), width=2)
        dr.line([(X - 4, Y), (X + 4, Y)], fill=(230, 0, 0), width=1)
        dr.line([(X, Y - 4), (X, Y + 4)], fill=(230, 0, 0), width=1)
        dr.text((X + r + 6, Y - r - 14), str(k), fill=(230, 0, 0))
    if curve is not None:
        cx0, cy = curve
        dr.line([(cx0 + i - x0, y - y0) for i, y in enumerate(cy)], fill=(0, 150, 60), width=1)
    dr.text((6, 4), title, fill=(230, 0, 0))
    im.save(path)
    path.with_suffix(".json").write_text(json.dumps(default=int, obj={
        "box": [int(x0), int(y0), int(x1), int(y1)],
        "calibration_x": [int(v) - x0 for v in xs_px],
        "calibration_y": [int(v) - y0 for v in ys_px],
        "points": [{"x": int(cx) - x0, "y": int(cy) - y0, "r": int(r)}
                   for cx, cy, r, _, _ in pts],
        "curve": None if curve is None else
                 [[curve[0] + i - x0, round(float(y) - y0, 1)] for i, y in enumerate(curve[1])],
    }))


def trace(g, x0, x1, y0, y1, circles):
    """The darkest continuous path from column x0 to x1 within rows [y0, y1) of the
    degridded page g: dynamic programming over columns, moving at most TRACE_STEP px per
    column with a penalty TRACE_BEND per px² of move. The band's median (the filter's
    constant background) is taken out first; then the test circles (cx, cy, r) and the long
    horizontal lines (the axis and zero lines: rows whose median across the range stands
    above the background, as no curve's does) are blanked to background. Returns the
    path's row at each column x0..x1. Where the solid and dashed lines run close it can
    jump between them; the reviewer's hand-drawn line (HAND_LINES) then replaces it."""
    band = g[y0:y1, :].copy()
    band -= np.median(band[:, int(x0):int(x1) + 1])  # the filter leaves a constant background
    rows_median = np.median(band[:, int(x0):int(x1) + 1], axis=1)
    band[rows_median > 0.1, :] = 0.0
    yy, xx = np.mgrid[y0:y1, 0:g.shape[1]]
    for cx, cy, r in circles:
        d = np.hypot(xx - cx, yy - cy)
        band[d < r + 5] = 0.0
    cols = range(int(x0), int(x1) + 1)
    n = y1 - y0
    cost = -band[:, int(x0)].copy()
    back = []
    steps = np.arange(-TRACE_STEP, TRACE_STEP + 1)
    for x in list(cols)[1:]:
        best = np.full(n, np.inf)
        arg = np.zeros(n, dtype=int)
        for d in steps:
            shifted = np.full(n, np.inf)
            if d >= 0:
                shifted[d:] = cost[:n - d]
            else:
                shifted[:n + d] = cost[-d:]
            c = shifted + TRACE_BEND * d * d
            better = c < best
            best[better] = c[better]
            arg[better] = d
        cost = best - band[:, x]
        back.append(arg)
    path = [int(np.argmin(cost))]
    for arg in reversed(back):
        path.append(path[-1] - arg[path[-1]])
    path.reverse()
    return np.array(path, dtype=float) + y0


def hand_lines():
    """The reviewer's hand-drawn GenHel curves (HAND_LINES): (figure, panel) -> (xs, ys)
    [page px at 300 dpi], left to right."""
    out = {}
    if not HAND_LINES.exists():
        return out
    with HAND_LINES.open(encoding="utf-8") as f:
        for r in csv.DictReader(line for line in f if not line.startswith("#")):
            out.setdefault((r["figure"], r["panel"]), []).append(
                (float(r["x_px"]), float(r["y_px"])))
    return {k: (np.array([p[0] for p in sorted(v)]), np.array([p[1] for p in sorted(v)]))
            for k, v in out.items()}


def hand_edits():
    """The reviewer's hand readings (HAND): (figure, panel, point) -> (x, y) [page px at
    300 dpi], or "none" where the reviewer found no test point."""
    out = {}
    if not HAND.exists():
        return out
    with HAND.open(encoding="utf-8") as f:
        for r in csv.DictReader(line for line in f if not line.startswith("#")):
            key = (r["figure"], r["panel"], int(r["point"]))
            out[key] = "none" if r["x_px"] == "none" else (float(r["x_px"]), float(r["y_px"]))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("pdf", type=Path, help="19870008277.pdf (NASA CR-177360)")
    ap.add_argument("overlays", type=Path, help="directory for the review overlays")
    args = ap.parse_args()
    args.overlays.mkdir(parents=True, exist_ok=True)
    doc = fitz.open(args.pdf)
    rows, review, curves = [], [], []
    hand = hand_edits()
    drawn = hand_lines()
    pages = {}
    for page, pid, qty, unit, ylabels, rough, on_curve in P:
        fig, kt, blabels = PAGES[page]
        if page not in pages:
            ink = load(doc, page)
            ys = [int(round(d[1] * SCALE)) for q in P if q[0] == page for d in q[5]]
            bx = calibrate(ink, blabels, 1, min(ys) - 100, max(ys) + 100,
                           f"Fig. {fig} sideslip")
            pages[page] = (ink, degrid(ink), bx)
            print(f"Fig. {fig}: sideslip lines at {bx[2]} px")
        ink, g, (ba, bb, bpx) = pages[page]
        ya, yb, ypx = calibrate(ink, ylabels, 0, 1150, 2100, f"Fig. {fig} {pid}")
        pts = []
        for k, (x, y) in enumerate(rough):
            cx, cy, r, s = fit(g, int(round(x * SCALE)), int(round(y * SCALE)))
            flag = "on_curve" if k in on_curve else ""
            if s < WEAK:
                flag = (flag + " weak").strip()
            edit = hand.get((fig, pid, k))
            if edit == "none":
                review.append(f"Fig. {fig} {pid} #{k}: dropped (no test point, reviewer)")
                continue
            if edit is not None:
                cx, cy, flag = edit[0], edit[1], "hand"
            if (fig, pid, k) in BETA_OPEN:
                flag = (flag + " beta_open").strip()
            beta, val = ba * cx + bb, ya * cy + yb
            pts.append((cx, cy, r, beta, val))
            rows.append(dict(figure=fig, panel=pid, quantity=qty, unit=unit, airspeed_kt=kt,
                             point=k, sideslip_deg=round(beta, 2), value=round(val, 3),
                             x_px=round(cx, 1), y_px=round(cy, 1), flag=flag))
            if flag and flag not in ("hand", "beta_open", "hand beta_open"):
                review.append(f"Fig. {fig} {pid} #{k}: {flag} (beta {beta:.2f}, {val:.2f})")
        curve = None
        if pid in TRACED:
            x0 = int(min(p_[0] for p_ in pts))
            x1 = int(max(p_[0] for p_ in pts))
            if (fig, pid) in TRACE_LIMITS:
                lo, hi = TRACE_LIMITS[(fig, pid)]
                x0 = max(x0, int(np.ceil((lo - bb) / ba)))
                x1 = min(x1, int(np.floor((hi - bb) / ba)))
            y0 = int(min(ypx)) - 5
            y1 = int(max(ypx)) + 5
            cy = trace(g, x0, x1, y0, y1, [(p_[0], p_[1], p_[2]) for p_ in pts])
            source = np.full(len(cy), "trace", dtype=object)
            if (fig, pid) in drawn:
                # the reviewer's line replaces the trace over its range, which may be wider
                hx, hy = drawn[(fig, pid)]
                lo, hi = min(x0, int(np.ceil(hx[0]))), max(x1, int(np.floor(hx[-1])))
                xs = np.arange(lo, hi + 1)
                old = np.interp(xs, np.arange(x0, x1 + 1), cy, left=np.nan, right=np.nan)
                inside = (xs >= hx[0]) & (xs <= hx[-1])
                cy = np.where(inside, np.interp(xs, hx, hy), old)
                source = np.where(inside, "hand", "trace")
                keep = ~np.isnan(cy)
                xs, cy, source = xs[keep], cy[keep], source[keep]
                x0, x1 = int(xs[0]), int(xs[-1])
            curve = (x0, cy)
            for b in range(int(np.ceil(ba * x0 + bb)), int(np.floor(ba * x1 + bb)) + 1):
                xb = (b - bb) / ba
                yb_ = float(np.interp(xb, np.arange(x0, x1 + 1), cy))
                curves.append(dict(figure=fig, panel=pid, quantity=qty, unit=unit,
                                   airspeed_kt=kt, series="genhel_existing", sideslip_deg=b,
                                   value=round(ya * yb_ + yb, 3), x_px=round(xb, 1),
                                   y_px=round(yb_, 1),
                                   source=source[min(len(source) - 1, max(0, int(round(xb)) - x0))]))
        overlay(ink, list(bpx.astype(int)), list(ypx.astype(int)), pts,
                f"Fig. {fig} {qty} [{unit}], {kt} kt", args.overlays / f"kaplita_{fig}_{pid}.png",
                curve)
        print(f"Fig. {fig} {pid}: " + ", ".join(f"({p[3]:.1f}, {p[4]:.2f})" for p in pts))
    # cross-panel check: the same runs at the same sideslips in every panel of a flight
    for kt in FLIGHTS:
        betas = [r["sideslip_deg"] for r in rows if r["airspeed_kt"] == kt]
        for r in rows:
            if r["airspeed_kt"] != kt:
                continue
            near = [b for b in betas if abs(b - r["sideslip_deg"]) < 2.5]
            med = float(np.median(near))
            if abs(r["sideslip_deg"] - med) > 0.6:
                review.append(f"Fig. {r['figure']} {r['panel']} #{r['point']}: sideslip "
                              f"{r['sideslip_deg']:.2f} against its run's {med:.2f}")
    with OUT.open("w", newline="", encoding="utf-8") as f:
        f.write(f"# source: {SOURCE}. Digitized from the NASA scan by "
                "tools/digitize_kaplita.py.\n")
        f.write("# status: VERIFIED by human review of the overlays (2026-10-20; DATA_GAPS.md), "
                "11 centres placed by the reviewer (hand); beta_open readings are placeholders\n")
        for kt, text in FLIGHTS.items():
            f.write(f"# {text}\n")
        f.write("# Flight-test circles only. x_px, y_px: centre in the upright 300 dpi page. "
                "Flags: on_curve, weak, hand, beta_open (tool docstring).\n")
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with CURVES.open("w", newline="", encoding="utf-8") as f:
        f.write(f"# source: {SOURCE}. Sikorsky's GenHel ('existing model', the solid curve; "
                "Kaplita p. 2: stabilator fixed at its calculated value at zero sideslip, SAS on, "
                "PBA disabled, FPS and trim off), traced from the NASA scan by "
                "tools/digitize_kaplita.py over the test points' sideslip range, sampled every "
                "degree.\n")
        f.write("# status: VERIFIED by human review (2026-10-20; DATA_GAPS.md): the reviewer "
                "redrew every curve by hand (source hand); trace samples, if any, are the tool's\n")
        f.write("# x_px, y_px: the line in the upright 300 dpi page. source: hand (the "
                "reviewer's line, kaplita_genhel_hand_lines.csv) or trace (the tool's).\n")
        w = csv.DictWriter(f, fieldnames=list(curves[0].keys()))
        w.writeheader()
        w.writerows(curves)
    (args.overlays / "review.txt").write_text("\n".join(review) + "\n", encoding="utf-8")
    print(f"{len(rows)} readings; {len(review)} listed in {args.overlays / 'review.txt'}")


if __name__ == "__main__":
    main()
