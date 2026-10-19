"""Digitize Ballin, NASA TM-88360 (1987), App. D and E transient responses (D-046).

    .venv/bin/python -I tools/digitize_ballin_d.py <NASA-TM-88360.pdf> <overlay dir> [--only D1]

Writes data/uh60a/validation/ballin_d/<fig>.csv (one file per figure, D1-D17, E1-E3) and, in
<overlay dir>, one PNG per panel with the readings drawn over the scan, for human review
(DATA_GAPS.md). Readings need a person's check of the overlays before use; a change to the
reading code needs a new review (REVIEWED).

Each figure is eight pages of four framed panels, in the same order in every figure (page:
panels): 0 the four controls; 1 pitch attitude, rate, acceleration, longitudinal cyclic; 2
roll; 3 heading, yaw rate, acceleration, pedals; 4 velocities; 5 angle of attack, sideslip,
blade lag and flap; 6 accelerations and shaft power; 7 rotor speed and the engine. This tool
reads the panels panels_of() lists: the controls (the inputs), the attitudes and rates, and
the vertical acceleration of the collective cases (page 6, panel 2).

How it reads a panel.
- Page geometry: a landscape page is turned a quarter turn, then the page is deskewed (the
  rotation that makes the frame lines sharpest), and turned half round if the tick labels
  are on the right. The frames are the four equal, equally spaced boxes whose edges best
  fit the rows inked across the frame width; each frame's edges are then refined on its
  own (the scans are slightly distorted).
- Calibration: the frame's left and right edges are 0 and 6.5 s. The y scale is fitted
  through the top and bottom edges and the inner major tick stubs, at the tick labels a
  person read off the scan into PANEL_KINDS and SCALES (the middle label must be the mean
  of the outer two); a stub more than 1 % of the frame height off the fit stops the panel.
- Curves (trace, chain): inside the frame, inset past the tick stubs, the ink is split into
  connected pieces. Long pieces are the solid curve (Ames GenHel); the dashes, chained left
  to right, the dashed curve (USAAEFA flight test); in App. E the long dashes, chained the
  same way, the dash-dotted curve (Sikorsky GenHel). Where the simulation's line swings at
  4/rev (the vertical accelerations) it is read as a band: its middle (ames_genhel) and
  the upper and lower edges of its swing (ames_genhel_upper, ames_genhel_lower).
- Hand edits (ballin_d_hand_edits.csv beside the readings): stretches the reviewer traced
  by hand on the review page's overlays, or marked unreadable, replace the tool's readings
  there (apply_hand).
Flags: `merged` (the dashed line can't be seen there and is taken to run along the other
curve), `gap` (interpolated across more than a dash gap, away from the other curve), `hand`
(traced by the reviewer), `rejected` (a series the reviewer did not accept). Not read (no rows): stretches off the scale, hidden in a
vibrating line's band, or marked unreadable. review.txt in the overlay directory lists the
gaps longer than 0.1 s, the stretches not read and the series whose readings don't span
the run, for review.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import fitz
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from digitize_ballin import runs  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "uh60a" / "validation" / "ballin_d"
DPI = 300
COARSE_DPI = 100  # for the page geometry
DARK = 140  # gray level below which a pixel is ink
T_END = 6.5  # [s] the frame's right edge

SOURCE = ("Ballin, M. G., Validation of a Real-Time Engineering Simulation of the UH-60A "
          "Helicopter, NASA TM-88360 (1987)")

# Figures: (first page, 1-based PDF page; Table C1 file; caption).
FIGURES = {
    "D1": (41, 201, "1-in. right cyclic input at hover"),
    "D2": (49, 202, "1-in. left cyclic input at hover"),
    "D3": (57, 204, "0.5-in. forward cyclic input at hover"),
    "D4": (65, 212, "1-in. up collective input at hover"),
    "D5": (73, 209, "1-in. left pedal input at hover"),
    "D6": (81, 504, "1-in. left cyclic input, 60 knots"),
    "D7": (89, 509, "1-in. aft cyclic input, 60 knots"),
    "D8": (97, 405, "0.5-in. down collective input, 60 knots, stabilator off nominal"),
    "D9": (105, 510, "0.5-in. up collective input, 60 knots"),
    "D10": (113, 512, "1-in. down collective input, 60 knots"),
    "D11": (121, 502, "0.5-in. right pedal input, 60 knots"),
    "D12": (129, 503, "1-in. left pedal input, 60 knots"),
    "D13": (137, 401, "1-in. aft cyclic input, 100 knots"),
    "D14": (145, 706, "1-in. forward cyclic input, 100 knots"),
    "D15": (153, 404, "0.5-in. up collective input, 100 knots"),
    "D16": (161, 102, "1-in. right pedal input, 100 knots"),
    "D17": (169, 103, "1-in. left pedal input, 100 knots"),
    "E1": (178, 205, "0.5-in. aft cyclic input at hover"),
    "E2": (186, 210, "0.5-in. down collective input at hover"),
    "E3": (194, 409, "1-in. right lateral input, 100 knots"),
}
COLLECTIVE = {"D4", "D8", "D9", "D10", "D15", "E2"}
# Panels where the simulation's line swings at 4/rev (the accelerations): read as a band.
VIBRATING = {(6, 2)}

# Panels: (page in figure, panel on page) -> (quantity, unit, default (top, middle, bottom)
# tick labels). The scales differ between figures; SCALES overrides the defaults.
PANEL_KINDS = {
    (0, 0): ("lateral_cyclic", "in", (5.0, 0.0, -5.0)),
    (0, 1): ("longitudinal_cyclic", "in", (5.0, 0.0, -5.0)),
    (0, 2): ("collective", "in", (10.0, 5.0, 0.0)),
    (0, 3): ("pedals", "in", (3.0, 0.0, -3.0)),
    (1, 0): ("pitch_attitude", "deg", (50.0, 0.0, -50.0)),
    (1, 1): ("pitch_rate", "deg/s", (20.0, 0.0, -20.0)),
    (2, 0): ("roll_attitude", "deg", (50.0, 0.0, -50.0)),
    (2, 1): ("roll_rate", "deg/s", (20.0, 0.0, -20.0)),
    (3, 0): ("heading", "deg", None),
    (3, 1): ("yaw_rate", "deg/s", (20.0, 0.0, -20.0)),
    (6, 2): ("vertical_acceleration_cg_sensor", "ft/s^2", None),
}

# Tick labels read off the scan (top, middle, bottom), where they differ from the defaults.
SCALES = {
    "D1": {(3, 0): (20.0, 5.0, -10.0)},
    "D2": {(2, 1): (40.0, 0.0, -40.0), (3, 0): (10.0, -5.0, -20.0)},
    "D3": {(3, 0): (20.0, 5.0, -10.0)},
    "D4": {(3, 0): (40.0, 15.0, -10.0), (6, 2): (-31.0, -35.5, -40.0)},
    "D5": {(3, 0): (10.0, -40.0, -90.0), (3, 1): (40.0, 0.0, -40.0)},
    "D6": {(3, 0): (10.0, 0.0, -10.0)},
    "D7": {(1, 1): (40.0, 0.0, -40.0), (3, 0): (3.0, 1.0, -1.0)},
    "D8": {(3, 0): (2.0, -1.0, -4.0), (6, 2): (-23.0, -29.0, -35.0)},
    "D9": {(3, 0): (5.0, 1.5, -2.0), (6, 2): (-31.0, -35.0, -39.0)},
    "D10": {(3, 0): (6.0, 0.5, -5.0), (6, 2): (-20.0, -30.0, -40.0)},
    "D11": {(3, 0): (30.0, 10.0, -10.0)},
    "D12": {(3, 0): (10.0, -5.0, -20.0)},
    "D13": {(1, 1): (40.0, 0.0, -40.0), (2, 0): (80.0, 0.0, -80.0),
            (3, 0): (40.0, 15.0, -10.0), (3, 1): (40.0, 0.0, -40.0)},
    "D14": {(3, 0): (5.0, 2.0, -1.0)},
    "D15": {(3, 0): (2.0, -1.5, -5.0), (6, 2): (-10.0, -35.0, -60.0)},
    "D16": {(1, 1): (40.0, 0.0, -40.0), (2, 1): (40.0, 0.0, -40.0),
            (3, 0): (30.0, 10.0, -10.0)},
    "D17": {(2, 1): (40.0, 0.0, -40.0), (3, 0): (10.0, -15.0, -40.0)},
    "E1": {(3, 0): (10.0, 0.0, -10.0)},
    "E2": {(3, 0): (10.0, -25.0, -60.0), (6, 2): (-25.0, -29.5, -34.0)},
    "E3": {(2, 0): (80.0, 0.0, -80.0), (2, 1): (40.0, 0.0, -40.0),
           (3, 0): (20.0, 5.0, -10.0)},
}

BASE_PANELS = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 0), (1, 1), (2, 0), (2, 1), (3, 0), (3, 1)]


def panels_of(fig):
    """[(page in figure, panel, quantity, unit, (top, middle, bottom))] read for a figure."""
    keys = BASE_PANELS + ([(6, 2)] if fig in COLLECTIVE else [])
    out = []
    for key in keys:
        qty, unit, scale = PANEL_KINDS[key]
        scale = SCALES.get(fig, {}).get(key, scale)
        if scale is None:
            continue  # scale not read yet
        out.append((key[0], key[1], qty, unit, scale))
    return out


class PanelError(Exception):
    pass


# ---- Page geometry -------------------------------------------------------------------------

def render(doc, page, rot, dpi):
    z = dpi / 72.0
    pix = doc[page - 1].get_pixmap(matrix=fitz.Matrix(z, z).prerotate(rot),
                                   colorspace=fitz.csGRAY)
    return np.frombuffer(pix.samples, np.uint8).reshape(pix.height, pix.width).copy()


def sharpness(dark):
    p = dark.sum(axis=1).astype(float)
    return float((p ** 2).sum())


def deskew(doc, page, base):
    """The rotation near `base` [deg] that makes the horizontal frame lines sharpest."""
    best = (base, -1.0)
    for a in np.arange(-4.0, 4.01, 0.25):
        s = sharpness(render(doc, page, base + a, COARSE_DPI) < DARK)
        if s > best[1]:
            best = (base + a, s)
    a0 = best[0]
    for a in np.arange(a0 - 0.2, a0 + 0.21, 0.05):
        s = sharpness(render(doc, page, a, COARSE_DPI) < DARK)
        if s > best[1]:
            best = (float(a), s)
    return best[0]


def frame_sides(dark):
    """x of the frames' left and right edges: the pair of strong vertical lines at least
    40 % of the width apart with the most ink."""
    H, W = dark.shape
    col = dark.sum(axis=0).astype(float)
    cands = [x for x in range(2, W - 2)
             if col[x] > 0.1 * H and col[x] >= col[x - 2:x + 3].max()]
    best = None
    for i, xa in enumerate(cands):
        for xb in cands[i + 1:]:
            if xb - xa > 0.4 * W:
                sc = col[xa] + col[xb]
                if best is None or sc > best[0]:
                    best = (sc, xa, xb)
    return (best[1], best[2]) if best else None


def frame_edges(dark, xa, xb):
    """Rows inked over at least 90 % of [xa, xb]: the frames' top and bottom edges, and any
    curve running flat across the whole frame."""
    d = dark.copy()
    d[1:] |= dark[:-1]
    d[:-1] |= dark[1:]
    frac = d[:, xa + 3:xb - 2].mean(axis=1)
    groups = []
    for y in np.flatnonzero(frac > 0.9):
        if groups and y - groups[-1][-1] <= 3:
            groups[-1].append(y)
        else:
            groups.append([y])
    return [float(np.mean(g)) for g in groups]


def lattice(edges, n=4, tol=4.0):
    """The n frames [(top, bottom)] whose edges best fit equal heights and spacing, and how
    many of their 2n edges were found."""
    best = None
    for t0 in edges:
        for t1 in edges:
            if t1 <= t0:
                continue
            pitch = t1 - t0
            for b0 in edges:
                h = b0 - t0
                if not 0.6 * pitch < h < 0.95 * pitch:
                    continue
                want = [t0 + k * pitch + d for k in range(n) for d in (0.0, h)]
                hit = [min(edges, key=lambda e: abs(e - w)) for w in want]
                sc = sum(abs(e - w) <= tol for e, w in zip(hit, want))
                if best is None or sc > best[0]:
                    best = (sc, [(hit[2 * k], hit[2 * k + 1]) for k in range(n)])
    return best


def page_geometry(doc, page):
    """(rotation [deg], [((left, right), (top, bottom))] for the four frames [px at DPI],
    the page image at DPI)."""
    pg = doc[page - 1]
    base = 90.0 if pg.rect.width > pg.rect.height else 0.0
    rot = deskew(doc, page, base)
    dark = render(doc, page, rot, COARSE_DPI) < DARK
    sides = frame_sides(dark)
    if sides is None:
        raise PanelError(f"p{page}: no frames found")
    xl, xr = sides
    left = dark[:, max(0, xl - 60):xl - 5].sum()
    right = dark[:, xr + 5:xr + 60].sum()
    if right > left:  # the tick labels and titles are left of the frames: turn half round
        rot += 180.0
        dark = render(doc, page, rot, COARSE_DPI) < DARK
        sides = frame_sides(dark)
    score, frames = lattice(frame_edges(dark, *sides))
    if score < 7:
        raise PanelError(f"p{page}: only {score} of 8 frame edges found")
    s = DPI / COARSE_DPI
    # Refine each frame at full resolution: each edge is the centre of the strongest line
    # within 12 px (4 coarse px) of the coarse one, along that frame only (the scans are
    # slightly distorted, so the frames on a page don't share exact sides).
    img = render(doc, page, rot, DPI)
    dk = img < DARK

    def centre(profile, guess, r=12):
        w = profile[guess - r:guess + r + 1].astype(float)
        hits = np.flatnonzero(w >= 0.6 * w.max())
        return guess - r + float(np.mean(hits))

    fine = []
    for t, b in frames:
        xa, xb, ft, fb = [int(round(v * s)) for v in (sides[0], sides[1], t, b)]
        col = dk[ft + 20:fb - 20].sum(axis=0)
        xa_, xb_ = centre(col, xa), centre(col, xb)
        row = dk[:, int(xa_) + 20:int(xb_) - 20].sum(axis=1)
        fine.append(((xa_, xb_), (centre(row, ft), centre(row, fb))))
    return rot, fine, img


# ---- One panel -----------------------------------------------------------------------------

INSET = 18  # [px at 300 dpi] past the frame line and its inward tick stubs


def y_scale(img, xb, top, bot, yt, yb):
    """Linear pixel -> value map for y, fitted through the frame's top and bottom edges
    (the top and bottom labels) and the three inner major tick stubs (the labels between,
    at the quarter points). The stubs are read on the right side: the curves end at 6 s,
    clear of it (on the left a curve can start on a stub's row). Some stubs are too faint
    to find; at least two of the three must be. The scans are slightly distorted: a stub can
    sit up to about 1 % of the frame height off where the edges put it, so the tolerance on
    the fit is the larger of 3 px and 1 % of the frame height, as digitize_ballin's.
    Returns (coefficients, largest residual [px])."""
    faint = img < 190
    xr = int(round(xb))
    pts = [(top, yt), (bot, yb)]
    for k in range(1, 4):
        y = top + (bot - top) * k / 4.0
        lo, hi = int(y) - 8, int(y) + 9
        prof = faint[lo:hi, xr - 16:xr - 3].sum(axis=1)
        if prof.max() < 4:
            continue
        hits = np.flatnonzero(prof >= 0.6 * prof.max())
        pts.append((lo + float(np.mean(hits)), yt + (yb - yt) * k / 4.0))
    if len(pts) < 4:
        raise PanelError(f"only {len(pts) - 2} of the 3 inner major tick stubs found")
    p = np.array(pts)
    A = np.vstack([p[:, 0], np.ones(len(p))]).T
    coef, *_ = np.linalg.lstsq(A, p[:, 1], rcond=None)
    res_px = float(np.abs((A @ coef - p[:, 1]) / coef[0]).max())
    if res_px > max(3.0, 0.01 * (bot - top)):
        raise PanelError(f"a tick is {res_px:.1f} px off the fitted scale")
    return coef, res_px


def components(ink):
    """8-connected components of a bool image: (label image, 0 for no ink; label count).
    Union-find over each row's runs."""
    H, W = ink.shape
    labels = np.zeros((H, W), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    prev = []
    for y in range(H):
        cur = []
        for s_, e_ in runs(ink[y]):
            lab = None
            for ps, pe, pl in prev:
                if ps <= e_ and s_ <= pe:  # touching, diagonals included
                    if lab is None:
                        lab = find(pl)
                    else:
                        ra, rb = find(lab), find(pl)
                        if ra != rb:
                            parent[max(ra, rb)] = min(ra, rb)
                            lab = min(ra, rb)
            if lab is None:
                parent.append(len(parent))
                lab = len(parent) - 1
            labels[y, s_:e_] = lab
            cur.append((s_, e_, lab))
        prev = cur
    lut = np.array([find(i) for i in range(len(parent))], np.int32)
    return lut[labels], len(parent)


SOLID_MIN = 80   # [px] x extent of a piece of the solid curve; the dashes are 15-45 px long
SPECK_MAX = 15   # [px] ink area of a speck
LINK_GAP = 220   # [px] longest gap bridged at a dash's slope (longer only under a curve)
DASH_GAP = 30    # [px] longest gap between two dashes of the dashed line
LONG_DASH = 24   # [px] dash length above which a dash is the dash-dot line's (App. E)
SLOPE_REACH = 40  # [px] how far a dash's slope is trusted to carry the curve on
EDGE = 2         # [px] a reading this close to the read area's edge is off the scale
VIB_PERIOD = 15  # [px] about one period of the simulation's 4/rev vibration (17 Hz)


def dash_pieces(lab, keep):
    """The pieces `keep` marks in a label image, as ({column: centre y}, length [px])."""
    pieces = []
    for k in np.flatnonzero(keep):
        ys, xs = np.nonzero(lab == k)
        cols = {}
        for x in range(xs.min(), xs.max() + 1):
            yy = ys[xs == x]
            if len(yy):
                cols[x] = float(yy.mean())
        pieces.append((cols, float(np.hypot(xs.max() - xs.min() + 1, ys.max() - ys.min() + 1))))
    return pieces


def classify(lab, n):
    """(solid, dash) masks over the labels: pieces SOLID_MIN columns wide or more are solid;
    the others, apart from specks, dashes."""
    H, W = lab.shape
    area = np.bincount(lab.ravel(), minlength=n)
    ys, xs = np.nonzero(lab)
    ls = lab[ys, xs]
    x0 = np.full(n, W); x1 = np.full(n, -1); y0 = np.full(n, H); y1 = np.full(n, -1)
    np.minimum.at(x0, ls, xs); np.maximum.at(x1, ls, xs)
    np.minimum.at(y0, ls, ys); np.maximum.at(y1, ls, ys)
    solid = np.zeros(n, bool)
    dash = np.zeros(n, bool)
    for k in range(1, n):
        if area[k] <= SPECK_MAX:
            continue
        if x1[k] - x0[k] + 1 >= SOLID_MIN:
            solid[k] = True
        elif max(x1[k] - x0[k], y1[k] - y0[k]) >= 6:
            dash[k] = True
    return solid, dash


def trace(ink, dash_dot=False, vibrating=False):
    """Follow the solid (simulation) and dashed (flight test) curves through a frame's
    interior. Returns {series: (y, flags)}, y per column [px from the top of the interior;
    NaN where not read]; with dash_dot (App. E) also the dash-dotted curve (Sikorsky
    GenHel), from the long dashes; with vibrating (the vertical accelerations, where the
    simulation's line swings at 4/rev) also the upper and lower edges of its swing.

    The ink is split into connected pieces. Pieces at least SOLID_MIN columns wide are the
    solid curve, the others, apart from specks, dashes.
    - The solid curve, column by column, is the run of its ink nearest its course. Its other
      runs are dashes touching it somewhere (a dashed line hugging the solid one): split
      off, they join the dashes.
    - Where the solid line is thicker than one line at its slope, a dash runs along it:
      the solid curve is the edge nearer its own course through the thin columns around,
      and the dash, the other edge, joins the dashes.
    - A vibrating solid curve is read as a band: its upper and lower edges, the extremes
      of its ink over about one vibration period, and its middle, their mean. The dashes
      touching it can't be told from it.
    - The dashed curve: see chain().
    """
    H, W = ink.shape
    lab, n = components(ink)
    solid, dash = classify(lab, n)
    if vibrating:
        return trace_band(lab, solid, dash, dash_dot)

    # The solid curve, column by column: the run of solid ink nearest the curve's course.
    sim = np.full(W, np.nan)
    band = [None] * W
    last = None
    residual = np.zeros((H, W), bool)  # solid ink off the curve: dashes touching it
    for x in range(W):
        rs = runs(solid[lab[:, x]])
        if not rs:
            continue
        r = rs[0] if last is None else min(rs, key=lambda r_: abs((r_[0] + r_[1] - 1) / 2 - last))
        band[x] = r
        sim[x] = last = (r[0] + r[1] - 1) / 2.0
        for s_, e_ in rs:
            if (s_, e_) != r:
                residual[s_:e_, x] = True
    widths = [b[1] - b[0] for b in band if b is not None]
    line_w = float(np.median(widths)) if widths else 4.0
    sim_f = np.array([""] * W, dtype=object)
    extra = []  # (column, y) of dashes running along the solid line
    thin = np.array([x for x in range(W) if band[x] is not None
                     and band[x][1] - band[x][0] <= line_w + 1])
    for x in range(W):
        if band[x] is None or len(thin) == 0:
            continue
        nb = thin[np.abs(thin - x) <= 30]
        if (nb < x).sum() < 3 or (nb > x).sum() < 3:
            continue
        c = np.polyfit(nb, sim[nb], 1)
        h = line_w * float(np.hypot(1.0, c[0]))
        s_, e_ = band[x]
        if e_ - s_ > h + 3:
            pred = np.polyval(c, x)
            top_c, bot_c = s_ + h / 2 - 0.5, e_ - h / 2 - 0.5
            near, far = (top_c, bot_c) if abs(top_c - pred) < abs(bot_c - pred) else (bot_c, top_c)
            sim[x] = near
            extra.append((x, far))

    # The dashes: the dash pieces, and the dashes split off the solid curve's pieces.
    pieces = dash_pieces(lab, dash)
    rlab, rn = components(residual)
    _, rdash = classify(rlab, rn)
    # leaving out the solid curve's own ink (where its line splits at a bend)
    pieces += [d for d in dash_pieces(rlab, rdash)
               if np.nanmean([abs(v - sim[x]) for x, v in d[0].items()]) > line_w]
    extra = [({x: y}, 0.0) for x, y in extra]
    out = {"ames_genhel": (sim, sim_f)}
    if dash_dot:
        # App. E: the flight test dashed (dashes 14-17 px long), the Sikorsky GenHel
        # dash-dotted (dashes 32-35 px long and dots, which are left out); where the flight
        # test can't be seen it may run along either simulation
        sik = chain([d for d in pieces if d[1] >= LONG_DASH] + extra, [sim], line_w)
        out["sikorsky_genhel"] = sik
        out["aefa_test"] = chain([d for d in pieces if d[1] < LONG_DASH] + extra,
                                 [sim, sik[0]], line_w)
    else:
        out["aefa_test"] = chain(pieces + extra, [sim], line_w)
    return off_scale(out, H)


def trace_band(lab, solid, dash, dash_dot):
    """trace() for a vibrating solid curve: its middle and the upper and lower edges of its
    swing, and the dashed curves where they show outside it."""
    H, W = lab.shape
    lo = np.full(W, np.nan)
    hi = np.full(W, np.nan)
    on = solid[lab]
    for x in range(W):
        yy = np.flatnonzero(on[:, x])
        if len(yy):
            lo[x], hi[x] = yy[0], yy[-1]
    top = np.full(W, np.nan)
    bot = np.full(W, np.nan)
    r = VIB_PERIOD // 2
    for x in range(W):
        w = slice(max(0, x - r), x + r + 1)
        if np.isfinite(lo[w]).sum() > r:
            top[x], bot[x] = np.nanmin(lo[w]), np.nanmax(hi[w])
    mid = (top + bot) / 2.0
    pieces = dash_pieces(lab, dash)
    line_w = 4.0
    blank = np.array([""] * W, dtype=object)
    out = {"ames_genhel": (mid, blank.copy()), "ames_genhel_upper": (top, blank.copy()),
           "ames_genhel_lower": (bot, blank.copy())}
    if dash_dot:
        sik = chain([d for d in pieces if d[1] >= LONG_DASH], [mid], line_w, (top, bot))
        out["sikorsky_genhel"] = sik
        out["aefa_test"] = chain([d for d in pieces if d[1] < LONG_DASH], [mid, sik[0]], line_w,
                                 (top, bot))
    else:
        out["aefa_test"] = chain(pieces, [mid], line_w, (top, bot))
    return off_scale(out, H)


def off_scale(out, H):
    """Readings within EDGE of the read area's top or bottom are off the scale (or in the
    tick margin): not read (NaN), flagged `off_scale`."""
    for y, f in out.values():
        off = np.isfinite(y) & ((y <= EDGE) | (y >= H - 1 - EDGE))
        y[off] = np.nan
        f[off] = "off_scale"
    return out


def chain(pieces, refs, line_w, band=None):
    """The dashed curve through `pieces` ([({column: y}, length)]), as (y, flags) per column.

    - The chain of dashes, left to right, covering the most columns, each dash starting
      where the one before, carried on at its slope, leads: within 8 px plus 15 % of the
      gap, plus half the slope over at most SLOPE_REACH columns (a dash's slope is
      uncertain, more so on a steep stretch). A gap longer than a dash gap is bridged only
      where both ends are near one of `refs` (the curves the dashed one can run under, the
      solid one first; any length) or the slope leads across it (up to LINK_GAP).
    - Then each longer gap is filled with the dashes not chained that lie in it, linked
      more loosely (a sharp bend between two dashes), and the chain is carried on past its
      ends the same way.
    - Across a gap longer than a dash gap, where both ends are near a ref curve, the dashed
      curve is hidden in it and runs along it (`merged`): its offset from it goes linearly
      from one end's to the other's, held within half a line width away from the ends.
      Elsewhere it is interpolated linearly (`merged` within 6 px of the solid curve, else
      `gap`). Dash gaps are interpolated linearly, unflagged.
    - Before the first dash and after the last, where the dashed curve meets a ref curve
      (within 6 px) it runs along it at that offset (`merged`).
    - With `band` ((top, bottom) per column of a vibrating solid curve), a gap whose ends
      are both in the band is hidden in it: not read (NaN), flagged `hidden`; so are the
      stretches before the first dash and after the last.
    - With no dashes at all, the curves coincide everywhere (`merged`).
    """
    sim = refs[0]
    W = len(sim)
    pieces = sorted((d for d, _ in pieces), key=lambda d: min(d))
    reach = 12 + 2 * line_w  # [px] an end this close to a ref curve may run under it

    def ends(d):
        xs_ = sorted(d)
        a_, b_ = xs_[0], xs_[-1]
        sl = (d[b_] - d[a_]) / (b_ - a_) if b_ > a_ else 0.0
        return a_, b_, sl

    def in_band(x, y_):
        return band is not None and np.isfinite(band[0][x]) and \
            band[0][x] - reach <= y_ <= band[1][x] + reach

    def near(x, y_):
        """The ref curve nearest the point (x, y_), if within `reach`, or None."""
        if in_band(x, y_):
            return 0
        d_ = [abs(y_ - c[x]) if np.isfinite(c[x]) else np.inf for c in refs]
        i_ = int(np.argmin(d_))
        return i_ if d_[i_] <= reach else None

    info = [ends(d) for d in pieces]
    m = len(pieces)

    def links(i, j, loose=False):
        ai, bi, sli = info[i]
        aj = info[j][0]
        gap = aj - bi
        if gap < -3:
            return False
        g = max(gap, 0)
        r_ = min(g, SLOPE_REACH)
        dy = abs(pieces[j][aj] - (pieces[i][bi] + sli * r_))
        # on a steep stretch the gap between dashes is mostly vertical, up to about 12 px
        steep = 12 * min(1.0, abs(sli))
        if loose:
            return g <= 2 * DASH_GAP and dy <= 8 + steep + 0.6 * g + 0.5 * abs(sli) * r_
        if gap <= LINK_GAP and dy <= 8 + steep + 0.15 * g + 0.5 * abs(sli) * r_:
            return True
        if gap > DASH_GAP:
            na, nb_ = near(bi, pieces[i][bi]), near(aj, pieces[j][aj])
            if na is not None and na == nb_:
                return True
            # into or out of a vibrating curve's band: the dash outside it, carried on at
            # its slope toward the other, enters the band
            if band is not None:
                slj = info[j][2]
                if in_band(bi, pieces[i][bi]) and \
                        in_band(aj - r_, pieces[j][aj] - slj * r_):
                    return True
                if in_band(aj, pieces[j][aj]) and in_band(bi + r_, pieces[i][bi] + sli * r_):
                    return True
        return False

    def longest(cands, start=None, end=None, loose=False):
        """Indices of the chain through `cands` (sorted) covering the most columns, from
        `start` (the first of `cands`) and to `end` when given (None if no chain reaches
        `end`)."""
        best, prev = {}, {}
        for jj, j in enumerate(cands):
            best[j] = len(pieces[j]) if start is None or j == start else None
            prev[j] = None
            for i in cands[:jj]:
                if best[i] is not None and links(i, j, loose) and \
                        (best[j] is None or best[i] + len(pieces[j]) > best[j]):
                    best[j], prev[j] = best[i] + len(pieces[j]), i
        if end is None:
            end = max((k for k in cands if best[k] is not None), key=best.get)
        if best[end] is None:
            return None
        path = []
        k = end
        while k is not None:
            path.append(k)
            k = prev[k]
        return path[::-1]

    path = longest(list(range(m))) if m else []
    # fill the longer gaps with the dashes not chained that lie in them
    used = set(path)
    filled = path[:1]
    for i, j in zip(path[:-1], path[1:]):
        if info[j][0] - info[i][1] > DASH_GAP:
            cands = [k for k in range(m) if k not in used
                     and info[k][0] > info[i][1] - 3 and info[k][1] < info[j][0] + 3]
            sub = longest([i] + cands + [j], start=i, end=j, loose=True) if cands else None
            if sub:
                filled += sub[1:-1]
                used.update(sub)
        filled.append(j)
    path = filled
    # carry the chain on past its ends
    if path:
        right = [path[-1]] + [k for k in range(m) if k not in used and info[k][0] > info[path[-1]][1] - 3]
        sub = longest(right, start=path[-1], loose=True)
        path += (sub or [path[-1]])[1:]
        used.update(path)
        # to the left: the same, mirrored
        flip = lambda d: {-x: v for x, v in d.items()}  # noqa: E731
        lp = [k for k in range(m) if k not in used and info[k][1] < info[path[0]][0] + 3]
        if lp:
            saved = pieces, info
            pieces = [flip(d) for d in pieces]
            info = [ends(d) for d in pieces]
            order = sorted([path[0]] + lp, key=lambda k: info[k][0])
            sub = longest(order, start=path[0], loose=True)
            pieces, info = saved
            path = (sub or [path[0]])[:0:-1] + path

    y = np.full(W, np.nan)
    for j in path:
        for x, v in pieces[j].items():
            y[x] = v if np.isnan(y[x]) else (y[x] + v) / 2
    f = np.array([""] * W, dtype=object)
    ok = np.flatnonzero(~np.isnan(y))
    if not len(ok):
        if band is not None:
            return np.full(W, np.nan), np.array(["hidden"] * W, dtype=object)
        return sim.copy(), np.array(["merged"] * W, dtype=object)
    for a_, b_ in zip(ok[:-1], ok[1:]):
        if b_ - a_ <= 1:
            continue
        gap = np.arange(a_ + 1, b_)
        long_ = b_ - a_ > DASH_GAP
        if long_ and in_band(a_, y[a_]) and in_band(b_, y[b_]):
            f[gap] = "hidden"
            continue
        na, nb_ = near(a_, y[a_]), near(b_, y[b_])
        c = refs[na] if na is not None and na == nb_ else None
        if long_ and c is not None and np.isfinite(c[a_ - 1:b_ + 1]).all():
            # hidden, the dashed line lies within the other line's ink: its offset, carried
            # in from the ends, is held within half a line width (over 10 columns)
            ramp = np.interp(gap, [a_, b_], [y[a_] - c[a_], y[b_] - c[b_]])
            w = np.minimum(1.0, np.minimum(gap - a_, b_ - gap) / 10.0)
            y[gap] = c[gap] + (1 - w) * ramp + w * np.clip(ramp, -line_w / 2, line_w / 2)
            f[gap] = "merged"
            continue
        y[gap] = np.interp(gap, [a_, b_], [y[a_], y[b_]])
        # a dash gap (up to DASH_GAP columns) is part of reading a dashed line
        if long_:
            f[gap] = np.where(np.abs(y[gap] - sim[gap]) <= 6, "merged", "gap")
    # before the first dash and after the last: the dashed curve runs along a ref curve,
    # at its offset there, if it meets one
    for edge, rng in ((ok[0], range(ok[0] - 1, -1, -1)), (ok[-1], range(ok[-1] + 1, W))):
        if in_band(edge, y[edge]):
            for x in rng:
                f[x] = "hidden"
            continue
        for c in sorted(refs, key=lambda c_: abs(y[edge] - c_[edge]) if np.isfinite(c_[edge])
                        else np.inf):
            if np.isfinite(c[edge]) and abs(y[edge] - c[edge]) <= 6:
                off = y[edge] - c[edge]
                for x in rng:
                    if np.isnan(c[x]):
                        break
                    y[x] = c[x] + off
                    f[x] = "merged"
                break
    y[f == "hidden"] = np.nan
    return y, f


def digitize(img, frame, spec, overlay_path, dash_dot=False, vibrating=False, hand=()):
    """Read one panel. Returns (readings [(series, t [s], value, flag)], stretches not read
    [(series, t [s], flag)] on the same time grid, largest tick offset [px]). `hand`: the
    reviewer's hand edits of this panel (apply_hand). Also writes the overlay and, beside
    it, its calibration (JSON: the overlay pixel -> time and value maps, for the review
    page's edit tool)."""
    (xa, xb), (top, bot) = frame
    qty, unit, (yt, ym, yb) = spec
    if abs((yt + yb) / 2.0 - ym) > 1e-9:
        raise PanelError(f"middle label {ym} is not the mean of {yt} and {yb}")
    dark = img < DARK
    ycoef, tick_off = y_scale(img, xb, top, bot, yt, yb)

    def to_t(px):
        return (px - xa) / (xb - xa) * T_END

    def to_y(py):
        return ycoef[0] * py + ycoef[1]

    box = tuple(int(round(v)) for v in (xa + INSET, top + INSET, xb - INSET, bot - INSET))
    series = trace(dark[box[1]:box[3], box[0]:box[2]], dash_dot, vibrating)

    step = 0.025  # [s]
    out = []
    unread = []
    for name, (y, f) in series.items():
        ok = np.flatnonzero(~np.isnan(y))
        if not len(ok):
            continue
        ts = to_t(box[0] + ok.astype(float))
        for tv in np.arange(np.ceil(ts[0] / step) * step, ts[-1] + 1e-9, step):
            j = int(np.argmin(np.abs(ts - tv)))
            if abs(ts[j] - tv) > 2 * T_END / (xb - xa):
                c = int(round((tv / T_END * (xb - xa) + xa) - box[0]))
                if 0 <= c < len(f) and f[c]:
                    unread.append((name, float(tv), f[c]))
                continue
            out.append((name, float(tv), float(to_y(box[1] + np.interp(tv, ts, y[ok]))),
                        f[ok[j]]))

    out, unread = apply_hand(out, unread, hand, step)

    # Overlay for review.
    x0, y0 = max(0, int(xa) - 150), max(0, int(top) - 40)
    x1, y1 = min(img.shape[1], int(xb) + 40), min(img.shape[0], int(bot) + 60)
    fig = plt.figure(figsize=((x1 - x0) / 100, (y1 - y0) / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img[y0:y1, x0:x1], cmap="gray", extent=(x0, x1, y1, y0))
    inv_t = lambda v: xa + v / T_END * (xb - xa)  # noqa: E731
    inv_y = lambda v: (v - ycoef[1]) / ycoef[0]  # noqa: E731
    # each series as a thin coloured line over the scan; gaps in magenta, merged in green
    colours = {"ames_genhel": "tab:blue", "aefa_test": "tab:red",
               "sikorsky_genhel": "tab:orange", "ames_genhel_upper": "deepskyblue",
               "ames_genhel_lower": "deepskyblue"}
    for name in colours:
        pts = [(inv_t(tv), inv_y(yv), flag) for nm, tv, yv, flag in out if nm == name]
        if not pts:
            continue
        # break the line where readings are missing
        xs_ = np.array([p_[0] for p_ in pts])
        ys_ = np.array([p_[1] for p_ in pts])
        cut = np.flatnonzero(np.diff(xs_) > 1.5 * step / T_END * (xb - xa)) + 1
        for xs_s, ys_s in zip(np.split(xs_, cut), np.split(ys_, cut)):
            ax.plot(xs_s, ys_s, "-", color=colours[name], lw=1.0, alpha=0.85)
        for flag, c in (("gap", "magenta"), ("merged", "lime"), ("hand", "gold")):
            sel = [(x_, y_) for x_, y_, f_ in pts if f_ == flag]
            if sel:
                ax.plot(*zip(*sel), ".", color=c, ms=2.5)
    for v in (yt, ym, yb, (yt + ym) / 2, (ym + yb) / 2):
        ax.axhline(inv_y(v), color="orange", lw=0.5, alpha=0.7)
    ax.add_patch(plt.Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], fill=False,
                               ec="green", lw=0.5, ls=":"))
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)
    ax.axis("off")
    fig.savefig(overlay_path)
    plt.close(fig)
    # overlay pixel (u, v) -> t = (u + x0 - xa) / (xb - xa) * T_END, value = a * (v + y0) + b
    Path(overlay_path).with_suffix(".json").write_text(json.dumps(
        {"x0": x0, "y0": y0, "xa": xa, "xb": xb, "t_end": T_END, "a": float(ycoef[0]),
         "b": float(ycoef[1]), "width": x1 - x0, "height": y1 - y0}), encoding="utf-8")
    return out, unread, tick_off


HAND = OUT / "ballin_d_hand_edits.csv"
# The human review of these readings' overlays (this tool's output as of the date, with the
# hand edits applied): every panel checked by the reviewer. Clear it (back to UNVERIFIED) when
# a change to the reading code changes the readings.
REVIEWED = "human-reviewed 2026-10-19: the reviewer checked every panel's overlay"
# series the reviewer did not accept, (figure, quantity, series) -> why; written with the
# flag `rejected`
REJECTED = {("E2", "vertical_acceleration_cg_sensor", "sikorsky_genhel"):
            "the Sikorsky GenHel's dash-dot line is buried in the solid line"}


def read_hand():
    """The reviewer's hand edits (HAND): {(figure, quantity): [(series, kind, [(t, value)])]}.
    kind `trace`: the series runs through the points (linearly between them); `unreadable`:
    the series can't be read between the first point's time and the last's."""
    if not HAND.exists():
        return {}
    edits = {}
    with HAND.open(encoding="utf-8") as f:
        for r in csv.DictReader(line for line in f if not line.startswith("#")):
            key = (r["figure"], r["quantity"], r["series"], r["edit"], r["kind"])
            edits.setdefault(key, []).append((float(r["t_s"]), float(r["value"])))
    out = {}
    for (fig, qty, series, _, kind), pts in edits.items():
        out.setdefault((fig, qty), []).append((series, kind, sorted(pts)))
    return out


def apply_hand(out, unread, hand, step):
    """Replace a series' readings over each hand edit's time span with the edit: the
    points, interpolated linearly on the reading grid (flag `hand`), or none (`unreadable`,
    listed as not read)."""
    for series, kind, pts in hand:
        t0, t1 = pts[0][0], pts[-1][0]
        inside = lambda r: r[0] == series and t0 - 1e-9 <= r[1] <= t1 + 1e-9  # noqa: E731
        out = [r for r in out if not inside(r)]
        unread = [r for r in unread if not inside(r)]
        grid = np.arange(np.ceil(t0 / step - 1e-9) * step, t1 + 1e-9, step)
        if kind == "trace":
            ts, vs = zip(*pts)
            out += [(series, float(tv), float(np.interp(tv, ts, vs)), "hand") for tv in grid]
        else:
            unread += [(series, float(tv), "unreadable") for tv in grid]
    out.sort(key=lambda r: (r[0], r[1]))
    unread.sort(key=lambda r: (r[0], r[1]))
    return out, unread


def stretches(times, join=0.1):
    """[(first, last)] of runs of times less than `join` apart."""
    out = []
    for tv in times:
        if out and tv - out[-1][1] < join:
            out[-1][1] = tv
        else:
            out.append([tv, tv])
    return out


def review_lines(tag, out, unread):
    """Low-confidence stretches of a panel's readings, for human review: interpolated
    (`gap`) stretches longer than 0.1 s (runs less than 0.1 s apart joined), stretches not
    read (off the scale, or hidden in the simulation's vibration), and a series whose
    readings don't span 0.15 s to 5.85 s."""
    lines = []
    for name in ("ames_genhel", "aefa_test", "sikorsky_genhel"):
        pts = [(tv, flag) for nm, tv, _, flag in out if nm == name]
        if not pts:
            if name != "sikorsky_genhel" or tag.startswith("E"):
                lines.append(f"{tag} {name}: not read")
            continue
        if pts[0][0] > 0.15 or pts[-1][0] < 5.85:
            lines.append(f"{tag} {name}: readings span only {pts[0][0]:.2f}-{pts[-1][0]:.2f} s")
        for t0, t1 in stretches([tv for tv, flag in pts if flag == "gap"]):
            if t1 - t0 > 0.1:
                lines.append(f"{tag} {name}: interpolated {t0:.2f}-{t1:.2f} s (gap)")
        for flag, what in (("off_scale", "off the scale, not read"),
                           ("hidden", "hidden in the simulation's vibration, not read"),
                           ("unreadable", "unreadable (reviewer), not read")):
            for t0, t1 in stretches([tv for nm, tv, fl in unread if nm == name and fl == flag]):
                lines.append(f"{tag} {name}: {t0:.2f}-{t1:.2f} s {what}")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pdf", type=Path, help="NASA-TM-88360.pdf")
    parser.add_argument("overlays", type=Path, help="directory for the review overlays")
    parser.add_argument("--only", help="digitize only these figures (comma separated)")
    args = parser.parse_args()
    args.overlays.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    doc = fitz.open(args.pdf)
    hand = read_hand()
    only = set(args.only.split(",")) if args.only else None
    failed = 0
    review = []
    for fig, (page0, file_no, caption) in FIGURES.items():
        if only and fig not in only:
            continue
        geo = {}
        rows = []
        for pi, k, qty, unit, scale in panels_of(fig):
            page = page0 + pi
            tag = f"{fig} p{page}.{k} {qty}"
            try:
                if page not in geo:
                    geo[page] = page_geometry(doc, page)
                rot, frames, img = geo[page]
                path = args.overlays / f"ballin_{fig.lower()}_{qty}.png"
                out, unread, tick_off = digitize(img, frames[k], (qty, unit, scale), path,
                                                 dash_dot=fig.startswith("E"),
                                                 vibrating=(pi, k) in VIBRATING,
                                                 hand=hand.get((fig, qty), ()))
            except PanelError as e:
                print(f"{tag}: FAILED: {e}", file=sys.stderr)
                failed += 1
                continue
            n = {}
            for r in out:
                n[r[0]] = n.get(r[0], 0) + 1
            print(f"{tag}: rotation {rot:+.2f} deg, y ticks within {tick_off:.1f} px of the fit; "
                  + ", ".join(f"{v} {k}" for k, v in n.items()))
            rows += [(qty, unit) + r[:-1] + ("rejected" if (fig, qty, r[0]) in REJECTED
                                             else r[-1],) for r in out]
            review += review_lines(tag, out, unread)
        path = OUT / f"ballin_{fig.lower()}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            f.write(f"# source: {SOURCE}, Fig. {fig} (PDF pp. {page0}-{page0 + 7}), File "
                    f"{file_no}: {caption}. Digitized from the NASA scan at {DPI} dpi by "
                    "tools/digitize_ballin_d.py.\n")
            rejected = [f"{q} {s} ({why})" for (g, q, s), why in REJECTED.items() if g == fig]
            f.write(f"# status: {REVIEWED or 'UNVERIFIED - needs human review of the overlays'}"
                    + (f"; not accepted: {'; '.join(rejected)}" if rejected else "") + "\n")
            f.write("# Series: ames_genhel (solid, Ames real-time GenHel), aefa_test (dashed, "
                    "USAAEFA flight test)"
                    + (", sikorsky_genhel (dash-dot, Sikorsky GenHel)" if fig.startswith("E")
                       else "")
                    + (", ames_genhel_upper and ames_genhel_lower (the edges of its 4/rev swing)"
                       if fig in COLLECTIVE else "")
                    + ". Flags: merged, gap, hand, rejected (tool docstring).\n")
            w = csv.writer(f, lineterminator="\n")
            w.writerow(["quantity", "unit", "series", "t_s", "value", "flag"])
            for qty, unit, name, tv, yv, flag in rows:
                w.writerow([qty, unit, name, f"{tv:.3f}", f"{yv:.4f}", flag])
        print(f"wrote {path.relative_to(ROOT)}")
    (args.overlays / "review.txt").write_text("\n".join(review) + "\n", encoding="utf-8")
    print(f"{len(review)} low-confidence stretches listed in {args.overlays / 'review.txt'}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
