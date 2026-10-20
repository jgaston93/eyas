"""Digitize Kaplita, NASA CR-177360 (1985), transient responses (Figs. 1-8): Sikorsky's GenHel
against the USAAEFA Project 79-24 flight test.

    .venv/bin/python -I tools/digitize_kaplita_transients.py <19870008277.pdf> <overlay dir> [--only 4]

Writes data/uh60a/validation/kaplita_transients/kaplita_fig<N>.csv and, in <overlay dir>, one
PNG per panel with the readings drawn over the scan (and its calibration JSON), for human review
(DATA_GAPS.md). Readings need a person's check of the overlays before use; a change to the
reading code needs a new review (REVIEWED).

The figures are plotted by the same program as Ballin's App. D (TM-88360), four framed panels a
page, time 0-10 s, solid "Calculated" (Sikorsky GenHel) and dashed "Test" (flight test); only
some of each figure's eight pages are printed. This tool reuses digitize_ballin_d's panel
reading (tick-stub calibration, the solid and dashed tracing, overlays, hand edits) and differs
in:
- page orientation, set per page (the scans' tick labels defeat digitize_ballin_d's guess);
- the frames' sides, from the extent of the strong horizontal lines (frame_sides);
- the frames: Kaplita's major grid lines are as dark as the frame lines, so a frame is a run
  of strong full-width lines (its edges and grid) between gaps of more than 65 px at 100 dpi,
  each edge then refined on the full-resolution page as digitize_ballin_d does;
- the y calibration, on the grid lines (8 or 10 divisions) instead of the faint tick stubs
  (y_scale);
- the grid, as heavy as the curves, erased before tracing, keeping the curves' crossings
  (clean_grid);
- the time axis, 0-10 s;
- the series: the solid line is sikorsky_genhel, the dashed aefa_test.
"""

import argparse
import csv
import sys
from pathlib import Path

import fitz
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import digitize_ballin_d as dbd  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "uh60a" / "validation" / "kaplita_transients"
HAND = OUT / "kaplita_transient_hand_edits.csv"
# The human review of these readings (this tool's output as of the date, with the hand edits
# applied): Fig. 4 in three rounds on the review page. Clear it (back to UNVERIFIED) when a change
# to the reading code changes the readings.
REVIEWED = ("human-reviewed 2026-10-20 (Fig. 4): the reviewer checked every trace and redrew the "
            "ones the tool misread (hand edits); the test heading is unreadable; roll tick labels "
            "read by the reviewer and checked by the attitude kinematics (DATA_GAPS.md)")

SOURCE = ("Kaplita, T. T., UH-60 Black Hawk Engineering Simulation Model Validation and "
          "Proposed Modifications, NASA CR-177360 (USAAVSCOM TR 85-A-2, 1985)")

# Figures: caption, header (as printed above the panels), and pages: PDF page -> (base
# rotation [deg], [(panel, quantity, unit, (top, middle, bottom) tick labels) or None]).
FIGURES = {
    "4": {
        "caption": "response to a half-inch right pedal step input at 144 knots, forward CG "
                   "352.1 in (flight 66, run 27)",
        "pages": {
            59: (90.0, [(0, "heading", "deg", (40.0, 0.0, -40.0)),
                        (1, "yaw_rate", "deg/s", (20.0, 0.0, -20.0)),
                        (3, "pedals", "in", (10.0, 5.0, 0.0))]),
            60: (90.0, [(0, "pitch_attitude", "deg", (50.0, 0.0, -50.0)),
                        (1, "pitch_rate", "deg/s", (20.0, 0.0, -20.0)),
                        (3, "longitudinal_cyclic", "in", (10.0, 5.0, 0.0))]),
            # Roll attitude and rate: tick labels merged into the axis titles at 600 dpi;
            # read by the reviewer, unsure (DATA_GAPS.md), checked by integrating the rate.
            # Lateral stick 0-10 in as the other sticks, checked against the header's trim
            # XA = 5.505 in.
            61: (90.0, [(0, "roll_attitude", "deg", (80.0, 0.0, -80.0)),
                        (1, "roll_rate", "deg/s", (40.0, 0.0, -40.0)),
                        (3, "lateral_cyclic", "in", (10.0, 5.0, 0.0))]),
            62: (90.0, [(0, "angle_of_attack", "deg", (20.0, 0.0, -20.0)),
                        (1, "sideslip", "deg", (20.0, 0.0, -20.0))]),
        },
    },
}


def frame_sides(dark):
    """x of the frames' left and right edges [coarse px]: the median extent of the strong
    full-width rows (the frames' edges and major grid lines), each followed outwards from the
    page's middle across gaps of up to 3 px. Kaplita's major vertical grid lines, one a
    second, are as heavy as the frame's sides, which defeats digitize_ballin_d's search for
    the strongest pair of vertical lines."""
    H, W = dark.shape
    mid = dark[:, int(W * 0.35):int(W * 0.6)].mean(axis=1)
    lefts, rights = [], []
    for r in np.flatnonzero(mid > 0.95):
        band = dark[max(0, r - 1):r + 2].any(axis=0)
        left = right = W // 2
        while left > 3 and band[left - 4:left].any():
            left -= 1
        while right < W - 4 and band[right:right + 4].any():
            right += 1
        lefts.append(left)
        rights.append(right)
    if len(lefts) < 8:
        return None
    return int(np.median(lefts)), int(np.median(rights))


def frames_of(dark, xa, xb):
    """The four frames [(top, bottom)] [coarse px]: runs of rows inked across at least 80 % of
    the frame width, split where consecutive strong rows are more than 65 px apart (the
    frames are about 75 px apart; a faint grid can leave 60 px between strong rows inside one)."""
    frac = dark[:, xa + 5:xb - 5].mean(axis=1)
    rows = np.flatnonzero(frac > 0.8)
    groups = [[rows[0]]]
    for r in rows[1:]:
        if r - groups[-1][-1] > 65:
            groups.append([r])
        else:
            groups[-1].append(r)
    frames = [(g[0], g[-1]) for g in groups if g[-1] - g[0] > 60]
    if len(frames) != 4:
        raise dbd.PanelError(f"{len(frames)} frames found, not 4")
    return frames


def page_geometry(doc, page, base):
    """(rotation, [((left, right), (top, bottom))] [px at dbd.DPI], page image)."""
    rot = dbd.deskew(doc, page, base)
    dark = dbd.render(doc, page, rot, dbd.COARSE_DPI) < dbd.DARK
    sides = frame_sides(dark)
    if sides is None:
        raise dbd.PanelError(f"p{page}: no frames found")
    frames = frames_of(dark, *sides)
    s = dbd.DPI / dbd.COARSE_DPI
    img = dbd.render(doc, page, rot, dbd.DPI)
    dk = img < dbd.DARK

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


def y_scale(img, xb, top, bot, yt, yb):
    """As digitize_ballin_d.y_scale (same signature and checks), but on Kaplita's grid lines
    instead of the tick stubs, which are too faint on these scans. A panel has 8 or 10
    divisions (the sticks and the larger attitude scales have 10). For n = 8, then 10, each
    interior line k/n is the centre of the rows within 1.5 % of the frame height of where the
    edges put it that are inked across at least 70 % of the frame (a curve crosses a row only
    briefly; one running flat along a grid line moves the centre by under its width); the
    first n with every line found is used, and the scale is fitted through the edges and all
    of them, at the labels (top, middle, bottom) interpolated linearly."""
    dark = img < dbd.DARK
    xr = int(round(xb))
    yt_row = int(round(top))
    xa = xr - 1
    while xa > 4 and dark[yt_row - 1:yt_row + 2, xa - 4:xa].any():
        xa -= 1
    tol = 0.015 * (bot - top)
    for n in (8, 10):
        pts = [(top, yt), (bot, yb)]
        for k in range(1, n):
            y = top + (bot - top) * k / n
            lo, hi = int(y - tol), int(y + tol) + 1
            frac = dark[lo:hi, xa + 20:xr - 20].mean(axis=1)
            hits = np.flatnonzero(frac >= 0.7)
            if not len(hits):
                break
            pts.append((lo + float(np.mean(hits)), yt + (yb - yt) * k / n))
        else:
            break
    else:
        raise dbd.PanelError("the grid lines fit neither 8 nor 10 divisions")
    _GRID["rows"] = sorted(r for r, _ in pts)
    _GRID["top"] = top
    p = np.array(pts)
    A = np.vstack([p[:, 0], np.ones(len(p))]).T
    coef, *_ = np.linalg.lstsq(A, p[:, 1], rcond=None)
    res_px = float(np.abs((A @ coef - p[:, 1]) / coef[0]).max())
    if res_px > max(3.0, 0.01 * (bot - top)):
        raise dbd.PanelError(f"a grid line is {res_px:.1f} px off the fitted scale")
    return coef, res_px


_GRID = {}  # the grid rows y_scale found for the panel being read [px in the page image]
GRID_BAND = 6  # [px] half-width searched around a grid line's centre for its ink
GRID_FRAC = 0.3  # a row or column inked over this fraction of the frame is grid
CROSS_REACH = 4  # [px] how far sideways a crossing curve's ink is looked for
DOT_MAX = 14  # [px] largest grid dot; the test line's dashes are 15-45 px long


def clean_grid(ink):
    """The panel's interior ink with Kaplita's grid erased, for digitize_ballin_d.trace: the
    horizontal lines at the divisions y_scale found (rows within GRID_BAND of a line's
    centre inked over GRID_FRAC of the width), then the vertical lines (columns inked over
    GRID_FRAC of the height once the horizontal lines are gone, at most 10 px wide). Where a
    curve crosses a line its ink is kept: a pixel of an erased band stays if there is ink
    just beyond both sides of the band within CROSS_REACH px. A curve running along a grid
    line is erased with it and read as a gap (flagged for review)."""
    out = ink.copy()
    H, W = ink.shape
    off = int(round(_GRID["top"] + dbd.INSET))

    def keep_crossings(band_mask, before, after, axis):
        if axis == 0:  # horizontal band rows [r0, r1]
            r0, r1 = before, after
            if r0 - 2 < 0 or r1 + 2 >= H:
                return band_mask
            above = np.convolve(ink[r0 - 2], np.ones(2 * CROSS_REACH + 1), "same") > 0
            below = np.convolve(ink[r1 + 2], np.ones(2 * CROSS_REACH + 1), "same") > 0
            return band_mask & ~(above & below)[None, :]
        c0, c1 = before, after
        if c0 - 2 < 0 or c1 + 2 >= W:
            return band_mask
        left = np.convolve(ink[:, c0 - 2], np.ones(2 * CROSS_REACH + 1), "same") > 0
        right = np.convolve(ink[:, c1 + 2], np.ones(2 * CROSS_REACH + 1), "same") > 0
        return band_mask & ~(left & right)[:, None]

    frac = ink.mean(axis=1)
    for centre in _GRID["rows"]:
        c = int(round(centre)) - off
        rows = [r for r in range(c - GRID_BAND, c + GRID_BAND + 1)
                if 0 <= r < H and frac[r] >= GRID_FRAC]
        if not rows:
            continue
        r0, r1 = min(rows), max(rows)
        erase = keep_crossings(ink[r0:r1 + 1].copy(), r0, r1, 0)
        out[r0:r1 + 1] &= ~erase
    cfrac = out.mean(axis=0)
    cols = np.flatnonzero(cfrac >= GRID_FRAC)
    groups = []
    for cc in cols:
        if groups and cc - groups[-1][-1] <= 1:
            groups[-1].append(cc)
        else:
            groups.append([cc])
    for g in groups:
        if len(g) > 10:
            continue  # not a grid line
        c0, c1 = g[0], g[-1]
        erase = keep_crossings(out[:, c0:c1 + 1].copy(), c0, c1, 1)
        out[:, c0:c1 + 1] &= ~erase
    # The grid's intersection dots: small pieces (at most DOT_MAX px each way) centred
    # within GRID_BAND of a grid row, or of a half division (where the minor dotted
    # lines run).
    lab, n = dbd.components(out)
    rows = [r - off for r in _GRID["rows"]]
    near = rows + [(a + b) / 2.0 for a, b in zip(rows, rows[1:])]
    ys, xs = np.nonzero(lab)
    ids = lab[ys, xs]
    for i in np.unique(ids):
        sel = ids == i
        y0, y1, x0, x1 = ys[sel].min(), ys[sel].max(), xs[sel].min(), xs[sel].max()
        if y1 - y0 >= DOT_MAX or x1 - x0 >= DOT_MAX:
            continue
        if min(abs((y0 + y1) / 2.0 - r) for r in near) <= GRID_BAND:
            out[ys[sel], xs[sel]] = False
    return out


_trace = dbd.trace


def trace(ink, dash_dot=False, vibrating=False):
    return _trace(clean_grid(ink), dash_dot, vibrating)


def read_hand():
    dbd.HAND = HAND
    return dbd.read_hand()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("pdf", type=Path, help="19870008277.pdf (NASA CR-177360)")
    parser.add_argument("overlays", type=Path, help="directory for the review overlays")
    parser.add_argument("--only", help="digitize only these figures (comma separated)")
    args = parser.parse_args()
    args.overlays.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    dbd.T_END = 10.0  # [s] Kaplita's frames run 0-10 s
    dbd.y_scale = y_scale  # calibrate on the grid lines (y_scale docstring)
    dbd.trace = trace  # erase the grid first (clean_grid)
    doc = fitz.open(args.pdf)
    hand = read_hand() if HAND.exists() else {}
    only = set(args.only.split(",")) if args.only else None
    failed = 0
    review = []
    for fig, spec in FIGURES.items():
        if only and fig not in only:
            continue
        rows = []
        pages = sorted(spec["pages"])
        for page, (base, panels) in spec["pages"].items():
            try:
                rot, frames, img = page_geometry(doc, page, base)
            except dbd.PanelError as e:
                print(f"Fig. {fig} p{page}: FAILED: {e}", file=sys.stderr)
                failed += 1
                continue
            for k, qty, unit, scale in panels:
                tag = f"Fig. {fig} p{page}.{k} {qty}"
                if scale is None:
                    print(f"{tag}: scale not read yet, skipped")
                    continue
                path = args.overlays / f"kaplita_fig{fig}_{qty}.png"
                try:
                    # digitize_ballin_d names the solid line ames_genhel: the hand edits
                    # must use its name to replace its readings
                    edits = [("ames_genhel" if e[0] == "sikorsky_genhel" else e[0],) + tuple(e[1:])
                             for e in hand.get((fig, qty), ())]
                    out, unread, tick_off = dbd.digitize(img, frames[k], (qty, unit, scale), path,
                                                         hand=edits)
                except dbd.PanelError as e:
                    print(f"{tag}: FAILED: {e}", file=sys.stderr)
                    failed += 1
                    continue
                rename = {"ames_genhel": "sikorsky_genhel"}
                out = [(rename.get(r[0], r[0]),) + tuple(r[1:]) for r in out]
                unread = [(rename.get(u[0], u[0]),) + tuple(u[1:]) for u in unread]
                n = {}
                for r in out:
                    n[r[0]] = n.get(r[0], 0) + 1
                print(f"{tag}: rotation {rot:+.2f} deg, y ticks within {tick_off:.1f} px of "
                      "the fit; " + ", ".join(f"{v} {s}" for s, v in n.items()))
                rows += [(qty, unit) + tuple(r) for r in out]
                review += dbd.review_lines(tag, out, unread)
        path = OUT / f"kaplita_fig{fig}.csv"
        with path.open("w", newline="", encoding="utf-8") as f:
            f.write(f"# source: {SOURCE}, Fig. {fig} (PDF pp. {pages[0]}-{pages[-1]}): "
                    f"{spec['caption']}. Digitized from the NASA scan at {dbd.DPI} dpi by "
                    "tools/digitize_kaplita_transients.py.\n")
            f.write(f"# status: {REVIEWED or 'UNVERIFIED - needs human review of the overlays'}\n")
            f.write("# Series: sikorsky_genhel (solid, 'Calculated'), aefa_test (dashed, "
                    "'Test'). Flags: merged, gap, hand (digitize_ballin_d docstring).\n")
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
