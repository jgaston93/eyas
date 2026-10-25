"""Digitize the function plots of Ballin, NASA TM-100991 (1988), "A High Fidelity Real-Time
Simulation of a Small Turboshaft Engine": the T700 engine model's maps f1-f11 (App. A,
Figs. A1-A11) and the fuel control's schedules (App. C, Figs. C23-C30).

    .venv/bin/python -I tools/digitize_ballin_t700.py <TM-100991.pdf> <overlay dir> [--only A7]
        [--grid]

Writes data/uh60a/ballin_t700/<fig>.csv and, in <overlay dir>, one PNG per figure with the
readings drawn over the scan (and the fitted axes), for human review (DATA_GAPS.md).
Readings need a person's check of the overlays before use; a change to the reading code needs
a new review.

The plots are machine-drawn polylines with every breakpoint marked (a cross, or in Fig. A1
the curve's number), one framed plot per page. How a figure is read:
- Page geometry: the page is deskewed as in digitize_ballin_d (the rotation that makes the
  horizontal lines sharpest); the frame's sides are the two strong vertical lines farthest
  apart, its top and bottom the rows inked across its width; each edge is then followed
  along its length (Frame).
- Calibration: a person reads the values at the frame's corners (the corner labels) and
  the major-tick step into FIGURES. Values are linear between the frame's edges, corrected
  by the scan's uneven stretch as the tick stubs inside the bottom and left edges measure
  it; the scans are sheared (a frame's sides lean 10-21 px over its height) and bowed, so a
  point's fractions across the frame are taken between the frame's local edges, each fitted
  by a quadratic (Frame). A major tick more than
  TICK_TOL off a line through them, or ticks that put the frame's edges more than 1 % off
  the corner labels, stop the figure.
- Breakpoints: --grid lists the crosses it finds (both diagonals inked; find_crosses) and
  draws the fitted axes over the scan; a person checks that list against the overlay and
  enters each breakpoint into FIGURES to well within a marker's size. The tool then snaps
  each to the centre of its marker: the point of a cross (the pixel whose two diagonals,
  ±MARK px, are most inked). A snap that moves a reading more than SNAP_MAX px, or a marker
  not found, stops the figure.
- Fig. A1 (read_dotted_map) is read by its geometry, not by a person: its eleven speed lines
  each run flat from a pressure ratio of 1 to a knee, then fall through six more points,
  marked with the line's number; dotted lines join point i of each speed line to point i of
  the next. The dots are found and chained into straight segments; a vertex lies where a
  segment ending at a glyph meets the next segment leaving it (the middle of the gap), or,
  at the outermost speed lines and where only one end shows, half a typical gap beyond the
  end. Speed lines are numbered by following the chains from the lowest and highest lines
  and from each flat run's end, which is put where its dotted line crosses the run. The
  numbers' glyphs are drawn up and right of their points, so they aren't used.
- Figs. C23 and C30 (read_glyph_polylines) mark their points with the curves' numbers,
  centred on them: a person's readings give the segments, and each interior point is where
  lines fitted to its two segments' ink meet.
- Fig. C30's seven curves cross and overlap through most of its range, so a person's reading
  is only good to about a glyph: each segment's line is first found by moving the reading's
  ends along its normal (up to SEARCH px each) to where it runs on ink over most of its middle,
  then fitted as above. Segments too short to fit between their glyphs (the points 0.8 %
  apart near 87 %) get no line: a point with one fitted neighbour is projected onto it
  ('side'), one with none keeps the person's reading ('read').
"""

import argparse
import csv
import sys
from pathlib import Path

import fitz
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
import digitize_ballin_d as dbd  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "data" / "uh60a" / "ballin_t700"
DPI = 300
MARK = 7       # half-size of a cross marker's diagonal probe [px at DPI]
SNAP_MAX = 14  # largest move from a person's reading to the marker [px]
TICK_TOL = 0.003  # largest tick misfit, fraction of the axis span

# Per figure: the PDF page (1-based), the axes' names and units, each axis's values at the
# frame's edges (left/bottom, right/top) and its major-tick step, read off the scan; the
# marker kind; and the breakpoints read off the --grid overlay ([(x, y), ...] per curve).
FIGURES = {
    "A1": {
        "page": 56, "title": "f1, compressor mass flow (one curve per corrected speed)",
        "x": ("pressure_ratio_ps3_p2", "-"), "y": ("corrected_flow_wa2c", "lbm/s"),
        "x_axis": (0.0, 20.0, 1.0), "y_axis": (2.0, 12.0, 2.0), "marker": "dotted",
        "x_major_min": 4.0,  # x's ticks are all short but the one at 10: fit through every one
        # The curves' symbols 1-11, from the figure's legend: corrected gas generator speed [%].
        "speeds": (65.0, 80.0, 82.0, 85.0, 87.0, 89.0, 92.0, 94.0, 96.0, 98.0, 100.0),
    },
    "A2": {
        "page": 57, "title": "f2, compressor temperature",
        "x": ("pressure_ratio_ps3_p2", "-"), "y": ("temperature_ratio_t3_t2", "-"),
        "x_axis": (0.0, 20.0, 5.0), "y_axis": (1.1, 3.0, 0.1), "marker": "cross",
        "curves": {"f2": [
            (1.0, 1.304), (2.0, 1.447), (3.0, 1.569), (4.0, 1.685), (5.0, 1.783), (6.0, 1.872),
            (7.0, 1.952), (8.0, 2.023), (9.0, 2.088), (10.0, 2.149), (11.0, 2.206),
            (12.0, 2.261), (13.0, 2.318), (14.0, 2.373), (15.0, 2.432), (16.0, 2.493),
            (17.0, 2.562), (18.0, 2.635), (19.0, 2.719), (20.0, 2.81),
        ]},
    },
    "A3": {
        "page": 58, "title": "f3, seal-pressurization bleed fraction",
        "x": ("corrected_gas_generator_speed", "%"), "y": ("bleed_fraction_b1", "-"),
        "x_axis": (65.0, 100.0, 5.0), "y_axis": (-0.02, 0.12, 0.01), "marker": "cross",
        "curves": {"f3": [
            (65.0, 0.1091), (77.9, 0.1091), (78.9, 0.1085), (79.9, 0.1063), (80.9, 0.1026),
            (81.9, 0.0976), (82.9, 0.0891), (83.9, 0.0738), (85.0, 0.05), (87.0, 0.013),
            (88.1, 0.0022), (89.1, -0.0004), (100.0, -0.0003),
        ]},
    },
    "A4": {
        "page": 59, "title": "f4, power-turbine-balance bleed fraction",
        "x": ("corrected_airflow_wa2c", "lbm/s"), "y": ("bleed_fraction_b2", "-"),
        "x_axis": (3.0, 12.0, 1.0), "y_axis": (0.0088, 0.0108, 0.0002), "marker": "cross",
        "curves": {"f4": [(2.95, 0.010570), (5.0, 0.008999), (11.98, 0.0089985)]},
    },
    "A5": {
        "page": 60, "title": "f5, impeller tip leakage and turbine cooling-bleed fraction",
        "x": ("corrected_airflow_wa2c", "lbm/s"), "y": ("bleed_fraction_b3", "-"),
        "x_axis": (3.0, 12.0, 1.0), "y_axis": (0.0770, 0.0860, 0.0005), "marker": "cross",
        "curves": {"f5": [(2.97, 0.085105), (8.27, 0.084600), (11.98, 0.077885)]},
    },
    "A6": {
        "page": 61, "title": "f6, combustor efficiency",
        "x": ("fuel_air_ratio", "-"), "y": ("combustor_efficiency", "-"),
        "x_axis": (0.010, 0.020, 0.001), "y_axis": (0.88, 1.10, 0.02), "marker": "cross",
        "curves": {"f6": [(0.010, 0.984), (0.020, 0.984)]},
    },
    "A7": {
        "page": 62, "title": "f7, gas-generator turbine energy",
        "x": ("pressure_ratio_p45_p41", "-"), "y": ("enthalpy_drop_parameter", "Btu/lbm"),
        "x_axis": (0.20, 0.35, 0.01), "y_axis": (20.0, 52.5, 2.5),
        "marker": "cross",
        "curves": {"f7": [(0.200, 48.33), (0.2145, 40.36), (0.2195, 39.11), (0.2245, 38.33),
                          (0.2296, 37.56), (0.350, 24.67)]},
    },
    "A8": {
        "page": 63, "title": "f8, power turbine energy",
        "x": ("pressure_ratio_p49_p45", "-"), "y": ("enthalpy_drop_parameter", "Btu/lbm"),
        "x_axis": (0.30, 0.85, 0.05), "y_axis": (-5.0, 40.0, 2.5), "marker": "cross",
        "curves": {"f8": [
            (0.3, 34.0), (0.35, 30.4), (0.4, 26.7), (0.45, 23.3), (0.5, 20.2), (0.55, 17.2),
            (0.6, 14.3), (0.65, 11.3), (0.7, 8.3), (0.75, 5.2), (0.8, 2.0), (0.85, -1.2),
        ]},
    },
    "A9": {
        "page": 64, "title": "f9, power turbine mass flow",
        "x": ("pressure_ratio_ps9_p45", "-"), "y": ("corrected_flow_w45c", "lbm/s"),
        "x_axis": (0.30, 0.85, 0.05), "y_axis": (0.25, 0.39, 0.02), "marker": "cross",
        "curves": {"f9": [
            (0.29679, 0.37199), (0.322, 0.37206), (0.34705, 0.37178), (0.37215, 0.37118),
            (0.39737, 0.37023), (0.42242, 0.36888), (0.44777, 0.36721), (0.47288, 0.36515),
            (0.4975, 0.36274), (0.52267, 0.36003), (0.54797, 0.35706), (0.57257, 0.3547),
            (0.59754, 0.351), (0.62267, 0.34612), (0.64773, 0.34002), (0.6727, 0.33295),
            (0.698, 0.32485), (0.72332, 0.3159), (0.74856, 0.30625), (0.77295, 0.29648),
            (0.799, 0.28493), (0.8249, 0.27318), (0.84893, 0.26083),
        ]},
    },
    "A10": {
        "page": 65, "title": "f10, exhaust pressure loss",
        "x": ("corrected_gas_generator_speed", "%"), "y": ("exhaust_pressure_ratio_ps9_p49", "-"),
        "x_axis": (65.0, 100.0, 5.0), "y_axis": (1.01, 1.14, 0.02), "marker": "cross",
        "curves": {"f10": [
            (64.701, 1.1238), (69.688, 1.1181), (70.705, 1.1162), (75.812, 1.0903),
            (78.364, 1.0791), (81.451, 1.0601), (85.462, 1.037), (86.472, 1.0305),
            (87.479, 1.0267), (88.702, 1.0236), (89.973, 1.0213), (90.975, 1.021),
            (91.969, 1.0218), (93.811, 1.0276), (95.989, 1.0338), (97.474, 1.0441),
            (99.326, 1.0638), (99.81, 1.0656),
        ]},
    },
    "A11": {
        "page": 66, "title": "f_hs, station 4.1 heat-sink constant",
        "x": ("corrected_gas_generator_speed", "%"), "y": ("heat_sink_constant_t41sgn", "lbm/s"),
        "x_axis": (65.0, 100.0, 5.0), "y_axis": (3.5, 8.0, 0.5), "marker": "cross",
        "curves": {"f_hs": [
            (65.0, 4.0), (70.0, 4.0), (75.0, 5.75), (80.0, 6.2), (85.0, 7.5), (100.0, 7.5),
        ]},
    },
    "C23": {
        "page": 94, "title": "F_EC1, ECU thermocouple sensor time constant (one curve per T45L)",
        "x": ("power_turbine_flow_parameter_w45r", "-"), "y": ("sensor_time_constant_tau45", "s"),
        "x_axis": (0.0, 16.0, 2.0), "y_axis": (1.0, 6.0, 1.0), "marker": "glyph",
        # The curves' symbols 1-4, from the figure's legend: T45L [degR].
        "curves": {
            1260.0: [(1.0, 5.0), (2.5, 3.95), (6.5, 2.55), (10.0, 2.2), (15.0, 1.97)],
            1660.0: [(1.0, 4.2), (2.5, 3.25), (6.5, 2.35), (10.0, 2.0), (15.0, 1.8)],
            2060.0: [(1.0, 3.65), (2.5, 2.98), (6.5, 2.15), (10.0, 1.92), (15.0, 1.72)],
            2460.0: [(1.0, 3.42), (2.5, 2.78), (6.5, 2.07), (10.0, 1.86), (15.0, 1.70)],
        },
    },
    "C24": {
        "page": 95, "title": "F_HM1, HMU topping line schedule",
        "x": ("inlet_temperature_t2", "degR"), "y": ("topping_line_parameter_wfptp", "-"),
        "x_axis": (390.0, 640.0, 10.0), "y_axis": (-0.5, 4.0, 0.25), "marker": "cross",
        "x_major_min": 3.0,  # the long ticks at 490 and 590 aren't found: fit every one
        "curves": {"wfptp": [
            (395.1, 0.0799), (501.9, 3.485), (515.1, 3.30), (535.1, 3.125), (555.0, 3.106),
            (575.0, 3.198), (594.9, 3.27), (614.7, 3.324), (633.6, 3.357),
        ]},
    },
    "C25": {
        "page": 96, "title": "F_HM2, HMU power-available input schedule",
        "x": ("power_available_spindle_angle", "deg"),
        "y": ("power_available_parameter_wfprf", "-"),
        "x_axis": (20.0, 120.0, 20.0), "y_axis": (-2.0, 12.0, 1.0), "marker": "cross",
        "curves": {"wfprf": [
            (28.51, 10.4), (30.11, 9.841), (40.09, 7.003), (50.1, 5.732), (60.02, 4.829),
            (70.0, 3.818), (79.89, 2.808), (89.9, 1.896), (99.9, 1.19), (109.9, 0.781),
            (119.9, -0.05),
        ]},
    },
    "C26": {
        "page": 97, "title": "F_HM3, HMU load-demand-compensation schedule function 1",
        "x": ("load_demand_spindle_angle_xldsa", "deg"),
        "y": ("gas_generator_speed_demand_png", "%"),
        "x_axis": (0.0, 100.0, 10.0), "y_axis": (75.0, 112.5, 2.5), "marker": "cross",
        "curves": {"png": [
            (0.09, 80.04), (10.01, 83.76), (19.91, 87.24), (29.9, 90.57), (39.95, 93.39),
            (49.95, 95.73), (59.97, 98.75), (70.01, 102.2), (79.94, 107.3), (89.88, 107.3),
            (99.82, 107.3),
        ]},
    },
    "C27": {
        "page": 98, "title": "F_HM4, HMU load-demand-compensation schedule function 2",
        "x": ("load_demand_spindle_angle_xldsa", "deg"),
        "y": ("delta_demand_parameter_wfdps3", "-"),
        "x_axis": (0.0, 100.0, 20.0), "y_axis": (2.1, 3.7, 0.2), "marker": "cross",
        "curves": {"wfdps3": [
            (0.37, 2.299), (10.13, 2.324), (20.03, 2.358), (30.11, 2.526), (39.95, 2.69),
            (49.97, 2.877), (59.98, 3.118), (69.97, 3.396), (79.9, 3.569), (89.74, 3.57),
            (99.58, 3.569),
        ]},
    },
    "C28": {
        "page": 99, "title": "F_HM5, HMU idle-schedule function 1",
        "x": ("inlet_temperature_t2", "degR"), "y": ("idle_schedule_limit_parameter_wfirf", "-"),
        "x_axis": (390.0, 640.0, 10.0), "y_axis": (2.05, 2.65, 0.05), "marker": "cross",
        "x_major_min": 3.0,  # the long ticks at 490 and 590 aren't found: fit every one
        "curves": {"wfirf": [
            (395.1, 2.1), (415.0, 2.129), (435.0, 2.16), (454.9, 2.192), (474.9, 2.226),
            (494.8, 2.263), (514.7, 2.301), (534.9, 2.341), (554.8, 2.383), (574.8, 2.428),
            (594.9, 2.474), (614.8, 2.522), (633.6, 2.57),
        ]},
    },
    "C29": {
        "page": 100, "title": "F_HM6, HMU idle-schedule function 2",
        "x": ("inlet_temperature_t2", "degR"), "y": ("droop_line_idle_reference_pcngi", "%"),
        "x_axis": (390.0, 640.0, 10.0), "y_axis": (62.0, 75.0, 2.0), "marker": "cross",
        "x_major_min": 3.0,  # the long ticks at 490 and 590 aren't found: fit every one
        "curves": {"pcngi": [(395.1, 63.9), (633.9, 73.54)]},
    },
    "C30": {
        "page": 101, "title": "F_HM7, HMU maximum fuel-parameter limit during acceleration "
                             "(one curve per T2)",
        "x": ("sensed_gas_generator_speed_pcnghl", "%"),
        "y": ("acceleration_limit_parameter_wfpac", "-"),
        "x_axis": (50.0, 110.0, 10.0), "y_axis": (0.0, 5.0, 1.0), "marker": "glyph",
        "search": True,  # readings to a glyph's size; the fit finds each segment's ink
        # The speeds the curves share points at, read off the glyphs' columns (on_columns).
        "columns": (54.0, 60.0, 64.0, 68.0, 72.0, 76.0, 80.0, 84.0, (86.5, "fixed"),
                    (87.25, "fixed"), 90.3, 92.35, 94.6, 96.7, 101.4, 104.0),
        # The curves' symbols 1-7, from the figure's legend: T2 [degR].
        "curves": {
            395.0: [(54.0, 3.638), (60.0, 3.261), (64.0, 3.086), (68.0, 3.010), (72.0, 2.985),
                    (76.0, 3.092), (80.0, 3.255), (84.0, 3.42), (86.5, 3.405), (90.3, 2.465),
                    (92.35, 1.964), (94.6, 1.0), (96.7, 1.0)],
            430.0: [(54.0, 3.73), (60.0, 3.494), (64.0, 3.327), (68.0, 3.199), (72.0, 3.137),
                    (76.0, 3.137), (80.0, 3.267), (84.0, 3.43), (86.5, 3.53), (87.25, 3.553),
                    (90.3, 3.553), (92.35, 3.04), (94.6, 2.488), (96.7, 2.004), (101.4, 1.0),
                    (104.0, 1.0)],
            460.0: [(54.0, 3.75), (60.0, 3.663), (64.0, 3.517), (68.0, 3.379), (72.0, 3.289),
                    (76.0, 3.261), (80.0, 3.295), (84.0, 3.435), (86.5, 3.55), (90.3, 3.71),
                    (92.35, 3.697), (94.6, 3.70), (96.7, 3.18), (101.4, 1.987), (104.0, 1.0)],
            480.0: [(54.0, 3.77), (60.0, 3.711), (64.0, 3.610), (68.0, 3.498), (72.0, 3.396),
                    (76.0, 3.331), (80.0, 3.345), (84.0, 3.44), (86.5, 3.56), (90.3, 3.72),
                    (92.35, 3.787), (94.6, 3.787), (96.7, 3.787), (101.4, 2.612), (104.0, 1.981)],
            519.0: [(54.0, 3.70), (60.0, 3.830), (64.0, 3.785), (68.0, 3.717), (72.0, 3.633),
                    (76.0, 3.531), (80.0, 3.469), (84.0, 3.509), (86.5, 3.58), (90.3, 3.72),
                    (92.35, 3.64), (94.6, 3.62), (96.7, 3.59), (101.4, 3.488), (104.0, 2.865)],
            555.0: [(54.0, 3.69), (60.0, 3.85), (64.0, 3.897), (68.0, 3.877), (72.0, 3.824),
                    (76.0, 3.723), (80.0, 3.644), (84.0, 3.604), (86.5, 3.625), (87.25, 3.658),
                    (90.3, 3.71), (92.35, 3.646), (94.6, 3.595), (96.7, 3.545), (101.4, 3.432),
                    (104.0, 2.78)],
            590.0: [(54.0, 3.68), (60.0, 3.864), (64.0, 3.959), (68.0, 4.012), (72.0, 3.993),
                    (76.0, 3.914), (80.0, 3.824), (84.0, 3.745), (86.5, 3.741), (87.25, 3.741),
                    (90.3, 3.70), (92.35, 3.612), (94.6, 3.561), (96.7, 3.505), (101.4, 3.398),
                    (104.0, 2.645)],
        },
    },
}


def frame_rows(dark, xa, xb, cover=0.6):
    """Rows inked over at least `cover` of [xa, xb] (the scans' bottom edges are slightly
    bowed, so less than digitize_ballin_d's 90 %), grouped: the frame's top and bottom."""
    d = dark.copy()
    d[1:] |= dark[:-1]
    d[:-1] |= dark[1:]
    frac = d[:, xa + 3:xb - 2].mean(axis=1)
    groups = []
    for y in np.flatnonzero(frac > cover):
        if groups and y - groups[-1][-1] <= 3:
            groups[-1].append(y)
        else:
            groups.append([y])
    return [float(np.mean(g)) for g in groups]


def geometry(doc, page):
    """The deskewed page image (dark mask) and the frame (left, right, top, bottom) [px]."""
    rot = dbd.deskew(doc, page, 0.0)
    img = dbd.render(doc, page, rot, DPI)
    dark = img < dbd.DARK
    inner = dark.copy()  # a scan line along the page's edge outinks any frame side
    inner[:, :100] = False
    inner[:, -100:] = False
    sides = dbd.frame_sides(inner)
    if sides is None:
        raise RuntimeError(f"page {page}: no frame sides")
    xa, xb = sides
    # The top and bottom, of the rows inked across: where the left side line (within ±12 px,
    # as the scans' verticals lean slightly) starts below and stops above. A flat stretch of a
    # curve has the side line on both sides; a line outside the frame, on neither.
    rows = frame_rows(dark, xa, xb, cover=0.4)
    side = dark[:, int(xa) - 12:int(xa) + 13].any(axis=1).astype(float)
    reach = 200

    def below(r):
        after = side[int(r) + 5:int(r) + reach].mean()
        return after - side[max(int(r) - reach, 0):int(r) - 5].mean()

    if len(rows) < 2:
        raise RuntimeError(f"page {page}: frame top and bottom not found ({rows})")
    top = max(rows, key=below)
    bot = min(rows, key=below)
    if not (below(top) > 0.25 and below(bot) < -0.25 and bot - top > 0.3 * dark.shape[0]):
        raise RuntimeError(f"page {page}: frame edges unclear (top {top}, bottom {bot})")
    return img, dark, (xa, xb, top, bot), rot


def edge_line(dark, along, at, outward, lo, hi, window=14, step=8):
    """The centre line of a frame edge as a function of position: `along` 'x' for the bottom
    or top edge near row `at` (positions are columns), 'y' for a side near column `at`
    (positions are rows); `outward` the direction away from the plot (+1 down or right).
    At each position the edge's outer side is the farthest inked pixel within `window` of
    `at` (tick stubs and curves leave the edges inwards), less half the edge's median
    thickness. A quadratic is fitted, dropping positions more than 1.5 px off (a curve or a
    label touching the edge) until none is. Returns the polynomial."""
    pos, outer, thick = [], [], []
    for p in range(int(lo) + 20, int(hi) - 19, step):
        if along == "x":
            line = dark[int(at) - window:int(at) + window + 1, p]
        else:
            line = dark[p, int(at) - window:int(at) + window + 1]
        hits = np.flatnonzero(line)
        if hits.size == 0:
            continue
        k = hits[-1] if outward > 0 else hits[0]
        n = 0  # the run of ink from the outer side inwards
        while 0 <= k - outward * n < line.size and line[k - outward * n]:
            n += 1
        pos.append(p)
        outer.append(int(at) - window + k)
        thick.append(n)
    pos, outer = np.array(pos, float), np.array(outer, float)
    half = 0.5 * (float(np.median(thick)) - 1.0)
    centre = outer - outward * half
    keep = np.ones(len(pos), bool)
    for _ in range(10):
        c = np.polyfit(pos[keep], centre[keep], 2)
        res = np.abs(np.polyval(c, pos) - centre)
        new = res < 1.5
        if (new == keep).all():
            break
        keep = new
    if keep.mean() < 0.6:
        raise RuntimeError(f"frame edge near {at:.0f}: under 60 % of it on one line")
    return np.poly1d(c)


class Frame:
    """The plot's frame on a scan that is sheared and bowed slightly: its four edges as
    curves (left and right sides as functions of the row, bottom and top of the column), and
    the map between pixels and the fractions (u, v) across it (0 at the left and bottom
    edges, 1 at the right and top), along the local edges."""

    def __init__(self, dark, xa, xb, top, bot):
        self.left = edge_line(dark, "y", xa, -1, top, bot)
        self.right = edge_line(dark, "y", xb, +1, top, bot)
        self.bottom = edge_line(dark, "x", bot, +1, xa, xb)
        self.top = edge_line(dark, "x", top, -1, xa, xb)
        self.box = (xa, xb, top, bot)

    def to_uv(self, px, py):
        u = v = 0.0
        for _ in range(6):  # each edge depends on the other coordinate; converges at once
            yb = self.bottom(px)
            yt = self.top(px)
            v = (yb - py) / (yb - yt)
            xl, xr = self.left(py), self.right(py)
            u = (px - xl) / (xr - xl)
        return float(u), float(v)

    def to_px(self, u, v):
        xa, xb, top, bot = self.box
        px, py = xa + u * (xb - xa), bot - v * (bot - top)
        for _ in range(8):
            xl, xr = self.left(py), self.right(py)
            px = xl + u * (xr - xl)
            yb, yt = self.bottom(px), self.top(px)
            py = yb - v * (yb - yt)
        return float(px), float(py)


def tick_stubs(dark, along, at, inward, lo, hi, window=14, reach=40):
    """Positions [px] along an edge of the stubs that leave it inwards, with their lengths:
    `along` 'x' for a horizontal edge near row `at` (inward −1, up), 'y' for a vertical edge
    near column `at` (inward +1, right). The scans' edges are slightly bowed, so the edge is
    followed: at each position its outer side is the farthest inked pixel within `window` of
    `at`, the ink run from there inwards is the edge plus any stub, and the edge's own
    thickness (the median run) is taken off."""
    runs = []
    for p in range(int(lo) + 6, int(hi) - 5):
        if along == "x":
            line = dark[int(at) - window:int(at) + window + 1, p][::-1]  # outer side first
        else:
            line = dark[p, int(at) - window:int(at) + window + 1]
        hits = np.flatnonzero(line)
        if hits.size == 0:
            runs.append((p, 0))
            continue
        k = hits[0]
        n = 0
        seq = (dark[int(at) + window - k - m, p] if along == "x"
               else dark[p, int(at) - window + k + m] for m in range(window + reach))
        for v in seq:
            if not v:
                break
            n += 1
        runs.append((p, n))
    thick = float(np.median([n for _, n in runs]))
    ticks, run = [], []
    for p, n in runs:
        if n - thick >= 3:
            run.append((p, n - thick))
        elif run:
            ticks.append((float(np.mean([q for q, _ in run])), max(m for _, m in run)))
            run = []
    if run:
        ticks.append((float(np.mean([q for q, _ in run])), max(m for _, m in run)))
    return ticks


def calibrate(ticks, lo, hi, step, edge_lo, edge_hi, axis, major_min=8.0):
    """The map from a fraction across the frame (edge_lo, edge_hi: 0 and 1, the Frame's u or
    v) to a value along an axis. A person reads off the scan the values at the frame's edges
    (lo, hi; the corner labels sit on the corners) and the major-tick step.

    The values are linear between the frame's edges; the tick stubs check that scale and
    measure the scan's uneven stretch:
    - the major ticks (stubs of at least `major_min` px), each given the nearest multiple of
      the step on the frame's scale (a second claimant to a value, as a curve's ink beside a
      tick, is dropped), are fitted by a line; it must put each within TICK_TOL of the span,
      and the frame's edges within 1 % of lo and hi (once the frame's shear is followed, they
      fall within 0.25 %: the plotted ticks sit on the frame's scale);
    - every stub, minor ones too (the minor step is the major step over the ratio of the
      stubs' spacings, 1, 2, 4, 5 or 10), is given its value on that line, and the ticks'
      departure from the line, interpolated between them, is the scan's uneven stretch
      (up to a few tenths of a percent of an axis), added to the frame's scale.
    Returns ((to_value, to_pixel), details)."""
    span = hi - lo
    to_v = lambda p: lo + (p - edge_lo) * span / (edge_hi - edge_lo)  # noqa: E731

    def assign(stubs, st, ref):
        best = {}
        for p in stubs:
            k = round((ref(p) - lo) / st)
            v = lo + k * st
            off = abs(ref(p) - v)
            if off < 0.25 * st and min(lo, hi) - 1e-9 <= v <= max(lo, hi) + 1e-9:
                if k not in best or off < best[k][2]:
                    best[k] = (p, v, off)
        return sorted((p, v) for p, v, _ in best.values())

    major = assign([p for p, n in ticks if n >= major_min], step, to_v)
    if len(major) < 3:
        raise RuntimeError(f"{axis}: fewer than three major ticks: {major}")
    dropped = []
    while True:
        a, b = np.polyfit([p for p, _ in major], [v for _, v in major], 1)
        off = [abs(a * p + b - v) / abs(span) for p, v in major]
        worst = max(off)
        if worst <= TICK_TOL:
            break
        # A curve's ink at the frame's edge can pass for a tick: drop the worst, at most two.
        if len(dropped) == 2 or len(major) < 5:
            raise RuntimeError(f"{axis}: a major tick {worst:.4f} of the span off the fit")
        dropped.append(major.pop(int(np.argmax(off))))
    edges = max(abs(a * edge_lo + b - lo), abs(a * edge_hi + b - hi)) / abs(span)
    if edges > 0.01:
        raise RuntimeError(f"{axis}: the ticks put the frame's edges {edges:.4f} of the span "
                           f"off {lo}, {hi}: check the edge values")
    pos = np.array(sorted(p for p, _ in ticks if all(p != q for q, _ in dropped)))
    minor_px = float(np.median(np.diff(pos)))
    major_px = abs(step / a)
    ratio = min((1, 2, 4, 5, 10), key=lambda r: abs(major_px / minor_px - r))
    minor = step / ratio
    every = assign(list(pos), minor, lambda p: a * p + b)
    px = np.array([p for p, _ in every])
    dev = np.array([v - (a * p + b) for p, v in every])  # the scan's stretch at each tick
    stretch = float(np.max(np.abs(dev)) / abs(span))
    # The ticks' offset from the frame's scale, at mid-axis.
    mid = 0.5 * (edge_lo + edge_hi)
    offset = float((a * mid + b - to_v(mid)) / abs(span))

    def to_value(p):
        p = np.asarray(p, float)
        return to_v(p) + np.interp(p, px, dev)  # constant beyond the outermost ticks

    grid = np.linspace(min(edge_lo, edge_hi) - 0.1 * abs(edge_hi - edge_lo),
                       max(edge_lo, edge_hi) + 0.1 * abs(edge_hi - edge_lo), 20001)
    gv = to_value(grid)
    if gv[0] > gv[-1]:
        grid, gv = grid[::-1], gv[::-1]

    def to_pixel(v):
        return float(np.interp(v, gv, grid))

    return (to_value, to_pixel), {"majors": len(major), "worst": worst, "edges": edges,
                                  "ticks": len(every), "minor": minor, "stretch": stretch,
                                  "offset": offset, "dropped": [v for _, v in dropped]}


def grid_overlay(img, frame, axes, x_axis, y_axis, path, fine=5):
    """The scan with the axes' maps drawn over it: a line at every major tick (blue) and
    `fine` divisions between them (light), each major line labelled, for reading breakpoints.
    The lines follow the frame's shear and bow."""
    _, to_pix = axes
    xa, xb, top, bot = frame.box
    im = Image.fromarray(img).convert("RGB")
    d = ImageDraw.Draw(im)
    (x0, x1, xs), (y0, y1, ys) = x_axis, y_axis
    for (lo, hi, step), vertical in ((x_axis, True), (y_axis, False)):
        n = int(round(abs(hi - lo) / step * fine))
        for k in range(n + 1):
            v = min(lo, hi) + k * step / fine
            major = k % fine == 0
            colour = (0, 90, 255) if major else (150, 200, 255)
            other = np.linspace(y0, y1, 40) if vertical else np.linspace(x0, x1, 40)
            pts = [to_pix(v, o) if vertical else to_pix(o, v) for o in other]
            d.line(pts, fill=colour, width=2 if major else 1)
            if major:
                px, py = pts[0]
                d.text((px + 3, py - 16) if vertical else (px + 4, py - 13), f"{v:g}",
                       fill=(0, 0, 200))
    im.crop((int(xa) - 40, int(top) - 40, int(xb) + 40, int(bot) + 40)).save(path)


def find_crosses(dark, frame, fill=0.75):
    """Candidate cross markers inside the frame: pixels where both diagonals (±MARK px) are
    at least `fill` inked (a line inks one at most), merged within MARK px. [(px, py)]"""
    xa, xb, top, bot = frame.box
    y0, y1, x0, x1 = int(top) + 4, int(bot) - 3, int(xa) + 4, int(xb) - 3
    d1 = np.zeros((y1 - y0, x1 - x0), int)
    d2 = np.zeros_like(d1)
    for k in range(-MARK, MARK + 1):
        d1 += dark[y0 + k:y1 + k, x0 + k:x1 + k]
        d2 += dark[y0 + k:y1 + k, x0 - k:x1 - k]
    ys, xs = np.nonzero(np.minimum(d1, d2) >= fill * (2 * MARK + 1))
    found = []
    for y, x in sorted(zip(ys, xs), key=lambda q: q[1]):
        for c in found:
            if abs(c[0][-1] - x) <= MARK and abs(np.mean(c[1]) - y) <= MARK:
                c[0].append(x)
                c[1].append(y)
                break
        else:
            found.append(([x], [y]))
    return [(float(np.mean(cx)) + x0, float(np.mean(cy)) + y0) for cx, cy in found]


def snap_cross(dark, px, py):
    """The centre of the cross marker nearest pixel (px, py): the pixel within SNAP_MAX whose
    two diagonals (±MARK px) are most inked, the mean of the best-scoring ones."""
    best, where = -1, []
    for y in range(int(py) - SNAP_MAX, int(py) + SNAP_MAX + 1):
        for x in range(int(px) - SNAP_MAX, int(px) + SNAP_MAX + 1):
            sc = 0
            for k in range(-MARK, MARK + 1):
                sc += int(dark[y + k, x + k]) + int(dark[y + k, x - k])
            if sc > best:
                best, where = sc, [(x, y)]
            elif sc == best:
                where.append((x, y))
    if best < 2 * (2 * MARK + 1) * 0.6:
        raise RuntimeError(f"no cross near ({px:.0f}, {py:.0f}): best diagonal ink {best}")
    return float(np.mean([w[0] for w in where])), float(np.mean([w[1] for w in where]))


def read_figure(dark, axes, f):
    """Each curve's breakpoints, snapped to their markers: {curve: [(x, y, dx_px, dy_px)]}."""
    to_xy, to_pix = axes
    out = {}
    for curve, pts in f["curves"].items():
        rows = []
        for x, y in pts:
            px, py = to_pix(x, y)
            sx, sy = snap_cross(dark, px, py)
            if np.hypot(sx - px, sy - py) > SNAP_MAX:
                raise RuntimeError(f"{curve}: snap from ({x}, {y}) moved too far")
            vx, vy = to_xy(sx, sy)
            rows.append((vx, vy, sx - px, sy - py, sx, sy))
        out[curve] = rows
    return out


GLYPH = 20  # half-size of a number glyph's neighbourhood, kept out of a segment's fit [px]
SEARCH = 12  # how far a reading's ends are moved to find its segment's ink (C30) [px]
SHORT = 10   # least length of a segment's middle, between its glyphs, to fit it [px]


def read_glyph_polylines(dark, axes, f):
    """Breakpoints of polylines marked with number glyphs centred on them (Figs. C23, C30):
    {curve: [(x, y, how, 0, 0, px, py)]}. A person's readings (to about a glyph's size) give
    each curve's segments; each segment's own ink (within 4 px of the reading's straight
    line, beyond GLYPH px of its ends) is fitted by a line, re-fitted once to the ink within
    2.5 px of the first fit. An interior breakpoint is where its two segments' lines meet
    ('meet') when they bend by over 4 deg, else the reading projected onto their mean line
    ('on-line'); an end point is the reading projected onto its segment's line ('end': its
    position along the line is the person's, to about 0.1 of a glyph)."""
    to_xy, to_pix = axes
    h, w = dark.shape
    ys, xs = np.nonzero(dark)
    ink = np.column_stack([xs, ys]).astype(float)

    def inked(p):
        x, y = int(round(p[0])), int(round(p[1]))
        return 0 <= y < h and 0 <= x < w and dark[y, x]

    def search(a, b):
        """The reading's ends moved along its normal (each by up to SEARCH px) to where a line
        between them runs on ink over most of the segment's middle: Fig. C30's crossing curves
        need readings closer than a glyph to start the fit."""
        d = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([-d[1], d[0]])
        length = float(np.linalg.norm(b - a))
        ts = np.arange(GLYPH, length - GLYPH, 1.0)
        best = (-1.0, a, b)
        for oa in range(-SEARCH, SEARCH + 1):
            for ob in range(-SEARCH, SEARCH + 1):
                a2, b2 = a + oa * nrm, b + ob * nrm
                dd = (b2 - a2) / np.linalg.norm(b2 - a2)
                n2 = np.array([-dd[1], dd[0]])
                hit = sum(any(inked(a2 + t * dd + s * n2) for s in (-1, 0, 1)) for t in ts)
                score = hit / len(ts)
                if score > best[0]:
                    best = (score, a2, b2)
        if best[0] < 0.8:
            raise RuntimeError(f"a segment from {to_xy(*a)}: no line on ink "
                               f"(best {best[0]:.2f} of its length)")
        return best[1], best[2]

    def fit(a, b):
        if search_ink:
            a, b = search(a, b)
        d = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([-d[1], d[0]])
        rel = ink - a
        t, r = rel @ d, rel @ nrm
        length = float(np.linalg.norm(b - a))
        band = (t > GLYPH) & (t < length - GLYPH)
        sel = band & (np.abs(r) < 4)
        for tol in (None, 2.5):
            q = ink[sel]
            if len(q) < 15:
                raise RuntimeError(f"a segment from {to_xy(*a)} has {len(q)} px of ink")
            m = q.mean(0)
            dd = np.linalg.svd(q - m)[2][0]
            if tol is not None:
                break
            nn = np.array([-dd[1], dd[0]])
            sel = band & (np.abs((ink - m) @ nn) < 2.5)
        return m, dd if dd @ d > 0 else -dd

    def meet(l1, l2):
        (m1, d1), (m2, d2) = l1, l2
        a = np.column_stack([d1, -d2])
        t = np.linalg.solve(a, m2 - m1)
        return m1 + t[0] * d1

    def project(p, line):
        m, d = line
        return m + ((p - m) @ d) * d

    out = {}
    search_ink = f.get("search", False)
    fitted = {}  # curve: (its points' near lines, each point's bend [deg] or None)
    for curve, pts in f["curves"].items():
        px = [np.array(to_pix(x, y)) for x, y in pts]
        # Segments too short to fit between their glyphs (C30's) get no line.
        lines = [fit(px[i], px[i + 1])
                 if np.linalg.norm(px[i + 1] - px[i]) > 2 * GLYPH + SHORT else None
                 for i in range(len(px) - 1)]
        rows, nears, bends = [], [], []
        for i, p in enumerate(px):
            near = [ln for ln in (lines[i - 1] if i > 0 else None,
                                  lines[i] if i < len(lines) else None) if ln is not None]
            bend = None
            if not near:
                q, how = p, "read"
            elif i == 0 or i == len(px) - 1:
                q, how = project(p, near[0]), "end"
            elif len(near) == 1:
                q, how = project(p, near[0]), "side"
            else:
                bend = np.degrees(np.arccos(np.clip(lines[i - 1][1] @ lines[i][1], -1, 1)))
                if bend > 4:
                    q, how = meet(lines[i - 1], lines[i]), "meet"
                else:
                    d = lines[i - 1][1] + lines[i][1]
                    mean = (0.5 * (lines[i - 1][0] + lines[i][0]), d / np.linalg.norm(d))
                    q, how = project(p, mean), "on-line"
            if np.linalg.norm(q - p) > 2 * GLYPH:
                raise RuntimeError(f"{curve}: point {i} moved {np.linalg.norm(q - p):.0f} px "
                                   "from the reading")
            x, y = to_xy(*q)
            dx, dy = q - p
            rows.append((x, y, how, float(dx), float(dy), float(q[0]), float(q[1])))
            nears.append(near)
            bends.append(bend)
        out[curve] = rows
        fitted[curve] = (nears, bends, [x for x, _ in pts])
    if "columns" in f:
        out = on_columns(out, fitted, f["columns"], to_xy)
    return out


COLUMN_NEAR = 0.3  # largest distance [x units] of a person's reading from its column


def on_columns(out, fitted, columns, to_xy):
    """Fig. C30's points share columns (the curves were tabulated at the same speeds), and a
    meet of two segments bending by a few degrees is poorly placed along them (or, where the
    curves crowd, on a neighbour's segment). A point's column is the one its person's reading
    is in; the column's x is the median of the meets in it (the person's x where there are
    none), robust to a few misplaced ones, or as the person gives it, read off the glyphs'
    column, where too few meets are well placed (a column given as (x, "fixed")); each
    interior point in a column then takes that x, and its y where its fitted lines cross it
    (their mean): 'column'. End points and points without a fitted line keep their place
    (an end's x is not tabulated beyond it: the curves are clamped there)."""
    fixed = {c[0]: True for c in columns if isinstance(c, tuple)}
    columns = [c[0] if isinstance(c, tuple) else c for c in columns]

    def column_of(x):
        c = min(columns, key=lambda c: abs(c - x))
        return c if abs(c - x) < COLUMN_NEAR else None

    xs = {}
    for c in columns:
        meets = [r[0] for curve, rows in out.items()
                 for r, x0 in zip(rows, fitted[curve][2]) if r[2] == "meet" and column_of(x0) == c]
        xs[c] = c if c in fixed or not meets else float(np.median(meets))
        print(f"  column {c:g}: x = {xs[c]:.5g}"
              + (" (fixed)" if c in fixed else f" from {len(meets)} meets"))

    def at_x(line, x):
        m, d = line
        f0 = lambda t: to_xy(*(m + t * d))[0] - x  # noqa: E731
        lo, hi = -4000.0, 4000.0
        if f0(lo) * f0(hi) > 0:
            raise RuntimeError(f"a line doesn't cross x = {x}")
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if f0(lo) * f0(mid) <= 0:
                hi = mid
            else:
                lo = mid
        return m + 0.5 * (lo + hi) * d

    res = {}
    for curve, rows in out.items():
        new = []
        for r, near, x0 in zip(rows, fitted[curve][0], fitted[curve][2]):
            c = column_of(x0)
            if not near or c is None or r[2] == "end":
                new.append(r)
                continue
            q = np.mean([at_x(ln, xs[c]) for ln in near], axis=0)
            x, y = to_xy(*q)
            new.append((x, y, "column", r[3] + q[0] - r[5], r[4] + q[1] - r[6],
                        float(q[0]), float(q[1])))
        res[curve] = new
    return res


def components(mask):
    """The 8-connected components of a boolean mask: a list of (n, 2) arrays of (row, col)."""
    lab = np.zeros(mask.shape, np.int32)
    out = []
    h, w = mask.shape
    for y0, x0 in zip(*np.nonzero(mask)):
        if lab[y0, x0]:
            continue
        lab[y0, x0] = len(out) + 1
        stack, pts = [(y0, x0)], []
        while stack:
            y, x = stack.pop()
            pts.append((y, x))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and mask[yy, xx] and not lab[yy, xx]:
                        lab[yy, xx] = len(out) + 1
                        stack.append((yy, xx))
        out.append(np.array(pts))
    return out


def dot_chains(dots, spacing):
    """Straight runs of evenly spaced dots: each dot linked to the dot one spacing on in the
    dotted lines' direction (38-64 deg up and right; then, among the dots left over, 10-40
    deg, for the flatter segments), keeping only mutual links. Each run of at least two dots
    is fitted by a line: {m: a point on it, d: its unit direction (rightwards), t0, t1: the
    ends' positions along it, n}."""
    v = dots[None, :, :] - dots[:, None, :]
    dist = np.sqrt((v ** 2).sum(-1))
    ang = np.degrees(np.arctan2(-v[..., 1], v[..., 0]))

    def link(idx, lo, hi):
        member = np.isin(np.arange(len(dots)), idx)
        ok = (ang > lo) & (ang < hi) & (dist > 0.85 * spacing) & (dist < 1.18 * spacing)
        ok &= member[None, :] & member[:, None]
        succ = -np.ones(len(dots), int)
        for a in idx:
            js = np.flatnonzero(ok[a])
            if js.size:
                succ[a] = js[np.argmin(np.abs(dist[a, js] - spacing))]
        pred = -np.ones(len(dots), int)
        for a, b in enumerate(succ):
            if b >= 0 and (pred[b] < 0
                           or abs(dist[a, b] - spacing) < abs(dist[pred[b], b] - spacing)):
                pred[b] = a
        for a, b in enumerate(succ):
            if b >= 0 and pred[b] != a:
                succ[a] = -1
        runs = []
        for a in idx:
            if pred[a] >= 0:
                continue
            run = [a]
            while succ[run[-1]] >= 0:
                run.append(succ[run[-1]])
            runs.append(run)
        return runs

    runs = [r for r in link(np.arange(len(dots)), 38, 64) if len(r) >= 4]
    rest = np.array(sorted(set(range(len(dots))) - {i for r in runs for i in r}), int)
    runs += link(rest, 10, 40)
    chains = []
    for r in runs:
        if len(r) < 2:
            continue
        q = dots[r]
        m = q.mean(0)
        d = np.linalg.svd(q - m)[2][0]
        d = d if d[0] > 0 else -d
        t = (q - m) @ d
        chains.append({"m": m, "d": d, "t0": t.min(), "t1": t.max(), "n": len(r)})
    return chains


def read_dotted_map(dark, fr, axes, f):
    """Fig. A1's speed lines, by their geometry (see the module's docstring):
    {speed: [(x, y, how, dx, dy, px, py)]}, how one of 'run' (a flat run's ends), 'pair' (the
    middle of a glyph's gap between two chains), 'end' (half a gap beyond a chain's end)."""
    to_xy, to_pix = axes
    xa, xb, top, bot = fr.box
    # The dots: small compact components inside the frame, away from its edges and the legend.
    inner = dark[int(top) + 6:int(bot) - 6, int(xa) + 6:int(xb) - 6]
    dots = []
    for c in components(inner):
        if len(c) <= 60 and np.ptp(c[:, 0]) < 10 and np.ptp(c[:, 1]) < 10:
            dots.append((c[:, 1].mean() + int(xa) + 6, c[:, 0].mean() + int(top) + 6))
    dots = np.array(dots)
    legend_x, legend_y = to_pix(11.5, 6.2)  # the legend's colons are dots too
    near = ((dots[:, 0] > xa + 20) & (dots[:, 0] < xb - 20) & (dots[:, 1] > top + 20)
            & (dots[:, 1] < bot - 20))
    column = dots[near & (dots[:, 0] < to_pix(1.5, 6.0)[0])]  # the dotted line at a ratio of 1
    knee = dots[near & (dots[:, 0] > to_pix(2.0, 6.0)[0])
                & ~((dots[:, 0] > legend_x) & (dots[:, 1] > legend_y))]
    nn = np.sqrt(((knee[:, None, :] - knee[None, :, :]) ** 2).sum(-1))
    np.fill_diagonal(nn, np.inf)
    spacing = float(np.median(nn.min(1)))
    chains = dot_chains(knee, spacing)
    # The flat runs: rows inked across from a ratio of 1.6 to 2.4, lowest speed first.
    c0, c1 = int(to_pix(1.6, 6.0)[0]), int(to_pix(2.4, 6.0)[0])
    rows = np.flatnonzero(dark[:, c0:c1].mean(1) > 0.95)
    groups = []
    for r in rows:
        if groups and r - groups[-1][-1] <= 1:
            groups[-1].append(r)
        else:
            groups.append([r])
    runs = sorted((float(np.mean(g)) for g in groups if top + 10 < np.mean(g) < bot - 10),
                  reverse=True)
    speeds = f["speeds"]
    if len(runs) != len(speeds):
        raise RuntimeError(f"A1: {len(runs)} flat runs, not {len(speeds)}")
    x_one = float(np.median([to_xy(x, y)[0] for x, y in column]))
    run_y = [to_xy(to_pix(2.0, 6.0)[0], r)[1] for r in runs]

    def ink(pt, r=9):
        x, y = int(round(pt[0])), int(round(pt[1]))
        return int(dark[y - r:y + r + 1, x - r:x + r + 1].sum())

    # Pair a chain's upper end with another's lower end across a glyph: each end close to the
    # other chain's line (a slope change at a vertex puts it up to ~8 px off; neighbouring
    # dotted lines are 12-25 px apart), best first; a gap without a glyph is a missing dot.
    ups = [(c["m"] + c["t1"] * c["d"], c["d"], k) for k, c in enumerate(chains)]
    los = [(c["m"] + c["t0"] * c["d"], c["d"], k) for k, c in enumerate(chains)]
    cand = []
    for ia, (pa, da, ka) in enumerate(ups):
        for ib, (pb, db, kb) in enumerate(los):
            g = pb - pa
            if ka == kb or np.linalg.norm(g) > 4.5 * spacing or g @ da <= 0 or g @ db <= 0:
                continue
            lat = abs(g @ np.array([-da[1], da[0]])) + abs(g @ np.array([-db[1], db[0]]))
            if lat < 16:
                cand.append((lat, ia, ib))
    used_a, used_b, verts = set(), set(), []
    for _, ia, ib in sorted(cand):
        if ia in used_a or ib in used_b:
            continue
        used_a.add(ia)
        used_b.add(ib)
        mid = 0.5 * (ups[ia][0] + los[ib][0])
        if ink(mid) >= 50:
            verts.append({"p": mid, "how": "pair", "below": ups[ia][2], "above": los[ib][2]})
    gaps = [np.linalg.norm(los[v["above"]][0] - ups[v["below"]][0]) for v in verts]
    half = 0.5 * float(np.median(gaps))
    loose = []
    for ia, (pa, da, ka) in enumerate(ups):
        if ia not in used_a and ink(pa + half * da) >= 50:
            loose.append({"p": pa + half * da, "how": "end", "below": ka, "above": None})
    for ib, (pb, db, kb) in enumerate(los):
        if ib not in used_b and ink(pb - half * db) >= 50:
            loose.append({"p": pb - half * db, "how": "end", "below": None, "above": kb})
    for v in loose:  # one end from below and one from above, both unpaired: one vertex
        for w in verts:
            if (w["how"] == "end" and np.linalg.norm(w["p"] - v["p"]) < 14
                    and (w["below"] is None) != (v["below"] is None)):
                w["p"] = 0.5 * (w["p"] + v["p"])
                w["below"] = w["below"] if w["below"] is not None else v["below"]
                w["above"] = w["above"] if w["above"] is not None else v["above"]
                break
        else:
            verts.append(v)
    # Number the speed lines along the chains: a chain runs from point i on line k to point i
    # on line k + 1. Seeds: every vertex below the second run is on the first line, above the
    # tenth's highest point on the eleventh, and the lowest-ratio vertex at each run's height.
    at_low = {v["above"]: n for n, v in enumerate(verts) if v["above"] is not None}
    at_high = {v["below"]: n for n, v in enumerate(verts) if v["below"] is not None}
    seeds = {}
    for k, y in enumerate(run_y):
        on = [n for n, v in enumerate(verts)
              if abs(to_xy(*v["p"])[1] - y) < 0.04 and to_xy(*v["p"])[0] > 2.0]
        if on:
            seeds[min(on, key=lambda n: verts[n]["p"][0])] = k
    for n, v in enumerate(verts):
        y = to_xy(*v["p"])[1]
        if y < 0.5 * (run_y[0] + run_y[1]):
            seeds[n] = 0
        elif y > run_y[-2] + 0.05:
            seeds[n] = len(speeds) - 1
    line, stack = {}, list(seeds.items())
    while stack:
        n, k = stack.pop()
        if n in line:
            if line[n] != k:
                raise RuntimeError(f"A1: vertex at {to_xy(*verts[n]['p'])} on lines {line[n]}, {k}")
            continue
        line[n] = k
        v = verts[n]
        if v["above"] is not None and v["above"] in at_high:
            stack.append((at_high[v["above"]], k + 1))
        if v["below"] is not None and v["below"] in at_low:
            stack.append((at_low[v["below"]], k - 1))
    if len(line) != len(verts):
        lost = [to_xy(*v["p"]) for n, v in enumerate(verts) if n not in line]
        raise RuntimeError(f"A1: vertices on no speed line: {lost}")
    out = {}
    for k, speed in enumerate(speeds):
        pts = sorted((verts[n] for n in line if line[n] == k), key=lambda v: v["p"][0])
        if len(pts) != 6:
            raise RuntimeError(f"A1: {len(pts)} knee points on the {speed:g} % line, not 6")
        first = pts[0]  # it ends the flat run: where its dotted line crosses the run
        c = chains[first["above"] if first["above"] is not None else first["below"]]
        first["p"] = c["m"] + (runs[k] - c["m"][1]) / c["d"][1] * c["d"]
        first["how"] = "run"
        start = to_pix(x_one, run_y[k])
        rows = [(x_one, run_y[k], "run", 0.0, 0.0, *start)]
        for v in pts:
            x, y = to_xy(*v["p"])
            rows.append((x, y, v["how"], 0.0, 0.0, float(v["p"][0]), float(v["p"][1])))
        out[speed] = rows
    print(f"  {len(knee)} dots, spacing {spacing:.2f} px; {len(chains)} chains; the dotted line "
          f"at a ratio of 1 reads {x_one:.4f} ({len(column)} dots); median gap {2 * half:.1f} px")
    return out


def review_overlay(img, frame, readings, path):
    """The scan with each reading circled (red) and labelled with its values."""
    xa, xb, top, bot = frame.box
    im = Image.fromarray(img).convert("RGB")
    d = ImageDraw.Draw(im)
    for rows in readings.values():
        for x, y, *_, sx, sy in rows:
            d.ellipse([sx - 10, sy - 10, sx + 10, sy + 10], outline=(230, 0, 0), width=3)
            d.text((sx + 12, sy + 4), f"({x:.4g}, {y:.4g})", fill=(200, 0, 0))
    im.crop((int(xa) - 40, int(top) - 40, int(xb) + 40, int(bot) + 40)).save(path)


def write_csv(name, f, readings, cal_info):
    OUT.mkdir(parents=True, exist_ok=True)
    xn, xu = f["x"]
    yn, yu = f["y"]
    with open(OUT / f"{name}.csv", "w", newline="") as fh:
        how = ("the speed lines' points located by the dotted lines that join them "
               "(read_dotted_map); located: run (a flat run's ends), pair (between two dotted "
               "segments, about +-0.01 lbm/s), end (half a gap beyond one segment's end, about "
               "+-0.015 lbm/s and +-0.03 in ratio)" if f["marker"] == "dotted" else
               "the polylines' breakpoints from lines fitted to each segment's ink "
               "(read_glyph_polylines); located: meet (two segments' lines meet), on-line, side, read, end "
               "(a person's reading projected onto the end segment's line)"
               if f["marker"] == "glyph" else
               "the plotted polyline's marked breakpoints, each snapped to its marker")
        fh.write(f"# Digitized from Ballin, NASA TM-100991 (1988), Fig. {name}: {f['title']}, "
                 f"PDF p. {f['page']}, by tools/digitize_ballin_t700.py at {DPI} dpi:\n"
                 f"# {how}. {cal_info}\n")
        fh.write("# status: UNREVIEWED; the overlay needs a person's check (DATA_GAPS.md)\n")
        fh.write(f"# x: {xn} [{xu}], y: {yn} [{yu}]\n")
        w = csv.writer(fh, lineterminator="\n")
        dotted = f["marker"] in ("dotted", "glyph")
        w.writerow(["curve", xn, yn] + (["located"] if dotted else []))
        for curve, rows in readings.items():
            for x, y, *rest in rows:
                w.writerow([f"{curve:g}" if not isinstance(curve, str) else curve,
                            f"{x:.5g}", f"{y:.5g}"]
                           + ([rest[0]] if dotted else []))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("pdf")
    ap.add_argument("overlays")
    ap.add_argument("--only")
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--probe", action="store_true",
                    help="print the major stubs on the frame-edge scale and stop")
    args = ap.parse_args()
    doc = fitz.open(args.pdf)
    Path(args.overlays).mkdir(parents=True, exist_ok=True)
    for name, f in FIGURES.items():
        if args.only and name != args.only:
            continue
        if f["marker"] != "dotted" and not f["curves"] and not (args.grid or args.probe):
            print(name, "not read yet: skipped")
            continue
        img, dark, (xa, xb, top, bot), rot = geometry(doc, f["page"])
        fr = Frame(dark, xa, xb, top, bot)
        # The ticks' positions as fractions across the frame, along their edges.
        xt = [(fr.to_uv(p, fr.bottom(p))[0], n) for p, n in tick_stubs(dark, "x", bot, -1, xa, xb)]
        yt = [(fr.to_uv(fr.left(p), p)[1], n) for p, n in tick_stubs(dark, "y", xa, +1, top, bot)]
        shear = (fr.left(top) - fr.left(bot), fr.bottom(xb) - fr.bottom(xa))
        print(name, f"rotation {rot:.2f} deg, frame x {xa}-{xb}, y {top:.1f}-{bot:.1f}; the left "
              f"side leans {shear[0]:+.1f} px top to bottom, the bottom {shear[1]:+.1f} px")
        if args.probe:
            for (lo, hi, _), stubs, nm in ((f["x_axis"], xt, "x"), (f["y_axis"], yt, "y")):
                vals = [lo + w * (hi - lo) for w, n in stubs if n >= 8]
                print(f"  {nm} major stubs at", [f"{v:.4g}" for v in vals])
            continue
        cal_x, ix = calibrate(xt, *f["x_axis"], 0.0, 1.0, "x", f.get("x_major_min", 8.0))
        cal_y, iy = calibrate(yt, *f["y_axis"], 0.0, 1.0, "y")
        for nm, info in (("x", ix), ("y", iy)):
            if info["dropped"]:
                print(f"  {nm}: dropped stubs taken for the ticks at {info['dropped']}")
            print(f"  {nm}: {info['majors']} major ticks (worst {info['worst']:.1e} of span); "
                  f"{info['ticks']} ticks every {info['minor']:g}; ticks off the frame's scale "
                  f"{info['offset']:+.1e}, the scan's stretch {info['stretch']:.1e} of span")

        def to_xy(px, py, cx=cal_x, cy=cal_y):
            u, v = fr.to_uv(px, py)
            return float(cx[0](u)), float(cy[0](v))

        def to_pix(x, y, cx=cal_x, cy=cal_y):
            return fr.to_px(cx[1](x), cy[1](y))

        axes = (to_xy, to_pix)
        if args.grid:
            path = Path(args.overlays) / f"{name}_grid.png"
            grid_overlay(img, fr, axes, f["x_axis"], f["y_axis"], path)
            print(f"  grid overlay: {path}")
            if f["marker"] == "cross":
                found = [to_xy(*c) for c in find_crosses(dark, fr)]
                print(f"  {len(found)} crosses found: " + ", ".join(f"({x:.4g}, {y:.4g})"
                                                                for x, y in found))
            continue
        if f["marker"] == "glyph":
            readings = read_glyph_polylines(dark, axes, f)
            for curve, rows in readings.items():
                for x, y, how, dx, dy, *_ in rows:
                    print(f"  {curve}: ({x:.5g}, {y:.5g})  {how}, moved ({dx:+.1f}, {dy:+.1f}) px")
        elif f["marker"] == "dotted":
            readings = read_dotted_map(dark, fr, axes, f)
            for curve, rows in readings.items():
                print(f"  {curve:g} %: " + " ".join(f"({x:.4g}, {y:.4g}{'' if h != 'end' else '*'})"
                                                     for x, y, h, *_ in rows))
        else:
            readings = read_figure(dark, axes, f)
            for curve, rows in readings.items():
                for x, y, dx, dy, *_ in rows:
                    print(f"  {curve}: ({x:.5g}, {y:.5g})  snap moved ({dx:+.1f}, {dy:+.1f}) px")
        review_overlay(img, fr, readings, Path(args.overlays) / f"{name}.png")
        info = (f"Values on the frame's scale (its edges {f['x_axis'][:2]}, "
                f"{f['y_axis'][:2]}), corrected by the scan's stretch measured on "
                f"{ix['ticks']} and {iy['ticks']} ticks (up to "
                f"{max(ix['stretch'], iy['stretch']):.1e} of an axis); the plotted ticks sit "
                f"{ix['offset']:+.1e} and {iy['offset']:+.1e} of the axes off that scale.")
        write_csv(name, f, readings, info)


if __name__ == "__main__":
    main()
