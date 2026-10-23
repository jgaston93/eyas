"""Build the UH-60A aircraft config data/uh60a/uh60a.yaml from the GenHel transcriptions.

Reads the as-printed constants in data/uh60a/genhel/ (GenHel, NASA CR-166309) and writes the
whole-aircraft config the model loads (DECISIONS.md D-034), plus the validation variants at
the weight and CG of a published test (D-035 M7c amendment, D-046; VARIANTS below):

    python3 tools/build_uh60a_config.py            # write the configs
    python3 tools/build_uh60a_config.py --check    # fail if a config is stale

What it works out:
- Mass split (D-002, D-032 item 7). GenHel's WEIGHT and CG (p. 5.1-49) are the whole
  aircraft's. Eyas carries the blades (in the main rotor) and the gear's unsprung masses (in
  the gear) as their own masses, so the airframe is WEIGHT minus both, and its CG is found by
  mass balance with the blades at the hub and the wheels at their axles (free extension). The
  inertias are GenHel's motion-module values as written (D-002).
- Positions. Every GenHel station becomes a body-axis offset from the airframe CG:
  x = -(FS - FS_cg), y = -(BL - BL_cg) (buttlines positive to port), z = -(WL - WL_cg).
  The exception is BLMGA: GenHel's landing module uses it as body y (Y_RMGA = BLMGA/12,
  p. 5.7-9, so positive to starboard), not as a buttline.
- Elastic twist (D-038): GenHel's K_FP0 against airspeed and its mode shape against r/R
  (pp. 5.1-24, 5.1-49), written as dynamic_twist_gain.csv and dynamic_twist_mode.csv.
- Controls (§5.5, Figs. 5.1.6(a)-(d)): normalized input -> stick or pedal inches -> GenHel's
  mechanical mixing [deg] -> the swashplate phase rotation (p. 5.1-24), written as the
  offsets and gains of Eyas's linear control map. The travels in inches come from GenHel's
  cockpit section (Figs. 6.3.1 and 6.4.1, pp. 6.16-6.17; D-035). Collective, lateral and pedal
  fit the rigging exactly (checked here). The longitudinal stroke is given but its rows put
  neutral cyclic in three places, 0.81 in apart. Neutral is the pinned row's estimate, the end
  of that range nearest the fit to Ballin's GenHel longitudinal stick (D-035 M7c amendment;
  the fit itself, 2.04 in, lies outside the range).
"""

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data" / "uh60a" / "genhel"
DATA = ROOT / "data" / "uh60a"
OUT = DATA / "uh60a.yaml"

# Validation variants: GenHel's aircraft at another gross weight and CG. Only WEIGHT and the CG
# change; the mass split (D-034 item 2) and every position follow. The inertias stay GenHel's
# motion-module values (D-002): neither source gives the test aircraft's for its trims.
# Each entry: file -> (weight [lbm], FS, BL, WL [in], source line). None keeps GenHel's value.
VARIANTS = {
    "uh60a_ballin.yaml": (
        16000.0, 351.0, None, 231.5,
        "Ballin, NASA TM-88360 (1987), Fig. B1 (p. 28): gross weight 16000 lbm, CG station "
        "351.0 in, CG waterline 231.5 in (the level-flight and climb trims of App. B)"),
    "uh60a_fletcher.yaml": (
        14350.0, 360.0, None, None,
        "Fletcher, 21st ERF (1995) Paper VII-4, p. VII-4-6: takeoff gross weight approximately "
        "14,350 lbm, CG at station 360; no waterline given, so GenHel's WLCG is kept (DATA_GAPS.md)"),
}

# Ballin's transient responses (App. D and E; D-046): one variant per plotted test file, at
# its gross weight and CG from Table C1 (CG buttline 0, App. C), written to ballin_app_d/.
# File 210's CG waterline is printed clearly as 243.8 in, against 231.6-232.7 in for every
# other file (DATA_GAPS.md, doubtful). Its case is run as printed and, for comparison, at the
# waterline of file 212, the same flight (50) with the same CG station: a stand-in that
# shows how much the waterline matters, not a reading (uh60a_file210_wl232.5.yaml).
TABLE_C1 = DATA / "validation" / "ballin_table_c1.csv"
TABLE_C1_STAND_INS = {"210": ("212", "cg_waterline_in")}  # file: (file it borrows from, field)


def table_c1_variants():
    import csv
    out = {}
    with TABLE_C1.open(encoding="utf-8") as f:
        rows = {r["file"]: r for r in csv.DictReader(line for line in f if not line.startswith("#"))}

    def variant(r, wl, suffix="", note=""):
        out[f"ballin_app_d/uh60a_file{r['file']}{suffix}.yaml"] = (
            float(r["gross_weight_lbm"]), float(r["cg_station_in"]), 0.0, float(wl),
            f"Ballin, NASA TM-88360 (1987), Table C1 (p. 37), file {r['file']} (Fig. "
            f"{r['figure']}): gross weight {r['gross_weight_lbm']} lbm, CG station "
            f"{r['cg_station_in']} in, CG waterline {r['cg_waterline_in']} in; CG buttline 0 "
            f"(App. C){note}")

    for r in rows.values():
        if r["figure"] == "none":
            continue
        doubtful = r["file"] in TABLE_C1_STAND_INS
        variant(r, r["cg_waterline_in"],
                note=". CG waterline as printed, doubtful (DATA_GAPS.md)" if doubtful else "")
        if doubtful:
            other, field = TABLE_C1_STAND_INS[r["file"]]
            assert field == "cg_waterline_in"
            wl = rows[other][field]
            variant(r, wl, f"_wl{wl}",
                    f". STAND-IN: CG waterline {wl} in, file {other}'s (the same flight), in "
                    f"place of the printed {r[field]} in, which is doubtful (DATA_GAPS.md); "
                    "for comparison only, not a reading")
    return out


VARIANTS.update(table_c1_variants())

# Kaplita, NASA CR-177360 (1985), Figs. 4 (flight 66, run 27) and 2 (run 12): the headers give
# the weight and CG station but no CG waterline. STAND-IN, approved by the spec owner
# (2026-10-20): the waterline of Kaplita's 140 kt steady-heading sideslip flight (flight 19,
# Fig. 13 header, 231.8 in, itself read 231.8 or 231.9; DATA_GAPS.md), so the file names say
# standin.
VARIANTS["kaplita/uh60a_kaplita_fig4_standin.yaml"] = (
    15410.0, 352.1, 0.0, 231.8,
    "Kaplita, NASA CR-177360 (1985), Fig. 4 header (PDF p. 59): gross weight 15410 lbm, CG "
    "station 352.1 in. STAND-IN: CG waterline 231.8 in, Kaplita Fig. 13's (flight 19), in place "
    "of Fig. 4's, which isn't given (DATA_GAPS.md); for comparison only, not a reading")
VARIANTS["kaplita/uh60a_kaplita_fig2_standin.yaml"] = (
    16170.0, 350.4, 0.0, 231.8,
    "Kaplita, NASA CR-177360 (1985), Fig. 2 header (PDF p. 51): gross weight 16170 lbm, CG "
    "station 350.4 in. STAND-IN: CG waterline 231.8 in, Kaplita Fig. 13's (flight 19), in place "
    "of Fig. 2's, which isn't given (DATA_GAPS.md); for comparison only, not a reading")

SLUG_LBM = 32.17404855643044  # lbm per slug (g0 / ft)
LBF_LBM = 1.0                 # a GenHel weight in lb is the same number in lbm


def constants(name):
    """name -> value of a constants transcription (first two columns), comments stripped."""
    out = {}
    for line in (SRC / name).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        if parts[0] in out:
            raise ValueError(f"{name}: {parts[0]} given twice")
        out[parts[0]] = float(parts[1])
    return out


def constant_range(name, key):
    """(lo, hi) of a two-value row of a constants transcription, e.g. a limit."""
    for line in (SRC / name).read_text(encoding="utf-8").splitlines():
        parts = line.split("#", 1)[0].split()
        if parts and parts[0] == key:
            return float(parts[1]), float(parts[2])
    raise ValueError(f"{name}: no {key}")


def fmt(v):
    """Shortest round-trip text of a float, -0 written as 0."""
    if v == 0.0:
        return "0"
    return repr(float(v))


def q(v, unit, placeholder=False):
    tag = "!placeholder " if placeholder else ""
    return f'{tag}"{fmt(v)} {unit}"'


def vertical_slope(csv_name, pressure):
    """Slope of a tire's vertical load over the table's first segment at `pressure` [lbf/in],
    linear in pressure, as the gear's default top-stop rule uses (D-032 item 6)."""
    rows = []
    for line in (DATA / "gear" / csv_name).read_text(encoding="utf-8").splitlines():
        if line.startswith("#") or line.startswith("deflection"):
            continue
        d, p, f = (float(x) for x in line.split(","))
        rows.append((d, p, f))
    defl = sorted({r[0] for r in rows})
    pres = sorted({r[1] for r in rows})
    load = {(r[0], r[1]): r[2] for r in rows}

    def at(d):
        if pressure <= pres[0]:
            return load[(d, pres[0])]
        for lo, hi in zip(pres, pres[1:]):
            if lo <= pressure <= hi:
                w = (pressure - lo) / (hi - lo)
                return (1 - w) * load[(d, lo)] + w * load[(d, hi)]
        return load[(d, pres[-1])]

    return (at(defl[1]) - at(defl[0])) / (defl[1] - defl[0])


def mass_split(mr, gear):
    """Airframe mass [lbm] and CG station (FS, BL, WL) [in]."""
    w_total = mr["WEIGHT"] * LBF_LBM
    blades = mr["NB"] * mr["W_B"] * LBF_LBM
    wheels = [  # (mass [lbm], FS, BL, WL) at the axles, free extension; BL positive to port
        (gear["MMG"] * SLUG_LBM, gear["FSMGA"], gear["BLMGA"], gear["WLMGA"]),   # left main
        (gear["MMG"] * SLUG_LBM, gear["FSMGA"], -gear["BLMGA"], gear["WLMGA"]),  # right main
        (gear["MNG"] * SLUG_LBM, gear["FSNGA"], gear["BLNGA"], gear["WLNGA"]),
    ]
    parts = [(blades, mr["FS_MR"], mr["BL_MR"], mr["WL_MR"])] + wheels
    m_af = w_total - sum(p[0] for p in parts)
    cg = []
    for k, total in zip((1, 2, 3), (mr["FSCG"], mr["BLCG"], mr["WLCG"])):
        cg.append((w_total * total - sum(p[0] * p[k] for p in parts)) / m_af)
    return m_af, tuple(cg)


def gear_constants():
    """The numeric gear constants (name value unit ...), keeping only parseable numbers."""
    out = {}
    for line in (SRC / "gear_constants_5_7_5.txt").read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        try:
            out[parts[0]] = float(parts[1])
        except ValueError:
            pass
    return out


def travels(rig, tv):
    """Stick and pedal travels [in] from Figs. 6.3.1 and 6.4.1, checked against the rigging.

    Returns (xc, xa, xb, xp, lon_spread): each travel as (inches at -1 or 0, inches at +1),
    and the spread [in] of the three neutral-cyclic estimates behind xb.
    """
    def thosum(xc_in):
        return rig["KXCTH0"] * xc_in

    def close(a, b, what, tol=0.01):
        if abs(a - b) > tol:
            raise ValueError(f"control travels: {what}: {a:.4f} against {b:.4f} deg")

    # Collective: 0 in (full down) to the stroke; the figure's ends must be the rigging's.
    xc = (0.0, tv["STROKE_C"])
    close(thosum(xc[0]) + rig["BIAS_C"], tv["TH0_LOW"], "collective low")
    close(thosum(xc[1]) + rig["BIAS_C"], tv["TH0_HIGH"], "collective high")
    xc_low, xc_mid, xc_high = xc[0], 0.5 * (xc[0] + xc[1]), xc[1]

    # Pedal: the pinned (neutral) pedal gives the figure's 15.0 deg at mid collective; the
    # travel is the stroke with overtravel, which the figure's end rows use.
    xp_n = (tv["P_PIN_THR"] - thosum(xc_mid) - rig["BIAS_P"]) / rig["KXPTHR"]
    h = 0.5 * tv["STROKE_P_OVR"]
    close(h, tv["P_LEFT_IN"], "pedal half stroke (left)", 1e-9)
    close(h, tv["P_RIGHT_IN"], "pedal half stroke (right)", 1e-9)
    xp = (xp_n - h, xp_n + h)  # -1 = full left (more thrust), +1 = full right
    close(rig["KXPTHR"] * xp[0] + thosum(xc_mid) + rig["BIAS_P"], tv["P_LEFT_THR"], "pedal left")
    close(rig["KXPTHR"] * xp[1] + thosum(xc_mid) + rig["BIAS_P"], tv["P_RIGHT_THR"], "pedal right")

    # Lateral: the rows are at low collective; neutral from the right row, the others checked.
    def a1s(xa_in, xc_in):
        return rig["KXAA1S"] * xa_in + rig["MIX_AC"] * thosum(xc_in) + rig["BIAS_A"]
    xa_n = (tv["A_RIGHT_A1S"] - rig["MIX_AC"] * thosum(xc_low) - rig["BIAS_A"]) / rig["KXAA1S"] \
        - tv["A_RIGHT_IN"]
    xa = (xa_n - 0.5 * tv["STROKE_A"], xa_n + 0.5 * tv["STROKE_A"])
    close(a1s(xa_n - tv["A_LEFT_IN"], xc_low), tv["A_LEFT_A1S"], "lateral left")
    close(a1s(xa_n - tv["A_PIN_IN"], xc_low), tv["A_PIN_A1S"], "lateral pinned")

    # Longitudinal, with the pedal at neutral: each row gives its own neutral cyclic. They
    # disagree by up to 0.8 in (DATA_GAPS.md). Fitted to Ballin's GenHel longitudinal stick
    # (NASA TM-88360 Fig. B1(a)), neutral comes out at 2.04 in, beyond the largest estimate,
    # so neutral is that estimate, the pinned row's (D-035 M7c amendment).
    kb = rig["KXBB1S"] * rig["SIGN_XB2"]

    def b1s_rest(xc_in):  # B1S with the stick at XB+2 = 0
        return (rig["MIX_BC"] * thosum(xc_in) + rig["MIX_BP"] * rig["KXPTHR"] * xp_n
                + rig["BIAS_B"])
    est = [(tv["B_FWD_B1S"] - b1s_rest(xc_high)) / kb - tv["B_FWD_IN"],
           (tv["B_PIN_B1S"] - b1s_rest(xc_low)) / kb + tv["B_PIN_IN"],
           (tv["B_AFT_B1S"] - b1s_rest(xc_mid)) / kb + tv["B_AFT_IN"]]
    xb_n = est[1]
    if xb_n != max(est):
        raise ValueError("control travels: the pinned row no longer gives the largest neutral")
    xb = (xb_n - 0.5 * tv["STROKE_B"], xb_n + 0.5 * tv["STROKE_B"])  # -1 = full aft
    return xc, xa, xb, xp, max(est) - min(est)


def dynamic_twist_tables():
    """GenHel's elastic-twist gain against airspeed and mode shape against r/R (D-038), as
    the text of two CSV files."""
    dt = constants("dynamic_twist_5_1.txt")
    src = ("# source: GenHel, NASA CR-166309 (1981), pp. 5.1-24, 5.1-25 and 5.1-49; transcription "
           "data/uh60a/genhel/dynamic_twist_5_1.txt. Generated by tools/build_uh60a_config.py.\n")
    # K_FP0 = BASE + SLOPE (V - 100) between its limits: BASE up to 100 kt, MIN from the speed
    # where the line meets it; linear in between, constant outside (the table holds its ends).
    v_min = 100.0 + (dt["KFP0_MIN"] - dt["KFP0_BASE"]) / dt["KFP0_SLOPE"]
    gain = src + "airspeed[kt],gain[deg/lbf]\n"
    for v, k in ((0.0, dt["KFP0_BASE"]), (100.0, dt["KFP0_BASE"]), (v_min, dt["KFP0_MIN"])):
        gain += f"{fmt(round(v, 9))},{fmt(k)}\n"
    # MODESP = A + B sin(90 deg r/R), tabulated every 0.025 in r/R (linear interpolation of
    # the sine then errs by at most 1.4e-4 of the tip twist).
    mode = src + "r[1],mode[1]\n"
    for i in range(41):
        r = i / 40.0
        mode += f"{fmt(r)},{fmt(round(dt['MODE_A'] + dt['MODE_B'] * math.sin(math.radians(90.0 * r)), 12))}\n"
    return gain, mode


def table_rows(name):
    """Numeric rows of a table transcription, comments stripped."""
    rows = []
    for line in (SRC / name).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            rows.append([float(v) for v in line.split()])
    return rows


ENGINE = DATA / "engine"


def engine_tables():
    """GenHel's T700 maps (CR-166309 Tables 6.5.1 to 6.5.6, pp. 5.6-25 to 5.6-28; D-053) as
    CSV files for the genhel_t700 component, as printed: path -> text."""
    def src(table, page, note=""):
        return (f"# source: GenHel, NASA CR-166309 (1981), {table}, {page}; transcription "
                f"data/uh60a/genhel/. Generated by tools/build_uh60a_config.py.{note}\n")
    out = {}
    rows = table_rows("table_6_5_1_ngg_trim.txt")
    out["t700_torque.csv"] = src(
        "Table 6.5.1 (steady state engine performance)", "p. 5.6-25 (PDF p. 221)",
        " Torque required per engine referred to the main rotor, (Q_REQ)_TRIM/N_E.") + \
        "q[ft*lbf],n_gg[%]\n" + "".join(f"{fmt(r[0])},{fmt(r[1])}\n" for r in rows)
    rows = table_rows("table_6_5_2_lds_rigging.txt")
    out["t700_lds_rigging.csv"] = src(
        "Table 6.5.2 (collective/static droop compensation rigging)", "p. 5.6-25 (PDF p. 221)") + \
        "collective[%],lds[deg]\n" + "".join(f"{fmt(r[0])},{fmt(r[1])}\n" for r in rows)
    rows = table_rows("table_6_5_3_lds_cam.txt")
    out["t700_lds_cam.csv"] = src(
        "Table 6.5.3 (load demand spindle cam output)", "p. 5.6-26 (PDF p. 222)") + \
        "lds[deg],n_gg[%]\n" + "".join(f"{fmt(r[0])},{fmt(r[1])}\n" for r in rows)
    rows = table_rows("table_6_5_4_fuel_flow.txt")
    out["t700_fuel.csv"] = src(
        "Table 6.5.4 (steady fuel flow required)", "p. 5.6-26 (PDF p. 222)", " Per engine.") + \
        "n_gg[%],wf[lbm/hr]\n" + "".join(f"{fmt(r[0])},{fmt(r[1])}\n" for r in rows)
    rows = table_rows("table_6_5_5_p3.txt")
    out["t700_p3.csv"] = src(
        "Table 6.5.5 (discharge pressure for steady conditions)", "p. 5.6-27 (PDF p. 223)") + \
        "n_gg[%],p3[psi]\n" + "".join(f"{fmt(r[0])},{fmt(r[1])}\n" for r in rows)
    rows = table_rows("table_6_5_6_partials.txt")
    out["t700_partials.csv"] = src(
        "Table 6.5.6 (time varying coefficients)", "pp. 5.6-27 and 5.6-28 (PDF pp. 223-224)",
        " Torques per engine: QGG at the gas generator, QPT at the power turbine.") + \
        ("n_gg[%],dp3_dwf[psi*hr/lbm],dp3_dngg[psi/%],dwf_dp3[lbm/(hr*psi)],"
         "dqgg_dwf[ft*lbf*hr/lbm],dqgg_dngg[ft*lbf/%],dqpt_dwf[ft*lbf*hr/lbm],"
         "dqpt_dnpt[ft*lbf/rpm],dqpt_dngg[ft*lbf/%]\n") + \
        "".join(",".join(fmt(v) for v in r) + "\n" for r in rows)
    return {ENGINE / name: text for name, text in out.items()}


BALLIN = DATA / "engine_ballin"
# The aircraft on Ballin's engines (D-054), for validation: GenHel's weight, and the weights
# of Ballin's (TM-88360) and Fletcher's flights.
BALLIN_AIRCRAFT = {"uh60a.yaml": None, "uh60a_ballin.yaml": "uh60a_ballin.yaml",
                   "uh60a_fletcher.yaml": "uh60a_fletcher.yaml",
                   "uh60a_table_b1.yaml": "uh60a_table_b1.yaml"}
# Weights and CGs flown only on Ballin's engines.
BALLIN_VARIANTS = {
    "uh60a_table_b1.yaml": (
        16825.0, 355.0, 0.0, 248.2,
        "Ballin, NASA TM-100991 (1988), Table B.1 (p. 53): aircraft weight 16825 lbm, CG "
        "station 355.0 in, waterline 248.2 in, buttline 0.0 in (the engine's trim conditions)"),
}
BALLIN_SRC = DATA / "ballin_t700"


def digitized(fig):
    """The readings of a figure digitized by tools/digitize_ballin_t700.py: {curve: [(x, y)]}."""
    curves = {}
    lines = (BALLIN_SRC / f"{fig}.csv").read_text(encoding="utf-8").splitlines()
    rows = [ln for ln in lines if ln and not ln.startswith("#")][1:]
    for row in csv.reader(rows):
        curves.setdefault(row[0], []).append((float(row[1]), float(row[2])))
    return curves


def ballin_engine_tables():
    """Ballin's T700 maps and fuel-control schedules (NASA TM-100991, Figs. A1-A11, C23-C29;
    D-054) as CSV files for the ballin_t700 component: the digitized readings as read, in the
    units printed on the figures. path -> text."""
    def src(fig, title, page, note=""):
        return (f"# source: Ballin, NASA TM-100991 (1988), Fig. {fig}, {title}, {page}; "
                f"digitized readings data/uh60a/ballin_t700/{fig}.csv (status: UNREVIEWED, "
                f"DATA_GAPS.md). Generated by tools/build_uh60a_config.py.{note}\n")

    def curve(fig):
        (pts,) = digitized(fig).values()
        return pts

    def one(fig, header, title, page, note=""):
        return src(fig, title, page, note) + header + "\n" + \
            "".join(f"{fmt(x)},{fmt(y)}\n" for x, y in curve(fig))

    def family(fig, header, title, page, note=""):
        rows = [(float(c), x, y) for c, pts in digitized(fig).items() for x, y in pts]
        return src(fig, title, page, note) + header + "\n" + \
            "".join(f"{fmt(c)},{fmt(x)},{fmt(y)}\n" for c, x, y in rows)

    out = {
        "f1_compressor_flow.csv": family(
            "A1", "ngc[%],pr[1],wa2c[lbm/s]", "f1, compressor mass flow", "p. 42 (PDF p. 56)",
            " One curve per corrected gas-generator speed NGC (the legend's symbols 1-11); "
            "pressure ratio PS3/P2."),
        "f2_compressor_temperature.csv": one(
            "A2", "pr[1],t3_t2[1]", "f2, compressor temperature", "p. 43 (PDF p. 57)"),
        "f3_seal_bleed.csv": one(
            "A3", "ngc[%],b1[1]", "f3, seal-pressurization bleed fraction", "p. 44 (PDF p. 58)"),
        "f4_interstage_bleed.csv": one(
            "A4", "wa2c[lbm/s],b2[1]", "f4, interstage bleed fraction", "p. 45 (PDF p. 59)"),
        "f5_cooling_bleed.csv": one(
            "A5", "wa2c[lbm/s],b3[1]", "f5, cooling bleed fraction", "p. 46 (PDF p. 60)"),
        "f6_combustor_efficiency.csv": one(
            "A6", "far[1],eta[1]", "f6, combustor efficiency", "p. 47 (PDF p. 61)"),
        "f7_gas_generator_turbine.csv": one(
            "A7", "pr[1],dh_theta[Btu/lbm]", "f7, gas-generator turbine energy",
            "p. 48 (PDF p. 62)", " Pressure ratio P45/P41; enthalpy drop over theta41."),
        "f8_power_turbine.csv": one(
            "A8", "pr[1],dh_theta[Btu/lbm]", "f8, power turbine energy", "p. 49 (PDF p. 63)",
            " Pressure ratio P49/P45; enthalpy drop over theta45."),
        "f9_power_turbine_flow.csv": one(
            "A9", "pr[1],w45c[lbm*in^2/(lbf*s)]", "f9, power turbine mass flow",
            "p. 50 (PDF p. 64)", " Pressure ratio PS9/P45. Printed LBM/SEC; eq. (34),"
            " W45 = W45C P45/sqrt(theta45) with P45 in psia, makes it lbm/(s psia)."),
        "f10_exhaust_pressure.csv": one(
            "A10", "ngc[%],p49_ps9[1]", "f10, exhaust pressure loss", "p. 51 (PDF p. 65)",
            " Printed PS9/P49; eq. (38), P49 = PS9 f10, and its values (> 1) make it P49/PS9."),
        "f_hs_heat_sink.csv": one(
            "A11", "ngc[%],t41sgn[lbm]", "f_hs, station 4.1 heat-sink constant",
            "p. 52 (PDF p. 66)",
            " Printed without a unit; eq. (53), Mc_pm/(W_g c_pg) [s] = T41SGN/W41, makes it lbm."),
        "fec1_harness_time_constant.csv": family(
            "C23", "t45l[degR],w45r[lbm*degR^0.5/(s*psi)],tau45[s]",
            "F_EC1, ECU thermocouple sensor time constant", "p. 80 (PDF p. 94)",
            " One curve per T45L (the legend's symbols 1-4); W45R = W45 sqrt(T45L)/P45 (Fig. C5)"
            " with W45 in lbm/s (D-054 item 6(c))."),
        "fhm1_topping_line.csv": one(
            "C24", "t2[degR],wfptp[lbm*in^2/(lbf*hr)]", "F_HM1, HMU topping line schedule",
            "p. 81 (PDF p. 95)"),
        "fhm2_power_available.csv": one(
            "C25", "pas[deg],wfprf[lbm*in^2/(lbf*hr)]",
            "F_HM2, HMU power-available input schedule", "p. 82 (PDF p. 96)"),
        "fhm3_load_demand_speed.csv": one(
            "C26", "xldsa[deg],png[%]", "F_HM3, HMU load-demand-compensation schedule 1",
            "p. 83 (PDF p. 97)"),
        "fhm4_load_demand_fuel.csv": one(
            "C27", "xldsa[deg],wfqps3[lbm*in^2/(lbf*hr)]",
            "F_HM4, HMU load-demand-compensation schedule 2", "p. 84 (PDF p. 98)"),
        "fhm5_idle_fuel.csv": one(
            "C28", "t2[degR],wfirf[lbm*in^2/(lbf*hr)]", "F_HM5, HMU idle-schedule function 1",
            "p. 85 (PDF p. 99)"),
        "fhm6_idle_speed.csv": one(
            "C29", "t2[degR],pcngi[%]", "F_HM6, HMU idle-schedule function 2",
            "p. 86 (PDF p. 100)"),
    }
    return {BALLIN / name: text for name, text in out.items()}


def transcription(name):
    """{constant: (value, unit)} of a constants transcription in data/uh60a/ballin_t700/."""
    out = {}
    for line in (BALLIN_SRC / name).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            parts = line.split()
            out[parts[0]] = (float(parts[1]), parts[2] if len(parts) > 2 else "-")
    return out


def ballin_engine_block():
    """The ballin_t700 component's config (D-054): Ballin, NASA TM-100991 Tables A.1 and C.1
    and the figures' constants, TM-88360 App. A's drive train; as a standalone YAML file."""
    a = transcription("table_a1_engine_constants.txt")
    c = transcription("table_c1_fuel_control_constants.txt")
    d = transcription("drive_train_constants.txt")

    def v(t, key, unit=None):
        value, printed = t[key]
        return f'"{fmt(value)} {unit or printed}"'

    def n(t, key):
        return fmt(t[key][0])

    return f"""# source: Ballin, NASA TM-100991 (1988): Table A.1 p. 41 (PDF p. 55), Table C.1
# pp. 69-70 (PDF pp. 83-84), Figs. C2-C22 pp. 71-79 (PDF pp. 85-93); Ballin, NASA TM-88360
# (1987) App. A p. 23 (PDF p. 25); transcriptions in data/uh60a/ballin_t700/. D-054.
# Generated by tools/build_uh60a_config.py; edit the transcriptions or the script, not this file.
# The power-available spindle at 120 deg and the NP reference from the shaft: D-054 item 2.
type: ballin_t700
name: engines
shaft: {{name: rotor_drive}}
rotor: main_rotor  # its shaft torque seeds trim
engines: {int(d['ENCOMP'][0])}
gear: {n(d, 'n_E')}  # TM-88360 p. 23
gas_path:  # Table A.1
  hvf: {v(a, 'HVF')}
  j_gt: {v(a, 'J_GT')}
  k_bl: {n(a, 'K_bl')}
  k_b3: {n(a, 'K_b3')}
  k_damp: {v(a, 'K_damp')}
  k_dpb: {v(a, 'K_dpb')}
  k_h2: {v(a, 'K_H2')}
  k_h3: {{slope: {v(a, 'K_H3_1')}, offset: {v(a, 'K_H3_2')}}}
  k_h41: {{slope: {v(a, 'K_H41_1')}, offset: {v(a, 'K_H41_2')}}}
  k_h45: {n(a, 'K_H45')}
  k_ps3: {n(a, 'K_Ps3')}
  k_qc_1: {n(a, 'K_QC_1')}
  k_qc_2: {n(a, 'K_QC_2')}
  k_t41: {{slope: {v(a, 'K_T41_1')}, offset: {v(a, 'K_T41_2')}}}
  k_t45: {{slope: {v(a, 'K_T45_1')}, offset: {v(a, 'K_T45_2')}}}
  k_th41: {{slope: {v(a, 'K_TH41_1')}, offset: "{n(a, 'K_TH41_2')} 1"}}
  k_th45: {{slope: {v(a, 'K_TH45_1')}, offset: "{n(a, 'K_TH45_2')} 1"}}
  k_wgt: {v(a, 'K_WGT')}
  ng_des: {v(a, 'NG_des')}
  np_des: {v(a, 'NP_des')}
  tc_t41: {v(a, 'TC_T41')}
output: {{inertia: {v(d, 'JE')}, axis: {{x: 1, y: 0, z: 0}}}}  # JE, TM-88360 p. 23; GenHel's power-turbine axis (D-053)
gas_generator_spool: {{axis: {{x: 1, y: 0, z: 0}}}}
maps:  # Figs. A1-A11, C23-C29, digitized (D-054 item 9)
  f1: f1_compressor_flow.csv
  f2: f2_compressor_temperature.csv
  f3: f3_seal_bleed.csv
  f4: f4_interstage_bleed.csv
  f5: f5_cooling_bleed.csv
  f6: f6_combustor_efficiency.csv
  f7: f7_gas_generator_turbine.csv
  f8: f8_power_turbine.csv
  f9: f9_power_turbine_flow.csv
  f10: f10_exhaust_pressure.csv
  f_hs: f_hs_heat_sink.csv
  fec1: fec1_harness_time_constant.csv
  fhm1: fhm1_topping_line.csv
  fhm2: fhm2_power_available.csv
  fhm3: fhm3_load_demand_speed.csv
  fhm4: fhm4_load_demand_fuel.csv
  fhm5: fhm5_idle_fuel.csv
  fhm6: fhm6_idle_speed.csv
ecu:  # Figs. C1-C8; speed errors in % NP, so the gains read V/%
  np_sensor_lag: {v(c, 'CTPL', 's')}
  dead_band: {v(c, 'CB', '%')}  # CB
  rate_lags: {{ct13: {v(c, 'CT13', 's')}, ct14: {v(c, 'CT14', 's')}}}
  # ZK9, ZK10 and ZK5 turn the error into volts (ZK5 through the washout (v1 - v2)/CT14,
  # so V*s/%); ZK7 and ZK1 scale the sum, 2.0 in all with B4 = 1 (H&W TM-83579 B64, B65).
  gains: {{zk1: {n(c, 'ZK1')}, zk5: {v(c, 'ZK5', 'V*s/%')}, zk7: {n(c, 'ZK7')}, zk9: {v(c, 'ZK9', 'V/%')}, zk10: {v(c, 'ZK10', 'V/%')}}}
  relay: {{low: "-1 %", high: "4 %", gain: "1000 ft*lbf*s", b6: {n(c, 'B6')}}}  # Fig. C3; breakpoints human-reviewed (DATA_GAPS.md)
  torque_switch: {{threshold: {v(c, 'CR', 'ft*lbf')}, lower: {v(c, 'YLOLIM', 'ft*lbf*s')}, upper: {v(c, 'YHILIM', 'ft*lbf*s')}, switch_point: {v(c, 'CORR', 'ft*lbf*s')}}}
  torque_sensor_lags: {{tl1: {v(c, 'TL1', 's')}, tl2: {v(c, 'TL2', 's')}}}
  governor: {{t11: {v(c, 'T11', 's')}, ct2: {v(c, 'CT2', 's')}, ct12: {v(c, 'CT12', 's')}}}
  harness: {{t45cor: {v(c, 'T45COR', 'degR')}, tlge: {v(c, 'TLGE', 's')}}}
  temperature: {{t45ref: {v(c, 'T45REF', 'degR')}, zk3: {v(c, 'ZK3', 'V/degR')}, ct9: {v(c, 'CT9', 's')}, t8: {v(c, 'T8', 's')}, t10: {v(c, 'T10', 's')}}}
  integrator:
    xkprop: {n(c, 'XKPROP')}
    xkintg: {v(c, 'XKINTG', '1/s')}
    zhilim: {v(c, 'ZHILIM', 'V')}
    zlolim: {{below: {v(c, 'ZLOLIM', 'V')}, above: "-0.3 V", torque: "180 ft*lbf"}}  # Fig. C8
    ct16: {v(c, 'CT16', 's')}
hmu:  # Figs. C9-C22
  torque_motor:  # Fig. C10; the stops are positions (D-054 item 6(b))
    bias: "-0.44 V"
    lead: "0.04 s"
    lag: "0.2 s"
    gain: "564.0 mA/V"
    offset: "-31.0 mA"
    dead_band: {v(c, 'TMDB', 'mA')}
    sensitivity: {v(c, 'TMGN', 'in/(mA*s)')}
    stops: {{lower: {v(c, 'XLOLIM', 'in')}, upper: {v(c, 'XHILIM', 'in')}}}
    feedback: {v(c, 'TMLVG', 'V/in')}
    linkage: {v(c, 'TMLG', 'lbm*in^2/(lbf*hr*in)')}
  ps3_sensor: {{lag: {v(c, 'CTPS3', 's')}, hysteresis: {v(c, 'PS3HYS', 'psi')}}}
  load_demand: {{rigging_gain: "0.914 deg/%", rigging_bias: "5.34 deg", hysteresis: {v(c, 'XLDHYS', 'deg')}, fuel_bias: "3.26 lbm*in^2/(lbf*hr)", lag: {v(c, 'CLLDS', 's')}}}  # Figs. C12, C13, C17
  droop: {{slope: {v(c, 'KNDRP', 'lbm*in^2/(lbf*hr*%)')}, reference: {v(c, 'NGREF', '%')}}}
  ng_sensor: {{hysteresis: {v(c, 'CH', '%')}, lag: {v(c, 'CNTL', 's')}}}
  power_available: "120.0 deg"  # D-054 item 2
  deceleration: {{gain: {v(c, 'AWFP', 'lbm*in^2/(lbf*hr*%)')}, bias: {v(c, 'BWFP', 'lbm*in^2/(lbf*hr)')}, lower: {v(c, 'WFPDCL', 'lbm*in^2/(lbf*hr)')}, upper: {v(c, 'WFPDCH', 'lbm*in^2/(lbf*hr)')}}}
  valve: {{min: {v(c, 'WFMIN', 'lbm/hr')}, max: {v(c, 'WFMAX', 'lbm/hr')}, lag: {v(c, 'CLMV', 's')}, delay: "0.015 s"}}  # Fig. C19
  hysteresis_lag: "0.005 s"  # D-054 item 7
drive_train: {{gearbox_damping: {v(d, 'DAMPBX', 'ft*lbf*s/rad')}, accessory_power: {v(d, 'ACCHP', 'hp')}}}  # TM-88360 pp. 17-23
"""


def control_map(rig, mr, tv):
    """Normalized controls -> GenHel inches -> blade pitch, as linear-map rows [deg].

    Returns (travel, main, tail, lon_spread): travel maps each input to (inches at -1 or 0, inches at +1);
    main and tail are {row: {input: gain [deg per unit], 'offset': deg}}.
    """
    sx = rig["SIGN_XB2"]
    kb = rig["KXBB1S"] * sx  # B1S per inch of forward stick (XB+2)
    xc, xa, xb, xp, lon_spread = travels(rig, tv)
    travel = {"collective": xc, "lat_cyclic": xa, "lon_cyclic": xb, "pedal": xp}

    def affine(lo, hi, zero_to_one):
        """inches = a + b*u for u in [0, 1] (collective) or [-1, 1]."""
        if zero_to_one:
            return lo, hi - lo
        return 0.5 * (lo + hi), 0.5 * (hi - lo)

    a_c, b_c = affine(*xc, True)
    a_a, b_a = affine(*xa, False)
    a_b, b_b = affine(*xb, False)
    a_p, b_p = affine(*xp, False)
    # Each GenHel quantity as {input: deg per unit, 'offset': deg}.
    thosum = {"offset": rig["KXCTH0"] * a_c, "collective": rig["KXCTH0"] * b_c}
    thrsum = {"offset": rig["KXPTHR"] * a_p, "pedal": rig["KXPTHR"] * b_p}

    def add(*terms):
        out = {}
        for scale, d in terms:
            for k, v in d.items():
                out[k] = out.get(k, 0.0) + scale * v
        return out

    theta0 = add((1, thosum), (1, {"offset": rig["BIAS_C"]}))
    a1s = add((1, {"offset": rig["KXAA1S"] * a_a, "lat_cyclic": rig["KXAA1S"] * b_a}),
              (rig["MIX_AC"], thosum), (1, {"offset": rig["BIAS_A"]}))
    b1s = add((1, {"offset": kb * a_b, "lon_cyclic": kb * b_b}),
              (rig["MIX_BC"], thosum), (rig["MIX_BP"], thrsum), (1, {"offset": rig["BIAS_B"]}))
    thettr = add((1, thrsum), (1, thosum), (1, {"offset": rig["BIAS_P"]}))
    # theta = theta0 - A1S cos(psi + D) - B1S sin(psi + D)   (p. 5.1-24)
    #       = theta0 + theta1c cos psi + theta1s sin psi with
    # theta1c = -A1S cos D - B1S sin D,  theta1s = A1S sin D - B1S cos D.
    d = math.radians(mr["DSP"])
    theta1c = add((-math.cos(d), a1s), (-math.sin(d), b1s))
    theta1s = add((math.sin(d), a1s), (-math.cos(d), b1s))
    main = {"theta0": theta0, "theta1s": theta1s, "theta1c": theta1c}
    tail = {"theta0": thettr}
    return travel, main, tail, lon_spread


def fcs_block(rig, mr, travel, sens, stab, pos, head_limits):
    """The genhel_fcs component (D-045): GenHel's mixer in inches, the primary servos, the
    Fig. 5.1.2 sensors and the dynamic stabilator law (Fig. 5.1.7). The rotor head limits
    are GenHel's "for piloted simulation" (p. 5.5-20): the flying aircraft has them, the
    validation variants (GenHel analysis runs) don't."""
    def lims(lo, hi):
        return f'{{min: "{fmt(rig[lo])} deg", max: "{fmt(rig[hi])} deg"}}'

    def tr(k):
        lo, hi = travel[k]
        return f'{{lo: "{fmt(round(lo, 9))} in", hi: "{fmt(round(hi, 9))} in"}}'

    # Fig. 5.1.7's KVYDT * limit(VF - 30, 0, LMKDT) as a gain ramping from 0 at 30 kt to its
    # full value at 30 + LMKDT kt. KVYDT's unit isn't printed; with Y.SENS in ft/s^2 (Fig.
    # 5.1.5(c)) and VF in kt it is deg per (ft/s^2 kt) (assumption, DATA_GAPS.md).
    head = ""
    if head_limits:
        head = (f'      head_limits: {{theta0: {lims("TH0_LL", "TH0_UL")}, '
                f'a1s: {lims("A1S_LL", "A1S_UL")}, b1s: {lims("B1S_LL", "B1S_UL")}}}  '
                f'# p. 5.5-20; THR not applied (DATA_GAPS.md)\n')
    lo_kt = -stab["BIAS_VX1"]
    kdt_lo, kdt_hi = constant_range("fig_5_5_1_7_stabilator.txt", "LMKDT")
    if kdt_lo != 0.0:
        raise ValueError("LMKDT: the ramp form assumes a lower limit of 0")
    ay_full = stab["KVYDT"] * kdt_hi
    stab_lo, stab_hi = constant_range("fig_5_5_1_7_stabilator.txt", "LMSTAB")
    return f"""\
  - type: genhel_fcs
    name: fcs
    rigging:  # Figs. 5.1.6(a)-(d), p. 5.5-20; control_rigging_5_5.txt
      travel: {{collective: {tr("collective")}, lat_cyclic: {tr("lat_cyclic")}, lon_cyclic: {tr("lon_cyclic")}, pedal: {tr("pedal")}}}
      gains: {{collective: "{fmt(rig["KXCTH0"])} deg/in", lateral: "{fmt(rig["KXAA1S"])} deg/in", longitudinal: "{fmt(rig["KXBB1S"])} deg/in", pedal: "{fmt(rig["KXPTHR"])} deg/in"}}
      longitudinal_stick_sign: {fmt(rig["SIGN_XB2"])}
      mixing: {{lateral_collective: {fmt(rig["MIX_AC"])}, longitudinal_collective: {fmt(rig["MIX_BC"])}, longitudinal_pedal: {fmt(rig["MIX_BP"])}}}
      bias: {{collective: "{fmt(rig["BIAS_C"])} deg", lateral: "{fmt(rig["BIAS_A"])} deg", longitudinal: "{fmt(rig["BIAS_B"])} deg", pedal: "{fmt(rig["BIAS_P"])} deg"}}
      swashplate_phase: "{fmt(mr["DSP"])} deg"
{head}    primary_servo: {{a2: "{fmt(rig["PSERVO_A2"])} s^2", a1: "{fmt(rig["PSERVO_A1"])} s"}}
    rate_gyro: {{a2: "{fmt(sens["RG_A2"])} s^2", a1: "{fmt(sens["RG_A1"])} s"}}  # Fig. 5.1.2
    lateral_accelerometer:
      a2: "{fmt(sens["LA_A2"])} s^2"
      a1: "{fmt(sens["LA_A1"])} s"
      position: {pos(sens["FSPS"], sens["BLPS"], sens["WLPS"])}  # p. 5.10-19
    stabilator:  # Fig. 5.1.7; fig_5_5_1_7_stabilator.txt
      schedule: {{table: airframe/stabilator_schedule.csv, min: "{fmt(stab_lo)} deg", max: "{fmt(stab_hi)} deg"}}
      airspeed_lag: "{fmt(stab["TAU"])} s"
      pitch_rate_gain: "{fmt(stab["KVTDT"])} s"  # deg per deg/s
      lateral_accel_gain: {{gain: "{fmt(round(ay_full, 12))} deg/(ft/s^2)", from: "{fmt(lo_kt)} kt", to: "{fmt(lo_kt + kdt_hi)} kt"}}
      servo: {{a2: "{fmt(stab["SERVO_A2"])} s^2", a1: "{fmt(stab["SERVO_A1"])} s", rate_limit: "{fmt(stab["RATE_LIMIT"])} deg/s"}}
{afcs_block(rig)}"""


def afcs_block(rig):
    """The SAS (Figs. 5.1.3(a)-(c)) and pitch bias actuator (Fig. 5.1.4) of the genhel_fcs
    component, M8c2 (D-045), in GenHel's units: percent of authority, deg, deg/s, ft/s^2, kt.
    Each limit is printed as +-L; the PBA's as (lo, hi). The SAS output reaches the blade pitch
    through KPCDG* [deg/%] (p. 5.5-20); every config switches them on per flight (Controls)."""
    s = constants("fig_5_1_3_sas.txt")
    b = constants("fig_5_1_4_pba.txt")

    def lag(gain, tau, unit=None):
        g = fmt(s[gain]) if unit is None else f'"{fmt(s[gain])} {unit}"'
        return f'{{gain: {g}, lag: "{fmt(s[tau])} s"}}'

    def pm(key, table="fig_5_1_4_pba.txt"):
        lo, hi = constant_range(table, key)
        if lo != -hi:
            raise ValueError(f"{key}: expected a symmetric limit, got {lo}, {hi}")
        return hi

    r = "%/(deg/s)"
    vx_lo, vx_hi = constant_range("fig_5_1_4_pba.txt", "LMVXBA")
    if vx_lo != 0.0:
        raise ValueError("LMVXBA: the airspeed term assumes a lower limit of 0")
    return f"""\
    sas:  # Figs. 5.1.3(a)-(c), pp. 5.5-2, 5.5-7 to 5.5-9; fig_5_1_3_sas.txt (M8c2)
      hold: "{fmt(s["ZOH_SAS"])} s"  # digital zero-order hold, "airborne computer update delay"
      switch_speed: "{fmt(s["V_SW"])} kt"
      fade_time: "{fmt(s["T_FADE"])} s"
      actuator: {{a2: "{fmt(rig["SASACT_A2"])} s^2", a1: "{fmt(rig["SASACT_A1"])} s"}}
      descaling: {{pitch: "{fmt(rig["KPCDGP"])} deg/%", roll: "{fmt(rig["KPCDGR"])} deg/%", yaw: "{fmt(rig["KPCDGY"])} deg/%"}}  # p. 5.5-20
      pitch:
        digital: {{ad_lag: "{fmt(s["P_AD"])} s", rate: {lag("KRQQPD", "TRQQPD", r)}, lagged_rate: {lag("KLQQPD", "TLQQPD", r)}, washout: "{fmt(s["TWQQPD"])} s", limit: "{fmt(s["LMPD"])} %"}}
        analog: {{filter: {lag("KQQPA", "TQQPA")}, washout: "{fmt(s["TWQQPA"])} s", rate: {lag("KRQQPA", "TRQQPA", r)}, lagged_rate: {lag("KLQQPA", "TLQQPA", r)}, scale: {fmt(s["KPA_HALF"])}, limit: "{fmt(s["LMPA"])} %"}}
      roll:
        digital: {{ad_lag: "{fmt(s["R_AD"])} s", rate_gain: "{fmt(s["KRPPRD"])} {r}", lagged_rate: {lag("KLPPRD", "TLPPRD", r)}, attitude: {{gain: "{fmt(s["KPHRD"])} %/deg", limit: "{fmt(s["LMPHRD"])} deg"}}, limit: "{fmt(s["LMRD"])} %"}}
        analog:
          attitude_filter: {{gain: {fmt(s["KPHF1"])}, lag1: "{fmt(s["T1PHF1"])} s", lag2: "{fmt(s["T2PHF1"])} s"}}
          shaping: {{gain: {fmt(s["KPHF2"])}, a2: "{fmt(s["A2PHF2"])} s^2", a1: "{fmt(s["A1PHF2"])} s", limit: "{fmt(s["LMPHR1"])} deg"}}
          derivative: {{gain: {fmt(s["KPHF3"])}, lag1: "{fmt(s["T1PHF3"])} s", lag2: "{fmt(s["T2PHF3"])} s"}}
          rate: {lag("KRPHRA", "TRPHRA", r)}
          lagged_rate: {lag("KLPHRA", "TLPHRA", r)}
          attitude: {{gain: {fmt(s["KPHR1"])}, limit: "{fmt(s["LMPHRA"])} deg", output_gain: "{fmt(s["KPHIRA"])} %/deg"}}
          limit: "{fmt(s["LMRA"])} %"
      yaw:
        digital: {{ad_lag: "{fmt(s["Y_AD"])} s", rate: {lag("KRRRYD", "TRRRYD", r)}, lagged_rate: {lag("KLRRYD", "TLRRYD", r)}, washout: "{fmt(s["TWRRYD"])} s", lateral_accel: {lag("KLAYYD", "TLAYYD", "%/(ft/s^2)")}, roll_rate_gain: "{fmt(s["KRPPYD"])} {r}", limit: "{fmt(s["LMYD"])} %"}}
        analog:
          filter: {lag("KRRFI", "TRRFI")}
          washout: "{fmt(s["TWRRYA"])} s"
          rate: {lag("KRRRYA", "TRRRYA", r)}
          lateral_accel_gain: "{fmt(s["KLAYF1"])} (deg/s)/(ft/s^2)"  # KLAYF1; its {fmt(s["TLAYF1"])} s lag is unity (p. 5.5-2)
          lagged: {lag("KLAYRR", "TLAYRR", r)}
          scale: {fmt(s["KYA_HALF"])}
          roll_rate_gain: {fmt(s["KRPPYA"])}  # on the roll analog rate path's output ("B"; DATA_GAPS.md)
          limit: "{fmt(s["LMYA"])} %"
    pitch_bias:  # Fig. 5.1.4, pp. 5.5-2, 5.5-3, 5.5-10; fig_5_1_4_pba.txt (M8c2)
      attitude: {{gain: "{fmt(b["KTHBA"])} %/deg", limit: "{fmt(pm("LMTHBA"))} deg"}}
      pitch_rate: {{gain: "{fmt(b["KQQBA"])} {r}", limit: "{fmt(pm("LMQQBA"))} deg/s"}}  # on the pitch SAS's A/D output
      airspeed: {{gain: "{fmt(b["KVXBA"])} %/kt", from: "{fmt(-b["BIAS_VXBA"])} kt", range: "{fmt(vx_hi)} kt"}}
      bias: "{fmt(b["BIAS_BA"])} %"
      hold: "{fmt(b["ZOH_BA"])} s"
      actuator: {{gain: "{fmt(b["GAIN_BA"])} 1/s", rate_limit: "{fmt(pm("RATE_BA"))} %/s", authority: "{fmt(pm("AUTH_BA"))} %"}}
{fps_block()}"""


def fps_block():
    """The FPS (Figs. 5.1.5(a)-(c)) of the genhel_fcs component, M8c3 (D-045), in GenHel's
    units. The yaw integrator takes the heading error with the turn switch off and the lateral
    acceleration with it on, the reverse of the figure as printed (DATA_GAPS.md, "UH-60A
    AFCS"; `as_printed` selects the figure's wiring)."""
    f = constants("fig_5_1_5_fps.txt")
    r = "%/(deg/s)"
    return f"""\
    fps:  # Figs. 5.1.5(a)-(c), pp. 5.5-3, 5.5-11 to 5.5-13; fig_5_1_5_fps.txt (M8c3)
      hold: "{fmt(f["ZOH_FPS"])} s"  # zero-order hold, "airborne computer update delay"
      trim_actuator: {{gain: "{fmt(f["TA_GAIN"])} 1/s", rate_limit: "{fmt(f["TA_RATE"])} %/s", authority: "{fmt(f["TA_AUTH"])} %"}}
      pitch:
        rate_gain: "{fmt(f["KRQQPO"])} {r}"  # on the pitch SAS's A/D output
        attitude_gain: "{fmt(f["KSTHPO"])} %/deg"
        airspeed: {{gain: "{fmt(f["KSVXPO"])} %/kt", limit: "{fmt(f["LMVX"])} %", integral_gain: "{fmt(f["KIVXPO"])} %/(kt*s)", hold_above: "{fmt(f["V_HOLD"])} kt"}}
      roll:
        rate_gain: "{fmt(f["KRPRO"])} {r}"  # on the roll SAS's A/D output
        attitude: {{gain: "{fmt(f["KPHRO1"])} %/deg", breakpoint: "{fmt(f["BPPHRO"])} deg", gain_beyond: "{fmt(f["KPHRO2"])} %/deg"}}
        integral_gain: "{fmt(f["KIPHRO"])} %/(deg*s)"
      yaw:
        heading: {{gain: "{fmt(f["KSPSYO"])} %/deg", integral_gain: "{fmt(f["VGPSYO"])} %/(deg*s)", deadband: "{fmt(f["DBPSYO"])} deg"}}
        lateral_accel: {{lag: "{fmt(f["TLAYYO"])} s", gain: "{fmt(f["KLAYYO"])} %/(ft/s^2)", integral_gain: "{fmt(f["KIAYYO"])} %/(ft/s^2*s)"}}
        roll_rate_gain: "{fmt(f["KRPPYO"])} {r}"  # turn switch on
        collective: {{gain: {fmt(f["KSXCYO"])}, airspeed_rate_limit: "{fmt(f["KLVXYO"])} kt/s", fade_from: "{fmt(f["LMIXYO_V0"])} kt", fade_to: "{fmt(f["LMIXYO_V1"])} kt"}}  # %/%
        integrator_wiring: physical  # DATA_GAPS.md, "UH-60A AFCS"
"""


def map_rows(rows, indent, placeholders=()):
    """YAML for a linear control map; the (row, input) pairs in `placeholders` are tagged."""
    order = ("offset", "collective", "lon_cyclic", "lat_cyclic", "pedal")
    lines = []
    for row, gains in rows.items():
        items = [f"{k}: {q(float(f'{gains[k]:.10g}'), 'deg', (row, k) in placeholders)}"
                 for k in order
                 if k in gains and gains[k] != 0.0]
        lines.append(f"{indent}{row}: {{{', '.join(items)}}}")
    return "\n".join(lines)


def build(variant=None, engine="genhel"):
    """Config text: GenHel's aircraft, or a VARIANTS entry's weight and CG; on GenHel's
    engines (D-053) or Ballin's (engine="ballin", D-054)."""
    mr = constants("main_rotor_constants_5_1_5.txt")
    note = ""
    if variant is not None:
        weight, fs, bl, wl, source = {**VARIANTS, **BALLIN_VARIANTS}[variant]
        for key, v in (("WEIGHT", weight), ("FSCG", fs), ("BLCG", bl), ("WLCG", wl)):
            if v is not None:
                mr[key] = v
        note = (f"#\n# Validation variant ({variant}): WEIGHT and CG from {source}.\n"
                f"# Inertias are GenHel's (D-002); not for use as the aircraft's own data.\n")
    tr = constants("tail_rotor_constants_5_4_5.txt")
    af = constants("airframe_constants_5_2_5_3.txt")
    mp = constants("mass_properties_5_10.txt")
    mo = constants("motion_input_5_10_5.txt")
    en = constants("engine_constants_5_6_5.txt")
    rig = constants("control_rigging_5_5.txt")
    gear = gear_constants()

    m_af, (fs_cg, bl_cg, wl_cg) = mass_split(mr, gear)

    def vec(x, y, z):
        return f'{{x: "{fmt(round(x, 6))} in", y: "{fmt(round(y, 6))} in", z: "{fmt(round(z, 6))} in"}}'

    def pos(fs, bl, wl):
        return vec(-(fs - fs_cg), -(bl - bl_cg), -(wl - wl_cg))

    tv = constants("control_travels_6_3.txt")
    travel, main_map, tail_map, lon_spread = control_map(rig, mr, tv)
    sens = constants("fig_5_1_2_sensors.txt")
    stab = constants("fig_5_5_1_7_stabilator.txt")
    k_main = 10.0 * vertical_slope("main_tire_vertical.csv", gear["PM"])
    k_tail = 10.0 * vertical_slope("tail_tire_vertical.csv", gear["PN"])

    def stop_damping(k_in, m_slug):
        return 2.0 * math.sqrt(k_in * 12.0 * m_slug)  # [lbf*s/ft]

    G = "GenHel, NASA CR-166309 (1981)"
    trav = ", ".join(f"{k} {fmt(round(a, 4))}..{fmt(round(b, 4))} in"
                     for k, (a, b) in travel.items())

    def main_gear(name, y_body, side):
        # y_body: GenHel's body y [in] from the centreline, positive to starboard
        # (Y_RMGA = BLMGA/12, p. 5.7-9). It is not a buttline, so it skips pos()'s sign flip;
        # the airframe CG's body y is -bl_cg. p. 5.7-10 prints Y_LMGA = Y_RMGA with the minus
        # missing; the left gear is mirrored (DATA_GAPS.md).
        axle = vec(-(gear["FSMGA"] - fs_cg), y_body + bl_cg, -(gear["WLMGA"] - wl_cg))
        return f"""\
      - name: {name}
        axle: {axle}
        unsprung_mass: "{fmt(gear["MMG"])} slug"
        brake: {side}
        tire:
          radius: "{fmt(gear["RTM"])} in"
          wheels: {int(gear["NTM"])}
          pressure: "{fmt(gear["PM"])} psi"
          rated_pressure: "{fmt(gear["PRM"])} psi"
          vertical_load: gear/main_tire_vertical.csv
          damping_stiffness: gear/main_tire_damping_stiffness.csv
          lateral_stiffness: gear/main_tire_lateral.csv
          fore_aft_coefficient: 0.53
          damping_factor: "{fmt(gear["GDR"])} s"
          damping_lag: "0.01 s"
        strut:
          preload: "{fmt(gear["FPREM"])} lbf"
          air_spring: "{fmt(gear["CPVM"])} ft*lbf"
          damper:
            break_speed: "{fmt(gear["SDOT_BRK_M"])} ft/s"
            low: {{coefficient: "{fmt(gear["CMG_LO"])} lbf*s^2/ft^2", offset: "{fmt(gear["FDRG0_LO"])} lbf"}}
            high: {{coefficient: "{fmt(gear["CMG_HI"])} lbf*s^2/ft^2", offset: "{fmt(gear["FDRG0_HI"])} lbf"}}
          stop_stiffness: {q(round(k_main, 3), "lbf/in", True)}
          stop_damping: {q(round(stop_damping(k_main, gear["MMG"]), 3), "lbf*s/ft", True)}
"""

    # The tail tire's K = 19.7 sqrt(Z) lbf/in (Z in lbf, p. 5.7-46) as K_ref sqrt(Z/Z_ref).
    z_ref = 1000.0
    k_ref = 19.7 * math.sqrt(z_ref)

    text = f"""\
# source: {G}: main rotor p. 5.1-49, tail rotor p. 5.4-14,
# fuselage p. 5.2-15, empennage p. 5.3-19, stabilator Fig. 5.1.7, control rigging
# Figs. 5.1.6(a)-(d) and p. 5.5-20, landing gear pp. 5.7-40 to 5.7-47, mass properties,
# gyroscopic components and downwash correction terms p. 5.10-19; transcriptions in
# data/uh60a/genhel/.
# Generated by tools/build_uh60a_config.py; edit the transcriptions or the script, not this file.
#
# Mass split (DECISIONS.md D-002, D-034): GenHel's WEIGHT {fmt(mr["WEIGHT"])} lbm and CG
# (FS {fmt(mr["FSCG"])}, BL {fmt(mr["BLCG"])}, WL {fmt(mr["WLCG"])} in) are the whole aircraft's. The airframe
# below excludes the four blades (main rotor) and the three unsprung masses (gear); its CG is at
# FS {fmt(round(fs_cg, 4))}, BL {fmt(round(bl_cg, 4))}, WL {fmt(round(wl_cg, 4))} in, the origin of every position here.
# Positions: x = -(FS - FS_cg), y = -(BL - BL_cg) (buttlines positive to port), z = -(WL - WL_cg).
#
# Controls (D-034, D-035): stick and pedal travels from GenHel Figs. 6.3.1 and 6.4.1 (pp. 6.16-6.17):
#   {trav}.
# Collective, lateral and pedal fit the rigging exactly; the pedal uses its stroke with overtravel.
# The longitudinal rows put neutral cyclic in three places {fmt(round(lon_spread, 3))} in apart; neutral is the
# pinned row's, the end nearest the fit to Ballin's GenHel longitudinal stick (D-035 M7c amendment).
{note}
geodetic_origin: {{latitude: "0 deg", longitude: "0 deg", altitude: "0 m"}}  # scenario, not aircraft data

airframe:
  mass: "{fmt(round(m_af, 6))} lbm"
  inertia: {{ixx: "{fmt(mp["I_X"])} slug*ft^2", iyy: "{fmt(mp["I_Y"])} slug*ft^2", izz: "{fmt(mp["I_Z"])} slug*ft^2", ixz: "{fmt(mp["I_XZ"])} slug*ft^2"}}

{shaft_comment(engine)}
shafts:
  - name: rotor_drive
    speed: "{fmt(mr["OMEGA"])} rad/s"
    parts:
{shaft_parts(mo, mr, en) if engine == "genhel" else ballin_shaft_parts(mo, mr)}
    drive: free

components:
  # Producers before consumers is the executive's job (D-003); config order is for reading.
{engine_block(en, mr, mo) if engine == "genhel" else ballin_component_entry()}
{fcs_block(rig, mr, travel, sens, stab, pos, variant is None)}
  - type: blade_element_rotor
    name: main_rotor
    blades: {int(mr["NB"])}
    radius: "{fmt(mr["R"])} ft"
    rotor_speed: "{fmt(mr["OMEGA"])} rad/s"
    direction: counter_clockwise
    inflow: pitt_peters
    vortex_ring: johnson_2005  # Johnson NASA/TP-2005-213477 (D-049)
    ground_effect: genhel  # GenHel CR-166309 Eq. 5.8.2-1 (D-050)
    shaft: {{name: rotor_drive}}  # D-052
    inflow_tip_loss: effective_disc  # Johnson 2013 §3.6.2 (D-051)
    hub:
      position: {pos(mr["FS_MR"], mr["BL_MR"], mr["WL_MR"])}
      pitch: "{fmt(mr["I_THETA"])} deg"
      roll: "{fmt(mr["I_PHI"])} deg"
    blade:
      root_cutout: "{fmt(round(mr["E"] + mr["E_PRIME"], 6))} ft"
      tip_loss: {fmt(mr["B_TL"])}
      chord: "{fmt(mr["C_75"])} ft"
      twist_table: twist.csv
      airfoil: sc1095.csv
      elements: 20
      flap:
        hinge_offset: "{fmt(mr["E"])} ft"
        mass: "{fmt(mr["W_B"])} lbm"
        first_moment: "{fmt(mr["M_B"])} slug*ft"
        inertia: "{fmt(mr["I_B"])} slug*ft^2"
        delta3: "{fmt(mr["DELTA_3"])} deg"
      dynamic_twist: {{gain_table: dynamic_twist_gain.csv, mode_table: dynamic_twist_mode.csv, lag: "0.01 s"}}  # D-038; lag numerical
      yawed_flow_drag: true  # D-039 (GenHel resolves its drag on the full segment velocity, p. 5.1-28)
    pitch_input: fcs  # GenHel's mixer, primary servos and head limits (D-045)

  - type: airframe_aero
    name: airframe
    fuselage:
      mounting_point: {pos(af["FS_WF"], af["BL_WF"], af["WL_WF"])}
      alpha_table: airframe/fuselage_alpha.csv
      psi_table: airframe/fuselage_psi.csv
      psi_low_table: airframe/fuselage_psi_low.csv
      main_rotor_wash: {{rotor: main_rotor, table: airframe/fuselage_wash.csv}}
    tail_wash: {{rotor: main_rotor, table: airframe/tail_wash.csv}}
    stabilator:
      position: {pos(af["FS_H1"], af["BL_H1"], af["WL_H1"])}
      area: "{fmt(af["SA_H1"])} ft^2"
      incidence_input: fcs  # GenHel Fig. 5.1.7, dynamic (D-045)
      coefficients: airframe/stabilator_coefficients.csv
      q_ratio: airframe/stabilator_q_ratio.csv
      downwash: airframe/stabilator_downwash.csv
    fin:
      position: {pos(af["FS_V1"], af["BL_V1"], af["WL_V1"])}
      area: "{fmt(af["SA_V1"])} ft^2"
      incidence: "{fmt(af["I_V1"])} deg"
      coefficients: airframe/fin_coefficients.csv
      q_ratio: airframe/fin_q_ratio.csv
      sidewash: airframe/fin_sidewash.csv
    additional_loads: airframe/motion_additions.csv  # p. 5.10-19 downwash correction terms (D-048)

  - type: bailey_rotor
    name: tail_rotor
    blades: {int(tr["NB_TR"])}
    radius: "{fmt(tr["R_TR"])} ft"
    rotor_speed: "{fmt(tr["OMEGA_TR"])} rad/s"
    shaft: {{name: rotor_drive, gear: {fmt(tr["OMEGA_TR"] / mr["OMEGA"])}}}  # tail/main speed (D-052)
    chord: "{fmt(tr["C_TR"])} ft"
    lift_slope: "{fmt(tr["A_TR"])} 1/rad"
    tip_loss: {fmt(tr["B_TR"])}
    twist: "{fmt(tr["TWST_TR"])} deg"
    delta3: "{fmt(tr["DELTA3_TR"])} deg"
    coning_per_thrust: "{fmt(tr["DA0DT_TR"])} deg/lbf"
    pitch_bias: "{fmt(tr["BIAS_TR"])} deg"
    drag_area: "{fmt(tr["CD_TR"])} ft^2"
    blockage: {{factor: {fmt(tr["K_BVT_TR"])}, speed: "{fmt(tr["V_BVT_TR"])} kt"}}
    hub:
      position: {pos(tr["FS_TR"], tr["BL_TR"], tr["WL_TR"])}
      roll: "{fmt(tr["GAMMA_TR"])} deg"
    pitch_input: fcs
    main_rotor_wash: {{rotor: main_rotor, table: airframe/tail_wash.csv}}
    fuselage_wake: {{airframe: airframe}}

  - type: landing_gear
    name: gear
    friction:
      coefficient: {fmt(gear["MU"])}
      brake_factor_set: {fmt(gear["BRK"])}
      brake_factor_off: {fmt(gear["NBRK"])}
      sliding_factor: {fmt(gear["KSLIP"])}
    gears:
{main_gear("left_main", -gear["BLMGA"], "left")}{main_gear("right_main", gear["BLMGA"], "right")}\
      - name: tail
        axle: {pos(gear["FSNGA"], gear["BLNGA"], gear["WLNGA"])}
        unsprung_mass: "{fmt(gear["MNG"])} slug"
        brake: none
        tire:
          radius: "{fmt(gear["RTN"])} in"
          wheels: {int(gear["NTN"])}
          pressure: "{fmt(gear["PN"])} psi"
          rated_pressure: "{fmt(gear["PRN"])} psi"
          vertical_load: gear/tail_tire_vertical.csv
          damping_stiffness: gear/tail_tire_damping_stiffness.csv
          lateral_stiffness_law: {{reference_load: "{fmt(z_ref)} lbf", reference_stiffness: "{fmt(k_ref)} lbf/in"}}
          fore_aft_coefficient: 0.53
          damping_factor: "{fmt(gear["GDN"])} s"
          damping_lag: "0.01 s"
        strut:
          preload: "{fmt(gear["FPREN"])} lbf"
          air_spring: "{fmt(gear["CPVN"])} ft*lbf"
          damper:
            break_speed: "1 ft/s"
            low: {{coefficient: "{fmt(gear["CNG"])} lbf*s^2/ft^2", offset: "{fmt(gear["FDNG0"])} lbf"}}
            high: {{coefficient: "{fmt(gear["CNG"])} lbf*s^2/ft^2", offset: "{fmt(gear["FDNG0"])} lbf"}}
          stop_stiffness: {q(round(k_tail, 3), "lbf/in", True)}
          stop_damping: {q(round(stop_damping(k_tail, gear["MNG"]), 3), "lbf*s/ft", True)}
"""
    return text


# GenHel's gyroscopic components (p. 5.10-19, item 1; D-048): J [slug*ft^2] spinning at W
# [rad/s] about a body axis (p. 5.10-14). D-053 item 5 places them: the gas-generator spools
# (JXGY1, 4188 rad/s) in the engines at N_GG, the power turbines (JXGY2, 2094 rad/s) as the
# engine side, whose momentum J_TOTE/G_E gives, and the rest on the drive shaft.
SPINNING = (("x", "JXGY", "WXGY"), ("y", "JYGY", "WYGY"), ("z", "JZGZ", "WZGY"))
ENGINE_PARTS = {"JXGY1", "JXGY2"}


def shaft_parts(mo, mr, en):
    """The drive shaft's parts (D-053 item 5) as config rows: GenHel's spinning parts geared to
    the drive at their speed, and the rest of J_REST as inertia on the shaft with no angular
    momentum, as GenHel takes its gyroscopics from the p. 5.10-19 list alone (D-053 amendment
    4). J_REST is referred to rotor speed, so each geared part takes J (W/Omega)^2 of it."""
    rows = []
    taken = 0.0
    for axis, j_name, w_name in SPINNING:
        i = 1
        while f"{j_name}{i}" in mo:
            name = f"{j_name}{i}"
            if name not in ENGINE_PARTS:
                j, w = mo[name], mo[f"{w_name}{i}"]
                vec = {"x": "{x: 1, y: 0, z: 0}", "y": "{x: 0, y: 1, z: 0}",
                       "z": "{x: 0, y: 0, z: 1}"}[axis]
                rows.append(f'      - {{inertia: "{fmt(j)} slug*ft^2", speed: "{fmt(w)} rad/s", '
                            f'axis: {vec}}}  # {name}, p. 5.10-19\n')
                taken += j * (w / mr["OMEGA"]) ** 2
            i += 1
    rest = en["J_REST"] - taken
    rows.append(f'      - {{inertia: "{fmt(round(rest, 6))} slug*ft^2"}}  # J_REST less the parts '
                f'above ({fmt(en["J_REST"])} - {fmt(round(taken, 4))}), no momentum\n')
    return "".join(rows).rstrip("\n")


def shaft_comment(engine):
    if engine == "genhel":
        return """# Rotor speed (D-052, D-053): main and tail rotors geared on one shaft, driven by
# GenHel's T700 engines through their clutch (free drive). Its parts: GenHel's
# spinning parts geared to the drive (p. 5.10-19, D-048), turning with it, each
# as its own inertia at its speed; and the rest of J_REST ("downstream of clutch
# less blades", p. 5.6-23), referred to rotor speed, with no angular momentum:
# GenHel's gyroscopics are the p. 5.10-19 list alone (D-053 amendment 4)."""
    return """# Rotor speed (D-052, D-054): main and tail rotors geared on one shaft, driven by
# Ballin's component-level T700s (NASA TM-100991). Its parts: GenHel's spinning
# parts geared to the drive (p. 5.10-19, D-048), each its own inertia at its speed,
# for the gyroscopics (D-053 amendment 4); and the rest of Ballin's drive train
# (TM-88360 App. A: gearbox, main-rotor shafting and hub, tail-rotor drive,
# accessories), referred to rotor speed, with no angular momentum (D-054 item 3).
# The engines' output inertia is the engine component's."""


def ballin_shaft_parts(mo, mr):
    """The drive shaft's parts on Ballin's engines (D-054 item 3): GenHel's spinning parts
    geared to the drive, as on GenHel's engines, and the rest of TM-88360 App. A's drive train
    referred to rotor speed (J n^2), less what those parts take, with no momentum."""
    d = transcription("drive_train_constants.txt")
    omega = mr["OMEGA"]
    n_tr = d["OMEGTR"][0] / omega
    budget = (d["JGBX"][0] + d["JSHMR"][0] + d["JSHTR"][0] * n_tr ** 2 +
              d["J_ACC"][0] * d["n_ACC"][0] ** 2)
    rows = []
    taken = 0.0
    for axis, j_name, w_name in SPINNING:
        i = 1
        while f"{j_name}{i}" in mo:
            name = f"{j_name}{i}"
            if name not in ENGINE_PARTS:
                j, w = mo[name], mo[f"{w_name}{i}"]
                vec = {"x": "{x: 1, y: 0, z: 0}", "y": "{x: 0, y: 1, z: 0}",
                       "z": "{x: 0, y: 0, z: 1}"}[axis]
                rows.append(f'      - {{inertia: "{fmt(j)} slug*ft^2", speed: "{fmt(w)} rad/s", '
                            f'axis: {vec}}}  # {name}, p. 5.10-19\n')
                taken += j * (w / omega) ** 2
            i += 1
    rest = budget - taken
    rows.append(
        f'      - {{inertia: "{fmt(round(rest, 6))} slug*ft^2"}}  # TM-88360 App. A: JGBX + JSHMR + '
        f'JSHTR (OMEGTR/OMEGA)^2 + J_ACC n_ACC^2 = {fmt(round(budget, 4))}, less the parts above '
        f'({fmt(round(taken, 4))}); no momentum\n')
    return "".join(rows).rstrip("\n")


def ballin_component_entry():
    """The ballin_t700 block (ballin_engine_block) as an entry of an aircraft config's
    components. The aircraft configs on Ballin's engines sit beside its tables, in
    engine_ballin/, so the tables' bare names stand."""
    lines = [ln for ln in ballin_engine_block().splitlines() if not ln.startswith("#")]
    out = []
    for k, ln in enumerate(lines):
        prefix = "  - " if k == 0 else "    "
        comment = "  # Ballin, NASA TM-100991 (1988); engine_ballin/ballin_t700.yaml (D-054)" \
            if k == 0 else ""
        out.append(prefix + ln + comment)
    return "\n".join(out)


def engine_block(en, mr, mo):
    """GenHel's T700 engines, governor and clutch (CR-166309 §5.6; D-053) as the genhel_t700
    component: the constants of p. 5.6-24 (engine_constants_5_6_5.txt) and Tables 6.5.1 to
    6.5.6 (engine/*.csv)."""
    # The drive's inertia for the clutch's logic: J_REST plus the main-rotor blades about the
    # shaft, 4 (I + 2 e S + m e^2) (D-053 item 5).
    m_slug = mr["W_B"] / SLUG_LBM
    blades = mr["NB"] * (mr["I_B"] + 2.0 * mr["E"] * mr["M_B"] + m_slug * mr["E"] ** 2)
    j_d = en["J_REST"] + blades
    return f"""  - type: genhel_t700  # GenHel §5.6, pp. 5.6-8 to 5.6-28; engine_constants_5_6_5.txt (D-053)
    name: engines
    shaft: {{name: rotor_drive}}
    rotor: main_rotor  # reads main_rotor.shaft_torque, GenHel's Q_HMR
    engines: {int(en["N_E"])}
    gear: {fmt(en["G_E"])}
    power_turbine_speed: "{fmt(en["N_PT100"])} rpm"
    gas_generator_speed: "{fmt(en["N_GG100"])} rpm"
    loss_factor: {fmt(en["K_Q"])}
    maps:
      torque: engine/t700_torque.csv
      lds_rigging: engine/t700_lds_rigging.csv
      lds_cam: engine/t700_lds_cam.csv
      fuel: engine/t700_fuel.csv
      p3: engine/t700_p3.csv
      partials: engine/t700_partials.csv
    ecu:  # Fig. 6.1.3, pp. 5.6-11 to 5.6-13
      k1: "{fmt(en["K_1"])} V/%"
      k3: "{fmt(en["K_3"])} V/%"
      k4: "{fmt(en["K_4"])} V/(ft*lbf)"  # in no printed equation; Q_INT (D-053 item 2)
      k6: {fmt(en["K_6"])}
      k9: "{fmt(en["K_9"])} 1/s"
      k10: "{fmt(en["K_10"])} s^2"
      k11: "{fmt(en["K_11"])} s"
      dead_band: {{a: "{fmt(en["A_DB"])} V", b: "{fmt(en["B_DB"])} V"}}
      t1: "{fmt(en["T_1"])} s"
      t2: "{fmt(en["T_2LL"])} s"
      t3: "{fmt(en["T_3LL"])} s"
      t4: "{fmt(en["T_4"])} s"
      integrator_limits: {{upper: "{fmt(en["A_U"])} V", lower: "{fmt(en["A_L"])} V"}}
      g4: "{fmt(en["G_4"])} mA/V"
      g5: "{fmt(en["G_5"])} in/mA"
      g6: "{fmt(en["G_6"])} V/in"
      g7: "{fmt(en["G_7"])} %/in"
      T2: "{fmt(en["T_2"])} s"
      T3: "{fmt(en["T_3"])} s"
    switch: {{speed_band: {{low: "-4 %", high: "1 %"}}, torque_gain: "0.7 1/s", threshold: "20 V", limit: "40 V"}}  # p. 5.6-12
    fuel_control:  # pp. 5.6-14, 5.6-15
      speed_lag: "{fmt(en["T_NGG"])} s"
      ratio_gain: "{fmt(en["DWP3_DNGG"])} lbm/(hr*psi*%)"
      p3_lag: "{fmt(en["T_P3"])} s"
      valve_lag: "{fmt(en["T_MV"])} s"
    gas_generator: {{acceleration: "{fmt(en["KCF_JGG"])} %/(ft*lbf*s)", heat_sink: {fmt(en["K_HS"])}}}
    clutch:  # Fig. 6.1.4, pp. 5.6-17, 5.6-18
      demand_lag: "0.3 s"
      drive_inertia: "{fmt(round(j_d, 6))} slug*ft^2"  # J_REST + 4 (I_B + 2 E M_B + m E^2)
      engine_inertia: "{fmt(en["J_TOTE"])} slug*ft^2"  # J_TOTE
    power_turbine_spool: {{axis: {{x: 1, y: 0, z: 0}}}}  # JXGY2's axis (p. 5.10-19)
    gas_generator_spool: {{inertia: "{fmt(mo["JXGY1"])} slug*ft^2", axis: {{x: 1, y: 0, z: 0}}}}  # JXGY1
"""


def ballin_relocate(text):
    """An aircraft config in engine_ballin/: every table path gains one "../" except the
    engine's own tables, which sit beside it."""
    import re
    own = set(re.findall(r"^\s+f[\w]*: (\S+\.csv)", ballin_engine_block(), re.M))
    return re.sub(r"([\s{:,])([A-Za-z0-9_.][A-Za-z0-9_./-]*\.csv)",
                  lambda m: m.group(1) + (m.group(2) if m.group(2) in own else "../" + m.group(2)),
                  text)


def relocate(text, name):
    """Config text written to a subdirectory of DATA (name "dir/file.yaml"): the table paths,
    relative to the config's own directory, gain one "../" per level."""
    up = "../" * name.count("/")
    if not up:
        return text
    import re
    return re.sub(r"([\s{:,])([A-Za-z0-9_.][A-Za-z0-9_./-]*\.csv)", r"\g<1>" + up + r"\g<2>", text)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if the config is stale")
    args = parser.parse_args()
    gain, mode = dynamic_twist_tables()
    outputs = [(OUT, build())] + [(DATA / name, relocate(build(name), name))
                                  for name in VARIANTS] + [
        (BALLIN / name, ballin_relocate(build(variant, "ballin")))
        for name, variant in BALLIN_AIRCRAFT.items()] + [
        (DATA / "dynamic_twist_gain.csv", gain), (DATA / "dynamic_twist_mode.csv", mode)] + \
        list(engine_tables().items()) + list(ballin_engine_tables().items()) + \
        [(BALLIN / "ballin_t700.yaml", ballin_engine_block())]
    stale = 0
    for path, text in outputs:
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != text:
                print(f"{path.relative_to(ROOT)} is stale; run tools/build_uh60a_config.py",
                      file=sys.stderr)
                stale = 1
            continue
        path.parent.mkdir(exist_ok=True)
        path.write_text(text, encoding="utf-8")
        print(f"wrote {path.relative_to(ROOT)}")
    return stale


if __name__ == "__main__":
    sys.exit(main())
