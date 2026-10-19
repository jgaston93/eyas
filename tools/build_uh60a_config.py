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
        "Ballin, NASA TM-88360 (1987), Fig. B1 (p. 28): gross weight 16000 lb, CG station "
        "351.0 in, CG waterline 231.5 in (the level-flight and climb trims of App. B)"),
    "uh60a_fletcher.yaml": (
        14350.0, 360.0, None, None,
        "Fletcher, 21st ERF (1995) Paper VII-4, p. VII-4-6: takeoff gross weight approximately "
        "14,350 lb, CG at station 360; no waterline given, so GenHel's WLCG is kept (DATA_GAPS.md)"),
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


def build(variant=None):
    """Config text: GenHel's aircraft, or a VARIANTS entry's weight and CG."""
    mr = constants("main_rotor_constants_5_1_5.txt")
    note = ""
    if variant is not None:
        weight, fs, bl, wl, source = VARIANTS[variant]
        for key, v in (("WEIGHT", weight), ("FSCG", fs), ("BLCG", bl), ("WLCG", wl)):
            if v is not None:
                mr[key] = v
        note = (f"#\n# Validation variant ({variant}): WEIGHT and CG from {source}.\n"
                f"# Inertias are GenHel's (D-002); not for use as the aircraft's own data.\n")
    tr = constants("tail_rotor_constants_5_4_5.txt")
    af = constants("airframe_constants_5_2_5_3.txt")
    mp = constants("mass_properties_5_10.txt")
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
# Figs. 5.1.6(a)-(d) and p. 5.5-20, landing gear pp. 5.7-40 to 5.7-47, mass properties
# p. 5.10-19; transcriptions in data/uh60a/genhel/.
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

components:
  # Producers before consumers is the executive's job (D-003); config order is for reading.
{fcs_block(rig, mr, travel, sens, stab, pos, variant is None)}
  - type: blade_element_rotor
    name: main_rotor
    blades: {int(mr["NB"])}
    radius: "{fmt(mr["R"])} ft"
    rotor_speed: "{fmt(mr["OMEGA"])} rad/s"
    direction: counter_clockwise
    inflow: pitt_peters
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

  - type: bailey_rotor
    name: tail_rotor
    blades: {int(tr["NB_TR"])}
    radius: "{fmt(tr["R_TR"])} ft"
    rotor_speed: "{fmt(tr["OMEGA_TR"])} rad/s"
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
        (DATA / "dynamic_twist_gain.csv", gain), (DATA / "dynamic_twist_mode.csv", mode)]
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
