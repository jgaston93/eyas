"""Build the UH-60A aircraft config data/uh60a/uh60a.yaml from the GenHel transcriptions.

Reads the as-printed constants in data/uh60a/genhel/ (GenHel, NASA CR-166309) and writes the
whole-aircraft config the model loads (DECISIONS.md D-034):

    python3 tools/build_uh60a_config.py            # write the config
    python3 tools/build_uh60a_config.py --check    # fail if the config is stale

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
- Controls (§5.5, Figs. 5.1.6(a)-(d)): normalized input -> stick or pedal inches -> GenHel's
  mechanical mixing [deg] -> the swashplate phase rotation (p. 5.1-24), written as the
  offsets and gains of Eyas's linear control map. The travels in inches come from GenHel's
  cockpit section (Figs. 6.3.1 and 6.4.1, pp. 6.16-6.17; D-035). Collective, lateral and pedal
  fit the rigging exactly (checked here). The longitudinal stroke is given but its rows put
  neutral cyclic in three places; neutral is their mean, and the values that depend on it
  (the theta1s and theta1c offsets) are tagged !placeholder.
"""

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT / "data" / "uh60a" / "genhel"
DATA = ROOT / "data" / "uh60a"
OUT = DATA / "uh60a.yaml"

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
    # disagree by up to 0.8 in (DATA_GAPS.md), so neutral is their mean.
    kb = rig["KXBB1S"] * rig["SIGN_XB2"]

    def b1s_rest(xc_in):  # B1S with the stick at XB+2 = 0
        return (rig["MIX_BC"] * thosum(xc_in) + rig["MIX_BP"] * rig["KXPTHR"] * xp_n
                + rig["BIAS_B"])
    est = [(tv["B_FWD_B1S"] - b1s_rest(xc_high)) / kb - tv["B_FWD_IN"],
           (tv["B_PIN_B1S"] - b1s_rest(xc_low)) / kb + tv["B_PIN_IN"],
           (tv["B_AFT_B1S"] - b1s_rest(xc_mid)) / kb + tv["B_AFT_IN"]]
    xb_n = sum(est) / len(est)
    xb = (xb_n - 0.5 * tv["STROKE_B"], xb_n + 0.5 * tv["STROKE_B"])  # -1 = full aft
    return xc, xa, xb, xp, max(est) - min(est)


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


def build():
    mr = constants("main_rotor_constants_5_1_5.txt")
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
# The longitudinal rows put neutral cyclic in three places {fmt(round(lon_spread, 3))} in apart; neutral is
# their mean, so the theta1s and theta1c offsets are tagged !placeholder.

geodetic_origin: {{latitude: "0 deg", longitude: "0 deg", altitude: "0 m"}}  # scenario, not aircraft data

airframe:
  mass: "{fmt(round(m_af, 6))} lbm"
  inertia: {{ixx: "{fmt(mp["I_X"])} slug*ft^2", iyy: "{fmt(mp["I_Y"])} slug*ft^2", izz: "{fmt(mp["I_Z"])} slug*ft^2", ixz: "{fmt(mp["I_XZ"])} slug*ft^2"}}

components:
  # Producers before consumers is the executive's job (D-003); config order is for reading.
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
    controls:
{map_rows(main_map, "      ", (("theta1s", "offset"), ("theta1c", "offset")))}

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
      schedule: {{table: airframe/stabilator_schedule.csv, min: "-8 deg", max: "39 deg"}}
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
    controls:
{map_rows(tail_map, "      ")}
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


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if the config is stale")
    args = parser.parse_args()
    text = build()
    if args.check:
        if not OUT.exists() or OUT.read_text(encoding="utf-8") != text:
            print(f"{OUT.relative_to(ROOT)} is stale; run tools/build_uh60a_config.py",
                  file=sys.stderr)
            return 1
        return 0
    OUT.write_text(text, encoding="utf-8")
    print(f"wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
