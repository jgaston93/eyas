# Data Gaps

Missing, unverified or ambiguous aircraft data. Values marked `UNVERIFIED — needs human review` must not be used in code or config until a person has checked them against the source page.

Page references use the GenHel page label, plus the 1-based PDF page of the NASA scan of CR-166309.

---

## UH-60A mass properties: which set the rigid body uses — `RESOLVED by decision; discrepancy deferred`
**Resolution (2026-10-10):** use the GenHel motion-module inertias as written, as airframe-only values (DECISIONS.md D-002). The digits on PDF p. 342 are confirmed by human review. WEIGHT is total helicopter weight (p. 5.1-39). The discrepancy below remains unexplained and is deferred. Revisit it if Eyas's yaw or roll response disagrees with reference data, or if GenHel Vol. II or the Ballin report turns up.

**Background on the discrepancy:**
**Source:** GenHel CR-166309.
- Table 4.2, p. 4.8 (PDF p. 20), typed: whole-aircraft mass properties for 8 loading conditions, in lb·in·s².
- Motion-module input, p. 5.10-19 (PDF p. 342), handwritten: I_x = 4659.0, I_y = 38512.0, I_z = 36796.0, I_xz = 1882.0 (slug·ft², per the nomenclature on p. 5.10-15).
- Main-rotor input, p. 5.1-49 (PDF p. 69), handwritten: WEIGHT = 16638.0 lb, FSCG = 355.9 in, WLCG = 248.2 in, BLCG = 0.

**Question:** §5.10.1 says the body equations use the aircraft *less rotor blades*. But the motion-module I_z matches the whole-aircraft value, not whole aircraft minus the ≈ 6970 slug·ft² blade contribution. Are these inertias airframe-only or whole-aircraft? Does WEIGHT include the blades? See SPEC_REVIEW #2 for the arithmetic.

**Human check requested:**
1. Confirm the handwritten digits above.
2. Look for any note in GenHel (Vol. I or II) on how the motion-module inertias were derived.

## UH-60A main-rotor constants — partly verified
**Status (2026-10-10):**
- **Digits confirmed by human review:** every value in the table below, plus FS_MR/WL_MR/BL_MR = 341.2/315.0/0 in, WEIGHT = 16638.0 lb, and FSCG/WLCG/BLCG = 355.9/248.2/0 in.
- **Definitions confirmed from the GenHel nomenclature:**
  - M_b is slug·ft and I_b is slug·ft², both about the hinge (p. 5.1-47, PDF p. 67).
  - e′ (SPRLMR, ft) is the "Spar length exposed" (p. 5.1-39, PDF p. 59), measured **from the hinge to the blade cuff**, where the lifting blade and its first segment begin. Source: the blade-geometry sketch on p. 5.1-12 (PDF p. 32; read by the human reviewer, 2026-10-11) and the first-segment formula below it, which is the equal-area midpoint of an annulus starting at (ξ + ξ′)R. **Aerodynamic root cutout = (e + e′)/R = (1.25 + 3.83)/26.83 = 0.189 R** (5.08 ft from the center of rotation). The same page gives ξ = e/R and ξ′ = e′/R, and 5 equal-annulus-area segments (NSS = 5, p. 5.1-49).
  - θ_GEOMR (THLDMR, deg) is a lag-damper kinematics geometry constant (p. 5.1-46, PDF p. 66). It is **not** a blade pitch reference; the geometric blade pitch is T_HOAMR on p. 5.1-44.
  - WEIGHT is "Total helicopter weight" (p. 5.1-39), so it includes the blades.
- **Still open:**
  - i_θ and i_φ are not in the nomenclature (pp. 5.1-39 to 5.1-48). Shaft tilt is assumed; confirm the meaning and sign from the main-rotor equations (§5.1.2) before using them in the hub transform.
  - The remaining constants on p. 5.1-49 (lag damper, δ_0, T_FILMR, K_FP*, NBS/NSS, δ_3, ΔC_DMR, K_CT/K_CM/K_CL, T_DW*, ρ, a) are not yet reviewed. Review them before use.
- **Note:** an AI assistant's definitions of i_θ, e′ and θ_GEOMR were checked against the source. Two of the three were wrong. AI answers are not sources.

**Source:** GenHel p. 5.1-49 (PDF p. 69), handwritten. As read:

| Symbol | Value | Symbol | Value |
|---|---|---|---|
| R | 26.83 ft | e | 1.25 ft |
| Ω | 27.0 rad/s | W_b | 256.9 lb |
| b | 4 | M_b | 86.7 |
| i_θ (presumably shaft tilt, deg) | −3 | I_b | 1512.6 |
| Δ_SP | −9.7 | c_r, c_0.75 | 1.73 ft |
| B (tip loss) | 0.97 | θ_GEOMR | +17.481 |


## UH-60A tail-rotor constants — digits verified; two items open
**Source:** GenHel §5.4.5, p. 5.4-14 (PDF p. 175), handwritten. Symbols are spelled as on the page; they match the program mnemonics. **All digits were confirmed by human review on 2026-10-10.**

| Symbol | Value | | Symbol | Value |
|---|---|---|---|---|
| F_STR | 732.0 in | | Γ_TR | 70.0 deg |
| W_LTR | 324.7 in | | δ3_TR | 35.0 deg |
| B_LTR | −14.0 in | | C_TR | 0.81 ft |
| R_TR | 5.5 ft | | A_TR | 5.73 /rad |
| Ω_TR | 124.62 rad/s | | B | 0.92 |
| b_TR | 4 | | C_DTR | 0 |
| TWSTTR | −18 deg/R | | (∂a₀/∂T_TR) | 0.001455 deg/lb |
| K_BVTTR | 0.796 | | V_BVTTR / BIASTR | 30 kt / 6.0 deg |

### Γ_TR interpretation — accepted, noted for revisit
- Notation p. 5.4-11 (PDF p. 172) defines Γ_TR (GAMTR, deg) as the "Tail rotor cant angle".
- The geometry sketch on p. 5.4-4 (PDF p. 165) is a hand-drawn rear view. Its 70° arc appears to run from the horizontal body Y axis to the edge-on rotor disk. That means the disk is 20° off vertical and the thrust points about 20° upward, consistent with the spec's "≈ 20° cant".
- The human reviewer agrees with this reading, but the sketch is rough. **Revisit if** hover pedal or roll trim, or tail-rotor lift, disagrees with reference data. Also cross-check against the tail-rotor force and moment equations when implementing.

### B_LTR sign — settled from the equations: buttlines positive to port (`needs human check of the reading`)
- The UH-60 tail rotor is on the starboard side and body Y is positive to starboard. GenHel's arm is B_TR = (BLTR − BLCGB)/12 (p. 5.4-4). With B_LTR = −14.0 that puts the rotor about 1.2 ft to port, unless Sikorsky buttlines are positive to port.
- **Finding (2026-10-12, M6 survey):** the moment equations of three modules all match r × F only if buttlines are positive to **port** (and fuselage stations positive aft, waterlines positive up):
  - tail rotor, p. 5.4-8 (PDF p. 169): L_TR = Y_TR·W_TR − Z_TR·B_TR, N_TR = X_TR·B_TR − Y_TR·F_TR, with F, W, B = (station − CG)/12. With the arm (x, y, z) = (−F, −B, −W) in body axes, r × F gives exactly these;
  - horizontal tail, p. 5.3-8 (PDF p. 130): L = Y·W − Z·B, M = −X·W + Z·F, N = −Y·F + X·B, same arms, same result;
  - fuselage, p. 5.2-9 (PDF p. 94): arms defined the other way round (CG − station), L = −Y·W + Z·B etc., same result;
  - the tail-rotor sketch on p. 5.4-4 (PDF p. 165) draws the BLTR and BLCGB arrows pointing to the left in a rear view with +Y to the right.
- So B_LTR = −14.0 in puts the tail rotor 14 in to **starboard**, which matches the aircraft. The tail-rotor thrust Y_TR = T sin Γ > 0 (p. 5.4-8) then gives a nose-left yaw moment, as needed against a counter-clockwise main rotor. **Human check requested:** the three moment equations as read above.

## Tiltrotor data — `missing` (not needed for v1)
**Context:** DECISIONS D-011.
- No tiltrotor aircraft data is available.
- Data for a specific production tiltrotor is likely controlled or proprietary. If obtained, it must live in the access-controlled data directory, never in this repo (spec §2). If it comes as a reference-model vehicle file, the clean-room rule (§2.1) applies.
- A public analogue would let us develop and validate tiltrotor features without controlled data, the way GenHel serves for the UH-60. The likely candidate is the XV-15, which NASA has documented publicly. The exact reports are to be identified; none has been verified yet.

## ISA oracle and WGS-84 values — `VERIFIED by human review (2026-10-10)`
**Context:** D-015. These are test oracles, not aircraft data, but they come from a scan, so the human-review rule applies. File: *U.S. Standard Atmosphere 1976* (NASA-TM-X-74335) in the reference library. PDF page = book page + 16 in Part 1 and + 15 in the Part 4 tables. Table I alternates: odd book pages are by geometric altitude, even pages by geopotential altitude.

**Consistency check already done:** every row below agrees with the USSA equations (Eqs. 18, 33a/b, 42) to within the table's 5-significant-figure rounding (largest deviation 4e-5). A misread digit would almost certainly have shown up, and a person confirmed every value on 2026-10-10.

**1. Table 2 print errors (book p. 2, PDF p. 18).**
- Read as printed: R* = 8.31432 × 10⁻³ N·m/(kmol·K) and r₀ = 6.356766 × 10⁶ km.
- Taken instead from the text: R* = 8.31432 × 10³ (book p. 3) and r₀ = 6,356,766 m (book p. 8, below Eq. 17).
- **Confirmed by human review (2026-10-10):** Table 2 prints 10⁻³ and 10⁶ km; the text reads R* = 8.31432 × 10³ (PDF p. 19) and r₀ = 6,356,766 m (PDF p. 24). Eyas uses the text values.

**2. Table I rows** — **confirmed by human review (2026-10-10)**, all seven rows. Columns: Z [m], H [m′], T [K], P [mb], ρ [kg/m³].

| Table (book p., PDF p.) | Z | H | T | P | ρ |
|---|---|---|---|---|---|
| Geometric (53, 68) | 0 | 0 | 288.150 | 1.01325 × 10³ | 1.2250 |
| Geometric (53, 68) | 1000 | 1000 | 281.651 | 8.9876 × 10² | 1.1117 |
| Geometric (55, 70) | 5000 | 4996 | 255.676 | 5.4048 × 10² | 7.3643 × 10⁻¹ |
| Geometric (59, 74) | 11000 | 10981 | 216.774 | 2.2699 × 10² | 3.6480 × 10⁻¹ |
| Geopotential (52, 67) | 1000 | 1000 | 281.650 | 8.9874 × 10² | 1.1116 |
| Geopotential (54, 69) | 5004 | 5000 | 255.650 | 5.4019 × 10² | 7.3612 × 10⁻¹ |
| Geopotential (58, 73) | 11019 | 11000 | 216.650 | 2.2632 × 10² | 3.6392 × 10⁻¹ |

**3. Constants from the text** (typed, clear): M₀ = 28.9644 kg/kmol (p. 9), γ = 1.40 (Table 2, book p. 2), T₀ = 288.15 K, P₀ = 1.013250 × 10⁵ Pa, g₀ = 9.80665 m/s², layer gradients −6.5 and 0.0 K/km′ at H_b = 0 and 11 km′ (Table 4, p. 3).

**WGS-84 (NGA.STND.0036):** typed text layer, values extracted directly: a = 6378137.0 m and 1/f = 298.257223563 (Eq. 3-2, p. 3-2, PDF p. 28); b = 6356752.3142 m and R_p = 6399593.6258 m (Table 3.5, p. 3-9, PDF p. 35). **Confirmed by human review (2026-10-10).**

## Knight & Hefner (1937) hover data, digitized from Leishman Fig. 3.2 — `VERIFIED by human review (2026-10-11)`
**Context:** D-007 layer 3 (validation entry, not a gate). Test oracle, not aircraft data.
**Source:** Leishman, *Principles of Helicopter Aerodynamics*, 2nd ed., Fig. 3.2, p. 121 (DjVu page 159). Rotors with untwisted blades; the theory curves use C_lα = 5.73/rad (p. 120, last paragraph, DjVu page 158; the top line of that sentence is partly cut off in the scan, so the value needs checking too).
**Method:** read by eye from a 500-dpi render of the plot area, calibrated on the axis ticks (θ₀ = 0, 5, 10° and C_T = 0, 0.005, 0.01). Pitch values are taken as whole degrees, which the marker positions suggest. Expect about ±0.0001 in C_T.

| σ (marker) | θ₀ [deg] : C_T as read |
|---|---|
| 0.042 (filled circle) | 0 : 0; 2 : 0.00042; 4 : 0.00119; 6 : 0.00218; 8 : 0.00323; 10 : 0.00421; 12 : 0.00494 |
| 0.064 (open circle) | 6 : 0.00274; 8 : 0.00417; 10 : 0.00562; 12 : 0.00685 (points below 6° are hidden under other markers) |
| 0.085 (filled square) | 0 : 0; 1 : 0.00014; 3 : 0.00105; 4 : 0.00169; 5 : 0.00236; 6 : 0.00320; 7 : 0.00396; 8 : 0.00489; 9 : 0.00590; 10 : 0.00688; 11 : 0.00796; 12 : 0.00874 |
| 0.106 (open square) | 4 : 0.00183; 6 : 0.00351; 8 : 0.00551; 10 : 0.00775; 12 : 0.01000 |

**Review (2026-10-11):** every value was drawn as a cross over the figure (the overlay image is not kept in the repo, since the figure is copyrighted). The human reviewer confirmed that each cross sits on its marker, and confirmed C_lα = 5.73/rad on p. 120. A possible open square (σ = 0.106) hidden near 2° and the stacked markers at the origin were not added. An AI assistant's independent reading of the same plot was rejected: it listed points not on the plot and read some values off the theory curves. The data is in `tests/validation/reference/knight_hefner_1937.csv`.

## M5 validation rotor (NACA TN 3044 / TN 1698 rotor B) — readings `VERIFIED by human review (2026-10-12)`; rotor data incomplete
Test oracle for D-027 layer 3, not aircraft data. Readings: `tests/validation/reference/carpenter_fridovich_1953.csv`.

**Verified by human review (2026-10-12)** (each reading shown beside its cropped scan):
- NACA 23015 section lift slope, NACA Report 824 p. 147 (PDF p. 149, page rotated): c_l = 0 at α ≈ −1.2°, ≈ 0.11 at 0°, ≈ 0.98 at 8°, ≈ −0.72 at −8°, so a₀ ≈ 0.105–0.110 per deg; linear from about −10° to +10° (R = 2.6–8.9 × 10⁶).
- Rotor B, NACA TN 1698 p. 5 (PDF p. 7): R = 19 ft, σ = 0.042, three-blade area 46.3 sq ft, modified NACA 23015, 60 lbf per blade, plywood.
- NACA TN 3044 Fig. 3(b)–(d), pp. 20–22 (PDF pp. 21–23): pitch ramps, initial rotor speeds, peak and steady C_T, v/v_s points and the time at v/v_s = 0.9, as in the CSV.

**Gaps and conflicts:**
1. **Twist conflict.** TN 1698 lists rotor B with −8° linear washout; TN 3044 p. 4 (PDF p. 5) calls its NACA 23015 plywood blades "untwisted". Unresolved: either a different blade set or an error in one report. The validation runs both, and for the twisted blade it assumes the pitch is referenced at 0.75R (the reference station isn't given).
2. **Flap inertia and blade CG not published.** TN 3044 uses I₁ and m_b·l (eqs. 19–20) but prints no values; TN 1698 gives only the weight. The fast ramp (Fig. 3(a)) is excluded for that reason. Don't estimate them from the weight: that would mean inventing a mass distribution.
3. **Chord distribution and root cutout not given.** The blade tapers (TN 3044 Fig. 1, TN 1698 Fig. 5, photos only). The validation uses a constant chord from σ = 0.042 (0.836 ft) and no root cutout. The blade area gives σ = 46.3/(π·19²) = 0.041, 3% less.
4. **The lift slope used by TN 3044 is not printed** (an "adjusted value", p. 4). Report 824's section value is used instead.

## UH-60A blade twist and SC1095 airfoil tables — `ACCEPTED by human review (2026-10-11); primary maps checked cursorily`
**Source:** GenHel CR-166309 §5.1.5, dot-matrix printouts with slashed zeros (the OCR layer reads 0 as 8, # or _, so it is unusable). Read by eye from 250–600 dpi renders.
- Table 1.5.1, preset twist, p. 5.1-57 (PDF p. 77): 21 values.
- Table 1.5.2, C_l: primary map pp. 5.1-58 to 5.1-59 (PDF pp. 78–79), 33 α × 11 Mach; high-angle map p. 5.1-60 (PDF p. 80), 181 values.
- Table 1.5.3, C_d: primary map pp. 5.1-61 to 5.1-62 (PDF pp. 81–82); high-angle map p. 5.1-63 (PDF p. 83).

**Files:** the transcriptions, laid out row for row as printed, are in `data/uh60a/genhel/`. `tools/build_uh60a_blade.py` turns them into `data/uh60a/sc1095.csv` and `data/uh60a/twist.csv`, which carried `# status: placeholder` until the review was accepted (D-025).

**Checks already done** (they catch misreads in the high-angle maps, not in the primary maps):
- Outside ±32°, the C_d high-angle map is symmetric in α and the C_l map antisymmetric, value for value (about 300 values).
- Inside ±32°, both high-angle maps repeat the Mach 0 primary map, value for value, with one exception (item 3 below).
- Every Mach row of both primary maps has the same values at ±30° and ±32° (C_l ∓1.0 and ∓0.9675; C_d 0.63 and 0.6975).
- The twist table agrees with its plot (Figure 1.5.1, p. 5.1-50, PDF p. 70), including the shape near the tip. The primary-map curves agree in shape with Figures 1.5.2(b) and 1.5.3(b) (PDF pp. 72, 74). The plots are too coarse to check digits.

**Human check requested:**
1. Compare every number in `data/uh60a/genhel/table_1_5_2_cl_low.txt` and `table_1_5_3_cd_low.txt` (726 values) and `table_1_5_1_twist.txt` against the PDF pages above. These have no internal redundancy, so a misread digit would go unnoticed.
2. Smudged in the scan:
   - C_d, Mach 0.9, α = −2° (PDF p. 82): read as 0.056 (the second digit is unclear); **reviewer reads 0.066** (see below).
   - C_d, Mach 0.4, α = 6° (PDF p. 81, 4th row of the block): read as 0.011 (middle digit unclear). The Mach 0.5 value there is 0.011.
3. C_l high-angle map at α = −24° (PDF p. 80, row 16, 4th value) reads −.998, where the Mach 0 primary map has −0.988. Source typo or misread? It has no effect: the primary map is used there.
4. Values that look irregular but were read as printed:
   - twist −10.75° at r/R = 0.85, between −10.3° and −12.3° (the plot shows the same kink);
   - C_l at Mach 0.5, α = 8…16°: 0.96, 1.01, 0.96, 1.08, 1.06;
   - C_l at Mach 0.4, α = −28°: −0.955, in a row otherwise spaced 0.002 apart (−0.96, −0.962, …).

**Review so far (2026-10-11):**
- Table 1.5.1 (twist): confirmed by the human reviewer.
- Smudged digits (item 2): the reviewer reads **0.066** for C_d at Mach 0.9, α = −2° (my 0.056 was wrong; corrected in the transcription) and confirms 0.011 for Mach 0.4, α = 6°.
- Item 4: C_l −0.955 at Mach 0.4, α = −28° confirmed as printed, and the Mach 0.5 row too.
- Also confirmed as printed: C_l Mach 0.5 at α = −22° (−0.925), α = −8° at Mach 0.4 (−0.30) and Mach 0.9 (−0.665); C_d Mach 0.3 at α = −8°, −6° (0.0082, 0.0079) and Mach 0.5 at α = −6° (0.01).
- The rest of the primary maps (item 1) had a **cursory check only**, and looked right.
- Item 3: the page reads −.998, as transcribed (a source inconsistency with the primary map; no effect).
- **Accepted (2026-10-11)** by the reviewer on that basis; the placeholder status was removed. **Revisit if** Eyas's hover power, thrust against collective or forward-flight trim looks wrong: a digit-by-digit check of the primary maps comes first, starting with C_d lines 3–4 and C_l lines 4–5 of the Mach 0.0–0.7 blocks (α ≈ −12° to +16°, the range the blades fly in), where one misread digit matters most.

**Interpretation (from the equations, §5.1.2):**
- The twist argument XSEGMR = y₂ + ξ (p. 5.1-25, PDF p. 45). p. 5.1-12 (PDF p. 32) defines ξ = e/R_T and ξ′ = e′/R_T, "normalized by R_T", and y₂ as the normalized distance from the hinge to the segment midpoint, so XSEGMR is r/R from the center of rotation, which is how `twist.csv` is written. (The nomenclature, p. 5.1-39, says only "ND".)
- The twist is added to θ_CUFF, the pitch at the blade cuff (p. 5.1-24), so in Eyas θ₀ is the cuff pitch.
- Lift is looked up on α_trans, a swept-flow angle built from α and the yaw angle γ (p. 5.1-26, PDF p. 46). With cos γ = 1 every branch reduces to α_trans = α, which matches Eyas's omission of the radial velocity (D-024). Drag uses the actual α (p. 5.1-27).
- The tip-loss factor multiplies lift only, by segment overlap with B·R (p. 5.1-27, PDF p. 47). This agrees with D-024, which keeps drag outboard of B·R.
- C_d is reduced by a profile-drag correction ΔC_DMR (p. 5.1-27), one of the unreviewed constants on p. 5.1-49.

## UH-60A flap constants (M4) — `VERIFIED by human review (2026-10-11)`; δ₀ meaning open (M9)
**Source:** GenHel p. 5.1-49 (PDF p. 69), handwritten. **Confirmed by human review (2026-10-11):** K_β = 0 (flapping spring stiffness), K_β̇ = 0 ("Flapping damper rate", nomenclature p. 5.1-47, PDF p. 67), δ₃ = 0, δ₀ = 7 deg.
**δ₀ (mnemonic LAGOMR, deg): meaning not yet established.** The nomenclature gives units but no description. Clues:
- The nomenclature (PDF p. 66) lists LAGOMR right after THLDMR, in the group of "input constants defining the geometry of the lag damper kinematics".
- Figure 1.1.4, "Lag damper kinematic geometry" (p. 5.1-10, PDF p. 30), draws δ₀ at the hinge in the plane of rotation, beside the lag angle δ. It looks like a reference lag angle of the damper geometry, e.g. the blade's lag position at the damper's neutral stroke.
- Settle it from the lag-damper equations in §5.1.2 (where δ₀ enters the pick-up point positions) and the text on p. 5.1-5 (PDF p. 25) when the lag DOF is implemented (M9). It plays no part in M4 (flap only).

## UH-60A airframe, tail-rotor interference, stabilator and gear data (M6) — `to transcribe`
**Context:** D-028 (proposed). Located during the M6 survey (2026-10-12); nothing transcribed yet. Every value goes through human review before use, as for the blade tables.
- Fuselage (§5.2.5, PDF pp. 100–104): FSWF = 345.5 in, WLWF = 234.0 in, BLWF = 0.0 in, I_WF = 0.0 (handwritten, p. 5.2-15); Tables 2.5.1–2.5.2 (rotor wash on the fuselage against χ = 0…100° and a₁ = −6, 0, 6°), Tables 2.5.3–2.5.11 (six coefficients against α_WF or ψ_WF; dot-matrix, slashed zeros).
- Empennage (§5.3.5, PDF pp. 141–145): FSH1 = 700.1 in, WLH1 = 244.0 in, BLH1 = 0, SAH1 = 45.0 ft²; FSV1 = 695.0 in, WLV1 = 273.0 in, BLV1 = 0.0, SAV1 = 32.3 ft², IV1 = 0.0 (handwritten, p. 5.3-19); Tables 3.5.1–3.5.10.
- Stabilator law (Fig. 5.5.1.7, PDF p. 194, handwritten, page rotated): biases, limits and gains (e.g. 42, −30 kt, −80 kt, 50, 70, 0.16, −0.0433, −0.1119, 0.67, 0.003286, −8°…39°, ±7°/s). Read as above at 110 dpi only; several digits need a closer look. The units of the collective input XCPC need §5.5.
- Landing gear (§5.7.5, PDF pp. 278–286): main gear constants and Tables 7.5.1–7.5.6.
- Control rigging (§5.5.1.6, collective and pedal to blade pitch): needed for M6d.

**Inconsistencies in the source, to resolve before use (human check requested):**
1. *Main-rotor downwash sign at the fin.* V_ZMRH1 = −DWSHMR·Ω·R·EKZH1 at the stabilator (p. 5.3-5) and V_ZMRTR = −DWSHMR·Ω·R·EKZTR at the tail rotor (p. 5.4-5), but V_ZMRV1 = **+**DWSHMR·Ω·R·EKZV1 at the fin (p. 5.3-9, PDF p. 131). With GenHel's velocities being the aircraft's velocity relative to the air, the minus sign is physical (downwash lowers the stabilator angle of attack). The fin's + looks like a transcription slip; it affects only the fin's dynamic pressure and α_V1.
2. *Map arguments.* The equations look up the stabilator dynamic-pressure ratio and fuselage downwash on α_WF (p. 5.3-6), but the table headers say ALFWF for the input and ALFHH1 in the range comment (Tables 3.5.3–3.5.4, PDF p. 143).
3. *Table 3.5.10 (fin drag) high-angle map:* the comment reads "−90 to 90, DELTA=5" but holds 19 values, i.e. a 10° spacing (PDF p. 145).
4. *DLQD, DMQD, DNQD* are added to the fuselage lift, pitch and yaw coefficients (p. 5.2-7, PDF p. 92) but aren't defined in the notation (pp. 5.2-12 to 5.2-14) or the input data. Presumably zero (stores or configuration increments); to confirm.
5. *C_DTR* is "ft²" in the notation (p. 5.4-12) and "−" on the data page (p. 5.4-14). The value is 0, so it has no effect.

## UH-60A tail-rotor torque — `missing` (M6, M7)
GenHel's tail rotor computes thrust only (p. 5.4-2) and gives no blade profile-drag coefficient, so Eyas's tail rotor has no torque in M6 (D-028). Its power is a sizable part of hover power, which matters for the M7 power comparison and the M9 drivetrain. Check whether GenHel's engine module (§5.6) accounts for tail-rotor power, and look for a public tail-rotor blade section or profile-drag value before M7.
