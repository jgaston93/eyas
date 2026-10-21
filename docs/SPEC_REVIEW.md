# Spec Review — EYAS_DESIGN.md v0.1

A living list of technical problems, inconsistencies and gaps found in the design spec. Each entry names the spec section, the problem, why it matters, and a suggested resolution. When an issue is resolved, record the decision in `docs/DECISIONS.md` and set its status to `resolved → DECISIONS.md (<date>)`.

The hard constraints (the reference-model clean-room rule, proprietary data handling, no invented data) are not open to review here. This list covers engineering content only.

---

## 1. Linearization ignores rotor periodicity — `resolved → DECISIONS.md D-035 (2026-10-15)`
**Spec:** §6.9, M7.
**Problem:** With individual blade states retained, the coupled system is linear time-periodic (LTP), not LTI. Central-difference A/B matrices taken at one rotor azimuth depend on that azimuth. The spec does not mention a multiblade coordinate (Coleman) transform or azimuth averaging.
**Why it matters:** M7 compares eigenvalues and derivatives against the reference model. Snapshot-azimuth Jacobians will not compare meaningfully.
**Suggested resolution:** Transform blade states to multiblade coordinates, then average the Jacobians over one revolution (or N_b-per-rev samples) for the retained model. For the 6-DOF condensed model, condense using the revolution-averaged response.
**Resolution:** D-035 and its amendment (#29), implemented in M7b. Multiblade coordinates, periodic and averaged models, Floquet exponents as the check on the averaging, and condensation of the averaged model.

## 2. Mass bookkeeping in the coupled solve — `resolved → DECISIONS.md D-002 (2026-10-10)`
**Spec:** §5 (Coupling), §6.1.
**Problem:** `M_rb` is built from aircraft mass and inertia, and the blade components add their inertial contribution through H. If the configured gross weight and inertia include the blades (GenHel's do), the blade mass is counted twice. The spec also takes loads "about the CG", but the system CG moves as the blades flap or lag.
**Why it matters:** This gives wrong accelerations and wrong trim attitude, with subtle errors that grow with blade mass fraction.
**Suggested resolution:** Define the rigid-body mass and inertia as the airframe *excluding* the modeled blades, and sum loads about a fixed body reference point (the nominal CG of the airframe). Document the split in config and DECISIONS.md.

**Findings (2026-10-10, GenHel CR-166309):**
- §5.10.1 (p. 5.10-2, PDF p. 325) confirms the concern. The body equations of motion are "the equations of the aircraft less rotor blades (which have their own degrees of freedom) and the appropriate mass is used". GenHel avoids double counting by excluding the blades.
- **The GenHel inertia data doesn't clearly follow that statement.**
  - Motion-module input (p. 5.10-19, PDF p. 342, handwritten): I_x = 4659, I_y = 38512, I_z = 36796, I_xz = 1882 slug·ft².
  - Table 4.2 (p. 4.8, PDF p. 20), whole aircraft at 16000.9 lb: I_x ≈ 5462, I_y ≈ 39469, I_z ≈ 36887 slug·ft² (converted from lb·in·s²).
  - Blade data (p. 5.1-49, PDF p. 69): W_b = 256.9 lb, M_b = 86.7, I_b = 1512.6, e = 1.25 ft, b = 4. From these, the blades contribute about 6970 slug·ft² to I_z about the shaft.
  - Motion-module I_z is therefore about the *whole-aircraft* value, not about 29 900 as expected without the blades. I_x fits neither interpretation (whole aircraft minus blades gives ≈ 990).
  - The main-rotor WEIGHT = 16638.0 lb is also ambiguous: is it gross weight, or gross weight minus the blades (1027.6 lb in total)?
- **Status: `resolved → DECISIONS.md D-002 (2026-10-10)`.** Use the GenHel motion-module inertias as written, as airframe-only values. The inertia discrepancy is deferred (see D-002 and DATA_GAPS.md).

## 3. No data path between components — `resolved → DECISIONS.md D-003 (2026-10-10)`
**Spec:** §6 interfaces (`SimContext`, `ForceModel`, `DynamicComponent`).
**Problem:** `SimContext` carries only body state, controls, environment, time and azimuth. Several models need outputs of other models from the same evaluation:
- fuselage and empennage need main-rotor wake, inflow and skew (downwash, interference);
- the tail rotor needs fin blockage;
- the engine and governor (M9) need main-rotor torque.
**Why it matters:** The interface as written cannot express these couplings. Ad-hoc workarounds would break purity or create hidden ordering dependencies.
**Suggested resolution:** Define an explicit evaluation order (main rotor before airframe) and a per-evaluation, pre-allocated "shared outputs" struct passed through the executive. Propose this in DECISIONS.md before implementing (interface change).

## 4. Stateless `ForceModel` is insufficient — `resolved → DECISIONS.md D-004 (2026-10-10)`
**Spec:** §6 (`ForceModel`), §6.7.
**Problem:**
- Stick–slip tire friction needs a state (a bristle or anchor-point displacement). Without one, the aircraft creeps on slopes and on the ground under small loads.
- Stabilator actuator dynamics and rate limits need states.
- `ForceModel::compute` is non-const, which contradicts the purity requirement that `DynamicComponent::derivatives` enforces with `const`.
**Suggested resolution:** Make gear and stabilator `DynamicComponent`s (with G = H = 0 where appropriate), and make `ForceModel::compute` const.

## 5. Timing budget applies to the wrong call — `resolved → DECISIONS.md D-005 (2026-10-10)`
**Spec:** §7 rule 5, M8.
**Problem:** "Worst-case `step()` must stay under 50% of the host frame time." `step()` is one 1 ms physics sub-step, but a 60 Hz host frame (~16.7 ms) contains ~17 sub-steps. As written, the budget is nearly vacuous (a single step could take 8 ms).
**Suggested resolution:** Budget worst-case `advance()` for a nominal host frame, i.e. worst-case `step()` × ceil(sub-steps per frame) < 50% of `dt_host`. The benchmark should report both.

## 6. Rounding `dt_host` to nanoseconds drifts against the host clock — `resolved → DECISIONS.md D-006 (2026-10-10)`
**Spec:** §6.10 (time keeping), M1.
**Problem:** 1/60 s rounds to 16 666 667 ns. Over 1 h (216 000 calls) that is 3 600 000 072 000 ns, so Eyas gains 72 µs/h on the host clock. The M1 "exact step count over 1 h" test passes only because the 72 µs remainder is below one 1 ms step. After about 14 h an extra physics step appears.
**Why it matters:** This is small but real. It affects long-run sync with the host and any test that assumes exact agreement with host time.
**Suggested resolution:** Let the host pass time as integer ticks (ns or a frame count with a declared rational period), keeping the `double` overload for convenience. At minimum, document the drift and fix the M1 test's expected value.

## 7. M3 acceptance compares against a different model — `resolved → DECISIONS.md D-007 (2026-10-10)`
**Spec:** M3.
**Problem:** "Hover thrust, torque and power match blade-element momentum theory within 2%." BEMT (annular momentum, radially varying inflow) and blade-element with *uniform* inflow are different models and legitimately differ, especially in torque and induced power.
**Suggested resolution:** Use the closed-form uniform-inflow blade-element result as the oracle, under its assumptions: constant lift slope, linear twist, no tip loss, e.g. C_T = (σa/2)(θ0/3 + θ_tw/4 − λ/2). **Verified** against Leishman 2nd ed. Eq. 3.25, p. 121 (2026-10-10); see D-007. Treat BEMT agreement as a looser sanity band.

## 8. 1 kHz is too coarse for a blade-element tail rotor — `resolved (deferred to M9) → DECISIONS.md D-008 (2026-10-10)`
**Spec:** §6.6 ("later" BE tail rotor), §6.10 (1000 Hz default).
**Problem:** GenHel §5.4.5 (p. 5.4-14, PDF p. 175, handwritten) gives Ω_TR = 124.62 rad/s, which is 4.62 × Ω_MR (27.0 rad/s). At 1 ms per step that is 7.14° of azimuth per step, too coarse for individual-blade tail-rotor dynamics. It's fine for the v1 Bailey closed-form tail rotor, which has no azimuth dependence. The value is handwritten, so it is logged in DATA_GAPS.md for confirmation; the conclusion doesn't depend on its exact digits.
**Suggested resolution:** When the tail rotor moves to blade-element, either sub-step it internally, raise the physics rate, or keep it disk-averaged. Decide before M9.

## 9. M4 flap-frequency criterion is misworded — `resolved → DECISIONS.md D-009 (2026-10-10)`
**Spec:** M4.
**Problem:** "The non-rotating and rotating flap natural frequencies match ν_β." For an articulated blade without a hinge spring, the non-rotating flap frequency is zero. The ν_β formula given is the rotating, non-dimensional (per-rev) frequency.
**Suggested resolution:** Test the non-rotating frequency only when K_β > 0 (ω = sqrt(K_β / I_β)). Test the rotating frequency against ν_β.

## 10. Pitt–Peters and azimuth practicalities — `resolved → DECISIONS.md D-027 (2026-10-12)`
**Spec:** §6.4, §6.5.
**Problem:**
- Sign conventions for the C_L and C_M forcing terms and the λ1s/λ1c ordering differ between sources. The spec says "as published" without fixing one.
- The mass-flow parameter V in [L] is singular, or ill-conditioned, at zero inflow in hover, so a nonzero initialization is needed.
- If ψ is integrated as an unbounded state, wrapping it inside an RK4 step must be handled carefully.
**Findings (2026-10-10):**
- The primary source is available: 6th ERF Paper 47 (1980), which is the *Vertica* 1981 paper; see DECISIONS D-001.
- It and Johnson 11.146 disagree by 4× on the moment apparent mass (16/45π vs 64/45π), with the same hover L magnitudes. This must be reconciled before M5.
- The 1980 paper uses (λ0, λs, λc) with loads (C_T, C_L, C_M), where C_L and C_M carry −r sinψ and −r cosψ weightings, ψ is CCW, and ψ = 180° points forward (p. 47-3). Johnson uses (λ0, λc, λs) with loads (C_T, −C_My, C_Mx).

**Suggested resolution:** Pick one primary reference for the Pitt–Peters form and conventions, and cite it in code. Initialize inflow from the momentum solution. Derive ψ from `n_steps` while Ω is constant, and wrap only between steps once Ω becomes a state.

**Findings (2026-10-12, M5):**
- *The 4× conflict is resolved in favour of 16/45π.* The 1980 paper gives 16/45π in both Table 3 (with the decimal −.1132, which confirms the fraction) and Table 4 (p. 47-13). Johnson (2013) p. 437 (§11.3, history) quotes Peters (1974) for the same impermeable-disc apparent masses (Tuckerman 1925): K_m = 8/3π and K_I = 16/45π, with τ = 2K_I/v, "agreeing with the identified τ value of Crews, Hohenemser, and Ormiston (1973)". Johnson's own Eqs. 11.140 and 11.146 (64/45π) disagree with his p. 437 and with the primary source. Both sources use standard coefficients, i.e. normalized by ρπR²(ΩR)²(R): the 1980 M₁₁ = 8/3π (uncorrected) is the impermeable-disc apparent mass (8/3)ρR³ only in that normalization.
- *Conventions.* Written with azimuth-weighted loads C_1s = −C_L and C_1c = −C_M (1980 eq. 5), the 1980 Table 4 L and Johnson's Eq. 11.143 (whose C_Mx and −C_My are defined by +sin ψ and +cos ψ weightings, Eqs. 11.150–11.151) are the same matrix, so the conventions are reconciled.
- *Mass-flow parameter.* The 1980 paper's L₂₂ = 2/v in hover (eq. 20) is "identical to … simple momentum theory" with v the flow speed (eq. 23a); eq. 23b is offered as a suggested extension, not derived. Johnson recommends V = √(μ²+λ²) for the moment columns (pp. 431–433). In the nonlinear total-state form the thrust row needs V = √(μ²+λ²) for the steady state to equal momentum theory, and its linearization then gives Johnson's Eq. 11.144 automatically. V multiplies λ (the equation uses L⁻¹ = V L̂⁻¹), so the singularity at V = 0 doesn't arise.
- *Wake skew direction.* Both sources assume the free stream comes from ψ = 180°. Sideways and rearward flight need the skew terms rotated to the in-plane wind direction; neither source says so.
- The ψ question is settled by D-024 (ψ from time while Ω is constant).
- *"Three real, stable roots" (1980 p. 47-9) doesn't hold for Table 4 as printed.* Computed from the Table 4 L and M (transcription checked against the scan and confirmed by the user, 2026-10-12), the (λ0, λ1c) roots are complex for α < 74.6° (wake skew χ > 15.4°), with damping ratio ≥ 0.744; all roots are stable and L is invertible, as the paper says. The cause is the antisymmetric effective coupling: L13 = L31 > 0 while M22, M33 < 0. The M5 test asserts stability, invertibility and ζ ≥ 0.7, not real roots.
- **Status: `resolved → DECISIONS.md D-027 (2026-10-12)`.**

## 11. Build and determinism details — `resolved → DECISIONS.md D-010 (2026-10-10)`
**Spec:** §3, §7 rule 4, M0.
**Problem:**
- `-Wall -Wextra -Werror` with Eigen 3.4 on recent GCC (12+) triggers warnings inside Eigen headers.
- Bit-identical determinism also requires no `-ffast-math` and a fixed FP-contraction (FMA) policy. It only holds within one build configuration; Debug and Release will differ.
**Suggested resolution:** Include Eigen as a SYSTEM include. Set `-ffp-contract` explicitly, forbid fast-math, and state in the determinism test that the guarantee holds per build.

## 12. Reference-model license confirmation is blocking and unconfirmed — `moot → DECISIONS.md D-035 (2026-10-15)`
**Spec:** §2.1, §12 Q6.
**Problem:** The spec makes the license confirmation a prerequisite for using reference-model outputs. Its status is unknown.
**Suggested resolution:** Record the answer in DECISIONS.md. Until then, do not build anything that ingests reference-model outputs.

## 13. Scope is narrower than the intended targets — `resolved → DECISIONS.md D-011 (2026-10-10)`
**Spec:** §1 Goals ("single-main-rotor helicopter with a tail rotor"), §12 Q2.
**Problem:** A tiltrotor is a likely future target. Several spec assumptions are single-main-rotor specific: a fixed shaft tilt, a singleton main rotor, a fixed evaluation order, linear control gearing, and a drivetrain folded into the main rotor.
**Resolution:** v1 scope is unchanged, but these assumptions are generalized cheaply now (D-011, D-003).

## 14. Two M1 rigid-body criteria can't detect errors — `resolved → DECISIONS.md D-013 (2026-10-10)`
**Spec:** M1.
**Problem:**
- *Quaternion norm drift < 1e-12 per step, before renormalization.* For constant ω, RK4 applied to q̇ = ½ q ⊗ [0, ω] gives a norm error per step of about (|ω|h/2)⁶/144. At h = 1 ms that stays below double-precision round-off up to about 10 rad/s. The criterion therefore passes on round-off alone, and it can't detect a wrong attitude, because any skew-symmetric rate matrix preserves the norm.
- *Intermediate-axis instability.* Torque-free motion near the intermediate axis is periodic: the body flips and comes back. A check of the state after 60 s can miss the instability entirely. In an offline check (inertias 1, 2, 3; spin 1 rad/s about axis 2; perturbation 1e-3), the state at t = 60 s was back near the initial spin.
- The energy and angular-momentum drift bounds (1e-8 over 60 s) hold with a large margin (about 1e-14 at 1–2 rad/s). They are fine as written, but they detect coding errors, not integrator accuracy.
**Suggested resolution:** see D-013.

## 15. ISA model and the `Environment` interface are underspecified — `resolved → DECISIONS.md D-015, D-016 (2026-10-10)`
**Spec:** §6.2, §6 (`Environment`), M2.
**Problem:**
- *No source.* "ISA values match the standard tables" names no table. None of the D-001 references defines the ISA. Leishman 2nd ed. §5.2 (pp. 212–214) is a summary with rounded or off-standard constants: 288.16 K, 101,301 N/m², 6.51 K/km, and "980 mm" Hg for 29.92 inHg. Its Table 5.1 stops at 12,000 ft, so it covers neither 5000 m nor 11000 m.
- *Geometric or geopotential altitude.* ISA tables are given in geopotential altitude H. Eyas's altitude comes from NED and is geometric, Z. At 11 km, Z − H ≈ 19 m, which changes ρ by ≈ 0.2%.
- *"Temperature offset" is ambiguous.* It can mean the performance-chart convention (standard pressure at a pressure altitude, temperature shifted; Leishman Eq. 5.11), or a hydrostatic atmosphere with the temperature profile shifted. At a given geometric altitude the two give different ρ.
- *Who computes the atmosphere.* §6.2 puts the atmosphere inside Eyas, but the §6 `Environment` passes ρ, T and a from the host, constant over a whole host frame. That makes density lag the aircraft's altitude, and it makes the atmosphere an input rather than a function of the state.
**Suggested resolution:** use the U.S. Standard Atmosphere 1976 (identical to the ICAO ISA up to 32 km; p. v) as the primary source. Evaluate it on geopotential altitude, converted from geometric (USSA Eq. 18). Give ΔT a hydrostatic meaning. Have Eyas compute the atmosphere from the state in every derivative evaluation, with the host passing conditions only.

## 16. Tier 1 NED-to-geodetic conversion is underspecified — `resolved → DECISIONS.md D-017 (2026-10-10)`
**Spec:** §6.10 (`AircraftState` Tier 1), M2.
**Problem:** "NED-to-geodetic conversion matches a reference calculation" doesn't say which conversion. Treating NED as a plane tangent at the origin (exact geometry) and mapping it onto the ellipsoid by arc length (flat-earth map) differ by about d²/2R in altitude: ≈ 8 m at 10 km from the origin and ≈ 200 m at 50 km. The tangent-plane version makes level flight in NED climb away from the terrain in the out-the-window view. No ellipsoid source is named either.
**Suggested resolution:** use the flat-earth map, with WGS-84 radii of curvature at the origin (NGA.STND.0036). See D-017.

## 17. The M2 equilibrium criterion is met trivially — `resolved → DECISIONS.md D-018 (2026-10-10)`
**Spec:** M2.
**Problem:** "Started at rest with the dummy force exactly equal to weight" at level attitude cancels exactly in floating point: C_bn g_n = (0, 0, g) and F = (0, 0, −mg). The acceleration is exactly 0.0, so position and velocity don't drift at all, and the test can't fail through any frame or sign error that keeps level flight exact. The same holds for "a step gives the expected vertical acceleration": RK4 is exact for constant acceleration.
**Suggested resolution:** also test at a non-level attitude with a steady translation, using a force fixed in inertial space and rotated into body axes, with a tolerance derived from round-off. See D-018.

## 18. Bad environment inputs and out-of-range altitude can't be seen by the host — `resolved → DECISIONS.md D-019 (2026-10-11)`
**Spec:** §6.2, §6.10; D-015, D-016.
**Problem:** `advance()` doesn't check `Environment`. A NaN `isa_delta_T`, or one low enough to make the absolute temperature non-positive (a °C/K mix-up), gives NaN density and speed of sound. Once a model uses `ctx.air`, every step then fails as `non_finite`, with no hint that the input caused it. A NaN `wind_n` would do the same. Separately, D-015's out-of-range flag is set only in `SimContext::air`, which the host can't read, so flight outside the tabulated atmosphere goes unreported. Found in the M2 code review. Neither has an effect yet: nothing reads `ctx.air` before M3.
**Suggested resolution:** reject an invalid `Environment` the way a negative frame time is rejected, and report out-of-range altitude in `AdvanceStatus`. See D-019.

## 19. The M3 reference-model criterion is underspecified and can't gate — `resolved → DECISIONS.md D-022 (2026-10-11)`
**Spec:** M3 ("thrust coefficient against collective matches the reference model within 3%").
**Problem:**
- The reference model's inflow, tip-loss, root-cutout, element and airfoil-table choices can't be inspected (clean-room rule, §2.1), so a 3% difference can't be attributed to an Eyas bug, and a match can hide one.
- The case isn't defined: flight condition, rotor speed, density, collective range and inflow model are not stated.
- A pure percentage blows up near zero thrust.
- The data depends on a licensed run by someone else (and on SPEC_REVIEW #12), so most of the time the test would skip and couldn't block anything.
**Suggested resolution:** keep the comparison as a validation entry ("how close are we"), not a gate, with a defined case and a tolerance with an absolute floor. See D-022.

## 20. M3 blade-element details are underspecified — `resolved → DECISIONS.md D-024 (2026-10-11)`
**Spec:** §6.3, §6.4, §6.5, M3.
**Problem:**
- Whether the M3 rotor has blade mass. §6.4 puts blade inertial loads with the flap DOF (M4), but D-002 removes the blades from the airframe mass.
- "Sign convention per Padfield" for φ, while the M3 oracle (D-007) is Leishman §3.2. Padfield 3rd ed. Eq. 3.25 takes U_P positive *up* through the disc; Leishman 2nd ed. Eq. 3.2 takes it positive *down* (inflow).
- Blade numbering: §6.4 has ψ_k = ψ + 2π(k−1)/N_b, Padfield 3rd ed. Eq. 3.49 has ψ_i = ψ − 2π(i−1)/N_b (blade i trails blade 1). This matters for the M4 multiblade transform.
- What "tip loss B" removes (lift only, or lift and drag), and where the element boundary falls relative to B·R.
- Which thrust the uniform inflow balances (instantaneous or revolution-averaged), and how the iteration is bounded and flagged (§7 rule 3).
- The control-input sign conventions (`Controls` says only [−1, 1]).
**Suggested resolution:** see D-024.

## 21. Uniform momentum inflow has no physical solution in the vortex-ring state — `resolved → DECISIONS.md D-049 (2026-10-20)`
**Spec:** §6.5 (uniform inflow, M3; Pitt–Peters, M5).
**Problem:** in axial descent between roughly 0 and 2× the hover induced velocity, momentum theory doesn't apply (Padfield 3rd ed. §3.2.1, pp. 102–103: the vertical-descent solutions in Fig. 3.15 are "nonphysical", and "recourse to empiricism is required"). Eq. 3.139 then has up to three roots. The M3 solve (D-024) returns one deterministically: in the test rotor it stays on the normal-working-state branch up to a descent of about 3 v_h, then jumps to the windmill-brake branch (`docs/validation/m3/m3_inflow.png`). The thrust jumps with it. Pitt–Peters (M5) uses the same mass-flow parameter and inherits the problem. It also treats upflow through the disc with |λ| in the wake skew (D-027, "Upflow"), a choice no source validates there.
**Why it matters:** a piloted steep approach or a vertical descent can enter this region. The model gives a thrust discontinuity there instead of the loss of control power and roughness pilots expect.
**Suggested resolution:** before M8 piloted evaluation, add an empirical vortex-ring correction to the induced-velocity solution, e.g. a fitted curve of induced velocity against descent rate between the normal and windmill branches, with the source chosen from the D-001 references (Johnson 2013 and Leishman 2nd ed. both discuss empirical curves; verify which before use). Until then, the region is a known limitation.
**Resolution (D-049):** Johnson's vortex-ring model (NASA/TP-2005-213477, Tables 3–4, the model Johnson 2013 §4.2 refers to), with two misprints corrected and fades at its two edges. Uniform inflow uses it quasi-statically; Pitt–Peters uses its baseline curve in the thrust row and the VRS increment as a lagged thrust deficit. The inflow is single-valued, and the heave damping changes sign on Johnson's flight-test boundary. The Pitt–Peters |λ| skew choice in upflow is unchanged.

## 22. M4 flap dynamics are underspecified, and the cited analytic results are for a different rotor model — `resolved → DECISIONS.md D-026 (2026-10-11)`
**Spec:** §6.3 (δ₃), §6.4 ("Blade flap dynamics"), M4.
**Problem:**
- *Padfield's model isn't an offset-hinge model.* Padfield 3rd ed. §3.2.1 (pp. 81–92) derives the flap equations, the multiblade form (Eqs. 3.55–3.58) and the quasi-steady flapping solution (Eqs. 3.64–3.72) for a **centre-spring equivalent rotor** (hinge at the centre, stiffness from K_β, λ_β² = 1 + K_β/(I_βΩ²), Eq. 3.32). The spec asks for an offset hinge e·R and gives the offset-hinge ν_β. "a1, b1 match Padfield's analytic expressions" therefore holds exactly only for e = 0. Johnson (2013) §6.15 (Eqs. 6.254–6.280) gives the offset-hinge version, with spanwise constants c_n, d_n, f_n.
- *Mass distribution.* The spec's ν_β² = 1 + (3/2)e/(1−e) + K_β/(I_βΩ²) is for a uniform blade. For a rigid thin blade the general result needs only the blade mass m_b and its first and second moments about the hinge, S_β and I_β: ν² = 1 + e·S_β/I_β + K_β/(I_βΩ²) (Johnson Eq. 6.261, "ν² = 1 + e r_CG M/I"; Eq. 16.54). GenHel gives exactly these (W_b, M_b, I_b about the hinge; p. 5.1-49). The spec doesn't say what the config holds.
- *Linearized or exact.* The spec's reduced equation I_β(β̈ + Ω²ν²β) = M_aero is linearized in β, and Padfield Eq. 3.20 linearizes the hub-motion accelerations. The coupled solve only needs the equations to be affine in the body acceleration, which the exact rigid-blade kinematics are.
- *Blade weight.* Padfield Eq. 3.18 drops it; Johnson §6.9 keeps it; D-002 requires each component to carry its own weight.
- *Aerodynamics on a flapped blade.* §6.4 doesn't say how U_P and the induced velocity are resolved once β ≠ 0, or which thrust feeds the uniform inflow (aerodynamic only, or including blade inertia).
- *Undefined test quantities.* "a1, b1": the spec mixes the classical (a1, b1) notation with Padfield's β1c, β1s. "Coning against thrust matches the analytic result": no result is named. "Phase lag of about 90° minus the hinge-offset effect": no formula and no tolerance.
- *Not mentioned:* hinge damping (named in §6.4 but not defined), precone, flap and droop stops, initial blade states.
**Why it matters:** testing an offset-hinge rotor against Padfield's centre-spring closed forms would mix a modeling difference into the tolerance, as in SPEC_REVIEW #7.
**Suggested resolution:** see D-026.

## 23. M5 acceptance criteria are underspecified — `resolved → DECISIONS.md D-027 (2026-10-12)`
**Spec:** M5.
**Problem:**
- *"The inflow time constant in hover matches the published value."* No value or source is named, and "the" time constant is ambiguous. The inflow equation alone (loading held fixed) has τ = M₁₁/(4λ₀Ω) (Padfield 3rd ed. Eq. 3.179 for the Carpenter–Fridovich mass 8/3π; Eqs. 5.76 and 5.80 for the Pitt–Peters value 128/75π). Coupled to a blade-element rotor at fixed pitch, the load feedback δC_T = −(σa/4)δλ₀ shortens it by the lift-deficiency factor (Johnson 2013 Eqs. 11.148–11.149). The moment states have their own time constants.
- *"The step-collective thrust overshoot … compared against the reference model."* This has the same problems as SPEC_REVIEW #19: the reference model's internals can't be inspected, no case is defined, and the comparison depends on the license check (#12). The overshoot also has a closed form for a small step on a rigid-blade rotor: the thrust jumps by (σa/6)δθ₀ and decays to C′ times that, with C′ = 1/(1 + σa/(16λ₀)).
- *"Reduces to uniform momentum inflow in hover."* This is right, but only in hover: in forward flight the Pitt–Peters λ₀ also depends on the pitch moment through L₁₃, so it no longer equals the momentum solution.
**Suggested resolution:** see D-027.

## 24. M6 is underspecified, and GenHel's airframe models carry hidden state — `resolved → DECISIONS.md D-028 (2026-10-12)`
**Spec:** §6.6, §6.7, M6.
**Problem:**
- *Hidden state in the GenHel models.* GenHel (CR-166309) is the data source for M6, but several of its equations use values from the previous frame or from history, which Eyas's pure derivative function can't (spec §5; D-004):
  - tail-rotor inflow: the mass-flow term uses λ_TR from the previous pass (p. 5.4-7, PDF p. 168), so the inflow lag depends on the frame rate;
  - tail-rotor δ₃: the pitch correction uses the previous-pass thrust T_TR(t−1) (same page);
  - fuselage downwash and sidewash at the stabilator, fin and tail rotor are "delayed by (FS − FSCG)/(12·V_xb) sec" (pp. 5.3-6, 5.3-10, 5.4-5), a transport delay;
  - landing gear: the retained tire contact point, a discrete stick/slip switch and strut preload-mode logic (pp. 5.7-2 to 5.7-4, PDF pp. 240–242).
- *The stabilator isn't a schedule.* §6.7 asks for "a table-driven scheduler" of airspeed, collective and pitch rate. GenHel's stabilator (Fig. 5.5.1.7, p. 5.5-18, PDF p. 194) is an analog control law: a 0.398 s lag on airspeed (first misread as 3.98 s; confirmed by human review, DATA_GAPS.md), collective, pitch-rate and lateral-acceleration feedback, a second-order servo, a ±7°/s rate limit and −8°…39° travel. The lateral-acceleration term would make an algebraic loop through the coupled solve without the servo dynamics.
- *Tail-rotor torque.* §6.6 says "thrust and torque"; GenHel's tail rotor computes thrust only (p. 5.4-2) and gives no blade profile-drag data.
- *Acceptance criteria.*
  - "Hover pedal position and tail-rotor thrust balance main-rotor torque": a yaw-moment balance is enforced by the equations of motion, so the check is nearly tautological. What it can catch is sign and arm errors. A pedal *position* also needs the UH-60A control rigging (§5.5.1.6), and there is no published hover pedal value in the reference library to compare with.
  - "Fuselage and empennage forces match GenHel table values at the test points": no test points are named, and a lookup compared with the same table tests only the lookup.
  - "Sits stably on the ground with gear loads equal to weight": sound, but the rotor's own thrust, the load split between gears and creep on a slope aren't mentioned.
- *Ordering.* The D-003 coupling store is still an empty struct; M6 is its first user.
**Why it matters:** implementing the previous-frame and delay terms literally would break purity, `snapshot()`/`restore()` and the finite-difference Jacobians of M7. Dropping them silently would lose the tail-rotor inflow lag and the downwash-lag pitch damping.
**Suggested resolution:** see D-028.

## 25. No milestone covers the AFCS — `resolved → DECISIONS.md D-029 (2026-10-12)`
**Spec:** §9 (all milestones), §1 Goals.
**Problem:** the UH-60A's automatic flight control system (SAS, pitch bias actuator, flight-path stabilization, stabilator; GenHel §5.5) appears in no milestone. Two parts matter before M8:
- *Trim (M7).* The pitch bias actuator (±15% authority, a function of airspeed among other inputs, PDF p. 179) and the stabilator schedule change the trimmed longitudinal cyclic and pitch attitude, so a flight-test trim comparison without them compares different aircraft.
- *Piloted evaluation (M8).* The UH-60A's handling qualities with SAS off are not representative of the aircraft pilots know.
**Suggested resolution:** add the static (steady-state) parts of the stabilator and pitch bias actuator before the M7 trim comparison, and the dynamic AFCS (SAS, FPS attitude hold, stabilator servo and feedback) before M8, as its own milestone or as part of M8. GenHel §5.5 has the block diagrams and gains. To decide with the spec owner.

## 26. GenHel's wind axes point the drag forward in rearward flight — `resolved → DECISIONS.md D-030 (2026-10-13)`
**Spec:** §6.7, §6.8; GenHel CR-166309 pp. 5.2-6, 5.2-9, 5.3-7, 5.3-8, 5.3-11, 5.3-12.
**Problem:** GenHel computes every angle of attack as atan(V_Z/|V_X|) (fuselage, stabilator, fin) and builds the wind-to-body rotation from it. With V_X < 0 the rotation's drag axis keeps a forward x component, so the drag force points aft while the aircraft moves aft: it adds energy. GenHel says the transformation "gives invalid body axes forces and moments when these angles approach 90°" (p. 5.2-2) and offers a low-speed filter instead, which it doesn't run ("not being executed on PDP10", p. 5.2-10) and whose parameters it doesn't give. The UH-60A flies rearward and sideways at low speed (hover repositioning, rearward taxi), so Eyas needs forces that are at least physically consistent there.
**Why it matters:** a drag that feeds energy into a low-speed rearward drift is a destabilizing artefact, not aircraft behaviour, and it would appear in exactly the piloted hover tasks of M8.
**Suggested resolution:** see D-030.

## 27. The wind axes are singular in pure sideward flow — `resolved → DECISIONS.md D-031 (2026-10-14)`
**Spec:** §6.7, §6.8; GenHel CR-166309 pp. 5.2-2, 5.2-6, 5.2-9, 5.2-10; DECISIONS D-030.
**Problem:** on the line V_X = V_Z = 0, V_Y ≠ 0 (β = ±90°) the angles α = atan(V_Z/|V_X|) and α' = atan2(V_Z, V_X) are undefined, and close to it they take any value, depending only on the direction of a vanishing x–z component. The table lookups (in α) and the wind-to-body rotation (in α') follow them, so the loads are not a function of the velocity there. Found in code review (2026-10-13), measured with the UH-60A fuselage at 30 ft/s sideward with a 0.001 ft/s x–z component swept around the circle (no wash):
- production (D-030): rolling moment −684 to +1836 N·m, yawing moment +394 to −1723 N·m, body x force +378 to −178 N;
- GenHel transcription: rolling moment −504 to +1354 ft·lbf (the same spread), yawing moment +295 to −704 ft·lbf, body x force −6 to +85 lbf.

GenHel has the singularity too, and says so: the transformation "gives invalid body axes forces and moments when these angles approach 90°" (p. 5.2-2). D-030 didn't create it; it changes its shape (α' sweeps a full turn, GenHel's α half of one). The stabilator and fin have the same singular line in their local flow.
**Why it matters:** in flight the main-rotor downwash keeps the fuselage's V_Z away from zero, and the tail wash does the same at the panels. It matters with little rotor thrust: on the ground in a crosswind (M6d, M8) and possibly at M7 trim points near hover, where a finite-difference Jacobian across the line would be meaningless. Every RK4 stage near the line sees a load that changes by kilonewton-metres over a millimetre per second.
**Suggested resolution (proposal, for review):** regularize near the line, taking the structure from GenHel's own low-speed filter (p. 5.2-10). That filter fades, below |V_XWF| = 25 ft/s, from the wind-axis loads to body-axis low-speed loads X_LS…N_LS·q, in proportion to |V_XWF|/25. It can't be used as printed: X_LS…N_LS aren't given, its sign factors (α_WF/|α_WF|, V_YWF/|V_YWF|) jump themselves, and fading on |V_X| also changes vertical descent, where D-030's axes are already continuous. Proposed instead:
- fade on direction, not speed: s = smoothstep(cos β / c₀), with cos β = |V_xz|/|V|, so the fade acts only for |β| > acos c₀ and is C¹ for the M7 Jacobians;
- loads = s · (D-030 loads) + (1 − s) · (the same model at α = α' = 0, same β and q): the fixed-orientation term is the value GenHel's own atan2(0, 0) = 0 picks on the line, it depends only on β and q, and so the loads become continuous across the line;
- the drag keeps the flow's true direction in both terms (only its table value is faded), so the drag stays dissipative (`DragOpposesTheFlow`);
- the same fade for the fuselage and both panels, in the shared `flow_angles`;
- c₀ (first guess sin 10°, i.e. |β| > 80°) set by a sensitivity sweep reported in VALIDATION.md, as for AA1FMR (D-028).

Verification: the probe above, as a test (loads continuous as the x–z component goes to zero from every direction); `DragOpposesTheFlow`; the Layer-1 GenHel match unchanged outside the fade (its random cases kept to |β| < 80°, or checked against the faded reference).

**Resolution:** adopted as proposed (D-031), with the table angle faded rather than the table values and c = 20° from the sweep (the first guess of 10° failed the steepness criterion).

## 28. M7 can't gate on the reference model, and trim and linearization are underspecified for a periodic rotor — `resolved → DECISIONS.md D-035 (2026-10-15)`
**Spec:** §6.8, §6.9, M7; SPEC_REVIEW #1, #12; D-022, D-027, D-029, D-034 item 5.
**Problem:**
- *The reference-model criteria can't gate.* Every M7 criterion (trimmed controls, attitudes and power within ±5 % / ±1°, eigenvalues "in the same neighborhood", key derivatives) compares against the reference model. As in #19 and #23: the license check (#12) is still open, the reference model available is a generic medium helicopter rather than a UH-60A (D-027), and its internals can't be inspected (§2.1), so a mismatch can't be told apart from a modelling difference. "Agreed tolerances" and "the same neighborhood" are not defined either.
- *No published UH-60A trim or derivative data is in the reference library.* Padfield's derivative charts and tables (App. 4B) are for the Lynx, Bo105 and Puma. GenHel Vol. I says its check cases are in Vol. II, which the library doesn't have.
- *Trim residual with individual blades.* The UH-60A main rotor has four flapping blades at ψ = Ωt (D-024), so no state is constant in trim: the blades, the inflow harmonics and the airframe's 4/rev response are periodic. §6.8 allows "averaged over one or more revolutions" or quasi-steady flapping, but doesn't say what is averaged, whether the body is free or held during the revolution, or how the rotor states are made periodic.
- *Six unknowns are blade angles.* §6.8 names θ0, θ1c, θ1s, θ_TR. Eyas's inputs are the four `Controls`, mapped through GenHel's mixing (D-034), so the unknowns are the four controls and the blade angles are outputs.
- *Linearization of an LTP system* (#1, still open): the A and B matrices of the individual-blade model depend on azimuth.
- *Reduced states and neutral states.* §6.9 asks for (u, v, w, p, q, r, φ, θ, ψ_h); Eyas's body states are a quaternion and NED position. Some non-body states are neutral at a trim point (the airframe's wash-lag states at zero airspeed, D-028, have identically zero rows), so a quasi-static condensation that inverts the non-body block fails there.
- *Held-body evaluation.* The M6d harness (`tests/support/m6d_fixtures.hpp`) copies the executive's ordering and `SimContext` setup because the executive offers only the free-body derivative function. Trim needs the same evaluation (VALIDATION.md M6d, "Deferred to M7").
- *Before a trim comparison,* D-029 requires the steady-state pitch bias actuator, and D-034 item 5 defers three GenHel features (elastic twist, rotor-force filter, profile-drag increment) to rulings "on evidence" from the trim comparison.
- *Control travels.* D-034 left the stick and pedal travels as placeholders ("Vol. I gives none"). GenHel Vol. I §6 does give them (Fig. 6.3.1, p. 6.16, PDF p. 360; Fig. 6.4.1, p. 6.17, PDF p. 361); see DATA_GAPS.md. Trim controls in percent of travel depend on them.

**Why it matters:** without a defined residual and a defined linear model, "trim converges" and "the eigenvalues match" can't be tested, and Jacobians taken at one azimuth would vary with it (#1). Without a gate that can run, M7 couldn't block anything.
**Suggested resolution:** see D-035 (proposed).


## 29. D-035's linearization gates L2 and L3 and its state elimination don't hold as written — `resolved → DECISIONS.md D-035 amendment (2026-10-15)`
**Spec:** D-035 (Linearization, Acceptance L2 and L3), §6.9.
**Problem:** found while planning M7b, then checked against Johnson (2013) and Padfield 3rd ed. and measured on the UH-60A. The measurements were taken with a scratch probe that used the trims of `solve_trim`, central differences of the free derivative function along the held orbit (1024 azimuths), the multiblade transform, and a monodromy matrix built from the sampled Jacobians (midpoint matrix exponentials; converged, since 512 azimuths change the gaps by under 3 %). The probe's reduced derivative function matches the time derivative of the reduced coordinates along the free flow to 1e-9. That check caught a sign error in the multiblade centrifugal term of a first probe run; the numbers here are from the corrected run.
- *L3 can't show second-order convergence against the averaged model.* An exact linearization leaves an error of second order in the perturbation (Taylor), so halving the amplitude should quarter it. Two first-order errors stop that.
  - *Averaging.* In forward flight the multiblade coefficients stay periodic (Johnson p. 557), and the averaged model drops that part. Over 3 s, sampled once a revolution, the averaged response differs from the periodic linear response by up to 0.014 % of the response in hover, 0.7 % at 60 kt, 4.8 % at 120 kt and 7.9 % at 150 kt.
  - *Height.* D-035 drops position. A perturbed run changes altitude, so it also changes density and thrust. Over 3 s, leaving out the height column changes the response by up to 0.24 % (w, hover), at most 0.17 % elsewhere. Padfield's linear model (Eq. 4.44) leaves height out too; he notes that climbing flight isn't strictly a trim because the density changes (p. 183).
- *L2's hover premise holds only for a symmetric rotor.* Johnson (pp. 556, 572) gives constant multiblade coefficients for a rotor in axial flow whose rotating-frame equations have constant coefficients. In the UH-60A hover trim the disc is tilted against the tail rotor (cyclic flapping), so each blade's equation still varies with azimuth. Around the revolution the body rows of the multiblade Jacobian vary by 9 % (Frobenius). Nearly all of it is in the differential-coning (β₀d) columns, whose mean is zero but whose 2/rev part is large. Johnson (p. 557) notes the (N/2)/rev harmonics in the β_{N/2} coupling of an even-bladed rotor; here they come from the trim flapping and centrifugal loads (the cos²ψ_k terms survive the (−1)^k sum). Without those columns the variation is 0.14 %. The averaged eigenvalues in the flight-dynamic band (|λ| < 3 rad/s) differ from the Floquet exponents by at most 1.2e-4 rad/s. That's small, but not finite-difference precision.
- *The rule for dropping states is fragile.* D-035 drops states whose row and column are identically zero. In hover the wake-lag rows contain |u|, which central differences give as about 2e-8 rather than 0, while their columns are exactly zero (the wake term is proportional to u). Out-of-contact tire deflection and gear vertical-lag states have zero columns but nonzero diagonals (−1/G, −1/τ). A state set whose columns are zero outside the set feeds nothing else; the matrix is then block triangular, so removing it leaves every other eigenvalue unchanged. Measured: the full model's eigenvalues equal the reduced model's together with the dropped states' to 1e-10, at 0–150 kt.
- *Size of the averaging error.* Johnson (p. 757) says the constant-coefficient approximation "should always be checked by comparing the exact and approximate solutions". The largest Floquet-to-averaged gap in the flight-dynamic band is 5.8e-4 rad/s at 60 kt, 2.9e-3 at 120 kt and 2.5e-2 at 150 kt (μ = 0.35), where two slow real roots differ by 10–11 % (−0.259 against −0.234, −0.128 against −0.143 rad/s). Scaling the periodic part by s = 1, ½, ¼ shrinks the gap by 3.8–4.9× per halving at every speed, hover included. This is the second-order behaviour that first-order perturbation theory predicts: the first-order change of a simple Floquet exponent is the revolution mean of the periodic part, which is zero (own derivation; it isn't in the reference library).

**Why it matters:** as written, L3 fails in forward flight on a correct model, L2 fails in hover on a correct model, and the state elimination depends on finite-difference noise.
**Suggested resolution:** see D-035 amendment (2026-10-15):
- L3 compares against the periodic linear model at every speed; the averaged model is reported against it.
- The retained model keeps height; the condensed model drops it, as Padfield does.
- L2 gates exactly on a symmetric hover fixture and is reported on the UH-60A, with the s-scaling.
- States are dropped on zero columns.

## 30. The GenHel airframe tables make the model non-differentiable in straight flight — `resolved → DECISIONS.md D-036 (2026-10-15)`
**Spec:** §6.6 (fuselage and empennage), §6.9; D-028, D-030, D-034; found in M7b (D-035 L3).
**Problem:** GenHel's fuselage and empennage maps are tabulated every 5° and interpolated linearly (CR-166309 pp. 5.2-7, 5.3-8, 5.3-11), with the even maps looked up on |angle|. Where a map is even about zero, the interpolation draws a V with its corner at zero angle. The fuselage drag increment against sideslip is 0, 1, 4, 9, 16.3 ft² at 0, 5, 10, 15, 20°, nearly (ψ/5)², but it becomes |ψ|/5 ft²/deg next to zero. The fuselage lift increment and the fin's dynamic-pressure ratio against sideslip have the same V, and so do the stabilator's drag and dynamic-pressure ratio against angle of attack. Every straight-flight trim sits at zero fuselage sideslip, so it sits on the corners.
- *Measured (M7b):* at the trim the forward and backward differences of the sideways-velocity and yaw-rate columns differ by a fixed amount however small the step: 4.5e-3 (hover), 1.2e-2 (60 kt) and 2.3e-2 (120 kt) in the body-acceleration rows, all from the airframe component. Every other column, and every other component, is smooth.
- *Effect:* a perturbation to the left and one to the right see different slopes, so no linear model fits both. L3's error against the periodic linear model stopped shrinking with the amplitude at 1–3 % of the response in forward flight, worst for lateral perturbations.
- *Physics:* the underlying quantities are smooth and even in the angle, flat at zero. The corner is an interpolation artifact. In the nonlinear simulation it gives lift and drag changes proportional to |β| instead of β², i.e. a small first-order heave and pitch disturbance at twice the Dutch-roll frequency.

**Why it matters:** the linearization is ill-defined at every straight-flight trim, gate L3 can't pass, and the nonlinear model has a non-physical first-order coupling.
**Suggested resolution:** see D-036.

## 31. D-035's V1 and V2 don't match what Ballin and Fletcher publish — `resolved → DECISIONS.md D-035 amendment (M7c, 2026-10-15)`
**Spec:** D-035 (Acceptance V1, V2), M7; D-029, D-034 item 5.
**Problem:** found while planning M7c, by reading the two sources D-035 names.
- *Fletcher (21st ERF, 1995, Paper VII-4) prints no derivatives.* It gives Table 1 (identified against theoretical Lock number 8.62 / 8.06, M_β/I_β 0.0321 / 0.0573, pitch–flap coupling 7.84° / 0, aerodynamic phase lag 38.8°), Table 2 (the same at 80 kt), and Bode plots of flight data, the identified model and two linearized blade-element models (Figs. 3–14). V2's "derivatives … against the identified models" can't be done from it.
- *Ballin (NASA TM-88360, 1987) App. B has no sideslip sweep.* It has level flight (Fig. B1, 0–160 kt), climbs and descents at 60, 100 and 137.5 kt (Figs. B2–B4) and steady coordinated turns at 60 and 100 kt (Fig. B5). Eyas's trim is straight flight only (§6.8; D-035 leaves turns as an additive field).
- *Ballin's lateral trim constraint changes with speed.* In Fig. B1(e) the GenHel roll curve starts near −2.3° in hover and reaches 0 at about 50 kt, then stays on 0; the GenHel sideslip curve in B1(f) starts at 50 kt (about +5°) and decays to about −1° at high speed. So GenHel appears to trim at zero sideslip (roll solved) below about 50 kt and at zero roll (sideslip solved) from 50 kt up. `TrimCondition` always fixes β and solves φ. (Interpretation of a plot, including the 50 kt switch: `VERIFIED by human review (2026-10-15)`.)
- *App. D step responses* (17 cases, SAS, FPS and pitch bias actuator off, stabilator fixed) need states Eyas doesn't have yet: lag (D-026), rotor speed and engine (M9), and the dynamic AFCS (D-029, before M8).
- *Pitch bias actuator in the trims.* Ballin says it was centred and locked for the transient responses (p. 9, App. C) but says nothing about the static trims; D-029 asks for its steady-state part before the trim comparison.

**Why it matters:** V2 has no data to compare against as written, and V1 compares against trims made under a different lateral constraint unless Eyas trims the same way.
**Suggested resolution:** see the D-035 amendment (M7c): V2 redefined around what Fletcher publishes; V1 on Figs. B1–B4 at Ballin's weight, CG and density altitude, under GenHel's lateral constraint; turns and App. D deferred.

## 32. The Tier 4 outputs are tied to concrete component types and recompute the step's loads — `open (M9 or the tiltrotor work)`
**Spec:** §6.10 (`AircraftState` Tier 4), §6 (component interfaces); D-043.
**Problem:** found in the M8a code review.
- `Model` finds the main rotor, the tail rotor and the gear by `dynamic_cast` to `BladeElementRotor`, `BaileyRotor` and `LandingGear`, and reads their loads through type-specific queries. A new rotor model (M9's variable-Ω rotor, a tiltrotor's proprotors, a second main rotor) leaves its Tier 4 fields NaN, or picks a rotor by config order, until `Model` is edited.
- The output evaluation repeats the main-rotor blade-element pass, the tail-rotor and the gear evaluations that the final derivative evaluation has just done: about 4.8 µs per `advance()` call (D-043), 2 % of a 60 Hz frame but +37 % for a host calling every 1 ms step.

**Why it matters:** fine for the UH-60A, but it doesn't scale to the vehicles D-011 keeps open, and it costs a 1 kHz host time inside the D-005 budget.
**Suggested resolution:** components declare named outputs with a role (main-rotor thrust, torque, power, flapping; tail-rotor thrust; gear normal load), published during the evaluation like the D-003 coupling outputs, and `Model` maps roles to Tier 4 fields. Changes the §6 interfaces, so propose in DECISIONS when M9 or the tiltrotor needs it.

## 33. Unconverged bounded iterations are invisible to the host, and the output evaluation's aren't counted — `open (M8, with the Tier 5 diagnostics)`
**Spec:** §7 rule 3; D-023 ("revisit when Tier 5 diagnostics are finalized (M8)"); D-043.
**Problem:** found in the M8a code review.
- D-023 counts unconverged derivative evaluations in the executive (`Executive::unconverged_evaluations()`), not in `AdvanceStatus` or `AircraftState`.
- The M8a output evaluation discards its convergence flag, so it isn't even counted. The UH-60A uses Pitt–Peters inflow, whose output evaluation has no iteration, so only a config with uniform inflow is affected.

**Why it matters:** a host can't tell that outputs came from an unconverged solve. D-023 judged that harmless (slightly wrong loads, not a fault), but asked to revisit it with Tier 5.
**Suggested resolution:** add the output evaluation to the executive's count now, or when Tier 5 is settled. Decide whether `AircraftState` Tier 5 carries a count of unconverged evaluations per `advance()`, with the M8e reconciliation of the host's output list.

## 34. A crash diverges to finite garbage before it is flagged — `open (M8e, with the Tier 5 diagnostics)`
**Spec:** §6.10 (`AdvanceStatus`), §7; D-014, D-032.
**Problem:** found flying the M8b pilot station hands-off (`--start hover --input none`).
- The trimmed bare UH-60A holds its hover for about 50 s. The instability then grows, as expected without an AFCS, and at 85.2 s the aircraft meets the ground rolled 72° and descending at 11 m/s.
- One gear touches at 85.23 s. Within three 60 Hz frames, before any step turns non-finite, the state reaches a height of −1.3e15 m, a vertical speed of 3e39 m/s and a coning of 4.6e14 rad.
- Only then does a sub-step go non-finite. D-014 discards it, keeping the last good state, which is already garbage, so the host sees it with no flag until the step that overflows.
- GenHel's gear is a landing-gear model: vertical struts with tyres, valid near level. A wheel struck at 72° of roll is far outside it, and its stiff contact then outruns RK4 at 1 ms. The model isn't wrong to fail here; the problem is that the failure is silent until it overflows.

**Why it matters:** a host shows the aircraft at absurd positions for a frame or more, and a host's crash logic (or the M8e piloted evaluation) can't rely on `non_finite` alone.
**Suggested resolution:** decide in M8e with the host's crash handling and the Tier 5 diagnostics. Options:
- an `AdvanceStatus` flag when a component leaves its validity envelope (the gear beyond its stops or its attitude range, the inflow beyond its bounds), raised before the state diverges;
- or a documented host rule that a crash is the host's to detect from the outputs (attitude and load factor with weight on wheels), as the M8b station does (D-044).
