# Spec Review — EYAS_DESIGN.md v0.1

A living list of technical problems, inconsistencies and gaps found in the design spec. Each entry names the spec section, the problem, why it matters, and a suggested resolution. When an issue is resolved, record the decision in `docs/DECISIONS.md` and set its status to `resolved → DECISIONS.md (<date>)`.

The hard constraints (the reference-model clean-room rule, proprietary data handling, no invented data) are not open to review here. This list covers engineering content only.

---

## 1. Linearization ignores rotor periodicity — `open`
**Spec:** §6.9, M7.
**Problem:** With individual blade states retained, the coupled system is linear time-periodic (LTP), not LTI. Central-difference A/B matrices taken at one rotor azimuth depend on that azimuth. The spec does not mention a multiblade coordinate (Coleman) transform or azimuth averaging.
**Why it matters:** M7 compares eigenvalues and derivatives against the reference model. Snapshot-azimuth Jacobians will not compare meaningfully.
**Suggested resolution:** Transform blade states to multiblade coordinates, then average the Jacobians over one revolution (or N_b-per-rev samples) for the retained model. For the 6-DOF condensed model, condense using the revolution-averaged response.

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

## 12. Reference-model license confirmation is blocking and unconfirmed — `open`
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

## 21. Uniform momentum inflow has no physical solution in the vortex-ring state — `open`
**Spec:** §6.5 (uniform inflow, M3; Pitt–Peters, M5).
**Problem:** in axial descent between roughly 0 and 2× the hover induced velocity, momentum theory doesn't apply (Padfield 3rd ed. §3.2.1, pp. 102–103: the vertical-descent solutions in Fig. 3.15 are "nonphysical", and "recourse to empiricism is required"). Eq. 3.139 then has up to three roots. The M3 solve (D-024) returns one deterministically: in the test rotor it stays on the normal-working-state branch up to a descent of about 3 v_h, then jumps to the windmill-brake branch (`docs/validation/m3/m3_inflow.png`). The thrust jumps with it. Pitt–Peters (M5) uses the same mass-flow parameter and inherits the problem. It also treats upflow through the disc with |λ| in the wake skew (D-027, "Upflow"), a choice no source validates there.
**Why it matters:** a piloted steep approach or a vertical descent can enter this region. The model gives a thrust discontinuity there instead of the loss of control power and roughness pilots expect.
**Suggested resolution:** before M8 piloted evaluation, add an empirical vortex-ring correction to the induced-velocity solution, e.g. a fitted curve of induced velocity against descent rate between the normal and windmill branches, with the source chosen from the D-001 references (Johnson 2013 and Leishman 2nd ed. both discuss empirical curves; verify which before use). Until then, the region is a known limitation.

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
