# Validation Log

One entry per milestone (spec §9): what was verified, how, and the result. Plots go in `docs/validation/` once a milestone produces any.

---

## M0 — Scaffolding (2026-10-10)

| Acceptance criterion (spec §9 M0) | Status | Evidence |
|---|---|---|
| Builds with `-Wall -Wextra -Werror` | ✅ | GCC 13.3 (Debug, Release), Clang 18.1 (Debug). Stricter set used: also `-Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wdouble-promotion -Wold-style-cast` (D-010). |
| Interpolation tests pass, incl. edge cases and clamping | ✅ | `Axis.*`, `Table.*`: exact node reproduction, multilinear reproduction to 1e-12, per-axis clamping, NaN propagation, and bit-identical hunt vs binary search over 20 000 random queries with random hints (purity). |
| CI runs the tests | ✅ | `.github/workflows/ci.yml`: Ubuntu GCC (Debug, Release, ASan+UBSan), Ubuntu Clang, Rocky Linux 9 (system GCC 11), Rocky Linux 8 (gcc-toolset-13), clang-format and clang-tidy. All 7 jobs green on the first GitHub run (run 38064913440, commit 849ae39). |

**Also verified:**
- No allocation during lookups or recorder writes, via the counting `operator new` (`AllocationCounter`, reusable for the M1 `advance()` test).
- AddressSanitizer and UBSan run clean (`asan` preset).
- clang-tidy is clean with the committed `.clang-tidy`.
- clang-format is clean with the committed `.clang-format`.

**Environments verified:**
- Ubuntu 24.04 (WSL2): GCC 13.3, Clang 18.1, CMake 3.28.
- Rocky Linux 9.3: GCC 11.5, CMake 3.31.8.
- Rocky Linux 8.9: gcc-toolset-13 (GCC 13.3), CMake 3.26.5.

RHEL 8's system GCC 8.5 is rejected at configure, with instructions (it lacks `<numbers>`).

---

## M1 — 6-DOF rigid body core (2026-10-10)

Every case runs through the production `Executive` (`src/sim/executive.*`): RK4 at 1 kHz, integer-nanosecond time, and the coupled 6×6 solve. The criteria are spec §9 M1 as amended by D-006 (integer time) and D-013 (strengthened quaternion and instability checks). The tests are in `tests/unit/test_rigid_body.cpp`, `test_coupled_solve.cpp` and `test_executive.cpp`. To reproduce the plots, see `tools/plot_m1.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| Free fall matches the analytic solution to 1e-9 relative over 10 s | ✅ | `RigidBody.FreeFallMatchesAnalyticSolution`. RK4 integrates the quadratic exactly, so only round-off remains. |
| Torque-free spin of an asymmetric body conserves energy and angular momentum (< 1e-8 relative drift over 60 s) | ✅ | `RigidBody.TorqueFreeSpinConservesEnergyAndMomentum`. Uses the full tensor with I_xz (D-002 inertias); angular momentum is measured in the inertial frame. Max drift: 1.3e-14 (energy), 3.0e-14 (momentum). |
| …and shows the intermediate-axis instability | ✅ | `RigidBody.IntermediateAxisSpinIsUnstableAtTheLinearizedRate` (D-013). The growth rate matches λ = Ω√((I₃−I₂)(I₂−I₁)/(I₁I₃)) to 1e-6. ω₂ reverses at t = 33.1 s, against ≈ 32 s predicted from ln(1/ε)/λ. Major- and minor-axis spins stay bounded. |
| Constant applied moment gives the analytic angular acceleration | ✅ | `RigidBody.ConstantMomentGivesAnalyticAngularAcceleration`: ω̇ = I⁻¹M to 1e-12, and ω(t) = I⁻¹M·t along a principal axis. |
| Coupled solve matches the analytic combined system | ✅ | `CoupledSolve.MatchesDirectSolutionOfCombinedSystem`. A spring–mass on a rail, with nonzero G and H, at a general state (rates, attitude, offset rail): the executive's Schur-complement solve agrees with an independent 7×7 Newton–Euler solve to 1e-12. `CoupledSolve.TwoMassOscillatorFrequency`: s(t) = s₀ cos(√(k/μ) t). Max error 8.25e-9 s₀ over 10 s, matching RK4's predicted phase error T ω_n⁵ h⁴/120 = 8.3e-9. |
| Purity: same (t, x, inputs) gives bit-identical derivatives, with other evaluations in between | ✅ | `Executive.DerivativeFunctionIsPure` (memcmp). Also `Executive.RunsAreBitIdentical`. |
| Time keeping: 1/60 s frames for 1 h give the exact step count, and sim time = n_steps × dt | ✅ | `Executive.OneHourAtSixtyHzGivesExactStepCount`: 216 000 calls of 16 666 667 ns give exactly 3 600 000 steps with a 72 000 ns remainder (D-006). `Executive.JitteredFramesKeepExactTime`: 5000 random frames, each call's step count exact. The sub-step cap and negative-dt handling are tested too. |
| Per-step quaternion norm drift, before renormalization, < 1e-12 | ✅ | `RigidBody.QuaternionNormDriftBelowLimitAtTypicalRates` (a 1 rad/s tumble). `RigidBody.QuaternionNormDriftMatchesRk4Prediction`: at 50 rad/s the drift equals RK4's predicted 1 − √(1 − θ⁶/72 + θ⁸/576) to 1e-3. `RigidBody.AttitudeMatchesClosedFormForConstantRate`: attitude matches the closed form to 1e-10 over 60 s (D-013). |
| Zero-allocation test passes | ✅ | `Executive.AdvanceDoesNotAllocate`, with a component and a force model in place, including the capped path. |

**Also verified:**
- The inertia tensor built from Padfield's (I_xx, I_yy, I_zz, I_xz) reproduces Padfield 3rd ed. Eq. 3A.33 (off-diagonal entries −I_xz).
- Gravity in body axes matches Eq. 3A.43.
- Builds and passes all 77 tests: GCC 13.3 (Debug, Release, ASan+UBSan) and Clang 18.1 (Debug). clang-format and clang-tidy are clean.
- Release build cost: about 0.54 µs per physics step with no components (3.6 M steps in 1.9 s). This is not the D-005 benchmark, which comes in M8.

**Plots** (`docs/validation/m1/`):

![Torque-free drift](validation/m1/m1_torque_free.png)
![Intermediate-axis flip](validation/m1/m1_intermediate_axis.png)
![Two-mass oscillator](validation/m1/m1_oscillator.png)
![Quaternion norm drift vs rate](validation/m1/m1_quat_norm.png)

The quaternion plot shows why D-013 was needed. Below about 10 rad/s, the per-step norm drift at 1 kHz is round-off (< 1e-15), so the spec's 1e-12 limit can't detect an attitude error. Above that rate, the drift follows RK4's θ⁶/144.

---

## M2 — Environment and static force stubs (2026-10-10)

The criteria are spec §9 M2 as amended by D-015 (ISA source and model), D-016 (the atmosphere is computed inside Eyas), D-017 (geodetic map) and D-018 (stronger equilibrium test, dummy rotor, public API). The acceptance cases run through the public `eyas::Model` (`include/eyas/model.hpp`) or the production `Executive`. Tests: `tests/unit/test_atmosphere.cpp`, `test_geodesy.cpp`, `test_attitude.cpp` and `test_model.cpp`. Plots: `tests/validation/m2_cases.cpp`, then `tools/plot_m2.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| ISA matches the standard tables at 0, 1000, 5000 and 11000 m | ✅ | `Atmosphere.MatchesUssaGeometricAltitudeTable`, `…GeopotentialAltitudeTable`. The oracle is USSA 1976 Table I, both the geometric-Z and geopotential-H tables (7 rows, human-verified; DATA_GAPS.md). T and ρ agree within ½ unit of the last printed digit. P agrees within 1 unit: see the note below the table. |
| Hydrostatic ISA + ΔT (D-015) | ✅ | `Atmosphere.OffsetAtmosphereIsHydrostatic`: dP/dZ = −ρ g(Z), using USSA Eq. 17 for g, holds to 1e-8 for ΔT = −30, 0 and +25 K in both layers. The code integrates in geopotential altitude, so this is an independent check. Also: sea-level P = P₀ for any ΔT, continuity at the tropopause, and a² = γP/ρ. |
| At rest, dummy force = weight: equilibrium for 60 s | ✅ | `Model.LevelEquilibriumIsExactFor60s`: position and velocity are **exactly** zero after 60 s, which the test asserts. At level attitude the cancellation is exact in floating point (SPEC_REVIEW #17). `Model.TiltedTranslatingEquilibriumHolds` (D-018): a tilted aircraft moving at 9.6 m/s under an inertially fixed balancing force drifts 6.8e-10 m in 60 s, within a 1e-8 m round-off bound. Its velocity error is exactly 0. |
| A step in the dummy force gives the expected vertical acceleration | ✅ | `Model.CollectiveStepGivesExpectedVerticalAcceleration`: collective 0.5 → 0.6 through `advance()` at 60 Hz gives a = 0.2 g up. w(t) and D(t) match the analytic values to round-off (1e-13 m/s, 1e-14 m), since RK4 is exact for constant acceleration. The Tier 1 altitude follows h₀ − D. |
| Tier 1 pose populated; NED-to-geodetic matches a reference calculation at several origins | ✅ | `Model.Tier1PoseMatchesStateAfterResetAndAdvance`. For the geodetic map (D-017), `Geodesy.MatchesEllipsoidArcLengthsAtSeveralOrigins` checks 5 origins (equator; 37° N; 34° S; 65° N at 1500 m; next to the antimeridian). North and east displacements up to 30 km match arc lengths on an independently constructed WGS-84 meridian ellipse, within the predicted second-order bound ½ (dR_M/dφ) Δφ². The radii match NGA Table 3.5 at the equator and the poles (b²/a, a, R_p). Euler angles: `Attitude.*`, round trip against Padfield Eqs. 3A.34–3A.40 over all quadrants. |

**Note on the pressure tolerance.** For all seven rows, the printed pressure equals the USSA equations' value with the last digit *truncated*. Rounding would fail three of them; at Z = 11000 m, Eq. 33a gives 226.9996 mb, printed as 226.99. T and ρ are consistent with rounding. So P is allowed one unit of the last digit, and T and ρ half a unit. This is an inference from the seven rows, not something the USSA document states.

**Also verified:**
- **Atmosphere in models (D-016):** `Model.ModelsSeeAtmosphereAtCgAltitude`. A force model sees ρ, T and a at h₀ − D with the host's ΔT.
- **API (D-018):**
  - `Model.RestoreReplaysBitIdentically`: state and remainder, compared with memcmp;
  - `Model.SnapshotAndRestoreDoNotAllocate`, `Model.RestoreRejectsMismatchedSnapshot`;
  - `Model.ResetSetsStateAndZeroesTime`;
  - `Model.OutputsCarryAdvanceStatus`;
  - config errors are `InitError`s, including a physics rate that doesn't divide 1 s into whole nanoseconds.
- **Real-time rules:** `Model.AdvanceDoesNotAllocate` (including the outputs update) and `Model.RunsAreBitIdentical`.
- **The tests can fail:** six deliberate bugs were planted, and each was caught by the intended test:
  1. no geopotential conversion;
  2. pressure-altitude ΔT convention;
  3. R_M and R_N swapped;
  4. gravity rotated the wrong way;
  5. air evaluated at the origin instead of the CG;
  6. restore ignoring the remainder.

  For bug 4, the spec's level equilibrium test **passed**, and only the D-018 tilted case caught it, which confirms SPEC_REVIEW #17.
- **Builds:** all 118 tests pass with GCC 13.3 (Debug, Release, ASan+UBSan) and Clang 18.1 (Debug). clang-format is clean. clang-tidy is clean, and now also covers `src/core`, `src/env` and `src/rotor` (CI updated).

**Plots** (`docs/validation/m2/`):

![ISA vs USSA Table I](validation/m2/m2_isa_table.png)
![ISA profiles](validation/m2/m2_isa_profile.png)
![Equilibrium drift](validation/m2/m2_equilibrium.png)
![Collective step](validation/m2/m2_collective_step.png)

In the profile plot, the ±20 K density curves cross near 8 km. That's expected with a hydrostatic ΔT: a hotter column is less dense at the bottom but holds more pressure aloft.

---

## M3 — Blade-element rotor, rigid blades, uniform inflow (2026-10-11)

The rotor is `src/rotor/blade_element_rotor.*` (with `airfoil.*` and `controls.*`), a `DynamicComponent` with no states: rigid, massless blades with flapping held at zero, and uniform momentum inflow solved in every evaluation (D-024). The criteria are spec §9 M3 as amended by D-007 (layered hover oracle) and D-022 (reference-model comparison as a validation entry). The tests are in `tests/unit/test_blade_element_rotor.cpp` and `test_rotor_inputs.cpp`. The exact-equation reference is `tests/support/bet_reference.hpp`. To reproduce the plots, see `tools/plot_m3.py`. All test rotors and airfoils are synthetic fixtures (`tests/support/m3_fixtures.hpp`): R = 8 m, Ω = 27 rad/s, 4 blades, σ = 0.084, a linear airfoil (C_l = aα, C_d = C_d0) and a nonlinear one with Mach effects, stall and reverse flow. No UH-60A data is used yet (it is transcribed after M3).

| Acceptance criterion | Status | Evidence |
|---|---|---|
| Hover T, Q, P match BEMT within 2%, replaced by D-007: **layer 1**, production code vs an exact-equation reference | ✅ | `HoverPassMatchesExactReference` (fixed λ, uniform and equal-area spacing), `HoverWithSolvedInflowMatchesExactReference`, `TipLossMatchesExactReference`, `ForwardFlightMatchesExactReference` (μ ≈ 0.2, sideslip, climb, wind, body rates, cyclic, hub tilt and offset, both rotation directions), `ReverseFlowAndStallMatchExactReference`. The reference integrates the same equations by adaptive Gauss–Kronrod quadrature, with velocities derived separately in scalar rotating-frame form and Leishman Eq. 3.5 resolution. All six hub load components and λ_i converge at second order, and the Richardson extrapolation from N and 2N elements agrees to ≤ 1e-9 (achieved 7e-11 to 2e-15 at N = 1920–3840). With the nonlinear table (kinks, so no Richardson) the raw error falls as N⁻² to 1.9e-7 at N = 3840. |
| **Layer 2**, the hover closed form (Leishman 2nd ed. Eqs. 3.25, 3.31, 3.32) | ✅ | `HoverMatchesLeishmanClosedForm`: C_T within 1%, C_P within 1.5% and λ within 0.5%, for θ_0.75 = 4–12° with no twist and with −10° twist. The residual is the closed form's small-angle approximation. `ClosedFormResidualVanishesWithInflowAngle` checks the derived bound \|ε_CT\| < (3/2)λ² (with C_d0 = 0) as θ₀ goes from 16° to 1.5°. The discretization error at N = 400 is below 1e-5. |
| **Layer 3**, Knight & Hefner (1937) data (validation entry) | ✅ (entry) | 26 points digitized from Leishman Fig. 3.2 and verified by human review (`tests/validation/reference/knight_hefner_1937.csv`; DATA_GAPS.md). Eyas uses the same assumptions as Leishman's theory curves: untwisted blades, C_lα = 5.73/rad, uniform inflow, B = 1. For θ₀ ≥ 4° it over-predicts thrust by −1% to +16% (mostly +3% to +8%), which matches Leishman's own remark (p. 120) that the theory shows "a slight overprediction of the thrust" because it leaves out nonuniform inflow and tip loss. With an effective-radius tip loss B = 0.97 the difference is −9% to +7%. Below 4° the relative difference is dominated by the ±0.0001 reading uncertainty. No tolerance is applied: this is 1937 data read from a plot (D-007). |
| C_T against collective matches the reference model within 3% (validation entry per D-022) | ⏸ | Needs the UH-60A blade data (transcribed after M3), the license check (SPEC_REVIEW #12) and a reference-model run set up per D-022. |
| T, Q, P at constant collective are steady across azimuth in hover | ✅ | `HoverLoadsAreSteadyAcrossAzimuth`: N_b = 2, 3, 4 and 5, nonlinear airfoil, 97 azimuths. T, Q and P are constant to 1e-12 relative, and the in-plane hub forces and moments vanish to 1e-12. In the plot, the peak-to-peak spread is 0 (thrust) and 3e-16 (torque). |

**Also verified:**
- **Conventions:**
  - `ClockwiseRotorMirrorsCounterClockwise`: the clockwise rotor is the x–z mirror of the counter-clockwise one.
  - `CounterClockwiseRotorLoadSigns`: thrust up, positive torque, the torque reaction yaws the nose right, and in forward flight with no flapping the advancing side rolls the rotor left.
  - `ForwardFlightLoadsHaveBladePassagePeriod`: N_b/rev periodicity.
  - `HubFrameAndBodyTransform`: a forward shaft tilt γ puts z_h at (−sin γ, 0, cos γ). The body pitching moment matches T(x_h cos γ + z_h sin γ) by hand.
  - `AzimuthFromTime`.
- **Inflow solve** (D-024):
  - `InflowSolveConvergesAcrossConditions`: hover, climb, windmill-brake descent, negative thrust and forward flight, each converging in ≤ 10 passes to a root of Padfield Eq. 3.139 within 1e-12.
  - `InflowSolveIsBoundedInVortexRing`: in the vortex-ring region the result stays finite and within the pass limit, and converged results satisfy Eq. 3.139.
  - The descent sweep below converges everywhere in ≤ 14 passes. It also shows momentum theory's vortex-ring gap: the result jumps between root branches at about 3 v_h. Logged as SPEC_REVIEW #21, resolved in M8d (D-049; this sweep is the uncorrected rotor, `vortex_ring: none`).
- **Inputs:**
  - Airfoil tables must cover α = ±180°, and their columns are checked for dimension (D-021).
  - The linear control map handles offsets, gains and cross-mixing, from config.
  - `Config::path` resolves relative to the config file.
  - The rotor is built from YAML through `eyas::Model`, and config errors name the full key path.
- **Real-time rules:**
  - `AdvanceDoesNotAllocateAndConverges` and `Model.BladeElementRotorFromConfig` check for zero allocations.
  - `DerivativesArePureWithRotor`: bit-identical derivatives despite the cached table-search hints.
  - Convergence flags are counted by the executive (D-023).
- **Cost** (Release, GCC 13.3, WSL2; not the D-005 benchmark, which comes in M8):
  - One rotor evaluation (4 blades × 20 elements, forward flight) takes 11 µs, at 6 blade-element passes per inflow solve.
  - One physics step with the rotor takes about 60 µs, against a ≈ 490 µs worst-case budget per step at 60 Hz (D-005).
  - The atmosphere costs 18 ns per evaluation (about 0.1% of a step), so the deferred M2 review item (caching T₁/P₁ per ΔT) isn't worth doing.
- **The tests can fail:** 13 deliberate bugs were planted in the rotor, and the tests caught every one:
  1. induced-velocity sign;
  2. drag sign in the normal force;
  3. moment cross-product order;
  4. no α wrap;
  5. body transform using C_hb instead of C_bh;
  6. ω × r_hub omitted;
  7. twist looked up at r instead of r/R;
  8. tip loss ignored;
  9. Mach number from U_T only;
  10. body-rate sign in U_P;
  11. cyclic sin/cos swapped;
  12. wind sign;
  13. torque sign.

  The α wrap was first missed: it only matters with upflow in the reverse-flow region, so an upflow case was added. Blade numbering (ψ ± 2πk/N_b) can't be seen in total loads, since the set of azimuths is the same either way; it becomes testable with per-blade states in M4.
- **Builds:** all 155 unit tests and 3 API tests pass with GCC 13.3 (Debug, Release, ASan+UBSan) and Clang 18.1 (Debug). clang-format and clang-tidy are clean.

**Plots** (`docs/validation/m3/`):

![Refinement vs exact reference](validation/m3/m3_refinement.png)
![Hover closed form](validation/m3/m3_closed_form.png)
![Knight & Hefner 1937](validation/m3/m3_knight_hefner.png)
![Loads against azimuth](validation/m3/m3_azimuth.png)
![Inflow vs vertical speed](validation/m3/m3_inflow.png)

In the refinement plot, the Richardson-extrapolated error of the smooth cases bottoms out at round-off (≈ 1e-15) by N ≈ 2000. The equal-area and tip-loss cases approach second order more slowly, because the equal-area spacing has a larger h⁴ term. The nonlinear-table case has no smooth expansion, so its Richardson curve is erratic, but its raw error still falls as N⁻².

---

## M4 — Individual blade flap dynamics (2026-10-11)

`BladeElementRotor` gains an optional flap hinge (`blade.flap` in config). With it, each blade is a rigid body on an offset hinge, with an optional spring, damper and δ₃. It has states [β_k, β̇_k], and its loads include the blade weight and inertia, split into a = 0 values plus G and H for the coupled solve. Without a flap section the rotor is the M3 rotor, unchanged. Multiblade coordinates come from `src/rotor/multiblade.hpp` (Padfield 3rd ed. Eqs. 3.42–3.45).

The criteria are spec §9 M4, replaced by the D-026 layers (SPEC_REVIEW #22):
- **Padfield's closed forms are for a centre-spring rotor**, so they're used only for e = 0 fixtures; Johnson §6.15 covers the offset hinge.
- The spec's ν_β formula assumes a uniform blade.
- The spec's coning and phase criteria named no formula.

Tests: `tests/unit/test_flapping_rotor.cpp` (layers 1–2) and `test_flap_anchors.cpp` (layer 3). The exact reference is `tests/support/flap_reference.hpp`, the closed forms are in `flap_closed_forms.hpp` and the fixtures in `m4_fixtures.hpp`. To reproduce the plots, see `tools/plot_m4.py`. All rotors are synthetic fixtures with the M3 geometry; the anchor rotor has γ ≈ 8.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| **Layer 1:** production flap equation and hub loads vs an exact-equation reference; G and H vs the full equation | ✅ | `MatchesExactReference`. The reference models each blade as point masses whose NED positions are explicit functions of time, differentiated exactly by Taylor-jet automatic differentiation. Blade directions come from ∂P/∂ψ, ∂P/∂β and ∂P/∂ρ, the flap equation from d'Alembert's principle, and the airframe load is Σ(F − mP̈). Three rotors (both rotation directions; offset or centre hinge; spring, damper, δ₃ = −15…20°; tilted, offset hubs; wind), random attitude, rates, flap states and controls, nonlinear airfoil. For a = 0, each unit vector and a random a: β̈ = ẋ₀ + G·a agrees to < 1e-6 rad/s², and load₀ + H·a to < 1e-7 relative. |
| **Layer 2:** consistent coupling through the production executive | ✅ | `CoupledSystemConservesMomentum`: an airframe (900 kg) tumbling at ≈ 1 rad/s with four flapping blades on springs and dampers, in vacuum, for 1 s. The angular momentum about the system CG and the linear momentum less weight × t are conserved to 7e-9 and 1e-10 at 1 kHz. **The drift falls by 16.0× per halving of the step from 4 ms to 0.25 ms**, which is pure RK4 truncation; a G/H inconsistency would give an O(1) drift that doesn't converge. |
| The non-rotating and rotating flap natural frequencies match ν_β (D-009) | ✅ | `FreeFlapFrequencyMatchesNu`, free response in vacuum, hub at rest, measured from 30 cycles: within 1e-4 of the spec's uniform-blade formula (e = 0.075R), of Johnson Eq. 6.261 for a non-uniform blade with a spring, and of √(K_β/I_β) non-rotating. |
| Hub-motion terms (spec §6.4) | ✅ | `HubMotionForcingMatchesJohnsonAndPadfield`, e = 0 and e = 0.05R: ∂β̈/∂(ṗ, q̇, ẇ) from G and ∂β̈/∂(p, q) by central differences match Padfield Eq. 3.20 (azimuth dependence, Coriolis 2Ω) with Johnson Eq. 16.240's weights (I_β + eS_β)/I_β and S_β/I_β to 1e-7; the u̇, v̇ and ṙ terms are zero. |
| Coning against thrust matches the analytic result | ✅ | `HoverConingMatchesJohnson`: Johnson Eq. 6.277 (μ = 0) plus the weight moment (Eq. 6.208, −0.15° here). The residual is ≤ 0.53% for θ₀ = 4–18° (1% gate). |
| a₁, b₁ against μ match Padfield's analytic expressions at low μ | ✅ | `CentreSpringFlappingMatchesPadfield` (Padfield Eqs. 3.57–3.58, 3.64; λ_β² = 1.12) and `OffsetHingeFlappingMatchesJohnson` (Johnson Eqs. 6.277–6.279). Each closed form is evaluated at Eyas's own inflow, against a fitted periodic solution. Worst component, as a share of the disc tilt: centre ≤ 0.14% and offset ≤ 0.99% up to μ = 0.1 (1.5% gate); 1.7% and 2.0% at μ = 0.2 (3% gate). The residual then grows as expected for expansions that drop the 2/rev flapping: 10.6% and 3.1% at μ = 0.3, reported only (Padfield p. 85 quotes < 10% below μ ≈ 0.35). |
| Flap response to a cyclic step has the expected phase lag | ✅ | `CyclicStepPhaseLag`: a θ₁s step in hover, steady response against 90° − atan(N*/d₁) (Johnson Eqs. 6.278–6.279; Padfield Eq. 3.40 for e = 0). Offset hinge 84.16° vs 84.05°; centre spring 80.23° vs 80.17° (0.3° gate). Amplitude within 0.33% (1% gate). |
| δ₃ coupling (spec M4 scope) | ✅ | `Delta3AddsAerodynamicSpring`: the change in ∂M_aero/∂β matches −I_βΩ²(γ/8) tan δ₃ (Johnson Eqs. 6.303–6.304) within 1% for δ₃ = 15°, 30° and −20°. |
| Coning and a₁, b₁ vs the reference model across the speed range | ⏸ | Validation entries (D-022 pattern). They need the license check (SPEC_REVIEW #12) and a reference-model run of a defined UH-60A isolated-rotor case. |

**Also verified:**
- **Hub force from disc tilt:** `HoverHubForceFollowsDiscTilt`. The mean in-plane hub force, including blade inertia, is C_T·β₁c and −C_T·β₁s (Padfield Eqs. 3.100–3.101) within 3% of the tilt.
- **M3 behaviour kept:** `ZeroFlapReproducesM3Aerodynamics`. All M3 tests pass unchanged.
- **Config:** `Model.FlappingRotorFromConfig` reads GenHel units (`lbm`, `slug*ft`, `slug*ft^2`, and the new `ft*lbf/rad` and `ft*lbf*s/rad`). The config errors name the key: wrong dimension, unknown key, and I_β < S_β²/m_b (Cauchy–Schwarz). `RejectsInvalidHinge` covers the constructor checks.
- **Real-time rules:** `DerivativesArePureAndDontAllocate`, and zero allocations through `Model::advance()` with a flapping rotor.
- **Cost** (Release, GCC 13.3, WSL2): one rotor evaluation in forward flight takes 10.1 µs with flapping and 9.7 µs without.
- **The tests can fail:** 12 deliberate bugs were planted, and each was caught by at least one test:
  1. Coriolis term dropped;
  2. G zeroed;
  3. gravity moment sign;
  4. U_P on the shaft axis instead of the blade normal;
  5. δ₃ sign;
  6. H missing the β̈ coupling;
  7. blade moment missing its span term;
  8. hinge-offset velocity dropped;
  9. induced velocity not projected on the flapped blade;
  10. flap moment arm r instead of ρ;
  11. blade weight missing from the hub load;
  12. the 2Ωβ̇ sin β term dropped.

  Two of them (G zeroed, induced velocity not projected) are caught only by the layer 1 reference. That is why it is a gate.
- **Builds:** all 184 tests pass with GCC 13.3 (Debug, Release, ASan+UBSan) and Clang 18.1 (Debug). clang-format and clang-tidy are clean.

**Plots** (`docs/validation/m4/`):

![Flapping against advance ratio](validation/m4/m4_flapping_mu.png)
![Coning against thrust](validation/m4/m4_coning.png)
![Cyclic step](validation/m4/m4_cyclic_step.png)
![Momentum conservation](validation/m4/m4_conservation.png)

In the cyclic-step plot, the disc settles to the new tilt in about 0.5 s (≈ 2 rev), which matches the flap time constant 16/(γΩ) ≈ 0.07 s per e-fold (Padfield Eq. 3.41). The phase lag converges to Johnson's steady value.

## M5 — Pitt–Peters dynamic inflow (2026-10-12)

`BladeElementRotor` gains `inflow: pitt_peters` (the `inflow:` key is now required, `uniform` or `pitt_peters`). It adds three states [λ₀, λ₁s, λ₁c] after the flap states. The induced velocity varies linearly with radius over the disc, λ₀ + (r/R)(λ₁s sin ψ + λ₁c cos ψ). The states are driven by C_T, C_1s and C_1c from the same blade-element pass, so there is one pass per evaluation and no iteration. The equations are Pitt & Peters 1980 Table 4 in total-state form, with V = √(μ²+λ²) on every column and the wake-skew terms applied in wind axes (`src/rotor/pitt_peters.{hpp,cpp}`; D-027).

The criteria are spec §9 M5, replaced by the D-027 layers (SPEC_REVIEW #10, #23):
- **No value was given for "the published time constant"**, and "the" time constant is ambiguous.
- **The thrust overshoot has a closed form**, so it doesn't need the reference model.
- **The 4× apparent-mass conflict between the sources is resolved in favour of 16/45π.**

Tests: `tests/unit/test_pitt_peters.cpp`. The independent transcriptions of both sources are in `tests/support/pitt_peters_reference.hpp`. To reproduce the plots, see `tools/plot_m5.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| **Layer 1:** the equations against independent transcriptions of both sources | ✅ | `MatchesPittPeters1980AndJohnson2013`. Each source is transcribed in its own notation, state order and load signs: the 1980 Table 4 L and M with (λ0, λs, λc), (C_T, C_L, C_M) and sin α; and Johnson Eq. 11.143 with (λ0, λc, λs), (C_T, −C_My, C_Mx), cos χ and M from p. 437. The wind-axis rotation is done independently, with complex numbers. 204 cases (hover, V = 0, edgewise, windmill, then random μ, μ_z, sideslip and loads) agree to 1e-12. This tests the sign reconciliation of SPEC_REVIEW #10. Also `WakeSkewMatchesJohnson` (X = tan(χ/2), Eq. 11.132) and `RotatesWithTheWind`. |
| **Layer 1:** the production rotor against the M3/M4 exact references with a linear inflow | ✅ | `LinearInflowPassMatchesExactReference`: C_T, C_1s, C_1c and the hub moments, both rotation directions, Richardson-extrapolated, agree to 1e-9 relative. `FlappingRotorMatchesExactReference`: flapping blades, offset hinge, spring, damper, δ₃, tilted and offset hub, random attitude, rates, flap and inflow states. β̈ agrees to 1e-6 rad/s², the airframe load (with G, H) to 1e-7, and the inflow rates (from the reference's own load integrals) to 1e-9; G = 0 on the inflow rows. `InflowRatesUseLoadsFromTheSamePass`: one pass, `converged` always true. |
| In steady state, reduces to uniform momentum inflow in hover (spec) | ✅ | `SteadyHoverEqualsUniformMomentum`: integrated from λ = 0 (no singularity at V = 0) through the production rotor, λ₀ equals the D-024 uniform solve to 1e-9 and λ₁s = λ₁c = 0 to 1e-12, in hover and in a 5 m/s climb, for the linear and the nonlinear airfoil. |
| The inflow time constant in hover matches the published value (spec) | ✅ | `HoverInflowTimeConstants`: with the loading held, τ₀ = (128/75π)/(4λ₀Ω) (1980 Table 4 with eq. 23b; Padfield 3rd ed. Eqs. 5.76 and 5.80 with C₀ = 1), τ₁ = 2(16/45π)/(λ₀Ω) for each first-harmonic state, and no cross-coupling, all to 1e-6. |
| Coupled hover eigenvalues | ✅ | `CoupledHoverEigenvalues`: through the production rotor, rigid blades at fixed pitch, θ₀ = 8°, 12° and 16°. s₀ = −(Ω/M₁₁)(4λ₀ + σa/4) (Leishman 2nd ed. Eq. 3.25; Johnson Eq. 11.148) and s₁ = −(Ω/M₂₂)(λ₀/2 + σa/16) (Johnson Eq. 11.152) agree within 0.5% (1% gate; the residual is the closed forms' small-angle and tip-loss-free assumptions). |
| Step-collective thrust overshoot (spec) | ✅ | `StepCollectiveOvershoot`: a 0.25° step at θ₀ = 12°, hub fixed. The jump (σa/6)δθ₀ agrees within 0.25%, the final value C′ × jump (C′ = 1/(1 + σa/(16λ₀)), Johnson Eq. 11.149) within 1.1%, and the decay rate s₀ within 0.93% (2% gates). The overshoot ratio is 1.71. |
| Wake skew | ✅ | `WakeSkewGradient`: with thrust-only loading, λ₁c/λ₀ = (15π/32) tan(χ/2) (1980 eq. 22b) to 1e-10, and λ₀ = C_T/(2V). `DownwashSkewsDownwind`: through the rotor, the extra downwash sits downwind in forward and sideways flight, for both rotation directions (D-027 wind axes). |
| Stability over the skew range | ✅ | `GainInvertibleAndStableAcrossSkew`: L̂ is invertible and every root is stable for X ∈ [0, 1], with damping ratio ≥ 0.744. **The 1980 paper's "three real, stable roots for all α" doesn't hold for its own Table 4.** The (λ₀, λ₁c) pair is complex for α < 74.6° (SPEC_REVIEW #10 findings; the transcription was confirmed by human review). |
| **Layer 3:** experimental data (validation entry) | ✅ (entry) | Carpenter & Fridovich, NACA TN 3044 (1953), slow collective ramps; see below. |

**Layer 3: NACA TN 3044 collective ramps.**
- **Rotor:** rotor B of NACA TN 1698 on the Langley helicopter tower (3 blades, R = 19 ft, σ = 0.042, NACA 23015). The blade pitch was ramped from 0° to 12° in hover.
- **Eyas setup** (`tests/validation/m5_cases.cpp`): the production rotor with Pitt–Peters inflow, driven by the measured pitch ramp, with the hub fixed and starting from zero inflow, as in the test.
- **Assumptions** (DATA_GAPS.md): constant chord from σ, no root cutout, B = 0.97, a linear airfoil with the Report 824 section lift slope, massless blades, and Ω constant at the initial speed.
- **Spread:** the lift slope ranges over the 0.105–0.110/deg reading and the twist over 0° and −8° (the reports conflict).
- **Tolerance:** none. This is 1953 data read from plots, with an incomplete rotor description (D-007).

Measured against Eyas, for 0° twist and a = 0.1075/deg:

| Fig. 3 case | Peak / steady C_T, measured | Peak / steady C_T, Eyas | v/v_s = 0.9 at, measured | λ₀/λ₀,final = 0.9 at, Eyas |
|---|---|---|---|---|
| (b) ≈ 48°/s | 0.0059 / 0.0045 (1.31) | 0.00634 / 0.00517 (1.23) | 0.65 s | 0.38 s |
| (c) ≈ 20°/s | 0.0053 / 0.0045 (1.18) | 0.00557 / 0.00517 (1.08) | 0.76 s | 0.62 s |
| (d) ≈ 6°/s | 0.00455 / 0.0044 (1.03) | 0.00525 / 0.00517 (1.02) | 1.75 s | 1.83 s |

- **Shape:** Pitt–Peters reproduces the thrust overshoot and the inflow lag, which uniform inflow can't (the dashed lines in the plot: no overshoot, and the inflow follows the pitch). The overshoot grows with ramp rate, as in the test.
- **Steady thrust is 13–22% high** across the spread. Possible contributors: the assumed constant chord with no root cutout; a section lift slope from a 2-D test at a higher Reynolds number; and blade-element momentum theory's known over-prediction (the M3 Knight & Hefner entry shows +3–8% for untwisted blades; TN 3044's authors used an "adjusted" lift slope for this reason). This is a steady-thrust offset, not a dynamic-inflow error.
- **In the faster ramps the overshoot is smaller and the inflow builds up faster than measured.** In the slowest ramp they agree within about 5%. Three possible contributors, none established:
  1. The measured hub thrust includes blade flap inertia and blade bending, which Eyas leaves out here. TN 3044 puts the blade-inertia share at a few percent of thrust for the slower ramps, rising to 38% for the fastest.
  2. v was measured with paddles about 2 ft below the disc, in the wake, which lags the disc inflow. The smoke photographs show the flow building at the tips first and then moving inboard, which a uniform λ₀ can't represent.
  3. The Pitt–Peters apparent mass 128/75π is about 0.64 of the 8/3π used in TN 3044's own calculation, so the Pitt–Peters inflow responds faster.
- **Decision for later:** revisit with the flap DOF enabled if the blade inertia and CG ever turn up (DATA_GAPS.md). If M7/M8 hover heave response is too fast against flight data, the apparent mass is the first thing to examine.

**Also verified:**
- **M3 and M4 behaviour kept:** with `inflow: uniform` the rotor is unchanged, and all M3 and M4 tests pass unchanged.
- **Config:** `Model.PittPetersRotorFromConfig` (3 inflow states after the 8 flap states, zero allocations through `advance()`). A missing or unknown `inflow:` value is rejected with the key path.
- **Real-time rules:** `DerivativesArePureAndDontAllocate`. There's one blade-element pass per evaluation, against 6–30 for the uniform-inflow solve.
- **The tests can fail:** 10 deliberate bugs were planted, and each was caught by at least one test:
  1. skew coupling sign, in L̂;
  2. skew coupling sign, in the rates;
  3. wind-axis sign;
  4. C_1s weighted by cos ψ;
  5. inflow gradient not projected on the flapped blade;
  6. moment apparent mass 64/45π;
  7. thrust apparent mass 8/3π;
  8. V = |λ| instead of √(μ²+λ²);
  9. inflow gradient dropped;
  10. hinge-offset term of the gradient dropped.

  Three of them (5, 8 and 10) are caught mainly by the layer 1 exact reference, so it stays a gate. Bug 1 is caught only by the stability test, because the rate function inverts L̂ in closed form rather than calling `pitt_peters_gain()`. The layer 1 transcriptions cover the rates.
- **Builds:** all 199 tests pass with GCC 13.3 (Debug, Release, ASan+UBSan) and Clang 18.1 (Debug). clang-format and clang-tidy are clean.

**Plots** (`docs/validation/m5/`):

![NACA TN 3044 collective ramps](validation/m5/m5_naca_ramps.png)
![Step collective against the closed form](validation/m5/m5_step_anchor.png)

## M6a — Component data path and tail rotor (2026-10-12)

M6 is staged in four steps (D-028). M6a adds:
- **The data path between components (D-003).** A component declares the outputs it produces and consumes, by name (`<instance>.<field>`). At init the executive gives each name a slot in one pre-allocated store and sorts the components so producers run before consumers, with ties kept in config order. A missing producer, a duplicate or a cycle is an init error naming the components. Debug builds fill the store with NaN at the start of every evaluation and assert on a read before the write (`src/core/coupling.{hpp,cpp}`, `src/sim/executive.cpp`). A named `blade_element_rotor` produces λ₀, μ_x, μ_z, β_1c and ΩR.
- **The GenHel tail rotor** (`type: bailey_rotor`, `src/rotor/bailey_rotor.{hpp,cpp}`): the closed-form Bailey thrust of CR-166309 §5.4.2 with tip loss, twist, δ₃ through the coning rate, the pitch bias, fin blockage and drag. Thrust only, as in GenHel. GenHel's previous-pass inflow becomes one Pitt–Peters inflow state, and its previous-pass δ₃ an exact solve in the same evaluation (D-028).
- **Main-rotor wash at an airframe point** (`src/rotor/rotor_wash.{hpp,cpp}`): GenHel's E_KX, E_KZ maps against wake skew χ and flapping a₁, from the rotor's coupling outputs. The tail rotor uses it now; the stabilator and fin will use it in M6b. GenHel's definitions are χ = atan(μ_x/|λ|) + a₁ (p. 5.1-37) and a₁ = −(2/b)Σβ cos ψ = −β_1c (p. 5.1-18).

The criteria are D-028's layers for the tail rotor. The spec's "pedal balances torque" check is in M6d, with the whole aircraft.

Tests: `tests/unit/test_coupling.cpp`, `tests/unit/test_bailey_rotor.cpp`. The GenHel transcription is in `tests/support/genhel_tail_rotor.hpp`. To reproduce the plots, see `tools/plot_m6.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| **Data path:** producers before consumers, values from the same evaluation | ✅ | `Coupling.ProducerRunsFirstWhateverTheConfigOrder`, `ChainsAcrossComponents`: consumers listed before their producers are evaluated after them; ties keep config order. `InitErrorsNameTheProblem`: missing producer, duplicate, cycle. `UndeclaredReadAssertsInDebug`: a read of an undeclared output dies in debug builds. `DerivativesStayPure`, `AdvanceDoesNotAllocate`. |
| **Layer 1:** the tail rotor against GenHel §5.4.2 | ✅ | `MatchesGenHelSteadyState`: GenHel's equations transcribed in GenHel's notation and units (inches, lb, degrees, buttlines positive to port, wind-tunnel-style velocities), with the previous-pass inflow and δ₃ iterated to their fixed point. 300 random cases (forward, rearward and sideways flight, body rates, climb and descent, blockage active and not, interference velocities): with its inflow state at GenHel's converged downwash, production's forces, moments and thrust agree to 1e-10 relative, the blade pitch after δ₃ to 1e-12 rad, and dλ_i/dt = 0 to 1e-10. |
| Sign and arm conventions | ✅ | `HoverThrustDirectionsMatchTheAircraft`: thrust to starboard and 20° up, nose-left yaw moment, hub 14 in to starboard. This confirms the buttline reading in DATA_GAPS.md ("B_LTR sign"). |
| **Layer 2:** Bailey's coefficients are the small-angle thrust integrals | ✅ | `CoefficientsAreSmallAngleIntegrals`: t₃₁ = E[∫|U_T|], t₃₂ = E[∫U_T²], t₃₃ = E[∫rU_T²] over r ≤ B by independent quadrature, to 2e-6, for B = 1 and 0.92 and μ ≤ 0.3. So GenHel keeps reverse flow on the inflow term only, which is the μ²λ term of Johnson (2013) Eq. 6.219. **GenHel's placement of B on the μ² terms is the exact one for lift carried to B·R; Johnson's B³μ², B⁴μ² placement is the approximation.** |
| **Layer 2:** the blade-element rotor against the same integrals with full reverse flow | ✅ | `BladeElementMatchesSmallAngleIntegrals`: the production M3 rotor (exact angles, 4000 elements, a thin-plate airfoil whose lift reverses with the wind) against the integrals with reverse flow on every term. The remaining difference is the small-angle error, measured at 10–43% of the gate G λ²(θ + λ) over B ∈ {1, 0.92}, twist 0° and −18°, μ = 0–0.3 and λ_i = 0.02 and 0.05. |
| **Layer 2:** hover inflow eigenvalue | ✅ | `HoverInflowEigenvalue`: s = −(Ω/M₀)(4λ_i + G t₃₁/(1 + G t₃₂ k)), with k the δ₃ coupling, to 1e-6, with and without δ₃. At B = 1 and δ₃ = 0 this is the D-027 coupled hover root. |
| Wake skew and wash velocities against GenHel | ✅ | `RotorWash.GenHelWakeSkewAndWashVelocity`: χ and a₁ in degrees as GenHel writes them, 200 random rotor states, to 1e-11; wash components and signs on a synthetic map. |
| End to end through the executive | ✅ | `ConsumesMainRotorWashThroughTheExecutive`: a Pitt–Peters flapping main rotor and the tail rotor, tail listed first. The tail rotor's inflow rate equals one computed from the main rotor's states independently of the store, and `advance()` makes no allocations. |
| Config | ✅ | `ReadsConfig`: units, including the new `ft^2` and `deg/lbf`; δ₃ without a coning rate is an error, not a silent zero (D-021). |

**How close is GenHel's Bailey thrust to an exact blade-element integral?** This doesn't gate anything: Eyas reproduces GenHel's model on purpose, and these are GenHel's own approximations. UH-60A tail rotor, θ₀ = 20° at the root, λ_i = 0.05, against the production blade-element rotor with the thin-plate airfoil:
- **In hover** Bailey is low by the small-angle error: −0.12% at λ_i = 0.02, −1.15% at 0.05, −2.2% at 0.06 (the relative figure grows faster as C_T falls).
- **In forward flight** Bailey is high, because it drops the reverse-flow loss on the pitch terms (4μ³/(9π) θ₀ at B = 1): +1.6% at μ = 0.2, +6.0% at μ = 0.3 and +8.8% at μ = 0.35. **The UH-60A tail rotor reaches μ ≈ 0.37 at 150 kt**, so GenHel's tail-rotor thrust there is about 9% higher than an exact blade-element integral would give. If high-speed pedal trim disagrees with flight data in M7, this is a known contributor. The blade-element tail rotor of spec §6.6 ("later", D-008) removes it.

**Pedal step: why the inflow lag is a state.** A 16° → 18° step in commanded pitch, hover, at sea level. GenHel's previous-pass scheme (inflow and δ₃ both from the last pass) and Eyas's state settle to the same thrust, 1034.3 lbf. On the way there:
- **GenHel oscillates from pass to pass**: 1194, 948, 1087, 1005, 1052 lbf, and so on. The sequence is the same per pass whatever the frame length, so at 1 kHz it dies out within about 7 ms, but at 60 Hz the same ±12% oscillation lasts more than 0.1 s.
- **Eyas jumps to 1201 lbf** (the blade pitch acts at once) and decays smoothly to the steady value with the Pitt–Peters time constant, at any frame rate.

This is the effect D-028 removes. GenHel's transient is a property of its integration scheme, not of the aircraft.

**Also verified:**
- **Earlier behaviour kept:** all M0–M5 tests pass unchanged. An unnamed rotor produces no outputs, and components with no declarations keep their config order.
- **The tests can fail:** 9 deliberate bugs were planted, and each was caught:
  1. Johnson's B² on the μ²λ term;
  2. δ₃ through the unblocked thrust;
  3. mass flow without μ;
  4. apparent mass 8/3π;
  5. the sign of the body-rate velocity at the hub;
  6. the sign of a₁;
  7. the sign of the downwash at the point;
  8. evaluating in config order instead of the sorted order (debug: the read-before-write assert; release: 4 tests fail);
  9. an unsigned μ_x.
- **Builds:** all tests pass with GCC 13.3 (Debug, Release, ASan+UBSan on the M6a tests) and Clang 18.1 (Debug). clang-format and clang-tidy are clean.

**Plots** (`docs/validation/m6/`):

![Bailey thrust against the blade-element rotor](validation/m6/m6a_bailey_anchor.png)
![Tail-rotor pedal step: Eyas against GenHel's previous-pass scheme](validation/m6/m6a_pedal_step.png)

## M6b — Airframe aerodynamics (2026-10-13)

M6b adds the fuselage, stabilator and fin (`type: airframe_aero`, `src/airframe/airframe_aero.{hpp,cpp}`): GenHel CR-166309 §5.2.2 and §5.3.2 with the UH-60A tables (`data/uh60a/airframe/`, built by `tools/build_uh60a_airframe.py` from the transcriptions accepted on 2026-10-13). The delayed fuselage downwash and sidewash at the tail are two lag states (D-028). The wind-to-body rotation follows the flow's actual direction, so drag opposes the flow in rearward flight too (D-030). The tunnel angles α_WF and ψ_WF go to the coupling store for the tail rotor.

The tail rotor now takes the airframe's wake too (GenHel p. 5.4-5): the fuselage's dynamic-pressure factor on its body velocity, and the delayed sidewash and downwash as two more lag states (`fuselage_wake: {airframe: <name>}` in a `bailey_rotor` config). GenHel reads the fin's and stabilator's maps there, so the airframe looks them up once and publishes them through the coupling store (`<name>.fin_q_factor`, `.sidewash`, `.downwash`); the tail rotor has no copy of the tables.

The stabilator follows GenHel's schedule (Fig. 5.1.7 on p. 5.5-18) in its steady state: the airspeed lag settled, no pitch-rate or lateral-acceleration feedback, the servo at unit gain, within the −8°…39° travel (D-028). The schedule is a table in airspeed and collective (`data/uh60a/airframe/stabilator_schedule.csv`) whose grid makes bilinear interpolation exact. GenHel doesn't give the collective input's units; percent of travel is assumed (DATA_GAPS.md). A fixed incidence (GenHel's STBSET) remains available instead.

The two reports D-028 promises are below the table: the lag against an exact delay, and the sensitivity to a lag on a₁.

Tests: `tests/unit/test_airframe_aero.cpp`, `tests/unit/test_tail_rotor_wake.cpp`; the GenHel transcriptions are in `tests/support/genhel_airframe.hpp` and `genhel_tail_rotor.hpp` (now with K_QTR).

| Acceptance criterion | Status | Evidence |
|---|---|---|
| **Layer 1:** fuselage, stabilator and fin against GenHel | ✅ | `MatchesGenHelSteadyState`: GenHel's equations in GenHel's notation and units, reading the raw transcriptions with GenHel's own low/high map lookups. 400 random cases (geometry, density, flight state up to ±45° sideslip, body rates, main-rotor downwash, wake skew and flapping), lag states at equilibrium: every panel's forces and moments to 1e-9 relative, tunnel and panel angles to 1e-12 rad. `MatchesGenHelWithDelayedWake`: 200 cases with the lag states away from equilibrium. Since production reads the merged CSVs, this also checks the merge. |
| Lag states | ✅ | `LagStateRates`: dε_d/dt = (\|u\|/l)(ε − ε_d) to 1e-12, rearward flight included; `derivatives()` reports the same rates and loads as `evaluate()`. |
| **Layer 2:** symmetry | ✅ | `FuselageMirrorSymmetry`: mirror-image flight gives mirror-image loads when the two tables GenHel prints asymmetric are replaced by symmetric stand-ins. `KnownTableAsymmetries`: those asymmetries are in the data (Table 2.5.11 yaw is odd only inside ±30°; Table 2.5.7 lift increment is not even). D-030 corrects D-028's expected parities. |
| **D-030:** drag never adds energy | ✅ | `DragOpposesTheFlow`: with only the drag tables nonzero, the force on the fuselage and each panel opposes the local flow and is parallel to it, over 500 random directions (over 200 rearward). GenHel's own axes fail this rearward: the same test shows its fuselage drag pointing aft while moving aft. `ContinuousThroughZeroForwardSpeed`: loads continuous as V_X changes sign with V_Z ≠ 0. `RearwardFuselageLoadsInLevelFlow`: rearward in level flow the fuselage loads are diag(−1, 1, −1) times the forward ones, forces and moments (the lift, roll and yaw reverse with the drag; D-030). |
| **D-031:** sideward fade (SPEC_REVIEW #27) | ✅ | `SidewardFadeMatchesReference`: 300 random directions within 24° of the sideward line, the reference built from GenHel's equations at angle of attack s·α with the fade applied as D-031 writes it: fuselage loads to 1e-9 relative, bit-identical outside the fade. `ContinuousThroughPureSidewardFlow`: on the line the loads, α_WF and the stabilator's angle and dynamic pressure no longer depend on the direction the flow comes from (the unfaded model spreads by more than 100 N·m). c = 20° from the sweep below. |
| **Ruling A:** fin downwash sign | ✅ | `MainRotorDownwashSameAtStabilatorFinAndTailRotor`: in hover the main-rotor downwash at the stabilator and fin is equal, downward, and equal to what the tail rotor adds (M6a). |
| Coupling, purity, real time | ✅ | `WritesTunnelAnglesToTheStore`; `ThroughTheExecutive`: with a Pitt–Peters flapping main rotor listed after it, the airframe is evaluated second, derivatives are repeatable and `advance()` makes no allocations. |
| Config | ✅ | `ReadsConfig`: the UH-60A CSVs through the config reader give the same loads as the fixture; no placeholders. |
| **Stabilator schedule:** against GenHel's block diagram | ✅ | `StabilatorSchedule.MatchesGenHel`: GenHel's Fig. 5.1.7 evaluated directly from the transcription (steady state), 2000 random cases from 40 kt rearward to 200 kt and collective past both stops: incidence to 1e-12 rad, so the table is exact. `HoverAndForwardFlight`: 39° (the travel limit) in hover and up to 30 kt, 5.214° at 80 kt and 50 % collective (by hand), saturation past 147 kt; the scheduled incidence gives the same loads as the same fixed incidence. `ReadsConfig`: incidence and schedule together is an error. Three planted bugs caught (no travel limit; airspeed magnitude instead of u; the table built without the 70 % breakpoint). |
| **Tail rotor in the airframe's wake:** against GenHel | ✅ | `TailRotorWake.AirframePublishesGenHelsTailRotorWake`: what the airframe publishes is GenHel's K_QTR, SIG_TR and EPS_TR (Tables 3.5.7, 3.5.8, 3.5.4 on PSIWF and ALFWF, GenHel's own map lookups), 300 random cases forward and rearward, to 1e-14. `TailRotorWake.MatchesGenHel`: the M6a transcription with those values and the wake velocities, 300 random cases, forward and rearward, half at the steady state and half with delayed wake: loads and thrust to 1e-10 relative, and the lag rates. `NoWakeIsM6a`: with a neutral wake it is the M6a rotor. |
| Tail rotor, end to end | ✅ | `TailRotorWake.ThroughTheExecutive`: tail rotor, airframe and main rotor listed in that order are evaluated main, airframe, tail; the tail rotor's three state rates equal ones computed from the other two components independently of the store; repeatable; no allocation. `ReadsConfig`. |

**The wake lag against the exact delay (D-028 check).** GenHel delays the fuselage downwash at the stabilator by l/u (l = 28.7 ft from the CG); Eyas uses a first-order lag with the same time constant so that the derivative function stays pure and the M7 Jacobians keep the effect. The check runs the production airframe through a sinusoidal heave (±1.1° angle of attack) from 20 to 150 kt and 0.5 to 10 rad/s, twice: once with its lag state integrated (RK4) and once with the exact delayed downwash fed in as the state. UH-60A tables, stations and stabilator schedule, collective 50 %, sea level, no main-rotor wash.
- *The downwash itself* departs from the delay as expected: in the piloted band (1–5 rad/s) the lag's gain falls to 0.24 at 20 kt and 0.78 at 100 kt, with up to 154° of phase difference at 20 kt and 7° at 100 kt. The simulated ratio matches the analytic e^{jωτ}/(1 + jωτ) to 2.5e-7, so the production lag rate is exact.
- *The stabilator lift barely notices:* in the piloted band the lag changes the lift response by at most **1.1 % in gain and 0.6° in phase** (both at 20 kt), and by at most 0.1 % and 0.25° from 70 kt up. Up to 10 rad/s the worst is 1.4 % and 0.7°. The reason is that the fuselage downwash map is shallow (Table 3.5.4: 0.01–0.08° of downwash per degree of α_WF within ±30°), so the delayed term is a small part of the stabilator's angle of attack; where the lag is worst (low speed), that term is also scaled by the low speed.
- **Decision:** the lag stays; no exact delay buffer is needed (D-028's condition for one was a significant error in the piloted band). Not covered: the fin's and tail rotor's delayed sidewash (the same lag form, with Table 3.5.8's sidewash of at most 1.4°), and flight with main-rotor wash on the tail.

![Lag against exact delay](validation/m6/m6b_lag_vs_delay.png)

**Sensitivity to a lag on AA1FMR (D-028 "Data rulings").** GenHel's wash maps read AA1FMR, which it never defines; Eyas uses the flapping a₁ itself, with no lag (DATA_GAPS.md M6 item 7). Two numbers bound what an undefined filter could change:
- *How much the stabilator feels a₁.* At level-flight operating points (momentum inflow at the UH-60A's 16 638 lbf, a₁ = 2°), the stabilator lift per degree of a₁, through both wash maps, against its lift per degree of angle of attack: **0.6 to 1.3 from 20 to 60 kt**, 0.33 at 70 kt, and below 0.17 from 80 kt up. At low speed the main-rotor wake sweeps over the tail and a₁ moves it as much as angle of attack does; at cruise it hardly matters. (The curve is uneven at low speed because the wash maps are coarse: 10° steps in wake skew, 6° in a₁.)
- *What a lag would do to that path.* A first-order lag τ on a₁ multiplies the a₁ path by 1/(1 + jωτ): confirmed through the production path (wash maps, airframe and its lag states) to 1e-5 in gain and 2e-5° in phase for τ = 17.5 ms, 50 ms and 0.3 s. **The a₁ path stays within 5 % gain and 5° phase in the piloted band only for τ ≤ 17.5 ms** (the phase is the binding limit, at 5 rad/s).
- For scale, the main rotor's output filter T_FILMR = 0.3 s (p. 5.1-49, read with high confidence; it filters the rotor's forces and moments, p. 5.1-36, and nothing says it filters a₁): on the a₁ path it would mean 17° of lag at 1 rad/s and 56° at 5 rad/s.
- **Conclusion:** if AA1FMR carries any filter longer than about 20 ms, it changes the stabilator's response to flapping noticeably below 60 kt, and not above 80 kt. Item 7 stays open with these numbers. Low-speed pitch response against flight data (M7, M8) is where it would show.

![Stabilator sensitivity to a₁ and to a lag on it](validation/m6/m6b_a1_sensitivity.png)

**Width of the sideward fade (D-031, SPEC_REVIEW #27).** On the sideward line (V_X = V_Z = 0) the wind axes are singular, in GenHel too; D-031 fades them into fixed axes within c of the line. The criterion, set before the sweep: within the fade the total airframe loads may change with the flow direction at most about twice as steeply as away from the line (20°–45° from it). Grid of directions 0.05°–20° from the line, all around it, UH-60A tables and stations, no wash, lag states zero (the loads scale with q, so the ratio holds at any speed):

| c | 2° | 5° | 10° | 15° | 20° |
|---|---|---|---|---|---|
| steepest force change / away from the line | 15.0 | 6.2 | 3.2 | 2.2 | 1.7 |
| steepest moment change / away from the line | 19.8 | 7.9 | 3.9 | 2.6 | 1.9 |

So **c = 20°**, not the first guess of 10°. What it costs, as the largest change of each moment around the line at 20 kt (kN·m; the unfaded moments swing by 3.0–3.2 in roll, 4.1 in pitch and 4.3–4.4 in yaw around it):

| Distance from the line | 10° fade: roll, pitch, yaw | 20° fade: roll, pitch, yaw |
|---|---|---|
| 1° | 1.72, 3.27, 3.44 | 1.77, 3.33, 3.49 |
| 4° | 1.54, 3.16, 2.44 | 1.65, 2.96, 3.20 |
| 8° | 0.32, 0.69, 0.76 | 1.53, 3.11, 2.38 |
| 15° | 0 | 0.45, 0.93, 0.94 |

Close to the line any continuous model must change the loads by a large part of their swing, because the unfaded ones swing across that whole range there (dotted curves below); this one changes them by 55–80 % of it at 1°, more than an average would, since level flow from ahead sits near one end of the pitch and yaw swings. On the line the faded loads take the value of level flow from slightly ahead (θ = 0: roll 0.80, pitch −1.13, yaw 3.19 kN·m), the tunnel's sideslip-sweep condition, not the average around the line (0.44, 0.23, 2.25). GenHel's own unexecuted filter (p. 5.2-10) would act up to about 48° from the line at 20 kt.

*Where the fade acts in flight.* The main-rotor wash keeps the flow at the fuselage and tail away from the line. With GenHel's UH-60A wash maps (pure sideward flight, so no wake skew) the flow enters the 20° fade at:

| Rotor thrust | Fuselage | Stabilator and fin |
|---|---|---|
| hover (16 638 lbf, λ₀ = 0.054) | beyond 45 kt sideward | about 35 kt |
| half | about 35 kt | about 20 kt |
| quarter | about 15 kt | about 10 kt |
| none (rotor stopped) | at any sideward speed | at any sideward speed |

So it matters at low thrust and on the ground in a crosswind (M6d, M8). Near the line the panel loads are not credible with or without the fade: GenHel's panels use the full dynamic pressure, spanwise flow included, and its wash maps don't skew the wake sideways. The fade buys continuity there, not accuracy.

![Sideward fade](validation/m6/m6b_sideward_fade.png)

To reproduce: `./build/release/tests/eyas_m6_cases build/m6_cases && .venv/bin/python tools/plot_m6.py build/m6_cases docs/validation/m6`.

**The tests can fail:** 10 deliberate bugs were planted and each was caught: the fin wake sign; GenHel's α for the wind axes (D-030); ψ_WF = +β_WF; the moment arm reversed; K_Q missing on the wake term; K_Q missing on the body velocity; the incidence subtracted; u instead of \|u\| in the lag; no coupling write (debug: the read-before-write assert); the pitching-moment sign.

For the tail-rotor wake, 7 more planted bugs were each caught: the sidewash sign; K_QTR missing on the wake term; K_QTR missing on the body velocity; downwash looked up on ψ_WF; u instead of \|u\| in the lag; the wake state rates not written; the two wake states swapped.

After the code review the tail rotor reads the wake from the airframe; 4 more planted bugs were each caught: the stabilator's K_Q published as the fin's; the sidewash and downwash slots swapped; the undelayed sidewash in the tail rotor's velocity; the store not read.

For the sideward fade (D-031), 7 more: lookups unfaded; a linear fade; the drag faded with the other components; R(α', β) instead of R(0, β) in the fixed term; the weights swapped; cos c instead of sin c; the moments left unfaded.

**Builds:** all 240 tests pass with GCC 13.3 (Debug) and Clang 18.1 (Debug); the M6b tests pass under ASan+UBSan (2026-10-14, after the code-review fixes). clang-format and clang-tidy are clean, and the generated tables match their transcriptions.


## M6c — Landing gear (2026-10-14)

M6c adds GenHel's landing gear (`type: landing_gear`, `src/airframe/landing_gear.{hpp,cpp}`): CR-166309 §5.7, three gears each with a tire in series with an oleo strut and an unsprung mass, with the UH-60A data (`data/uh60a/gear/`, built by `tools/build_uh60a_gear.py` from the transcriptions of §5.7.5; the two doubtful cells verified by human review on 2026-10-14). The wheel brakes are pilot inputs, `Controls::brake_left` and `brake_right` in [0, 1] (an additive public API change; GenHel's set and off at the ends, linear between). GenHel's frame-to-frame memory becomes five states per gear (D-032): the stroke and its rate, the in-plane tire deflection (replacing the retained contact point and its slip reset) and the tire's lagged vertical deflection rate. Friction is GenHel's elasto-plastic law in continuous time, with a smooth blend from breakout to sliding friction; the strut's preload lock becomes a stiff top stop; the unsprung mass feels the body's acceleration and its own weight, so the gear has acceleration coupling (G, H ≠ 0) as the M4 blades do.

New units for the data: pressure (Pa, psi), stiffness (N/m, lbf/in, lbf/ft), linear damping (N·s/m, lbf·s/ft) and quadratic damping (N·s²/m², lbf·s²/ft²).

Tests: `tests/unit/test_landing_gear.cpp`; the GenHel transcription is `tests/support/genhel_gear.hpp`, which reads the raw transcriptions and interpolates them itself, so it also checks the CSV build and production's slicing at the tire pressure.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| **Layer 1:** gear against GenHel | ✅ | `MatchesGenHel`: GenHel §5.7.2 in its own notation and units, 4000 random states (attitude ±3°, any heading, CG position, ground height, strokes in both damper regimes, in-plane deflections, body velocity and rates), the retained contact points and filtered rates set to match production's states: tire forces in body axes, slip flags, strut forces, GenHel's δ̈ and the CG loads to 1e-9 relative, with K_SLIP = 1 (over 3000 contacts and 500 slipping axes compared). `MatchesGenHelSliding`: GenHel's K_SLIP = 0.85 with every contact point faster than v_s, so production's limit is the sliding one. D-032 item 7's terms (the wheel's weight and inertia) are set aside by name. Compared only inside GenHel's tables (beyond them GenHel's map clamps, production extends the last slope) and away from three touchdown edge cases listed in the transcription's header. |
| Acceleration coupling (D-032 item 7) | ✅ | `AccelerationCouplingIsDAlembert`: load0 + H a and s̈0 + G a against d'Alembert's principle for each wheel written independently (tire force + weight − m a_P), 200 random states and accelerations, to 1e-9. |
| **Layer 2:** static load split | ✅ | `StaticLoadSplit`: the full gear settled on level ground (with a test-only body dashpot that vanishes at rest), with and without a lift of half the weight: the three normal loads equal the statically determinate closed form (wheel weights at their axles) to 1e-6 of the weight; they sum to weight minus lift. |
| **Layer 2:** energy | ✅ | `DropConservesEnergy`: one gear under the CG, damping and friction off, dropped from 0.3 m carrying a third of the weight: kinetic (airframe and wheel) + gravity + lift + strut (isothermal, and the stop) + tire energy is conserved to 7.7e-4 of the exchanged energy at 1 kHz and 5.7e-6 at 8 kHz (plot below): the model is conservative, and RK4's error comes from the kinks. Pure heave: no rotation to 1e-12. |
| **Layer 2:** slope | ✅ | `SlopeHoldAndSlide`, every wheel braked (synthetic layout): at 0.8 atan μ (24.8°) every tire sticks for 10 s and the mean position moves less than 0.1 mm between two 5 s windows; at atan μ + 3° (34.0°) the slide accelerates at 1.3311 m/s² against g(sin α − K_SLIP μ cos α) = 1.3306. |
| Preload (D-032 item 6) | ✅ | `PreloadPassThrough`: with 75 % lift the mains carry about 1700 lbf, under their 3000 lbf preload: the stroke stays within 0.1 in of the stop and the strut passes the tire load less the wheel's weight, to 1e-6. Without lift the strut strokes and carries the load on the air spring. `DefaultTopStop`: 43 333 lbf/in for the main gear (10 × the 140 psi slope of Table 7.5.3), critically damped. |
| Lift-off (D-032 item 1) | ✅ | `LiftOffClearsDeflection`: off the ground δ' = −δ/G exactly. `UnloadedAxisRelaxes`: an in-plane axis with no stiffness (the tail tire's √F law with the vertical force clamped at zero in a rebound) relaxes the same way instead of following v_c. |
| Turning axes, vertical lag (D-032 review fixes) | ✅ | `StickingDeflectionIsEarthFixed`: while sticking, δ' − v_c = ψ̇ (δ_y, −δ_x) on level ground, with ψ̇ from Padfield 3rd ed. Eq. 3A.42, to 1e-12. `DampingLagRestsOffTheGround`: off the ground the lagged rate decays to zero instead of tracking the closing rate. `DampingLagSeesExactDeflectionRate`: the lag's input equals a central difference of ΔZ along the state's own motion (translating, turning, stroking, deflected) to 1e-7 m/s. |
| Brake inputs (D-032 notes) | ✅ | `BrakeInputs`: `Controls::brake_left` and `brake_right` act on their own main wheel only; k_B is N_BRK at 0, B_RK at 1 and linear between; inputs outside [0, 1] clamp and NaN counts as released; the tail wheel stays at N_BRK. The GenHel comparison draws a random brake input in [0, 1] per state (GenHel at the ends, the linear blend between). The validation data are bit-identical to the run with the earlier config switch. |
| Friction blend (D-032 item 2) | ✅ | `FrictionLimitBlend`: breakout at rest, K_SLIP of it from v_s up, the smoothstep between, monotone; the tail wheel always at N_BRK. |
| Wheel hop at 1 kHz | ✅ | `WheelHopRk4Accuracy`: the stroke history of a released main gear agrees with a 10 µs solution to 1e-3 of its swing over 0.2 s. |
| Purity, real time | ✅ | `PureAndAllocationFree`: repeat evaluations bit-identical, no allocation. `RestoreMidSlideIsBitIdentical`: snapshot and restore in a 35° slide reproduce the run bit for bit. |
| Config | ✅ | `ReadsConfig` (the UH-60A gear through the reader; the 140 psi slice of Table 7.5.3 is bilinear), `RejectsInvalidConfig`, `Units.GearUnits`. |

Planted bugs, each caught: the in-plane forces' moment arm at the axle instead of the contact point; the G row's sign; the wheel's weight left out of s̈; the slip relaxation's sign; no relaxation off the ground. After the review fixes, each fix reverted on its own and caught by its test: the turning term in the stick rate; the lag running off the ground; Ċ_bg left out of the lag's input; an unstiff axis sticking.

**Drop of the full gear.** 0.3 m (2.4 m/s at touchdown), the 7000 kg test airframe level, brakes set. The mains peak at 14.8 klbf and stroke 10.7 in; the tail strokes 16.4 in. The heave settles in about 1.5 s; the tail strut's pitch oscillation is still going at 3 s, because its damper (66.72 lbf·s²/ft², ∝ ṡ²) gives almost nothing at small stroke rates. That is GenHel's data, the slow settling D-032 flags for M6d.

![Drop](validation/m6/m6c_drop.png)

**Energy against the step.** Damping and friction off: the drift falls by 130× from 1 ms to 0.125 ms and then stops at about 6e-6, the test's own quadrature of the tire energy.

![Energy](validation/m6/m6c_energy.png)

**Slope hold and slide.** Every wheel braked; the hold settles in about a second and then oscillates by ±0.3 mm around a fixed point (the rocking on the tires); the slide follows the sliding-friction parabola.

![Slope](validation/m6/m6c_slope.png)

**Brakes off.** Rolling from 2 m/s on level ground, every tire's fore-aft limit is N_BRK μ N and sliding, so the speed falls at K_SLIP N_BRK μ g: 0.0501 m/s² measured over 5 s against 0.0500. (GenHel's rolling resistance is this friction model with N_BRK = 0.01; there is no separate rolling term.)

![Brakes off](validation/m6/m6c_roll.png)

**Sensitivity to v_s (D-032 item 2).** A braked stop from 4 ft/s on the full gear: the distance to rest changes by 0.4 % between v_s = 0.01 and 0.3 ft/s and by 4.6 % at 1 ft/s. v_s = 0.1 ft/s is in the flat part. No case chatters.

![v_s sweep](validation/m6/m6c_vs_sweep.png)

**Builds:** all 262 tests pass with GCC 13.3 (Debug) and Clang 18.1 (Debug); the M6c tests pass under ASan+UBSan. clang-format and clang-tidy are clean, and `tools/build_uh60a_gear.py --check` matches the CSVs to the transcriptions.

## M6d — The UH-60A aircraft (2026-10-14)

M6d puts the UH-60A together from data (D-034):
- the blade-element main rotor with flapping blades and Pitt–Peters inflow (M3–M5);
- the GenHel tail rotor (M6a), airframe aerodynamics (M6b) and landing gear (M6c).

The config is `data/uh60a/uh60a.yaml`, written by `tools/build_uh60a_config.py` from the GenHel transcriptions (new: main- and tail-rotor constants, airframe stations, mass properties and the control rigging of §5.5).
- **Mass:** GenHel's weight and CG are the whole aircraft's. The airframe excludes the blades and the unsprung masses, which carry their own weight. Its CG lies 1.03 in aft of and 3.61 in below GenHel's, and every position in the config is measured from it.
- **Controls:** the controls follow GenHel's mechanical mixing and swashplate phase. The stick and pedal travels were placeholders in M6d; since M7a they come from GenHel's cockpit figures (Figs. 6.3.1 and 6.4.1; D-035), with neutral longitudinal cyclic uncertain by 0.8 in. The hover controls below are also given in GenHel's inches and degrees.

This completes M6.

Tests: `tests/unit/test_uh60a_aircraft.cpp`. The harness in `tests/support/m6d_fixtures.hpp` evaluates the config's components one by one, with the body held, so the checks can read each component's loads. It stops on any unconverged component evaluation (D-023).

*Deferred to M7:* the harness repeats the executive's producer-first ordering and its `SimContext` setup, because the executive offers only the free-body derivative function. Trim (spec §6.8) needs the same held-body evaluation with per-component loads. Its first step adds that to the executive (with a DECISIONS entry), and the harness then wraps it instead of copying.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| Config | ✅ | `ConfigLoadsWithTheDeclaredPlaceholders`: the model loads through `Model::from_file`, and the placeholders are exactly the main rotor's two cyclic offsets (neutral longitudinal cyclic, D-035) and the strut top stops (D-032). `tools/build_uh60a_config.py --check` matches the YAML to the transcriptions (in CI). |
| Mass and geometry (D-034 items 2, 3) | ✅ | `MassSplitKeepsGenHelsWeightAndCg`: airframe + blades + unsprung masses = 16 638 lbm to 1e-5 lbm. Their CG is GenHel's to 1e-5 in, placed from the config's hub and GenHel's station differences, not from the generator's arithmetic. The shaft leans forward 3° with no lateral component. |
| **Layer 1:** control rigging | ✅ | `ControlMapsAreGenHelsRigging`: GenHel's Figs. 5.1.6(a)–(d) and the p. 5.5-20 gains, test-side in inches and degrees, through the travels of Figs. 6.3.1 and 6.4.1 and the p. 5.1-24 blade-pitch equation with the swashplate phase. Blade pitch matches at 12 azimuths and the tail-rotor command matches, both to 1e-7°, for 200 random inputs. The directions are those of `Controls`. |
| **Whole aircraft:** pedal balances torque (D-028 3b) | ✅ | `HoverPedalBalancesMainRotorTorque`: out of ground effect, body held level and at rest, cyclic centred, a root search sets collective and pedal for zero vertical force and zero yaw moment. The tail rotor's force is along its 70° cant (to 1e-12), and its moment is r × F with r from GenHel's stations (to 1e-9). That hand-worked yaw moment balances the other components' to 1 N·m. The main rotor turns the airframe nose right; the tail rotor pushes right and nose left. |
| **Whole aircraft:** sits stably on the ground (D-028 3a) | ✅ | `SitsStablyOnTheGround`: dropped level from 0.3 m with collective down, brakes set and the pedal that balances yaw at that collective. Averaged over 15–20 s, the ground normal loads equal weight minus aerodynamic lift within 0.2 % (2e-6 over 30–40 s in the validation run). It doesn't creep: under 5 mm and 1e-4 rad of heading between the 5–10 s and 15–20 s windows. |
| Gear sides and brakes | ✅ | `MainGearSidesMatchTheirBrakes`: `left_main` at y = −53.2 in (port) with the left brake, `right_main` at +53.2 in with the right brake, the tail wheel on the centreline and unbraked (GenHel p. 5.7-9; p. 5.7-10's missing minus, DATA_GAPS.md). `OneBrakeTurnsTowardsItsSide`: rolling at 3 m/s from tire contact, collective down, pedal balanced. Against the same run with no brakes (0.017° drift in 1.5 s), the left brake alone turns the nose 0.083° left and the right brake 0.090° right; the test asks for more than three times the drift. (With M6d's placeholder travels the aircraft sat 1.1° less nose-up, with less load on the tail wheel, and turned 0.26° and 0.51°.) The tire side forces take most of the brake's yaw moment, the non-castering tail wheel's above all (D-032 item 10). |
| Real time | ✅ | `RealTimeRules`: the whole aircraft through `Model` at 60 Hz host frames. The window from 0.2 s to 0.53 s takes in the first tire contact (asserted: no gear load at its start, load at its end). Over it: no allocation in `advance()`, a bit-identical repeat run, and an exact snapshot and restore. |

Planted bugs, each caught: the tail rotor on the port side (BL sign); the main gears' names and brakes swapped (the generator's former BLMGA sign, caught by both gear-side tests); the shaft tilted aft; a wrong blade mass; the tail-rotor cant flipped; the swashplate phase's sign; the sign of the pedal-to-longitudinal-cyclic mixing.

**Hover out of ground effect.** ISA sea level, 16 638 lbm, body held level and at rest, cyclic centred, from `eyas_m6d_cases`:

| Quantity | Value |
|---|---|
| Collective | 0.488 (XC 4.88 in), θ0 = 17.71° at the cuff |
| Pedal | −0.386 (XP 1.65 in), tail-rotor command 20.57°, pitch after δ₃ 25.49° |
| Main rotor | thrust 16 627 lbf, torque 31 840 ft·lbf, 1563 hp |
| Tail rotor | thrust 1058 lbf |
| Yaw moments | main rotor +31 114 ft·lbf, tail rotor −31 076, airframe −37.5; sum −2e-3 |

The thrust exceeds the weight by the airframe's download and the tail rotor's downward component. The main-rotor power gives a figure of merit of about 0.76 against ideal momentum power, in the usual range for a hover rotor. The tail rotor has no power model (DATA_GAPS.md). Since M7a (D-035) these use GenHel's travels. With M6d's placeholders the same check gave 16 753 lbf, 32 290 ft·lbf and 1585 hp: centred cyclic was a different disc tilt (centred cyclic is not a trim; M7 trims it).

There is no published UH-60A hover control position in the library yet. This is a validation entry for M7, not a gate. The comparison will also bring in the deferred elastic twist, rotor-force filter and profile-drag correction (D-034 item 5).

**Drop and settle.** Level from 0.3 m: collective down, brakes set, pedal −0.310 (the yaw balance at collective down). The rotor at flat pitch still lifts 570 lbf. After 30 s the gear carries 16 068 lbf, against weight minus lift of 16 068 lbf. The aircraft sits at 0.83° nose up and 0.08° right wing down; the main rotor's side force and the tail rotor's thrust both act on the airframe. The heading moves by 0.0046° after the first 5 s. (With M6d's placeholder travels, centred cyclic tilted the disc differently and the aircraft sat at 0.26° nose up and 1.10° right wing down.)

![Drop](validation/m6/m6d_drop.png)

In M6d, at 1.10° right wing down, the right main carried more than the left (35.0 against 26.7 kN at the end); now the aircraft sits nearly level. Before the gear-side fix the plot's two main-wheel curves were labelled the wrong way round. The run itself was bit-identical, since both mains have the same data and both brakes were set.

**The rocking D-032 warned of.** After the drop the aircraft rocks in pitch at 0.9–1.0 Hz. The motion halves about every 5 s (ζ ≈ 0.02): RMS pitch rate 0.66°/s over 5–10 s, 0.14 over 15–20 s, 0.019 over 35–40 s (M6d, with the placeholder travels: 0.7–0.8 Hz, 0.54, 0.14, 0.023). This is the pitch mode on the struts and tires: the strut dampers (∝ ṡ²) give almost nothing at small stroke rates, as GenHel's data have it. It is not D-032's 13.5 rad/s fore–aft mode, which stays quiet here.

The roll rate levels off at about 0.006°/s peak at 17.2 Hz (0.046°/s in M6d, with the placeholder travels' larger disc tilt). That is exactly 4Ω: the main rotor's blade-passage vibration, not an instability.

![Rocking](validation/m6/m6d_rocking.png)

**Cost.** The whole UH-60A takes 13 µs per 1 kHz physics step (Release, GCC 13.3, WSL2; 20 000 steps in hover). This is not the D-005 benchmark, which comes in M8.

**Builds:** all 270 tests pass with GCC 13.3 (Debug) and Clang 18.1 (Debug). The M6d, Model and Executive tests pass under ASan+UBSan. clang-format is clean; no `src/` file changed, so clang-tidy is as before.

## M7a — Trim (2026-10-15)

M7a adds the periodic trim (D-035): `Model::trim()` finds the four controls, pitch and roll, and every rotor, inflow and lag state at azimuth zero for steady straight flight. Over one rotor revolution the body is held at the flight condition while the other states are integrated through azimuth (RK4 at T/256). The trim is solved by Newton's method on all unknowns together, with a finite-difference Jacobian kept between iterations by Broyden's update and rebuilt only when progress stalls: the force and moment on the held body average to zero over the revolution, and every other state returns to its start value. The free body is then started on its own 4/rev vibration cycle (below).

Also in M7a:
- the control travels from GenHel's cockpit figures (D-035; M6d above updated);
- `Executive::held_derivatives`, the evaluation trim needs. The M6d test harness now wraps it instead of copying the executive (its validation output is unchanged);
- the config readers in `sim/build`, shared by `Model`, the tests and the tools.

Tests: `tests/unit/test_trim.cpp`; held evaluation in `tests/unit/test_executive.cpp`; data from `eyas_m7_cases`, plots from `tools/plot_m7.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| **T1** trim converges, 0–150 kt (spec) | ✅ | `Uh60aTrimsFromHoverTo150Knots`: level flight at ISA sea level, 16 638 lbm, every 10 kt, each from the previous trim, in 4–5 iterations; ±1000 ft/min at 60 and 100 kt, ±10° sideslip at 60 kt and 150 kt from a cold start all within the test's limit of 10, the cold start to the same trim as the sweep (1e-6). Net force below 1e-7 of the airframe weight, states periodic to 1e-8. Each body state has the condition's airspeed, sideslip and flight-path angle to 1e-9; a climb takes more collective than level flight and a descent less. |
| **T2** free flight from trim | ✅ | `Uh60aFliesOnFromTrim`: from trims every 30 kt, `Model::advance` at 1 kHz for four revolutions. The body's mean accelerations stay below 5e-5 m/s² and 2e-5 rad/s² up to 60 kt (measured ≤ 5.4e-6 and 1.3e-6), and below 2.5e-3 m/s² and 1e-3 rad/s² from 90 to 150 kt (measured ≤ 1.2e-3 and 2.8e-4). See the drift study below. |
| Independent of the step | ✅ | `IndependentOfTheIntegrationStep`: T/512 against T/256 at hover and 120 kt changes the controls by under 1e-6 of travel and the attitude by under 2.7e-6°. |
| Independent of heading and steady wind | ✅ | `IndependentOfHeadingAndSteadyWind`: at 30 kt, a 1.2 rad heading, and a 30 kt headwind (a hover over the ground), give the same controls (1e-6) and attitude (1e-6°). |
| Body-state geometry | ✅ | `BodyStateHasTheConditionsAirVelocity`: 500 random conditions, attitudes and winds. |
| Held evaluation is the free one at a = 0 | ✅ | `Executive.HeldEvaluationIsTheFreeOneAtZeroAcceleration`: with known G and H, (M_rb − H) a_free equals the held net wrench, and the held component rates are the free ones less G a. No allocation. |
| Model API, real-time rules | ✅ | `ModelStartsFromTrimAndKeepsTheRealTimeRules`: `trim()` resets to t = 0 at the trimmed state; then `advance()` allocates nothing and repeats bit-identically from a snapshot. `RejectsInvalidInputsAndLeavesTheModel`, `UnconvergedTrimIsReported`. |

**The UH-60A trim.** ISA sea level, 16 638 lbm, every 10 kt (`m7_trim.csv`):

![Trim](validation/m7/m7a_trim.png)

- *Hover:* 4.0° nose up and 2.4° left wing down. The tail rotor pushes right, so the disc tilts left to balance it: the UH-60's familiar left-wheel-low hover.
- *Power:* the main rotor needs 1535 hp in hover, 873 hp at the bottom of the power bucket at 70 kt, and 1882 hp at 150 kt (1890 hp before D-036; M7b below). The tail rotor has no power model (DATA_GAPS.md).
- *Pitch attitude and longitudinal cyclic:* not monotonic. The attitude rises from 0.9° at 50 kt to 3.4° at 80 kt, while the stabilator schedule (D-028) sweeps from 39° trailing edge down at 30 kt to 2.6° at 80 kt. As the stabilator's download falls, the nose comes up. Whether the real aircraft does this is the first question for the comparison with flight test (Ballin, NASA TM-88360, in M7c); the stabilator schedule's collective input scale is still an assumption (DATA_GAPS.md).
- The controls in percent depend on the travels; neutral longitudinal cyclic is uncertain by 0.8 in (8 % of travel), which shifts the longitudinal curve, not its shape.

None of this is compared with data yet: that is M7c.

**Drift study: why the free aircraft doesn't stay exactly in trim.** The trim holds the body still for a revolution. The free aircraft's airframe vibrates at 4/rev (VALIDATION.md M6d), which a held trim leaves out. Measured from each trim (`m7_drift.csv`):

![Drift](validation/m7/m7a_drift.png)

- *Starting on the vibration cycle.* Started from the held trim with zero vibration, the free body is off its vibration cycle by a small rate (about 0.006 rad/s in roll at 150 kt), and the roll damping reacts to it: 0.011 rad/s² of mean roll acceleration over the first revolution at 150 kt. The trim therefore adds to the start state the zero-mean velocity and rate oscillation that the free-body accelerations along the held orbit imply (`Problem::vibration`). That cut the drift by about 100× up to 60 kt and 30× at 150 kt.
- *What is left.* Up to 60 kt the drift (≈ 1e-6) is at the solver's tolerance, so the solver itself is exact there. From 90 kt it grows to about 1e-3 m/s² and 3e-4 rad/s² at 150 kt: the free blades respond to the airframe's 4/rev vibration, which feeds back into their mean loads, and the blade inertia coupling H varies over the revolution. Both are left out by holding the body, the standard averaged trim. The mean free-body acceleration along the held orbit is 2e-4 m/s² and 1.2e-4 rad/s² at 150 kt, the same size. 3e-4 rad/s² for one second is 0.015°/s of roll rate.
- *Not a measurement artifact:* the same run with RK4 at T/256, which divides the revolution exactly, gives the same values as the Model at 1 ms.
- A trim with the body free and periodic too would remove the rest. It isn't worth its complexity at this size; revisit if the M7b linear models or M8 piloted runs show a drift from trim.

**Cost.** A trim from a nearby trim costs about 45 revolutions: one Jacobian (about 40 unknowns, a revolution per column) and a few iterations. That is about 0.15 s (Release, GCC 13.3, WSL2); the 0–150 kt sweep with its free-flight runs takes about 3 s. Rebuilding the Jacobian every iteration instead (plain Newton, 2–3 iterations) was 2.5× slower.

## M7b — Linearization (2026-10-15)

M7b adds the linear models of D-035 and its amendment (SPEC_REVIEW #29):
- *Coordinates:* the state in body Euler angles, height and multiblade coordinates (`ReducedCoordinates`, any blade count).
- *Periodic model:* central-difference A(ψ_j), B(ψ_j) along the held trim orbit.
- *Averaged model:* their average, the constant-coefficient approximation averaged in the non-rotating frame (Johnson 2013 §15.4).
- *Floquet exponents:* from the free flow map over one revolution (§15.6).
- *Condensed 6-DOF model:* height removed, states that feed nothing removed, the rest quasi-static.
- *Outputs:* the derivatives in Padfield's form, and the modes with frequency, damping and an indicative label.

The linearization is internal (`src/analysis/linearize`), init-time, and not in the public API.

Also in M7b:
- *D-036 (SPEC_REVIEW #30):* GenHel's even airframe tables had a corner at zero angle, where every straight-flight trim sits. The two segments next to zero now use the parabola through the three nodes. The M7a trims move by at most 0.18 % of collective travel and 0.08° of pitch (150 kt); M7a above and its plots are regenerated. The M6d figures are unchanged at their quoted precision.
- *The held-orbit integrator* is shared by trim and linearization; the M7a outputs were bit-identical before D-036.
- *CI (D-037):* Eyas and its host deploy in a RHEL 9 container, so the Rocky Linux 8 job is dropped. ctest runs in parallel, and compiled objects are cached with ccache. The ASan job on each push skips the two slowest tests (L3 and the L4 rate derivatives). A new nightly and on-demand workflow, `full-verification.yml`, runs every test under ASan and every validation generator.

Tests: `tests/unit/test_linearize.cpp` (helpers in `tests/support/m7b_support.hpp`, fixtures in `tests/support/m7_fixtures.hpp`); D-036 in `tests/unit/test_airframe_aero.cpp`. Data from `eyas_m7b_cases`, plots from `tools/plot_m7b.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| Reduced coordinates (gate, amendment) | ✅ | `RatesAreTheTimeDerivative`: the reduced rates equal the time derivative of the reduced coordinates along the free flow, to 1e-7 relative, for the UH-60A and for 3-, 4- and 5-bladed fixtures. A planted sign error in the multiblade centrifugal term fails it by 3–11 (relative). `RoundTrip` (1e-12), `MatchesMultibladeFlap` (Padfield 3rd ed. Eqs. 3.42–3.45). |
| **L1** gravity and kinematics (gate) | ✅ | `GravityAndKinematicEntriesAreExact`: at 0, 60 and 120 kt and at every azimuth, the φ and θ columns of A hold gravity alone, the ψ_h column is zero, and the Euler-rate and ḣ rows are Padfield's Eqs. 4.36–4.37 (measured ≤ 2.5e-9, gated at 1e-7). In still air the loads don't depend on attitude and uniform gravity moves every particle alike, so the blade and moment rows have no attitude dependence. |
| **L2** Floquet, symmetric hover fixture (gate) | ✅ | `SymmetricHoverIsConstantCoefficient`: coaxial counter-rotating rotors over a mass, trimmed with no cyclic and no attitude (1e-14). The 64 multiblade Jacobians agree to 9e-11 of the largest entry (gated at 1e-8). All 32 Floquet exponents of the free flow map equal the averaged eigenvalues to 1.7e-8 relative (gated at 1e-7); the same planted sign error fails this by 1.2. |
| L2 on the UH-60A (reported) | ✅ | Below: the averaging error against μ, and second order in the periodic part. |
| State elimination (amendment) | ✅ | `FeedNothingStatesSplitTheEigenvalues`: at hover and 120 kt the full model's eigenvalues are the reduced model's together with the dropped states', to 1e-8 (measured 1e-10). Dropped: the out-of-contact tire and vertical-lag states at every speed, plus the airframe and tail-rotor wake lags in hover. |
| **L3** linear against nonlinear (gate, amendment option A) | ✅ | `PeriodicModelMatchesTheNonlinearResponse`, in hover, at 60 kt and at 150 kt (the sweep below covers every 10 kt): every body coordinate perturbed and every control given a 1 s sine doublet, at four halving amplitudes, flown for 3 s, against the periodic model. The relative error fits c0 + c1 a to within 15 % at every amplitude, i.e. the absolute error is second order, and the leftover c0 is below 1e-4 up to 60 kt and 1e-3 above. Measured: ≤ 3.2e-5 up to 60 kt, ≤ 5.4e-4 above. In hover the averaged model is gated too (≤ 2e-3; measured 1.04e-3, for longitudinal cyclic). |
| **L4** Padfield's closed forms (gate) | ✅ | `HoverHeaveDampingMatchesPadfield`: Z_w against Table 4.2 to 0.20 % and 0.09 % at two weights (gated at 0.5 %). `HoverRateDerivativesMatchPadfield`: ∂β1c/∂q and ∂β1s/∂p against Eq. 4.84 to 0.5 % at S_β = 0, 0.15 and 0.3 (gated at 1 %); the couplings against Eq. 4.85 within 1 % of the direct value; M_q and L_p against Eq. 4.81 to 1.1 % on the stiff hub (gated at 2 %). |
| Step-size study (D-035) | ✅ | Every central-difference step scaled from 0.1× to 10× at 0, 60 and 150 kt: the key derivatives change by at most 1e-4, most by under 1e-6. The defaults sit in the middle of a flat range two decades wide. |
| Mode labels | ✅ | `ModesAndLabels`: frequency, damping and labels on synthetic decoupled models. |

**L3 over the whole sweep.** From `eyas_m7b_cases`, every 10 kt, the largest leftover over the 12 cases:

![L3](validation/m7/m7b_l3.png)

- *Periodic model:* ≤ 3.2e-5 up to 60 kt and ≤ 5.4e-4 above, except at 30 kt.
  - About half of the high-speed leftover is the held-body linearization. A linear model built about the free, vibrating flight halves it (comparison below).
  - The rest is the integration step. Stepping both models at the same rate (T/256 to 60 kt, T/512 above) took the 140 kt collective case from 1.1e-3 to 7e-5.
- *30 kt:* 12 % leftover, from GenHel's tail-rotor fin blockage (CR-166309 pp. 5.4-2, 5.4-8). The blockage factor rises as a parabola up to its breakpoint, exactly 30.0 kt ("empirical and based on flight test data of other helicopters"), and is flat above it, so a 30.0 kt trim sits on a corner. It isn't smoothed: no source gives the shape (D-035 notes, approved 2026-10-15). A linear model at exactly 30 kt holds the average of the two slopes, and one within about a knot of 30 kt is less reliable for speed changes across it.
- *Averaged model:* its leftover grows from 0.1 % in hover to 7 % at 150 kt. This is the periodicity it drops, as SPEC_REVIEW #29 predicted.
- *Amplitudes:* the largest amplitudes are chosen to stay clear of nearby map nodes. In hover the wake skew χ is −0.85°, next to the 0° node of the rotor-wash maps; at 60 kt the flapping a₁ is 0.12°, next to its 0° node. A 0.25 m/s perturbation crossed them.

**The averaging error and the periodic part.**

![Floquet](validation/m7/m7b_floquet.png)

- *Left:* the largest gap between the averaged eigenvalues and the Floquet exponents of the free flow map.
  - Slow modes (|λ| < 1 rad/s): 4e-5 rad/s in hover, and 4e-4 to 4.4e-3 from 10 to 140 kt, growing with μ. The exceptions are 6.7e-3 at 30 kt, where the flow map's central differences straddle the blockage corner, and 0.028 at μ = 0.35 (150 kt), where a slow real root moves by 10 % (−0.269 to −0.241 rad/s).
  - Faster modes (1–3 rad/s): up to 0.04 rad/s at 40–50 kt, a root near −2.5 ± 0.8i where roll subsidence couples with the tail-wake lags. Johnson (p. 757) expects the approximation to be least satisfactory for faster, rotor-coupled modes.
  - M7c's comparison with identified models therefore reports the Floquet exponents alongside the averaged ones.
- *Middle:* the multiblade Jacobian's body rows vary around the revolution by 9 % already in hover, almost all in the differential-coning (β_d) columns. Their mean is zero; the 2/rev part comes from the trim's cyclic flapping (Johnson p. 557). Without those columns the variation is 0.14 % in hover and 3.9 % at 150 kt.
- *Right:* the gap with the periodic part scaled by s (J = 1024 samples, midpoint matrix exponentials) shrinks 3.7–4.9× per halving at 60, 120 and 150 kt: second order. This follows because the first-order change of a simple Floquet exponent is the revolution mean of the periodic part, which is zero (own perturbation argument, SPEC_REVIEW #29).

**The held-body shortcut.** The linear models are taken about the held trim orbit, while the free aircraft vibrates at 4/rev (M7a). Rebuilt about the free flight (its first and its third revolution, `m7b_heldfree.csv`):
- *Eigenvalues (|λ| < 3 rad/s):* they move by at most 1.3e-6 relative in hover, 6.7e-5 at 60 kt, 8.1e-5 at 120 kt, and 6.9e-4 rad/s (4.2e-3 relative) at 150 kt.
- *Key derivatives:* they move by 1.7e-6, 6.8e-5, 3.5e-4 and 2.9e-3. At 150 kt M_w is the exception, at 5 %: it is nearly zero there (5e-4, changing sign with speed), and its change is 2.5e-5 rad/(m·s).
- *How it compares:* the first and third revolutions differ from each other by as much, so this is the size of the shortcut's effect, well below the averaging error.

**The UH-60A's modes and derivatives.** ISA sea level, 16 638 lbm, condensed 6-DOF model (`m7b_modes.csv`, `m7b_derivatives.csv`):

![Modes](validation/m7/m7b_modes.png)

![Derivatives](validation/m7/m7b_derivatives.png)

- *Hover:* coupled pitch–roll oscillations, one unstable (0.198 ± 0.374i, ω_n 0.42 rad/s, ζ = −0.47) and one lightly damped (−0.153 ± 0.485i); pitch subsidence −1.03, roll subsidence −5.87 and a heave–yaw pair −0.27 ± 0.07i (ζ = 0.97). L_p = −5.86 /s, Z_w = −0.30 /s, M_q = −0.85 /s.
- *Forward flight:* an unstable phugoid (0.157 ± 0.430i at 60 kt, 0.123 ± 0.358i at 120 kt), a Dutch roll (ζ ≈ 0.27–0.29, ω_n 1.0 rad/s at 60 kt and 1.46 at 120 kt), a stable spiral and roll subsidence. Labels are a heuristic from the eigenvectors, indicative where modes couple strongly.
- With speed, Z_w and M_q grow in magnitude, L_p falls and N_v grows; the control derivatives change little, except Z_col and M_lon, which grow. None of this is compared with data yet: the comparison with the UH-60's identified models (Fletcher, 1995), with Padfield's App. 4B trends and with the literature's mode character is M7c (D-035 V2).

**L4 beyond the gates** (`m7b_l4.csv`):
- *Heave:* the retained model's heave root differs from Z_w by 7 % and 14 % at the two weights. The Pitt–Peters inflow lag (τ = 0.16 s here) couples into heave, which the condensed model and Padfield's first-order heave mode leave out.
- *Pitch damping:* Eq. 4.81 leaves out the in-plane rotor loads (Padfield p. 197). They grow as the hub softens: 2.7 % at S_β = 0.15. On the centre-hinged hub (S_β = 0) the model's pitch damping even changes sign against the thrust-tilt estimate (+599 against −2531 N·m·s). The in-plane loads Padfield describes (the Amer effect, Eqs. 4.79–4.80, strongest at low thrust coefficient and high collective) act in that direction; Eq. 4.79 is derived for an untwisted rotor and isn't checked quantitatively here.

**Cost.** Linearizing the UH-60A (41 states, 64 azimuths) takes 25 ms, condensing 0.02 ms, and the Floquet flow map 0.25 s (Release, GCC 13.3, WSL2).

**Builds:** all 295 tests pass with GCC 13.3 (Debug) and Clang 18.1 (Debug), and all 294 in a Rocky Linux 9 container (GCC 11, Release; the CI job rehearsed in Docker with ccache). The new and changed tests (linearization, trim, airframe, tail-rotor wake; 43) pass under ASan+UBSan. clang-format is clean, and clang-tidy is clean on the changed `src/` files.

## M7c — Validation against Ballin and Fletcher (done, 2026-10-17)

M7c compares the UH-60A with published data (D-035 M7c amendment). It closes M7 (D-035):
- **V1:** trims against Ballin, NASA TM-88360, App. B: GenHel (Sikorsky and Ames) and the USAAEFA flight test. Level flight is Fig. B1; climb and descent are Figs. B2–B4.
- **V2:** linear models against Fletcher's flight-identified UH-60 data, 21st ERF 1995 Paper VII-4.

Every digitized figure is human-reviewed (DATA_GAPS.md).

Eyas runs:
- *V1:* `uh60a_ballin.yaml`, 16 000 lbf, 5250 ft, with D-038 to D-040 and under GenHel's lateral constraint (zero roll from 50 kt). The V1 gate is against the Sikorsky GenHel curves (D-035).
- *V2:* `uh60a_fletcher.yaml`, 14 350 lbm, ISA sea level. The averaged linear model is used, with GenHel's primary servo.

Data and plots:
- `eyas_m7c_cases` with `tools/plot_m7c.py` (V1);
- `eyas_m7c_fletcher` with `tools/plot_m7c_fletcher.py` (V2).

Both run in the nightly verification (D-037).

| Item (D-035 M7c amendment) | Status | Evidence |
|---|---|---|
| **V1 gate**: longitudinal cyclic (10–150 kt, shape only: its mean set neutral cyclic), lateral cyclic (10–110 kt), stabilator (10–150 kt), pitch attitude (0–80 kt) against Sikorsky GenHel | ✅ | `BallinTrim.LevelFlightAgainstSikorskyGenHel`. The tolerance comes from the measured read-off and isn't widened for any quantity. |
| V1 open differences, tracked at their recorded values | ✅ tracked | `BallinTrim.*` fails if any of them grows or shrinks; see the gap register below. |
| V1 waterline sensitivity | ✅ | At most 1.3 % of travel and 0.36°, within the gate tolerance ("V1 side questions"). |
| Neutral longitudinal cyclic in the GenHel spread | ✅ | The fit gives 1.904 in, inside 1.10–1.91 in (D-040). |
| Pitch bias actuator in the static trims | ✅ | Off. With it on, the stick would sit 4.3 % aft of both GenHel and the flight test ("Pitch bias actuator"). |
| D-034 item 5 rulings: elastic twist, ΔC_DMR, yawed drag | ✅ | D-038, D-039; the Ames twist question is resolved. |
| Pedal through the 30 kt corner | ✅ | Sikorsky GenHel shows the same corner; it stays unsmoothed. |
| V2 Table 1: M_β/I_β and Lock number | ✅ reported | M_β/I_β is exact. The effective Lock number is 7.87, against Fletcher's 8.06 theoretical and 8.62 identified. |
| V2 on-axis responses, 1.5–12 rad/s | ✅ reported | Within 1.6 dB and 12° RMS, in hover and at 80 kt. |
| V2 on-axis responses below 1.5 rad/s; yaw rate to collective | open, reported | Parked or assigned to M9 (D-041). |
| V2 off-axis responses | known limitation | Wake distortion goes to M9 (D-041). |
| V2 mode character against Padfield §4.3 | ✅ hover; 80 kt open | 80 kt has a phugoid-type divergence from M_u < 0 (parked, D-041). |
| V2 Floquet against averaged roots | ✅ | Within 0.0025 rad/s below 3 rad/s. |

**Level flight (Fig. B1).** Eyas is in red. The dotted curves hold sideslip at zero instead of roll.

![Level flight](validation/m7/m7c_level.png)

Eyas − Sikorsky GenHel over 0–160 kt, and Eyas − flight test at its eight points (`m7c_differences.csv`):

| | GenHel mean / RMS | flight test mean / RMS |
|---|---|---|
| longitudinal cyclic [%] | 0.03 / 0.60 | 0.85 / 2.83 |
| lateral cyclic [%] | 1.41 / 2.45 | −3.04 / 6.71 |
| collective [%] | −0.34 / 2.79 | −4.05 / 7.58 |
| pitch attitude [deg] | 0.20 / 0.37 | 0.51 / 0.74 |
| sideslip [deg] | 3.52 / 3.54 | 3.57 / 3.94 |
| pedal [%] | −2.83 / 4.48 | 10.1 / 11.5 |
| stabilator [deg] | 0.09 / 0.66 | −5.40 / 5.75 |
| main-rotor power [hp] | −49 / 57 | −191 / 280 |

The flight-test columns include what Ballin (p. 10) notes for both GenHel models: a test pedal bias of about 0.5 in right, and more collective and power at low speed than either model predicts.

**Frequency responses (V2).** Black is flight and red is Eyas. Grey marks what isn't compared.

![Hover](validation/m7/m7c_fletcher_hover.png)

![80 kt](validation/m7/m7c_fletcher_80kt.png)

![Yaw to collective](validation/m7/m7c_fletcher_yaw.png)

### Gap register

Every M7c difference that isn't explained by the data's accuracy, with where it goes. D-041 records the dispositions. The investigations behind each one are in the subsections below.

| Gap | Size | Status | Goes to |
|---|---|---|---|
| Sideslip at zero roll | +3.5° against GenHel; Eyas rolls left where the test rolled right | **Closed by D-048 (2026-10-20):** GenHel's downwash correction terms (p. 5.10-19) were missing; now +0.2 to −1.1° against GenHel, roll 0.29° rms against the test | Below, "Kaplita steady-heading sideslips" |
| Lateral cyclic above 110 kt | Up to +7.8 % at 155 kt | Unexplained; not linked to the sideslip after all (unchanged by D-048) | Parked |
| Longitudinal cyclic at cruise (since D-048) | 2.1–3.5 % aft at 100–150 kt (unchanged by D-051: its wash input is the full disc's) | About 1.1 % explained (inflow-model sideslip, D-036); about 1.9 % open, of the size of the collective and lateral-cyclic gaps | USAAEFA report and M9 rotor items (issue #54) |
| Pedal | GenHel − Eyas +0.8 to +7.8 % | Explained by the sideslip and collective (residual ±0.65 %); −1.3 to +3.3 % since D-048 | Follows the sideslip |
| Hover main-rotor power | −71 hp (4.4 %); **−38 hp (2.3 %) with D-051** | **Tip loss on the inflow done (D-051, 2026-10-22): +38 hp.** About 140 hp of GenHel's power is unexplained like for like (below) | Nonuniform-inflow losses, issue #66 |
| Cruise power and collective | −49 hp mean; collective −6.4 % at 160 kt; **with D-051, −5 hp mean (36 rms) over 0–160 kt and collective −4.2 % at 160 kt** | Within the yawed-drag ruling (D-039) | Tracked by the gate |
| B2–B4 descent collective | Eyas falls 6–9 % less at 100–137.5 kt | Unexplained | Parked |
| B2–B4 climb lateral cyclic | +5.3 and +9.5 % at +3000 ft/min | Unexplained; the 137.5 kt end trims beyond full collective | Parked |
| B2–B4 power slope at 137.5 kt | 14 % above GenHel; Eyas − GenHel +88 hp per 1000 ft/min, **+91 with D-051** | Unexplained | Parked |
| Slow on-axis response below 1.5 rad/s | Hover pitch +3.7 dB; 80 kt phase lead 27° (roll), 43° (pitch) | At 80 kt, M_u < 0 from the held stabilator in the rotor downwash, as simple wake theory gives | Parked |
| 80 kt phugoid-type divergence | +0.289 1/s | Same cause as the line above | Parked |
| Off-axis angular rates | Phase off by 140–160° (hover) | Known limitation of Pitt–Peters without wake distortion | M9 wake distortion (D-041) |
| Yaw rate to collective, hover | −3.8 dB mean, +43°; on GenHel's engines (M9 Stage B) −2.5 dB, +96° | Rotor speed and engine dynamics (Fletcher); GenHel's engine model's rotor-speed response to collective (Ballin TM-88360 p. 12) | M9: NASA T700 model (TM-100991) next |
| Yaw rate to collective, 80 kt | −8 dB | Unexplained; Fletcher's BE model misses it too | Parked |
| Tail-rotor torque | Not modelled | DATA_GAPS.md | M9 |
| Steady turns (B5) | Not compared | Needs an interface change | M8 or later (D-035) |
| Step responses (App. D and E) | Reported | Below, "Ballin App. D and E" | D-046 |

**Builds:** all 312 tests pass with GCC 13.3 and Clang 18.1 (Debug), including both `BallinTrim` gates.

The investigations follow, in the order they were done.

### Open: Eyas trims with right sideslip at zero roll

D-040 (the stabilator schedule on equivalent airspeed) came after most of this work and moves the zero-roll sideslip by at most 0.04°, so these numbers stand.

| kt | 60 | 80 | 100 | 120 | 140 | 160 |
|---|---|---|---|---|---|---|
| Eyas β [deg] | 5.89 | 3.98 | 3.29 | 3.07 | 3.11 | 3.39 |
| GenHel β [deg] (B1(f)) | 3.05 | 0.87 | −0.32 | −0.78 | −0.78 | −0.45 |

Trimmed at the flight test's sideslip with roll free, Eyas rolls left where the test rolled right (test roll from B1(e), unverified):

| kt | 49 | 73 | 95 | 118 | 140 | 163 |
|---|---|---|---|---|---|---|
| test β / φ [deg] | 0.9 / −0.2 | 0.4 / 0.1 | 0.8 / 0.8 | 1.1 / 1.7 | 0.6 / 1.5 | −1.3 / −0.8 |
| Eyas φ [deg] | −0.85 | −0.86 | −0.89 | −1.03 | −1.84 | −4.74 |
| W sin Δφ [lbf] | ≈ 180 | 270 | 470 | 760 | 930 | 1100 |

So Eyas carries a rightward side force (or lacks a leftward one) of ≈ 370 lbf at 100 kt against GenHel, growing with speed against the test. At these rolls ΣF_y = W sin φ ≠ 0, so the CG height enters the roll balance (Ballin's waterline question, DATA_GAPS.md) at about W sin φ × 1.39 ft ≈ 650 ft·lbf at 95 kt, of order 0.1° of roll.

Side-force budget at 100 kt, zero roll (β = 3.30°): main rotor −290 lbf, tail rotor +598, fuselage −263, fin −45, stabilator −1.

**Ruled out, each by a test:**
- *Main-rotor side force against theory.* Split by term (scratch instrumentation, 60/100/140 kt): lift tilted in-plane by the inflow angle −278/−253/−289 lbf, lift along the flapped normal −34/−23/−60, drag terms −6/−13/−62. The blade inertial terms (Coriolis, centrifugal, flap acceleration) sum to |F_y| < 10 lbf: a periodic blade's mass centre has zero mean acceleration. The lift part against Johnson (2013) Eqs. 6.75 and 6.85 evaluated at Eyas's own flapping, pitch and inflow harmonics, in wind axes, scaled by Eyas's thrust (Y/T, so the lift slope cancels): the closed form and the numerical integral agree to 0.1 lbf; adding Eyas's twist table, elastic twist, tip loss, root cutout, hinge offset and Pitt–Peters harmonics gives −246/−254/−295 lbf against Eyas's −274/−294/−372. The 10–20 % rest is the nonlinear, Mach-dependent airfoil tables, and goes the way GenHel needs. Eq. 6.85 is a difference of terms of 1000–3000 lbf, so the side force is sensitive to the rotor state, not to the force integration.
- *Lateral cyclic against GenHel.* θ1c computed for both models from all four plotted controls through the shared GenHel rigging (the offsets cancel, to the then-placeholder's ±0.2°): Eyas − GenHel within ±0.25° from 60 to 110 kt (−0.13° at 100 kt; with D-040), −0.4 to −1.6° at 120–160 kt. Tilting the disc for 370 lbf would take about 1.3°.
- *Wash-map inputs* (D-028; the maps use each model's own trimmed χ and a₁): a₁ ±1° and ±3°, χ ±2° and ±5°, wash ×0.9 and ×1.1 move β by ≤ 0.1°; removing the wash moves it 0.04° at 100 kt and 0.75° at 60 kt.
- *Blade lag* (Eyas has none, D-026): a self-consistent rigid lag probe moves β by +0.2 to +0.4° (wrong way); closing the gap would need ~14× the self-consistent 1/rev lag.
- *GenHel's inflow* (uniform with K_CM = K_CL = 0, as in the 1981 dataset): β +3.9°, worse. Uniform inflow: +4.15°.
- *Rotor variants at 100 kt:* yawed drag off 3.21°, elastic twist off 3.28°, 60 elements 3.32°, no tip loss 3.29°.
- *CG waterline* (WL 248.2 against 231.5 in): β 3.30 → 3.31°; at zero roll ΣF_y = 0, so CG height cancels.
- *Fin* reproduces GenHel Table 3.5.9 (C_L(0) = −0.06, about +45 lbf at β_v = 0, 100 kt). *Fuselage:* the side-force table is antisymmetric and the yaw table has N(0) = 0, so neither model has a lateral offset. *DNQD = 0* (DATA_GAPS.md): moving 370 lbf of tail-rotor thrust at the 33 ft arm needs about 12 000 ft·lbf, N/q ≈ 400 ft³ at 100 kt, comparable to the yaw map's extremes at 70–90°: not plausible near β = 0.
- *Tail rotor* against GenHel pp. 5.4-4 to 5.4-8 and *hub loads* against pp. 5.1-33 to 5.1-36: same equations; the 20° cant reading stands (DATA_GAPS.md). Yaw balance pins the tail-rotor thrust to within the main-rotor torque difference, a few % of ~28 000 ft·lbf over ~33 ft, i.e. 30–50 lbf of the 370.

**Pedal gap explained.** GenHel − Eyas pedal at zero roll grows from +0.8 % (60 kt) to +7.8 % (150 kt). Eyas's own dPedal/dβ (−0.6 to −1.4 %/deg, from its zero-sideslip and zero-roll trims) times the sideslip difference, plus the collective-to-tail-rotor mixing (16° θ0 per unit collective), leaves a residual within ±0.65 % at every speed to 160 kt (with D-040 and the corrected readings). It follows from the sideslip and collective; it isn't a separate defect.

**Status:** unexplained, recorded open. Since the force integration matches theory and the lateral cyclic matches GenHel, the difference is in the rotor state GenHel trims to or in a term of GenHel's not visible from its equations as read; the flight test (which flew rolled right) is the referee. Parked (D-041): revisit with the USAAEFA report or if M8's piloted evaluation shows it.

**Closed (2026-10-20, D-048):** it was "a term of GenHel's not visible from its equations as read": the motion module's downwash correction terms (p. 5.10-19), a constant side force and sideslip-dependent rolling and pitching moments, never transcribed. With them Eyas's zero-roll sideslip is within +0.2 to −1.1° of GenHel's, and at the test's sideslip Eyas rolls right with the test (0.29° rms). Section "Kaplita steady-heading sideslips" below.

### Stabilator airspeed (D-040) and neutral longitudinal cyclic

**Stabilator.** Against Ballin's Sikorsky GenHel B1(h), Eyas's scheduled stabilator sat 1–4° low from 40 to 90 kt. GenHel's schedule (Fig. 5.1.7) evidently reads equivalent airspeed, as the aircraft's air-data sensors give it: fed GenHel's own collective, the schedule fits B1(h) to 0.61° rms on equivalent airspeed and 2.06° on true airspeed (5250 ft; at sea level they are the same). D-040 makes the change. Eyas − Sikorsky GenHel, 40–150 kt, rms (mean), before → after:

| | stabilator [deg] | longitudinal cyclic [%] | pitch [deg] | lateral cyclic [%] |
|---|---|---|---|---|
| true airspeed | 2.13 (−1.91) | 2.10 (−1.74) | 0.92 (+0.91) | 2.73 (+1.76) |
| equivalent airspeed | 0.77 (+0.06) | 0.57 (+0.14) | 0.40 (+0.27) | 2.43 (+1.37) |

Collective, sideslip, pedal and main-rotor power move by under 0.2 %, 0.04° and 2 hp.

**Neutral longitudinal cyclic.** Fitted to Ballin's GenHel longitudinal stick (B1(a), human-reviewed). Before D-040 the fit needed neutral at 2.06 in, 0.14 in beyond the range of GenHel's three Fig. 6.3.1 estimates (1.10–1.91 in), so the D-035 check failed; neutral was set at the pinned row's 1.91 in and 1.4 % of stick was recorded as an Eyas–GenHel pitch-trim difference. With D-040 the fit lands at 1.904 in, inside the range and 0.07 % of travel from 1.91 in; the check passes, and the residual is 0.64 % rms (largest 1.2 %), within the read-off accuracy: the 1.4 % was the stabilator's airspeed. With this neutral the cyclic offsets come out at −0.009° (θ1s) and +0.001° (θ1c), i.e. a centred stick gives a level swashplate. Since the fit chose neutral from GenHel's three estimates, the V1 gate tests B1(a)'s shape against speed only, its mean removed (largest 1.17 % against a tolerance of at least 1.79 %), and tracks the mean (Eyas 0.07 % aft over 10–150 kt) as a calibration. The check on neutral independent of the choice is the flight test, whose mean differs by 0.4 % (pitch bias actuator, below).

### Pitch bias actuator

GenHel's PBA (Fig. 5.1.4) settles to −0.375 %/deg of pitch attitude plus 0.0525 %/kt above 80 kt, less 4.9 %, in percent of longitudinal stick travel, and in a trim it moves only the stick (the blade pitch, and so the trimmed state, are the same). With it on, Eyas's stick would sit 4.3 % aft of the Sikorsky GenHel curve with a 1.75 % rms shape error; with it off, −0.07 % and 0.64 %. The flight test agrees (off −0.4 %, on −4.4 %). V1 compares with it off (DECISIONS.md D-035).

### Climbs and descents (Figs. B2–B4)

Ballin's B2–B4 give no weight, CG or density altitude, and their level points differ from B1's (DATA_GAPS.md), so only the **change from level flight** against climb rate is compared, Eyas at B1's conditions. Sikorsky's GenHel held the stabilator at its level-flight value for each speed in these runs (Ballin p. 10); `eyas_m7c_cases` now writes both Eyas variants, `fixed_stabilator` (held at Eyas's own level incidence, through GenHel's STBSET override) and `genhel` (scheduled). Against the Sikorsky curves, holding the stabilator halves the longitudinal and pitch errors, as expected:

| rms of Eyas − Sikorsky change from level, −3 to +3 thousand ft/min | 60 kt | 100 kt | 137.5 kt |
|---|---|---|---|
| longitudinal cyclic [%], held / scheduled | 2.4 / 3.9 | 3.3 / 6.9 | 2.9 / 7.1 |
| pitch attitude [deg], held / scheduled | 0.23 / 0.79 | 0.34 / 1.06 | 0.65 / 1.39 |

With the stabilator held, against Sikorsky (rms over the Sikorsky curve's readings, from `m7c_differences.csv`; Eyas − GenHel at −3 / +3 thousand ft/min):

![Climb 60 kt](validation/m7/m7c_climb_60kt.png)

![Climb 100 kt](validation/m7/m7c_climb_100kt.png)

![Climb 137.5 kt](validation/m7/m7c_climb_137.5kt.png)


| | 60 kt | 100 kt | 137.5 kt |
|---|---|---|---|
| collective [%] | 1.3 (+3.4 / +0.4) | 2.4 (+6.0 / −0.3) | 3.4 (+9.2 / +1.4) |
| longitudinal cyclic [%] | 2.4 (−3.7 / +3.1) | 3.3 (−4.9 / +6.1) | 2.9 (−5.6 / +2.5) |
| lateral cyclic [%] | 1.5 (+0.7 / +3.2) | 2.5 (−2.4 / +5.3) | 4.5 (−5.0 / +9.5) |
| pedal [%] | 0.6 | 1.8 | 1.7 |
| pitch attitude [deg] | 0.23 | 0.34 | 0.65 |
| main-rotor power [hp] | 64 | 61 | 92 |

- *Power against climb rate* (slope over ±2000 ft/min, hp per 1000 ft/min): Eyas 465 / 479 / 521, Sikorsky 445 / 469 / 447, Ames 441 / 455 / 447, flight test 414 / 483 / 368 (four points each); W·V_c at 16 000 lbf is 485. At 60 and 100 kt Eyas is within 2–4 % of GenHel, so the unknown weight of B2–B4 can't be far from B1's; at 137.5 kt Eyas's slope is 14 % above GenHel's and above W·V_c. `Open.`
- *Collective in steep descent:* Eyas's collective falls less than GenHel's, by 3.4, 6.0 and 9.2 % at −3000 ft/min, growing with speed. `Open.`
- *Lateral cyclic in steep climb:* Eyas moves right of GenHel by 3.2, 5.3 and 9.5 % at +3000 ft/min. `Open.`
- These are trends at constant weight and density altitude of unknown value; the climb runs also sit at Eyas's level-flight sideslip, which differs from GenHel's (above).
- *Investigated (2026-10-16), each by a probe at the climb conditions, stabilator held; Eyas − Sikorsky change from level at 60 / 100 / 137.5 kt:*
  - *Inflow model:* uniform inflow in place of Pitt–Peters (GenHel's inflow has no first-harmonic terms, K_CM = K_CL = 0) removes the climb lateral drift at 60 kt (+3.2 → −0.2 %) but not at 100 and 137.5 kt (+4.1, +9.0 %), and leaves the descent collective (+3.8 / +5.8 / +9.0 %) and the power slope (+78 at 137.5 kt) unchanged.
  - *Yawed-flow drag (D-039) off:* descent collective +3.2 / +4.6 / +7.0 %, power slope +60 at 137.5 kt: about a quarter of each, from physics Eyas keeps (Johnson 2013 §6.22–6.23).
  - *Elastic twist (D-038) off:* no help (descent collective +10.1 % and power slope +97 at 137.5 kt).
  - *Pitch attitude:* nudging the held stabilator ±2° in the −3000 ft/min descent moves the collective by only +0.15 and +0.24 % per degree of pitch at 100 and 137.5 kt (the trim compensates), so Eyas's ~0.5° pitch difference from GenHel explains about 0.1 % of the descent-collective gap.
  - *Advancing tip near Mach 0.86 at 137.5 kt:* the near-sonic part of the disc (M > 0.8, α −3° to +1°) gives 26 % of the profile torque in the −3000 ft/min descent and 24 % in level flight; negative lift is 8 % against 4 % of the total lift. Nothing compressibility-related switches on in the descent, so it doesn't explain a descent-only gap.
  - *Beyond the collective travel:* at 137.5 kt and +3000 ft/min Eyas's trim needs 105 % collective, past full up (the trim doesn't limit the controls); GenHel's curve is just under 100 % there. Eyas's lateral cyclic and power at that point come from a trim the aircraft can't fly; read the 137.5 kt climb end with that in mind.
  - Still unexplained: the descent collective (Eyas falls 6–9 % less than GenHel at 100–137.5 kt; at 60 kt within tolerance), the climb lateral drift at 100–137.5 kt, and the power slope at 137.5 kt. Parked (D-041).

### V1 side questions (D-035 M7c amendment)

Answered with a scratch probe on Ballin's B1 trims (GenHel's lateral constraint, stabilator scheduled, 5250 ft):
- *Pedal through the 30 kt fin-blockage corner (D-036 notes):* Eyas's pedal slope halves at exactly 30 kt (0.62–0.68 %/kt from 20 to 30 kt, 0.28–0.33 %/kt from 30 to 40 kt; 1 kt steps). Ballin's Sikorsky GenHel curve changes the same way (0.613 and 0.332 %/kt over 20–30 and 30–40 kt, from the reviewed readings at 5 kt steps). The flight test has three points near the corner (21.8, 29.8 and 31.8 kt) scattered by 2 % and 10–18 % left of GenHel (Ballin's pedal bias), too few to tell a corner from a smooth curve. So there is no evidence to smooth it; the D-036 ruling stands.
- *Ballin's "no dynamic twisting or bending" for the Ames GenHel (p. 3) against CR-166309's K_FP0:* removing the elastic twist (D-038) lowers Eyas's trimmed collective by 6.9–7.1 % of travel up to 100 kt and 9.1–12.7 % from 120 to 160 kt. The Ames and Sikorsky GenHel collective curves of Fig. B1(c) coincide within a line width (about 1 %) from 80 to 150 kt (0.7 % apart at 40 kt). A model without the twist couldn't have drawn the Ames curve, so it had the twist or an equivalent. Ballin's sentence reads as no torsional or bending degrees of freedom, which K_FP0, a quasi-static pitch correction proportional to the blade load, isn't. D-038 stands.
- *Waterline (DATA_GAPS.md, Ballin's datum open):* the same aircraft with the CG at GenHel's WL 248.2 in instead of Ballin's 231.5 in moves the B1 trims by at most 1.30 % lateral cyclic (hover; 0.55 % at 40 kt, ≤ 0.2 % from 60 kt), 1.15 % longitudinal cyclic (20 kt), 0.11 % collective, 0.09 % pedal, 0.36° pitch, 0.22° roll, 0.01° sideslip and 5 hp. All of that is within the gate's 1.75 % read-off tolerance, so the open datum doesn't affect a V1 conclusion.

### The V1 gate

`BallinTrim.LevelFlightAgainstSikorskyGenHel` and `BallinTrim.ClimbAndDescentTrendsAgainstSikorskyGenHel` (`tests/unit/test_ballin_trim.cpp`; slow, about 30 s and 60 s in Debug) trim the Ballin aircraft and compare against the Sikorsky GenHel curves with a tolerance from the measured read-off: 1.75 % of the axis span plus the local slope times 1.5 kt (0.1 thousand ft/min for B2–B4); DECISIONS.md D-035. They gate the longitudinal cyclic (10–150 kt; its shape, with the mean removed, since the mean set neutral cyclic, and the mean tracked at its record), lateral cyclic (10–110 kt), stabilator (10–150 kt) and pitch attitude (0–80 kt), all within tolerance, and track every open difference above (lateral cyclic above 110 kt, longitudinal cyclic at 100–150 kt, pitch at 50 kt, collective, sideslip, pedal, main-rotor power; the descent collective, climb lateral cyclic and power slope of B2–B4) at its recorded value, so a change either way fails the test. A deliberate change to one record was caught.

### Open: hover power

Eyas 1537 hp main-rotor power at 0 kt against GenHel 1608 (B1(i)): induced 1210 hp equals ideal momentum (κ = 1.00), profile 325 hp, figure of merit 0.787.
- *ΔC_DMR = 0.002:* GenHel subtracts it (CR-166309 p. 5.1-27, re-read at 400 dpi; nomenclature p. 5.1-45 "Profile drag correction", no units). GenHel's profile power is therefore about 65 hp below Eyas's, leaving ≈ 140 hp of GenHel's power unexplained like for like.
- *Tip loss:* both models zero the lift outboard of B·R but take momentum inflow over the full disc. Leishman 2nd ed. §3.2.4 (Eqs. 3.36, 3.41) raises the induced velocity by 1/B: about +37 hp. Deferred to M9 with the inflow upgrades (D-041); a departure from GenHel needing an M5 recheck. **Done (D-051, 2026-10-22):** Johnson's effective disc on both inflow models; at Ballin's B1 conditions the hover power rose from 1535 to 1572 hp, −38 hp against GenHel. M9 below.
- *Ruled out:* fuselage download (334 lbf), stabilator download (−16 lbf), element count (within 5 hp).
- The real aircraft needs more power than both models at low speed (Ballin p. 10); the USAAEFA flight-test report (Project 79-24) would be the referee (DATA_GAPS.md).

Cruise main-rotor power is in D-039 (mean −45 hp against GenHel); with GenHel's uniform-plus-harmonic inflow it would be −10 hp mean against flight test (probe, not adopted; left to the V2 frequency responses).

### V2: frequency responses against Fletcher's flight data (reported)

`eyas_m7c_fletcher` (`tests/validation/m7c_fletcher.cpp`) trims `uh60a_fletcher.yaml` (about 14 350 lbm, CG FS 360) in hover and at 80 kt with the stabilator held at Fletcher's 40° and 4° trailing edge down and no stability augmentation. It writes p and q per inch of lateral and longitudinal cyclic from the averaged linear model (M7b), with and without GenHel's primary servo. `tools/plot_m7c_fletcher.py` compares them with Fletcher's flight data (21st ERF 1995 Paper VII-4, Figs. 3 and 4; digitized and human-reviewed, DATA_GAPS.md). The comparison uses only the stretches where the coherence is at least 0.6, up to 12 rad/s. Fletcher's phase curves sit on their own 360° branch, so each Eyas phase curve is moved by whole turns. The figures below are Eyas with the servo, the like-for-like case (the flight data include the actuators); the servo adds 28° of lag at 10 rad/s and changes the magnitude by less than 0.05 dB.
- *Conditions assumed:* ISA sea level. Fletcher flew at Crows Landing, California, in September 1992 in very calm air (p. VII-4-6) and gives no altitude or temperature.

| | p/δlat hover | q/δlon hover | p/δlat 80 kt | q/δlon 80 kt |
|---|---|---|---|---|
| magnitude, 1.5–12 rad/s: mean / RMS [dB] | −0.7 / 1.6 | 0.0 / 1.2 | −0.1 / 0.9 | 0.8 / 1.1 |
| phase, 1.5–12 rad/s: mean / RMS [deg] | −3.1 / 11.7 | 3.0 / 8.6 | 0.1 / 6.2 | 4.6 / 7.7 |
| magnitude, below 1.5 rad/s: mean / RMS [dB] | 0.7 / 2.2 | 3.7 / 4.0 | −0.8 / 2.5 | 2.3 / 3.6 |
| phase, below 1.5 rad/s: mean / RMS [deg] | 1.5 / 3.0 | 9.6 / 11.5 | 27.3 / 33.2 | 43.0 / 49.0 |

- *On-axis, 1.5–12 rad/s (roll from lateral, pitch from longitudinal):* Eyas matches the flight data in both conditions: magnitude within 1.6 dB RMS, phase within 12° RMS. This is the band of the rotor-fuselage coupled roll and pitch response that flight control design works in.
- *On-axis, below 1.5 rad/s (the whole-aircraft modes), open:* in hover, Eyas's pitch response is 3.7 dB too large on average from 0.56 rad/s (where the coherence first reaches 0.6) to 1.5 rad/s, with a higher peak near 0.7 rad/s. At 80 kt the magnitudes are close, but Eyas leads in phase: 27° on average in roll, below 1 rad/s, and 43° in pitch at 0.2–1 rad/s, up to 85°. The flight roll response stays flat in magnitude down to 0.1 rad/s; Eyas's falls by about 5 dB, with phase lead, as with a stronger roll restoring moment through sideslip. The likely suspects are the slow-mode derivatives: the speed and incidence stability in pitch, and the dihedral and weathercock stability in roll and yaw. The condensed-model modes against Padfield §4.3 (the rest of V2) are the next check.
- *Off-axis (roll from longitudinal, pitch from lateral), known limitation:* Eyas misses badly in both conditions (phase off by 140–160° on average in hover; magnitude off by up to 13 dB in hover and 28 dB at 80 kt). This is the documented limitation of blade-element rotors with Pitt–Peters inflow. Fletcher (p. VII-4-3) writes that such models "predict the on-axis responses well to about 10 rad/sec but have deficiencies in their off-axis angular rate response prediction", reports the same for AH-64 and BO 105 models, and attributes it to the wake distortion in pitch and roll that Pitt–Peters doesn't model (p. VII-4-4, after Rosen). Padfield 3rd ed. §5.3.3 (p. 298) calls it "a common feature of current modelling standards". Fletcher's fix is an identified aerodynamic phase lag in the flapping equations, a fitted equivalent. A wake-distortion model is added to M9 (D-041).
- *Yaw rate to collective (Figs. 7 and 13), open:* `tools/plot_m7c_fletcher.py` also compares r per inch of collective (`m7c_fletcher_yaw.png`) against the reviewed flight curves (DATA_GAPS.md). Every reading Fletcher drew is compared, since these figures have no coherence panel: 0.53–9.5 rad/s in hover and 3.2–9.6 rad/s at 80 kt. Eyas with the servo, mean / RMS:

  | | magnitude [dB] | phase [deg] |
  |---|---|---|
  | hover | −3.8 / 6.6 | +42.8 / 59.2 |
  | 80 kt | −8.1 / 8.4 | +24.3 / 25.5 |

  - *Hover:* Eyas matches near 0.5–1 rad/s, then falls off faster than the flight data (about 10 dB low by 5–8 rad/s), and the flight lags up to about 110° more. Fletcher (p. VII-4-4) puts the UH-60's yaw/heave coupling in hover down to rotor speed and engine/governor dynamics, which his identified model and BE model B include and Eyas doesn't (M9). This is the baseline the M9 drivetrain must improve. *Since M9 Stage B* (GenHel's T700s, D-053) the plot shows the aircraft on its engines: the magnitude comes closer (−2.5 / 4.6 dB) and the phase goes further off (+96 / 122°); see the M9 Stage B entry.
  - *80 kt:* Eyas is 8 dB low across the band (on the engines since M9 Stage B: −10.0 / 10.1 dB, phase −103 / 104°). Fletcher finds the blade-element model he compares at 80 kt also mismatched, "greater than in hover", and his identified model fits without drivetrain dynamics, so the drivetrain "is not the source of the problem" (p. VII-4-11). Parked (D-041). Eyas's tail rotor has no torque yet (DATA_GAPS.md, M9), but that is a power term, not this response's.
- *Not compared:* the heave responses to pedal (Figs. 8 and 14), which V2 doesn't list.

### V2: rotor parameters against Fletcher's Table 1 (reported)

Fletcher's Table 1 (p. VII-4-9, PDF p. 11; read from the scan at 400 dpi, clear) compares his hover-identified flapping parameters with theoretical values. His notation (p. VII-4-1) defines M_β and I_β as the blade's first and second flapwise mass moments, in slug·ft and slug·ft², so M_β/I_β is in 1/ft. He doesn't say how the theoretical values were computed.

| | Eyas | Fletcher theoretical | Fletcher identified (hover) |
|---|---|---|---|
| M_β/I_β [1/ft] | 0.0573 | 0.0573 | 0.0321 |
| γ, textbook form ρ a c R⁴/I_β | 10.1 | 8.06 | 8.62 |
| γ, effective (Eyas's lifting span) | 7.87 | | |
| δ3 [deg] | 0 | 0 | 7.84 |
| aerodynamic phase lag ψ_a [deg] | none | n/a | 38.8 |

- *M_β/I_β:* Eyas's blade is GenHel's (S_β = 86.7 slug·ft, I_β = 1512.6 slug·ft², CR-166309 p. 5.1-49), and its ratio equals Fletcher's theoretical value to the printed digits, so his theory starts from the same blade data. His identified value is 44 % lower; he calls it "reasonably close".
- *Lock number:* Fletcher's 8.06 is the textbook form at sea level with Eyas's (GenHel's) chord, radius and I_β and a lift slope of 5.72/rad, i.e. 0.1/deg (1.4087 × 5.72). Eyas takes its lift slope from the SC1095 tables, which give 6.45/rad at low Mach rising to 6.96 at Mach 0.5. Weighted along the span by r(r − e)² at each section's hover Mach number, that is 7.20/rad, so the textbook form gives 10.1. But Eyas's blade lifts only from the 5.08 ft root cutout to the tip-loss radius (0.97 R) and flaps about a 1.25 ft hinge offset. The Lock number equivalent to the aerodynamic flap damping Eyas actually computes, γ_eff = 4ρ∫a(r) c r (r − e)² dr / I_β over that span, is 7.87. That is 2.4 % below Fletcher's theoretical value and 9 % below his identified one, within the 7 % spread between those two.
- *δ3 and ψ_a:* Eyas, like GenHel, has no pitch-flap coupling. Fletcher identifies 7.84°, a stabilizing equivalent coupling that "contributes significantly to the measured response", and the 38.8° aerodynamic phase lag that stands in for the wake distortion (the off-axis limitation above). Both are identified equivalents, not measured hardware, so neither is adopted.

### V2: mode character (reported)

`eyas_m7c_fletcher` also writes the condensed 6-DOF model's modes and derivatives at the same two conditions (`m7c_fletcher_modes.csv`, `m7c_fletcher_derivatives.csv`), and the averaged model's roots below 3 rad/s against the Floquet exponents of the free flow map (`m7c_fletcher_floquet.csv`). Fletcher gives no eigenvalues or stability derivatives (his Tables 1 and 2 list rotor parameters only, and he puts his identified models' validity at 0.3–0.5 to 20 rad/s), so the comparison is with the mode character Padfield 3rd ed. §4.3.3 describes for the Lynx, Bo105 and Puma.
- *Floquet:* every root below 3 rad/s is within 0.0004 rad/s of its Floquet exponent in hover and 0.0025 rad/s at 80 kt, so the averaged model is adequate for these modes.

| Hover | λ [1/s] | ω_n [rad/s], ζ | Padfield |
|---|---|---|---|
| Coupled pitch–roll oscillation | 0.202 ± 0.354i | 0.41, −0.49 (doubles in 3.4 s) | Unstable phugoid-type oscillation, coupled longitudinal/lateral, doubling in about 2.5 s for the Lynx and Bo105 (p. 206); "naturally unstable in hover" (p. 211) |
| Lateral phugoid | −0.170 ± 0.465i | 0.49, 0.34 | Similar frequency, stable or almost stable through its yaw content (p. 214) |
| Heave and yaw (labelled yaw subsidence) | −0.242 ± 0.065i | 0.25, 0.97 | Heave subsidence set by Z_w (−0.29 here) and yaw by N_r (−0.24); the two near-equal roots couple, through the rotor torque, into a heavily damped pair. Fletcher (p. VII-4-4) finds strong UH-60 yaw/heave coupling in hover through rotor speed and the engine, which Eyas doesn't model yet (M9) |
| Pitch subsidence | −1.03 | | Set by M_q (−0.89) (p. 211) |
| Roll subsidence | −6.03 | | Set by L_p (−6.00) (p. 214) |

| 80 kt | λ [1/s] | ω_n [rad/s], ζ | Padfield |
|---|---|---|---|
| Dutch roll | −0.337 ± 1.093i | 1.14, 0.29 | O(1 rad/s) (p. 215) |
| Spiral | −0.073 | | O(0.1 rad/s) (p. 215) |
| Roll subsidence (two, one with pitch) | −2.83, −2.37 | | Set by L_p (−3.20); the on-axis roll response above 1.5 rad/s, which L_p dominates, matches the flight data (V2 above) |
| Phugoid-type divergence (labelled speed subsidence) | **+0.289** | doubles in 2.4 s | Articulated Puma: "a classical short-period pitch/heave oscillation and neutrally stable phugoid" (p. 212) |
| Oscillation | −0.201 ± 0.261i | 0.33, 0.61 | |

- *Hover:* every feature Padfield describes is there, with the expected signs and sizes.
- *80 kt, open:* the longitudinal set differs. Eyas has a real divergence where Padfield's articulated example has a neutrally stable phugoid, and where the flight pitch response shows a lightly damped resonance near 0.4 rad/s (Fig. 4). The cause is negative speed stability: M_u = −0.0085 (rad/s)/(m/s), where Padfield's examples are positive (p. 189).
  - *Padfield's low-modulus phugoid approximation* (Eq. 4A.44, p. 225; its small (X_q − W_e) term, which only affects damping, left out) on Eyas's own derivatives gives real roots +0.38 and −0.30. With M_u = 0 it gives a near-neutral oscillation, and with M_u = +0.005 one at 0.26 rad/s, ζ = 0.20. The manoeuvre-margin term M_q Z_w − M_w (Z_q + U_e) is +0.54, so the short period is stable; M_w = +0.0117 is destabilizing but not divergent on its own.
  - *Where M_u comes from* (scratch probe, quasi-steady: body held at the trim state with u or w perturbed by ±0.25 m/s, component states settled for 3 s, loads averaged over a revolution; the components sum to the condensed M_u with an effective inertia of 51 950 kg·m², I_yy being 52 210): in N·m per m/s, main rotor +317 (flapback, stabilizing, as Padfield p. 189 describes), stabilator −577, tail rotor −108, fuselage −72. For M_w: main rotor +1919, fuselage +1017 (both destabilizing), stabilator −2037, tail rotor −149.
  - *Why the stabilator is speed-destabilizing:* held at 4° trailing edge down, it carries a 345 N download in a rotor downwash of about 5.6 m/s, twice the momentum induced velocity (a developed wake). Its angle of attack rises 0.0050 rad per m/s of speed: 0.0024 because the same downflow is a smaller angle at higher speed, and 0.0034 because the wash weakens. The local downflow falls by 0.13 m/s per m/s, where momentum theory (wash ∝ 1/V) gives 0.136. This is Padfield's Eq. 4.62 (p. 191), α_tp = α_f − λ_tp/μ: the downwash angle at the tail shrinks with speed. A fixed tail's own contribution, proportional to its trim load (p. 189), would be +145 N·m per m/s. So Eyas's wash follows simple wake theory, and the speed instability is that theory applied to a held stabilator, not a code or data quirk. Whether the real aircraft's tail sees as much downwash at 80 kt is what the flight data question.
  - *Not one derivative:* shifting M_u, M_w or both in the condensed model doesn't reconcile the 80 kt pitch response below 1.5 rad/s (best phase 31° RMS with M_u made positive, but magnitude 5.3 dB RMS against 3.6 now). Fletcher reports the same for the blade-element model he compares at 80 kt: "the mismatch between BE model A and the flight test data below 3 rad/sec... the errors are very significant" (p. VII-4-11). Parked (D-041); a multi-parameter identification against the flight data is beyond M7c.

## M8a — Library finish: decoupling, timing, outputs, examples (2026-10-17)

M8a is the first of five M8 stages (D-042). It adds:
- host-rate decoupling tests on the full UH-60A;
- the timing benchmark (`bench/eyas_bench`);
- the long-run checks that replace "free of drift and NaNs" (D-042);
- output Tiers 2–4 (D-043; air data from Gracey, NASA RP-1046);
- a scripted driver and a reference adapter in `examples/`.

Tests: `tests/unit/test_realtime.cpp`, `test_outputs.cpp`, `test_air_data.cpp`; smoke tests `ExampleDrive.*`, `ExampleAdapter.Flies5s`, `Bench.Smoke`. Data from `eyas_m8a_cases` and `eyas_bench`, plots from `tools/plot_m8a.py`.

| Acceptance criterion | Status | Evidence |
|---|---|---|
| 60 Hz host against 1 kHz physics, bit-identical (spec) | ✅ | `RealTime.HostRateDecoupling`: the UH-60A trimmed at 60 kt, then 5 s with a ±3 % longitudinal doublet. 60 Hz frames (16 666 667 ns) against 1 kHz calls with each frame's control on its sub-steps: after every frame the full state and every physical output field (Tiers 1–4) are identical to the bit. |
| Jittered host frames, same total (spec) | ✅ | `RealTime.JitteredHostRate`: 1–40 ms frames, not whole steps (seeded), same test; the sub-step count is floor(Σ dt / step) after every frame (D-006). |
| Determinism; no allocation (spec §7) | ✅ | The two above, plus `RealTime.ZeroAllocationInFlight`: 120 frames of `advance()` and `outputs()` with the Tier 2–4 evaluation in flight allocate nothing. On the ground: `Uh60aAircraft.RealTimeRules`, which now includes the outputs. |
| 30-min run free of drift and NaNs (spec; D-042 (b)–(d)) | ✅ | See the long runs below. (a), free flight with the AFCS on, comes in M8c. |
| Timing budget (spec §7, D-005, D-042) | ✅ here; RHEL target in M8e | See timing below: worst `advance()` 0.65 ms idle against the 8.33 ms budget (8 %). Under full CPU load at equal priority, up to 7.8 ms (93 %) from scheduling. |
| Output Tiers 2–4 (spec §6.10, D-043) | ✅ | `Outputs.*`: hand computation of every field at three trims and on the ground; Gracey's Table A13 for the airspeed relation (`AirData.MatchesGraceyTableA13`, 20 entries to 0.05 kt; one printed entry is a misprint, confirmed by review, DATA_GAPS.md). |
| Reference adapter and test driver in `examples/` (spec) | ✅ | `ExampleDrive.Doublet10s`: trims at 60 kt, flies the 10-s doublet at 60 Hz, 601 rows, all finite. `ExampleAdapter.Flies5s`: a toy host interface in feet, knots and degrees, with an `EyasAdapter` behind it; the host code names the flight model in one line. |
| Earlier results unchanged | ✅ | The outputs evaluation is pure: `eyas_m6d_cases`, `eyas_m7_cases` and `eyas_m7c_cases` write byte-identical files on `main` and on this branch (Release, GCC 13.3), except M6d's timing line (below). |
| Swapping adapters needs no host change (spec) | M8e | The host team's acceptance; the example shows the pattern. |

**The long runs (D-042), 30 min of simulated time each, from `eyas_m8a_cases` (74 s in Release).**

![Held orbit](validation/m8/m8a_held_orbit.png)

- *(b) The held periodic orbit:* hover and 120 kt, 7734 revolutions each. The rotor, inflow and other component states come back to the trim's start within 1.4e-9 (hover) and 2.6e-10 (120 kt), in each state's own units. The deviation settles within the first few revolutions and stays constant to the last digit printed for half an hour: the orbit is stable and there is no secular drift.
  - A first attempt started the orbit from the state `Model::trim` installs, which includes the airframe's 4/rev vibration. Holding the body at that vibrating state gives a different orbit (deviation 5e-5 in hover, 7e-3 rad/s in a flap rate at 120 kt, also constant). The check uses `held_orbit_start()`, the trim's own orbit.
- *(c) The M2 dummy-rotor equilibrium:* every state unchanged to the bit for 30 min; sim time exact at every second.

![Parked](validation/m8/m8a_parked.png)

- *(d) The free UH-60A parked on its gear,* brakes set, dropped from 2.2 m:
  - It lands at 0.27 s and settles within about 40 s.
  - From then to 30 min, the CG stays within 3 µm of its 60-s position, and the attitude within 5e-6 rad.
  - What is left is the rotor's 4/rev vibration on the tyres and struts: |v| ≈ 2e-5 m/s, |ω| ≈ 2e-4 rad/s.
  - The quaternion norm error before renormalization is 2.2e-16 per step.

**Timing (`eyas_bench`, Release, GCC 13.3, AMD Ryzen 7 9800X3D, 16 hardware threads, WSL2; the development machine, not the target).** 3600 frames at 60 Hz per case. A step is 1 ms of physics, a frame 17 steps plus the output evaluation. [µs]:

| Case | step() mean / p99 / max, idle | advance() mean / p99 / max, idle | advance() max, 15 busy threads | advance() max, 16 busy threads |
|---|---|---|---|---|
| Hover | 12.5 / 24.4 / 183 | 213 / 278 / 648 | 5183 | 6203 |
| 120 kt | 12.3 / 24.1 / 232 | 211 / 312 / 477 | 4790 | 4320 |
| Ground | 12.8 / 25.0 / 124 | 217 / 262 / 498 | 7777 | 4603 |

![Timing](validation/m8/m8a_bench.png)

- *Idle:*
  - A frame takes 0.21 ms on average, 2.6 % of the 60 Hz frame. Its worst case is 0.65 ms, 8 % of the 8.33 ms budget.
  - The output evaluation costs about 5 µs per `advance()` call (D-043), within the frame-to-frame noise here (17 × 12.5 µs = 213 µs; frames 211–217 µs). Called once per 1 ms step, as the M6d cost line does, it shows: 13 µs per call on `main` against 18 µs now.
- *Under load* (N threads spinning at the same priority):
  - The mean frame rises to about 0.33 ms, from shared cores and caches.
  - The worst frames rise to 4–8 ms. That is the scheduler preempting the simulation thread for a time slice, not Eyas's arithmetic: the worst `step()` reaches 3–6 ms under load, against 0.2 ms idle.
  - Here the D-005 budget is just met (93 % in the ground case with 15 busy threads).
  - This is D-005's point that scheduling jitter is the host's job. Thread priority (SCHED_FIFO), a reserved core or the RHEL real-time kernel would remove it. **For M8e:** measure on the RHEL 9 target with the host's real thread setup, not with equal-priority spinners.
- *Why the verdict is the measured frame (D-042):* worst `step()` × 17 would give 90 ms under load, but a preemption doesn't recur in each of a frame's steps; idle, the product (3.9 ms) still bounds the measured frame.

**Air data.** Gracey, NASA RP-1046 (1980), Ch. III (read from page images: Eqs. 3.14, 3.16, 3.20 on book pp. 15–16; "altimeters are calibrated in geopotential feet", p. 12; Table A13, p. 246):
- The calibrated airspeed inverts Eq. 3.16 exactly.
- Run backwards from Table A13's true airspeeds, it gives the table's calibrated airspeed within its rounding (0.05 kt) at every entry used from sea level to 40 000 ft, both ISA layers. The exception is V_c = 100 kt at 20 000 ft, printed 137.2 kt, which is above V_c/√σ = 137.0 kt and so can't be right; the equations give 136.5 kt. A person confirmed the page prints 137.2, so it is a misprint in the source (DATA_GAPS.md).
- On the UH-60A at 1000 m, ISA + 15 K and 100 kt: V_c exceeds V_e by under 1 %; the pressure altitude is below the geopotential altitude, as on any hot day.
- The adapter example at 60 kt TAS and 1000 ft shows 59.1 kt IAS and 1000 ft pressure altitude.

## M8b — Desktop pilot station (2026-10-17)

M8b is the second M8 stage (D-042): a pilot station in `examples/pilot_station/` (D-044), host code on the public API.
- It flies the aircraft by hand at a fixed 60 Hz host frame, paced to the wall clock.
- It shows the flight in its own window (GLFW and OpenGL 2.1), and optionally in FlightGear over UDP.
- Its inputs are a gamepad or joystick, mouse and keyboard, or FlightGear's stick forwarded from Windows.
- It records each flight as an `eyas_drive` control file.
- It adds one library call, `Model::from_file(path, GeodeticOrigin)`, so the host places the aircraft (D-044).

Tests: `Model.OriginOverride*` and `Model.InvalidOriginOverrideThrows` (`tests/unit/test_model.cpp`), `eyas_station_tests` (31 cases), and the smoke tests `ExampleStation.RecordingReplaysExactly` and `ExampleStation.ScriptRunsFromTheStartPoint`.

| Check | Status | Evidence |
|---|---|---|
| The host's origin replaces the config's, and nothing else | ✅ | With the config's own origin passed in, 60 frames are bit-identical to `from_file(path)`. Another origin moves Tier 1 by the D-017 map from it; invalid origins throw. The config's block stays optional under an override, but an invalid one is rejected by both calls. |
| A recorded flight replays exactly (from an airborne start, brakes unchanged) | ✅ | `ExampleStation.RecordingReplaysExactly`: the station flies the 10-s doublet headless from the 60 kt start at another origin (51.47° N, 0.45° W, 25 m), recording it; `eyas_drive` runs the command in the recording's header; the two state histories (602 lines, every `AircraftState` field at 17 digits) are byte-identical. |
| Control mapping and force trim | ✅ | `StationControls.*`: deadband and expo; absolute axes from the start; the lever collective; rate integration (exact to 1e-14 for a constant input); every control within its D-024 range to an ulp; force trim holds the release point while the stick returns, then flies about it; trim reset; brake toggle. |
| FlightGear link | ✅ (pose and lines, live); stick values pending a person at the pad | `StationFlightGear.*`: the control line round-trips; short, long, non-numeric and NaN lines are rejected; the FDM line has the protocol's 22 fields in FlightGear units; the three FlightGear files are well-formed and match the code's field counts; a UDP loopback through `FgLink` delivers an FDM line and reads back the newest control line. Live run below. |
| Crash handling | ✅ | `StationStarts.CrashFreezesUntilRestart`: collective down from the 100 ft hover; the station calls the crash, stops advancing, and a start clears it (D-044, SPEC_REVIEW #34). |
| Start points | ✅ | `StationStarts.*`: on the pad the aircraft carries weight on its wheels and has settled (< 1e-3 m/s after the 40 s D-042 (d) settle); the three trims fly at their airspeed (within the rotor's 4/rev, 2e-3 m/s) 100 ft above the ground; a restart replays bit for bit. |
| Real-time rules on the frame path | ✅ | `StationRealTime.FramePathDoesNotAllocate`: map the controls, `advance()`, send the FlightGear line and poll for controls, 30 frames, no allocation. |

![Chase view at 60 kt](validation/m8/m8b_chase_60kt.png)

![Ground observer, hover](validation/m8/m8b_observer_hover.png)

**The window** (Release, WSLg on the development machine; screenshots from `--screenshot`): the chase view at the 60 kt start and the ground observer's view of the hover start, 100 ft up. The ground is tiled so the per-vertex fog stays right near the eye (a first version drew one 40 km quad, which fogged to sky colour everywhere). The blocks and masts give scale and parallax; the rotor disc is tilted by the multiblade flapping. The HUD blanks the track below 1 kt of ground speed and the angle of attack and sideslip below 5 kt of airspeed, where D-043 leaves blanking to the display.

**FlightGear, live** (FlightGear 2024.1 on Windows, the station in WSL2 in NAT networking, 2026-10-17):
- *Two faults found and fixed.* FlightGear first rejected both protocol files as "not well-formed": their comments contained `--` (from `--fdm=null`), which XML forbids. A test now checks every FlightGear file for it.
  - FlightGear also doesn't publish raw stick values: the `/devices/status/joysticks/joystick[0]` nodes stayed empty.
  - `eyas_joystick.xml`, loaded with `--config`, now binds joystick 0's axes and buttons to them. FlightGear loads it (its log reads the property file, and `/input/joysticks/js[0]` carries the bindings).
- *Pose into FlightGear:* a 20 s flight from the 60 kt start at 37.6188° N, 122.3756° W (ground 4 m MSL). FlightGear's own properties, read back through its web server, showed:
  - the aircraft 618 m north of the origin (60 kt × 20 s = 617 m);
  - 113 ft MSL (100 ft above the 13 ft ground);
  - heading 360.0° and 59.9 kt.
- *Lines from FlightGear:* 1188 received in 20 s (FlightGear sends at 60 Hz), none rejected.
- *Timing with FlightGear rendering on the same machine:* worst `advance()` 0.59–0.61 ms, no overruns after the first frame, sim time / wall time 1.0000.
- *Pending:* stick values arriving from the Xbox pad (needs someone moving the sticks), the Xbox axis numbers in the `fg` map, and a piloted session for the latency and overrun figures. Pilot comments belong to M8e (D-042).

**Hands-off, the hover holds and then diverges, as the bare UH-60A should.** From the hover start with the controls fixed at trim (`--input none`):
- the aircraft holds within 0.01 m and 0.1° for about 50 s (the trim's residuals);
- it then departs through the unstable modes (M7c);
- at 85 s it meets the ground rolled 72°.

That impact is what exposed SPEC_REVIEW #34: the state diverged to finite garbage within three frames of the gear touching. The station's crash rule now stops it at the touch.

**Code review fixes (2026-10-17).** The review's findings were each checked by reproduction before fixing:
- a script flown from the pad skipped every row before 40 s;
- brake changes weren't recorded, though the header promised an exact replay;
- the station and `eyas_drive` counted frames with different formulas, and kept two copies of the frame period (853 durations disagreed at 50 Hz and 1704 at 100 Hz in a sweep from 0.001 to 200 s; none at 60 Hz);
- an `--origin` with spaces broke the replay command;
- `--listen 70000` bound port 4464;
- a negative `--hover-height` crashed on the first frame;
- fixed test ports collided with another process holding them;
- the origin override accepted a misspelled origin block that plain `from_file` rejects;
- the heavy tests ran under ASan (58–81 s each).

All are fixed with tests (D-044 verification). Two points weren't acted on as stated:
- *The socket error text read after `freeaddrinfo()`:* theoretical, since glibc ≥ 2.33 leaves `errno` alone there. It was tidied anyway.
- *A `from_string` overload with an origin:* left out, since no caller needs it yet.

**First flights with a HOTAS (2026-10-17; developer, not a pilot evaluation).** A Logitech/Saitek X-56 (stick and throttle) on Windows, through FlightGear (`maps/x56_fg.map`), in five flights from the pad, the longest about 4 min, up to 145 kt.
- *Before the fixes:*
  - FlightGear forwarded only one device, so several-device support and the `device:index` map syntax were added.
  - The first lift-off jumped the collective from 0.2 to 0.5. The untouched throttle reads 0 (mid travel) until it moves, because FlightGear doesn't report an axis before then. This led to lever pickup (COLL SYNC). A code review then found, and a reproduction confirmed, that its first version still dropped the 60 kt start's collective to 0 on FlightGear's first real reading, and didn't re-arm after an input loss; both are fixed (D-044).
  - The pad's half right pedal, from the parked case, spun the aircraft at lift-off. The pad now starts at the hover trim's pedal and cyclic.
- *The station:*
  - In the last flight the worst `advance()` was 0.68 ms.
  - There was one overrun over the session (the first frame), with FlightGear rendering on the same machine.
  - It received 80 445 control lines from FlightGear and rejected 156 (0.2 %); the newest good line is used, so no input was lost. A 30-s capture afterwards had no bad line, so the cause is not yet known. The station now prints the last rejected line at exit.
- *What the developer reported:* "during forward flight it felt like the nose wanted to keep going up". The recording agrees with the model's trims. The forward cyclic held at 100–130 kt was +0.31 to +0.39, against level-flight trims of +0.22 (100 kt) to +0.32 (140 kt), more when accelerating or descending. The model needs about 0.2 more forward stick between 60 and 80 kt, from the stabilator schedule (D-040). Its trimmed stick matches GenHel to within 1 % of travel (M7c). Other contributors:
  - flapback with speed (Padfield p. 189);
  - the bare aircraft's pitch instability: the unstable phugoid and M_w > 0 (M7c modes);
  - the missing pitch bias actuator and SAS (M8c, D-029).

  Nothing points to a fault; re-check after M8c. The open 80 kt speed-stability gap (M_u < 0, D-041) tends to pitch the nose down rather than up, so it isn't this.

## M8c — The flight control system (M8c1 and M8c2, 2026-10-18; M8c3 library, 2026-10-19)

M8c is the third M8 stage (D-042), the UH-60A's flight control system from GenHel CR-166309 §5.5 (D-029, D-045), in three PRs. M8c1 (this entry): the Fig. 5.1.2 sensors, the mechanical controls and mixer (Figs. 5.1.6(a)–(d)) with the rotor head limits (p. 5.5-20), the primary servos, and the dynamic stabilator (Fig. 5.1.7: airspeed schedule, pitch-rate and lateral-acceleration feedback, servo with its rate limit). M8c2 adds the SAS and the pitch bias actuator, M8c3 the FPS. One component, `genhel_fcs` (`src/control/`), drives both rotors' blade pitch and the stabilator through the D-003 data path. The data come from `data/uh60a/genhel/` (DATA_GAPS.md "UH-60A AFCS (M8c)").

Generator `eyas_m8c_cases` (`tests/validation/m8c_cases.cpp`), plots `tools/plot_m8c.py`; nightly in `full-verification.yml`.

| Check | Status | Evidence |
|---|---|---|
| Mixer and head limits against GenHel's rigging | ✅ | `Uh60aAircraft.ControlMapsAreGenHelsRigging`: blade pitch at 12 azimuths and the tail-rotor pitch for 400 random stick positions, against GenHel's mixer written test-side in inches and degrees (1e-7°), with the B1S limits exercised. |
| Servos, sensors and stabilator network against GenHel's diagrams | ✅ | `GenHelFcs.ServosAndSensorsAreGenHelsTransferFunctions`, `AccelerometerSensesTheSpecificForceAtItsStation`, `StabilatorServoRateLimit`, `SteadyStatesAreAHeldBodyEquilibrium`: every second-order block on its input, the airspeed lag on the equivalent airspeed, IHT0 in GenHel's degrees and knots, and the accelerometer's specific force (xdot0 + G a) against an independent point-acceleration formula. Flipping the gravity sign in the reference fails two of them. Anchor: at rest, rolled 10° right, the accelerometer reads −g sin 10°. |
| The flying config is GenHel's | ✅ | `GenHelFcs.Uh60aConfigIsGenHels`: the servo, gyro, accelerometer and stabilator constants and the transducer station (FS 307, WL 263) from the shipped config, in GenHel's units. |
| Acceleration-coupled states linearize exactly | ✅ | `Linearize.AccelerationCoupledStatesAreExact` (D-045). |
| Real-time rules | ✅ | `GenHelFcs.DerivativesDoNotAllocate`; the whole-aircraft allocation and determinism tests run with the flight control system in. |
| V1 trims against Ballin (M7c gate) | ✅ | `BallinTrim.*` pass. The GenHel-constraint trims (zero roll above 50 kt) are unchanged to the printed digits, since at zero roll the accelerometer reads no lateral force. Only the zero-sideslip comparison trims move: by up to 5.2 % of longitudinal travel and 2.4° of stabilator at 160 kt. |
| V2 against Fletcher with the servos in the model | ✅ | `eyas_m7c_fletcher` now takes the primary servos from the model and divides them out for the no-servo curves. Both sets equal M7c's to the CSV's six digits, as they must for a linear unit-gain servo, so the M7c figures stand. |
| L3 with the lateral-acceleration feedback | ✅ | Below, resolved by the D-045 amendment. |
| Start-up transient | ✅ | `Trim.FcsStartTransientIsSmall`: under 0.002° of stabilator in the first 2 s (measured 0.0004° at 120 kt; 0.026° before the amendment). |
| A disturbed flight control system settles | ✅ | `Trim.FcsKickDiesOut`, below. A pitch-rate gain of −0.6 s in place of +0.16 fails it while the trim still converges, which is the case it guards against. |
| Free flight with the AFCS on, 30 min (D-042 (a)) | M8c3 | With the stabilator alone the aircraft keeps its unstable phugoid (modes below), so a 30-min free flight diverges whatever the integrator does. The check waits for the SAS and FPS. |

**Trims.** Level flight, ISA sea level, 16 638 lbm, zero sideslip, with the lateral-acceleration feedback on and zeroed:

![Trims](validation/m8/m8c_trims.png)

- The stabilator rests on its 39° stop up to 30 kt and follows its airspeed schedule down to about 3° at 80 kt.
- At zero sideslip a single-rotor helicopter trims with about 1–3° of roll, so the accelerometer reads −g sin φ, 0.18–0.58 m/s². Through the feedback (−1.299 deg per ft/s² above 60 kt) that moves the stabilator by up to 2.5° at 150 kt. The aircraft then trims 1.7° more nose-up, with forward stick up by 0.11 (5.6 % of travel). With the ball centred the accelerometer reads zero, since the ball shows the same lateral specific force. So the shift belongs to the zero-sideslip trim condition, not to how the aircraft is flown. GenHel's zero-roll trims (the V1 gate) read zero too.

**Modes.** Condensed 6-DOF model, with the dynamic stabilator and with the stabilator held at its trimmed incidence (the same trim):

![Modes](validation/m8/m8c_modes.png)

- *Speed stability.* Held, the aircraft has an unstable real root at 60 to 120 kt: +0.39, +0.18 and +0.15 1/s. That is M7c's parked M_u < 0 gap (D-041), which came from the held stabilator in the rotor downwash. With the airspeed-scheduled stabilator that root is gone. A mildly unstable phugoid remains: +0.050 ± 0.369i at 60 kt and +0.022 ± 0.140i at 90 kt.
- *Dutch roll.* With the stabilator live, its damping drops: ζ 0.31 to 0.27 at 60 kt, 0.28 to 0.07 at 120 kt, 0.21 to 0.05 at 150 kt. That is the lateral-acceleration feedback coupling sideslip into pitch. Nothing checks it against data yet; the SAS (M8c2) adds yaw damping. **Corrected in M8c2:** these modes made the flight control system's states quasi-static, the lateral accelerometer (ω_n 6.3 rad/s) among them, which drops part of the loop's lag. Kept dynamic, the drop is larger: ζ 0.019 at 120 kt and 0.000 at 150 kt (M8c2 below).
- These are the flying config's modes; M7c's comparison with Fletcher holds the stabilator, as Fletcher's flights did.

**Start-up transient and a kick.**

![Start](validation/m8/m8c_start.png)

![Kick](validation/m8/m8c_kick.png)

- *Start (D-045 amendment).* From the trim's state, the stabilator's change in free flight with the sensors started on their free-flight orbit, against their held-body values. At 150 kt the first swing falls from 0.062° to about 0.0014°. What remains grows slowly, the bare aircraft's own drift.
- *Kick.* The stabilator servo kicked by 0.5°, the accelerometer by 0.3 m/s² and the pitch gyro by 0.01 rad/s. It swings to −0.6° and settles in about 1.5 s. After that it stays within 0.09° of the unkicked flight, the slow airframe motion fed back through the gyro. The gate is ±0.25° after 1.5 s.

**L3 with the stabilator's lateral-acceleration feedback (resolved 2026-10-18, D-045 amendment).** The gate failed with that feedback on: c0 was 2.1e-4 at 60 kt (collective doublet; floor 1e-4) and 1.1–3.3e-3 at 150 kt (floor 1e-3). There were two causes.
- At 150 kt the trim started the flight control system's sensors at their held-body values, off the 4/rev they see in free flight. Every free flight then carried a decaying transient, and the perturbed flights' cross term with it reads as a leftover. The trim now starts these states on their free-flight orbit.
- At 60 kt the collective doublet's error was not yet straight in the amplitude, which biased the fitted intercept. Its largest amplitude is now a quarter of the other controls'. On a √2 amplitude grid down to 1/90 of the old largest, e/a stays within 2.5–3.1 with the feedback on and with it zeroed. So there is no leftover and no corner crossing (either would make e/a grow without bound as the amplitude shrinks). The ±10 % wander in e/a is in both configs; its source is not identified.

`Linearize.PeriodicModelMatchesTheNonlinearResponse` now runs on the flying config with the feedback on and gates |c0|. Worst |c0| over the 12 cases:

| Condition | Worst \|c0\| | Floor |
|---|---|---|
| hover | 2e-6 | 1e-4 |
| 60 kt | 1e-5 | 1e-4 |
| 80 kt (probe, not gated) | 2e-5 | 1e-3 |
| 150 kt | 4.3e-4 (p) | 1e-3 |

**The L3 sweep, regenerated** (`eyas_m7b_cases` on the flying config, every 10 kt; M7b's figure and figures are left as that milestone's record):

![L3 sweep](validation/m8/m8c_l3_sweep.png)

- Periodic model: ≤ 4.2e-5 up to 60 kt and ≤ 4.3e-4 above, except at 30 kt (12 %, the tail-rotor fin-blockage corner, as in M7b). M7b recorded 3.2e-5 and 5.4e-4.
- Averaged model in hover: 1.03e-3 (M7b: 1.04e-3).

**Open items:**
- *Dutch-roll damping* with the live stabilator (above). The two readings it rests on are confirmed from GenHel's own equations (DATA_GAPS.md, AFCS, 2026-10-18): KVYDT is in deg per (ft/s² · kt), and AYPS1 is the specific force at the transducer, as modelled. The drop is therefore GenHel's stabilator law as written, still unchecked against flight data, since Fletcher's lateral data were flown with the stabilator held. M8c2: with the SAS in, ζ is 0.33–0.44 at 90–150 kt; corrected value with the SAS off, ζ ≈ 0 at 150 kt.
- *The L3 misfit's noise allowance* (half the floor) can hide a 3 % curvature in e(a). Tightened to a tenth of the floor in M8c2.
- *The stabilator schedule's 147 kt node* sits 3 kt below the 150 kt gate trim. A hotter or higher 150 kt condition (EAS below 147 kt) puts V_F on its other slope; not a fault, but L3 at such a condition crosses it.

**Builds:** all 386 tests pass with GCC 13.3 (Release), all 387 with GCC 13.3 and Clang 18.1 (Debug), and all 386 in a Rocky Linux 9 container (GCC 11.5, Release), with and without GLFW. Without GLFW the pilot station's window-only variable broke the GCC 11 build under `-Werror`; it was on main, CI missed it because its Rocky job installs GLFW, and it is fixed here. Under ASan+UBSan the 31 new and changed tests pass (flight control, trim, aircraft, acceleration coupling, Ballin level flight), except `BallinTrim.ClimbAndDescentTrendsAgainstSikorskyGenHel`, stopped after 29 min (in CI's `SLOW_TESTS`; the nightly job runs it). clang-format is clean, and clang-tidy is clean on the changed `src/` files.

### M8c2 — SAS and pitch bias actuator (2026-10-18)

GenHel's SAS (Figs. 5.1.3(a)–(c): a digital and an analog channel per axis, the 2:1 gain switch, 5 % authority limits, washouts, the 60 kt switches and 2 s fade, the SAS actuators) and pitch bias actuator (Fig. 5.1.4) in `genhel_fcs`, switched by `Controls::afcs` (default off; D-045 M8c2 amendment). The pilot station starts with them on. Same generator and plot script as M8c1 (`m8c2_*` files).

| Check | Status | Evidence |
|---|---|---|
| Every SAS and PBA state against GenHel's diagrams | ✅ | `GenHelSas.ChannelsAreGenHelsDiagrams`: the rates of all 39 states and the six channel outputs and PBA position, at 400 random states, flight conditions and switch positions (both sides of 60 kt), against the diagrams written test-side in GenHel's units (deg, %, ft/s², kt, in); `InnerLoopInchesJoinTheMixer` (XAILS, XBILS, XBBAS, XPILS into Figs. 5.1.6(b)–(d)); `YawFadeRunsOverTwoSeconds`; `PitchBiasActuatorLimits` (rate limit, stop, off). |
| The digital holds stay within their limits | ✅ | `GenHelSas.DigitalHoldStaysWithinTheLimit`: a saturated channel reversing would read +15 % from the Padé; clamped after the hold, as a real hold's output never leaves its input's range (code review). |
| The zero-order holds' Padé | ✅ | `GenHelSas.HoldPadeMatchesTheZeroOrderHold`: against the hold's exact response, within 1° and 3 % up to 5 Hz (25 ms) and 1 Hz (0.1 s); worst 0.28° and 2.6 % (SAS), 0.15° and 1.7 % (PBA). |
| IC mode and switched off | ✅ | `GenHelSas.SteadyStatesAreAHeldBodyEquilibrium` (every rate zero to 1e-9, any switches); `SwitchedOffAddsNothing` (the M8c1 aircraft's states, exactly). |
| The flying config is GenHel's | ✅ | `GenHelSas.Uh60aConfigIsGenHels`: every SAS and PBA constant of the shipped config, in GenHel's units. |
| Real-time rules | ✅ | `GenHelSas.DoesNotAllocate`; `Trim.ModelStartsFromTrimAndKeepsTheRealTimeRules` now trims and flies with the SAS and PBA on (no allocation, bit-identical repeats). |
| In trim the SAS and PBA move only the stick | ✅ | `Trim.SasAndPbaMoveOnlyTheStick` (0, 100, 150 kt): attitude, velocity and blade pitch equal to 1e-6; the stick moves by GenHel's steady SAS and PBA output computed test-side from the trimmed φ, θ, Y.SENS and VXBIKT, to 1e-5 in. |
| The SAS damps the Dutch roll (nonlinear) | ✅ | `Trim.SasDampsTheDutchRoll`, below. |
| L3 with the SAS and PBA on | ✅ | `Linearize.PeriodicModelMatchesTheNonlinearResponse` adds hover and 150 kt with them on (not 60 kt, below). Worst \|c0\| 9e-7 (hover), 4.4e-4 (150 kt); floors 1e-4 and 1e-3. |
| L3 misfit noise allowance | ✅ | A tenth of the floor (was half; M8c1 open item). Worst misfit 0.118 (150 kt collective, SAS off) against 0.15; with no allowance at all it is 0.26. |
| Against flight data | — | No SAS-on flight data in the library (Ballin's and Fletcher's were flown with the SAS off or the stabilator held). The AFCS is verified against GenHel's design, not the aircraft. |

**In trim** (level, ISA sea level; on minus off):

![Trim stick](validation/m8/m8c2_trims.png)

- *Longitudinal:* the PBA puts the stick 10–14 % of half travel aft below 100 kt, 2 % at 150 kt (KTHBA on the nose-up hover attitude, the −4.9 % bias, KVXBA above 80 kt). At 100 kt that is 4.7 % of full travel; M7c estimated 4.3 % against GenHel's curve, and the flight test's PBA-on and PBA-off sticks differ by 4.0 % (VALIDATION.md M7c, pitch bias actuator).
- *Lateral:* 0.6–2 % of half travel left, the roll attitude paths (KPHRD, KPHIRA KPHR1 KPHF1) on the 1.5–3.4° of roll a zero-sideslip trim holds.
- *Pedal:* zero to 60 kt; above it the yaw lateral-acceleration paths (KLAYYD, and KLAYRR on KLAYF1·Y.SENS) hold up to 4.4 % of SAS at 150 kt, 9 % of half pedal travel. That step at 60 kt is GenHel's hard switch; a level trim at 60 kt has 0.95° of pitch, so its body-x airspeed is 59.99 kt, just below it.
- The aircraft's own trim (attitude, blade pitch) is unchanged, to the trims' convergence.

**Modes** (90–150 kt; every flight control state kept dynamic, `condense(m, retained)`; the M8c1 method, all quasi-static, shown for SAS off):

![Modes](validation/m8/m8c2_modes.png)

| Speed | Dutch roll ζ (ω_n): SAS off, quasi-static (M8c1) | SAS off, dynamic | SAS and PBA on | Phugoid ζ: off / on |
|---|---|---|---|---|
| 90 kt | 0.164 (1.19) | 0.151 (1.11) | 0.441 (0.79) | −0.205 / 0.307 |
| 120 kt | 0.067 (1.46) | 0.019 (1.35) | 0.371 (1.03) | (long-period lateral-longitudinal, −0.25) / 0.367 |
| 150 kt | 0.051 (1.73) | 0.000 (1.60) | 0.327 (1.26) | (−0.83) / 0.847 |

- *Correction to M8c1.* Keeping the flight control states dynamic, the bare aircraft's Dutch roll with the live stabilator is neutrally damped at 150 kt, not ζ 0.05. The quasi-static accelerometer (ω_n 6.3 rad/s against a 1.6 rad/s Dutch roll) dropped part of the lateral-acceleration loop's lag. The M8c1 figure (`m8c_modes.png`) is left as that milestone's record.
- *With the SAS on,* the Dutch roll has ζ 0.33–0.44 and the unstable phugoid becomes stable (the PBA's attitude and airspeed terms). In hover one lateral phugoid stays unstable (ζ −0.10, −0.47 with the SAS off): the SAS damps rates; holding attitude is the FPS's job (M8c3).
- Not at 60 kt: the trim sits on the SAS's switches, where the linearization's u perturbation crosses them.

**Dutch roll in free flight** (yaw rate after a 0.05 rad/s kick, kicked minus unkicked):

![Kick](validation/m8/m8c2_kick.png)

- With the SAS on the swing falls 52×, 30× and 68× over 6 s at 90, 120 and 150 kt; with it off 4.2×, 1.8× and 1.9×. `Trim.SasDampsTheDutchRoll` gates 4× with it on (the linear ζ ω_n predicts 9× or more) and fails if it is off at 120 and 150 kt. *Since D-048 (2026-10-20)* the bare aircraft falls 6.5× and 11.8× at 90 and 120 kt, so the missing-SAS check is made at 150 kt only.

**Open items:**
- *Flight data with the SAS on.* None in the library. The USAAEFA Project 79-24 report (DATA_GAPS.md) may have SAS-on responses.
- *Interpretations* (DATA_GAPS.md, AFCS, M8c2): the yaw analog roll-rate cross-feed's input (equivalent gain 0.060 %/(deg/s) against the digital 0.138), the KLAYRR/KLAYF1 units, the fade-out, VXBIKT.
- *Trim cost.* Every SAS and PBA state is solved on the free-flight orbit, so each trim takes about 1.85× as long.

**Cost.** `eyas_bench` (Release, GCC 13.3, the development machine): 13.6–14.0 µs mean per physics step (M8a: 12.5 µs, before the whole flight control system); worst `advance()` 556 µs against the D-005 budget of 8333 µs. Computing the SAS signals once per derivative call (code review) saved about 0.2 µs per step.

**Builds:** all 404 tests pass with GCC 13.3 (Release) and all 405 with GCC 13.3 and Clang 18.1 (Debug). In a Rocky Linux 9 container (GCC 11, Release) all 403 passed with and without GLFW before the code-review fixes; CI's Rocky job covers the final state. Under ASan+UBSan the 95 new and changed tests pass, and the 34 the review fixes touched again after them (SAS, flight control, units, trim, the linearization tests touched, aircraft data, the station's controls, mapping and recorder). `Trim.Uh60aTrimsFromHoverTo150Knots` now takes 14 min there (13.7 s in Release, with the longer trims), so it and the two new SAS trim tests join CI's `SLOW_TESTS`; the nightly job runs them. clang-format is clean; clang-tidy is clean on the changed files, and CI now runs it on `src/control/` too (M8c1 left it out).

### M8c3 — FPS (2026-10-19)

GenHel's flight path stabilization (Figs. 5.1.5(a)–(c), text p. 5.5-3) in `genhel_fcs`: pitch and roll attitude hold, airspeed hold above 30 kt, heading hold, and turn coordination with the turn switch on, each through a 0.1 s hold and a trim actuator of 100 % authority. Switched by `AfcsSwitches::fps`, `fps_turn` and `trim_release` (default off; D-045 M8c3 amendment). The yaw integrator is wired the physical way, the reverse of the figure as printed (DATA_GAPS.md, AFCS); the tests fly both. The pilot station's switches and turn-switch latch come in the next PR. Same generator and plot script (`m8c3_*` files). No FPS-on flight data is in the library, so the checks are against GenHel's diagrams and the FPS's stated jobs.

| Check | Status | Evidence |
|---|---|---|
| Every FPS state against GenHel's diagrams | ✅ | `GenHelFps.ChannelsAreGenHelsDiagrams`: the rates of all 16 states, POTEMP, ROTEMP, YOTEMP and the trim actuators' inches, at 600 random states, flight conditions and switch positions per wiring (both sides of 30 kt, inside the collective fade), against the figures written test-side in GenHel's units (`tests/support/genhel_fps_reference.hpp`). Breaking the heading wrap on purpose fails it 1139 times. `TrimActuatorInchesJoinTheMixer`, `TrimActuatorLimits` (gain, ±10 %/s, stop at ±100 %, back to centre when off). |
| Synchronizers, trim release, turn switch | ✅ | `GenHelFps.EngagingGivesNoVisibleKick` (engaging in a 3°/s turn holds the heading 0.03° back, 0.07 % of yaw command), `TrimReleaseResynchronizes`, `TurnSwitch` (both wirings). |
| IC mode, steady kinds, switched off | ✅ | `GenHelFps.SteadyStatesAreAHeldBodyEquilibrium`, `SteadyKinds` (references and integrators held), `SwitchedOffAddsNothing` (the M8c2 mixer commands exactly). |
| The flying config is GenHel's | ✅ | `GenHelFps.Uh60aConfigIsGenHels`. KRPRO's last digit is unclear; a reviewer leaned to −0.189, which is used (DATA_GAPS.md). |
| Trims with the FPS on | ✅ | `Uh60aFps.TrimsMoveOnlyTheStick`: bit for bit the SAS-only trims (the references hold the trim). With the turn switch on only the pedal moves, by KLAYYO's share of the steady lateral acceleration, and `TrimResult::held_drift_rate` reports the lateral-acceleration integral's rate (below). |
| The FPS's jobs, flown | ✅ | `Uh60aFps.*`, below: attitude, airspeed above 30 kt (not below), heading in hover, turn coordination, a collective step. |
| Modes with the FPS on | ⚠️ | Below: the hover lateral phugoid the SAS left unstable is stable; a 5 rad/s mode of the stabilator's lateral-acceleration loop loses its damping at high speed (open). The engaged references have no dynamics of their own, so condensing them is refused by name (`Uh60aFps.CondenseNamesTheEngagedReferences`); the modes keep them dynamic. |
| D-042 (a): 30 min hands-off with the whole AFCS on | ✅ | Hover and 120 kt, through `Model::advance` at 60 Hz: finite, sim time exact, quaternion norm error ≤ 2.2e-16; the FPS holds the trim (below). `RealTime.AfcsFreeFlightFor60s` gates 60 s. |
| Real-time rules | ✅ | `GenHelFps.DoesNotAllocate`; `Trim.ModelStartsFromTrimAndKeepsTheRealTimeRules` now trims and flies with the SAS, PBA and FPS on (no allocation, bit-identical repeats). |
| Earlier results unchanged | ✅ | Full suite. The M8c1 and M8c2 data regenerate identical to the last printed digit; the M8c2 mode lists differ only in how two copies of a repeated filter pole (SAS off) are printed. |
| Against flight data | — | None with the FPS on in the library. |

**Trims** (level, ISA sea level):

![Trims](validation/m8/m8c3_trims.png)

- *FPS on, turn switch off:* every control and attitude equal to the SAS-only trim, from hover to 150 kt.
- *Turn switch on:* the pedal moves 2.8–9 % of half travel right, KLAYYO times the trim's steady lateral acceleration (zero sideslip leaves 0.85 ft/s² at 100 kt). The integral of that acceleration (KIAYYO) would retrim it at 0.38–1.25 %/s; trim holds the integrators (GenHel's IC mode) and reports that rate as the held drift. In a real turn the integral is what centres the ball (below).
- *Correction to M8c3 step 1:* trim now freezes held states through its held revolutions. Integrated, the drifting integral fed the yaw hold and trim actuator, and these turn-switch trims failed to converge (D-045 M8c3 amendment).

**Attitude hold** (hands-off after a 0.05 rad/s pitch-rate or 0.1 rad/s roll-rate kick):

![Attitude](validation/m8/m8c3_attitude.png)

- With the FPS the attitude returns to within 0.008–0.080° by 20–30 s (gate 0.1°); with the SAS and PBA alone the 60 kt kicks leave 0.79° and 0.29° then (gate 0.2°: the test fails without the FPS). At 120 kt the SAS alone returns them nearly as well, slowly.
- At 120 kt the FPS response rings at 5.4 rad/s for about 10 s. That is the lightly damped mode below, not the FPS's own loops.

**Airspeed** (stick held 0.01 forward; a 5 kt gust at 20 kt):

![Airspeed](validation/m8/m8c3_airspeed.png)

- Above 30 kt the airspeed integral (KIVXPO) takes out a steady stick offset: within 0.015 kt (60 kt) and 0.025 kt (100 kt) over 120–150 s (gate 0.1 kt), where the SAS-only aircraft settles 1.0 and 3.1 kt fast.
- Below 30 kt the airspeed reference tracks (VG30SW): the gust leaves the aircraft 5.4 kt fast after 40 s, its pitch within 2°. The SAS alone lets the low-speed phugoid diverge (38 kt fast by 20 s, the 30 kt speed mode +0.15/s in the SAS-only modes); the FPS's attitude hold removes it.

**Yaw** (hover, pedal held 0.05 right; a 10° turn at 100 kt with the trim release held for 8 s; a collective step at 20 kt):

![Yaw](validation/m8/m8c3_yaw.png)

- *Heading hold, the two wirings (D-045 M8c3, owner).* Physical: the heading settles at the ±1° deadband's edge, 0.95° (gate 1.05° over 10–90 s), where only the proportional path acts. As printed: the heading integral is synchronized to zero and the lateral acceleration's integral runs instead, so the heading drifts 35° in 90 s. With the SAS alone the aircraft spins, 384° in 60 s.
- *Turn coordination.* With the turn switch on and the physical wiring the ball centres, |Y.SENS| 0.015 m/s² over 30–40 s (gate 0.03); as printed it stays at 0.12 (gate 0.06). With the turn switch off the heading hold yaws against the turn: 0.82 m/s² by 20 s, 28° of sideslip by 45 s, and the run departs after that. The host's latch (GenHel p. 5.5-3, the station, next PR) turns the switch on in exactly that case.
- *Collective to yaw (KSXCYO).* A 0.05 collective step at 20 kt turns the SAS-only aircraft 3.7° in 10 s; with the FPS 1.1° (gate: half the SAS's). The path on its own overcorrects in Eyas: with KSXCYO zeroed the FPS keeps the heading within 0.7°, and with it the first second's yaw rate reverses sign (up to +0.8 deg/s with the SAS alone, down to −0.6 with the FPS; the yaw path adds 0.93° of tail-rotor pitch to the mixer's 0.8°). It is GenHel's gain; the torque response it compensates depends on rotor-speed and engine dynamics Eyas doesn't have yet (M9). Open, revisited then.

**Modes** (the M8c2 method: flight control states dynamic, rotor and inflow quasi-static; body share ≥ ½):

![Modes](validation/m8/m8c3_modes.png)

- *Hover:* the lateral phugoid the SAS left unstable (ζ −0.10) is stable with the FPS (ζ 0.81 and 0.89 for the two lateral oscillations). Every body mode is stable with the FPS from hover to 150 kt; the 30 kt speed divergence (+0.15/s with the SAS alone) is gone.
- *Zero modes:* each held reference adds an eigenvalue at 0 (|Re λ| ≤ 2e-15), as a memory should.
- *Open: a 5 rad/s mode of the stabilator's lateral-acceleration loop.* The condensed model misses it. The full averaged model (116 states, rotor dynamic) has a mode at 4.7–5.2 rad/s, mostly the lateral accelerometer and gyros with heave: with the SAS alone ζ 0.27 (60 kt), 0.21 (90), 0.15 (120), 0.09 (150); with the FPS 0.16, 0.10, 0.05 and 0.00 (neutral at 150 kt). Nonlinear free flight agrees (σ 0.25/s at 120 kt against ζ ω_n 0.26). With the stabilator's lateral-acceleration feedback (KVYDT, M8c1) zeroed, ζ is 0.32 (SAS) and 0.19 (FPS) at 120 kt and 0.29 and 0.19 at 150 kt. So the loop is M8c1's stabilator law, which the FPS's delayed attitude loops push to neutral. M8c1 found the same feedback costs the Dutch roll damping; the law stays unchecked against data (DATA_GAPS.md). M8c2's condensed modes (ζ 0.18 at 150 kt for its nearest "control system" mode) understated it.

**30 min hands-off** with the SAS, PBA and FPS on (D-042 a):

![Long run](validation/m8/m8c3_afcs_30min.png)

- Hover: the aircraft stays at its start to 0.1 m, attitude constant to 1e-4°. 120 kt: airspeed, pitch and roll within 0.008 kt, 0.0009° and 0.0034° of the start (the 4/rev vibration, sampled once a second), heading and height held, the track 69 m to the side after 111 km. Every state finite, sim time exactly 1800 s, quaternion norm error ≤ 2.2e-16.

**Open items:**
- *KRPRO's last digit* (DATA_GAPS.md): reviewed, inconclusive, −0.189 used; the 0.5 % alternative changes no result.
- *The stabilator's lateral-acceleration loop* (above): neutral at 150 kt with the FPS on. A decision for the spec owner; the data to judge KVYDT against are not in the library.
- *KSXCYO overcorrects* at 20 kt; revisit with rotor speed and the engine (M9).

**Pilot station (M8c3 step 3).** The FPS button (gamepad X, keyboard F), the trim button as the FPS's trim release, and GenHel's turn-switch latch (`StationControls.FpsButtonAndTrimRelease`, `TurnSwitchLatchesAsGenHel`). Recordings carry the switches and replay exactly (`StationRecorder.AfcsChangesAreRecorded`, `ExampleStation.AfcsRecordingReplaysExactly`: 601 frames from 60 kt through the trim release, the turn switch and the FPS switched off, byte-identical in eyas_drive).

**Cost.** `eyas_bench` (Release, GCC 13.3, the development machine; trims now with the whole AFCS on): 13.8–14.4 µs mean per physics step (M8c2: 13.6–14.0 µs); worst `advance()` 637 µs against the D-005 budget of 8333 µs.

**Builds:** all 428 tests pass with GCC 13.3 (Release); all but the three slow Ballin trim tests pass with Clang 18.1 and GCC 13.3 (Debug), 427 each. Under ASan+UBSan the 44 new and changed unit tests of the flight control, units, trim kinds and condensation pass; the whole-aircraft FPS flights and the 60 s long run join CI's `SLOW_TESTS` (nightly job). clang-format and clang-tidy are clean. The code review's findings were each reproduced first; the fixes are listed in the D-045 M8c3 amendment.

## Ballin App. D and E — step responses (2026-10-19; readings reviewed)

Ballin, NASA TM-88360 (1987), App. D (17 cases: Ames real-time GenHel against USAAEFA flight test) and App. E (3 cases, with the Sikorsky GenHel too), deferred from M7c (D-035) and taken up now that the flight control system is in (D-046). Each case is trimmed at its Table C1 weight, CG and atmosphere, the stabilator held at the control system's trim incidence, and flown for 6 s through the digitized control histories, as Ballin flew the Ames simulation. Reported, not gated (D-046). The readings were checked by a human reviewer, panel by panel, against overlays on the scan (DATA_GAPS.md); 40 stretches the tool misread were traced by hand by the reviewer and replace its readings. E2 is flagged: Table C1 prints its CG waterline as 243.8 in, 11 in above every other file and doubtful (DATA_GAPS.md), so it is flown both as printed and with file 212's 232.5 in (the same flight) as a stand-in; its Sikorsky GenHel vertical acceleration was not accepted in review and is left out.

Generator `eyas_ballin_app_d_cases` (`tests/validation/ballin_app_d_cases.cpp`), plots and `ballin_app_d_differences.csv` from `tools/plot_ballin_app_d.py`; nightly in `full-verification.yml`.

| Check | Status | Evidence |
|---|---|---|
| Every case trims at its test conditions | ✅ | 20 of 20, E2 at both waterlines, with both servo sets; no unconverged evaluation in flight. The held stabilator comes out at 39° in hover (the schedule's stop), 22–25° at 60 kt and 3.3–3.9° at 100 kt; Ballin quotes 3.75° from the control system for a 100 kt case (p. 13). |
| Readings reviewed | ✅ | 206 of 206 panels checked (2026-10-18/19); 205 accepted, E2's vertical acceleration not (the Sikorsky GenHel's line is buried in the Ames one). DATA_GAPS.md. |
| Gate | — | To be chosen from these results (D-046). |

**On-axis response,** RMS difference over the 2 s from the stepped control's first movement, Eyas with GenHel's servos unless marked; the last column is Ballin's own simulation against the flight test, for scale:

| Fig. | Input | On-axis | Eyas − Ames GenHel | Eyas (Ames servos) − Ames GenHel | Eyas − flight test | Ames GenHel − flight test |
|---|---|---|---|---|---|---|
| D1 | 1-in. right cyclic, hover | roll rate [deg/s] | 4.40 | 4.41 | 1.20 | 3.58 |
| D2 | 1-in. left cyclic, hover | roll rate [deg/s] | 6.95 | 6.93 | 1.25 | 6.28 |
| D3 | 0.5-in. forward cyclic, hover | pitch rate [deg/s] | 1.18 | 1.13 | 0.95 | 2.13 |
| D4 | 1-in. up collective, hover | vertical accel. [ft/s²] | 0.78 | 0.55 | 0.79 | 0.71 |
| D5 | 1-in. left pedal, hover | yaw rate [deg/s] | 2.41 | 2.09 | 5.15 | 2.77 |
| D6 | 1-in. left cyclic, 60 kt | roll rate [deg/s] | 1.67 | 1.51 | 0.57 | 1.33 |
| D7 | 1-in. aft cyclic, 60 kt | pitch rate [deg/s] | 1.31 | 1.14 | 2.21 | 1.50 |
| D8 | 0.5-in. down collective, 60 kt | vertical accel. [ft/s²] | 0.45 | 0.42 | 1.29 | 1.14 |
| D9 | 0.5-in. up collective, 60 kt | vertical accel. [ft/s²] | 0.40 | 0.30 | 1.32 | 1.51 |
| D10 | 1-in. down collective, 60 kt | vertical accel. [ft/s²] | 1.00 | 0.80 | 1.82 | 2.54 |
| D11 | 0.5-in. right pedal, 60 kt | yaw rate [deg/s] | 1.25 | 1.06 | 2.20 | 2.54 |
| D12 | 1-in. left pedal, 60 kt | yaw rate [deg/s] | 1.04 | 0.76 | 3.72 | 3.19 |
| D13 | 1-in. aft cyclic, 100 kt | pitch rate [deg/s] | 0.74 | 0.52 | 0.74 | 1.28 |
| D14 | 1-in. forward cyclic, 100 kt | pitch rate [deg/s] | 1.56 | 1.43 | 1.61 | 1.91 |
| D15 | 0.5-in. up collective, 100 kt | vertical accel. [ft/s²] | 1.30 | 1.28 | 5.24 | 4.12 |
| D16 | 1-in. right pedal, 100 kt | yaw rate [deg/s] | 1.73 | 1.47 | 2.21 | 2.78 |
| D17 | 1-in. left pedal, 100 kt | yaw rate [deg/s] | 1.67 | 1.47 | 5.34 | 5.60 |
| E1 | 0.5-in. aft cyclic, hover | pitch rate [deg/s] | 1.37 | 1.31 | 2.10 | 3.39 |
| E2 ⚑ | 0.5-in. down collective, hover (WL 243.8 as printed) | vertical accel. [ft/s²] | 0.49 | 0.37 | 1.25 | 1.03 |
| E2 ⚑ | same, WL 232.5 (stand-in) | vertical accel. [ft/s²] | 0.49 | 0.36 | 1.24 | 1.03 |
| E3 | 1-in. right lateral, 100 kt | roll rate [deg/s] | 1.84 | 1.59 | 1.74 | 1.57 |

**Findings.**
- *Regenerated 2026-10-20 with D-048* (GenHel's downwash correction terms and spinning parts; the table above has the new numbers). Eyas is now closer than the Ames GenHel to the test on-axis in 12 of the 20 cases and within 0.1 in one more; more than 0.5 further in D5, D7, D12 and D15. Against the Ames GenHel it is within 2.4 units except D1, D2 and D5 (D17 3.74 → 1.67). The pedal steps at speed move most: D16 2.94 → 2.21 and D13 1.35 → 0.74 deg/s from the test, D12 2.70 → 3.72, D17 2.98 → 5.34 (it now follows the Ames GenHel, 5.60, more closely). The bullets below are as written on 2026-10-19, before D-048, except where marked.
- *Against the flight test,* Eyas is closer than Ballin's GenHel on-axis in 11 of the 20 cases, and within 0.1 of it in two more (D4, D13). The largest gains are the hover lateral steps: from its trim, the Ames GenHel's roll rate peaks at 14.1 deg/s in D1 and −20.4 deg/s in D2, the test at 9.4 and −13.3, Eyas at 10.1 and −14.1. In App. E the Sikorsky GenHel is 2.81 (E1) and 0.98 (E3) from the test, so Eyas is closer than both GenHels in E1 and just behind the Sikorsky one in E3.
- *Against the Ames GenHel,* Eyas is within 2.4 units RMS on-axis except those two hover lateral steps and D17 (3.7); see below. Ames's faster servos bring Eyas closer to the Ames curves in 18 of the 20 cases, by up to 0.35, and the two Eyas runs are otherwise hard to tell apart over 2 s.
- *Hover left pedal (D5)* is the one case where Eyas is clearly further from the test than GenHel (yaw rate 5.12 against 2.77 deg/s RMS): Ballin attributes the hover yaw response to the rotor-speed and engine dynamics (p. 12), which Eyas gets at M9. D15 (up collective, 100 kt) is the other case more than 0.5 further, where both simulations are far from the test (5.3 and 4.1 ft/s²).
- *E2 (⚑, down collective in hover)* is insensitive to its doubtful waterline. Lowering the CG from 243.8 to 232.5 in changes Eyas's trim pitch from 4.78° to 4.52° and roll from −2.55° to −2.40°, and every on-axis and off-axis RMS difference by 0.17 or less (pitch rate, the most affected, 0.67 → 0.83 deg/s from the test). That is well below Eyas's distance from either GenHel, so neither the test nor Ballin's curves can tell which waterline Ballin's simulations used. On-axis, Eyas is 0.22 ft/s² further from the test than the Ames GenHel. Off-axis, both GenHels pitch nose-up by about 9° over the 6 s and Eyas by about 5°, while the test holds within 1°; Ballin puts the hover collective response down to the rotor-speed and engine dynamics (p. 12), M9.
- *Why Eyas differs from GenHel in the hover lateral steps (D1, D2): the inflow model.* GenHel's main-rotor inflow is uniform momentum downwash through a first-order lag, skewed with forward speed only; a harmonic inflow driven by the rotor moments "has not been established for Black Hawk" (CR-166309 §5.1.1, p. 5.1-2). Eyas uses Pitt–Peters (D-027), whose first-harmonic inflow responds to the rotor's roll moment. Flown with Eyas's uniform inflow instead (a one-off probe: the case configs with `inflow: uniform`, not kept), Eyas reproduces the Ames GenHel: the roll rate peaks at 14.8 and −21.2 deg/s (GenHel 14.1 and −20.4), and the RMS difference from GenHel falls from 4.41 to 1.23 deg/s in D1 and from 6.98 to 2.37 in D2. With Pitt–Peters, Eyas is at 10.1 and −14.1, against the test's 9.4 and −13.3. Two independent sets of flight data put Eyas's hover roll response right and GenHel's too large: these steps, and Fletcher's flight-identified hover p/δlat (M7c V2: 0.7 dB mean, 1.6 dB RMS in magnitude). Ballin, too, finds the simulation's rates "somewhat less damp[ed]" than the test's (p. 11). Eyas's gain here is the inflow model's, not chance.
- *Pitch response to pedal at speed* (largely closed by D-048, 2026-10-20: the peak pitch rate within 2 s of the step is now −7.0 (D11), +5.6 (D12), −8.7 (D16) and +12.1 deg/s (D17), from −4.3, +3.9, −4.2 and +4.5 measured the same way; the sideslip-dependent term the paragraph below suspects was GenHel's M_ADD). *As written before D-048: too weak, known limitation.* After a pedal step at 60 and 100 kt the test pitches nose-down for right pedal and nose-up for left, the peak pitch rate within 2 s of the step being 11.8 (D11), 8.4 (D12), 16.9 (D16) and 17.2 deg/s (D17; with the pilot's aft stick). The Ames GenHel gets 7.9–14.4 deg/s, the same way. Eyas gets the right sign but only 3.3–4.8 deg/s in all four, a third to half of the test's; in hover (D5) it is in line with GenHel. The direct pedal path is GenHel's. Three probes held the body at D16's trim (103 kt) and stepped the pedal; scratch probes, not kept. Each per-inch change was checked by hand against GenHel's data and equations:
  - *Tail rotor:* 147 lbf/deg of blade pitch, against 150 from CR-166309 p. 5.4-7 at the same operating point. The pitch change of −4.82°/in, the rigging's −5.54°/in less the δ₃ coning feedback, matches. The cant moment is 7 740 ft·lbf/in, against 709 lbf × sin 20° × 31.8 ft.
  - *Main rotor, through the mixer's MIX_BP:* −9 480 ft·lbf/in, against −9 300 from the hinge-offset spring (N/2)eM_βΩ², the thrust tilt T·h and the thrust change at the hub's offset, for Eyas's 1.90°/in of β₁c. The flapping per degree of cyclic behind it matched Fletcher's flight data at M7c (on-axis pitch, 0.8 dB at 80 kt).
  - *Stabilator wash:* +1.95 ft/s/in, against +1.94 from the tail-wash table (Table 3.5.2, p. 5.3-20). It comes from the main rotor's downwash dropping with its thrust; the flapping input a₁ barely moves the factor, so the AA1FMR ruling (DATA_GAPS.md) doesn't enter.
  The net held response is about −2 800 ft·lbf/in nose-down for right pedal. Most of the gap from GenHel builds later. In D16, GenHel's pitch rate is −1.6 deg/s and Eyas's −0.8 at 0.55 s after the pedal moves, but −8.0 and −3.9 at 1.3 s, while the sideslip grows from −1° to −11° and the yaw rate to 15 deg/s. In that phase Eyas's stabilator turns nose-up (+2 000 to +3 700 ft·lbf as its angle of attack falls by about 1.3°) and stops the motion. The suspects are now the sideslip- and rate-dependent airframe terms (the fuselage and stabilator tables against sideslip, the stabilator's dynamic-pressure ratio), and anything in the Ames simulation that CR-166309 doesn't describe. Open.
- *Off-axis* (pitch and yaw to lateral, roll to longitudinal): all three simulations miss parts of the test's off-axis response, as Ballin says of GenHel (p. 11, the linear mechanical mixing). Not yet compared in detail.

![D1](validation/m8/ballin_app_d_d1.png)
![D13](validation/m8/ballin_app_d_d13.png)
![D4](validation/m8/ballin_app_d_d4.png)
![D5](validation/m8/ballin_app_d_d5.png)
![E2, both waterlines](validation/m8/ballin_app_d_e2.png)
![D16, right pedal at 100 kt (with D-048)](validation/m8/ballin_app_d_d16.png)

## Kaplita steady-heading sideslips and GenHel's motion-module additions (D-048, 2026-10-20; issue #24)

Kaplita, NASA CR-177360 (1985), Figs. 11–13: Sikorsky's GenHel ("existing model", identical to CR-166309) and the USAAEFA Project 79-24 flight test in steady-heading sideslips at 60, 100 and 140 kt, digitized and human-reviewed (DATA_GAPS.md). Eyas flies Kaplita's simulation setup (SAS on, PBA disabled and centred, FPS off, stabilator held at its zero-sideslip trim incidence) on Ballin's aircraft at 5250 ft as a stand-in for the three test flights, so gradients are compared, not levels.

Generator `eyas_kaplita_cases` (`tests/validation/kaplita_cases.cpp`), plot and gradients `tools/plot_kaplita.py`; nightly in `full-verification.yml`. The generator also flies the aircraft without the correction terms, to show what D-048 changed.

| Check | Status | Evidence |
|---|---|---|
| Correction terms match GenHel's printed formulas | ✅ | `AirframeAero.MotionAdditionsMatchGenHel` (D-036 parabola within ±5°) |
| Spinning parts match GenHel's gyroscopic equations | ✅ | `SpinningParts.MatchesGenHel`, `SpinningParts.ReadFromConfigAndAddedToTheBody` |
| Every sideslip gradient within a quarter of GenHel's (plus read-off) | ✅ | `KaplitaSideslip.GradientsAgainstSikorskyGenHel`; fails without the terms (longitudinal stick, pitch, lateral stick) |

![Kaplita sideslips](validation/kaplita/kaplita_sideslip.png)

**Gradients** per degree of sideslip, least squares over ±12° (`kaplita_gradients.csv`): flight test / Sikorsky GenHel / Eyas / Eyas without the terms.

| | 60 kt | 100 kt | 140 kt |
|---|---|---|---|
| Longitudinal stick [%] | −0.56 / −0.51 / −0.42 / −0.22 | −1.09 / −0.88 / −0.78 / −0.27 | −1.37 / −1.37 / −1.14 / −0.28 |
| Pitch [deg] | +0.035 / +0.064 / +0.054 / −0.003 | +0.158 / +0.132 / +0.130 / −0.019 | +0.237 / +0.233 / +0.207 / −0.039 |
| Lateral stick [%] | +0.30 / +0.47 / +0.38 / +0.31 | +0.36 / +0.81 / +0.78 / +0.56 | +0.43 / +1.38 / +1.38 / +0.90 |
| Roll [deg] | +0.06 / +0.17 / +0.15 / +0.16 | +0.34 / +0.39 / +0.36 / +0.39 | +0.67 / +0.73 / +0.71 / +0.77 |
| Pedal [%] | −0.96 / −0.85 / −0.63 / −0.64 | −1.80 / −1.75 / −1.56 / −1.63 | −2.70 / −2.54 / −2.38 / −2.44 |

- *Longitudinal stick and pitch:* without the terms Eyas had a quarter to a half of the stick gradient and the pitch gradient's wrong sign at every speed. With them both follow GenHel and the test, including the bends in the pitch curves near ±8–10° of sideslip, where M_AD reaches its limits.
- *Lateral stick:* GenHel's gradient is two to three times the flight test's at 100 and 140 kt; Eyas now follows GenHel there. Kaplita reports the same at 140 kt ("lateral stick compares less favorably than at lower speeds", p. 9, PDF p. 23). Open with GenHel's own.
- *Roll and pedal:* unchanged, and with both references.

**Effect on M7c** (level trims under GenHel's lateral constraint; Eyas − Sikorsky GenHel, before → after; the gap register above is the M7c record):
- *Sideslip at zero roll:* +2.8 to +3.9° → +0.2 to −1.1° from 50 to 160 kt. The open gap "Eyas trims with right sideslip at zero roll" was the missing correction terms.
- *Pedal:* −1.0 to −8.9 % → +1.3 to −3.3 %. *Main-rotor power:* 5–30 hp closer to GenHel's from 50 to 140 kt.
- *Pitch attitude:* within ±0.4° from 60 to 160 kt (was up to +0.54°); at 50 kt +0.34 → +0.60°.
- *Longitudinal cyclic:* within about ±1.3 % → 2.1–3.5 % aft of GenHel from 100 to 150 kt.
- *Lateral cyclic above 120 kt:* unchanged (+1.2 to +6.3 %), so it isn't linked to the sideslip, as the register had guessed.

**Against the flight test at its own sideslip, roll free** (the six B1 test points, 49–164 kt; Eyas − test, rms before → after): roll 2.57 → 0.29° (the register's "Eyas rolls left where the test rolled right" closes), pitch 2.12 → 0.91°, longitudinal stick 5.7 → 4.0 % (mean +0.3 → +2.9 %), lateral stick 3.1 → 4.5 %; pedal (+8.5 %, the open pedal-scale question) and collective unchanged.

**Dutch roll** (the M8c2 method, flight control states dynamic; ζ before → after): SAS off 0.15 → 0.37 (90 kt), 0.02 → 0.31 (120 kt), 0.00 → 0.06 (150 kt); SAS and PBA on 0.44 → 0.49, 0.37 → 0.42, 0.33 → 0.35. The correction terms add lateral-directional stiffness and damping in sideslip. Not yet checked against flight data; Kaplita Fig. 4 (pedal step, 144 kt, SAS off, stabilator fixed) is the referee to digitize next.

**Longitudinal cyclic at cruise, investigated** (scratch probes, 2026-10-20; zero-roll Ballin trims, Eyas − Sikorsky GenHel): with the terms Eyas's stick sits 2.1–3.5 % aft from 100 to 150 kt, where it had been within about 1 %.
- *The earlier agreement was two errors cancelling.* Without the terms but held at GenHel's own sideslip, Eyas sat about 2.5 % forward of GenHel at 140 kt; its 3.9° sideslip error added the same amount back.
- *The pitching-moment budget at 140 kt* (passage averages, ft·lbf, without → with the terms): the pitch addition 0 → −3270 at β_WF −1.32°; stabilator +17 700 → +19 280; fuselage −12 390 → −12 770; tail rotor unchanged; the main rotor balances it, +5 730 → +7 800, which is the aft stick.
- *Accounted for, about 1.1 % of the 3.2 % at 140 kt:* Eyas's zero-roll sideslip, 0.5° more negative than GenHel's, makes the addition larger (about 0.6 %; with GenHel's uniform inflow the sideslip lands within 0.1–0.2° of GenHel's at 100–120 kt, so this is the inflow model, D-027), and the D-036 parabola near zero sideslip (about 0.5 %).
- *Ruled out:* the inflow model as the stick's cause (with uniform inflow the stick is within 0.2 % of the Pitt–Peters result), the CG waterline (≤ 1.15 % of stick, and at 20 kt, M7c), the stabilator (within 0.5° of GenHel's B1(h), not the way that would add nose-down), and the terms themselves (sign and size confirmed by the Kaplita gradients).
- *Open, about 1.9 % from 100 kt up* (about 1 300 ft·lbf of nose-down at 140 kt, 0.3° of cyclic): of the size of the open rotor differences above it, collective 2–5 % low and lateral cyclic 2–6 % high from 120 kt. Against the flight test the stick improved (5.7 → 4.0 % rms). Recorded open in `BallinTrim.LevelFlightAgainstSikorskyGenHel` (D-048, test dispositions); to revisit with the USAAEFA report (DATA_GAPS.md) and M9's rotor items (tip loss, the profile-drag correction, lag, wake distortion).

**Test dispositions** (D-048): the Ballin records re-recorded, pitch at 90–150 kt and pedal at 50–130 kt gated again, the longitudinal cyclic at 100–150 kt and pitch at 50 kt recorded open; the climb record at 100 kt updated; the missing-SAS check at 150 kt only and the SAS-drift check at 100 kt only, thresholds unchanged.

## Kaplita transient responses: Figs. 4 and 2 at 144 kt (2026-10-20; readings reviewed)

### Fig. 4: half-inch right pedal step

Kaplita, NASA CR-177360 (1985), Fig. 4: a half-inch right pedal step at 144 kt, Sikorsky's GenHel against the USAAEFA flight test, digitized and human-reviewed (DATA_GAPS.md). Eyas flies Kaplita's setup (p. 2): SAS, FPS and PBA off, the stabilator fixed at the flight-test value (6.54°), the test's pedal and stick histories as inputs, collective held. **STAND-INS** (DATA_GAPS.md; approved by the spec owner): the CG waterline (231.8 in) and the density altitude (5750 ft) aren't given for Fig. 4's flight and are Kaplita Fig. 13's. The thin lines fly 4000 and 7500 ft.

Generator `eyas_kaplita_transient_cases` (`tests/validation/kaplita_transient_cases.cpp`, both figures), plot and differences `tools/plot_kaplita_transients.py`; nightly in `full-verification.yml`. Report only, like D-046.

![Kaplita Fig. 4](validation/kaplita/kaplita_fig4.png)

**RMS differences, 1.5–6.5 s** (attitudes as changes from 1.5 s; `kaplita_fig4_differences.csv`):

| | Eyas − GenHel (with / without the D-048 terms) | Eyas − flight test (with / without) | GenHel − flight test |
|---|---|---|---|
| Yaw rate [deg/s] | 1.3 / 4.4 | 4.6 / 4.4 | 4.4 |
| Sideslip [deg] | 0.8 / 4.4 | 6.8 / 4.8 | 7.5 |
| Pitch rate [deg/s] | 1.9 / 6.7 | 2.9 / 6.9 | 2.8 |
| Pitch [deg] | 1.0 / 12.2 | 4.1 / 15.4 | 4.5 |
| Roll rate [deg/s] | 5.8 / 4.8 | 17.6 / 20.3 | 22.2 |
| Roll [deg] | 17.8 / 13.7 | 24.2 / 28.3 | 40.6 |

**Findings.**
- *D-048 in a transient.* With the downwash correction terms Eyas follows GenHel through the manoeuvre (yaw rate 1.3 deg/s, sideslip 0.8°, pitch 1.0° rms); without them its pitch response is wrong (12° rms from GenHel). The pitch rate and attitude, where the terms act most, are as close to the flight test as GenHel's own.
- *Adverse roll with pedal, a GenHel limitation Eyas shares.* After the right pedal step both models roll right hard (40–60° of bank by 6 s); the test aircraft barely rolls. Kaplita's Table II (pp. 27–28) rates GenHel's roll response to pedal at 120–150 kt "strong adverse roll", unsatisfactory, and its §2.2.3 (p. 15) proposes a tail-rotor downwash lag for it. Not taken up (it would be a departure from CR-166309, for the spec owner). From about 7 s, at that bank, Eyas spirals away (roll rate climbing past 40 deg/s); GenHel levels out.
- *The Dutch roll is not measured here.* The test shows a lateral-directional oscillation near 1.9 rad/s that dies in two or three cycles; in both models the adverse roll swamps it. Fig. 2 (a lateral doublet at 140 kt) is the next case to measure it.
- *The stand-ins* barely matter: 4000 to 7500 ft moves the responses by a line width to 6 s.

### Fig. 2: one-inch lateral stick doublet

Kaplita Fig. 2 (run 12): the same setup, the stabilator fixed at 6.49°, at the header's weight and CG station with the same **STAND-INS** (CG waterline 231.8 in, density altitude 5750 ft; DATA_GAPS.md). Three tick labels (roll attitude, roll rate, pitch rate) are a best guess, ±40, checked by the attitude kinematics; the test's roll carries a constant offset of about 21° and is compared as a change (DATA_GAPS.md). At 7.8 s the pilot pulls aft (all three pitch up sharply); the compared window ends there.

![Kaplita Fig. 2](validation/kaplita/kaplita_fig2.png)

**RMS differences, 0.5–7.8 s** (attitudes as changes from 0.5 s):

| | Eyas − GenHel (with / without the D-048 terms) | Eyas − flight test (with / without) | GenHel − flight test |
|---|---|---|---|
| Yaw rate [deg/s] | 0.80 / 0.61 | 0.89 / 0.62 | 0.49 |
| Sideslip [deg] | 1.07 / 3.31 | 1.91 / 2.56 | 1.06 |
| Roll rate [deg/s] | 1.88 / 2.04 | 1.96 / 1.92 | 2.27 |
| Roll [deg] | 2.64 / 2.12 | 3.41 / 2.49 | 2.25 |
| Pitch rate [deg/s] | 1.12 / 2.28 | 3.45 / 3.24 | 4.12 |
| Pitch [deg] | 1.65 / 3.67 | 7.07 / 5.29 | 8.52 |

**Damped-oscillation fits to the sideslip after the doublet** (5.3–7.8 s, under one cycle; fit residual under 0.1°): flight test ω 1.98 rad/s, ζ 0.22; GenHel 2.05 rad/s, 0.41; Eyas 1.73 rad/s, 0.39; Eyas without the terms 1.55 rad/s, 0.36. The fits to yaw rate don't converge (the frequency runs to the search bound) and aren't used. Less than a cycle, with the pilot's inputs before and after it, makes these rough: a frequency to about 10 %, a damping ratio to about ±0.1.

**Findings.**
- *The roll response to lateral stick* matches the test and GenHel: peak roll rate 23 deg/s (test 21.5, GenHel 23.5), roll change 14° (test 14.5°).
- *Pitch coupling:* Eyas follows GenHel (1.1 deg/s, 1.7° rms); both pitch up slightly after the doublet where the test pitches down by up to 6° before the pilot's pull. Without the D-048 terms Eyas is further from GenHel in pitch and sideslip.
- *The Dutch roll with the stabilator fixed:* every model's is damped (ζ about 0.4), more than the test's (about 0.2), with a frequency within about 15 %. Eyas's bare-airframe Dutch roll was nearly neutral at 150 kt (M8c2, SAS off: ζ 0.000 before D-048, 0.06 after) because those modes had the stabilator live: its lateral-acceleration feedback (KVYDT) takes the damping away (M8c1: ζ 0.28 fixed against 0.07 live at 120 kt). So with the stabilator fixed, as Kaplita flew, the airframe isn't the cause: the low-damping problem is the stabilator loop, issue #24's subject.


---

## M8d — Vortex-ring-state correction (2026-10-20)

Johnson's vortex-ring model (NASA/TP-2005-213477, Tables 3 and 4; the model Johnson 2013 §4.2 refers to) in the main rotor's mean inflow (D-049; SPEC_REVIEW #21 resolved). Two Table 3 misprints are corrected and marked (DATA_GAPS.md), and the model's two edge jumps are faded over 0.05 v_h. Uniform inflow uses the curve quasi-statically. Pitt–Peters uses the baseline curve's mass flow in its thrust row and the VRS increment as a thrust deficit lagged by Johnson's 0.7/λ_h revolutions. Generator `tests/validation/m8d_cases.cpp`, plots `tools/plot_m8d.py`. No UH-60A vortex-ring flight data is in the library. The realism checks are Johnson's flight-test stability boundary (helicopter and tiltrotor, his Fig. 2) and the shape of his D6075 comparison.

| Check | Status | Evidence |
|---|---|---|
| The curves against an independent transcription | ✅ | `VortexRing.CurvesMatchJohnsonTable3Transcription`: both curves on a 131 × 401 grid (vx 0–1.3, vz −3.5–0.5) to 1e-11. The reference (`tests/support/vrs_reference.hpp`) finds momentum roots as companion-matrix eigenvalues and builds the cubics by Hermite divided differences. `MomentumRootsMatchClosedFormsAndTheReference`. |
| Johnson's specification | ✅ | `InflowHasZeroSlopeOnTheStabilityBoundary`: (vz + v)/v_h = 0.85 at −0.45 and 1.25 at −1.5, zero slope on the boundary N(vx), X(vx) up to vx = 0.90, an unstable band between, none on the baseline. `CurvesJoinMomentumTheoryWhereJohnsonJoinsThem` (value and slope at A and D, value at B and E; momentum theory outside, to the bit). The curves on Johnson's axes are below. |
| The fades | ✅ | `FadesMakeTheCurvesContinuousInEdgewiseSpeed`: continuous across vx = 0.75 and 0.95, where the published model jumps by about 0.003 and up to 0.14 v_h (recorded in the test). |
| The inversion is unique and bounded | ✅ | `InducedVelocityRisesWithThrust`: dλ0/dλ_h = f − x f_x − z f_z ≥ 0.4 over the region (finer in the fade bands). `ThrustInversionRecoversTheCurveThrust`: 2000 random round trips, both thrust signs, 1e-12. `InversionIsBoundedAndFinite` (limit 100 curve evaluations per curve; a sweep of the region needed at most 27 per call). |
| The rotor | ✅ | `UniformSolveFollowsJohnsonsCurveWithoutJumps`: refining a 0–75 m/s descent sweep 4× shrinks the largest thrust step (continuous), while uncorrected the root switch stays. Windmill brake past 2 v_h. `PittPetersSteadyStateIsJohnsonsCurve`: steady state equal to the uniform solve, with D at its quasi-static value; rates bit-identical to the uncorrected rotor outside the region. `DeficitLagsWithJohnsonsTimeConstant`. |
| Heave damping on an axial rotor | ✅ | `HeaveDampingChangesSignOnTheBoundary`. With thrust a function of collective and total inflow only, sign(dT/dw) = sign(d(vz + v)/dvz at fixed T), so it changes sign exactly on the curve's stationary points. Found at −0.45 and −1.5 v_h to 1e-4. |
| Heave damping, the whole UH-60A | ✅ | `Uh60aVortexRing.HeaveDampingChangesSignOnTheBoundary` (slow test): the bare aircraft's condensed Z_w in vertical descent changes sign at vz/v_h = −0.4495 (boundary −0.450 at the rotor's vx/v_h = 0.020) and −1.453 (boundary −1.494 at vx/v_h = 0.083). Gate 0.05. Below. |
| Real-time rules | ✅ | `VortexRing.AllocatesNothing`. `eyas_bench` gained a 10 m/s vertical-descent case (mid-region, whole AFCS on): step 29.4 µs mean (hover 14.3), worst `advance()` 920 µs against the 8333 µs budget. One inversion costs about 5.3 µs. |
| Earlier results unchanged | ✅ | All 436 earlier tests pass with the correction on in every UH-60A config; none of their conditions is in the region. |
| Against flight data | — | No UH-60A vortex-ring data in the library. The D6075 comparison below is of shape only. |

**The curves** (Eyas's, on the axes of Johnson's Figs. 38 and 39):

![Curves](validation/m8/m8d_curves.png)

They match Johnson's figures, including the kinks at his transition points. At vx/v_h = 0 the hump peaks at 1.25 at −1.5 and the dip bottoms at 0.85 at −0.45. At vx/v_h = 0.8 the peak is v/v_h ≈ 1.9 near −1.2.

**The fades** (D-049): just below each edge, the published model against momentum theory at the edge, and Eyas's faded curve:

![Fades](validation/m8/m8d_fades.png)

At vx/v_h = 0.95 the published curve still carries an S-shaped increment (zero slope at the merged N and X), and then it vanishes. Eyas's has faded to momentum theory. At 0.75 the baseline's offset (about 0.003 v_h) is too small to see here; the test records it.

**The UH-60A in vertical descent** (bare, collective fixed, ISA sea level; the main rotor's descent over its own v_h):

![Heave](validation/m8/m8d_heave.png)

- *Heave damping:* Z_w = −0.25 /s near hover, as before. With the correction it turns unstable between −0.45 and −1.45 v_h, peaking at +0.35 /s near −1.0 v_h. Johnson's D6075 peaks at +0.8 /s (Fig. 52), a lighter aircraft.
  - *With the lag* (the heave root with D dynamic), the peak is +0.18 /s, about half. Johnson finds the lag reduces the damping magnitude without moving the sign changes (Fig. 52: 0.23 against 0.8), and so does Eyas.
  - *Without the correction,* Z_w stays negative throughout (−0.10 to −0.25 /s), as Johnson finds with his baseline.
- *Where the crossings sit:* N lies on the boundary (0.0005 v_h). X is 0.04 v_h inside it.
  - The airframe's own heave damping, measured at the trim by finite differences, is −0.031 /s near N and −0.057 /s near X. At X's Z_w slope that moves the crossing about 0.02 v_h inward. At N it would move it about 0.03 v_h *outward*, which isn't seen.
  - So a further +0.02–0.03 v_h comes from the rest of the aircraft (flapping, the edgewise flow from the shaft tilt, vx/v_h up to 0.09). It is not separated further.
- *Trimmed collective:* falls from hover to a minimum of 0.499 at 5.5 m/s (−0.46 v_h), then *rises* to 0.618 at 18 m/s (−1.5 v_h): in the vortex ring a faster steady descent needs more collective. Uncorrected, the collective falls steadily.

**A vortex-ring entry** (the shape of Johnson's D6075 vertical case, Figs. 56–57). From a trimmed 1 m/s vertical descent with the whole AFCS on, the collective is lowered by 0.24° of θ0 at 5, 15 and 25 s (0.72° in all; the D6075's reductions total about 0.7°), each ramped over 1 s. Then 100 s free:

![Encounter](validation/m8/m8d_encounter.png)

- *With the correction:* the first two steps each add about 1 m/s of descent. After the third, the descent creeps past 0.5 v_h (37.9 s), then runs away: 1.0 v_h at 56.6 s, 1.5 v_h at 63.3 s. It overshoots to 1.79 v_h and settles at 1.68 v_h (19.3 m/s) by 100 s, just past the boundary's lower point X, where the heave damping is positive again. That is Johnson's "helicopter stabilization".
  - *Against the D6075:* Johnson's measured drop runs from about 0.5 to 1.7 v_h in roughly 10–15 s and stabilizes near 1.7–1.75 v_h (his Figs. 57, 59). Eyas's takes about 25 s (different aircraft, inputs and AFCS), and settles at the same depth.
  - *Flow state:* the total flow is still down through the disc at the end (vortex-ring state past X, not windmill brake).
- *Without the correction:* the same inputs level off near 4–5 m/s (0.4 v_h; 4.0 m/s at 100 s).

## M9 — Ground effect (2026-10-21)

Ground effect on the main rotor's mean inflow (D-050), with two models chosen in config:
- `genhel`, the UH-60A's: GenHel CR-166309 Eq. 5.8.2-1;
- `cheeseman_bennett`: Johnson 2013 Eqs. 4.71 and 5.76, κ_g = f_g^(−3/2).

Both inflow models are solved for the out-of-ground-effect inflow λ₀/κ_g, so in hover λ₀ = κ_g λ_h. The wake angle comes from the induced inflow. Generator `tests/validation/m9_ground_effect_cases.cpp`, plots `tools/plot_m9_ground_effect.py`:

    ./build/release/tests/eyas_m9_ground_effect_cases build/m9_ground_effect_cases
    .venv/bin/python tools/plot_m9_ground_effect.py build/m9_ground_effect_cases docs/validation/m9

The flight data are USAAEFA's UH-60A tethered hover at 2 ft and 100 ft wheel height (ADA180181 App. E Figs. 2 and 3; the printed fits, DATA_GAPS.md).

| Check | Status | Evidence |
|---|---|---|
| The factors against an independent transcription | ✅ | `GroundEffect.FactorsMatchTheSourceTranscription`: both models on a 60 × 31 × 45 grid (z/R 0.1–6, μ 0–0.3, λ₀ −0.1–0.12) to 1e-14. The reference (`tests/support/ground_effect_reference.hpp`) follows each source's variables: GenHel's λ/μ_TOT, and Johnson's f_g with z√(1 + (μ/λ)²). |
| Hover values and limits | ✅ | `HoverValuesAndLimits`: GenHel 1.115^(−2/3) and Cheeseman–Bennett (15/16)^(3/2) at z = R; 0.7906 at USAAEFA's hub height; held below 0.3R; 1 with no ground, no model, or no wake under the disc. `FactorRisesToOneWithHeight`: in (0, 1], rising with height, falling with cos χ, above 0.996 at 5R (GenHel's switch would be a 0.3 % step there). |
| The inflow solve stays unique | ✅ | `OutOfGroundInflowRisesWithInflow`: d(λ₀/κ_g)/dλ₀ ≥ 1 over heights 0.3–1R, μ 0–0.1, λ₀ −0.2–0.2. With the total flow in cos χ (GenHel's and Johnson's own form) the slope reaches −50 and −90 near ideal autorotation (D-050). |
| The rotor in hover | ✅ | `HoverInflowIsKappaTimesMomentum`: both models, with and without the vortex-ring correction, hub heights 0.45–2R, the body pitched, rolled and yawed. The uniform solve gives λ₀ = κ_g √(C_T/2) to 1e-13, and Pitt–Peters is at rest there (1e-9, the solve's tolerance). `HoverThrustRisesNearTheGround`: at fixed collective, the thrust rises as the hub descends and tends to its no-ground value. |
| No ground, no change | ✅ | `NoGroundChangesNothing`: with no ground plane, or with `none`, rates and loads are bit-identical to the uncorrected rotor's in hover, climb, forward flight and the vortex ring. All 450 earlier tests pass unchanged (no earlier test has a ground plane under a flying rotor). |
| The UH-60A trims in ground effect | ✅ | `Uh60aGroundEffect.TrimsInGroundEffectAndFliesOn` (slow): hovering with the wheels about 2.7 ft up, the trim converges off the gear, main-rotor power is between 70 and 90 % of the no-ground power, and the aircraft flies on (4 revolutions) with v̇ < 5e-5 m/s² and ω̇ < 2e-5 rad/s². `TrimOnTheWheelsIsRejected`: a ground plane above the wheels gives no convergence and an unchanged model and outputs. `LightlyTouchingWheelIsRejected` (was `GrazingWheelIsRejectedOverTheRevolution` until D-051): a hover with the tail wheel carrying under 1 N, found by bisecting the ground height, whose residuals converge, is rejected (and the test fails with the check removed). At first, at USAAEFA's 2 ft, the tail wheel cleared the ground at the start of the revolution and touched during it (4.8 N); that edge was 0.04 in wide and moved with D-051's small change in attitude, and the wheel load varies by only about 0.01 N over the revolution, so a clear-then-touching case isn't robust to test. `RejectsGroundConditionsItCannotHold`: an infinite ground height, flight that changes height near a ground, and linearizing near a ground without h. |
| Real-time rules | ✅ | `GroundEffect.AllocatesNothing`. `eyas_bench` gained a hover in ground effect (wheels about 3 ft up, whole AFCS on): step 14.0 µs mean, the same as out of ground effect; worst `advance()` 642 µs against the 8333 µs budget. |
| UH-60A hover power ratio against flight test | ✅ (gate on a known gap) | `Uh60aGroundEffect.HoverPowerRatioAgainstUsaaefa` (slow): with `genhel`, within 0.05 of USAAEFA's ratio at 0, 6000 and 12 000 ft (measured 0.038, 0.035, 0.046 above; 0.038, 0.035, 0.045 with D-051's tip loss). The gate holds the known shortfall from growing (spec owner, 2026-10-21); the shortfall itself is open (below). Cheeseman–Bennett is 0.055–0.066 below, reported. |

**The factor** in hover, both models with Johnson's empirical fits (Eqs. 4.73–4.76, converted with κ_g = f_g^(−3/2) at C_T/σ = 0.08):

![Factor](validation/m9/m9_ge_factor.png)

- GenHel's curve lies on Schmaus et al.'s from 0.4R up.
- Cheeseman–Bennett with Johnson's conversion lies with Law's and Hayden's, about 0.1 lower at USAAEFA's height.

**Hover power against USAAEFA** (ISA, 16 638 lbm, density altitude 0–12 000 ft for C_W 0.0059–0.0085; the bare aircraft; main-rotor power):

![Hover](validation/m9/m9_ge_hover.png)

- *Geometry:* USAAEFA measures wheel height from the bottom of the left main wheel, with the hub 12 ft above the main wheels (Eyas's config: 11.90 ft at full extension).
- *Why Eyas isn't trimmed at 2 ft:* at Eyas's hover attitude (4.4° nose up, 2.0° left wing down) the tail wheel hangs 23.9 in below the left main wheel. At 2 ft it clears the ground by 0.04 in at sea level and touches during the revolution (4.8 N), and the trims near that edge don't all converge. (With D-051 the edge moved to 1.97–1.98 ft at sea level; the method is kept.) So Eyas is trimmed at 2.5 and 3 ft, with every gear load zero at those heights, and its ratio is extrapolated linearly in wheel height to 2 ft. The extrapolation moves the ratio by 0.009 (GenHel) and 0.016 (Cheeseman–Bennett).
- *Like for like:* USAAEFA's C_P is engine power: main and tail rotors, transmission and accessories. Eyas's is the main rotor's, which out of ground effect is about 25 % below the engine fit. That gap is the tail rotor, the losses, and the open hover-power gap (M7c; issue #42). So the comparison is the ratio at the same weight, P(2 ft)/P(100 ft). That is like for like if the other shares scale with the main rotor's power.

| C_W × 10⁴ | USAAEFA (1985) | USAAEFA 77-17 (dashed) | Eyas, GenHel | Eyas, Cheeseman–Bennett |
|---|---|---|---|---|
| 59.0 | 0.789 | 0.775 | 0.828 | 0.732 |
| 70.6 | 0.787 | 0.771 | 0.822 | 0.723 |
| 85.1 | 0.769 | 0.755 | 0.815 | 0.714 |

With D-051's tip loss on the inflow (2026-10-22; the plot shows it): Eyas with GenHel's model 0.827, 0.821, 0.815; with Cheeseman–Bennett 0.730, 0.722, 0.713.

- *GenHel's model reduces the power too little:* about 17 % against the 21 % measured, 0.035–0.046 in the ratio.
  - In Eyas the induced power is 79 % of the hover total (1210 of 1537 hp, M7c). So a factor κ on it gives a total ratio near 0.21 + 0.79 κ: 0.83 at GenHel's κ = 0.79, as trimmed (the lighter download near the ground adds little).
  - The measured 0.79 needs κ ≈ 0.73.
  - GenHel's κ at this height equals the measured *total*-power ratio almost exactly. That suggests GenHel's factor was fitted as a total-power ratio and is applied to the induced downwash only. This is a hypothesis: §5.8.1 says only that it "was derived from Black Hawk hover-power flight test results".
  - Tip loss on the inflow (issue #42) would make the induced share a little larger and the ratio about 0.005 lower. *Done (D-051):* it lowered it by up to 0.001, less than estimated; the shortfall stays.
- *Cheeseman–Bennett reduces it too much:* about 28 %, 0.055–0.066 below.
- *USAAEFA's two data sets* (1985 solid, the reanalysed 77-17 dashed) differ by 0.015. The measured ratio lies between Eyas's two models.
- *Open (issue #61):* GenHel's shortfall, and whether to apply its factor to the total power instead (a departure needing a ruling).

**Power against hub height** (sea level, both models over no ground):

![Height](validation/m9/m9_ge_height.png)

**Forward flight near the ground** (level flight at hub heights 0.55, 0.65 and 1.0R, ISA sea level; cf. Johnson 2013 Fig. 5.12):

![Speed](validation/m9/m9_ge_speed.png)

- *Ground effect fades out by about 40–50 kt with both models,* the 1.5–2 v_h Johnson gives (§5.5.1; v_h ≈ 23 kt here).
- *With Cheeseman–Bennett's faster fade (cos² χ in the height),* power at 0.55R *rises* from hover to a peak near 16 kt before translational lift takes over. Johnson notes that "very close to the ground, the power can even increase with speed" (§5.5.1). Padfield §3.2.1 (Fig. 3.29) also gives the power increase as a helicopter flies off the ground cushion.
- *GenHel's fade (cos χ)* is slower: power is flat to about 8 kt and then falls.

## M9 — Tip loss on the inflow (2026-10-22; issue #42)

Tip loss on the main rotor's mean inflow (D-051): the momentum relation of both inflow models is taken over the disc of radius B·R (Johnson 2013 §3.6.2, Eqs. 3.114–3.115), as well as the blade carrying no lift outboard of B·R (D-024, unchanged). In hover the induced velocity is 1/B of the full disc's, about 3 % more for B = 0.97; in edgewise flight 1/B², about 6 %. Rotor key `inflow_tip_loss: none | effective_disc`; the UH-60A's main rotor sets `effective_disc` (a departure from GenHel, which takes the full disc).

| Check | Status | Evidence |
|---|---|---|
| The rotor against an exact reference | ✅ | `BladeElementRotor.InflowTipLossMatchesExactReference`: hover and μ ≈ 0.2 against `bet_reference.hpp`, whose own bisection solves the momentum relation for the disc of radius B·R: second-order convergence and Richardson extrapolation to 1e-9. |
| Hover closed form | ✅ | `HoverWithInflowTipLossMatchesJohnsonClosedForm`: Johnson Eqs. 3.112 and 3.115 solved together, θ_.75 4–12°, twist 0 and −10°, to the M3 closed-form tolerances (the same small-angle residual); Johnson Eq. 3.114's induced power C_T^{3/2}/(√2 B) to 1e-9. |
| Pitt–Peters thrust row | ✅ | `PittPetersAnchors.InflowTipLossIsTheEffectiveDiscsThrustRow`: the independent Pitt & Peters 1980 transcription run on the effective disc in its own coefficients, 50 random states and loads incl. skew and upflow, to 1e-12; moment rows unchanged to the bit; hover steady state λ₀ = √(C_T/2)/B and time constant B·(128/75π)/(4λ₀Ω). `SteadyHoverEqualsUniformMomentum` (also with tip loss): the Pitt–Peters steady state equals the uniform solve to 1e-9, in hover and climb. |
| Nothing changes without it | ✅ | `InflowTipLossAtUnitRadiusChangesNothing`: B = 1 with `effective_disc` is bit-identical to `none`, both inflow models. All 463 earlier tests passed unchanged with the UH-60A on `none`. |
| Config | ✅ | `Model.InflowTipLossFromConfig`: required; `none` and `effective_disc` read; anything else, or the key missing, is an error. |
| With ground effect and the vortex ring | ✅ | `GroundEffect.ComposesWithTipLossAndTheVortexRing` (code review): all three on, as the UH-60A flies. Hover over the ground at 0.45–2R: λ₀ = κ_g √(C_T/2)/B to 1e-13 and Pitt–Peters at rest there; axial descents through the VRS region: the uniform solve's momentum term is B² times Johnson's curve (1e-12) and Pitt–Peters and the deficit are at rest at it. Caught with the deficit wrongly scaled by 1/B². |
| The wash maps' inflow | ✅ | `GroundEffect.FullDiscInflowRatioMatchesMomentum`: the closed-form ratio against independent bisections of the momentum relation, B 0.9 and 0.97, λ₀ 0.005–0.1, μ 0–0.4, to 1e-12; B in hover, B² edgewise. Through the rotor's coupling output: √(C_T/2) in hover, the ratio times λ₀ in forward flight, λ₀ itself without tip loss. |
| Mutation checks | ✅ | The full disc restored in the uniform solve, the B dropped from the λ₁c coupling, and the B dropped from the apparent mass are each caught. |
| M5 recheck (D-041) | ✅ reported | The M5 anchors are extended as above. The NACA TN 3044 comparison (`eyas_m5_cases`, B = 0.97 on lift, uniform-blade rotor B) stays on `none`, its recorded form; run with `effective_disc` as a probe: steady C_T 1.4 % lower (5.17 → 5.10 × 10⁻³, toward the test, which Eyas overpredicts by 13–22 %), overshoot peak/final 1.226 → 1.220 in the fastest ramp, t₉₀ 0.381 → 0.367 s. The inflow build-up is essentially unchanged. |

**Evidence for the formulation** (`tools/plot_m9_tip_loss.py`, from the reviewed Knight & Hefner readings, DATA_GAPS.md):

    .venv/bin/python tools/plot_m9_tip_loss.py docs/validation/m9

![Knight & Hefner](validation/m9/m9_tip_loss_knight_hefner.png)

- *Thrust against collective* (left; lift slope fitted per reading, θ₀ ≥ 4°): rms 4.7 % with no tip loss, 4.1 % with the lift cut only (GenHel, Eyas before), 4.1 % with Leishman's reading (full-span lift, inflow / B), 3.5 % with Johnson's (D-051); the drift across solidity shrinks the same way (−1.2 to +2.0 % against −2.6 to +3.3 %). The lift slope and the readings can't separate Johnson's reading from Leishman's.
- *Power against thrust* (right; profile drag fitted): rms 14.8 % with ideal momentum, 13.8 % with κ = 1/B; a fitted κ = 1.45 gives 3.4 % (1.32 with a drag rising with the mean lift coefficient; bootstrap 1.20–1.45). Tip loss is in the right direction but a small part of these untwisted rotors' induced losses; the rest, mainly nonuniform inflow, is issue #66.
- *B itself:* Prandtl's B = 1 − √(2C_T)/N (Johnson Eq. 3.108) is 0.968–0.974 for the UH-60A, so GenHel's 0.97 agrees with the wake theory it comes from.

**The UH-60A against Ballin's B1** (16 000 lbf, 5250 ft; Eyas − Sikorsky GenHel and Eyas − flight test, mean over each speed band; `m7c_level.csv` from `eyas_m7c_cases`). "Flat 1/B" is a scratch probe, not kept: the same κ = 1/B on the inflow at every speed, the fallback D-051 named for forward flight (run before the review's wash change, which moves the power by under 5 hp).

| main-rotor power [hp] | hover | GenHel 0–45 kt | GenHel 45–95 kt | GenHel 95–160 kt | test 45–95 kt | test 95–160 kt |
|---|---|---|---|---|---|---|
| before | 1535 | −33 | −34 | −42 | −23 | −36 |
| **effective disc (D-051)** | **1572** | **+10** | **0** | **−19** | **+13** | **−12** |
| flat 1/B (probe) | 1575 | −1 | −27 | −39 | −14 | −33 |

- The effective disc removes most of the cruise power shortfall against the flight test (−36 → −12 hp); the flat factor leaves it (−33). At 45–95 kt the two are about equally far from the test, on either side. The flight test's power at 0–45 kt is 380–440 hp above both (Ballin p. 10: the aircraft needs more power at low speed than either GenHel model).
- *Collective* rises about 1 % of travel at low speed and 0.6 % at cruise: Eyas − flight test −11.4 → −10.3 % (0–45 kt), +2.7 → +3.6 % (45–95 kt), −0.5 → +0.1 % (95–160 kt). Against GenHel the low-speed gap grows (+1.5–2.4 → +2.4–3.3 %) and the cruise gap shrinks (−4.8 → −4.3 % at 160 kt).
- *Over 0–160 kt against GenHel* (`m7c_differences.csv`, mean / rms): power −37 / 50 → −5 / 36 hp; collective +0.04 / 2.38 → +0.82 / 2.69 %; lateral cyclic +1.35 / 2.46 → +1.04 / 2.36 %; pedal +0.02 / 1.51 → −0.23 / 1.43 %; longitudinal cyclic, pitch, sideslip and stabilator within 0.1 in mean.
- *The wash maps' inflow (code review):* first the airframe wash read the effective disc's larger λ₀, which counted tip loss twice on the airframe: pitch at 90 kt went to +0.51° against GenHel (tolerance 0.40°) and the cruise longitudinal cyclic 0.4 % forward. With the full disc's equivalent λ₀ (D-051) pitch is +0.37° and the cyclic as before, the power within 5 hp.

**Records re-recorded (D-035 process):** every tracked open difference of `BallinTrim.*` at its new value (VALIDATION M7c gap register notes; the longitudinal cyclic's calibrated mean 0.28 → 0.33 %); every gated quantity stays within its tolerance; the 137.5 kt B4 power slope moved from +88 to +91 hp per 1000 ft/min against GenHel, past its old record (73.1 ± 16.6). `Uh60aGroundEffect.GrazingWheelIsRejectedOverTheRevolution` became `LightlyTouchingWheelIsRejected` (M9 ground effect above).

**The L3 linearization gate's step at 150 kt** (`Linearize.PeriodicModelMatchesTheNonlinearResponse`): halved to T/1024. With D-051 the 150 kt collective doublet's leftover c₀ came out at 0.00116 against the 1e-3 floor (0.00083 before D-051); at T/1024 it is 0.00021, so it was the step, not the linearization. The test takes 77 s in Release instead of 48 s. `eyas_m7b_cases` keeps T/512 for its sweep.

**Everything else moves little** (plots regenerated 2026-10-22; the statistics quoted in the earlier sections predate D-051 where not noted): the ground-effect ratio up to 0.001 lower; Fletcher's on-axis responses (servo) by at most 0.1 dB and 2.1° rms; Ballin App. D and E (GenHel servo) median rms 1.22 → 1.20 in each figure's units; Kaplita's transients by at most 0.3 in any rms; the VRS encounter (M8d) by about 2 % (the lag on the effective radius).

**Builds:** 470 tests pass (GCC 13.3 Release) and the fast set with Clang 18.1 (Debug); clang-format and clang-tidy clean.

## M9 — Rotor speed, Stage A: shafts in the coupled solve (2026-10-22; issue #39)

The rotor speed becomes a degree of freedom (D-052). Shafts are executive states, (ψ, Ω) each, after the components'; their accelerations join the airframe's in the coupled solve (spec §5), so the shaft's acceleration reaches the blades' loads exactly. Components on a shaft turn at gear × its speed; the rotor reports the torque its blades put on the hub, and its inflow states (velocities over ΩR) follow the tip speed. An `ideal` drive holds the governed speed, the model as before; a `free` one is solved (engines in Stage B). The UH-60A's main and tail rotors are on one `ideal` shaft.

| Check | Status | Evidence |
|---|---|---|
| Closed form: airframe and free shaft | ✅ | `Shafts.FreeShaftWithInternalMotorMatchesClosedForm`: an internal motor, lumped parts, two spin axes, a full inertia tensor; the shaft and body accelerations from angular-momentum conservation, to 1e-12. |
| Conservation with blades | ✅ | `Shafts.RotorOnFreeShaftConservesMomentum`: a tumbling airframe with flapping blades on a free shaft, in vacuum, 1 s; angular momentum about the system CG conserved to RK4 truncation (fourth order in the step), the shaft speed exchanging momentum with the tumbling. It caught the missing gyroscopic term of the shaft's parts while building, and catches the blades' Ω̇ terms dropped. |
| The rotor's shaft terms | ✅ | `Shafts.RotorShaftTermsAreConsistent`: the shaft torque's sensitivity to the body acceleration equals the airframe load's to the shaft acceleration (a symmetric coupled matrix), with and without air; the blades' in-plane inertia; a massless rotor's shaft torque minus gear × its aerodynamic torque. |
| Ideal drive = constant speed | ✅ | `Shafts.IdealDriveIsTheConstantSpeedRotor` (bit-identical derivatives, a run within round-off). The UH-60A on its ideal shaft: all 477 tests pass; Ballin's trims differ by round-off only (roll of order 1e-8 deg); the ground-effect runs identical. |
| Inflow, config, real-time | ✅ | `InflowStatesFollowTheTipSpeed`, `ConfigIsReadAndChecked`, `FreeShaftAllocatesNothing`. `eyas_bench` against `main`: about 4 % on step() (15.4 against 14.8 µs), every `advance()` within budget. |
| Trim and linearization | ✅ (ideal) | Each shaft starts at ψ = 0 at its governed speed; ψ is a phase, not a trim unknown or linear-model state. Free shafts are refused until the engines (Stage B) give them an equilibrium. |

No plots in Stage A: with the ideal drive nothing a plot would show moves. Stage B (engines, governor, clutch) brings the rotor-speed responses and their checks against GenHel and flight test.

## M9 — Rotor speed, Stage B1: GenHel's T700 engines, governor and clutch (2026-10-23; issue #39)

The UH-60A's rotors turn on a `free` shaft driven by GenHel's T700s (CR-166309 §5.6; D-053 and its amendments 2–4): the gas generator, the hydromechanical fuel control and the electronic control unit (ECU) with its high/low-gain switch, re-anchored on GenHel's steady-state maps (Tables 6.5.1–6.5.6, transcribed from the page images); a clutch with a slip state; losses; the spools' momentum. Trim solves the rotor speed and the revolution's period; the linear models carry the shaft's azimuth and speed.

| Check | Status | Evidence |
|---|---|---|
| GenHel's equations as printed | ✅ | `GenHelT700.EcuMatchesGenHelAsPrinted`: driven by the same rotor speed through low gain, high gain and V_SS on its limits, the ECU's states equal an independent transcription of GenHel's incremental equations (`tests/support/genhel_t700_reference.hpp`) to one integration step's overshoot past a limit (2.1e-6 at h = 1e-4 s, 1.2e-6 at h/2). `GasGeneratorMatchesGenHelToSecondOrder`: where GenHel's partials agree with its maps' slopes, the re-anchored gas generator is GenHel's to second order in the departure from trim. |
| Closed forms | ✅ | `SteadyStatesLieOnTheMaps`; `GovernorIsIsochronous` (rotor speed back to 27 rad/s within 1e-7 after a 50 % load step); `ClutchLetsGoAndTakesUp`; `ConservesAngularMomentum` (engine, clutch disengaged, spools on a tumbling airframe); `SwitchIntegratorTurnsAt339FootPounds`. |
| Uniform gravity changes nothing on the shaft | ✅ | `Linearize.GravityAndKinematicEntriesAreExact` (M7b's L1 gate) caught the engine's losses and clutch demand using the rotor's torque without its body-acceleration terms; with them (`LossesAndDemandFollowTheRotorsAccelerationTerms`) it passes at 1e-7 as before. |
| Trim | ✅ | `Trim.Uh60aRotorOnEnginesClosesItsRevolution`: the sea-level hover droops 0.21 % (Ω = 26.944 rad/s, the ECU at A_U) and its revolution closes, Ω T = 2π; at 5250 ft the governor holds 27 rad/s to 1e-6. Every level trim 0–150 kt, sea level and 5250 ft, converges (below); the vortex-ring descents converge with the two-stage cold start. |
| Linearization | ✅ | `Linearize.RotorSpeedIsAStateOnAFreeShaft`: a rotor-speed perturbation at the 5250 ft hover is followed by the periodic linear model with an error falling with the amplitude (1.4e-4, 6.5e-5, 2.7e-5); without the shaft's azimuth as a state it was 1.7 % at every amplitude. `PinnedEngineStatesAreHeld`: the engaged clutch's slip and the limited integrators are held, in the linear model and the flow map (differenced across the switch they gave entries of 2.8e7 and Floquet exponents of 40 and 11). |
| Real-time | ✅ | `GenHelT700.AllocatesNothing`; `eyas_bench` against Stage A on the same machine: step() 6–9 % slower (hover 15.3 to 16.3 µs, 120 kt 15.1 to 16.5 µs), every advance() within budget. |

**Trims.** `eyas_m9_engine_cases` trims the UH-60A in level flight at sea level and 5250 ft (`tools/plot_m9_engines.py`). The load-demand spindle schedules more gas-generator speed than the torque needs at 5250 ft (by 0.9–2.3 %) and the ECU trims it down, holding 27 rad/s; at sea level it schedules 0.17 % too little in hover and at 10 kt, the ECU sits at its upper limit (A_U = 0 V: it can only trim down) and the rotor droops 0.21 %. That droop is within the resolution of GenHel's tables (Table 6.5.1 gives N_GG to 0.1 % at 6000 ft·lbf spacing), so it is reported as a consequence of GenHel's data, not a prediction. Every trim runs in the governor's low gain (140–306 ft·lbf per engine, below the switch at 339).

![Engine trims](validation/m9/m9_engine_trims.png)

**A collective step.** From each hover trim, 0.05 collective (half an inch) held for 10 s, no stability augmentation, against the same step with the rotor speed held (Stage A). The engines' torque jumps within about 0.1 s, faster than the rotor's demand grows, so the rotor speed first rises (0.08 % at sea level, 0.3 % at 5250 ft) before settling slightly low. That is the behaviour Ballin (NASA TM-88360 p. 12, PDF p. 14) reports for GenHel's engine model: its rotor speed "varies adversely in response to a collective input because of an insufficient level of sophistication of the partial-derivative engine model and fuel control system (detailed in ref. 2) [CR-166309]. Its response is also too fast and is of the wrong magnitude. These rotor speed errors manifest themselves in yaw-acceleration, rate-, and heading-time histories". The yaw rate barely differs from the held rotor's.

![Engine collective step](validation/m9/m9_engine_step.png)

**Against flight data.**
- *Fletcher's yaw rate to collective (M7c V2; the plot there now shows the aircraft on its engines).* Mean / RMS, Eyas minus flight:

  | | rotor speed held (Stage A) | on GenHel's engines |
  |---|---|---|
  | hover, magnitude [dB] | −3.4 / 6.3 | −2.5 / 4.6 |
  | hover, phase [deg] | +45 / 61 | +96 / 122 |
  | 80 kt, magnitude [dB] | −8.7 / 9.0 | −10.0 / 10.1 |
  | 80 kt, phase [deg] | +28 / 29 | −103 / 104 |

  The engines bring the hover magnitude closer and take the phase further off in both conditions, as Ballin's account of GenHel's engine model leads one to expect. The engine is GenHel's as printed (the checks above), so the remaining difference is GenHel's engine model's, not the implementation's. Roll and pitch to cyclic are unchanged to within 0.6 dB and 6° (the largest changes at 80 kt below 1.5 rad/s), computed the same way before and after.
- *The FPS's collective-to-yaw path at 20 kt (D-045 M8c3).* After a 0.05 collective step the SAS-only aircraft turns 3.4° in 10 s (3.9° with the rotor speed held), with the FPS 1.12° (1.13°), and with the FPS's collective-to-yaw gain KSXCYO zeroed 0.70° (0.76°): the path still overcorrects, its first-second yaw rate swinging from +0.40 to −1.12 deg/s. The engines don't explain it. Open.

**Found and corrected in this validation (D-053 amendment 4).** The remainder of GenHel's J_REST (868 slug·ft²) had been put at rotor speed about the rotor axis, an upper bound on its angular momentum. It moved hover pitch from lateral cyclic by 15 dB and made the 80 kt phugoid unstable (ζ 0.137 to −0.041). GenHel takes its gyroscopics from its p. 5.10-19 list alone, using J_REST only in the rotor-speed equation; Eyas now does the same, and those responses are back to Stage A's. Also corrected while building (D-053 amendments 2 and 3): limiters past their limit return to it; the tail rotor keeps its own azimuth, so its Tier 4 output no longer jumps at each shaft revolution (`Outputs.TailRotorAzimuthIsContinuous`).

**Decisions with the spec owner (D-053 amendment 2).** Ballin's climb and descent comparison holds the rotor speed: at 60 kt from −2000 ft/min and at 100 kt at −3000 ft/min the rotor needs no power at 100 % and would autorotate, which comes with engine-off flight (Stage B2). The M7a sweep's iteration limit is 15 (the two-stage cold start; measured 12). The Dutch-roll gate at 150 kt is relative (the SAS damps at least 4× the bare aircraft, measured 14×): the free rotor, no longer locked to the airframe in yaw, doubles the bare aircraft's damping there.

**Next (agreed 2026-10-23).** Ballin's component-level T700 model and UH-60A fuel control (NASA TM-100991, 1988; public), validated in GenHel against USAAEFA flight test, with the Ames drive train (TM-88360 App. A): a second engine type, then Stage B2's limits, idle and engine failure on it. It also depends on altitude and temperature, which GenHel's maps don't, and gives fuel flow for a check against USAAEFA's level-flight data (ADA180181 Figs. 32–59; not yet made).

