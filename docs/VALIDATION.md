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
  - The descent sweep below converges everywhere in ≤ 14 passes. It also shows momentum theory's vortex-ring gap: the result jumps between root branches at about 3 v_h. Logged as SPEC_REVIEW #21 (open).
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
