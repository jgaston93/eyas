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
