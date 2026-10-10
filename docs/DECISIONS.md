# Design Decisions

Decisions and departures from `EYAS_DESIGN.md`, each with a date and rationale. The spec owner approves the entries. Status is one of `Proposed`, `Approved (<date>)` or `Rejected (<date>)`.

---

## D-001 Reference editions to cite — `Approved (2026-10-10)`
**Decision:** Cite these editions in code comments and docs:
- Padfield, *Helicopter Flight Dynamics*, **3rd ed.** (2018);
- Leishman, *Principles of Helicopter Aerodynamics*, **2nd ed.** (2006);
- Johnson, *Rotorcraft Aeromechanics* (2013);
- Howlett, NASA CR-166309 (1981).

**Rationale:** These are the copies available to the project. The spec allows Padfield 2nd or 3rd. Equation numbers differ between editions, so we fix one per book.

**Pitt–Peters source:**
- **Primary:** Pitt & Peters, "Theoretical Prediction of Dynamic-Inflow Derivatives", 6th ERF, Paper 47, Bristol (1980). This is the paper republished as *Vertica* 5(1):21–34 (1981), which the spec cites. The final analytic L and M matrices are in Table 4 (p. 47-13). The choice of the corrected thrust column and the uncorrected moment columns is explained on p. 47-9.
- **Cross-check:** Johnson (2013) §11.3, eqs. 11.141–11.146, p. 433. It uses a different state ordering and signs: (λ0, λc, λs), with loads (C_T, −C_My, C_Mx).
- **Supporting only:** Pitt & Peters, 9th ERF, Paper 55 (1983). It defines the loading integrals with standard normalization (eq. 2). Its Table 1 is a *different*, 5-state model; **do not use it for the 3-state matrices.**
- **Open conflict for M5 (SPEC_REVIEW #10):** the moment apparent mass is 16/45π in the 1980 Table 3/4 but 64/45π in Johnson 11.146. The hover moment-row L magnitudes agree (2/V), so the moment time constants differ by 4×. Resolve this by reconciling the moment-coefficient normalizations (1980 eq. 5, whose OCR is poor, so read the image) before implementing Pitt–Peters.
- **Choice deferred to M5:** Johnson uses V_eff (11.144) for the thrust column and √(μ²+λ²) (11.145) for the moment columns, while Pitt & Peters used 11.144 for all. Decide which, and record it here.

## D-002 Rigid-body mass excludes modeled rotor blades (SPEC_REVIEW #2) — `Approved (2026-10-10)`
**Decision:**
- `M_rb` and `I_b` describe the airframe *without* the blades that are modeled as their own components. The rigid-body reference point is the fixed CG of that airframe.
- The config takes **airframe-only inertias directly**. For the UH-60A these are the GenHel motion-module values, used as written (p. 5.10-19, PDF p. 342; digits confirmed by human review): I_x = 4659.0, I_y = 38512.0, I_z = 36796.0, I_xz = 1882.0 slug·ft².
- Airframe weight and CG are derived from the total-helicopter values minus the blades, as GenHel does (W_bd, FSCGB, WLCGB; p. 5.1-39). Use GenHel's own expressions from the main-rotor equations, §5.1.2, once they're located and verified. Total WEIGHT is 16638.0 lb (p. 5.1-49).
- The output CG and inertias for the whole aircraft are computed from the airframe and the current blade states.

**Rationale:**
- This matches GenHel §5.10.1 ("aircraft less rotor blades") and avoids double counting through H.
- Using GenHel's inputs unchanged means Eyas reproduces the model that was validated against flight test. Differences against GenHel or NASA results then point to our implementation, not to different mass properties.
- Deriving our own airframe inertias from Table 4.2 was rejected for that reason.

**Known discrepancy (deferred; revisit if yaw or roll dynamics disagree with reference data):** the GenHel motion-module inertias don't obviously exclude the blades. I_z ≈ the whole-aircraft Table 4.2 value, where ≈ 6970 slug·ft² less would be expected, and I_x is below every Table 4.2 loading condition. No Table 4.2 row matches the 16638 lb total weight. The derivation is undocumented in Vol. I. Details are in DATA_GAPS.md, "mass properties".

## D-003 Inter-component data path and config-driven evaluation order (SPEC_REVIEW #3) — `Approved (2026-10-10)` — interface change
**Decision:**
- **Declared dependencies:** each component instance declares the coupling outputs it *produces* and *consumes*, e.g. `rotor[0].wake_skew`, `rotor[0].torque`.
- **Order computed once:** at `init()` the executive sorts components topologically so producers run before consumers. Ties break by config order, so the result is deterministic. The order is then fixed for the run. A dependency cycle is an `init()` error that names the cycle. The UH-60 order this produces is main rotor → tail rotor → airframe → engine.
- **Coupling outputs:** shared per-evaluation results go in a pre-allocated `CouplingOutputs` store, with fields **grouped by producer instance** (not a singleton "main rotor"). It is sized at `init()` and rebuilt on every derivative evaluation, so no state is kept between evaluations and purity is preserved. Consumers see it read-only through `SimContext`.
- **Read-before-write check:** in debug builds every field is reset to NaN at the start of each evaluation, and a consumer reading an unwritten field asserts.
- **Field discipline:** a field is added only when some component consumes it. Fields never change meaning, units or frame. Unit tests populate only the fields their component reads.

**Rationale:**
- Fuselage and empennage downwash, tail-rotor fin blockage and engine torque all need values from the same evaluation. GenHel passes exactly these between modules (p. 5.1-38).
- A config-driven order keeps multi-rotor configurations, e.g. tiltrotors (D-011), possible without executive code changes.

**Known limits and notes:**
- **One-way flow per evaluation.** Two-way interactions within one evaluation (airframe upwash on the rotor, tiltrotor wing–rotor interference) must either depend only on states and inputs already in `SimContext` (GenHel's airframe upwash on the main rotor appears to; confirm when implementing), or be handled by an explicit, documented choice: bounded fixed-point iteration, a deliberate simplification, or a lag. Record each case here.
- **Coupled solve.** Airframe force models consume rotor *outputs* that don't depend on body acceleration `a`, so they add no H terms. If a consumed output depends on `a`, its consumer can't take it from the coupling store and must take part in the coupled solve. **Known M9 case:** rotor shaft torque includes blade inertial terms that depend on body yaw acceleration, so the drivetrain and rotor-speed DOF must be in the coupled solve, not a plain consumer of `torque`.

## D-004 Stateful gear and stabilator; const `ForceModel` (SPEC_REVIEW #4) — `Approved (2026-10-10)` — interface change
**Decision:**
- `ForceModel::compute` becomes `const`.
- Landing gear becomes a `DynamicComponent`, with per-contact tire friction states (a bristle or anchor-point displacement) and G = H = 0.
- The stabilator schedule plus its actuator becomes a `DynamicComponent` when rate or lag dynamics are modeled. Before that it stays a `ForceModel`.

**Rationale:** Stick–slip needs memory, or the aircraft creeps on slopes. A non-const `compute` contradicts the purity requirement.
**Notes for M6:**
- Contact states must reset smoothly at touchdown and liftoff, with no discontinuity in the derivative function.
- Check tire and strut stiffness against RK4 stability at the physics step, and record the chosen stiffness and damping, with their sources, in DATA_GAPS.md.

## D-005 Timing budget defined per host frame (SPEC_REVIEW #5) — `Approved (2026-10-10)`
**Decision:** Replace §7 rule 5. The rule becomes: worst-case `advance()` for a nominal host frame, i.e. worst-case `step()` × ceil(dt_host / dt_physics), must stay below 50% of `dt_host` on the target hardware. The benchmark reports per-`step()` and per-`advance()` mean, p99 and max.
- At 60 Hz host and 1 kHz physics (17 sub-steps), that means worst-case `step()` ≲ 0.49 ms.
- **Target hardware (spec §12 Q4, answered by the spec owner on 2026-10-10):** a Linux server running RHEL 8 or RHEL 9. This is a general-purpose Linux server, not an embedded RTOS. The budget is measured there, with the release build configuration.

**Rationale:** The spec's per-`step()` rule allows an 8 ms step inside a 16.7 ms frame that contains about 17 steps.
**Notes:**
- **Scheduling jitter is the host's job.** A standard RHEL kernel gives no hard real-time guarantee; worst-case latency comes from scheduling, not from Eyas's arithmetic. Thread priority, CPU isolation, memory locking and the optional RHEL real-time kernel are host-side decisions (Eyas creates no threads). Eyas's part: no allocation, syscalls or I/O inside `advance()` (§7), and all buffers touched at `init()` so the loop takes no first-touch page faults.
- **The benchmark should report max and p99** under a realistic host load, not only on an idle machine.

## D-006 Integer-nanosecond `advance()` (SPEC_REVIEW #6) — `Approved (2026-10-10)` — interface change
**Decision:**
- The public time input is an integer: `advance(controls, env, std::int64_t dt_host_ns)`. **There is no `double`-seconds overload.** Tools and tests convert explicitly at their own boundary, which keeps one code path and makes rounding visible to the caller.
- Internally nothing changes from spec §6.10: sim time = `n_steps × step_ns`, the integer remainder carries between calls, sub-steps per call are capped, and `snapshot()`/`restore()` include `n_steps` and the remainder.
- The M1 time-keeping test is restated with integer inputs. Over any sequence of calls, the total sub-steps must equal floor(Σ dt_host_ns / step_ns) exactly. Include an irregular, jittered sequence.

**Rationale:** with integer input, Eyas's clock can never drift from the host's. The host application is being rewritten (spec owner, 2026-10-10), so it can keep integer time.
**Guidance for the host team** (host-side choices, recorded here so they aren't lost):
- Keep host time as an integer, e.g. a monotonic nanosecond clock or a frame counter with an integer period.
- **Prefer a host rate whose period is a whole number of physics steps.** At 1 kHz physics, 50, 100, 125, 200, 250 or 500 Hz all qualify (a 20, 10, 8, 5, 4 or 2 ms period). Every frame then runs the same number of sub-steps, with no 16/17 alternation and no remainder, which gives steadier frame timing (D-005) and outputs that align to frame boundaries. 60 Hz still works correctly through the remainder carry.
- Decide whether the host passes the *nominal* frame period (deterministic, independent of wall-clock jitter) or the *measured* elapsed time (tracks the wall clock, but not reproducible). Nominal is recommended for validation runs.

## D-007 M3 hover verification: layered oracle (SPEC_REVIEW #7) — `Approved (2026-10-10)`
**Decision:** replace the spec's "within 2% of BEMT" with three layers. Layers 1 and 2 form the M3 gate; layer 3 is a validation entry.

1. **Exact-equation reference (gate; catches implementation bugs).**
   - A separate, deliberately simple and slow reference written in the test suite computes the *same exact* blade-element and uniform-inflow equations as Eyas. It uses the same velocity geometry (`atan2`, full U) and the same test airfoil model, high-order or adaptive quadrature, and inflow solved by robust root-finding.
   - Eyas's **unmodified production code path** must agree with it to ≈ 1e-9 relative as elements are refined. Use a convergence check, not a single-element-count comparison.
   - No test-only switches go into the blade-element loop. (A "small-angle mode" was considered and rejected: it would test a different code path from the one that flies.)
2. **Literature anchor (gate; catches misunderstood physics).**
   - Compare against Leishman 2nd ed. §3.2 (verified 2026-10-10):
     - C_T = ½σC_lα[θ₀/3 + θ_tw/4 − λ/2] (Eq. 3.25), with θ(r) = θ₀ + rθ_tw;
     - C_P = λC_T + σC_d0/8 (Eq. 3.31);
     - hover λ = √(C_T/2) (Eq. 3.32), solved simultaneously with C_T.
   - Use a test config that matches the closed form's assumptions: constant chord, C_l = C_lα·α (no stall or Mach effect), C_d = C_d0, r from 0 to 1 (no root cutout), B = 1, no flapping.
   - The closed form makes small-angle approximations (φ ≈ λ/r, U ≈ U_T), so an explained residual remains, largest near the root. Set the tolerance from an element-refinement study, and record the value and its explanation in VALIDATION.md.
3. **Experimental data (validation entry, not a gate).**
   - Compare C_T vs θ₀ against Knight & Hefner (1937) as plotted in Leishman 2nd ed. Fig. 3.2, p. 121 (σ = 0.042, 0.064, 0.085, 0.106).
   - Digitizing that figure is a human-review item: plot readings are uncertain, so record the digitized points and have them checked.
   - Use a looser band, appropriate to 1937 experimental data.

**Rationale:**
- BEMT and uniform-inflow blade-element theory are different models, so the spec's 2% band can fail without a bug, or pass with one.
- Each layer covers a different failure. Layer 1 catches errors in the code, layer 2 catches errors in our understanding of the physics, and layer 3 catches a model that is self-consistent but wrong about the real world.
## D-008 Tail-rotor azimuth resolution deferred to M9 (SPEC_REVIEW #8) — `Approved (2026-10-10)`
**Decision:** Keep the 1 kHz default. The v1 Bailey tail rotor has no azimuth dependence, so the 7.1° per step at Ω_TR = 124.62 rad/s does not matter for it. Before any blade-element tail rotor (M9), choose internal sub-stepping, a higher physics rate, or a disk-averaged model, and record that choice here.

## D-009 M4 flap-frequency criterion reworded (SPEC_REVIEW #9) — `Approved (2026-10-10)`
**Decision:** M4 tests the rotating flap frequency against ν_β. It tests the non-rotating frequency (ω = sqrt(K_β / I_β)) only for configurations with K_β > 0. The UH-60 has K_β = 0 (GenHel p. 5.1-49, pending DATA_GAPS review).

## D-010 Build flags for warnings and determinism (SPEC_REVIEW #11) — `Approved (2026-10-10)`
**Decision:**
- Third-party includes (Eigen, yaml-cpp, GoogleTest) are SYSTEM includes.
- `-ffast-math` and its sub-flags are forbidden.
- `-ffp-contract=off` is set explicitly on all targets.
- The determinism guarantee is documented as holding per build configuration and compiler.

**Rationale:** `-Werror` would otherwise fail on warnings inside Eigen with GCC 12+. FMA contraction differs across optimization levels and targets.

## D-011 Keep the architecture open to tiltrotors — `Approved (2026-10-10)` — scope
**Context:** a tiltrotor is a likely future target (spec owner, 2026-10-10). v1 remains the UH-60A single-main-rotor helicopter (SPEC_REVIEW #13).
**Decision:** don't build tiltrotor features now, but don't hard-code assumptions that would block them:
- **Rotors:** rotor components are generic, N-instance components, each with its own config: rotation direction, position, geometry. There is no singleton "main rotor" in the executive, the coupling store, or `AircraftState` internals. `AircraftState` may still present convenience fields for single-rotor aircraft.
- **Hub frame:** a rotor's body-to-hub transform comes from config *plus* an optional time-varying input (nacelle or mast angle). v1 passes a constant. The gyroscopic and hub-motion terms must not assume a fixed shaft, so derive them for a hub frame that may rotate relative to the body.
- **Order:** the evaluation order is config-driven (D-003).
- **Control mapping:** it sits behind an interface, so a mode- or conversion-dependent mixer can replace the UH-60 linear gearing.
- **Drivetrain:** a component, not part of the main rotor, so a cross-shaft linking two rotors fits later.
- **Not now:** wing aero, nacelle-tilt dynamics, cross-shaft dynamics, the conversion control laws, wing–rotor interference modeling.

**Rationale:** these choices cost little in v1 and avoid a restructure later. Padfield 3rd ed. Ch. 10 covers tiltrotor modeling.
**Data:** none is available (see DATA_GAPS.md, "Tiltrotor data").

## D-012 Data file formats, placeholders and init-time errors (M0) — `Approved (2026-10-10)`
**Decision:**
- **Tables** are CSV in "long" form, one row per grid point, in any order; every grid point must appear exactly once (`src/util/csv_table.hpp`).
  - Each header cell is `name[unit]`. Values are converted to SI on load through an explicit unit registry (`src/util/units.cpp`).
  - A non-empty `# source:` comment is **mandatory**, so no table enters the model without provenance.
  - `# status: placeholder` marks a whole table as a data gap.
- **Config quantities** are YAML strings with an explicit unit, e.g. `radius: "26.83 ft"`. A quantity without a unit is an error, so a missing unit can never be silently read as SI. Dimensionless values are plain numbers.
- **Placeholders:** a config value is marked with the YAML tag `!placeholder`, e.g. `blade_weight: !placeholder "256.9 lbf"`. *Departure from spec §0 rule 3*, which says to use a `# PLACEHOLDER — needs source` comment: a YAML parser cannot see comments, so the comment form can't be detected or reported. With the tag, `Config::placeholders()` lists every gap and init can report them.
- **Units:** `lb` is rejected as ambiguous; write `lbf` (force) or `lbm` (mass). GenHel uses "LB" for both weights and forces, so every transcription must state which it means.
- **Table lookups clamp** at the edges (no extrapolation). A NaN query returns NaN rather than an error, so it shows up in diagnostics.
- **Init-time errors** throw `eyas::InitError`, naming the file, line or key path. Nothing reachable from `advance()` throws (spec §7 rule 2); the loaders are init-only.
- **Parsing** uses `std::from_chars`, never streams or `strtod`, so results don't depend on the C locale.

**Rationale:** these make data provenance and units checkable by the loader rather than by convention, which directly supports the "never invent data" and SI-at-load rules.

## D-013 M1 rigid-body acceptance tests strengthened (SPEC_REVIEW #14) — `Approved (2026-10-10)`
**Decision:** the M1 criteria stay, with these additions:
- **Attitude anchor.** For constant body rate ω, compare the integrated q_nb against the closed-form solution q(t) = q0 ⊗ [cos(|ω|t/2), sin(|ω|t/2) ω/|ω|]. The norm-drift check stays as a diagnostic, with the test's body rate stated in the test.
- **Intermediate-axis instability.** Test, for a spin Ω about the intermediate principal axis (I₁ < I₂ < I₃) with a small perturbation:
  - growth in the linear phase at λ = Ω√((I₃ − I₂)(I₂ − I₁)/(I₁I₃)), the rate obtained by linearizing Euler's equations (Padfield 3rd ed. Eq. 3A.24 and its rotational counterpart, App. 3A);
  - a sign reversal of ω₂ (the flip).

  Do not test the state at the end of the run. Spins about the major and minor axes stay bounded.
- **Conserved quantities.** Measure angular momentum in the inertial frame (H_n = C_nb I_b ω_b), so the check also exercises the attitude integration.

**Rationale:** as written, the norm criterion is met by round-off, and the end-state instability check can be fooled by the periodic flip (SPEC_REVIEW #14).
