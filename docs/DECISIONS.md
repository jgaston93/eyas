# Design Decisions

Decisions and departures from `EYAS_DESIGN.md`, each with a date and rationale. The spec owner approves the entries. Status is one of `Proposed`, `Approved (<date>)` or `Rejected (<date>)`.

---

## D-001 Reference editions to cite — `Approved (2026-10-10)`
**Decision:** Cite these editions in code comments and docs:
- Padfield, *Helicopter Flight Dynamics*, **3rd ed.** (2018);
- Leishman, *Principles of Helicopter Aerodynamics*, **2nd ed.** (2006);
- Johnson, *Rotorcraft Aeromechanics* (2013);
- Howlett, NASA CR-166309 (1981);
- *U.S. Standard Atmosphere, 1976*, NOAA/NASA/USAF, NASA-TM-X-74335 (added 2026-10-10, M2; D-015). Pages are cited by the printed book page; PDF page = book page + 16 in Part 1 and + 15 in the Part 4 tables (Table I starts on book p. 51 = PDF p. 66);
- *Department of Defense World Geodetic System 1984*, NGA.STND.0036_1.0.0_WGS84 (2014-07-08), the successor to TR8350.2 (added 2026-10-10, M2; D-017).

**Rationale:** These are the copies available to the project. The spec allows Padfield 2nd or 3rd. Equation numbers differ between editions, so we fix one per book.

**Pitt–Peters source:**
- **Primary:** Pitt & Peters, "Theoretical Prediction of Dynamic-Inflow Derivatives", 6th ERF, Paper 47, Bristol (1980). This is the paper republished as *Vertica* 5(1):21–34 (1981), which the spec cites. The final analytic L and M matrices are in Table 4 (p. 47-13). The choice of the corrected thrust column and the uncorrected moment columns is explained on p. 47-9.
- **Cross-check:** Johnson (2013) §11.3, eqs. 11.141–11.146, p. 433. It uses a different state ordering and signs: (λ0, λc, λs), with loads (C_T, −C_My, C_Mx).
- **Supporting only:** Pitt & Peters, 9th ERF, Paper 55 (1983). It defines the loading integrals with standard normalization (eq. 2). Its Table 1 is a *different*, 5-state model; **do not use it for the 3-state matrices.**
- **Resolved by D-027 (2026-10-12): 16/45π.** Was an open conflict for M5 (SPEC_REVIEW #10): the moment apparent mass is 16/45π in the 1980 Table 3/4 but 64/45π in Johnson 11.146. The hover moment-row L magnitudes agree (2/V), so the moment time constants differ by 4×. Resolve this by reconciling the moment-coefficient normalizations (1980 eq. 5, whose OCR is poor, so read the image) before implementing Pitt–Peters.
- **Decided in D-027 (2026-10-12): √(μ²+λ²) on all columns of the total-state form.** Was deferred to M5: Johnson uses V_eff (11.144) for the thrust column and √(μ²+λ²) (11.145) for the moment columns, while Pitt & Peters used 11.144 for all. Decide which, and record it here.

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
- **Read-before-write check:** in debug builds every field is marked unwritten at the start of each evaluation, and a consumer reading an unwritten field asserts. (M6a review: a per-field flag, not a NaN fill, because a producer may legitimately write NaN from a diverging RK4 stage, which the executive must be able to reject in debug as in release.)
- **Field discipline:** a field is added only when some component consumes it. Fields never change meaning, units or frame. Unit tests populate only the fields their component reads.

**Rationale:**
- Fuselage and empennage downwash, tail-rotor fin blockage and engine torque all need values from the same evaluation. GenHel passes exactly these between modules (p. 5.1-38).
- A config-driven order keeps multi-rotor configurations, e.g. tiltrotors (D-011), possible without executive code changes.

**Known limits and notes:**
- **One-way flow per evaluation.** Two-way interactions within one evaluation (airframe upwash on the rotor, tiltrotor wing–rotor interference) must either depend only on states and inputs already in `SimContext` (GenHel's airframe upwash on the main rotor appears to; confirm when implementing), or be handled by an explicit, documented choice: bounded fixed-point iteration, a deliberate simplification, or a lag. Record each case here.
- **Coupled solve.** Airframe force models consume rotor *outputs* that don't depend on body acceleration `a`, so they add no H terms. If a consumed output depends on `a`, its consumer can't take it from the coupling store and must take part in the coupled solve. **Known M9 case:** rotor shaft torque includes blade inertial terms that depend on body yaw acceleration, so the drivetrain and rotor-speed DOF must be in the coupled solve, not a plain consumer of `torque`.

**Implementation notes (2026-10-12, M6a):**
- `DynamicComponent` gained `name()`, `produces()`, `consumes()` and `bind()`, all with empty defaults, so existing components are unchanged. Names are scalar fields, `"<instance>.<field>"`; `CouplingRegistry` maps them to `CouplingSlot`s at init and each component resolves its slots in `bind()`.
- The store is `CouplingOutputs`, one `std::vector<double>` sized at init. Producers write through `ComponentDerivs::coupling`; consumers read through `SimContext::coupling`. Both are null when a component is evaluated on its own, as in unit tests.
- Force models take no part: they are evaluated before every component and can't consume outputs, so their `SimContext::coupling` is null. None needs to yet (the M6 airframe aerodynamics is a `DynamicComponent`, D-028).
- The state layout stays in config order; only the evaluation order is sorted (`Executive::evaluation_order()`).

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
- Debug builds of Eyas targets use `-Og` (added 2026-10-10, M1). At `-O0`, Eigen made the M1 1-hour time-keeping test take about 5 min (about 84 µs per physics step, against 0.54 µs in Release). `-Og` keeps debug info, assertions and sanitizers.

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
- **Placeholders:** a config value is marked with the YAML tag `!placeholder`, e.g. `blade_weight: !placeholder "256.9 lbf"`. *Departure from spec §0 rule 3*, which says to use a `# PLACEHOLDER — needs source` comment: a YAML parser cannot see comments, so the comment form can't be detected or reported. With the tag, `Config::placeholders()` lists every gap and init can report them. The host sees them through `Model::placeholders()` (D-025).
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

## D-014 `AdvanceStatus.non_finite` and a unit quaternion in `SimContext` — `Approved (2026-10-10)` — interface change
**Context:** the M1 code review found that a singular coupled solve, or a component returning NaN or Inf, silently filled the whole state with NaN, and `quat_norm_error()` still looked healthy (`std::max` ignores NaN). The real-time rules (spec §7) require failures to be flagged, not hidden. The same review found that inside RK4 stages components received the stored, slightly off-unit quaternion in `SimContext::body.q_nb` and used it to rotate vectors.
**Decision:**
- Add `bool non_finite` to `AdvanceStatus`, the status `advance()` returns. A sub-step whose result is not finite, or whose quaternion has zero norm, is discarded. The state stays at the last good step, the rest of that call's time is dropped, and the flag is set. Deterministic inputs fail the same way again, so the host sees the flag on every later call until it intervenes (`reset`/`restore`).
- `SimContext::body.q_nb` is always unit length. The executive normalizes it once per derivative evaluation; q̇ is still computed from the stored quaternion.

**Host handling (guidance for host adapters, which live in the host codebase):** treat `non_finite` as a flight-model fault.
1. Freeze: stop advancing, or keep calling and accept that the state holds; keep presenting the last good outputs.
2. Alert the operator that the flight model has faulted and the simulation is frozen.
3. Log the failing call's controls, environment and time, plus a snapshot of the last good state. Eyas is deterministic, so this reproduces the fault offline.
4. Recover only on an operator's decision: `restore` a recent snapshot (the usual choice), or `reset` to a trim condition if the fault recurs.

Do not restore and continue automatically and silently: an input-driven fault recurs at the same point and loops, and a model bug goes unreported. (`snapshot`, `restore` and `reset` arrive with the M2 API; until then a host can only freeze, alert and log.)

**Rationale:** additive only; no existing field changes meaning. Detecting ill-conditioned but finite solves (e.g. with an `rcond` estimate) is left until the rotor components exist and there is a real H to size a threshold against.

## D-015 ISA: U.S. Standard Atmosphere 1976, geopotential altitude, hydrostatic ΔT (SPEC_REVIEW #15) — `Approved (2026-10-10)`
**Decision:**
- **Source:** *U.S. Standard Atmosphere, 1976* (USSA). It is identical to the ICAO standard atmosphere up to 32 km (Abstract, p. v).
- **Constants** (USSA Table 2, p. 2, with the corrections below): g₀ = 9.80665 m/s², P₀ = 101325 Pa, T₀ = 288.15 K, r₀ = 6 356 766 m, R* = 8.31432 × 10³ N·m/(kmol·K), γ = 1.40. M₀ = 28.9644 kg/kmol (p. 9). Layers: H_b = 0 and 11 km′, with L_M,b = −6.5 and 0.0 K/km′ (Table 4, p. 3).
  - Table 2 *as printed* gives R* = 8.31432 × 10⁻³ and r₀ = 6.356766 × 10⁶ km. Both are print errors. The text gives R* = 8.31432 × 10³ (p. 3) and r₀ = 6,356,766 m (p. 8, below Eq. 17), and the Symbols page (p. xi) gives 6356.766 km. Eyas uses the text values.
- **Altitude:** geometric altitude Z above mean sea level converts to geopotential H = r₀Z/(r₀ + Z) (USSA Eq. 18, Γ = 1 m′/m).
- **Equations, ΔT = 0:** T = T_b + L_b(H − H_b); P from Eq. 33a (L_b ≠ 0) or Eq. 33b (L_b = 0); ρ = P M₀/(R* T) (Eq. 42); a = √(γR*T/M₀) (Eq. 50). Below 80 km, T_M = T (p. 9).
- **Range:** layers 0 and 1, so up to H = 20 km′. Above that, the isothermal layer is extended and an out-of-range flag is set. Below sea level, layer 0 is extended (USSA allows down to −5 km′, p. 12).
- **ISA + ΔT is hydrostatic:** T(H) = T_std(H) + ΔT, and sea-level pressure stays P₀. Pressure comes from the same hydrostatic integration as Eqs. 33a/b, with T_b replaced by T_b + ΔT in every layer (including the pressure at the 11 km′ boundary). With ΔT = 0 this reduces exactly to the USSA equations.
- **Tests:** USSA Table I rows at Z = 0, 1000, 5000 and 11000 m (geometric table) and H = 1000, 5000 and 11000 m′ (geopotential table), within the table's rounding. The hydrostatic relation dP/dZ = −ρ g(Z), with g(Z) from Eq. 17, is checked by finite differences for ΔT ≠ 0, so the offset case has an independent check.

**Rationale:** USSA is the public primary definition. The geopotential conversion costs one division. Hydrostatic ΔT is physically consistent with an altitude taken from the state; the performance-chart convention is meant for pressure altitude.

## D-016 Atmosphere computed inside Eyas; `Environment` carries conditions (SPEC_REVIEW #15) — `Approved (2026-10-10)` — interface change
**Decision:**
- The `Environment` passed to `advance()` becomes the host's *conditions*:
  - `isa_delta_T` [K];
  - `wind_n` [m/s];
  - `ground_height` [m, above MSL].
  The ρ, T and a fields are removed.
- In every derivative evaluation the executive evaluates the atmosphere at the CG's geometric altitude, h = h₀ − D, where h₀ is the geodetic origin's altitude (from config, D-017). It puts the result in `SimContext::air`: ρ, P, T, a, h and the out-of-range flag. The derivative function stays a pure function of (t, x, inputs).
- Models that need air at another point (a rotor hub, a tail) evaluate it themselves from the same function. The vertical gradient is about 1e-4 per metre in ρ, so the CG value is acceptable for v1.

**Rationale:** density follows the aircraft inside each frame and inside RK4 stages, and the atmosphere is a function of the state, not of when the host sampled it.

## D-017 Tier 1 geodetic position: flat-earth map on WGS-84 (SPEC_REVIEW #16) — `Approved (2026-10-10)`
**Decision:**
- **Origin:** config gives a geodetic origin (lat₀, lon₀ in WGS-84; h₀ in metres above MSL) at which NED (0, 0, 0) sits.
- **Map:**
  - lat = lat₀ + N/(R_M + h₀);
  - lon = lon₀ + E/((R_N + h₀) cos lat₀);
  - alt = h₀ − D.
  - R_M = a(1 − e²)/(1 − e² sin²lat₀)^{3/2} and R_N = a/√(1 − e² sin²lat₀) are the meridian and prime-vertical radii of curvature at the origin.
- **Constants:** a = 6378137.0 m and 1/f = 298.257223563 (NGA.STND.0036, Eq. 3-2, p. 3-2), with e² = f(2 − f).
- **Anchors for the radius expressions:** the NGA document doesn't print them, so they're checked against values it tabulates:
  - at the pole R_M = R_N = R_p = 6399593.6258 m (Table 3.5, p. 3-9);
  - at the equator R_N = a and R_M = b²/a, with b = 6356752.3142 m (Table 3.5);
  - at intermediate latitudes, R_M is checked as the curvature of the meridian ellipse by finite differences of an independently computed meridian.
- **Longitude** is wrapped to (−180°, 180°]. At |lat₀| > 89°, init is rejected (cos lat₀ → 0).
- **Datum:** the altitude is MSL height, used for the atmosphere. Geoid–ellipsoid separation is not modelled; a host that needs ellipsoidal height adds it.

**Rationale:** level flight in NED stays level geodetically, which keeps the out-the-window view consistent with the terrain. Horizontal distortion grows slowly with range (the cos lat change over the area), which is acceptable for a flat-earth model; WGS-84 round-earth dynamics remain a later option (spec §4).

## D-018 M2 tests, dummy rotor and the first public API (SPEC_REVIEW #17) — `Approved (2026-10-10)` — interface change
**Decision:**
- **Equilibrium:** keep the spec's case, which is exact at level attitude (the test asserts bit-exact zero drift). Add a case at a non-level attitude with a steady velocity and an inertially fixed force, with the tolerance derived from round-off. A constant force gives a neutral equilibrium, so no disturbance-recovery test (spec note).
- **Dummy rotor:** a `ForceModel` you can select in config (`type: ideal_thrust`) with thrust T = T_max·collective along −z_b, massless and with no moment. This is test scaffolding until M3, and lets the host team integrate Tier 1 early. The force step is a collective step through `advance()`.
- **Public API** (`include/eyas/`): an `eyas::Model` class with `init` (from a config file or string, throws `InitError`), `advance(controls, env, dt_ns)`, `outputs() → const AircraftState&`, `reset(body_state)`, `snapshot(Snapshot&)` and `restore(const Snapshot&)`. `make_snapshot()` allocates the buffer at init time, so `snapshot()`/`restore()` don't allocate (spec §7). `trim()` arrives with M7, not as a stub.
- **`AircraftState` Tier 1:** sim time [s, ns]; lat, lon [rad], alt [m MSL]; q_nb; Euler angles (φ, θ, ψ_h) [rad] in Padfield's yaw–pitch–roll order. They are extracted from C_bn = Γ = ΦΘΨ (Padfield 3rd ed. Eqs. 3A.34–3A.40): θ = −asin Γ₁₃, φ = atan2(Γ₂₃, Γ₃₃), ψ_h = atan2(Γ₁₂, Γ₁₁). ψ_h is in [0, 2π), and θ is clamped to ±π/2 against round-off; plus the `AdvanceStatus` flags.

## D-019 Environment input checks and atmosphere range reporting (SPEC_REVIEW #18) — `Approved (2026-10-11)` — interface change
**Decision:** two fields are added to `AdvanceStatus` (additive; no existing field changes meaning).
- **`rejected_env`:** `advance()` checks `Environment` once, before any sub-step. If it is invalid, the call does nothing (no time passes, the remainder is kept) and sets `rejected_env`, exactly like `rejected_dt`. Valid means:
  - `isa_delta_T` is finite and the absolute temperature stays positive everywhere in the profile. In layers 0 and 1 the coldest point is the isothermal layer at T_std = 216.65 K (USSA Table 4, p. 3: 288.15 K − 6.5 K/km′ × 11 km′), so the test is ΔT > −216.65 K. No other bound is applied, since no public source gives an operational ΔT range.
  - `wind_n` is finite.
  - `ground_height` is not checked yet (unused until M6; NaN is its default).
- **`atmosphere_out_of_range`:** set if the CG's geopotential altitude was outside [−5, 20] km′ (D-015) after any sub-step committed in the call. It is computed from the committed state after each step, not inside RK4 stages, so the derivative function stays a pure function of (t, x, inputs) (D-016). The atmosphere is still evaluated (extended) out of range.

**Rationale:** a rejected call follows the existing `rejected_dt` precedent and never runs physics on inputs the host didn't mean. One check per call costs nothing measurable. Checking the committed state is one division per step, and RK4 stage points lie within one step's travel of it.

## D-020 Public API boundary: `include/eyas/` only, implementation behind a pointer — `Approved (2026-10-11)` — interface change
**Decision:**
- The library API is exactly the headers in `include/eyas/`: `model.hpp`, `aircraft_state.hpp`, `types.hpp` (`Vec3`, `Quat`, `Controls`, `Environment`, `AdvanceStatus`, `BodyState`) and `error.hpp` (`InitError`). They include nothing from `src/`.
- `Model` holds its implementation (executive, geodetic map) behind `std::unique_ptr<Impl>`. It is movable, not copyable. `Model(const Config&)` is removed from the API; hosts use `from_file`/`from_string`.
- In CMake, `src/` is a PRIVATE include path of `eyas` and yaml-cpp a PRIVATE dependency; Eigen stays PUBLIC because the API value types use it. Unit tests and validation generators get the internals through an `eyas_internal` interface target.
- `eyas_api_tests` is built with only `include/` on the path, so an internal header leaking into the API fails the build.
- The types keep their names, namespace, fields and meaning: only their header moved (`src/core/types.hpp` and `src/util/error.hpp` include the public ones).

**Rationale:** with `src/` public, a host compiled against Eyas internals, and any internal refactor could break host builds. One pointer indirection per call is negligible against a physics step. Found in the M2 code review.

## D-021 Strict config reading — `Approved (2026-10-11)`
**Decision:** extends D-012.
- **Dimensions:** every registered unit has a dimension (length, time, frequency, mass, force, angle, per angle, speed, angular rate, acceleration, density, moment, mass moment, inertia, dimensionless). `Config::quantity(key, dimension)` rejects a unit of any other dimension. Frequency (Hz) and angular rate (rad/s, rpm) are distinct, since they differ by 2π. CSV table columns are converted by scale only for now; their expected dimensions arrive with the table schemas that use them.
- **Unknown keys:** each config section is checked against the keys its reader knows (`Config::allow_keys`). A misspelled optional key is an error, not a silent default. New sections and keys are added to the allowed lists as the code that reads them lands.
- **Error messages:** every init error from a config, including checks made later inside a constructor, reads `<file>: <full key path>: <message>`.

**Rationale:** each of these let a wrong config load and run with no diagnostic: `physics_rate: "1000 s"`, `latitude: "37 m"`, or a misspelled `force_models` that silently removes the rotor. Found in the M2 code review.

## D-022 M3 reference-model comparison is a validation entry, not a gate (SPEC_REVIEW #19) — `Approved (2026-10-11)`
**Decision:** the spec's M3 criterion "C_T against collective matches the reference model within 3%" becomes a validation entry: it reports how close Eyas is, and doesn't block M3. The M3 gates are D-007 layers 1 and 2.
- **Case (for whoever runs the reference model; set up to match Eyas, §2.1):** UH-60A main rotor from the public GenHel geometry (radius, blade count, chord, twist, airfoil tables, rotor speed, tip loss, root cutout, as in Eyas's config); isolated rotor; hover, out of ground effect, no wind; ISA sea level; flapping locked at zero (rigid blades); uniform momentum inflow; a collective sweep at fixed rotor speed covering negative to near-stall thrust. Outputs: C_T, C_Q and C_P against collective (θ₀ at the hub center, and θ at 0.75R).
- **Tolerance:** |ΔC_T| ≤ max(0.03·C_T, 0.03·C_T,hover), where C_T,hover is the C_T at the UH-60A design gross weight (16638.0 lbf, DATA_GAPS.md) at sea level. The floor stops the percentage from blowing up near zero thrust.
- **Recorded with the result:** the reference model's version, the case setup, and every modeling assumption that might differ (inflow, tip loss, element count, airfoil table handling), so differences can be read sensibly.
- **Prerequisites:** the UH-60A blade data (transcribed after M3) and the license check (SPEC_REVIEW #12). The comparison test skips when the external data directory is absent (§2.1).

**Rationale:** the reference model's assumptions can't be inspected, so a percentage against it can't separate an Eyas bug from a modeling difference. The comparison is still useful as a sanity band.

## D-023 Bounded iterations report convergence to the executive, not to the host — `Approved (2026-10-11)` — interface change (internal)
**Decision:**
- `ComponentDerivs` gains `bool converged` (true by default). A component with an iterative solve (in M3, the uniform-inflow solve) sets it false when the solve hit its iteration limit.
- The executive counts derivative evaluations with an unconverged component, per `advance()` call (`Executive::unconverged_evaluations()`). Tests and validation tools read the count.
- **Not in the public API.** `AdvanceStatus` and `AircraftState` are unchanged. The host can't act on it: an unconverged solve gives slightly wrong loads, not a fault. Revisit when Tier 5 diagnostics are finalized (M8).

**Rationale:** spec §7 rule 3 requires a convergence flag on every bounded iteration. Once Pitt–Peters (M5) is the default, the uniform-inflow iteration runs only when chosen in config.

## D-024 M3 blade-element rotor (SPEC_REVIEW #20) — `Approved (2026-10-11)`
**Decision:**
- **No blade mass in M3.** The M3 rotor is aerodynamic only: no blade weight, inertial loads or gyroscopic terms (H = 0, G = 0). Blade mass arrives with the flap DOF in M4, where the hub-motion inertial terms are derived and tested (spec §6.4). An M3 rotor therefore isn't for flying the UH-60A: its airframe mass excludes the blades (D-002).
- **Frames.** Hub frame h: origin at the hub center, z_h down along the shaft, x_h forward, y_h right; with zero hub angles it is parallel to body axes. Config gives the hub position (body axes, from the airframe CG) and the hub orientation as Euler angles (pitch, roll) relative to body; a forward shaft tilt is a negative pitch. The blade direction is e_r = (−cos ψ_k, s sin ψ_k, 0), with s = +1 for counter-clockwise from above and −1 for clockwise, so ψ_k = 0 points aft and increases with rotation (spec §4; Padfield 3rd ed. §3.2.1).
- **Azimuth.** ψ = Ω t mod 2π, from the evaluation time (Ω constant until M9; SPEC_REVIEW #10). Blade k (k = 1…N_b) is at ψ_k = ψ − 2π(k−1)/N_b, as in Padfield 3rd ed. Eq. 3.49 (*departure from spec §6.4*, which has +), so the M4 multiblade transform can follow Padfield.
- **Element velocities and angles (Leishman 2nd ed. §3.2 convention, *departure from "per Padfield" in §6.4*).** U_T is the in-plane velocity of the element relative to the air along the direction of rotation; U_P is the air velocity through the disc, positive *down* (inflow), so U_P = V_c + v_i in climb (Leishman Eq. 3.2). φ = atan2(U_P, U_T) and α = θ − φ, wrapped to [−π, π) for reverse flow. Velocities include hub translation, body rates and rotor rotation; the induced velocity is uniform and along the shaft. The radial component U_R is ignored (independence principle, Leishman §3.1). Leishman notes U_R affects drag in forward flight; revisit with the forward-flight validation (M4–M7).
- **Loads.** dL = ½ρU²cC_l dr and dD = ½ρU²cC_d dr (Leishman Eq. 3.4) with U² = U_T² + U_P², resolved as lift perpendicular and drag parallel to the relative wind (Leishman Eq. 3.5). The code uses the equivalent vector form, which holds in every quadrant, including reverse flow. The Mach number is U/a. ρ and a are the CG values (D-016).
- **Pitch.** θ = θ₀ + θ_tw(r) + θ_1c cos ψ_k + θ_1s sin ψ_k (Padfield 3rd ed. Eq. 3.59 plus spec §6.3 twist). θ_tw(r) is a linear twist or a table against r/R. The δ₃ term is zero while flapping is held at zero.
- **Span.** Midpoint-rule elements on two meshes: N_lift elements on [r_root, B·R] and N_tip on [B·R, R], N_tip = max(1, round(N(R − B·R)/(R − r_root))), each with uniform or equal-area spacing. Lift is zero outboard of B·R and drag is kept. Padfield 3rd ed. §3.2.1 (assumption 3) describes tip loss as integrating the loads to an effective tip; keeping the tip drag is our choice, so profile power isn't understated. Check it against GenHel's main-rotor equations when the UH-60A data is transcribed. *Checked (2026-10-11): GenHel applies its tip-loss factor to lift only (CR-166309 p. 5.1-27); see DATA_GAPS.md.* No loads inboard of the root cutout.
- **Airfoil.** One airfoil per blade in v1. A CSV table of C_l and C_d against (α, M), with α covering [−180°, 180°] for reverse flow (spec §6.4); columns are checked for dimension (D-021).
- **Uniform inflow (spec §6.5, 0 states).** λ_i = C_T / (2√(μ² + (λ_i − μ_z)²)) (Padfield 3rd ed. Eq. 3.139), with μ and μ_z from the hub velocity relative to the air in hub axes (Eq. 3.140) and C_T from the blade-element loop in the same evaluation, at the blades' instantaneous azimuths. Solved in every evaluation from a fixed start (no warm start, so the derivative function stays pure) on g(λ_i) = 2λ_i√(μ² + (λ_i − μ_z)²) − C_T(λ_i), which has no singularity at λ_i = 0: a bracket from λ_i = 0 and the momentum estimate for that pass's C_T, then Brent's method (Brent 1973, Ch. 4) to |Δλ_i| ≤ 1e-13. At most 30 blade-element passes (6 in typical forward flight, ≤ 14 across a hover-to-windmill descent sweep); a capped solve sets `converged = false` (D-023). In the vortex-ring region momentum theory has several roots; the solve returns one of them deterministically (SPEC_REVIEW #21). This is the M5 oracle for "Pitt–Peters reduces to uniform momentum inflow in hover".
- **Controls (spec §6.3, D-011).** Each rotor owns a linear map from `Controls` to (θ₀, θ_1s, θ_1c): offset + gains, from config. Mechanical mixing is a nonzero cross gain. It sits behind an interface, so a nonlinear or mode-dependent mixer can replace it. Sign conventions of `Controls` (documented in `include/eyas/types.hpp`): collective 0 = full down; lon_cyclic +1 = full forward stick; lat_cyclic +1 = full right stick; pedal +1 = full right pedal. Actuator dynamics are not modeled (optional in v1).

**Rationale:** each item settles an ambiguity in SPEC_REVIEW #20 in the way that keeps the M3 oracle (D-007) and the M4 derivations aligned with their references.

## D-025 Report data gaps through the public API — `Approved (2026-10-11)` — interface change (public API)
**Decision:** extends D-012, which said init "can report" placeholders but gave the host no way to see them.
- **One list per config document.** `Config::placeholders()` lists the full key path of every `!placeholder` value, as before. A reader that loads a CSV table marked `# status: placeholder` adds it with `Config::note_placeholder(key, what)`, as `"<key path>: <file> (# status: placeholder)"`. In M3 these are the blade's `chord_table`, `twist_table` and `airfoil`.
- **Public API:** `const std::vector<std::string>& Model::placeholders() const noexcept`, init-time. It lists every data gap the model was built with, and is empty when every value and table is sourced.
- **No failure.** Placeholders still load (D-012). Eyas does no I/O, so the host decides whether to log, display or refuse a model with gaps.

**Rationale:** the UH-60A blade data is transcribed next, and some tables may start as placeholders. Without this, a model running on placeholder data was indistinguishable from a sourced one.

## D-026 M4 rigid flapping blades: model and layered acceptance tests (SPEC_REVIEW #22) — `Approved (2026-10-11)`
**Model:**
- **Blade:** rigid and thin (mass on the span line), hinged at e from the hub centre on the blade's radial line. The hinge axis is perpendicular to the span and lies in the hub plane. No lag hinge (M9) and no pitch inertia. Flap β is positive up (toward −z_h).
- **Mass data (config, per blade):** mass m_b, and S_β and I_β about the hinge. These are GenHel's W_b, M_b and I_b (p. 5.1-49). GenHel gives W_b as a weight in lb, so the config writes it as the same number in lbm (exact under standard gravity). New units: `N*m/rad`, `ft*lbf/rad` (spring) and `N*m*s/rad`, `ft*lbf*s/rad` (damper). Init checks m_b > 0, S_β > 0 and I_β ≥ S_β²/m_b (Cauchy–Schwarz). The blade CG is at the hinge plus S_β/m_b.
- **Hinge:** optional linear spring K_β about β = 0 and damper C_β. No precone and no flap or droop stops in M4; stops are logged for M6 (ground handling).
- **States:** [β_k, β̇_k] for each blade k, in the D-024 numbering (ψ_k = ψ − 2π(k−1)/N_b). They start at zero (`initial_states` has no context); trim sets them in M7.
- **Equation:** exact rigid-body kinematics with no small-angle approximation. Each blade point is at r_hub + e e_r + ρ(cos β e_r − sin β e_z). Its inertial acceleration includes the CG acceleration (v̇_b + ω × v_b), ω̇ × r, ω × (ω × r), rotor rotation at constant Ω relative to the hub, and flap motion. Moments about the hinge give
  I_β β̈ = M_aero + M_grav − K_β β − C_β β̇ − M_inertial(β, β̇, ω, Ω) − J·a,
  which is affine in a = [v̇_b; ω̇_b], so G comes directly from J. The span integrals reduce to m_b, S_β and I_β. The hub-frame angular velocity is written as one variable (ω_b today), so a tilting nacelle (D-011) only adds ω_hb terms to it.
- **Loads on the airframe:** treated as a free body, Σ over blades of ∫(df_aero + g dm − a dm). Spring and damper moments are internal and cancel. With Ω prescribed, the reaction to the drive torque (including Coriolis torque from flapping) passes to the airframe. H comes from the a-dependent part of the blade accelerations.
- **Aerodynamics on the flapped blade:** element velocity relative to the air is resolved on the flapped blade axes: U_T along e_t, and U_P along the blade normal n = sin β e_r + cos β e_z (positive down, as in D-024). The induced velocity is along z_h and projected onto n. U_R is still ignored (D-024). θ = θ_swash + θ_tw − tan δ₃ · β (spec §6.3). Sign checked against Johnson 2013 §6.18: Δθ = −K_P β with K_P = tan δ₃, so positive δ₃ means flap up, pitch down. The uniform inflow balances the **aerodynamic** thrust only, i.e. the aero force along the shaft.
- **No flap section in the config = the M3 rotor** (massless, β ≡ 0, no states), so the M3 tests and the D-022 locked-flap case are unchanged.
- **Outputs:** multiblade β₀, β_0d, β_1c, β_1s (Padfield 3rd ed. Eqs. 3.42–3.45). Tests and outputs use Padfield's β_1c and β_1s. The classical a₁, b₁ notation is not used.

**Acceptance (replaces spec M4; same layering as D-007):**
1. **Exact-equation reference (gate).** An independent test-side model of each blade as many point masses. Point positions are written as explicit functions of time under a prescribed body motion with constant a, and differentiated numerically (high-order central differences with Richardson extrapolation) to get velocity and acceleration. This doesn't reuse the production vector algebra. From it, hinge-moment balance gives β̈, and the free-body sum gives the airframe load. The aerodynamics reuse the M3 exact reference, fed with these numerically derived element velocities. The production β̈₀, G, load₀ and H must agree, under random states, rates, accelerations, controls, wind and hub tilt, both rotation directions and nonzero e, K_β, C_β and δ₃. This is also the spec's "G and H against finite differences".
2. **Conservation (gate; exercises the coupled solve through the production executive).** With no air and no gravity, an airframe tumbling with flapping blades conserves the total linear and angular momentum (airframe plus blades, computed independently in the test). With gravity, the system's momentum changes by the total weight × t. These checks catch any G/H inconsistency.
3. **Literature anchors (gate, with tolerances set from small-angle residual studies and recorded in VALIDATION.md):**
   - free flap frequency in vacuum with the hub fixed: ν² = 1 + e S_β/I_β + K_β/(I_βΩ²) (Johnson Eq. 6.261); the spec's uniform-blade formula for a uniform fixture; non-rotating √(K_β/I_β) only when K_β > 0 (D-009);
   - linearized hub-motion flap forcing from p, q, ṗ, q̇ and ẅ, at small β: Johnson Eq. 16.240 (offset hinge) and Padfield Eq. 3.20 (e = 0);
   - hover coning against θ₀ and λ, and so against C_T, including the weight moment: Johnson Eq. 6.277 (offset hinge, constants integrated over the lifting span) plus §6.9;
   - β_1c and β_1s against μ: Padfield Eqs. 3.57–3.58 and 3.64 for a centre-spring fixture (e = 0, K_β > 0), and Johnson Eqs. 6.277–6.279 for an offset-hinge fixture. Tight below μ ≈ 0.1, reported up to 0.3 (the closed forms drop the 2/rev flapping);
   - cyclic step in hover: the steady phase lag matches Johnson Eqs. 6.278–6.279 at μ = 0, i.e. 90° − atan(N*/d₁) with N* = 8(ν² − 1)/γ (Padfield Eq. 3.40 for e = 0). The transient settles with the 16/(γΩ) time constant (Padfield Eq. 3.41), reported only;
   - δ₃: the effective flap frequency rises as in Johnson §19.6.
   Fixtures: linear airfoil, constant chord, root cutout at the hinge, B = 1 — the closed forms' assumptions.
4. **Reference model (validation entries, not gates; D-022 pattern):** coning against thrust, and β_1c, β_1s against μ, on a defined UH-60A isolated-rotor case. These skip without the external data directory.
- **Real-time rules:** zero allocation, purity and bit-identical runs, as for M3.

**Implementation notes (2026-10-11):**
- The layer 1 reference is `tests/support/flap_reference.hpp`. It uses second-order forward-mode automatic differentiation (Taylor jets) in time, not numerical differences. That makes it exact to round-off, so the gate is 1e-7 relative on loads and 1e-6 rad/s² on β̈ (the inflow solves agree to ~1e-13). It takes the production element mesh, which the M3 tests verify on their own.
- The layer 3 tolerances come from measured residuals, which grow with μ as the closed forms drop the 2/rev flapping and O(μ⁴) terms:
  - flapping: 1.5% of the disc tilt up to μ = 0.1 (measured ≤ 1.1%) and 3% at μ = 0.2 (≤ 2%); μ = 0.3 is reported only;
  - coning: 1% + 0.002° (measured ≤ 0.53%; the floor covers coning near zero);
  - phase lag: 0.3° (measured ≤ 0.11°);
  - amplitude: 1% (measured 0.33%);
  - δ₃ stiffness: 1%;
  - hub force tilt: 3% of the disc tilt.
- Johnson's offset-hinge β is the tip-line angle, β_J = (1 − e/R)·β_hinge, and his γ uses the mode generalized mass, so γ = ρacR²(R − e)²/I_β. The conversion is in `tests/support/flap_closed_forms.hpp`. Johnson's hinge-spring term for e > 0 uses a different normalization, so spring cases are tested only with e = 0.

**Rationale:** the offset-hinge closed forms come from Johnson and the centre-spring ones from Padfield, so each analytic test compares like with like. Layers 1 and 2 test the exact production path, so the closed-form tolerances only have to cover small-angle and harmonic-truncation residuals.

## D-027 M5 Pitt–Peters dynamic inflow: model and layered acceptance tests (SPEC_REVIEW #10, #23) — `Approved (2026-10-12)`
**Sources:** Pitt & Peters 1980 (6th ERF Paper 47) Table 4, p. 47-13, is primary (D-001). Johnson (2013) Eqs. 11.132–11.134 and 11.143 are the cross-check, with the moment apparent mass taken from Johnson p. 437, not Eq. 11.146 (SPEC_REVIEW #10 findings).

**Model:**
- **States:** three inflow states [λ₀, λ_1s, λ_1c] per rotor, owned by the rotor component (spec §6.4) and appended after the flap states. The induced velocity is λ(r, ψ) = λ₀ + (r/R)(λ_1s sin ψ + λ_1c cos ψ) (spec §6.5; 1980 eq. 1), positive down along z_h. ψ is the blade azimuth of D-024, so the same form holds for both rotation directions. r is the element's radius in the hub plane, e + ρ cos β (D-026). As in D-024 and D-026, the velocity is projected onto the flapped blade normal.
- **Forcing:** aerodynamic loads only (Johnson Eq. 11.142; Leishman 2nd ed. Eq. 10.123). Each element's force normal to the hub plane, positive up (dF_up), is summed in the same blade-element pass that uses the inflow:
  C_T = ΣdF_up/(ρA(ΩR)²), C_1s = ΣdF_up (r/R) sin ψ_k /(ρA(ΩR)²), C_1c = ΣdF_up (r/R) cos ψ_k /(ρA(ΩR)²).
  In the 1980 notation C_L = −C_1s and C_M = −C_1c (eqs. 5b–5c). C_T is the D-024/D-026 aerodynamic thrust.
- **Equations (nonlinear, total states), in wind axes w:**
  M̃ dλ_w/dτ + V L̂(X)⁻¹ λ_w = C_w, with τ = Ωt,
  - M̃ = diag(128/75π, 16/45π, 16/45π) (1980 Table 4, signs absorbed by C_1s and C_1c);
  - L̂ = [[1/2, 0, −(15π/64)X], [0, 2(1+X²), 0], [(15π/64)X, 0, 2(1−X²)]], i.e. V × the 1980 Table 4 L with columns 2–3 negated, written with Johnson's identities 4/(1+cos χ) = 2(1+X²) and 4 cos χ/(1+cos χ) = 2(1−X²) (11.133–11.134; 1980 sin α = cos χ);
  - λ = λ₀ − μ_z (total flow down through the disc; μ, μ_z as in D-024), V = √(μ² + λ²), and X = tan(χ/2) = μ/(V + |λ|) (Johnson Eq. 11.132), with X = 0 at V = 0. L̂ is invertible for every X in [0, 1].
  - **Upflow (λ < 0) uses |λ|, as Johnson does.** The 1980 paper's disc angle α = tan⁻¹((λ̄+ν̄)/μ) (eq. 23c, p. 47-10) is signed, but the paper derives L only for 0° ≤ α ≤ 90° (conclusions 3–4, same page), so upflow is outside its range. Taken literally, a signed α puts the wake above the disc (χ > 90°, X > 1). det L̂ = 1 − (1 − (15π/64)²) X² then reaches zero at X ≈ 1.48 (χ ≈ 112°, λ ≈ −0.37 V), i.e. at modest upflow in forward flight. With |λ|, upflow is treated as the mirror image of downflow. That keeps L̂ invertible. The region (autorotation, windmill brake) is one where momentum theory is doubtful anyway (SPEC_REVIEW #21). The test-side 1980 transcription uses |λ| too, so the layer-1 gate doesn't test this choice.
  - **Mass flow:** one V = √(μ²+λ²) for all columns. On the thrust row this is required for the steady state to equal momentum theory, and its linearization gives the differential-momentum gain (Johnson Eq. 11.144, 1980 eq. 23b) and τ = M₁₁/(4λ₀Ω) in hover. On the moment rows it is Johnson's recommendation (Eq. 11.145) and the 1980 potential-theory value (eqs. 20, 23a). *This departs from Pitt & Peters (1981), who used eq. 23b on the moment columns too, which halves the hover moment gain.* Hover moment lift deficiency: 1/(1 + σa/(8λ₀)) (Johnson Eq. 11.153; Johnson quotes C′ ≈ 0.5 for moments in hover).
- **Wind axes (extension; neither source covers it):** both sources put the free stream at ψ = 180°. For in-plane hub velocity (v_x, v_y) in hub axes, the downwind azimuth ψ_w has cos ψ_w = v_x/|v_xy| and sin ψ_w = −s v_y/|v_xy| (ψ_w = 0 in forward flight; ψ_w = 0 when |v_xy| = 0, where the first-harmonic block of L̂ is isotropic, since X = 0). The first harmonics rotate as (λ_1c, λ_1s)_w = (λ_1c cos ψ_w + λ_1s sin ψ_w, −λ_1c sin ψ_w + λ_1s cos ψ_w), and the same for (C_1c, C_1s). The result is rotated back to hub axes.
- **Body coupling:** the inflow equations don't depend on the body acceleration, so G = 0 for the inflow rows. H is unchanged.
- **No iteration:** one blade-element pass per derivative evaluation, so `converged` is always true for a Pitt–Peters rotor. This is faster than the uniform-inflow solve (6–30 passes).
- **Initial states:** zero (`initial_states` has no context; trim sets them in M7, as for the flap states, D-026). This departs from SPEC_REVIEW #10's suggested momentum start: the uniform inflow depends on the flight condition and controls, which `initial_states` can't see, and a momentum start without trimmed flap states would still give a start-up transient. Until M7, a Pitt–Peters rotor starts with zero inflow, so its thrust overshoots for about M₁₁/(4λ₀Ω) after `init()` or `reset()`, more than with uniform inflow. At V = 0 the equation reduces to M̃ λ̇ = C, so the inflow builds up from zero with no singularity. `BladeElementRotor::evaluate_uniform()` gives the uniform momentum inflow whatever the configured model, i.e. the steady hover start for tests and tools.
- **Vortex ring (SPEC_REVIEW #21, still open):** the steady state is the momentum equation, so the multiple roots remain, and the dynamics pick a branch by history. Reported in VALIDATION.md, not fixed.
- **Config:** a `inflow:` key on the rotor, `uniform` or `pitt_peters`, **required** (D-021: an explicit choice, no silent default). The C++ struct default stays `uniform`, so the existing M3 and M4 tests are unchanged.
- **Outputs:** `RotorLoads` gains λ_1s, λ_1c, C_1s and C_1c (λ_i stays λ₀). Internal only, no public API change.

**Acceptance (replaces spec M5; same layering as D-007 and D-026):**
1. **Exact-equation reference (gate).** A test-side implementation that transcribes the 1980 Table 4 L and M in the paper's own notation ((λ₀, λ_s, λ_c), (C_T, C_L, C_M), α with sin α = cos χ, eq. 1 inflow). Its loads come from the M3/M4 exact reference fed with the linear inflow, and the wind-axis rotation is done independently: the downwind azimuth ψ_w is found by root search on the reference's own blade kinematics, not from production's closed form. The production dλ/dt must agree under random states, μ, μ_z, sideslip, both rotation directions and flapping. A second check transcribes Johnson Eq. 11.143 with (λ₀, λ_c, λ_s) and (C_T, −C_My, C_Mx), and M from p. 437, and must agree too, which tests the sign reconciliation. The load integrals C_1s and C_1c are checked against the hub aerodynamic moments for unflapped blades.
2. **Closed-form anchors (gate; fixtures: linear airfoil, C_d = 0, constant chord, B = 1, no cutout; tolerances from residual studies, recorded in VALIDATION.md):**
   - *Steady hover → uniform momentum (spec):* integrated to steady state through the production executive, λ₀ equals the D-024 uniform solve to 1e-9, and λ_1s = λ_1c = 0, also in axial climb.
   - *Inflow time constant in hover (spec):* with the loading held fixed, τ = (128/75π)/(4λ₀Ω) (1980 Table 4 with eq. 23b; Padfield 3rd ed. Eqs. 5.76 and 5.80 with C₀ = 1). The moment time constant is τ_m = 2(16/45π)/(λ₀Ω). Both come from finite-difference Jacobians of the inflow equations.
   - *Coupled hover eigenvalues:* through the production rotor with rigid blades at fixed pitch, s₀ = −(Ω/M₁₁)(4λ₀ + σa/4) and s₁ = −(Ω/M₂₂)(λ₀/2 + σa/16), from δC_T = −(σa/4)δλ₀ (Leishman 2nd ed. Eq. 3.25; Johnson Eq. 11.148) and δC_1s = −(σa/16)δλ_1s (Johnson Eq. 11.152).
   - *Step collective (spec):* small step δθ₀, rigid blades, body fixed: C_T jumps by (σa/6)δθ₀, then decays exponentially with rate s₀ to C′ times the jump, C′ = 1/(1 + σa/(16λ₀)), so the overshoot ratio is 1/C′. Large steps and flapping blades are reported only.
   - *Wake skew:* with thrust-only loading, λ_1c/λ₀ = (15π/32) tan(χ/2) (1980 eq. 22b / Table 4), and rotating the in-plane wind by an angle rotates (λ_1c, λ_1s) by the same angle.
3. **Experimental data (validation entry, not a gate; amended 2026-10-12):** the slow collective ramps of Carpenter & Fridovich, NACA TN 3044 (1953), Fig. 3(b)–(d), on rotor B of NACA TN 1698, with the lift slope from NACA Report 824 (NACA 23015). The readings and the rotor data were confirmed by human review (DATA_GAPS.md). The case and its assumptions are in `tests/validation/m5_cases.cpp`; the result is reported in VALIDATION.md, with no tolerance.
   - *Replaces the reference-model step-collective comparison.* The reference model available is a generic medium helicopter, not a UH-60A, and matching its rotor would need its parameter values, which the clean-room rule (§2.1) forbids. With different rotors, a mismatch couldn't be told apart from a model error, so it would carry no signal for M5. Whole-aircraft comparisons are left to M7/M8.
   - *The fast ramp (≈ 200°/s, Fig. 3(a)) is excluded:* blade inertia is ≈ 38% of its peak thrust (TN 3044 p. 14), and the blade inertia and CG are not published.
- **Real-time rules:** zero allocation, purity and bit-identical runs, as for M3 and M4.

**Decision notes (approved 2026-10-12; revisit if flight-test comparisons disagree):**
- *Moment mass-flow parameter (a departure from Pitt & Peters 1981).* Choice: V = √(μ²+λ²) on the moment rows. Alternative: the 1981 practice of eq. 23b on all columns. Effect in hover: the moment gain is 2/λ₀ rather than 1/λ₀, and the moment time constant 2(16/45π)/(λ₀Ω) rather than (16/45π)/(λ₀Ω). For the UH-60A (σa ≈ 0.47, λ₀ ≈ 0.075) the hover moment lift deficiency is C′ ≈ 0.56 rather than ≈ 0.72. Why: Johnson's recommendation (Eq. 11.145, pp. 431–433: harmonic loading doesn't change the mean mass flow), the 1980 paper's own potential-theory result (eqs. 20, 23a; eq. 23b was offered there as an unverified extension), and Johnson's quoted typical C′ ≈ 0.5 for moments in hover. This mainly affects the pitch and roll response to cyclic in hover (damping and off-axis coupling). If M7/M8 hover pitch and roll derivatives or frequency responses disagree with flight data, eq. 23b on the moment columns is the first alternative to try. It is a one-line change, and the gate tests are written in terms of V_m so they follow it.
- *Wake-skew direction (our extension; no source).* Choice: the skew terms are applied in wind axes aligned with the in-plane hub velocity and rotated back to hub axes. Alternative: the published form, which assumes the free stream comes from ψ = 180° whatever the flight direction. Why: with the published form, sideways and rearward flight put the fore–aft inflow gradient on the wrong axis, giving wrong flapping and hub moments in hover-taxi, crosswind and sideward-flight tasks. The rotation is the published model applied in a frame where its assumption holds. It changes nothing in pure forward flight (ψ_w = 0) or at μ = 0 (isotropic), so it can't affect the forward-flight validation. Not verified against data: check sideward-flight trim (lateral cyclic and pedal against lateral speed) against flight data in M7 if available, and treat any disagreement as a possible cause.

**Rationale:** the 1980 paper and Johnson p. 437 agree on the apparent masses, and the two L matrices agree once the load signs are reconciled, so the model rests on two consistent readings. One V for all columns in the total-state form gives momentum theory in the steady state and the published hover time constant on linearization. The closed-form gates use the same rigid-blade, linear-airfoil fixtures as M3 and M4, so their tolerances only cover the small-angle residuals.

## D-028 M6 tail rotor, airframe aerodynamics and landing gear (SPEC_REVIEW #24) — `Approved (2026-10-12)`
**Sources:** GenHel CR-166309 is primary for the model structure and the UH-60A data: §5.2 fuselage (PDF pp. 86–122), §5.3 empennage (pp. 123–161), §5.4 tail rotor (pp. 162–176), Fig. 5.5.1.7 stabilator (p. 194), §5.7 landing interface (pp. 239–292). Every equation is re-derived or checked against Padfield 3rd ed., Johnson (2013) or Leishman 2nd ed. where they cover it, and cited in code.

**Rule for purity replacements.** Where a GenHel equation uses remembered values (a previous pass, a time delay, mode logic), the replacement must match or beat GenHel against the literature: the same steady state, and dynamics with a published or derivable form. Its error against GenHel's form is measured by a test, not argued, and reported in VALIDATION.md. A replacement that loses fidelity in a way that matters is not accepted because of the purity rule. The alternative is then a declared discrete state, proposed as an interface change.

**Staging.** M6 is split into four steps, each its own PR with its own gate tests:
- **M6a** — the coupling store (D-003, first use) and the tail rotor;
- **M6b** — fuselage, empennage and stabilator incidence;
- **M6c** — landing gear;
- **M6d** — the UH-60A aircraft config (main rotor, tail rotor, airframe, gear) and the whole-aircraft checks.

**Coupling store (D-003).** First fields, per rotor instance: λ₀ (the uniform part of the induced inflow: the Pitt–Peters state, or the uniform solve), wake skew χ, longitudinal flapping (GenHel's a₁; its relation to Padfield's β_1c, expected a₁ = −β_1c, is to be confirmed from GenHel §5.1.2), Ω and R. Per fuselage: α_WF and ψ_WF (GenHel's wind-tunnel angles, consumed by the empennage and tail-rotor maps). None depends on the body acceleration, so the consumers add no H terms (D-003, "Coupled solve").

**Tail rotor (GenHel §5.4.2: Bailey closed form, thrust only).**
- *Thrust:* C_T = (aσ/2)(λ t₃₁ + θ t₃₂ + θ_tw t₃₃) with Bailey's coefficients including tip loss B (p. 5.4-7), λ = μ_z − λ_i. To be checked against Padfield 3rd ed. Eq. 3.62 (B = 1) when implemented.
- *Inflow lag (replaces GenHel's previous-pass λ):* one inflow state per tail rotor, (128/75π) dλ_i/dτ = C_T − 2λ_i√(μ² + λ²), the Pitt–Peters thrust row of D-027. Its steady state is exactly GenHel's algebraic inflow, and its time constant has a published source instead of depending on the frame rate. Hover estimate for the UH-60A tail rotor: about 7 ms with the lift-deficiency feedback, so h|s| ≈ 0.15 at 1 kHz (RK4-stable). *Alternative considered:* an algebraic solve in every evaluation (D-024 style), which has no lag at all; rejected, since it drops a real effect and costs an iteration.
- *δ₃ (replaces GenHel's T(t−1)):* θ = (θ_cmd + BIAS) − (∂a₀/∂T)·T·tan δ₃ (p. 5.4-7), with T from the same evaluation. For a given λ_i, T is linear in θ, so this is a closed-form solve, with no iteration.
- *δ₃ sign (M6a review):* the solve divides by 1 + (aσ/2)·t₃₂·k, with k = (∂a₀/∂T)·tan δ₃·K_BLK·ρπR²(ΩR)². With δ₃ < 0, k < 0 and the denominator falls as ρ and μ² grow, with no bound that can be checked at init; for the UH-60A tail rotor at sea level it reaches zero near δ₃ = −73° in hover. `init()` therefore rejects δ₃ < 0 when ∂a₀/∂T > 0. A rotor with negative δ₃ would need a different treatment.
- *Interference:* main-rotor wash from the same maps as the stabilator (GenHel: "same as horizontal tail for Black Hawk"), fuselage sidewash and dynamic-pressure ratio from the fin maps, with the delay below. Fin blockage K_BLK against airspeed as GenHel (p. 5.4-8).
- *Ω_TR/Ω_MR* is 1 until the rotor-speed DOF (M9).
- *Torque:* none, as in GenHel. GenHel gives no tail-rotor profile-drag data, and estimating it would be inventing data. Logged in DATA_GAPS.md; revisit with the M7 power comparison and the M9 drivetrain.

**Fuselage (GenHel §5.2.2).** Six wind-axis coefficients (in ft² and ft³) from 1-D tables in α_WF and ψ_WF (Tables 2.5.3–2.5.11), at the tunnel mounting point, transformed to body axes at the CG (p. 5.2-9). Rotor wash is added as GenHel does (uniform downwash × maps of χ and a₁, Tables 2.5.1–2.5.2). As in GenHel, body-rate velocities at the mounting point are ignored, and the low-speed filter is not used (p. 5.2-10: "not being executed on PDP10"; its hover parameters aren't given).

**Empennage (GenHel §5.3.2).** One stabilator panel and one fin panel, with local velocities including body rates, rotor wash maps, fuselage downwash/sidewash and dynamic-pressure ratios, and lift/drag tables (Tables 3.5.1–3.5.10).
- *Transport delay (replaces GenHel's delay):* each delayed quantity ε (stabilator downwash, fin sidewash, tail-rotor downwash and sidewash: 4 states) gets a first-order lag written as dε_d/dt = (|u|/l)(ε − ε_d), with l the distance aft of the CG. This matches the pure delay to first order in frequency (e^{−sτ} ≈ 1/(1 + sτ)), so the downwash-lag pitch damping is kept. At u → 0 the state freezes, which is harmless because the wash velocity is proportional to u. The airframe aerodynamics is therefore a `DynamicComponent` with G = H = 0 (D-004).
  - *Known error:* the lag departs from the delay as ωτ grows. With the stabilator arm (≈ 28.7 ft), τ ≈ 0.17 s at 100 kt and ≈ 0.42 s at 40 kt. At 3 rad/s that is −27° against −29° of phase (amplitude 0.89) at 100 kt, and −52° against −73° (amplitude 0.62) at 40 kt. The error grows at low speed, where the delayed fuselage wash is itself small (proportional to u) and the undelayed rotor wash dominates.
  - *Why not an exact delay:* a history buffer would need a declared discrete state outside the state vector. It would also be invisible to the M7 finite-difference Jacobians, so the linear models would lose the downwash-lag damping (M_ẇ) entirely; the lag keeps it.
  - *Check (M6b):* a test compares the lag with the exact delay on UH-60A geometry from 20 to 150 kt and 0.5 to 10 rad/s (gain and phase of the delayed wash, and of the resulting stabilator lift), and VALIDATION.md reports it. If the error is significant in the piloted band (about 1–5 rad/s), the time simulation switches to an exact delay buffer (a declared discrete state, proposed then as an interface change), and linearization keeps the lag.
- *Stabilator incidence:* the steady-state part of GenHel's stabilator law (Fig. 5.5.1.7: the airspeed and collective branches, with the servo at unit gain and the −8°…39° travel limit), plus a fixed-incidence override from config (GenHel's STBSET). The airspeed lag, servo, rate limit and the pitch-rate and lateral-acceleration feedback come with the AFCS (SPEC_REVIEW #25). In trim (q = 0, a_y = 0, steady airspeed) this equals the full law.

**Landing gear (GenHel §5.7, three gears).** GenHel's structure: per gear, a tire (vertical load–deflection table, in-plane stiffness) in series with a strut (polytropic air spring with preload, velocity-squared damper with two regimes, unsprung-mass DOF). Changes for purity (D-004):
- *States per gear:* strut stroke and stroke rate (GenHel's unsprung-mass DOF) and the two in-plane tire deflections, which replace GenHel's retained contact point.
- *Friction:* GenHel's discrete stick/slip switch becomes a continuous elasto-plastic law: the tire deflection follows the contact-point ground velocity while the tire force is below μN and saturates at μN. The specific law (e.g. Dahl's) needs a source added to the reference library; decided in M6c.
- *No velocity resets.* The strut top stop and the preload mode are decided in M6c once the §5.7.2 equations (pp. 247–265) are read; any stop force needs a source or is logged as a gap.
- Stiffness check against RK4 (D-004 note): the main-tire vertical stiffness near zero deflection is about 38 000 lbf/in (Table 7.5.3) on a 2.67 slug unsprung mass, ω ≈ 410 rad/s, so h·ω ≈ 0.41 at 1 kHz: stable, and accurate enough to be checked in M6c.

**Acceptance (replaces spec M6; same layering as D-007, D-026, D-027):**
1. **Exact-equation references (gate).** Independent test-side transcriptions of GenHel §5.4.2, §5.2.2 and §5.3.2 in GenHel's own notation and sign conventions (inches, buttlines positive to port, wind-tunnel angles), sharing only the tables with production. Production loads and state derivatives must agree with them under random velocities, rates, controls, wind and rotor outputs. These replace "forces match GenHel table values at the test points".
2. **Closed-form and independent anchors (gate):**
   - tail rotor: the Bailey thrust against the production blade-element rotor (M3/M4 code) configured as the tail rotor (linear airfoil, C_d = 0, B, flapping free), with the tolerance from a residual study (the closed form drops O(μ³)); the hover inflow time constant (D-027 form); the δ₃ thrust reduction;
   - airframe: the symmetries the tables imply (corrected in D-030: GenHel's tables don't have all the parities first expected), and the lag-against-delay phase at low frequency;
   - gear: the static load split, which is statically determinate for three contact points (closed form from the CG and gear positions), summing to weight minus rotor thrust; energy conservation with damping and friction off; no creep on a slope below atan μ with brakes on, and sliding at μ_slide·N above it.
3. **Whole aircraft (M6d):**
   - *spec "sits stably on the ground":* dropped from a small height, it settles with the gear-load sum equal to weight (rotor thrust subtracted), and stays in place;
   - *spec "pedal balances torque", made into a sign and arm check:* in hover, the pedal found by a yaw-moment root search in the test gives tail-rotor thrust × arm (about the CG, with cant and the BL convention) equal to main-rotor torque minus the other yaw moments, to round-off. The hover pedal position itself is a validation entry (needs the §5.5 control rigging; there is no published value in the library yet).
4. **Data (human review).** Every transcribed table and constant is reviewed against the scan before use (DATA_GAPS.md).
- **Real-time rules:** zero allocation, purity and bit-identical runs, as for M3–M5.

**Rationale:** each replacement keeps GenHel's steady-state behaviour exactly and gives the dynamics a published or derivable form, so the models stay pure functions of (t, x, inputs). The staging keeps each PR reviewable and lets the data review run alongside the code.

**Implementation notes (2026-10-12, M6a; VALIDATION.md):**
- *Coupling fields of a rotor:* λ₀, μ_x (signed, along x_h), μ_z, β_1c and ΩR. GenHel's wake skew and flapping are computed by the consumer from these: χ = atan(μ_x/|λ|) + a₁ (p. 5.1-37, PDF p. 57), with a₁ = −(2/b) Σ β cos ψ (p. 5.1-18, PDF p. 38), i.e. a₁ = −β_1c if GenHel's azimuth is zero aft, as Eyas's is (to confirm, DATA_GAPS.md).
- *Hub frame:* GenHel's tail-rotor shaft axes (p. 5.4-6) are the D-024 hub frame with roll = Γ_TR = 70°, so the tail rotor reuses `hub_rotation()`.
- *Bailey coefficients:* GenHel's t₃₁, t₃₂, t₃₃ are the exact small-angle integrals for lift carried to B·R, with reverse flow kept on the inflow term only (tested by quadrature). They equal Johnson (2013) Eq. 6.219 at B = 1. Against an exact blade-element integral, GenHel's thrust is about 1% low in hover and up to 9% high at the tail rotor's μ ≈ 0.37 at 150 kt. That is GenHel's model, reproduced; the blade-element tail rotor (spec §6.6, "later") would remove it.
- *Previous-pass scheme measured:* in a pedal step, GenHel's scheme oscillates from pass to pass (±12% thrust) with the same sequence per pass whatever the frame length, so at 60 Hz it lasts more than 0.1 s. The inflow state settles to the same thrust smoothly. This is the error the purity rule's replacement removes, measured as required above.
- *Drag:* production uses −½ρS v|v|, so the drag reverses in rearward flight; GenHel's −½ρS v² doesn't. The UH-60A value is S = 0, so it has no effect.

**Data rulings (2026-10-13, M6b; DATA_GAPS.md "UH-60A airframe tables" and M6 items 1–7).** Two source conflicts were settled from the source itself (the map arguments of Tables 3.5.3–3.5.4 and the spacing of Table 3.5.10, both from the printed figures). Three are assumptions that fill a gap. The reviewer adopted them, on the condition that each is backed by a check wherever one is possible:
- *Fin downwash sign:* Eyas uses V_ZMRV1 = −DWSHMR·Ω·R·EKZV1, as at the stabilator and tail rotor; the fin page prints no sign (p. 5.3-9). Checks:
  - *gate:* for the same map value, the main-rotor wash velocity at the stabilator, fin and tail rotor points the same way in body axes, and downward (+z_b) when the rotor thrust is upward;
  - *gate:* the test-side GenHel reference applies the same sign, as a named and commented departure from the printed equation, so that it is visible rather than hidden;
  - *sensitivity:* VALIDATION.md reports the change in fin side force and yaw moment between the two signs across 20–150 kt level flight, which shows how much the choice can matter;
  - *validation (M7):* directional trim (pedal, sideslip) against published UH-60A data, when available.
- *DLQD, DMQD, DNQD = 0:* undefined in the source. Zero can't be tested directly, since there is nothing to compare it with. It is covered by the M7 trim comparison: a pitch-attitude or pitching-moment residual against published trim data would reopen it.
- *AA1FMR = A1FMR, with no lag:* AA1FMR is undefined (DATA_GAPS.md M6 item 7). Check (sensitivity, M6b): pass a₁ through a first-order lag of time constant τ before the wash maps and report, in VALIDATION.md, the gain and phase change of the stabilator angle of attack and lift over 0.5–10 rad/s against τ. The result is the largest τ that stays within 5% gain and 5° phase in the piloted band (1–5 rad/s). The sweep is a sensitivity bound, not a model parameter. If a definition turns up (T_FILMR on p. 5.1-49 is the candidate), the bound shows at once whether it matters.
- *Fuselage data configuration ("TAIL OFF IRS OFF"):* accepted as is. The fuselage drag excludes the infrared suppressors; M7 and validation comparisons against flight data with suppressors fitted must note it.

## D-029 AFCS placement (SPEC_REVIEW #25) — `Approved (2026-10-12)` — scope
**Decision:**
- **Before the M7 trim comparison:** the steady-state parts of the UH-60A stabilator law and pitch bias actuator (GenHel §5.5), i.e. their outputs with the filters and servos settled and the rate feedbacks at zero. The stabilator part is already in M6b (D-028).
- **Before the M8 piloted evaluation:** the dynamic AFCS (SAS, FPS attitude hold, pitch bias actuator dynamics, stabilator servo, rate limit and feedbacks), as its own milestone or as part of M8; to be scheduled when M7 is done.
- The AFCS components are `DynamicComponent`s whose states are the filter and servo states, so the lateral-acceleration feedback loop closes through the servo dynamics, as in GenHel, and not through an algebraic loop.

**Rationale:** the pitch bias actuator and stabilator change trimmed longitudinal cyclic and pitch attitude, so a trim comparison against flight test without them compares a different aircraft. Pilots know the UH-60A with its AFCS on, so M8 pilot comments need it.

## D-030 Wind axes for the airframe aerodynamics in rearward flight (SPEC_REVIEW #26) — `Approved (2026-10-13)`
**Decision:** the fuselage, stabilator and fin look their tables up on GenHel's angles exactly (α = atan(V_Z/|V_X|), β = atan(V_Y/√(V_X² + V_Z²)), ψ_WF = −β_WF), but build the wind-to-body rotation from the flow's actual direction, α' = atan2(V_Z, V_X), with the same β. For V_X > 0, α' = α and this is GenHel exactly. For V_X < 0 the drag axis is the flow direction, so drag opposes the flow, and the lift and side-force axes stay perpendicular to it. GenHel's untransmitted low-speed filter (p. 5.2-10) is not used (D-028).

**Verification (`tests/unit/test_airframe_aero.cpp`):**
- *Layer 1:* production matches the GenHel transcription for V_X > 0 at every panel (`MatchesGenHelSteadyState`, `MatchesGenHelWithDelayedWake`).
- *Energy:* with only the drag tables nonzero, the force on the fuselage and each panel opposes the local flow (F·V ≤ 0) and is parallel to it, over 500 random directions, more than 200 of them rearward (`DragOpposesTheFlow`). The same test shows GenHel's own axes fail this in rearward flight.
- *Continuity:* the loads are continuous as V_X changes sign (`ContinuousThroughZeroForwardSpeed`). Both α (through ±90°) and α' are continuous there.

**What it doesn't fix:** the tables come from a tunnel sweep of ±90°, so the coefficients in rearward flight are still GenHel's (for α within ±90°, mirrored by the |V_X|); no data exists for flow from behind. The fuselage wake terms (−V_xb ε K_Q, −V_xb σ K_Q) are kept as GenHel writes them; they change sign in rearward flight, where the tail is upstream of the fuselage and the wake angles have no physical meaning. The lag rates use |V_xb|, as D-028 says. Both are small at the low rearward speeds the aircraft flies.

**Also found while transcribing (M6b):**
- *Symmetry acceptance (D-028 layer 2) corrected.* D-028 expected "drag, lift and pitch even, side force, roll and yaw odd" in sideslip. The data says otherwise: GenHel makes the pitching-moment increment odd (ψ/|ψ|·f(|ψ|), p. 5.2-7), Table 2.5.7 (lift increment) is not even, and Table 2.5.11 (yaw) is odd only inside ±30°. The test checks the parities GenHel imposes on each table, and mirror symmetry of the transformation with symmetric stand-ins for the asymmetric tables (`FuselageMirrorSymmetry`, `KnownTableAsymmetries`).
- *Single maps outside their range* (Tables 2.5.7, 2.5.10, 3.5.3, 3.5.7 cover ±30° only) hold their end values. GenHel doesn't say what its map routine does; holding is the only choice that adds no data, and the maps' ends are flat or at their physical limit (the dynamic-pressure ratios reach 1.0 at ±30°). DATA_GAPS.md records it.

