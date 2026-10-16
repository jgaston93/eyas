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
- **Target hardware (spec §12 Q4, answered by the spec owner on 2026-10-10):** a Linux server running RHEL 8 or RHEL 9 (since D-037, Eyas and its host run in a RHEL 9 container on it). This is a general-purpose Linux server, not an embedded RTOS. The budget is measured there, with the release build configuration.

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
  - *Result (2026-10-13, VALIDATION.md M6b):* in the piloted band the lag changes the stabilator lift response by at most 1.1 % in gain and 0.6° in phase (at 20 kt), at most 0.1 % and 0.25° from 70 kt up, although the downwash itself departs strongly from the delay at low speed: the fuselage downwash map is shallow, so the delayed term is a small part of the stabilator's angle of attack. **The lag stays; no delay buffer.**
- *Stabilator incidence:* the steady-state part of GenHel's stabilator law (Fig. 5.5.1.7: the airspeed and collective branches, with the servo at unit gain and the −8°…39° travel limit), plus a fixed-incidence override from config (GenHel's STBSET). *Implemented (2026-10-13):* as a table in airspeed and collective whose cells are each bilinear in the law, so the table is exact (tested against the law); the collective input is taken as percent of travel, an assumption (DATA_GAPS.md). The airspeed lag, servo, rate limit and the pitch-rate and lateral-acceleration feedback come with the AFCS (SPEC_REVIEW #25). In trim (q = 0, a_y = 0, steady airspeed) this equals the full law.

**Landing gear (GenHel §5.7, three gears).** GenHel's structure: per gear, a tire (vertical load–deflection table, in-plane stiffness) in series with a strut (polytropic air spring with preload, velocity-squared damper with two regimes, unsprung-mass DOF). Changes for purity (D-004):
- *States per gear:* strut stroke and stroke rate (GenHel's unsprung-mass DOF) and the two in-plane tire deflections, which replace GenHel's retained contact point.
- *Friction:* GenHel's discrete stick/slip switch becomes a continuous elasto-plastic law: the tire deflection follows the contact-point ground velocity while the tire force is below μN and saturates at μN. The specific law (e.g. Dahl's) needs a source added to the reference library; decided in M6c.
- *No velocity resets.* The strut top stop and the preload mode are decided in M6c once the §5.7.2 equations (pp. 247–265) are read; any stop force needs a source or is logged as a gap.
- Stiffness check against RK4 (D-004 note): the main-tire vertical stiffness near zero deflection is about 38 000 lbf/in (Table 7.5.3) on a 2.67 slug unsprung mass, ω ≈ 410 rad/s, so h·ω ≈ 0.41 at 1 kHz: stable, and accurate enough to be checked in M6c.
  - *Corrected (2026-10-14, M6c):* Table 7.5.3's second row is 1 in, not 0.1 in (DATA_GAPS.md), so the stiffness near zero deflection is about 4300 lbf/in at 140 psi: with the strut, ω ≈ 152 rad/s and h·ω ≈ 0.15. About 36 000 lbf/in is reached only near bottoming (7–8 in), where h·ω ≈ 0.40. D-032 covers the gear's other stiff terms.

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
- *AA1FMR = A1FMR, with no lag:* AA1FMR is undefined (DATA_GAPS.md M6 item 7). Check (sensitivity, M6b): pass a₁ through a first-order lag of time constant τ before the wash maps and report, in VALIDATION.md, the gain and phase change of the stabilator angle of attack and lift over 0.5–10 rad/s against τ. The result is the largest τ that stays within 5% gain and 5° phase in the piloted band (1–5 rad/s). The sweep is a sensitivity bound, not a model parameter. If a definition turns up (T_FILMR on p. 5.1-49 is the candidate), the bound shows at once whether it matters. *Result (2026-10-13, VALIDATION.md M6b):* τ ≤ 17.5 ms; the stabilator's sensitivity to a₁ is 0.6–1.3 times its sensitivity to angle of attack from 20 to 60 kt and below 0.17 from 80 kt, so an undefined filter would matter at low speed only. Item 7 stays open.
- *Fuselage data configuration ("TAIL OFF IRS OFF"):* accepted as is. The fuselage drag excludes the infrared suppressors; M7 and validation comparisons against flight data with suppressors fitted must note it.

**Code review (2026-10-14, M6b):**
- *The tail rotor reads the fuselage wake from the airframe.* GenHel's tail rotor uses the fin's and stabilator's own maps (QV1MP, SGV1MP, EPH1MP; p. 5.4-5), so the airframe publishes K_Q at the fin, σ and ε (undelayed) through the coupling store (D-003) and the tail rotor keeps only its two lag states. Before, the tail rotor loaded its own copies of the three CSVs, which could drift from the fin's. Its `fuselage_wake` config is now just the airframe's name. `AirframePublishesGenHelsTailRotorWake` checks the published values against GenHel.
- *The main-rotor wash at the tail rotor stays its own.* GenHel gives the tail rotor its own wash maps (EXH1MP, EZH1MP on pp. 5.4-4, 5.4-5) and notes they are "same as horizontal tail for Black Hawk": equal for the UH-60A, not in general. The two lookups cost two small table interpolations per evaluation.
- *One CSV reader for configs:* `Config::csv_table(key)` reads the file a key names and records a placeholder table (D-025); the rotor, wash-map and airframe readers use it.

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
- *Continuity:* the loads are continuous as V_X changes sign while V_Z ≠ 0 (`ContinuousThroughZeroForwardSpeed`). Both α (through ±90°) and α' are continuous there. Pure sideward flow (V_X = V_Z = 0, β = ±90°) is a singular line of the wind axes, in GenHel and here: the loads there depend on the direction from which the flow approaches it (SPEC_REVIEW #27), until D-031 fades the wind axes there.

**What it doesn't fix:** the tables come from a tunnel sweep of ±90°, so the coefficients in rearward flight are still GenHel's (for α within ±90°, mirrored by the |V_X|); no data exists for flow from behind. The fuselage wake terms (−V_xb ε K_Q, −V_xb σ K_Q) are kept as GenHel writes them; they change sign in rearward flight, where the tail is upstream of the fuselage and the wake angles have no physical meaning. The lag rates use |V_xb|, as D-028 says. Both are small at the low rearward speeds the aircraft flies.
- *Rearward, the lift and the rolling and yawing moments reverse too (code review, 2026-10-13).* The tables see the same angles at −V_X as at +V_X, and α' = π − α. In level flow (V_Z = 0) the rotation becomes diag(−1, 1, −1): the drag and the lift both reverse in body axes, as do the rolling and yawing moments, while the side force and the pitching moment don't (`RearwardFuselageLoadsInLevelFlow` pins this). The decision argued only for the drag. No data exists for flow from behind, so no choice is right. For comparison, a mirror image of the forward loads (the fuselage treated as the same front to back, the one assumption that adds no data) would reverse the x force, the pitch and the yaw, and keep the lift, the side force and the roll. That rule jumps at V_X = 0 unless the loads at α = ±90° happen to have no x force, pitch or yaw, so it was not adopted. GenHel keeps the lift, side force and roll and reverses nothing. The dynamic pressures are small at the rearward speeds flown, so this mostly affects realism, not stability; M8 pilot comments on rearward flight would reopen it.

**Also found while transcribing (M6b):**
- *Symmetry acceptance (D-028 layer 2) corrected.* D-028 expected "drag, lift and pitch even, side force, roll and yaw odd" in sideslip. The data says otherwise: GenHel makes the pitching-moment increment odd (ψ/|ψ|·f(|ψ|), p. 5.2-7), Table 2.5.7 (lift increment) is not even, and Table 2.5.11 (yaw) is odd only inside ±30°. The test checks the parities GenHel imposes on each table, and mirror symmetry of the transformation with symmetric stand-ins for the asymmetric tables (`FuselageMirrorSymmetry`, `KnownTableAsymmetries`).
- *Single maps outside their range* (Tables 2.5.7, 2.5.10, 3.5.3, 3.5.7 cover ±30° only) hold their end values. GenHel doesn't say what its map routine does; holding is the only choice that adds no data, and the maps' ends are flat or at their physical limit (the dynamic-pressure ratios reach 1.0 at ±30°). DATA_GAPS.md records it.

## D-031 Sideward fade of the airframe's wind axes (SPEC_REVIEW #27) — `Approved (2026-10-14)`
**Decision:** within c of the sideward line (|β| > 90° − c), the fuselage, stabilator and fin fade from D-030's wind axes into fixed ones. With s = smoothstep(cos β / sin c) (smoothstep(t) = t²(3 − 2t), s = 1 for t ≥ 1):
- every lookup angle derived from α is s·α: the fuselage tables, the published α_WF (and so the tail rotor's wake), the stabilator's K_Q and downwash, and its angle of attack;
- of each wind-axis force or moment w, the x component stays along the flow, R(α', β)·(w_x, 0, 0), and the rest is s·R(α', β)·(0, w_y, w_z) + (1 − s)·R(0, β)·(0, w_y, w_z).

On the line the loads then depend only on β and q, so they are continuous, and drag still acts along the flow, so it still only takes energy out. The fixed axes R(0, β) are the ones GenHel's own atan2(0, 0) = 0 picks on the line. c = 20° by default; `sideward_fade` (an angle in [0°, 90°); 0 turns the fade off) overrides it. β itself (the fin and the ψ_WF tables) is continuous and isn't faded.

**Anchor:** GenHel's low-speed filter (CR-166309 p. 5.2-10, "not being executed on PDP10") has the same structure: below |V_XWF| = 25 ft/s it fades the wind-axis loads, in proportion to |V_XWF|/25, into body-axis loads X_LS…N_LS·q. It can't be used as printed: X_LS…N_LS aren't given, its sign factors (α_WF/|α_WF|, V_YWF/|V_YWF|) jump themselves, and a fade on |V_X| would also change vertical descent, where D-030's axes are already continuous. The fade here is on direction, so it doesn't depend on speed, and is C¹, for the M7 Jacobians.

**Choosing c (VALIDATION.md M6b):** the criterion, set before the sweep, was that within the fade the loads may change with the flow direction at most about twice as steeply as they do away from the line (20°–45° from it, UH-60A tables, no wash). The ratio falls roughly as 1/c: 15–20× at c = 2°, 3.2–3.9× at 10°, 2.2–2.6× at 15°, 1.7–1.9× at 20° (force–moment). So c = 20°. The cost, as the largest change of each moment at 20 kt: 15° from the line, 0.45, 0.93 and 0.94 kN·m in roll, pitch and yaw, where a 10° fade changes nothing; 8° from it, 1.5, 3.1 and 2.4 kN·m (10° fade: 0.3, 0.7, 0.8). Closer to the line any continuous choice changes the loads by a large part of their unfaded swing (3–4.4 kN·m), since they swing across it there; this one by 55–80 % of it at 1°. For comparison, GenHel's filter at 20 kt sideward would act up to about 48° from the line.

**The value on the line:** level flow from slightly ahead (α = 0, ψ_WF = ∓90°), the condition the sideslip tables describe directly. The unfaded model instead combines α = ±90° with ψ_WF = ∓90°, two extremes the tunnel never measured together; Padfield warns that such one-angle-at-a-time data lack the cross effects. It is not the average around the line (at 20 kt the yawing moment is 3.19 kN·m against an average of 2.25).

**Where it acts in flight:** the main-rotor wash keeps the flow away from the line. With GenHel's wash maps at hover thrust the fuselage stays outside the fade beyond 45 kt sideward and the tail enters it at about 35 kt; at half thrust, about 35 and 20 kt; with the rotor stopped, always (VALIDATION.md M6b). Forward flight, trim and normal hover are unchanged. Near the line GenHel's panel loads aren't credible either way (full dynamic pressure including spanwise flow, no sideward wake skew), so the fade buys continuity there, not accuracy.

**Literature:** Padfield 3rd ed. §3.4 (pp. 123–124, Eqs. 3.241–3.251; Appendix 4B, p. 233) supports the approach, though it doesn't treat this singularity:
- Helisim tabulates body-axis coefficients over ±180°, so it has no wind-to-body rotation to break, though its α = tan⁻¹(w/u) is still undefined in pure sideways flow;
- "The small angle approximations should be faired into the large angle piecewise forms": blending between data regimes is accepted practice;
- the forward and rearward yaw tables of Eq. 3.251 (Table 4B.4) differ elsewhere but agree at β = ±90°, so that pure sideward flow has one answer.

Johnson (2013) has nothing specific. No source gives the fade width; c = 20° is ours, from the criterion above, and no flight data in the library tests sideward fuselage loads.

**Verification (`tests/unit/test_airframe_aero.cpp`):**
- *Exact equation:* `SidewardFadeMatchesReference`: 300 random directions within 24° of the line, both ways, 10–80 ft/s. The reference takes w from GenHel's own equations at the same speed and β with angle of attack s·α (V_X > 0, where D-030 is GenHel), undoes GenHel's rotation and applies the fade as written above: fuselage forces and moments to 1e-9 relative. Outside the fade the loads are bit-identical to the unfaded model.
- *Continuity:* `ContinuousThroughPureSidewardFlow`: with the x–z component at 1e-9 of the speed, the total loads, α_WF, the stabilator's angle and dynamic pressure don't depend on its direction (16 directions, 1e-6 relative); the unfaded model spreads by more than 100 N·m there.
- *Unchanged:* `DragOpposesTheFlow` (now with directions inside the fade), the Layer-1 GenHel matches (|β| < 55°, outside the fade), `ReadsConfig` and the range check in `RejectsInvalidPanels`.
- 7 planted bugs were each caught: lookups unfaded; a linear fade; the drag faded with the other components; R(α', β) instead of R(0, β) in the fixed term; the weights swapped; cos c instead of sin c; the moments left unfaded.


## D-032 M6c landing gear: replacements for GenHel's hidden state (refines D-028) — `Approved (2026-10-14)` — interface change (public API: brake inputs)
**Source:** GenHel CR-166309 §5.7.2 equations (pp. 5.7-9 to 5.7-26, PDF pp. 247–264, handwritten) and §5.7.5 data (pp. 5.7-40 to 5.7-47, PDF pp. 278–285; DATA_GAPS.md "UH-60A landing gear (M6c)"). GenHel's tire and strut (Figure 7.1.1, p. 5.7-5): a tire with a spring and a damper vertically and in both ground directions, in series with an unsprung mass on an oleo strut (an air spring with a preload, and a damper in ṡ²), the strut sliding parallel to body z. The checks below were run on reduced models before implementation (reviewed 2026-10-14); the gate tests repeat them on the production code.

**States per gear (5, 15 in all):** stroke s, stroke rate ṡ, in-plane tire deflections δx, δy (ground-plane axes aligned with the heading), and the lagged vertical deflection rate (D3 below).

1. **In-plane tire: a slipping spring–damper (replaces the retained contact point and its slip reset, pp. 5.7-16 to 5.7-22).** GenHel's tire element is a spring K with a damper G·K (G = 0.003 s). With L the friction limit and v_c the ground speed of the contact point:
   - stick, |K(δ + G·v_c)| ≤ L: δ' = v_c, F = −K(δ + G·v_c);
   - slip: F = −sgn·L, δ' = (sgn·L/K − δ)/G.
   The branches agree on the boundary, so δ' is continuous. This is GenHel's reset written in continuous time, and it adds no constant. At lift-off L → 0 and δ relaxes to 0 in a few G. *Rejected:* letting δ track the limit μN/K; for the main tire's lateral stiffness (Table 7.5.2, linear in deflection near zero) μN/K stays near 0.2 in as the load vanishes, so a stale deflection would return at the next touchdown. *Checked (1-DOF, 200 slug per tire, K at 1.5 in deflection):* after a load ramp to zero while sliding, δ = 2·10⁻¹⁰ in 50 ms later; h/G = 0.33 at 1 kHz; a braked slide from 4 ft/s sticks once, with no re-slip and no speed-up.
2. **Breakout and sliding friction (replaces the 0.85 reset).** Breakout μN (μ = 0.6; fore–aft × 1.0 brakes set, × 0.01 brakes off, the tail wheel always 0.01; pp. 5.7-18, 5.7-19). GenHel's slide resets the force to K_SLIP = 0.85 of breakout and lets it rebuild each frame, so its mean sliding force depends on the frame rate: 0.85 at 30 and 60 Hz but 0.91–0.92 at 1 kHz (0.3–3 ft/s). Eyas blends L from breakout at v_c = 0 to 0.85 × breakout at |v_c| ≥ v_s with a smoothstep; v_s = 0.1 ft/s. At rest the threshold is GenHel's (the slope test's atan μ), and sliding is GenHel's at its own frame rates. A hard switch would make the derivative discontinuous. *Checked:* v_s has no source; across 0.01–1 ft/s the braked-slide stopping distance changes by 3% (0.446–0.461 ft) and no case chatters. The full-gear test repeats the sweep (VALIDATION.md).
3. **Vertical tire damping keeps GenHel's 0.01 s lag (p. 5.7-17), as a state.** GenHel's damping force is G·K′·(deflection rate), K′ from Tables 7.5.4 and 7.5.6, with the rate a frame difference passed through 1/(1 + 0.01 s). Eyas lags the exact rate (from the body velocity, rates and ṡ). *Checked:* the lag cuts wheel-hop damping by a factor of four (main: ζ 0.177 → 0.046 at 152 → 165 rad/s; tail: 0.248 → 0.043 at 191 → 211 rad/s), so G was presumably set with it; D-028's purity rule asks to match.
4. **In-plane damping without the lag.** A lagged rate would make item 1's slip law implicit. At the in-plane rocking mode (≈ 13.5 rad/s) the lag gives 7.7° of phase and 1.8% less damping.
5. **Isothermal strut as printed** (pp. 5.7-23, 5.7-24): F = sgn(ṡ)(C ṡ² + F_DRG0) + C_PV/(C_PV/F_PRE − s), the denominator limited to ≥ 0.01 ft. γ = 1.34 is listed with the data but used by no equation; the notation (p. 5.7-38) says isothermal, and C_PV in ft·lbf is a pV product. With γ the aircraft would sit 1.3–2.2 in lower (about 0.15° of pitch). Revisit against static ground attitude data if any appears. The damper's two regimes join exactly at 1.4167 ft/s.
6. **Preload and top stop (replaces the locked mode and its reset, p. 5.7-23).** For s < 0, F = F_PRE + k_stop·s + c_stop·ṡ, continuous with the air spring at s = 0; k_stop = 10 × the tire's slope at zero deflection (Tables 7.5.3, 7.5.5), c_stop = 2√(k_stop m). No source gives a stop stiffness: `!placeholder`, DATA_GAPS.md. *Checked:* hanging overshoot 0.071 in (main), 0.068 in (tail); wheel-hop h·ω under preload 0.46 and 0.50 at 1 kHz, against 0.40 for the bottomed main tire.
7. **The unsprung mass feels the attachment point's acceleration and its own weight.** GenHel writes s̈ = −(Z_FTA + F_STR)/m (p. 5.7-23), omitting both (about 86 lbf of weight per main gear). Eyas includes them (D-002), so the gear has nonzero G and H, and the airframe mass excludes the unsprung masses (M6d config).
8. **No vertical gear-load filter.** GenHel filters each vertical gear load by 1/(1 + T_FGR s) (p. 5.7-25), but T_FGR is given nowhere (notation pp. 5.7-28 to 5.7-39, data pp. 5.7-40 to 5.7-47, input list p. 5.7-27; search of the typed pages). T_FGR = 0 is GenHel's own equation at unity. A lag on a spring force feeds energy in: for airframe heave on the struts (≈ 7 rad/s), each 10 ms of lag removes about 0.035 of damping ratio, and 50 ms makes the mode unstable with 0.1 of strut damping. Choosing a value would invent data with a large effect. If T_FGR turns up, it comes back as one state per gear, like item 3. DATA_GAPS.md.
9. **Ground.** Internally a plane (height, normal); GenHel's is level (p. 5.7-15). The API stays flat: `Environment::ground_height` (NaN: no ground) gives z_n = −(ground_height − origin_altitude) with the normal up (passed to components as `SimContext::ground_z_n`, D-033). The slope gate sets the normal at component level. A host terrain normal is an additive API change, deferred to M8.
10. **Kept as GenHel:** separate fore–aft and lateral friction limits (a box, not a circle); a non-castering tail wheel; strut travel parallel to body z, no drag-link arc; tire forces from body-axis deflections, checked for friction in heading-aligned ground axes; the main-tire fore–aft stiffness formula with the exact cube root (Table 7.5.1 equals p^(1/3) to 5 decimals at all 24 points).

**Acceptance (refines D-028 layer 1 and 2 for the gear):**
- *Layer 1:* `tests/support/genhel_gear.hpp`, GenHel §5.7.2 in its own units and notation. For the same deflections and stroke, production gives the same tire forces, strut forces and CG loads, with item 7's terms set aside by name.
- *Layer 2:* the static load split (three points, closed form, summing to weight minus an applied vertical force); energy conserved through a drop with damping and friction off, within a stated RK4 bound; no creep below atan μ with brakes on over a long hold and sliding at 0.85μN above it; preload pass-through; in-plane deflections cleared within 5G of lift-off; wheel-hop at 1 kHz against a fine-step solution; purity (bit-identical, snapshot/restore mid-slide, zero allocation).
- *Validation (VALIDATION.md M6c):* drop, slope hold and slide, brakes-off roll, the v_s sweep.

**To watch in M6d:** with the tires stuck, the fore–aft rocking on the tires (≈ 13.5 rad/s) has ζ ≈ 0.02 from G alone, and the strut damper (∝ ṡ²) gives almost no damping to small heave motions. Settling may be slow; that is GenHel's data.

**Implementation notes (2026-10-14, M6c; VALIDATION.md):**
- *Off the ground (corrects item 1).* The slip law alone doesn't clear the deflection at lift-off: the main tire's lateral stiffness (Table 7.5.2) and its friction limit both vanish linearly with the load, so their ratio stays near 0.2 in, and off the ground the stick branch would let δ drift with the gear line's ground speed. Out of contact the unloaded tire relaxes through its own damper instead, δ' = −δ/G, which clears it before the next touchdown as GenHel's contact-point reset does. No constant is added.
- *Review fixes (2026-10-14, after a code review of M6c):*
  - *Turning ground axes (corrects item 1).* δ is written in ground axes that turn with the heading, but while the tire sticks it is an earth-fixed vector, as GenHel's retained contact point is (p. 5.7-16). The stick rate is therefore δ' = v_c − Ω_g e_z × δ, with Ω_g the heading rate in the ground plane. The trial force's damper uses the same rate, so the two branches still agree on the boundary. Before this fix, a tire stuck through a turn kept its deflection fixed in heading axes, so its force direction drifted by the heading change (the size stayed capped at μN).
  - *The vertical lag runs only in contact (refines item 3).* Off the ground ż_f decays to zero (−z_f/τ). GenHel does no tire calculation at all while no tire touches (p. 5.7-14), so its frame difference starts at touchdown. Tracking the closing rate before contact put a step of G·K′·ḣ into the vertical force at zero deflection: about 1400 lbf per main tire at 10 ft/s. The rate fed to the lag is now the exact d(ΔZ)/dt, including the turning of C_bg (−ω × C_bg δ + C_bg(Ω_g e_z × δ)), as item 3 states. That term had been left out; it is worth under 1 lbf in practice.
  - *An axis without stiffness relaxes.* An in-plane axis whose stiffness is below 10⁻⁹ of the tire's in-plane total carries no force, so δ on that axis relaxes with δ' = −δ/G, as it does off the ground. This happens with the √F lateral law when the vertical force is clamped at zero during a fast rebound. Sticking let δ follow v_c and return as a stale force when the load came back, and the slip law would divide by the vanishing stiffness. The threshold is a numerical guard, not data.
  - *The vertical table must increase throughout* (checked at init), because the load continues beyond the table with the last segment's slope.
  - Effect on the M6c validation runs: the drop's main-tire peak rises from 14.75 to 14.85 klbf; every other quantity moves by less than 0.6 %.
- *Top stop (refines item 6).* For s < 0 the strut force is the air spring continued below zero plus the stop, F_air(s) + k_stop s + c_stop ṡ, so it is continuous in s; the stop's damping term still switches on at s = 0 (a jump of c_stop ṡ, the usual penalty-contact compromise). The default k_stop and c_stop are D-032's rule; a config can set both.
- *Departures from GenHel at touchdown*, each where the tire is barely loaded:
  - the tire can't pull: the vertical tire force is clamped at zero;
  - GenHel enters its tables with |ΔZ|; Eyas has no vertical load when the body-axis deflection ΔZ ≤ 0 (only when a barely touching tire's tilted in-plane deflection outweighs δ_T);
  - GenHel's friction limit uses |Z_FTD|; Eyas uses the normal load only while it pushes.
- *K′ (Tables 7.5.4, 7.5.6)* is looked up on the spring part of the vertical load, which GenHel's frame order takes from the previous pass; this avoids an algebraic loop.
- *Beyond the vertical tables* (8 in main, 5 in tail) the load continues with the last segment's slope, so a very hard landing doesn't meet a tire with no stiffness. GenHel doesn't say what its map routine does; the main tire is at 94 000 lbf there, far beyond any landing the model is meant for (DATA_GAPS.md).
- *Brake inputs (public API, approved 2026-10-14).* `Controls` gains `brake_left` and `brake_right` in [0, 1] (additive, D-018), one per main wheel, since the UH-60A's toe brakes are differential and steer it on the ground. Each gear names its input (`brake: left | right | none`; GenHel's tail wheel is `none`). A brake setting in config was rejected: the derivative is a function of (t, x, inputs), so a pilot control must arrive through `Controls`. A parking brake is the host holding both at 1.
  - *Partial braking:* k_B = N_BRK + (B_RK − N_BRK)·b. GenHel has only "set" (B_RK = 1.0) and "off" (N_BRK = 0.01, p. 5.7-18), which the ends reproduce exactly; nothing in the library relates pedal force to brake torque, so the linear blend is an assumption (DATA_GAPS.md). It is continuous, for the M7 Jacobians. M8 ground-handling comments would reopen it.
  - Inputs outside [0, 1] are clamped and NaN counts as released, so a bad input can't produce a negative or runaway friction limit.
  - Not modelled: the tail-wheel lock (GenHel's tail wheel is neither castering nor lockable).
- *Static settling is slow, as predicted:* the full UH-60A gear on level ground still moves at about 5 mm/s after 30 s (heave and rocking, the ṡ² damper). The static tests use a test-only body dashpot, which vanishes at rest and so can't move the equilibrium.

## D-033 The ground plane in `SimContext` (refines D-032 item 9) — `Approved (2026-10-14)` — interface change
**Context:** the M6c gear rebuilt the ground plane's NED height from the air data, as `ctx.air.altitude + pos_n.z − env.ground_height`. That is right only while `SimContext::air` holds the atmosphere at exactly origin altitude − pos_n.z. The executive guarantees this, but `air` exists for the atmosphere (D-016), and nothing in the interface ties it to the ground. Any other code that builds a context could silently move the ground: unit tests, trim and linearization in M7, or a future atmosphere that clamps or references its altitude differently. Every gear test had to set `air.altitude` by hand for the ground to come out right. Found in the M6c code review.
**Decision:** add `double ground_z_n` to `SimContext`: the NED z of the ground plane where it crosses the NED origin's vertical, in metres; NaN means no ground. The executive sets it once per derivative evaluation, as origin altitude − `Environment::ground_height`, beside the atmosphere. Components read the ground from it and never from `air`. The change is additive: no existing field changes meaning, and the public API is unchanged.
**Not chosen:** passing the origin altitude instead. Each consumer would redo the same subtraction, and the field would describe the map rather than the ground. A terrain normal from the host stays deferred to M8 (D-032 item 9); the plane's normal is still component config.
**Tests:** `Executive.GroundPlaneIsGivenInNed` (the value with a nonzero origin altitude, and NaN passing through). The landing-gear tests set `ground_z_n` directly. The M6c validation outputs are bit-identical to before.

## D-034 M6d: the UH-60A aircraft config and the whole-aircraft checks (refines D-028) — `Approved (2026-10-14)`
**Context:** M6a–M6c built and checked each UH-60A component alone. M6d assembles them into one aircraft defined by data (D-028 staging) and runs D-028's two whole-aircraft checks. That needed four things that weren't settled: where the airframe's mass and CG come from once the blades and wheels carry their own mass; GenHel's control rigging (§5.5); the meaning of the main rotor's shaft-tilt and swashplate-phase constants; and what to do with two GenHel main-rotor features Eyas doesn't have.

**Decisions (rulings by the reviewer, 2026-10-14):**
1. **Generated config.** `data/uh60a/uh60a.yaml` is written by `tools/build_uh60a_config.py` from the GenHel transcriptions in `data/uh60a/genhel/` (`--check` fails if it is stale). Positions in config stay offsets from the airframe CG; the script converts GenHel's stations. No C++ schema change. *Not chosen:* stations and a CG reference in the config schema, read at init. It is easier to edit by hand, but changes every component's position reader; worth revisiting for the next aircraft.
2. **Mass split (D-002, D-032 item 7).** GenHel's WEIGHT (16 638 lbm) and CG (FS 355.9, WL 248.2; p. 5.1-49) are the whole aircraft's. The airframe is WEIGHT less the four blades (4 × 256.9 lbm, at the hub) and the three unsprung masses (2 × 2.6708 + 1.0 slug, at their axles at free extension): 15 406.37 lbm. Its CG, by mass balance, is at FS 356.93, WL 244.59: 1.03 in aft and 3.61 in below GenHel's. *Checked:* `MassSplitKeepsGenHelsWeightAndCg` puts the composite CG at GenHel's to 1e-5 in, placing it from the config's hub position and GenHel's station differences, independently of the script's arithmetic.
   - *Inertias* are GenHel's motion-module values as written (D-002), now taken about the airframe CG. Moving the reference point by the CG shift changes them by m·d²: 43 slug·ft² in roll (0.9 % of I_x), 47 in pitch (0.1 % of I_y) and 3.5 in yaw. This is small next to the open blade-inertia discrepancy (DATA_GAPS.md, "UH-60A mass properties"), and is revisited with it if the M7 roll response disagrees.
3. **Main-rotor geometry, settled from the source.** i_θ = −3° and i_φ = 0 are the shaft inclination: i_θ about Y_B, then i_φ about X_S (p. 5.1-2). Fig. 1.1.1 (p. 5.1-7) draws a positive i_θ nose-up, so −3° tilts the shaft forward, which is Eyas's hub pitch directly. Δ_SP = −9.7° is the swashplate phase in the blade pitch, θ = θ_cuff − A1S cos(ψ + Δ_SP) − B1S sin(ψ + Δ_SP) (p. 5.1-24). With GenHel's azimuth the same as Eyas's (DATA_GAPS.md item 6), this gives θ1c = −A1S cos Δ_SP − B1S sin Δ_SP and θ1s = A1S sin Δ_SP − B1S cos Δ_SP. θ_cuff is Eyas's θ0, since the twist table is added to the cuff pitch.
4. **Control rigging (Figs. 5.1.6(a)–(d), p. 5.5-20).** This is the mechanical path only: SAS, FPS and PBA at zero, primary servos settled (D-029). In GenHel's inches and degrees:
   - θ0 = 1.60 XC + 9.9;
   - A1S = 1.60 XA − 0.16 THØSUM − 8;
   - B1S = −2.83 XB + 0.29 THØSUM − 0.2936 THRSUM − 9.7754;
   - θ_TR = −5.539 XP + THØSUM + 21.9.

   Here THØSUM = 1.60 XC and THRSUM = −5.539 XP. The pilot's and the trim algorithm's longitudinal inputs both enter XB with a minus sign, XB = −XB+1 − XB+2 + XBOLS (verified by human review, 2026-10-14; DATA_GAPS.md), so forward stick raises B1S and tilts the disc forward. The chain from normalized input to inches, through the mixing and the phase rotation, is linear, so it is written as the offsets and gains of the existing `LinearControlMap` (D-011).
   - *Travels are placeholders.* GenHel Vol. I gives no stick or pedal travel in inches ("see Volume II"). Each travel spans the rotor-head pitch limits (p. 5.5-20) with collective down and the other inputs at zero, its length the head range over the gain: collective 0–10 in, lateral 0–10 in, longitudinal −0.963 to 9.214 in, pedal −2.636 to 3.141 in. The first two come out as round numbers, the others do not. Every control-map value depends on them and is tagged `!placeholder`. The gate tests don't depend on the travels, and the hover controls are reported in GenHel's inches and degrees too.
   - *Head travel limits not applied.* GenHel gives them "for piloted simulation"; they come with the M8 piloted work (`LinearControlMap` has no clamp).
   - The stabilator schedule's collective input, assumed to be percent of travel (DATA_GAPS.md), is Eyas's collective × 100. With the 0–10 in placeholder that is percent of XC.
5. **Deferred to M7, decided on evidence:**
   - GenHel's elastic (dynamic) twist: K_FP0 = −0.0003 − 0.0000044 (V_kt − 100), limited to [−0.00052, −0.0003] (p. 5.1-49, applied on p. 5.1-24). Eyas's blades are rigid. *Rule:* add it in M7 if the trimmed collective or cyclic error against the reference grows with speed; otherwise leave it, and say why. M10 (elastic blades) would model the twist from the blade's structure.
   - GenHel's 0.3 s first-order filter on the rotor forces and moments (T_FILMR, p. 5.1-36). Eyas couples the rotor exactly (spec §5). *Rule:* revisit only if the M7/M8 response comparisons show a phase mismatch that this lag would explain.
   - ΔC_DMR = 0.002, GenHel's profile-drag correction (p. 5.1-49): there is no Eyas config key for an airfoil drag increment, and the SC1095 tables are used as printed. It changes torque and power, not the M6d gates. *Rule:* revisit with the M7 power comparison.
6. **Elements:** 20 equal-area elements on the main rotor; GenHel uses 5 segments (NSS = 5), a numerical choice of its own, not data.

**Acceptance (refines D-028 layer 3):**
- *Config:* the model loads through the public API, with exactly the placeholders above and the strut top stops (D-032 item 6, written out in the config so they are listed).
- *Rigging, layer 1:* `ControlMapsAreGenHelsRigging` checks the production maps against a test-side transcription of Figs. 5.1.6 and the p. 5.1-24 pitch equation in GenHel's units: 200 random inputs, blade pitch at 12 azimuths to 1e-7°, and the tail-rotor command.
- *Hover:* out of ground effect, with the body held level and at rest and cyclic centred, a root search sets collective and pedal for zero vertical force and zero yaw moment. The tail rotor's yaw moment, worked out by hand from its thrust, its 70° cant and GenHel's station differences, must balance the other components' yaw moments, to the search's tolerance. The signs must be the aircraft's.
- *Ground:* dropped level from 0.3 m with collective down, brakes set, and the pedal that balances yaw at that collective. Averaged over 5 s windows, the ground normal loads must equal weight minus aerodynamic lift within 0.2 %, and the aircraft must not creep (5 mm, 1e-4 rad of heading) between the 5–10 s and 15–20 s windows. The settling itself is measured and reported, not tuned (D-032 "to watch").
- *Real time:* the whole aircraft through `Model` must show no allocation in `advance()`, bit-identical repeat runs, and an exact snapshot and restore across touchdown.

**Harness:** the executive keeps no per-component loads, so the tests evaluate the same components from the same config, in a producer-first order, with the body held (`tests/support/m6d_fixtures.hpp`). With the body held, a = 0, so `load0` is the whole load and ẋ_c = ẋ0_c exactly. No production interface changed.

## D-035 M7 trim and linearization: method and layered acceptance (SPEC_REVIEW #1, #28) — `Approved (2026-10-15)` — interface change (internal executive; public API: `trim`)
**Sources:** Padfield 3rd ed. Ch. 4 (trim, §4.2; derivatives by two-sided numerical perturbation of the nonlinear model, §4.3, p. 186; derivative approximations, Table 4.2 and Eqs. 4.81–4.85) and Eqs. 3.47–3.53 (multiblade coordinates); Johnson (2013) Ch. 15 (§15.4 multiblade coordinates and the constant-coefficient approximation, averaged in the non-rotating frame; §15.6 Floquet theory) and Ch. 21 (linearized flight dynamics).

**Staging (as M6, one PR each):**
- **M7a** — the executive's held-body evaluation, the trim solver and `Model::trim`;
- **M7b** — linearization (retained and condensed models);
- **M7c** — the UH-60A trim sweep, eigenvalues and derivatives; the steady-state pitch bias actuator (D-029); the control travels (SPEC_REVIEW #28); the D-034 item 5 rulings; the validation entries.

**Trim (M7a):**
- *Condition:* true airspeed V, flight-path angle γ, sideslip β, altitude, mass as configured, `Environment` (ΔT, wind; ground NaN). Straight flight: ω = 0. Steady turns are "later" in §6.8; a turn rate is an additive field when they come.
- *Unknowns:* the four `Controls` (not blade angles; the blade angles are reported), θ and φ, and the values at ψ = 0 of every non-body state (blades, inflow, tail-rotor inflow, wash lags, gear).
- *Held body.* Over one revolution T = 2π/Ω the body state is held at the trim condition (velocity from V, γ, β, θ, φ; heading 0) and the non-body states are integrated with a = 0, by the production derivative function in a held mode and RK4 with h = T/256 (the production 1 ms step doesn't divide T).
- *Residuals:* (a) the wrench that holds the body (the net force and moment at a = 0, airframe gravity included), averaged over the revolution and scaled by weight and weight × R, = 0; (b) x_c(T) − x_c(0) = 0. This is §6.8's "averaged over a revolution with the blades integrated through azimuth", made exact: the body equations are averaged and the rotor states are periodic, with no quasi-steady flapping.
- *Solver:* Newton on all unknowns together (periodic shooting), the Jacobian by finite differences (one revolution per column, about 35 columns), a minimum-norm least-squares step so neutral states (the wash lags at zero airspeed) keep their starting values, backtracking, at most 30 iterations, a convergence flag (spec §7 rule 3). Start: uniform inflow, unflapped blades, a few revolutions of held relaxation.
- *Why the body is held:* in the free periodic orbit the airframe also vibrates at 4/rev (about 4e-4° in roll, VALIDATION.md M6d). Holding it is the standard averaged trim; the error it leaves is the small 4/rev transient when a free run starts from trim, measured by gate T2.
- *Executive (internal interface change):* a held-body evaluation that returns the non-body ẋ at a = 0, the net wrench and, optionally, each component's load. The M6d harness then wraps it instead of copying the executive (VALIDATION.md M6d).
- *Public API (interface change, `include/eyas/trim.hpp`):* `TrimCondition`, `TrimResult` (controls, θ, φ, blade angles, main-rotor power, residual norms, iterations, `converged`) and `TrimResult Model::trim(const TrimCondition&)`. Init-time (it allocates; not callable from a real-time loop). On convergence the model is reset to t = 0 at the trimmed state, so `advance()` flies on from trim; otherwise the model is unchanged.

**Linearization (M7b):**
- *Internal* (`src/analysis/`), used by tests and the validation generators; not in the public API until a host needs it.
- *Coordinates:* body (u, v, w, p, q, r, φ, θ, ψ_h), perturbed as Euler angles and mapped to the quaternion; position is dropped (it enters only through density, and §6.9 omits it). The main-rotor blade states in multiblade coordinates (Padfield Eqs. 3.47–3.48 with their rates; Johnson §15.4).
- *Averaged model:* central-difference A(ψ_j), B(ψ_j) of the free-body derivative function at J = 64 azimuths of the trim orbit, transformed to multiblade coordinates (A_M = L⁻¹(A L − L̇), Padfield Eqs. 3.52–3.53 in first-order form) and averaged: Johnson's constant-coefficient approximation with the coefficients averaged in the non-rotating frame. This resolves SPEC_REVIEW #1. Perturbation sizes come from a step-size study (results flat over a decade), recorded in VALIDATION.md.
- *Retained model:* the averaged system, all states. *Condensed 6-DOF model:* states with identically zero row and column are dropped (decoupled; the wash lags at hover), then A_bb − A_br A_rr⁻¹ A_rb and the same for B (quasi-static rotor, inflow and lags, as Padfield's 6-DOF derivatives). Outputs: the matrices, the derivatives in Padfield's ordering (§4.3), eigenvalues with frequency, damping and mode labels.

**Acceptance (replaces spec M7; same layering as D-007, D-026–D-028):**
- *T1 Convergence (spec, gate):* the UH-60A trims in level flight at ISA sea level, 16 638 lbm, 0–150 kt in 10 kt steps; also ±1000 ft/min at 60 and 100 kt and ±10° sideslip at 60 kt.
- *T2 Free flight from trim (gate):* from each T1 trim, through `Model::advance` at 1 kHz, the body accelerations averaged over the first revolution are below a bound set from a residual study (reported), and the trim is unchanged within tolerance when h is halved (T/512).
- *L1 Exact entries (gate):* on the full UH-60A at each trim, the gravity and kinematic entries of A (∂u̇/∂θ = −g cos Θ, the Euler-rate rows, the trim-velocity terms of Padfield's Eq. 4.44 structure) equal their closed forms to finite-difference precision.
- *L2 Floquet (gate in hover, reported in forward flight):* the monodromy matrix of the free system over one revolution (from the trim's shooting machinery) gives Floquet exponents (Johnson §15.6). In hover the multiblade system is constant-coefficient (Johnson §15.4, N_b ≥ 3; the airframe and the Bailey tail rotor don't depend on azimuth), so the averaged model's eigenvalues must equal them to finite-difference precision. In forward flight the difference measures the periodicity the averaging drops; it is reported against μ, with a gate only on the body modes up to μ ≈ 0.2 if the measured trend supports one.
- *L3 Linear against nonlinear (gate):* small initial perturbations of each body state and small control doublets, flown through `Model::advance`, against the retained linear model over 3 s (revolution-averaged). The error must shrink with the square of the amplitude (checked by halving it).
- *L4 Closed forms on fixtures (gate; tolerances from residual studies, recorded):* hover heave damping Z_w and the heave eigenvalue against Padfield Table 4.2 (∂C_T/∂μ_z = 2aσλ₀/(16λ₀ + aσ)) on a rotor-on-a-mass fixture with Pitt–Peters inflow; hover M_q and L_p against Padfield Eqs. 4.81 and 4.84 on a centre-spring, uniform-inflow fixture (linear airfoil, C_d = 0, constant chord, B = 1, as in D-026).
- *V1 Trim against GenHel and flight test (validation entry; a gate once the plots are read):* Ballin, NASA TM-88360 (1987), App. B: level-flight trim (16 000 lbf, 5250 ft density altitude) of the Ames and Sikorsky GenHel against USAAEFA flight test, plus climbs and sideslip; App. D has small step responses against flight test. Readings from its plots go through human review. Two separate checks:
  - *Eyas against GenHel:* same data, so every difference must be explained by a recorded departure (D-024, D-026–D-034) or it is an Eyas bug. Once the plots are read, this becomes a gate with a tolerance set from the read-off accuracy and the named departures.
  - *Eyas against flight test:* the realism check, reported. Where GenHel misses flight test, Eyas isn't expected to do better.
- *V2 Derivatives and modes (validation entry; replaced by the M7c amendment below):* hover and 80 kt against the identified UH-60 models of Fletcher, 21st ERF (1995) Paper VII-4. (The fuller NASA TM-110362 carries a limited-distribution notice and is not used.) The identified models carry flap, lag, inflow, rotor-speed and engine states and an empirical flap phase lag, so the comparison names which states are matched. The mode character is also checked against what the literature describes (Padfield 3rd ed. §4.3: an unstable hover phugoid, roll damping set by L_p, a Dutch roll in forward flight).
- *D-034 item 5 rulings:* made on physics and on V1, not on agreement with any one model. Padfield 3rd ed. (p. 143) reports that elastic torsion had a negligible effect on UH-60 trim, stability and response in two studies against test data, which favours leaving GenHel's elastic-twist correction out unless V1 shows a speed-growing control error.
- *No reference-model comparison.* The spec assumed a reference model that could be set up as a UH-60A; none is available (D-027), so the reference model has no part in M7 (approved 2026-10-15: the spec is a guide, and the goal is a model that agrees with the literature and basic physics). SPEC_REVIEW #12 no longer blocks anything.
- *Real-time rules:* `trim` and the linearization are init-time and may allocate; `advance()` after `trim` keeps zero allocation and bit-identical repeat runs (tested).

**Rationale:** the reference-model criteria can't gate (SPEC_REVIEW #28), so, as in M3–M6, the gates test the production code against independent exact relations (periodicity, free flight, Floquet, gravity and kinematics) and against Padfield's closed forms on fixtures, and published UH-60A data (GenHel and flight test) is the realism check. Holding the body and shooting for periodic rotor states is §6.8 made exact. Averaging the multiblade-coordinate Jacobians is Johnson's constant-coefficient approximation, and the Floquet gate measures what it drops.

**Approval (2026-10-15):** approved by the spec owner, with the reference model removed from M7 and published data as the realism check (above).

**Implementation notes (2026-10-15, M7a; VALIDATION.md M7a):**
- *Control travels (SPEC_REVIEW #28):* from GenHel Figs. 6.3.1 and 6.4.1, readings verified by human review. Collective, lateral and pedal fit the rigging exactly (the generator checks it); the pedal travel is its stroke with overtravel, 5.38 in, which the figure's end rows use. Neutral longitudinal cyclic is the mean of the three rows' estimates, which spread by 0.81 in, so only the main rotor's θ1s and θ1c offsets stay `!placeholder`, until M7c pins neutral against Ballin's GenHel trim plots.
- *Executive:* `held_derivatives()` and `derivatives()` share one evaluation; `context()` builds the `SimContext` both use. The config readers moved from `model.cpp` to `sim/build`, so tests and tools build exactly the Model's aircraft. No §6 component interface changed.
- *Period:* the trim finds Ω from the aircraft's blade-element rotors (all must turn at the same speed; without one, the equations don't depend on time and any period works). No interface addition was needed.
- *Vibration start (addition):* the held trim's start state gets the zero-mean 4/rev velocity and rate oscillation implied by the free-body accelerations along the held orbit, so a free run starts on its vibration cycle. Without it the free body starts off the cycle by a constant rate and its damping turns that into a drift 30–100× larger. `TrimResult::vel_b` stays the mean velocity the condition asks for.
- *Not in `TrimResult` yet:* the blade angles and main-rotor power. They are component-specific; the validation tools report them, and they join the API with the Tier 4 outputs (M8) if a host needs them.
- *T2 bounds* come from the drift study: up to 60 kt the solver's tolerance, above it the held-body approximation (VALIDATION.md M7a).

**Amendment (2026-10-15, before M7b; SPEC_REVIEW #29):** approved by the spec owner. It replaces the items below; the rest of D-035 stands.
- *Coordinates:* the retained model adds height h (positive up; density through the ISA, D-015) to the body coordinates. This departs from Padfield's Eq. 4.44, which leaves height out (p. 183 notes the density change in climbing flight). It is needed so that L3's second-order check isn't spoiled by a first-order error (up to 0.24 % over 3 s). The condensed model drops h, keeping Padfield's 9 states. x_n and y_n stay out: they enter nothing while the ground is absent and the wind uniform.
- *Periodic linear model (addition):* the per-azimuth multiblade matrices A_M(ψ_j), B_M(ψ_j) are kept as the periodic (LTP) linear model, alongside their average.
- *State elimination:* a set of states whose columns are zero outside the set feeds no other state. The matrix is then block triangular, so the set can be removed without changing any other eigenvalue. These sets are removed before the condensation, and their own eigenvalues are reported. This replaces "identically zero row and column", which depends on finite-difference noise for the hover wake lags.
- *Derivatives:* reported as Padfield reports them in App. 4B, i.e. the entries of A with the inertial and gravitational terms of Eq. 4.44 included (Padfield p. 187). The moment rows are the primed derivatives L′, N′ (Eqs. 4.47–4.48), which the coupled solve gives directly.
- *L2 Floquet:*
  - *Gate:* a symmetric hover fixture (shaft along gravity, no cyclic, no tail rotor or airframe, Pitt–Peters inflow, N_b = 4, β₀d included). There the rotating-frame equations have constant coefficients, so the multiblade equations do too (Johnson pp. 556, 572). The A_M(ψ_j) must agree at every azimuth, and the averaged eigenvalues must equal the Floquet exponents, both to finite-difference precision.
  - *Reported (UH-60A):* the hover trim's cyclic flapping makes the coefficients periodic even in hover (mostly the 2/rev β₀d coupling; Johnson p. 557), so for the UH-60A, hover and forward flight alike, the variation of A_M around the revolution and the Floquet-to-averaged gap are reported against μ. Also reported: the gap's scaling with the periodic part scaled by s ∈ {1, ½, ¼}, expected second order since the first-order change of a simple Floquet exponent is the zero revolution mean of the periodic part (own perturbation argument, recorded in VALIDATION.md). Johnson (p. 757) asks for exactly this check of the constant-coefficient approximation.
  - *Monodromy:* central differences of the reduced free flow map over one revolution (RK4 at T/256, height free, x_n and y_n held), independent of the sampled Jacobians. Modes with |Λ| < 1e-6 (the gear struts and the 0.01 s lags) aren't compared. The branch of ln Λ follows Johnson's rule (pp. 570–571): continuity from the averaged root, the frequency picked by the eigenvector's largest harmonic.
- *L3 Linear against nonlinear:*
  - *Gate, every T1 speed:* responses to small initial perturbations and control doublets through `Model::advance`, against the **periodic** linear model. The error's order, fitted as the log–log slope over at least three amplitudes, must be 2 within the fit's uncertainty.
  - *Reported:* the averaged model against the same runs, with the error's first-order part against μ (up to 7.9 % at 150 kt in the probe).
  - *Gate, hover only:* the averaged model, with a tolerance set from the measured gap (0.014 % in the probe).
- *Reduced-coordinate check (gate, addition):* the reduced derivative function must equal the time derivative of the reduced coordinates along the free flow, to finite-difference precision. It checks the Euler-rate, height and multiblade terms, and caught a sign error in the planning probe.
- *M7c consequence:* at high μ the averaged eigenvalues of some slow modes are about 10 % from the Floquet exponents (probe: 0.025 rad/s at 150 kt), so V2 reports the Floquet exponents alongside the averaged eigenvalues.

**Amendment (2026-10-15, before M7c; SPEC_REVIEW #31):** approved by the spec owner (items marked as pending stay open until resolved). It replaces V1 and V2 above; the rest of D-035 stands.
- *V1 data:* Ballin (NASA TM-88360) App. B, Figs. B1 (level flight, 0–160 kt) and B2–B4 (climb and descent at 60, 100 and 137.5 kt): the four controls in percent, pitch, roll and sideslip (B1), stabilator angle (B1) and main-rotor power. Digitized from rendered images by a tool (`tools/digitize_ballin.py`), with overlays of the readings on the scan for human review; readings are `UNVERIFIED` until reviewed (DATA_GAPS.md).
- *V1 conditions:* Ballin's 16 000 lbf, CG FS 351.0, WL 231.5, 5250 ft density altitude (ISA with ΔT = 0 at that altitude gives it exactly). The config generator writes validation variants at a given weight and CG with D-034 item 2's mass split (`data/uh60a/uh60a_ballin.yaml`, `uh60a_fletcher.yaml`). Ballin's waterline is 16.7 in below GenHel's (pending: DATA_GAPS.md), so V1 reports the trim's sensitivity to it.
- *V1 lateral constraint (reading verified by human review, 2026-10-15):* Ballin's GenHel trims appear to hold β = 0 below about 50 kt and φ = 0 (β free) from 50 kt up (Fig. B1(e)–(f); SPEC_REVIEW #31). Eyas uses its own trim below the switch and, above it, an internal outer solve on β around `solve_trim` (`src/analysis/`); `TrimCondition` is unchanged. For climbs and descents (B2–B4, 60 kt and up) the same zero-roll rule is assumed; their figures plot neither roll nor sideslip.
- *V1 gates and reports* are as above (Eyas against GenHel a gate once read; Eyas against flight test reported). Main-rotor power is compared as such, so the tail rotor's missing torque (DATA_GAPS.md) doesn't enter. Ballin's flight-test pedal bias (about 0.5 in right, p. 10) and weaker low-speed agreement are noted with the flight-test comparison.
- *Neutral longitudinal cyclic:* the θ1s and θ1c offsets are fitted to Ballin's GenHel longitudinal stick in B1. Check: the fitted neutral must lie within the 0.81 in spread of the three GenHel Fig. 6.3.1 estimates (DATA_GAPS.md). The longitudinal-stick gate then tests the shape against speed only, which the validation entry states.
  - *Result (2026-10-16):* the least-squares fit over 10–150 kt puts neutral at 2.04 in, outside the spread (1.10–1.91 in) by 0.13 in, so the check fails. A low-speed fit (10–40 kt) lands inside at 1.90 in; cruise fits near 2.1 in. **Decision (spec owner):** neutral is the end of the spread nearest the fit, the pinned row's 1.91 in, and the remaining mean difference, 1.3 % of stick travel (Eyas forward of GenHel), is recorded as an Eyas–GenHel difference in the pitch trim rather than absorbed into the rigging. The cyclic offsets lose their placeholder tags.
- *Pitch bias actuator (D-029):* its steady-state law from GenHel §5.5, behind a config switch. Ballin doesn't say whether it was active in the static trims (pending: check GenHel and Ballin), so V1 reports both.
- *Also settled by V1:* the stabilator schedule's collective units (DATA_GAPS.md, against Fig. B1(h)); whether the pedal is smooth through the 30 kt fin-blockage breakpoint (D-036 implementation notes); D-034 item 5's elastic-twist ruling, on the speed trend of the control errors against GenHel, and its profile-drag increment (ΔC_DMR = 0.002), on main-rotor power against GenHel. Ballin (p. 3) describes the Ames GenHel as having no dynamic twist, which conflicts with CR-166309's K_FP0 (pending: resolve before the ruling).
- *V2, redefined (reported):*
  - Lock number and M_β/I_β from the config against Fletcher's theoretical values (Table 1).
  - Frequency responses of Eyas's linear models (on-axis and off-axis angular rates to cyclic, yaw rate to collective) at hover and 80 kt against Fletcher's flight-data curves (Figs. 3–14), below about 12 rad/s, where Fletcher sees the first lag-mode effects (Eyas has no lag, D-026). At Fletcher's 14 350 lbf and CG FS 360.
  - Mode character against Padfield 3rd ed. §4.3, with the Floquet exponents alongside the averaged eigenvalues.
- *Deferred:* steady turns (Fig. B5; needs a turn rate in `TrimCondition`, an interface change) and App. D step responses (needs lag, rotor speed and the dynamic AFCS) to M8 or later. D-034 item 5's rotor-force filter ruling waits for those response comparisons.

## D-036 Flow-angle tables: a parabola through the three nodes around zero (SPEC_REVIEW #30) — `Approved (2026-10-15)` — departure from GenHel
**Context:** SPEC_REVIEW #30. GenHel's linear interpolation puts a corner at zero angle in its even fuselage and empennage maps, where every straight-flight trim sits. That makes the model non-differentiable there and blocks D-035 L3.

**Decision:**
- Every airframe-aerodynamics table against a flow angle (the fuselage maps against α_WF and ψ_WF; each panel's c_l, c_d, dynamic-pressure ratio and wake angle) is read through `AngleTable`.
- Where the table has a breakpoint at zero with symmetric neighbours ±a (all of the UH-60A's, a = 5°), the two segments next to zero use the parabola through the three nodes: f(x) = f(0) + o u + (e − f(0)) u², with u = x/a, e = (f(a) + f(−a))/2 and o = (f(a) − f(−a))/2. Elsewhere, and in tables without such a breakpoint, interpolation stays linear, as in GenHel.
- Properties:
  - it passes through every GenHel node;
  - an odd table is unchanged (e = f(0) = 0);
  - the odd part keeps its slope o/a through zero, and the even part is flat there;
  - the only new slope changes are at the ±a nodes, where linear interpolation already had them.
  `AngleTable::min_value()` includes the parabola's vertex for the dynamic-pressure-ratio check.
- The GenHel reference used by the Layer-1 tests (`tests/support/genhel_airframe.hpp`) applies the same parabola to its own lookups, implemented independently on GenHel's printed maps, so those tests still check the production code against GenHel, with this departure.

**Alternatives considered:**
- *Keep the corners and weaken L3.* Average the ± responses to cancel the even part, and require only convergence. Rejected: the corner isn't physics, and the nonlinear model would keep a |β| coupling.
- *Smooth interpolation everywhere (splines).* Rejected: it changes every value between nodes. The corner only matters where trims sit, i.e. at zero.

**Effect (measured, VALIDATION.md M7b):**
- *Linearization:* at the M7b trims the sideways-velocity and yaw-rate columns are now smooth (their one-sided differences converge as the step shrinks). L3's error shrinks with the amplitude at all speeds.
- *M7a trims:* 0–150 kt, all converged; the largest changes are at 150 kt: collective 0.18 % of travel, pitch attitude 0.08°, roll 0.01°, main-rotor power 7.5 hp (1890 → 1882 hp), tail-rotor thrust 5 lbf.
- *Validation entries:* VALIDATION.md M7a is regenerated with it. The M7c comparison with Ballin's GenHel trims lists D-036 among the explained departures.

**Data note:** the fuselage lift-increment table header said "odd in PSIWF". That applied to the pitching-moment column only; the lift increment is printed for −30 to 30° and isn't symmetric. The header was corrected; no value changed.

**Implementation notes (2026-10-15, M7b; VALIDATION.md M7b):**
- *Coordinates:* `ReducedCoordinates` handles any blade count: β0, β_nc and β_ns, and β_d for even N_b (Johnson 2013 Eq. 15.77), with q̈ = a − 2Ω D q̇ − Ω² D² q, which reduces to Johnson's Eq. 15.80 for the flap equation. Flapping rotors are found by `dynamic_cast<BladeElementRotor>` and blade azimuths come from `blade_azimuth()`, so no §6 interface changed. ψ_h is taken within ±π of the reference heading.
- *Held orbit:* the trim's held RK4 integrator moved to `analysis/held_orbit` and is shared; the linear models are taken about the mean body state (without the vibration offset).
- *Floquet:* exponents are matched to the averaged roots nearest first, one to one, with the branch nearest to the matched root's frequency. That is Johnson's continuity rule (pp. 570–571) while gaps are small against root spacing. The monodromy starts at the held orbit's start.
- *L3 as implemented (option A, approved 2026-10-15):*
  - *Speeds:* hover, 60 and 150 kt in the test (first 0, 35, 60, 90, 120 and 150 kt; cut for CI time with D-037, since the generator covers the rest); every 10 kt in `eyas_m7b_cases`, reported.
  - *Cases:* each body coordinate perturbed and each control given a 1 s sine doublet (continuous, so RK4 stays accurate), at four amplitudes halving from 0.0625 m/s, 0.005 rad/s, 0.0025 rad and 0.0025 of travel. These keep the largest clear of nearby map nodes: χ = −0.85° next to 0° in hover, a₁ = 0.12° next to 0° at 60 kt.
  - *Steps:* both models step at T/256 up to 60 kt and T/512 above.
  - *Criterion:* the relative error must fit c0 + c1 a to within 15 % beyond a noise allowance of half the limit. The leftover c0 must stay below 1e-4 up to 60 kt and 1e-3 above (measured ≤ 3.2e-5 and ≤ 5.4e-4).
  - *Hover, averaged model:* gated at c0 < 2e-3 (measured 1.04e-3, for longitudinal cyclic). The probe's 0.014 % was for state perturbations only.
  - *Why not "order 2 within the fit's uncertainty"* (the amendment's wording): the data are deterministic, so the fit has no statistical uncertainty, and part of the leftover is the held-body linearization. That part was measured (half the high-speed leftover; eigenvalues within 0.07 % to 120 kt and 0.4 % at 150 kt, with the free flight wandering by as much between revolutions) and approved as option A.
- *30 kt:* GenHel's tail-rotor fin blockage has its breakpoint at exactly 30.0 kt (CR-166309 p. 5.4-2, "empirical and based on flight test data of other helicopters"; p. 5.4-12), a corner in airspeed. A 30.0 kt trim sits on it, and L3 there has a 12 % leftover. The corner isn't smoothed, because no source gives the shape (approved 2026-10-15), and 30 kt isn't an L3 gate speed. If M7c shows the aircraft's pedal smooth through 30 kt, that is the evidence to revisit it.
- *L4 as implemented:*
  - *Fixture:* the symmetric coaxial fixture with a centre hinge, no root cutout, B = 1, a linear airfoil without drag, and Pitt–Peters (heave) or uniform inflow (rate derivatives).
  - *Rate derivatives:* quasi-steady with the body held (Padfield's definition: the disc settles before the fuselage moves).
  - *M_q, L_p:* from the condensed model on a stiff hub (S_β = 0.3) over a heavy airframe (10⁶ kg·m²), so blade inertia and the acceleration coupling stay below 0.2 %. Eq. 4.81 drops the in-plane loads, which grow as the hub softens, so softer hubs are reported, not gated.
- *Condensed outputs:* derivatives as Padfield's App. 4B plots them (gravity and kinematic terms included; moments primed), and modes with an eigenvector-based label (`modes()`). The label is a heuristic, indicative where modes couple strongly.
- *Cost:* linearizing the UH-60A takes 25 ms (41 states, 64 azimuths) and its Floquet flow map 0.25 s (Release).

## D-037 Deployment in a RHEL 9 container; RHEL 8 dropped from CI (refines D-005) — `Approved (2026-10-15)`
**Context:** D-005 recorded the target as a Linux server running RHEL 8 or RHEL 9, so CI built and tested in Rocky Linux 8 (gcc-toolset-13) and 9 (system GCC 11). Eyas is an in-process library, so it must be built for the userspace the host simulator runs in: a RHEL 9 build (glibc 2.34) won't load into a host running natively on RHEL 8 (glibc 2.28). The deployment team (the spec owner) decided on 2026-10-15 to containerize the host.

**Decision:**
- Eyas and the host simulator always run in a RHEL 9 container, whether the server runs RHEL 8 or RHEL 9.
- CI keeps the Rocky Linux 9 job (system GCC 11) and drops the Rocky Linux 8 job.
- Configure still rejects GCC < 11 with instructions, which also covers anyone building elsewhere.
- *Fidelity:* a container shares the host kernel and runs native code, so it changes neither speed nor results. A given binary gives bit-identical results on either host; determinism stays per build and compiler (D-005).
- *Host-side setup:* real-time scheduling, CPU pinning and memory locking (D-005) need the container's privileges and cpusets, which is part of the host deployment, not Eyas.

**CI changes made with it:**
- ctest runs in parallel (`--parallel $(nproc)`), so a job takes about its longest test rather than the sum.
- Compiled objects are cached with ccache through `actions/cache` (keys per job). ccache reuses an object only when the compiler, flags and preprocessed source match, so results are unchanged; the D-010 flags are untouched.
- The L3 gate runs in hover, at 60 kt and at 150 kt. `eyas_m7b_cases` covers every 10 kt (VALIDATION.md M7b).
- *Two tiers (approved 2026-10-15):*
  - *Every push and PR (`ci.yml`):* every job runs every test, except that the ASan job skips the tests listed in `SLOW_TESTS`, which the sanitizers slow about 60×. These are L3 and the L4 rate derivatives, each about 15 min under ASan.
  - *`full-verification.yml`, nightly on `main` and on demand:* every test under ASan, plus every validation generator and plot script, uploaded as an artifact for comparison with VALIDATION.md.
  - *The trade:* a memory error that only a slow test reaches could merge and be caught by the next nightly run. The slow tests still run in every other build on every push, and the trim sweep flies the same components at every speed under ASan.

## D-038 GenHel's elastic blade twist, with a lagged blade load; the profile-drag correction parked (D-034 item 5) — `Approved (2026-10-15)` — departure from GenHel (lag), addition (twist)
**Context:** D-034 item 5 deferred three GenHel main-rotor features to rulings on evidence from the M7 trim comparison. The M7c preview against Ballin's Fig. B1 (D-035 M7c amendment; readings still `UNVERIFIED`) showed Eyas's trimmed collective below GenHel's by 5.6 % of travel at 20 kt, 8.5 % at 100 kt and 24 % at 160 kt: an error growing with speed, the symptom item 5 names for the elastic twist.

**GenHel's model (CR-166309, read from the page images):**
- p. 5.1-24: blade dynamic twist. F_PDYMR = (F_pb² + F_Tb²)^½ per blade, the blade's aerodynamic force in blade span axes (p. 5.1-45: perpendicular and tangential, lbf); F_P0 = (1/b_s)(b/b_s) Σ F_PDYMR, with b = b_s = NBS = 4 simulated blades the mean over blades; harmonic terms F_PC, F_PS with gains K_FPC = K_FPS = 0 (p. 5.1-49), so only the mean acts. θ_DYTIP = K_FP0 F_P0 [deg], spread by MODESP = 0.28 + 0.72 sin(90° (y₂ + ξ)) with y₂ + ξ = r/R (p. 5.1-39), the "equivalent blade first torsional mode" (p. 5.1-45).
- p. 5.1-25: θ_I = THOAMR + THDYMR + preset twist: the dynamic twist adds to every segment's pitch.
- p. 5.1-49: K_FP0 = −0.0003 − 0.0000044 (V_kt − 100) deg/lbf, limited to [−0.00052, −0.0003]: −0.0003 up to 100 kt, −0.00052 from 150 kt.
- p. 5.1-27: C_DY = C_DY − ΔC_DMR, ΔC_DMR = 0.002: the "profile drag correction" *lowers* the section drag.

**Evidence (scratch probe, the twist folded into the twist table and iterated with the trim, F_P0 ≈ thrust/4):** tip twist −1.2° to 100 kt, −2.2° at 160 kt. Eyas − GenHel, rigid → twisted: collective −5.6 → +1.4 % (20 kt), −8.5 → −1.6 % (100 kt), −18 → −7 % (140 kt), −24 → −11 % (160 kt); longitudinal cyclic −6 → −4 %, −7 → −3 %, −12 → −5 %; pedal −2.6 → +1.3 %, −7 → −3.5 %, −12 → −6.5 %. Power and sideslip unchanged.

**Decision:**
- *Elastic twist: added* (item 5's rule is met). Per element, Δθ = θ_e m(r/R), with the gain K against the CG true airspeed and m against r/R as config tables (`blade.dynamic_twist`; the UH-60A's generated from GenHel's formulas). The tip twist θ_e is one rotor state: dθ_e/dt = (K(V) F − θ_e)/τ, F the mean over blades of |(F_t, F_n)| of the same evaluation (aerodynamic, blade span axes, as GenHel's F_PDYMR). The state is the angle, not the force, so it is scaled like the other rotor states in the trim's residual (a force state, about 17 000 N, swamped the trim's Newton step).
- *Why a lagged state (option A):* Δθ depends on the loads, which depend on Δθ. GenHel breaks the loop by using the previous time step's loads, which makes the derivative function depend on the step history; Eyas's must be a pure function of (t, x, inputs) (spec §6, trim and linearization rely on it). A state removes the loop. In any steady or periodic condition θ_e is the revolution mean of K F, so the trim is GenHel's fixed point and doesn't depend on τ (tested). τ is a numerical choice, not aircraft data: it only has to be fast against the flight dynamics and resolved by the 1 ms step. UH-60A: τ = 0.01 s.
- *Gain corners rounded (departure, as D-036):* K_FP0 is piecewise linear in airspeed with corners at 100 and 150 kt, where sweep trims sit; at a corner the model has no single slope, and L3 failed at 150 kt (a 16 % first-order leftover in the forward-speed perturbation). The gain table is read through `rounded()` (`util/table`): within δ = 0.1 × the shorter neighbouring segment of each corner (±5 kt here), the parabola tangent to both pieces. C¹; equal to GenHel's schedule outside the bands; at a corner off by (s_r − s_l) δ/4 = 5.5e-6 deg/lbf, under 2 % of the gain. Tested in `Table.RoundedCornersAreSmoothAndLocal`; with it L3 passes again at every gate speed.
- *Option B, documented for later:* solve the loop inside each evaluation (fixed-point iteration on θ_e, bounded, with a convergence flag, spec §7 rule 3). Exact at every instant, no extra state, but 2–3 blade-element passes per evaluation, against the real-time budget. Revisit if a response comparison shows τ matters, or if the twist's harmonic terms (K_FPC, K_FPS ≠ 0) are ever needed, where a lag would also shift their phase.
- *Profile-drag correction: parked, not ruled out.* GenHel lowers the drag, so adding it would lower Eyas's power, which is already 5–10 % below GenHel's. It can't explain that gap. Decide it once the gap's cause is found: if Eyas then sits above GenHel, the correction may be what GenHel used it for.
- *Rotor-force filter (T_FILMR):* unchanged, waits for the response comparisons (D-035 M7c amendment).
- *Verification (`tests/unit/test_dynamic_twist.cpp`):* at tip twist θ_e the loads equal the rigid rotor's with θ_e m(r/R) added to its twist table (to 1e-12); the state equation against blade forces taken from the rotor's total loads on a one-blade rotor; on the UH-60A at 100 kt the revolution mean of θ_e equals that of K F (7e-8 relative), the tip twist is −1.28° (GenHel's size: −0.0003 deg/lbf × about 4300 lbf per blade at sea level, 16 638 lbm), and halving τ moves the trim by under 1e-5 of control travel.
- *Validation records:* the M7a and M7b numbers in VALIDATION.md were measured without the twist; the M7c entry regenerates the trims and linear models with it.

## D-039 Yawed-flow drag on the blade elements (Johnson 2013 §6.22–6.23) — `Approved (2026-10-15)` — addition (refines D-024)
**Context:** with the elastic twist in (D-038), the M7c preview still had Eyas's main-rotor power 5–10 % below GenHel's (Ballin Fig. B1; readings `UNVERIFIED`), the gap growing with speed (−97 hp at 100 kt, −208 hp at 160 kt). Eyas's blade elements (D-024) use U = √(u_T² + u_P²) only. GenHel resolves its segment forces on the resultant velocity including the spanwise component, U_Y = √(U_T² + U_P² + U_R²) (CR-166309 p. 5.1-23), with a radial force F_R (p. 5.1-28); its lift additionally uses a yaw-angle transformation (pp. 5.1-25, 5.1-26).

**Source:** Johnson (2013) §6.22 "Yawed Flow and Radial Drag" and §6.23 "Profile Power" (pp. 226–237): by the equivalence assumptions for swept wings, the normal section's lift is not affected by yaw below stall, and the total viscous drag acts along the yawed flow, with c_d = c_d2D(α cos Λ)/cos Λ (Eq. 6.387), cos Λ = U/W, W² = U² + u_R² (Eq. 6.389). The section drag forces are (c_d2D/2) W (u_T, u_P, u_R) (Eqs. 6.397–6.399), so the profile power is ∫σc_d W³/2 dr (Eq. 6.403); of the familiar 1 + 4.65μ², radial drag contributes μ² and the yawed-flow drag coefficient 0.45μ² (p. 233).

**Decision:** an optional `blade.yawed_flow_drag` (off by default, so the M3–M5 reference fixtures keep the D-024 element): the drag force is ½ρc dr c_d W along the full relative wind (−u_T e_t + u_P n + u_R e_s), u_R the air's spanwise velocity relative to the blade, induced velocity included; lift unchanged; Mach from U (Johnson's normal-section Mach for an unswept blade, Eq. 6.378, and GenHel's, p. 5.1-23). On for the UH-60A.
- *Departure from Johnson:* c_d2D is read at the section α, not α cos Λ. The two differ only where c_d varies steeply with α, i.e. near stall; GenHel also reads drag at its section angle (p. 5.1-27). Johnson's form needs a wash-out towards ±90° that he doesn't define.
- *Not included:* the yawed-flow stall delay of the lift (Johnson Eq. 6.386; GenHel's α transformation), a stall effect to revisit with dynamic stall (M9).

**Verification (`tests/unit/test_yawed_flow.cpp`):**
- *Layer 1:* a lift-free rotor with constant c_d in edgewise flight against Johnson's integrals (Eqs. 6.418–6.419, reverse flow included), evaluated by independent quadrature: H-force and torque within 3e-6.
- *Layer 2:* profile power C_Q + μC_H against Johnson's Eq. 6.423 within 0.2 % from μ = 0.1 to 0.8 (Johnson: 1 % to μ = 1), and against 1 + 4.65μ² within 0.12 % to μ = 0.35, drifting to 1.8 % at 0.4 and 4 % at 0.5, as Johnson states for that approximation.
- No spanwise flow (hover, unflapped): loads bit-identical with the switch on and off.

**Effect (M7c preview, Ballin Fig. B1, readings `UNVERIFIED`):** Eyas − GenHel main-rotor power, mean over 0–160 kt −89 → −45 hp (largest −208 hp at 160 kt → −105 hp at 120 kt); collective mean −1.2 → −0.3 % of travel, at 160 kt −11 → −6 %; pitch attitude +0.86 → +0.68°. Sideslip unchanged (+3.5°): its cause is in the main rotor's side force, under investigation (VALIDATION.md M7c). *(2026-10-16: the main rotor's force integration matches Johnson 2013 Eqs. 6.75/6.85 and its lateral cyclic matches GenHel's; the cause is open, VALIDATION.md M7c.)* Hover power, where there is no spanwise flow, is still 73 hp (4.5 %) below GenHel's.
