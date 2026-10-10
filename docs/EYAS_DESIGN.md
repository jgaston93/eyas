# Eyas — Real-Time Rotorcraft Flight Model

**Design Specification v0.1**

Eyas is an independent, real-time, piloted rotorcraft flight dynamics model. It is intended to replace an incumbent commercial flight-dynamics package as the six-degree-of-freedom (6-DOF) flight model in a host simulator. That incumbent package is called **the reference model** throughout this document.

The architecture is built to grow toward multibody fidelity: individual blades with their own degrees of freedom, dynamic inflow, unsteady aerodynamics and, later, elastic blades. The first version deliberately uses rigid blades and simple inflow. That keeps the first flying model achievable and makes it possible to validate each layer before the next one is added.

---

## 0. Instructions for the Implementing Agent (Claude Code)

Read this section first and follow it for the whole project.

1. **Work one milestone at a time** (Section 9). Do not start milestone N+1 until every acceptance test for milestone N passes.
2. **Write the tests first.** Each milestone lists acceptance criteria. Turn them into GoogleTest cases before or alongside the implementation.
3. **Never invent aircraft data.** If a mass, inertia, airfoil table or geometry value is missing:
   - put a clearly marked placeholder in the config (`# PLACEHOLDER — needs source`);
   - log the gap in `docs/DATA_GAPS.md`;
   - continue.
4. **Cite the equation source in the code.** Every non-trivial equation gets a comment naming its reference and the equation or section number, e.g. `// Padfield 2nd ed. Eq. 3.xx`.
5. **Record design decisions.** Write each decision and each departure from this spec in `docs/DECISIONS.md`, with its date and a short rationale.
6. **Ask before changing an interface.** The interfaces in Section 6 are the contract between modules. Propose a change in `DECISIONS.md` before making it.
7. **Respect the real-time rules** (Section 7) from day one. Retrofitting them later is painful.
8. **Clean-room rule for the reference model** (see Section 2.1). Derive every equation, algorithm and data value from the public references in Section 2 only. Never read, copy, paraphrase or port the reference model's source code, documentation, manuals, model files or scripts, and do not ask for them. The reference model is treated as a black box: Eyas may be compared against its *outputs*, nothing more. If a task seems to need the reference model's internals, stop and ask.

---

## 1. Goals and Non-Goals

### Goals
- Run a real-time, fixed-step, deterministic piloted simulation of a single-main-rotor helicopter with a tail rotor.
- Use blade-element rotor aerodynamics with an individual state for each blade (flap first, then lag).
- Use dynamic inflow: Pitt–Peters first, then Peters–He finite-state.
- Provide a trim solver and a linearization tool, so results can be compared against the reference model and the published data.
- Keep a clean, modular C++ codebase where fidelity can be increased one component at a time.
- Stay data-driven: each aircraft is defined entirely by config and table files, never by code.

### Non-Goals (for v1)
- CFD or free-wake models (not compatible with real time).
- Visuals, cockpit I/O hardware drivers, or instructor-station UI. Eyas is the flight model only, behind a clean interface.
- **Transport and IPC.** Eyas is an in-process library. Shared memory, UDP and similar mechanisms belong to adapters in the host application (Section 6.10), not to Eyas.
- **Aircraft systems models.** Fuel, electrical, hydraulics, cautions/advisories and engine start sequencing are out of scope. A separate systems/avionics layer owns them; Eyas only supplies the physical quantities that layer needs (e.g. torque, engine speeds).
- Certification or qualification artifacts. Write code that would not prevent them later, but do not produce them now.

---

## 2. Reference Vehicle and Sources

### Primary reference vehicle: UH-60A Black Hawk
It is chosen because its full math model and data are publicly documented:
- Howlett, J.J., *UH-60A Black Hawk Engineering Simulation Program: Volume I — Mathematical Model*, NASA CR-166309, 1981 (the GenHel model). This is the primary source for geometry, mass properties, airfoil tables, fuselage aero tables and model structure.

### Cross-check vehicles
- The aircraft data in the appendices of Padfield's *Helicopter Flight Dynamics* (Bo105, Lynx, Puma).

### Core references
| Topic | Reference |
|---|---|
| Overall flight dynamics modeling, conventions, trim, linearization | G. Padfield, *Helicopter Flight Dynamics* (2nd/3rd ed.) |
| Blade-element theory, airfoils, dynamic stall, inflow | J.G. Leishman, *Principles of Helicopter Aerodynamics* (2nd ed.) |
| Comprehensive rotor theory, flap/lag dynamics | W. Johnson, *Helicopter Theory*; *Rotorcraft Aeromechanics* |
| Pitt–Peters dynamic inflow | Pitt & Peters, *Vertica* 5(1), 1981 |
| Peters–He finite-state inflow | Peters, Boyd & He, *JAHS* 34(4), 1989; Peters & He, *JAHS* 1991 |
| Dynamic stall | Leishman & Beddoes, *JAHS* 34(3), 1989 |

### Ground-truth oracle
The reference model is the main validation target. For each milestone, the acceptance tests compare against reference-model output for the same configuration wherever possible.

> **Data handling note:** keep any controlled or proprietary vehicle data (anything beyond the public NASA reports) out of public repositories. Load such data from a separate, access-controlled data directory.

### 2.1 Proprietary-Information Boundary (Reference Model)

The reference model is a commercial product of a third-party vendor. Eyas must be an independent implementation.

**Allowed inputs to Eyas development:**
- the public references in Section 2 (NASA CR-166309, Padfield, Leishman, Johnson, the cited journal papers);
- the host simulator's own code and documentation;
- *outputs* of a licensed installation of the reference model (trim tables, time histories, linear models), used only as a comparison target, and only as far as the license permits (see below).

**Not allowed in the Eyas repo, in agent context, or as a basis for design:**
- the reference model's source code, binaries, manuals, theory documentation, training material, scripts or GUI exports of model structure;
- the reference model's vehicle model or configuration files for any aircraft, and any parameter values taken from them. A vendor-supplied vehicle model may itself contain vendor-proprietary data, even for the UH-60.

**Direction of configuration matching:** when a test needs "the same configuration" in both models, Eyas is configured from public data (GenHel), and the reference-model comparison case is set up to match Eyas — never the other way round.

**Comparison data:**
- Store reference-model outputs in the separate, access-controlled data directory, not in the Eyas repo. Tests that need them are skipped (not failed) when that directory is absent.
- Log the provenance of each reference-model dataset (who generated it, software version, case setup) alongside the data.
- Eyas's own outputs and the `AircraftState` interface are ours and may live in the repo.

**Before using any reference-model outputs:** confirm that the license and any vendor agreements permit using those outputs to benchmark an independent replacement, and whether any vendor-supplied model data carries restrictions. Record that the check was done in `docs/DECISIONS.md`.

---

## 3. Technology Stack

- **Language:** C++20
- **Build:** CMake ≥ 3.24, with presets for Debug, Release and RelWithDebInfo
- **Linear algebra:** Eigen 3.4 (fixed-size types inside the real-time loop)
- **Tests:** GoogleTest; use `ctest` integration
- **Config:** YAML (yaml-cpp), loaded at init time only
- **Tables:** CSV, loaded at init into pre-allocated interpolation tables
- **Logging:** binary or CSV time-history recorder with a pre-allocated ring buffer
- **Plotting and analysis (offline):** Python scripts in `tools/` (numpy, matplotlib)
- **Units:** SI throughout. Convert imperial source data at load time, never at run time.
- **Precision:** `double` everywhere

---

## 4. Coordinate Frames and Conventions

Adopt **Padfield's conventions** throughout, and document any departure in `DECISIONS.md`.

| Frame | Symbol | Definition |
|---|---|---|
| Inertial | `n` | Flat-earth North-East-Down (NED). The WGS-84 round-earth model is a later option. |
| Body | `b` | Forward-Right-Down, origin at the aircraft CG |
| Hub (non-rotating) | `h` | Origin at the rotor hub; includes shaft tilt |
| Rotating hub | `r` | Rotates at azimuth ψ about the shaft axis |
| Blade | `bl` | Origin at the flap/lag hinge; x-axis along the blade span |

**Naming rule:** every vector variable carries its frame suffix, e.g. `v_b`, `F_h`, `omega_b`. Every rotation matrix is named `C_xy`, meaning it maps frame-y quantities into frame x.

**Attitude:** unit quaternion, renormalized every step. Euler angles are output only.

**Symbol rule:** ψ alone always means rotor azimuth. The Euler heading angle is written ψ_h (code: `heading`), so the two are never confused.

**Rotor:** the main rotor turns counter-clockwise viewed from above, as on the UH-60 (US convention). Make the direction a config flag. Azimuth ψ = 0 points aft (over the tail boom).

---

## 5. Architecture Overview

```
                 ┌─────────────────────┐
 Pilot inputs →  │  Control Mapping    │  collective, cyclic, pedals → θ0, θ1c, θ1s, θ_TR
                 └─────────┬───────────┘
                           ▼
 ┌──────────────────────────────────────────────────────────────┐
 │                    Force & Moment Models                     │
 │  ┌───────────────────────────────┐  ┌───────────┐ ┌────────┐ │
 │  │ Main Rotor                    │  │Tail Rotor │ │Fuselage│ │
 │  │  ├ Blade dynamics (per blade) │  └───────────┘ └────────┘ │
 │  │  ├ Blade-element aero         │  ┌───────────┐ ┌────────┐ │
 │  │  └ Inflow model ◄─────────────┤  │Empennage  │ │ Gear   │ │
 │  └───────────────────────────────┘  └───────────┘ └────────┘ │
 └───────────────────────────┬──────────────────────────────────┘
                             ▼  Σ forces & moments at CG
                 ┌─────────────────────┐
                 │  6-DOF Rigid Body   │
                 └─────────┬───────────┘
                           ▼
                 ┌─────────────────────┐
                 │  State Integrator   │  integrates ALL states as one vector
                 └─────────────────────┘
```

### Key architectural decision: one global state vector

Every component that has dynamics declares its own states and computes its own derivatives. These include:
- the 6-DOF body;
- each blade's flap (later lag and elastic modes);
- the inflow states;
- the rotor speed and engine (later).

The integrator assembles all of these into a single state vector and advances them together with one integration scheme.

This is the design choice that lets Eyas grow into multibody fidelity without restructuring. Adding blade lag or elastic modes then only means:
1. adding states to a component, and
2. adding their derivative computation.

### Coupling
The body motion (the hub's velocity, angular rate and acceleration) feeds into the blade equations. The blade root loads feed back into the body as hub forces and moments.

**Decision: solve the coupling exactly in every derivative evaluation** (no lagged accelerations).

The blade equations and the hub loads are *affine* in the body acceleration a = [v̇_b; ω̇_b] (6×1): the hub-motion inertial terms enter linearly. This holds for rigid flap/lag blades and for modal elastic blades. Each component therefore returns its results in two parts, the value at a = 0 plus a sensitivity to a:
```
ẋ_c    = ẋ0_c    + G_c a        (G_c: n_c × 6)
load_c = load0_c + H_c a        (H_c: 6 × 6, body frame about CG)
```
The executive then:
1. evaluates every component and force model at the current state;
2. solves the 6×6 system for a:
   `(M_rb − Σ_c H_c) a = Σ_c load0_c + Σ ForceModel loads + gravity − gyroscopic terms`,
   where M_rb is the rigid-body mass matrix (m·I₃, I_b);
3. back-substitutes `ẋ_c = ẋ0_c + G_c a` for each component.

This is a Schur complement of the full multibody mass matrix. Its cost is one 6×6 solve per evaluation, regardless of the number of blades or modes.

**Why:**
- The derivative function becomes a pure function of (t, x, inputs). Trim and linearization need that, because their finite-difference Jacobians must not depend on the order of earlier calls.
- `snapshot()` and `restore()` need no hidden acceleration state.
- It removes the one-evaluation lag in the hub-motion terms that M4 tests.
- M9 and M10 (lag, elastic modes) reuse the same mechanism unchanged.

Components whose loads do not depend on body acceleration set G_c and H_c to zero. Moving to an implicit *time integrator* (for stiff elastic modes) is a separate, later question (M10).

---

## 6. Module Specifications and Interfaces

The code below is an interface sketch. Refine it, but treat the shapes as the contract between modules.

```cpp
namespace eyas {

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

struct Wrench {            // force + moment, expressed in a named frame
    Vec3 F;                // [N]
    Vec3 M;                // [N·m] about the reference point
};

struct Controls {          // normalized pilot inputs
    double collective;     // [0, 1]
    double lat_cyclic;     // [-1, 1]
    double lon_cyclic;     // [-1, 1]
    double pedal;          // [-1, 1]
};

struct Environment {       // ISA atmosphere + wind
    double rho;            // [kg/m^3]
    double temperature;    // [K]
    double speed_of_sound; // [m/s]
    Vec3   wind_n;         // [m/s]
    double ground_height;  // [m]
};

struct BodyState {
    Vec3 pos_n;            // [m]
    Vec3 vel_b;            // [m/s]
    Quat q_nb;             // body → NED
    Vec3 omega_b;          // [rad/s]
};

using Mat66 = Eigen::Matrix<double, 6, 6>;

// A component's outputs for one derivative evaluation, split into the value
// at zero body acceleration plus the sensitivity to body acceleration
// a = [v̇_b; ω̇_b] (see Section 5, "Coupling"). Buffers are sized at init().
struct ComponentDerivs {
    std::span<double> xdot0;     // n_c          : state derivatives at a = 0
    std::span<double> G;         // n_c × 6, row-major : ∂ẋ/∂a
    Wrench            load0_b;   // body frame, about CG, at a = 0
    Mat66             H;         // ∂[F_b; M_b]/∂a
};

// Every dynamic component implements this.
class DynamicComponent {
public:
    virtual ~DynamicComponent() = default;
    virtual std::size_t num_states() const = 0;
    virtual void initial_states(std::span<double> x) const = 0;
    // Pure function of (ctx, x): const, no hidden state that affects results.
    // A cached table-search index is allowed only because it cannot change
    // the result, just the search time.
    virtual void derivatives(const SimContext& ctx,
                             std::span<const double> x,
                             ComponentDerivs& out) const = 0;
};

// Purely algebraic force models (no states): fuselage, empennage, gear.
class ForceModel {
public:
    virtual ~ForceModel() = default;
    virtual Wrench compute(const SimContext& ctx) = 0;   // body frame, about CG
};

}  // namespace eyas
```

`SimContext` carries read-only data for the current derivative evaluation:
- body state;
- controls;
- environment;
- time;
- rotor azimuth.

### 6.1 Six-DOF Rigid Body (`core/rigid_body`)

These are the rigid-body equations about the CG, written in body axes:

```
m (v̇_b + ω_b × v_b) = F_b + m C_bn g_n
I_b ω̇_b + ω_b × (I_b ω_b) = M_b
q̇_nb = ½ q_nb ⊗ [0, ω_b]
ṗ_n = C_nb v_b
```

- `I_b` is the full inertia tensor, including the products of inertia (I_xz matters for helicopters).
- The quaternion is renormalized after each step. Report quaternion-norm drift as a diagnostic.
- *Ref: Padfield Ch. 3; any standard flight dynamics text.*

### 6.2 Atmosphere (`env/atmosphere`)

- ISA standard atmosphere up to 11 km, with a temperature-offset parameter.
- Wind as a constant vector in v1. Turbulence models (Dryden or von Kármán) come later.

### 6.3 Control Mapping (`rotor/controls`)

Pilot inputs map to swashplate and tail-rotor pitch through linear gearing tables from the config. The mixing includes:
- collective → θ0;
- cyclic → θ1c and θ1s;
- pedal → θ_TR;
- any mechanical mixing (e.g. the UH-60 collective-to-pedal mixing).

The blade pitch at azimuth ψ and radius r is:
```
θ(ψ, r) = θ0 + θ_tw·(r/R) + θ1c cos ψ + θ1s sin ψ − tan(δ3)·β
```
Here θ_tw is the blade twist (a linear placeholder; use the twist table if available) and δ3 is the pitch–flap coupling angle.

Actuator dynamics are a first-order lag or rate limit, set in config. They are optional in v1.

### 6.4 Main Rotor (`rotor/main_rotor`)

The main rotor is a `DynamicComponent` that owns:
- the rotor azimuth ψ. In v1, rotor speed Ω is constant, so ψ is integrated as a state or computed from time. A rotor-speed DOF comes in M9.
- Per blade k = 1…N_b:
  - flap angle β_k and its rate β̇_k (M4);
  - later, lag angle ζ_k and its rate ζ̇_k (M9);
  - later still, elastic modal coordinates (M10).
- The inflow states (from the inflow sub-model).

#### Blade-element loop (each derivative evaluation)

```
for each blade k (azimuth ψ_k = ψ + 2π(k-1)/N_b):
    for each spanwise element j (radius r_j, width dr_j):
        1. Element velocity in the blade frame:
              hub translation + body rotation + rotor rotation (Ω r)
              + blade flap/lag rates − induced inflow at (r_j, ψ_k)
           → U_T (tangential), U_P (perpendicular), U_R (radial)
        2. Inflow angle φ = atan2(U_P, U_T)
           (sign convention per Padfield; document it)
        3. Angle of attack α = θ(ψ_k, r_j) − φ
        4. Mach number M = sqrt(U_T² + U_P²) / a
        5. Look up Cl(α, M) and Cd(α, M) in the airfoil tables
           (use the full ±180° tables for reverse flow)
        6. dL = ½ ρ U² c Cl dr ;  dD = ½ ρ U² c Cd dr
        7. Resolve to the blade frame, then accumulate:
              - the aerodynamic flap moment about the hinge (for blade dynamics)
              - the force and moment at the hub (for the body)
```

- **Elements:** 15–25 per blade. Use non-uniform spacing that is denser near the tip, or annuli of equal area.
- **Tip loss:** a Prandtl tip-loss factor, or an effective radius B·R with B ≈ 0.97, chosen in config.
- **Root cutout:** no aerodynamic loads inboard of the cutout radius.
- *Refs: Leishman Ch. 3; Padfield Ch. 3; GenHel (NASA CR-166309) for the UH-60 specifics.*

#### Blade flap dynamics (rigid blade, articulated, M4)

Each blade is rigid and hinged at an offset e·R, with an optional hinge spring K_β and damping. Derive the full flap equation, including the hub-motion inertial terms:
- body angular rates p and q;
- body angular accelerations;
- hub linear acceleration.

Follow Padfield Ch. 3 / Johnson for the derivation. The body-acceleration terms are not evaluated from a known acceleration; they are returned as the sensitivities G and H (Section 5, "Coupling"). Unit-test G and H against finite differences of the full equation. In the limit of no hub motion, the equation must reduce to:
```
I_β β̈ + I_β Ω² ν_β² β = M_aero(β, β̇, …)
```
with the rotating flap natural frequency:
```
ν_β² = 1 + (3/2)·e/(1−e) + K_β/(I_β Ω²)     (uniform blade)
```
Use the actual mass distribution from the data, rather than the uniform-blade formula, once it is available.

The blade root shear and moment are transmitted to the hub. They include:
- the aerodynamic loads;
- the inertial loads (centrifugal and Coriolis);
- the blade weight.

#### Hub loads to the body
Sum the root loads of all blades in the hub frame, then transform to body axes about the CG using the hub position and shaft tilt from config.

### 6.5 Inflow Models (`rotor/inflow`)

Use a pluggable interface so that models can be swapped by config.

| Model | Milestone | States | Notes |
|---|---|---|---|
| Uniform momentum (Glauert) | M3 | 0 (solved iteratively each step), or 1 with a lag | Baseline |
| Pitt–Peters | M5 | 3 (λ0, λ1s, λ1c) | Standard real-time workhorse |
| Peters–He finite-state | M9 | Configurable harmonics | Higher fidelity |

**Pitt–Peters** (non-dimensional, in rotor time τ = Ωt):
```
[M]{λ̇} + [L]⁻¹{λ} = {C_T, −C_L, −C_M}ᵀ_aero
```
- The thrust and moment coefficients come from the blade-element loads in the same evaluation, so the coupling is captured naturally by the global integrator.
- Use the [M] and [L] matrices as published, including the wake skew angle dependence.
- Induced velocity at a point: `λ_i(r, ψ) = λ0 + λ1s (r/R) sin ψ + λ1c (r/R) cos ψ`.
- *Refs: Pitt & Peters 1981; Leishman Ch. 10; Padfield Ch. 3.*

Ground effect (M9) applies a Cheeseman–Bennett-type correction or an inflow reduction, set in config.

### 6.6 Tail Rotor (`rotor/tail_rotor`)

- **v1:** a Bailey-type closed-form rotor model, as used in GenHel. Thrust and torque come from θ_TR, the local inflow and the advance ratio, with a uniform inflow lag.
- **Later:** reuse the main-rotor blade-element module with tail-rotor geometry and δ3.
- **Blockage:** include fin blockage and canting (UH-60 tail rotor cant ≈ 20°; take the value from the data).

### 6.7 Fuselage, Empennage and Gear (`airframe/`)

- **Fuselage:** six-component aero tables against angle of attack and sideslip (from GenHel), using dynamic pressure at the fuselage reference point. Include main-rotor downwash on the fuselage through an interference factor or table.
- **Horizontal stabilator and vertical fin:** flat-plate or table lift and drag, computed with the local velocity. That velocity includes:
  - body rates × arm;
  - main-rotor wake impingement (a downwash table against wake skew).
  
  The UH-60 stabilator schedule is a function of airspeed, collective and pitch rate; implement it as a table-driven scheduler.
- **Landing gear:** spring-damper struts with tire friction (stick–slip with a smooth transition for numerical stability). Use contact points from config.

### 6.8 Trim Solver (`analysis/trim`)

Use Newton–Raphson with finite-difference Jacobians.
- **Unknowns:** θ0, θ1c, θ1s, θ_TR, pitch attitude θ and roll attitude φ (6).
- **Residuals:** the body accelerations (u̇, v̇, ẇ, ṗ, q̇, ṙ) = 0, with the rotor states periodic or converged.

For rotor states, average the residuals over one or more full rotor revolutions with the blades integrated through azimuth (a periodic trim), or use quasi-steady harmonic flapping. Document which.

Trim cases: hover, and level forward flight from 0 to max speed in 10-knot steps; later, climbs, descents and steady turns.

### 6.9 Linearization (`analysis/linearize`)

Compute central-difference A and B matrices about a trim point for the reduced rigid-body states (u, v, w, p, q, r, φ, θ, ψ_h), where ψ_h is the heading (see the symbol rule in Section 4), with the rotor and inflow states either:
- **quasi-statically condensed** (to compare with Padfield's 6-DOF derivatives), or
- **retained** (higher-order model).

Output the stability and control derivatives and the eigenvalues, so they can be compared with the reference model and the published data.

### 6.10 Simulation Executive and Library API (`sim/`, `include/eyas/`)

Eyas is delivered as an **in-process C++ library**. It knows nothing about shared memory, sockets or the host's frame scheduling.

- Fixed-step physics loop. The physics rate is configurable; the default is 1000 Hz (about a 1.55° azimuth step at the UH-60 rotor speed of ~258 rpm, Ω ≈ 27 rad/s).
- The caller advances Eyas by a host frame time `dt_host` (e.g. 1/60 s).
  - Eyas works out the number of sub-steps itself, by dividing `dt_host` by the configured physics step.
  - It runs that many fixed sub-steps and carries any remaining time to the next call.
  - It never assumes a fixed host rate, and `dt_host` may vary from call to call. The host rate and the physics rate are therefore fully decoupled.
- **Time keeping in integer ticks.** Floating-point remainder accumulation drifts and makes runs depend on how time was split across calls, so:
  - The physics step is stored as an integer number of nanoseconds (1000 Hz → 1 000 000 ns). `init()` rejects a physics rate that does not divide 1 s into whole nanoseconds.
  - `advance()` converts `dt_host` to integer nanoseconds once, at entry (round to nearest), and adds it to an integer remainder. It runs `remainder / step_ns` sub-steps and keeps `remainder % step_ns`.
  - Sim time is never accumulated. It is computed as `t = n_steps × dt`, where `n_steps` is a 64-bit step counter.
  - `snapshot()`/`restore()` include `n_steps` and the remainder.
- **Over-long frames:** cap the number of sub-steps per call. A configurable maximum, e.g. 100, covers a debugger pause or a host hitch without trying to catch up all at once. When the cap is hit, drop the excess time and raise a diagnostic flag in the output.
- **Terminology:** `advance()` is the public call. `step()` is the internal function that runs one fixed physics sub-step; `advance()` calls it zero or more times.
- **Integrator:** RK4 by default. Provide an interface so a semi-implicit or symplectic scheme can be swapped in.
- **Library API:** `init(config)`, `trim(condition)`, `advance(controls, env, dt_host)`, `outputs() → AircraftState`, `reset(state)`, `snapshot()` and `restore()`.

#### Threading

Eyas itself is **single-threaded and synchronous**. `advance()` runs on the caller's thread and returns once the sub-steps are done. This is deliberate, and it is the only supported mode for v1.

**Why synchronous: determinism.**
- The same inputs produce bit-identical outputs on every run.
- That is essential for validation against the reference model, for regression tests and for debugging.
- The cost: a physics frame that runs over budget stalls the caller. The timing budget in Section 7 exists to prevent that.

**If asynchronous execution is wanted** (Eyas running in its own thread at its own rate while the host samples the latest published state, as the reference model does):
- Put the thread, its loop and the state handoff in a wrapper on the **host application side** that owns the Eyas instance. Use a double buffer or a seqlock for the handoff so the host never reads a half-updated state.
- Do not add threads inside Eyas. That keeps the core deterministic and testable, and makes asynchronous running a deliberate choice by the host.
- Trade-offs to accept with that wrapper:
  - bit-for-bit determinism is lost;
  - the usual concurrency burden applies.

#### Adapter pattern (host application side)

The host application codes against an abstract flight-model interface. Each flight model plugs in behind an adapter:

```
           Host application
                  │
        IFlightModel (abstract)
           ┌──────┴────────┐
 ReferenceAdapter      EyasAdapter
 (reads reference      (links libeyas,
  shared memory)        calls advance())
```

- The host's interface and the adapters live in the host application's codebase, not in the Eyas repo. Eyas provides only the library API and its `AircraftState` record.
- The adapters translate between each model's native outputs and the host's interface: units, frames, field names.
- Tests and tools link Eyas directly, with no transport.

#### Output record: `AircraftState`

A single structured output record that grows in tiers. Fields are added over time; existing fields never change meaning, units or frame. Every field is documented with its units and frame.

| Tier | Milestone | Purpose | Contents |
|---|---|---|---|
| 1 — Pose | M2 onward | Drive the out-the-window view | Geodetic position (lat, lon, alt) computed from NED + a configurable geodetic origin; attitude (quaternion and Euler angles); sim time |
| 2 — Inertial / nav state | M7–M8 | Drive EGI/INS-type consumers | Velocities (NED and body), body angular rates, body accelerations and specific force, load factor, true heading, track, ground speed |
| 3 — Air data | M8 | Feed PFD-type displays | Indicated/calibrated/true airspeed, angle of attack, sideslip, pressure altitude, vertical speed, outside air temperature |
| 4 — Rotor and powertrain | M8 (rotor), M9 (engine) | Feed engine/rotor instruments | Rotor speed (%NR), main-rotor thrust, torque and power, tail-rotor thrust, flapping; with the M9 engine DOF: engine torque (%Q), gas-generator speed (NG), power-turbine speed (NP) |
| 5 — Diagnostics | Any | Engineering use | Inflow states, per-blade states, integrator health flags, quaternion norm drift |

The exact field list for Tiers 1–3 will be finalized against the host application's required outputs (see Section 12). Until then, implement the fields above.

---

## 7. Real-Time Rules

These apply to everything reachable from `advance()`, including each internal `step()` (Section 6.10).

1. **No heap allocation.** All buffers are sized at `init()`. Add a test that installs an allocation hook (a counting `operator new`) and asserts zero allocations during `advance()`.
2. **No exceptions and no I/O in the loop.** Report errors through status codes and flags.
3. **No unbounded loops.** Iterative solves (e.g. uniform inflow) have a fixed maximum number of iterations and a convergence flag.
4. **Deterministic.** The same inputs always give bit-identical outputs on the same build. Test this.
5. **Timing budget.** On the target hardware, worst-case `step()` must stay under 50% of the host frame time. Add a benchmark executable that reports the mean, p99 and maximum.
6. **Table lookups.** Use pre-computed breakpoint search with a cached index (hunt search). Never use a linear scan of a large table.

---

## 8. Repository Layout

```
eyas/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md
├── docs/
│   ├── EYAS_DESIGN.md        ← this file
│   ├── DECISIONS.md
│   ├── DATA_GAPS.md
│   └── VALIDATION.md         ← results for each milestone, with plots
├── include/eyas/             ← public headers: library API + AircraftState output record
├── src/
│   ├── core/                 ← rigid body, quaternion, integrator, state vector
│   ├── env/                  ← atmosphere, wind
│   ├── rotor/                ← main rotor, blade dynamics, BEMT, inflow, tail rotor, controls
│   ├── airframe/             ← fuselage, empennage, gear
│   ├── analysis/             ← trim, linearization
│   ├── sim/                  ← executive, host API, recorder
│   └── util/                 ← tables and interpolation, units, config loading
├── data/
│   └── uh60a/                ← YAML config + CSV tables (public data only)
├── tests/
│   ├── unit/
│   └── validation/           ← tests against published data and the reference model
├── bench/                    ← real-time timing benchmark
└── tools/                    ← Python: plotting, reference-model output comparison
```

---

## 9. Milestones

Each milestone ends with all of its tests passing and a short entry in `VALIDATION.md` with plots.

### M0 — Scaffolding
- CMake project with presets, GoogleTest, Eigen and yaml-cpp, via FetchContent or vcpkg.
- Config loader, table loader and interpolation (1-D, 2-D, 3-D linear), units helpers.
- Time-history recorder.
- **Accept:**
  - the project builds with `-Wall -Wextra -Werror`;
  - interpolation tests pass, including edge cases and clamping;
  - CI runs the tests.

### M1 — 6-DOF Rigid Body Core
- Rigid body, quaternion kinematics, RK4, global state vector assembly, `DynamicComponent` plumbing.
- **Accept:**
  - Free fall matches the analytic solution to 1e-9 relative error over 10 s.
  - Torque-free spin of an asymmetric body conserves kinetic energy and angular momentum (both to < 1e-8 relative drift over 60 s), and shows the intermediate-axis instability.
  - Constant applied moment gives the analytic angular acceleration.
  - Coupled solve: a test component with known G and H (e.g. a point mass on a spring attached to the body) gives the same accelerations as the analytic combined system.
  - Purity: calling the full derivative function twice on the same (t, x, inputs), with other evaluations in between, gives bit-identical results.
  - Time keeping: `advance()` with a 1/60 s `dt_host` for 1 hour gives exactly the expected step count, and sim time equals `n_steps × dt` exactly.
  - Per-step quaternion norm drift, measured *before* renormalization, stays below 1e-12 (measuring after renormalization is trivially satisfied and tests nothing).
  - The zero-allocation test passes.

### M2 — Environment and Static Force Stubs
- ISA atmosphere, wind, and a constant "dummy rotor" force that balances weight.
- **Accept:**
  - ISA values match the standard tables at 0, 1000, 5000 and 11000 m.
  - Started at rest with the dummy force exactly equal to weight, the aircraft stays in equilibrium (position and velocity drift within tolerance) for 60 s. Note: a constant force gives a neutral equilibrium, not a stable one, so do not test recovery from disturbances here.
  - A step in the dummy force gives the expected vertical acceleration.
  - Tier 1 (pose) of `AircraftState` is populated; NED-to-geodetic conversion matches a reference calculation at several origins.

### M3 — Blade-Element Rotor, Rigid Blades, Uniform Inflow
- Blade-element loop, airfoil tables, control mapping, uniform momentum inflow, hub loads. Flapping is held at zero.
- **Accept:**
  - Hover thrust, torque and power match blade-element momentum theory within 2%.
  - Thrust coefficient against collective matches the reference model within 3% (rigid blades; the reference-model case is set up to match Eyas's GenHel-derived geometry, per Section 2.1).
  - Thrust, torque and power at constant collective are steady across azimuth for N_b blades in hover.

### M4 — Individual Blade Flap Dynamics
- Flap degree of freedom per blade with hinge offset, spring and hub-motion coupling; δ3 coupling.
- **Accept:**
  - The non-rotating and rotating flap natural frequencies match ν_β.
  - Coning angle against thrust matches the analytic result and the reference model.
  - Longitudinal and lateral flapping (a1, b1) against advance ratio match Padfield's analytic expressions at low μ and the reference model across the speed range.
  - The flap response to a cyclic step has the expected phase lag of about 90° minus the hinge-offset effect.

### M5 — Pitt–Peters Dynamic Inflow
- **Accept:**
  - In steady state, the result reduces to uniform momentum inflow in hover.
  - The inflow time constant in hover matches the published value.
  - The step-collective thrust overshoot shows the characteristic dynamic-inflow response, compared against the reference model.

### M6 — Tail Rotor, Fuselage, Empennage, Gear
- **Accept:**
  - The aircraft sits stably on the ground with gear loads equal to weight.
  - Hover pedal position and tail-rotor thrust balance main-rotor torque.
  - Fuselage and empennage forces match GenHel table values at the test points.

### M7 — Trim and Linearization (first full validation gate)
- **Accept:**
  - Trim converges from 0 to 150 kt.
  - Trimmed control positions, pitch attitude, roll attitude and power each match the reference model within agreed tolerances. Suggested starting tolerances: ±5% of control travel and ±1° attitude; tune them with the team.
  - Hover and cruise eigenvalues match the reference model in mode character (phugoid, Dutch roll, spiral, heave and so on), with frequency and damping in the same neighborhood.
  - Key derivatives (X_u, Z_w, M_q, L_p, N_r, Z_θ0 …) are within the agreed tolerance of the reference model.

### M8 — Real-Time Library and Piloted Integration
- Finalize the library API and sub-stepping (`advance()` with remainder carry).
- Populate Tiers 2–4 of `AircraftState` (rotor portion of Tier 4).
- Support the host team in writing the `EyasAdapter` against the host's flight-model interface. A minimal reference adapter and a standalone test driver live in Eyas under `examples/`.
- **Accept:**
  - Running at a 60 Hz host rate and at a 1000 Hz physics rate gives a **bit-identical** trajectory to calling at 1000 Hz directly, given the same control input on each physics sub-step. This proves the decoupling. The same holds for a jittered `dt_host` sequence with the same total time.
  - Swapping the host between `ReferenceAdapter` and `EyasAdapter` needs no change to host code.
  - The timing budget is met (see Section 7).
  - The determinism test passes.
  - A 30-minute run is free of drift and NaNs.
  - A pilot can hover, transition and fly a pattern.
  - Pilot comments are logged in `VALIDATION.md`.

### M9 — Fidelity Upgrades (each is independently toggleable in config)
- Blade lag degree of freedom with a lead–lag damper (check for ground resonance behavior on the gear).
- Peters–He finite-state inflow.
- Leishman–Beddoes dynamic stall.
- Ground effect.
- Rotor-speed degree of freedom, with drivetrain inertia, engine and governor (needed for autorotation). This also completes Tier 4 of `AircraftState` (engine torque, NG, NP).
- Dryden turbulence.
- **Accept:** each feature has its own validation entry against the reference model or published data, and the M7 regression still passes with the feature on and with it off.

### M10 — Elastic Blades (multibody target)
- Modal representation of blade flexibility: coupled flap/lag/torsion modes from a beam model of the blade properties.
- Revisit the time integrator: the rotor–body coupling is already exact (Section 5), but stiff elastic modes may need an implicit or semi-implicit integrator if RK4 limits the step size.
- **Accept:**
  - Blade natural frequencies (fan plot) match the reference.
  - Vibratory hub loads trend correctly against the reference model.
  - The real-time budget is still met. If it is not, document the fidelity-versus-rate trade-off.

---

## 10. Validation Strategy

- **Continuous regression:** every validation test runs in CI. Store reference data derived from public sources, and Eyas's own regression baselines, under `tests/validation/reference/`. Reference-model-generated references live in the access-controlled data directory (Section 2.1); CI jobs that have access to it run those comparisons, and the others skip them.
- **Reference-model comparison toolkit:** Python scripts that ingest reference-model trim and time-history exports in the same format as Eyas output, overlay the two, and compute error metrics.
- **Standard maneuver set:** 1-second doublets on each axis, collective and cyclic steps, and frequency sweeps (for comparisons in the style of the CIFER system-identification tool) at hover, 60 kt and 120 kt.
- **Handling-qualities checks (later):** ADS-33E bandwidth and phase delay from frequency sweeps, compared between Eyas and the reference model.

---

## 11. Coding Standards

- C++20. Prefer `constexpr`, `std::span` and fixed-size Eigen types in hot paths.
- No raw `new` or `delete`. Ownership is clear: components are owned by the executive.
- Every public function is documented with its units and frames.
- Write a unit test for every equation implementation, with at least one hand-computed case.
- Use clang-format and clang-tidy, with the configs committed.
- Keep coding patterns MISRA-friendly where it does not cost clarity (no dynamic memory or exceptions in the loop, explicit casts). Full MISRA conformance is not a v1 goal.

---

## 12. Open Questions (to resolve with the team)

1. **Required outputs.** What exact fields does the host application read from the reference model today, with their units and frames? This finalizes Tiers 1–3 of `AircraftState` and defines what the `EyasAdapter` must produce. Known so far: the reference model is read over shared memory at about 60 Hz; it supplies engine power but not fuel.
2. Which aircraft does the simulator actually need, after the UH-60 reference? Does its data exist in a usable form?
3. What validation tolerances do the stakeholders accept for "replaces the reference model"?
4. What are the target hardware and OS for real-time (RTOS or Linux PREEMPT_RT)?
5. Is autorotation or engine failure in scope for the first piloted release? If so, pull the rotor-speed degree of freedom from M9 into M8.
6. **Reference-model license terms (blocking for comparisons).** Does the license, or any vendor agreement, restrict using its outputs to benchmark an independent replacement? Does any vendor-supplied model data carry restrictions? See Section 2.1.
