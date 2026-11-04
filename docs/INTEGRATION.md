# Integrating Eyas into a host simulator

How a host loads an aircraft, trims it, drives it frame by frame and reads
its state back. Written for the team that owns the host's flight-model
interface. The API is `include/eyas/`; this guide summarizes it, and the
header comments are the authority where they go into more detail.

*Keep this guide current:* a change to the public API, a config, the
defaults, the status flags or the timing updates it in the same PR
(CLAUDE.md, Process).

## 1. Overview

Eyas is a C++20 static library. One `eyas::Model` is one aircraft. It is
single-threaded and synchronous: every call runs on the caller's thread and
returns when it is done. Eyas creates no threads and does no I/O after start-up,
so it never talks to the host over a socket or shared memory; the host links it
and calls it.

The recommended shape is an **adapter**: the host keeps its own flight-model
interface, and one small class translates between it and Eyas (units, sign
conventions, field names, health flags). `examples/adapter/` is a working
example, written against a toy host interface in feet, knots and degrees with
the pedal positive left, so every translation it needs is visible.

The API is only `include/eyas/`: `model.hpp`, `types.hpp`, `trim.hpp`,
`aircraft_state.hpp` and `error.hpp`. These headers never include anything
from `src/` (a test builds them without it), and the implementation sits behind
a pointer, so internal changes don't reach host builds. Output fields are only
ever added: an existing field never changes meaning, units or frame.

## 2. Building and linking

Add the repository to the host's CMake build and link `eyas::eyas`. The target
carries its include path and its dependencies.

```cmake
add_subdirectory(third_party/eyas)        # or FetchContent at a pinned commit
target_link_libraries(host_sim PRIVATE eyas::eyas)
```

- **Compiler:** the target platform is a RHEL 9 container with its system GCC
  11; configure rejects older GCC. CMake 3.24 or newer (RHEL 9 ships
  3.26–3.31). Clang builds it too and is tested in CI.
- **Dependencies** are fetched at configure time, pinned by version and
  SHA-256: Eigen 3.4.0 and yaml-cpp 0.8.0 (GoogleTest only for the tests). On
  an offline machine, point `-DFETCHCONTENT_SOURCE_DIR_<NAME>` at local copies.
- **Eigen is in the public headers.** `eyas::Vec3` and `eyas::Quat` are
  `Eigen::Vector3d` and `Eigen::Quaterniond`. The target provides Eigen's
  include path as a system include, so the host needs nothing extra.
- **Options:** when Eyas isn't the top-level project, its tests, benchmark and
  examples are off (`EYAS_BUILD_TESTS`, `EYAS_BUILD_BENCH`,
  `EYAS_BUILD_EXAMPLES`).
- **Determinism is per build and per compiler.** Eyas compiles itself with
  `-ffp-contract=off` and refuses fast-math. Two runs with the same inputs from
  the same binary are bit-identical; another compiler or version may differ in
  the last bits. Pin the compiler for any build whose runs must be compared.
- There are no install rules yet: build Eyas as part of the host.

## 3. Lifecycle

`Model::from_file` → `placeholders()` → `trim(condition)` → each frame
`advance()` and `outputs()`.

| Call | When | What it does |
|---|---|---|
| `Model::from_file(path)`, `Model::from_file(path, origin)` | init | Builds the aircraft from a YAML config. Throws `eyas::InitError` naming the file and key on any error. The second form takes the host's `GeodeticOrigin`, which replaces the config's (section 9). `from_string` takes YAML text. |
| `placeholders()` | init | Every value in the aircraft without a published source yet, by config key path. Eyas does no I/O, so the host decides whether to log, show or refuse them. |
| `trim(condition)`, `trim(condition, start)` | init | Solves for the controls and attitude of steady straight flight. On convergence the model is reset to t = 0 at the trimmed state, so the next `advance()` flies on from trim; otherwise it is unchanged. Check `converged`. Allocates: never call it in the real-time loop. Pass a previous result as `start` for a nearby condition: faster and more robust. |
| `advance(controls, env, dt_ns)` | real time | Runs one host frame of physics and returns an `AdvanceStatus`. |
| `outputs()` | real time | The `AircraftState` at the end of the last frame: a reference into the model; copy what you need. |
| `reset(body)` | real time | Restarts at t = 0 from a rigid-body state, every other state at its initial value. Returns false and changes nothing on a non-finite state. For a trimmed restart, call `trim()`. |
| `make_snapshot()` | init | Allocates a snapshot buffer sized for this model, once. |
| `snapshot(buf)`, `restore(buf)` | real time | Save and restore the whole state, including the controls and environment the outputs were evaluated with. Reusing the buffer never allocates. For freeze, rewind and instructor-station recall. |

`Model` is movable, not copyable. Keep it in a `std::optional` or
`std::unique_ptr` on the host side, so it can be empty before initialization.

## 4. A minimal adapter

Condensed from `examples/adapter/eyas_adapter.cpp`; the host's own types are in
namespace `host`.

```cpp
// Init time: may throw and allocate.
bool EyasAdapter::initialize(const std::string& aircraft, const host::InitialCondition& ic,
                             std::string& error) {
    try {
        model_.emplace(eyas::Model::from_file(aircraft, origin_));
        eyas::TrimCondition c;
        c.airspeed = ic.airspeed_kt * kKnot;  // [m/s] true airspeed
        c.altitude = ic.altitude_ft * kFoot;  // [m] MSL
        c.heading = ic.heading_deg / kDeg;    // [rad] true
        c.afcs = {ic.sas, ic.sas, ic.pitch_bias, ic.fps, false, false};
        const eyas::TrimResult r = model_->trim(c);
        if (!r.converged) {
            error = "trim did not converge";
            return false;
        }
        env_ = eyas::Environment{c.isa_delta_T, c.wind_n, c.ground_height};  // the trim's air
        trim_inputs_ = to_host(r.controls);  // start the cockpit controls here
    } catch (const std::exception& e) {
        error = e.what();
        model_.reset();
        return false;
    }
    return true;
}

// Real time: no allocation, no exceptions, no I/O.
void EyasAdapter::update(const host::PilotInputs& in, std::int64_t dt_ns) {
    eyas::Controls c;
    c.collective = in.collective;
    c.lat_cyclic = in.lateral;
    c.lon_cyclic = in.longitudinal;
    c.pedal = -in.pedal;  // host: + left; Eyas: + right
    c.brake_left = c.brake_right = in.brakes ? 1.0 : 0.0;
    c.afcs = {in.sas, in.sas, in.pitch_bias, in.fps, in.turn_coordination, in.trim_release};
    const eyas::AdvanceStatus s = model_->advance(c, env_, dt_ns);
    if (s.non_finite || s.rejected_dt || s.rejected_env || s.capped) {
        healthy_ = false;
    }
}
```

The host code above the adapter names the flight model in one factory
function, so swapping flight models changes one line. Build and run the
example with `./build/release/examples/eyas_adapter_example data/uh60a/uh60a.yaml`.

## 5. Frames and time

- **Frame time is integer nanoseconds.** `advance()` runs as many fixed physics
  steps as fit in `dt_ns` (1000 Hz by default; `simulation.physics_rate` in
  the config) and carries the remainder to the next call. Simulation time is
  steps × step, exact, so nothing drifts over a long session.
- **Pass the nominal frame period**, not a measured one, when runs must be
  reproducible: at 60 Hz, 16 666 667 ns every frame. Measured periods work, but
  their jitter changes the trajectory in the last bits.
- **A long frame is capped.** One call runs at most `simulation.max_substeps`
  steps (100 by default: 0.1 s of physics). The extra time is dropped and
  `capped` is set: after a host hitch the aircraft falls behind the wall clock
  rather than trying to catch up.
- A negative `dt_ns` is rejected and nothing happens. Zero runs no steps.

## 6. The real-time contract

Everything reachable from `advance()`, `outputs()`, `reset()`, `snapshot()`
and `restore()` follows these rules, each enforced by a test:
- no heap allocation (checked with a counting `operator new`);
- no exceptions: the calls are `noexcept`, and faults come back as status flags;
- no I/O;
- bounded iterations, each with a convergence flag;
- bit-identical results for the same inputs from the same binary.

**Threading.** Call a given `Model` from one thread at a time. Thread priority
and CPU placement are the host's: Eyas has no threads of its own, and its
timing assumes the simulation thread isn't preempted mid-frame.

**Timing.** `eyas_bench` measures `step()` (one 1 ms physics step) and
`advance()` (one 60 Hz frame, 17 steps plus the outputs) and checks the worst
frame against the budget of 8.3 ms (D-005). On the development machine
(desktop, WSL2, GCC 13.3, Release; 2026-11-04), in µs:

| Config | Case | `step()` mean | `advance()` mean / p99 / worst |
|---|---|---|---|
| `uh60a.yaml` (GenHel's engine) | 120 kt | 16 | 268 / 348 / 631 |
| | vortex-ring descent | 32 | 541 / 687 / 1107 |
| `engine_ballin/uh60a.yaml` (Ballin's engines) | 120 kt | 28 | 472 / 657 / 1044 |
| | vortex-ring descent | 44 | 741 / 1044 / 1371 |

Run `eyas_bench --config <yaml>` on the target hardware before committing to a
frame rate; `--load N` adds N busy threads to mimic the host's load, and
`--rate` sets the frame rate.

## 7. Inputs: `eyas::Controls`

Normalized cockpit positions. The trim returns them in the same form, so the
host can drive its control loaders or force-feel system to the trimmed position
before the first frame.

| Field | Range | Convention |
|---|---|---|
| `collective` | 0 to 1 | 0 full down, 1 full up |
| `lat_cyclic` | −1 to 1 | +1 full right stick |
| `lon_cyclic` | −1 to 1 | +1 full forward stick |
| `pedal` | −1 to 1 | +1 full **right** pedal (many hosts use + left: check yours) |
| `brake_left`, `brake_right` | 0 to 1 | 0 released, 1 full; clamped; NaN reads as released |
| `afcs` | switches | The flight control system's switches, below |
| `engines[k].power_lever` | 0 to 1 | 0 the idle stop, 1 fly; clamped; NaN reads as fly. Ignored by GenHel's engine |
| `engines[k].fuel_on` | bool | false closes the fuel shutoff: the engine runs down and flames out; true again relights it at once (no starter). Ignored by GenHel's engine |
| `rotor_speed_reference` | ≈ 1 | The engine-speed trim, a fraction of the governed rotor speed; a value that isn't positive and finite reads as 1 |

Engines take `engines[]` entries in config order (the UH-60A uses 0 and 1; at
most `kMaxEngines` = 4).

**Flight control system switches** (`AfcsSwitches`, all off by default):

| Field | Cockpit control |
|---|---|
| `sas_digital`, `sas_analog` | The two stability augmentation channels. With both on each works at gain 1; with one on, that one works at gain 2. A host with one SAS switch sets both. |
| `pitch_bias` | The pitch bias actuator. Switched off it drives back to centre at its rate limit. |
| `fps` | Flight path stabilization: holds the pitch and roll attitude, the airspeed above 30 kt and the heading. |
| `trim_release` | The cyclic's momentary trim release, true while held: the FPS references follow the aircraft and its integrators stop, so the pilot can fly to a new attitude and let go there. |
| `fps_turn` | Turn coordination in place of heading hold. The host latches it, as the aircraft's logic does: on with the FPS on, above 60 kt, the trim release pressed and more than 2° of bank; off with the feet on the pedals and the trim release pressed, or with the FPS off. `examples/common/afcs.hpp` has `latch_turn_switch()` to copy. |

The UH-60A normally flies with the SAS, pitch bias and FPS on. With everything
off it is unstable and hard to hover, as the bare aircraft is. Trim with the
switches the session starts with (`TrimCondition::afcs`), and with its engine
controls (`TrimCondition::engines`, `rotor_speed_reference`).

## 8. Environment: `eyas::Environment`

| Field | Units | Meaning |
|---|---|---|
| `isa_delta_T` | K | Offset from the ISA temperature profile. Eyas evaluates the atmosphere itself at the aircraft's altitude in every step. |
| `wind_n` | m/s, NED | Constant wind: the air mass's velocity. A host with turbulence or gusts can vary it frame to frame. |
| `ground_height` | m MSL | A level ground plane under the aircraft, for the landing gear and ground effect. NaN: no ground. A host with terrain sets it from its terrain query under the aircraft each frame. |

An invalid environment (non-finite, or ΔT ≤ −216.65 K) is rejected and the
frame does nothing (`rejected_env`). After a trim, hold the trim's environment
(the condition's ΔT, wind and ground height), so the aircraft flies on from
trim without a jump.

## 9. Outputs: `eyas::AircraftState`

All SI units and radians. Frame suffixes follow Padfield: `_n` NED, `_b` body
forward-right-down at the airframe CG. Evaluated once per `advance()`, at the
end of the frame.

| Tier | Fields | Use |
|---|---|---|
| 1 · Pose | `sim_time_ns`, `sim_time`, `latitude`, `longitude`, `altitude`, `q_nb`, `roll`, `pitch`, `heading` | Visual system, map. Heading is true; the Euler angles are Padfield's yaw–pitch–roll order. |
| 2 · Inertial | `pos_n`, `vel_n`, `vel_b`, `omega_b`, `accel_b`, `specific_force_b`, `load_factor`, `ground_speed`, `track` | Motion platform (`specific_force_b` is what an accelerometer at the CG reads), navigation. `track` is noise at very low ground speed: blank it below the display's own threshold. |
| 3 · Air data | `true_airspeed`, `equivalent_airspeed`, `calibrated_airspeed`, `indicated_airspeed`, `angle_of_attack`, `sideslip`, `pressure_altitude`, `vertical_speed`, `outside_air_temperature` | Cockpit instruments. An ideal pitot-static system: indicated equals calibrated (no position-error data). |
| 4 · Rotor | `rotor_speed_percent`, `main_rotor_thrust`, `main_rotor_torque`, `main_rotor_power`, `coning`, `flap_1c`, `flap_1s`, `main_rotor_azimuth`, `tail_rotor_thrust`, `tail_rotor_azimuth`, `weight_on_wheels` | Rotor tachometer, torque, rotor visuals, sound. |
| 4 · Drive | `tail_rotor_torque`, `tail_rotor_power`, `engines[k]`: `gas_generator_speed_percent` (NG), `power_turbine_speed_percent` (NP), `torque`, `power`, `fuel_flow`, `turbine_gas_temperature`, `out` | Engine instruments. Torque is in N·m; a percent-torque gauge waits for a sourced 100 % reference. |
| Controls | `stabilator_incidence`, `main_rotor_theta0`, `main_rotor_theta1s`, `main_rotor_theta1c`, `tail_rotor_theta0` | Stabilator position indicator, control-position displays. |
| 5 · Diagnostics | `last_advance`, `quat_norm_error` | Health monitoring. |

A field for a part the aircraft doesn't have, or that its model doesn't
compute, is NaN: display code should treat NaN as "not available".

**Placing the aircraft.** Position is on a flat-earth map tangent to the
WGS-84 ellipsoid at a geodetic origin. The origin is scenario data, so the host
supplies it (its airfield, say) with `from_file(path, origin)`, in radians and
metres MSL, with |latitude| ≤ 89°. The aircraft starts at the NED origin;
`reset(body)` with `pos_n` moves it. The flat earth is accurate over a training
sortie's distances near the origin; reinitialize at a new origin for a long
cross-country.

## 10. Status and health

`advance()` never throws. Its `AdvanceStatus` (also in
`outputs().last_advance`) says what happened:

| Flag | Meaning | Suggested host action |
|---|---|---|
| `non_finite` | A step produced a non-finite state; the model kept the last good state and dropped the rest of the frame. The same inputs fail the same way again. | Fault: freeze and show the instructor; restore a snapshot or re-trim. |
| `rejected_dt` | A negative frame time; nothing was done. | Fault: a host bug. |
| `rejected_env` | An invalid environment; nothing was done. | Fault: a host bug. |
| `capped` | The frame was longer than `max_substeps` steps; the extra time was dropped. | Warn and log: a host hitch. |
| `atmosphere_out_of_range` | The aircraft left −5 to 20 km; the atmosphere was extended. | Warn and log. |
| `shaft_on_floor` | The rotor has all but stopped in flight (5 % of governed speed): a crash. | Treat as a crash, as a ground impact. |

`substeps` is the number of physics steps the frame ran, useful for checking
the host's frame period.

Init-time errors (a bad config, a missing table, an invalid trim condition)
throw `eyas::InitError`, a `std::runtime_error` whose message names the file
and key. A trim that doesn't converge returns `converged = false` and leaves
the model unchanged; it doesn't throw.

## 11. Aircraft configs

Aircraft are YAML files with CSV tables, loaded at start-up and converted to
SI. Two UH-60A configs are ready to fly:

| Config | Engines | Notes |
|---|---|---|
| `data/uh60a/uh60a.yaml` | GenHel's T700 model | Faster. No power levers or fuel shutoffs; the rotor speed is governed. |
| `data/uh60a/engine_ballin/uh60a.yaml` | Ballin's component-level T700s and fuel control, one per engine | For training that involves the engines: power levers, failures and relights, autorotation with the clutches overrunning, the tail rotor's power draw. About 1.7 times the cost per step. |

Tables are referenced relative to the YAML file, some from the shared
`data/uh60a/` directory, so ship the whole tree. The files are generated by
`tools/build_uh60a_config.py`; don't edit them by hand.

**Placeholders.** A few values have no public source yet and are tagged
`!placeholder`: the landing gear's stop springs and dampers and, on Ballin's
engines, the tail rotor's sense of rotation (for its torque) and the power
lever's idle angle.
`Model::placeholders()` lists them. The model flies with them; the host should
log the list so it is visible in every session.

## 12. Known limits

- **Trim is a straight-flight solver:** level flight, climbs and descents,
  sideslip, wind, hover in or out of ground effect. Some steep climbs and
  descents don't converge from a cold start; trimming from a converged nearby
  condition (`trim(condition, start)`) usually does. Steady turns and trims on
  the ground aren't supported: start on the ground with `reset()` and let the
  aircraft settle on its gear.
- **Engine start isn't modeled.** The engines start running; a relight after a
  fuel shutoff is immediate.
- **The ground is a level plane** at `ground_height`, with no slope or surface
  type.
- **Rotor:** rigid blades with flap only; blade lag, a higher-order inflow,
  dynamic stall and turbulence are still to come (M9). Hover roll from
  longitudinal cyclic is larger than in flight test (issue #100).
- `Model::advance` doesn't clamp controls outside their travel (issue #94):
  keep the host's inputs in range.
- Timing hasn't been measured on the target server yet, and the piloted
  evaluation in the host is still to come (M8e).

Validation against flight test and published simulation results is in
[VALIDATION.md](VALIDATION.md); the decisions and their reasons are in
[DECISIONS.md](DECISIONS.md).

## 13. Checklist

1. Link `eyas::eyas` with GCC 11 on RHEL 9, and pin the compiler for the build.
2. Write an adapter on the host's flight-model interface; include only
   `include/eyas/` headers.
3. Load the aircraft with the host's geodetic origin; log `placeholders()`.
4. Trim with the session's AFCS switches and engine controls; check
   `converged`.
5. Drive the cockpit controls to `TrimResult::controls` before the first frame.
6. Check the pedal sign (Eyas: + right) and every unit at the adapter boundary.
7. Hold the trim's environment, then feed wind and ground height from the host.
8. Call `advance()` once per frame with the nominal integer frame period.
9. Latch the FPS turn switch on the host side.
10. Map `AdvanceStatus` to the host's health and crash handling.
11. Allocate the snapshot buffer at start-up, for freeze, rewind and recall.
12. Run `eyas_bench` on the target hardware under the host's load.
