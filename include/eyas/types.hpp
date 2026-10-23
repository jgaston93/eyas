#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Value types of the Eyas library API (D-020). Frame suffixes follow spec §4:
// _n NED inertial, _b body FRD at the airframe CG.
namespace eyas {

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

// The pilot's switches of an automatic flight control system (D-045), all off
// by default. A config without one ignores them. For the UH-60A (GenHel
// CR-166309 §5.5): the SAS has a digital and an analog channel per axis; with
// both on each works at gain 1, with one on that one works at gain 2. The
// pitch bias actuator (PBA) drives back to centre at its rate limit when
// switched off.
//
// The flight path stabilization (FPS) holds the pitch and roll attitude, the
// airspeed above 30 kt and the heading, through trim actuators of 100 %
// authority. Switched off, its references follow the aircraft and its trim
// actuators drive back to centre at their rate limit. The turn switch hands
// heading hold over to turn coordination; the host latches it (GenHel p.
// 5.5-3: on with the FPS on, above 60 kt, the trim release pressed and more
// than 2° of bank; off with the feet on the pedals and the trim release
// pressed). While the trim release is held with the FPS on, its references
// follow the aircraft, its integrators stop and its trim actuators stay
// where they are, so the pilot can fly to a new attitude and let go there.
struct AfcsSwitches {
    bool sas_digital = false;   // GenHel DSASSW
    bool sas_analog = false;    // GenHel ASASSW
    bool pitch_bias = false;    // pitch bias actuator
    bool fps = false;           // GenHel FPSSW
    bool fps_turn = false;      // GenHel TURNSW (NTRNSW is its inverse)
    bool trim_release = false;  // momentary, held while pressed

    // Every switch alike (so a switch added later is compared too).
    friend bool operator==(const AfcsSwitches&, const AfcsSwitches&) = default;
};

// At most this many engines per aircraft (D-055): Controls::engines' size.
inline constexpr std::size_t kMaxEngines = 4;

// One engine's cockpit controls (D-055). The aircraft's engines take
// Controls::engines' entries in config order; entries past them are unused.
struct EngineControls {
    // Power control lever: 0 = the idle stop, 1 = the fly stop. Clamped to
    // [0, 1]; NaN reads as fly. Engines without levers (GenHel's) ignore it.
    double power_lever = 1.0;  // [0, 1]

    friend bool operator==(const EngineControls&, const EngineControls&) = default;
};

// Normalized pilot inputs (sign conventions per D-024).
struct Controls {
    double collective = 0.0;  // [0, 1]  0 = full down, 1 = full up
    double lat_cyclic = 0.0;  // [-1, 1] +1 = full right stick
    double lon_cyclic = 0.0;  // [-1, 1] +1 = full forward stick
    double pedal = 0.0;       // [-1, 1] +1 = full right pedal
    // Wheel brakes, per side (D-032): 0 = released, 1 = full (GenHel's "set").
    // Values outside [0, 1] are clamped; NaN counts as released.
    double brake_left = 0.0;   // [0, 1]
    double brake_right = 0.0;  // [0, 1]
    AfcsSwitches afcs;         // switch positions, not inputs that trim solves for
    // Engine controls (D-055), not inputs that trim solves for.
    std::array<EngineControls, kMaxEngines> engines{};
    // The engines' power-turbine speed reference (the cockpit's engine-speed
    // trim) as a fraction of the drive's governed speed; a value that isn't
    // positive and finite reads as 1. The governors hold the rotor there.
    double rotor_speed_reference = 1.0;  // [-]
};

// Atmospheric conditions set by the host (D-016). Eyas evaluates the
// atmosphere itself at the current altitude in every derivative evaluation
// (SimContext::air), so density follows the aircraft within a host frame.
struct Environment {
    double isa_delta_T = 0.0;    // [K] offset from the ISA temperature profile (D-015)
    Vec3 wind_n = Vec3::Zero();  // [m/s] constant wind, NED
    double ground_height = std::numeric_limits<double>::quiet_NaN();  // [m] above MSL (M6)
};

// Result of one advance() call. Additive only: fields never change meaning.
struct AdvanceStatus {
    std::int64_t substeps = 0;  // physics steps run by this call
    bool capped = false;        // hit max_substeps; the excess time was dropped
    bool rejected_dt = false;   // dt_host_ns < 0; nothing was done
    // A sub-step produced a non-finite state (e.g. a singular coupled solve).
    // That step was discarded, the state is the last good one, and the rest of
    // this call's time was dropped. The same inputs fail the same way again
    // (D-014).
    bool non_finite = false;
    // The Environment was invalid (D-019: non-finite, or ΔT ≤ −216.65 K);
    // nothing was done.
    bool rejected_env = false;
    // After some sub-step of this call the CG was outside the tabulated
    // atmosphere, H outside [−5, 20] km' (D-015, D-019); the profile was extended.
    bool atmosphere_out_of_range = false;
};

// The geodetic origin of the NED frame (D-017, D-044): the point the flat-earth
// map is tangent to. Its altitude is the NED origin's height above MSL.
struct GeodeticOrigin {
    double latitude = 0.0;   // [rad] WGS-84 geodetic
    double longitude = 0.0;  // [rad]
    double altitude = 0.0;   // [m] above MSL
};

// Rigid-body state of the airframe.
struct BodyState {
    Vec3 pos_n = Vec3::Zero();     // [m]     CG position, NED
    Vec3 vel_b = Vec3::Zero();     // [m/s]   CG inertial velocity, body axes
    Quat q_nb = Quat::Identity();  // [-]     attitude: maps body vectors to NED
    Vec3 omega_b = Vec3::Zero();   // [rad/s] body angular rate, body axes
};

}  // namespace eyas
