#pragma once

#include <vector>

#include "eyas/types.hpp"

// Trim (spec §6.8, D-035): the controls and attitude that hold the aircraft
// in steady straight flight. Init-time: Model::trim allocates and is not for
// a real-time loop. Frame suffixes follow spec §4.
namespace eyas {

// The flight condition to trim for. Speed, flight path and sideslip are
// relative to the air mass; the ground velocity adds `wind_n`.
struct TrimCondition {
    double airspeed = 0.0;           // V   [m/s] true airspeed, ≥ 0
    double flight_path_angle = 0.0;  // γ   [rad] climb angle relative to the air, + up
    double sideslip = 0.0;           // β   [rad] asin(v_air_b,y / V), + wind from the right;
                                     //     0 in vertical flight, where it follows from the attitude
    double heading = 0.0;            // ψ_h [rad] from north
    double altitude = 0.0;           // [m] MSL height of the CG, held during the trim
    double isa_delta_T = 0.0;        // [K] as Environment::isa_delta_T (D-015)
    Vec3 wind_n = Vec3::Zero();      // [m/s] constant wind, NED
    // The flight control system's switches during the trim; copied into
    // TrimResult::controls (D-045).
    AfcsSwitches afcs;
};

// The trimmed state. Hold the same environment (no ground, the condition's
// ΔT and wind) in advance() to fly on from it.
struct TrimResult {
    bool converged = false;  // residuals within tolerance, start state finite
    int iterations = 0;      // Newton iterations used
    Controls controls;       // the trimmed pilot inputs (brakes released)
    double pitch = 0.0;      // θ [rad]
    double roll = 0.0;       // φ [rad]
    // [m/s] CG inertial velocity, body axes: its mean over the revolution,
    // as the condition asks. The state Model::trim installs adds the body's
    // zero-mean 4/rev vibration to it and to ω = 0 (their values at t = 0),
    // so that the free body starts on its vibration cycle (D-035).
    Vec3 vel_b = Vec3::Zero();
    // Residuals at the solution: the net force and moment on the held body
    // averaged over one rotor revolution [N], [N·m] (body axes, about the
    // airframe CG), and the largest change of a rotor, inflow or other
    // component state over the revolution (in that state's own units),
    // including the flight control states' free-flight orbit (below). NaN
    // if the solver stopped at a point with no valid residual.
    double force_residual = 0.0;
    double moment_residual = 0.0;
    double periodic_residual = 0.0;
    // Every component state at the start of the held revolution, in the
    // Model's state order after the body states. Engineering data: pass the
    // result back to Model::trim as a starting point for a nearby condition.
    // The state Model::trim installs differs in the flight control system's
    // sensors and servos: there they start on their response to the free
    // body's 4/rev vibration, as v and ω do (D-045 amendment).
    std::vector<double> component_states;
};

}  // namespace eyas
