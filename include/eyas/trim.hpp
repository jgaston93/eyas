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
    double sideslip = 0.0;           // β   [rad] asin(v_air_b,y / V), + wind from the right
    double heading = 0.0;            // ψ_h [rad] from north
    double altitude = 0.0;           // [m] MSL height of the CG, held during the trim
    double isa_delta_T = 0.0;        // [K] as Environment::isa_delta_T (D-015)
    Vec3 wind_n = Vec3::Zero();      // [m/s] constant wind, NED
};

// The trimmed state. Hold the same environment (no ground, the condition's
// ΔT and wind) in advance() to fly on from it.
struct TrimResult {
    bool converged = false;     // all residuals below the solver's tolerances
    int iterations = 0;         // Newton iterations used
    Controls controls;          // the trimmed pilot inputs (brakes released)
    double pitch = 0.0;         // θ [rad]
    double roll = 0.0;          // φ [rad]
    Vec3 vel_b = Vec3::Zero();  // [m/s] CG inertial velocity, body axes
    // Residuals at the solution: the net force and moment on the held body
    // averaged over one rotor revolution [N], [N·m] (body axes, about the
    // airframe CG), and the largest change of a rotor, inflow or other
    // component state over the revolution (in that state's own units).
    double force_residual = 0.0;
    double moment_residual = 0.0;
    double periodic_residual = 0.0;
    // Every component state at the start of the revolution, in the Model's
    // state order after the body states. Engineering data: pass the result
    // back to Model::trim as a starting point for a nearby condition.
    std::vector<double> component_states;
};

}  // namespace eyas
