#pragma once

#include <array>
#include <limits>
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
    // The engine controls during the trim (D-055), as Controls'; copied into
    // TrimResult::controls. A free shaft trims at the reference's speed.
    std::array<EngineControls, kMaxEngines> engines{};
    double rotor_speed_reference = 1.0;  // [-]
    // [m] MSL height of the ground plane, as Environment::ground_height; NaN:
    // no ground. Near it the main rotor is in ground effect (D-050). Finite
    // or NaN; with a ground the flight must be at constant height (no climb,
    // descent or vertical wind), and a trim that leaves a wheel loaded at any
    // point of the revolution doesn't converge.
    double ground_height = std::numeric_limits<double>::quiet_NaN();
};

// The trimmed state. Hold the same environment (the condition's ground
// height, ΔT and wind) in advance() to fly on from it.
struct TrimResult {
    bool converged = false;  // residuals within tolerance, start state finite
    int iterations = 0;      // Newton iterations used
    Controls controls;       // the trimmed pilot inputs (brakes released)
    double pitch = 0.0;      // θ [rad]
    double roll = 0.0;       // φ [rad]
    // [s] The held revolution's period: one turn of the rotor, 2π/Ω at its
    // trimmed speed, which a free shaft's governor may hold a little off
    // its governed speed (D-053).
    double period = 0.0;
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
    // The fastest any state the trim holds at its start value would move,
    // as its mean rate over the held revolution [that state's units per
    // second] (D-045: GenHel's IC mode holds the flight control system's
    // integrators and references through the trim). With the body held a
    // reference's input is constant, so in practice this is an FPS
    // integrator's rate [fraction of authority per second]. Not part of
    // `converged`: nonzero means the flight control system would still
    // retrim after release, e.g. an FPS integrating a lateral acceleration
    // that the trim's sideslip leaves.
    double held_drift_rate = 0.0;
    // Every state after the body's at the start of the held revolution, in
    // the Model's state order: each component's, then each rotor shaft's
    // azimuth and speed (ψ = 0 [rad], Ω [rad/s]; D-052). Engineering data:
    // pass the result back to Model::trim as a starting point for a nearby
    // condition; there a shaft held at its governed speed (an ideal drive)
    // starts at that speed and ψ = 0 whatever these say, and a free one at
    // the Ω given here.
    // The state Model::trim installs differs in the flight control system's
    // sensors and servos: there they start on their response to the free
    // body's 4/rev vibration, as v and ω do (D-045 amendment).
    std::vector<double> component_states;
};

}  // namespace eyas
