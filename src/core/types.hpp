#pragma once

#include <cstdint>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Basic types shared across modules (spec §6). Frame suffixes follow §4:
// _n NED inertial, _b body FRD at the airframe CG.
namespace eyas {

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat66 = Eigen::Matrix<double, 6, 6>;
using Quat = Eigen::Quaterniond;

// Force and moment in body axes, moment about the airframe CG (D-002).
struct Wrench {
    Vec3 F = Vec3::Zero();  // [N]
    Vec3 M = Vec3::Zero();  // [N·m]
};

// Normalized pilot inputs.
struct Controls {
    double collective = 0.0;  // [0, 1]
    double lat_cyclic = 0.0;  // [-1, 1]
    double lon_cyclic = 0.0;  // [-1, 1]
    double pedal = 0.0;       // [-1, 1]
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
};

// Rigid-body state of the airframe.
struct BodyState {
    Vec3 pos_n = Vec3::Zero();     // [m]     CG position, NED
    Vec3 vel_b = Vec3::Zero();     // [m/s]   CG inertial velocity, body axes
    Quat q_nb = Quat::Identity();  // [-]     attitude: maps body vectors to NED
    Vec3 omega_b = Vec3::Zero();   // [rad/s] body angular rate, body axes
};

}  // namespace eyas
