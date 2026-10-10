#pragma once

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

// Atmosphere and wind. Populated from M2 (ISA); until then the scalars are NaN
// so any accidental use shows up in results.
struct Environment {
    double rho = std::numeric_limits<double>::quiet_NaN();             // [kg/m^3]
    double temperature = std::numeric_limits<double>::quiet_NaN();     // [K]
    double speed_of_sound = std::numeric_limits<double>::quiet_NaN();  // [m/s]
    Vec3 wind_n = Vec3::Zero();                                        // [m/s]
    double ground_height = std::numeric_limits<double>::quiet_NaN();   // [m]
};

// Rigid-body state of the airframe.
struct BodyState {
    Vec3 pos_n = Vec3::Zero();     // [m]     CG position, NED
    Vec3 vel_b = Vec3::Zero();     // [m/s]   CG inertial velocity, body axes
    Quat q_nb = Quat::Identity();  // [-]     attitude: maps body vectors to NED
    Vec3 omega_b = Vec3::Zero();   // [rad/s] body angular rate, body axes
};

}  // namespace eyas
