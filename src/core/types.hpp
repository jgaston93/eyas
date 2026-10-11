#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "eyas/types.hpp"

// Basic types shared across modules (spec §6). Frame suffixes follow §4:
// _n NED inertial, _b body FRD at the airframe CG.
namespace eyas {

// Vec3, Quat and the API value types come from eyas/types.hpp.
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat66 = Eigen::Matrix<double, 6, 6>;

// Force and moment in body axes, moment about the airframe CG (D-002).
struct Wrench {
    Vec3 F = Vec3::Zero();  // [N]
    Vec3 M = Vec3::Zero();  // [N·m]
};

}  // namespace eyas
