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

// Rotor-speed degrees of freedom (D-052): at most kMaxShafts shafts, so the
// coupled solve and every component's shaft terms have fixed sizes and
// advance() allocates nothing (spec §7). Raising it costs only stack space.
inline constexpr int kMaxShafts = 4;
using VecS = Eigen::Matrix<double, kMaxShafts, 1>;
using Mat6S = Eigen::Matrix<double, 6, kMaxShafts>;
using MatS6 = Eigen::Matrix<double, kMaxShafts, 6>;
using MatSS = Eigen::Matrix<double, kMaxShafts, kMaxShafts>;

// One shaft's state (D-052): its azimuth ψ [rad] (unbounded, ψ̇ = Ω) and its
// speed Ω [rad/s] relative to the airframe, about its own axis.
struct ShaftState {
    double psi = 0.0;
    double omega = 0.0;
};

// Force and moment in body axes, moment about the airframe CG (D-002).
struct Wrench {
    Vec3 F = Vec3::Zero();  // [N]
    Vec3 M = Vec3::Zero();  // [N·m]
};

}  // namespace eyas
