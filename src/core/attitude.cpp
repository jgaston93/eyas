#include "core/attitude.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace eyas {

EulerAngles euler_angles(const Quat& q_nb) noexcept {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    const Mat3 G = q_nb.toRotationMatrix().transpose();  // C_bn = Γ
    EulerAngles e;
    e.pitch = -std::asin(std::clamp(G(0, 2), -1.0, 1.0));  // clamp: round-off near ±90°
    e.roll = std::atan2(G(1, 2), G(2, 2));
    if (e.roll <= -std::numbers::pi) {  // atan2(−0.0, x < 0) is −π; the range is (−π, π]
        e.roll = std::numbers::pi;
    }
    double h = std::atan2(G(0, 1), G(0, 0));
    if (h < 0.0) {
        h += two_pi;
    }
    if (h >= two_pi) {  // a tiny negative angle rounds up to 2π
        h -= two_pi;
    }
    e.heading = h;
    return e;
}

Quat quaternion(const EulerAngles& e) noexcept {
    const Quat q = Quat(Eigen::AngleAxisd(e.heading, Vec3::UnitZ())) *
                   Quat(Eigen::AngleAxisd(e.pitch, Vec3::UnitY())) *
                   Quat(Eigen::AngleAxisd(e.roll, Vec3::UnitX()));
    return q.normalized();
}

}  // namespace eyas
