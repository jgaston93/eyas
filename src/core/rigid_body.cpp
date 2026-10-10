#include "core/rigid_body.hpp"

namespace eyas {

MassProperties make_mass_properties(double mass, double Ixx, double Iyy, double Izz, double Ixz) {
    MassProperties mp;
    mp.mass = mass;
    mp.inertia_b << Ixx, 0.0, -Ixz, 0.0, Iyy, 0.0, -Ixz, 0.0, Izz;
    return mp;
}

BodyState unpack_body(std::span<const double> x) noexcept {
    using namespace body_layout;
    BodyState b;
    b.pos_n = Vec3(x[kPos], x[kPos + 1], x[kPos + 2]);
    b.vel_b = Vec3(x[kVel], x[kVel + 1], x[kVel + 2]);
    b.q_nb = Quat(x[kQuat], x[kQuat + 1], x[kQuat + 2], x[kQuat + 3]);
    b.omega_b = Vec3(x[kOmega], x[kOmega + 1], x[kOmega + 2]);
    return b;
}

void pack_body(const BodyState& b, std::span<double> x) noexcept {
    using namespace body_layout;
    for (Eigen::Index i = 0; i < 3; ++i) {
        const auto k = static_cast<std::size_t>(i);
        x[kPos + k] = b.pos_n(i);
        x[kVel + k] = b.vel_b(i);
        x[kOmega + k] = b.omega_b(i);
    }
    x[kQuat] = b.q_nb.w();
    x[kQuat + 1] = b.q_nb.x();
    x[kQuat + 2] = b.q_nb.y();
    x[kQuat + 3] = b.q_nb.z();
}

Mat66 rigid_body_mass_matrix(const MassProperties& mp) noexcept {
    Mat66 m = Mat66::Zero();
    m.topLeftCorner<3, 3>() = mp.mass * Mat3::Identity();
    m.bottomRightCorner<3, 3>() = mp.inertia_b;
    return m;
}

Vec6 rigid_body_bias(const MassProperties& mp, const BodyState& b) noexcept {
    // Rotation matrices need a unit quaternion; inside RK4 stages the stored
    // one is slightly off unit.
    const Quat q = b.q_nb.normalized();
    const Vec3 g_b = q.conjugate() * gravity_n();
    Vec6 rhs;
    rhs.head<3>() = mp.mass * g_b - mp.mass * b.omega_b.cross(b.vel_b);  // Padfield Eq. 3A.24
    rhs.tail<3>() = -b.omega_b.cross(mp.inertia_b * b.omega_b);          // Padfield Eq. 3A.33
    return rhs;
}

void body_kinematics(const BodyState& b, const Vec6& a, std::span<double> xdot) noexcept {
    using namespace body_layout;
    const Vec3 pdot_n = b.q_nb.normalized() * b.vel_b;
    const Quat w(0.0, b.omega_b.x(), b.omega_b.y(), b.omega_b.z());
    const Quat qd = b.q_nb * w;  // Hamilton product; ½ applied below
    for (Eigen::Index i = 0; i < 3; ++i) {
        const auto k = static_cast<std::size_t>(i);
        xdot[kPos + k] = pdot_n(i);
        xdot[kVel + k] = a(i);
        xdot[kOmega + k] = a(3 + i);
    }
    xdot[kQuat] = 0.5 * qd.w();
    xdot[kQuat + 1] = 0.5 * qd.x();
    xdot[kQuat + 2] = 0.5 * qd.y();
    xdot[kQuat + 3] = 0.5 * qd.z();
}

}  // namespace eyas
