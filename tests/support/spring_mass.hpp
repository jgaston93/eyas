#pragma once

// Test component shared by the coupled-solve tests and the M1 validation
// cases: a point mass on a spring, sliding on a rail fixed to the airframe.
// It has known, nonzero G and H (spec §5, "Coupling").

#include <cstddef>
#include <span>
#include <utility>

#include <Eigen/Dense>

#include "core/component.hpp"
#include "core/rigid_body.hpp"

namespace eyas::test {

inline Mat3 skew(const Vec3& r) {
    Mat3 m;
    m << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
    return m;
}

struct Rail {
    double mass;  // [kg]   point mass m_p (not part of the airframe M_rb, D-002)
    double k;     // [N/m]  spring stiffness
    double c;     // [N·s/m] damping
    Vec3 r0_b;    // [m]    rail origin from the airframe CG, body axes
    Vec3 e_b;     // [-]    unit vector along the rail, body axes
};

// Common kinematics: the particle sits at r = r0 + s e and its inertial
// acceleration, in body axes, is
//   a_p = v̇ + ω̇ × r + s̈ e + b,   b = ω × v + ω × (ω × r) + 2 ṡ ω × e
// (acceleration of a point moving in a rotating frame; Padfield 3rd ed.
// App. 3A, Eqs. 3A.12–3A.17, plus the relative-motion terms).
inline Vec3 bias_accel(const Rail& rail, const BodyState& b, double s, double sd) {
    const Vec3 r = rail.r0_b + s * rail.e_b;
    const Vec3& w = b.omega_b;
    return w.cross(b.vel_b) + w.cross(w.cross(r)) + 2.0 * sd * w.cross(rail.e_b);
}

// Production-style component: reports s̈ and the load on the airframe split
// as value at a = 0 plus sensitivity to a = [v̇; ω̇].
class SpringMassOnRail : public DynamicComponent {
public:
    SpringMassOnRail(Rail rail, double s0, double sd0)
        : rail_(std::move(rail)), s0_(s0), sd0_(sd0) {}

    std::size_t num_states() const override { return 2; }
    void initial_states(std::span<double> x) const override {
        x[0] = s0_;
        x[1] = sd0_;
    }

    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override {
        const double s = x[0];
        const double sd = x[1];
        const Vec3& e = rail_.e_b;
        const Vec3 r = rail_.r0_b + s * e;
        const Vec3 g_b = ctx.body.q_nb.conjugate() * gravity_n();
        const Vec3 b = bias_accel(rail_, ctx.body, s, sd);
        const double f = -rail_.k * s - rail_.c * sd;

        // Rail direction: m_p e·a_p = f + m_p e·g_b, and e·(ω̇ × r) = ω̇·(r × e).
        Eigen::Matrix<double, 1, 6> G_sdd;
        G_sdd << -e.transpose(), -r.cross(e).transpose();
        const double sdd0 = f / rail_.mass + e.dot(g_b) - e.dot(b);
        out.xdot0[0] = sd;
        out.xdot0[1] = sdd0;
        for (std::size_t j = 0; j < 6; ++j) {
            out.G[6 + j] = G_sdd(0, static_cast<Eigen::Index>(j));
        }

        // a_p = J a + a_p0. The airframe carries m_p (g_b − a_p) at r,
        // including the particle's own weight (D-002).
        Eigen::Matrix<double, 3, 6> J;
        J << Mat3::Identity(), -skew(r);
        J += e * G_sdd;
        const Vec3 a_p0 = sdd0 * e + b;
        out.load0_b.F = rail_.mass * (g_b - a_p0);
        out.load0_b.M = r.cross(out.load0_b.F);
        out.H.topRows<3>() = -rail_.mass * J;
        out.H.bottomRows<3>() = -rail_.mass * skew(r) * J;
    }

private:
    Rail rail_;
    double s0_;
    double sd0_;
};

}  // namespace eyas::test
