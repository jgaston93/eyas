// M1 acceptance test: the coupled body/component solve (spec §5 "Coupling",
// §9 M1). A point mass on a spring, sliding on a rail fixed to the airframe,
// is a DynamicComponent with known G and H. The executive's Schur-complement
// solve must give the same accelerations as the combined body + particle
// system solved directly.

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "support/spring_mass.hpp"

namespace eyas {
namespace {

using test::Rail;
using test::SpringMassOnRail;

// Independent oracle: Newton–Euler for the whole system with unknowns
// z = [v̇; ω̇; s̈], solved as one 7×7 system. It shares no kinematics code
// with SpringMassOnRail, so an error there can't cancel out.
//   total force:       M (v̇ + ω × v) + m_p a_p = (M + m_p) g_b
//   moment about CG:   I ω̇ + ω × I ω + m_p r × a_p = m_p r × g_b
//   along the rail:    m_p e·a_p = f + m_p e·g_b
Eigen::Matrix<double, 7, 1> oracle(const MassProperties& mp, const Rail& rail, const BodyState& b,
                                   double s, double sd) {
    const double M = mp.mass;
    const double m = rail.mass;
    const Mat3& I = mp.inertia_b;
    const Vec3& e = rail.e_b;
    const Vec3& v = b.vel_b;
    const Vec3& w = b.omega_b;
    const Vec3 r = rail.r0_b + s * e;
    const Vec3 g_b = b.q_nb.toRotationMatrix().transpose() * gravity_n();
    // Acceleration of a point moving along a rail in the rotating body frame:
    // a_p = v̇ + ω̇ × r + s̈ e + ω × v + ω × (ω × r) + 2 ṡ ω × e
    const Vec3 bias = w.cross(v) + w.cross(w.cross(r)) + 2.0 * sd * w.cross(e);
    const double f = -rail.k * s - rail.c * sd;
    Mat3 r_x;  // r_x u = r × u
    r_x << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;

    Eigen::Matrix<double, 3, 7> Ap;
    Ap << Mat3::Identity(), -r_x, e;

    Eigen::Matrix<double, 7, 7> A = Eigen::Matrix<double, 7, 7>::Zero();
    Eigen::Matrix<double, 7, 1> rhs;
    A.topRows<3>() = m * Ap;
    A.block<3, 3>(0, 0) += M * Mat3::Identity();
    rhs.head<3>() = (M + m) * g_b - M * w.cross(v) - m * bias;

    A.middleRows<3>(3) = m * r_x * Ap;
    A.block<3, 3>(3, 3) += I;
    rhs.segment<3>(3) = m * r.cross(g_b) - w.cross(I * w) - m * r.cross(bias);

    A.row(6) = m * e.transpose() * Ap;
    rhs(6) = f + m * e.dot(g_b) - m * e.dot(bias);
    return A.partialPivLu().solve(rhs);
}

TEST(CoupledSolve, MatchesDirectSolutionOfCombinedSystem) {
    const MassProperties mp = make_mass_properties(900.0, 700.0, 2100.0, 1800.0, 150.0);
    const Rail rail{120.0, 4.0e4, 300.0, Vec3(1.5, -0.4, -1.2), Vec3(0.3, 0.2, 1.0).normalized()};
    const double s = 0.07;
    const double sd = -0.4;

    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<SpringMassOnRail>(rail, s, sd));
    ExecutiveConfig cfg;
    cfg.mass = mp;
    Executive ex(cfg, std::move(comps));

    BodyState b;
    b.vel_b = Vec3(25.0, -3.0, 4.0);
    b.omega_b = Vec3(0.4, -0.3, 0.6);
    b.q_nb = Quat(Eigen::AngleAxisd(0.5, Vec3(1.0, 2.0, -1.0).normalized()));
    ex.set_body_state(b);

    std::vector<double> xdot(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, xdot));
    const auto z = oracle(mp, rail, b, s, sd);

    const std::size_t k = body_layout::kSize;  // first component state
    for (std::size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(xdot[body_layout::kVel + i], z(static_cast<Eigen::Index>(i)),
                    1e-12 * z.head<3>().norm())
            << "v̇[" << i << "]";
        EXPECT_NEAR(xdot[body_layout::kOmega + i], z(static_cast<Eigen::Index>(3 + i)),
                    1e-12 * z.segment<3>(3).norm())
            << "ω̇[" << i << "]";
    }
    EXPECT_EQ(xdot[k], sd);
    EXPECT_NEAR(xdot[k + 1], z(6), 1e-12 * std::abs(z(6)));
}

// Inside RK4 stages 2–4 the stored quaternion is off unit length. The state
// it represents is the same attitude, so every derivative except q̇ must
// match the unit-quaternion result, and q̇ = ½ q ⊗ [0, ω] scales with q.
// Components rotate vectors with ctx.body.q_nb, so the executive must hand
// them a unit quaternion.
TEST(CoupledSolve, OffUnitQuaternionGivesSameAccelerations) {
    const MassProperties mp = make_mass_properties(900.0, 700.0, 2100.0, 1800.0, 150.0);
    const Rail rail{120.0, 4.0e4, 300.0, Vec3(1.5, -0.4, -1.2), Vec3(0.3, 0.2, 1.0).normalized()};

    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<SpringMassOnRail>(rail, 0.07, -0.4));
    ExecutiveConfig cfg;
    cfg.mass = mp;
    Executive ex(cfg, std::move(comps));

    BodyState b;
    b.vel_b = Vec3(25.0, -3.0, 4.0);
    b.omega_b = Vec3(0.4, -0.3, 0.6);
    b.q_nb = Quat(Eigen::AngleAxisd(0.5, Vec3(1.0, 2.0, -1.0).normalized()));
    ex.set_body_state(b);

    const std::vector<double> x_unit(ex.state().begin(), ex.state().end());
    std::vector<double> x_off = x_unit;
    const double scale = 1.01;  // far larger than any RK4 stage drift, to make errors obvious
    for (std::size_t i = 0; i < 4; ++i) {
        x_off[body_layout::kQuat + i] *= scale;
    }

    std::vector<double> xd_unit(ex.num_states());
    std::vector<double> xd_off(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, x_unit, Controls{}, Environment{}, xd_unit));
    EXPECT_TRUE(ex.derivatives(0.0, x_off, Controls{}, Environment{}, xd_off));

    for (std::size_t i = 0; i < ex.num_states(); ++i) {
        const bool is_quat = i >= body_layout::kQuat && i < body_layout::kQuat + 4;
        const double expected = is_quat ? scale * xd_unit[i] : xd_unit[i];
        EXPECT_NEAR(xd_off[i], expected, 1e-12 * (1.0 + std::abs(expected))) << "state " << i;
    }
}

// Closed-form anchor: rail through the CG, no rotation, no damping. Gravity
// accelerates body and particle alike, so the relative motion is a two-mass
// oscillator, s(t) = s0 cos(ω_n t), ω_n = sqrt(k/μ), μ = M m_p / (M + m_p).
// RK4's phase error over T is about T ω_n⁵ h⁴ / 120 ≈ 8e-9 rad here, so 1e-7
// of s0 is a meaningful bound.
TEST(CoupledSolve, TwoMassOscillatorFrequency) {
    const double M = 900.0;
    const double m = 100.0;
    const double mu = M * m / (M + m);
    const double wn = 10.0;  // [rad/s]
    const Rail rail{m, wn * wn * mu, 0.0, Vec3::Zero(), Vec3::UnitX()};
    const double s0 = 0.05;

    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<SpringMassOnRail>(rail, s0, 0.0));
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(M, 700.0, 2100.0, 1800.0, 0.0);
    Executive ex(cfg, std::move(comps));
    ex.set_body_state(BodyState{});

    const double T = 10.0;
    for (int i = 0; i < 10'000; ++i) {
        (void)ex.advance(Controls{}, Environment{}, 1'000'000);
    }
    const double s = ex.state()[body_layout::kSize];
    EXPECT_NEAR(s, s0 * std::cos(wn * T), 1e-7 * s0);
    EXPECT_LT(ex.body_state().omega_b.norm(), 1e-12);  // round-off only
}

}  // namespace
}  // namespace eyas
