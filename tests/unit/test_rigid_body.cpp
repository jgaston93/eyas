// M1 acceptance tests for the 6-DOF rigid body (spec §9 M1, as amended by
// DECISIONS D-013). Everything runs through the production Executive.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

constexpr std::int64_t kStepNs = 1'000'000;  // 1 kHz
constexpr double kStep = 1e-3;               // [s]

// Runs `seconds` of simulated time in whole physics steps.
void run_for(Executive& ex, double seconds) {
    const auto steps = static_cast<std::int64_t>(std::llround(seconds / kStep));
    for (std::int64_t i = 0; i < steps; ++i) {
        (void)ex.advance(Controls{}, Environment{}, kStepNs);
    }
}

Executive make_executive(const MassProperties& mass,
                         std::vector<std::unique_ptr<ForceModel>> forces = {}) {
    ExecutiveConfig cfg;
    cfg.mass = mass;
    cfg.step_ns = kStepNs;
    return Executive(cfg, {}, std::move(forces));
}

double rel_err(const Vec3& got, const Vec3& want) {
    return (got - want).norm() / want.norm();
}

// A realistic, non-diagonal inertia tensor: the UH-60A airframe inertias from
// the GenHel motion module (DECISIONS D-002). The mass is an arbitrary test
// value, not aircraft data; none of these tests depend on it.
MassProperties helicopter_like_body() {
    const double s = units::kSlug * units::kFoot * units::kFoot;
    return make_mass_properties(1000.0, 4659.0 * s, 38512.0 * s, 36796.0 * s, 1882.0 * s);
}

Quat attitude(double roll, double pitch, double heading) {
    // Padfield 3rd ed. Eq. 3A.37: heading, then pitch, then roll.
    return Quat(Eigen::AngleAxisd(heading, Vec3::UnitZ()) *
                Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX()));
}

// ---- Inertia tensor and gravity against Padfield App. 3A ------------------

// The tensor built from Padfield's (Ixx, Iyy, Izz, Ixz) must reproduce his
// rotational equations, Padfield 3rd ed. Eq. 3A.33, through I ω̇ + ω × (I ω).
TEST(RigidBody, InertiaTensorMatchesPadfieldMomentEquations) {
    const double Ixx = 3.0, Iyy = 7.0, Izz = 5.0, Ixz = 0.8;
    const MassProperties mp = make_mass_properties(1.0, Ixx, Iyy, Izz, Ixz);
    const Vec3 w(0.3, -0.7, 1.1);
    const Vec3 wd(-0.2, 0.5, 0.9);
    const double p = w.x(), q = w.y(), r = w.z();
    const double pd = wd.x(), qd = wd.y(), rd = wd.z();
    const Vec3 padfield(Ixx * pd - (Iyy - Izz) * q * r - Ixz * (p * q + rd),
                        Iyy * qd - (Izz - Ixx) * p * r + Ixz * (p * p - r * r),
                        Izz * rd - (Ixx - Iyy) * p * q - Ixz * (pd - r * q));
    const Vec3 ours = mp.inertia_b * wd + w.cross(mp.inertia_b * w);
    EXPECT_LT((ours - padfield).norm(), 1e-14);
}

// Gravity in body axes, Padfield 3rd ed. Eq. 3A.43.
TEST(RigidBody, GravityInBodyAxesMatchesPadfield) {
    const double phi = 0.4, theta = -0.25, psi_h = 2.1;
    BodyState b;
    b.q_nb = attitude(phi, theta, psi_h);
    const Vec3 g_b = b.q_nb.conjugate() * gravity_n();
    const double g = units::kStandardGravity;
    const Vec3 padfield(-g * std::sin(theta), g * std::cos(theta) * std::sin(phi),
                        g * std::cos(theta) * std::cos(phi));
    EXPECT_LT((g_b - padfield).norm(), 1e-14);
}

// ---- Free fall --------------------------------------------------------------

// No applied loads: v_b(t) = v0 + C_bn g t and p_n(t) = p0 + C_nb v0 t + g t²/2.
// RK4 integrates these low-order polynomials exactly, so only round-off remains.
TEST(RigidBody, FreeFallMatchesAnalyticSolution) {
    Executive ex = make_executive(helicopter_like_body());
    BodyState b0;
    b0.pos_n = Vec3(100.0, -50.0, -1000.0);
    b0.vel_b = Vec3(30.0, 2.0, -5.0);
    b0.q_nb = attitude(0.1, -0.2, 0.7);
    b0.omega_b = Vec3::Zero();
    ex.set_body_state(b0);

    const double T = 10.0;
    run_for(ex, T);

    const BodyState b = ex.body_state();
    const Vec3 v_want = b0.vel_b + b0.q_nb.conjugate() * gravity_n() * T;
    const Vec3 p_want = b0.pos_n + b0.q_nb * b0.vel_b * T + 0.5 * gravity_n() * T * T;
    EXPECT_LT(rel_err(b.vel_b, v_want), 1e-9);
    EXPECT_LT(rel_err(b.pos_n, p_want), 1e-9);
    EXPECT_EQ(ex.sim_time_ns(), 10'000'000'000);
}

// ---- Torque-free spin ------------------------------------------------------

double kinetic_energy(const MassProperties& mp, const BodyState& b) {
    return 0.5 * b.omega_b.dot(mp.inertia_b * b.omega_b);
}

// Angular momentum in the inertial frame, so the check also covers attitude.
Vec3 angular_momentum_n(const MassProperties& mp, const BodyState& b) {
    return b.q_nb * (mp.inertia_b * b.omega_b);
}

// Rotational energy and inertial angular momentum are conserved, < 1e-8
// relative over 60 s (spec M1; D-013). Full tensor with Ixz, tumbling motion.
TEST(RigidBody, TorqueFreeSpinConservesEnergyAndMomentum) {
    const MassProperties mp = helicopter_like_body();
    Executive ex = make_executive(mp);
    BodyState b0;
    b0.q_nb = attitude(0.2, 0.1, -0.4);
    b0.omega_b = Vec3(0.6, -0.5, 0.8);  // [rad/s], not near a principal axis
    ex.set_body_state(b0);

    const double E0 = kinetic_energy(mp, b0);
    const Vec3 H0 = angular_momentum_n(mp, b0);
    double max_dE = 0.0;
    double max_dH = 0.0;
    for (int i = 0; i < 60; ++i) {
        run_for(ex, 1.0);
        const BodyState b = ex.body_state();
        max_dE = std::max(max_dE, std::abs(kinetic_energy(mp, b) - E0) / E0);
        max_dH = std::max(max_dH, rel_err(angular_momentum_n(mp, b), H0));
    }
    EXPECT_LT(max_dE, 1e-8);
    EXPECT_LT(max_dH, 1e-8);
}

// Spin Ω about the intermediate principal axis (I1 < I2 < I3). Linearizing
// Euler's equations (Padfield 3rd ed. Eq. 3A.33 with Ixz = 0) about
// ω = (0, Ω, 0) gives δω̈1 = λ² δω1, with λ = Ω √((I3 − I2)(I2 − I1)/(I1 I3)).
// Starting on the unstable eigenvector, ω1 = ε e^{λt} while the motion stays
// linear. Then the body flips: ω2 reverses sign (D-013).
TEST(RigidBody, IntermediateAxisSpinIsUnstableAtTheLinearizedRate) {
    const double I1 = 1.0, I2 = 2.0, I3 = 3.0, W = 1.0, eps = 1e-8;
    const MassProperties mp = make_mass_properties(1.0, I1, I2, I3, 0.0);
    Executive ex = make_executive(mp);
    const double lambda = W * std::sqrt((I3 - I2) * (I2 - I1) / (I1 * I3));
    const double alpha = (I3 - I2) * W / I1;  // δω̇1 = −α δω3
    BodyState b0;
    b0.omega_b = Vec3(eps, W, -lambda * eps / alpha);
    ex.set_body_state(b0);

    // Linear phase: amplitude grows from 1e-8 to ~3e-5 by t = 14 s, so
    // nonlinear terms are ~1e-9 relative.
    run_for(ex, 4.0);
    const double w1_a = ex.body_state().omega_b.x();
    run_for(ex, 10.0);
    const double w1_b = ex.body_state().omega_b.x();
    const double lambda_measured = std::log(w1_b / w1_a) / 10.0;
    EXPECT_NEAR(lambda_measured / lambda, 1.0, 1e-6);

    // Nonlinear phase: the flip.
    double min_w2 = ex.body_state().omega_b.y();
    for (int i = 0; i < 460; ++i) {
        run_for(ex, 0.1);
        min_w2 = std::min(min_w2, ex.body_state().omega_b.y());
    }
    EXPECT_LT(min_w2, -0.9 * W);
}

// Spins about the minor and major axes are stable: a small perturbation stays small.
TEST(RigidBody, MajorAndMinorAxisSpinsStayBounded) {
    const MassProperties mp = make_mass_properties(1.0, 1.0, 2.0, 3.0, 0.0);
    for (const Vec3& w0 : {Vec3(1.0, 1e-6, 1e-6), Vec3(1e-6, 1e-6, 1.0)}) {
        Executive ex = make_executive(mp);
        BodyState b0;
        b0.omega_b = w0;
        ex.set_body_state(b0);
        double max_off = 0.0;
        for (int i = 0; i < 60; ++i) {
            run_for(ex, 1.0);
            const Vec3 w = ex.body_state().omega_b;
            const double off = w0.x() > 0.5 ? std::hypot(w.y(), w.z()) : std::hypot(w.x(), w.y());
            max_off = std::max(max_off, off);
        }
        EXPECT_LT(max_off, 1e-5) << "spin " << w0.transpose();
    }
}

// ---- Constant applied moment ----------------------------------------------

class ConstantWrench : public ForceModel {
public:
    explicit ConstantWrench(Wrench w) : w_(std::move(w)) {}
    Wrench compute(const SimContext&) const override { return w_; }

private:
    Wrench w_;
};

// From rest, ω̇ = I⁻¹ M (gyroscopic term zero). Checked at the derivative and,
// for a moment about a principal axis, along a 2 s trajectory: ω(t) = I⁻¹ M t.
TEST(RigidBody, ConstantMomentGivesAnalyticAngularAcceleration) {
    const MassProperties mp = helicopter_like_body();
    const Vec3 M(1500.0, -2500.0, 4000.0);  // [N·m]
    Wrench w;
    w.M = M;
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::make_unique<ConstantWrench>(w));
    Executive ex = make_executive(mp, std::move(forces));
    ex.set_body_state(BodyState{});

    std::vector<double> xdot(ex.num_states());
    ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, xdot);
    const Vec3 wdot(xdot[body_layout::kOmega], xdot[body_layout::kOmega + 1],
                    xdot[body_layout::kOmega + 2]);
    const Vec3 want = mp.inertia_b.ldlt().solve(M);
    EXPECT_LT(rel_err(wdot, want), 1e-12);

    // Principal axis (y: Ixz couples x and z only), so ω stays along y.
    Wrench wy;
    wy.M = Vec3(0.0, -2500.0, 0.0);
    std::vector<std::unique_ptr<ForceModel>> forces_y;
    forces_y.push_back(std::make_unique<ConstantWrench>(wy));
    Executive ey = make_executive(mp, std::move(forces_y));
    ey.set_body_state(BodyState{});
    run_for(ey, 2.0);
    const Vec3 w_want(0.0, wy.M.y() / mp.inertia_b(1, 1) * 2.0, 0.0);
    EXPECT_LT((ey.body_state().omega_b - w_want).norm(), 1e-12 * w_want.norm());
}

// ---- Quaternion kinematics ------------------------------------------------

// For constant ω, q(t) = q0 ⊗ [cos(|ω|t/2), sin(|ω|t/2) ω/|ω|] (D-013). A
// principal-axis spin keeps ω constant. RK4's truncation error is far below
// round-off at this rate, so the tolerance reflects round-off growth over
// 60 000 steps.
TEST(RigidBody, AttitudeMatchesClosedFormForConstantRate) {
    const MassProperties mp = make_mass_properties(1.0, 1.0, 2.0, 3.0, 0.0);
    Executive ex = make_executive(mp);
    BodyState b0;
    b0.q_nb = attitude(0.3, -0.2, 1.0);
    b0.omega_b = Vec3(0.0, 0.0, 2.0);
    ex.set_body_state(b0);
    run_for(ex, 60.0);
    const double T = 60.0;
    const double ang = b0.omega_b.norm() * T;
    const Quat q_want = b0.q_nb * Quat(Eigen::AngleAxisd(ang, b0.omega_b.normalized()));
    // Compare as rotations; q and −q are the same attitude.
    const double dot = std::abs(ex.body_state().q_nb.coeffs().dot(q_want.coeffs()));
    EXPECT_NEAR(dot, 1.0, 1e-12);
    EXPECT_LT((ex.body_state().q_nb * Vec3::UnitX() - q_want * Vec3::UnitX()).norm(), 1e-10);
}

// Spec M1: per-step norm drift before renormalization < 1e-12, at a stated
// rate (D-013): a 1–2 rad/s tumble.
TEST(RigidBody, QuaternionNormDriftBelowLimitAtTypicalRates) {
    Executive ex = make_executive(helicopter_like_body());
    BodyState b0;
    b0.omega_b = Vec3(0.6, -0.5, 0.8);
    ex.set_body_state(b0);
    double worst = 0.0;
    for (int i = 0; i < 10'000; ++i) {
        (void)ex.advance(Controls{}, Environment{}, kStepNs);
        worst = std::max(worst, ex.quat_norm_error());
    }
    EXPECT_LT(worst, 1e-12);
}

// Anchor for the norm diagnostic. RK4 on q̇ = ½ Ω q with constant ω multiplies
// q by the 4th-order Taylor polynomial of exp(Ω h), whose norm is
// sqrt(1 − θ⁶/72 + θ⁸/576) with θ = |ω| h / 2. At 50 rad/s the drift
// (≈ 1.7e-12) is far above round-off, so it must match this formula.
TEST(RigidBody, QuaternionNormDriftMatchesRk4Prediction) {
    Executive ex = make_executive(make_mass_properties(1.0, 1.0, 2.0, 3.0, 0.0));
    BodyState b0;
    b0.omega_b = Vec3(50.0, 0.0, 0.0);
    ex.set_body_state(b0);
    (void)ex.advance(Controls{}, Environment{}, kStepNs);
    const double th = 50.0 * kStep / 2.0;
    // 1 − sqrt(1 − u) = u/2 + u²/8 + …, evaluated without cancellation.
    const double u = std::pow(th, 6) / 72.0 - std::pow(th, 8) / 576.0;
    const double want = u / 2.0 + u * u / 8.0;
    EXPECT_NEAR(ex.quat_norm_error() / want, 1.0, 1e-3);
}

}  // namespace
}  // namespace eyas
