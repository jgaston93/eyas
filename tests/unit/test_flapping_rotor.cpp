// M4 flapping rotor: exact-equation reference, conservation through the
// coupled solve, config and real-time rules (D-026 layers 1 and 2).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <random>
#include <vector>

#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/multiblade.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/flap_momentum.hpp"
#include "support/flap_reference.hpp"
#include "support/m4_fixtures.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using test::Beads;
using test::kDeg;

struct Case {
    const char* name;
    RotationDirection dir;
    double offset;  // [m]
    double spring;  // [N·m/rad]
    double damper;  // [N·m·s/rad]
    double delta3;  // [rad]
    double hub_pitch;
    double hub_roll;
    Vec3 hub_pos;
    Vec3 wind_n;
};

// Production derivatives against the exact reference: β̈ = ẋ0 + G a and
// load = load0 + H a for a = 0, each unit vector and a random a.
TEST(FlappingRotor, MatchesExactReference) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    const Beads beads = test::nonuniform_beads(8.0 - 0.4);
    const std::vector<Case> cases{
        {"ccw, plain", RotationDirection::counter_clockwise, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0,
         Vec3(0.3, 0.0, -2.0), Vec3::Zero()},
        {"cw, spring, damper, delta3", RotationDirection::clockwise, 0.4, 2.0e4, 300.0, 20 * kDeg,
         -5 * kDeg, 2 * kDeg, Vec3(-0.2, 0.1, -1.8), Vec3(3.0, -2.0, 0.5)},
        {"ccw, centre spring, tilted", RotationDirection::counter_clockwise, 0.0, 3.0e4, 0.0,
         -15 * kDeg, -3 * kDeg, -4 * kDeg, Vec3(0.4, -0.3, -2.2), Vec3(-4.0, 1.0, 0.0)},
    };
    std::mt19937 rng(20261011);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (const Case& cs : cases) {
        SCOPED_TRACE(cs.name);
        BladeElementRotorConfig c =
            test::flap_rotor(airfoil, {9 * kDeg, -3 * kDeg, 2 * kDeg}, beads, cs.offset, 20);
        c.root_cutout = 1.2;
        c.direction = cs.dir;
        c.hub_pitch = cs.hub_pitch;
        c.hub_roll = cs.hub_roll;
        c.hub_position_b = cs.hub_pos;
        c.flap->spring = cs.spring;
        c.flap->damper = cs.damper;
        c.flap->delta3 = cs.delta3;
        const BladeElementRotor rotor(c);
        const test::FlapReference ref(rotor, beads);

        for (int trial = 0; trial < 3; ++trial) {
            SimContext ctx = test::sea_level_context(0.37 + 0.11 * trial);
            ctx.body.vel_b = Vec3(40.0 * (1 + u(rng)), 5.0 * u(rng), 3.0 * u(rng));
            ctx.body.omega_b = Vec3(0.4 * u(rng), 0.4 * u(rng), 0.4 * u(rng));
            ctx.body.q_nb = Quat(Eigen::AngleAxisd(0.3 * u(rng), Vec3::UnitZ()) *
                                 Eigen::AngleAxisd(0.2 * u(rng), Vec3::UnitY()) *
                                 Eigen::AngleAxisd(0.2 * u(rng), Vec3::UnitX()));
            ctx.env.wind_n = cs.wind_n;
            std::vector<double> x(rotor.num_states());
            for (std::size_t i = 0; i < x.size(); i += 2) {
                x[i] = 0.08 + 0.06 * u(rng);  // β [rad]
                x[i + 1] = 0.5 * u(rng);      // β̇ [rad/s]
            }
            std::vector<double> xdot0(x.size());
            std::vector<double> G(x.size() * 6);
            ComponentDerivs d{xdot0, G, Wrench{}, Mat66::Zero()};
            rotor.derivatives(ctx, x, d);
            ASSERT_TRUE(d.converged);

            std::vector<Vec6> accels{Vec6::Zero()};
            for (int j = 0; j < 6; ++j) {
                accels.push_back(Vec6::Unit(j) * (j < 3 ? 9.0 : 2.0));
            }
            Vec6 ar;
            ar << 5 * u(rng), 5 * u(rng), 9 * u(rng), 2 * u(rng), 2 * u(rng), 2 * u(rng);
            accels.push_back(ar);
            for (const Vec6& a : accels) {
                const auto r = ref.evaluate(ctx, x, a);
                Vec6 load_p;
                load_p << d.load0_b.F, d.load0_b.M;
                load_p += d.H * a;
                Vec6 load_r;
                load_r << r.load_b.F, r.load_b.M;
                // Loads are ~1e4 N and ~1e5 N·m; the inflow solves agree to ~1e-13.
                EXPECT_LT((load_p.head<3>() - load_r.head<3>()).norm(),
                          1e-7 * load_r.head<3>().norm());
                EXPECT_LT((load_p.tail<3>() - load_r.tail<3>()).norm(),
                          1e-7 * load_r.tail<3>().norm());
                for (int k = 0; k < c.num_blades; ++k) {
                    const std::size_t ib = 2 * static_cast<std::size_t>(k);
                    EXPECT_EQ(xdot0[ib], x[ib + 1]);
                    double bdd = xdot0[ib + 1];
                    for (std::size_t j = 0; j < 6; ++j) {
                        EXPECT_EQ(G[ib * 6 + j], 0.0);
                        bdd += G[(ib + 1) * 6 + j] * a(static_cast<Eigen::Index>(j));
                    }
                    // β̈ is ~1e2 rad/s² (centrifugal Ω²β).
                    EXPECT_NEAR(bdd, r.beta_ddot[static_cast<std::size_t>(k)], 1e-8 * 100.0)
                        << "blade " << k;
                }
            }
        }
    }
}

// The same rotor without a flap hinge is the M3 rotor: no states, and the
// flapping rotor at β = β̇ = 0 has the same aerodynamic loads.
TEST(FlappingRotor, ZeroFlapReproducesM3Aerodynamics) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    const SwashplatePitch pitch{8 * kDeg, -2 * kDeg, 1 * kDeg};
    BladeElementRotorConfig c3 = test::test_rotor(airfoil, pitch, 20);
    const BladeElementRotor m3(c3);
    BladeElementRotorConfig c4 = c3;
    c4.flap = test::hinge_from(test::nonuniform_beads(8.0), 0.0);
    const BladeElementRotor m4(c4);
    EXPECT_EQ(m3.num_states(), 0U);
    EXPECT_EQ(m4.num_states(), 8U);
    SimContext ctx = test::sea_level_context(0.2);
    ctx.body.vel_b = Vec3(30.0, 2.0, 1.0);
    ctx.body.omega_b = Vec3(0.1, -0.2, 0.05);
    const std::vector<double> x(8, 0.0);
    const RotorLoads a = m3.evaluate(ctx);
    const RotorLoads b = m4.evaluate(ctx, x);
    EXPECT_LT((a.F_h - b.F_h).norm(), 1e-12 * a.F_h.norm());
    EXPECT_LT((a.M_h - b.M_h).norm(), 1e-12 * a.M_h.norm());
    EXPECT_NEAR(a.lambda_i, b.lambda_i, 1e-15);
}

// Through the production executive and coupled solve: an airframe tumbling
// with flapping blades, in vacuum. Total momentum changes only by the
// weight, and the angular momentum about the system CG is constant. The
// drift is RK4 truncation, so it must fall as h⁴; a wrong G or H gives an
// O(1) drift that doesn't converge.
TEST(FlappingRotor, CoupledSystemConservesMomentum) {
    const auto airfoil = test::airfoil_from(test::zero_airfoil_csv());
    const double e = 0.4;
    const Beads beads = test::nonuniform_beads(8.0 - e);
    BladeElementRotorConfig c = test::flap_rotor(airfoil, {}, beads, e, 8);
    c.hub_position_b = Vec3(0.3, 0.1, -2.0);
    c.hub_pitch = -4 * kDeg;
    c.flap->spring = 5e3;
    c.flap->damper = 200.0;
    const MassProperties mp = make_mass_properties(900.0, 600.0, 1500.0, 1400.0, 120.0);

    const auto run = [&](std::int64_t step_ns, double& dP, double& dH) {
        ExecutiveConfig ec;
        ec.mass = mp;
        ec.step_ns = step_ns;
        ec.max_substeps = 100000;
        std::vector<std::unique_ptr<DynamicComponent>> comps;
        comps.push_back(std::make_unique<BladeElementRotor>(c));
        const auto* rotor = dynamic_cast<const BladeElementRotor*>(comps.back().get());
        Executive ex(ec, std::move(comps));
        BodyState b;
        b.vel_b = Vec3(3.0, -1.0, 2.0);
        b.omega_b = Vec3(0.6, -0.4, 0.8);
        b.q_nb = Quat(Eigen::AngleAxisd(0.3, Vec3(1, 2, 3).normalized()));
        ex.set_body_state(b);
        std::vector<double> x(ex.state().begin(), ex.state().end());
        for (std::size_t k = 0; k < 4; ++k) {
            x[body_layout::kSize + 2 * k] = 0.05 * (1.0 + static_cast<double>(k));
            x[body_layout::kSize + 2 * k + 1] = 0.3 - 0.2 * static_cast<double>(k);
        }
        ASSERT_TRUE(ex.restore(x, 0, 0));
        const test::FlapReference ref(*rotor, beads);
        const test::Momentum m0 = test::system_momentum(ex, *rotor, ref, beads, mp);
        const auto st = ex.advance({}, {}, 1'000'000'000);  // 1 s
        ASSERT_FALSE(st.non_finite);
        ASSERT_EQ(st.substeps, 1'000'000'000 / step_ns);
        const test::Momentum m1 = test::system_momentum(ex, *rotor, ref, beads, mp);
        dP = (m1.P_n - (m0.P_n + m0.mass * gravity_n() * 1.0)).norm() / (m0.mass * 10.0);
        dH = (m1.H_cg_n - m0.H_cg_n).norm() / m0.H_cg_n.norm();
    };
    double dP1 = 0.0;
    double dH1 = 0.0;
    double dP2 = 0.0;
    double dH2 = 0.0;
    run(1'000'000, dP1, dH1);
    run(500'000, dP2, dH2);
    RecordProperty("dH_1kHz", test::fmt17(dH1));
    RecordProperty("dH_2kHz", test::fmt17(dH2));
    RecordProperty("dP_1kHz", test::fmt17(dP1));
    EXPECT_LT(dH1, 1e-7);
    EXPECT_LT(dP1, 1e-9);
    // Fourth-order convergence (ratio 16), unless already at round-off.
    if (dH2 > 1e-13) {
        EXPECT_GT(dH1 / dH2, 10.0);
        EXPECT_LT(dH1 / dH2, 25.0);
    }
}

TEST(FlappingRotor, MultibladeCoordinatesReconstructDiscModes) {
    // Padfield 3rd ed. Eqs. 3.42–3.45: blades on a disc β_i = β0 + β0d(−1)^i
    // + β1c cos ψ_i + β1s sin ψ_i give back the four coordinates.
    const double psi = 0.7;
    std::vector<double> x(8, 0.0);
    for (int k = 0; k < 4; ++k) {
        const double p = psi - 0.5 * std::numbers::pi * k;
        const double sign = (k % 2 == 0) ? -1.0 : 1.0;
        x[2 * static_cast<std::size_t>(k)] =
            0.05 + 0.01 * sign + 0.02 * std::cos(p) - 0.03 * std::sin(p);
    }
    const MultibladeFlap m = multiblade_flap(x, psi, 4);
    EXPECT_NEAR(m.beta0, 0.05, 1e-15);
    EXPECT_NEAR(m.beta0d, 0.01, 1e-15);
    EXPECT_NEAR(m.beta1c, 0.02, 1e-15);
    EXPECT_NEAR(m.beta1s, -0.03, 1e-15);
}

TEST(FlappingRotor, DerivativesArePureAndDontAllocate) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    const BladeElementRotor rotor(test::flap_rotor(airfoil, {8 * kDeg, -2 * kDeg, 1 * kDeg},
                                                   test::nonuniform_beads(7.6), 0.4));
    SimContext ctx = test::sea_level_context(0.3);
    ctx.body.vel_b = Vec3(50.0, 0.0, 2.0);
    std::vector<double> x{0.05, 0.1, 0.06, -0.2, 0.04, 0.3, 0.07, 0.0};
    std::vector<double> xd1(8);
    std::vector<double> G1(48);
    std::vector<double> xd2(8);
    std::vector<double> G2(48);
    ComponentDerivs d1{xd1, G1, Wrench{}, Mat66::Zero()};
    ComponentDerivs d2{xd2, G2, Wrench{}, Mat66::Zero()};
    rotor.derivatives(ctx, x, d1);
    SimContext other = ctx;
    other.t = 1.7;
    other.body.vel_b = Vec3(-20.0, 5.0, 0.0);
    {
        test::AllocationCounter count;
        rotor.derivatives(other, x, d2);
        rotor.derivatives(ctx, x, d2);
        EXPECT_EQ(count.count(), 0U);
    }
    EXPECT_EQ(xd1, xd2);
    EXPECT_EQ(G1, G2);
    EXPECT_EQ(d1.H, d2.H);
    EXPECT_EQ(d1.load0_b.F, d2.load0_b.F);
    EXPECT_EQ(d1.load0_b.M, d2.load0_b.M);
}

TEST(FlappingRotor, RejectsInvalidHinge) {
    const auto airfoil = test::airfoil_from(test::linear_airfoil_csv(5.7, 0.0));
    const Beads beads = test::uniform_beads(100.0, 7.0, 10);
    const auto make = [&](auto edit) {
        BladeElementRotorConfig c = test::flap_rotor(airfoil, {}, beads, 1.0);
        edit(*c.flap, c);
        return BladeElementRotor(c);
    };
    EXPECT_NO_THROW(make([](FlapHinge&, BladeElementRotorConfig&) {}));
    EXPECT_THROW(make([](FlapHinge&, BladeElementRotorConfig& c) { c.root_cutout = 0.5; }),
                 InitError);
    EXPECT_THROW(make([](FlapHinge& f, BladeElementRotorConfig&) { f.mass = 0.0; }), InitError);
    EXPECT_THROW(make([](FlapHinge& f, BladeElementRotorConfig&) { f.first_moment = -1.0; }),
                 InitError);
    EXPECT_THROW(make([](FlapHinge& f, BladeElementRotorConfig&) { f.inertia *= 0.25; }),
                 InitError);
    EXPECT_THROW(make([](FlapHinge& f, BladeElementRotorConfig&) { f.spring = -1.0; }), InitError);
    EXPECT_THROW(make([](FlapHinge& f, BladeElementRotorConfig&) { f.delta3 = 1.6; }), InitError);
}

}  // namespace
}  // namespace eyas
