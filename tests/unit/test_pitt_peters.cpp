// M5 Pitt–Peters dynamic inflow (D-027): layer 1, the production equations
// against independent transcriptions of Pitt & Peters 1980 Table 4 and
// Johnson 2013 Eq. 11.143, and the production rotor against the exact
// references fed with a linear inflow; layer 2, closed-form anchors (steady
// hover, time constants, coupled hover eigenvalues, step collective, wake
// skew); and the real-time rules.

#include <gtest/gtest.h>

#include <cmath>
#include <complex>
#include <memory>
#include <numbers>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "rotor/blade_element_rotor.hpp"
#include "rotor/pitt_peters.hpp"
#include "support/alloc_counter.hpp"
#include "support/bet_reference.hpp"
#include "support/flap_reference.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m4_fixtures.hpp"
#include "support/pitt_peters_reference.hpp"

namespace eyas {
namespace {

using test::kDeg;
using test::PPInputs;
using test::PPRates;

constexpr double kPi = std::numbers::pi;
constexpr double kM0 = 128.0 / (75.0 * kPi);  // Pitt & Peters 1980 Table 4
constexpr double kM1 = 16.0 / (45.0 * kPi);   // Table 3/4; Johnson 2013 p. 437

// The moment-column mass-flow speed in hover, V_m (D-027 decision note: the
// first alternative to try is Pitt & Peters 1980 eq. 23b, V_m = 2λ0).
double hover_moment_mass_flow(double lambda0) {
    return lambda0;
}

Vec3 rates(const PPInputs& in, const Vec3& loads) {
    return pitt_peters_rates(Vec3(in.l0, in.l1s, in.l1c), loads, in.mu, in.mu_z, std::cos(in.psi_w),
                             std::sin(in.psi_w), in.omega);
}

void expect_rates_near(const Vec3& p, const PPRates& r, double tol) {
    const double scale = std::max({std::abs(r.l0), std::abs(r.l1s), std::abs(r.l1c), 1e-12});
    EXPECT_NEAR(p.x(), r.l0, tol * scale);
    EXPECT_NEAR(p.y(), r.l1s, tol * scale);
    EXPECT_NEAR(p.z(), r.l1c, tol * scale);
}

// ---- Layer 1: the equations against both sources ------------------------------------

TEST(PittPeters, MatchesPittPeters1980AndJohnson2013) {
    std::mt19937 rng(20261012);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::vector<PPInputs> cases;
    // Hover, V = 0 (no flow at all), edgewise (λ = 0, X = 1), windmill
    // (λ < 0), then random flight conditions.
    cases.push_back({0.07, 0.01, -0.02, 0.0, 0.0, 0.0, 27.0});
    cases.push_back({0.0, 0.01, 0.02, 0.0, 0.0, 0.4, 27.0});
    cases.push_back({0.03, -0.01, 0.02, 0.2, 0.03, 0.0, 27.0});
    cases.push_back({0.02, 0.005, 0.01, 0.05, 0.08, 1.0, 27.0});
    for (int i = 0; i < 200; ++i) {
        cases.push_back({0.08 * u(rng), 0.03 * u(rng), 0.03 * u(rng), 0.2 * (1.0 + u(rng)),
                         0.1 * u(rng), kPi * u(rng), 20.0 + 10.0 * u(rng)});
    }
    for (const PPInputs& in : cases) {
        const double ct = 0.01 * u(rng);
        const double c1s = 0.002 * u(rng);
        const double c1c = 0.002 * u(rng);
        const Vec3 p = rates(in, Vec3(ct, c1s, c1c));
        // Pitt & Peters 1980: C_L = −C_1s, C_M = −C_1c (eqs. 5b–5c).
        expect_rates_near(p, test::pitt_peters_1980(in, ct, -c1s, -c1c), 1e-12);
        // Johnson 2013: −C_My = C_1c, C_Mx = C_1s (Eqs. 11.150–11.151).
        expect_rates_near(p, test::johnson_2013(in, ct, c1c, c1s), 1e-12);
    }
}

// Pitt & Peters 1980 p. 47-9: "L is always invertible", and the roots are
// stable. The paper also says the roots are real for all α; that doesn't hold
// for its own Table 4 matrices, whose (λ0, λ1c) pair is complex for α < 74.6°
// (X > 0.136), though well damped (SPEC_REVIEW #10 findings). Real parts must
// be negative and the damping ratio at least 0.7 (computed minimum 0.744).
TEST(PittPeters, GainInvertibleAndStableAcrossSkew) {
    const Eigen::Matrix3d M = Eigen::Vector3d(kM0, kM1, kM1).asDiagonal();
    double min_zeta = 1.0;
    for (int i = 0; i <= 100; ++i) {
        const double X = i / 100.0;
        const Mat3 L = pitt_peters_gain(X);
        EXPECT_GT(L.determinant(), 0.0) << "X = " << X;
        const Eigen::EigenSolver<Eigen::Matrix3d> es(-(M.inverse() * L.inverse()));
        for (int k = 0; k < 3; ++k) {
            const std::complex<double> e = es.eigenvalues()(k);
            EXPECT_LT(e.real(), 0.0) << "X = " << X;
            min_zeta = std::min(min_zeta, -e.real() / std::abs(e));
        }
    }
    RecordProperty("min_damping_ratio", test::fmt17(min_zeta));
    EXPECT_GE(min_zeta, 0.7);
}

TEST(PittPeters, WakeSkewMatchesJohnson) {
    // X = tan(χ/2), tan χ = μ/|λ| (Johnson 2013 Eqs. 11.131–11.132).
    for (const double mu : {0.0, 0.01, 0.1, 0.3}) {
        for (const double lam : {-0.05, 0.0, 0.02, 0.08}) {
            const WakeSkew w = wake_skew(mu, lam);
            EXPECT_NEAR(w.V, std::hypot(mu, lam), 1e-16);
            const double X =
                (mu == 0.0 && lam == 0.0) ? 0.0 : std::tan(0.5 * std::atan2(mu, std::abs(lam)));
            EXPECT_NEAR(w.X, X, 1e-15) << mu << " " << lam;
        }
    }
}

// Rotating the in-plane wind by Δ about the shaft rotates the steady
// pattern with it: rates(R λ, R C, ψ_w + Δ) = R rates(λ, C, ψ_w).
TEST(PittPeters, RotatesWithTheWind) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int i = 0; i < 50; ++i) {
        PPInputs in{0.06 * u(rng), 0.02 * u(rng), 0.02 * u(rng), 0.15 * (1 + u(rng)),
                    0.05 * u(rng), kPi * u(rng),  27.0};
        const Vec3 C(0.008, 0.001 * u(rng), 0.001 * u(rng));
        const double d = kPi * u(rng);
        const auto rot = [d](double c, double s) {
            const std::complex<double> z = std::complex<double>(c, s) * std::polar(1.0, d);
            return z;
        };
        const Vec3 r0 = rates(in, C);
        PPInputs in2 = in;
        const std::complex<double> l = rot(in.l1c, in.l1s);
        in2.l1c = l.real();
        in2.l1s = l.imag();
        in2.psi_w = in.psi_w + d;
        const std::complex<double> c2 = rot(C.z(), C.y());
        const Vec3 r1 = rates(in2, Vec3(C.x(), c2.imag(), c2.real()));
        const std::complex<double> e = rot(r0.z(), r0.y());
        EXPECT_NEAR(r1.x(), r0.x(), 1e-12 * r0.norm());
        EXPECT_NEAR(r1.y(), e.imag(), 1e-12 * r0.norm());
        EXPECT_NEAR(r1.z(), e.real(), 1e-12 * r0.norm());
    }
}

// ---- Layer 1: the production rotor -------------------------------------------------

BladeElementRotorConfig pp_rotor(std::shared_ptr<const Airfoil> airfoil, SwashplatePitch pitch,
                                 int elements) {
    BladeElementRotorConfig c = test::test_rotor(std::move(airfoil), pitch, elements);
    c.inflow = InflowModel::pitt_peters;
    return c;
}

// The downwind azimuth from the hub velocity (D-027), and the PP inputs.
PPInputs inputs_for(const BladeElementRotor& rotor, const SimContext& ctx, const LinearInflow& l) {
    const test::RefCondition k = test::reference_condition(rotor, ctx);
    const double vr = rotor.tip_speed();
    const double s = rotor.config().direction == RotationDirection::counter_clockwise ? 1.0 : -1.0;
    PPInputs in;
    in.l0 = l.l0;
    in.l1s = l.l1s;
    in.l1c = l.l1c;
    in.mu = std::hypot(k.v_hub_h.x(), k.v_hub_h.y()) / vr;
    in.mu_z = k.v_hub_h.z() / vr;
    in.psi_w = std::atan2(-s * k.v_hub_h.y(), k.v_hub_h.x());
    in.omega = rotor.config().rotor_speed;
    return in;
}

// Unflapped rotor at a given linear inflow: C_T, C_1s, C_1c and the hub
// loads converge to the exact reference (adaptive quadrature) at second order.
// As in the M3 forward-flight test: a linear airfoil (a piecewise-linear
// table has kinks, so Richardson extrapolation wouldn't apply) and a root
// cutout that keeps the blades out of reverse flow.
TEST(PittPetersRotor, LinearInflowPassMatchesExactReference) {
    const auto airfoil = test::airfoil_from(test::linear_airfoil_csv(5.9, 0.01));
    for (const RotationDirection dir :
         {RotationDirection::counter_clockwise, RotationDirection::clockwise}) {
        SimContext ctx = test::sea_level_context(0.041);
        ctx.body.vel_b = Vec3(35.0, -6.0, 2.0);
        ctx.body.omega_b = Vec3(0.1, -0.15, 0.05);
        const LinearInflow l{0.05, -0.012, 0.03};
        const auto make = [&](int n) {
            BladeElementRotorConfig c = pp_rotor(airfoil, {20 * kDeg, -6 * kDeg, 2.5 * kDeg}, n);
            c.direction = dir;
            c.root_cutout = 2.4;
            return BladeElementRotor(c);
        };
        const BladeElementRotor r0 = make(10);
        const test::RefLoads ref = test::reference_loads_at(
            test::reference_rotor(r0), test::reference_condition(r0, ctx), l.l0, l.l1s, l.l1c);
        ASSERT_GT(ref.thrust, 0.0);
        const std::array<double, 5> want{ref.ct, -ref.c_l_pp, -ref.c_m_pp, ref.M_h.x(),
                                         ref.M_h.y()};
        std::vector<std::array<double, 5>> got;
        for (int n = 160; n <= 1280; n *= 2) {
            const RotorLoads p = make(n).evaluate_at_inflow(ctx, l);
            EXPECT_EQ(p.lambda_i, l.l0);
            EXPECT_EQ(p.lambda_1s, l.l1s);
            EXPECT_EQ(p.lambda_1c, l.l1c);
            got.push_back({p.ct, p.c1s, p.c1c, p.M_h.x(), p.M_h.y()});
        }
        const std::array<double, 5> scale{ref.ct, ref.ct, ref.ct, ref.thrust * 8.0,
                                          ref.thrust * 8.0};
        for (std::size_t i = 0; i < 5; ++i) {
            const double extrapolated = (4.0 * got[3][i] - got[2][i]) / 3.0;
            EXPECT_NEAR(extrapolated, want[i], 1e-9 * scale[i]) << "component " << i;
        }
    }
}

// Unflapped rotor: the inflow rates are the Pitt–Peters equations evaluated
// on the loads of the same blade-element pass.
TEST(PittPetersRotor, InflowRatesUseLoadsFromTheSamePass) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (const RotationDirection dir :
         {RotationDirection::counter_clockwise, RotationDirection::clockwise}) {
        BladeElementRotorConfig c = pp_rotor(airfoil, {9 * kDeg, 2 * kDeg, -1 * kDeg}, 20);
        c.direction = dir;
        c.hub_pitch = -4 * kDeg;
        c.hub_roll = 2 * kDeg;
        const BladeElementRotor rotor(c);
        ASSERT_EQ(rotor.num_states(), 3U);
        ASSERT_EQ(rotor.inflow_offset(), 0U);
        for (int trial = 0; trial < 10; ++trial) {
            SimContext ctx = test::sea_level_context(0.1 * trial);
            ctx.body.vel_b = Vec3(30.0 * u(rng), 30.0 * u(rng), 5.0 * u(rng));
            ctx.body.omega_b = Vec3(0.3 * u(rng), 0.3 * u(rng), 0.3 * u(rng));
            ctx.env.wind_n = Vec3(5.0 * u(rng), 5.0 * u(rng), 0.0);
            const std::vector<double> x{0.05 + 0.03 * u(rng), 0.02 * u(rng), 0.02 * u(rng)};
            std::vector<double> xdot0(3);
            std::vector<double> G(18);
            ComponentDerivs d{xdot0, G, Wrench{}, Mat66::Zero()};
            rotor.derivatives(ctx, x, d);
            EXPECT_TRUE(d.converged);
            const RotorLoads l = rotor.evaluate(ctx, x);
            EXPECT_EQ(l.passes, 1);
            EXPECT_EQ(l.lambda_i, x[0]);
            const PPInputs in = inputs_for(rotor, ctx, {x[0], x[1], x[2]});
            const PPRates want = test::pitt_peters_1980(in, l.ct, -l.c1s, -l.c1c);
            expect_rates_near(Vec3(xdot0[0], xdot0[1], xdot0[2]), want, 1e-11);
            for (const double g : G) {
                EXPECT_EQ(g, 0.0);
            }
            // The aerodynamic loads reach the body as for uniform inflow.
            EXPECT_EQ(d.load0_b.F, l.body.F);
            EXPECT_EQ(d.load0_b.M, l.body.M);
        }
    }
}

// Flapping rotor with Pitt–Peters inflow against the M4 exact reference at
// the same linear inflow: β̈, airframe loads (with G and H), and the inflow
// rates from the reference's own load integrals.
TEST(PittPetersRotor, FlappingRotorMatchesExactReference) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    const test::Beads beads = test::nonuniform_beads(8.0 - 0.4);
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (const RotationDirection dir :
         {RotationDirection::counter_clockwise, RotationDirection::clockwise}) {
        BladeElementRotorConfig c =
            test::flap_rotor(airfoil, {9 * kDeg, -3 * kDeg, 2 * kDeg}, beads, 0.4, 20);
        c.root_cutout = 1.2;
        c.direction = dir;
        c.hub_pitch = -5 * kDeg;
        c.hub_roll = 2 * kDeg;
        c.hub_position_b = Vec3(-0.2, 0.1, -1.8);
        c.flap->spring = 2e4;
        c.flap->damper = 300.0;
        c.flap->delta3 = 20 * kDeg;
        c.inflow = InflowModel::pitt_peters;
        const BladeElementRotor rotor(c);
        const test::FlapReference ref(rotor, beads);
        ASSERT_EQ(rotor.num_states(), 11U);
        ASSERT_EQ(rotor.inflow_offset(), 8U);
        for (int trial = 0; trial < 3; ++trial) {
            SimContext ctx = test::sea_level_context(0.29 + 0.13 * trial);
            ctx.body.vel_b = Vec3(40.0 * (1 + u(rng)), 8.0 * u(rng), 3.0 * u(rng));
            ctx.body.omega_b = Vec3(0.4 * u(rng), 0.4 * u(rng), 0.4 * u(rng));
            ctx.body.q_nb = Quat(Eigen::AngleAxisd(0.3 * u(rng), Vec3::UnitZ()) *
                                 Eigen::AngleAxisd(0.2 * u(rng), Vec3::UnitY()));
            ctx.env.wind_n = Vec3(3.0, -2.0, 0.5);
            std::vector<double> x(rotor.num_states());
            for (std::size_t i = 0; i < 8; i += 2) {
                x[i] = 0.08 + 0.06 * u(rng);
                x[i + 1] = 0.5 * u(rng);
            }
            const LinearInflow l{0.05 + 0.02 * u(rng), 0.02 * u(rng), 0.02 * u(rng)};
            x[8] = l.l0;
            x[9] = l.l1s;
            x[10] = l.l1c;
            std::vector<double> xdot0(x.size());
            std::vector<double> G(x.size() * 6);
            ComponentDerivs d{xdot0, G, Wrench{}, Mat66::Zero()};
            rotor.derivatives(ctx, x, d);

            Vec6 a;
            a << 5 * u(rng), 5 * u(rng), 9 * u(rng), 2 * u(rng), 2 * u(rng), 2 * u(rng);
            const auto r = ref.evaluate_at_inflow(ctx, x, a, {l.l0, l.l1s, l.l1c});
            Vec6 load_p;
            load_p << d.load0_b.F, d.load0_b.M;
            load_p += d.H * a;
            EXPECT_LT((load_p.head<3>() - r.load_b.F).norm(), 1e-7 * r.load_b.F.norm());
            EXPECT_LT((load_p.tail<3>() - r.load_b.M).norm(), 1e-7 * r.load_b.M.norm());
            for (std::size_t k = 0; k < 4; ++k) {
                double bdd = xdot0[2 * k + 1];
                for (std::size_t j = 0; j < 6; ++j) {
                    bdd += G[(2 * k + 1) * 6 + j] * a(static_cast<Eigen::Index>(j));
                }
                EXPECT_NEAR(bdd, r.beta_ddot[k], 1e-6) << "blade " << k;
            }
            const PPInputs in = inputs_for(rotor, ctx, l);
            const PPRates want = test::pitt_peters_1980(in, r.ct, r.c_l_pp, r.c_m_pp);
            expect_rates_near(Vec3(xdot0[8], xdot0[9], xdot0[10]), want, 1e-9);
            for (std::size_t j = std::size_t{8} * 6; j < G.size(); ++j) {
                EXPECT_EQ(G[j], 0.0);
            }
        }
    }
}

// ---- Layer 2: closed-form anchors --------------------------------------------------
// Fixture: the closed forms' assumptions (Leishman 2nd ed. §3.2): linear
// airfoil without drag, constant chord, linear twist, no root cutout, B = 1,
// rigid blades, 4 blades (instantaneous sums of sin² ψ_k are N_b/2).

constexpr double kLiftSlope = 5.73;

BladeElementRotorConfig anchor_rotor(double theta0) {
    BladeElementRotorConfig c = pp_rotor(
        test::airfoil_from(test::linear_airfoil_csv(kLiftSlope, 0.0)), {theta0, 0.0, 0.0}, 200);
    c.root_cutout = 0.0;
    c.twist = Table1D(Axis({0.0, 1.0}), {0.0, -8.0 * kDeg});
    return c;
}

double solidity(const BladeElementRotorConfig& c) {
    return c.num_blades * c.chord(0.5) / (kPi * c.radius);
}

// Steady state with the hub held: integrate the inflow states alone.
std::vector<double> settle(const BladeElementRotor& rotor, SimContext ctx, std::vector<double> x,
                           double seconds) {
    const int steps = static_cast<int>(std::lround(seconds / 1e-3));
    test::whirl(rotor, ctx, x, 1e-3, steps, [](double, const std::vector<double>&) {});
    return x;
}

// Spec M5: "in steady state, the result reduces to uniform momentum inflow
// in hover", from a zero start (no singularity at V = 0), and in climb.
TEST(PittPetersAnchors, SteadyHoverEqualsUniformMomentum) {
    for (const double climb : {0.0, 5.0}) {
        for (const bool synthetic : {false, true}) {
            BladeElementRotorConfig c = anchor_rotor(12 * kDeg);
            if (synthetic) {
                c.airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
                c.root_cutout = 1.2;
            }
            const BladeElementRotor rotor(c);
            SimContext ctx = test::sea_level_context();
            ctx.body.vel_b = Vec3(0.0, 0.0, -climb);
            const std::vector<double> x = settle(rotor, ctx, {0.0, 0.0, 0.0}, 3.0);
            const RotorLoads uni = rotor.evaluate_uniform(ctx);
            ASSERT_TRUE(uni.converged);
            EXPECT_NEAR(x[0], uni.lambda_i, 1e-9) << "climb " << climb;
            EXPECT_NEAR(x[1], 0.0, 1e-12);
            EXPECT_NEAR(x[2], 0.0, 1e-12);
        }
    }
}

// Spec M5: "the inflow time constant in hover matches the published value",
// for the inflow equations with the loading held fixed:
//   τ0 = (128/75π)/(4 λ0 Ω)  (Pitt & Peters 1980 Table 4 with eq. 23b, the
//   linearization of V λ0; Padfield 3rd ed. Eqs. 5.76 and 5.80 with C0 = 1:
//   1/τ0 = 4 k1 λ0, k1 = 75πΩ/128),
//   τ1 = 2 (16/45π)/(V_m Ω) for each first-harmonic state.
TEST(PittPetersAnchors, HoverInflowTimeConstants) {
    const double W = 27.0;
    for (const double l0 : {0.04, 0.06, 0.09}) {
        const Vec3 C(2.0 * l0 * l0, 0.0, 0.0);  // hover trim loading
        const auto f = [&](const Vec3& l) {
            return pitt_peters_rates(l, C, 0.0, 0.0, 1.0, 0.0, W);
        };
        const Vec3 x0(l0, 0.0, 0.0);
        EXPECT_NEAR(f(x0).norm(), 0.0, 1e-15);
        Eigen::Matrix3d J;
        const double h = 1e-7;
        for (int j = 0; j < 3; ++j) {
            const Vec3 dx = Vec3::Unit(j) * h;
            J.col(j) = (f(x0 + dx) - f(x0 - dx)) / (2.0 * h);
        }
        const double tau0 = kM0 / (4.0 * l0 * W);
        const double k1 = 75.0 * kPi * W / 128.0;  // Padfield Eq. 5.80
        EXPECT_NEAR(tau0, 1.0 / (4.0 * k1 * l0), 1e-15);
        const double tau1 = 2.0 * kM1 / (hover_moment_mass_flow(l0) * W);
        EXPECT_NEAR(-1.0 / J(0, 0), tau0, 1e-6 * tau0);
        EXPECT_NEAR(-1.0 / J(1, 1), tau1, 1e-6 * tau1);
        EXPECT_NEAR(-1.0 / J(2, 2), tau1, 1e-6 * tau1);
        EXPECT_NEAR((J - Eigen::Matrix3d(J.diagonal().asDiagonal())).norm(), 0.0, 1e-6 * J.norm());
    }
}

// Linearized hover eigenvalues through the production rotor (rigid blades,
// fixed pitch): the loads feed back as δC_T = −(σa/4) δλ0 (Leishman 2nd ed.
// Eq. 3.25; Johnson 2013 Eq. 11.148) and δC_1s = −(σa/16) δλ1s (Johnson
// Eq. 11.152), so
//   s0 = −(Ω/M0)(4λ0 + σa/4),  s1 = −(Ω/M1)(V_m/2 + σa/16).
TEST(PittPetersAnchors, CoupledHoverEigenvalues) {
    for (const double theta0 : {8 * kDeg, 12 * kDeg, 16 * kDeg}) {
        const BladeElementRotor rotor(anchor_rotor(theta0));
        const SimContext ctx = test::sea_level_context(0.0123);
        const double l0 = rotor.evaluate_uniform(ctx).lambda_i;
        const auto f = [&](const Vec3& l) {
            const std::vector<double> x{l.x(), l.y(), l.z()};
            std::vector<double> xd(3);
            std::vector<double> G(18);
            ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
            rotor.derivatives(ctx, x, d);
            return Vec3(xd[0], xd[1], xd[2]);
        };
        const Vec3 x0(l0, 0.0, 0.0);
        Eigen::Matrix3d J;
        const double h = 1e-6;
        for (int j = 0; j < 3; ++j) {
            const Vec3 dx = Vec3::Unit(j) * h;
            J.col(j) = (f(x0 + dx) - f(x0 - dx)) / (2.0 * h);
        }
        const double W = rotor.config().rotor_speed;
        const double sa = solidity(rotor.config()) * kLiftSlope;
        const double s0 = -(W / kM0) * (4.0 * l0 + sa / 4.0);
        const double s1 = -(W / kM1) * (0.5 * hover_moment_mass_flow(l0) + sa / 16.0);
        RecordProperty("lambda0_theta" + std::to_string(std::lround(theta0 / kDeg)),
                       test::fmt17(l0));
        RecordProperty("s0_err", test::fmt17(J(0, 0) / s0 - 1.0));
        RecordProperty("s1_err", test::fmt17(J(1, 1) / s1 - 1.0));
        EXPECT_NEAR(J(0, 0), s0, 0.01 * std::abs(s0)) << "θ0 = " << theta0 / kDeg;
        EXPECT_NEAR(J(1, 1), s1, 0.01 * std::abs(s1)) << "θ0 = " << theta0 / kDeg;
        EXPECT_NEAR(J(2, 2), s1, 0.01 * std::abs(s1)) << "θ0 = " << theta0 / kDeg;
        // No coupling between the states in hover (axisymmetry).
        EXPECT_LT(std::abs(J(0, 1)) + std::abs(J(0, 2)) + std::abs(J(1, 0)) + std::abs(J(1, 2)) +
                      std::abs(J(2, 0)) + std::abs(J(2, 1)),
                  1e-6 * std::abs(J(0, 0)));
    }
}

// Spec M5: the step-collective thrust overshoot. A small step δθ0 on a
// rigid-blade rotor with the hub fixed: C_T jumps by (σa/6) δθ0 (Leishman
// 2nd ed. Eq. 3.25 at fixed λ), then decays at rate s0 to C′ times the jump,
// C′ = 1/(1 + σa/(16λ0)) (Johnson 2013 Eq. 11.149; Padfield 3rd ed. Eq.
// 3.171). Overshoot ratio jump/final = 1/C′.
TEST(PittPetersAnchors, StepCollectiveOvershoot) {
    const double theta0 = 12 * kDeg;
    const double step = 0.25 * kDeg;
    const BladeElementRotor before(anchor_rotor(theta0));
    const BladeElementRotor after(anchor_rotor(theta0 + step));
    SimContext ctx = test::sea_level_context();
    const double l0 = before.evaluate_uniform(ctx).lambda_i;
    std::vector<double> x{l0, 0.0, 0.0};
    const double ct0 = before.evaluate(ctx, x).ct;

    std::vector<double> t;
    std::vector<double> dct;
    const double t_step = ctx.t;
    t.push_back(0.0);
    dct.push_back(after.evaluate(ctx, x).ct - ct0);
    test::whirl(after, ctx, x, 1e-4, 10000, [&](double tt, const std::vector<double>& xs) {
        t.push_back(tt - t_step);
        dct.push_back(after.evaluate(ctx, xs).ct - ct0);
    });
    const double sa = solidity(after.config()) * kLiftSlope;
    const double W = after.config().rotor_speed;
    const double jump = sa / 6.0 * step;
    const double cprime = 1.0 / (1.0 + sa / (16.0 * l0));
    const double s0 = -(W / kM0) * (4.0 * l0 + sa / 4.0);
    const double final_ct = dct.back();
    RecordProperty("jump_err", test::fmt17(dct.front() / jump - 1.0));
    RecordProperty("final_err", test::fmt17(final_ct / (cprime * jump) - 1.0));
    RecordProperty("overshoot", test::fmt17(dct.front() / final_ct));
    EXPECT_NEAR(dct.front(), jump, 0.02 * jump);
    EXPECT_NEAR(final_ct, cprime * jump, 0.02 * cprime * jump);
    EXPECT_NEAR(dct.front() / final_ct, 1.0 / cprime, 0.02 / cprime);
    // Decay rate from the first time constant: log of the excess.
    std::size_t i1 = 0;
    while (t[i1] < -1.0 / s0) {
        ++i1;
    }
    const double rate = std::log((dct[i1] - final_ct) / (dct.front() - final_ct)) / t[i1];
    RecordProperty("rate_err", test::fmt17(rate / s0 - 1.0));
    EXPECT_NEAR(rate, s0, 0.02 * std::abs(s0));
}

// With thrust-only loading the steady first harmonic is
// λ1c/λ0 = (15π/32) tan(χ/2) (Pitt & Peters 1980 eq. 22b / Table 4), and λ0
// is the momentum value C_T/(2V).
TEST(PittPetersAnchors, WakeSkewGradient) {
    for (const double mu : {0.05, 0.1, 0.2, 0.35}) {
        for (const double mu_z : {-0.02, 0.0, 0.02}) {
            const Vec3 C(0.007, 0.0, 0.0);
            Vec3 l(0.05, 0.0, 0.0);
            for (int i = 0; i < 40000; ++i) {
                l += 0.01 * pitt_peters_rates(l, C, mu, mu_z, 1.0, 0.0, 1.0);
            }
            const double lam = l.x() - mu_z;
            const double V = std::hypot(mu, lam);
            const double X = std::tan(0.5 * std::atan2(mu, std::abs(lam)));
            EXPECT_NEAR(l.x(), C.x() / (2.0 * V), 1e-12);
            EXPECT_NEAR(l.y(), 0.0, 1e-14);
            EXPECT_NEAR(l.z() / l.x(), 15.0 * kPi / 32.0 * X, 1e-10) << mu << " " << mu_z;
        }
    }
}

// Through the rotor: the extra downwash sits downwind, whichever way the
// rotor turns and whichever way it flies (D-027 wind axes). Averaged over a
// revolution at steady state.
TEST(PittPetersAnchors, DownwashSkewsDownwind) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    for (const RotationDirection dir :
         {RotationDirection::counter_clockwise, RotationDirection::clockwise}) {
        const double s = dir == RotationDirection::counter_clockwise ? 1.0 : -1.0;
        // 20° collective: the fixture has −16° twist, so it must lift (λ0 > 0).
        BladeElementRotorConfig c = pp_rotor(airfoil, {20 * kDeg, 0.0, 0.0}, 20);
        c.direction = dir;
        const BladeElementRotor rotor(c);
        const double rev = 2.0 * kPi / c.rotor_speed;
        // Forward: downwind is aft (ψ = 0), so λ1c > 0. Rightward: downwind
        // is the left side, at ψ = 270° for counter-clockwise (λ1s < 0) and
        // ψ = 90° for clockwise (λ1s > 0).
        for (const bool sideways : {false, true}) {
            SimContext ctx = test::sea_level_context();
            ctx.body.vel_b = sideways ? Vec3(0.0, 15.0, 0.0) : Vec3(15.0, 0.0, 0.0);
            std::vector<double> x = settle(rotor, ctx, {0.06, 0.0, 0.0}, 2.0);
            ctx.t = 2.0;
            double l1s = 0.0;
            double l1c = 0.0;
            int n = 0;
            test::whirl(rotor, ctx, x, 1e-3, static_cast<int>(rev / 1e-3),
                        [&](double, const std::vector<double>& xs) {
                            l1s += xs[1];
                            l1c += xs[2];
                            ++n;
                        });
            l1s /= n;
            l1c /= n;
            if (sideways) {
                EXPECT_LT(s * l1s, -0.3 * x[0]) << "s = " << s;
                EXPECT_LT(std::abs(l1c), std::abs(l1s));
            } else {
                EXPECT_GT(l1c, 0.3 * x[0]) << "s = " << s;
                EXPECT_LT(std::abs(l1s), std::abs(l1c));
            }
        }
    }
}

// ---- Real-time rules ---------------------------------------------------------------

TEST(PittPetersRotor, DerivativesArePureAndDontAllocate) {
    const auto airfoil = test::airfoil_from(test::synthetic_airfoil_csv());
    BladeElementRotorConfig c = test::flap_rotor(airfoil, {8 * kDeg, -2 * kDeg, 1 * kDeg},
                                                 test::nonuniform_beads(7.6), 0.4);
    c.inflow = InflowModel::pitt_peters;
    const BladeElementRotor rotor(c);
    SimContext ctx = test::sea_level_context(0.3);
    ctx.body.vel_b = Vec3(50.0, 0.0, 2.0);
    std::vector<double> x{0.05, 0.1, 0.06, -0.2, 0.04, 0.3, 0.07, 0.0, 0.05, 0.01, 0.02};
    std::vector<double> xd1(11);
    std::vector<double> G1(66);
    std::vector<double> xd2(11);
    std::vector<double> G2(66);
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

}  // namespace
}  // namespace eyas
