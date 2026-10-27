// M6a acceptance tests for the closed-form (Bailey) tail rotor (D-028):
// - layer 1: production against a literal transcription of GenHel §5.4.2
//   iterated to its steady state (tests/support/genhel_tail_rotor.hpp);
// - layer 2: the Bailey thrust against the production blade-element rotor
//   (M3 code), and the hover inflow eigenvalue in closed form;
// - the coupling path from a main rotor (wash maps) and the config reader.

#include <cmath>
#include <cstring>
#include <memory>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/rigid_body.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/hub_frame.hpp"
#include "rotor/pitt_peters.hpp"
#include "rotor/rotor_wash.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/bailey_torque_reference.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m6_fixtures.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using test::bailey_ct;
using test::bare;
using test::blade_element_ct;
using test::thrust_integrals;
using test::ThrustIntegrals;
using units::kDegree;
using units::kFoot;
using units::kKnot;
using units::kPoundForce;

constexpr double kSlugPerFt3 = units::kSlug / (kFoot * kFoot * kFoot);

SimContext context_from(const test::GenHelTailRotorInput& in) {
    SimContext ctx;
    ctx.body.vel_b = Vec3(in.VXB, in.VYB, in.VZB) * kFoot;
    ctx.body.omega_b = Vec3(in.P, in.Q, in.R);
    ctx.air.density = in.RHO * kSlugPerFt3;
    ctx.air.speed_of_sound = 340.0;
    return ctx;
}

// Layer 1 (D-028): with its inflow state at GenHel's converged downwash,
// production gives GenHel's loads, pitch and thrust, and zero dλ/dt.
TEST(BaileyRotor, MatchesGenHelSteadyState) {
    std::mt19937_64 rng(6001);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int i = 0; i < 300; ++i) {
        test::GenHelTailRotorInput in = test::genhel_uh60a_tail_rotor();
        const bool drag = i % 3 != 0;  // GenHel's V² drag is compared only forward
        in.VXB = drag ? 80.0 + 85.0 * (1.0 + u(rng)) : 150.0 * u(rng);  // V_XTR > 0 with drag
        in.VYB = 50.0 * u(rng);
        in.VZB = 40.0 * u(rng);
        in.P = 0.6 * u(rng);
        in.Q = 0.6 * u(rng);
        in.R = 0.6 * u(rng);
        in.THETTR = 8.0 + 12.0 * u(rng);
        in.CDTR = drag ? 1.5 : 0.0;
        in.VXITR = 15.0 * u(rng);
        in.VYITR = 0.0;
        in.VZITR = 15.0 * u(rng);
        in.VKT = std::sqrt(in.VXB * in.VXB + in.VYB * in.VYB + in.VZB * in.VZB) * kFoot / kKnot;
        const test::GenHelTailRotorOutput ref = test::genhel_tail_rotor(in);
        ASSERT_TRUE(ref.converged) << "case " << i;

        const BaileyRotor rotor(test::bailey_from_genhel(in));
        SimContext ctx = context_from(in);
        ctx.controls.pedal = in.THETTR * kDegree;
        const Vec3 wash = Vec3(in.VXITR, in.VYITR, in.VZITR) * kFoot;
        const BaileyRotorLoads p = rotor.evaluate(ctx, ref.DWSHTR, wash);

        const double scale = std::max(std::abs(ref.TTR), 50.0) * kPoundForce;  // [N]
        const double arm = 32.0 * kFoot;
        EXPECT_NEAR(p.body.F.x(), ref.XTR * kPoundForce, 1e-10 * scale) << "case " << i;
        EXPECT_NEAR(p.body.F.y(), ref.YTR * kPoundForce, 1e-10 * scale) << "case " << i;
        EXPECT_NEAR(p.body.F.z(), ref.ZTR * kPoundForce, 1e-10 * scale) << "case " << i;
        EXPECT_NEAR(p.body.M.x(), ref.LTR * kPoundForce * kFoot, 1e-10 * scale * arm);
        EXPECT_NEAR(p.body.M.y(), ref.MTR * kPoundForce * kFoot, 1e-10 * scale * arm);
        EXPECT_NEAR(p.body.M.z(), ref.NTR * kPoundForce * kFoot, 1e-10 * scale * arm);
        EXPECT_NEAR(p.thrust, ref.TTR * kPoundForce, 1e-10 * scale);
        EXPECT_NEAR(p.pitch, ref.THETA, 1e-12);
        EXPECT_NEAR(p.mu_z, ref.MUZTR, 1e-14);
        EXPECT_NEAR(p.mu, ref.MUTR, 1e-14);
        // dλ/dt = (Ω/M₀)(C_T − 2λV): zero at GenHel's fixed point.
        const double rate_scale =
            rotor.config().rotor_speed * std::max(std::abs(p.ct), 1e-3) / PittPetersMass::m0;
        EXPECT_NEAR(p.lambda_dot, 0.0, 1e-10 * rate_scale) << "case " << i;
    }
}

// GenHel's sign conventions, checked once by hand: in hover the UH-60A tail
// rotor pushes to starboard and slightly up, and its yaw moment is nose-left
// (against the counter-clockwise main rotor's torque reaction).
TEST(BaileyRotor, HoverThrustDirectionsMatchTheAircraft) {
    test::GenHelTailRotorInput in = test::genhel_uh60a_tail_rotor();
    const BaileyRotor rotor(test::bailey_from_genhel(in));
    SimContext ctx = context_from(in);
    ctx.controls.pedal = 10.0 * kDegree;
    const test::GenHelTailRotorOutput ref = test::genhel_tail_rotor([&] {
        in.THETTR = 10.0;
        return in;
    }());
    const BaileyRotorLoads p = rotor.evaluate(ctx, ref.DWSHTR, Vec3::Zero());
    EXPECT_GT(p.thrust, 0.0);
    EXPECT_GT(p.body.F.y(), 0.0);  // to starboard
    EXPECT_LT(p.body.F.z(), 0.0);  // up: 20° cant
    EXPECT_NEAR(p.body.F.z() / p.body.F.y(), -std::tan(20.0 * kDegree), 1e-12);
    EXPECT_LT(p.body.M.z(), 0.0);                       // nose left
    EXPECT_GT(rotor.config().hub_position_b.y(), 0.0);  // hub to starboard (BL −14 in)
}

// Layer 2 (D-028): GenHel's Bailey coefficients (p. 5.4-7) are the
// small-angle thrust integrals out to B·R with reverse flow kept on the
// inflow term only: t₃₁ = E[∫|U_T|], t₃₂ = E[∫U_T²], t₃₃ = E[∫rU_T²]
// (Johnson 2013 Eq. 6.219 at B = 1; Johnson puts B on the μ² terms
// differently).
TEST(BaileyRotor, CoefficientsAreSmallAngleIntegrals) {
    for (const double B : {1.0, 0.92}) {
        for (const double mu : {0.0, 0.1, 0.2, 0.3}) {
            const ThrustIntegrals t = thrust_integrals(B, mu, false);
            EXPECT_NEAR(t.inflow, B * B / 2.0 + mu * mu / 4.0, 2e-6) << B << " " << mu;
            EXPECT_NEAR(t.theta, B * B * B / 3.0 + B * mu * mu / 2.0, 2e-6);
            EXPECT_NEAR(t.twist, std::pow(B, 4) / 4.0 + B * B * mu * mu / 4.0, 2e-6);
            // Production C_T is (aσ/2) times the same combination.
            const BaileyRotorConfig b = bare(B, -18.0 * kDegree);
            const double th = 15.0 * kDegree;
            const double lam_i = 0.04;
            const double G = b.lift_slope * BaileyRotor(b).solidity() / 2.0;
            const double expect = G * (th * t.theta + b.twist * t.twist - lam_i * t.inflow);
            EXPECT_NEAR(bailey_ct(b, th, lam_i, mu), expect, 1e-6 * G);
        }
    }
}

// Layer 2 (D-028): the production blade-element rotor (M3 code, exact
// angles) against the small-angle integrals with reverse flow on every
// term. What remains is the small-angle error, which grows as λ²; the
// tolerance G λ²(θ + λ) bounds it; the measured residuals are in
// VALIDATION.md (M6a). Bailey drops
// the reverse-flow loss on the pitch terms, 4μ³/(9π) θ₀ at B = 1, which is
// reported there, not gated.
TEST(BaileyRotor, BladeElementMatchesSmallAngleIntegrals) {
    for (const double B : {1.0, 0.92}) {
        for (const double tw : {0.0, -18.0}) {
            for (const double mu : {0.0, 0.1, 0.2, 0.3}) {
                for (const double lam_i : {0.02, 0.05}) {
                    const BaileyRotorConfig b = bare(B, tw * kDegree);
                    const double th = 20.0 * kDegree;
                    const ThrustIntegrals t = thrust_integrals(B, mu, true);
                    const double G = b.lift_slope * BaileyRotor(b).solidity() / 2.0;
                    const double expect = G * (th * t.theta + b.twist * t.twist - lam_i * t.inflow);
                    const double be = blade_element_ct(b, th, lam_i, mu);
                    // Small-angle error: U² = U_T² + U_P² and cos φ, O(θλ²).
                    const double tol = G * lam_i * lam_i * (th + lam_i);
                    EXPECT_NEAR(be, expect, tol)
                        << "B " << B << " tw " << tw << " mu " << mu << " lambda " << lam_i;
                }
            }
        }
    }
}

// Layer 2: hover inflow eigenvalue. With dC_T/dλ_i = −G t₃₁/(1 + G t₃₂ k)
// (k the δ₃ coupling) and d(2λ_i V)/dλ_i = 4λ_i in hover,
//   s = −(Ω/M₀)(4λ_i + G t₃₁/(1 + G t₃₂ k)),
// which at B = 1, δ₃ = 0 is the D-027 coupled hover root −(Ω/M₀)(4λ₀ + σa/4).
TEST(BaileyRotor, HoverInflowEigenvalue) {
    for (const bool with_delta3 : {false, true}) {
        test::GenHelTailRotorInput in = test::genhel_uh60a_tail_rotor();
        BaileyRotorConfig c = test::bailey_from_genhel(in);
        c.blockage_speed = 0.0;
        if (!with_delta3) {
            c.delta3 = 0.0;
        }
        const BaileyRotor rotor(c);
        SimContext ctx;
        ctx.air.density = in.RHO * kSlugPerFt3;
        ctx.controls.pedal = 10.0 * kDegree;
        in.THETTR = 10.0;
        in.KBVTTR = 1.0;
        in.DEL3TR = with_delta3 ? in.DEL3TR : 0.0;
        const double l0 = test::genhel_tail_rotor(in).DWSHTR;
        ASSERT_NEAR(rotor.evaluate(ctx, l0, Vec3::Zero()).lambda_dot, 0.0, 1e-9);

        const double h = 1e-7;
        const double s = (rotor.evaluate(ctx, l0 + h, Vec3::Zero()).lambda_dot -
                          rotor.evaluate(ctx, l0 - h, Vec3::Zero()).lambda_dot) /
                         (2.0 * h);
        const double B = c.tip_loss;
        const double G = c.lift_slope * rotor.solidity() / 2.0;
        const double t31 = B * B / 2.0;
        const double t32 = B * B * B / 3.0;
        const double q_area =
            ctx.air.density * rotor.disk_area() * rotor.tip_speed() * rotor.tip_speed();
        const double k = c.coning_per_thrust * std::tan(c.delta3) * q_area;
        const double expected =
            -(c.rotor_speed / PittPetersMass::m0) * (4.0 * l0 + G * t31 / (1.0 + G * t32 * k));
        EXPECT_NEAR(s / expected, 1.0, 1e-6) << "delta3 " << with_delta3;
    }
}

// GenHel's wake skew and flapping (pp. 5.1-37, 5.1-18), written in degrees
// as on the page, against RotorWash; and the wash velocity components.
TEST(RotorWash, GenHelWakeSkewAndWashVelocity) {
    const CsvTable csv = parse_csv_table(test::synthetic_wash_csv(), "wash");
    const RotorWash wash("main_rotor", make_multi_table2d<2>(csv, "chi", "a1", {"ekx", "ekz"}));
    std::mt19937_64 rng(6002);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int i = 0; i < 200; ++i) {
        const double lambda0 = 0.05 + 0.04 * u(rng);
        const double mu_x = 0.2 * u(rng);
        const double mu_z = 0.05 * u(rng);
        const double beta_1c = 0.08 * u(rng);
        const double tip = 220.0;
        const RotorWash::Result r = wash.evaluate(lambda0, mu_x, mu_z, beta_1c, tip);
        // GenHel: A1FMR = −2·57.3/b Σ β cos ψ = −57.3 β_1c [deg];
        // CHIPMR = atan(μ_xs/|λ|) + AA1FMR with λ = μ_z − D_W (GenHel's sign).
        const double a1_deg = -beta_1c / kDegree;
        const double lam = mu_z - lambda0;
        const double chi_deg = std::atan(mu_x / std::abs(lam)) / kDegree + a1_deg;
        EXPECT_NEAR(r.a1 / kDegree, a1_deg, 1e-12);
        EXPECT_NEAR(r.chi / kDegree, chi_deg, 1e-11);
        // Synthetic map, linear in both axes inside the grid: exact.
        const double ekx = 0.01 * chi_deg - 0.03 * a1_deg;
        const double ekz = 1.2 - 0.004 * chi_deg + 0.02 * a1_deg;
        if (chi_deg >= 0.0 && chi_deg <= 100.0 && std::abs(a1_deg) <= 6.0) {
            EXPECT_NEAR(r.velocity_b.x(), lambda0 * tip * ekx, 1e-6);
            EXPECT_NEAR(r.velocity_b.z(), -lambda0 * tip * ekz, 1e-6);
        }
        EXPECT_EQ(r.velocity_b.y(), 0.0);
    }
}

std::vector<std::unique_ptr<DynamicComponent>> main_and_tail(const std::string& wash_file) {
    BladeElementRotorConfig m =
        test::test_rotor(test::airfoil_from(test::linear_airfoil_csv(5.73, 0.01)),
                         SwashplatePitch{8.0 * kDegree, 0.0, 0.0}, 12);
    m.name = "main_rotor";
    m.inflow = InflowModel::pitt_peters;
    m.hub_position_b = Vec3(0.0, 0.0, -1.5);
    FlapHinge f;
    f.offset = 0.38;
    f.mass = 116.0;
    f.first_moment = 400.0;
    f.inertia = 2050.0;
    m.flap = f;
    BaileyRotorConfig t = test::bailey_from_genhel(test::genhel_uh60a_tail_rotor());
    const CsvTable csv = parse_csv_table(wash_file, "wash");
    t.wash = RotorWash("main_rotor", make_multi_table2d<2>(csv, "chi", "a1", {"ekx", "ekz"}));
    std::vector<std::unique_ptr<DynamicComponent>> v;
    v.push_back(std::make_unique<BaileyRotor>(std::move(t)));  // tail first in config
    v.push_back(std::make_unique<BladeElementRotor>(std::move(m)));
    return v;
}

ExecutiveConfig executive_config() {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(7000.0, 6000.0, 50000.0, 48000.0, 2500.0);
    return cfg;
}

// End to end: the tail rotor consumes the main rotor's outputs from the same
// evaluation, through the coupling store, whatever the config order.
TEST(BaileyRotor, ConsumesMainRotorWashThroughTheExecutive) {
    auto components = main_and_tail(test::synthetic_wash_csv());
    const auto* tail = dynamic_cast<const BaileyRotor*>(components[0].get());
    const auto* main = dynamic_cast<const BladeElementRotor*>(components[1].get());
    Executive ex(executive_config(), std::move(components));
    ASSERT_EQ(ex.evaluation_order()[0], 1U);  // main rotor first

    BodyState b;
    b.vel_b = Vec3(30.0, 2.0, 1.0);
    b.omega_b = Vec3(0.05, -0.02, 0.1);
    ex.set_body_state(b);
    (void)ex.advance(Controls{}, Environment{}, 200'000'000);  // build up inflow and flapping

    const std::span<const double> x = ex.state();
    const std::size_t i_tail = body_layout::kSize;
    const std::size_t i_main = i_tail + 1;
    const std::span<const double> xm = x.subspan(i_main, main->num_states());
    SimContext ctx;
    ctx.t = ex.sim_time();
    ctx.body = ex.body_state();
    ctx.air = isa_atmosphere(-ctx.body.pos_n.z(), 0.0);
    // The main rotor's outputs, computed here independently of the store.
    const RotorLoads ml = main->evaluate(ctx, xm);
    const double lambda0 = xm[main->inflow_offset()];
    const double psi = main->azimuth(ctx.t);
    double beta_1c = 0.0;
    for (int k = 0; k < 4; ++k) {
        beta_1c +=
            0.5 * xm[2 * static_cast<std::size_t>(k)] * std::cos(main->blade_azimuth(psi, k));
    }
    const Vec3 v_hub_h =
        main->C_hb() * (ctx.body.vel_b + ctx.body.omega_b.cross(main->config().hub_position_b));
    const RotorWash::Result w = tail->config().wash->evaluate(
        lambda0, v_hub_h.x() / main->tip_speed(), ml.mu_z, beta_1c, main->tip_speed());
    const BaileyRotorLoads expect = tail->evaluate(ctx, x[i_tail], w.velocity_b);
    EXPECT_GT(w.velocity_b.norm(), 1.0);

    std::vector<double> xdot(ex.num_states());
    EXPECT_TRUE(ex.derivatives(ctx.t, x, Controls{}, Environment{}, xdot));
    EXPECT_NEAR(xdot[i_tail], expect.lambda_dot, 1e-9 * std::abs(expect.lambda_dot) + 1e-12);

    // And no allocation while advancing.
    const test::AllocationCounter count;
    (void)ex.advance(Controls{}, Environment{}, 16'666'667);
    EXPECT_EQ(count.count(), 0U);
}

// ---- Torque (D-058) ----------------------------------------------------------

// The UH-60A tail rotor's Bailey torque constants (Ballin, NASA TM-88360
// p. 26: Bailey's NACA 23012 polar, NACA Report 716 p. 217, and IB_TR).
BaileyTorque ballin_torque(RotationDirection dir = RotationDirection::counter_clockwise) {
    return BaileyTorque{dir, 3.1 * units::kSlug * kFoot * kFoot, 0.0087, -0.0216, 0.4};
}

// A bare rotor (hub frame = body axes) at advance ratio μ and inflow λ
// (positive up through the disk), its pitch θ through the pedal; λ_i fixed.
struct TorqueCase {
    BaileyRotorLoads loads;
    double lambda_i = 0.03;
};
TorqueCase evaluate_at(const BaileyRotor& rotor, double mu, double lambda, double theta) {
    SimContext ctx;
    ctx.air.density = 1.2;
    TorqueCase t;
    ctx.body.vel_b = Vec3(mu, 0.0, lambda + t.lambda_i) * rotor.tip_speed();
    ctx.controls.pedal = theta;
    t.loads = rotor.evaluate(ctx, t.lambda_i, Vec3::Zero());
    return t;
}

// Layer 1 (D-058): the closed-form torque against Bailey's assumptions
// integrated numerically (tests/support/bailey_torque_reference.hpp), with the
// reversed-velocity region and the flapping kept whole. The closed form keeps
// terms to μ², so they agree exactly in hover and to O(μ³) in forward flight;
// the bound 0.5 σμ³ holds with margin (measured at most 0.34 σμ³ over these
// cases, to μ = 0.35), and at the smallest μ a wrong μ² coefficient exceeds
// it (μ down to 0.002 for the small ones, such as δ₁θ_tw's). Inflows from
// −0.15 to +0.05, Lock numbers about 3 and 10, Ballin's polar and a
// synthetic stronger one.
TEST(BaileyRotor, TorqueMatchesBaileysAssumptionsIntegrated) {
    struct Polar {
        double d0, d1, d2;
    };
    for (const double B : {1.0, 0.92}) {
        for (const double tw : {0.0, -18.0 * kDegree}) {
            for (const double inertia : {4.2, 1.4}) {
                for (const Polar p : {Polar{0.0087, -0.0216, 0.4}, Polar{0.01, 0.3, 2.0}}) {
                    BaileyRotorConfig c = bare(B, tw);
                    c.torque = BaileyTorque{RotationDirection::counter_clockwise, inertia, p.d0,
                                            p.d1, p.d2};
                    const BaileyRotor rotor(c);
                    const double sigma = rotor.solidity();
                    const double gamma =
                        1.2 * c.lift_slope * c.chord * std::pow(c.radius, 4) / inertia;
                    for (const double th : {5.0 * kDegree, 20.0 * kDegree}) {
                        for (const double lam : {-0.15, -0.06, 0.05}) {
                            for (const double mu :
                                 {0.0, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.35}) {
                                const TorqueCase t = evaluate_at(rotor, mu, lam, th);
                                const test::BaileyTorqueCase ref_case{B,
                                                                      gamma,
                                                                      t.loads.mu,
                                                                      t.loads.mu_z - t.lambda_i,
                                                                      t.loads.pitch,
                                                                      tw,
                                                                      p.d0,
                                                                      p.d1,
                                                                      p.d2};
                                const double ref =
                                    test::bailey_torque_reference(ref_case).cq(sigma, c.lift_slope);
                                const double tol = sigma * (0.5 * mu * mu * mu + 1e-14);
                                EXPECT_NEAR(t.loads.cq, ref, tol)
                                    << "B " << B << " tw " << tw << " I1 " << inertia << " d1 "
                                    << p.d1 << " th " << th << " lam " << lam << " mu " << mu;
                            }
                        }
                    }
                }
            }
        }
    }
}

// Layer 2 (D-058): in hover the torque is the induced power λ_J C_T plus
// Johnson's profile power of a constant-chord, linearly twisted blade in
// uniform inflow for the same polar (Johnson 2013 Eq. 3.90, p. 58: θ.75 the
// pitch at 0.75R, λ_J the inflow positive down), independent of Bailey.
TEST(BaileyRotor, HoverTorqueIsInducedPlusJohnsonsProfilePower) {
    for (const double tw : {0.0, -18.0 * kDegree}) {
        BaileyRotorConfig c = bare(1.0, tw);
        c.torque = ballin_torque();
        const BaileyTorque& q = *c.torque;
        const BaileyRotor rotor(c);
        const double sigma = rotor.solidity();
        for (const double th : {2.0 * kDegree, 12.0 * kDegree, 25.0 * kDegree}) {
            for (const double lam : {-0.08, -0.03, 0.02}) {
                const BaileyRotorLoads l = evaluate_at(rotor, 0.0, lam, th).loads;
                const double lj = -lam;
                const double t75 = th + 0.75 * tw;
                const double cpo = sigma * q.d0 / 8.0 +
                                   sigma * q.d1 / 8.0 * (t75 + tw / 20.0 - 4.0 / 3.0 * lj) +
                                   sigma * q.d2 / 8.0 *
                                       (t75 * t75 + t75 * tw / 10.0 + 7.0 / 240.0 * tw * tw +
                                        2.0 * lj * lj - 8.0 / 3.0 * t75 * lj);
                EXPECT_NEAR(l.cq, lj * l.ct + cpo, 1e-15) << "tw " << tw << " th " << th;
                // Q = ρπR²(ΩR)²R C_Q.
                const double qa = 1.2 * rotor.disk_area() * rotor.tip_speed() * rotor.tip_speed();
                EXPECT_NEAR(l.torque, qa * c.radius * l.cq, 1e-12 * std::abs(l.torque));
            }
        }
    }
}

// The torque on the airframe and the shaft (D-058, D-052): the airframe gets
// −Q along the rotation axis (−s z_h, viewed from the thrust side), the
// thrust and drag unchanged; the shaft −gear × Q, and nothing without the
// torque block.
TEST(BaileyRotor, TorqueActsAgainstTheRotation) {
    test::GenHelTailRotorInput in = test::genhel_uh60a_tail_rotor();
    BaileyRotorConfig plain = test::bailey_from_genhel(in);
    plain.shaft = ShaftLink{"drive", 124.62 / 27.0};
    SimContext ctx = context_from(in);
    ctx.body.vel_b = Vec3(40.0, 3.0, -2.0);
    ctx.controls.pedal = 15.0 * kDegree;
    ctx.num_shafts = 1;
    ctx.shafts[0] = {0.0, 27.0};
    const std::vector<ShaftInfo> info = {{"drive", 27.0}};
    const double lambda_i = 0.02;
    BaileyRotor without(plain);
    without.bind_shafts(info);
    const BaileyRotorLoads l0 = without.evaluate(ctx, lambda_i, Vec3::Zero());
    EXPECT_EQ(l0.torque, 0.0);
    EXPECT_TRUE(without.produces().empty());

    for (const auto dir : {RotationDirection::counter_clockwise, RotationDirection::clockwise}) {
        BaileyRotorConfig c = plain;
        c.torque = ballin_torque(dir);
        BaileyRotor rotor(c);
        rotor.bind_shafts(info);
        const BaileyRotorLoads l = rotor.evaluate(ctx, lambda_i, Vec3::Zero());
        EXPECT_GT(l.thrust, 0.0);
        EXPECT_GT(l.torque, 0.0);  // a demand at this thrust
        EXPECT_EQ(l.body.F, l0.body.F);
        const double s = dir == RotationDirection::counter_clockwise ? 1.0 : -1.0;
        const Vec3 axis_b = hub_rotation(c.hub_pitch, c.hub_roll).transpose() * Vec3(0, 0, -s);
        EXPECT_LT((l.body.M - (l0.body.M - l.torque * axis_b)).norm(), 1e-12 * l.torque);
        // The UH-60A's cant: the axis is mostly along body y (the pitch axis).
        EXPECT_NEAR(std::abs(axis_b.y()), std::sin(70.0 * kDegree), 1e-12);

        std::vector<double> xdot(rotor.num_states());
        std::vector<double> g(6 * rotor.num_states());
        std::vector<double> gs(rotor.num_states() * kMaxShafts);
        ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
        d.Gs = gs;
        const std::vector<double> x = {lambda_i, 0.0};
        rotor.derivatives(ctx, x, d);
        EXPECT_DOUBLE_EQ(d.Q0(0), -c.shaft->gear * l.torque);
        EXPECT_EQ(d.Ka.norm() + d.Ks.norm(), 0.0);
        EXPECT_EQ(rotor.produces(), std::vector<std::string>{"tail_rotor.shaft_torque"});
    }
}

// End to end on a free shaft (D-052): with the tail rotor alone on it, the
// shaft decelerates at gear × Q / J.
TEST(BaileyRotor, TorqueSlowsAFreeShaft) {
    BaileyRotorConfig c = test::bailey_from_genhel(test::genhel_uh60a_tail_rotor());
    const double gear = 124.62 / 27.0;
    c.shaft = ShaftLink{"drive", gear};
    c.torque = ballin_torque();
    ExecutiveConfig ec = executive_config();
    ShaftConfig s;
    s.name = "drive";
    s.speed = 27.0;
    const double J = 900.0;
    s.parts = {ShaftPart{J, 1.0, Vec3::Zero()}};
    s.drive = ShaftDrive::free;
    ec.shafts = {s};
    std::vector<std::unique_ptr<DynamicComponent>> v;
    v.push_back(std::make_unique<BaileyRotor>(c));
    const auto* tail = dynamic_cast<const BaileyRotor*>(v.back().get());
    Executive ex(ec, std::move(v));
    BodyState b;
    b.vel_b = Vec3(30.0, 0.0, 0.0);
    ex.set_body_state(b);
    std::vector<double> x(ex.state().begin(), ex.state().end());
    x[body_layout::kSize] = 0.04;
    ASSERT_TRUE(ex.restore(x, 0, 0));
    Controls u;
    u.pedal = 0.5;
    std::vector<double> xdot(ex.num_states());
    ASSERT_TRUE(ex.derivatives(0.0, ex.state(), u, Environment{}, xdot));
    SimContext ctx;
    ctx.body = ex.body_state();
    ctx.air = isa_atmosphere(-ctx.body.pos_n.z(), 0.0);
    ctx.controls = u;
    ctx.num_shafts = 1;
    ctx.shafts[0] = {0.0, 27.0};
    const BaileyRotorLoads l = tail->evaluate(ctx, 0.04, Vec3::Zero());
    ASSERT_GT(l.torque, 0.0);
    EXPECT_NEAR(xdot[ex.shaft_offset(0) + 1], -gear * l.torque / J, 1e-12 * gear * l.torque / J);
}

TEST(BaileyRotor, ReadsConfig) {
    const Config cfg = Config::from_string(R"(
type: bailey_rotor
name: tail_rotor
blades: 4
radius: "5.5 ft"
rotor_speed: "124.62 rad/s"
chord: "0.81 ft"
lift_slope: "5.73 1/rad"
tip_loss: 0.92
twist: "-18 deg"
delta3: "35 deg"
coning_per_thrust: "0.001455 deg/lbf"
pitch_bias: "6 deg"
drag_area: "0 ft^2"
blockage: {factor: 0.796, speed: "30 kt"}
hub:
  position: {x: "-31.34 ft", y: "1.17 ft", z: "-6.375 ft"}
  roll: "70 deg"
controls:
  theta0: {pedal: "20 deg"}
)");
    const auto r = BaileyRotor::from_config(cfg);
    const BaileyRotorConfig& c = r->config();
    EXPECT_EQ(c.name, "tail_rotor");
    EXPECT_DOUBLE_EQ(c.radius, 5.5 * kFoot);
    EXPECT_DOUBLE_EQ(c.coning_per_thrust, 0.001455 * kDegree / kPoundForce);
    EXPECT_DOUBLE_EQ(c.blockage_speed, 30.0 * kKnot);
    EXPECT_DOUBLE_EQ(c.hub_roll, 70.0 * kDegree);
    EXPECT_FALSE(c.wash.has_value());
    EXPECT_TRUE(r->consumes().empty());
    EXPECT_FALSE(c.torque.has_value());

    // The torque block (D-058).
    const std::string base = R"(
type: bailey_rotor
name: tail_rotor
blades: 4
radius: "5.5 ft"
rotor_speed: "124.62 rad/s"
chord: "0.81 ft"
lift_slope: "5.73 1/rad"
tip_loss: 0.92
twist: "-18 deg"
hub: {position: {x: "0 m", y: "0 m", z: "0 m"}}
controls: {theta0: {pedal: "20 deg"}}
)";
    const auto torque_cfg = [&](const std::string& dir) {
        return Config::from_string(base + "torque:\n  direction: " + dir +
                                   "\n  blade_flap_inertia: \"3.1 slug*ft^2\"\n"
                                   "  profile_drag: {d0: 0.0087, d1: \"-0.0216 1/rad\", "
                                   "d2: \"0.4 1/rad^2\"}\n");
    };
    const auto rt = BaileyRotor::from_config(torque_cfg("clockwise"));
    ASSERT_TRUE(rt->config().torque.has_value());
    const BaileyTorque& t = *rt->config().torque;
    EXPECT_EQ(t.direction, RotationDirection::clockwise);
    EXPECT_DOUBLE_EQ(t.flap_inertia, 3.1 * units::kSlug * kFoot * kFoot);
    EXPECT_DOUBLE_EQ(t.d0, 0.0087);
    EXPECT_DOUBLE_EQ(t.d1, -0.0216);
    EXPECT_DOUBLE_EQ(t.d2, 0.4);
    EXPECT_THROW((void)BaileyRotor::from_config(torque_cfg("sideways")), InitError);

    // δ₃ without its coning rate is an error, not a silent zero (D-021).
    const Config bad = Config::from_string(R"(
type: bailey_rotor
blades: 4
radius: "5.5 ft"
rotor_speed: "124.62 rad/s"
chord: "0.81 ft"
lift_slope: "5.73 1/rad"
tip_loss: 0.92
twist: "-18 deg"
delta3: "35 deg"
hub: {position: {x: "0 m", y: "0 m", z: "0 m"}}
controls: {}
)");
    EXPECT_THROW((void)BaileyRotor::from_config(bad), InitError);
}

// δ₃ < 0 with coning lets the closed-form δ₃ denominator reach zero in
// flight (for the UH-60A tail rotor at sea level, near −73° in hover and
// −64° at μ = 0.4 unblocked), so init refuses it. Without coning, δ₃ has
// no effect and any sign is accepted.
TEST(BaileyRotor, RejectsNegativeDelta3WithConing) {
    BaileyRotorConfig c = test::bailey_from_genhel(test::genhel_uh60a_tail_rotor());
    c.delta3 = -35.0 * kDegree;
    EXPECT_THROW(BaileyRotor{c}, InitError);
    c.coning_per_thrust = 0.0;
    EXPECT_NO_THROW(BaileyRotor{c});
}

}  // namespace
}  // namespace eyas
