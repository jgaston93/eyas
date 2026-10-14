// M6b acceptance tests for the airframe's wake at the tail rotor (GenHel
// CR-166309 p. 5.4-5; D-028):
// - layer 1: the tail rotor with the fuselage's dynamic-pressure factor and
//   delayed sidewash and downwash against GenHel §5.4.2 (the M6a
//   transcription with K_QTR, tests/support/genhel_tail_rotor.hpp) and the
//   wake maps (tests/support/genhel_airframe.hpp);
// - the lag-state rates;
// - end to end: main rotor, airframe and tail rotor through the coupling
//   store, in any config order, pure and allocation-free.

#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "airframe/airframe_aero.hpp"
#include "core/rigid_body.hpp"
#include "env/atmosphere.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/rotor_wash.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/genhel_airframe.hpp"
#include "support/genhel_tail_rotor.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m6_fixtures.hpp"
#include "support/m6b_fixtures.hpp"
#include "util/config.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kDegree;
using units::kFoot;
using units::kKnot;
using units::kPoundForce;

constexpr double kSlugPerFt3 = units::kSlug / (kFoot * kFoot * kFoot);

const test::GenHelAirframeTables& genhel_tables() {
    static const test::GenHelAirframeTables t =
        test::read_genhel_airframe_tables(test::uh60a_genhel_dir());
    return t;
}

BaileyRotorConfig tail_with_wake() {
    BaileyRotorConfig c = test::bailey_from_genhel(test::genhel_uh60a_tail_rotor());
    c.fuselage_wake = test::uh60a_tail_rotor_wake();
    return c;
}

// Layer 1: with its inflow state at GenHel's converged downwash, the tail
// rotor gives GenHel's loads with the fuselage wake, steady or delayed.
TEST(TailRotorWake, MatchesGenHel) {
    std::mt19937_64 rng(6201);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    const BaileyRotor rotor(tail_with_wake());
    for (int i = 0; i < 300; ++i) {
        test::GenHelTailRotorInput in = test::genhel_uh60a_tail_rotor();
        in.VXB = 150.0 * u(rng);  // forward and rearward
        in.VYB = 50.0 * u(rng);
        in.VZB = 40.0 * u(rng);
        in.P = 0.6 * u(rng);
        in.Q = 0.6 * u(rng);
        in.R = 0.6 * u(rng);
        in.THETTR = 8.0 + 12.0 * u(rng);
        in.VKT = std::sqrt(in.VXB * in.VXB + in.VYB * in.VYB + in.VZB * in.VZB) * kFoot / kKnot;
        // The airframe's angles and the delayed wake (deg); half the cases at
        // the steady state.
        const double alfwf = 50.0 * u(rng);
        const double psiwf = 60.0 * u(rng);
        const bool steady = i % 2 == 0;
        const double nan = std::numeric_limits<double>::quiet_NaN();
        const double sig_d = steady ? nan : 2.0 * u(rng);
        const double eps_d = steady ? nan : 2.0 * u(rng);
        const test::GenHelTailRotorWake w =
            test::genhel_tail_rotor_wake(genhel_tables(), alfwf, psiwf, in.VXB, sig_d, eps_d);
        // Main-rotor wash (any values: the tail rotor only adds it).
        const double vxmr = 15.0 * u(rng);
        const double vzmr = 15.0 * u(rng);
        in.KQTR = w.KQTR;
        in.VXITR = vxmr;
        in.VYITR = w.VYWFTR;
        in.VZITR = vzmr + w.VZWFTR;
        const test::GenHelTailRotorOutput ref = test::genhel_tail_rotor(in);
        ASSERT_TRUE(ref.converged) << "case " << i;

        SimContext ctx;
        ctx.body.vel_b = Vec3(in.VXB, in.VYB, in.VZB) * kFoot;
        ctx.body.omega_b = Vec3(in.P, in.Q, in.R);
        ctx.air.density = in.RHO * kSlugPerFt3;
        ctx.controls.pedal = in.THETTR * kDegree;
        FuselageWakeInput wake;
        wake.alpha_wf = alfwf * kDegree;
        wake.psi_wf = psiwf * kDegree;
        wake.sidewash = (steady ? w.SIGTR : sig_d) * kDegree;
        wake.downwash = (steady ? w.EPSTR : eps_d) * kDegree;
        const BaileyRotorLoads p =
            rotor.evaluate(ctx, ref.DWSHTR, Vec3(vxmr, 0.0, vzmr) * kFoot, &wake);

        const double scale = std::max(std::abs(ref.TTR), 50.0) * kPoundForce;
        const double arm = 32.0 * kFoot;
        EXPECT_NEAR(p.body.F.y(), ref.YTR * kPoundForce, 1e-10 * scale) << "case " << i;
        EXPECT_NEAR(p.body.F.z(), ref.ZTR * kPoundForce, 1e-10 * scale) << "case " << i;
        EXPECT_NEAR(p.body.M.x(), ref.LTR * kPoundForce * kFoot, 1e-10 * scale * arm);
        EXPECT_NEAR(p.body.M.y(), ref.MTR * kPoundForce * kFoot, 1e-10 * scale * arm);
        EXPECT_NEAR(p.body.M.z(), ref.NTR * kPoundForce * kFoot, 1e-10 * scale * arm);
        EXPECT_NEAR(p.thrust, ref.TTR * kPoundForce, 1e-10 * scale) << "case " << i;
        EXPECT_NEAR(p.q_factor, w.KQTR, 1e-14) << "case " << i;
        EXPECT_NEAR(p.sidewash, w.SIGTR * kDegree, 1e-14) << "case " << i;
        EXPECT_NEAR(p.downwash, w.EPSTR * kDegree, 1e-14) << "case " << i;

        // D-028 lag: dσ_d/dt = (|u|/l)(σ − σ_d), l the hub's distance aft.
        const double rate = std::abs(ctx.body.vel_b.x()) / -rotor.config().hub_position_b.x();
        EXPECT_NEAR(p.sidewash_rate, rate * (p.sidewash - wake.sidewash), 1e-12);
        EXPECT_NEAR(p.downwash_rate, rate * (p.downwash - wake.downwash), 1e-12);
    }
}

// Without a wake the tail rotor is M6a's, state for state.
TEST(TailRotorWake, NoWakeIsM6a) {
    const BaileyRotor m6a(test::bailey_from_genhel(test::genhel_uh60a_tail_rotor()));
    const BaileyRotor m6b(tail_with_wake());
    EXPECT_EQ(m6a.num_states(), 1U);
    EXPECT_EQ(m6b.num_states(), 3U);
    // With the airframe at zero angles and a zero wake, K_Q = √0.62 differs
    // from 1, so the two differ; with K = 1 tables they agree.
    BaileyRotorConfig c = tail_with_wake();
    c.fuselage_wake->q_ratio = Table1D(Axis({0.0, 1.0}), {1.0, 1.0});
    c.fuselage_wake->sidewash = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
    c.fuselage_wake->downwash = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
    const BaileyRotor unit(std::move(c));
    SimContext ctx;
    ctx.body.vel_b = Vec3(30.0, 4.0, -2.0);
    ctx.body.omega_b = Vec3(0.1, 0.2, -0.1);
    ctx.air.density = 1.2;
    ctx.controls.pedal = 12.0 * kDegree;
    const FuselageWakeInput zero;
    const BaileyRotorLoads a = m6a.evaluate(ctx, 0.04, Vec3(1.0, 0.0, -3.0));
    const BaileyRotorLoads b = unit.evaluate(ctx, 0.04, Vec3(1.0, 0.0, -3.0), &zero);
    EXPECT_NEAR((a.body.F - b.body.F).norm(), 0.0, 1e-12 * a.body.F.norm());
    EXPECT_EQ(a.lambda_dot, b.lambda_dot);
}

// End to end: the tail rotor reads the airframe's tunnel angles and the main
// rotor's outputs from the same evaluation, whatever the config order.
TEST(TailRotorWake, ThroughTheExecutive) {
    BladeElementRotorConfig m =
        test::test_rotor(test::airfoil_from(test::linear_airfoil_csv(5.73, 0.01)),
                         SwashplatePitch{8.0 * kDegree, 0.0, 0.0}, 12);
    m.name = "main_rotor";
    m.inflow = InflowModel::pitt_peters;
    m.hub_position_b = Vec3(0.0, 0.0, -1.5);
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = 700.1;
    g.WLH1 = 244.0;
    g.SAH1 = 45.0;
    g.FSV1 = 695.0;
    g.WLV1 = 273.0;
    g.SAV1 = 32.3;
    BaileyRotorConfig t = tail_with_wake();
    t.wash = test::uh60a_wash("tail_wash.csv");
    std::vector<std::unique_ptr<DynamicComponent>> v;
    v.push_back(std::make_unique<BaileyRotor>(std::move(t)));              // 0: tail
    v.push_back(std::make_unique<AirframeAero>(test::uh60a_airframe(g)));  // 1: airframe
    v.push_back(std::make_unique<BladeElementRotor>(std::move(m)));        // 2: main
    const auto* tail = dynamic_cast<const BaileyRotor*>(v[0].get());
    const auto* aero = dynamic_cast<const AirframeAero*>(v[1].get());
    const auto* main = dynamic_cast<const BladeElementRotor*>(v[2].get());
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(7000.0, 6000.0, 50000.0, 48000.0, 2500.0);
    Executive ex(cfg, std::move(v));
    const std::span<const std::size_t> order = ex.evaluation_order();
    ASSERT_EQ(std::vector<std::size_t>(order.begin(), order.end()),
              (std::vector<std::size_t>{2, 1, 0}));

    BodyState b;
    b.vel_b = Vec3(35.0, -4.0, 2.0);
    b.omega_b = Vec3(0.05, -0.02, 0.1);
    ex.set_body_state(b);
    (void)ex.advance(Controls{}, Environment{}, 300'000'000);

    // Every input to the tail rotor computed here, independently of the store.
    const std::span<const double> x = ex.state();
    const std::size_t i_tail = body_layout::kSize;
    const std::size_t i_aero = i_tail + 3;
    const std::size_t i_main = i_aero + 2;
    const std::span<const double> xm = x.subspan(i_main, main->num_states());
    SimContext ctx;
    ctx.t = ex.sim_time();
    ctx.body = ex.body_state();
    ctx.air = isa_atmosphere(-ctx.body.pos_n.z(), 0.0);
    const RotorLoads ml = main->evaluate(ctx, xm);
    const double lambda0 = xm[main->inflow_offset()];
    const Vec3 v_hub_h =
        main->C_hb() * (ctx.body.vel_b + ctx.body.omega_b.cross(main->config().hub_position_b));
    const double mu_x = v_hub_h.x() / main->tip_speed();
    const double beta_1c = 0.0;  // rigid main rotor (no flap hinge): no flapping
    const RotorWash fus = test::uh60a_wash("fuselage_wash.csv");
    const RotorWash tw = test::uh60a_wash("tail_wash.csv");
    const Vec3 wf = fus.evaluate(lambda0, mu_x, ml.mu_z, beta_1c, main->tip_speed()).velocity_b;
    const Vec3 wt = tw.evaluate(lambda0, mu_x, ml.mu_z, beta_1c, main->tip_speed()).velocity_b;
    const AirframeAeroLoads al = aero->evaluate(ctx, x.subspan(i_aero, 2), wf, wt);
    FuselageWakeInput wake;
    wake.alpha_wf = al.alpha_wf;
    wake.psi_wf = al.psi_wf;
    wake.sidewash = x[i_tail + 1];
    wake.downwash = x[i_tail + 2];
    const BaileyRotorLoads expect = tail->evaluate(ctx, x[i_tail], wt, &wake);
    EXPECT_GT(std::abs(al.psi_wf), 1.0 * kDegree);

    std::vector<double> xdot(ex.num_states());
    ex.derivatives(ctx.t, x, Controls{}, Environment{}, xdot);
    EXPECT_NEAR(xdot[i_tail], expect.lambda_dot, 1e-9 * std::abs(expect.lambda_dot) + 1e-12);
    EXPECT_NEAR(xdot[i_tail + 1], expect.sidewash_rate,
                1e-9 * std::abs(expect.sidewash_rate) + 1e-12);
    EXPECT_NEAR(xdot[i_tail + 2], expect.downwash_rate,
                1e-9 * std::abs(expect.downwash_rate) + 1e-12);
    EXPECT_NE(x[i_tail + 1], 0.0);  // the wake states moved

    std::vector<double> again(ex.num_states());
    ex.derivatives(ctx.t, x, Controls{}, Environment{}, again);
    EXPECT_EQ(xdot, again);
    const test::AllocationCounter count;
    (void)ex.advance(Controls{}, Environment{}, 16'666'667);
    EXPECT_EQ(count.count(), 0U);
}

TEST(TailRotorWake, ReadsConfig) {
    const std::string dir = test::uh60a_airframe_dir();
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
hub: {position: {x: "-376.1 in", y: "14 in", z: "-76.5 in"}, roll: "70 deg"}
controls: {theta0: {pedal: "1 rad"}}
fuselage_wake:
  airframe: airframe
  q_ratio: )" + dir + R"(/fin_q_ratio.csv
  sidewash: )" + dir + R"(/fin_sidewash.csv
  downwash: )" + dir + R"(/stabilator_downwash.csv
)");
    const std::unique_ptr<BaileyRotor> r = BaileyRotor::from_config(cfg);
    EXPECT_EQ(r->num_states(), 3U);
    EXPECT_EQ(r->consumes(), (std::vector<std::string>{"airframe.alpha_wf", "airframe.psi_wf"}));
    EXPECT_TRUE(cfg.placeholders().empty());

    // A wake needs the hub aft of the CG (its delay is l/u).
    BaileyRotorConfig c = tail_with_wake();
    c.hub_position_b.x() = 0.5;
    EXPECT_THROW(BaileyRotor{std::move(c)}, InitError);
    // A dynamic-pressure ratio below zero has no square root (K_QTR = √ratio).
    c = tail_with_wake();
    c.fuselage_wake->q_ratio = Table1D(Axis({0.0, 1.0}), {1.0, -0.01});
    EXPECT_THROW(BaileyRotor{std::move(c)}, InitError);
}

}  // namespace
}  // namespace eyas
