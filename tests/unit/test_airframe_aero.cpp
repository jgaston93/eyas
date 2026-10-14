// M6b acceptance tests for the airframe aerodynamics (D-028, D-030):
// - layer 1: production against a literal transcription of GenHel §5.2.2 and
//   §5.3.2, with the UH-60A tables and random geometry
//   (tests/support/genhel_airframe.hpp);
// - layer 2: properties that hold whatever the data: the parity GenHel gives
//   each table, mirror symmetry, drag that never adds energy, continuity
//   through V_X = 0, and the lag-state rates;
// - the data rulings of 2026-10-13: the main-rotor downwash points the same
//   way at the stabilator, fin and tail rotor;
// - the coupling outputs through the executive, purity and no allocation.

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "airframe/airframe_aero.hpp"
#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/rotor_wash.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/genhel_airframe.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m6b_fixtures.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kDegree;
using units::kFoot;
using units::kPoundForce;

constexpr double kSlugPerFt3 = units::kSlug / (kFoot * kFoot * kFoot);
constexpr double kOmegaMR = 27.0;    // GenHel UH-60A main rotor (p. 5.1-49, verified)
constexpr double kRadiusMR = 26.83;  // [ft]

const test::GenHelAirframeTables& genhel_tables() {
    static const test::GenHelAirframeTables t =
        test::read_genhel_airframe_tables(test::uh60a_genhel_dir());
    return t;
}

SimContext context(double vx, double vy, double vz, double p, double q, double r, double rho_slug) {
    SimContext ctx;
    ctx.body.vel_b = Vec3(vx, vy, vz) * kFoot;
    ctx.body.omega_b = Vec3(p, q, r);
    ctx.air.density = rho_slug * kSlugPerFt3;
    ctx.air.speed_of_sound = 340.0;
    return ctx;
}

void expect_loads(const Wrench& w, const test::GenHelLoads& ref, double scale_lbf, const char* what,
                  int i) {
    const double f = 1e-9 * std::max(scale_lbf, 1.0) * kPoundForce;
    const double m = f * 40.0 * kFoot;
    EXPECT_NEAR(w.F.x(), ref.X * kPoundForce, f) << what << " case " << i;
    EXPECT_NEAR(w.F.y(), ref.Y * kPoundForce, f) << what << " case " << i;
    EXPECT_NEAR(w.F.z(), ref.Z * kPoundForce, f) << what << " case " << i;
    EXPECT_NEAR(w.M.x(), ref.L * kPoundForce * kFoot, m) << what << " case " << i;
    EXPECT_NEAR(w.M.y(), ref.M * kPoundForce * kFoot, m) << what << " case " << i;
    EXPECT_NEAR(w.M.z(), ref.N * kPoundForce * kFoot, m) << what << " case " << i;
}

double magnitude(const test::GenHelLoads& l) {
    return std::max({std::abs(l.X), std::abs(l.Y), std::abs(l.Z)});
}

// One random case: geometry, flight state and main-rotor state, with the
// GenHel input and the production wash velocities built from the same rotor
// state.
struct Case {
    test::GenHelAirframeGeometry g{};
    test::GenHelAirframeInput in;
    SimContext ctx;
    Vec3 fuselage_wash_b = Vec3::Zero();
    Vec3 tail_wash_b = Vec3::Zero();
};

Case random_case(std::mt19937_64& rng, const RotorWash& fus, const RotorWash& tail) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Case c;
    c.g = test::random_geometry(rng);
    test::apply(c.g, c.in);
    c.in.RHO = 0.0023769 * (1.0 + 0.2 * u(rng));
    c.in.VXB = 155.0 + 95.0 * u(rng);  // 60–250 ft/s: V_X > 0 at every panel
    c.in.VYB = 60.0 * u(rng);          // |ψ_WF| up to about 45°
    c.in.VZB = 40.0 * u(rng);
    c.in.P = 0.5 * u(rng);
    c.in.Q = 0.5 * u(rng);
    c.in.R = 0.5 * u(rng);
    // Main rotor: uniform downwash, hub velocity, flapping.
    const double tip = kOmegaMR * kRadiusMR * kFoot;
    const double lambda0 = 0.04 * (1.0 + u(rng));
    const double mu_x = 0.2 * (1.0 + u(rng));
    const double mu_z = 0.05 * u(rng);
    const double beta_1c = 7.0 * kDegree * u(rng);
    const RotorWash::Result wf = fus.evaluate(lambda0, mu_x, mu_z, beta_1c, tip);
    const RotorWash::Result wt = tail.evaluate(lambda0, mu_x, mu_z, beta_1c, tip);
    c.fuselage_wash_b = wf.velocity_b;
    c.tail_wash_b = wt.velocity_b;
    c.in.DWSHMR = lambda0;
    c.in.OMGTMR = kOmegaMR;
    c.in.RMR = kRadiusMR;
    c.in.CHIPMR = wf.chi / kDegree;
    c.in.A1FMR = wf.a1 / kDegree;
    c.ctx = context(c.in.VXB, c.in.VYB, c.in.VZB, c.in.P, c.in.Q, c.in.R, c.in.RHO);
    return c;
}

// The lag states at equilibrium: equal to the undelayed wake angles, which
// depend only on α_WF and ψ_WF, not on the states.
std::vector<double> steady_states(const AirframeAero& a, const Case& c) {
    std::vector<double> x(2, 0.0);
    const AirframeAeroLoads l0 = a.evaluate(c.ctx, x, c.fuselage_wash_b, c.tail_wash_b);
    x[0] = l0.downwash;
    x[1] = l0.sidewash;
    return x;
}

// Layer 1 (D-028): with its lag states at equilibrium, production gives
// GenHel's loads for every panel, and GenHel's tunnel angles.
TEST(AirframeAero, MatchesGenHelSteadyState) {
    std::mt19937_64 rng(6101);
    const RotorWash fus = test::uh60a_wash("fuselage_wash.csv");
    const RotorWash tail = test::uh60a_wash("tail_wash.csv");
    for (int i = 0; i < 400; ++i) {
        const Case c = random_case(rng, fus, tail);
        const test::GenHelAirframeOutput ref = test::genhel_airframe(genhel_tables(), c.in);
        const AirframeAero aero(test::uh60a_airframe(c.g));
        const std::vector<double> x = steady_states(aero, c);
        const AirframeAeroLoads p = aero.evaluate(c.ctx, x, c.fuselage_wash_b, c.tail_wash_b);

        EXPECT_NEAR(p.alpha_wf, ref.ALFWF * kDegree, 1e-12) << "case " << i;
        EXPECT_NEAR(p.psi_wf, ref.PSIWF * kDegree, 1e-12) << "case " << i;
        EXPECT_NEAR(p.downwash, ref.EPSH1 * kDegree, 1e-12) << "case " << i;
        EXPECT_NEAR(p.sidewash, ref.SIGV1 * kDegree, 1e-12) << "case " << i;
        EXPECT_NEAR(p.alpha_h, ref.ALFHH1 * kDegree, 1e-12) << "case " << i;
        EXPECT_NEAR(p.beta_v, ref.BETVV1 * kDegree, 1e-12) << "case " << i;
        expect_loads(p.fuselage, ref.WF, magnitude(ref.WF), "fuselage", i);
        expect_loads(p.stabilator, ref.H1, magnitude(ref.H1), "stabilator", i);
        expect_loads(p.fin, ref.V1, magnitude(ref.V1), "fin", i);
        const test::GenHelLoads sum{ref.WF.X + ref.H1.X + ref.V1.X, ref.WF.Y + ref.H1.Y + ref.V1.Y,
                                    ref.WF.Z + ref.H1.Z + ref.V1.Z, ref.WF.L + ref.H1.L + ref.V1.L,
                                    ref.WF.M + ref.H1.M + ref.V1.M, ref.WF.N + ref.H1.N + ref.V1.N};
        expect_loads(p.body, sum, magnitude(sum) + magnitude(ref.WF), "total", i);
    }
}

// Layer 1 out of equilibrium: the delayed downwash and sidewash enter as
// GenHel's delayed values do.
TEST(AirframeAero, MatchesGenHelWithDelayedWake) {
    std::mt19937_64 rng(6102);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    const RotorWash fus = test::uh60a_wash("fuselage_wash.csv");
    const RotorWash tail = test::uh60a_wash("tail_wash.csv");
    for (int i = 0; i < 200; ++i) {
        Case c = random_case(rng, fus, tail);
        const std::vector<double> x = {3.0 * kDegree * u(rng), 2.0 * kDegree * u(rng)};
        c.in.EPSH1_DELAYED = x[0] / kDegree;
        c.in.SIGV1_DELAYED = x[1] / kDegree;
        const test::GenHelAirframeOutput ref = test::genhel_airframe(genhel_tables(), c.in);
        const AirframeAero aero(test::uh60a_airframe(c.g));
        const AirframeAeroLoads p = aero.evaluate(c.ctx, x, c.fuselage_wash_b, c.tail_wash_b);
        expect_loads(p.stabilator, ref.H1, magnitude(ref.H1), "stabilator", i);
        expect_loads(p.fin, ref.V1, magnitude(ref.V1), "fin", i);
    }
}

// D-028: the transport delay l/u becomes dε_d/dt = (|u|/l)(ε − ε_d), with u
// the body x velocity relative to the air and l the panel's distance aft.
TEST(AirframeAero, LagStateRates) {
    std::mt19937_64 rng(6103);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    const RotorWash fus = test::uh60a_wash("fuselage_wash.csv");
    const RotorWash tail = test::uh60a_wash("tail_wash.csv");
    for (int i = 0; i < 100; ++i) {
        Case c = random_case(rng, fus, tail);
        if (i % 4 == 0) {
            c.ctx.body.vel_b.x() = -c.ctx.body.vel_b.x();  // rearward: |u|
        }
        const AirframeAero aero(test::uh60a_airframe(c.g));
        const std::vector<double> x = {3.0 * kDegree * u(rng), 2.0 * kDegree * u(rng)};
        const AirframeAeroLoads p = aero.evaluate(c.ctx, x, c.fuselage_wash_b, c.tail_wash_b);
        const double ux = std::abs(c.ctx.body.vel_b.x());
        const double lh = -aero.config().stabilator.position_b.x();
        const double lv = -aero.config().fin.position_b.x();
        EXPECT_NEAR(p.downwash_rate, ux / lh * (p.downwash - x[0]), 1e-12) << "case " << i;
        EXPECT_NEAR(p.sidewash_rate, ux / lv * (p.sidewash - x[1]), 1e-12) << "case " << i;

        // And derivatives() reports the same rates and the same total load.
        std::vector<double> xdot(2, 0.0);
        ComponentDerivs d{xdot, {}, {}, Mat66::Zero(), true, nullptr};
        SimContext no_coupling = c.ctx;  // no wash: compare with zero wash
        aero.derivatives(no_coupling, x, d);
        const AirframeAeroLoads q = aero.evaluate(no_coupling, x, Vec3::Zero(), Vec3::Zero());
        EXPECT_EQ(xdot[0], q.downwash_rate);
        EXPECT_EQ(xdot[1], q.sidewash_rate);
        EXPECT_EQ(d.load0_b.F, q.body.F);
        EXPECT_EQ(d.load0_b.M, q.body.M);
    }
}

// Layer 2: GenHel gives each sideslip table a parity (p. 5.2-7, Tables
// 2.5.4–2.5.11): side force, rolling moment and the pitching-moment increment
// are odd in ψ_WF, the drag increment is even. Yaw is odd only inside ±30°
// (its high-angle map is not), and the lift increment is not even (Table
// 2.5.7 as printed). With those two made symmetric and the pitch increment
// zeroed, a fuselage on the centerline must give mirror-image loads for
// mirror-image flight: F_x, F_z, M_y unchanged; F_y, M_x, M_z reversed.
TEST(AirframeAero, FuselageMirrorSymmetry) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5 + 10.0;  // synthetic, on the centerline
    g.WLWF = 234.0 + 3.0;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    AirframeAeroConfig cfg = test::uh60a_airframe(g);
    // Symmetric stand-ins for the two asymmetric tables (synthetic).
    cfg.fuselage.yaw = Table1D(Axis({-90 * kDegree, 90 * kDegree}), {200.0, -200.0});
    cfg.fuselage.lift_increment =
        Table1D(Axis({-30 * kDegree, 0.0, 30 * kDegree}), {3.0, 0.0, 3.0});
    cfg.fuselage.pitch_increment = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
    const AirframeAero aero(std::move(cfg));
    std::mt19937_64 rng(6104);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    const std::vector<double> x = {0.0, 0.0};
    for (int i = 0; i < 200; ++i) {
        const double vx = 150.0 * u(rng);
        const double vy = 80.0 * u(rng);
        const double vz = 60.0 * u(rng);
        const SimContext a = context(vx, vy, vz, 0.0, 0.0, 0.0, 0.0023769);
        const SimContext b = context(vx, -vy, vz, 0.0, 0.0, 0.0, 0.0023769);
        const Wrench la = aero.evaluate(a, x, Vec3::Zero(), Vec3::Zero()).fuselage;
        const Wrench lb = aero.evaluate(b, x, Vec3::Zero(), Vec3::Zero()).fuselage;
        ASSERT_GT(la.F.norm(), 1.0) << "case " << i;  // not vacuous
        const double s = 1e-12 * (la.F.norm() + la.M.norm() + 1.0);
        EXPECT_NEAR(la.F.x(), lb.F.x(), s) << "case " << i;
        EXPECT_NEAR(la.F.y(), -lb.F.y(), s) << "case " << i;
        EXPECT_NEAR(la.F.z(), lb.F.z(), s) << "case " << i;
        EXPECT_NEAR(la.M.x(), -lb.M.x(), s) << "case " << i;
        EXPECT_NEAR(la.M.y(), lb.M.y(), s) << "case " << i;
        EXPECT_NEAR(la.M.z(), -lb.M.z(), s) << "case " << i;
    }
}

// The asymmetries above are in the data, not in the code: GenHel's yaw map is
// odd inside ±30° but not beyond, and the lift increment is not even.
TEST(AirframeAero, KnownTableAsymmetries) {
    const test::GenHelAirframeTables& t = genhel_tables();
    for (double psi = 5.0; psi <= 30.0; psi += 5.0) {
        EXPECT_EQ(t.NQF(psi), -t.NQF(-psi)) << psi;
    }
    EXPECT_NE(t.NQF(40.0), -t.NQF(-40.0));
    EXPECT_NE(t.DLQF(20.0), t.DLQF(-20.0));
}

// D-030: drag never adds energy. With every lift, side-force and moment table
// zero and no wake, the force at each panel and on the fuselage opposes the
// flow, in every direction including rearward flight: F·V ≤ 0. GenHel's
// atan(V_Z/|V_X|) wind axes fail this in rearward flight (checked below on
// the reference), which is why D-030 departs from them.
TEST(AirframeAero, DragOpposesTheFlow) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = g.FSV1 = 700.0;  // synthetic
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    AirframeAeroConfig cfg = test::uh60a_airframe(g);
    const Table1D zero(Axis({0.0, 1.0}), {0.0, 0.0});
    cfg.fuselage.lift = cfg.fuselage.pitch = cfg.fuselage.side = cfg.fuselage.lift_increment =
        cfg.fuselage.roll = cfg.fuselage.pitch_increment = cfg.fuselage.yaw = zero;
    cfg.stabilator.cl = cfg.fin.cl = zero;
    cfg.stabilator.wake_angle = cfg.fin.wake_angle = zero;
    const AirframeAero aero(std::move(cfg));
    std::mt19937_64 rng(6105);
    std::normal_distribution<double> n(0.0, 1.0);
    const std::vector<double> x = {0.0, 0.0};
    int rearward = 0;
    for (int i = 0; i < 500; ++i) {
        const Vec3 v = Vec3(n(rng), n(rng), n(rng)).normalized() * 40.0;  // [ft/s]
        rearward += v.x() < 0.0 ? 1 : 0;
        const SimContext ctx = context(v.x(), v.y(), v.z(), 0.0, 0.0, 0.0, 0.0023769);
        const AirframeAeroLoads p = aero.evaluate(ctx, x, Vec3::Zero(), Vec3::Zero());
        const Vec3 vb = v * kFoot;
        ASSERT_GT(p.fuselage.F.norm(), 1.0) << "case " << i;  // not vacuous
        ASSERT_GT(p.stabilator.F.norm(), 0.1) << "case " << i;
        ASSERT_GT(p.fin.F.norm(), 0.1) << "case " << i;
        EXPECT_LE(p.fuselage.F.dot(vb), 1e-12 * p.fuselage.F.norm() * vb.norm()) << "case " << i;
        EXPECT_LE(p.stabilator.F.dot(p.v_stabilator_b), 1e-12 * p.stabilator.F.norm() * vb.norm())
            << "case " << i;
        EXPECT_LE(p.fin.F.dot(p.v_fin_b), 1e-12 * p.fin.F.norm() * vb.norm()) << "case " << i;
        // And the force is exactly along the flow: |F × V| = 0.
        EXPECT_LE(p.fuselage.F.cross(vb).norm(), 1e-12 * p.fuselage.F.norm() * vb.norm());
    }
    EXPECT_GT(rearward, 200);

    // GenHel's own wind axes, rearward with no sideslip: the drag points aft,
    // along the motion.
    test::GenHelAirframeInput in;
    test::apply(g, in);
    in.RHO = 0.0023769;
    in.VXB = -40.0;
    const test::GenHelAirframeOutput ref = test::genhel_airframe(genhel_tables(), in);
    EXPECT_LT(ref.WF.X, 0.0);  // force aft while moving aft: positive power
}

// A dynamic-pressure ratio below zero has no square root (K_Q = √ratio), and
// only the stabilator has a schedule: both are init errors, not a NaN or a
// silently ignored table at run time.
TEST(AirframeAero, RejectsInvalidPanels) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    const AirframeAeroConfig good = test::uh60a_airframe(g);
    ASSERT_NO_THROW(AirframeAero{good});
    const Table1D negative(Axis({0.0, 1.0}), {1.0, -0.01});
    AirframeAeroConfig c = good;
    c.stabilator.q_ratio = negative;
    EXPECT_THROW(AirframeAero{std::move(c)}, InitError);
    c = good;
    c.fin.q_ratio = negative;
    EXPECT_THROW(AirframeAero{std::move(c)}, InitError);
    c = good;
    c.fin.schedule =
        IncidenceSchedule{Table2D(Axis({0.0, 1.0}), Axis({0.0, 1.0}), {0, 0, 0, 0}), -0.1, 0.1};
    EXPECT_THROW(AirframeAero{std::move(c)}, InitError);
}

// D-030 in level flow (V_Z = 0), rearward: the tables see the same angles as
// at +|V_X|, and α' = π turns the wind-to-body rotation into diag(−1, 1, −1).
// So the drag and the lift both reverse in body axes, as do the rolling and
// yawing moments; the side force and the pitching moment don't. A mirror
// image of the forward loads would keep the lift and the roll and reverse the
// pitch instead (DECISIONS.md D-030, "What it doesn't fix").
TEST(AirframeAero, RearwardFuselageLoadsInLevelFlow) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    const AirframeAero aero(test::uh60a_airframe(g));
    const Vec3 r_b = aero.config().fuselage_point_b;
    const Vec3 flip(-1.0, 1.0, -1.0);
    const std::vector<double> x = {0.0, 0.0};
    for (const double vy : {0.0, 15.0, -25.0}) {
        const Wrench fwd = aero.evaluate(context(40.0, vy, 0.0, 0.0, 0.0, 0.0, 0.0023769), x,
                                         Vec3::Zero(), Vec3::Zero())
                               .fuselage;
        const Wrench aft = aero.evaluate(context(-40.0, vy, 0.0, 0.0, 0.0, 0.0, 0.0023769), x,
                                         Vec3::Zero(), Vec3::Zero())
                               .fuselage;
        ASSERT_GT(std::abs(fwd.F.z()), 1.0);  // not vacuous: the lift is there
        // Moments about the mounting point, where the tables put them.
        const Vec3 m_fwd_b = fwd.M - r_b.cross(fwd.F);
        const Vec3 m_aft_b = aft.M - r_b.cross(aft.F);
        if (vy != 0.0) {
            ASSERT_GT(std::abs(m_fwd_b.x()), 1.0) << vy;
            ASSERT_GT(std::abs(m_fwd_b.z()), 1.0) << vy;
        }
        EXPECT_NEAR((aft.F - flip.cwiseProduct(fwd.F)).norm(), 0.0, 1e-12 * fwd.F.norm()) << vy;
        EXPECT_NEAR((m_aft_b - flip.cwiseProduct(m_fwd_b)).norm(), 0.0, 1e-12 * m_fwd_b.norm())
            << vy;
    }
}

// D-030: the loads are continuous as the flow goes from ahead to behind, with
// flow in the x–z plane (V_Z ≠ 0). Pure sideward flow (V_X = V_Z = 0) is a
// singular line of the wind axes, here and in GenHel (SPEC_REVIEW #27).
TEST(AirframeAero, ContinuousThroughZeroForwardSpeed) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    const AirframeAero aero(test::uh60a_airframe(g));
    const std::vector<double> x = {1.0 * kDegree, -0.5 * kDegree};
    for (const double vy : {0.0, 20.0, -35.0}) {
        for (const double vz : {-30.0, 5.0, 25.0}) {
            const double e = 1e-7;
            const AirframeAeroLoads a = aero.evaluate(
                context(e, vy, vz, 0.1, -0.05, 0.2, 0.0023769), x, Vec3::Zero(), Vec3::Zero());
            const AirframeAeroLoads b = aero.evaluate(
                context(-e, vy, vz, 0.1, -0.05, 0.2, 0.0023769), x, Vec3::Zero(), Vec3::Zero());
            ASSERT_GT(a.body.F.norm(), 1.0);  // not vacuous
            const double s = 1e-5 * (a.body.F.norm() + 1.0);
            EXPECT_NEAR((a.body.F - b.body.F).norm(), 0.0, s) << vy << " " << vz;
            EXPECT_NEAR((a.body.M - b.body.M).norm(), 0.0, s * 10.0) << vy << " " << vz;
        }
    }
}

// Data ruling A (2026-10-13, D-028): the main-rotor downwash takes the same
// sign at the stabilator, the fin and the tail rotor, from the same map
// (Table 3.5.2), and points down (V_Z < 0, air moving down past the panel)
// when the rotor pushes air down. In hover with no rates the body velocity
// and the fuselage wake vanish, so only the main-rotor wash remains.
TEST(AirframeAero, MainRotorDownwashSameAtStabilatorFinAndTailRotor) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    const AirframeAero aero(test::uh60a_airframe(g));
    const RotorWash tail = test::uh60a_wash("tail_wash.csv");
    const double tip = kOmegaMR * kRadiusMR * kFoot;
    for (const double lambda0 : {0.03, 0.06}) {
        const RotorWash::Result w = tail.evaluate(lambda0, 0.0, 0.0, 0.0, tip);
        ASSERT_LT(w.velocity_b.z(), 0.0);
        const SimContext ctx = context(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0023769);
        const std::vector<double> x = {0.0, 0.0};
        const AirframeAeroLoads p = aero.evaluate(ctx, x, Vec3::Zero(), w.velocity_b);
        EXPECT_LT(p.v_stabilator_b.z(), 0.0);
        EXPECT_LT(p.v_fin_b.z(), 0.0);
        EXPECT_EQ(p.v_fin_b.z(), p.v_stabilator_b.z());
        EXPECT_EQ(p.v_fin_b.z(), w.velocity_b.z());  // what the tail rotor adds (M6a)
    }
}

// The tunnel angles reach the tail rotor through the coupling store (D-003),
// and the airframe's derivatives are pure and allocation-free in advance().
std::vector<std::unique_ptr<DynamicComponent>> main_rotor_and_airframe() {
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
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    std::vector<std::unique_ptr<DynamicComponent>> v;
    v.push_back(std::make_unique<AirframeAero>(test::uh60a_airframe(g)));  // consumer first
    v.push_back(std::make_unique<BladeElementRotor>(std::move(m)));
    return v;
}

TEST(AirframeAero, ThroughTheExecutive) {
    auto components = main_rotor_and_airframe();
    const auto* aero = dynamic_cast<const AirframeAero*>(components[0].get());
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(7000.0, 6000.0, 50000.0, 48000.0, 2500.0);
    Executive ex(cfg, std::move(components));
    ASSERT_EQ(ex.evaluation_order()[0], 1U);  // the main rotor first

    BodyState b;
    b.vel_b = Vec3(40.0, 3.0, 2.0);
    b.omega_b = Vec3(0.05, -0.02, 0.1);
    ex.set_body_state(b);
    (void)ex.advance(Controls{}, Environment{}, 200'000'000);

    // Pure: the same derivatives twice.
    const std::span<const double> x = ex.state();
    std::vector<double> d1(ex.num_states());
    std::vector<double> d2(ex.num_states());
    ex.derivatives(ex.sim_time(), x, Controls{}, Environment{}, d1);
    ex.derivatives(ex.sim_time(), x, Controls{}, Environment{}, d2);
    EXPECT_EQ(d1, d2);
    // The lag states moved off zero: the wake is being tracked.
    const std::size_t i_aero = body_layout::kSize;
    EXPECT_NE(x[i_aero], 0.0);
    EXPECT_EQ(aero->produces(), (std::vector<std::string>{"airframe.alpha_wf", "airframe.psi_wf"}));

    const test::AllocationCounter count;
    (void)ex.advance(Controls{}, Environment{}, 16'666'667);
    EXPECT_EQ(count.count(), 0U);
}

}  // namespace
}  // namespace eyas

namespace eyas {
namespace {

// The config reader gives the same model as the fixture built by hand, with
// the UH-60A tables from data/uh60a/airframe/ and no placeholders.
TEST(AirframeAero, ReadsConfig) {
    const std::string dir = test::uh60a_airframe_dir();
    const Config cfg = Config::from_string(R"(
type: airframe_aero
name: airframe
fuselage:
  mounting_point: {x: "10.4 in", y: "0 in", z: "14.2 in"}
  alpha_table: )" + dir + R"(/fuselage_alpha.csv
  psi_table: )" + dir + R"(/fuselage_psi.csv
  psi_low_table: )" + dir + R"(/fuselage_psi_low.csv
  main_rotor_wash: {rotor: main_rotor, table: )" +
                                           dir + R"(/fuselage_wash.csv}
tail_wash: {rotor: main_rotor, table: )" + dir +
                                           R"(/tail_wash.csv}
stabilator:
  position: {x: "-344.2 in", y: "0 in", z: "4.2 in"}
  area: "45 ft^2"
  incidence: "5 deg"
  coefficients: )" + dir + R"(/stabilator_coefficients.csv
  q_ratio: )" + dir + R"(/stabilator_q_ratio.csv
  downwash: )" + dir + R"(/stabilator_downwash.csv
fin:
  position: {x: "-339.1 in", y: "0 in", z: "-24.8 in"}
  area: "32.3 ft^2"
  coefficients: )" + dir + R"(/fin_coefficients.csv
  q_ratio: )" + dir + R"(/fin_q_ratio.csv
  sidewash: )" + dir + R"(/fin_sidewash.csv
)");
    const std::unique_ptr<AirframeAero> read = AirframeAero::from_config(cfg);
    EXPECT_TRUE(cfg.placeholders().empty());
    EXPECT_EQ(read->name(), "airframe");

    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = 700.1;
    g.WLH1 = 244.0;
    g.SAH1 = 45.0;
    g.IH1 = 5.0;
    g.FSV1 = 695.0;
    g.WLV1 = 273.0;
    g.SAV1 = 32.3;
    const AirframeAero built(test::uh60a_airframe(g));
    const std::vector<double> x = {0.01, -0.02};
    const SimContext ctx = context(150.0, 20.0, -10.0, 0.1, 0.05, -0.1, 0.0023769);
    const Vec3 w(4.0, 0.0, -9.0);
    const AirframeAeroLoads a = read->evaluate(ctx, x, w, w);
    const AirframeAeroLoads b = built.evaluate(ctx, x, w, w);
    EXPECT_NEAR((a.body.F - b.body.F).norm(), 0.0, 1e-9 * b.body.F.norm());
    EXPECT_NEAR((a.body.M - b.body.M).norm(), 0.0, 1e-9 * b.body.M.norm());

    // An incomplete node is an error, not a model with zero tables.
    EXPECT_THROW((void)AirframeAero::from_config(Config::from_string(R"(
type: airframe_aero
fuselage: {mounting_point: {x: 0, y: 0, z: 0}}
)")),
                 InitError);
}

}  // namespace
}  // namespace eyas

namespace eyas {
namespace {

// D-003: derivatives() writes GenHel's tunnel angles to the store, for the
// tail rotor's fuselage-wake maps.
TEST(AirframeAero, WritesTunnelAnglesToTheStore) {
    test::GenHelAirframeGeometry g{};
    g.FSCGB = 355.9;
    g.WLCGB = 248.2;
    g.FSWF = 345.5;
    g.WLWF = 234.0;
    g.FSH1 = g.FSV1 = 700.0;
    g.SAH1 = 45.0;
    g.SAV1 = 32.3;
    AirframeAeroConfig cfg = test::uh60a_airframe(g);
    cfg.fuselage_wash.reset();  // no main rotor in this test
    cfg.tail_wash.reset();
    AirframeAero aero(std::move(cfg));
    CouplingRegistry registry;
    for (const std::string& n : aero.produces()) {
        (void)registry.add(n, aero.name());
    }
    aero.bind(registry);
    CouplingOutputs store(registry.size());
    store.begin_evaluation();

    const SimContext ctx = context(120.0, -25.0, 15.0, 0.0, 0.0, 0.0, 0.0023769);
    const std::vector<double> x = {0.0, 0.0};
    std::vector<double> xdot(2, 0.0);
    ComponentDerivs d{xdot, {}, {}, Mat66::Zero(), true, &store};
    aero.derivatives(ctx, x, d);
    const AirframeAeroLoads l = aero.evaluate(ctx, x, Vec3::Zero(), Vec3::Zero());
    EXPECT_EQ(store.get(registry.find("airframe.alpha_wf", "test")), l.alpha_wf);
    EXPECT_EQ(store.get(registry.find("airframe.psi_wf", "test")), l.psi_wf);
    EXPECT_GT(l.psi_wf, 0.0);  // v_y < 0: ψ_WF = −β_WF > 0
}

}  // namespace
}  // namespace eyas

namespace eyas {
namespace {

AirframeAeroConfig scheduled_airframe() {
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
    AirframeAeroConfig c = test::uh60a_airframe(g);
    IncidenceSchedule sch;
    const CsvTable csv = read_csv_table(test::uh60a_airframe_dir() + "/stabilator_schedule.csv");
    sch.command = make_table2d(csv, "airspeed", "collective", "incidence");
    sch.min = -8.0 * kDegree;
    sch.max = 39.0 * kDegree;
    c.stabilator.schedule = sch;
    return c;
}

// The stabilator schedule (GenHel Fig. 5.1.7, steady state; D-028) against
// GenHel's block diagram evaluated directly: the merged table is exact (each
// cell is bilinear), so they agree to rounding everywhere, outside the table
// too (rearward flight, past 147 kt, collective past its stops).
TEST(StabilatorSchedule, MatchesGenHel) {
    const test::GenHelStabilator ref = test::read_genhel_stabilator(test::uh60a_genhel_dir());
    const AirframeAero aero(scheduled_airframe());
    std::mt19937_64 rng(6301);
    std::uniform_real_distribution<double> v(-40.0, 200.0);  // [kt]
    std::uniform_real_distribution<double> c(-0.1, 1.1);
    const std::vector<double> x = {0.0, 0.0};
    for (int i = 0; i < 2000; ++i) {
        const double v_kt = v(rng);
        SimContext ctx = context(v_kt * units::kKnot / kFoot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0023769);
        ctx.controls.collective = c(rng);
        const AirframeAeroLoads p = aero.evaluate(ctx, x, Vec3::Zero(), Vec3::Zero());
        const double expect = ref.incidence(v_kt, 100.0 * ctx.controls.collective);
        EXPECT_NEAR(p.stabilator_incidence, expect * kDegree, 1e-12) << v_kt << " kt";
    }
}

// What the schedule does, checked by hand from the diagram: in hover the 42°
// bias meets the 39° travel limit; the command starts falling at 30 kt, and
// more steeply at low collective.
TEST(StabilatorSchedule, HoverAndForwardFlight) {
    const AirframeAero aero(scheduled_airframe());
    const std::vector<double> x = {0.0, 0.0};
    const auto incidence = [&](double v_kt, double collective) {
        SimContext ctx = context(v_kt * units::kKnot / kFoot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0023769);
        ctx.controls.collective = collective;
        return aero.evaluate(ctx, x, Vec3::Zero(), Vec3::Zero()).stabilator_incidence / kDegree;
    };
    EXPECT_NEAR(incidence(0.0, 0.6), 39.0, 1e-12);
    EXPECT_NEAR(incidence(30.0, 0.6), 39.0, 1e-12);
    // 80 kt, 50 %: 42 − (0.67 + 0.003286·20)·50 = 5.214°.
    EXPECT_NEAR(incidence(80.0, 0.5), 5.214, 1e-10);
    EXPECT_LT(incidence(80.0, 0.2), incidence(80.0, 0.5));
    // Past 147 kt both airspeed terms are saturated.
    EXPECT_NEAR(incidence(160.0, 0.5), incidence(147.0, 0.5), 1e-12);

    // The scheduled incidence drives the loads exactly as a fixed one would.
    AirframeAeroConfig fixed = scheduled_airframe();
    fixed.stabilator.schedule.reset();
    fixed.stabilator.incidence = incidence(80.0, 0.5) * kDegree;
    const AirframeAero f(std::move(fixed));
    SimContext ctx = context(80.0 * units::kKnot / kFoot, 3.0, -4.0, 0.0, 0.1, 0.0, 0.0023769);
    ctx.controls.collective = 0.5;
    const AirframeAeroLoads a = aero.evaluate(ctx, x, Vec3::Zero(), Vec3::Zero());
    const AirframeAeroLoads b = f.evaluate(ctx, x, Vec3::Zero(), Vec3::Zero());
    EXPECT_EQ(a.stabilator.F, b.stabilator.F);
}

TEST(StabilatorSchedule, ReadsConfig) {
    const std::string dir = test::uh60a_airframe_dir();
    const std::string common = R"(
type: airframe_aero
fuselage:
  mounting_point: {x: "0 in", y: "0 in", z: "0 in"}
  alpha_table: )" + dir + R"(/fuselage_alpha.csv
  psi_table: )" + dir + R"(/fuselage_psi.csv
  psi_low_table: )" + dir + R"(/fuselage_psi_low.csv
fin:
  position: {x: "-339.1 in", y: "0 in", z: "-24.8 in"}
  area: "32.3 ft^2"
  coefficients: )" + dir + R"(/fin_coefficients.csv
  q_ratio: )" + dir + R"(/fin_q_ratio.csv
  sidewash: )" + dir + R"(/fin_sidewash.csv
stabilator:
  position: {x: "-344.2 in", y: "0 in", z: "4.2 in"}
  area: "45 ft^2"
  coefficients: )" + dir + R"(/stabilator_coefficients.csv
  q_ratio: )" + dir + R"(/stabilator_q_ratio.csv
  downwash: )" + dir + R"(/stabilator_downwash.csv
)";
    const std::string schedule = "  schedule: {table: " + dir +
                                 "/stabilator_schedule.csv, min: \"-8 deg\", max: \"39 deg\"}\n";
    const auto r = AirframeAero::from_config(Config::from_string(common + schedule));
    ASSERT_TRUE(r->config().stabilator.schedule.has_value());
    EXPECT_DOUBLE_EQ(r->config().stabilator.schedule->max, 39.0 * kDegree);
    // Incidence and schedule together is an error.
    EXPECT_THROW((void)AirframeAero::from_config(
                     Config::from_string(common + schedule + "  incidence: \"2 deg\"\n")),
                 InitError);
    // The fin has no schedule: the key is unknown there, not ignored.
    std::string fin_scheduled = common;
    fin_scheduled.insert(fin_scheduled.find("fin:\n") + 5, schedule);
    EXPECT_THROW((void)AirframeAero::from_config(Config::from_string(fin_scheduled)), InitError);
}

}  // namespace
}  // namespace eyas
