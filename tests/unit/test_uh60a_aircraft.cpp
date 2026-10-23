// The UH-60A aircraft (M6d, D-028, D-034): the config data/uh60a/uh60a.yaml
// built by tools/build_uh60a_config.py from GenHel CR-166309, and the
// whole-aircraft checks.

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "control/genhel_fcs.hpp"
#include "eyas/model.hpp"
#include "support/alloc_counter.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"

namespace eyas {
namespace {

using test::Uh60aHarness;
constexpr double kDeg = std::numbers::pi / 180.0;

// ---- Config ------------------------------------------------------------------------

// The config loads through the public API, with exactly the placeholders
// D-034 declares: the strut top stops (D-032 item 6). The cyclic offsets
// were placeholders until M7c set neutral longitudinal cyclic (D-035 M7c
// amendment).
TEST(Uh60aAircraft, ConfigLoadsWithTheDeclaredPlaceholders) {
    const Model m = Model::from_file(test::uh60a_config_path());
    std::vector<std::string> p = m.placeholders();
    std::sort(p.begin(), p.end());
    std::vector<std::string> expect;
    for (const char* g : {"0", "1", "2"}) {
        // components[5]: the engines (D-053) come first in the config.
        expect.push_back(std::string("components[5].gears[") + g + "].strut.stop_damping");
        expect.push_back(std::string("components[5].gears[") + g + "].strut.stop_stiffness");
    }
    std::sort(expect.begin(), expect.end());
    EXPECT_EQ(p, expect);
}

// Airframe + blades + unsprung masses is GenHel's WEIGHT, and their common
// CG is GenHel's CG (p. 5.1-49). The CG is placed in body axes from the
// config's own hub position and GenHel's station differences, independently
// of the generator's mass balance.
TEST(Uh60aAircraft, MassSplitKeepsGenHelsWeightAndCg) {
    const Uh60aHarness ac;
    const auto& mr = ac.get<BladeElementRotor>("main_rotor").config();
    const auto& gear = ac.get<LandingGear>("gear").config();
    double m = ac.airframe_mass().mass;
    Vec3 mr_sum = Vec3::Zero();  // airframe at the origin
    const double blades = mr.num_blades * mr.flap->mass;
    m += blades;
    mr_sum += blades * mr.hub_position_b;  // blade centroid on the shaft at the hub (no coning)
    for (const auto& g : gear.gears) {
        m += g.unsprung_mass;
        mr_sum += g.unsprung_mass * g.axle_b;
    }
    EXPECT_NEAR(m / units::kPoundMass, test::kGenHelWeight, 1e-5);
    // GenHel's CG from the hub: Δ = (−(FSCG − FS_MR), 0, −(WLCG − WL_MR)) in.
    const Vec3 cg_expect = mr.hub_position_b +
                           Vec3(-(test::kFSCG - 341.2), 0.0, -(test::kWLCG - 315.0)) * units::kInch;
    EXPECT_NEAR(((mr_sum / m) - cg_expect).norm() / units::kInch, 0.0, 1e-5);
    // The airframe CG is below and aft of GenHel's (the blades sit high and
    // forward of it), so GenHel's CG is up (z < 0) and forward (x > 0) of the
    // body origin: reported in D-034.
    EXPECT_LT(cg_expect.z(), 0.0);
    EXPECT_GT(cg_expect.x(), 0.0);

    // The shaft leans forward by i_θ = 3° (Fig. 1.1.1, p. 5.1-7; D-034): the
    // thrust axis −z_h has a forward body component, none sideways.
    const Vec3 up_shaft =
        ac.get<BladeElementRotor>("main_rotor").C_hb().transpose() * Vec3(0.0, 0.0, -1.0);
    EXPECT_NEAR(std::asin(up_shaft.x()) / kDeg, 3.0, 1e-12);
    EXPECT_NEAR(up_shaft.y(), 0.0, 1e-15);
}

// Fletcher (21st ERF 1995, Paper VII-4, Table 1, p. VII-4-9) lists the
// theoretical flap parameters his identification started from: blade mass
// moment ratio M_β/I_β = 0.0573 [1/ft] and pitch–flap coupling 0. The rotor
// as loaded must have them (D-035 M7c amendment, V2). His theoretical Lock
// number, 8.06, assumes a lift slope the paper doesn't give (5.72 /rad at
// sea level); VALIDATION.md M7c reports Eyas's against it.
TEST(Uh60aAircraft, FlapParametersAreFletchersTheoreticalOnes) {
    const Uh60aHarness ac;
    const auto& flap = *ac.get<BladeElementRotor>("main_rotor").config().flap;
    EXPECT_NEAR(flap.first_moment / flap.inertia * units::kFoot, 0.0573, 5e-5);
    EXPECT_EQ(flap.delta3, 0.0);
}

// The main wheels are mirrored about the centreline with the brakes on the
// matching sides: GenHel's Y_RMGA = BLMGA/12 is body y, positive to
// starboard (p. 5.7-9), and p. 5.7-10's Y_LMGA = Y_RMGA has lost its minus
// sign (DATA_GAPS.md). The mass and CG checks are symmetric in y, so this is
// the only check of which wheel is which.
TEST(Uh60aAircraft, MainGearSidesMatchTheirBrakes) {
    const Uh60aHarness ac;
    const auto& gears = ac.get<LandingGear>("gear").config().gears;
    ASSERT_EQ(gears.size(), 3U);
    const auto by_name = [&](std::string_view name) -> const GearConfig* {
        const auto it = std::find_if(gears.begin(), gears.end(),
                                     [&](const GearConfig& g) { return g.name == name; });
        return it == gears.end() ? nullptr : &*it;
    };
    const GearConfig* l = by_name("left_main");
    const GearConfig* r = by_name("right_main");
    const GearConfig* t = by_name("tail");
    ASSERT_TRUE(l != nullptr && r != nullptr && t != nullptr);
    const GearConfig& left = *l;
    const GearConfig& right = *r;
    const GearConfig& tail = *t;
    EXPECT_NEAR(left.axle_b.y() / units::kInch, -53.2, 1e-9);  // port
    EXPECT_NEAR(right.axle_b.y() / units::kInch, 53.2, 1e-9);  // starboard
    EXPECT_EQ(left.axle_b.x(), right.axle_b.x());
    EXPECT_EQ(left.axle_b.z(), right.axle_b.z());
    EXPECT_EQ(left.brake, BrakeSide::left);
    EXPECT_EQ(right.brake, BrakeSide::right);
    EXPECT_EQ(tail.axle_b.y(), 0.0);
    EXPECT_EQ(tail.brake, BrakeSide::none);
}

// The left brake alone turns the nose left, the right brake alone right
// (D-032). Rolling forward at 3 m/s from tire contact, collective down and
// the pedal that balances the yaw moment there (otherwise the run drifts and
// the tail-dragger's directional instability amplifies the drift): the
// heading after 1.5 s with one brake set, against the same run with both
// brakes off.
TEST(Uh60aAircraft, OneBrakeTurnsTowardsItsSide) {
    const Uh60aHarness ac;
    Controls trim;
    {
        BodyState b;
        b.pos_n = Vec3(0.0, 0.0, -500.0);
        std::vector<double> x = ac.initial(b);
        double t = 0.0;
        ASSERT_TRUE(test::balance_pedal(ac, x, t, trim, Environment{}));
    }
    double contact = 0.0;  // lowest tire bottom below the CG at free extension [m]
    for (const auto& g : ac.get<LandingGear>("gear").config().gears) {
        contact = std::max(contact, g.axle_b.z() + g.tire.radius);
    }
    Environment env;
    env.ground_height = 0.0;
    const auto heading_after = [&](double left, double right) {
        Model m = Model::from_file(test::uh60a_config_path());
        BodyState s;
        s.pos_n = Vec3(0.0, 0.0, -contact);
        s.vel_b = Vec3(3.0, 0.0, 0.0);
        EXPECT_TRUE(m.reset(s));
        Controls c = trim;
        c.brake_left = left;
        c.brake_right = right;
        for (int k = 0; k < 1500; ++k) {
            (void)m.advance(c, env, 1'000'000);
        }
        return m.outputs().heading;
    };
    const auto turn = [](double h, double h0) {  // h − h0 wrapped to (−π, π] [rad]
        return std::remainder(h - h0, 2.0 * std::numbers::pi);
    };
    const double free = heading_after(0.0, 0.0);
    const double drift = std::abs(turn(free, 0.0));
    const double left = turn(heading_after(1.0, 0.0), free);
    const double right = turn(heading_after(0.0, 1.0), free);
    // About −0.083° and +0.090°, against a free-run drift of 0.017°: the tire
    // side forces, the tail wheel's above all (it doesn't swivel, D-032), take
    // most of the brake's yaw moment. The turn depends on the load on the tail
    // wheel, and so on the attitude the rotor's flat-pitch cyclic gives (0.26°
    // and 0.51° with D-034's placeholder travels, 1.1° less nose-up).
    EXPECT_GT(drift, 0.0);
    EXPECT_LT(left, -3.0 * drift);  // nose left: heading decreases
    EXPECT_GT(right, 3.0 * drift);
}

// ---- Control rigging (layer 1) -----------------------------------------------------

// GenHel Figs. 5.1.6(a)–(d) and p. 5.5-20, the mechanical path in GenHel's
// own units (inches, degrees), test-side; XB = −XB+1 − XB+2 + XBOLS
// (verified by human review, DATA_GAPS.md).
struct GenHelRiggingRef {
    double KXCTH0 = 1.60, KXAA1S = 1.60, KXBB1S = -2.83, KXPTHR = -5.539;
    double BIAS_C = 9.9, BIAS_A = -8.0, BIAS_B = -9.7754, BIAS_P = 21.9;
    double MIX_AC = -0.16, MIX_BC = 0.29, MIX_BP = -0.2936;
    double SIGN_XB2 = -1.0;
    double DSP = -9.7;  // swashplate phase [deg] (p. 5.1-49)

    struct Out {
        double theta0, a1s, b1s, thettr;  // [deg]
    };
    Out operator()(double xc, double xa, double xb2, double xp) const {
        const double thosum = KXCTH0 * xc;
        const double thrsum = KXPTHR * xp;
        const double xb = SIGN_XB2 * xb2;
        return {thosum + BIAS_C, KXAA1S * xa + MIX_AC * thosum + BIAS_A,
                KXBB1S * xb + MIX_BC * thosum + MIX_BP * thrsum + BIAS_B, thrsum + thosum + BIAS_P};
    }
    // Blade pitch at the cuff, p. 5.1-24 (δ₃ = 0, no lag, no dynamic twist).
    double blade_pitch(const Out& o, double psi) const {
        const double p = psi + DSP * kDeg;
        return o.theta0 - o.a1s * std::cos(p) - o.b1s * std::sin(p);
    }
};

// The flight control system's mixer (D-045), with its servos at rest, gives
// GenHel's rigging, rotor head limits included (p. 5.5-20).
TEST(Uh60aAircraft, ControlMapsAreGenHelsRigging) {
    const Uh60aHarness ac;
    const auto& fcs = ac.get<GenHelFcs>("fcs");
    const GenHelRiggingRef g;
    using tr = test::Travels;
    BodyState b;
    b.pos_n = Vec3(0.0, 0.0, -500.0);
    const std::vector<double> x = ac.initial(b);
    std::vector<double> xf(fcs.num_states(), 0.0);
    // The servos' outputs at rest at the mixer's commands.
    const auto pitch = [&](const Controls& c) {
        const SimContext ctx = ac.context(0.0, x, c, Environment{});
        fcs.steady_states(ctx, xf);
        return fcs.evaluate(ctx, xf);
    };
    std::mt19937_64 rng(6004);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int limited = 0;
    for (int n = 0; n < 400; ++n) {
        Controls c;
        c.collective = 0.5 + 0.5 * u(rng);
        c.lat_cyclic = u(rng);
        c.lon_cyclic = u(rng);
        c.pedal = u(rng);
        auto o =
            g(tr::xc(c.collective), tr::xa(c.lat_cyclic), tr::xb2(c.lon_cyclic), tr::xp(c.pedal));
        const double b1s = std::clamp(o.b1s, -12.5, 16.3);
        limited += b1s != o.b1s ? 1 : 0;
        o.b1s = b1s;
        o.a1s = std::clamp(o.a1s, -8.0, 8.0);
        o.theta0 = std::clamp(o.theta0, 9.9, 25.9);
        const GenHelFcsOutputs p = pitch(c);
        for (int k = 0; k < 12; ++k) {
            const double psi = 2.0 * std::numbers::pi * k / 12;
            const double eyas = p.theta0 + p.theta1c * std::cos(psi) + p.theta1s * std::sin(psi);
            EXPECT_NEAR(eyas / kDeg, g.blade_pitch(o, psi), 1e-7);
        }
        EXPECT_NEAR(p.theta_tr / kDeg, o.thettr, 1e-7);  // travels written to 1e-9 in
    }
    EXPECT_GT(limited, 0);  // the B1S limits were exercised
    // Directions (Controls, include/eyas/types.hpp): forward stick tilts the
    // disc forward (θ1s down), right stick to the right (θ1c down for a
    // counter-clockwise rotor), right pedal lowers the tail-rotor pitch.
    Controls c;
    c.collective = 0.5;
    const GenHelFcsOutputs p0 = pitch(c);
    c.lon_cyclic = 0.1;
    EXPECT_LT(pitch(c).theta1s, p0.theta1s);
    c.lon_cyclic = 0.0;
    c.lat_cyclic = 0.1;
    // With the phase the lateral input also moves θ1s a little; the main
    // effect is on θ1c.
    EXPECT_LT(pitch(c).theta1c, p0.theta1c);
    c.lat_cyclic = 0.0;
    c.pedal = 0.1;
    EXPECT_LT(pitch(c).theta_tr, p0.theta_tr);
}

// ---- Hover yaw balance (D-028 whole aircraft, layer 2) -------------------------------

// Spec M6 "pedal balances torque", made into a sign and arm check (D-028):
// out of ground effect, body held level and at rest, cyclic centred, the
// collective and pedal found by a root search give zero vertical force and
// zero yaw moment. At that point the tail rotor's yaw moment, worked out by
// hand from its thrust, its cant and GenHel's stations, balances the other
// components' yaw moments; and every sign is the aircraft's. The rotor shaft
// is held at its governed speed: on the engines' free shaft (D-053) the
// held body has no steady state until they deliver the rotor's torque, and
// this is a check of the airframe's arms, not of the engines.
TEST(Uh60aAircraft, HoverPedalBalancesMainRotorTorque) {
    const Uh60aHarness ac(test::ballin::ideal_shaft_config());
    BodyState b;
    b.pos_n = Vec3(0.0, 0.0, -500.0);  // no ground (Environment::ground_height is NaN)
    std::vector<double> x = ac.initial(b);
    double t = 0.0;
    Controls c;
    c.collective = 0.5;
    c.pedal = 0.5;
    const Environment env;
    test::HeldBalance bal;
    ASSERT_TRUE(test::find_hover(ac, x, t, c, env, bal));
    EXPECT_GT(c.collective, 0.0);
    EXPECT_LT(c.collective, 1.0);
    EXPECT_GT(c.pedal, -1.0);
    EXPECT_LT(c.pedal, 1.0);

    const std::size_t i_mr = ac.index("main_rotor");
    const std::size_t i_tr = ac.index("tail_rotor");
    const Wrench& tr = bal.loads[i_tr];
    // Signs: a counter-clockwise main rotor turns the airframe nose right
    // (+N); the tail rotor on the right side pushes to the right (+Y), nose
    // left.
    EXPECT_GT(bal.loads[i_mr].M.z(), 0.0);
    EXPECT_GT(tr.F.y(), 0.0);
    EXPECT_LT(tr.M.z(), 0.0);

    // The arm, by hand. Thrust direction: canted Γ = 70° from the body y axis
    // toward −z (thrust up and to the right; DATA_GAPS.md). Position: the
    // config's main-rotor hub plus GenHel's station differences (FS, BL, WL
    // of the tail rotor minus the main rotor's; buttlines positive to port).
    const double gamma = 70.0 * kDeg;
    const Vec3 dir(0.0, std::sin(gamma), -std::cos(gamma));
    const double thrust = tr.F.norm();
    EXPECT_NEAR((tr.F - thrust * dir).norm(), 0.0, 1e-12 * thrust);
    const Vec3 r_tr = ac.get<BladeElementRotor>("main_rotor").config().hub_position_b +
                      Vec3(-(732.0 - 341.2), -(-14.0 - 0.0), -(324.7 - 315.0)) * units::kInch;
    const Vec3 m_hand = r_tr.cross(thrust * dir);
    EXPECT_NEAR((tr.M - m_hand).norm(), 0.0, 1e-9 * m_hand.norm());
    double others = 0.0;
    for (std::size_t i = 0; i < bal.loads.size(); ++i) {
        if (i != i_tr) {
            others += bal.loads[i].M.z();
        }
    }
    // Balanced to the root search's tolerance (0.5 N·m of ~5·10⁴).
    EXPECT_NEAR(m_hand.z(), -others, 1.0);
}

// ---- On the ground (D-028 whole aircraft, layer 2) ---------------------------------

// Spec M6 "sits stably on the ground with gear loads equal to weight"
// (D-028): dropped level from 0.3 m with collective down, cyclic centred,
// brakes set and the pedal that balances the yaw moment at that collective,
// it settles; the gear's ground loads then equal the weight minus the
// aerodynamic lift, and it stays where it is.
TEST(Uh60aAircraft, SitsStablyOnTheGround) {
    const Uh60aHarness ac;
    Controls c;
    {
        BodyState b;
        b.pos_n = Vec3(0.0, 0.0, -500.0);
        std::vector<double> x = ac.initial(b);
        double t = 0.0;
        ASSERT_TRUE(test::balance_pedal(ac, x, t, c, Environment{}));
    }
    c.brake_left = 1.0;
    c.brake_right = 1.0;

    Model m = Model::from_file(test::uh60a_config_path());
    const auto& gear = ac.get<LandingGear>("gear");
    double contact = 0.0;  // lowest tire bottom below the CG at free extension [m]
    for (const auto& g : gear.config().gears) {
        contact = std::max(contact, g.axle_b.z() + g.tire.radius);
    }
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -(contact + 0.3));
    ASSERT_TRUE(m.reset(s));
    Environment env;
    env.ground_height = 0.0;  // the origin altitude: ground at z_n = 0

    // Averages over a window of `seconds`, sampled every step.
    struct Window {
        double normal = 0.0;  // Σ gear ground normal load [N]
        double aero = 0.0;    // Σ aerodynamic F_z, NED (down) [N]
        Vec3 pos = Vec3::Zero();
        double heading = 0.0;
    };
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto window = [&](double seconds) {
        Window w;
        const int n = static_cast<int>(std::lround(seconds * 1000.0));
        for (int k = 0; k < n; ++k) {
            EXPECT_EQ(m.advance(c, env, 1'000'000).substeps, 1);
            const auto st = m.state();
            const SimContext ctx = ac.context(m.outputs().sim_time, st, c, env);
            const auto xg = ac.states(st, ac.index("gear"));
            for (std::size_t i = 0; i < gear.config().gears.size(); ++i) {
                w.normal += gear.gear_state(ctx, xg, i).normal_load / n;
            }
            // Aerodynamic loads only (the main rotor's blade weight is in the
            // total weight), rotated to NED.
            const Mat3 C_nb = ctx.body.q_nb.toRotationMatrix();
            const std::vector<Wrench> all = ac.evaluate(ctx.t, st, c, env);
            const Vec3 f_aero = mr.evaluate(ctx, ac.states(st, ac.index("main_rotor"))).body.F +
                                all[ac.index("airframe")].F + all[ac.index("tail_rotor")].F;
            w.aero += (C_nb * f_aero).z() / n;
            w.pos += ctx.body.pos_n / n;
            w.heading += m.outputs().heading / n;
        }
        return w;
    };
    (void)window(5.0);  // the drop and the first rocking
    const Window early = window(5.0);
    (void)window(5.0);
    const Window late = window(5.0);

    // Ground loads = weight − aerodynamic lift (lift is −F_z in NED), within
    // what the slow residual rocking (D-032) leaves in a 5 s average.
    const double weight = ac.total_weight();
    EXPECT_NEAR(late.normal, weight + late.aero, 2e-3 * weight);
    EXPECT_GT(-late.aero, 0.0);           // the rotor at flat pitch still lifts a little
    EXPECT_LT(-late.aero, 0.1 * weight);  // ... but most of the weight is on the wheels
    // It stays put: no creep between the windows 5–10 s and 15–20 s.
    EXPECT_LT((late.pos - early.pos).head<2>().norm(), 5e-3);
    EXPECT_NEAR(late.heading, early.heading, 1e-4);
}

// ---- Real-time rules (spec §7) ------------------------------------------------------

// The whole aircraft through the public API: no allocation in advance(),
// bit-identical repeat runs, and snapshot/restore, each across the first
// tire contact of the drop.
TEST(Uh60aAircraft, RealTimeRules) {
    Controls c;
    c.collective = 0.2;
    c.pedal = 0.5;
    c.brake_left = 1.0;
    c.brake_right = 1.0;
    Environment env;
    env.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);
    const auto run = [&](Model& m, int steps) {
        for (int k = 0; k < steps; ++k) {
            (void)m.advance(c, env, 16'666'667);  // 60 Hz host frames
        }
    };
    Model a = Model::from_file(test::uh60a_config_path());
    Model b = Model::from_file(test::uh60a_config_path());
    ASSERT_TRUE(a.reset(s));
    ASSERT_TRUE(b.reset(s));
    // Σ gear normal load [N] at the model's current state.
    const Uh60aHarness ac;
    const auto& gear = ac.get<LandingGear>("gear");
    const auto normal = [&](const Model& m) {
        const auto st = m.state();
        const SimContext ctx = ac.context(m.outputs().sim_time, st, c, env);
        double n = 0.0;
        for (std::size_t i = 0; i < gear.config().gears.size(); ++i) {
            n += gear.gear_state(ctx, ac.states(st, ac.index("gear")), i).normal_load;
        }
        return n;
    };
    run(a, 12);  // 0.2 s; first contact is at about 0.27 s
    ASSERT_EQ(normal(a), 0.0);
    auto snap = a.make_snapshot();
    ASSERT_TRUE(a.snapshot(snap));
    {
        const test::AllocationCounter count;
        run(a, 20);
        EXPECT_EQ(count.count(), 0U);
    }
    ASSERT_GT(normal(a), 0.0);  // the window took in the first contact
    const std::vector<double> first(a.state().begin(), a.state().end());
    run(b, 32);
    EXPECT_TRUE(std::equal(first.begin(), first.end(), b.state().begin()));
    ASSERT_TRUE(a.restore(snap));
    run(a, 20);
    EXPECT_TRUE(std::equal(first.begin(), first.end(), a.state().begin()));
}

}  // namespace
}  // namespace eyas
