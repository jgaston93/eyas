// M9 ground effect on the main-rotor inflow (D-050): GenHel CR-166309
// Eq. 5.8.2-1 and Cheeseman–Bennett as Johnson 2013 Eqs. 4.71 and 5.76 give
// it. Layer 1, the production factors against an independent transcription
// (tests/support/ground_effect_reference.hpp); layer 2, the properties the
// model and the inflow solve rely on (κ_g ≤ 1, rising to 1 with height,
// λ0/κ_g rising with λ0), the hover closed form λ0 = κ_g λ_h for both inflow
// models, the hub height from the body pose, no change with no ground; and
// the real-time rules.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <vector>

#include "airframe/landing_gear.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "eyas/model.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/ground_effect.hpp"
#include "support/alloc_counter.hpp"
#include "support/ground_effect_reference.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m9_ground_effect.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using test::airfoil_from;
using test::sea_level_context;
using test::test_rotor;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr GroundEffectModel kModels[] = {GroundEffectModel::genhel,
                                         GroundEffectModel::cheeseman_bennett};

double reference(GroundEffectModel m, double h, double mu, double lambda0) {
    return m == GroundEffectModel::genhel ? test::genhel_kge(h, mu, lambda0)
                                          : test::cheeseman_bennett_kappa(h, mu, lambda0);
}

double production(GroundEffectModel m, double h, double mu, double lambda0) {
    return ground_effect_factor(m, h, ground_effect_cos_chi(lambda0, mu));
}

// ---- The factors ---------------------------------------------------------------------

// Heights 0.1–6 R (below the 0.3 floor too), μ 0–0.3, λ0 −0.1–0.12 (both
// signs of thrust, and zero).
TEST(GroundEffect, FactorsMatchTheSourceTranscription) {
    for (const GroundEffectModel m : kModels) {
        for (int i = 0; i <= 59; ++i) {
            const double h = 0.1 + 0.1 * i;
            for (int j = 0; j <= 30; ++j) {
                const double mu = 0.01 * j;
                for (int k = 0; k <= 44; ++k) {
                    const double l0 = -0.1 + 0.005 * k;
                    EXPECT_NEAR(production(m, h, mu, l0), reference(m, h, mu, l0), 1e-14)
                        << static_cast<int>(m) << ' ' << h << ' ' << mu << ' ' << l0;
                }
            }
        }
    }
}

// Hand-worked hover values (cos χ = 1), and κ_g = 1 with no ground or no model.
TEST(GroundEffect, HoverValuesAndLimits) {
    // z/R = 1: GenHel 1.115^(−2/3); Cheeseman–Bennett (15/16)^(3/2).
    EXPECT_NEAR(production(GroundEffectModel::genhel, 1.0, 0.0, 0.05), 0.930001, 1e-6);
    EXPECT_NEAR(production(GroundEffectModel::cheeseman_bennett, 1.0, 0.0, 0.05),
                std::pow(15.0 / 16.0, 1.5), 1e-15);
    // USAAEFA's 2 ft wheel height, hub 14 ft above the ground (z/R = 0.522).
    EXPECT_NEAR(production(GroundEffectModel::genhel, 14.0 / 26.833, 0.0, 0.05), 0.7906, 1e-4);
    for (const GroundEffectModel m : kModels) {
        EXPECT_EQ(ground_effect_factor(m, kNaN, 1.0), 1.0);
        // Held at the floor below it.
        EXPECT_EQ(production(m, 0.1, 0.02, 0.05), production(m, 0.3, 0.02, 0.05));
        EXPECT_EQ(production(m, -2.0, 0.02, 0.05), production(m, 0.3, 0.02, 0.05));
        // No wake under the disc: λ0 = 0 in forward flight.
        EXPECT_EQ(production(m, 0.5, 0.1, 0.0), 1.0);
    }
    EXPECT_EQ(ground_effect_factor(GroundEffectModel::none, 0.5, 1.0), 1.0);
    EXPECT_EQ(ground_effect_cos_chi(0.0, 0.0), 1.0);
}

// κ_g in (0, 1], rising with height and falling with cos χ; within 0.4 % of
// 1 by 5R in hover (GenHel switches it off there, a 0.3 % step; Eyas lets
// it tend to 1).
TEST(GroundEffect, FactorRisesToOneWithHeight) {
    for (const GroundEffectModel m : kModels) {
        for (int j = 0; j <= 10; ++j) {
            const double c = 0.1 * j;
            double prev = 0.0;
            for (int i = 0; i <= 200; ++i) {
                const double h = 0.3 + 0.05 * i;
                const double k = ground_effect_factor(m, h, c);
                EXPECT_GT(k, 0.0);
                EXPECT_LE(k, 1.0);
                EXPECT_GE(k, prev) << h;
                if (j > 0) {
                    EXPECT_LE(k, ground_effect_factor(m, h, c - 0.1));
                }
                prev = k;
            }
        }
        EXPECT_GT(ground_effect_factor(m, 5.0, 1.0), 0.996);
    }
}

// The inflow equations are solved for λ0/κ_g(λ0). With cos χ from the
// induced inflow, κ_g falls as |λ0| rises, so d(λ0/κ_g)/dλ0 ≥ 1/κ_g ≥ 1 and
// the uniform solve keeps a unique root (D-050). Checked by differences,
// with the hub on the floor height where the factor varies most.
TEST(GroundEffect, OutOfGroundInflowRisesWithInflow) {
    for (const GroundEffectModel m : kModels) {
        for (const double h : {0.3, 0.45, 1.0}) {
            for (int j = 0; j <= 20; ++j) {
                const double mu = 0.005 * j;
                for (int k = 0; k < 400; ++k) {
                    const double a = -0.2 + 0.001 * k;
                    const double b = a + 0.001;
                    const double ga = a / production(m, h, mu, a);
                    const double gb = b / production(m, h, mu, b);
                    EXPECT_GE((gb - ga) / (b - a), 1.0 - 1e-9) << h << ' ' << mu << ' ' << a;
                }
            }
        }
    }
}

// ---- The rotor -------------------------------------------------------------------------

BladeElementRotorConfig ge_rotor(InflowModel inflow, VortexRingModel vrs, GroundEffectModel ge) {
    SwashplatePitch p;
    p.theta0 = 22.0 * kDeg;
    BladeElementRotorConfig c =
        test_rotor(airfoil_from(test::linear_airfoil_csv(5.73, 0.01)), p, 20);
    c.inflow = inflow;
    c.vortex_ring = vrs;
    c.ground_effect = ge;
    return c;
}

// A context with the hub `height` R above the ground; the body at rest,
// pitched and rolled so the hub position matters (the hub is 2 m above the
// CG and 0.5 m forward).
SimContext hover_context(const BladeElementRotor& rotor, double height) {
    SimContext ctx = sea_level_context(0.0);
    ctx.body.pos_n = Vec3(3.0, -1.0, -40.0);
    ctx.body.q_nb =
        Quat(Eigen::AngleAxisd(0.3, Vec3::UnitZ()) * Eigen::AngleAxisd(0.08, Vec3::UnitY()) *
             Eigen::AngleAxisd(-0.05, Vec3::UnitX()));
    const Vec3 hub_n = ctx.body.pos_n + ctx.body.q_nb * rotor.config().hub_position_b;
    ctx.ground_z_n = hub_n.z() + height * rotor.config().radius;
    return ctx;
}

BladeElementRotorConfig with_hub(BladeElementRotorConfig c) {
    c.hub_position_b = Vec3(0.5, 0.0, -2.0);
    return c;
}

// Hover (no velocity: the hub's flow is axial whatever the attitude), both
// models, with and without the vortex-ring correction (hover is outside its
// region): the uniform solve gives λ0 = κ_g λ_h with λ_h = √(C_T/2) from the
// thrust it lands on, and Pitt–Peters is at rest there.
TEST(GroundEffect, HoverInflowIsKappaTimesMomentum) {
    for (const GroundEffectModel m : kModels) {
        for (const VortexRingModel v : {VortexRingModel::none, VortexRingModel::johnson_2005}) {
            const BladeElementRotor uni(with_hub(ge_rotor(InflowModel::uniform, v, m)));
            const BladeElementRotor pp(with_hub(ge_rotor(InflowModel::pitt_peters, v, m)));
            for (const double h : {0.45, 0.6, 1.0, 2.0}) {
                const SimContext ctx = hover_context(uni, h);
                const RotorLoads l = uni.evaluate(ctx);
                ASSERT_TRUE(l.converged);
                const double kappa = ground_effect_factor(m, h, 1.0);
                EXPECT_NEAR(l.lambda_i, kappa * std::sqrt(0.5 * l.ct), 1e-13) << h;

                const std::size_t n = pp.num_states();
                std::vector<double> x(n, 0.0);
                x[0] = l.lambda_i;
                std::vector<double> xd(n, 0.0);
                std::vector<double> G(6 * n, 0.0);
                ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
                pp.derivatives(ctx, x, d);
                EXPECT_TRUE(d.converged);
                EXPECT_NEAR(xd[0], 0.0, 1e-9) << h;  // the uniform solve's tolerance
                EXPECT_NEAR(xd[1], 0.0, 1e-12);
                EXPECT_NEAR(xd[2], 0.0, 1e-12);
            }
        }
    }
}

// At fixed collective the thrust rises as the hub nears the ground, and
// out of ground effect (no ground) it equals the far-field limit.
TEST(GroundEffect, HoverThrustRisesNearTheGround) {
    for (const GroundEffectModel m : kModels) {
        const BladeElementRotor uni(
            with_hub(ge_rotor(InflowModel::uniform, VortexRingModel::johnson_2005, m)));
        double prev = 0.0;
        for (const double h : {40.0, 4.0, 2.0, 1.0, 0.7, 0.5, 0.4}) {
            const double ct = uni.evaluate(hover_context(uni, h)).ct;
            EXPECT_GT(ct, prev) << h;
            prev = ct;
        }
        SimContext free = hover_context(uni, 1.0);
        free.ground_z_n = kNaN;
        const double ct_free = uni.evaluate(free).ct;
        EXPECT_NEAR(uni.evaluate(hover_context(uni, 1e4)).ct, ct_free, 1e-10);
    }
}

// With no ground, or no model, the rates are the uncorrected rotor's to
// the bit, in hover, climb, forward flight and the vortex ring.
TEST(GroundEffect, NoGroundChangesNothing) {
    for (const InflowModel inflow : {InflowModel::uniform, InflowModel::pitt_peters}) {
        const BladeElementRotor plain(
            ge_rotor(inflow, VortexRingModel::johnson_2005, GroundEffectModel::none));
        const BladeElementRotor gh(
            ge_rotor(inflow, VortexRingModel::johnson_2005, GroundEffectModel::genhel));
        const std::size_t n = plain.num_states();
        for (const Vec3& v : {Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -8.0), Vec3(40.0, 3.0, 4.0),
                              Vec3(1.0, 0.0, 12.0)}) {
            SimContext ctx = sea_level_context(0.013);
            ctx.body.vel_b = v;
            std::vector<double> x = {0.06, 0.004, -0.007, 0.001};
            x.resize(n);
            std::vector<double> xa(n, 0.0);
            std::vector<double> xb(n, 0.0);
            std::vector<double> G(6 * n, 0.0);
            ComponentDerivs da{xa, G, Wrench{}, Mat66::Zero()};
            ComponentDerivs db{xb, G, Wrench{}, Mat66::Zero()};
            plain.derivatives(ctx, x, da);
            gh.derivatives(ctx, x, db);  // genhel, no ground
            for (std::size_t k = 0; k < n; ++k) {
                EXPECT_EQ(xa[k], xb[k]) << k;
            }
            EXPECT_EQ(da.load0_b.F, db.load0_b.F);
            EXPECT_EQ(da.load0_b.M, db.load0_b.M);
            ctx.ground_z_n = 5.0;  // none, with ground
            plain.derivatives(ctx, x, db);
            for (std::size_t k = 0; k < n; ++k) {
                EXPECT_EQ(xa[k], xb[k]) << k;
            }
        }
    }
}

// ---- The UH-60A ------------------------------------------------------------------------

// MSL height of the bottom of the UH-60A's left main wheel, level, with the
// CG at MSL 0: the axle at full extension plus the tire radius below it
// (USAAEFA measures wheel height there; ADA180181 App. E Fig. 2 note 1).
double left_wheel_bottom() {
    const test::Uh60aHarness ac;
    const GearConfig& left = ac.get<LandingGear>("gear").config().gears.front();
    return -(left.axle_b.z() + left.tire.radius);
}

// Hover with the ground 2.5 ft below the left main wheel as it sits level
// (about 2.7 ft at the hover attitude, hub about 0.54R up; at USAAEFA's 2 ft
// the tail wheel, the lowest point at the hover's 4.4° nose-up attitude, is
// 0.04 in off the ground): the trim converges off the wheels, the main rotor
// needs less power than out of ground effect (GenHel's κ_g = 0.80 there), and
// the aircraft flies on from the trim held in the same environment.
TEST(Uh60aGroundEffect, TrimsInGroundEffectAndFliesOn) {
    Model m = Model::from_file(test::uh60a_config_path());
    TrimCondition c;
    const TrimResult oge = m.trim(c);
    ASSERT_TRUE(oge.converged);
    const double p_oge = m.outputs().main_rotor_power;
    c.ground_height = left_wheel_bottom() - 2.5 * units::kFoot;
    const TrimResult r = m.trim(c, oge);
    ASSERT_TRUE(r.converged);
    EXPECT_FALSE(m.outputs().weight_on_wheels);
    const double p_ige = m.outputs().main_rotor_power;
    EXPECT_LT(p_ige, 0.9 * p_oge);
    EXPECT_GT(p_ige, 0.7 * p_oge);
    EXPECT_LT(r.controls.collective, oge.controls.collective);

    Environment env;
    env.ground_height = c.ground_height;
    const BodyState b0 = m.body_state();
    // 1 ms steps nearest 4 revolutions, so the 4/rev vibration cancels.
    const double period =
        2.0 * std::numbers::pi /
        test::Uh60aHarness().get<BladeElementRotor>("main_rotor").config().rotor_speed;
    const int steps = static_cast<int>(std::lround(4.0 * period * 1000.0));
    for (int k = 0; k < steps; ++k) {
        const AdvanceStatus s = m.advance(r.controls, env, 1'000'000);
        ASSERT_FALSE(s.non_finite);
    }
    const BodyState b1 = m.body_state();
    const double T = steps * 1e-3;
    EXPECT_LT(((b1.vel_b - b0.vel_b) / T).norm(), 5e-5);  // [m/s²], as Trim.Uh60aFliesOnFromTrim
    EXPECT_LT(((b1.omega_b - b0.omega_b) / T).norm(), 2e-5);  // [rad/s²]
}

// A ground plane above the wheels: the trim would rest on the gear, so it
// reports no convergence and leaves the model as it was, its outputs
// included (D-050).
TEST(Uh60aGroundEffect, TrimOnTheWheelsIsRejected) {
    Model m = Model::from_file(test::uh60a_config_path());
    Environment env;
    env.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);
    ASSERT_TRUE(m.reset(s));
    for (int k = 0; k < 10; ++k) {
        (void)m.advance(Controls{}, env, 16'666'667);
    }
    const std::vector<double> x0(m.state().begin(), m.state().end());
    const AircraftState out0 = m.outputs();
    TrimCondition c;
    c.ground_height = left_wheel_bottom() + 0.1;
    const TrimResult r = m.trim(c);
    EXPECT_FALSE(r.converged);
    EXPECT_EQ(std::vector<double>(m.state().begin(), m.state().end()), x0);
    EXPECT_EQ(m.outputs().sim_time_ns, out0.sim_time_ns);
    EXPECT_EQ(m.outputs().last_advance.substeps, out0.last_advance.substeps);
}

// A wheel that only grazes the ground: hovering at USAAEFA's 2 ft wheel
// height at sea level, the tail wheel (lowest at the 4.4° nose-up hover)
// clears it by 0.04 in at the start and touches during the revolution
// (4.8 N). The trim's residual converges there; solve_trim's check over the
// whole revolution must still reject it, for every caller (D-050).
TEST(Uh60aGroundEffect, GrazingWheelIsRejectedOverTheRevolution) {
    const test::Uh60aHarness ac = test::m9::aircraft("genhel");
    const test::m9::Point p = test::m9::trim_near_ground(ac, 0.0, 0.0, 2.0 * units::kFoot, nullptr);
    EXPECT_FALSE(p.converged);
    const test::m9::Point clear =
        test::m9::trim_near_ground(ac, 0.0, 0.0, 2.5 * units::kFoot, nullptr);
    EXPECT_TRUE(clear.converged);
    EXPECT_EQ(clear.gear_load, 0.0);
}

// Inputs the trim refuses (D-050): an infinite ground height, and flight
// that changes height near a ground (climb, descent or vertical wind),
// which the held body can't make steady. Linearizing near a ground without
// the height coordinate is refused too.
TEST(Uh60aGroundEffect, RejectsGroundConditionsItCannotHold) {
    const test::Uh60aHarness ac;
    const Executive& ex = ac.executive();
    std::vector<double> x;
    TrimCondition c;
    c.ground_height = std::numeric_limits<double>::infinity();
    EXPECT_THROW((void)solve_trim(ex, c, nullptr, x), InitError);
    c.ground_height = -std::numeric_limits<double>::infinity();
    EXPECT_THROW((void)solve_trim(ex, c, nullptr, x), InitError);
    c.ground_height = left_wheel_bottom() - 3.0;
    c.airspeed = 10.0;
    c.flight_path_angle = -0.1;
    EXPECT_THROW((void)solve_trim(ex, c, nullptr, x), InitError);
    c.flight_path_angle = 0.0;
    c.wind_n = Vec3(0.0, 0.0, 0.5);
    EXPECT_THROW((void)solve_trim(ex, c, nullptr, x), InitError);
    // Level flight over the ground, and a descent with no ground, are fine.
    c.wind_n = Vec3::Zero();
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    ASSERT_TRUE(r.converged);
    LinearizeOptions o;
    o.height = false;
    EXPECT_THROW((void)linearize(ex, c, r, o), InitError);
}

// The gate against flight test (D-050; spec owner's choice, 2026-10-21): the
// UH-60A's hover power ratio P(2 ft)/P(100 ft) wheel height at the same
// weight, with GenHel's model, within 0.05 of the ratio of USAAEFA's fits
// (ADA180181 App. E Figs. 2 and 3) at density altitudes 0, 6000 and 12 000 ft
// (C_W 0.0059–0.0085, inside both fits' data). Eyas's is extrapolated from
// 2.5 and 3 ft, where no wheel touches (m9::hover_ratio). Measured 0.038,
// 0.035 and 0.046 above: GenHel's factor reduces the power too little
// (VALIDATION.md M9); the gate guards against it getting worse.
TEST(Uh60aGroundEffect, HoverPowerRatioAgainstUsaaefa) {
    const test::Uh60aHarness ac = test::m9::aircraft("genhel");
    for (const double alt_ft : {0.0, 6000.0, 12000.0}) {
        SCOPED_TRACE(alt_ft);
        const test::m9::HoverRatio r = test::m9::hover_ratio(ac, alt_ft * units::kFoot);
        ASSERT_TRUE(r.ok);
        const double flight = test::m9::usaaefa_cp_2ft(r.cw) / test::m9::usaaefa_cp_100ft(r.cw);
        std::printf("C_W %.5f: Eyas %.4f, USAAEFA %.4f\n", r.cw, r.ratio, flight);
        EXPECT_NEAR(r.ratio, flight, 0.05);
    }
}

// ---- Real-time rules -------------------------------------------------------------------

TEST(GroundEffect, AllocatesNothing) {
    const BladeElementRotor pp(with_hub(ge_rotor(
        InflowModel::pitt_peters, VortexRingModel::johnson_2005, GroundEffectModel::genhel)));
    const BladeElementRotor uni(
        with_hub(ge_rotor(InflowModel::uniform, VortexRingModel::johnson_2005,
                          GroundEffectModel::cheeseman_bennett)));
    SimContext ctx = hover_context(pp, 0.5);
    ctx.body.vel_b = Vec3(2.0, 0.0, 3.0);
    std::vector<double> x = {0.09, 0.0, 0.0, 0.001};
    std::vector<double> xd(4, 0.0);
    std::vector<double> G(24, 0.0);
    ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
    test::AllocationCounter counter;
    for (int i = 0; i < 10; ++i) {
        pp.derivatives(ctx, x, d);
        (void)uni.evaluate(ctx);
    }
    EXPECT_EQ(counter.count(), 0U);
}

}  // namespace
}  // namespace eyas
