// M7a acceptance tests for the periodic trim (D-035): the UH-60A trims from
// hover to 150 kt (T1), flies on from trim (T2), and the trim is independent
// of the integration step, of the heading and of a steady wind, as the
// physics requires. The geometry of the trimmed body state and the Model API
// are checked on their own.

#include <cmath>
#include <numbers>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "eyas/model.hpp"
#include "sim/build.hpp"
#include "support/alloc_counter.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 1852.0 / 3600.0;  // [m/s]
constexpr double kFpm = 0.3048 / 60.0;     // [m/s] per ft/min

Executive uh60a_executive() {
    const Config cfg = Config::from_file(test::uh60a_config_path());
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

TrimCondition level(double knots) {
    TrimCondition c;
    c.airspeed = knots * kKnot;
    return c;
}

// The air velocity of a trimmed body state, in body axes [m/s].
Vec3 air_velocity_b(const BodyState& b, const TrimCondition& c) {
    return b.vel_b - b.q_nb.toRotationMatrix().transpose() * c.wind_n;
}

// ---- Geometry of the trimmed body state -----------------------------------------

// The body state has the condition's airspeed, sideslip and flight-path angle
// (relative to the air), the attitude asked for, and the ground velocity adds
// the wind: random attitudes, conditions and winds.
TEST(Trim, BodyStateHasTheConditionsAirVelocity) {
    std::mt19937_64 rng(35);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    int checked = 0;
    for (int k = 0; k < 500; ++k) {
        TrimCondition c;
        c.airspeed = 80.0 * (0.5 + 0.5 * u(rng));
        c.flight_path_angle = 0.4 * u(rng);
        c.sideslip = 0.4 * u(rng);
        c.heading = 3.0 * u(rng);
        c.altitude = 1000.0;
        c.wind_n = Vec3(10.0 * u(rng), 10.0 * u(rng), 2.0 * u(rng));
        const double pitch = 0.3 * u(rng);
        const double roll = 0.4 * u(rng);
        BodyState b;
        if (!trim_body_state(c, pitch, roll, 200.0, b)) {
            continue;  // no track gives this sideslip at this attitude
        }
        ++checked;
        const EulerAngles e = euler_angles(b.q_nb);
        EXPECT_NEAR(e.pitch, pitch, 1e-12);
        EXPECT_NEAR(e.roll, roll, 1e-12);
        EXPECT_NEAR(std::remainder(e.heading - c.heading, 2.0 * std::numbers::pi), 0.0, 1e-12);
        EXPECT_EQ(b.omega_b, Vec3::Zero());
        EXPECT_DOUBLE_EQ(b.pos_n.z(), -(1000.0 - 200.0));
        const Vec3 va_b = air_velocity_b(b, c);
        const Vec3 va_n = b.q_nb * va_b;
        EXPECT_NEAR(va_b.norm(), c.airspeed, 1e-9);
        EXPECT_NEAR(std::asin(va_b.y() / c.airspeed), c.sideslip, 1e-9);
        EXPECT_NEAR(std::asin(-va_n.z() / c.airspeed), c.flight_path_angle, 1e-9);
        EXPECT_GT(va_b.x(), 0.0);  // the forward root
    }
    EXPECT_GT(checked, 400);
}

// With the body y axis vertical (φ = 90°) no track changes v_air_b,y, so a
// sideslip it doesn't already have has no body state; one it has (here 0 in
// level flight) is met on the heading.
TEST(Trim, BodyStateWithTheBodyYAxisVertical) {
    TrimCondition c = level(60);
    const double roll = 0.5 * std::numbers::pi;
    BodyState b;
    c.sideslip = 0.2;
    EXPECT_FALSE(trim_body_state(c, 0.0, roll, 0.0, b));
    c.sideslip = 0.0;
    ASSERT_TRUE(trim_body_state(c, 0.0, roll, 0.0, b));
    EXPECT_NEAR(air_velocity_b(b, c).y(), 0.0, 1e-9);
    EXPECT_NEAR(air_velocity_b(b, c).norm(), c.airspeed, 1e-9);
}

// ---- T1: the UH-60A trims from hover to 150 kt ------------------------------------

// Spec M7, D-035 T1: level flight at ISA sea level from 0 to 150 kt in 10 kt
// steps, each from the previous trim; then ±1000 ft/min at 60 and 100 kt and
// ±10° of sideslip at 60 kt, from the level trim at that speed; and 150 kt from
// a cold start. Each converges with residuals within the solver's tolerances,
// and its body state has the condition's air velocity.
TEST(Trim, Uh60aTrimsFromHoverTo150Knots) {
    const Executive ex = uh60a_executive();
    const double weight = ex.airframe_mass().mass * units::kStandardGravity;
    const auto expect_trimmed = [&](const TrimCondition& c, const TrimResult& r,
                                    const std::vector<double>& x) {
        SCOPED_TRACE(testing::Message()
                     << c.airspeed / kKnot << " kt, γ " << c.flight_path_angle / kDeg << "°, β "
                     << c.sideslip / kDeg << "°");
        ASSERT_TRUE(r.converged);
        EXPECT_LE(r.iterations, 10);
        EXPECT_LT(r.force_residual, 1e-7 * weight);
        EXPECT_LT(r.periodic_residual, 1e-8);
        // The mean body state (without the vibration start, D-035) has the
        // condition's air velocity.
        BodyState b = unpack_body(x);
        b.vel_b = r.vel_b;
        const Vec3 va_b = air_velocity_b(b, c);
        EXPECT_NEAR(va_b.norm(), c.airspeed, 1e-9);
        if (c.airspeed > 0.0) {
            EXPECT_NEAR(std::asin(va_b.y() / c.airspeed), c.sideslip, 1e-9);
            EXPECT_NEAR(std::asin(-(b.q_nb * va_b).z() / c.airspeed), c.flight_path_angle, 1e-9);
        }
        // Controls inside their travel.
        EXPECT_GT(r.controls.collective, 0.0);
        EXPECT_LT(r.controls.collective, 1.0);
        for (const double v : {r.controls.lat_cyclic, r.controls.lon_cyclic, r.controls.pedal}) {
            EXPECT_LT(std::abs(v), 1.0);
        }
    };
    std::vector<TrimResult> sweep;
    std::vector<double> x;
    for (int kt = 0; kt <= 150; kt += 10) {
        const TrimCondition c = level(kt);
        sweep.push_back(solve_trim(ex, c, sweep.empty() ? nullptr : &sweep.back(), x));
        expect_trimmed(c, sweep.back(), x);
    }
    for (const int kt : {60, 100}) {
        for (const double fpm : {-1000.0, 1000.0}) {
            TrimCondition c = level(kt);
            c.flight_path_angle = std::asin(fpm * kFpm / c.airspeed);
            const TrimResult r = solve_trim(ex, c, &sweep[static_cast<std::size_t>(kt / 10)], x);
            expect_trimmed(c, r, x);
            // A climb takes more collective than level flight, a descent less.
            const double level_collective =
                sweep[static_cast<std::size_t>(kt / 10)].controls.collective;
            if (fpm > 0.0) {
                EXPECT_GT(r.controls.collective, level_collective);
            } else {
                EXPECT_LT(r.controls.collective, level_collective);
            }
        }
    }
    for (const double beta : {-10.0, 10.0}) {
        TrimCondition c = level(60);
        c.sideslip = beta * kDeg;
        expect_trimmed(c, solve_trim(ex, c, &sweep[6], x), x);
    }
    const TrimCondition fast = level(150);
    const TrimResult cold = solve_trim(ex, fast, nullptr, x);
    expect_trimmed(fast, cold, x);
    // The same trim as the one reached by continuation.
    EXPECT_NEAR(cold.controls.collective, sweep.back().controls.collective, 1e-6);
    EXPECT_NEAR(cold.controls.lon_cyclic, sweep.back().controls.lon_cyclic, 1e-6);
    EXPECT_NEAR(cold.pitch, sweep.back().pitch, 1e-6);
}

// ---- Independence of step, heading and steady wind ---------------------------------

// The trim doesn't depend on the integration step: halving it (T/512) changes
// the controls and attitude by far less than anything measurable in flight.
TEST(Trim, IndependentOfTheIntegrationStep) {
    const Executive ex = uh60a_executive();
    std::vector<double> x;
    for (const int kt : {0, 120}) {
        const TrimResult a = solve_trim(ex, level(kt), nullptr, x);
        TrimOptions fine;
        fine.steps_per_period = 512;
        const TrimResult b = solve_trim(ex, level(kt), &a, x, fine);
        ASSERT_TRUE(a.converged);
        ASSERT_TRUE(b.converged);
        SCOPED_TRACE(kt);
        EXPECT_NEAR(a.controls.collective, b.controls.collective, 1e-6);
        EXPECT_NEAR(a.controls.lat_cyclic, b.controls.lat_cyclic, 1e-6);
        EXPECT_NEAR(a.controls.lon_cyclic, b.controls.lon_cyclic, 1e-6);
        EXPECT_NEAR(a.controls.pedal, b.controls.pedal, 1e-6);
        // Measured: up to 2.7e-6° (roll at 120 kt), RK4's h⁴ error.
        EXPECT_NEAR(a.pitch, b.pitch, 1e-5 * kDeg);
        EXPECT_NEAR(a.roll, b.roll, 1e-5 * kDeg);
    }
}

// The loads depend on the air velocity and the attitude relative to the
// vertical only (flat earth, uniform wind), so a trim at a different heading,
// or in a steady wind at the same airspeed, has the same controls and
// attitude. A 30 kt airspeed into a 30 kt headwind is a hover over the ground.
TEST(Trim, IndependentOfHeadingAndSteadyWind) {
    const Executive ex = uh60a_executive();
    std::vector<double> x;
    const TrimResult ref = solve_trim(ex, level(30), nullptr, x);
    ASSERT_TRUE(ref.converged);
    TrimCondition turned = level(30);
    turned.heading = 1.2;
    TrimCondition into_wind = level(30);
    into_wind.heading = 0.7;
    // The track lies close to the heading; a wind along the air velocity's
    // ground track cancels the ground speed.
    BodyState b;
    ASSERT_TRUE(trim_body_state(into_wind, ref.pitch, ref.roll, 0.0, b));
    into_wind.wind_n = -(b.q_nb * b.vel_b);
    for (const TrimCondition& c : {turned, into_wind}) {
        const TrimResult r = solve_trim(ex, c, &ref, x);
        ASSERT_TRUE(r.converged);
        EXPECT_NEAR(r.controls.collective, ref.controls.collective, 1e-6);
        EXPECT_NEAR(r.controls.lat_cyclic, ref.controls.lat_cyclic, 1e-6);
        EXPECT_NEAR(r.controls.lon_cyclic, ref.controls.lon_cyclic, 1e-6);
        EXPECT_NEAR(r.controls.pedal, ref.controls.pedal, 1e-6);
        EXPECT_NEAR(r.pitch, ref.pitch, 1e-6 * kDeg);
        EXPECT_NEAR(r.roll, ref.roll, 1e-6 * kDeg);
    }
    // The headwind case hovers over the ground.
    const TrimResult r = solve_trim(ex, into_wind, &ref, x);
    EXPECT_LT(r.vel_b.norm(), 1e-4);  // [m/s]; the wind was set from the reference attitude
}

// ---- T2: free flight from trim -----------------------------------------------------

// D-035 T2: from each trim, the Model flies on (advance() at 1 kHz). Over the
// first four revolutions the body's mean accelerations stay below bounds set
// from the residual study in VALIDATION.md M7a. Up to 60 kt they are at the
// solver's tolerance; above, the held-body trim leaves out the blades'
// response to the airframe's own 4/rev vibration, which grows with speed.
TEST(Trim, Uh60aFliesOnFromTrim) {
    Model m = Model::from_file(test::uh60a_config_path());
    TrimResult prev;
    bool have = false;
    const double period = trim_period(uh60a_executive());  // [s]
    const auto revolution_steps = [&](int revs) {          // 1 ms steps nearest `revs` revolutions
        return static_cast<int>(std::lround(revs * period * 1000.0));
    };
    for (int kt = 0; kt <= 150; kt += 30) {
        const TrimCondition c = level(kt);
        const TrimResult r = have ? m.trim(c, prev) : m.trim(c);
        ASSERT_TRUE(r.converged);
        prev = r;
        have = true;
        EXPECT_EQ(m.outputs().sim_time_ns, 0);
        const BodyState b0 = m.body_state();
        const int steps = revolution_steps(4);
        for (int k = 0; k < steps; ++k) {
            const AdvanceStatus s = m.advance(r.controls, Environment{}, 1'000'000);
            ASSERT_EQ(s.substeps, 1);
            ASSERT_FALSE(s.non_finite);
        }
        const BodyState b1 = m.body_state();
        const double T = steps * 1e-3;
        const double vdot = ((b1.vel_b - b0.vel_b) / T).norm();      // [m/s²]
        const double wdot = ((b1.omega_b - b0.omega_b) / T).norm();  // [rad/s²]
        SCOPED_TRACE(kt);
        if (kt <= 60) {
            EXPECT_LT(vdot, 5e-5);
            EXPECT_LT(wdot, 2e-5);
        } else {
            EXPECT_LT(vdot, 2.5e-3);  // measured ≤ 1.2e-3
            EXPECT_LT(wdot, 1e-3);    // measured ≤ 2.8e-4
        }
    }
}

// ---- Model API -----------------------------------------------------------------------

// trim() resets the model to t = 0 at the trimmed state; advance() then keeps
// the real-time rules: no allocation and bit-identical repeat runs.
TEST(Trim, ModelStartsFromTrimAndKeepsTheRealTimeRules) {
    Model m = Model::from_file(test::uh60a_config_path());
    Controls c;
    for (int k = 0; k < 50; ++k) {
        (void)m.advance(c, Environment{}, 1'000'000);
    }
    const TrimResult r = m.trim(level(80));
    ASSERT_TRUE(r.converged);
    EXPECT_EQ(m.outputs().sim_time_ns, 0);
    EXPECT_NEAR(m.outputs().pitch, r.pitch, 1e-12);
    EXPECT_NEAR(m.outputs().roll, r.roll, 1e-12);
    const std::vector<double> x0(m.state().begin(), m.state().end());
    for (std::size_t i = body_layout::kSize; i < x0.size(); ++i) {
        EXPECT_DOUBLE_EQ(x0[i], r.component_states[i - body_layout::kSize]);
    }

    Snapshot s = m.make_snapshot();
    const auto run = [&] {
        for (int k = 0; k < 100; ++k) {
            (void)m.advance(r.controls, Environment{}, 1'000'000);
        }
        return std::vector<double>(m.state().begin(), m.state().end());
    };
    std::vector<double> first;
    {
        const test::AllocationCounter count;
        first = run();
        // The vector copy in run() is the only allocation.
        EXPECT_EQ(count.count(), 1U);
    }
    ASSERT_TRUE(m.restore(s));
    EXPECT_EQ(first, run());
}

// An invalid condition is rejected with InitError and the model is left as
// it was; so is a starting point from a different model.
TEST(Trim, RejectsInvalidInputsAndLeavesTheModel) {
    Model m = Model::from_file(test::uh60a_config_path());
    const std::vector<double> x0(m.state().begin(), m.state().end());
    TrimCondition bad = level(60);
    bad.airspeed = -1.0;
    EXPECT_THROW((void)m.trim(bad), InitError);
    bad = level(60);
    bad.sideslip = 0.5 * std::numbers::pi;
    EXPECT_THROW((void)m.trim(bad), InitError);
    bad = level(10);
    bad.flight_path_angle = 0.5 * std::numbers::pi;  // vertical: β follows from the attitude
    bad.sideslip = 0.1;
    EXPECT_THROW((void)m.trim(bad), InitError);
    bad = level(60);
    bad.isa_delta_T = -300.0;
    EXPECT_THROW((void)m.trim(bad), InitError);
    TrimResult foreign;
    foreign.component_states.assign(3, 0.0);
    EXPECT_THROW((void)m.trim(level(60), foreign), InitError);
    EXPECT_EQ(std::vector<double>(m.state().begin(), m.state().end()), x0);
}

// A trim that doesn't converge reports it and leaves the model unchanged
// (here: no Newton iterations allowed).
TEST(Trim, UnconvergedTrimIsReported) {
    const Executive ex = uh60a_executive();
    TrimOptions none;
    none.max_iterations = 0;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, level(100), nullptr, x, none);
    EXPECT_FALSE(r.converged);
    EXPECT_EQ(r.iterations, 0);
    EXPECT_GT(r.force_residual, 1.0);  // [N]
}

// A starting point whose attitude admits no track with the condition's
// sideslip gives no residual at all: the result is unconverged with NaN
// residuals, not zero ones, and the model is unchanged.
TEST(Trim, StartWithNoValidResidualIsReported) {
    Model m = Model::from_file(test::uh60a_config_path());
    const std::vector<double> x0(m.state().begin(), m.state().end());
    TrimCondition c = level(60);
    c.sideslip = 0.5;  // sin β = 0.48 > cos φ = 0.17: no track
    TrimResult start;
    start.roll = 80.0 * kDeg;
    start.component_states.assign(x0.size() - body_layout::kSize, 0.0);
    const TrimResult r = m.trim(c, start);
    EXPECT_FALSE(r.converged);
    EXPECT_EQ(r.iterations, 0);
    EXPECT_TRUE(std::isnan(r.force_residual));
    EXPECT_TRUE(std::isnan(r.moment_residual));
    EXPECT_TRUE(std::isnan(r.periodic_residual));
    EXPECT_EQ(std::vector<double>(m.state().begin(), m.state().end()), x0);
}

}  // namespace
}  // namespace eyas
