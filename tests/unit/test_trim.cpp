// M7a acceptance tests for the periodic trim (D-035): the UH-60A trims from
// hover to 150 kt (T1), flies on from trim (T2), and the trim is independent
// of the integration step, of the heading and of a steady wind, as the
// physics requires. The geometry of the trimmed body state and the Model API
// are checked on their own.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <random>
#include <span>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "control/genhel_fcs.hpp"
#include "core/attitude.hpp"
#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "engine/genhel_t700.hpp"
#include "eyas/model.hpp"
#include "sim/build.hpp"
#include "support/alloc_counter.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7_fixtures.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

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
        // From a cold start the rotor on its engines takes two stages (D-053
        // amendment 2): measured 10 in hover, 11 at 150 kt, the most here.
        EXPECT_LE(r.iterations, 15);
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

// ---- The rotor on engines (D-053) ----------------------------------------------------

// The UH-60A's rotor turns on a free shaft driven by GenHel's T700s. At 5250 ft
// the load-demand spindle schedules more gas-generator speed than the hover
// needs and the ECU trims it down, holding the governed 27 rad/s exactly
// (isochronous). At sea level it schedules 0.17 % less than needed; the ECU
// can only trim down (A_U = 0), so it saturates and the rotor droops (D-053
// amendment). Either way the trim's revolution is one turn of the rotor at
// the speed it trims to: Ω T = 2π to the 4/rev ripple in Ω, the shaft's
// azimuth closes, and the switch integrator rests at a limit.
TEST(Trim, Uh60aRotorOnEnginesClosesItsRevolution) {
    const Executive ex = uh60a_executive();
    const GenHelT700* engine = nullptr;
    std::size_t first = 0;
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* e = dynamic_cast<const GenHelT700*>(&ex.component(i))) {
            engine = e;
            first = ex.component_offset(i) - body_layout::kSize;
        }
    }
    ASSERT_NE(engine, nullptr);
    ASSERT_EQ(ex.shafts().size(), 1U);
    ASSERT_EQ(ex.shafts()[0].drive, ShaftDrive::free);
    const std::size_t omega_at = ex.shaft_offset(0) + 1 - body_layout::kSize;
    const double governed = ex.shafts()[0].speed;
    std::vector<double> x;
    for (const double feet : {0.0, 5250.0}) {
        SCOPED_TRACE(feet);
        TrimCondition c = level(0);
        c.altitude = feet * units::kFoot;
        const TrimResult r = solve_trim(ex, c, nullptr, x);
        ASSERT_TRUE(r.converged);
        EXPECT_LT(r.periodic_residual, 1e-8);
        const double omega = r.component_states[omega_at];
        EXPECT_NEAR(omega * r.period, 2.0 * std::numbers::pi, 1e-5);
        EXPECT_NEAR(trim_period(ex, r), r.period, 0.0);
        const double q_int = r.component_states[first + GenHelT700::kQint];
        EXPECT_TRUE(q_int == 0.0 || q_int == engine->config().qint_limit) << q_int;
        const double v_ss = r.component_states[first + GenHelT700::kVss];
        if (feet == 0.0) {
            // Measured: Ω = 26.944 rad/s, the ECU at A_U (a limiter past its
            // limit returns to it, D-053 amendment 2).
            EXPECT_NEAR(v_ss, engine->config().upper, 1e-9);
            EXPECT_LT(omega, governed * (1.0 - 1e-3));
            EXPECT_GT(omega, governed * (1.0 - 5e-3));
        } else {
            EXPECT_LT(v_ss, engine->config().upper - 1e-3);
            EXPECT_NEAR(omega, governed, 1e-6 * governed);
        }
    }
}

// Code review of #69: the two stages of a cold start (D-053 amendment 2) each
// have the iteration budget, so a first stage that uses much of it can't
// leave the second none. The cold hover takes 4 then 6 (measured); with a
// budget of 6 a counter shared by both stages stopped it short.
TEST(Trim, ColdStartStagesHaveTheirOwnIterationBudget) {
    const Executive ex = uh60a_executive();
    TrimOptions o;
    o.max_iterations = 6;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, level(0), nullptr, x, o);
    EXPECT_TRUE(r.converged);
    EXPECT_GT(r.iterations, 6);  // both stages, counted together
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
// The flight control system flies too, its sensors and servos started on
// their free-flight orbit (D-045 amendment).
TEST(Trim, Uh60aFliesOnFromTrim) {
    Model m = Model::from_file(test::uh60a_config_path());
    TrimResult prev;
    bool have = false;
    const double period = trim_period(uh60a_executive());  // [s]
    const auto revolution_steps = [&](int revs) {          // 1 ms steps nearest `revs` revolutions
        return static_cast<int>(std::lround(revs * period * 1000.0));
    };
    for (int kt = 0; kt <= 150; kt += 30) {
        SCOPED_TRACE(kt);
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
        if (kt <= 60) {
            EXPECT_LT(vdot, 5e-5);
            EXPECT_LT(wdot, 2e-5);
        } else {
            EXPECT_LT(vdot, 2.5e-3);  // measured ≤ 1.2e-3
            EXPECT_LT(wdot, 1e-3);    // measured ≤ 2.8e-4
        }
    }
}

// D-045 amendment: the trim starts the flight control system's sensors and
// servos on their free-flight 4/rev orbit, not at their held-body values, so
// free flight from it has no start-up transient through the stabilator's
// lateral-acceleration feedback. Over the first 2 s the incidence moves less
// than 0.002° (measured 1e-5° at 60 kt and 0.0004° at 120 kt, the slow drift
// of the free aircraft; from the held-body values it was 0.026° at 120 kt)
// and the rotor pitch less than 1e-4 rad (servos at rest on unchanged
// commands).
TEST(Trim, FcsStartTransientIsSmall) {
    Model m = Model::from_file(test::uh60a_config_path());
    for (const int kt : {60, 120}) {
        SCOPED_TRACE(kt);
        const TrimResult r = m.trim(level(kt));
        ASSERT_TRUE(r.converged);
        const AircraftState o0 = m.outputs();
        double stab = 0.0;
        double pitch = 0.0;
        for (int k = 0; k < 2000; ++k) {
            ASSERT_FALSE(m.advance(r.controls, Environment{}, 1'000'000).non_finite);
            const AircraftState o = m.outputs();
            stab = std::max(stab, std::abs(o.stabilator_incidence - o0.stabilator_incidence));
            pitch = std::max({pitch, std::abs(o.main_rotor_theta0 - o0.main_rotor_theta0),
                              std::abs(o.main_rotor_theta1s - o0.main_rotor_theta1s),
                              std::abs(o.main_rotor_theta1c - o0.main_rotor_theta1c),
                              std::abs(o.tail_rotor_theta0 - o0.tail_rotor_theta0)});
        }
        EXPECT_LT(stab, 0.002 * units::kDegree);
        EXPECT_LT(pitch, 1e-4);
    }
}

// The trim starts the flight control system on its free-flight orbit
// (D-045 amendment), which it finds whether or not that orbit attracts: an
// unstable loop would trim as cleanly. So from each trim the stabilator
// servo is kicked by 0.5°, with the accelerometer and pitch gyro states
// (+0.3 m/s², +0.01 rad/s), and the kick must die out: after 1.5 s the
// incidence stays within 0.25° of the unkicked flight's (measured ≤ 0.09°,
// the bare airframe's slow motion fed back through the gyro; the servo and
// sensors settle in about 1 s). In hover and at 30 kt the stabilator rests
// on its 39° stop, where the kick doesn't reach the incidence.
TEST(Trim, FcsKickDiesOut) {
    const Executive ex = uh60a_executive();
    std::size_t off = 0;
    while (dynamic_cast<const GenHelFcs*>(&ex.component(off)) == nullptr) {
        ++off;
    }
    off = ex.component_offset(off);
    const std::size_t stab = off + GenHelFcs::kServoStab;
    TrimResult prev;
    bool have = false;
    for (const int kt : {60, 90, 120, 150}) {
        SCOPED_TRACE(kt);
        const TrimCondition c = level(kt);
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
        ASSERT_TRUE(r.converged);
        prev = r;
        have = true;
        const Environment env = trim_environment(c);
        const double dt = trim_period(ex) / 256;
        const int steps = static_cast<int>(3.0 / dt);
        const auto controls = [&](double) { return r.controls; };
        std::size_t missed = 0;
        const auto base = test::free_flight(ex, env, x, controls, dt, steps, &missed);
        x[off + GenHelFcs::kAccelY] += 0.3;
        x[off + GenHelFcs::kGyroQ] += 0.01;
        x[stab] += 0.5 * units::kDegree;
        const auto kicked = test::free_flight(ex, env, x, controls, dt, steps, &missed);
        EXPECT_EQ(missed, 0U);
        double late = 0.0;
        for (int n = static_cast<int>(1.5 / dt); n <= steps; ++n) {
            const auto i = static_cast<std::size_t>(n);
            late = std::max(late, std::abs(kicked[i][stab] - base[i][stab]));
        }
        EXPECT_LT(late, 0.25 * units::kDegree);
    }
}

// M8c2 (D-045): with the SAS and PBA on, the trimmed aircraft is the same
// (attitude, velocity, blade pitch: the SAS adds inches ahead of the mixer,
// and the trim finds the blade pitch the forces need); only the stick moves,
// by the SAS and PBA steady output. That output is written here from
// GenHel's diagrams in GenHel's units (Figs. 5.1.3(a)–(c), 5.1.4): the rate
// paths are washed out or zero in straight flight; what is left is the roll
// attitude (digital KPHRD φ, analog KPHIRA KPHR1 KPHF1 φ), the yaw lateral
// acceleration above 60 kt (digital KLAYYD, analog .5 KLAYRR KLAYF1, on
// Y.SENS), and the PBA (KTHBA θ + KVXBA (VXBIKT − 80) + BIAS). Both SAS channels on:
// each at gain 1 (p. 5.5-2). Tolerances: the trims converge to 1e-7 of the
// weight in force, so the two solves agree to about 1e-7 rad.
TEST(Trim, SasAndPbaMoveOnlyTheStick) {
    const Executive ex = uh60a_executive();
    std::size_t fcs = 0;
    while (dynamic_cast<const GenHelFcs*>(&ex.component(fcs)) == nullptr) {
        ++fcs;
    }
    const GenHelFcsConfig& cfg = dynamic_cast<const GenHelFcs&>(ex.component(fcs)).config();
    const std::size_t off = ex.component_offset(fcs) - body_layout::kSize;  // in component_states
    constexpr double kFt = 0.3048;
    constexpr double kIn = 0.0254;
    const auto half = [](const StickTravel& t) { return 0.5 * (t.hi - t.lo) / kIn; };  // [in]
    const GenHelRigging& rig = cfg.rigging;
    for (const int kt : {0, 100, 150}) {
        SCOPED_TRACE(kt);
        std::vector<double> x;
        const TrimResult bare = solve_trim(ex, level(kt), nullptr, x);
        ASSERT_TRUE(bare.converged);
        TrimCondition c = level(kt);
        c.afcs = {true, true, true};
        const TrimResult on = solve_trim(ex, c, &bare, x);
        ASSERT_TRUE(on.converged);
        EXPECT_TRUE(on.controls.afcs.sas_digital && on.controls.afcs.sas_analog &&
                    on.controls.afcs.pitch_bias);
        EXPECT_NEAR(on.pitch, bare.pitch, 1e-6);
        EXPECT_NEAR(on.roll, bare.roll, 1e-6);
        EXPECT_NEAR((on.vel_b - bare.vel_b).norm(), 0.0, 1e-5);
        for (const std::size_t s : {GenHelFcs::kServoTheta0, GenHelFcs::kServoA1s,
                                    GenHelFcs::kServoB1s, GenHelFcs::kServoThettr}) {
            EXPECT_NEAR(on.component_states[off + s], bare.component_states[off + s], 1e-6) << s;
        }

        // The SAS and PBA steady output [%] at the trim, from GenHel's diagrams.
        const double phi = on.roll / kDeg;
        const double theta = on.pitch / kDeg;
        const double ysens = on.component_states[off + GenHelFcs::kAccelY] / kFt;  // [ft/s²]
        // VXBIKT: the body-x airspeed (equivalent, D-040; sea level, no wind).
        const double vx_kt = on.vel_b.x() / kKnot;
        const double fast = vx_kt > 60.0 ? 1.0 : 0.0;
        const double roll =
            std::clamp(-0.139 * std::clamp(phi, -7.0, 7.0), -5.0, 5.0) +
            std::clamp(2.413 * std::clamp(-0.703 * 0.0885 * phi, -7.0, 7.0), -5.5, 5.5);
        const double yaw = std::clamp(fast * -1.1 * ysens, -5.0, 5.0) +
                           std::clamp(fast * 0.5 * -28.11 * 0.0876 * ysens, -5.5, 5.5);
        const double pba = std::clamp(-0.375 * std::clamp(theta, -20.0, 20.0) +
                                          0.0525 * std::clamp(vx_kt - 80.0, 0.0, 100.0) - 4.9,
                                      -15.0, 15.0);
        // As inches ahead of each mixer (% → deg by KPCDG*, → in by 1/KX*), and
        // the pilot's stick takes them out.
        const double xa = roll * 0.16 / 1.6;
        const double xb = pba * -0.283 / -2.83;
        const double xp = yaw * -0.298 / -5.539;
        EXPECT_NEAR((on.controls.lat_cyclic - bare.controls.lat_cyclic) * half(rig.lat_cyclic), -xa,
                    1e-5);
        EXPECT_NEAR((on.controls.lon_cyclic - bare.controls.lon_cyclic) * half(rig.lon_cyclic) *
                        rig.sign_lon_stick,
                    -xb, 1e-5);
        EXPECT_NEAR((on.controls.pedal - bare.controls.pedal) * half(rig.pedal), -xp, 1e-5);
        EXPECT_NEAR(on.controls.collective, bare.controls.collective, 1e-6);
        std::printf(
            "%3d kt: phi %.3f deg, Y.SENS %.4f ft/s^2; SAS roll %.3f %%, yaw %.3f %%, "
            "PBA %.3f %%; stick lat %.4f, lon %.4f, pedal %.4f in\n",
            kt, phi, ysens, roll, yaw, pba, -xa, -xb * rig.sign_lon_stick, -xp);
    }
}

// M8c2: the SAS damps the Dutch roll, in nonlinear free flight. From level
// trims at 90 to 150 kt a 0.05 rad/s yaw-rate kick starts it; the yaw rate's
// change from the unkicked flight is measured as its largest swing over the
// first 3 s and over 6 to 9 s. With the SAS and PBA on the linear model's
// Dutch roll has ζ 0.33 to 0.44 and ω_n 1.0 to 1.3 rad/s (VALIDATION.md
// M8c2), so over the 6 s between the windows the swing falls by
// e^(−ζ ω_n 6 s), 9× or more; the gate takes 4× (measured 30 to 68×, the
// faster modes adding to the first window). With them off (ζ 0.15 at 90 kt,
// 0.02 at 120 kt, 0 at 150 kt) it fell 4.2×, 1.8× and 1.9×: the gate caught
// a missing SAS at 120 and 150 kt. Since D-048 (GenHel's downwash correction
// terms) the bare aircraft's Dutch roll is better damped at 90 and 120 kt
// (ζ 0.37 and 0.31; it falls 6.5× and 11.8×), so the check that the gate
// would catch a missing SAS is made at 150 kt only (ζ 0.06, 1.9×), with the
// same 4× threshold. Since D-053 the rotor turns on the engines' free shaft,
// no longer locked to the airframe in yaw, and the bare aircraft's swing at
// 150 kt falls 4.5× (2.2× with the shaft held ideal; the SAS on: 63× and
// 65×). The absolute 4× no longer tells a missing SAS apart there, so at
// 150 kt the SAS must also damp the swing four times as much as the bare
// aircraft does (measured 14×).
TEST(Trim, SasDampsTheDutchRoll) {
    const Executive ex = uh60a_executive();
    TrimResult prev;
    bool have = false;
    for (const int kt : {90, 120, 150}) {
        SCOPED_TRACE(kt);
        std::array<double, 2> ratio = {0.0, 0.0};
        for (const bool on : {false, true}) {
            TrimCondition c = level(kt);
            if (on) {
                c.afcs = {true, true, true};
            }
            std::vector<double> x;
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            ASSERT_TRUE(r.converged);
            if (!on) {
                prev = r;
                have = true;
            }
            const Environment env = trim_environment(c);
            const double dt = trim_period(ex) / 64;
            const int steps = static_cast<int>(9.0 / dt);
            const auto controls = [&](double) { return r.controls; };
            std::size_t missed = 0;
            const auto base = test::free_flight(ex, env, x, controls, dt, steps, &missed);
            std::vector<double> xk = x;
            xk[body_layout::kOmega + 2] += 0.05;
            const auto kicked = test::free_flight(ex, env, xk, controls, dt, steps, &missed);
            EXPECT_EQ(missed, 0U);
            const auto swing = [&](double t0, double t1) {
                double lo = 1e9;
                double hi = -1e9;
                for (int n = static_cast<int>(t0 / dt); n <= static_cast<int>(t1 / dt); ++n) {
                    const auto i = static_cast<std::size_t>(n);
                    const double d =
                        kicked[i][body_layout::kOmega + 2] - base[i][body_layout::kOmega + 2];
                    lo = std::min(lo, d);
                    hi = std::max(hi, d);
                }
                return hi - lo;
            };
            ratio[on ? 1 : 0] = swing(0.0, 3.0) / swing(6.0, 9.0);
        }
        std::printf("%d kt: yaw-rate swing falls %.2fx (SAS off), %.2fx (SAS on) over 6 s\n", kt,
                    ratio[0], ratio[1]);
        EXPECT_GT(ratio[1], 4.0);
        if (kt == 150) {
            EXPECT_GT(ratio[1], 4.0 * ratio[0]);  // the gate would catch the SAS missing
        }
    }
}

// ---- Steady kinds (D-045, M8c3 amendment) --------------------------------------------

// Linear damping on every body axis and the controls on X, Y, Z and N: a
// stateless aircraft that trims at any airspeed.
class DampedPlant : public DynamicComponent {
public:
    explicit DampedPlant(double weight) : weight_(weight) {}
    std::size_t num_states() const override { return 0; }
    void initial_states(std::span<double> /*x*/) const override {}
    void derivatives(const SimContext& ctx, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        const Vec3& v = ctx.body.vel_b;
        const Vec3& w = ctx.body.omega_b;
        const Controls& c = ctx.controls;
        out.load0_b.F =
            Vec3(-200.0 * v.x() + 2e4 * c.lon_cyclic, -300.0 * v.y() + 2e4 * c.lat_cyclic,
                 -2.0 * weight_ * c.collective - 500.0 * v.z());
        out.load0_b.M = Vec3(-3000.0 * w.x(), -5000.0 * w.y(), -4000.0 * w.z() + 1e4 * c.pedal);
    }

private:
    double weight_;
};

// One state of each kind: [0] free, a lag on the collective, which trim
// solves; [1] orbit, a lag on the body-x velocity; [2] held, an integrator
// whose input isn't zero in trim (as an FPS integrator's in a trim that
// isn't coordinated), which steady_states() starts at the velocity plus
// kOffset. `labels` breaks the labelling for the tests below.
enum class Labels { right, written_free, unwritten_held, drifting_orbit };

class ThreeKinds : public DynamicComponent {
public:
    static constexpr double kTau = 0.5;     // [s]
    static constexpr double kOffset = 0.7;  // [m/s]
    static constexpr double kDrift = 0.01;  // [m/s²], the held state's rate
    explicit ThreeKinds(Labels labels) : labels_(labels) {}
    std::size_t num_states() const override { return 3; }
    void initial_states(std::span<double> x) const override { std::fill(x.begin(), x.end(), 0.0); }
    std::string_view name() const noexcept override { return "three_kinds"; }
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override {
        out.xdot0[0] = (ctx.controls.collective - x[0]) / kTau;
        out.xdot0[1] = (ctx.body.vel_b.x() - x[1]) / kTau;
        out.xdot0[2] = kDrift;
    }
    void steady_states(const SimContext& ctx, std::span<double> x) const override {
        x[1] = ctx.body.vel_b.x();
        x[2] = ctx.body.vel_b.x() + kOffset;
    }
    void steady_kinds(std::span<SteadyKind> kinds) const override {
        kinds[0] = labels_ == Labels::unwritten_held ? SteadyKind::held : SteadyKind::free;
        kinds[1] = labels_ == Labels::written_free ? SteadyKind::free : SteadyKind::orbit;
        kinds[2] = labels_ == Labels::drifting_orbit ? SteadyKind::orbit : SteadyKind::held;
    }

private:
    Labels labels_;
};

Executive three_kinds_executive(Labels labels) {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(1000.0, 500.0, 800.0, 900.0, 100.0);
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<DampedPlant>(cfg.mass.mass * units::kStandardGravity));
    comps.push_back(std::make_unique<ThreeKinds>(labels));
    return Executive(cfg, std::move(comps));
}

// Trim honours each label: the free state is solved (the lag at the trimmed
// collective), the orbit state starts at its input, and the held state starts
// at the value steady_states() gave it, offset and all, although it drifts
// over the revolution. (x's body velocity adds the trim's vibration offset,
// D-035, about 1e-10 m/s here, to the mean velocity steady_states() saw.)
TEST(Trim, SteadyKindsAreHonoured) {
    const Executive ex = three_kinds_executive(Labels::right);
    for (const double kt : {0.0, 60.0}) {
        SCOPED_TRACE(kt);
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, level(kt), nullptr, x);
        ASSERT_TRUE(r.converged);
        const std::size_t off = ex.component_offset(1);
        const double u = unpack_body(x).vel_b.x();
        EXPECT_NEAR(x[off], r.controls.collective, 1e-9);
        EXPECT_NEAR(x[off + 1], u, 1e-9);
        EXPECT_NEAR(x[off + 2], u + ThreeKinds::kOffset, 1e-9);
        // The held state's drift is reported, not counted against
        // convergence (D-045 M8c3): its rate, to RK4's exactness on a
        // constant rate.
        EXPECT_NEAR(r.held_drift_rate, ThreeKinds::kDrift, 1e-12);
    }
}

// The held label is what lets that trim converge: labelled orbit, the same
// drifting state leaves a change over the revolution that no unknown can
// remove (D-045).
TEST(Trim, DriftingStateLabelledOrbitDoesNotTrim) {
    const Executive ex = three_kinds_executive(Labels::drifting_orbit);
    std::vector<double> x;
    EXPECT_FALSE(solve_trim(ex, level(60.0), nullptr, x).converged);
}

// The executive checks the labels against steady_states() when it is built,
// both ways: a state set but labelled free, and one labelled held but not set.
TEST(Trim, SteadyKindsMismatchIsRejected) {
    EXPECT_THROW((void)three_kinds_executive(Labels::written_free), InitError);
    EXPECT_THROW((void)three_kinds_executive(Labels::unwritten_held), InitError);
    EXPECT_NO_THROW((void)three_kinds_executive(Labels::right));
}

// ---- Model API -----------------------------------------------------------------------

// trim() resets the model to t = 0 at the trimmed state; advance() then keeps
// the real-time rules: no allocation and bit-identical repeat runs. With the
// SAS, PBA (M8c2) and FPS (M8c3) on, so their paths run too.
TEST(Trim, ModelStartsFromTrimAndKeepsTheRealTimeRules) {
    Model m = Model::from_file(test::uh60a_config_path());
    Controls c;
    for (int k = 0; k < 50; ++k) {
        (void)m.advance(c, Environment{}, 1'000'000);
    }
    TrimCondition tc = level(80);
    tc.afcs = {true, true, true, true, false, false};
    const TrimResult r = m.trim(tc);
    ASSERT_TRUE(r.converged);
    EXPECT_EQ(m.outputs().sim_time_ns, 0);
    EXPECT_NEAR(m.outputs().pitch, r.pitch, 1e-12);
    EXPECT_NEAR(m.outputs().roll, r.roll, 1e-12);
    // The component states are the trim's, except the flight control
    // system's, which start on their free-flight orbit (D-045 amendment;
    // FcsStartTransientIsSmall).
    const Executive ex = uh60a_executive();
    std::size_t fcs = 0;
    while (dynamic_cast<const GenHelFcs*>(&ex.component(fcs)) == nullptr) {
        ++fcs;
    }
    const std::size_t fcs_lo = ex.component_offset(fcs);
    const std::size_t fcs_hi = fcs_lo + ex.component_size(fcs);
    const std::vector<double> x0(m.state().begin(), m.state().end());
    for (std::size_t i = body_layout::kSize; i < x0.size(); ++i) {
        if (i < fcs_lo || i >= fcs_hi) {
            EXPECT_DOUBLE_EQ(x0[i], r.component_states[i - body_layout::kSize]);
        }
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

// ---- Trim at a given roll angle (D-035 M7c amendment) ------------------------------

// The roll-constrained trim inverts the sideslip-constrained one: asking for
// the roll that a β = 0 trim gives, from a start 3° away, must return β = 0
// and the same trim; and the returned trim is solve_trim's at the returned β.
TEST(TrimAtRoll, InvertsTheSideslipTrim) {
    const Executive ex = uh60a_executive();
    std::vector<double> x;
    const TrimResult ref = solve_trim(ex, level(80), nullptr, x);
    ASSERT_TRUE(ref.converged);
    TrimCondition c = level(80);
    c.sideslip = 3.0 * kDeg;
    const RollTrimResult r = solve_trim_at_roll(ex, c, ref.roll, &ref, x);
    ASSERT_TRUE(r.converged);
    EXPECT_NEAR(r.trim.roll, ref.roll, 1e-8);
    EXPECT_NEAR(r.sideslip, 0.0, 1e-6);
    EXPECT_NEAR(r.trim.pitch, ref.pitch, 1e-6);
    EXPECT_NEAR(r.trim.controls.pedal, ref.controls.pedal, 1e-6);
    EXPECT_NEAR(r.trim.controls.lat_cyclic, ref.controls.lat_cyclic, 1e-6);
    EXPECT_LE(r.iterations, 10);
}

// Zero roll, as GenHel trims above about 50 kt: φ = 0 to the tolerance with
// every inner residual converged, and solve_trim at the returned β agrees.
TEST(TrimAtRoll, ZeroRollIsReached) {
    const Executive ex = uh60a_executive();
    std::vector<double> x;
    const RollTrimResult r = solve_trim_at_roll(ex, level(100), 0.0, nullptr, x);
    ASSERT_TRUE(r.converged);
    EXPECT_LE(std::abs(r.trim.roll), 1e-8);
    EXPECT_LT(std::abs(r.sideslip), 20.0 * kDeg);
    TrimCondition c = level(100);
    c.sideslip = r.sideslip;
    std::vector<double> y;
    const TrimResult again = solve_trim(ex, c, &r.trim, y);
    ASSERT_TRUE(again.converged);
    EXPECT_NEAR(again.roll, 0.0, 1e-7);
}

// Where sideslip is undefined the roll-constrained trim refuses.
TEST(TrimAtRoll, RejectsUndefinedSideslip) {
    const Executive ex = uh60a_executive();
    std::vector<double> x;
    EXPECT_THROW((void)solve_trim_at_roll(ex, level(0), 0.0, nullptr, x), InitError);
    TrimCondition up = level(10);
    up.flight_path_angle = 0.5 * std::numbers::pi;
    EXPECT_THROW((void)solve_trim_at_roll(ex, up, 0.0, nullptr, x), InitError);
}

}  // namespace
}  // namespace eyas
