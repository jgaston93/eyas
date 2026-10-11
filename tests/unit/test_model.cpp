// M2 acceptance tests through the public API (spec §9 M2, D-016–D-018):
// equilibrium under the dummy rotor, the collective-step response, the Tier 1
// pose output, the atmosphere seen by models, snapshot/restore/reset, and the
// real-time rules for eyas::Model.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/attitude.hpp"
#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "env/atmosphere.hpp"
#include "env/geodesy.hpp"
#include "eyas/model.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/m2_fixtures.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using test::InertialWeightBalance;
constexpr double kG = units::kStandardGravity;
constexpr std::int64_t kFrameNs = 16'666'667;  // 60 Hz host frame

std::string test_yaml() {
    return test::m2_yaml();
}

Model make_model() {
    return Model::from_string(test_yaml(), "test_model");
}

Controls hover_controls() {
    Controls c;
    c.collective = 0.5;
    return c;
}

// ---- Spec M2: equilibrium ---------------------------------------------------------

// Spec M2, as written: at rest, level, dummy force exactly equal to weight.
// Gravity and thrust cancel exactly in floating point at level attitude
// (SPEC_REVIEW #17), so the drift is asserted to be exactly zero.
TEST(Model, LevelEquilibriumIsExactFor60s) {
    Model m = make_model();
    for (int i = 0; i < 3600; ++i) {
        const AdvanceStatus st = m.advance(hover_controls(), Environment{}, kFrameNs);
        ASSERT_FALSE(st.non_finite);
    }
    const BodyState b = m.body_state();
    EXPECT_EQ(b.pos_n, Vec3::Zero());
    EXPECT_EQ(b.vel_b, Vec3::Zero());
    EXPECT_EQ(b.omega_b, Vec3::Zero());
    EXPECT_GE(m.outputs().sim_time, 60.0);
}

// D-018: a non-level attitude with a steady velocity. Gravity and the balancing
// force now go through the attitude rotation separately, so a frame or sign
// error would leave a residual acceleration. Tolerances are round-off: the
// residual |a| ~ few·eps·g gives < 1e-11 m over 60 s; summing 60 000 position
// increments with |p| ≤ 700 m adds ≤ 60 000 × 1.1e-16 × 700 ≈ 5e-9 m.
TEST(Model, TiltedTranslatingEquilibriumHolds) {
    ExecutiveConfig cfg;
    cfg.mass = test::m2_mass_properties();
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::make_unique<InertialWeightBalance>());
    Executive ex(cfg, {}, std::move(forces));

    BodyState b0;
    b0.q_nb = Quat(Eigen::AngleAxisd(2.1, Vec3::UnitZ()) * Eigen::AngleAxisd(-0.3, Vec3::UnitY()) *
                   Eigen::AngleAxisd(0.45, Vec3::UnitX()));
    b0.vel_b = Vec3(9.0, -2.5, 1.2);
    ex.set_body_state(b0);
    const BodyState start = ex.body_state();
    for (int i = 0; i < 60'000; ++i) {
        ASSERT_FALSE(ex.advance(Controls{}, Environment{}, 1'000'000).non_finite);
    }
    const BodyState b = ex.body_state();
    const double t = ex.sim_time();
    const Vec3 expected_pos = start.q_nb * start.vel_b * t;
    EXPECT_LT((b.pos_n - expected_pos).norm(), 1e-8);
    EXPECT_LT((b.vel_b - start.vel_b).norm(), 1e-12);
    EXPECT_LT(b.omega_b.norm(), 1e-15);
    EXPECT_LT(b.q_nb.angularDistance(start.q_nb), 1e-13);
}

// ---- Spec M2: step in the dummy force ----------------------------------------------

// Collective 0.5 → 0.6 adds 0.1·T_max = 0.2·m·g upward: a = −0.2 g along D.
// RK4 integrates constant acceleration exactly, so the checks are round-off
// level: w(t) = a t, D(t) = ½ a t², and alt = h0 − D in the Tier 1 output.
TEST(Model, CollectiveStepGivesExpectedVerticalAcceleration) {
    Model m = make_model();
    Controls c = hover_controls();
    c.collective = 0.6;
    for (int i = 0; i < 120; ++i) {  // 2 s of 60 Hz frames
        ASSERT_FALSE(m.advance(c, Environment{}, kFrameNs).non_finite);
    }
    const double t = m.outputs().sim_time;
    EXPECT_EQ(m.outputs().sim_time_ns, 2'000'000'000);
    const double a = -0.2 * kG;
    const BodyState b = m.body_state();
    EXPECT_NEAR(b.vel_b.z(), a * t, 1e-12 * std::abs(a * t));
    EXPECT_NEAR(b.pos_n.z(), 0.5 * a * t * t, 1e-11 * std::abs(0.5 * a * t * t));
    EXPECT_EQ(b.vel_b.x(), 0.0);
    EXPECT_EQ(b.vel_b.y(), 0.0);
    EXPECT_NEAR(m.outputs().altitude, 10.0 - 0.5 * a * t * t, 1e-9);
}

// ---- Tier 1 pose ---------------------------------------------------------------------

TEST(Model, InitialOutputsAreTheOrigin) {
    const Model m = make_model();
    const AircraftState& s = m.outputs();
    EXPECT_EQ(s.sim_time_ns, 0);
    EXPECT_EQ(s.latitude, 37.41 * units::kDegree);
    EXPECT_DOUBLE_EQ(s.longitude, -122.05 * units::kDegree);
    EXPECT_EQ(s.altitude, 10.0);
    EXPECT_EQ(s.heading, 0.0);
}

// The pose output is the geodetic map (D-017) and the Euler extraction of the
// current state, after reset and after advance().
TEST(Model, Tier1PoseMatchesStateAfterResetAndAdvance) {
    Model m = make_model();
    BodyState b;
    b.pos_n = Vec3(1200.0, -3400.0, -250.0);
    b.vel_b = Vec3(30.0, 1.0, -0.5);
    b.q_nb = Quat(Eigen::AngleAxisd(-2.0, Vec3::UnitZ()) * Eigen::AngleAxisd(0.1, Vec3::UnitY()) *
                  Eigen::AngleAxisd(-0.2, Vec3::UnitX()));
    b.omega_b = Vec3(0.01, -0.02, 0.03);
    ASSERT_TRUE(m.reset(b));

    const FlatEarthMap map({37.41 * units::kDegree, -122.05 * units::kDegree, 10.0});
    for (int pass = 0; pass < 2; ++pass) {
        const BodyState now = m.body_state();
        const AircraftState& s = m.outputs();
        const GeodeticPosition g = map.to_geodetic(now.pos_n);
        const EulerAngles e = euler_angles(now.q_nb);
        EXPECT_EQ(s.latitude, g.latitude);
        EXPECT_EQ(s.longitude, g.longitude);
        EXPECT_EQ(s.altitude, g.altitude);
        EXPECT_EQ(s.roll, e.roll);
        EXPECT_EQ(s.pitch, e.pitch);
        EXPECT_EQ(s.heading, e.heading);
        EXPECT_EQ(s.q_nb.coeffs(), now.q_nb.coeffs());
        (void)m.advance(hover_controls(), Environment{}, kFrameNs);
    }
}

// ---- Atmosphere in SimContext (D-016) ------------------------------------------------

class AirSpy : public ForceModel {
public:
    Wrench compute(const SimContext& ctx) const override {
        seen = ctx.air;
        return {};
    }
    mutable AirState seen{};
};

TEST(Model, ModelsSeeAtmosphereAtCgAltitude) {
    ExecutiveConfig cfg;
    cfg.mass = test::m2_mass_properties();
    cfg.origin_altitude = 250.0;
    auto spy = std::make_unique<AirSpy>();
    const AirSpy* probe = spy.get();
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::move(spy));
    Executive ex(cfg, {}, std::move(forces));
    BodyState b;
    b.pos_n = Vec3(10.0, 20.0, -1500.0);
    ex.set_body_state(b);

    Environment env;
    env.isa_delta_T = 12.0;
    std::vector<double> xdot(ex.num_states());
    ex.derivatives(0.0, ex.state(), Controls{}, env, xdot);
    const AirState expected = isa_atmosphere(1750.0, 12.0);
    EXPECT_EQ(probe->seen.altitude, 1750.0);
    EXPECT_EQ(probe->seen.density, expected.density);
    EXPECT_EQ(probe->seen.temperature, expected.temperature);
    EXPECT_EQ(probe->seen.speed_of_sound, expected.speed_of_sound);
}

// ---- Snapshot, restore, reset ---------------------------------------------------------

TEST(Model, RestoreReplaysBitIdentically) {
    Model m = make_model();
    Controls c = hover_controls();
    c.collective = 0.55;
    for (int i = 0; i < 60; ++i) {
        (void)m.advance(c, Environment{}, 16'666'667 + (i % 3));
    }
    Snapshot snap = m.make_snapshot();
    ASSERT_TRUE(m.snapshot(snap));

    auto run = [&](std::vector<double>& trace) {
        for (int i = 0; i < 120; ++i) {
            (void)m.advance(c, Environment{}, kFrameNs);
            const std::span<const double> x = m.state();
            trace.insert(trace.end(), x.begin(), x.end());
            trace.push_back(m.outputs().altitude);
        }
    };
    std::vector<double> first;
    run(first);
    const std::int64_t remainder_after_first = m.remainder_ns();
    ASSERT_TRUE(m.restore(snap));
    std::vector<double> second;
    run(second);
    ASSERT_EQ(first.size(), second.size());
    EXPECT_EQ(std::memcmp(first.data(), second.data(), first.size() * sizeof(double)), 0);
    EXPECT_EQ(m.remainder_ns(), remainder_after_first);  // the remainder was restored too
}

TEST(Model, SnapshotAndRestoreDoNotAllocate) {
    Model m = make_model();
    Snapshot snap = m.make_snapshot();
    const eyas::test::AllocationCounter counter;
    EXPECT_TRUE(m.snapshot(snap));
    EXPECT_TRUE(m.restore(snap));
    EXPECT_EQ(counter.count(), 0U);
}

TEST(Model, RestoreRejectsMismatchedSnapshot) {
    Model m = make_model();
    (void)m.advance(hover_controls(), Environment{}, kFrameNs);
    const std::int64_t steps = m.outputs().sim_time_ns;
    Snapshot empty;
    EXPECT_FALSE(m.restore(empty));
    EXPECT_FALSE(m.snapshot(empty));  // never resized (no allocation)
    EXPECT_EQ(m.outputs().sim_time_ns, steps);
}

// reset() starts the run again from the given state at t = 0.
TEST(Model, ResetSetsStateAndZeroesTime) {
    Model m = make_model();
    for (int i = 0; i < 10; ++i) {
        (void)m.advance(hover_controls(), Environment{}, 16'666'667);
    }
    BodyState b;
    b.pos_n = Vec3(0.0, 0.0, -100.0);
    ASSERT_TRUE(m.reset(b));
    EXPECT_EQ(m.outputs().sim_time_ns, 0);
    EXPECT_EQ(m.remainder_ns(), 0);
    EXPECT_EQ(m.outputs().altitude, 110.0);

    BodyState bad;
    bad.q_nb = Quat(0.0, 0.0, 0.0, 0.0);
    EXPECT_FALSE(m.reset(bad));
    EXPECT_EQ(m.outputs().altitude, 110.0);  // unchanged
}

// ---- Real-time rules -------------------------------------------------------------------

// Spec §7.1: advance(), including the outputs update, never allocates.
TEST(Model, AdvanceDoesNotAllocate) {
    Model m = make_model();
    (void)m.advance(hover_controls(), Environment{}, kFrameNs);
    const eyas::test::AllocationCounter counter;
    for (int i = 0; i < 120; ++i) {
        (void)m.advance(hover_controls(), Environment{}, kFrameNs);
    }
    (void)m.outputs();
    EXPECT_EQ(counter.count(), 0U);
}

// Spec §7.4: same inputs, same build → bit-identical outputs.
TEST(Model, RunsAreBitIdentical) {
    Model a = make_model();
    Model b = make_model();
    Controls c = hover_controls();
    Environment env;
    env.isa_delta_T = 7.0;
    for (int i = 0; i < 300; ++i) {
        c.collective = 0.5 + 0.1 * std::sin(0.05 * i);
        (void)a.advance(c, env, kFrameNs + i % 5);
        (void)b.advance(c, env, kFrameNs + i % 5);
    }
    ASSERT_EQ(a.state().size(), b.state().size());
    EXPECT_EQ(std::memcmp(a.state().data(), b.state().data(), a.state().size() * sizeof(double)),
              0);
    EXPECT_EQ(a.outputs().latitude, b.outputs().latitude);
}

// The status of the last advance() is part of the output record (Tier 5).
TEST(Model, OutputsCarryAdvanceStatus) {
    Model m = make_model();
    (void)m.advance(hover_controls(), Environment{}, 5'000'000'000);  // 5 s hitch
    EXPECT_TRUE(m.outputs().last_advance.capped);
    EXPECT_EQ(m.outputs().last_advance.substeps, 100);
}

// ---- Configuration -----------------------------------------------------------------------

TEST(Model, ConfigErrorsAreInitErrors) {
    const std::string good = test_yaml();
    auto replaced = [&](const std::string& from, const std::string& to) {
        std::string s = good;
        s.replace(s.find(from), from.size(), to);
        return s;
    };
    EXPECT_NO_THROW((void)Model::from_string(good));
    EXPECT_THROW((void)Model::from_string(replaced("1000 Hz", "7 Hz")), InitError);
    EXPECT_THROW((void)Model::from_string(replaced("1000 Hz", "1000.5 Hz")), InitError);
    EXPECT_THROW((void)Model::from_string(replaced("37.41 deg", "89.5 deg")), InitError);
    EXPECT_THROW((void)Model::from_string(replaced("ideal_thrust", "warp_drive")), InitError);
    EXPECT_THROW((void)Model::from_string(replaced("mass: \"8000 kg\"", "mass: \"-1 kg\"")),
                 InitError);
    EXPECT_THROW((void)Model::from_string(replaced("airframe:", "airframes:")), InitError);
}

// The simulation section is optional: 1000 Hz and 100 sub-steps by default.
TEST(Model, SimulationSectionDefaults) {
    std::string s = test_yaml();
    const std::string sim = "simulation:\n  physics_rate: \"1000 Hz\"\n  max_substeps: 100\n";
    s.erase(s.find(sim), sim.size());
    Model m = Model::from_string(s);
    const AdvanceStatus st = m.advance(hover_controls(), Environment{}, 3'500'000);
    EXPECT_EQ(st.substeps, 3);
}

}  // namespace
}  // namespace eyas
