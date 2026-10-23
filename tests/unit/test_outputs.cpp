// M8a output Tiers 2–4 (D-043) on the UH-60A through Model: each field
// against a hand computation from Model::state(), written here from the
// textbook relations, and against the physics of the trimmed condition.

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "env/atmosphere.hpp"
#include "eyas/model.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "support/alloc_counter.hpp"
#include "support/bit_identical.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using test::Uh60aHarness;
constexpr double kKnot = units::kKnot;
constexpr double kG0 = units::kStandardGravity;
constexpr double kPi = std::numbers::pi;

TrimCondition condition(double kt, double gamma_deg = 0.0) {
    TrimCondition c;
    c.airspeed = kt * kKnot;
    c.flight_path_angle = gamma_deg * units::kDegree;
    c.altitude = 1000.0;
    return c;
}

// Standard-day geopotential altitude [m'] with pressure P, by bisection on
// the (separately verified) standard atmosphere: independent of the closed
// form in pressure_altitude().
double pressure_altitude_by_bisection(double P) {
    double lo = -2000.0;
    double hi = 20000.0;
    for (int i = 0; i < 200; ++i) {
        const double mid = 0.5 * (lo + hi);
        (isa_atmosphere(mid, 0.0).pressure > P ? lo : hi) = mid;
    }
    return geopotential_altitude(0.5 * (lo + hi));
}

// Gracey, NASA RP-1046: Eq. 3.14 and Eq. 3.16 solved for V_c, written out.
double calibrated_by_hand(double P, double rho, double V) {
    const double g = 1.4;
    const double P0 = 101325.0;
    const double rho0 = isa::kSeaLevelDensity;
    const double qc =
        P * (std::pow(1.0 + (g - 1.0) / (2.0 * g) * rho / P * V * V, g / (g - 1.0)) - 1.0);
    return std::sqrt(2.0 * g / (g - 1.0) * P0 / rho0 *
                     (std::pow(qc / P0 + 1.0, (g - 1.0) / g) - 1.0));
}

// Tiers 2 and 3 at a 100 kt level trim at 1000 m, on an ISA + 15 K day, in
// a 10 m/s wind from the west (blowing east). The outputs use the trim's
// environment until the first advance() (D-043).
TEST(Outputs, InertialAndAirDataAtAWindyHotDayTrim) {
    Model m = Model::from_file(test::uh60a_config_path());
    TrimCondition c = condition(100.0);
    c.isa_delta_T = 15.0;
    c.wind_n = Vec3(0.0, 10.0, 0.0);
    const TrimResult r = m.trim(c);
    ASSERT_TRUE(r.converged);
    const AircraftState& o = m.outputs();
    const BodyState b = m.body_state();
    const Mat3 C_nb = b.q_nb.toRotationMatrix();

    // Tier 2
    EXPECT_EQ(o.pos_n, b.pos_n);
    EXPECT_EQ(o.vel_b, b.vel_b);
    EXPECT_EQ(o.omega_b, b.omega_b);
    EXPECT_LT((o.vel_n - C_nb * b.vel_b).norm(), 1e-12);
    const Vec3 ground = C_nb * b.vel_b;
    EXPECT_NEAR(o.ground_speed, std::hypot(ground.x(), ground.y()), 1e-12);
    double track = std::atan2(ground.y(), ground.x());
    track += track < 0.0 ? 2.0 * kPi : 0.0;
    EXPECT_NEAR(o.track, track, 1e-12);
    // Heading north into a crosswind from the west: tracking right of north.
    EXPECT_GT(o.track, 0.0);
    EXPECT_LT(o.track, 0.5 * kPi);
    EXPECT_NEAR(o.vertical_speed, -ground.z(), 1e-12);

    // Tier 3
    const AirState a = isa_atmosphere(o.altitude, 15.0);  // at the CG
    const Vec3 v_air = b.vel_b - C_nb.transpose() * c.wind_n;
    const double V = v_air.norm();
    EXPECT_NEAR(o.true_airspeed, V, 1e-12);
    EXPECT_NEAR(o.true_airspeed, 100.0 * kKnot, 0.05);  // the trim's mean, plus 4/rev vibration
    EXPECT_NEAR(o.angle_of_attack, std::atan2(v_air.z(), v_air.x()), 1e-12);
    EXPECT_NEAR(o.sideslip, std::asin(v_air.y() / V), 1e-12);
    EXPECT_NEAR(o.sideslip, 0.0, 2e-3);  // trimmed for zero sideslip
    EXPECT_NEAR(o.equivalent_airspeed, V * std::sqrt(a.density / isa::kSeaLevelDensity), 1e-9);
    EXPECT_NEAR(o.calibrated_airspeed, calibrated_by_hand(a.pressure, a.density, V), 1e-9);
    EXPECT_EQ(o.indicated_airspeed, o.calibrated_airspeed);
    // Low Mach: V_c just above V_e (compressibility), both below V aloft.
    EXPECT_GT(o.calibrated_airspeed, o.equivalent_airspeed);
    EXPECT_LT(o.calibrated_airspeed - o.equivalent_airspeed, 0.01 * o.equivalent_airspeed);
    EXPECT_LT(o.calibrated_airspeed, V);
    EXPECT_NEAR(o.outside_air_temperature, a.temperature, 1e-12);
    EXPECT_NEAR(o.outside_air_temperature, 288.15 - 6.5e-3 * 1000.0 + 15.0, 0.01);
    EXPECT_NEAR(o.pressure_altitude, pressure_altitude_by_bisection(a.pressure), 1e-6);
    // Hot day: pressure falls more slowly, so the pressure altitude is below
    // the geopotential altitude, by roughly ΔT/T ≈ 5 % of it.
    EXPECT_LT(o.pressure_altitude, geopotential_altitude(1000.0));
    EXPECT_GT(o.pressure_altitude, 0.9 * geopotential_altitude(1000.0));
}

// Climbing at 5° and 60 kt: the vertical speed is V sin γ (no wind).
TEST(Outputs, VerticalSpeedInAClimb) {
    Model m = Model::from_file(test::uh60a_config_path());
    const TrimResult r = m.trim(condition(60.0, 5.0));
    ASSERT_TRUE(r.converged);
    const AircraftState& o = m.outputs();
    EXPECT_NEAR(o.vertical_speed, 60.0 * kKnot * std::sin(5.0 * units::kDegree), 0.02);
    EXPECT_NEAR(o.ground_speed, 60.0 * kKnot * std::cos(5.0 * units::kDegree), 0.05);
    EXPECT_NEAR(std::remainder(o.track, 2.0 * kPi), 0.0, 2e-3);  // north, either side of 0
}

// accel_b against a central difference of the flown velocity, the inertial
// acceleration being v̇_b + ω × v_b; specific force and load factor follow.
// Over one revolution of level flight the mean load factor is 1.
TEST(Outputs, AccelerationAndLoadFactorInFlight) {
    Model m = Model::from_file(test::uh60a_config_path());
    const TrimResult r = m.trim(condition(80.0));
    ASSERT_TRUE(r.converged);
    const Uh60aHarness ac;
    const auto& rotor = ac.get<BladeElementRotor>("main_rotor");
    const int rev_steps =
        static_cast<int>(std::lround(2.0 * kPi / rotor.config().rotor_speed * 1e3));

    std::vector<Vec3> v;
    std::vector<AircraftState> out;
    double nz_sum = 0.0;
    for (int k = 0; k <= rev_steps + 1; ++k) {
        v.push_back(m.body_state().vel_b);
        out.push_back(m.outputs());
        if (k >= 1 && k <= rev_steps) {
            nz_sum += m.outputs().load_factor;
        }
        (void)m.advance(r.controls, Environment{0.0, Vec3::Zero()}, 1'000'000);
    }
    const double h = 1e-3;
    double worst = 0.0;
    for (int k = 1; k <= rev_steps; ++k) {
        const AircraftState& o = out[static_cast<std::size_t>(k)];
        const Vec3 vdot =
            (v[static_cast<std::size_t>(k) + 1] - v[static_cast<std::size_t>(k) - 1]) / (2.0 * h);
        const Vec3 accel = vdot + o.omega_b.cross(o.vel_b);
        worst = std::max(worst, (accel - o.accel_b).norm());
        const Vec3 g_b = o.q_nb.conjugate() * Vec3(0.0, 0.0, kG0);
        EXPECT_LT((o.specific_force_b - (o.accel_b - g_b)).norm(), 1e-12);
        EXPECT_NEAR(o.load_factor, -o.specific_force_b.z() / kG0, 1e-15);
    }
    // Central difference error h²/6 |v⃛|: the 4/rev (17 Hz) vibration of
    // amplitude ~0.5 m/s² gives ~(2π 17)² 1e-6 / 6 × 0.5 ≈ 1e-3 m/s².
    EXPECT_LT(worst, 5e-3);
    EXPECT_NEAR(nz_sum / rev_steps, 1.0, 2e-3);
}

// Tier 4 in a hover trim: the rotor fields, and the yaw balance between the
// main-rotor torque and the tail-rotor thrust about its arm.
TEST(Outputs, RotorFieldsInHover) {
    Model m = Model::from_file(test::uh60a_config_path());
    const TrimResult r = m.trim(condition(0.0));
    ASSERT_TRUE(r.converged);
    const Uh60aHarness ac;
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto& tr = ac.get<BaileyRotor>("tail_rotor");
    const double omega = mr.config().rotor_speed;
    for (int k = 0; k < 3; ++k) {
        (void)m.advance(r.controls, Environment{}, 16'666'667);
    }
    const AircraftState& o = m.outputs();
    // The rotor on the engines' free shaft (D-053), governed: 100 % to its
    // 4/rev ripple, and the azimuth Ω t to within that over 0.05 s.
    EXPECT_NEAR(o.rotor_speed_percent, 100.0, 1e-4);
    const double speed = omega * o.rotor_speed_percent / 100.0;
    EXPECT_NEAR(o.main_rotor_power, o.main_rotor_torque * speed, 1e-6 * o.main_rotor_power);
    EXPECT_NEAR(o.main_rotor_azimuth, std::fmod(omega * o.sim_time, 2.0 * kPi), 1e-6);
    // Both rotors turn on the one shaft: before its azimuth first wraps (at
    // 2π, 0.23 s), the tail rotor's is the main rotor's times their gear.
    ASSERT_LT(omega * o.sim_time, 2.0 * kPi);
    EXPECT_NEAR(o.tail_rotor_azimuth,
                std::fmod(tr.config().rotor_speed / omega * o.main_rotor_azimuth, 2.0 * kPi), 1e-9);
    // The rotors carry the weight, 16 638 lbm (GenHel, p. 5.1-49), and the
    // fuselage download: the canted tail rotor (hub rolled 70°) lifts with
    // T_TR cos 70°, so the main rotor alone is just below the weight.
    const double weight = test::kGenHelWeight * units::kPoundMass * kG0;
    const double lift = o.main_rotor_thrust + o.tail_rotor_thrust * std::cos(tr.config().hub_roll);
    EXPECT_GT(lift, weight);
    EXPECT_LT(lift, 1.1 * weight);
    // Coning of a few degrees; little disc tilt in hover.
    EXPECT_GT(o.coning, 1.0 * units::kDegree);
    EXPECT_LT(o.coning, 6.0 * units::kDegree);
    EXPECT_LT(std::abs(o.flap_1c), 2.0 * units::kDegree);
    EXPECT_LT(std::abs(o.flap_1s), 2.0 * units::kDegree);
    // Tail-rotor thrust about the CG balances the main-rotor torque to within
    // the airframe's and the rotor hub's own yaw moments.
    const double arm = -tr.config().hub_position_b.x();
    EXPECT_NEAR(o.tail_rotor_thrust * arm, o.main_rotor_torque, 0.15 * o.main_rotor_torque);
    EXPECT_FALSE(o.weight_on_wheels);
}

// On the ground, brakes set, at low collective: weight on wheels, which is
// false before the first contact; the load factor then settles to 1.
TEST(Outputs, WeightOnWheels) {
    Model m = Model::from_file(test::uh60a_config_path());
    Controls c;
    c.collective = 0.2;
    c.pedal = 0.5;
    c.brake_left = 1.0;
    c.brake_right = 1.0;
    Environment env;
    env.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);  // first contact is at about 0.27 s
    ASSERT_TRUE(m.reset(s));
    (void)m.advance(c, env, 200'000'000);
    EXPECT_FALSE(m.outputs().weight_on_wheels);
    for (int k = 0; k < 60 * 6; ++k) {
        (void)m.advance(c, env, 16'666'667);
    }
    EXPECT_TRUE(m.outputs().weight_on_wheels);
    EXPECT_NEAR(m.outputs().load_factor, 1.0, 0.02);
    EXPECT_NEAR(m.outputs().true_airspeed, 0.0, 0.05);
}

// A config without rotors or gear: Tier 4 is NaN and weight on wheels false;
// Tiers 2 and 3 still follow the body (here, at rest at the origin).
TEST(Outputs, AbsentComponentsGiveNan) {
    const Model m = Model::from_string(R"(
geodetic_origin: {latitude: "0 deg", longitude: "0 deg", altitude: "500 m"}
airframe:
  mass: "1000 kg"
  inertia: {ixx: "1000 kg*m^2", iyy: "1000 kg*m^2", izz: "1000 kg*m^2", ixz: "0 kg*m^2"}
)");
    const AircraftState& o = m.outputs();
    EXPECT_TRUE(std::isnan(o.rotor_speed_percent));
    EXPECT_TRUE(std::isnan(o.main_rotor_thrust));
    EXPECT_TRUE(std::isnan(o.coning));
    EXPECT_TRUE(std::isnan(o.tail_rotor_thrust));
    EXPECT_TRUE(std::isnan(o.tail_rotor_azimuth));
    EXPECT_FALSE(o.weight_on_wheels);
    EXPECT_TRUE(std::isnan(o.track));  // at rest
    // Free fall from rest: a = g, so no specific force and zero load factor.
    EXPECT_LT((o.accel_b - Vec3(0.0, 0.0, kG0)).norm(), 1e-12);
    EXPECT_LT(o.specific_force_b.norm(), 1e-12);
    EXPECT_EQ(o.true_airspeed, 0.0);
    EXPECT_EQ(o.sideslip, 0.0);
    EXPECT_NEAR(o.pressure_altitude, geopotential_altitude(500.0), 1e-6);
}

// restore() brings back the controls and environment of the snapshot, so the
// outputs right after it are those at the snapshot, bit for bit, even after
// flying in another environment (D-043); restoring allocates nothing.
TEST(Outputs, RestoreBringsBackTheOutputs) {
    Model m = Model::from_file(test::uh60a_config_path());
    Controls c;
    c.collective = 0.2;
    c.pedal = 0.5;
    c.brake_left = 1.0;
    c.brake_right = 1.0;
    Environment ground;
    ground.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);
    ASSERT_TRUE(m.reset(s));
    for (int f = 0; f < 120; ++f) {
        (void)m.advance(c, ground, 16'666'667);
    }
    ASSERT_TRUE(m.outputs().weight_on_wheels);
    Snapshot snap = m.make_snapshot();
    ASSERT_TRUE(m.snapshot(snap));
    const std::vector<double> before = test::physical_outputs(m.outputs());

    Environment air;  // no ground, a wind, a hot day
    air.wind_n = Vec3(5.0, -3.0, 0.0);
    air.isa_delta_T = 20.0;
    (void)m.advance(Controls{}, air, 16'666'667);
    ASSERT_FALSE(m.outputs().weight_on_wheels);
    {
        const test::AllocationCounter count;
        ASSERT_TRUE(m.restore(snap));
        EXPECT_EQ(count.count(), 0U);
    }
    EXPECT_TRUE(m.outputs().weight_on_wheels);
    EXPECT_TRUE(test::bit_identical(test::physical_outputs(m.outputs()), before));
}

}  // namespace
}  // namespace eyas
