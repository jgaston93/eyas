// Uses Eyas the way a host does: only include/eyas is on the include path
// (D-020). That this file compiles is the main check; the tests confirm the
// API is usable on its own.
#include <cmath>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "eyas/model.hpp"

namespace {

constexpr const char* kYaml = R"(
geodetic_origin:
  latitude: "37.41 deg"
  longitude: "-122.05 deg"
  altitude: "10 m"
airframe:
  mass: "8000 kg"
  inertia:
    ixx: "6000 kg*m^2"
    iyy: "40000 kg*m^2"
    izz: "37000 kg*m^2"
    ixz: "1900 kg*m^2"
)";

TEST(PublicApi, BuildAdvanceSnapshotRestore) {
    eyas::Model m = eyas::Model::from_string(kYaml, "api.yaml");
    eyas::Snapshot snap = m.make_snapshot();
    const eyas::AdvanceStatus st = m.advance(eyas::Controls{}, eyas::Environment{}, 16'666'667);
    EXPECT_EQ(st.substeps, 16);
    EXPECT_GT(m.outputs().sim_time, 0.0);
    EXPECT_LT(m.outputs().altitude, 10.0);  // no rotor: it falls
    ASSERT_TRUE(m.restore(snap));
    EXPECT_EQ(m.outputs().sim_time_ns, 0);
    EXPECT_EQ(m.outputs().altitude, 10.0);
}

TEST(PublicApi, ModelIsMovable) {
    eyas::Model a = eyas::Model::from_string(kYaml);
    eyas::Model b = std::move(a);
    EXPECT_EQ(b.advance(eyas::Controls{}, eyas::Environment{}, 1'000'000).substeps, 1);
    a = std::move(b);
    EXPECT_EQ(a.outputs().sim_time_ns, 1'000'000);
}

// Output Tiers 2–4 (D-043) are reachable from the public headers alone. With
// no rotor or gear, Tier 4 is NaN; the falling airframe reads one g down
// and no specific force.
TEST(PublicApi, OutputTiers) {
    eyas::Model m = eyas::Model::from_string(kYaml);
    (void)m.advance(eyas::Controls{}, eyas::Environment{}, 100'000'000);
    const eyas::AircraftState& o = m.outputs();
    EXPECT_GT(o.vel_n.z(), 0.0);  // falling: NED z down
    EXPECT_NEAR(o.vertical_speed, -o.vel_n.z(), 1e-12);
    EXPECT_NEAR(o.accel_b.z(), 9.80665, 1e-9);
    EXPECT_NEAR(o.load_factor, 0.0, 1e-9);
    EXPECT_NEAR(o.true_airspeed, o.vel_b.norm(), 1e-12);
    EXPECT_GT(o.pressure_altitude, 0.0);
    EXPECT_GT(o.outside_air_temperature, 280.0);
    EXPECT_TRUE(std::isnan(o.main_rotor_thrust));
    EXPECT_TRUE(std::isnan(o.rotor_speed_percent));
    EXPECT_FALSE(o.weight_on_wheels);
}

TEST(PublicApi, InitErrorIsCatchable) {
    try {
        (void)eyas::Model::from_string("airframe: {}\n", "bad.yaml");
        FAIL() << "expected InitError";
    } catch (const eyas::InitError& e) {
        EXPECT_NE(std::string(e.what()).find("bad.yaml"), std::string::npos) << e.what();
    }
}

}  // namespace
