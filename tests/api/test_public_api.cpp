// Uses Eyas the way a host does: only include/eyas is on the include path
// (D-020). That this file compiles is the main check; the tests confirm the
// API is usable on its own.
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

TEST(PublicApi, InitErrorIsCatchable) {
    try {
        (void)eyas::Model::from_string("airframe: {}\n", "bad.yaml");
        FAIL() << "expected InitError";
    } catch (const eyas::InitError& e) {
        EXPECT_NE(std::string(e.what()).find("bad.yaml"), std::string::npos) << e.what();
    }
}

}  // namespace
