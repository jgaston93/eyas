// M3 inputs to the rotor: airfoil tables (D-024, D-021 column dimensions),
// the linear control map (spec §6.3), config file paths, and the rotor built
// from YAML through the public Model.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "eyas/model.hpp"
#include "rotor/airfoil.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/controls.hpp"
#include "support/alloc_counter.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
const std::string kData = EYAS_TEST_DATA_DIR;

// ---- Airfoil tables ----------------------------------------------------------------

constexpr const char* kSmallAirfoil = R"(# source: test
alpha[deg],mach[1],cl[1],cd[1]
-180,0,0,0.02
0,0,0,0.01
10,0,1.0,0.012
180,0,0,0.02
-180,0.5,0,0.03
0,0.5,0.1,0.02
10,0.5,1.2,0.022
180,0.5,0,0.03
)";

TEST(Airfoil, InterpolatesBothCoefficients) {
    const Airfoil af = Airfoil::from_csv(parse_csv_table(kSmallAirfoil, "small"));
    TableCursor<2> cur;
    const AirfoilCoefficients a = af(10.0 * kDeg, 0.0, cur);
    EXPECT_DOUBLE_EQ(a.cl, 1.0);
    EXPECT_DOUBLE_EQ(a.cd, 0.012);
    const AirfoilCoefficients b = af(5.0 * kDeg, 0.25, cur);
    EXPECT_NEAR(b.cl, 0.25 * (0.0 + 1.0 + 0.1 + 1.2), 1e-15);
    EXPECT_NEAR(b.cd, 0.25 * (0.01 + 0.012 + 0.02 + 0.022), 1e-15);
    // Mach above the table clamps (D-012); ±π are table edges.
    EXPECT_DOUBLE_EQ(af(10.0 * kDeg, 0.9, cur).cl, 1.2);
    EXPECT_DOUBLE_EQ(af(std::numbers::pi, 0.0, cur).cd, 0.02);
    EXPECT_FALSE(af.is_placeholder());
    EXPECT_EQ(af.source(), "test");
}

TEST(Airfoil, RejectsBadTables) {
    // α in a length unit (D-021 column dimensions).
    EXPECT_THROW(Airfoil::from_csv(parse_csv_table(
                     "# source: t\nalpha[m],mach[1],cl[1],cd[1]\n-180,0,0,0\n180,0,0,0\n"
                     "-180,1,0,0\n180,1,0,0\n",
                     "t")),
                 InitError);
    // α not covering ±180° (reverse flow would be clamped).
    EXPECT_THROW(Airfoil::from_csv(parse_csv_table(
                     "# source: t\nalpha[deg],mach[1],cl[1],cd[1]\n-30,0,0,0\n30,0,0,0\n"
                     "-30,1,0,0\n30,1,0,0\n",
                     "t")),
                 InitError);
    // Missing C_d column.
    EXPECT_THROW(
        Airfoil::from_csv(parse_csv_table(
            "# source: t\nalpha[deg],mach[1],cl[1]\n-180,0,0\n180,0,0\n-180,1,0\n180,1,0\n", "t")),
        InitError);
}

TEST(CsvTable, RequireDimension) {
    const CsvTable t = parse_csv_table("# source: t\nx[ft],y[deg]\n1,2\n", "t");
    EXPECT_NO_THROW(t.require_dimension("x", units::Dimension::length));
    EXPECT_NO_THROW(t.require_dimension("y", units::Dimension::angle));
    EXPECT_THROW(t.require_dimension("x", units::Dimension::angle), InitError);
    EXPECT_THROW(t.require_dimension("z", units::Dimension::length), InitError);
}

// ---- Control map -------------------------------------------------------------------

TEST(LinearControlMap, OffsetsGainsAndMixing) {
    const LinearControlMap map({1.0, 2.0, 0.0, 0.0, 0.5}, {0.0, 0.1, -3.0, 0.0, 0.0},
                               {0.2, 0.0, 0.0, 4.0, 0.0});
    Controls c;
    c.collective = 0.5;
    c.lon_cyclic = 0.25;
    c.lat_cyclic = -0.5;
    c.pedal = 1.0;
    const SwashplatePitch p = map.pitch(c);
    EXPECT_DOUBLE_EQ(p.theta0, 1.0 + 2.0 * 0.5 + 0.5 * 1.0);
    EXPECT_DOUBLE_EQ(p.theta1s, 0.1 * 0.5 - 3.0 * 0.25);
    EXPECT_DOUBLE_EQ(p.theta1c, 0.2 + 4.0 * -0.5);
}

TEST(LinearControlMap, FromConfig) {
    const Config cfg = Config::from_string(R"(
theta0: {offset: "2 deg", collective: "16 deg"}
theta1s: {lon_cyclic: "-10 deg", collective: "1 deg"}
)");
    const auto map = LinearControlMap::from_config(cfg);
    Controls c;
    c.collective = 1.0;
    c.lon_cyclic = 1.0;
    const SwashplatePitch p = map->pitch(c);
    EXPECT_NEAR(p.theta0, 18.0 * kDeg, 1e-15);
    EXPECT_NEAR(p.theta1s, -9.0 * kDeg, 1e-15);
    EXPECT_EQ(p.theta1c, 0.0);  // row absent: zero
    EXPECT_THROW(LinearControlMap::from_config(Config::from_string("theta2: {offset: \"1 deg\"}")),
                 InitError);
    EXPECT_THROW(
        LinearControlMap::from_config(Config::from_string("theta0: {colective: \"1 deg\"}")),
        InitError);
    EXPECT_THROW(LinearControlMap::from_config(Config::from_string("theta0: {offset: \"1 m\"}")),
                 InitError);
}

// ---- Config file paths -------------------------------------------------------------

TEST(Config, PathIsRelativeToConfigFile) {
    const auto dir = std::filesystem::temp_directory_path() / "eyas_config_path_test";
    std::filesystem::create_directories(dir);
    const auto file = dir / "c.yaml";
    { std::ofstream(file) << "a: tables/x.csv\nb: /abs/y.csv\n"; }
    const Config c = Config::from_file(file);
    EXPECT_EQ(c.path("a"), dir / "tables/x.csv");
    EXPECT_EQ(c.path("b"), std::filesystem::path("/abs/y.csv"));
    std::filesystem::remove_all(dir);
    const Config s = Config::from_string("a: x.csv\n");
    EXPECT_EQ(s.path("a"), std::filesystem::path("x.csv"));
}

// ---- The rotor through the public Model --------------------------------------------

std::string rotor_yaml(const std::string& airfoil,
                       const std::string& extra_blade = "      twist: \"-16 deg\"\n") {
    return R"(
geodetic_origin: {latitude: "37 deg", longitude: "-122 deg", altitude: "0 m"}
airframe:
  mass: "7000 kg"
  inertia: {ixx: "6000 kg*m^2", iyy: "40000 kg*m^2", izz: "37000 kg*m^2", ixz: "1900 kg*m^2"}
components:
  - type: blade_element_rotor
    blades: 4
    radius: "8 m"
    rotor_speed: "27 rad/s"
    direction: counter_clockwise
    inflow: uniform
    vortex_ring: none
    ground_effect: none
    inflow_tip_loss: none
    hub:
      position: {x: "0 m", y: "0 m", z: "-2 m"}
      pitch: "-3 deg"
    blade:
      root_cutout: "1.2 m"
      tip_loss: 0.97
      chord: "0.53 m"
      airfoil: )" +
           airfoil + "\n" + extra_blade + R"(      elements: 20
    controls:
      theta0: {offset: "14 deg", collective: "16 deg"}
      theta1s: {lon_cyclic: "-8 deg"}
)";
}

TEST(Model, BladeElementRotorFromConfig) {
    const std::string twist = "      twist_table: " + kData + "/twist_table.csv\n";
    Model m = Model::from_string(rotor_yaml(kData + "/linear_airfoil.csv", twist), "rotor.yaml");
    Controls c;
    c.collective = 0.6;
    // A short run: the rotor lifts, and advance() doesn't allocate.
    test::AllocationCounter count;
    for (int i = 0; i < 30; ++i) {
        const AdvanceStatus st = m.advance(c, Environment{}, 16'666'667);
        ASSERT_FALSE(st.non_finite);
    }
    EXPECT_EQ(count.count(), 0U);
    EXPECT_LT(m.body_state().vel_b.z(), 0.0);  // climbing: thrust exceeds 7000 kg weight
}

// D-051: inflow_tip_loss is required, none or effective_disc.
TEST(Model, InflowTipLossFromConfig) {
    const std::string base = rotor_yaml(kData + "/linear_airfoil.csv");
    const auto with = [&](const std::string& line) {
        std::string y = base;
        y.replace(y.find("    inflow_tip_loss: none\n"), 26, line);
        return Config::from_string(y, "rotor.yaml");
    };
    const auto read = [&](const std::string& line) {
        return BladeElementRotor::from_config(with(line).child("components")[0]);
    };
    EXPECT_EQ(read("    inflow_tip_loss: none\n")->config().inflow_tip_loss, InflowTipLoss::none);
    EXPECT_EQ(read("    inflow_tip_loss: effective_disc\n")->config().inflow_tip_loss,
              InflowTipLoss::effective_disc);
    EXPECT_THROW(read("    inflow_tip_loss: prandtl\n"), InitError);
    EXPECT_THROW(read(""), InitError);
}

// D-025: placeholder YAML values and placeholder CSV tables are all reported
// through the public Model, with the key path that loaded them.
TEST(Model, ReportsPlaceholderValuesAndTables) {
    const std::string sourced_twist = "      twist_table: " + kData + "/twist_table.csv\n";
    const Model sourced =
        Model::from_string(rotor_yaml(kData + "/linear_airfoil.csv", sourced_twist), "rotor.yaml");
    EXPECT_TRUE(sourced.placeholders().empty());

    // Copies of the test tables, marked as placeholders.
    const auto dir = std::filesystem::temp_directory_path() / "eyas_placeholder_test";
    std::filesystem::create_directories(dir);
    const auto mark = [&](const std::string& name) {
        std::ifstream in(kData + "/" + name);
        std::ostringstream text;
        text << "# status: placeholder\n" << in.rdbuf();
        const auto out = dir / name;
        std::ofstream(out) << text.str();
        return out.string();
    };
    const std::string twist = mark("twist_table.csv");
    const std::string airfoil = mark("linear_airfoil.csv");
    std::string yaml = rotor_yaml(airfoil, "      twist_table: " + twist + "\n");
    yaml.replace(yaml.find("chord: \"0.53 m\""), 15, "chord: !placeholder \"0.53 m\"");
    const Model m = Model::from_string(yaml, "rotor.yaml");
    std::filesystem::remove_all(dir);

    const auto& p = m.placeholders();
    ASSERT_EQ(p.size(), 3U);
    EXPECT_EQ(p[0], "components[0].blade.chord");
    EXPECT_EQ(p[1], "components[0].blade.twist_table: " + twist + " (# status: placeholder)");
    EXPECT_EQ(p[2], "components[0].blade.airfoil: " + airfoil + " (# status: placeholder)");
}

// D-026: the flap section, in GenHel's units (W_b given as lbm, M_b slug·ft,
// I_b slug·ft²), through the public Model: the rotor gets 2 states per
// blade, and advance() still doesn't allocate.
constexpr const char* kFlap = R"(      flap:
        hinge_offset: "1.25 ft"
        mass: "256.9 lbm"
        first_moment: "86.7 slug*ft"
        inertia: "1512.6 slug*ft^2"
        spring: "0 ft*lbf/rad"
        damper: "0 ft*lbf*s/rad"
        delta3: "0 deg"
)";

TEST(Model, FlappingRotorFromConfig) {
    const std::string blade = std::string("      twist: \"-16 deg\"\n") + kFlap;
    const Config cfg =
        Config::from_string(rotor_yaml(kData + "/linear_airfoil.csv", blade), "rotor.yaml");
    const auto rotor = BladeElementRotor::from_config(cfg.child("components")[0]);
    ASSERT_TRUE(rotor->config().flap.has_value());
    const FlapHinge& f = *rotor->config().flap;
    EXPECT_NEAR(f.offset, 1.25 * units::kFoot, 1e-15);
    EXPECT_NEAR(f.mass, 256.9 * units::kPoundMass, 1e-12);
    EXPECT_NEAR(f.first_moment, 86.7 * units::kSlug * units::kFoot, 1e-12);
    EXPECT_NEAR(f.inertia, 1512.6 * units::kSlug * units::kFoot * units::kFoot, 1e-9);
    EXPECT_EQ(rotor->num_states(), 8U);

    Model m = Model::from_string(rotor_yaml(kData + "/linear_airfoil.csv", blade), "rotor.yaml");
    Controls c;
    c.collective = 0.6;
    test::AllocationCounter count;
    for (int i = 0; i < 30; ++i) {
        const AdvanceStatus st = m.advance(c, Environment{}, 16'666'667);
        ASSERT_FALSE(st.non_finite);
    }
    EXPECT_EQ(count.count(), 0U);
}

// D-027: Pitt–Peters inflow adds three states after the flap states, and
// advance() still doesn't allocate.
TEST(Model, PittPetersRotorFromConfig) {
    std::string yaml = rotor_yaml(kData + "/linear_airfoil.csv",
                                  std::string("      twist: \"-16 deg\"\n") + kFlap);
    yaml.replace(yaml.find("inflow: uniform"), 15, "inflow: pitt_peters");
    const Config cfg = Config::from_string(yaml, "rotor.yaml");
    const auto rotor = BladeElementRotor::from_config(cfg.child("components")[0]);
    EXPECT_EQ(rotor->config().inflow, InflowModel::pitt_peters);
    EXPECT_EQ(rotor->num_states(), 11U);
    EXPECT_EQ(rotor->inflow_offset(), 8U);

    Model m = Model::from_string(yaml, "rotor.yaml");
    Controls c;
    c.collective = 0.6;
    test::AllocationCounter count;
    for (int i = 0; i < 30; ++i) {
        const AdvanceStatus st = m.advance(c, Environment{}, 16'666'667);
        ASSERT_FALSE(st.non_finite);
    }
    EXPECT_EQ(count.count(), 0U);
}

TEST(Model, BladeElementRotorConfigErrors) {
    const std::string af = kData + "/linear_airfoil.csv";
    const auto fails_with = [](const std::string& yaml, const std::string& text) {
        try {
            (void)Model::from_string(yaml, "rotor.yaml");
            ADD_FAILURE() << "expected InitError mentioning '" << text << "'";
        } catch (const InitError& e) {
            EXPECT_NE(std::string(e.what()).find(text), std::string::npos) << e.what();
        }
    };
    fails_with(rotor_yaml(kData + "/missing.csv"), "components[0].blade.airfoil");
    fails_with(rotor_yaml(af, "      twist: \"-16 deg\"\n      twist_table: x.csv\n"),
               "components[0].blade.twist");
    fails_with(rotor_yaml(af, "      twist: \"-16 deg\"\n      spacing: cosine\n"),
               "components[0].blade.spacing");
    std::string yaml = rotor_yaml(af);
    yaml.replace(yaml.find("counter_clockwise"), 17, "anticlockwise");
    fails_with(yaml, "components[0].direction");
    // The inflow model is required (D-027).
    yaml = rotor_yaml(af);
    yaml.replace(yaml.find("    inflow: uniform\n"), 20, "");
    fails_with(yaml, "components[0].inflow");
    yaml = rotor_yaml(af);
    yaml.replace(yaml.find("inflow: uniform"), 15, "inflow: peters_he");
    fails_with(yaml, "components[0].inflow");
    yaml = rotor_yaml(af);
    yaml.replace(yaml.find("blade_element_rotor"), 19, "teetering_rotor");
    fails_with(yaml, "components[0].type");
    yaml = rotor_yaml(af);
    yaml.replace(yaml.find("tip_loss: 0.97"), 14, "tip_loss: 1.5");
    fails_with(yaml, "components[0]");
    // Flap section: wrong dimensions, unknown keys, and an invalid hinge.
    const std::string tw = "      twist: \"-16 deg\"\n";
    std::string flap = kFlap;
    flap.replace(flap.find("0 ft*lbf/rad"), 12, "0 ft*lbf");
    fails_with(rotor_yaml(af, tw + flap), "components[0].blade.flap.spring");
    flap = kFlap;
    flap.replace(flap.find("86.7 slug*ft"), 12, "86.7 slug");
    fails_with(rotor_yaml(af, tw + flap), "components[0].blade.flap.first_moment");
    flap = kFlap;
    flap.replace(flap.find("delta3"), 6, "delta_3");
    fails_with(rotor_yaml(af, tw + flap), "components[0].blade.flap");
    flap = kFlap;
    flap.replace(flap.find("1512.6"), 6, "100.0");  // I_β < S_β²/m_b
    fails_with(rotor_yaml(af, tw + flap), "inertia must satisfy");
}

}  // namespace
}  // namespace eyas
