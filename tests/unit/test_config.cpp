#include "util/config.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using D = units::Dimension;

constexpr const char* kYaml = R"(
name: UH-60A
main_rotor:
  radius: "26.83 ft"
  omega: "27.0 rad/s"
  blades: 4
  tip_loss: 0.97
  blade_weight: !placeholder "256.9 lbf"
  counter_clockwise: true
rotors:
  - radius: "26.83 ft"
  - radius: "5.5 ft"
    hub_offset: !placeholder "1.0 m"
)";

Config load() {
    return Config::from_string(kYaml, "test.yaml");
}

TEST(Config, QuantitiesConvertToSi) {
    const Config c = load();
    EXPECT_EQ(c.quantity("main_rotor.radius", D::length), 26.83 * units::kFoot);
    EXPECT_EQ(c.quantity("main_rotor.omega", D::angular_rate), 27.0);
    EXPECT_EQ(c.quantity("main_rotor.blade_weight", D::force), 256.9 * units::kPoundForce);
}

TEST(Config, ScalarsOfOtherTypes) {
    const Config c = load();
    EXPECT_EQ(c.string("name"), "UH-60A");
    EXPECT_EQ(c.integer("main_rotor.blades"), 4);
    EXPECT_EQ(c.number("main_rotor.tip_loss"), 0.97);
    EXPECT_TRUE(c.boolean("main_rotor.counter_clockwise"));
}

TEST(Config, QuantityWithoutUnitIsAnError) {
    const Config c = load();
    EXPECT_THROW((void)c.quantity("main_rotor.tip_loss", D::dimensionless), InitError);
}

TEST(Config, NumberWithUnitIsAnError) {
    const Config c = load();
    EXPECT_THROW((void)c.number("main_rotor.radius"), InitError);
}

TEST(Config, UnknownUnitIsAnError) {
    const Config c = Config::from_string("w: \"3 lb\"\nv: \"3 furlong\"\n");
    EXPECT_THROW((void)c.quantity("w", D::force), InitError);
    EXPECT_THROW((void)c.quantity("v", D::length), InitError);
}

// A unit of the wrong kind is an error, with both dimensions named; the
// value is never converted.
TEST(Config, QuantityOfWrongDimensionIsAnError) {
    const Config c =
        Config::from_string("rate: \"1000 s\"\nlat: \"37 m\"\nw: \"27 Hz\"\n", "dims.yaml");
    EXPECT_THROW((void)c.quantity("lat", D::angle), InitError);
    // Hz is a frequency, not an angular rate: they differ by 2π.
    EXPECT_THROW((void)c.quantity("w", D::angular_rate), InitError);
    EXPECT_EQ(c.quantity("w", D::frequency), 27.0);
    try {
        (void)c.quantity("rate", D::frequency);
        FAIL() << "expected InitError";
    } catch (const InitError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("dims.yaml: rate:"), std::string::npos) << what;
        EXPECT_NE(what.find("dimension time; expected frequency"), std::string::npos) << what;
    }
}

// allow_keys() rejects keys the reader doesn't know, naming the full path.
TEST(Config, UnknownKeysAreErrors) {
    const Config c = load();
    EXPECT_NO_THROW(c.child("main_rotor")
                        .allow_keys({"radius", "omega", "blades", "tip_loss", "blade_weight",
                                     "counter_clockwise"}));
    try {
        c.child("main_rotor").allow_keys({"radius", "omega", "blades", "tip_loss"});
        FAIL() << "expected InitError";
    } catch (const InitError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("test.yaml: main_rotor.blade_weight: unknown key"), std::string::npos)
            << what;
    }
    EXPECT_THROW(c.child("rotors").allow_keys({"radius"}), InitError);  // not a map
}

// yaml-cpp accepts duplicate keys and returns the first, so a corrected value
// added further down a hand-edited file would be silently ignored.
TEST(Config, DuplicateKeysAreErrors) {
    EXPECT_THROW((void)Config::from_string("r: \"1 m\"\nr: \"2 m\"\n"), InitError);
    EXPECT_THROW((void)Config::from_string("a:\n  r: \"1 m\"\n  r: \"2 m\"\n"), InitError);
    EXPECT_THROW((void)Config::from_string("s:\n  - r: 1\n    r: 2\n"), InitError);
    EXPECT_NO_THROW((void)Config::from_string("a:\n  r: 1\nb:\n  r: 2\n"));
}

// Config keys are plain names. A list, map or null used as a key has no name,
// so it is rejected as such rather than misreported as a duplicate.
TEST(Config, NonScalarKeysAreErrors) {
    for (const char* yaml :
         {"? [a, b]\n: 1\n", "? {a: 1}\n: 1\n", "~: 1\n", "? [a, b]\n: 1\n? [c, d]\n: 2\n"}) {
        try {
            (void)Config::from_string(yaml);
            ADD_FAILURE() << "no error for: " << yaml;
        } catch (const InitError& e) {
            const std::string msg = e.what();
            EXPECT_NE(msg.find("key must be a plain name"), std::string::npos) << msg;
        }
    }
}

TEST(Config, NumberWithTwoSignsIsAnError) {
    const Config c = Config::from_string("n: +-5\nq: \"+-5 m\"\n");
    EXPECT_THROW((void)c.number("n"), InitError);
    EXPECT_THROW((void)c.quantity("q", D::length), InitError);
}

TEST(Config, TypeMismatchesAreErrors) {
    const Config c = load();
    EXPECT_THROW((void)c.integer("main_rotor.tip_loss"), InitError);
    EXPECT_THROW((void)c.boolean("main_rotor.blades"), InitError);
    EXPECT_THROW((void)c.number("main_rotor"), InitError);
}

TEST(Config, MissingKeyErrorNamesFileAndFullPath) {
    const Config c = load();
    try {
        (void)c.child("main_rotor").quantity("chord", D::length);
        FAIL() << "expected InitError";
    } catch (const InitError& e) {
        const std::string what = e.what();
        EXPECT_NE(what.find("test.yaml"), std::string::npos) << what;
        EXPECT_NE(what.find("main_rotor.chord"), std::string::npos) << what;
    }
}

// in_context passes a value through, and re-raises an InitError from inside
// (e.g. a constructor's check) naming the file, key path and original text.
TEST(Config, InContextNamesTheKeyOfAnInnerError) {
    const Config rotor = load().child("main_rotor");
    EXPECT_EQ(rotor.in_context("radius", [] { return 7; }), 7);
    try {
        rotor.in_context("radius", []() -> int { throw InitError("must be positive"); });
        FAIL() << "expected InitError";
    } catch (const InitError& e) {
        EXPECT_EQ(std::string(e.what()), "test.yaml: main_rotor.radius: must be positive");
    }
}

TEST(Config, LookupDoesNotModifyTheDocument) {
    const Config c = load();
    EXPECT_FALSE(c.contains("main_rotor.chord"));
    EXPECT_THROW((void)c.quantity("main_rotor.chord", D::length), InitError);
    EXPECT_FALSE(c.contains("main_rotor.chord"));
    EXPECT_TRUE(c.contains("main_rotor.radius"));
}

TEST(Config, ChildAndSequenceAccess) {
    const Config c = load();
    EXPECT_EQ(c.child("main_rotor").integer("blades"), 4);
    const Config rotors = c.child("rotors");
    ASSERT_EQ(rotors.size(), 2U);
    EXPECT_EQ(rotors[1].quantity("radius", D::length), 5.5 * units::kFoot);
    EXPECT_THROW((void)rotors[2], InitError);
    EXPECT_THROW((void)c.child("main_rotor").size(), InitError);
}

TEST(Config, ListsPlaceholdersWithFullPaths) {
    const Config c = load();
    const auto& p = c.placeholders();
    ASSERT_EQ(p.size(), 2U);
    EXPECT_EQ(p[0], "main_rotor.blade_weight");
    EXPECT_EQ(p[1], "rotors[1].hub_offset");
    EXPECT_EQ(c.child("rotors").placeholders().size(), 2U);
}

TEST(Config, NotedPlaceholdersAreSharedByTheWholeDocument) {
    const Config c = load();
    c.child("rotors")[0].note_placeholder("table", "t.csv (# status: placeholder)");
    const auto& p = c.placeholders();
    ASSERT_EQ(p.size(), 3U);
    EXPECT_EQ(p[2], "rotors[0].table: t.csv (# status: placeholder)");
    EXPECT_EQ(c.child("main_rotor").placeholders().size(), 3U);
}

TEST(Config, SyntaxErrorsAndMissingFilesAreInitErrors) {
    EXPECT_THROW(Config::from_string("a: [1, 2\n"), InitError);
    EXPECT_THROW(Config::from_file("/nonexistent/eyas.yaml"), InitError);
}

TEST(Config, LoadsFromFile) {
    const auto path = std::filesystem::path(testing::TempDir()) / "eyas_config_test.yaml";
    {
        std::ofstream out(path);
        out << kYaml;
    }
    EXPECT_EQ(Config::from_file(path).integer("main_rotor.blades"), 4);
    std::filesystem::remove(path);
}

}  // namespace
}  // namespace eyas
