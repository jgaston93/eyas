#include "util/units.hpp"

#include <numbers>

#include <gtest/gtest.h>

namespace eyas::units {
namespace {

// Reference values: NIST SP 811 (2008), Appendix B.
TEST(Units, ExactDefinitions) {
    EXPECT_EQ(kFoot, 0.3048);
    EXPECT_EQ(kInch, 0.0254);
    EXPECT_EQ(kStandardGravity, 9.80665);
    EXPECT_EQ(kPoundMass, 0.45359237);
}

TEST(Units, DerivedFactorsMatchNist) {
    EXPECT_NEAR(kPoundForce, 4.4482216152605, 1e-13);
    EXPECT_NEAR(kSlug, 14.593902937206364, 1e-12);
    EXPECT_NEAR(kKnot, 0.51444444444444444, 1e-15);
    EXPECT_NEAR(kDegree, std::numbers::pi / 180.0, 0.0);
}

TEST(Units, RegistryScales) {
    EXPECT_EQ(si_scale("1"), 1.0);
    EXPECT_EQ(si_scale("m"), 1.0);
    EXPECT_EQ(si_scale("ft"), kFoot);
    EXPECT_EQ(si_scale("in"), kInch);
    EXPECT_EQ(si_scale("deg"), kDegree);
    EXPECT_EQ(si_scale("lbf"), kPoundForce);
    EXPECT_EQ(si_scale("lbm"), kPoundMass);
    EXPECT_EQ(si_scale("slug"), kSlug);
    EXPECT_EQ(si_scale("kt"), kKnot);
    EXPECT_EQ(si_scale("rpm"), kRpm);
    EXPECT_EQ(si_scale("1/deg"), 1.0 / kDegree);
}

// GenHel lists inertias in slug·ft² and lb·in·s² (NASA CR-166309, Table 4.2
// and p. 5.10-19); both must convert to the same SI values.
TEST(Units, InertiaUnitsAgree) {
    const double slug_ft2 = *si_scale("slug*ft^2");
    const double lbf_in_s2 = *si_scale("lbf*in*s^2");
    EXPECT_NEAR(slug_ft2, 1.3558179483314004, 1e-15);
    EXPECT_NEAR(lbf_in_s2, 0.1129848290276167, 1e-15);
    EXPECT_NEAR(12.0 * lbf_in_s2, slug_ft2, 1e-15);
}

// Every unit carries the dimension its name says; the pairs that only differ
// by dimension (Hz vs rad/s, rad vs 1, N*m vs kg*m) must not collapse.
TEST(Units, Dimensions) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("1"), Dimension::dimensionless);
    EXPECT_EQ(dim("ft"), Dimension::length);
    EXPECT_EQ(dim("s"), Dimension::time);
    EXPECT_EQ(dim("Hz"), Dimension::frequency);
    EXPECT_EQ(dim("rpm"), Dimension::angular_rate);
    EXPECT_EQ(dim("rad/s"), Dimension::angular_rate);
    EXPECT_EQ(dim("deg"), Dimension::angle);
    EXPECT_EQ(dim("1/deg"), Dimension::per_angle);
    EXPECT_EQ(dim("lbm"), Dimension::mass);
    EXPECT_EQ(dim("slug"), Dimension::mass);
    EXPECT_EQ(dim("lbf"), Dimension::force);
    EXPECT_EQ(dim("kt"), Dimension::speed);
    EXPECT_EQ(dim("ft/s^2"), Dimension::acceleration);
    EXPECT_EQ(dim("slug/ft^3"), Dimension::density);
    EXPECT_EQ(dim("ft*lbf"), Dimension::moment);
    EXPECT_EQ(dim("slug*ft"), Dimension::mass_moment);
    EXPECT_EQ(dim("lbf*in*s^2"), Dimension::inertia);
    EXPECT_EQ(dim("ft*lbf/rad"), Dimension::rotational_stiffness);
    EXPECT_EQ(dim("N*m/rad"), Dimension::rotational_stiffness);
    EXPECT_EQ(dim("ft*lbf*s/rad"), Dimension::rotational_damping);
    EXPECT_EQ(dim("N*m*s/rad"), Dimension::rotational_damping);
    EXPECT_EQ(dim("ft^2"), Dimension::area);
    EXPECT_EQ(dim("deg/lbf"), Dimension::angle_per_force);
    EXPECT_EQ(find_unit("ft^2")->si_scale, kFoot * kFoot);
    EXPECT_EQ(dim("ft^3"), Dimension::volume);
    EXPECT_EQ(dim("m^3"), Dimension::volume);
    EXPECT_EQ(find_unit("ft^3")->si_scale, kFoot * kFoot * kFoot);
    EXPECT_EQ(find_unit("deg/lbf")->si_scale, kDegree / kPoundForce);
    EXPECT_EQ(find_unit("ft*lbf/rad")->si_scale, find_unit("ft*lbf")->si_scale);
    EXPECT_EQ(find_unit("ft")->si_scale, kFoot);
    EXPECT_FALSE(find_unit("lb").has_value());
}

// AFCS units (GenHel §5.5, M8c2): the SAS and PBA work in percent of control
// authority, a fraction in SI (1 % = 0.01). A gain in % per deg/s is a
// fraction per angular rate, kept apart from time (deg per deg/s, a time),
// as per angle is from dimensionless.
TEST(Units, AfcsUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("%"), Dimension::dimensionless);
    EXPECT_EQ(*si_scale("%"), 0.01);
    EXPECT_EQ(dim("%/s"), Dimension::frequency);
    EXPECT_EQ(dim("%/deg"), Dimension::per_angle);
    EXPECT_EQ(*si_scale("%/deg"), 0.01 / kDegree);
    EXPECT_EQ(dim("%/(deg/s)"), Dimension::per_angular_rate);
    EXPECT_EQ(dim("1/(rad/s)"), Dimension::per_angular_rate);
    EXPECT_EQ(*si_scale("%/(deg/s)"), 0.01 / kDegree);
    EXPECT_EQ(dim("%/kt"), Dimension::per_speed);
    EXPECT_EQ(dim("s/m"), Dimension::per_speed);
    EXPECT_EQ(*si_scale("%/kt"), 0.01 / kKnot);
    EXPECT_EQ(dim("%/(ft/s^2)"), Dimension::per_acceleration);
    EXPECT_EQ(dim("s^2/m"), Dimension::per_acceleration);
    EXPECT_EQ(*si_scale("%/(ft/s^2)"), 0.01 / kFoot);
    EXPECT_EQ(dim("(deg/s)/(ft/s^2)"), Dimension::rate_per_acceleration);
    EXPECT_EQ(dim("(rad/s)/(m/s^2)"), Dimension::rate_per_acceleration);
    EXPECT_EQ(*si_scale("(deg/s)/(ft/s^2)"), kDegree / kFoot);
    EXPECT_EQ(dim("deg/%"), Dimension::angle);
    EXPECT_EQ(*si_scale("deg/%"), kDegree / 0.01);
    EXPECT_NE(Dimension::per_angular_rate, Dimension::time);
    EXPECT_NE(dimension_name(Dimension::per_angular_rate), "unknown");
    EXPECT_NE(dimension_name(Dimension::per_speed), "unknown");
    EXPECT_NE(dimension_name(Dimension::per_acceleration), "unknown");
    EXPECT_NE(dimension_name(Dimension::rate_per_acceleration), "unknown");
}

// Landing-gear units (GenHel §5.7.5): tire pressure, tire stiffness in lbf/in
// and lbf/ft, linear and quadratic damping. NIST: 1 psi = 6894.757293168 Pa.
TEST(Units, GearUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("psi"), Dimension::pressure);
    EXPECT_EQ(dim("Pa"), Dimension::pressure);
    EXPECT_NEAR(kPsi, 6894.757293168361, 1e-9);
    EXPECT_EQ(dim("lbf/in"), Dimension::stiffness);
    EXPECT_EQ(dim("lbf/ft"), Dimension::stiffness);
    EXPECT_EQ(dim("N/m"), Dimension::stiffness);
    EXPECT_NEAR(*si_scale("lbf/in"), 12.0 * *si_scale("lbf/ft"), 1e-10);
    EXPECT_EQ(dim("lbf*s/ft"), Dimension::damping);
    EXPECT_EQ(dim("N*s/m"), Dimension::damping);
    EXPECT_EQ(dim("lbf*s^2/ft^2"), Dimension::quadratic_damping);
    EXPECT_EQ(dim("N*s^2/m^2"), Dimension::quadratic_damping);
    EXPECT_NEAR(*si_scale("lbf*s^2/ft^2"), kSlug / kFoot, 1e-12);
}

// Flight-control units (GenHel §5.5, M8c): rigging gains in deg/in and a
// lateral-acceleration feedback gain in deg/(ft/s²).
TEST(Units, FlightControlUnits) {
    const auto dim = [](std::string_view u) { return find_unit(u)->dimension; };
    EXPECT_EQ(dim("deg/in"), Dimension::angle_per_length);
    EXPECT_EQ(dim("rad/m"), Dimension::angle_per_length);
    EXPECT_NEAR(*si_scale("deg/in"), kDegree / 0.0254, 1e-12);
    EXPECT_EQ(dim("deg/(ft/s^2)"), Dimension::angle_per_acceleration);
    EXPECT_EQ(dim("rad/(m/s^2)"), Dimension::angle_per_acceleration);
    EXPECT_NEAR(*si_scale("deg/(ft/s^2)"), kDegree / 0.3048, 1e-12);
    EXPECT_EQ(dim("s^2"), Dimension::time_squared);
}

TEST(Units, RejectsUnknownAndAmbiguous) {
    EXPECT_FALSE(si_scale("lb").has_value());
    EXPECT_FALSE(si_scale("furlong").has_value());
    EXPECT_FALSE(si_scale("").has_value());
    EXPECT_FALSE(si_scale("FT").has_value());
}

}  // namespace
}  // namespace eyas::units
