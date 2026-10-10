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

TEST(Units, RejectsUnknownAndAmbiguous) {
    EXPECT_FALSE(si_scale("lb").has_value());
    EXPECT_FALSE(si_scale("furlong").has_value());
    EXPECT_FALSE(si_scale("").has_value());
    EXPECT_FALSE(si_scale("FT").has_value());
}

}  // namespace
}  // namespace eyas::units
