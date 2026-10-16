// M2 acceptance tests for the ISA atmosphere (spec §6.2, D-015): the
// U.S. Standard Atmosphere 1976 (USSA) tables, geometric-to-geopotential
// conversion, the hydrostatic temperature offset, and range flags.

#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "env/atmosphere.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

// One USSA Table I row (human-verified 2026-10-10; DATA_GAPS.md).
struct TableRow {
    double altitude;  // [m] the table's argument: Z (geometric) or H (geopotential)
    double other;     // [m] the other altitude column, printed to the metre
    double T;         // [K]  printed to 0.001
    double P_mb;      // [mb] printed to 5 significant figures (6 at sea level)
    double P_unit;    // [mb] one unit in the last printed digit of P
    double rho;       // [kg/m^3] printed to 5 significant figures
    double rho_unit;  // [kg/m^3] one unit in the last printed digit of rho
};

// Tolerances follow from how the table is printed. T and rho are rounded, so
// they must agree to half a unit in the last digit. P agrees with the USSA
// equations in all rows only if its last digit is truncated, not rounded
// (e.g. Z = 11000 m: Eq. 33a gives 226.9996 mb, printed 226.99), so P is
// allowed one full unit. The margin covers our own round-off only.
constexpr double kMargin = 1.0 + 1e-9;

void expect_row(const AirState& air, const TableRow& r) {
    EXPECT_NEAR(air.temperature, r.T, 0.5e-3 * kMargin);
    EXPECT_NEAR(air.pressure / 100.0, r.P_mb, r.P_unit * kMargin);
    EXPECT_NEAR(air.density, r.rho, 0.5 * r.rho_unit * kMargin);
}

// USSA Table I, geometric altitude: book pp. 53, 55, 59.
TEST(Atmosphere, MatchesUssaGeometricAltitudeTable) {
    const std::array<TableRow, 4> rows = {{
        {0.0, 0.0, 288.150, 1013.25, 0.01, 1.2250, 1e-4},
        {1000.0, 1000.0, 281.651, 898.76, 0.01, 1.1117, 1e-4},
        {5000.0, 4996.0, 255.676, 540.48, 0.01, 0.73643, 1e-5},
        {11000.0, 10981.0, 216.774, 226.99, 0.01, 0.36480, 1e-5},
    }};
    for (const TableRow& r : rows) {
        SCOPED_TRACE(r.altitude);
        const AirState air = isa_atmosphere(r.altitude, 0.0);
        EXPECT_EQ(air.altitude, r.altitude);
        EXPECT_NEAR(air.geopotential_altitude, r.other, 0.5);
        expect_row(air, r);
        EXPECT_FALSE(air.out_of_range);
    }
}

// USSA Table I, geopotential altitude: book pp. 52, 54, 58. The geometric
// altitude is recovered from H with the inverse of Eq. 18 (Eq. 19).
TEST(Atmosphere, MatchesUssaGeopotentialAltitudeTable) {
    const std::array<TableRow, 3> rows = {{
        {1000.0, 1000.0, 281.650, 898.74, 0.01, 1.1116, 1e-4},
        {5000.0, 5004.0, 255.650, 540.19, 0.01, 0.73612, 1e-5},
        {11000.0, 11019.0, 216.650, 226.32, 0.01, 0.36392, 1e-5},
    }};
    constexpr double r0 = isa::kEarthRadius;
    for (const TableRow& r : rows) {
        SCOPED_TRACE(r.altitude);
        const double Z = r0 * r.altitude / (r0 - r.altitude);  // USSA Eq. 19
        EXPECT_NEAR(Z, r.other, 0.5);
        const AirState air = isa_atmosphere(Z, 0.0);
        EXPECT_NEAR(air.geopotential_altitude, r.altitude, 1e-9);
        expect_row(air, r);
    }
}

// USSA Eq. 18 at a hand-computed point: Z = 11 000 m gives
// H = 6356766 × 11000 / 6367766 = 10980.9977... m'.
// ρ0 from the sea-level constants through the equation of state (USSA
// Eq. 42) is the standard's tabulated sea-level density, 1.2250 kg/m³.
TEST(Atmosphere, SeaLevelDensityMatchesUssa) {
    EXPECT_NEAR(isa::kSeaLevelDensity, 1.2250, 5e-5);
    EXPECT_NEAR(isa::kSeaLevelDensity, isa_atmosphere(0.0, 0.0).density, 1e-12);
}

TEST(Atmosphere, GeopotentialAltitudeMatchesEq18) {
    EXPECT_NEAR(geopotential_altitude(11000.0), 69'924'426'000.0 / 6'367'766.0, 1e-9);
    EXPECT_EQ(geopotential_altitude(0.0), 0.0);
}

// Speed of sound: Eq. 50 with the equation of state (Eq. 42) is a² = γP/ρ,
// an identity the implementation must satisfy at any altitude and offset.
TEST(Atmosphere, SpeedOfSoundConsistentWithEquationOfState) {
    for (const double h : {-1000.0, 0.0, 3000.0, 11000.0, 15000.0}) {
        for (const double dT : {-20.0, 0.0, 30.0}) {
            const AirState air = isa_atmosphere(h, dT);
            EXPECT_NEAR(air.speed_of_sound * air.speed_of_sound,
                        isa::kGamma * air.pressure / air.density,
                        1e-12 * air.speed_of_sound * air.speed_of_sound);
        }
    }
}

// D-015: ISA + ΔT shifts the temperature profile and keeps the sea-level
// pressure at P0.
TEST(Atmosphere, TemperatureOffsetShiftsProfileAndKeepsSeaLevelPressure) {
    for (const double dT : {-25.0, 15.0}) {
        const AirState sl = isa_atmosphere(0.0, dT);
        EXPECT_EQ(sl.pressure, isa::kSeaLevelPressure);
        EXPECT_DOUBLE_EQ(sl.temperature, isa::kSeaLevelTemperature + dT);
        for (const double h : {2000.0, 9000.0, 14000.0}) {
            EXPECT_NEAR(isa_atmosphere(h, dT).temperature, isa_atmosphere(h, 0.0).temperature + dT,
                        1e-12);
        }
    }
    // Hotter air is less dense at sea level, and at altitude the hydrostatic
    // column makes the pressure higher than standard.
    EXPECT_LT(isa_atmosphere(0.0, 20.0).density, isa_atmosphere(0.0, 0.0).density);
    EXPECT_GT(isa_atmosphere(3000.0, 20.0).pressure, isa_atmosphere(3000.0, 0.0).pressure);
}

// Independent check of the offset atmosphere: it must satisfy the hydrostatic
// equation dP/dZ = −ρ g(Z), with g(Z) = g0 (r0/(r0 + Z))² (USSA Eq. 17), which
// the implementation never evaluates (it integrates in geopotential altitude).
// Central differences with a 1 m step: truncation ~ h² P'''/6 ≈ 1e-9 relative.
TEST(Atmosphere, OffsetAtmosphereIsHydrostatic) {
    constexpr double r0 = isa::kEarthRadius;
    constexpr double step = 1.0;
    for (const double dT : {-30.0, 0.0, 25.0}) {
        for (const double Z : {0.0, 2500.0, 8000.0, 10900.0, 11100.0, 16000.0}) {
            SCOPED_TRACE(testing::Message() << "dT=" << dT << " Z=" << Z);
            const double dPdZ =
                (isa_atmosphere(Z + step, dT).pressure - isa_atmosphere(Z - step, dT).pressure) /
                (2.0 * step);
            const double g = units::kStandardGravity * (r0 / (r0 + Z)) * (r0 / (r0 + Z));
            const double expected = -isa_atmosphere(Z, dT).density * g;
            EXPECT_NEAR(dPdZ, expected, 1e-8 * std::abs(expected));
        }
    }
}

// T and P are continuous across the 11 km' tropopause, with and without offset.
TEST(Atmosphere, ContinuousAtTropopause) {
    const double Z11 = isa::kEarthRadius * 11000.0 / (isa::kEarthRadius - 11000.0);
    for (const double dT : {-10.0, 0.0, 10.0}) {
        const AirState below = isa_atmosphere(Z11 - 1e-6, dT);
        const AirState above = isa_atmosphere(Z11 + 1e-6, dT);
        EXPECT_NEAR(below.temperature, above.temperature, 1e-8);
        // |dP/dZ| ≈ 3.6 Pa/m across a 2 µm gap: ≈ 7e-6 Pa.
        EXPECT_NEAR(below.pressure, above.pressure, 2e-5);
    }
}

// D-015 range: layers 0 and 1 cover H = −5 to 20 km'. Outside that the profile
// is extended (finite values) and flagged.
TEST(Atmosphere, OutOfRangeIsFlaggedButFinite) {
    for (const double h : {-6000.0, 25000.0}) {
        const AirState air = isa_atmosphere(h, 0.0);
        EXPECT_TRUE(air.out_of_range);
        EXPECT_TRUE(std::isfinite(air.density));
        EXPECT_TRUE(std::isfinite(air.pressure));
        EXPECT_GT(air.density, 0.0);
    }
    EXPECT_FALSE(isa_atmosphere(19000.0, 0.0).out_of_range);
    EXPECT_FALSE(isa_atmosphere(-4000.0, 0.0).out_of_range);
}

// D-019: ΔT is valid if finite and the coldest point of the profile, the
// isothermal layer at 216.65 K (USSA Table 4), stays above 0 K.
TEST(Atmosphere, DeltaTValidity) {
    EXPECT_TRUE(isa_delta_T_valid(0.0));
    EXPECT_TRUE(isa_delta_T_valid(-216.0));
    EXPECT_TRUE(isa_delta_T_valid(1000.0));
    EXPECT_FALSE(isa_delta_T_valid(-216.65));  // T = 0 K in the stratosphere
    EXPECT_FALSE(isa_delta_T_valid(-300.0));
    EXPECT_FALSE(isa_delta_T_valid(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(isa_delta_T_valid(std::numeric_limits<double>::infinity()));
    // Near the bound, every altitude in and beyond range stays physical.
    for (const double dT : {-216.6, -216.64999}) {
        ASSERT_TRUE(isa_delta_T_valid(dT));
        for (const double h : {-6000.0, 0.0, 11000.0, 25000.0}) {
            const AirState air = isa_atmosphere(h, dT);
            EXPECT_GT(air.temperature, 0.0);
            EXPECT_TRUE(std::isfinite(air.density));
        }
    }
}

// The executive's range check and the AirState flag agree (D-019).
TEST(Atmosphere, RangeHelperMatchesFlag) {
    for (const double h : {-6000.0, -5000.0, 0.0, 19999.0, 20100.0, 25000.0}) {
        EXPECT_EQ(isa_out_of_range(h), isa_atmosphere(h, 0.0).out_of_range) << h;
    }
    EXPECT_TRUE(isa_out_of_range(std::numeric_limits<double>::quiet_NaN()));
}

// No exceptions in the loop (spec §7): a NaN altitude gives NaN, not a throw.
TEST(Atmosphere, NanAltitudePropagates) {
    const AirState air = isa_atmosphere(std::numeric_limits<double>::quiet_NaN(), 0.0);
    EXPECT_TRUE(std::isnan(air.density));
}

}  // namespace
}  // namespace eyas
