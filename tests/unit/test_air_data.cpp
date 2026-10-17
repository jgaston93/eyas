// M8a air data (D-043): pressure altitude, impact pressure, calibrated and
// equivalent airspeed, against Gracey, NASA RP-1046 (1980), and the U.S.
// Standard Atmosphere 1976 (USSA).

#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "env/air_data.hpp"
#include "env/atmosphere.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

constexpr double kKnot = units::kKnot;
constexpr double kFoot = units::kFoot;

// Geometric altitude [m] of geopotential altitude H [m'] (USSA Eq. 19).
double geometric(double H) {
    return isa::kEarthRadius * H / (isa::kEarthRadius - H);
}

// Pressure altitude inverts the standard pressure in both layers.
TEST(AirData, PressureAltitudeInvertsTheStandardAtmosphere) {
    for (const double Z :
         {-2000.0, 0.0, 1000.0, 5000.0, 10999.0, 11000.0, 11100.0, 15000.0, 20000.0}) {
        SCOPED_TRACE(Z);
        const AirState air = isa_atmosphere(Z, 0.0);
        EXPECT_NEAR(pressure_altitude(air.pressure), air.geopotential_altitude, 1e-6);
    }
    EXPECT_EQ(pressure_altitude(isa::kSeaLevelPressure), 0.0);
}

// USSA Table I (geopotential argument; book pp. 52, 54, 58, as in
// test_atmosphere.cpp): the printed pressure gives back the printed altitude
// within the altitude change of one unit in the pressure's last digit,
// 0.01 mb / (ρ g).
TEST(AirData, PressureAltitudeMatchesUssaTable) {
    struct Row {
        double H, P_mb, rho;
    };
    const std::array<Row, 3> rows = {{
        {1000.0, 898.74, 1.1116},
        {5000.0, 540.19, 0.73612},
        {11000.0, 226.32, 0.36392},
    }};
    for (const Row& r : rows) {
        SCOPED_TRACE(r.H);
        const double tol = 1.0 / (r.rho * units::kStandardGravity);  // 1 Pa as height
        EXPECT_NEAR(pressure_altitude(r.P_mb * 100.0), r.H, tol);
    }
}

// On a hot or cold day the sea-level pressure is unchanged (D-015), so the
// pressure altitude at MSL is zero; aloft it differs from the geopotential
// altitude, by less than the geopotential altitude on a hot day (pressure
// falls more slowly in warm air).
TEST(AirData, PressureAltitudeOnANonStandardDay) {
    EXPECT_NEAR(pressure_altitude(isa_atmosphere(0.0, 20.0).pressure), 0.0, 1e-9);
    const AirState hot = isa_atmosphere(3000.0, 20.0);
    const AirState cold = isa_atmosphere(3000.0, -20.0);
    EXPECT_LT(pressure_altitude(hot.pressure), hot.geopotential_altitude);
    EXPECT_GT(pressure_altitude(cold.pressure), cold.geopotential_altitude);
}

TEST(AirData, PressureAltitudeOfInvalidPressureIsNan) {
    EXPECT_TRUE(std::isnan(pressure_altitude(0.0)));
    EXPECT_TRUE(std::isnan(pressure_altitude(-1.0)));
    EXPECT_TRUE(std::isnan(pressure_altitude(std::nan(""))));
}

// At standard sea level, V_c = V by definition (Gracey p. 15), and V_e = V.
TEST(AirData, CalibratedEqualsTrueAirspeedAtStandardSeaLevel) {
    const AirState sl = isa_atmosphere(0.0, 0.0);
    for (const double v : {0.0, 10.0, 50.0, 100.0, 200.0, 300.0}) {
        SCOPED_TRACE(v);
        const double qc = impact_pressure(sl.pressure, sl.density, v);
        EXPECT_NEAR(calibrated_airspeed(qc), v, 1e-9 * (1.0 + v));
        EXPECT_NEAR(equivalent_airspeed(sl.density, v), v, 1e-12 * (1.0 + v));
    }
}

// Low speed: q_c → ½ρV² (Gracey Eq. 3.10), the compressibility correction
// being (1 + M²/4 + …).
TEST(AirData, ImpactPressureTendsToDynamicPressure) {
    const AirState air = isa_atmosphere(2000.0, 0.0);
    const double v = 5.0;
    const double q = 0.5 * air.density * v * v;
    const double M = v / air.speed_of_sound;
    EXPECT_NEAR(impact_pressure(air.pressure, air.density, v), q * (1.0 + M * M / 4.0), 1e-6 * q);
}

TEST(AirData, NegativeImpactPressureGivesZero) {
    EXPECT_EQ(calibrated_airspeed(-1.0), 0.0);
    EXPECT_EQ(calibrated_airspeed(0.0), 0.0);
}

// Gracey Table A13 (book p. 246, PDF p. 260): true airspeed V [kt] for
// calibrated airspeed V_c [kt] at pressure altitude H [geopotential ft], on
// a standard day. Built on the 1962 standard atmosphere, which is the 1976
// one below 51 km. Read 2026-10-17 from the page image; every entry used is
// printed to 0.1 kt and sharp. We run the table backwards: from (H, V) to
// V_c through Eyas's atmosphere, Eq. 3.14 and Eq. 3.16 inverted. The table's
// V is rounded to 0.05 kt, and dV_c/dV ≤ 1, so V_c must agree to 0.05 kt.
// One entry is left out (NaN): V_c = 100 kt at 20 000 ft is printed 137.2,
// above V_c/√σ = 137.0, which no subsonic V can be since V_c > V_e there; the
// equations give 136.5. A misprint (DATA_GAPS.md, M8a).
TEST(AirData, MatchesGraceyTableA13) {
    const std::array<double, 7> H_ft = {0.0, 5000.0, 10000.0, 15000.0, 20000.0, 30000.0, 40000.0};
    const std::array<double, 3> vc_kt = {100.0, 200.0, 300.0};
    const double skip = std::nan("");
    const std::array<std::array<double, 7>, 3> v_kt = {{
        {100.0, 107.7, 116.2, 125.8, skip, 162.4, 199.1},
        {200.0, 215.0, 231.6, 250.0, 270.5, 318.9, 385.6},
        {300.0, 321.6, 345.4, 371.5, 400.1, 465.9, 553.7},
    }};
    for (std::size_t j = 0; j < vc_kt.size(); ++j) {
        for (std::size_t i = 0; i < H_ft.size(); ++i) {
            if (std::isnan(v_kt[j][i])) {
                continue;
            }
            SCOPED_TRACE(testing::Message() << "H = " << H_ft[i] << " ft, Vc = " << vc_kt[j]);
            const AirState air = isa_atmosphere(geometric(H_ft[i] * kFoot), 0.0);
            EXPECT_NEAR(pressure_altitude(air.pressure) / kFoot, H_ft[i], 1e-6);
            const double qc = impact_pressure(air.pressure, air.density, v_kt[j][i] * kKnot);
            EXPECT_NEAR(calibrated_airspeed(qc) / kKnot, vc_kt[j], 0.05 * (1.0 + 1e-9));
        }
    }
}

// Eq. 3.20 at a hand-computed point: ρ = ρ0/4 halves the speed.
TEST(AirData, EquivalentAirspeedScalesWithRootDensity) {
    EXPECT_NEAR(equivalent_airspeed(isa::kSeaLevelDensity / 4.0, 80.0), 40.0, 1e-12);
}

}  // namespace
}  // namespace eyas
