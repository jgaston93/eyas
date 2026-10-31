// V1 gate (D-035 M7c amendment): the UH-60A trimmed at the conditions of
// Ballin, NASA TM-88360 (1987), App. B, against the Sikorsky GenHel curves of
// Figs. B1-B4, digitized and human-reviewed (data/uh60a/validation/; DATA_GAPS
// "Ballin App. B trim plots"). The gate is against Sikorsky's GenHel, the model
// CR-166309 documents and Eyas's data come from (D-035).
//
// Tolerance, from the measured read-off accuracy (DATA_GAPS): 1 % of the axis
// span for the tick calibration plus 0.75 % for centring on the drawn line
// (half its width), i.e. 1.75 % of the span, plus the curve's local slope
// times the x read-off, 1.5 kt (B1) or 0.1 thousand ft/min (B2-B4), twice the
// measured scatter. A change from level flight is two readings: the span term
// grows by √2.
//
// Every difference beyond that must be explained (D-035) or it is open:
// VALIDATION.md M7c lists the open ones, and this test tracks each at its
// recorded value within the same tolerance, so it fails if an open difference
// grows (a new defect) or shrinks (a fix: update the record and VALIDATION.md).
// No tolerance is widened to make a quantity pass.
//
// The longitudinal stick's mean offset is not tested against GenHel: neutral
// longitudinal cyclic was chosen from GenHel's three Fig. 6.3.1 estimates by
// the fit to this same curve (D-035 M7c amendment), so B1(a) gates the shape
// against speed only, its mean removed. The mean is tracked at its recorded
// value, a calibration, not a validation. Since D-048 the shape is gated to
// 90 kt; above, the cyclic is an open difference (b1_open).

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "support/m7c_ballin.hpp"

namespace eyas {
namespace {

using namespace test::ballin;

constexpr double kSpanTerm = 0.0175;  // tick calibration 1 % + half the line width 0.75 %
constexpr double kKtReadOff = 1.5;    // [kt] B1 x read-off (twice the measured scatter)
constexpr double kRateReadOff = 0.1;  // [1000 ft/min] B2-B4 x read-off

struct Quantity {
    const char* panel;  // Fig. B1 panel
    const char* name;
    double span;  // axis span [unit]
    double (*eyas)(const Point&, const Averages&);
};

const std::vector<Quantity>& b1_quantities() {
    static const std::vector<Quantity> q = {
        {"a", "longitudinal cyclic [%]", 100.0,
         [](const Point& p, const Averages&) { return lon_pct(p.trim.controls); }},
        {"b", "lateral cyclic [%]", 100.0,
         [](const Point& p, const Averages&) { return lat_pct(p.trim.controls); }},
        {"c", "collective [%]", 100.0,
         [](const Point& p, const Averages&) { return collective_pct(p.trim.controls); }},
        {"d", "pitch attitude [deg]", 20.0,
         [](const Point& p, const Averages&) { return p.trim.pitch / kDeg; }},
        {"f", "sideslip [deg]", 15.0,
         [](const Point& p, const Averages&) { return p.sideslip / kDeg; }},
        {"g", "pedal [%]", 100.0,
         [](const Point& p, const Averages&) { return pedal_pct(p.trim.controls); }},
        {"h", "stabilator [deg]", 50.0,
         [](const Point&, const Averages& a) { return a.stabilator / kDeg; }},
        {"i", "main-rotor power [hp]", 2500.0,
         [](const Point&, const Averages& a) { return a.power / kHp; }},
    };
    return q;
}

// Open differences, Eyas − Sikorsky GenHel at each speed (VALIDATION.md M7c,
// recorded 2026-10-16 with D-038 to D-040; re-recorded 2026-10-20 with D-048,
// GenHel's downwash correction terms, and 2026-10-22 with D-051, tip loss on
// the inflow). Any (panel, speed) not listed here is
// gated. D-048 closed the pitch-attitude gap at cruise (90-150 kt) and the
// pedal's sideslip-driven part (50-130 kt), which are gated again; it opened
// the longitudinal cyclic at 100-150 kt and the pitch attitude at 50 kt
// (VALIDATION.md, "Kaplita steady-heading sideslips").
const std::map<std::string, std::map<int, double>>& b1_open() {
    static const std::map<std::string, std::map<int, double>> open = {
        // longitudinal cyclic at cruise, 2.1-3.5 % aft (D-048: about 1.1 % from
        // the inflow model's sideslip and the D-036 parabola, about 1.9 % open,
        // with the collective and lateral cyclic below); not in the shape gate
        {"a", {{100, 2.73}, {110, 3.51}, {120, 2.78}, {130, 2.11}, {140, 3.15}, {150, 2.40}}},
        // lateral cyclic at high speed (not linked to the sideslip, D-048)
        {"b", {{120, 0.84}, {130, 2.18}, {140, 3.80}, {150, 6.13}}},
        // collective: shape against speed
        {"c",
         {{10, 2.42},
          {20, 2.66},
          {30, 2.62},
          {40, 3.12},
          {50, 3.29},
          {60, 2.88},
          {70, 3.19},
          {80, 2.47},
          {90, 1.92},
          {100, 0.66},
          {110, -0.16},
          {120, -1.81},
          {130, -2.13},
          {140, -3.10},
          {150, -3.58},
          {160, -4.33}}},
        // pitch attitude at 50 kt, where the lateral constraint switches (D-048)
        {"d", {{50, 0.58}}},
        // sideslip at zero roll (VALIDATION.md M7c; most of it closed by D-048,
        // the rest the inflow model, D-027)
        {"f",
         {{50, 0.53},
          {60, -0.61},
          {70, -1.00},
          {80, -1.00},
          {90, -0.92},
          {100, -0.77},
          {110, -0.85},
          {120, -0.61},
          {130, -0.62},
          {140, -0.56},
          {150, -0.48},
          {160, -0.53}}},
        // pedal: hover and low speed, and above 130 kt with the sideslip
        {"g",
         {{0, 1.65},
          {10, 0.28},
          {20, 0.87},
          {30, 1.40},
          {40, 1.11},
          {140, -2.20},
          {150, -2.50},
          {160, -3.44}}},
        // main-rotor power (hover and cruise power)
        {"i",
         {{0, -37.48},
          {10, 15.46},
          {20, 27.41},
          {30, 19.57},
          {40, 26.86},
          {50, 7.65},
          {60, -1.24},
          {70, 3.97},
          {80, -10.81},
          {90, -19.70},
          {100, -30.85},
          {110, -43.27},
          {120, -67.53},
          {130, -53.78},
          {140, -31.45},
          {150, 29.43}}},
    };
    return open;
}

// Level flight, 0 to 160 kt, under GenHel's lateral constraint (Fig. B1), on
// the aircraft `config`.
void level_gate(const Config& config) {
    const test::Uh60aHarness ac(config);
    std::map<std::string, Curve> genhel;
    for (const Quantity& q : b1_quantities()) {
        genhel.emplace(q.panel, Curve("b1", q.panel, "sikorsky_genhel"));
    }
    Point last;
    bool have = false;
    int gated = 0;
    int tracked = 0;
    struct Reading {
        int kt;
        double d;
        double tol;
    };
    std::vector<Reading> lon;  // panel a, gated after the loop with its mean removed
    for (int kt = 0; kt <= 160; kt += 10) {
        Point p = genhel_trim(ac.executive(), kt, 0.0, have ? &last : nullptr);
        ASSERT_TRUE(p.converged) << kt << " kt";
        const Averages a = revolution_average(ac, p.x, p.trim.controls, {}, p.trim.period);
        for (const Quantity& q : b1_quantities()) {
            const Curve& c = genhel.at(q.panel);
            const double g = c(kt);
            if (std::isnan(g)) {
                continue;  // outside the plotted curve
            }
            const double d = q.eyas(p, a) - g;
            const double tol = kSpanTerm * q.span + std::abs(c.slope(kt, 5.0)) * kKtReadOff;
            const auto& open = b1_open();
            const bool is_open = open.contains(q.panel) && open.at(q.panel).contains(kt);
            if (std::string(q.panel) == "a" && !is_open) {
                lon.push_back({kt, d, tol});
                continue;
            }
            const auto it = open.find(q.panel);
            if (it != open.end() && it->second.contains(kt)) {
                EXPECT_NEAR(d, it->second.at(kt), tol)
                    << q.name << " at " << kt << " kt: an open difference changed (record "
                    << it->second.at(kt) << "); update this record and VALIDATION.md M7c";
                ++tracked;
            } else {
                EXPECT_LE(std::abs(d), tol)
                    << q.name << " at " << kt << " kt: Eyas − Sikorsky GenHel = " << d
                    << " beyond the read-off tolerance " << tol;
                ++gated;
            }
        }
        last = p;
        have = true;
    }
    // Longitudinal stick: the shape gated, the calibrated mean tracked (header).
    // [%] Eyas − GenHel over the gated speeds (to 90 kt), recorded 2026-10-22
    // with D-051 (0.28 % with D-048; 0.07 % over every speed before it)
    constexpr double kLonMeanRecord = 0.33;
    ASSERT_EQ(lon.size(), 9U);  // 10 to 90 kt (D-048)
    double mean = 0.0;
    for (const Reading& r : lon) {
        mean += r.d / static_cast<double>(lon.size());
    }
    EXPECT_NEAR(mean, kLonMeanRecord, kSpanTerm * 100.0)
        << "longitudinal cyclic: the calibrated mean offset changed (record " << kLonMeanRecord
        << " %); update this record and VALIDATION.md M7c";
    for (const Reading& r : lon) {
        EXPECT_LE(std::abs(r.d - mean), r.tol)
            << "longitudinal cyclic at " << r.kt << " kt: Eyas − Sikorsky GenHel = " << r.d << ", "
            << r.d - mean << " about the mean, beyond the read-off tolerance " << r.tol;
        ++gated;
    }
    EXPECT_GT(gated, 40);  // the gate covers what it should
    EXPECT_EQ(tracked, 63);
}

// On GenHel's engines (D-053), the rotor speed free.
TEST(BallinTrim, LevelFlightAgainstSikorskyGenHel) {
    level_gate(Config::from_file(config_path()));
}

// On Ballin's engines (D-054), the rotor speed free: the M7 regression on the
// engine configs (issue #92). The same records hold; at 160 kt the engines
// run at 98.7 % corrected gas-generator speed, inside their maps (D-061).
TEST(BallinTrim, LevelFlightOnBallinsEngines) {
    level_gate(Config::from_file(engine_config_path()));
}

// Climb and descent (Figs. B2-B4), the stabilator held at Eyas's level-flight
// incidence as Sikorsky's runs held theirs (Ballin p. 10): changes from level
// flight. The conditions of B2-B4 are missing (DATA_GAPS.md), so only these
// trends are compared, and only the open ones are recorded here.
//
// A climb past the aircraft's limit has no trim, and the gate checks the
// limit it ends on: past full collective (trim keeps the controls inside
// their travel, D-053 amendment 8) the collective on its upper stop; past
// the engines' ceiling (trim keeps Ballin's gas generators inside their maps,
// D-061) every engine at the maps' top. The lateral climb comparison is then
// made at the highest climb that trims, and the power slope over the climbs
// that trim.
enum class Limit { collective, engines };

struct ClimbRecord {
    const char* figure;
    double kt;
    int climb_top;              // the highest climb that trims [ft/min]
    Limit limit;                // what the next one up ends on
    double collective_descent;  // Eyas − GenHel change in collective at −3000 ft/min [%]
    double lateral_climb;       // the same, lateral cyclic at climb_top [%]
    // Eyas − GenHel dP/dVc over −2000 ft/min to min(+2000, climb_top)
    // [hp per 1000 ft/min]
    double power_slope;
};

// Power read-off: 1.75 % of the 3000 hp axis per reading; a least-squares
// slope over points with abscissae x [1000 ft/min] has σ = 52.5 / √Σ(x − x̄)²
// (16.6 hp per 1000 ft/min over the five from −2000 to +2000).
constexpr double kPowerReadOff = 52.5;  // [hp]

// The aircraft `config` against the record `r`, its stabilator held at the
// level-flight incidence [deg] by `held_at`.
void climb_gate(const Config& config, const std::function<test::Uh60aHarness(double)>& held_at,
                const ClimbRecord& r) {
    const test::Uh60aHarness ac(config);
    Point level = genhel_trim(ac.executive(), r.kt, 0.0, nullptr);
    ASSERT_TRUE(level.converged) << r.kt << " kt";
    const double stab =
        revolution_average(ac, level.x, level.trim.controls, {}, level.trim.period).stabilator;
    const test::Uh60aHarness held = held_at(stab / kDeg);
    std::map<int, Point> pts;
    std::map<int, double> power;
    pts[0] = genhel_trim(held.executive(), r.kt, 0.0, &level);
    ASSERT_TRUE(pts[0].converged);
    for (int sign : {1, -1}) {
        for (int k = 1; k <= 3; ++k) {
            const int fpm = sign * 1000 * k;
            pts[fpm] = genhel_trim(held.executive(), r.kt, fpm, &pts[fpm - sign * 1000]);
            if (fpm > r.climb_top) {
                // Past the limit: no trim, on the limit.
                const Point& p = pts[fpm];
                EXPECT_FALSE(p.converged) << r.kt << " kt, " << fpm << " ft/min";
                if (r.limit == Limit::collective) {
                    EXPECT_EQ(p.trim.controls.collective, 1.0)
                        << r.kt << " kt, " << fpm
                        << " ft/min: not trimmed short of full collective";
                } else {
                    EXPECT_TRUE(engines_on_maps_top(held.executive(), p.x, condition(r.kt, fpm),
                                                    p.trim.controls))
                        << r.kt << " kt, " << fpm
                        << " ft/min: not trimmed short of the engines' maps' top";
                }
                break;
            }
            ASSERT_TRUE(pts[fpm].converged) << r.kt << " kt, " << fpm << " ft/min";
        }
    }
    const int slope_top = std::min(2000, r.climb_top);
    for (int fpm = -2000; fpm <= slope_top; fpm += 1000) {
        power[fpm] =
            revolution_average(held, pts[fpm].x, pts[fpm].trim.controls, {}, pts[fpm].trim.period)
                .power /
            kHp;
    }
    const Curve coll(r.figure, "collective", "sikorsky_genhel");
    const Curve lat(r.figure, "lateral", "sikorsky_genhel");
    const Curve pow(r.figure, "power", "sikorsky_genhel");

    const double d_coll =
        (collective_pct(pts[-3000].trim.controls) - collective_pct(pts[0].trim.controls)) -
        (coll(-3.0) - coll(0.0));
    const double tol_coll =
        std::sqrt(2.0) * kSpanTerm * 100.0 + std::abs(coll.slope(-3.0, 0.25)) * kRateReadOff;
    EXPECT_NEAR(d_coll, r.collective_descent, tol_coll)
        << r.kt << " kt: collective change at -3000 ft/min against GenHel (record "
        << r.collective_descent << "); update this record and VALIDATION.md M7c";

    const double top = r.climb_top / 1000.0;
    const double d_lat = (lat_pct(pts[r.climb_top].trim.controls) - lat_pct(pts[0].trim.controls)) -
                         (lat(top) - lat(0.0));
    const double tol_lat =
        std::sqrt(2.0) * kSpanTerm * 100.0 + std::abs(lat.slope(top, 0.25)) * kRateReadOff;
    EXPECT_NEAR(d_lat, r.lateral_climb, tol_lat)
        << r.kt << " kt: lateral cyclic change at +" << r.climb_top
        << " ft/min against GenHel (record " << r.lateral_climb
        << "); update this record and VALIDATION.md M7c";

    // Least-squares slope of Eyas − GenHel over the equally spaced points.
    double mean = 0.0;
    for (const auto& [fpm, p] : power) {
        mean += fpm / 1000.0 / static_cast<double>(power.size());
    }
    double sxd = 0.0;
    double sxx = 0.0;
    for (const auto& [fpm, p] : power) {
        const double x = fpm / 1000.0;
        sxd += (x - mean) * (p - 100.0 * pow(x));  // B2-B4 power is plotted in hp/100
        sxx += (x - mean) * (x - mean);
    }
    const double d_slope = sxd / sxx;
    EXPECT_NEAR(d_slope, r.power_slope, kPowerReadOff / std::sqrt(sxx))
        << r.kt << " kt: power slope against GenHel (record " << r.power_slope
        << "); update this record and VALIDATION.md M7c";
}

// The rotor held at its governed speed (an ideal drive), as these trims were
// made before the engines (D-053). At 137.5 kt +2500 ft/min and up is past
// full collective (+2000 trims on 92.5 %). Before the bound, +3000 ft/min
// "trimmed" on 106.6 % collective, the extra still mixed into the cyclic and
// the tail rotor. GenHel's own curve leaves Fig. B4's 0–100 % scale near
// +2500 ft/min; its lateral and pedal curves go on to +3000.
//
// VALIDATION.md M7c, recorded 2026-10-16, re-recorded 2026-10-22 with D-051
// (tip loss on the inflow; every record moved within its tolerance but the
// 137.5 kt power slope, 73.1 to 91.3). At 60 kt the descent collective
// (within its tolerance, the curve being steep) and the power slopes at 60
// and 100 kt (within the read-off) match; they are recorded here too, so the
// record is one table. The 100 kt lateral climb record moved from 5.27 to
// 8.74 % with GenHel's downwash correction terms (D-048). The 137.5 kt
// lateral climb record moved from 11.88 % at +3000 to 7.18 % at +2000 ft/min
// on 2026-10-29: +3000 is past full collective (D-053 amendment 8).
constexpr std::array<ClimbRecord, 3> kHeldRecords = {{
    {"b2", 60.0, 3000, Limit::collective, 3.21, 5.43, 19.8},
    {"b3", 100.0, 3000, Limit::collective, 5.29, 8.91, 18.0},
    {"b4", 137.5, 2000, Limit::collective, 8.63, 7.18, 91.3},
}};

class ClimbHeld : public ::testing::TestWithParam<ClimbRecord> {};

TEST_P(ClimbHeld, TrendsAgainstSikorskyGenHel) {
    climb_gate(
        ideal_shaft_config(config_path()),
        [](double inc) { return fixed_stabilator(inc, config_path(), true); }, GetParam());
}

// On Ballin's engines (D-054), the rotor speed free: the M7 regression on the
// engine configs (issue #92). The descents at 60 and 100 kt −3000 ft/min
// autorotate, the clutches slipping (D-056). Past the engines' ceiling the
// climbs end with the engines at the maps' top (D-061): +3000 ft/min at 60
// and 100 kt, +2000 at 137.5 kt, where the ideal drive trims on 92.5 %
// collective.
//
// VALIDATION.md M7c, recorded 2026-10-30 (D-061). Where the condition is the
// ideal drive's the records agree with its within their tolerances: the
// power slopes at 60 and 100 kt (19.8 and 18.3 against 19.8 and 18.0), the
// descent collective at 100 and 137.5 kt (4.77 and 8.63 against 5.29 and
// 8.63). At 60 kt −3000 ft/min the rotor autorotates at 108.6 % and the
// descent collective moves from 3.21 to −0.54 %, inside GenHel's read-off
// (3.78). The lateral climb records are at the engines' climb tops, +2000
// and +1000 ft/min, and the 137.5 kt power slope is over −2000 to +1000 ft/min.
constexpr std::array<ClimbRecord, 3> kEngineRecords = {{
    {"b2", 60.0, 2000, Limit::engines, -0.54, 4.09, 19.8},
    {"b3", 100.0, 2000, Limit::engines, 4.77, 5.10, 18.3},
    {"b4", 137.5, 1000, Limit::engines, 8.63, 3.04, 71.7},
}};

class ClimbOnEngines : public ::testing::TestWithParam<ClimbRecord> {};

TEST_P(ClimbOnEngines, TrendsAgainstSikorskyGenHel) {
    climb_gate(
        Config::from_file(engine_config_path()),
        [](double inc) { return fixed_stabilator(inc, engine_config_path(), false); }, GetParam());
}

void PrintTo(const ClimbRecord& r, std::ostream* os) {
    *os << r.figure << " (" << r.kt << " kt)";
}

std::string figure_name(const ::testing::TestParamInfo<ClimbRecord>& info) {
    return info.param.figure;
}

INSTANTIATE_TEST_SUITE_P(BallinTrim, ClimbHeld, ::testing::ValuesIn(kHeldRecords), figure_name);
INSTANTIATE_TEST_SUITE_P(BallinTrim, ClimbOnEngines, ::testing::ValuesIn(kEngineRecords),
                         figure_name);

}  // namespace
}  // namespace eyas
