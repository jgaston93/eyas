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
// value, a calibration, not a validation.

#include <array>
#include <cmath>
#include <map>
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
// recorded 2026-10-16 with D-038 to D-040). Any (panel, speed) not listed here
// is gated.
const std::map<std::string, std::map<int, double>>& b1_open() {
    static const std::map<std::string, std::map<int, double>> open = {
        // lateral cyclic at high speed (sideslip investigation)
        {"b", {{120, 1.38}, {130, 2.63}, {140, 4.17}, {150, 6.29}}},
        // collective: shape against speed
        {"c",
         {{10, 1.48},
          {20, 1.60},
          {30, 1.50},
          {40, 2.04},
          {50, 2.23},
          {60, 1.84},
          {70, 2.16},
          {80, 1.44},
          {90, 0.89},
          {100, -0.38},
          {110, -1.22},
          {120, -2.94},
          {130, -3.36},
          {140, -4.47},
          {150, -5.17},
          {160, -6.44}}},
        // pitch attitude about 0.5° nose up at cruise
        {"d",
         {{90, 0.50},
          {100, 0.51},
          {110, 0.50},
          {120, 0.54},
          {130, 0.46},
          {140, 0.20},
          {150, 0.50}}},
        // sideslip at zero roll (VALIDATION.md M7c, the side-force investigation)
        {"f",
         {{50, 3.30},
          {60, 2.83},
          {70, 2.84},
          {80, 3.11},
          {90, 3.37},
          {100, 3.61},
          {110, 3.59},
          {120, 3.86},
          {130, 3.85},
          {140, 3.89},
          {150, 3.94},
          {160, 3.83}}},
        // pedal: follows from the sideslip and collective
        {"g",
         {{0, 2.07},
          {10, 0.70},
          {20, 1.29},
          {30, 1.75},
          {40, 1.43},
          {50, -2.11},
          {60, -1.03},
          {70, -1.39},
          {80, -2.04},
          {90, -2.82},
          {100, -3.96},
          {110, -5.06},
          {120, -5.83},
          {130, -6.89},
          {140, -7.50},
          {150, -8.06},
          {160, -8.91}}},
        // main-rotor power (hover and cruise power)
        {"i",
         {{0, -74.99},
          {10, -24.82},
          {20, -18.16},
          {30, -27.10},
          {40, -16.64},
          {50, -35.84},
          {60, -42.50},
          {70, -35.45},
          {80, -49.27},
          {90, -57.46},
          {100, -67.57},
          {110, -80.02},
          {120, -106.12},
          {130, -96.48},
          {140, -81.16},
          {150, -36.67}}},
    };
    return open;
}

// Level flight, 0 to 160 kt, under GenHel's lateral constraint (Fig. B1).
TEST(BallinTrim, LevelFlightAgainstSikorskyGenHel) {
    const test::Uh60aHarness ac(config_path());
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
        const Averages a = revolution_average(ac, p.x, p.trim.controls);
        for (const Quantity& q : b1_quantities()) {
            const Curve& c = genhel.at(q.panel);
            const double g = c(kt);
            if (std::isnan(g)) {
                continue;  // outside the plotted curve
            }
            const double d = q.eyas(p, a) - g;
            const double tol = kSpanTerm * q.span + std::abs(c.slope(kt, 5.0)) * kKtReadOff;
            if (std::string(q.panel) == "a") {
                lon.push_back({kt, d, tol});
                continue;
            }
            const auto& open = b1_open();
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
    constexpr double kLonMeanRecord = 0.07;  // [%] Eyas − GenHel, recorded 2026-10-17
    ASSERT_GT(lon.size(), 10U);
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
    EXPECT_EQ(tracked, 72);
}

// Climb and descent (Figs. B2-B4), the stabilator held at Eyas's level-flight
// incidence as Sikorsky's runs held theirs (Ballin p. 10): changes from level
// flight. The conditions of B2-B4 are missing (DATA_GAPS.md), so only these
// trends are compared, and only the open ones are recorded here.
TEST(BallinTrim, ClimbAndDescentTrendsAgainstSikorskyGenHel) {
    const test::Uh60aHarness ac(config_path());
    struct Record {
        const char* figure;
        double kt;
        double collective_descent;  // Eyas − GenHel change in collective at −3000 ft/min [%]
        double lateral_climb;       // the same, lateral cyclic at +3000 ft/min [%]
        double power_slope;  // Eyas − GenHel dP/dVc over ±2000 ft/min [hp per 1000 ft/min]
    };
    // VALIDATION.md M7c, recorded 2026-10-16. At 60 kt the descent collective
    // (within its tolerance, the curve being steep) and the power slopes at 60
    // and 100 kt (within the read-off) match; they are recorded here too, so
    // the record is one table. The 100 kt lateral climb record moved from
    // 5.27 to 8.74 % with GenHel's downwash correction terms (D-048).
    const std::array<Record, 3> records = {{
        {"b2", 60.0, 3.44, 3.16, 17.5},
        {"b3", 100.0, 5.97, 8.74, 12.4},
        {"b4", 137.5, 9.15, 9.50, 73.1},
    }};
    // Power slope read-off: 1.75 % of the 3000 hp axis per reading, a
    // five-point fit over 4000 ft/min: σ_slope = 52.5 / √10 ≈ 16.6 hp per
    // 1000 ft/min.
    constexpr double kSlopeTol = 16.6;
    for (const Record& r : records) {
        Point level = genhel_trim(ac.executive(), r.kt, 0.0, nullptr);
        ASSERT_TRUE(level.converged) << r.kt << " kt";
        const double stab = revolution_average(ac, level.x, level.trim.controls).stabilator;
        const test::Uh60aHarness held = fixed_stabilator(stab / kDeg);
        std::map<int, Point> pts;
        std::map<int, double> power;
        pts[0] = genhel_trim(held.executive(), r.kt, 0.0, &level);
        ASSERT_TRUE(pts[0].converged);
        for (int sign : {1, -1}) {
            for (int k = 1; k <= 3; ++k) {
                const int fpm = sign * 1000 * k;
                pts[fpm] = genhel_trim(held.executive(), r.kt, fpm, &pts[fpm - sign * 1000]);
                ASSERT_TRUE(pts[fpm].converged) << r.kt << " kt, " << fpm << " ft/min";
            }
        }
        for (int fpm = -2000; fpm <= 2000; fpm += 1000) {
            power[fpm] = revolution_average(held, pts[fpm].x, pts[fpm].trim.controls).power / kHp;
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

        const double d_lat = (lat_pct(pts[3000].trim.controls) - lat_pct(pts[0].trim.controls)) -
                             (lat(3.0) - lat(0.0));
        const double tol_lat =
            std::sqrt(2.0) * kSpanTerm * 100.0 + std::abs(lat.slope(3.0, 0.25)) * kRateReadOff;
        EXPECT_NEAR(d_lat, r.lateral_climb, tol_lat)
            << r.kt << " kt: lateral cyclic change at +3000 ft/min against GenHel (record "
            << r.lateral_climb << "); update this record and VALIDATION.md M7c";

        // Least-squares slope over -2000..+2000 ft/min, five equally spaced points.
        double se = 0.0;
        double sg = 0.0;
        for (int fpm = -2000; fpm <= 2000; fpm += 1000) {
            const double x = fpm / 1000.0;
            se += x * power[fpm];
            sg += x * 100.0 * pow(x);  // B2-B4 power is plotted in hp/100
        }
        const double d_slope = (se - sg) / 10.0;  // Σx² = 10
        EXPECT_NEAR(d_slope, r.power_slope, kSlopeTol)
            << r.kt << " kt: power slope against GenHel (record " << r.power_slope
            << "); update this record and VALIDATION.md M7c";
    }
}

}  // namespace
}  // namespace eyas
