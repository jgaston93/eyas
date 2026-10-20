// D-048 validation: steady-heading sideslips against Kaplita, NASA CR-177360
// (1985), Figs. 11-13: Sikorsky's GenHel ("existing model", identical to
// CR-166309, Kaplita p. 1) and the USAAEFA flight test at 60, 100 and 140 kt
// (data/uh60a/validation/kaplita_sideslip*.csv, human-reviewed).
//
// Kaplita's simulated runs (p. 2): SAS on, PBA disabled and centred, FPS and
// trim off, the stabilator fixed at its calculated value at zero sideslip.
// Eyas does the same: a zero-sideslip trim with the stabilator on its law
// gives the incidence, then the stabilator is held there and the sideslip
// swept. The aircraft is Ballin's (uh60a_ballin.yaml, 16000 lbm, CG FS 351.0,
// WL 231.5 in, density altitude 5250 ft) as a stand-in for the three test
// flights (15500-16000 lb, FS 348.7-352.0 in, WL about 231.8 in, Hd
// 5500-5900 ft): only gradients are compared, not levels.
//
// Each gradient is the least-squares slope over the sideslips below, Eyas's
// trims and GenHel's curve taken at the same points. The read-off tolerance
// of a slope is σ/√Σ(β − β̄)² with σ one line width, about 1 % of the axis
// span (DATA_GAPS.md, Kaplita), times 2 for the two-sided comparison.
//
// The gate (D-048): every gradient within a quarter of GenHel's, plus the
// read-off. The quarter covers what the comparison doesn't hold equal: the
// stand-in aircraft (dynamic pressure 2-6 % above the tests' at the same
// airspeed), the D-036 parabola within ±5° of sideslip, and the trim
// differences M7c records. Without GenHel's downwash correction terms the
// longitudinal-stick gradients were 43-78 % short of GenHel's and the pitch
// gradients had the wrong sign (measured 2026-10-20: 60/100/140 kt stick
// −0.22/−0.27/−0.29 %/deg against −0.51/−0.86/−1.33, pitch −0.003/−0.021/
// −0.042 against +0.059/+0.122/+0.215); with them, the largest miss is 26 %
// of GenHel's pedal gradient at 60 kt, inside the read-off allowance
// (VALIDATION.md, "Kaplita steady-heading sideslips").

#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "control/genhel_fcs.hpp"
#include "support/m7c_ballin.hpp"
#include "support/m8c3_fps.hpp"

namespace eyas {
namespace {

namespace b = test::ballin;

constexpr std::array<double, 7> kBetas = {-12.0, -8.0, -4.0, 0.0, 4.0, 8.0, 12.0};  // [deg]

// One digitized GenHel curve: Fig. 11-13 panel at a speed, against sideslip [deg].
class SideslipCurve {
public:
    SideslipCurve(int kt, const std::string& panel) {
        std::ifstream f(std::string(EYAS_VEHICLE_DATA_DIR) +
                        "/uh60a/validation/kaplita_sideslip_genhel.csv");
        std::string line;
        bool header = false;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            if (!header) {
                header =
                    true;  // figure,panel,quantity,unit,airspeed_kt,series,sideslip_deg,value,...
                continue;
            }
            std::vector<std::string> c;
            std::stringstream ls(line);
            std::string cell;
            while (std::getline(ls, cell, ',')) {
                c.push_back(cell);
            }
            if (c[1] == panel && std::stoi(c[4]) == kt) {
                v_[std::stod(c[6])] = std::stod(c[7]);
            }
        }
        if (v_.size() < 10) {
            throw std::runtime_error("no GenHel curve " + panel + " at " + std::to_string(kt));
        }
    }
    double operator()(double beta) const {
        const auto hi = v_.lower_bound(beta);
        if (hi == v_.end() || (hi == v_.begin() && hi->first != beta)) {
            throw std::runtime_error("sideslip outside the curve");
        }
        if (hi->first == beta) {
            return hi->second;
        }
        const auto lo = std::prev(hi);
        return lo->second +
               (beta - lo->first) / (hi->first - lo->first) * (hi->second - lo->second);
    }

private:
    std::map<double, double> v_;
};

double slope(const std::vector<double>& x, const std::vector<double>& y) {
    double mx = 0.0;
    double my = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        mx += x[i] / static_cast<double>(x.size());
        my += y[i] / static_cast<double>(x.size());
    }
    double sxy = 0.0;
    double sxx = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        sxy += (x[i] - mx) * (y[i] - my);
        sxx += (x[i] - mx) * (x[i] - mx);
    }
    return sxy / sxx;
}

struct Quantity {
    const char* panel;
    const char* name;
    double span;  // the axis span of the figure's panel, for the read-off
    double (*eyas)(const TrimResult&);
};

const std::array<Quantity, 5>& quantities() {
    static const std::array<Quantity, 5> q = {{
        {"lon", "longitudinal stick [%]", 100.0,
         [](const TrimResult& r) { return b::lon_pct(r.controls); }},
        {"pitch", "pitch attitude [deg]", 20.0,
         [](const TrimResult& r) { return r.pitch / b::kDeg; }},
        {"lat", "lateral stick [%]", 100.0,
         [](const TrimResult& r) { return b::lat_pct(r.controls); }},
        {"roll", "roll attitude [deg]", 20.0, [](const TrimResult& r) { return r.roll / b::kDeg; }},
        {"pedal", "pedal [%]", 100.0, [](const TrimResult& r) { return b::pedal_pct(r.controls); }},
    }};
    return q;
}

TEST(KaplitaSideslip, GradientsAgainstSikorskyGenHel) {
    const AfcsSwitches sas{true, true, false, false, false, false};
    const test::Uh60aHarness law(b::config_path());
    std::vector<double> betas(kBetas.begin(), kBetas.end());
    double sxx = 0.0;
    for (const double x : betas) {
        sxx += x * x;  // the β are centred on zero
    }
    for (const int kt : {60, 100, 140}) {
        SCOPED_TRACE(kt);
        TrimCondition c0 = b::condition(kt, 0.0);
        c0.afcs = sas;
        std::vector<double> x0;
        const TrimResult r0 = solve_trim(law.executive(), c0, nullptr, x0);
        ASSERT_TRUE(r0.converged);
        const double inc =
            x0[test::m8c3::fcs_offset(law.executive()) + GenHelFcs::kServoStab] / b::kDeg;
        const test::Uh60aHarness held = b::fixed_stabilator(inc);
        std::map<double, TrimResult> trims;
        TrimResult prev;
        bool have = false;
        for (const std::vector<double>& sweep :
             {std::vector<double>{0.0, 4.0, 8.0, 12.0}, std::vector<double>{-4.0, -8.0, -12.0}}) {
            have = trims.contains(0.0);
            if (have) {
                prev = trims.at(0.0);
            }
            for (const double beta : sweep) {
                TrimCondition c = c0;
                c.sideslip = beta * b::kDeg;
                std::vector<double> x;
                const TrimResult r = solve_trim(held.executive(), c, have ? &prev : nullptr, x);
                ASSERT_TRUE(r.converged) << beta << " deg";
                trims[beta] = r;
                prev = r;
                have = true;
            }
        }
        for (const Quantity& q : quantities()) {
            const SideslipCurve g(kt, q.panel);
            std::vector<double> ye;
            std::vector<double> yg;
            for (const double beta : betas) {
                ye.push_back(q.eyas(trims.at(beta)));
                yg.push_back(g(beta));
            }
            const double se = slope(betas, ye);
            const double sg = slope(betas, yg);
            const double tol = 2.0 * 0.01 * q.span / std::sqrt(sxx);
            std::printf("%3d kt %-24s Eyas %+.3f GenHel %+.3f per deg (read-off %.3f)\n", kt,
                        q.name, se, sg, tol);
            EXPECT_LE(std::abs(se - sg), 0.25 * std::abs(sg) + tol)
                << q.name << " gradient at " << kt << " kt: Eyas " << se << ", GenHel " << sg;
            EXPECT_GT(se * sg, 0.0) << q.name << " gradient at " << kt << " kt has GenHel's sign";
        }
    }
}

}  // namespace
}  // namespace eyas
