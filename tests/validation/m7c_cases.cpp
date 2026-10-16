// Writes the M7c trim cases to CSV for VALIDATION.md (tools/plot_m7c.py;
// D-035 M7c amendment, V1): the UH-60A at the conditions of Ballin, NASA
// TM-88360 (1987), App. B, for comparison with its GenHel and flight-test
// trims (data/uh60a/validation/ballin_b*.csv):
// - level flight, 0 to 160 kt in 10 kt steps (Fig. B1), under GenHel's
//   lateral constraint as read from Fig. B1(e)-(f): zero sideslip below
//   50 kt, zero roll from 50 kt up (sideslip solved); and, for comparison,
//   zero sideslip at every speed;
// - climb and descent at 60, 100 and 137.5 kt (Figs. B2-B4), -3000 to
//   +3000 ft/min in 500 ft/min steps, at zero roll: with the stabilator on its
//   schedule ("genhel", as Ames's GenHel), and held at Eyas's level-flight
//   incidence for the speed ("fixed_stabilator", as Sikorsky's GenHel in
//   Ballin's Figs. B2-B4, p. 10).
// The aircraft is data/uh60a/uh60a_ballin.yaml (16000 lbm, CG FS 351.0 in,
// WL 231.5 in) at 5250 ft in the ISA (density altitude 5250 ft). Controls
// are written in Ballin's percent scales: longitudinal 0 = full forward,
// lateral and pedal 100 = full right, collective 100 = full up.
//
// Usage: eyas_m7c_cases <output-dir>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "support/m7c_ballin.hpp"

namespace {

using namespace eyas;
using namespace eyas::test::ballin;

const char* kHeader =
    "knots,rate_of_climb_fpm,constraint,converged,collective_pct,lat_cyclic_pct,"
    "lon_cyclic_pct,pedal_pct,pitch_deg,roll_deg,sideslip_deg,stabilator_deg,"
    "mr_power_hp\n";

void row(std::ofstream& out, const test::Uh60aHarness& ac, double kt, double fpm,
         const char* constraint, const Point& p) {
    const Controls& u = p.trim.controls;
    const Averages a = revolution_average(ac, p.x, u);
    out << kt << "," << fpm << "," << constraint << "," << p.converged << "," << collective_pct(u)
        << "," << lat_pct(u) << "," << lon_pct(u) << "," << pedal_pct(u) << ","
        << p.trim.pitch / kDeg << "," << p.trim.roll / kDeg << "," << p.sideslip / kDeg << ","
        << a.stabilator / kDeg << "," << a.power / kHp << "\n";
}

void write(const std::filesystem::path& dir) {
    const test::Uh60aHarness ac(config_path());
    const Executive& ex = ac.executive();
    const std::string comment =
        "# UH-60A at Ballin's App. B conditions (NASA TM-88360): uh60a_ballin.yaml, ISA 5250 ft "
        "(M7c, D-035)\n";

    std::ofstream level(dir / "m7c_level.csv");
    level << comment << kHeader;
    std::vector<Point> genhel;  // under GenHel's constraint, for the climb starts
    TrimResult prev_beta;       // the zero-sideslip sweep
    bool have_beta = false;
    for (int kt = 0; kt <= 160; kt += 10) {
        const TrimCondition c = condition(kt, 0.0);
        const Point b = trim_at(ex, c, false, have_beta ? &prev_beta : nullptr);
        row(level, ac, kt, 0.0, "zero_sideslip", b);
        prev_beta = b.trim;
        have_beta = true;
        if (kt < kRollSwitch) {
            genhel.push_back(b);
            row(level, ac, kt, 0.0, "genhel", b);
        } else {
            TrimCondition cr = c;
            cr.sideslip = genhel.back().sideslip;
            const Point r = trim_at(ex, cr, true, &genhel.back().trim);
            genhel.push_back(r);
            row(level, ac, kt, 0.0, "genhel", r);
        }
        std::cout << kt << " kt: " << (genhel.back().converged ? "converged" : "NOT converged")
                  << "\n";
    }

    std::ofstream climb(dir / "m7c_climb.csv");
    climb << comment << kHeader;
    for (double kt : {60.0, 100.0, 137.5}) {
        // Start from the level trim nearest in speed, then step outwards in rate.
        const auto near = static_cast<std::size_t>(std::lround(kt / 10.0));
        Point level0 = trim_at(ex, condition(kt, 0.0), true, &genhel[near].trim);
        const double level_stab = revolution_average(ac, level0.x, level0.trim.controls).stabilator;
        const test::Uh60aHarness held = fixed_stabilator(level_stab / kDeg);
        for (const auto& [name, h] :
             {std::pair<const char*, const test::Uh60aHarness*>{"genhel", &ac},
              {"fixed_stabilator", &held}}) {
            const Point start = trim_at(h->executive(), condition(kt, 0.0), true, &level0.trim);
            row(climb, *h, kt, 0.0, name, start);
            for (double sign : {1.0, -1.0}) {
                Point last = start;
                for (int k = 1; k <= 6; ++k) {
                    const double fpm = sign * 500.0 * k;
                    TrimCondition c = condition(kt, fpm);
                    c.sideslip = last.sideslip;
                    Point p = trim_at(h->executive(), c, true, &last.trim);
                    row(climb, *h, kt, fpm, name, p);
                    std::cout << kt << " kt, " << fpm << " ft/min (" << name
                              << "): " << (p.converged ? "converged" : "NOT converged") << "\n";
                    if (p.converged) {
                        last = p;
                    }
                }
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m7c_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m7c_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
