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

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <numbers>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "analysis/trim.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = units::kKnot;
constexpr double kFpm = units::kFoot / 60.0;         // [m/s] per ft/min
constexpr double kHp = 745.69987158227022;           // [W]
constexpr double kAltitude = 5250.0 * units::kFoot;  // [m] ISA: density altitude 5250 ft
constexpr double kRollSwitch = 50.0;                 // [kt] zero roll from here up (Fig. B1(e)-(f))

std::string ballin_config() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/uh60a_ballin.yaml";
}

// The Ballin aircraft with the stabilator held at `incidence_deg` (GenHel's
// STBSET override) instead of its schedule: the config text with its table
// paths made absolute, read from a string.
test::Uh60aHarness fixed_stabilator(double incidence_deg) {
    const std::filesystem::path path = ballin_config();
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    const std::string dir = path.parent_path().string() + "/";
    text = std::regex_replace(text, std::regex(R"(([\s{:,])([A-Za-z0-9_][A-Za-z0-9_./-]*\.csv))"),
                              "$1" + dir + "$2");
    const std::regex schedule(R"(schedule: \{[^}]*\})");
    if (std::distance(std::sregex_iterator(text.begin(), text.end(), schedule),
                      std::sregex_iterator()) != 1) {
        throw std::runtime_error("expected exactly one stabilator schedule in " + path.string());
    }
    std::ostringstream inc;
    inc.precision(17);
    inc << "incidence: \"" << incidence_deg << " deg\"";
    text = std::regex_replace(text, schedule, inc.str());
    return test::Uh60aHarness(Config::from_string(text, path.string() + " (stabilator held)"));
}

struct Averages {
    double power = 0.0;       // [W] main rotor
    double stabilator = 0.0;  // [rad]
};

// Loads averaged over the trimmed revolution with the body held (the trim's
// own orbit, at its own step), as m7_cases.cpp.
Averages revolution_average(const test::Uh60aHarness& ac, std::vector<double> x,
                            const Controls& c) {
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto& af = ac.get<AirframeAero>("airframe");
    const double T = trim_period(ac.executive());
    const int N = 256;
    Averages a;
    double t = 0.0;
    for (int k = 0; k < N; ++k) {
        (void)ac.evaluate(t, x, c, Environment{});
        const SimContext ctx = ac.context(t, x, c, Environment{});
        a.power += mr.evaluate(ctx, ac.states(x, ac.index("main_rotor"))).power / N;
        a.stabilator +=
            af.evaluate(ctx, ac.states(x, ac.index("airframe"))).stabilator_incidence / N;
        ac.hold(t, x, c, Environment{}, T / N, 1);
    }
    return a;
}

struct Point {
    bool converged = false;
    TrimResult trim;
    double sideslip = 0.0;  // [rad]
    std::vector<double> x;
};

// One trim: zero roll (sideslip solved) or the condition's sideslip.
Point trim_at(const Executive& ex, const TrimCondition& c, bool zero_roll,
              const TrimResult* start) {
    Point p;
    if (zero_roll) {
        const RollTrimResult r = solve_trim_at_roll(ex, c, 0.0, start, p.x);
        p.converged = r.converged;
        p.trim = r.trim;
        p.sideslip = r.sideslip;
    } else {
        p.trim = solve_trim(ex, c, start, p.x);
        p.converged = p.trim.converged;
        p.sideslip = c.sideslip;
    }
    return p;
}

const char* kHeader =
    "knots,rate_of_climb_fpm,constraint,converged,collective_pct,lat_cyclic_pct,"
    "lon_cyclic_pct,pedal_pct,pitch_deg,roll_deg,sideslip_deg,stabilator_deg,"
    "mr_power_hp\n";

void row(std::ofstream& out, const test::Uh60aHarness& ac, double kt, double fpm,
         const char* constraint, const Point& p) {
    const Controls& u = p.trim.controls;
    const Averages a = revolution_average(ac, p.x, u);
    out << kt << "," << fpm << "," << constraint << "," << p.converged << ","
        << 100.0 * u.collective << "," << 50.0 * (1.0 + u.lat_cyclic) << ","
        << 50.0 * (1.0 - u.lon_cyclic) << "," << 50.0 * (1.0 + u.pedal) << ","
        << p.trim.pitch / kDeg << "," << p.trim.roll / kDeg << "," << p.sideslip / kDeg << ","
        << a.stabilator / kDeg << "," << a.power / kHp << "\n";
}

TrimCondition condition(double kt, double fpm) {
    TrimCondition c;
    c.airspeed = kt * kKnot;
    c.altitude = kAltitude;
    if (fpm != 0.0) {
        c.flight_path_angle = std::asin(fpm * kFpm / c.airspeed);
    }
    return c;
}

void write(const std::filesystem::path& dir) {
    const test::Uh60aHarness ac(ballin_config());
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
