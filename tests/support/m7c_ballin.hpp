#pragma once

// The UH-60A at the conditions of Ballin, NASA TM-88360 (1987), App. B (D-035
// M7c amendment, V1), shared by the validation generator
// (tests/validation/m7c_cases.cpp) and the V1 gate (tests/unit/
// test_ballin_trim.cpp): the aircraft data/uh60a/uh60a_ballin.yaml (16000 lbm,
// CG FS 351.0 in, WL 231.5 in) at 5250 ft in the ISA (density altitude 5250
// ft); trims under GenHel's lateral constraint; the stabilator held, as in
// Sikorsky's climb runs; and the digitized, human-reviewed Ballin curves
// (data/uh60a/validation/ballin_b*.csv).

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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

namespace eyas::test::ballin {

inline constexpr double kDeg = std::numbers::pi / 180.0;
inline constexpr double kFpm = units::kFoot / 60.0;         // [m/s] per ft/min
inline constexpr double kHp = 745.69987158227022;           // [W]
inline constexpr double kAltitude = 5250.0 * units::kFoot;  // [m] ISA: density altitude 5250 ft
inline constexpr double kRollSwitch = 50.0;  // [kt] zero roll from here up (Fig. B1(e)-(f))

inline std::string config_path() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/uh60a_ballin.yaml";
}

// Level flight or a steady climb [ft/min] at airspeed kt [kt], straight.
inline TrimCondition condition(double kt, double fpm) {
    TrimCondition c;
    c.airspeed = kt * units::kKnot;
    c.altitude = kAltitude;
    if (fpm != 0.0) {
        c.flight_path_angle = std::asin(fpm * kFpm / c.airspeed);
    }
    return c;
}

struct Point {
    bool converged = false;
    TrimResult trim;
    double sideslip = 0.0;  // [rad]
    std::vector<double> x;
};

// One trim: zero roll (sideslip solved) or the condition's sideslip.
inline Point trim_at(const Executive& ex, const TrimCondition& c, bool zero_roll,
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

// GenHel's lateral constraint as read from Fig. B1(e)-(f) (SPEC_REVIEW #31):
// zero sideslip below 50 kt, zero roll (sideslip solved) from 50 kt up.
inline Point genhel_trim(const Executive& ex, double kt, double fpm, const Point* start) {
    TrimCondition c = condition(kt, fpm);
    if (kt < kRollSwitch) {
        return trim_at(ex, c, false, start != nullptr ? &start->trim : nullptr);
    }
    c.sideslip = start != nullptr ? start->sideslip : 3.0 * kDeg;
    return trim_at(ex, c, true, start != nullptr ? &start->trim : nullptr);
}

struct Averages {
    double power = 0.0;       // [W] main rotor
    double stabilator = 0.0;  // [rad]
};

// Loads averaged over the trimmed revolution with the body held (the trim's
// own orbit, at its own step), as m7_cases.cpp, on the day `env` (the one
// the trim was solved on).
inline Averages revolution_average(const Uh60aHarness& ac, std::vector<double> x, const Controls& c,
                                   const Environment& env = {}) {
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto& af = ac.get<AirframeAero>("airframe");
    const double T = trim_period(ac.executive());
    const int N = 256;
    Averages a;
    double t = 0.0;
    for (int k = 0; k < N; ++k) {
        (void)ac.evaluate(t, x, c, env);
        const SimContext ctx = ac.context(t, x, c, env);
        a.power += mr.evaluate(ctx, ac.states(x, ac.index("main_rotor"))).power / N;
        a.stabilator +=
            af.evaluate(ctx, ac.states(x, ac.index("airframe"))).stabilator_incidence / N;
        ac.hold(t, x, c, env, T / N, 1);
    }
    return a;
}

// The config text at `path` with its table paths (relative, "../" allowed)
// made absolute, so that it can be edited and read from a string.
inline std::string config_text(const std::filesystem::path& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string dir = path.parent_path().string() + "/";
    return std::regex_replace(ss.str(),
                              std::regex(R"(([\s{:,])([A-Za-z0-9_.][A-Za-z0-9_./-]*\.csv))"),
                              "$1" + dir + "$2");
}

// The config text at `path` with the stabilator held at `incidence_deg`
// (GenHel's STBSET override) instead of taking the flight control system's
// incidence (D-045).
inline std::string held_stabilator_text(double incidence_deg, const std::filesystem::path& path) {
    const std::string text = config_text(path);
    const std::regex input(R"(incidence_input: [A-Za-z0-9_]+[^\n]*)");
    if (std::distance(std::sregex_iterator(text.begin(), text.end(), input),
                      std::sregex_iterator()) != 1) {
        throw std::runtime_error("expected exactly one stabilator incidence input in " +
                                 path.string());
    }
    std::ostringstream inc;
    inc.precision(17);
    inc << "incidence: \"" << incidence_deg << " deg\"";
    return std::regex_replace(text, input, inc.str());
}

// Config text with its rotor shaft held at the governed speed (an `ideal`
// drive, D-052) instead of driven by the engines through a `free` one
// (D-053): for checks at constant rotor speed.
inline std::string ideal_shaft_text(const std::string& text) {
    const std::regex drive(R"(drive: free)");
    if (std::distance(std::sregex_iterator(text.begin(), text.end(), drive),
                      std::sregex_iterator()) != 1) {
        throw std::runtime_error("expected exactly one free drive");
    }
    return std::regex_replace(text, drive, "drive: ideal");
}

// The config at `path` with its rotor shaft held (ideal_shaft_text).
inline Config ideal_shaft_config(const std::filesystem::path& path = uh60a_config_path()) {
    return Config::from_string(ideal_shaft_text(config_text(path)),
                               path.string() + " (ideal shaft)");
}

// The Ballin aircraft (or the config at `path`) with the stabilator held at
// `incidence_deg`, and its rotor shaft held too if `ideal_shaft`.
inline Uh60aHarness fixed_stabilator(double incidence_deg,
                                     const std::filesystem::path& path = config_path(),
                                     bool ideal_shaft = false) {
    const std::string text = held_stabilator_text(incidence_deg, path);
    return Uh60aHarness(Config::from_string(ideal_shaft ? ideal_shaft_text(text) : text,
                                            path.string() + " (stabilator held)"));
}

// Controls in Ballin's percent scales: longitudinal 0 = full forward,
// lateral and pedal 100 = full right, collective 100 = full up.
inline double collective_pct(const Controls& u) {
    return 100.0 * u.collective;
}
inline double lat_pct(const Controls& u) {
    return 50.0 * (1.0 + u.lat_cyclic);
}
inline double lon_pct(const Controls& u) {
    return 50.0 * (1.0 - u.lon_cyclic);
}
inline double pedal_pct(const Controls& u) {
    return 50.0 * (1.0 + u.pedal);
}

// One digitized Ballin curve (a figure's panel and series), x ascending, read
// linearly between the samples; outside them, NaN.
class Curve {
public:
    Curve(const std::string& figure, const std::string& panel, const std::string& series) {
        const std::string path =
            std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/validation/ballin_" + figure + ".csv";
        std::ifstream f(path);
        if (!f) {
            throw std::runtime_error("cannot open " + path);
        }
        std::string line;
        std::vector<std::pair<double, double>> pts;
        bool header = false;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            if (!header) {
                header = true;  // panel,quantity,unit,x_quantity,x_unit,series,x,y,flag
                continue;
            }
            std::vector<std::string> col;
            std::stringstream ls(line);
            std::string cell;
            while (std::getline(ls, cell, ',')) {
                col.push_back(cell);
            }
            if (col.size() >= 8 && col[0] == panel && col[5] == series) {
                pts.emplace_back(std::stod(col[6]), std::stod(col[7]));
            }
        }
        if (pts.size() < 2) {
            throw std::runtime_error("no curve " + figure + ":" + panel + ":" + series);
        }
        std::sort(pts.begin(), pts.end());
        for (const auto& [x, y] : pts) {
            x_.push_back(x);
            y_.push_back(y);
        }
    }

    double operator()(double x) const {
        if (x < x_.front() - 1e-9 || x > x_.back() + 1e-9) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        const auto it = std::lower_bound(x_.begin(), x_.end(), x);
        if (it == x_.begin()) {
            return y_.front();
        }
        if (it == x_.end()) {
            return y_.back();
        }
        const auto i = static_cast<std::size_t>(it - x_.begin());
        const double f = (x - x_[i - 1]) / (x_[i] - x_[i - 1]);
        return y_[i - 1] + f * (y_[i] - y_[i - 1]);
    }
    // Local slope dy/dx, central difference over ±h.
    double slope(double x, double h) const {
        const double a = (*this)(std::max(x - h, x_.front()));
        const double b = (*this)(std::min(x + h, x_.back()));
        return (b - a) / (std::min(x + h, x_.back()) - std::max(x - h, x_.front()));
    }

private:
    std::vector<double> x_;
    std::vector<double> y_;
};

}  // namespace eyas::test::ballin
