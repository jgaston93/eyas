#pragma once

// Ballin, NASA TM-88360 (1987), App. D and E transient responses (D-046):
// the test conditions of Table C1, the digitized control and response time
// histories (data/uh60a/validation/ballin_d/), and Eyas flown through them as
// Ballin flew the Ames real-time GenHel (p. 10): trimmed to the test
// conditions with the stabilator set by the control system, the stabilator
// then held, SAS, flight-path stabilization and pitch bias actuator off, and
// the test aircraft's control histories fed in with constant biases so that
// the run starts in trim. Used by tests/validation/ballin_app_d_cases.cpp.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "analysis/free_flight.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"
#include "env/atmosphere.hpp"
#include "support/m7c_ballin.hpp"
#include "util/units.hpp"

namespace eyas::test::ballin_d {

using ballin::kDeg;

inline constexpr double kInch = units::kFoot / 12.0;  // [m]

// One row of Table C1 (p. 37) with the figure that plots it.
struct Case {
    std::string file;    // test file, e.g. "201"
    std::string figure;  // "D1" ... "D17", "E1" ... "E3"
    double weight_lbm = 0.0;
    double cg_station_in = 0.0;
    double cg_waterline_in = 0.0;
    double density_altitude_ft = 0.0;
    double pressure_altitude_ft = 0.0;
    double eas_kt = 0.0;       // equivalent airspeed
    double rotor_speed = 0.0;  // [rad/s]
    // "" for the Table C1 loading; else the suffix of a stand-in variant of it
    // (e.g. "wl232.5", tools/build_uh60a_config.py), flown for comparison only
    std::string variant;
};

inline std::string data_dir() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a";
}

inline std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> col;
    std::stringstream ls(line);
    std::string cell;
    while (std::getline(ls, cell, ',')) {
        col.push_back(cell);
    }
    return col;
}

// The plotted rows of Table C1 (data/uh60a/validation/ballin_table_c1.csv).
inline std::vector<Case> cases() {
    const std::string path = data_dir() + "/validation/ballin_table_c1.csv";
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error("cannot open " + path);
    }
    std::vector<Case> out;
    std::string line;
    bool header = false;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (!header) {
            header = true;
            continue;
        }
        const auto c = split_csv(line);
        if (c.size() < 11 || c[3] == "none") {
            continue;
        }
        out.push_back({c[0],
                       c[3],
                       std::stod(c[4]),
                       std::stod(c[5]),
                       std::stod(c[6]),
                       std::stod(c[7]),
                       std::stod(c[8]),
                       std::stod(c[9]),
                       std::stod(c[10]),
                       {}});
    }
    return out;
}

// The case's weight-and-CG variant of the aircraft (tools/build_uh60a_config.py).
inline std::filesystem::path config_path(const Case& c) {
    return data_dir() + "/ballin_app_d/uh60a_file" + c.file +
           (c.variant.empty() ? "" : "_" + c.variant) + ".yaml";
}

// The case as Table C1 gives it, then its stand-in variants: one per generated
// config "uh60a_file<file>_<variant>.yaml", sorted by name.
inline std::vector<Case> with_variants(const Case& c) {
    std::vector<Case> out{c};
    const std::string prefix = "uh60a_file" + c.file + "_";
    std::vector<std::string> names;
    for (const auto& e : std::filesystem::directory_iterator(data_dir() + "/ballin_app_d")) {
        const std::string n = e.path().filename().string();
        if (n.starts_with(prefix) && e.path().extension() == ".yaml") {
            names.push_back(n.substr(prefix.size(), n.size() - prefix.size() - 5));
        }
    }
    std::ranges::sort(names);
    for (const auto& v : names) {
        out.push_back(c);
        out.back().variant = v;
    }
    return out;
}

// The atmosphere of a test point: the geometric altitude [m] and ISA ΔT [K]
// at which the ISA + ΔT day (D-015) has the standard atmosphere's pressure at
// the pressure altitude and its density at the density altitude (both
// geopotential, as altimeters read; Gracey, NASA RP-1046, p. 12).
struct TestAtmosphere {
    double altitude = 0.0;
    double delta_T = 0.0;
    double density = 0.0;  // [kg/m³]
};

// Geometric altitude [m] of geopotential altitude [m'] (USSA Eq. 19).
inline double geometric(double geopotential) {
    return isa::kEarthRadius * geopotential / (isa::kEarthRadius - geopotential);
}

inline TestAtmosphere test_atmosphere(const Case& c) {
    const double p = isa_atmosphere(geometric(c.pressure_altitude_ft * units::kFoot), 0.0).pressure;
    const double rho = isa_atmosphere(geometric(c.density_altitude_ft * units::kFoot), 0.0).density;
    // Newton on (Z, ΔT), finite-difference Jacobian; smooth and nearly linear
    double z = c.pressure_altitude_ft * units::kFoot;
    double dT = 0.0;
    for (int it = 0; it < 30; ++it) {
        auto r = [&](double zz, double tt) {
            const AirState a = isa_atmosphere(zz, tt);
            return std::pair{a.pressure / p - 1.0, a.density / rho - 1.0};
        };
        const auto [f1, f2] = r(z, dT);
        if (std::abs(f1) < 1e-13 && std::abs(f2) < 1e-13) {
            return {z, dT, rho};
        }
        const double hz = 1.0;
        const double ht = 1e-3;
        const auto [a1, a2] = r(z + hz, dT);
        const auto [b1, b2] = r(z, dT + ht);
        const double j11 = (a1 - f1) / hz;
        const double j21 = (a2 - f2) / hz;
        const double j12 = (b1 - f1) / ht;
        const double j22 = (b2 - f2) / ht;
        const double det = j11 * j22 - j12 * j21;
        z -= (j22 * f1 - j12 * f2) / det;
        dT -= (-j21 * f1 + j11 * f2) / det;
    }
    throw std::runtime_error("test atmosphere did not converge for file " + c.file);
}

// True airspeed [m/s] from the equivalent airspeed: V = V_E √(ρ0/ρ)
// (Gracey Eq. 3.20).
inline double true_airspeed(const Case& c, const TestAtmosphere& a) {
    return c.eas_kt * units::kKnot * std::sqrt(isa::kSeaLevelDensity / a.density);
}

// One digitized time history (a figure's quantity and series), read linearly
// between the samples; before the first sample its first value, after the
// last its last (some histories end a sample before the 6 s run does).
class History {
public:
    History(const std::string& figure, const std::string& quantity, const std::string& series) {
        std::string fig = figure;
        for (auto& ch : fig) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        const std::string path = data_dir() + "/validation/ballin_d/ballin_" + fig + ".csv";
        std::ifstream f(path);
        if (!f) {
            throw std::runtime_error("cannot open " + path);
        }
        std::string line;
        bool header = false;
        while (std::getline(f, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            if (!header) {
                header = true;  // quantity,unit,series,t_s,value,flag
                continue;
            }
            const auto col = split_csv(line);
            if (col.size() >= 5 && col[0] == quantity && col[2] == series) {
                t_.push_back(std::stod(col[3]));
                y_.push_back(std::stod(col[4]));
            }
        }
        if (t_.size() < 2) {
            throw std::runtime_error("no history " + figure + ":" + quantity + ":" + series);
        }
    }
    double first() const { return y_.front(); }
    double operator()(double t) const {
        if (t <= t_.front()) {
            return y_.front();
        }
        if (t >= t_.back()) {
            return y_.back();
        }
        std::size_t i = 1;
        while (i < t_.size() - 1 && t_[i] < t) {
            ++i;
        }
        const double f = (t - t_[i - 1]) / (t_[i] - t_[i - 1]);
        return y_[i - 1] + f * (y_[i] - y_[i - 1]);
    }

private:
    std::vector<double> t_;
    std::vector<double> y_;
};

// The control inputs [in] of a run: the plotted simulation's (Ames GenHel,
// the flight test's plus a constant bias, p. 10), as changes from their
// first sample. Ballin's signs: lateral and pedal + right, longitudinal +
// aft (Fig. D3's forward step goes negative), collective + up.
struct Inputs {
    History lat, lon, coll, ped;
    explicit Inputs(const std::string& figure)
        : lat(figure, "lateral_cyclic", "ames_genhel"),
          lon(figure, "longitudinal_cyclic", "ames_genhel"),
          coll(figure, "collective", "ames_genhel"),
          ped(figure, "pedals", "ames_genhel") {}

    // The trim's controls moved by the input changes at t, through the
    // config's travels (10 in collective, lateral and longitudinal, 5.38 in
    // pedal, the normalized ranges [0, 1] and [−1, 1]).
    Controls at(const Controls& trim, double t) const {
        Controls u = trim;
        auto d = [t](const History& h) { return h(t) - h.first(); };
        u.collective += d(coll) / 10.0;
        u.lat_cyclic += 2.0 * d(lat) / 10.0;
        u.lon_cyclic -= 2.0 * d(lon) / 10.0;
        u.pedal += 2.0 * d(ped) / 5.38;
        return u;
    }
};

// The primary servos of GenHel CR-166309 (pp. 5.5-14 to 5.5-17), as in the
// config, or Ames's from no-load acceptance testing (Ballin App. A, p. 27),
// which App. D's simulation uses.
enum class Servo : std::uint8_t { genhel, ames };

// Config text: the case's aircraft with the stabilator held at
// `incidence_deg` (if finite) and the given primary servos.
inline std::string config_text(const Case& c, double incidence_deg, Servo servo) {
    std::string text = std::isfinite(incidence_deg)
                           ? ballin::held_stabilator_text(incidence_deg, config_path(c))
                           : ballin::config_text(config_path(c));
    if (servo == Servo::ames) {
        const std::regex genhel(R"(primary_servo: \{a2: "0\.00114 s\^2", a1: "0\.0473 s"\})");
        if (!std::regex_search(text, genhel)) {
            throw std::runtime_error("expected GenHel's primary servo in " +
                                     config_path(c).string());
        }
        text = std::regex_replace(text, genhel,
                                  R"(primary_servo: {a2: "0.0002041 s^2", a1: "0.020 s"})");
    }
    return text;
}

inline Uh60aHarness aircraft(const Case& c, double incidence_deg, Servo servo) {
    return Uh60aHarness(Config::from_string(config_text(c, incidence_deg, servo),
                                            config_path(c).string() + " (Ballin App. D)"));
}

// The airframe CG's fuselage station and waterline [in], the origin of the
// config's positions, from the comment the config generator writes.
inline std::pair<double, double> airframe_cg(const Case& c) {
    const std::string text = ballin::config_text(config_path(c));
    std::smatch m;
    if (!std::regex_search(
            text, m, std::regex(R"(FS ([0-9.]+), BL [-0-9.]+, WL ([0-9.]+) in, the origin)"))) {
        throw std::runtime_error("no airframe CG in " + config_path(c).string());
    }
    return {std::stod(m[1]), std::stod(m[2])};
}

// App. C (p. 36): the CG accelerometer's station, buttline and waterline [in].
inline constexpr double kAccelFs = 389.0;
inline constexpr double kAccelBl = -31.0;
inline constexpr double kAccelWl = 207.7;

// One sample of a run, in Ballin's units.
struct Sample {
    double t = 0.0;        // [s]
    Controls u;            // the controls flown
    double pitch = 0.0;    // [deg]
    double roll = 0.0;     // [deg]
    double heading = 0.0;  // [deg]
    double p = 0.0;        // [deg/s] body rates
    double q = 0.0;
    double r = 0.0;
    double az_sensor = 0.0;  // [ft/s²] body-z specific force at the CG accelerometer
};

struct Run {
    bool trim_converged = false;
    double stabilator_deg = 0.0;  // held incidence, from the control system's trim
    double airspeed_kt = 0.0;     // true
    TestAtmosphere air;
    TrimResult trim;
    std::size_t unconverged = 0;  // evaluations with an unconverged iteration (D-023)
    std::vector<Sample> samples;
};

// Trims the case under GenHel's lateral constraint (ballin::genhel_trim) with
// the stabilator on the control system, holds it at that trim's mean
// incidence, trims again and flies the input histories for `duration` [s]
// with step dt [s], sampling every `every` steps.
inline Run fly(const Case& c, Servo servo, double duration = 6.0, double dt = 1e-3,
               int every = 25) {
    Run run;
    run.air = test_atmosphere(c);
    const double V = true_airspeed(c, run.air);
    run.airspeed_kt = V / units::kKnot;
    auto at_condition = [&](TrimCondition tc) {
        tc.airspeed = V;
        tc.altitude = run.air.altitude;
        tc.isa_delta_T = run.air.delta_T;
        return tc;
    };
    // GenHel's constraint switches at 50 kt (Ballin Fig. B1; SPEC_REVIEW #31);
    // above it the sideslip is solved, each trim starting from the last one's
    // (as ballin::genhel_trim)
    double sideslip = 3.0 * kDeg;
    auto trim = [&](const Uh60aHarness& ac, const TrimResult* start, std::vector<double>& x) {
        TrimCondition tc = at_condition(TrimCondition{});
        if (run.airspeed_kt < ballin::kRollSwitch) {
            return solve_trim(ac.executive(), tc, start, x);
        }
        tc.sideslip = sideslip;
        const RollTrimResult r = solve_trim_at_roll(ac.executive(), tc, 0.0, start, x);
        sideslip = r.sideslip;
        TrimResult t = r.trim;
        t.converged = r.converged;
        return t;
    };
    const Environment env{.isa_delta_T = run.air.delta_T};
    std::vector<double> x;
    const Uh60aHarness scheduled = aircraft(c, std::numeric_limits<double>::quiet_NaN(), servo);
    const TrimResult first = trim(scheduled, nullptr, x);
    run.stabilator_deg =
        ballin::revolution_average(scheduled, x, first.controls, env).stabilator / kDeg;
    const Uh60aHarness ac = aircraft(c, run.stabilator_deg, servo);
    run.trim = trim(ac, &first, x);
    run.trim_converged = first.converged && run.trim.converged;

    const Inputs in(c.figure);
    const Controls trim_u = run.trim.controls;
    const auto [fs_cg, wl_cg] = airframe_cg(c);
    const Vec3 r_acc{-(kAccelFs - fs_cg) * kInch, -(kAccelBl - 0.0) * kInch,
                     -(kAccelWl - wl_cg) * kInch};
    const int steps = static_cast<int>(std::lround(duration / dt));
    Rk4 rk4(x.size());
    std::vector<double> xdot(x.size());
    auto controls = [&](double t) { return in.at(trim_u, t); };
    for (int s = 0; s <= steps; ++s) {
        const double t = dt * s;
        if (s % every == 0) {
            const Controls u = controls(t);
            if (!ac.executive().derivatives(t, x, u, env, xdot)) {
                ++run.unconverged;
            }
            const BodyState b = unpack_body(x);
            const EulerAngles e = euler_angles(b.q_nb);
            const Vec3 vdot{xdot[body_layout::kVel], xdot[body_layout::kVel + 1],
                            xdot[body_layout::kVel + 2]};
            const Vec3 wdot{xdot[body_layout::kOmega], xdot[body_layout::kOmega + 1],
                            xdot[body_layout::kOmega + 2]};
            const Vec3 g_b = b.q_nb.conjugate() * gravity_n();
            // specific force at the sensor: the acceleration of a point off the
            // CG (Padfield 3rd ed. Eqs. 3A.15-3A.17, in vector form) less gravity
            const Vec3 a = vdot + b.omega_b.cross(b.vel_b) + wdot.cross(r_acc) +
                           b.omega_b.cross(b.omega_b.cross(r_acc)) - g_b;
            Sample smp;
            smp.t = t;
            smp.u = u;
            smp.pitch = e.pitch / kDeg;
            smp.roll = e.roll / kDeg;
            smp.heading = e.heading / kDeg;
            smp.p = b.omega_b.x() / kDeg;
            smp.q = b.omega_b.y() / kDeg;
            smp.r = b.omega_b.z() / kDeg;
            smp.az_sensor = a.z() / units::kFoot;
            run.samples.push_back(smp);
        }
        if (s < steps) {
            run.unconverged +=
                static_cast<std::size_t>(free_step(ac.executive(), rk4, controls, env, t, dt, x));
        }
    }
    return run;
}

}  // namespace eyas::test::ballin_d
