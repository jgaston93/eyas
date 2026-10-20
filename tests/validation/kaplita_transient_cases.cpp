// Writes Eyas flown through Kaplita, NASA CR-177360 (1985), Figs. 4 and 2 for
// comparison with Sikorsky's GenHel and the USAAEFA flight test (VALIDATION.md, "Kaplita
// transient responses"; plotted by tools/plot_kaplita_transients.py over the
// digitized, human-reviewed traces in data/uh60a/validation/kaplita_transients/).
//
// Fig. 4: a half-inch right pedal step at 144 kt (flight 66, run 27); Fig. 2:
// a one-inch lateral stick doublet at 144 kt (run 12; captioned 140 knots, the
// header's VKT 144.0). As Kaplita's simulated transients (p. 2): SAS,
// flight-path stabilization and pitch bias actuator off, the stabilator fixed
// at the flight-test value (the header's IHT: 6.54 and 6.49 deg). Eyas trims at zero roll (sideslip
// solved, GenHel's constraint above 50 kt) and is flown with the flight test's pedal and stick
// histories as changes from their values at 0.5 s (inside every reviewed
// trace; DATA_GAPS.md). Collective isn't plotted and is held.
//
// STAND-INS (approved by the spec owner, 2026-10-20; DATA_GAPS.md): neither
// figure gives the CG waterline or the atmosphere.
// - CG waterline: 231.8 in, Kaplita Fig. 13's (the configs
//   kaplita/uh60a_kaplita_fig<N>_standin.yaml, at each header's weight and CG
//   station).
// - Density altitude: 5750 ft, Kaplita Fig. 13's (flight 19), in the ISA. Also
//   flown at 4000 and 7500 ft to show the sensitivity.
// Not applied: the headers' rotor speed ratios (1.0111, 1.0037; Eyas turns at
// 27.0 rad/s until M9). Variants: the UH-60A as configured ("eyas") and without GenHel's
// downwash correction terms ("no_additions", D-048).
//
// Usage: eyas_kaplita_transient_cases <output-dir>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "analysis/free_flight.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "sim/build.hpp"
#include "support/m7c_ballin.hpp"
#include "util/config.hpp"

namespace {

using namespace eyas;
namespace b = eyas::test::ballin;

// One figure: its number and the header's airspeed (VKT) and stabilator (IHT).
struct Case {
    const char* figure;
    double airspeed_kt;
    double stabilator_deg;
};
constexpr std::array<Case, 2> kCases = {{{"4", 144.0, 6.54}, {"2", 144.0, 6.49}}};
constexpr double kStandInDensityAltitudeFt = 5750.0;  // STAND-IN: Kaplita Fig. 13's
constexpr std::array<double, 2> kSensitivityAltitudesFt = {4000.0, 7500.0};
constexpr double kBaselineS = 0.5;       // [s] input baselines, inside every reviewed trace
constexpr double kDuration = 9.0;        // [s] as plotted
constexpr double kPedalTravelIn = 5.38;  // config: pedal stroke with overtravel
constexpr double kStickTravelIn = 10.0;

std::string config_path(const Case& k) {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/kaplita/uh60a_kaplita_fig" + k.figure +
           "_standin.yaml";
}

// A reviewed flight-test history [in] (data/uh60a/validation/kaplita_transients/).
class History {
public:
    History(const std::string& figure, const std::string& quantity) {
        std::ifstream f(std::string(EYAS_VEHICLE_DATA_DIR) +
                        "/uh60a/validation/kaplita_transients/kaplita_fig" + figure + ".csv");
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
            std::stringstream ss(line);
            std::vector<std::string> c;
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                c.push_back(cell);
            }
            if (c.size() >= 5 && c[0] == quantity && c[2] == "aefa_test") {
                t_.push_back(std::stod(c[3]));
                y_.push_back(std::stod(c[4]));
            }
        }
        if (t_.size() < 2) {
            throw std::runtime_error("no flight-test history " + quantity);
        }
    }
    double operator()(double t) const {
        if (t <= t_.front()) {
            return y_.front();
        }
        if (t >= t_.back()) {
            return y_.back();
        }
        const auto i =
            static_cast<std::size_t>(std::lower_bound(t_.begin(), t_.end(), t) - t_.begin());
        return y_[i - 1] + (t - t_[i - 1]) / (t_[i] - t_[i - 1]) * (y_[i] - y_[i - 1]);
    }

private:
    std::vector<double> t_;
    std::vector<double> y_;
};

void fly(std::ofstream& out, const Case& k, const char* variant, const std::string& text,
         double altitude_ft) {
    const Config cfg = Config::from_string(text, variant);
    const Executive ex = build_executive(cfg, 0.0);
    TrimCondition c;
    c.airspeed = k.airspeed_kt * units::kKnot;
    c.altitude = altitude_ft * units::kFoot;  // ISA: density altitude = altitude
    std::vector<double> x;
    const RollTrimResult rt = solve_trim_at_roll(ex, c, 0.0, nullptr, x);
    std::cout << "Fig. " << k.figure << ", " << variant << ", " << altitude_ft << " ft: trim "
              << (rt.converged ? "converged" : "NOT CONVERGED") << ", sideslip "
              << rt.sideslip / b::kDeg << " deg, pitch " << rt.trim.pitch / b::kDeg << " deg\n";
    if (!rt.converged) {
        throw std::runtime_error("trim did not converge");
    }
    c.sideslip = rt.sideslip;
    const History ped(k.figure, "pedals");
    const History lat(k.figure, "lateral_cyclic");
    const History lon(k.figure, "longitudinal_cyclic");
    const Controls u0 = rt.trim.controls;
    // Kaplita's signs: pedal + right, lateral + right, longitudinal + aft
    auto controls = [&](double t) {
        const double tt = std::max(t, kBaselineS);
        Controls u = u0;
        u.pedal += 2.0 * (ped(tt) - ped(kBaselineS)) / kPedalTravelIn;
        u.lat_cyclic += 2.0 * (lat(tt) - lat(kBaselineS)) / kStickTravelIn;
        u.lon_cyclic -= 2.0 * (lon(tt) - lon(kBaselineS)) / kStickTravelIn;
        return u;
    };
    const Environment env = trim_environment(c);
    constexpr double dt = 1e-3;
    const int steps = static_cast<int>(std::lround(kDuration / dt));
    Rk4 rk4(x.size());
    std::size_t unconverged = 0;
    for (int s = 0; s <= steps; ++s) {
        const double t = s * dt;
        if (s % 25 == 0) {
            const BodyState bs = unpack_body(x);
            const EulerAngles e = euler_angles(bs.q_nb);
            const Vec3 va = bs.vel_b - bs.q_nb.conjugate() * env.wind_n;
            const Controls u = controls(t);
            out << variant << "," << altitude_ft << "," << t << "," << bs.omega_b.x() / b::kDeg
                << "," << bs.omega_b.y() / b::kDeg << "," << bs.omega_b.z() / b::kDeg << ","
                << e.roll / b::kDeg << "," << e.pitch / b::kDeg << "," << e.heading / b::kDeg << ","
                << std::asin(va.y() / va.norm()) / b::kDeg << "," << u.pedal << "," << u.lat_cyclic
                << "," << u.lon_cyclic << "\n";
        }
        if (s < steps) {
            unconverged +=
                static_cast<std::size_t>(free_step(ex, rk4, controls, env, t, dt, x) != 0);
        }
    }
    if (unconverged != 0) {
        std::cout << "  " << unconverged << " steps with an unconverged iteration\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_kaplita_transient_cases <output-dir>\n";
        return 2;
    }
    const std::filesystem::path dir(argv[1]);
    std::filesystem::create_directories(dir);
    try {
        for (const Case& k : kCases) {
            std::string text = b::config_text(config_path(k));
            const std::regex input(R"(incidence_input: [A-Za-z0-9_]+[^\n]*)");
            std::ostringstream inc;
            inc << "incidence: \"" << k.stabilator_deg << " deg\"";
            text = std::regex_replace(text, input, inc.str());
            const std::string bare =
                std::regex_replace(text, std::regex(R"(\n\s*additional_loads: [^\n]*)"), "");
            std::ofstream out(dir / ("kaplita_fig" + std::string(k.figure) + ".csv"));
            out << "# Eyas flown through Kaplita (NASA CR-177360) Fig. " << k.figure << ": "
                << k.airspeed_kt << " kt, SAS/FPS/PBA off, stabilator held at " << k.stabilator_deg
                << " deg, the flight test's pedal and stick histories. STAND-INS (DATA_GAPS.md): "
                   "CG waterline 231.8 in and density altitude 5750 ft, Kaplita Fig. 13's; the "
                   "altitude column also has 4000 and 7500 ft for the sensitivity. Rates [deg/s], "
                   "angles [deg], controls normalized (D-024).\n"
                << "variant,density_altitude_ft,t_s,p_dps,q_dps,r_dps,roll_deg,pitch_deg,"
                   "heading_deg,sideslip_deg,pedal,lat_cyclic,lon_cyclic\n";
            fly(out, k, "eyas", text, kStandInDensityAltitudeFt);
            fly(out, k, "no_additions", bare, kStandInDensityAltitudeFt);
            for (const double h : kSensitivityAltitudesFt) {
                fly(out, k, "eyas", text, h);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
