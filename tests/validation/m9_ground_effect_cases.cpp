// Writes the M9 ground-effect data to CSV for VALIDATION.md
// (tools/plot_m9_ground_effect.py; D-050):
// - m9_ge_factor.csv: κ_g against hub height in hover for both models, with
//   Johnson 2013's empirical fits (Eqs. 4.72–4.76) as κ_g = f_g^(−3/2);
// - m9_ge_hover.csv: the UH-60A (data/uh60a/uh60a.yaml, GenHel's 16 638 lbm,
//   ISA, bare) trimmed in hover at 2.5, 3 and 100 ft wheel height (the
//   bottom of the left main wheel, USAAEFA's measure) over density altitudes
//   0–12 000 ft, with each model: the weight coefficient C_W, the main
//   rotor's mean power coefficient C_P, the hub height z/R. USAAEFA flew at 2
//   and 100 ft (ADA180181 App. E Figs. 2 and 3, whose fits the plotting
//   script evaluates). At 2 ft Eyas's tail wheel, the lowest point at the
//   hover's 4.4° nose-up attitude, grazes the ground (0.04 in at sea level),
//   so the plotting script extrapolates the ratio from 2.5 and 3 ft;
// - m9_ge_height.csv: at ISA sea level, the main rotor's power against wheel
//   height, 2.5–100 ft, all three settings;
// - m9_ge_speed.csv: power required against airspeed, 0–60 kt level flight,
//   at hub heights z/R = 0.55, 0.65 and 1.0 and with no ground, both models
//   (the shape of Johnson 2013 Fig. 5.12).
// Every row has the largest gear normal load over the trimmed revolution:
// zero when no wheel touches.
//
// Usage: eyas_m9_ground_effect_cases <output-dir>

#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "airframe/landing_gear.hpp"
#include "env/atmosphere.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/ground_effect.hpp"
#include "support/m9_ground_effect.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;
using test::Uh60aHarness;
using test::m9::aircraft;
using test::m9::gross_mass;
using test::m9::kNaN;
using test::m9::Point;
using test::m9::trim_near_ground;

constexpr double kFt = units::kFoot;

void write_factor(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m9_ge_factor.csv");
    f.precision(10);
    f << "z_over_r,genhel,cheeseman_bennett\n";
    for (int i = 0; i <= 170; ++i) {
        const double h = 0.3 + 0.01 * i;
        f << h << ',' << ground_effect_factor(GroundEffectModel::genhel, h, 1.0) << ','
          << ground_effect_factor(GroundEffectModel::cheeseman_bennett, h, 1.0) << '\n';
    }
}

void write_hover(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m9_ge_hover.csv");
    f.precision(10);
    f << "model,density_altitude_ft,wheel_ft,converged,hub_z_over_r,cw,cp,power_w,gear_load_n\n";
    for (const std::string model : {"genhel", "cheeseman_bennett"}) {
        const Uh60aHarness ac = aircraft(model);
        const auto& mr = ac.get<BladeElementRotor>("main_rotor");
        const double weight = gross_mass(ac) * units::kStandardGravity;
        const double area = mr.disk_area();
        const double vt = mr.tip_speed();
        for (const double wheel_ft : {2.5, 3.0, 100.0}) {
            const TrimResult* start = nullptr;
            Point prev;
            for (int k = 0; k <= 12; ++k) {
                const double alt_ft = 1000.0 * k;
                const double alt = alt_ft * kFt;
                const Point p = trim_near_ground(ac, 0.0, alt, wheel_ft * kFt, start);
                const double rho = isa_atmosphere(alt, 0.0).density;
                f << model << ',' << alt_ft << ',' << wheel_ft << ',' << p.converged << ','
                  << p.hub_height << ',' << weight / (rho * area * vt * vt) << ','
                  << p.power / (rho * area * vt * vt * vt) << ',' << p.power << ',' << p.gear_load
                  << '\n';
                std::printf("%s %g ft wheel, %g ft: %s, P = %.0f kW\n", model.c_str(), wheel_ft,
                            alt_ft, p.converged ? "ok" : "NOT CONVERGED", p.power / 1000.0);
                prev = p;
                start = p.converged ? &prev.trim : nullptr;
            }
        }
    }
}

void write_height(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m9_ge_height.csv");
    f.precision(10);
    f << "model,wheel_ft,converged,hub_z_over_r,power_w,gear_load_n\n";
    for (const std::string model : {"genhel", "cheeseman_bennett", "none"}) {
        const Uh60aHarness ac = aircraft(model);
        Point prev;
        const TrimResult* start = nullptr;
        for (const double wheel_ft :
             {100.0, 60.0, 40.0, 30.0, 20.0, 15.0, 10.0, 7.0, 5.0, 4.0, 3.0, 2.5}) {
            const Point p = trim_near_ground(ac, 0.0, 0.0, wheel_ft * kFt, start);
            f << model << ',' << wheel_ft << ',' << p.converged << ',' << p.hub_height << ','
              << p.power << ',' << p.gear_load << '\n';
            prev = p;
            start = p.converged ? &prev.trim : nullptr;
        }
    }
}

void write_speed(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m9_ge_speed.csv");
    f.precision(10);
    f << "model,hub_z_over_r_target,airspeed_kt,converged,hub_z_over_r,power_w,gear_load_n\n";
    for (const std::string model : {"genhel", "cheeseman_bennett"}) {
        const Uh60aHarness ac = aircraft(model);
        const auto& mr = ac.get<BladeElementRotor>("main_rotor");
        const auto& left = ac.get<LandingGear>("gear").config().gears.front();
        // Hub to the left wheel's bottom, level [m]: the wheel height for a
        // target hub height (the attitude moves it by a few cm).
        const double hub_to_wheel =
            -mr.config().hub_position_b.z() + left.axle_b.z() + left.tire.radius;
        for (const double target : {0.55, 0.65, 1.0, kNaN}) {
            const double wheel =
                std::isnan(target) ? kNaN : target * mr.config().radius - hub_to_wheel;
            Point prev;
            const TrimResult* start = nullptr;
            for (int k = 0; k <= 15; ++k) {
                const double kt = 4.0 * k;
                const Point p = trim_near_ground(ac, kt * units::kKnot, 0.0, wheel, start);
                f << model << ',' << target << ',' << kt << ',' << p.converged << ','
                  << p.hub_height << ',' << p.power << ',' << p.gear_load << '\n';
                prev = p;
                start = p.converged ? &prev.trim : nullptr;
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m9_ground_effect_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write_factor(dir);
        write_hover(dir);
        write_height(dir);
        write_speed(dir);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
