// Writes the M9 Stage B1′ data (Ballin's T700, D-054) to CSV for VALIDATION.md
// (tools/plot_m9_ballin.py):
// - m9_ballin_table_b1.csv: the UH-60A on Ballin's engines at Table B.1's
//   loading (data/uh60a/engine_ballin/uh60a_table_b1.yaml; 16 825 lbm, CG FS
//   355.0, WL 248.2 in), sea level ISA, bare, trimmed at Table B.1's
//   conditions (hover; 80 kt level; 80 kt at -1000 ft/min, gamma -7.08 deg):
//   the main rotor's torque, the engine's torque, power, fuel flow, speeds,
//   pressures and temperatures, beside Table B.1's (TM-100991 p. 53);
// - m9_ballin_step.csv: from the sea-level and 5250 ft hover trims (GenHel's
//   16 638 lbm), a 0.05 collective step held for 10 s, the body free, no
//   stability augmentation: on Ballin's engines, on GenHel's (D-053), and with
//   the rotor speed held (an ideal drive): rotor speed, the engines' torque
//   referred to the rotor, NG, the fuel flow and the yaw rate, every 0.05 s;
// - m9_ballin_lever.csv (Stage B2a, D-055): the UH-60A on Ballin's engines
//   (GenHel's loading) trimmed at 80 kt, sea level, SAS and FPS on; at 2 s
//   engine 2's power lever goes to the idle stop, at 22 s back to fly, the
//   other controls held: rotor speed, and per engine the turbine's torque,
//   NG, the fuel ratio selected (HMUSEL) with the acceleration limit (WFPAC),
//   the idle and deceleration floors (WFIDM, WFPDC) and the demand (WFPDM), and
//   the clutch's slip, every 0.05 s.
//
// Usage: eyas_m9_ballin_engine_cases <output-dir>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "analysis/free_flight.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"
#include "engine/ballin_t700.hpp"
#include "engine/genhel_t700.hpp"
#include "env/atmosphere.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;

constexpr double kFtLbf = units::kFoot * units::kPoundForce;  // [N·m]
constexpr double kLbmHr = units::kPoundMass / 3600.0;         // [kg/s]
constexpr double kHp = 550.0 * kFtLbf;                        // [W]

std::string ballin_config(const std::string& name) {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/engine_ballin/" + name;
}

Executive build(const Config& cfg) {
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

template <class E>
std::pair<const E*, std::size_t> find(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* e = dynamic_cast<const E*>(&ex.component(i))) {
            return {e, ex.component_offset(i)};
        }
    }
    return {nullptr, 0};
}

void write_table_b1(const std::filesystem::path& dir) {
    const Executive ex = build(Config::from_file(ballin_config("uh60a_table_b1.yaml")));
    const auto [engine, offset] = find<BallinT700>(ex);
    struct Case {
        const char* name;
        double kt, gamma_deg;
        // Table B.1: required torque (hub), engine torque (shaft), hp per engine, fuel per
        // engine, NG, P_s3, P41, T41, P45, T45.
        std::array<double, 10> b1;
    };
    const std::array<Case, 3> cases{{
        {"hover",
         0.0,
         0.0,
         {32865., 229.0, 911.1, 476.3, 41638., 176.34, 174.28, 2292., 37.42, 1632.}},
        {"80kt",
         80.0,
         0.0,
         {20747., 138.9, 552.6, 349.3, 39768., 142.13, 140.26, 2102., 30.66, 1501.}},
        {"80kt_descent",
         80.0,
         -7.08,
         {10792., 76.06, 302.6, 267.7, 38072., 114.27, 112.77, 1982., 25.54, 1424.}},
    }};
    std::ofstream f(dir / "m9_ballin_table_b1.csv");
    f.precision(8);
    f << "# The UH-60A on Ballin's T700s at Table B.1's loading and conditions (D-054): Eyas "
         "and Ballin, NASA TM-100991 Table B.1 (p. 53)\n";
    f << "case,quantity,eyas,ballin\n";
    std::vector<double> x;
    for (const Case& c : cases) {
        TrimCondition tc;
        tc.airspeed = c.kt * units::kKnot;
        tc.flight_path_angle = c.gamma_deg * units::kDegree;
        const TrimResult r = solve_trim(ex, tc, nullptr, x);
        if (!r.converged) {
            throw std::runtime_error(std::string("Table B.1 trim did not converge: ") + c.name);
        }
        const double omega = x[ex.shaft_offset(0) + 1];
        const std::span<const double> xe(x.data() + offset, BallinT700::kEngineStates);
        const AirState air = isa_atmosphere(0.0, 0.0);
        const BallinT700Signals s =
            engine->signals(xe, {omega, r.controls.collective, 1.0, engine->np_reference(),
                                 air.pressure, air.temperature});
        const BallinT700Config& k = engine->config();
        // The engines less the drive train: the rotors' demand.
        const double hub = k.engines * s.q_engine - engine->drive_train_load(omega);
        const std::array<std::pair<const char*, double>, 12> rows{{
            {"collective_pct", 100.0 * r.controls.collective},
            {"rotor_speed_rad_s", omega},
            {"required_torque_hub_ftlbf", hub / kFtLbf},
            {"engine_torque_shaft_ftlbf", s.gas.q_pt / kFtLbf},
            {"hp_per_engine", s.gas.q_pt * k.gear * omega / kHp},
            {"fuel_per_engine_lbm_hr", s.fuel / kLbmHr},
            {"ng_rpm", xe[BallinT700::kNg] * k.ng_des / units::kRpm},
            {"ps3_psia", s.gas.ps3 / units::kPsi},
            {"p41_psia", s.gas.p41 / units::kPsi},
            {"t41_degr", s.gas.t41 / units::kRankine},
            {"p45_psia", s.gas.p45 / units::kPsi},
            {"t45_degr", s.gas.t45 / units::kRankine},
        }};
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const double b = i < 2 ? std::nan("") : c.b1[i - 2];
            f << c.name << ',' << rows[i].first << ',' << rows[i].second << ',' << b << '\n';
        }
        std::cout << c.name << ": " << r.iterations << " iterations, engine torque "
                  << s.gas.q_pt / kFtLbf << " ft*lbf\n";
    }
}

// A collective step from the hover trim at `feet`, held for `seconds`.
void write_step(std::ofstream& f, const Executive& ex, const std::string& drive, double feet,
                double step, double seconds) {
    TrimCondition c;
    c.altitude = feet * units::kFoot;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    if (!r.converged) {
        throw std::runtime_error("hover trim did not converge: " + drive);
    }
    const auto [ballin, ballin_at] = find<BallinT700>(ex);
    const auto [genhel, genhel_at] = find<GenHelT700>(ex);
    const Environment env = trim_environment(c);
    const double dt = trim_period(ex, r) / 64.0;
    Controls u = r.controls;
    u.collective += step;
    Rk4 rk4(x.size());
    const double governed = ex.shafts()[0].speed;
    const AirState air = isa_atmosphere(feet * units::kFoot, 0.0);
    const int every = static_cast<int>(std::lround(0.05 / dt));
    const int steps = static_cast<int>(std::lround(seconds / dt));
    for (int n = 0; n <= steps; ++n) {
        if (n % every == 0) {
            const double omega = x[ex.shaft_offset(0) + 1];
            const BodyState b = unpack_body(x);
            double q_e = 0.0;
            double n_g = 0.0;
            double fuel = 0.0;
            if (ballin != nullptr) {
                const std::span<const double> xe(x.data() + ballin_at, BallinT700::kEngineStates);
                const BallinT700Signals s =
                    ballin->signals(xe, {omega, u.collective, 1.0, ballin->np_reference(),
                                         air.pressure, air.temperature});
                q_e = ballin->config().engines * ballin->config().gear * s.gas.q_pt;
                n_g = xe[BallinT700::kNg];
                fuel = s.fuel * ballin->config().engines;
            } else if (genhel != nullptr) {
                const std::span<const double> xe(x.data() + genhel_at, GenHelT700::kNumStates);
                const GenHelT700Signals s = genhel->signals(xe, omega, u.collective, 0.0);
                q_e = s.q_engine;
                n_g = s.n_gg;
                fuel = s.fuel * genhel->config().engines;
            }
            f << feet << ',' << drive << ',' << n * dt << ',' << 100.0 * omega / governed << ','
              << q_e / kFtLbf << ',' << 100.0 * n_g << ',' << fuel / kLbmHr << ','
              << b.omega_b.z() / units::kDegree << '\n';
        }
        if (n < steps) {
            (void)free_step(ex, rk4, [&](double) { return u; }, env, n * dt, dt, x);
        }
    }
}

// Engine 2's lever to idle and back at 80 kt (D-055).
void write_lever(const std::filesystem::path& dir, const Executive& ex) {
    TrimCondition c;
    c.airspeed = 80.0 * units::kKnot;
    c.afcs.sas_digital = true;
    c.afcs.sas_analog = true;
    c.afcs.fps = true;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    if (!r.converged) {
        throw std::runtime_error("80 kt trim did not converge");
    }
    const auto [engine, at] = find<BallinT700>(ex);
    const Environment env = trim_environment(c);
    const double dt = trim_period(ex, r) / 64.0;
    Rk4 rk4(x.size());
    const double governed = ex.shafts()[0].speed;
    const AirState air = isa_atmosphere(0.0, 0.0);
    const double ratio = kLbmHr / units::kPsi;  // lbm*in^2/(lbf*hr)
    const auto controls = [&](double t) {
        Controls u = r.controls;
        u.engines[1].power_lever = (t >= 2.0 && t < 22.0) ? 0.0 : 1.0;
        return u;
    };
    std::ofstream f(dir / "m9_ballin_lever.csv");
    f.precision(8);
    f << "# UH-60A on Ballin's T700s, 80 kt, sea level, SAS and FPS on: engine 2's power lever "
         "to idle at 2 s, back to fly at 22 s (D-055)\n";
    f << "time_s,rotor_speed_pct,engine,lever,torque_ftlbf,n_g_pct,hmusel,wfpac,wfidm,wfpdc,wfpdm,"
         "slip_rad_s\n";
    const int every = static_cast<int>(std::lround(0.05 / dt));
    const int steps = static_cast<int>(std::lround(40.0 / dt));
    double low = governed;
    for (int n = 0; n <= steps; ++n) {
        const double t = n * dt;
        const double omega = x[ex.shaft_offset(0) + 1];
        low = std::min(low, omega);
        if (n % every == 0) {
            const Controls u = controls(t);
            for (int e = 0; e < engine->config().engines; ++e) {
                const std::span<const double> xe(
                    x.data() + at + static_cast<std::size_t>(e) * BallinT700::kEngineStates,
                    BallinT700::kEngineStates);
                const double lever = u.engines[static_cast<std::size_t>(e)].power_lever;
                const BallinT700Signals s =
                    engine->signals(xe, {omega, u.collective, lever, engine->np_reference(),
                                         air.pressure, air.temperature});
                f << t << ',' << 100.0 * omega / governed << ',' << e + 1 << ',' << lever << ','
                  << s.gas.q_pt / kFtLbf << ',' << 100.0 * xe[BallinT700::kNg] << ','
                  << s.hmusel / ratio << ',' << s.wfpac / ratio << ',' << s.wfidm / ratio << ','
                  << s.wfpdc / ratio << ',' << s.wfpdm / ratio << ',' << xe[BallinT700::kSlip]
                  << '\n';
            }
        }
        if (n < steps) {
            (void)free_step(ex, rk4, controls, env, t, dt, x);
        }
    }
    std::cout << "lever cut: lowest rotor speed " << 100.0 * low / governed << " %\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m9_ballin_engine_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write_table_b1(dir);
        const Executive ballin = build(Config::from_file(ballin_config("uh60a.yaml")));
        const Executive genhel = build(Config::from_file(test::uh60a_config_path()));
        const Executive held = build(test::ballin::ideal_shaft_config());
        std::ofstream f(dir / "m9_ballin_step.csv");
        f.precision(10);
        f << "# UH-60A, 0.05 collective step from the hover trim, 10 s, no stability "
             "augmentation: on Ballin's T700s, on GenHel's, and with the rotor speed held\n";
        f << "altitude_ft,drive,time_s,rotor_speed_pct,engine_torque_ftlbf,n_g_pct,"
             "fuel_lbm_hr,yaw_rate_deg_s\n";
        for (const double feet : {0.0, 5250.0}) {
            write_step(f, ballin, "ballin", feet, 0.05, 10.0);
            write_step(f, genhel, "genhel", feet, 0.05, 10.0);
            write_step(f, held, "held", feet, 0.05, 10.0);
            std::cout << "collective step at " << feet << " ft done\n";
        }
        write_lever(dir, ballin);
    } catch (const std::exception& err) {
        std::cerr << "eyas_m9_ballin_engine_cases: " << err.what() << '\n';
        return 1;
    }
    return 0;
}
