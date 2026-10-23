// Writes the M9 Stage B engine data to CSV for VALIDATION.md
// (tools/plot_m9_engines.py; D-053):
// - m9_engine_trims.csv: the UH-60A (data/uh60a/uh60a.yaml, GenHel's 16 638
//   lbm, ISA, bare) in level flight at 0-150 kt, sea level and 5250 ft, each
//   from the last: the rotor speed and the trim's period, the gas-generator
//   speed N_GG against the load-demand spindle's N_GG,LDS, the ECU's V_SS (A_U
//   = 0 V: saturated), the engines' torque per engine at the power turbine and
//   the governor's mode (Q_INT's limit);
// - m9_engine_step.csv: from the sea-level and 5250 ft hover trims, a 0.05
//   collective step (half an inch) held for 10 s, the body free, no
//   stability augmentation: the rotor speed, the engines' torque Q_E referred
//   to the rotor, N_GG, the fuel flow and the yaw rate, every 0.05 s; and the
//   same step with the rotor shaft held at its governed speed (an ideal
//   drive, Stage A), for the yaw rate the engines change.
//
// Usage: eyas_m9_engine_cases <output-dir>

#include <cmath>
#include <cstdio>
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
#include "engine/genhel_t700.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;

constexpr double kFtLbf = units::kFoot * units::kPoundForce;  // [N·m]
constexpr double kLbmHr = units::kPoundMass / 3600.0;         // [kg/s]

Executive build(const Config& cfg) {
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

// The engine component and its offset in the full state, or null.
struct EngineAt {
    const GenHelT700* engine = nullptr;
    std::size_t offset = 0;
};
EngineAt find_engine(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* e = dynamic_cast<const GenHelT700*>(&ex.component(i))) {
            return {e, ex.component_offset(i)};
        }
    }
    return {};
}

TrimCondition level(double kt, double feet) {
    TrimCondition c;
    c.airspeed = kt * units::kKnot;
    c.altitude = feet * units::kFoot;
    return c;
}

void write_trims(const Executive& ex, const std::filesystem::path& dir) {
    const EngineAt e = find_engine(ex);
    std::ofstream f(dir / "m9_engine_trims.csv");
    f.precision(10);
    f << "# UH-60A level trims on GenHel's T700s (D-053): rotor speed, N_GG against the "
         "spindle's schedule, the ECU, the engines' torque\n";
    f << "altitude_ft,airspeed_kt,converged,collective,rotor_speed_pct,period_s,n_gg_pct,"
         "n_gg_lds_pct,vss_v,q_pt_per_engine_ftlbf,high_gain\n";
    for (const double feet : {0.0, 5250.0}) {
        TrimResult prev;
        bool have = false;
        for (int kt = 0; kt <= 150; kt += 10) {
            std::vector<double> x;
            const TrimCondition c = level(kt, feet);
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            prev = r;
            have = r.converged;
            const double omega = x[ex.shaft_offset(0) + 1];
            const std::span<const double> xe(x.data() + e.offset, GenHelT700::kNumStates);
            const GenHelT700Signals s = e.engine->signals(xe, omega, r.controls.collective, 0.0);
            f << feet << ',' << kt << ',' << (r.converged ? 1 : 0) << ',' << r.controls.collective
              << ',' << 100.0 * omega / ex.shafts()[0].speed << ',' << r.period << ','
              << 100.0 * s.n_gg << ',' << 100.0 * s.n_gg_lds << ',' << xe[GenHelT700::kVss] << ','
              << s.q_pt / kFtLbf << ',' << (s.high_gain ? 1 : 0) << '\n';
            std::cout << feet << " ft " << kt << " kt: converged " << r.converged << ", Nr "
                      << 100.0 * omega / ex.shafts()[0].speed << " %\n";
        }
    }
}

// A collective step from the hover trim at `feet`, held for `seconds`: one
// row every 0.05 s.
void write_step(std::ofstream& f, const Executive& ex, const std::string& drive, double feet,
                double step, double seconds) {
    const EngineAt e = find_engine(ex);
    const TrimCondition c = level(0.0, feet);
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    if (!r.converged) {
        throw std::runtime_error("m9 engines: hover trim did not converge at " +
                                 std::to_string(feet) + " ft");
    }
    const Environment env = trim_environment(c);
    const double dt = trim_period(ex, r) / 64.0;
    Controls u = r.controls;
    u.collective += step;
    Rk4 rk4(x.size());
    std::vector<double> xd(x.size());
    const double governed = ex.shafts()[0].speed;
    const int every = static_cast<int>(std::lround(0.05 / dt));
    const int steps = static_cast<int>(std::lround(seconds / dt));
    for (int n = 0; n <= steps; ++n) {
        if (n % every == 0) {
            const double t = n * dt;
            const double omega = x[ex.shaft_offset(0) + 1];
            const BodyState b = unpack_body(x);
            double q_e = 0.0;
            double n_gg = 0.0;
            double fuel = 0.0;
            if (e.engine != nullptr) {
                const std::span<const double> xe(x.data() + e.offset, GenHelT700::kNumStates);
                const GenHelT700Signals s = e.engine->signals(xe, omega, u.collective, 0.0);
                q_e = s.q_engine;
                n_gg = s.n_gg;
                fuel = s.fuel * e.engine->config().engines;
            }
            f << feet << ',' << drive << ',' << t << ',' << 100.0 * omega / governed << ','
              << q_e / kFtLbf << ',' << 100.0 * n_gg << ',' << fuel / kLbmHr << ','
              << b.omega_b.z() / units::kDegree << '\n';
        }
        if (n < steps) {
            (void)free_step(ex, rk4, [&](double) { return u; }, env, n * dt, dt, x);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m9_engine_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        const Executive ex = build(Config::from_file(test::uh60a_config_path()));
        write_trims(ex, dir);
        // Held: the same aircraft with its rotor shaft on an ideal drive.
        const Executive held = build(test::ballin::ideal_shaft_config());
        std::ofstream f(dir / "m9_engine_step.csv");
        f.precision(10);
        f << "# UH-60A, 0.05 collective step from the hover trim, 10 s, no stability "
             "augmentation: on GenHel's T700s (free) and with the rotor speed held (ideal)\n";
        f << "altitude_ft,drive,time_s,rotor_speed_pct,engine_torque_ftlbf,n_gg_pct,"
             "fuel_lbm_hr,yaw_rate_deg_s\n";
        for (const double feet : {0.0, 5250.0}) {
            write_step(f, ex, "free", feet, 0.05, 10.0);
            write_step(f, held, "ideal", feet, 0.05, 10.0);
            std::cout << "collective step at " << feet << " ft done\n";
        }
    } catch (const std::exception& err) {
        std::cerr << "eyas_m9_engine_cases: " << err.what() << '\n';
        return 1;
    }
    return 0;
}
