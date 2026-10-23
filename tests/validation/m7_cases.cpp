// Writes the M7a trim cases to CSV for VALIDATION.md (tools/plot_m7.py;
// D-035):
// - the UH-60A level-flight trim from 0 to 150 kt in 10 kt steps (ISA sea
//   level, 16 638 lbm): controls, attitude, blade angles, main-rotor power,
//   tail-rotor thrust and stabilator incidence, averaged over the trimmed
//   revolution with the body held;
// - free flight from each trim: the mean body accelerations over the first
//   four revolutions, through Model::advance at 1 kHz and through RK4 at the
//   trim's own step (T/256), which divides the revolution exactly.
//
// Usage: eyas_m7_cases <output-dir>

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "analysis/trim.hpp"
#include "control/genhel_fcs.hpp"
#include "core/rk4.hpp"
#include "eyas/model.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "support/frame_status.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kHp = 745.69987158227022;  // [W]
constexpr double kLbf = units::kPoundForce;

struct Averages {
    double power = 0.0;       // [W] main rotor
    double tr_thrust = 0.0;   // [N]
    double stabilator = 0.0;  // [rad]
    // The blade pitch the flight control system sends the rotors (pitch_input:
    // fcs; its mixer, servos and head limits, D-045) [rad].
    SwashplatePitch mr_pitch;
    double tr_pitch = 0.0;
};

// Loads averaged over the trimmed revolution, the body held (the trim's own
// orbit, at its own step).
Averages revolution_average(const test::Uh60aHarness& ac, std::vector<double> x, const Controls& c,
                            double period) {
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto& tr = ac.get<BaileyRotor>("tail_rotor");
    const auto& af = ac.get<AirframeAero>("airframe");
    const auto& fcs = ac.get<GenHelFcs>("fcs");
    const double T = period;  // the trim's own revolution (D-053)
    const int N = 256;
    Averages a;
    double t = 0.0;
    for (int k = 0; k < N; ++k) {
        (void)ac.evaluate(t, x, c, Environment{});  // fills the coupling store at (t, x)
        const SimContext ctx = ac.context(t, x, c, Environment{});
        a.power += mr.evaluate(ctx, ac.states(x, ac.index("main_rotor"))).power / N;
        a.tr_thrust += tr.evaluate(ctx, ac.states(x, ac.index("tail_rotor"))).thrust / N;
        a.stabilator +=
            af.evaluate(ctx, ac.states(x, ac.index("airframe"))).stabilator_incidence / N;
        const GenHelFcsOutputs f = fcs.evaluate(ctx, ac.states(x, ac.index("fcs")));
        a.mr_pitch.theta0 += f.theta0 / N;
        a.mr_pitch.theta1s += f.theta1s / N;
        a.mr_pitch.theta1c += f.theta1c / N;
        a.tr_pitch += f.theta_tr / N;
        ac.hold(t, x, c, Environment{}, T / N, 1);
    }
    return a;
}

void write(const std::filesystem::path& dir) {
    const test::Uh60aHarness ac;
    const Executive& ex = ac.executive();

    std::ofstream trim(dir / "m7_trim.csv");
    trim << "# UH-60A level-flight trim, ISA sea level, 16638 lbm (M7a, D-035)\n"
         << "knots,converged,iterations,collective,lat_cyclic,lon_cyclic,pedal,"
            "pitch_deg,roll_deg,xc_in,xa_in,xb_in,xp_in,theta0_deg,theta1s_deg,theta1c_deg,"
            "theta_tr_cmd_deg,mr_power_hp,tr_thrust_lbf,stabilator_deg,"
            "force_residual_n,moment_residual_nm,periodic_residual\n";
    std::ofstream drift(dir / "m7_drift.csv");
    drift << "# Free flight from trim: mean accelerations over the first four revolutions (M7a)\n"
          << "knots,model_vdot,model_wdot,exact_vdot,exact_wdot\n";

    Model m = Model::from_file(test::uh60a_config_path());
    std::vector<TrimResult> sweep;
    for (int kt = 0; kt <= 150; kt += 10) {
        TrimCondition c;
        c.airspeed = kt * kKnot;
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, c, sweep.empty() ? nullptr : &sweep.back(), x);
        sweep.push_back(r);
        const double T = trim_period(ex, r);  // the trim's own revolution (D-053)
        const Averages a = revolution_average(ac, x, r.controls, T);
        const SwashplatePitch& p = a.mr_pitch;
        const double tr_cmd = a.tr_pitch;
        using tv = test::Travels;
        trim << kt << "," << r.converged << "," << r.iterations << "," << r.controls.collective
             << "," << r.controls.lat_cyclic << "," << r.controls.lon_cyclic << ","
             << r.controls.pedal << "," << r.pitch / kDeg << "," << r.roll / kDeg << ","
             << tv::xc(r.controls.collective) << "," << tv::xa(r.controls.lat_cyclic) << ","
             << tv::xb2(r.controls.lon_cyclic) << "," << tv::xp(r.controls.pedal) << ","
             << p.theta0 / kDeg << "," << p.theta1s / kDeg << "," << p.theta1c / kDeg << ","
             << tr_cmd / kDeg << "," << a.power / kHp << "," << a.tr_thrust / kLbf << ","
             << a.stabilator / kDeg << "," << r.force_residual << "," << r.moment_residual << ","
             << r.periodic_residual << "\n";

        if (kt % 30 != 0) {
            continue;
        }
        // Free flight through the Model at 1 kHz: 931 steps ≈ 4 revolutions.
        if (!m.trim(c, r).converged) {
            throw std::runtime_error("Model::trim did not converge at " + std::to_string(kt) +
                                     " kt");
        }
        const BodyState m0 = m.body_state();
        const int steps = static_cast<int>(std::lround(4.0 * T * 1000.0));
        for (int k = 0; k < steps; ++k) {
            test::require_frame(m.advance(r.controls, Environment{}, 1'000'000), "m7_cases");
        }
        const BodyState m1 = m.body_state();
        const double tm = steps * 1e-3;
        // The same with RK4 at T/256, which divides the revolution exactly.
        std::vector<double> xe = x;
        Rk4 rk(xe.size());
        const int N = 256;
        std::size_t missed = 0;
        for (int k = 0; k < 4 * N; ++k) {
            rk.step(
                [&](double ts, std::span<const double> xs, std::span<double> xd) {
                    if (!ex.derivatives(ts, xs, r.controls, Environment{}, xd)) {
                        ++missed;
                    }
                },
                T * k / N, T / N, xe);
        }
        if (missed != 0) {
            throw std::runtime_error("RK4 flight: unconverged evaluations at " +
                                     std::to_string(kt) + " kt");
        }
        const BodyState e0 = unpack_body(x);
        const BodyState e1 = unpack_body(xe);
        drift << kt << "," << ((m1.vel_b - m0.vel_b) / tm).norm() << ","
              << ((m1.omega_b - m0.omega_b) / tm).norm() << ","
              << ((e1.vel_b - e0.vel_b) / (4.0 * T)).norm() << ","
              << ((e1.omega_b - e0.omega_b) / (4.0 * T)).norm() << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m7_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m7_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
