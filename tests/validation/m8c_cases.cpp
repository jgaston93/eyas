// Writes the M8c data to CSV for VALIDATION.md (tools/plot_m8c.py; D-045):
// the flying UH-60A (data/uh60a/uh60a.yaml), ISA sea level, level flight,
// - trims from hover to 150 kt with the flight control system, with the
//   stabilator's lateral-acceleration feedback on and zeroed;
// - the condensed 6-DOF modes at 60, 90, 120 and 150 kt with the dynamic
//   stabilator and with the stabilator held at its trimmed incidence;
// - the stabilator over the first 3 s of free flight from the trim, with the
//   flight control system started on its free-flight orbit (D-045
//   amendment) and at its held-body values (before it);
// - the stabilator's response to a 0.5° kick of its servo, with the
//   accelerometer and pitch gyro kicked too (Trim.FcsKickDiesOut);
// and the M8c2 data:
// - the trims' stick with the SAS and PBA on against off
//   (Trim.SasAndPbaMoveOnlyTheStick);
// - the modes with the SAS and PBA off and on, the flight control system's
//   states kept dynamic (condense() with them retained), and with the SAS
//   off and them quasi-static, as M8c1 had them;
// - the yaw rate after a 0.05 rad/s kick, SAS off and on
//   (Trim.SasDampsTheDutchRoll).
//
// Usage: eyas_m8c_cases <output-dir>

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "control/genhel_fcs.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;
using units::kDegree;
using units::kKnot;

Executive executive(const std::string& text, const std::string& name) {
    const Config cfg = Config::from_string(text, name);
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

std::string flying_text() {
    return test::ballin::config_text(test::uh60a_config_path());
}

std::string no_lateral_accel_text() {
    const std::string text = flying_text();
    const std::regex gain(R"(lateral_accel_gain: \{gain: "[^"]*")");
    if (std::distance(std::sregex_iterator(text.begin(), text.end(), gain),
                      std::sregex_iterator()) != 1) {
        throw std::runtime_error("expected exactly one lateral_accel_gain");
    }
    return std::regex_replace(text, gain, R"x(lateral_accel_gain: {gain: "0 deg/(ft/s^2)")x");
}

TrimCondition level(double kt) {
    TrimCondition c;
    c.airspeed = kt * kKnot;
    c.altitude = 0.0;
    return c;
}

std::size_t fcs_offset(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const GenHelFcs*>(&ex.component(i)) != nullptr) {
            return ex.component_offset(i);
        }
    }
    throw std::runtime_error("no genhel_fcs component");
}

const GenHelFcs& fcs(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* f = dynamic_cast<const GenHelFcs*>(&ex.component(i))) {
            return *f;
        }
    }
    throw std::runtime_error("no genhel_fcs component");
}

void require(bool ok, const std::string& what) {
    if (!ok) {
        throw std::runtime_error(what);
    }
}

void trims(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c_trims.csv");
    f.precision(12);
    f << "# UH-60A level trims with the flight control system, ISA sea level (M8c1, D-045)\n"
      << "lateral_accel,knots,collective,lon_cyclic,lat_cyclic,pedal,pitch_deg,roll_deg,"
         "stabilator_deg,ay_sensor_mps2\n";
    for (const bool on : {true, false}) {
        const Executive ex = on ? executive(flying_text(), "flying")
                                : executive(no_lateral_accel_text(), "no lateral accel");
        const std::size_t off = fcs_offset(ex);
        TrimResult prev;
        bool have = false;
        for (int kt = 0; kt <= 150; kt += 10) {
            const TrimCondition c = level(kt);
            std::vector<double> x;
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
            prev = r;
            have = true;
            // The held orbit's start: the trimmed (mean) sensor values.
            const std::vector<double> xh = held_orbit_start(ex, c, r);
            const std::span<const double> xf(xh.data() + off, GenHelFcs::kNumStates);
            const SimContext ctx = ex.context(0.0, xh, r.controls, trim_environment(c));
            const GenHelFcsOutputs o = fcs(ex).evaluate(ctx, xf);
            f << (on ? 1 : 0) << ',' << kt << ',' << r.controls.collective << ','
              << r.controls.lon_cyclic << ',' << r.controls.lat_cyclic << ',' << r.controls.pedal
              << ',' << r.pitch / kDegree << ',' << r.roll / kDegree << ','
              << o.stabilator_incidence / kDegree << ',' << xf[GenHelFcs::kAccelY] << '\n';
        }
        std::cout << "trims (lateral acceleration " << (on ? "on" : "off") << ") done\n";
    }
}

void modes_csv(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c_modes.csv");
    f.precision(12);
    f << "# UH-60A condensed 6-DOF modes, dynamic stabilator against held at its trim "
         "(M8c1, D-045)\nstabilator,knots,label,re,im,wn,zeta,longitudinal\n";
    const Executive ex = executive(flying_text(), "flying");
    const std::size_t off = fcs_offset(ex);
    TrimResult prev;
    bool have = false;
    for (const double kt : {0.0, 30.0, 60.0, 90.0, 120.0, 150.0}) {
        const TrimCondition c = level(kt);
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
        require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
        prev = r;
        have = true;
        if (kt < 60.0) {
            continue;  // the chain to 60 kt
        }
        const std::vector<double> xh = held_orbit_start(ex, c, r);
        const SimContext ctx = ex.context(0.0, xh, r.controls, trim_environment(c));
        const double incidence =
            fcs(ex)
                .evaluate(ctx, std::span<const double>(xh.data() + off, GenHelFcs::kNumStates))
                .stabilator_incidence;
        const Executive held = executive(
            test::ballin::held_stabilator_text(incidence / kDegree, test::uh60a_config_path()),
            "stabilator held");
        std::vector<double> xs;
        const TrimResult rh = solve_trim(held, c, &r, xs);
        require(rh.converged, "held trim did not converge at " + std::to_string(kt) + " kt");
        for (const bool dynamic : {true, false}) {
            const Executive& e = dynamic ? ex : held;
            const TrimResult& t = dynamic ? r : rh;
            const LinearModel m = linearize(e, c, t);
            require(m.unconverged == 0, "linearize: unconverged evaluations");
            for (const Mode& md : modes(condense(m), t.vel_b.x())) {
                f << (dynamic ? "dynamic" : "held") << ',' << kt << ',' << md.label << ','
                  << md.eigenvalue.real() << ',' << md.eigenvalue.imag() << ',' << md.frequency
                  << ',' << md.damping << ',' << md.longitudinal << '\n';
            }
        }
        std::cout << "modes at " << kt << " kt done (stabilator " << incidence / kDegree
                  << " deg)\n";
    }
}

void start_and_kick(const std::filesystem::path& dir) {
    std::ofstream fs(dir / "m8c_start.csv");
    std::ofstream fk(dir / "m8c_kick.csv");
    fs.precision(12);
    fk.precision(12);
    fs << "# Stabilator in free flight from the trim, change from t = 0 (M8c1, D-045 "
          "amendment)\nknots,start,time_s,stabilator_change_deg\n";
    fk << "# Stabilator after a 0.5 deg servo kick (+0.3 m/s^2 accelerometer, +0.01 rad/s "
          "gyro), kicked minus unkicked (M8c1)\nknots,time_s,stabilator_difference_deg\n";
    const Executive ex = executive(flying_text(), "flying");
    const std::size_t off = fcs_offset(ex);
    const std::size_t stab = off + GenHelFcs::kServoStab;
    TrimResult prev;
    bool have = false;
    for (const double kt : {0.0, 30.0, 60.0, 90.0, 120.0, 150.0}) {
        const TrimCondition c = level(kt);
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
        require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
        prev = r;
        have = true;
        if (kt < 60.0) {
            continue;
        }
        const Environment env = trim_environment(c);
        const double dt = trim_period(ex) / 256;
        const int steps = static_cast<int>(3.0 / dt);
        const auto controls = [&](double) { return r.controls; };
        std::size_t missed = 0;
        // Before the amendment: the flight control states at their held values.
        std::vector<double> x_old = x;
        for (std::size_t i = 0; i < GenHelFcs::kNumStates; ++i) {
            x_old[off + i] = r.component_states[off - body_layout::kSize + i];
        }
        for (const bool orbit : {true, false}) {
            const auto run =
                test::free_flight(ex, env, orbit ? x : x_old, controls, dt, steps, &missed);
            for (int n = 0; n <= steps; n += 4) {
                const auto i = static_cast<std::size_t>(n);
                fs << kt << ',' << (orbit ? "orbit" : "held") << ',' << n * dt << ','
                   << (run[i][stab] - run[0][stab]) / kDegree << '\n';
            }
        }
        const auto base = test::free_flight(ex, env, x, controls, dt, steps, &missed);
        std::vector<double> xk = x;
        xk[off + GenHelFcs::kAccelY] += 0.3;
        xk[off + GenHelFcs::kGyroQ] += 0.01;
        xk[stab] += 0.5 * kDegree;
        const auto kicked = test::free_flight(ex, env, xk, controls, dt, steps, &missed);
        for (int n = 0; n <= steps; n += 4) {
            const auto i = static_cast<std::size_t>(n);
            fk << kt << ',' << n * dt << ',' << (kicked[i][stab] - base[i][stab]) / kDegree << '\n';
        }
        require(missed == 0, "free flight: unconverged evaluations");
        std::cout << "start and kick at " << kt << " kt done\n";
    }
}

// ---- M8c2 ----------------------------------------------------------------------

constexpr AfcsSwitches kAllOn{true, true, true};

void sas_trims(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c2_trims.csv");
    f.precision(12);
    f << "# UH-60A level trims, SAS and PBA off and on, ISA sea level (M8c2, D-045)\n"
         "afcs,knots,collective,lon_cyclic,lat_cyclic,pedal,pitch_deg,roll_deg\n";
    const Executive ex = executive(flying_text(), "flying");
    TrimResult prev;
    bool have = false;
    for (int kt = 0; kt <= 150; kt += 10) {
        for (const bool on : {false, true}) {
            TrimCondition c = level(kt);
            if (on) {
                c.afcs = kAllOn;
            }
            std::vector<double> x;
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
            if (!on) {
                prev = r;
                have = true;
            }
            f << (on ? 1 : 0) << ',' << kt << ',' << r.controls.collective << ','
              << r.controls.lon_cyclic << ',' << r.controls.lat_cyclic << ',' << r.controls.pedal
              << ',' << r.pitch / kDegree << ',' << r.roll / kDegree << '\n';
        }
    }
    std::cout << "M8c2 trims done\n";
}

// The flight control system's states in a linear model, to keep dynamic in
// condense(), and their weights for modes(): one over a typical size, with
// the sensors' and filters' tenfold below the body's (they echo the body's
// rates and attitudes, so equal weights would give every body mode half to
// them); channel signals per 0.1 %, actuators per 0.025 in.
void fcs_retained(const LinearModel& m, std::vector<std::size_t>& retained,
                  std::vector<double>& weights) {
    using F = GenHelFcs;
    for (std::size_t i = 0; i < m.states.size(); ++i) {
        if (m.states[i].rfind("fcs[", 0) != 0) {
            continue;
        }
        const auto j = static_cast<std::size_t>(std::stoi(m.states[i].substr(4)));
        double w = 1.0;  // rad, rad/s: a tenth of the body's
        if (j == F::kAccelY || j == F::kAccelY + 1 || j == F::kAirspeedLag || j == F::kYawAccelD) {
            w = 0.1;  // m/s², m/s
        }
        for (const std::size_t k : {F::kPitchWashD, F::kPitchHoldD, F::kRollHoldD, F::kYawWashD,
                                    F::kYawHoldD, F::kPbaHold, F::kPba}) {
            if (j == k) {
                w = 10.0;  // fractions
            }
        }
        for (const std::size_t k : {F::kPitchAct, F::kRollAct, F::kYawAct}) {
            if (j == k || j == k + 1) {
                w = 40.0;  // m, m/s
            }
        }
        if (j == F::kYawFade) {
            w = 0.0;
        }
        retained.push_back(i);
        weights.push_back(w);
    }
}

void sas_modes(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c2_modes.csv");
    f.precision(12);
    f << "# UH-60A modes, SAS and PBA off and on (M8c2): 'quasi-static' condenses every "
         "non-body state (M8c1); 'dynamic' keeps the flight control system's states\n"
         "variant,knots,label,re,im,wn,zeta,longitudinal,body\n";
    const Executive ex = executive(flying_text(), "flying");
    TrimResult prev;
    bool have = false;
    // Not 60 kt: a level trim's body-x airspeed sits on the SAS's switches.
    for (const double kt : {0.0, 30.0, 90.0, 120.0, 150.0}) {
        for (const bool on : {false, true}) {
            TrimCondition c = level(kt);
            if (on) {
                c.afcs = kAllOn;
            }
            std::vector<double> x;
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
            if (!on) {
                prev = r;
                have = true;
            }
            const LinearModel m = linearize(ex, c, r);
            require(m.unconverged == 0, "linearize: unconverged evaluations");
            std::vector<std::size_t> retained;
            std::vector<double> weights;
            fcs_retained(m, retained, weights);
            const auto write = [&](const std::string& variant, const std::vector<Mode>& ms) {
                for (const Mode& md : ms) {
                    f << variant << ',' << kt << ',' << md.label << ',' << md.eigenvalue.real()
                      << ',' << md.eigenvalue.imag() << ',' << md.frequency << ',' << md.damping
                      << ',' << md.longitudinal << ',' << md.body << '\n';
                }
            };
            if (!on) {
                write("off_quasi_static", modes(condense(m), r.vel_b.x()));
            }
            write(on ? "on" : "off", modes(condense(m, retained), r.vel_b.x(), weights));
        }
        std::cout << "M8c2 modes at " << kt << " kt done\n";
    }
}

void sas_kick(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c2_kick.csv");
    f.precision(12);
    f << "# Yaw rate after a 0.05 rad/s kick, kicked minus unkicked, SAS and PBA off and on "
         "(M8c2)\nafcs,knots,time_s,yaw_rate_difference_deg_s\n";
    const Executive ex = executive(flying_text(), "flying");
    TrimResult prev;
    bool have = false;
    for (const double kt : {0.0, 30.0, 60.0, 90.0, 120.0, 150.0}) {
        for (const bool on : {false, true}) {
            TrimCondition c = level(kt);
            if (on) {
                c.afcs = kAllOn;
            }
            std::vector<double> x;
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
            if (!on) {
                prev = r;
                have = true;
            }
            if (kt < 90.0) {
                continue;
            }
            const Environment env = trim_environment(c);
            const double dt = trim_period(ex) / 64;
            const int steps = static_cast<int>(12.0 / dt);
            const auto controls = [&](double) { return r.controls; };
            std::size_t missed = 0;
            const auto base = test::free_flight(ex, env, x, controls, dt, steps, &missed);
            std::vector<double> xk = x;
            xk[body_layout::kOmega + 2] += 0.05;
            const auto kicked = test::free_flight(ex, env, xk, controls, dt, steps, &missed);
            require(missed == 0, "free flight: unconverged evaluations");
            for (int n = 0; n <= steps; n += 4) {
                const auto i = static_cast<std::size_t>(n);
                f << (on ? 1 : 0) << ',' << kt << ',' << n * dt << ','
                  << (kicked[i][body_layout::kOmega + 2] - base[i][body_layout::kOmega + 2]) /
                         kDegree
                  << '\n';
            }
        }
        if (kt >= 90.0) {
            std::cout << "M8c2 kick at " << kt << " kt done\n";
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <output-dir>\n", argv[0]);
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        trims(dir);
        modes_csv(dir);
        start_and_kick(dir);
        sas_trims(dir);
        sas_modes(dir);
        sas_kick(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m8c_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
