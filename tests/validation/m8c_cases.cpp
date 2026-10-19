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
//   (Trim.SasDampsTheDutchRoll);
// and the M8c3 data (the FPS):
// - the trims with the FPS on, and with the turn switch on (pedal, held
//   drift rate; Uh60aFps.TrimsMoveOnlyTheStick);
// - the modes with the FPS on against the SAS and PBA alone;
// - the flights of test_uh60a_fps.cpp (tests/support/m8c3_fps.hpp);
// - D-042 (a): 30 min hands-off with the whole AFCS on, hover and 120 kt.
//
// Usage: eyas_m8c_cases <output-dir>

#include <algorithm>
#include <cmath>
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
#include "support/m8a_long_runs.hpp"
#include "support/m8c3_fps.hpp"
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
// them); channel signals per 0.1 %, actuators per 0.025 in. Without `fps`
// the FPS's states are left to condense(), which drops those that feed
// nothing with the FPS off (its integrators would be zero modes).
void fcs_retained(const LinearModel& m, std::vector<std::size_t>& retained,
                  std::vector<double>& weights, bool fps = false) {
    using F = GenHelFcs;
    for (std::size_t i = 0; i < m.states.size(); ++i) {
        if (m.states[i].rfind("fcs[", 0) != 0) {
            continue;
        }
        const auto j = static_cast<std::size_t>(std::stoi(m.states[i].substr(4)));
        if (!fps && j >= F::kFpsBegin) {
            continue;
        }
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
        // FPS (M8c3): integrators, holds, trim actuators and the collective
        // reference are fractions; the airspeed reference, rate limiter and
        // lateral-acceleration lag m/s and m/s²; the attitude and heading
        // references rad.
        for (const std::size_t k :
             {F::kFpsSpeedInt, F::kFpsPitchHold, F::kFpsPitchAct, F::kFpsRollInt, F::kFpsRollHold,
              F::kFpsRollAct, F::kFpsYawInt, F::kFpsCollRef, F::kFpsYawHold, F::kFpsYawAct}) {
            if (j == k) {
                w = 10.0;
            }
        }
        for (const std::size_t k : {F::kFpsSpeedRef, F::kFpsSpeedRl, F::kFpsAccelLag}) {
            if (j == k) {
                w = 0.1;
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

// ---- M8c3 ----------------------------------------------------------------------

// Level trims with the FPS on, against the SAS and PBA alone (equal, bit for
// bit), and with the turn switch on: the pedal it moves and the held drift
// rate of its lateral-acceleration integral.
void fps_trims(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c3_trims.csv");
    f.precision(12);
    f << "# UH-60A level trims with the FPS on, ISA sea level (M8c3, D-045): largest control "
         "or attitude difference from SAS and PBA alone; with the turn switch on, its pedal "
         "change and held drift rate\nknots,fps_largest_difference,turn_pedal_change,"
         "turn_held_drift_pct_s,turn_ay_mps2\n";
    const Executive ex = test::m8c3::executive();
    const std::size_t off = fcs_offset(ex) - body_layout::kSize;
    TrimResult prev;
    bool have = false;
    for (int kt = 0; kt <= 150; kt += 10) {
        std::vector<double> x;
        const TrimResult sas =
            solve_trim(ex, test::m8c3::level(kt, test::m8c3::kSas), have ? &prev : nullptr, x);
        require(sas.converged, "trim did not converge at " + std::to_string(kt) + " kt");
        prev = sas;
        have = true;
        const TrimResult fps = solve_trim(ex, test::m8c3::level(kt, test::m8c3::kFps), &sas, x);
        TrimCondition c = test::m8c3::level(kt, test::m8c3::kFps);
        c.afcs.fps_turn = true;
        const TrimResult turn = solve_trim(ex, c, &sas, x);
        require(fps.converged && turn.converged, "FPS trim failed at " + std::to_string(kt));
        const double diff =
            std::max({std::abs(fps.controls.collective - sas.controls.collective),
                      std::abs(fps.controls.lat_cyclic - sas.controls.lat_cyclic),
                      std::abs(fps.controls.lon_cyclic - sas.controls.lon_cyclic),
                      std::abs(fps.controls.pedal - sas.controls.pedal),
                      std::abs(fps.pitch - sas.pitch), std::abs(fps.roll - sas.roll)});
        f << kt << ',' << diff << ',' << turn.controls.pedal - sas.controls.pedal << ','
          << turn.held_drift_rate * 100.0 << ','
          << turn.component_states[off + GenHelFcs::kFpsAccelLag] << '\n';
    }
    std::cout << "M8c3 trims done\n";
}

// The modes with the SAS and PBA, and with the FPS too, every flight control
// state kept dynamic (as M8c2's).
void fps_modes(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c3_modes.csv");
    f.precision(12);
    f << "# UH-60A modes, SAS and PBA on, and the FPS on too (M8c3); the flight control "
         "system's states kept dynamic\nvariant,knots,label,re,im,wn,zeta,longitudinal,body\n";
    const Executive ex = test::m8c3::executive();
    TrimResult prev;
    bool have = false;
    for (const double kt : {0.0, 30.0, 90.0, 120.0, 150.0}) {
        for (const bool fps : {false, true}) {
            const TrimCondition c =
                test::m8c3::level(kt, fps ? test::m8c3::kFps : test::m8c3::kSas);
            std::vector<double> x;
            const TrimResult r = solve_trim(ex, c, have ? &prev : nullptr, x);
            require(r.converged, "trim did not converge at " + std::to_string(kt) + " kt");
            if (!fps) {
                prev = r;
                have = true;
            }
            const LinearModel m = linearize(ex, c, r);
            require(m.unconverged == 0, "linearize: unconverged evaluations");
            std::vector<std::size_t> retained;
            std::vector<double> weights;
            fcs_retained(m, retained, weights, fps);
            for (const Mode& md : modes(condense(m, retained), r.vel_b.x(), weights)) {
                f << (fps ? "fps" : "sas") << ',' << kt << ',' << md.label << ','
                  << md.eigenvalue.real() << ',' << md.eigenvalue.imag() << ',' << md.frequency
                  << ',' << md.damping << ',' << md.longitudinal << ',' << md.body << '\n';
            }
        }
        std::cout << "M8c3 modes at " << kt << " kt done\n";
    }
}

// The flights of test_uh60a_fps.cpp, sampled every 0.25 s.
void fps_flights(const std::filesystem::path& dir) {
    namespace c3 = test::m8c3;
    std::ofstream f(dir / "m8c3_flights.csv");
    f.precision(10);
    f << "# UH-60A flights with the FPS (M8c3; tests/support/m8c3_fps.hpp): changes from the "
         "start\ncase,variant,time_s,roll_deg,pitch_deg,heading_deg,airspeed_kt,ay_mps2,"
         "yaw_rate_deg_s\n";
    const auto write = [&](const std::string& name, const std::string& variant,
                           const std::vector<c3::Sample>& s) {
        const c3::Sample& a = s.front();
        for (const c3::Sample& q : s) {
            f << name << ',' << variant << ',' << q.t << ',' << q.phi - a.phi << ','
              << q.theta - a.theta << ',' << q.heading << ',' << q.airspeed - a.airspeed << ','
              << q.ay << ',' << q.r << '\n';
        }
    };
    const Executive ex = c3::executive();
    for (const double kt : {60.0, 120.0}) {
        const c3::Trims t = c3::trims(ex, kt);
        for (const int axis : {1, 0}) {
            const std::string name = std::to_string(static_cast<int>(kt)) +
                                     (axis == 1 ? "kt_pitch_kick" : "kt_roll_kick");
            for (const bool fps : {false, true}) {
                write(name, fps ? "fps" : "sas", c3::rate_kick(ex, t, kt, axis, fps, 30.0));
            }
        }
    }
    for (const double kt : {60.0, 100.0}) {
        const c3::Trims t = c3::trims(ex, kt);
        for (const bool fps : {false, true}) {
            write(std::to_string(static_cast<int>(kt)) + "kt_stick_push", fps ? "fps" : "sas",
                  c3::stick_push(ex, t, kt, fps, 150.0));
        }
    }
    {
        const c3::Trims t = c3::trims(ex, 20.0);
        write("20kt_gust", "fps", c3::gust(ex, t, 20.0, true, 40.0));
        write("20kt_gust", "sas", c3::gust(ex, t, 20.0, false, 20.0));
        for (const bool fps : {false, true}) {
            write("20kt_collective_step", fps ? "fps" : "sas",
                  c3::collective_step(ex, t, 20.0, fps, 0.05, 10.0));
        }
        const Executive no_coll = c3::executive("physical", false);
        const c3::Trims tn = c3::trims(no_coll, 20.0);
        write("20kt_collective_step", "fps_no_ksxcyo",
              c3::collective_step(no_coll, tn, 20.0, true, 0.05, 10.0));
    }
    for (const char* wiring : {"physical", "as_printed"}) {
        const Executive e = c3::executive(wiring);
        const c3::Trims hover = c3::trims(e, 0.0);
        write("hover_pedal_push", std::string("fps_") + wiring,
              c3::pedal_push(e, hover, 0.0, true, 0.05, 90.0));
        const c3::Trims t = c3::trims(e, 100.0);
        write("100kt_turn", std::string("on_") + wiring, c3::turn(e, t, 100.0, true, 40.0));
        if (std::string(wiring) == "physical") {
            write("hover_pedal_push", "sas", c3::pedal_push(e, hover, 0.0, false, 0.05, 90.0));
            write("100kt_turn", "off_physical", c3::turn(e, t, 100.0, false, 40.0));
        }
    }
    std::cout << "M8c3 flights done\n";
}

// D-042 (a): 30 min hands-off with the whole AFCS on (test::afcs_free_flight).
void afcs_long_run(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8c3_afcs_30min.csv");
    f.precision(17);
    f << "# UH-60A hands-off for 30 min with the SAS, PBA and FPS on, from level trims at 60 Hz "
         "(D-042 a, M8c3)\nspeed_kt,time_s,x_m,y_m,z_m,roll_rad,pitch_rad,heading_rad,"
         "airspeed_mps,rate_radps,quat_norm_error,finite\n";
    for (const double kt : {0.0, 120.0}) {
        const std::vector<test::AfcsSample> p = test::afcs_free_flight(kt, 1800);
        require(!p.empty(), "AFCS trim failed at " + std::to_string(kt) + " kt");
        for (const test::AfcsSample& s : p) {
            f << kt << ',' << s.t << ',' << s.pos_n.x() << ',' << s.pos_n.y() << ',' << s.pos_n.z()
              << ',' << s.roll << ',' << s.pitch << ',' << s.heading << ',' << s.speed << ','
              << s.rate << ',' << s.quat_norm_error << ',' << (s.finite ? 1 : 0) << '\n';
        }
        std::cout << "AFCS 30 min at " << kt << " kt done (finite " << p.back().finite << ", t "
                  << p.back().t_ns << " ns)\n";
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
        fps_trims(dir);
        fps_modes(dir);
        fps_flights(dir);
        afcs_long_run(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m8c_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
