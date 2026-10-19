#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "analysis/free_flight.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "control/genhel_fcs.hpp"
#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "util/units.hpp"

// M8c3 (D-045): the flying UH-60A with the FPS, flown hands-off (or by a
// simple pilot) from level trims, for the FPS's jobs: attitude, airspeed
// above 30 kt and heading held, turns coordinated with the turn switch on.
// Shared by test_uh60a_fps.cpp and the VALIDATION generator m8c_cases.cpp.
namespace eyas::test::m8c3 {

constexpr AfcsSwitches kSas{true, true, true, false, false, false};  // SAS and PBA
constexpr AfcsSwitches kFps{true, true, true, true, false, false};   // and the FPS

// The flying UH-60A config with the yaw integrator wired `wiring` ("physical"
// or "as_printed") and, if `collective_gain` is false, KSXCYO zeroed.
inline Executive executive(const std::string& wiring = "physical", bool collective_gain = true) {
    std::string t = ballin::config_text(uh60a_config_path());
    const auto replace_one = [&](const std::string& pattern, const std::string& with) {
        const std::regex re(pattern);
        if (std::distance(std::sregex_iterator(t.begin(), t.end(), re), std::sregex_iterator()) !=
            1) {
            throw std::runtime_error("m8c3: expected exactly one " + pattern);
        }
        t = std::regex_replace(t, re, with);
    };
    replace_one("integrator_wiring: physical", "integrator_wiring: " + wiring);
    if (!collective_gain) {
        replace_one(R"(collective: \{gain: -0\.625)", "collective: {gain: 0");
    }
    const Config cfg = Config::from_string(t, "UH-60A (M8c3, " + wiring + ")");
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

inline std::size_t fcs_offset(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const GenHelFcs*>(&ex.component(i)) != nullptr) {
            return ex.component_offset(i);
        }
    }
    throw std::runtime_error("m8c3: no genhel_fcs component");
}

inline TrimCondition level(double kt, const AfcsSwitches& s) {
    TrimCondition c;
    c.airspeed = kt * units::kKnot;
    c.afcs = s;
    return c;
}

// A level trim with the SAS and PBA, then with the FPS on from it.
struct Trims {
    TrimResult sas, fps;
    std::vector<double> x_sas, x_fps;  // the start states
};
inline Trims trims(const Executive& ex, double kt) {
    Trims t;
    t.sas = solve_trim(ex, level(kt, kSas), nullptr, t.x_sas);
    t.fps = solve_trim(ex, level(kt, kFps), &t.sas, t.x_fps);
    if (!t.sas.converged || !t.fps.converged) {
        throw std::runtime_error("m8c3: trim did not converge at " + std::to_string(kt) + " kt");
    }
    return t;
}

// One sample of a flight [deg, kt, m/s², deg/s], every 0.25 s.
struct Sample {
    double t = 0.0;
    double phi = 0.0, theta = 0.0;
    double heading = 0.0;  // change from t = 0, unwrapped
    double airspeed = 0.0;
    double ay = 0.0;  // the lateral accelerometer's reading
    double r = 0.0;   // yaw rate
};
constexpr double kSampleEvery = 0.25;  // [s]

// Flies `seconds` from x at RK4 steps of a 64th of a revolution, the controls
// from pilot(t, x) at the start of each step. Throws if an evaluation's
// iteration fails to converge or the state turns non-finite.
template <class Pilot>
std::vector<Sample> fly(const Executive& ex, const Environment& env, std::vector<double> x,
                        Pilot&& pilot, double seconds) {
    const double dt = trim_period(ex) / 64;
    const int steps = static_cast<int>(std::ceil(seconds / dt));
    const std::size_t accel = fcs_offset(ex) + GenHelFcs::kAccelY;
    Rk4 rk4(x.size());
    std::vector<Sample> out;
    double next = 0.0;
    double unwrapped = 0.0;  // heading change [rad]
    double last = 0.0;
    for (int n = 0; n <= steps; ++n) {
        const double t = n * dt;
        const BodyState b = unpack_body(x);
        const EulerAngles e = euler_angles(b.q_nb);
        if (n == 0) {
            last = e.heading;
        }
        unwrapped += std::remainder(e.heading - last, 2.0 * std::numbers::pi);
        last = e.heading;
        if (t >= next - 1e-9) {
            Sample s;
            s.t = t;
            s.phi = e.roll / units::kDegree;
            s.theta = e.pitch / units::kDegree;
            s.heading = unwrapped / units::kDegree;
            s.airspeed = b.vel_b.norm() / units::kKnot;
            s.ay = x[accel];
            s.r = b.omega_b.z() / units::kDegree;
            out.push_back(s);
            next += kSampleEvery;
        }
        if (n == steps) {
            break;
        }
        const Controls u = pilot(t, x);
        if (free_step(ex, rk4, [&](double) { return u; }, env, t, dt, x) != 0) {
            throw std::runtime_error("m8c3: unconverged evaluation in free flight");
        }
        if (!std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); })) {
            throw std::runtime_error("m8c3: free flight turned non-finite");
        }
    }
    return out;
}

// The samples within [t0, t1].
inline std::vector<Sample> window(const std::vector<Sample>& s, double t0, double t1) {
    std::vector<Sample> w;
    for (const Sample& q : s) {
        if (q.t >= t0 - 1e-9 && q.t <= t1 + 1e-9) {
            w.push_back(q);
        }
    }
    return w;
}

// The largest |f(sample)| over [t0, t1].
template <class F>
double largest(const std::vector<Sample>& s, double t0, double t1, F&& f) {
    double m = 0.0;
    for (const Sample& q : window(s, t0, t1)) {
        m = std::max(m, std::abs(f(q)));
    }
    return m;
}

// ---- The cases --------------------------------------------------------------

// Hands-off from the trim after a kick of the pitch (axis 1, 0.05 rad/s) or
// roll rate (axis 0, 0.1 rad/s), with the SAS and PBA or with the FPS too.
inline std::vector<Sample> rate_kick(const Executive& ex, const Trims& tr, double kt, int axis,
                                     bool fps, double seconds) {
    std::vector<double> x = fps ? tr.x_fps : tr.x_sas;
    x[body_layout::kOmega + static_cast<std::size_t>(axis)] += axis == 1 ? 0.05 : 0.1;
    const Controls u = (fps ? tr.fps : tr.sas).controls;
    return fly(
        ex, trim_environment(level(kt, kSas)), x,
        [&](double, const std::vector<double>&) { return u; }, seconds);
}

// The stick held 0.01 (0.05 in) forward of the trim from t = 0.
inline std::vector<Sample> stick_push(const Executive& ex, const Trims& tr, double kt, bool fps,
                                      double seconds) {
    Controls u = (fps ? tr.fps : tr.sas).controls;
    u.lon_cyclic += 0.01;
    return fly(
        ex, trim_environment(level(kt, kSas)), fps ? tr.x_fps : tr.x_sas,
        [&](double, const std::vector<double>&) { return u; }, seconds);
}

// A 5 kt gust along the body x axis at t = 0 (the body's velocity raised).
inline std::vector<Sample> gust(const Executive& ex, const Trims& tr, double kt, bool fps,
                                double seconds) {
    std::vector<double> x = fps ? tr.x_fps : tr.x_sas;
    x[body_layout::kVel] += 5.0 * units::kKnot;
    const Controls u = (fps ? tr.fps : tr.sas).controls;
    return fly(
        ex, trim_environment(level(kt, kSas)), x,
        [&](double, const std::vector<double>&) { return u; }, seconds);
}

// The pedal held `offset` right of the trim from t = 0.
inline std::vector<Sample> pedal_push(const Executive& ex, const Trims& tr, double kt, bool fps,
                                      double offset, double seconds) {
    Controls u = (fps ? tr.fps : tr.sas).controls;
    u.pedal += offset;
    return fly(
        ex, trim_environment(level(kt, kSas)), fps ? tr.x_fps : tr.x_sas,
        [&](double, const std::vector<double>&) { return u; }, seconds);
}

// The collective raised `step` from t = 0.
inline std::vector<Sample> collective_step(const Executive& ex, const Trims& tr, double kt,
                                           bool fps, double step, double seconds) {
    Controls u = (fps ? tr.fps : tr.sas).controls;
    u.collective += step;
    return fly(
        ex, trim_environment(level(kt, kSas)), fps ? tr.x_fps : tr.x_sas,
        [&](double, const std::vector<double>&) { return u; }, seconds);
}

// A right turn from the FPS trim: the pilot holds the trim release for the
// first 8 s and banks with the lateral stick, a proportional-derivative law
// toward 20° within ±0.15 of the trim (the SAS's attitude paths hold it to
// about 10°), then lets go with the stick where it is; feet off the pedals.
// The turn switch is latched on from 2 s (as the host would with the trim
// release pressed, above 60 kt and 2° of bank) if `turn_switch`, else left
// off.
inline std::vector<Sample> turn(const Executive& ex, const Trims& tr, double kt, bool turn_switch,
                                double seconds) {
    double lat = tr.fps.controls.lat_cyclic;
    const auto pilot = [&](double t, const std::vector<double>& x) {
        Controls u = tr.fps.controls;
        if (t < 8.0) {
            const BodyState b = unpack_body(x);
            const double phi = euler_angles(b.q_nb).roll;
            const double target = 20.0 * units::kDegree;
            lat = tr.fps.controls.lat_cyclic +
                  std::clamp(0.4 * (target - phi) - 0.3 * b.omega_b.x(), -0.15, 0.15);
            u.afcs.trim_release = true;
        }
        u.lat_cyclic = lat;
        u.afcs.fps_turn = turn_switch && t > 2.0;
        return u;
    };
    return fly(ex, trim_environment(level(kt, kSas)), tr.x_fps, pilot, seconds);
}

}  // namespace eyas::test::m8c3
