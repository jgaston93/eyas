#pragma once

// The D-042 long runs, shared by the M8a unit gates (60 s) and the
// VALIDATION.md generator (tests/validation/m8a_cases.cpp, 30 min):
// (a) the free UH-60A with its whole AFCS on (M8c3), (b) the UH-60A's held
// periodic orbit, (c) the M2 dummy-rotor equilibrium and (d) the free UH-60A
// parked on its gear.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "eyas/model.hpp"
#include "support/m2_fixtures.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/units.hpp"

namespace eyas::test {

// (b) The held periodic orbit of a level trim at `kt` (ISA sea level): the
// body held at its trim state while the rotor, inflow and other component
// states are integrated by RK4 at T/steps for `revolutions` revolutions,
// from the trim's start state. deviation[n] is the largest change of a
// component state (each in its own units) between the start of revolution
// 0 and of revolution n + 1: the orbit's drift.
struct HeldOrbitDrift {
    bool converged = false;
    double period = 0.0;             // T [s]
    double periodic_residual = 0.0;  // the trim's (TrimResult)
    std::vector<double> deviation;   // one per revolution
    std::size_t unconverged = 0;     // evaluations with an unconverged iteration (D-023)
};

// The UH-60A's rotor period T [s] (trim_period), without trimming.
inline double uh60a_period() {
    return trim_period(Uh60aHarness().executive());
}

inline HeldOrbitDrift held_orbit_drift(double kt, int revolutions, int steps = 256) {
    const Uh60aHarness ac;
    const Executive& ex = ac.executive();
    TrimCondition c;
    c.airspeed = kt * units::kKnot;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    HeldOrbitDrift d;
    d.converged = r.converged;
    d.periodic_residual = r.periodic_residual;
    d.period = trim_period(ex);
    if (!r.converged) {
        return d;
    }
    // The trim's own orbit: the body at its mean trim state, not the state
    // Model::trim installs (which adds the 4/rev vibration to v and ω).
    x = held_orbit_start(ex, c, r);
    HeldIntegrator held(ex, trim_environment(c), d.period, steps);
    const std::vector<double> x0 = x;
    Vec6 integral = Vec6::Zero();
    for (int n = 0; n < revolutions; ++n) {
        for (int s = 0; s < steps; ++s) {
            held.step(s, r.controls, x, integral);
        }
        double dev = 0.0;
        // The component states; a shaft's ψ advances 2π a revolution, a
        // phase, not a state that repeats (D-052).
        for (std::size_t i = body_layout::kSize; i < body_layout::kSize + ex.num_component_states();
             ++i) {
            dev = std::max(dev, std::abs(x[i] - x0[i]));
        }
        d.deviation.push_back(dev);
    }
    d.unconverged = held.unconverged();
    return d;
}

// (a) The free UH-60A with its whole AFCS on (SAS, PBA and FPS, the turn
// switch off; M8c3), trimmed level at `kt` (ISA sea level) by Model::trim and
// flown hands-off at 60 Hz for `seconds`, the trim's controls held. One
// sample per second of the outputs, as the parked run's (below); `speed` is
// the true airspeed. Empty if the trim fails.
struct AfcsSample {
    std::int64_t t_ns = 0;         // [ns] sim time, exact
    double t = 0.0;                // [s]
    Vec3 pos_n = Vec3::Zero();     // [m]
    double roll = 0.0;             // [rad]
    double pitch = 0.0;            // [rad]
    double heading = 0.0;          // [rad]
    double speed = 0.0;            // [m/s]
    double rate = 0.0;             // |ω_b| [rad/s]
    double quat_norm_error = 0.0;  // max over the second's frames [-]
    bool finite = true;            // every state finite, no frame flagged non_finite
};

inline std::vector<AfcsSample> afcs_free_flight(double kt, int seconds) {
    Model m = Model::from_file(uh60a_config_path());
    TrimCondition c;
    c.airspeed = kt * units::kKnot;
    c.afcs = {true, true, true, true, false, false};
    const TrimResult r = m.trim(c);
    std::vector<AfcsSample> out;
    if (!r.converged) {
        return out;
    }
    const Environment env{c.isa_delta_T, c.wind_n};  // no ground, as trimmed
    for (int sec = 1; sec <= seconds; ++sec) {
        AfcsSample p;
        for (int f = 0; f < 60; ++f) {
            const AdvanceStatus st =
                m.advance(r.controls, env, f % 3 == 2 ? 16'666'666 : 16'666'667);
            p.finite = p.finite && !st.non_finite;
            p.quat_norm_error = std::max(p.quat_norm_error, m.outputs().quat_norm_error);
        }
        const AircraftState& o = m.outputs();
        p.t_ns = o.sim_time_ns;
        p.t = o.sim_time;
        p.pos_n = o.pos_n;
        p.roll = o.roll;
        p.pitch = o.pitch;
        p.heading = o.heading;
        p.speed = o.true_airspeed;
        p.rate = o.omega_b.norm();
        const auto x = m.state();
        p.finite =
            p.finite && std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); });
        out.push_back(p);
    }
    return out;
}

// (d) The free UH-60A dropped onto level ground from 2.2 m (first contact at
// about 0.27 s), brakes set, collective 0.2, pedal 0.5, flown at 60 Hz for
// `seconds`. One sample per second of the outputs.
struct ParkedSample {
    std::int64_t t_ns = 0;         // [ns] sim time, exact
    double t = 0.0;                // [s]
    Vec3 pos_n = Vec3::Zero();     // [m]
    double roll = 0.0;             // [rad]
    double pitch = 0.0;            // [rad]
    double heading = 0.0;          // [rad]
    double speed = 0.0;            // |v_b| [m/s]
    double rate = 0.0;             // |ω_b| [rad/s]
    double quat_norm_error = 0.0;  // max over the second's frames [-]
    bool weight_on_wheels = false;
    bool finite = true;  // every state finite, no frame flagged non_finite
};

inline std::vector<ParkedSample> parked_run(int seconds) {
    Model m = Model::from_file(uh60a_config_path());
    Controls c;
    c.collective = 0.2;
    c.pedal = 0.5;
    c.brake_left = 1.0;
    c.brake_right = 1.0;
    Environment env;
    env.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);
    std::vector<ParkedSample> out;
    if (!m.reset(s)) {
        return out;
    }
    // 60 Hz frames: 16 666 667 ns rounds the 1/60 s period; the remainder
    // carry keeps sim time exact (D-006).
    for (int sec = 1; sec <= seconds; ++sec) {
        ParkedSample p;
        for (int f = 0; f < 60; ++f) {
            const AdvanceStatus st = m.advance(c, env, f % 3 == 2 ? 16'666'666 : 16'666'667);
            p.finite = p.finite && !st.non_finite;
            p.quat_norm_error = std::max(p.quat_norm_error, m.outputs().quat_norm_error);
        }
        const AircraftState& o = m.outputs();
        p.t_ns = o.sim_time_ns;
        p.t = o.sim_time;
        p.pos_n = o.pos_n;
        p.roll = o.roll;
        p.pitch = o.pitch;
        p.heading = o.heading;
        p.speed = o.vel_b.norm();
        p.rate = o.omega_b.norm();
        p.weight_on_wheels = o.weight_on_wheels;
        const auto x = m.state();
        p.finite =
            p.finite && std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); });
        out.push_back(p);
    }
    return out;
}

// (c) The M2 dummy-rotor aircraft at rest, level, thrust exactly equal to
// weight (Model.LevelEquilibriumIsExactFor60s), flown at 60 Hz for
// `seconds`. Returns the largest |state change| seen at the per-second
// samples (0 if exact) and whether sim time stayed exact.
struct DummyEquilibrium {
    double max_change = 0.0;
    bool time_exact = true;
};

inline DummyEquilibrium dummy_equilibrium(int seconds) {
    Model m = Model::from_string(m2_yaml());
    Controls c;
    c.collective = 0.5;
    const std::vector<double> x0(m.state().begin(), m.state().end());
    DummyEquilibrium d;
    for (int sec = 1; sec <= seconds; ++sec) {
        for (int f = 0; f < 60; ++f) {
            (void)m.advance(c, Environment{}, f % 3 == 2 ? 16'666'666 : 16'666'667);
        }
        const auto x = m.state();
        for (std::size_t i = 0; i < x.size(); ++i) {
            d.max_change = std::max(d.max_change, std::abs(x[i] - x0[i]));
        }
        d.time_exact = d.time_exact && m.outputs().sim_time_ns == std::int64_t{sec} * 1'000'000'000;
    }
    return d;
}

}  // namespace eyas::test
