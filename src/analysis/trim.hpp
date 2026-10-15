#pragma once

#include <vector>

#include "eyas/trim.hpp"
#include "sim/executive.hpp"

// Periodic trim (spec §6.8, D-035). The body is held at the flight condition
// for one rotor revolution while every other state (blades, inflow, lags,
// gear) is integrated through azimuth. The trim makes the net force and
// moment on the held body average to zero over the revolution, and every
// other state periodic. Init-time: allocates, not for a real-time loop.
namespace eyas {

struct TrimOptions {
    int steps_per_period = 256;  // RK4 steps per revolution
    int max_iterations = 30;     // Newton iterations (spec §7 rule 3)
    int settle_periods = 8;      // held revolutions before Newton, from a cold start
    // Convergence: mean force below force_tol × airframe weight, mean moment
    // below moment_tol × airframe weight × rotor radius, and every state
    // periodic to periodic_tol in its own units.
    double force_tol = 1e-7;
    double moment_tol = 1e-7;
    double periodic_tol = 1e-8;
};

// The trim period: one revolution of the aircraft's rotor, 2π/Ω [s]. Throws
// InitError if rotors turn at different speeds. Without a rotor the
// equations don't depend on time and any period will do; 0.1 s is used.
double trim_period(const Executive& ex);

// The body state of a trim: CG at the condition's altitude above the NED
// origin, attitude (heading, θ, φ), ω = 0, and the body velocity whose air
// velocity has the condition's speed, flight-path angle and sideslip. The
// track is solved for the sideslip; the root with the air velocity more
// along +x_b is taken. Returns false if no track gives that sideslip.
bool trim_body_state(const TrimCondition& c, double pitch, double roll, double origin_altitude,
                     BodyState& body);

// Trims `ex`'s aircraft for `c`, starting from `start` if given (a nearby
// trim) or from a cold start. `x` receives the full state at t = 0 (rotor
// azimuth zero), the start of the periodic orbit, whether or not the trim
// converged. Throws InitError on an invalid condition.
TrimResult solve_trim(const Executive& ex, const TrimCondition& c, const TrimResult* start,
                      std::vector<double>& x, const TrimOptions& options = {});

}  // namespace eyas
