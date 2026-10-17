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

// A trim at a given roll angle with the sideslip free, as GenHel's trims are
// made above about 50 kt (Ballin, NASA TM-88360, Fig. B1; D-035 M7c
// amendment). `sideslip` is the solved β [rad]; `trim` is the solve_trim
// result at it, whose roll is the asked-for one to `roll_tol`.
struct RollTrimResult {
    bool converged = false;  // the inner trim converged and |φ − roll| ≤ roll_tol
    int iterations = 0;      // secant iterations on β
    double sideslip = 0.0;   // β [rad]
    TrimResult trim;
};

// Trims `ex`'s aircraft for `c` with φ = `roll` [rad], solving for β by the
// secant method from c.sideslip (at most 20 iterations, steps of at most
// 10°; spec §7 rule 3). Each inner trim starts from the previous one, the
// first from `start` if given. `x` is as in solve_trim, at the final β.
// Throws InitError if the airspeed is zero or the flight vertical, where the
// sideslip is undefined, or on an invalid condition.
[[nodiscard]] RollTrimResult solve_trim_at_roll(const Executive& ex, const TrimCondition& c,
                                                double roll, const TrimResult* start,
                                                std::vector<double>& x,
                                                const TrimOptions& options = {},
                                                double roll_tol = 1e-8);

}  // namespace eyas
