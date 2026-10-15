#pragma once

#include <cstddef>
#include <vector>

#include "eyas/trim.hpp"
#include "sim/executive.hpp"

// The held-body orbit of a trim (D-035): the body held at its trim state
// while every component state (blades, inflow, lags, gear) is integrated
// through azimuth by RK4 at a step that divides the revolution. Shared by
// the trim solver and the linearization. Init-time: allocates.
namespace eyas {

class HeldIntegrator {
public:
    // `period` [s] split into `steps` RK4 steps.
    HeldIntegrator(const Executive& ex, Environment env, double period, int steps);

    // One held RK4 step from node s of the revolution (t = period·s/steps,
    // so every revolution sees the same azimuths): advances the component
    // part of x (num_states() long; the body part stays) and adds the step's
    // integral of the net wrench into `integral`, with RK4's weights, so it
    // is the RK4 solution of İ = net. [s]
    void step(int s, const Controls& u, std::vector<double>& x, Vec6& integral);

    double period() const noexcept { return period_; }
    int steps() const noexcept { return steps_; }
    // Evaluations in which some component's bounded iteration did not
    // converge (D-023), since construction or the last clear.
    std::size_t unconverged() const noexcept { return unconverged_; }
    void clear_unconverged() noexcept { unconverged_ = 0; }

private:
    const Executive& ex_;
    Environment env_;
    double period_;
    int steps_;
    std::size_t n_;
    std::vector<double> tmp_;
    std::vector<double> k1_;
    std::vector<double> k2_;
    std::vector<double> k3_;
    std::vector<double> k4_;
    std::size_t unconverged_ = 0;
};

// The environment a trim holds: the condition's ΔT and wind, no ground.
Environment trim_environment(const TrimCondition& c);

// The full state at t = 0 on the held orbit of trim `r` for `c`: the body
// at its mean trim state (trim_body_state(), ω = 0, without the vibration
// offset Model::trim adds) and the components at r.component_states.
// Throws InitError if the component states don't fit `ex` or the attitude
// admits no body state.
std::vector<double> held_orbit_start(const Executive& ex, const TrimCondition& c,
                                     const TrimResult& r);

// The held orbit sampled at `samples` evenly spaced times t_j = jT/samples,
// j = 0…samples−1, over one revolution T = trim_period(ex), integrated with
// `steps` RK4 steps per revolution (`samples` must divide `steps`, else
// InitError).
struct HeldOrbit {
    std::vector<std::vector<double>> states;  // x(t_j), num_states() long
    std::size_t unconverged = 0;              // evaluations with an unconverged iteration (D-023)
};
HeldOrbit held_orbit(const Executive& ex, const TrimCondition& c, const TrimResult& r, int samples,
                     int steps);

}  // namespace eyas
