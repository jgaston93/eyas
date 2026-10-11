#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"

// Simulation executive (spec §6.10). Owns the global state vector, with the
// airframe body states first (body_layout) and each component's states after
// them in config order. Solves the coupled body/component accelerations in
// every derivative evaluation (spec §5, "Coupling") and advances everything
// with RK4 in fixed integer-nanosecond steps (D-006).
namespace eyas {

struct ExecutiveConfig {
    MassProperties mass;               // airframe only, about its CG (D-002)
    double origin_altitude = 0.0;      // [m] MSL height of the NED origin (D-016, D-017)
    std::int64_t step_ns = 1'000'000;  // [ns] physics step (1 kHz)
    std::int64_t max_substeps = 100;   // per advance() call
};

class Executive {
public:
    // Throws InitError on an invalid configuration. Allocates every buffer
    // used by advance().
    explicit Executive(const ExecutiveConfig& config,
                       std::vector<std::unique_ptr<DynamicComponent>> components = {},
                       std::vector<std::unique_ptr<ForceModel>> forces = {});

    // Init-time: sets the airframe state (the quaternion is normalized).
    void set_body_state(const BodyState& body);

    // Starts the run again from `body` at t = 0: components return to their
    // initial states, the step count and remainder are zeroed. Returns false,
    // changing nothing, if the state is not finite or the quaternion is zero.
    bool reset(const BodyState& body) noexcept;

    // Sets the full state, step count and remainder, e.g. from a snapshot.
    // Returns false, changing nothing, on a size mismatch, invalid time, a
    // non-finite state or a zero quaternion.
    bool restore(std::span<const double> x, std::int64_t n_steps,
                 std::int64_t remainder_ns) noexcept;

    // Advances by one host frame: runs floor((remainder + dt) / step) fixed
    // sub-steps, at most max_substeps, and carries the remainder. [ns]
    AdvanceStatus advance(const Controls& controls, const Environment& env,
                          std::int64_t dt_host_ns) noexcept;

    // The full derivative function ẋ = f(t, x, controls, env). Pure: the
    // result depends only on its arguments (spec §5). Writes into internal
    // scratch buffers, which never affect results. [s]
    void derivatives(double t, std::span<const double> x, const Controls& controls,
                     const Environment& env, std::span<double> xdot) const noexcept;

    std::size_t num_states() const noexcept { return x_.size(); }
    std::span<const double> state() const noexcept { return x_; }
    BodyState body_state() const noexcept { return unpack_body(x_); }

    std::int64_t n_steps() const noexcept { return n_steps_; }
    std::int64_t step_ns() const noexcept { return step_ns_; }
    std::int64_t sim_time_ns() const noexcept { return n_steps_ * step_ns_; }  // [ns]
    double sim_time() const noexcept;                                          // [s]
    std::int64_t remainder_ns() const noexcept { return remainder_ns_; }

    // Largest |‖q‖ − 1| before renormalization over the last advance() (spec §6.1).
    double quat_norm_error() const noexcept { return quat_norm_error_; }

private:
    // One RK4 step. Returns false, leaving the state unchanged, if the result
    // is not finite.
    bool step(const Controls& controls, const Environment& env) noexcept;

    MassProperties mass_;
    Mat66 mass_matrix_;
    double origin_altitude_;
    std::int64_t step_ns_;
    std::int64_t max_substeps_;

    std::vector<std::unique_ptr<DynamicComponent>> components_;
    std::vector<std::unique_ptr<ForceModel>> forces_;
    std::vector<std::size_t> offsets_;  // each component's first state index
    std::vector<std::size_t> sizes_;    // each component's state count, fixed at init

    std::vector<double> x_;
    std::vector<double> x_prev_;  // state before the current step
    Rk4 rk4_;

    // Per-evaluation scratch (sized at init, contents never carried over).
    mutable std::vector<double> xdot0_;  // all components' xdot0, in state order
    mutable std::vector<double> G_;      // all components' G, n × 6 row-major
    CouplingOutputs coupling_;

    std::int64_t n_steps_ = 0;
    std::int64_t remainder_ns_ = 0;
    double quat_norm_error_ = 0.0;
};

}  // namespace eyas
