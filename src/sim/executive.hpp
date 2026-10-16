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
// them in config order. Evaluates the components in an order fixed at init,
// producers of coupling outputs before their consumers (D-003). Solves the
// coupled body/component accelerations in every derivative evaluation (spec
// §5, "Coupling") and advances everything with RK4 in fixed
// integer-nanosecond steps (D-006).
namespace eyas {

struct ExecutiveConfig {
    MassProperties mass;               // airframe only, about its CG (D-002)
    double origin_altitude = 0.0;      // [m] MSL height of the NED origin (D-016, D-017)
    std::int64_t step_ns = 1'000'000;  // [ns] physics step (1 kHz)
    std::int64_t max_substeps = 100;   // per advance() call
};

class Executive {
public:
    // Throws InitError on an invalid configuration, including a coupling
    // output consumed but not produced, produced twice, or a dependency
    // cycle (D-003). Allocates every buffer used by advance().
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
    // scratch buffers, which never affect results. Returns false if some
    // component's bounded iteration did not converge (D-023). [s]
    [[nodiscard]] bool derivatives(double t, std::span<const double> x, const Controls& controls,
                                   const Environment& env, std::span<double> xdot) const noexcept;

    // The derivative function with the body held (D-035): a constraint keeps
    // the body acceleration at zero, so every component state moves with
    // ẋ_c = ẋ0_c. Writes the component part of xdot (its body part is left
    // as is) and returns in `net` the wrench the free body would feel at
    // a = 0: [m C_bn g − m ω × v; −ω × (I ω)] + Σ forces + Σ load0 (body axes,
    // about the airframe CG), i.e. minus the wrench that holds it. Zero
    // `net` with a = 0 is an equilibrium of the free body. If
    // `component_loads` has one entry per component (config order), each
    // component's load0 goes there; any other size is ignored. Pure, like derivatives(). Returns
    // false if some component's bounded iteration did not converge (D-023). [s]
    [[nodiscard]] bool held_derivatives(double t, std::span<const double> x,
                                        const Controls& controls, const Environment& env,
                                        std::span<double> xdot, Vec6& net,
                                        std::span<Wrench> component_loads = {}) const noexcept;

    // The context a derivative evaluation at (t, x) gives every component:
    // unit quaternion (D-014), atmosphere at the CG (D-016), ground plane
    // (D-033), and the coupling store (D-003). The store holds the outputs of
    // the latest evaluation, so a caller reading a component's coupled inputs
    // through it must evaluate at the same (t, x) first. [s]
    SimContext context(double t, std::span<const double> x, const Controls& controls,
                       const Environment& env) const noexcept;

    std::size_t num_states() const noexcept { return x_.size(); }
    std::size_t num_components() const noexcept { return components_.size(); }
    // Component i (config order), its first state index and its state count.
    const DynamicComponent& component(std::size_t i) const noexcept { return *components_[i]; }
    std::size_t component_offset(std::size_t i) const noexcept { return offsets_[i]; }
    std::size_t component_size(std::size_t i) const noexcept { return sizes_[i]; }
    const MassProperties& airframe_mass() const noexcept { return mass_; }
    double origin_altitude() const noexcept { return origin_altitude_; }
    std::span<const double> state() const noexcept { return x_; }
    BodyState body_state() const noexcept { return unpack_body(x_); }

    std::int64_t n_steps() const noexcept { return n_steps_; }
    std::int64_t step_ns() const noexcept { return step_ns_; }
    std::int64_t sim_time_ns() const noexcept { return n_steps_ * step_ns_; }  // [ns]
    double sim_time() const noexcept;                                          // [s]
    std::int64_t remainder_ns() const noexcept { return remainder_ns_; }

    // Largest |‖q‖ − 1| before renormalization over the last advance() (spec §6.1).
    double quat_norm_error() const noexcept { return quat_norm_error_; }

    // Derivative evaluations in the last advance() in which some component's
    // bounded iteration did not converge (D-023). Engineering diagnostic only.
    std::int64_t unconverged_evaluations() const noexcept { return unconverged_evaluations_; }

    // Component indices (config order) in evaluation order (D-003).
    std::span<const std::size_t> evaluation_order() const noexcept { return order_; }

private:
    // Shared by derivatives() and held_derivatives(): evaluates the force
    // models and the components at (t, x), writing xdot0_ and G_, adding the
    // loads into `rhs` and their H into `A`, and each component's load0 into
    // `component_loads` if it isn't empty. Returns false if some bounded
    // iteration did not converge.
    bool evaluate(double t, std::span<const double> x, const Controls& controls,
                  const Environment& env, Mat66& A, Vec6& rhs,
                  std::span<Wrench> component_loads) const noexcept;

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
    std::vector<std::size_t> order_;    // evaluation order, producers first (D-003)

    std::vector<double> x_;
    std::vector<double> x_prev_;  // state before the current step
    Rk4 rk4_;

    // Per-evaluation scratch (sized at init, contents never carried over).
    mutable std::vector<double> xdot0_;  // all components' xdot0, in state order
    mutable std::vector<double> G_;      // all components' G, n × 6 row-major
    mutable CouplingOutputs coupling_;   // rebuilt in every evaluation (D-003)

    std::int64_t n_steps_ = 0;
    std::int64_t remainder_ns_ = 0;
    double quat_norm_error_ = 0.0;
    std::int64_t unconverged_evaluations_ = 0;
};

}  // namespace eyas
