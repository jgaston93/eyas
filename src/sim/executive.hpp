#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"

// Simulation executive (spec §6.10). Owns the global state vector, with the
// airframe body states first (body_layout), each component's states after
// them in config order, and each shaft's (ψ, Ω) last (D-052). Evaluates the components in an order
// fixed at init, producers of coupling outputs before their consumers (D-003). Solves the coupled
// body/component accelerations in every derivative evaluation (spec §5, "Coupling") and advances
// everything with RK4 in fixed integer-nanosecond steps (D-006).
namespace eyas {

// A free shaft's floor, a fraction of its governed speed (D-057): a bound on
// the models' domain, not aircraft data. The rotor models are written per
// unit ΩR and break down as Ω → 0 (issue #74), so a free shaft that reaches
// the floor and would slow further is held there.
inline constexpr double kShaftFloor = 0.05;

// How a shaft's speed is set (D-052).
enum class ShaftDrive : unsigned char {
    ideal,  // held at its governed speed whatever the load (Ω̇ ≡ 0): no equation
    free,   // its equation is solved with the airframe's (engines in M9 Stage B)
};

// A spinning part on a shaft that no component models (D-053 item 5): it
// turns at gear × Ω about axis_b relative to the airframe, so its angular
// momentum is (J/gear) Ω n with J its inertia referred to the shaft (its own
// inertia about the axis × gear²). Its rigid inertia is the airframe's (D-002).
struct ShaftPart {
    double inertia = 0.0;  // J [kg·m²], referred to the shaft
    double gear = 1.0;     // its speed over the shaft's (sign: sense about axis_b)
    // Spin axis, body axes: unit, or zero for inertia on the shaft with no
    // angular momentum relative to the airframe (gear trains whose momenta
    // cancel or are negligible: GenHel's J_REST beyond its gyroscopic list,
    // D-053 amendment 4).
    Vec3 axis_b = Vec3(0.0, 0.0, -1.0);
};

// A rotor-speed degree of freedom (D-052): the rotating parts geared together,
// each component on it turning at gear × Ω about its own axis.
struct ShaftConfig {
    std::string name;
    double speed = 0.0;            // governed speed [rad/s], the initial Ω
    std::vector<ShaftPart> parts;  // the parts no component models (D-053)
    ShaftDrive drive = ShaftDrive::ideal;
};

struct ExecutiveConfig {
    MassProperties mass;               // airframe only, about its CG (D-002)
    double origin_altitude = 0.0;      // [m] MSL height of the NED origin (D-016, D-017)
    std::int64_t step_ns = 1'000'000;  // [ns] physics step (1 kHz)
    std::int64_t max_substeps = 100;   // per advance() call
    std::vector<ShaftConfig> shafts;   // at most kMaxShafts (D-052)
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

    // The body acceleration a = [v̇_b; ω̇_b] [m/s², rad/s²] (body axes) that
    // derivatives() solves for at the current state and sim time, for the
    // outputs (D-043). Afterwards the coupling store holds this evaluation's
    // outputs, so component queries through context(sim_time(), state(), ...)
    // read consistent coupled inputs. Changes no state. Returns false if
    // some component's bounded iteration did not converge (D-023).
    [[nodiscard]] bool body_acceleration(const Controls& controls, const Environment& env,
                                         Vec6& a) const noexcept;
    // As body_acceleration(), and the components' readouts in the same
    // evaluation (D-059): `readouts` is reset, then each role's component
    // writes its fields (the first of each role in config order, every
    // landing gear for weight on wheels); a role no component fills keeps
    // NaN (false).
    [[nodiscard]] bool body_acceleration(const Controls& controls, const Environment& env, Vec6& a,
                                         Readouts& readouts) const noexcept;

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
    // The shafts (D-052): their configs, and shaft k's ψ at state index
    // shaft_offset(k), its Ω at shaft_offset(k) + 1.
    std::span<const ShaftConfig> shafts() const noexcept { return shafts_; }
    std::size_t shaft_offset(std::size_t k) const noexcept { return shaft_offset_ + 2 * k; }
    // The components' states only: those between the body's and the shafts'.
    std::size_t num_component_states() const noexcept { return shaft_offset_ - body_layout::kSize; }
    // Writes every shaft's start state, ψ = 0 at its governed speed, into a
    // full state vector x (D-052).
    void initial_shaft_states(std::span<double> x) const noexcept;
    // Wraps every shaft's ψ and every component state labelled phase in x
    // into [0, 2π) (D-052, D-053 amendment 3): a phase, kept small so
    // its rounding doesn't grow with the run, as the quaternion is kept unit.
    // Called after each step; changes nothing a component sees, which reads
    // the azimuth modulo 2π.
    void wrap_shaft_azimuths(std::span<double> x) const noexcept;
    // Every state's label for trim (D-045): the body's free, each
    // component's from its steady_kinds(), checked against its
    // steady_states() at construction.
    std::span<const SteadyKind> steady_kinds() const noexcept { return steady_kinds_; }
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
    // Fills steady_kinds_ and checks the steady_states() contract (D-045);
    // throws InitError naming the component and state on a mismatch.
    void label_steady_states();

    // The coupled system (spec §5, D-052): unknowns z = [a; Ω̇ of the free
    // shafts], A z = rhs, sized 6 + free shafts (fixed capacity, no heap).
    static constexpr int kMaxUnknowns = 6 + kMaxShafts;
    using MatZ =
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, kMaxUnknowns, kMaxUnknowns>;
    using VecZ = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxUnknowns, 1>;

    // Shared by derivatives() and held_derivatives(): evaluates the force
    // models and the components at (t, x), writing xdot0_, G_ and Gs_, adding
    // the loads into the body rows of `rhs` and their H into `A`, the free
    // shafts' rows after them, and each component's load0 into
    // `component_loads` if it isn't empty. Returns false if some bounded
    // iteration did not converge.
    // The output evaluation at the current state (D-043, D-059), readouts
    // written when non-null.
    bool output_evaluation(const Controls& controls, const Environment& env, Vec6& a,
                           Readouts* readouts) const noexcept;
    bool evaluate(double t, std::span<const double> x, const Controls& controls,
                  const Environment& env, MatZ& A, VecZ& rhs, std::span<Wrench> component_loads,
                  Readouts* readouts = nullptr) const noexcept;
    // Solves A z = rhs at state x: with no free shaft through the 6 × 6 path
    // exactly as before D-052 (bit for bit), else through the generalized
    // one, with the shafts' floors (D-057). Returns false if which shafts
    // are held didn't settle (D-023; z is the last solve's).
    bool solve(std::span<const double> x, const MatZ& A, const VecZ& rhs, VecZ& z) const noexcept;
    // Solves A z = rhs, the free shafts' rows from `first` on, holding at
    // Ω̇ = 0 the shafts on their floors that would slow down (D-057).
    // Returns false if that didn't settle.
    bool solve_with_floors(std::span<const double> x, const MatZ& A, const VecZ& rhs,
                           Eigen::Index first, VecZ& z) const noexcept;
    // Whether shaft k is at or below its floor in state x (D-057).
    bool on_floor(std::span<const double> x, std::size_t k) const noexcept {
        return x[shaft_offset(k) + 1] <= shaft_floor_[k];
    }
    // Writes ẋ from the solution z: body kinematics, components, shafts.
    void back_substitute(std::span<const double> x, const VecZ& z,
                         std::span<double> xdot) const noexcept;

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
    std::vector<std::size_t> offsets_;      // each component's first state index
    std::vector<std::size_t> sizes_;        // each component's state count, fixed at init
    std::vector<SteadyKind> steady_kinds_;  // one per state (D-045)
    std::vector<std::size_t> phases_;       // the states labelled phase
    std::vector<std::size_t> order_;        // evaluation order, producers first (D-003)
    // Whether each component (config order) reports its readout role's
    // fields (D-059): the first of each role, and every landing gear.
    std::vector<unsigned char> reports_;
    std::vector<ShaftConfig> shafts_;  // D-052
    std::size_t shaft_offset_ = 0;     // first shaft state index
    std::vector<int> free_shafts_;     // the free shafts' indices, in order
    // Each shaft's floor [rad/s] (D-057).
    std::array<double, kMaxShafts> shaft_floor_{};

    std::vector<double> x_;
    std::vector<double> x_prev_;  // state before the current step
    Rk4 rk4_;

    // Per-evaluation scratch (sized at init, contents never carried over).
    mutable std::vector<double> xdot0_;  // all components' xdot0, in state order
    mutable std::vector<double> G_;      // all components' G, n × 6 row-major
    mutable std::vector<double> Gs_;     // all components' G_s, n × kMaxShafts (D-052)
    mutable CouplingOutputs coupling_;   // rebuilt in every evaluation (D-003)

    std::int64_t n_steps_ = 0;
    std::int64_t remainder_ns_ = 0;
    double quat_norm_error_ = 0.0;
    std::int64_t unconverged_evaluations_ = 0;
};

}  // namespace eyas
