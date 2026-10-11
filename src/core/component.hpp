#pragma once

#include <cstddef>
#include <span>

#include "core/types.hpp"
#include "env/atmosphere.hpp"

// Interfaces between the executive and the models (spec §6, with D-003 and
// D-004). Everything here is called from advance(): no allocation, no
// exceptions, no I/O (spec §7).
namespace eyas {

// Per-evaluation results shared between components (D-003), grouped by
// producer instance. Fields are added only when a consumer needs them; there
// are none yet in M1.
struct CouplingOutputs {};

// Read-only inputs to one derivative evaluation.
struct SimContext {
    double t = 0.0;  // [s] sim time of this evaluation (RK4 stage time)
    BodyState body;  // q_nb is unit length, even inside RK4 stages (D-014)
    Controls controls;
    Environment env;
    AirState air;  // atmosphere at the CG altitude, from env (D-016)
    const CouplingOutputs* coupling = nullptr;
};

// One component's results for one derivative evaluation, split into the value
// at zero body acceleration plus the sensitivity to the body acceleration
// a = [v̇_b; ω̇_b] (spec §5, "Coupling"):
//   ẋ_c    = xdot0 + G a
//   load_c = load0_b + H a
// The executive zeroes everything before each call, so a component sets only
// what it uses. load0_b includes gravity on the component's own mass, since
// the airframe mass excludes it (D-002).
struct ComponentDerivs {
    std::span<double> xdot0;  // n_c
    std::span<double> G;      // n_c × 6, row-major
    Wrench load0_b;           // body axes, about the airframe CG
    Mat66 H;                  // ∂[F_b; M_b]/∂a
};

// A model with states: blades, inflow, gear, ... (spec §6; D-004).
class DynamicComponent {
public:
    DynamicComponent() = default;
    DynamicComponent(const DynamicComponent&) = delete;
    DynamicComponent& operator=(const DynamicComponent&) = delete;
    virtual ~DynamicComponent() = default;

    virtual std::size_t num_states() const = 0;
    virtual void initial_states(std::span<double> x) const = 0;

    // Pure function of (ctx, x). A cached table-search hint is the only hidden
    // state allowed, because it can change the search time but not the result.
    virtual void derivatives(const SimContext& ctx, std::span<const double> x,
                             ComponentDerivs& out) const = 0;
};

// A stateless model whose load doesn't depend on body acceleration
// (const per D-004).
class ForceModel {
public:
    ForceModel() = default;
    ForceModel(const ForceModel&) = delete;
    ForceModel& operator=(const ForceModel&) = delete;
    virtual ~ForceModel() = default;

    // Body axes, about the airframe CG.
    virtual Wrench compute(const SimContext& ctx) const = 0;
};

}  // namespace eyas
