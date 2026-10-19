#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/coupling.hpp"
#include "core/types.hpp"
#include "env/atmosphere.hpp"

// Interfaces between the executive and the models (spec §6, with D-003 and
// D-004). Everything here is called from advance(): no allocation, no
// exceptions, no I/O (spec §7).
namespace eyas {

// Read-only inputs to one derivative evaluation.
struct SimContext {
    double t = 0.0;  // [s] sim time of this evaluation (RK4 stage time)
    BodyState body;  // q_nb is unit length, even inside RK4 stages (D-014)
    Controls controls;
    Environment env;
    AirState air;  // atmosphere at the CG altitude, from env (D-016)
    // NED z of the ground plane where it crosses the origin's vertical [m]:
    // origin altitude − env.ground_height (D-032 item 9, D-033). NaN: no ground.
    double ground_z_n = std::numeric_limits<double>::quiet_NaN();
    // Outputs of the components evaluated before this one (D-003); read
    // only the slots returned by bind().
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
    // False if a bounded iteration in this evaluation hit its limit (spec §7
    // rule 3, D-023). The results are still used; the executive counts it.
    bool converged = true;
    // Where a producer writes its coupling outputs (D-003); null when the
    // component is evaluated on its own, e.g. in a unit test.
    CouplingOutputs* coupling = nullptr;
};

// How trim treats one state of a component (D-045, M8c3 amendment): its
// label from DynamicComponent::steady_kinds().
enum class SteadyKind : unsigned char {
    free,   // not set by steady_states(); trim solves it as an unknown
    orbit,  // set by steady_states(), then started on its free-flight orbit
    held,   // set by steady_states() and kept there (integrators, synchronizers)
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

    // Data path between components (D-003). Names are "<instance>.<field>".
    // The executive evaluates producers before consumers and calls bind()
    // once, at init, after every component has declared its names; a
    // component resolves its slots there (CouplingRegistry::find throws
    // InitError for a name nobody produces). The defaults declare nothing.
    virtual std::string_view name() const noexcept { return {}; }
    virtual std::vector<std::string> produces() const { return {}; }
    virtual std::vector<std::string> consumes() const { return {}; }
    virtual void bind(const CouplingRegistry& /*registry*/) {}

    // Pure function of (ctx, x). A cached table-search hint is the only hidden
    // state allowed, because it can change the search time but not the result.
    virtual void derivatives(const SimContext& ctx, std::span<const double> x,
                             ComponentDerivs& out) const = 0;

    // GenHel's "IC mode" (CR-166309 p. 5.5-2; D-045): sets the states whose
    // equilibrium follows algebraically from ctx with the body held (filters,
    // servos, synchronizers) to it, and leaves the others alone. Trim calls it
    // on every state it builds, so these states drop out of its unknowns. The
    // default sets nothing.
    // Contract: write exactly the states steady_kinds() labels orbit or held,
    // and don't depend on the values x arrives with (reading back what was
    // just written is fine). The executive checks the first part when it is
    // built (InitError).
    virtual void steady_states(const SimContext& /*ctx*/, std::span<double> /*x*/) const {}

    // Labels every state (kinds.size() == num_states()) for trim: free,
    // orbit or held (SteadyKind). Fixed for a configured component. The
    // default labels every state free, matching the default steady_states().
    virtual void steady_kinds(std::span<SteadyKind> kinds) const {
        std::fill(kinds.begin(), kinds.end(), SteadyKind::free);
    }
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
