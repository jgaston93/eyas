#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/coupling.hpp"
#include "core/readouts.hpp"
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
    // The aircraft's shafts (D-052), in config order: shafts[0..num_shafts).
    std::array<ShaftState, kMaxShafts> shafts{};
    int num_shafts = 0;
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
    // The shaft parts of the affine form (D-052), with Ω̇ the shaft
    // accelerations [rad/s²] (entries past the aircraft's shaft count unused):
    //   ẋ_c    += G_s Ω̇                 (Gs: n_c × kMaxShafts, row-major; may
    //                                     be empty when the aircraft has no shaft)
    //   load_c += H_s Ω̇                 (on the airframe, body axes)
    //   Q_c     = Q0 + K_a a + K_s Ω̇     the torque the component applies to
    //                                     each shaft, referred to it [N·m]:
    //                                     gear ratio × its axial torque about
    //                                     its own axis, positive along rotation.
    // A component on no shaft leaves them zero.
    // The braces keep existing aggregate initializations free of
    // -Wmissing-field-initializers.
    std::span<double> Gs{};  // NOLINT(readability-redundant-member-init)
    Mat6S Hs = Mat6S::Zero();
    VecS Q0 = VecS::Zero();
    MatS6 Ka = MatS6::Zero();
    MatSS Ks = MatSS::Zero();
    // The output evaluation's readouts (D-059): non-null only for the
    // component the executive picked for its readout_role(), in the one
    // evaluation Model makes for the outputs; the component writes its
    // role's fields from this evaluation's results.
    Readouts* readouts = nullptr;
};

// What a component on a shaft is told at init (D-052).
struct ShaftInfo {
    std::string name;
    double speed = 0.0;  // governed speed [rad/s]
};

// How trim treats one state of a component (D-045, M8c3 amendment): its
// label from DynamicComponent::steady_kinds().
enum class SteadyKind : unsigned char {
    free,   // not set by steady_states(); trim solves it as an unknown
    orbit,  // set by steady_states(), then started on its free-flight orbit
    held,   // set by steady_states() and kept there (integrators, synchronizers)
    // A rotor's own azimuth, ψ̇ its speed (D-053 amendment 3): set by
    // steady_states() (zero at a trim's start, as the shafts' azimuths are)
    // and wrapped into [0, 2π) by the executive after every step. A geared
    // rotor needn't turn a whole number of times in a trim's revolution, so
    // trim leaves it out of its periodicity and drift.
    phase,
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

    // The outputs this component reports (D-059): its role's Readouts fields,
    // written in derivatives() when out.readouts is non-null. Fixed for a
    // configured component. The default reports nothing.
    virtual ReadoutRole readout_role() const noexcept { return ReadoutRole::none; }

    // The aircraft's shafts (D-052), in config order, at init after bind():
    // a component configured on a shaft finds its index here and checks its
    // speed against the shaft's (throw InitError on a mismatch or a missing
    // name). The default uses none.
    virtual void bind_shafts(std::span<const ShaftInfo> /*shafts*/) {}

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
    // and don't depend on the values those states arrive with (reading back
    // what was just written is fine); the free states may be read, as trim
    // solves for them (a limited integrator's limit follows the torque the
    // free states give, D-053). The executive checks the first part when it
    // is built (InitError).
    virtual void steady_states(const SimContext& /*ctx*/, std::span<double> /*x*/) const {}

    // Trim (D-053 amendments 2 and 5): a component whose states have an
    // algebraic equilibrium given what another component puts in the
    // coupling store (an engine at the torque its rotor demands) says so,
    // and trim_seed() sets them there. Trim starts from it, and in the first
    // stage of a cold start the states follow it on every evaluation, with
    // the free shafts at their governed speeds. `ctx.coupling` holds the
    // outputs of a derivative evaluation at the state being seeded. The
    // default seeds nothing.
    virtual bool has_trim_seed() const noexcept { return false; }
    virtual void trim_seed(const SimContext& /*ctx*/, std::span<double> /*x*/) const {}

    // Trim (D-061): the range of each state the model is valid in, on the
    // day in ctx.air (not depending on the state). Trim keeps its unknowns
    // inside it, as it keeps the controls inside their travel: past the end
    // of a model's data, where its tables hold their edge values, a state
    // moves nothing, and Newton's minimum-norm step could run off along it.
    // A condition that needs more doesn't converge and ends with the state on
    // its bound. lower.size() == upper.size() == num_states(), arriving at
    // −∞ and +∞; the default leaves them so.
    virtual void trim_bounds(const SimContext& /*ctx*/, std::span<double> /*lower*/,
                             std::span<double> /*upper*/) const {}

    // Marks (1) the states pinned at x on a limit or a switch, whose rates
    // aren't differentiable there: a limited integrator at the limit its
    // input pushes against, an engaged clutch's slip. Linearization holds
    // them (D-053 amendment 2): small perturbations leave them where they
    // are. pinned.size() == num_states(); the default marks none (0).
    virtual void pinned_states(const SimContext& /*ctx*/, std::span<const double> /*x*/,
                               std::span<unsigned char> pinned) const {
        std::fill(pinned.begin(), pinned.end(), static_cast<unsigned char>(0));
    }

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
