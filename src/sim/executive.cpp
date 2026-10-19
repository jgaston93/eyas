#include "sim/executive.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include "util/error.hpp"

namespace eyas {
namespace {

void validate(const ExecutiveConfig& c) {
    if (c.step_ns <= 0) {
        throw InitError("executive: step_ns must be positive, got " + std::to_string(c.step_ns));
    }
    if (!std::isfinite(c.origin_altitude)) {
        throw InitError("executive: origin altitude must be finite");
    }
    if (c.max_substeps <= 0) {
        throw InitError("executive: max_substeps must be positive");
    }
    if (!(c.mass.mass > 0.0) || !std::isfinite(c.mass.mass)) {
        throw InitError("executive: airframe mass must be positive and finite");
    }
    const Mat3& I = c.mass.inertia_b;
    if (!I.allFinite() || I != I.transpose()) {
        throw InitError("executive: inertia tensor must be finite and symmetric");
    }
    if (I.llt().info() != Eigen::Success) {
        throw InitError("executive: inertia tensor must be positive definite");
    }
}

// Runs in the member initializers, so it checks for null before the body can.
std::size_t total_states(const std::vector<std::unique_ptr<DynamicComponent>>& components) {
    std::size_t n = body_layout::kSize;
    for (const auto& c : components) {
        if (!c) {
            throw InitError("executive: null component");
        }
        n += c->num_states();
    }
    return n;
}

std::string label(const DynamicComponent& c, std::size_t i) {
    const std::string_view name = c.name();
    return name.empty() ? "component #" + std::to_string(i) : std::string(name);
}

// Builds the coupling registry and returns the evaluation order: a
// topological sort of producer → consumer edges, ties broken by config order
// so the order is deterministic (D-003). Then binds every component.
std::vector<std::size_t> order_and_bind(
    const std::vector<std::unique_ptr<DynamicComponent>>& components, CouplingRegistry& registry) {
    const std::size_t n = components.size();
    std::vector<std::size_t> producer_of;  // per registry slot: its producer's index
    for (std::size_t i = 0; i < n; ++i) {
        for (const std::string& name : components[i]->produces()) {
            registry.add(name, label(*components[i], i));
            producer_of.push_back(i);
        }
    }
    // needs[c] = producers that c depends on.
    std::vector<std::vector<std::size_t>> needs(n);
    for (std::size_t i = 0; i < n; ++i) {
        for (const std::string& name : components[i]->consumes()) {
            const CouplingSlot slot = registry.find(name, label(*components[i], i));  // throws
            needs[i].push_back(producer_of[slot.index]);
        }
    }
    std::vector<std::size_t> order;
    std::vector<bool> done(n, false);
    while (order.size() < n) {
        bool progressed = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (done[i]) {
                continue;
            }
            const bool ready = std::all_of(needs[i].begin(), needs[i].end(),
                                           [&](std::size_t j) { return done[j]; });
            if (ready) {
                order.push_back(i);
                done[i] = true;
                progressed = true;
                break;  // restart from the first index: ties go by config order
            }
        }
        if (!progressed) {
            std::string cycle;
            for (std::size_t i = 0; i < n; ++i) {
                if (!done[i]) {
                    cycle += (cycle.empty() ? "" : ", ") + label(*components[i], i);
                }
            }
            throw InitError("executive: coupling outputs form a dependency cycle; unresolved: " +
                            cycle);
        }
    }
    for (const auto& c : components) {
        c->bind(registry);
    }
    return order;
}

}  // namespace

Executive::Executive(const ExecutiveConfig& config,
                     std::vector<std::unique_ptr<DynamicComponent>> components,
                     std::vector<std::unique_ptr<ForceModel>> forces)
    : mass_(config.mass),
      mass_matrix_(rigid_body_mass_matrix(config.mass)),
      origin_altitude_(config.origin_altitude),
      step_ns_(config.step_ns),
      max_substeps_(config.max_substeps),
      components_(std::move(components)),
      forces_(std::move(forces)),
      x_(total_states(components_), 0.0),
      x_prev_(x_.size(), 0.0),
      rk4_(x_.size()),
      xdot0_(x_.size() - body_layout::kSize, 0.0),
      G_((x_.size() - body_layout::kSize) * 6, 0.0) {
    validate(config);
    std::size_t offset = body_layout::kSize;
    for (const auto& c : components_) {
        const std::size_t n = c->num_states();
        offsets_.push_back(offset);
        sizes_.push_back(n);
        c->initial_states(std::span<double>(x_).subspan(offset, n));
        offset += n;
    }
    for (const auto& f : forces_) {
        if (!f) {
            throw InitError("executive: null force model");
        }
    }
    CouplingRegistry registry;
    order_ = order_and_bind(components_, registry);
    coupling_ = CouplingOutputs(registry.size());
    pack_body(BodyState{}, x_);
    label_steady_states();
}

// A slot steady_states() writes holds the same value whatever x arrives
// with (its contract); one it leaves keeps the value it arrived with. So two
// probes filled differently tell them apart, a written NaN included.
void Executive::label_steady_states() {
    steady_kinds_.assign(x_.size(), SteadyKind::free);
    const SimContext ctx = context(0.0, x_, Controls{}, Environment{});
    for (std::size_t i = 0; i < components_.size(); ++i) {
        const DynamicComponent& c = *components_[i];
        const std::span<SteadyKind> kinds =
            std::span(steady_kinds_).subspan(offsets_[i], sizes_[i]);
        c.steady_kinds(kinds);
        std::vector<double> a(sizes_[i], 0.0);
        std::vector<double> b(sizes_[i], 1.0);
        c.steady_states(ctx, a);
        c.steady_states(ctx, b);
        for (std::size_t j = 0; j < sizes_[i]; ++j) {
            const bool written =
                std::bit_cast<std::uint64_t>(a[j]) == std::bit_cast<std::uint64_t>(b[j]);
            if (written == (kinds[j] == SteadyKind::free)) {
                throw InitError("executive: component '" + std::string(c.name()) + "' state " +
                                std::to_string(j) +
                                (written ? ": steady_states() sets it but steady_kinds() labels it "
                                           "free (D-045)"
                                         : ": steady_kinds() labels it orbit or held but "
                                           "steady_states() doesn't set it (D-045)"));
            }
        }
    }
}

void Executive::set_body_state(const BodyState& body) {
    BodyState b = body;
    if (!(b.q_nb.norm() > 0.0)) {
        throw InitError("executive: attitude quaternion is zero");
    }
    b.q_nb.normalize();
    pack_body(b, x_);
}

bool Executive::reset(const BodyState& body) noexcept {
    const bool finite = body.pos_n.allFinite() && body.vel_b.allFinite() &&
                        body.q_nb.coeffs().allFinite() && body.omega_b.allFinite();
    if (!finite || !(body.q_nb.norm() > 0.0)) {
        return false;
    }
    BodyState b = body;
    b.q_nb.normalize();
    for (std::size_t i = 0; i < components_.size(); ++i) {
        components_[i]->initial_states(std::span<double>(x_).subspan(offsets_[i], sizes_[i]));
    }
    pack_body(b, x_);
    n_steps_ = 0;
    remainder_ns_ = 0;
    quat_norm_error_ = 0.0;
    unconverged_evaluations_ = 0;
    return true;
}

bool Executive::restore(std::span<const double> x, std::int64_t n_steps,
                        std::int64_t remainder_ns) noexcept {
    if (x.size() != x_.size() || n_steps < 0 || remainder_ns < 0 || remainder_ns >= step_ns_) {
        return false;
    }
    // Same checks as reset(). No renormalization: a snapshot's quaternion is
    // already unit, and replay after restore must be bit-identical.
    const bool finite = std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); });
    if (!finite || !(unpack_body(x).q_nb.norm() > 0.0)) {
        return false;
    }
    std::copy(x.begin(), x.end(), x_.begin());
    n_steps_ = n_steps;
    remainder_ns_ = remainder_ns;
    quat_norm_error_ = 0.0;
    unconverged_evaluations_ = 0;
    return true;
}

double Executive::sim_time() const noexcept {
    return static_cast<double>(sim_time_ns()) * 1e-9;
}

SimContext Executive::context(double t, std::span<const double> x, const Controls& controls,
                              const Environment& env) const noexcept {
    // Inside RK4 stages the stored quaternion is off unit length. Components
    // get the unit one, since they use it to rotate vectors; q̇ uses the
    // stored one.
    SimContext ctx;
    ctx.t = t;
    ctx.body = unpack_body(x);
    ctx.body.q_nb.normalize();
    ctx.controls = controls;
    ctx.env = env;
    ctx.air = isa_atmosphere(origin_altitude_ - ctx.body.pos_n.z(), env.isa_delta_T);  // D-016
    ctx.ground_z_n = origin_altitude_ - env.ground_height;                             // D-033
    ctx.coupling = &coupling_;
    return ctx;
}

bool Executive::evaluate(double t, std::span<const double> x, const Controls& controls,
                         const Environment& env, Mat66& A, Vec6& rhs,
                         std::span<Wrench> component_loads) const noexcept {
    SimContext ctx = context(t, x, controls, env);
    // Force models run before every component and can't consume (D-003), so
    // they get no store; components get it once it is reset for this evaluation.
    ctx.coupling = nullptr;

    // (M_rb − Σ H) a = bias + Σ load0 + Σ forces   (spec §5, "Coupling")
    A = mass_matrix_;
    rhs = rigid_body_bias(mass_, ctx.body);
    for (const auto& f : forces_) {
        const Wrench w = f->compute(ctx);
        rhs.head<3>() += w.F;
        rhs.tail<3>() += w.M;
    }
    std::fill(xdot0_.begin(), xdot0_.end(), 0.0);
    std::fill(G_.begin(), G_.end(), 0.0);
    coupling_.begin_evaluation();
    ctx.coupling = &coupling_;
    bool converged = true;
    for (const std::size_t i : order_) {
        const std::size_t n = sizes_[i];
        const std::size_t local = offsets_[i] - body_layout::kSize;
        ComponentDerivs d{std::span<double>(xdot0_).subspan(local, n),
                          std::span<double>(G_).subspan(local * 6, n * 6),
                          Wrench{},
                          Mat66::Zero(),
                          true,
                          &coupling_};
        components_[i]->derivatives(ctx, x.subspan(offsets_[i], n), d);
        rhs.head<3>() += d.load0_b.F;
        rhs.tail<3>() += d.load0_b.M;
        A -= d.H;
        converged = converged && d.converged;
        if (component_loads.size() == components_.size()) {
            component_loads[i] = d.load0_b;
        }
    }
    return converged;
}

bool Executive::derivatives(double t, std::span<const double> x, const Controls& controls,
                            const Environment& env, std::span<double> xdot) const noexcept {
    Mat66 A;
    Vec6 rhs;
    const bool converged = evaluate(t, x, controls, env, A, rhs, {});
    const Vec6 a = A.partialPivLu().solve(rhs);

    // Back-substitution: ẋ_c = ẋ0_c + G_c a
    body_kinematics(unpack_body(x), a, xdot);
    for (std::size_t j = 0; j < xdot0_.size(); ++j) {
        double v = xdot0_[j];
        for (std::size_t k = 0; k < 6; ++k) {
            v += G_[j * 6 + k] * a(static_cast<Eigen::Index>(k));
        }
        xdot[body_layout::kSize + j] = v;
    }
    return converged;
}

bool Executive::body_acceleration(const Controls& controls, const Environment& env,
                                  Vec6& a) const noexcept {
    Mat66 A;
    Vec6 rhs;
    const bool converged = evaluate(sim_time(), x_, controls, env, A, rhs, {});
    a = A.partialPivLu().solve(rhs);
    return converged;
}

bool Executive::held_derivatives(double t, std::span<const double> x, const Controls& controls,
                                 const Environment& env, std::span<double> xdot, Vec6& net,
                                 std::span<Wrench> component_loads) const noexcept {
    Mat66 A;
    const bool converged = evaluate(t, x, controls, env, A, net, component_loads);
    std::copy(xdot0_.begin(), xdot0_.end(),
              xdot.begin() + static_cast<std::ptrdiff_t>(body_layout::kSize));
    return converged;
}

bool Executive::step(const Controls& controls, const Environment& env) noexcept {
    const double h = static_cast<double>(step_ns_) * 1e-9;
    const double t = sim_time();
    std::copy(x_.begin(), x_.end(), x_prev_.begin());
    rk4_.step(
        [&](double ts, std::span<const double> xs, std::span<double> xd) {
            if (!derivatives(ts, xs, controls, env, xd)) {
                ++unconverged_evaluations_;
            }
        },
        t, h, x_);

    // A singular coupled solve or a component returning NaN/Inf poisons the
    // whole state. Keep the last good state instead of committing the step.
    double* q = x_.data() + body_layout::kQuat;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const bool finite =
        std::all_of(x_.begin(), x_.end(), [](double v) { return std::isfinite(v); });
    if (!finite || !(norm > 0.0)) {
        std::copy(x_prev_.begin(), x_prev_.end(), x_.begin());
        return false;
    }
    ++n_steps_;

    // Renormalize; record the drift first (spec §6.1, M1).
    quat_norm_error_ = std::max(quat_norm_error_, std::abs(norm - 1.0));
    for (int i = 0; i < 4; ++i) {
        q[i] /= norm;
    }
    return true;
}

AdvanceStatus Executive::advance(const Controls& controls, const Environment& env,
                                 std::int64_t dt_host_ns) noexcept {
    AdvanceStatus status;
    if (dt_host_ns < 0) {
        status.rejected_dt = true;
        return status;
    }
    if (!isa_delta_T_valid(env.isa_delta_T) || !env.wind_n.allFinite()) {  // D-019
        status.rejected_env = true;
        return status;
    }
    // Split before adding so a huge dt can't overflow the remainder.
    std::int64_t steps = dt_host_ns / step_ns_;
    std::int64_t rem = remainder_ns_ + dt_host_ns % step_ns_;
    steps += rem / step_ns_;
    rem %= step_ns_;
    if (steps > max_substeps_) {
        steps = max_substeps_;
        status.capped = true;
    }
    remainder_ns_ = rem;

    quat_norm_error_ = 0.0;
    unconverged_evaluations_ = 0;
    for (std::int64_t i = 0; i < steps; ++i) {
        if (!step(controls, env)) {
            status.non_finite = true;
            break;
        }
        ++status.substeps;
        // From the committed state, so derivatives() stays pure (D-019).
        const double altitude = origin_altitude_ - x_[body_layout::kPos + 2];
        status.atmosphere_out_of_range =
            status.atmosphere_out_of_range || isa_out_of_range(altitude);
    }
    return status;
}

}  // namespace eyas
