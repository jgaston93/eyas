#include "sim/executive.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <numbers>
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
    if (c.shafts.size() > static_cast<std::size_t>(kMaxShafts)) {
        throw InitError("executive: at most " + std::to_string(kMaxShafts) + " shafts (D-052)");
    }
    for (std::size_t k = 0; k < c.shafts.size(); ++k) {
        const ShaftConfig& sh = c.shafts[k];
        const std::string who = "executive: shaft '" + sh.name + "'";
        if (sh.name.empty()) {
            throw InitError("executive: a shaft needs a name (D-052)");
        }
        for (std::size_t j = 0; j < k; ++j) {
            if (c.shafts[j].name == sh.name) {
                throw InitError(who + " is declared twice");
            }
        }
        if (!(sh.speed > 0.0) || !std::isfinite(sh.speed)) {
            throw InitError(who + ": speed must be positive and finite");
        }
        for (const ShaftPart& p : sh.parts) {
            if (!(p.inertia >= 0.0) || !std::isfinite(p.inertia)) {
                throw InitError(who + ": a part's inertia must be non-negative and finite");
            }
            if (!(p.gear != 0.0) || !std::isfinite(p.gear)) {
                throw InitError(who + ": a part's gear must be nonzero and finite");
            }
            if (!p.axis_b.allFinite() ||
                (p.axis_b.norm() != 0.0 && std::abs(p.axis_b.norm() - 1.0) > 1e-9)) {
                throw InitError(who + ": a part's axis must be a unit vector, or zero");
            }
        }
    }
}

std::size_t component_states(const std::vector<std::unique_ptr<DynamicComponent>>& components) {
    std::size_t n = 0;
    for (const auto& c : components) {
        if (!c) {
            throw InitError("executive: null component");
        }
        n += c->num_states();
    }
    return n;
}

// Runs in the member initializers, so it checks for null before the body can.
std::size_t total_states(const std::vector<std::unique_ptr<DynamicComponent>>& components,
                         const ExecutiveConfig& config) {
    return body_layout::kSize + component_states(components) + 2 * config.shafts.size();
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
      shafts_(config.shafts),
      x_(total_states(components_, config), 0.0),
      x_prev_(x_.size(), 0.0),
      rk4_(x_.size()),
      xdot0_(component_states(components_), 0.0),
      G_(component_states(components_) * 6, 0.0),
      Gs_(config.shafts.empty() ? 0 : component_states(components_) * kMaxShafts, 0.0) {
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
    shaft_offset_ = offset;
    std::vector<ShaftInfo> info;
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        x_[shaft_offset(k)] = 0.0;
        x_[shaft_offset(k) + 1] = shafts_[k].speed;
        if (shafts_[k].drive == ShaftDrive::free) {
            free_shafts_.push_back(static_cast<int>(k));
        }
        info.push_back({shafts_[k].name, shafts_[k].speed});
    }
    CouplingRegistry registry;
    order_ = order_and_bind(components_, registry);
    coupling_ = CouplingOutputs(registry.size());
    for (const auto& c : components_) {
        c->bind_shafts(info);
    }
    pack_body(BodyState{}, x_);
    label_steady_states();
}

// A slot steady_states() writes holds the same value whatever that slot
// arrives with (its contract; it may read the free states, D-053); one it
// leaves keeps the value it arrived with. So probes that differ only in the
// labelled slots tell the written ones apart, a written NaN included, and
// free slots must come back as they went in.
void Executive::label_steady_states() {
    steady_kinds_.assign(x_.size(), SteadyKind::free);
    const SimContext ctx = context(0.0, x_, Controls{}, Environment{});
    for (std::size_t i = 0; i < components_.size(); ++i) {
        const DynamicComponent& c = *components_[i];
        const std::span<SteadyKind> kinds =
            std::span(steady_kinds_).subspan(offsets_[i], sizes_[i]);
        c.steady_kinds(kinds);
        std::vector<double> a(sizes_[i], 0.0);
        std::vector<double> b(sizes_[i], 0.0);
        std::vector<double> ones(sizes_[i], 1.0);
        for (std::size_t j = 0; j < sizes_[i]; ++j) {
            if (kinds[j] != SteadyKind::free) {
                b[j] = 1.0;
            }
        }
        c.steady_states(ctx, a);
        c.steady_states(ctx, b);
        c.steady_states(ctx, ones);
        const auto same = [](double u, double v) {
            return std::bit_cast<std::uint64_t>(u) == std::bit_cast<std::uint64_t>(v);
        };
        for (std::size_t j = 0; j < sizes_[i]; ++j) {
            const bool free = kinds[j] == SteadyKind::free;
            const bool written = free ? !(same(a[j], 0.0) && same(ones[j], 1.0)) : same(a[j], b[j]);
            if (written == free) {
                throw InitError("executive: component '" + std::string(c.name()) + "' state " +
                                std::to_string(j) +
                                (written ? ": steady_states() sets it but steady_kinds() labels it "
                                           "free (D-045)"
                                         : ": steady_kinds() labels it orbit, held or phase but "
                                           "steady_states() doesn't set it (D-045)"));
            }
        }
    }
    for (std::size_t i = 0; i < steady_kinds_.size(); ++i) {
        if (steady_kinds_[i] == SteadyKind::phase) {
            phases_.push_back(i);
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

void Executive::initial_shaft_states(std::span<double> x) const noexcept {
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        x[shaft_offset(k)] = 0.0;
        x[shaft_offset(k) + 1] = shafts_[k].speed;
    }
}

void Executive::wrap_shaft_azimuths(std::span<double> x) const noexcept {
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    const auto wrap = [&](double& psi) {
        if (psi >= kTwoPi || psi < 0.0) {
            psi -= kTwoPi * std::floor(psi / kTwoPi);
        }
    };
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        wrap(x[shaft_offset(k)]);
    }
    for (const std::size_t i : phases_) {
        wrap(x[i]);
    }
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
    initial_shaft_states(x_);
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
    ctx.num_shafts = static_cast<int>(shafts_.size());  // D-052
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        ctx.shafts[k] = {x[shaft_offset(k)], x[shaft_offset(k) + 1]};
    }
    return ctx;
}

bool Executive::evaluate(double t, std::span<const double> x, const Controls& controls,
                         const Environment& env, MatZ& A, VecZ& rhs,
                         std::span<Wrench> component_loads) const noexcept {
    SimContext ctx = context(t, x, controls, env);
    // Force models run before every component and can't consume (D-003), so
    // they get no store; components get it once it is reset for this evaluation.
    ctx.coupling = nullptr;

    // (M_rb − Σ H) a = bias + Σ load0 + Σ forces   (spec §5, "Coupling"),
    // with the free shafts' accelerations and equations added (D-052).
    const auto nf = static_cast<Eigen::Index>(free_shafts_.size());
    A.setZero(6 + nf, 6 + nf);
    rhs.setZero(6 + nf);
    A.topLeftCorner<6, 6>() = mass_matrix_;
    rhs.head<6>() = rigid_body_bias(mass_, ctx.body);
    // The shafts' parts spin relative to the airframe with angular momentum
    // (J/g) Ω n, which turns with it: the gyroscopic moment −ω × ((J/g) Ω n),
    // as GenHel's spinning parts give it (CR-166309 p. 5.10-19, J_XGY ω_XGY
    // and the like; D-048), here following the shaft's speed (D-052, D-053).
    // None without parts, so models without them are unchanged.
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        for (const ShaftPart& p : shafts_[k].parts) {
            rhs.segment<3>(3) -=
                ctx.body.omega_b.cross((p.inertia / p.gear * ctx.shafts[k].omega) * p.axis_b);
        }
    }
    for (const auto& f : forces_) {
        const Wrench w = f->compute(ctx);
        rhs.head<3>() += w.F;
        rhs.segment<3>(3) += w.M;
    }
    std::fill(xdot0_.begin(), xdot0_.end(), 0.0);
    std::fill(G_.begin(), G_.end(), 0.0);
    std::fill(Gs_.begin(), Gs_.end(), 0.0);
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
        if (!Gs_.empty()) {
            d.Gs = std::span<double>(Gs_).subspan(local * kMaxShafts, n * kMaxShafts);
        }
        components_[i]->derivatives(ctx, x.subspan(offsets_[i], n), d);
        rhs.head<3>() += d.load0_b.F;
        rhs.segment<3>(3) += d.load0_b.M;
        A.topLeftCorner<6, 6>() -= d.H;
        // Free shafts (D-052): the loads' Ω̇ terms in the body rows, and each
        // shaft's row  Σ Q_c − J(Ω̇ + n·ω̇) = 0, Q_c = Q0 + K_a a + K_s Ω̇:
        // GenHel's rotor-speed degree of freedom, Ω̇ = (Q_E − Q_REQ)/(J_TOTE
        // + J_REST) (CR-166309 p. 5.6-17), with the blades' inertia and every
        // torque's dependence on the accelerations kept in the coupled solve.
        for (Eigen::Index r = 0; r < nf; ++r) {
            const int k = free_shafts_[static_cast<std::size_t>(r)];
            A.block<6, 1>(0, 6 + r) -= d.Hs.col(k);
            A.block<1, 6>(6 + r, 0) -= d.Ka.row(k);
            for (Eigen::Index c = 0; c < nf; ++c) {
                A(6 + r, 6 + c) -= d.Ks(k, free_shafts_[static_cast<std::size_t>(c)]);
            }
            rhs(6 + r) += d.Q0(k);
        }
        converged = converged && d.converged;
        if (component_loads.size() == components_.size()) {
            component_loads[i] = d.load0_b;
        }
    }
    // The parts no component models, J referred to the shaft, gear g, about
    // axis n (D-052, D-053): their rigid inertia is the airframe's (D-002), so
    // only the spin relative to it is added: −(J/g) Ω̇ n on the airframe and
    // −J Ω̇ − (J/g) n·ω̇ on the shaft, a symmetric pair.
    for (Eigen::Index r = 0; r < nf; ++r) {
        const ShaftConfig& sh =
            shafts_[static_cast<std::size_t>(free_shafts_[static_cast<std::size_t>(r)])];
        for (const ShaftPart& p : sh.parts) {
            A.block<3, 1>(3, 6 + r) += (p.inertia / p.gear) * p.axis_b;
            A.block<1, 3>(6 + r, 3) += (p.inertia / p.gear) * p.axis_b.transpose();
            A(6 + r, 6 + r) += p.inertia;
        }
    }
    return converged;
}

Executive::VecZ Executive::solve(const MatZ& A, const VecZ& rhs) const noexcept {
    if (free_shafts_.empty()) {
        // Exactly the pre-D-052 solve, so models without free shafts keep
        // their results to the bit.
        const Mat66 A6 = A.topLeftCorner<6, 6>();
        const Vec6 r6 = rhs.head<6>();
        return VecZ(A6.partialPivLu().solve(r6));
    }
    return A.partialPivLu().solve(rhs);
}

void Executive::back_substitute(std::span<const double> x, const VecZ& z,
                                std::span<double> xdot) const noexcept {
    const Vec6 a = z.head<6>();
    // ẋ_c = ẋ0_c + G_c a + G_s,c Ω̇
    body_kinematics(unpack_body(x), a, xdot);
    std::array<double, kMaxShafts> shaft_acc{};
    for (std::size_t r = 0; r < free_shafts_.size(); ++r) {
        shaft_acc[static_cast<std::size_t>(free_shafts_[r])] = z(static_cast<Eigen::Index>(6 + r));
    }
    for (std::size_t j = 0; j < xdot0_.size(); ++j) {
        double v = xdot0_[j];
        for (std::size_t k = 0; k < 6; ++k) {
            v += G_[j * 6 + k] * a(static_cast<Eigen::Index>(k));
        }
        if (!Gs_.empty()) {
            for (std::size_t k = 0; k < shafts_.size(); ++k) {
                v += Gs_[j * kMaxShafts + k] * shaft_acc[k];
            }
        }
        xdot[body_layout::kSize + j] = v;
    }
    // ψ̇ = Ω; Ω̇ from the solve, 0 for an ideal drive (D-052).
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        xdot[shaft_offset(k)] = x[shaft_offset(k) + 1];
        xdot[shaft_offset(k) + 1] = shaft_acc[k];
    }
}

bool Executive::derivatives(double t, std::span<const double> x, const Controls& controls,
                            const Environment& env, std::span<double> xdot) const noexcept {
    MatZ A;
    VecZ rhs;
    const bool converged = evaluate(t, x, controls, env, A, rhs, {});
    back_substitute(x, solve(A, rhs), xdot);
    return converged;
}

bool Executive::body_acceleration(const Controls& controls, const Environment& env,
                                  Vec6& a) const noexcept {
    MatZ A;
    VecZ rhs;
    const bool converged = evaluate(sim_time(), x_, controls, env, A, rhs, {});
    a = solve(A, rhs).head<6>();
    return converged;
}

bool Executive::held_derivatives(double t, std::span<const double> x, const Controls& controls,
                                 const Environment& env, std::span<double> xdot, Vec6& net,
                                 std::span<Wrench> component_loads) const noexcept {
    MatZ A;
    VecZ rhs;
    const bool converged = evaluate(t, x, controls, env, A, rhs, component_loads);
    // The body held (a = 0): the free shafts still accelerate, by their own
    // rows; the body feels what is left of its rows (D-052).
    VecZ z = VecZ::Zero(A.rows());
    const auto nf = static_cast<Eigen::Index>(free_shafts_.size());
    if (nf > 0) {
        z.tail(nf) = A.bottomRightCorner(nf, nf).partialPivLu().solve(rhs.tail(nf));
    }
    net = rhs.head<6>();
    if (nf > 0) {
        net -= A.topRows<6>() * z;
    }
    std::copy(xdot0_.begin(), xdot0_.end(),
              xdot.begin() + static_cast<std::ptrdiff_t>(body_layout::kSize));
    for (std::size_t j = 0; j < xdot0_.size() && !Gs_.empty(); ++j) {
        for (Eigen::Index r = 0; r < nf; ++r) {
            const auto k = static_cast<std::size_t>(free_shafts_[static_cast<std::size_t>(r)]);
            xdot[body_layout::kSize + j] += Gs_[j * kMaxShafts + k] * z(6 + r);
        }
    }
    for (std::size_t k = 0; k < shafts_.size(); ++k) {
        xdot[shaft_offset(k)] = x[shaft_offset(k) + 1];
        xdot[shaft_offset(k) + 1] = 0.0;
    }
    for (Eigen::Index r = 0; r < nf; ++r) {
        xdot[shaft_offset(static_cast<std::size_t>(free_shafts_[static_cast<std::size_t>(r)])) +
             1] = z(6 + r);
    }
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
    wrap_shaft_azimuths(x_);  // D-052
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
