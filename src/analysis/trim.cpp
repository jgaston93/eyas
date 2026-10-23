#include "analysis/trim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <span>

#include <Eigen/Dense>

#include "airframe/landing_gear.hpp"
#include "analysis/held_orbit.hpp"
#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "engine/genhel_t700.hpp"
#include "env/atmosphere.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

constexpr std::size_t kPitch = 4;
constexpr std::size_t kRoll = 5;
constexpr std::size_t kHead = 6;  // controls (4) and attitude; component states follow

struct RotorInfo {
    double omega = 0.0;   // [rad/s]; 0: no rotor
    double radius = 1.0;  // [m], the moment scale
    int shaft = -1;       // the shaft it turns on (D-052), or −1
    double gear = 1.0;    // its speed over the shaft's
};

RotorInfo rotor_info(const Executive& ex) {
    RotorInfo r;
    bool found = false;
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        const auto* b = dynamic_cast<const BladeElementRotor*>(&ex.component(i));
        if (b == nullptr) {
            continue;
        }
        const BladeElementRotorConfig& cfg = b->config();
        if (found && cfg.rotor_speed != r.omega) {
            throw InitError(
                "trim: rotors turning at different speeds have no common period (D-035)");
        }
        if (!found) {
            r.omega = cfg.rotor_speed;
            r.radius = cfg.radius;
            r.shaft = b->shaft_index();
            r.gear = cfg.shaft ? cfg.shaft->gear : 1.0;
        }
        found = true;
    }
    return r;
}

// |γ| = 90° to within the rounding of cos γ: the track is undefined.
bool vertical(double flight_path_angle) {
    return std::cos(flight_path_angle) <= 1e-12;
}

void check(const TrimCondition& c) {
    const double half_pi = 0.5 * std::numbers::pi;
    if (!std::isfinite(c.airspeed) || c.airspeed < 0.0) {
        throw InitError("trim: airspeed must be finite and non-negative");
    }
    if (!std::isfinite(c.flight_path_angle) || std::abs(c.flight_path_angle) > half_pi) {
        throw InitError("trim: flight-path angle must be within ±90°");
    }
    if (!std::isfinite(c.sideslip) || std::abs(c.sideslip) >= half_pi) {
        throw InitError("trim: sideslip must be within ±90°, exclusive");
    }
    // In vertical flight the air velocity is along the body's vertical, so
    // the sideslip follows from the attitude and can't be asked for.
    if (vertical(c.flight_path_angle) && c.sideslip != 0.0) {
        throw InitError("trim: sideslip must be 0 in vertical flight (|γ| = 90°)");
    }
    if (!std::isfinite(c.heading) || !std::isfinite(c.altitude) || !c.wind_n.allFinite()) {
        throw InitError("trim: heading, altitude and wind must be finite");
    }
    // D-050: NaN is no ground. With a ground the rotor's ground effect
    // depends on the height, which the held body keeps constant, so only
    // flight at constant height is a steady state near it.
    if (std::isinf(c.ground_height)) {
        throw InitError("trim: ground height must be finite, or NaN for no ground");
    }
    if (!std::isnan(c.ground_height) &&
        std::abs(c.airspeed * std::sin(c.flight_path_angle) - c.wind_n.z()) > 1e-9) {
        throw InitError(
            "trim: with a ground the flight must be at constant height (no climb, descent or "
            "vertical wind): ground effect changes with height");
    }
    if (!isa_delta_T_valid(c.isa_delta_T)) {
        throw InitError("trim: ISA temperature offset is invalid (D-019)");
    }
}

// The trim problem: unknowns z = [controls (4), θ, φ, component states at
// t = 0, the free shafts' speeds at t = 0, T], residual r = [mean net force /
// W, mean net moment / (W R), x_c(T) − x_c(0), Ω(T) − Ω(0), ψ(T) − ψ(0) −
// 2π/g] over one revolution with the body held (D-035). An ideal shaft holds
// its governed speed; a free one's speed is an unknown whose periodicity the
// engines' governor sets (D-053 item 6). When the rotor turns on a free shaft
// its revolution's period T is an unknown too, closed by the shaft turning
// 2π/g in it (g the rotor's gear): the governor need not hold the governed
// speed exactly (an ECU at its limit droops, D-053 amendment), and the
// revolution must still be one turn of the rotor for the orbit to close.
class Problem {
public:
    Problem(const Executive& ex, const TrimCondition& c, const TrimOptions& o)
        : ex_(ex),
          c_(c),
          n_(ex.num_states()),
          nc_(ex.num_component_states()),
          held_(ex, trim_environment(c), trim_period(ex), o.steps_per_period),
          x_(n_, 0.0) {
        weight_ = ex.airframe_mass().mass * units::kStandardGravity;
        length_ = rotor_info(ex).radius;
        for (std::size_t k = 0; k < ex.shafts().size(); ++k) {
            if (ex.shafts()[k].drive == ShaftDrive::free) {
                free_.push_back(k);
            }
        }
        for (std::size_t i = 0; i < ex.num_components(); ++i) {
            if (dynamic_cast<const GenHelT700*>(&ex.component(i)) != nullptr) {
                engines_.push_back(i);
            }
        }
        const RotorInfo rotor = rotor_info(ex);
        if (rotor.shaft >= 0 &&
            ex.shafts()[static_cast<std::size_t>(rotor.shaft)].drive == ShaftDrive::free) {
            period_shaft_ = rotor.shaft;
            turn_ = 2.0 * std::numbers::pi / rotor.gear;
        }
    }

    std::size_t unknowns() const { return kHead + nc_ + free_.size() + (free_period() ? 1 : 0); }
    // The period is an unknown, z's last entry.
    bool free_period() const { return period_shaft_ >= 0; }
    bool has_engines() const { return !engines_.empty(); }
    // Stage 1 of a cold start (D-053 amendment 2): every evaluation puts the
    // engines at their equilibrium for the rotor's demand and the free shafts
    // at their governed speeds, with the governed period, so the shafts
    // barely move, as on ideal drives; their unknowns and residuals drop out.
    void set_seeded(bool on) { seeded_ = on; }
    // The period [s] for z.
    double period(const VecX& z) const { return free_period() ? z(z.size() - 1) : held_.period(); }
    const Environment& env() const { return held_.env(); }
    // The free shafts' indices, in z's order after the component states.
    const std::vector<std::size_t>& free_shafts() const { return free_; }
    double weight() const { return weight_; }
    double length() const { return length_; }
    std::size_t evaluations_unconverged() const { return held_.unconverged(); }
    void clear_unconverged() { held_.clear_unconverged(); }

    // The pilot inputs in z with the condition's AFCS switches (D-045).
    Controls controls(const VecX& z) const {
        Controls u;
        u.afcs = c_.afcs;
        u.collective = z(0);
        u.lat_cyclic = z(1);
        u.lon_cyclic = z(2);
        u.pedal = z(3);
        return u;
    }

    // Packs the full state for z into x_; false if the attitude admits no
    // track with the condition's sideslip.
    bool pack(const VecX& z) {
        if (free_period()) {
            const double T = seeded_ ? trim_period(ex_) : z(z.size() - 1);
            if (!(T > 0.0) || !std::isfinite(T)) {
                return false;
            }
            held_.set_period(T);
        }
        BodyState body;
        if (!trim_body_state(c_, z(kPitch), z(kRoll), ex_.origin_altitude(), body)) {
            return false;
        }
        pack_body(body, x_);
        for (std::size_t j = 0; j < nc_; ++j) {
            x_[body_layout::kSize + j] = z(static_cast<Eigen::Index>(kHead + j));
        }
        // The shafts start at ψ = 0 at their governed speeds; with ideal drives
        // the speeds hold and ψ advances 2π a revolution (D-052). A free one
        // starts at its unknown speed (D-053).
        ex_.initial_shaft_states(x_);
        for (std::size_t r = 0; r < free_.size(); ++r) {
            x_[ex_.shaft_offset(free_[r]) + 1] =
                seeded_ ? ex_.shafts()[free_[r]].speed
                        : z(static_cast<Eigen::Index>(kHead + nc_ + r));
        }
        if (seeded_) {
            seed_engines(controls(z));
        }
        // States with an algebraic equilibrium on the held body take it
        // (GenHel's IC mode, D-045); their unknowns in z then have no effect,
        // and the minimum-norm step leaves them alone.
        const SimContext ctx = ex_.context(0.0, x_, controls(z), held_.env());
        for (std::size_t i = 0; i < ex_.num_components(); ++i) {
            ex_.component(i).steady_states(
                ctx, std::span<double>(x_).subspan(ex_.component_offset(i), ex_.component_size(i)));
        }
        start_ = x_;
        return true;
    }

    const std::vector<double>& state() const { return x_; }

    // Integrates `periods` revolutions with the body held from x_ (t = 0),
    // leaving the end state in x_ and the time integral of the net wrench in
    // `integral`. RK4 on the component states; the integral uses the same
    // weights, so it is the RK4 solution of İ = net.
    void hold(int periods, const Controls& u, Vec6& integral) {
        integral.setZero();
        const int steps = held_.steps();
        for (int s = 0; s < periods * steps; ++s) {
            hold_one_step(s % steps, u, integral);
        }
    }

    // One held RK4 step from node s of the revolution, adding the step's
    // integral of the net wrench into `integral`.
    void hold_one_step(int s, const Controls& u, Vec6& integral) { held_.step(s, u, x_, integral); }

    // The body's vibration at t = 0 for z: the free body, flown along the
    // held orbit, accelerates with a(t) = [v̇_b; ω̇_b] (the full coupled
    // solve), periodic with zero mean in trim. Its velocity and rate then
    // oscillate as ∫a dt about their means; this returns the oscillation's
    // value at t = 0, i.e. minus the revolution mean of ∫₀ᵗ a ds, so that a
    // free run started from the mean state plus it starts on its periodic
    // orbit instead of off it by a constant rate (D-035, M7a).
    Vec6 vibration(const VecX& z) {
        const Controls u = controls(z);
        Vec6 integral = Vec6::Zero();  // ∫₀ᵗ a ds at the current node
        Vec6 mean = Vec6::Zero();      // revolution mean of it (trapezoid)
        if (!pack(z)) {
            return mean;
        }
        const double period = held_.period();
        const int steps = held_.steps();
        const double h = period / steps;
        std::vector<double> xd(n_, 0.0);
        const auto accel = [&](double t) {
            (void)ex_.derivatives(t, x_, u, held_.env(), xd);
            Vec6 a;
            for (int k = 0; k < 3; ++k) {
                a(k) = xd[body_layout::kVel + static_cast<std::size_t>(k)];
                a(3 + k) = xd[body_layout::kOmega + static_cast<std::size_t>(k)];
            }
            return a;
        };
        Vec6 a0 = accel(0.0);
        Vec6 ignored;
        for (int s = 0; s < steps; ++s) {
            const Vec6 before = integral;
            hold_one_step(s, u, ignored);  // advances x_ by one step
            const Vec6 a1 = accel(period * static_cast<double>(s + 1) / steps);
            integral += 0.5 * h * (a0 + a1);
            mean += 0.5 * h * (before + integral) / period;
            a0 = a1;
        }
        return -mean;
    }

    // The states labelled orbit (GenHel's IC mode, D-045), as indices into
    // the full state; the executive checked the labels when it was built.
    std::vector<std::size_t> orbit_states() const {
        std::vector<std::size_t> orbit;
        const std::span<const SteadyKind> kinds = ex_.steady_kinds();
        for (std::size_t i = 0; i < kinds.size(); ++i) {
            if (kinds[i] == SteadyKind::orbit) {
                orbit.push_back(i);
            }
        }
        return orbit;
    }

    // The IC-mode states on their free-flight periodic orbit (D-045
    // amendment). steady_states() puts them where the held body leaves them,
    // but the free body vibrates at 4/rev from t = 0, and these states
    // (sensors and the servos they drive) respond to that vibration: started
    // from their held values they ring at their own frequencies, a decaying
    // transient on every free flight from the trim. Here they are integrated
    // along the held orbit with the body's velocity and rate free (from the
    // mean plus `vib`, as solve_trim installs them) under the coupled body
    // acceleration, position, attitude and every other component state on
    // the held orbit as D-035 approximates them; the start values that
    // return after one revolution come from Newton's method on r(w) =
    // w(T) − w(0), (Φ − I) Δw = −r, with one finite-difference Jacobian Φ − I
    // (the map is affine up to the rate limits and table corners, which a
    // trim keeps clear of). `residual` is the largest change over the
    // revolution at the result. No IC-mode states: nothing to do.
    struct FreeOrbit {
        std::vector<std::size_t> index;  // into the full state
        std::vector<double> value;
        double residual = 0.0;
        std::size_t unconverged = 0;  // coupled evaluations (D-023)
    };
    FreeOrbit free_ic_orbit(const VecX& z, const Vec6& vib, double tol) {
        FreeOrbit out;
        out.index = orbit_states();
        if (out.index.empty() || !pack(z)) {
            out.index.clear();
            return out;
        }
        const Controls u = controls(z);
        const int steps = held_.steps();
        const double period = held_.period();
        const double h = period / steps;
        const auto nic = static_cast<Eigen::Index>(out.index.size());
        // The held orbit's RK4 stage states, which this pass reads, not changes.
        std::vector<std::array<std::vector<double>, 4>> stages(static_cast<std::size_t>(steps));
        Vec6 ignored = Vec6::Zero();
        VecX w0(nic);
        for (Eigen::Index j = 0; j < nic; ++j) {
            w0(j) = x_[out.index[static_cast<std::size_t>(j)]];
        }
        VecX y0(6 + nic);
        for (int k = 0; k < 3; ++k) {
            y0(k) = x_[body_layout::kVel + static_cast<std::size_t>(k)] + vib(k);
            y0(3 + k) = x_[body_layout::kOmega + static_cast<std::size_t>(k)] + vib(3 + k);
        }
        for (int s = 0; s < steps; ++s) {
            held_.step(s, u, x_, ignored, &stages[static_cast<std::size_t>(s)]);
        }
        std::vector<double> xc(n_, 0.0);
        std::vector<double> xd(n_, 0.0);
        // ẏ for y = [v_b; ω_b; IC-mode states] at RK4 stage k of step s.
        const auto rate = [&](int s, std::size_t k, double t, const VecX& y) {
            xc = stages[static_cast<std::size_t>(s)][k];
            for (std::size_t i = 0; i < 3; ++i) {
                xc[body_layout::kVel + i] = y(static_cast<Eigen::Index>(i));
                xc[body_layout::kOmega + i] = y(static_cast<Eigen::Index>(3 + i));
            }
            for (Eigen::Index j = 0; j < nic; ++j) {
                xc[out.index[static_cast<std::size_t>(j)]] = y(6 + j);
            }
            if (!ex_.derivatives(t, xc, u, held_.env(), xd)) {
                ++out.unconverged;
            }
            VecX dy(y.size());
            for (std::size_t i = 0; i < 3; ++i) {
                dy(static_cast<Eigen::Index>(i)) = xd[body_layout::kVel + i];
                dy(static_cast<Eigen::Index>(3 + i)) = xd[body_layout::kOmega + i];
            }
            for (Eigen::Index j = 0; j < nic; ++j) {
                dy(6 + j) = xd[out.index[static_cast<std::size_t>(j)]];
            }
            return dy;
        };
        // w(T) − w(0) for the IC-mode states starting at w.
        const auto change = [&](const VecX& w) {
            VecX y = y0;
            y.tail(nic) = w;
            for (int s = 0; s < steps; ++s) {
                const double t = period * static_cast<double>(s) / steps;
                const VecX k1 = rate(s, 0, t, y);
                const VecX k2 = rate(s, 1, t + 0.5 * h, y + 0.5 * h * k1);
                const VecX k3 = rate(s, 2, t + 0.5 * h, y + 0.5 * h * k2);
                const VecX k4 = rate(s, 3, t + h, y + h * k3);
                y += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            }
            return VecX(y.tail(nic) - w);
        };
        VecX w = w0;
        VecX r = change(w);
        MatX J(nic, nic);
        for (Eigen::Index j = 0; j < nic; ++j) {
            const double dw = 1e-6 * std::max(1.0, std::abs(w(j)));
            VecX wj = w;
            wj(j) += dw;
            J.col(j) = (change(wj) - r) / dw;
        }
        const Eigen::FullPivLU<MatX> lu(J);
        constexpr int kMaxIterations = 6;  // spec §7 rule 3
        for (int it = 0; it < kMaxIterations && r.allFinite() && r.cwiseAbs().maxCoeff() >= tol;
             ++it) {
            w -= lu.solve(r);
            r = change(w);
        }
        out.value.assign(w.data(), w.data() + nic);
        out.residual =
            r.allFinite() ? r.cwiseAbs().maxCoeff() : std::numeric_limits<double>::quiet_NaN();
        (void)pack(z);
        return out;
    }

    // r(z); false if z has no valid body state or the result isn't finite.
    bool residual(const VecX& z, VecX& r) {
        if (!pack(z)) {
            return false;
        }
        Vec6 integral;
        held_.clear_held_change();
        hold(1, controls(z), integral);
        const Vec6 mean = integral / held_.period();
        r.resize(static_cast<Eigen::Index>(unknowns()));
        r.head<3>() = mean.head<3>() / weight_;
        r.segment<3>(3) = mean.tail<3>() / (weight_ * length_);
        // A held state stays at its steady value through the revolution (the
        // held integrator keeps it, as GenHel's IC mode holds integrators, p.
        // 5.5-2; D-045), so its residual is zero. The change it would have
        // made is reported as a mean rate instead (held_drift_rate()).
        held_drift_ = 0.0;
        const std::span<const SteadyKind> kinds = ex_.steady_kinds();
        for (std::size_t j = 0; j < nc_; ++j) {
            const std::size_t i = body_layout::kSize + j;
            // A rotor's own azimuth needn't come round in a revolution of the
            // trim's rotor (D-053 amendment 3), as the shafts' needn't.
            r(static_cast<Eigen::Index>(kHead + j)) =
                kinds[i] == SteadyKind::phase ? 0.0 : x_[i] - start_[i];
            held_drift_ = std::max(held_drift_, std::abs(held_.held_change()[i]) / held_.period());
        }
        for (std::size_t q = 0; q < free_.size(); ++q) {
            const std::size_t i = ex_.shaft_offset(free_[q]) + 1;
            r(static_cast<Eigen::Index>(kHead + nc_ + q)) = seeded_ ? 0.0 : x_[i] - start_[i];
        }
        if (free_period()) {
            // The turn less one rotor revolution, in (−π, π]: ψ is wrapped.
            const std::size_t i = ex_.shaft_offset(static_cast<std::size_t>(period_shaft_));
            r(r.size() - 1) =
                seeded_ ? 0.0 : std::remainder(x_[i] - start_[i] - turn_, 2.0 * std::numbers::pi);
        }
        if (seeded_) {
            for (const std::size_t e : engines_) {
                const std::size_t first = ex_.component_offset(e) - body_layout::kSize;
                for (std::size_t j = 0; j < ex_.component_size(e); ++j) {
                    r(static_cast<Eigen::Index>(kHead + first + j)) = 0.0;
                }
            }
        }
        return r.allFinite() && std::isfinite(held_drift_);
    }

    // The largest mean rate of a held state over the revolution of the last
    // residual() [its units per second].
    double held_drift_rate() const { return held_drift_; }

    // Every GenHel T700 engine's states in x_ at their equilibrium for the
    // main rotor's demand at x_ (D-053 item 6).
    void seed_engines(const Controls& u) {
        if (engines_.empty()) {
            return;
        }
        std::vector<double>& xd = scratch_;
        xd.assign(x_.size(), 0.0);
        (void)ex_.derivatives(0.0, x_, u, held_.env(), xd);  // fills the coupling store
        const SimContext ctx = ex_.context(0.0, x_, u, held_.env());
        for (const std::size_t i : engines_) {
            const auto& e = static_cast<const GenHelT700&>(ex_.component(i));
            const double demand = e.rotor_demand(ctx);
            e.equilibrium(
                (1.0 + e.config().loss_factor) * demand, demand, u.collective,
                std::span<double>(x_).subspan(ex_.component_offset(i), ex_.component_size(i)));
        }
    }

private:
    const Executive& ex_;
    TrimCondition c_;
    std::size_t n_;
    std::size_t nc_;
    double weight_ = 0.0;
    double length_ = 1.0;
    HeldIntegrator held_;
    std::vector<double> x_;
    std::vector<double> start_;  // x_ as packed, before the revolution
    double held_drift_ = 0.0;
    std::vector<std::size_t> free_;     // the free shafts (D-053)
    std::vector<std::size_t> engines_;  // GenHel T700 components
    bool seeded_ = false;               // stage 1 of a cold start
    std::vector<double> scratch_;
    int period_shaft_ = -1;  // the free shaft the rotor turns on, or −1
    double turn_ = 0.0;      // its turn in one rotor revolution, 2π/g [rad]
};

// The engines' states in z at their equilibrium for the main rotor's demand
// at the state the problem packs for z (D-053 item 6): a start for Newton's
// method, the engines delivering the rotor's demand with losses.
void engine_start(const Executive& ex, Problem& p, VecX& z, std::size_t nc) {
    if (!p.has_engines() || !p.pack(z)) {
        return;
    }
    p.seed_engines(p.controls(z));
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const GenHelT700*>(&ex.component(i)) == nullptr) {
            continue;
        }
        const std::size_t first = ex.component_offset(i) - body_layout::kSize;
        for (std::size_t j = 0; j < ex.component_size(i) && first + j < nc; ++j) {
            z(static_cast<Eigen::Index>(kHead + first + j)) = p.state()[ex.component_offset(i) + j];
        }
    }
}

struct Norms {
    double force = 0.0;     // [-] of weight
    double moment = 0.0;    // [-] of weight × radius
    double periodic = 0.0;  // max |Δx_c|
};

Norms norms(const VecX& r) {
    Norms m;
    m.force = r.head<3>().norm();
    m.moment = r.segment<3>(3).norm();
    if (r.size() > static_cast<Eigen::Index>(kHead)) {
        m.periodic = r.tail(r.size() - static_cast<Eigen::Index>(kHead)).cwiseAbs().maxCoeff();
    }
    return m;
}

bool converged(const Norms& m, const TrimOptions& o) {
    return m.force < o.force_tol && m.moment < o.moment_tol && m.periodic < o.periodic_tol;
}

// True if any landing-gear wheel carries a normal load at any of the held
// revolution's integration steps (the trim's own orbit and step).
bool gear_loaded(const Executive& ex, const TrimCondition& c, const TrimResult& r,
                 const TrimOptions& o) {
    std::vector<std::size_t> gears;
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const LandingGear*>(&ex.component(i)) != nullptr) {
            gears.push_back(i);
        }
    }
    if (gears.empty()) {
        return false;
    }
    const HeldOrbit orbit = held_orbit(ex, c, r, o.steps_per_period, o.steps_per_period);
    const Environment env = trim_environment(c);
    const double T = trim_period(ex, r);
    for (std::size_t j = 0; j < orbit.states.size(); ++j) {
        const std::vector<double>& x = orbit.states[j];
        const double t = T * static_cast<double>(j) / static_cast<double>(orbit.states.size());
        const SimContext ctx = ex.context(t, x, r.controls, env);
        for (const std::size_t i : gears) {
            const auto& gear = static_cast<const LandingGear&>(ex.component(i));
            const std::span<const double> xs =
                std::span<const double>(x).subspan(ex.component_offset(i), ex.component_size(i));
            for (std::size_t k = 0; k < gear.config().gears.size(); ++k) {
                if (gear.gear_state(ctx, xs, k).normal_load > 0.0) {
                    return true;
                }
            }
        }
    }
    return false;
}

}  // namespace

double trim_period(const Executive& ex) {
    const RotorInfo r = rotor_info(ex);
    return r.omega > 0.0 ? 2.0 * std::numbers::pi / r.omega : 0.1;
}

double trim_period(const Executive& ex, const TrimResult& r) {
    return r.period > 0.0 ? r.period : trim_period(ex);
}

bool trim_body_state(const TrimCondition& c, double pitch, double roll, double origin_altitude,
                     BodyState& body) {
    body = BodyState{};
    body.q_nb = quaternion(EulerAngles{roll, pitch, c.heading});
    const Mat3 C_bn = body.q_nb.toRotationMatrix().transpose();
    body.pos_n = Vec3(0.0, 0.0, -(c.altitude - origin_altitude));
    const double cg = std::cos(c.flight_path_angle);
    const double sg = std::sin(c.flight_path_angle);
    const auto air_n = [&](double chi) {  // unit air velocity on track χ, NED
        return Vec3(cg * std::cos(chi), cg * std::sin(chi), -sg);
    };
    Vec3 v_air_n = Vec3::Zero();
    if (c.airspeed > 0.0) {
        // v_air_b,y / V = sin β:  A cos χ + B sin χ = C.
        const double A = C_bn(1, 0) * cg;
        const double B = C_bn(1, 1) * cg;
        const double C = std::sin(c.sideslip) + C_bn(1, 2) * sg;
        const double R = std::hypot(A, B);
        double chi = c.heading;  // vertical flight: the track is undefined (check())
        if (R <= 1e-12 && !vertical(c.flight_path_angle) && std::abs(C) > 1e-12) {
            return false;  // body y axis vertical: no track changes v_air_b,y
        }
        if (R > 1e-12) {
            const double ratio = C / R;
            if (std::abs(ratio) > 1.0) {
                return false;
            }
            const double delta = std::atan2(B, A);
            const double spread = std::acos(ratio);
            const double chi1 = delta + spread;
            const double chi2 = delta - spread;
            chi = (C_bn.row(0) * air_n(chi1)) >= (C_bn.row(0) * air_n(chi2)) ? chi1 : chi2;
        }
        v_air_n = c.airspeed * air_n(chi);
    }
    body.vel_b = C_bn * (v_air_n + c.wind_n);
    return true;
}

TrimResult solve_trim(const Executive& ex, const TrimCondition& c, const TrimResult* start,
                      std::vector<double>& x, const TrimOptions& o) {
    check(c);
    Problem p(ex, c, o);
    const std::size_t nc = ex.num_component_states();
    const auto n_unknowns = static_cast<Eigen::Index>(p.unknowns());
    VecX z = VecX::Zero(n_unknowns);
    if (start != nullptr) {
        if (start->component_states.size() != ex.num_states() - body_layout::kSize) {
            throw InitError("trim: the starting point's component states don't fit this model");
        }
        z(0) = start->controls.collective;
        z(1) = start->controls.lat_cyclic;
        z(2) = start->controls.lon_cyclic;
        z(3) = start->controls.pedal;
        z(kPitch) = start->pitch;
        z(kRoll) = start->roll;
        for (std::size_t j = 0; j < nc; ++j) {
            z(static_cast<Eigen::Index>(kHead + j)) = start->component_states[j];
        }
        for (std::size_t r = 0; r < p.free_shafts().size(); ++r) {
            const std::size_t i = ex.shaft_offset(p.free_shafts()[r]) + 1 - body_layout::kSize;
            z(static_cast<Eigen::Index>(kHead + nc + r)) = start->component_states[i];
        }
        if (p.free_period()) {
            z(n_unknowns - 1) = start->period > 0.0 ? start->period : trim_period(ex);
        }
        // A nearby trim's engines deliver its torque, not this one's, so they
        // start from their equilibrium for this condition's demand, unless
        // the start already is this condition's trim: a converged start then
        // stays where it is (bit for bit, as the trims with the FPS on and
        // off must be). Its engine states can't simply be kept otherwise: an
        // ECU integrator past its limit has no effect, so Newton couldn't
        // move it back inside.
        VecX r_kept;
        if (!(p.residual(z, r_kept) && converged(norms(r_kept), o))) {
            engine_start(ex, p, z, nc);
        }
    } else {
        // Cold start: mid collective, level, every component at its initial
        // state, then a few held revolutions so the inflow and blades settle.
        z(0) = 0.5;
        std::vector<double> x0(ex.num_states(), 0.0);
        for (std::size_t i = 0; i < ex.num_components(); ++i) {
            ex.component(i).initial_states(
                std::span<double>(x0).subspan(ex.component_offset(i), ex.component_size(i)));
        }
        for (std::size_t j = 0; j < nc; ++j) {
            z(static_cast<Eigen::Index>(kHead + j)) = x0[body_layout::kSize + j];
        }
        for (std::size_t r = 0; r < p.free_shafts().size(); ++r) {
            z(static_cast<Eigen::Index>(kHead + nc + r)) = ex.shafts()[p.free_shafts()[r]].speed;
        }
        if (p.free_period()) {
            z(n_unknowns - 1) = trim_period(ex);
        }
        // The engines at their equilibrium for the rotor's demand, before and
        // after settling, so the free shafts hold their speed meanwhile.
        engine_start(ex, p, z, nc);
        if (p.pack(z)) {
            Vec6 ignored;
            p.hold(o.settle_periods, p.controls(z), ignored);
            for (std::size_t j = 0; j < nc; ++j) {
                z(static_cast<Eigen::Index>(kHead + j)) = p.state()[body_layout::kSize + j];
            }
        }
        engine_start(ex, p, z, nc);
    }

    TrimResult result;
    VecX r;
    bool ok = false;
    // A z with no valid residual has no residual norms either, not zero ones.
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr Norms invalid{nan, nan, nan};
    Norms m = invalid;
    int it = 0;
    const Eigen::Index n = n_unknowns;
    // Newton with a finite-difference Jacobian, kept between iterations with
    // Broyden's rank-one update (Broyden 1965; Dennis & Schnabel 1983,
    // §8.1) and recomputed only when a step makes too little progress. Each
    // column costs a revolution, each update nothing.
    MatX J;
    bool fresh = false;  // J is a finite-difference Jacobian at the current z
    const auto jacobian = [&] {
        J.resize(n, n);
        VecX rj;
        for (Eigen::Index j = 0; j < n; ++j) {
            const double dz = 1e-6 * std::max(1.0, std::abs(z(j)));
            VecX zj = z;
            zj(j) += dz;
            if (!p.residual(zj, rj)) {
                return false;
            }
            J.col(j) = (rj - r) / dz;
        }
        fresh = true;
        return true;
    };
    const auto newton = [&](const TrimOptions& to) {
        J.resize(0, 0);
        fresh = false;
        ok = p.residual(z, r);
        m = ok ? norms(r) : invalid;
        while (ok && !converged(m, to) && it < o.max_iterations) {
            ++it;
            if (J.size() == 0 && !jacobian()) {
                ok = false;
                break;
            }
            // Minimum-norm step: states with no effect on the residual (the
            // wash lags at zero airspeed, D-028) keep their values.
            const VecX step = J.completeOrthogonalDecomposition().solve(-r);
            // Backtracking on the residual norm.
            bool accepted = false;
            VecX zt;
            VecX rt;
            for (int k = 0; k < 12 && !accepted; ++k) {
                zt = z + std::ldexp(1.0, -k) * step;
                accepted = p.residual(zt, rt) && rt.norm() < r.norm();
            }
            if (!accepted) {
                if (fresh) {
                    break;  // a true Newton step failed: give up
                }
                if (!jacobian()) {  // an updated Jacobian failed: rebuild it and retry
                    ok = false;
                }
                continue;
            }
            const double ratio = rt.norm() / r.norm();
            const VecX dz = zt - z;
            J += ((rt - r) - J * dz) * dz.transpose() / dz.squaredNorm();
            fresh = false;
            z = zt;
            r = rt;
            m = norms(r);
            // Too little progress for a quasi-Newton step: rebuild the Jacobian.
            if (ratio > 0.5 && !converged(m, to) && !jacobian()) {
                ok = false;
            }
        }
    };
    if (start == nullptr && p.has_engines()) {
        // A cold start solves first with the engines at their equilibrium and
        // the shafts governed, as on ideal drives, then frees them from there
        // (D-053 amendment 2): from the cold start's mid collective the full
        // problem can wander off (in the vortex ring it stalled 10 % of
        // collective from the trim the ideal drive finds). Stage 1 only has
        // to reach the right neighbourhood: 10³ times the tolerances.
        TrimOptions loose = o;
        loose.force_tol *= 1e3;
        loose.moment_tol *= 1e3;
        loose.periodic_tol *= 1e3;
        p.set_seeded(true);
        newton(loose);
        p.set_seeded(false);
        engine_start(ex, p, z, nc);
    }
    newton(o);

    // The final revolution again, counting unconverged evaluations in it
    // alone (D-023), then the state at t = 0 for the final z.
    p.clear_unconverged();
    if (ok) {
        ok = p.residual(z, r);
        m = ok ? norms(r) : invalid;
    }
    const Vec6 vib = ok ? p.vibration(z) : Vec6::Zero();
    const Problem::FreeOrbit free =
        ok ? p.free_ic_orbit(z, vib, o.periodic_tol) : Problem::FreeOrbit{};
    (void)p.pack(z);
    x = p.state();
    // The held orbit's component states, from which a trim or linearization
    // continues; the start state x differs in its IC-mode states.
    result.component_states.assign(x.begin() + static_cast<std::ptrdiff_t>(body_layout::kSize),
                                   x.end());
    result.vel_b = unpack_body(x).vel_b;  // the mean, as the condition asks
    for (std::size_t k = 0; k < 3; ++k) {
        x[body_layout::kVel + k] += vib(static_cast<Eigen::Index>(k));
        x[body_layout::kOmega + k] += vib(static_cast<Eigen::Index>(3 + k));
    }
    for (std::size_t j = 0; j < free.index.size(); ++j) {
        x[free.index[j]] = free.value[j];
    }
    if (ok && !(free.residual <= m.periodic)) {  // NaN too
        m.periodic = free.residual;
    }
    // A non-finite start state (e.g. from a singular coupled solve in the
    // vibration offset) can't be flown on from, whatever the residuals say.
    const bool finite = std::all_of(x.begin(), x.end(), [](double v) { return std::isfinite(v); });
    result.converged = ok && finite && converged(m, o) && p.evaluations_unconverged() == 0 &&
                       free.unconverged == 0;
    result.iterations = it;
    result.controls = p.controls(z);
    result.pitch = z(kPitch);
    result.roll = z(kRoll);
    result.period = p.period(z);
    result.force_residual = m.force * p.weight();
    result.moment_residual = m.moment * p.weight() * p.length();
    result.periodic_residual = m.periodic;
    result.held_drift_rate = ok ? p.held_drift_rate() : nan;
    // A trim near the ground is flight in ground effect, not rest on the
    // gear (D-050): with a wheel loaded anywhere on the held revolution it
    // doesn't converge.
    if (result.converged && std::isfinite(c.ground_height) && gear_loaded(ex, c, result, o)) {
        result.converged = false;
    }
    return result;
}

RollTrimResult solve_trim_at_roll(const Executive& ex, const TrimCondition& c, double roll,
                                  const TrimResult* start, std::vector<double>& x,
                                  const TrimOptions& o, double roll_tol) {
    check(c);
    if (c.airspeed == 0.0 || vertical(c.flight_path_angle)) {
        throw InitError(
            "trim at roll: sideslip is undefined at zero airspeed or in vertical flight");
    }
    if (!std::isfinite(roll) || !(roll_tol > 0.0)) {
        throw InitError("trim at roll: roll must be finite and the tolerance positive");
    }
    constexpr int kMaxIterations = 20;
    const double max_step = 10.0 * std::numbers::pi / 180.0;
    const double beta_limit = 80.0 * std::numbers::pi / 180.0;

    RollTrimResult out;
    TrimCondition ci = c;
    // f(β) = φ(β) − roll, one trim per evaluation, each from the last.
    const auto eval = [&](double beta, const TrimResult* from) {
        ci.sideslip = beta;
        out.trim = solve_trim(ex, ci, from, x, o);
        out.sideslip = beta;
        return out.trim.roll - roll;
    };
    double b0 = c.sideslip;
    double f0 = eval(b0, start);
    if (!out.trim.converged) {
        return out;
    }
    if (std::abs(f0) <= roll_tol) {
        out.converged = true;
        return out;
    }
    // The second secant point 1° away, towards zero sideslip.
    const double one_deg = std::numbers::pi / 180.0;
    double b1 = b0 > 0.0 ? b0 - one_deg : b0 + one_deg;
    TrimResult previous = out.trim;
    double f1 = eval(b1, &previous);
    while (out.trim.converged && std::abs(f1) > roll_tol && out.iterations < kMaxIterations) {
        ++out.iterations;
        const double slope = (f1 - f0) / (b1 - b0);
        if (!std::isfinite(slope) || slope == 0.0) {
            break;
        }
        const double b2 =
            std::clamp(b1 + std::clamp(-f1 / slope, -max_step, max_step), -beta_limit, beta_limit);
        if (b2 == b1) {
            break;  // pinned at the β limit
        }
        b0 = b1;
        f0 = f1;
        b1 = b2;
        previous = out.trim;
        f1 = eval(b1, &previous);
    }
    out.converged = out.trim.converged && std::abs(f1) <= roll_tol;
    return out;
}

}  // namespace eyas
