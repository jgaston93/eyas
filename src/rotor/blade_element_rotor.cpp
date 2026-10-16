#include "rotor/blade_element_rotor.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>

#include "core/rigid_body.hpp"
#include "rotor/hub_frame.hpp"
#include "rotor/multiblade.hpp"
#include "rotor/pitt_peters.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Element edges on [a, b]: n elements, uniform or equal-area.
void add_mesh(double a, double b, int n, ElementSpacing spacing, std::vector<double>& edges) {
    for (int k = 1; k <= n; ++k) {
        const double f = static_cast<double>(k) / static_cast<double>(n);
        const double e = spacing == ElementSpacing::uniform
                             ? a + f * (b - a)
                             : std::sqrt(a * a + f * (b * b - a * a));
        edges.push_back(k == n ? b : e);
    }
}

void validate(const BladeElementRotorConfig& c) {
    const auto fail = [](const std::string& what) {
        throw InitError("blade_element_rotor: " + what);
    };
    if (c.num_blades < 1 || c.num_blades > kMaxBlades) {
        fail("blades must be between 1 and " + std::to_string(kMaxBlades));
    }
    if (!(c.radius > 0.0) || !std::isfinite(c.radius)) {
        fail("radius must be positive and finite");
    }
    if (!(c.rotor_speed > 0.0) || !std::isfinite(c.rotor_speed)) {
        fail("rotor_speed must be positive and finite");
    }
    if (!c.hub_position_b.allFinite() || !std::isfinite(c.hub_pitch) ||
        !std::isfinite(c.hub_roll)) {
        fail("hub position and angles must be finite");
    }
    if (!(c.tip_loss > 0.0) || c.tip_loss > 1.0) {
        fail("tip_loss B must be in (0, 1]");
    }
    if (!(c.root_cutout >= 0.0) || !(c.root_cutout < c.tip_loss * c.radius)) {
        fail("root_cutout must be in [0, B·R)");
    }
    if (c.num_elements < 1) {
        fail("elements must be at least 1");
    }
    if (!c.airfoil) {
        fail("no airfoil");
    }
    if (!c.controls) {
        fail("no control map");
    }
    if (c.flap) {
        const FlapHinge& f = *c.flap;
        if (!(f.offset >= 0.0) || !(f.offset <= c.root_cutout)) {
            fail(
                "flap hinge offset must be in [0, root_cutout]: no lifting span inboard of the "
                "hinge");
        }
        if (!(f.mass > 0.0) || !std::isfinite(f.mass)) {
            fail("flap: blade mass must be positive and finite");
        }
        if (!(f.first_moment > 0.0) || !std::isfinite(f.first_moment) ||
            !(f.first_moment < f.mass * (c.radius - f.offset))) {
            fail("flap: first moment must be positive, with the blade CG between hinge and tip");
        }
        // Cauchy–Schwarz: (∫ρ dm)² ≤ ∫dm ∫ρ² dm for any mass distribution.
        if (!std::isfinite(f.inertia) || !(f.inertia * f.mass >= f.first_moment * f.first_moment)) {
            fail("flap: inertia must satisfy I_β ≥ S_β²/m_b (no real blade has less)");
        }
        if (!(f.spring >= 0.0) || !std::isfinite(f.spring) || !(f.damper >= 0.0) ||
            !std::isfinite(f.damper)) {
            fail("flap: spring and damper must be non-negative and finite");
        }
        if (!(std::abs(f.delta3) < 0.5 * std::numbers::pi)) {
            fail("flap: delta3 must be within ±90°");
        }
    }
    if (c.dynamic_twist &&
        (!(c.dynamic_twist->lag > 0.0) || !std::isfinite(c.dynamic_twist->lag))) {
        fail("dynamic_twist: lag must be positive and finite");
    }
}

}  // namespace

BladeElementRotor::BladeElementRotor(BladeElementRotorConfig config)
    : config_(std::move(config)),
      s_(config_.direction == RotationDirection::counter_clockwise ? 1.0 : -1.0),
      area_(std::numbers::pi * config_.radius * config_.radius),
      tip_speed_(config_.rotor_speed * config_.radius),
      C_hb_(hub_rotation(config_.hub_pitch, config_.hub_roll)),
      C_bh_(C_hb_.transpose()) {
    validate(config_);

    // Two meshes, so an element edge falls exactly on B·R (D-024).
    const BladeElementRotorConfig& c = config_;
    const double r_lift = c.tip_loss * c.radius;
    int n_tip = 0;
    if (c.tip_loss < 1.0) {
        const double share = (c.radius - r_lift) / (c.radius - c.root_cutout);
        n_tip = std::max(1, static_cast<int>(std::lround(c.num_elements * share)));
    }
    const int n_lift = c.num_elements - n_tip;
    if (n_lift < 1) {
        throw InitError("blade_element_rotor: too few elements for the tip-loss split");
    }
    std::vector<double> edges{c.root_cutout};
    add_mesh(c.root_cutout, r_lift, n_lift, c.spacing, edges);
    if (n_tip > 0) {
        add_mesh(r_lift, c.radius, n_tip, c.spacing, edges);
    }
    for (std::size_t j = 0; j + 1 < edges.size(); ++j) {
        BladeElement e;
        e.r = 0.5 * (edges[j] + edges[j + 1]);
        e.dr = edges[j + 1] - edges[j];
        e.chord = c.chord(e.r / c.radius);
        e.twist = c.twist(e.r / c.radius);
        e.lift = j < static_cast<std::size_t>(n_lift) ? 1.0 : 0.0;
        e.twist_mode = c.dynamic_twist ? c.dynamic_twist->mode(e.r / c.radius) : 0.0;
        if (!(e.chord > 0.0) || !std::isfinite(e.twist)) {
            throw InitError("blade_element_rotor: chord must be positive and twist finite at r = " +
                            std::to_string(e.r) + " m");
        }
        elements_.push_back(e);
    }
    cursors_.resize(static_cast<std::size_t>(c.num_blades) * elements_.size());
}

double BladeElementRotor::azimuth(double t) const noexcept {
    double psi = std::fmod(config_.rotor_speed * t, kTwoPi);
    if (psi < 0.0) {
        psi += kTwoPi;
    }
    return psi;
}

double BladeElementRotor::blade_azimuth(double psi, int k) const noexcept {
    return psi - kTwoPi * static_cast<double>(k) / static_cast<double>(config_.num_blades);
}

BladeElementRotor::Kinematics BladeElementRotor::kinematics(const SimContext& ctx) const noexcept {
    const BodyState& b = ctx.body;
    // Air-relative velocity of the hub: CG velocity less wind, plus ω × r.
    const Vec3 v_air_b = b.vel_b - b.q_nb.conjugate() * ctx.env.wind_n;
    const Vec3 v_hub_b = v_air_b + b.omega_b.cross(config_.hub_position_b);
    Kinematics k;
    k.v_hub_h = C_hb_ * v_hub_b;
    k.omega_h = C_hb_ * b.omega_b;
    k.psi = azimuth(ctx.t);
    k.pitch = config_.controls->pitch(ctx.controls);
    k.rho = ctx.air.density;
    k.a = ctx.air.speed_of_sound;
    const double v_xy = std::hypot(k.v_hub_h.x(), k.v_hub_h.y());
    k.mu = v_xy / tip_speed_;  // Padfield 3rd ed. Eq. 3.140
    k.mu_z = k.v_hub_h.z() / tip_speed_;
    // Downwind azimuth (D-027): the blade at ψ_w points along the in-plane
    // free stream relative to the hub, e_r(ψ_w) ∥ −v_xy.
    k.cos_w = v_xy > 0.0 ? k.v_hub_h.x() / v_xy : 1.0;
    k.sin_w = v_xy > 0.0 ? -s_ * k.v_hub_h.y() / v_xy : 0.0;
    k.airspeed = v_air_b.norm();
    return k;
}

double BladeElementRotor::twist_gain(const Kinematics& k) const noexcept {
    if (!config_.dynamic_twist) {
        return 0.0;
    }
    const DynamicTwist& d = *config_.dynamic_twist;
    return rounded(d.gain, k.airspeed, d.gain_corner_fraction, twist_gain_cursor_);
}

RotorLoads BladeElementRotor::pass(const Kinematics& k, const LinearInflow& lambda,
                                   std::span<const double> x) const noexcept {
    const BladeElementRotorConfig& c = config_;
    const double omega = c.rotor_speed;
    const bool flapping = c.flap.has_value() && !x.empty();
    const Vec3 e_z(0.0, 0.0, 1.0);
    const std::size_t n_el = elements_.size();
    const double e = hinge_offset();
    const double tan_d3 = c.flap ? std::tan(c.flap->delta3) : 0.0;
    // Elastic twist at the tip θ_e [rad] (D-038); 0 without it or with x empty.
    const double tip = c.dynamic_twist && !x.empty() ? x[twist_offset()] : 0.0;

    RotorLoads out;
    for (int blade = 0; blade < c.num_blades; ++blade) {
        const double psi = blade_azimuth(k.psi, blade);
        const double cp = std::cos(psi);
        const double sp = std::sin(psi);
        const std::size_t ib = 2 * static_cast<std::size_t>(blade);
        const double beta = flapping ? x[ib] : 0.0;
        const double beta_dot = flapping ? x[ib + 1] : 0.0;
        const double cb = std::cos(beta);
        const double sb = std::sin(beta);
        const Vec3 e_r(-cp, s_ * sp, 0.0);     // radial, in the hub plane
        const Vec3 e_t(sp, s_ * cp, 0.0);      // direction of blade motion
        const Vec3 e_s = cb * e_r - sb * e_z;  // span, flapped up by β (D-026)
        const Vec3 n = sb * e_r + cb * e_z;    // blade normal, positive down
        // A point at distance ρ outboard of the hinge is at e e_r + ρ e_s from
        // the hub center. Its velocity relative to the air, hub axes, is
        // affine in ρ:
        //   v = v_hub + ω × (e e_r) + Ω e e_t
        //     + ρ (ω × e_s + Ω cos β e_t − β̇ n)
        // U_T = v·e_t (along rotation); U_P = −v·n + v_i (e_z·n) (through the
        // blade, positive down; Leishman 2nd ed. Eq. 3.2 convention, D-024,
        // with the flapped normal, D-026). With β = 0 and e = 0 this is the
        // M3 geometry. The induced velocity is linear in the hub-plane radius
        // r = e + ρ cos β (Pitt & Peters 1980 eq. 1; D-027):
        //   v_i = ΩR (λ0 + (r/R) λ_h),  λ_h = λ1s sin ψ + λ1c cos ψ,
        // so U_P stays affine in ρ.
        const Vec3 v0 = k.v_hub_h + k.omega_h.cross(e * e_r) + (omega * e) * e_t;
        const Vec3 v1 = k.omega_h.cross(e_s) + (omega * cb) * e_t - beta_dot * n;
        const double a_T = v0.dot(e_t);
        const double b_T = v1.dot(e_t);
        const double lambda_h = lambda.l1s * sp + lambda.l1c * cp;
        const double a_P = -v0.dot(n) + tip_speed_ * cb * (lambda.l0 + e / c.radius * lambda_h);
        const double b_P = -v1.dot(n) + omega * cb * cb * lambda_h;
        // The air's velocity relative to the blade along the span, outward
        // positive (Johnson 2013 u_R; D-039): −v·e_s plus the induced
        // velocity's component, v_i e_z·e_s = −sin β v_i. Affine in ρ too.
        const double a_S = -v0.dot(e_s) - sb * tip_speed_ * (lambda.l0 + e / c.radius * lambda_h);
        const double b_S = -v1.dot(e_s) - sb * omega * cb * lambda_h;
        // Padfield 3rd ed. Eq. 3.59, with pitch–flap coupling −tan δ₃ β
        // (Johnson 2013 §6.18); twist is added per element (spec §6.3).
        const double theta =
            k.pitch.theta0 + k.pitch.theta1c * cp + k.pitch.theta1s * sp - tan_d3 * beta;

        double f_t = 0.0;   // Σ force along e_t [N]
        double f_n = 0.0;   // Σ force along n [N]
        double rf_t = 0.0;  // Σ ρ × force along e_t [N·m]
        double rf_n = 0.0;  // Σ ρ × force along n [N·m]
        double f_s = 0.0;   // Σ radial drag along e_s [N] (D-039)
        double rf_s = 0.0;  // Σ ρ × radial drag [N·m]
        TableCursor<2>* cursor = cursors_.data() + static_cast<std::size_t>(blade) * n_el;
        for (std::size_t j = 0; j < n_el; ++j) {
            const BladeElement& el = elements_[j];
            const double rho_j = el.r - e;  // [m] outboard of the hinge
            const double u_t = a_T + b_T * rho_j;
            const double u_p = a_P + b_P * rho_j;
            // Leishman 2nd ed. Eqs. 3.1–3.3; φ in all quadrants.
            const double u = std::sqrt(u_t * u_t + u_p * u_p);
            const double phi = std::atan2(u_p, u_t);
            // Elastic twist added to the preset twist (CR-166309 p. 5.1-25; D-038).
            double alpha = theta + el.twist + tip * el.twist_mode - phi;
            if (alpha >= std::numbers::pi) {
                alpha -= kTwoPi;
            } else if (alpha < -std::numbers::pi) {
                alpha += kTwoPi;
            }
            const AirfoilCoefficients coef = (*c.airfoil)(alpha, u / k.a, cursor[j]);
            const double cl = el.lift * coef.cl;
            // Lift ⟂ and drag ∥ to the relative wind (Leishman 2nd ed. Eqs. 3.4–3.5),
            // in vector form: with the relative wind d = (−U_T e_t + U_P n)/U
            // and lift direction l = (−U_P e_t − U_T n)/U,
            //   dF = ½ρU²c dr (C_l l + C_d d).
            const double q = 0.5 * k.rho * el.chord * el.dr * u;
            // Yawed-flow drag (Johnson 2013 Eqs. 6.397–6.399; D-039): the drag
            // force is ½ρc dr c_d W along the full relative wind (−u_T e_t +
            // u_P n + u_R e_s), W² = U² + u_R²; without it, W = U and u_R = 0.
            double q_d = q;
            if (c.yawed_flow_drag) {
                const double u_r = a_S + b_S * rho_j;
                q_d = 0.5 * k.rho * el.chord * el.dr * std::sqrt(u * u + u_r * u_r);
                const double dfs = q_d * coef.cd * u_r;
                f_s += dfs;
                rf_s += rho_j * dfs;
            }
            const double dft = -q * cl * u_p - q_d * coef.cd * u_t;
            const double dfn = -q * cl * u_t + q_d * coef.cd * u_p;
            f_t += dft;
            f_n += dfn;
            rf_t += rho_j * dft;
            rf_n += rho_j * dfn;
        }
        const Vec3 F = f_t * e_t + f_n * n + f_s * e_s;
        out.F_h += F;
        // GenHel's F_PDYMR, the blade's force in blade span axes (p. 5.1-24).
        out.blade_force += std::hypot(f_t, f_n);
        // About the hub center: Σ (e e_r + ρ e_s) × dF (radial drag along e_s
        // has no moment about the hinge line, only through e e_r).
        out.M_h += (e * e_r).cross(F) + e_s.cross(rf_t * e_t + rf_n * n);
        // About the hinge, positive flap up: −Σ ρ n·dF (D-026).
        out.flap_moment[static_cast<std::size_t>(blade)] = -rf_n;
        // Σ dF_up r for the Pitt–Peters loads (D-027): dF_up = −cos β dF·n
        // + sin β dF·e_s (e_t is in the hub plane; e_s·(−e_z) = sin β),
        // r = e + ρ cos β.
        const double fr_up = -cb * (e * f_n + cb * rf_n) + sb * (e * f_s + cb * rf_s);
        out.c1s += fr_up * sp;
        out.c1c += fr_up * cp;
    }

    out.blade_force /= static_cast<double>(c.num_blades);
    const double q_disk = k.rho * area_ * tip_speed_ * tip_speed_;  // ρA(ΩR)² [N]
    out.thrust = -out.F_h.z();
    out.torque = s_ * out.M_h.z();  // rotation axis is −s e_z
    out.power = out.torque * omega;
    out.ct = out.thrust / q_disk;
    out.cq = out.torque / (q_disk * c.radius);
    out.c1s /= q_disk * c.radius;
    out.c1c /= q_disk * c.radius;
    out.lambda_i = lambda.l0;
    out.lambda_1s = lambda.l1s;
    out.lambda_1c = lambda.l1c;
    return out;
}

void BladeElementRotor::finish(RotorLoads& loads, const Kinematics& k) const noexcept {
    loads.mu = k.mu;
    loads.mu_z = k.mu_z;
    loads.body.F = C_bh_ * loads.F_h;
    loads.body.M = C_bh_ * loads.M_h + config_.hub_position_b.cross(loads.body.F);
}

RotorLoads BladeElementRotor::evaluate_at_inflow(const SimContext& ctx, double lambda_i,
                                                 std::span<const double> x) const noexcept {
    return evaluate_at_inflow(ctx, LinearInflow{lambda_i, 0.0, 0.0}, x);
}

RotorLoads BladeElementRotor::evaluate_at_inflow(const SimContext& ctx, const LinearInflow& lambda,
                                                 std::span<const double> x) const noexcept {
    const Kinematics k = kinematics(ctx);
    RotorLoads loads = pass(k, lambda, x);
    loads.passes = 1;
    finish(loads, k);
    return loads;
}

RotorLoads BladeElementRotor::evaluate(const SimContext& ctx,
                                       std::span<const double> x) const noexcept {
    return evaluate_at(kinematics(ctx), x);
}

RotorLoads BladeElementRotor::evaluate_at(const Kinematics& k,
                                          std::span<const double> x) const noexcept {
    assert(x.empty() || x.size() == num_states());
    RotorLoads out;
    if (config_.inflow == InflowModel::pitt_peters) {
        // One pass at the inflow states (D-027); nothing to iterate.
        LinearInflow lambda;
        if (!x.empty()) {
            const std::size_t i = inflow_offset();
            lambda = {x[i], x[i + 1], x[i + 2]};
        }
        out = pass(k, lambda, x);
        out.passes = 1;
    } else {
        out = solve_uniform(k, x);
    }
    finish(out, k);
    return out;
}

RotorLoads BladeElementRotor::evaluate_uniform(const SimContext& ctx,
                                               std::span<const double> x) const noexcept {
    const Kinematics k = kinematics(ctx);
    RotorLoads out = solve_uniform(k, x);
    finish(out, k);
    return out;
}

RotorLoads BladeElementRotor::solve_uniform(const Kinematics& k,
                                            std::span<const double> x) const noexcept {
    const double mu = k.mu;
    const double mu_z = k.mu_z;
    // Uniform momentum inflow, Padfield 3rd ed. Eq. 3.139, multiplied out so
    // it has no singularity at λ_i = 0:
    //   g(λ) = 2λ √(μ² + (λ − μ_z)²) − C_T(λ) = 0,
    // with C_T from the blade-element pass at λ. A root always exists: g → ±∞
    // as λ → ±∞ while C_T stays bounded.
    int passes = 0;
    const auto eval = [&](double lambda, RotorLoads& l) {
        l = pass(k, LinearInflow{lambda, 0.0, 0.0}, x);
        ++passes;
        const double d = lambda - mu_z;
        return 2.0 * lambda * std::sqrt(mu * mu + d * d) - l.ct;
    };

    // Bracket from a fixed start, with no warm start from earlier calls
    // (purity, D-024): λ = 0, then the momentum estimate for that pass's C_T
    // (exact in hover and at high μ for a fixed C_T), then steps doubling away
    // from 0 until g changes sign.
    RotorLoads la;
    RotorLoads lb;
    double a = 0.0;
    double fa = eval(a, la);
    bool converged = fa == 0.0;
    RotorLoads* result = &la;
    if (!converged) {
        const double c0 = la.ct;
        double b = c0 / (2.0 * std::sqrt(mu * mu + mu_z * mu_z + 0.5 * std::abs(c0)));
        double fb = eval(b, lb);
        result = &lb;
        while ((fa > 0.0) == (fb > 0.0) && fb != 0.0 && passes < kMaxPasses) {
            const double step = 2.0 * (b - a);
            a = b;
            fa = fb;
            la = lb;
            b += step;
            fb = eval(b, lb);
        }
        converged = fb == 0.0;
        if (!converged && (fa > 0.0) != (fb > 0.0)) {
            // Brent's method (Brent, "Algorithms for Minimization without
            // Derivatives", 1973, Ch. 4): inverse quadratic interpolation or
            // secant steps, falling back to bisection, always inside the
            // bracket [b, c]. Superlinear, and never slower than bisection.
            RotorLoads lc = la;
            double c = a;
            double fc = fa;
            double d = b - a;
            double e = d;
            while (passes < kMaxPasses) {
                if ((fb > 0.0) == (fc > 0.0)) {
                    c = a;
                    fc = fa;
                    lc = la;
                    d = b - a;
                    e = d;
                }
                if (std::abs(fc) < std::abs(fb)) {
                    a = b;
                    b = c;
                    c = a;
                    fa = fb;
                    fb = fc;
                    fc = fa;
                    la = lb;
                    lb = lc;
                    lc = la;
                }
                const double tol = 2.0 * std::numeric_limits<double>::epsilon() * std::abs(b) +
                                   0.5 * kInflowTolerance;
                const double xm = 0.5 * (c - b);
                if (std::abs(xm) <= tol || fb == 0.0) {
                    converged = true;
                    break;
                }
                if (std::abs(e) >= tol && std::abs(fa) > std::abs(fb)) {
                    const double sr = fb / fa;
                    double p = 0.0;
                    double q = 0.0;
                    if (a == c) {
                        p = 2.0 * xm * sr;
                        q = 1.0 - sr;
                    } else {
                        const double qa = fa / fc;
                        const double r = fb / fc;
                        p = sr * (2.0 * xm * qa * (qa - r) - (b - a) * (r - 1.0));
                        q = (qa - 1.0) * (r - 1.0) * (sr - 1.0);
                    }
                    if (p > 0.0) {
                        q = -q;
                    }
                    p = std::abs(p);
                    if (2.0 * p < std::min(3.0 * xm * q - std::abs(tol * q), std::abs(e * q))) {
                        e = d;
                        d = p / q;
                    } else {
                        d = xm;
                        e = d;
                    }
                } else {
                    d = xm;
                    e = d;
                }
                a = b;
                fa = fb;
                la = lb;
                b += std::abs(d) > tol ? d : std::copysign(tol, xm);
                fb = eval(b, lb);
            }
        }
    }
    RotorLoads out = *result;
    out.passes = passes;
    out.converged = converged;
    return out;
}

std::size_t BladeElementRotor::inflow_offset() const noexcept {
    return config_.flap ? 2 * static_cast<std::size_t>(config_.num_blades) : 0;
}

std::size_t BladeElementRotor::twist_offset() const noexcept {
    return inflow_offset() + (config_.inflow == InflowModel::pitt_peters ? 3 : 0);
}

std::size_t BladeElementRotor::num_states() const {
    return twist_offset() + (config_.dynamic_twist ? 1 : 0);
}

void BladeElementRotor::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
}

std::vector<std::string> BladeElementRotor::produces() const {
    if (config_.name.empty()) {
        return {};
    }
    const std::string& n = config_.name;
    return {n + ".lambda0", n + ".mu_x", n + ".mu_z", n + ".beta_1c", n + ".tip_speed"};
}

void BladeElementRotor::bind(const CouplingRegistry& registry) {
    if (config_.name.empty()) {
        return;
    }
    const std::string& n = config_.name;
    slots_.lambda0 = registry.find(n + ".lambda0", n);
    slots_.mu_x = registry.find(n + ".mu_x", n);
    slots_.mu_z = registry.find(n + ".mu_z", n);
    slots_.beta_1c = registry.find(n + ".beta_1c", n);
    slots_.tip_speed = registry.find(n + ".tip_speed", n);
}

void BladeElementRotor::derivatives(const SimContext& ctx, std::span<const double> x,
                                    ComponentDerivs& out) const {
    const Kinematics kin = kinematics(ctx);
    const RotorLoads loads = evaluate_at(kin, x);
    out.load0_b = loads.body;
    out.converged = loads.converged;
    if (out.coupling != nullptr && slots_.lambda0.valid()) {
        const double lambda0 =
            config_.inflow == InflowModel::pitt_peters ? x[inflow_offset()] : loads.lambda_i;
        const double beta_1c =
            config_.flap ? multiblade_flap(x, kin.psi, config_.num_blades).beta1c : 0.0;
        out.coupling->set(slots_.lambda0, lambda0);
        out.coupling->set(slots_.mu_x, kin.v_hub_h.x() / tip_speed_);
        out.coupling->set(slots_.mu_z, kin.mu_z);
        out.coupling->set(slots_.beta_1c, beta_1c);
        out.coupling->set(slots_.tip_speed, tip_speed_);
    }
    if (config_.inflow == InflowModel::pitt_peters) {
        // Pitt–Peters inflow (D-027), forced by the loads of the same pass.
        // Independent of the body acceleration: the G rows stay zero.
        const std::size_t i = inflow_offset();
        const Vec3 rates =
            pitt_peters_rates(Vec3(x[i], x[i + 1], x[i + 2]), Vec3(loads.ct, loads.c1s, loads.c1c),
                              kin.mu, kin.mu_z, kin.cos_w, kin.sin_w, config_.rotor_speed);
        out.xdot0[i] = rates.x();
        out.xdot0[i + 1] = rates.y();
        out.xdot0[i + 2] = rates.z();
    }
    if (config_.dynamic_twist) {
        // The tip twist lags GenHel's θ_DYTIP = K_FP0 F_P0 (CR-166309
        // p. 5.1-24; D-038); independent of the body acceleration, so its G
        // row stays zero.
        const std::size_t i = twist_offset();
        out.xdot0[i] = (twist_gain(kin) * loads.blade_force - x[i]) / config_.dynamic_twist->lag;
    }
    if (!config_.flap) {
        return;
    }

    // Rigid flapping blades (D-026). A blade point at distance ρ outboard of
    // the hinge is at p = p0 + ρ e_s from the airframe CG (hub axes), with
    // p0 = r_hub + e e_r. Hub axes are fixed in the body, so its inertial
    // acceleration is (transport theorem)
    //   a_p = a_cg + ω̇ × p + ω × (ω × p) + 2 ω × ṗ_rel + p̈_rel,
    // a_cg = v̇_b + ω × v_b. Relative to the hub, e_r turns at Ω about −s e_z
    // (ė_r = Ω e_t, ė_t = −Ω e_r) and the blade flaps about the hinge:
    //   ṗ_rel = Ω e e_t + ρ (Ω cos β e_t − β̇ n)
    //   p̈_rel = −Ω² e e_r + ρ (−Ω² cos β e_r − 2Ω β̇ sin β e_t − β̇² e_s − β̈ n).
    // Everything is affine in ρ, a_p = c0 + ρ c1 − ρ β̈ n, so the span
    // integrals need only m = ∫dm, S = ∫ρ dm and I = ∫ρ² dm. The flap
    // equation is the virtual work for δp = −ρ n δβ (moments about the hinge):
    //   I β̈ = n·(S c0 + I c1) + M_aero − S n·g − K_β β − C_β β̇,
    // and the load on the airframe is the blade's free-body balance,
    // Σ (dF_aero + g dm − a_p dm) and its moment about the CG; the hinge
    // spring and damper are internal and cancel. The terms in ω̇ and v̇ give
    // G and H (spec §5, "Coupling"). Linearized at β = 0 this reduces to
    // Johnson (2013) Eqs. 16.240 and 6.261; tests check both.
    const FlapHinge& f = *config_.flap;
    const double W = config_.rotor_speed;
    const double e = f.offset;
    const double m = f.mass;
    const double S = f.first_moment;
    const double I = f.inertia;
    const BodyState& b = ctx.body;
    const Vec3 e_z(0.0, 0.0, 1.0);
    const Vec3 w = C_hb_ * b.omega_b;  // hub-frame angular velocity (D-011: + ω_hb later)
    const Vec3 a_cg0 = C_hb_ * b.omega_b.cross(b.vel_b);
    const Vec3 g = C_hb_ * (b.q_nb.conjugate() * gravity_n());
    const Vec3 r_hub = C_hb_ * config_.hub_position_b;
    const double psi0 = azimuth(ctx.t);

    using Mat36 = Eigen::Matrix<double, 3, 6>;
    const auto skew = [](const Vec3& v) {
        Mat3 k;
        k << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return k;
    };
    Vec3 F = Vec3::Zero();  // hub axes, about the airframe CG, at a = 0
    Vec3 M = Vec3::Zero();
    Mat36 FA = Mat36::Zero();  // ∂F/∂a, ∂M/∂a (hub axes)
    Mat36 MA = Mat36::Zero();
    for (int blade = 0; blade < config_.num_blades; ++blade) {
        const std::size_t ib = 2 * static_cast<std::size_t>(blade);
        const double beta = x[ib];
        const double bd = x[ib + 1];
        const double psi = blade_azimuth(psi0, blade);
        const double cp = std::cos(psi);
        const double sp = std::sin(psi);
        const double cb = std::cos(beta);
        const double sb = std::sin(beta);
        const Vec3 e_r(-cp, s_ * sp, 0.0);
        const Vec3 e_t(sp, s_ * cp, 0.0);
        const Vec3 e_s = cb * e_r - sb * e_z;
        const Vec3 n = sb * e_r + cb * e_z;
        const Vec3 p0 = r_hub + e * e_r;

        const Vec3 v_rel1 = (W * cb) * e_t - bd * n;
        const Vec3 c0 =
            a_cg0 + w.cross(w.cross(p0)) + (2.0 * W * e) * w.cross(e_t) - (W * W * e) * e_r;
        const Vec3 c1 = w.cross(w.cross(e_s)) + 2.0 * w.cross(v_rel1) - (W * W * cb) * e_r -
                        (2.0 * W * sb * bd) * e_t - (bd * bd) * e_s;
        Mat36 C0a;
        C0a << C_hb_, -skew(p0) * C_hb_;
        Mat36 C1a;
        C1a << Mat3::Zero(), -skew(e_s) * C_hb_;

        const Vec3 K0 = S * c0 + I * c1;
        const Mat36 KA = S * C0a + I * C1a;
        const double bdd0 = (n.dot(K0) + loads.flap_moment[static_cast<std::size_t>(blade)] -
                             S * n.dot(g) - f.spring * beta - f.damper * bd) /
                            I;
        const Eigen::Matrix<double, 1, 6> Gb = n.transpose() * KA / I;
        out.xdot0[ib] = bd;
        out.xdot0[ib + 1] = bdd0;
        for (std::size_t j = 0; j < 6; ++j) {
            out.G[(ib + 1) * 6 + j] = Gb(0, static_cast<Eigen::Index>(j));
        }

        // ∫ a_p dm and ∫ ρ a_p dm, split into a = 0 values and sensitivities.
        const Vec3 Fi0 = m * c0 + S * c1 - (S * bdd0) * n;
        const Mat36 FiA = m * C0a + S * C1a - S * n * Gb;
        const Vec3 Ki0 = K0 - (I * bdd0) * n;
        const Mat36 KiA = KA - I * n * Gb;
        F += m * g - Fi0;
        M += (m * p0 + S * e_s).cross(g) - p0.cross(Fi0) - e_s.cross(Ki0);
        FA -= FiA;
        MA -= skew(p0) * FiA + skew(e_s) * KiA;
    }
    out.load0_b.F += C_bh_ * F;
    out.load0_b.M += C_bh_ * M;
    out.H.topRows<3>() = C_bh_ * FA;
    out.H.bottomRows<3>() = C_bh_ * MA;
}

// ---- Config ------------------------------------------------------------------

namespace {

using units::Dimension;

// A constant (2-point) table against r/R.
Table1D constant_table(double v) {
    return Table1D(Axis({0.0, 1.0}), {v, v});
}

// Reads `key` (a constant, or for twist a linear θ_tw) or `key_table` (a CSV
// with columns r [1] and `column`).
Table1D read_spanwise(const Config& blade, const std::string& key, Dimension d, bool linear) {
    const std::string table_key = key + "_table";
    const bool has_value = blade.contains(key);
    const bool has_table = blade.contains(table_key);
    if (has_value == has_table) {
        blade.fail(key, "give exactly one of " + key + " and " + table_key);
    }
    if (has_value) {
        const double v = blade.quantity(key, d);
        return linear ? Table1D(Axis({0.0, 1.0}), {0.0, v}) : constant_table(v);
    }
    return blade.in_context(table_key, [&] {
        const CsvTable csv = blade.csv_table(table_key);
        csv.require_dimension("r", Dimension::dimensionless);
        csv.require_dimension(key, d);
        return make_table1d(csv, "r", key);
    });
}

}  // namespace

std::unique_ptr<BladeElementRotor> BladeElementRotor::from_config(const Config& node) {
    node.allow_keys({"type", "name", "blades", "radius", "rotor_speed", "direction", "inflow",
                     "hub", "blade", "controls"});
    BladeElementRotorConfig c;
    if (node.contains("name")) {
        c.name = node.string("name");
        if (c.name.empty()) {
            node.fail("name", "must not be empty");
        }
    }
    const std::int64_t blades = node.integer("blades");
    if (blades < 1 || blades > kMaxBlades) {
        node.fail("blades", "must be between 1 and " + std::to_string(kMaxBlades));
    }
    c.num_blades = static_cast<int>(blades);
    c.radius = node.quantity("radius", Dimension::length);
    c.rotor_speed = node.quantity("rotor_speed", Dimension::angular_rate);
    const std::string dir = node.string("direction");
    if (dir == "counter_clockwise") {
        c.direction = RotationDirection::counter_clockwise;
    } else if (dir == "clockwise") {
        c.direction = RotationDirection::clockwise;
    } else {
        node.fail("direction", "must be counter_clockwise or clockwise (viewed from above)");
    }
    // Required, no default (D-021, D-027).
    const std::string inflow = node.string("inflow");
    if (inflow == "uniform") {
        c.inflow = InflowModel::uniform;
    } else if (inflow == "pitt_peters") {
        c.inflow = InflowModel::pitt_peters;
    } else {
        node.fail("inflow", "must be uniform or pitt_peters");
    }

    const Config hub = node.child("hub");
    hub.allow_keys({"position", "pitch", "roll"});
    const Config pos = hub.child("position");
    pos.allow_keys({"x", "y", "z"});
    c.hub_position_b =
        Vec3(pos.quantity("x", Dimension::length), pos.quantity("y", Dimension::length),
             pos.quantity("z", Dimension::length));
    c.hub_pitch = hub.contains("pitch") ? hub.quantity("pitch", Dimension::angle) : 0.0;
    c.hub_roll = hub.contains("roll") ? hub.quantity("roll", Dimension::angle) : 0.0;

    const Config blade = node.child("blade");
    blade.allow_keys({"root_cutout", "tip_loss", "chord", "chord_table", "twist", "twist_table",
                      "airfoil", "elements", "spacing", "flap", "dynamic_twist",
                      "yawed_flow_drag"});
    c.root_cutout = blade.quantity("root_cutout", Dimension::length);
    c.tip_loss = blade.number("tip_loss");
    c.chord = read_spanwise(blade, "chord", Dimension::length, false);
    c.twist = read_spanwise(blade, "twist", Dimension::angle, true);
    c.yawed_flow_drag = blade.contains("yawed_flow_drag") && blade.boolean("yawed_flow_drag");
    if (blade.contains("dynamic_twist")) {
        // Elastic twist (D-038): gain_table (columns airspeed and gain),
        // mode or mode_table (against r/R), lag.
        const Config dt = blade.child("dynamic_twist");
        dt.allow_keys({"gain_table", "mode", "mode_table", "lag"});
        DynamicTwist d;
        d.gain = dt.in_context("gain_table", [&] {
            const CsvTable csv = dt.csv_table("gain_table");
            csv.require_dimension("airspeed", Dimension::speed);
            csv.require_dimension("gain", Dimension::angle_per_force);
            return make_table1d(csv, "airspeed", "gain");
        });
        d.mode = read_spanwise(dt, "mode", Dimension::dimensionless, false);
        d.lag = dt.quantity("lag", Dimension::time);
        c.dynamic_twist = d;
    }
    c.airfoil = blade.in_context("airfoil", [&] {
        return std::make_shared<const Airfoil>(Airfoil::from_csv(blade.csv_table("airfoil")));
    });
    const std::int64_t elements = blade.integer("elements");
    if (elements < 1 || elements > 1000) {
        blade.fail("elements", "must be between 1 and 1000");
    }
    c.num_elements = static_cast<int>(elements);
    const std::string spacing = blade.contains("spacing") ? blade.string("spacing") : "equal_area";
    if (spacing == "equal_area") {
        c.spacing = ElementSpacing::equal_area;
    } else if (spacing == "uniform") {
        c.spacing = ElementSpacing::uniform;
    } else {
        blade.fail("spacing", "must be equal_area or uniform");
    }
    if (blade.contains("flap")) {
        // Rigid flapping blade (D-026). Mass, first and second moments are
        // about the hinge, outboard of it (GenHel W_b, M_b, I_b; a weight in
        // lbf is written as the same number in lbm).
        const Config flap = blade.child("flap");
        flap.allow_keys(
            {"hinge_offset", "mass", "first_moment", "inertia", "spring", "damper", "delta3"});
        FlapHinge h;
        h.offset = flap.quantity("hinge_offset", Dimension::length);
        h.mass = flap.quantity("mass", Dimension::mass);
        h.first_moment = flap.quantity("first_moment", Dimension::mass_moment);
        h.inertia = flap.quantity("inertia", Dimension::inertia);
        h.spring = flap.contains("spring")
                       ? flap.quantity("spring", Dimension::rotational_stiffness)
                       : 0.0;
        h.damper =
            flap.contains("damper") ? flap.quantity("damper", Dimension::rotational_damping) : 0.0;
        h.delta3 = flap.contains("delta3") ? flap.quantity("delta3", Dimension::angle) : 0.0;
        c.flap = h;
    }

    c.controls = LinearControlMap::from_config(node.child("controls"));
    return node.in_context("", [&] { return std::make_unique<BladeElementRotor>(std::move(c)); });
}

}  // namespace eyas
