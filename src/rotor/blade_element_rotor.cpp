#include "rotor/blade_element_rotor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>

#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// Hub-frame Euler angles (pitch θ_h, roll φ_h; no yaw) to C_hb = Φ Θ, the
// same rotation sequence as the body attitude (Padfield 3rd ed. Eq. 3A.37
// with ψ = 0).
Mat3 hub_rotation(double pitch, double roll) {
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    Mat3 theta;
    theta << cp, 0.0, -sp, 0.0, 1.0, 0.0, sp, 0.0, cp;
    Mat3 phi;
    phi << 1.0, 0.0, 0.0, 0.0, cr, sr, 0.0, -sr, cr;
    return phi * theta;
}

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
    if (c.num_blades < 1) {
        fail("blades must be at least 1");
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
    return k;
}

RotorLoads BladeElementRotor::pass(const Kinematics& k, double lambda_i) const noexcept {
    const BladeElementRotorConfig& c = config_;
    const double omega = c.rotor_speed;
    const double v_i = lambda_i * tip_speed_;  // [m/s] induced, down along z_h
    const Vec3 e_z(0.0, 0.0, 1.0);
    const std::size_t n_el = elements_.size();

    RotorLoads out;
    for (int blade = 0; blade < c.num_blades; ++blade) {
        const double psi = blade_azimuth(k.psi, blade);
        const double cp = std::cos(psi);
        const double sp = std::sin(psi);
        const Vec3 e_r(-cp, s_ * sp, 0.0);  // spanwise, outward
        const Vec3 e_t(sp, s_ * cp, 0.0);   // direction of blade motion
        // Element velocity relative to the air, hub axes:
        //   v = v_hub + ω × (r e_r) + Ω r e_t
        // U_T = v·e_t (in-plane, along rotation); U_P = −v·e_z + v_i (through
        // the disc, positive down; Leishman 2nd ed. Eq. 3.2 convention, D-024).
        // Both are affine in r: U = a + b r.
        const Vec3 w_er = k.omega_h.cross(e_r);
        const double a_T = k.v_hub_h.dot(e_t);
        const double b_T = omega + w_er.dot(e_t);
        const double a_P = -k.v_hub_h.dot(e_z) + v_i;
        const double b_P = -w_er.dot(e_z);
        // Padfield 3rd ed. Eq. 3.59; twist is added per element (spec §6.3).
        const double theta = k.pitch.theta0 + k.pitch.theta1c * cp + k.pitch.theta1s * sp;

        double f_t = 0.0;   // Σ force along e_t [N]
        double f_z = 0.0;   // Σ force along e_z [N]
        double rf_t = 0.0;  // Σ r × force along e_t [N·m]
        double rf_z = 0.0;  // Σ r × force along e_z [N·m]
        TableCursor<2>* cursor = cursors_.data() + static_cast<std::size_t>(blade) * n_el;
        for (std::size_t j = 0; j < n_el; ++j) {
            const BladeElement& e = elements_[j];
            const double u_t = a_T + b_T * e.r;
            const double u_p = a_P + b_P * e.r;
            const double u = std::sqrt(u_t * u_t + u_p * u_p);  // Leishman Eq. 3.1
            const double phi = std::atan2(u_p, u_t);            // Leishman Eq. 3.2, all quadrants
            double alpha = theta + e.twist - phi;               // Leishman Eq. 3.3
            if (alpha >= std::numbers::pi) {
                alpha -= kTwoPi;
            } else if (alpha < -std::numbers::pi) {
                alpha += kTwoPi;
            }
            const AirfoilCoefficients coef = (*c.airfoil)(alpha, u / k.a, cursor[j]);
            const double cl = e.lift * coef.cl;
            // Lift ⟂ and drag ∥ to the relative wind (Leishman Eqs. 3.4–3.5),
            // in vector form: with the relative wind d = (−U_T e_t + U_P e_z)/U
            // and lift direction l = (−U_P e_t − U_T e_z)/U,
            //   dF = ½ρU²c dr (C_l l + C_d d).
            const double q = 0.5 * k.rho * e.chord * e.dr * u;
            const double dft = q * (-cl * u_p - coef.cd * u_t);
            const double dfz = q * (-cl * u_t + coef.cd * u_p);
            f_t += dft;
            f_z += dfz;
            rf_t += e.r * dft;
            rf_z += e.r * dfz;
        }
        out.F_h += f_t * e_t + f_z * e_z;
        out.M_h += rf_t * e_r.cross(e_t) + rf_z * e_r.cross(e_z);
    }

    const double q_disk = k.rho * area_ * tip_speed_ * tip_speed_;  // ρA(ΩR)² [N]
    out.thrust = -out.F_h.z();
    out.torque = s_ * out.M_h.z();  // rotation axis is −s e_z
    out.power = out.torque * omega;
    out.ct = out.thrust / q_disk;
    out.cq = out.torque / (q_disk * c.radius);
    out.lambda_i = lambda_i;
    out.passes = 1;
    return out;
}

void BladeElementRotor::finish(RotorLoads& loads, const Kinematics& k) const noexcept {
    loads.mu = std::hypot(k.v_hub_h.x(), k.v_hub_h.y()) / tip_speed_;  // Padfield Eq. 3.140
    loads.mu_z = k.v_hub_h.z() / tip_speed_;
    loads.body.F = C_bh_ * loads.F_h;
    loads.body.M = C_bh_ * loads.M_h + config_.hub_position_b.cross(loads.body.F);
}

RotorLoads BladeElementRotor::evaluate_at_inflow(const SimContext& ctx,
                                                 double lambda_i) const noexcept {
    const Kinematics k = kinematics(ctx);
    RotorLoads loads = pass(k, lambda_i);
    finish(loads, k);
    return loads;
}

RotorLoads BladeElementRotor::evaluate(const SimContext& ctx) const noexcept {
    const Kinematics k = kinematics(ctx);
    const double mu = std::hypot(k.v_hub_h.x(), k.v_hub_h.y()) / tip_speed_;
    const double mu_z = k.v_hub_h.z() / tip_speed_;
    // Uniform momentum inflow, Padfield 3rd ed. Eq. 3.139, multiplied out so
    // it has no singularity at λ_i = 0:
    //   g(λ) = 2λ √(μ² + (λ − μ_z)²) − C_T(λ) = 0,
    // with C_T from the blade-element pass at λ. A root always exists: g → ±∞
    // as λ → ±∞ while C_T stays bounded.
    int passes = 0;
    const auto eval = [&](double lambda, RotorLoads& l) {
        l = pass(k, lambda);
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
    finish(out, k);
    return out;
}

void BladeElementRotor::derivatives(const SimContext& ctx, std::span<const double> /*x*/,
                                    ComponentDerivs& out) const {
    const RotorLoads loads = evaluate(ctx);
    out.load0_b = loads.body;
    out.converged = loads.converged;
}

// ---- Config ------------------------------------------------------------------

namespace {

using units::Dimension;

// A constant (2-point) table against r/R.
Table1D constant_table(double v) {
    return Table1D(Axis({0.0, 1.0}), {v, v});
}

// Reports a CSV table marked "# status: placeholder" as a data gap (D-025).
void note_if_placeholder(const Config& node, std::string_view key, const CsvTable& csv) {
    if (csv.is_placeholder()) {
        node.note_placeholder(key, csv.origin + " (# status: placeholder)");
    }
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
    try {
        const CsvTable csv = read_csv_table(blade.path(table_key));
        csv.require_dimension("r", Dimension::dimensionless);
        csv.require_dimension(key, d);
        note_if_placeholder(blade, table_key, csv);
        return make_table1d(csv, "r", key);
    } catch (const InitError& e) {
        blade.fail(table_key, e.what());
    }
}

}  // namespace

std::unique_ptr<BladeElementRotor> BladeElementRotor::from_config(const Config& node) {
    node.allow_keys(
        {"type", "blades", "radius", "rotor_speed", "direction", "hub", "blade", "controls"});
    BladeElementRotorConfig c;
    const std::int64_t blades = node.integer("blades");
    if (blades < 1 || blades > 64) {
        node.fail("blades", "must be between 1 and 64");
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
                      "airfoil", "elements", "spacing"});
    c.root_cutout = blade.quantity("root_cutout", Dimension::length);
    c.tip_loss = blade.number("tip_loss");
    c.chord = read_spanwise(blade, "chord", Dimension::length, false);
    c.twist = read_spanwise(blade, "twist", Dimension::angle, true);
    try {
        const CsvTable csv = read_csv_table(blade.path("airfoil"));
        c.airfoil = std::make_shared<const Airfoil>(Airfoil::from_csv(csv));
        note_if_placeholder(blade, "airfoil", csv);
    } catch (const InitError& e) {
        blade.fail("airfoil", e.what());
    }
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

    c.controls = LinearControlMap::from_config(node.child("controls"));
    try {
        return std::make_unique<BladeElementRotor>(std::move(c));
    } catch (const InitError& e) {
        node.fail("", e.what());
    }
}

}  // namespace eyas
