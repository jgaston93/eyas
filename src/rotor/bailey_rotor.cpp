#include "rotor/bailey_rotor.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

#include "rotor/hub_frame.hpp"
#include "rotor/pitt_peters.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

void validate(const BaileyRotorConfig& c) {
    const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
    if (c.num_blades < 1) {
        throw InitError("bailey rotor: blades must be at least 1");
    }
    if (!positive(c.radius) || !positive(c.rotor_speed) || !positive(c.chord) ||
        !positive(c.lift_slope)) {
        throw InitError("bailey rotor: radius, rotor speed, chord and lift slope must be positive");
    }
    if (!(c.tip_loss > 0.0) || c.tip_loss > 1.0) {
        throw InitError("bailey rotor: tip loss must be in (0, 1]");
    }
    if (!std::isfinite(c.twist) || !std::isfinite(c.pitch_bias)) {
        throw InitError("bailey rotor: twist and pitch bias must be finite");
    }
    if (!(std::abs(c.delta3) < 0.5 * std::numbers::pi)) {
        throw InitError("bailey rotor: delta3 must be within (-90, 90) deg");
    }
    if (!(c.coning_per_thrust >= 0.0) || !std::isfinite(c.coning_per_thrust)) {
        throw InitError("bailey rotor: coning per thrust must be finite and not negative");
    }
    // With δ₃ < 0, coning raises pitch: the closed-form denominator
    // 1 + (aσ/2) t₃₂ k falls with ρ and μ and has no bound that holds at
    // init, so it could reach zero in flight (D-028).
    if (c.coning_per_thrust > 0.0 && c.delta3 < 0.0) {
        throw InitError("bailey rotor: delta3 must not be negative");
    }
    if (!(c.drag_area >= 0.0) || !std::isfinite(c.drag_area)) {
        throw InitError("bailey rotor: drag area must be finite and not negative");
    }
    if (!(c.blockage_factor > 0.0) || c.blockage_factor > 1.0) {
        throw InitError("bailey rotor: blockage factor must be in (0, 1]");
    }
    if (!(c.blockage_speed >= 0.0) || !std::isfinite(c.blockage_speed)) {
        throw InitError("bailey rotor: blockage speed must be finite and not negative");
    }
    if (!c.hub_position_b.allFinite() || !std::isfinite(c.hub_pitch) ||
        !std::isfinite(c.hub_roll)) {
        throw InitError("bailey rotor: hub position and angles must be finite");
    }
    if (!c.controls && c.pitch_input.empty()) {
        throw InitError("bailey rotor: no control map or pitch input");
    }
    if (c.controls && !c.pitch_input.empty()) {
        throw InitError("bailey rotor: give either a control map or a pitch input, not both");
    }
    if (c.fuselage_wake) {
        if (c.fuselage_wake->airframe.empty()) {
            throw InitError("bailey rotor: the fuselage wake needs the airframe's name");
        }
        if (!(c.hub_position_b.x() < 0.0)) {
            throw InitError("bailey rotor: a fuselage wake needs the hub aft of the CG (x < 0)");
        }
    }
}

}  // namespace

BaileyRotor::BaileyRotor(BaileyRotorConfig config) : config_(std::move(config)) {
    validate(config_);
    solidity_ = static_cast<double>(config_.num_blades) * config_.chord /
                (std::numbers::pi * config_.radius);
    area_ = std::numbers::pi * config_.radius * config_.radius;
    tip_speed_ = config_.rotor_speed * config_.radius;
    lift_factor_ = config_.lift_slope * solidity_ / 2.0;
    coning_pitch_ = config_.coning_per_thrust * std::tan(config_.delta3);
    C_hb_ = hub_rotation(config_.hub_pitch, config_.hub_roll);
    C_bh_ = C_hb_.transpose();
}

void BaileyRotor::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
}

std::vector<std::string> BaileyRotor::consumes() const {
    std::vector<std::string> names =
        config_.wash ? config_.wash->consumes() : std::vector<std::string>{};
    if (config_.fuselage_wake) {
        const std::string& a = config_.fuselage_wake->airframe;
        names.push_back(a + ".fin_q_factor");
        names.push_back(a + ".sidewash");
        names.push_back(a + ".downwash");
    }
    if (!config_.pitch_input.empty()) {
        names.push_back(config_.pitch_input + ".theta_tr");
    }
    return names;
}

void BaileyRotor::bind(const CouplingRegistry& registry) {
    const std::string consumer = config_.name.empty() ? "bailey_rotor" : config_.name;
    if (config_.wash) {
        config_.wash->bind(registry, consumer);
    }
    if (config_.fuselage_wake) {
        const std::string& a = config_.fuselage_wake->airframe;
        q_factor_ = registry.find(a + ".fin_q_factor", consumer);
        sidewash_ = registry.find(a + ".sidewash", consumer);
        downwash_ = registry.find(a + ".downwash", consumer);
    }
    if (!config_.pitch_input.empty()) {
        pitch_ = registry.find(config_.pitch_input + ".theta_tr", consumer);
    }
}

double BaileyRotor::rotor_speed(const SimContext& ctx) const noexcept {
    if (shaft_index_ < 0) {
        return config_.rotor_speed;
    }
    return gear_ * ctx.shafts[static_cast<std::size_t>(shaft_index_)].omega;
}

double BaileyRotor::azimuth(const SimContext& ctx) const noexcept {
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    const double turned = shaft_index_ < 0
                              ? config_.rotor_speed * ctx.t
                              : gear_ * ctx.shafts[static_cast<std::size_t>(shaft_index_)].psi;
    double psi = std::fmod(turned, kTwoPi);
    if (psi < 0.0) {
        psi += kTwoPi;
    }
    return psi;
}

void BaileyRotor::bind_shafts(std::span<const ShaftInfo> shafts) {
    if (!config_.shaft) {
        return;
    }
    const std::string who = "bailey_rotor '" + config_.name + "'";
    const auto& link = config_.shaft.value();
    for (std::size_t k = 0; k < shafts.size(); ++k) {
        if (shafts[k].name == link.name) {
            const double want = link.gear * shafts[k].speed;
            if (std::abs(config_.rotor_speed - want) > 1e-9 * want) {
                throw InitError(who + ": rotor_speed must be gear × shaft '" + shafts[k].name +
                                "' speed (D-052)");
            }
            shaft_index_ = static_cast<int>(k);
            gear_ = link.gear;
            return;
        }
    }
    throw InitError(who + ": no shaft named '" + link.name + "' (D-052)");
}

BaileyRotorLoads BaileyRotor::evaluate(const SimContext& ctx,
                                       std::span<const double> x) const noexcept {
    Vec3 wash_b = Vec3::Zero();
    if (config_.wash && ctx.coupling != nullptr) {
        wash_b = config_.wash->evaluate(*ctx.coupling).velocity_b;
    }
    if (!config_.fuselage_wake) {
        return evaluate(ctx, x[0], wash_b);
    }
    FuselageWakeInput wake;
    if (ctx.coupling != nullptr) {
        wake.q_factor = ctx.coupling->get(q_factor_);
        wake.sidewash = ctx.coupling->get(sidewash_);
        wake.downwash = ctx.coupling->get(downwash_);
    }
    wake.sidewash_d = x[1];
    wake.downwash_d = x[2];
    return evaluate(ctx, x[0], wash_b, &wake);
}

BaileyRotorLoads BaileyRotor::evaluate(const SimContext& ctx, double lambda_i, const Vec3& wash_b,
                                       const FuselageWakeInput* wake) const noexcept {
    const BaileyRotorConfig& c = config_;
    const BodyState& b = ctx.body;
    BaileyRotorLoads out;
    out.lambda_i = lambda_i;
    out.wash_b = wash_b;

    // Hub velocity relative to the air, with body rates and main-rotor wash
    // (GenHel p. 5.4-6; body-axis wind stands in for GenHel's gust input).
    const Vec3 v_air_b = b.vel_b - b.q_nb.conjugate() * ctx.env.wind_n;
    Vec3 v_b = v_air_b + b.omega_b.cross(c.hub_position_b) + wash_b;
    if (c.fuselage_wake && wake != nullptr) {
        // Fuselage wake (p. 5.4-5): K_QTR = √(Q_TRQWF(|ψ_WF|)) scales the body
        // velocity; the delayed sidewash and downwash enter as −u σ_d K_QTR
        // and −u ε_d K_QTR (GenHel: −(V_xb/57.3) SIG_TR K_QTR, with degrees).
        // K_QTR, σ and ε are the airframe's: GenHel reads the fin's and the
        // stabilator's maps here (QV1MP, SGV1MP, EPH1MP).
        const double u = v_air_b.x();
        out.q_factor = wake->q_factor;
        out.sidewash = wake->sidewash;
        out.downwash = wake->downwash;
        v_b += (out.q_factor - 1.0) * v_air_b + Vec3(0.0, -u * wake->sidewash_d * out.q_factor,
                                                     -u * wake->downwash_d * out.q_factor);
        const double rate = std::abs(u) / -c.hub_position_b.x();
        out.sidewash_rate = rate * (out.sidewash - wake->sidewash_d);
        out.downwash_rate = rate * (out.downwash - wake->downwash_d);
    }
    const Vec3 v_h = C_hb_ * v_b;
    const double omega = rotor_speed(ctx);  // D-052
    const double tip_speed = omega * c.radius;
    const double mu2 = (v_h.x() * v_h.x() + v_h.y() * v_h.y()) / (tip_speed * tip_speed);
    out.mu = std::sqrt(mu2);
    out.mu_z = v_h.z() / tip_speed;

    // Bailey coefficients (p. 5.4-7).
    const double B = c.tip_loss;
    const double t31 = B * B / 2.0 + mu2 / 4.0;
    const double t32 = B * B * B / 3.0 + B * mu2 / 2.0;
    const double t33 = B * B * B * B / 4.0 + B * B * mu2 / 4.0;
    const double G = lift_factor_;
    const double lambda = out.mu_z - lambda_i;

    // Fin blockage on the airspeed (p. 5.4-8).
    const double speed = v_air_b.norm();
    out.blockage = 1.0;
    if (c.blockage_speed > 0.0 && speed < c.blockage_speed) {
        const double r = speed / c.blockage_speed;
        out.blockage = (1.0 - c.blockage_factor) * r * r + c.blockage_factor;
    }

    // θ = θ_c − (∂a₀/∂T) T tan δ₃ with T = K_BLK ρA(ΩR)² C_T and
    // C_T = G(λ t₃₁ + θ t₃₂ + θ_tw t₃₃), solved for C_T (D-028).
    const double rho = ctx.air.density;
    const double q_area = rho * area_ * tip_speed * tip_speed;
    double theta_cmd = 0.0;  // a pitch input evaluated on its own: zero (D-003)
    if (c.controls) {
        theta_cmd = c.controls->pitch(ctx.controls).theta0;
    } else if (ctx.coupling != nullptr && pitch_.valid()) {
        theta_cmd = ctx.coupling->get(pitch_);
    }
    const double theta_c = theta_cmd + c.pitch_bias;
    const double k = coning_pitch_ * out.blockage * q_area;
    out.ct = G * (lambda * t31 + theta_c * t32 + c.twist * t33) / (1.0 + G * t32 * k);
    out.pitch = theta_c - k * out.ct;
    out.thrust = out.blockage * q_area * out.ct;

    // Inflow: Pitt–Peters thrust row (D-027, D-028).
    const double V = std::sqrt(mu2 + lambda * lambda);
    out.lambda_dot = omega * (out.ct - 2.0 * lambda_i * V) / PittPetersMass::m0;

    // Thrust along −z_h, drag along x_b (p. 5.4-8), at the hub.
    Vec3 F_b = C_bh_ * Vec3(0.0, 0.0, -out.thrust);
    F_b.x() -= 0.5 * rho * c.drag_area * v_b.x() * std::abs(v_b.x());
    out.body.F = F_b;
    out.body.M = c.hub_position_b.cross(F_b);
    return out;
}

void BaileyRotor::derivatives(const SimContext& ctx, std::span<const double> x,
                              ComponentDerivs& out) const {
    const BaileyRotorLoads loads = evaluate(ctx, x);
    out.load0_b = loads.body;
    out.xdot0[0] = loads.lambda_dot;
    if (shaft_index_ >= 0 && !out.Gs.empty()) {
        // λ_i is a velocity over ΩR: with it held, λ̇ gains −λ Ω̇/Ω (D-052).
        // No torque on the shaft: the tail-rotor torque isn't modelled (issue
        // #40), and the thrust and drag act at the hub, with no axial moment.
        out.Gs[static_cast<std::size_t>(shaft_index_)] = -gear_ * x[0] / rotor_speed(ctx);
    }
    if (config_.fuselage_wake) {
        out.xdot0[1] = loads.sidewash_rate;
        out.xdot0[2] = loads.downwash_rate;
    }
}

// ---- Config ------------------------------------------------------------------

namespace {

// {airframe: <name>}: the airframe instance whose wake values are read.
FuselageWake fuselage_wake_from_config(const Config& node) {
    node.allow_keys({"airframe"});
    return FuselageWake{node.string("airframe")};
}

}  // namespace

std::unique_ptr<BaileyRotor> BaileyRotor::from_config(const Config& node) {
    using units::Dimension;
    node.allow_keys(
        {"type",          "name",     "blades", "radius",   "rotor_speed",       "chord",
         "lift_slope",    "tip_loss", "twist",  "delta3",   "coning_per_thrust", "pitch_bias",
         "drag_area",     "blockage", "hub",    "controls", "pitch_input",       "main_rotor_wash",
         "fuselage_wake", "shaft"});
    BaileyRotorConfig c;
    if (node.contains("name")) {
        c.name = node.string("name");
        if (c.name.empty()) {
            node.fail("name", "must not be empty");
        }
    }
    const std::int64_t blades = node.integer("blades");
    if (blades < 1 || blades > 64) {
        node.fail("blades", "must be between 1 and 64");
    }
    c.num_blades = static_cast<int>(blades);
    c.radius = node.quantity("radius", Dimension::length);
    c.rotor_speed = node.quantity("rotor_speed", Dimension::angular_rate);
    if (node.contains("shaft")) {  // optional (D-052)
        const Config sh = node.child("shaft");
        sh.allow_keys({"name", "gear"});
        BaileyRotorConfig::ShaftLink link;
        link.name = sh.string("name");
        link.gear = sh.contains("gear") ? sh.number("gear") : 1.0;
        if (link.name.empty()) {
            sh.fail("name", "must not be empty");
        }
        if (!(link.gear > 0.0) || !std::isfinite(link.gear)) {
            sh.fail("gear", "must be positive and finite");
        }
        c.shaft = link;
    }
    c.chord = node.quantity("chord", Dimension::length);
    c.lift_slope = node.quantity("lift_slope", Dimension::per_angle);
    c.tip_loss = node.number("tip_loss");
    c.twist = node.quantity("twist", Dimension::angle);
    if (node.contains("delta3")) {
        // δ₃ needs the coning rate it acts through; no silent zero (D-021).
        c.delta3 = node.quantity("delta3", Dimension::angle);
        c.coning_per_thrust = node.quantity("coning_per_thrust", Dimension::angle_per_force);
    } else if (node.contains("coning_per_thrust")) {
        node.fail("coning_per_thrust", "is used only with delta3");
    }
    c.pitch_bias =
        node.contains("pitch_bias") ? node.quantity("pitch_bias", Dimension::angle) : 0.0;
    c.drag_area = node.contains("drag_area") ? node.quantity("drag_area", Dimension::area) : 0.0;
    if (node.contains("blockage")) {
        const Config blk = node.child("blockage");
        blk.allow_keys({"factor", "speed"});
        c.blockage_factor = blk.number("factor");
        c.blockage_speed = blk.quantity("speed", Dimension::speed);
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

    if (node.contains("pitch_input")) {
        if (node.contains("controls")) {
            node.fail("pitch_input", "give either controls or pitch_input, not both");
        }
        c.pitch_input = node.string("pitch_input");
    } else {
        c.controls = LinearControlMap::from_config(node.child("controls"));
    }
    if (node.contains("main_rotor_wash")) {
        c.wash = RotorWash::from_config(node.child("main_rotor_wash"));
    }
    if (node.contains("fuselage_wake")) {
        c.fuselage_wake = node.in_context("fuselage_wake", [&] {
            return fuselage_wake_from_config(node.child("fuselage_wake"));
        });
    }
    return node.in_context("", [&] { return std::make_unique<BaileyRotor>(std::move(c)); });
}

}  // namespace eyas
