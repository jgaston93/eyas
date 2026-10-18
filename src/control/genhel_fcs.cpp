#include "control/genhel_fcs.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "core/rigid_body.hpp"
#include "env/atmosphere.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {

namespace {

// Inches at normalized input u: collective spans [0, 1], the others [−1, 1].
double inches(const StickTravel& t, double u, bool zero_to_one) noexcept {
    if (zero_to_one) {
        return t.lo + (t.hi - t.lo) * u;
    }
    return 0.5 * (t.lo + t.hi) + 0.5 * (t.hi - t.lo) * u;
}

struct Mixer {
    double theta0 = 0.0, a1s = 0.0, b1s = 0.0, thettr = 0.0;  // [rad]
};

// GenHel Figs. 5.1.6(a)–(d) (p. 5.5-14 to 5.5-17; control_rigging_5_5.txt):
//   THOSUM = KXCTH0 XCSUM,  THRSUM = KXPTHR XPSUM
//   THETA0 = THOSUM + BIAS_C
//   A1S    = KXAA1S XASUM + MIX_AC THOSUM + BIAS_A
//   B1S    = KXBB1S XBSUM + MIX_BC THOSUM + MIX_BP THRSUM + BIAS_B
//   THETTR = THRSUM + THOSUM + BIAS_P
// with XBSUM = SIGN_XB2 XB+2 and the other sums the pilot's inches (the SAS,
// PBA and FPS inch inputs join here in M8c2 and M8c3), then the rotor head
// limits of p. 5.5-20 on THETA0, A1S and B1S (D-045).
Mixer mix(const GenHelRigging& r, const Controls& c) noexcept {
    const double xc = inches(r.collective, c.collective, true);
    const double xa = inches(r.lat_cyclic, c.lat_cyclic, false);
    const double xb = r.sign_lon_stick * inches(r.lon_cyclic, c.lon_cyclic, false);
    const double xp = inches(r.pedal, c.pedal, false);
    const double thosum = r.k_collective * xc;
    const double thrsum = r.k_pedal * xp;
    Mixer m;
    m.theta0 = std::clamp(thosum + r.bias_collective, r.theta0_min, r.theta0_max);
    m.a1s = std::clamp(r.k_lateral * xa + r.mix_lateral_collective * thosum + r.bias_lateral,
                       r.a1s_min, r.a1s_max);
    m.b1s = std::clamp(r.k_longitudinal * xb + r.mix_longitudinal_collective * thosum +
                           r.mix_longitudinal_pedal * thrsum + r.bias_longitudinal,
                       r.b1s_min, r.b1s_max);
    m.thettr = thrsum + thosum + r.bias_pedal;
    return m;
}

// ẋ of a servo 1/(a2 s² + a1 s + 1) at states (p, v) and command u. The rate
// limit holds the velocity at ±R: the acceleration stops where it would push
// past it.
void servo(const ServoConfig& s, double u, double p, double v, double& p_dot,
           double& v_dot) noexcept {
    double acc = (u - p - s.a1 * v) / s.a2;
    if (s.rate_limit > 0.0) {
        if ((v >= s.rate_limit && acc > 0.0) || (v <= -s.rate_limit && acc < 0.0)) {
            acc = 0.0;
        }
        p_dot = std::clamp(v, -s.rate_limit, s.rate_limit);
    } else {
        p_dot = v;
    }
    v_dot = acc;
}

// IHT0 within the travel limits; `ramp` is the lateral-acceleration gain's
// fraction [0, 1] at v_f.
double stabilator_command(const GenHelStabilatorLaw& l, double v_f, double collective, double q_s,
                          double ay_s, double ramp) noexcept {
    const double cmd =
        l.schedule.command(v_f, collective) + l.pitch_rate_gain * q_s + l.ay_gain * ramp * ay_s;
    return std::clamp(cmd, l.schedule.min, l.schedule.max);
}

// The ramp's corners are rounded like D-038's gain corners: a level trim at
// 60 kt sits on the upper one, which a linearization can't take a slope at.
constexpr double kRampRounding = 0.1;

// Body-y specific force at the transducer r with the body acceleration zero:
// (ω × v + ω × (ω × r) − g)_y [m/s²]. The full value adds (v̇ + ω̇ × r)_y.
double lateral_specific_force0(const BodyState& b, const Vec3& r) noexcept {
    const Vec3& w = b.omega_b;
    const Vec3 g_b = b.q_nb.conjugate() * gravity_n();
    return w.cross(b.vel_b).y() + w.cross(w.cross(r)).y() - g_b.y();
}

// Equivalent airspeed from the body-x air velocity, u √(ρ/ρ0) (D-040) [m/s].
double equivalent_airspeed(const SimContext& ctx) noexcept {
    const BodyState& b = ctx.body;
    const double u_air = (b.vel_b - b.q_nb.conjugate() * ctx.env.wind_n).x();
    return u_air * std::sqrt(ctx.air.density / isa::kSeaLevelDensity);
}

}  // namespace

GenHelFcs::GenHelFcs(GenHelFcsConfig config)
    : config_(std::move(config)),
      ay_ramp_(Axis({config_.stabilator.ay_speed_lo, config_.stabilator.ay_speed_hi}), {0.0, 1.0}) {
    const auto positive = [](const ServoConfig& s, const char* what) {
        if (!(s.a2 > 0.0) || !(s.a1 > 0.0) || s.rate_limit < 0.0) {
            throw InitError(std::string("genhel_fcs: ") + what +
                            " needs a2 > 0, a1 > 0 and a rate limit >= 0");
        }
    };
    positive(config_.primary_servo, "primary_servo");
    positive(config_.rate_gyro, "rate_gyro");
    positive(config_.lateral_accelerometer, "lateral_accelerometer");
    positive(config_.stabilator.servo, "stabilator servo");
    if (config_.name.empty()) {
        throw InitError("genhel_fcs: needs a name (its outputs are named after it)");
    }
    if (!(config_.stabilator.airspeed_lag > 0.0)) {
        throw InitError("genhel_fcs: the stabilator airspeed lag must be positive");
    }
    if (!(config_.stabilator.ay_speed_lo < config_.stabilator.ay_speed_hi)) {
        throw InitError("genhel_fcs: the lateral-acceleration ramp needs from < to");
    }
    const GenHelRigging& r = config_.rigging;
    if (!(r.theta0_min < r.theta0_max) || !(r.a1s_min < r.a1s_max) || !(r.b1s_min < r.b1s_max)) {
        throw InitError("genhel_fcs: each head limit's min must be below its max");
    }
}

void GenHelFcs::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
    const Mixer m = mix(config_.rigging, Controls{});
    x[kServoTheta0] = m.theta0;
    x[kServoA1s] = m.a1s;
    x[kServoB1s] = m.b1s;
    x[kServoThettr] = m.thettr;
    x[kServoStab] = stabilator_command(config_.stabilator, 0.0, 0.0, 0.0, 0.0, 0.0);
}

std::vector<std::string> GenHelFcs::produces() const {
    const std::string& n = config_.name;
    return {n + ".theta0", n + ".theta1s", n + ".theta1c", n + ".theta_tr",
            n + ".stabilator_incidence"};
}

void GenHelFcs::bind(const CouplingRegistry& registry) {
    const std::string& n = config_.name;
    theta0_ = registry.find(n + ".theta0", n);
    theta1s_ = registry.find(n + ".theta1s", n);
    theta1c_ = registry.find(n + ".theta1c", n);
    theta_tr_ = registry.find(n + ".theta_tr", n);
    stab_ = registry.find(n + ".stabilator_incidence", n);
}

GenHelFcsOutputs GenHelFcs::evaluate(const SimContext& ctx,
                                     std::span<const double> x) const noexcept {
    const GenHelRigging& r = config_.rigging;
    const GenHelStabilatorLaw& l = config_.stabilator;
    GenHelFcsOutputs o;
    const Mixer m = mix(r, ctx.controls);
    o.theta0_cmd = m.theta0;
    o.a1s_cmd = m.a1s;
    o.b1s_cmd = m.b1s;
    o.thettr_cmd = m.thettr;
    o.theta0_cuff = std::clamp(x[kServoTheta0], r.theta0_min, r.theta0_max);
    o.a1s = std::clamp(x[kServoA1s], r.a1s_min, r.a1s_max);
    o.b1s = std::clamp(x[kServoB1s], r.b1s_min, r.b1s_max);
    o.theta_tr = x[kServoThettr];
    // θ = THETA0 − A1S cos(ψ + D) − B1S sin(ψ + D) (GenHel p. 5.1-24) =
    // θ0 + θ1c cos ψ + θ1s sin ψ (Padfield 3rd ed. Eq. 3.59).
    const double cd = std::cos(r.swashplate_phase);
    const double sd = std::sin(r.swashplate_phase);
    o.theta0 = o.theta0_cuff;
    o.theta1c = -o.a1s * cd - o.b1s * sd;
    o.theta1s = o.a1s * sd - o.b1s * cd;
    const double ramp = rounded(ay_ramp_, x[kAirspeedLag], kRampRounding, ay_cursor_);
    o.stabilator_cmd = stabilator_command(l, x[kAirspeedLag], ctx.controls.collective, x[kGyroQ],
                                          x[kAccelY], ramp);
    o.stabilator_incidence = std::clamp(x[kServoStab], l.schedule.min, l.schedule.max);
    return o;
}

void GenHelFcs::derivatives(const SimContext& ctx, std::span<const double> x,
                            ComponentDerivs& out) const {
    const GenHelFcsOutputs o = evaluate(ctx, x);
    if (out.coupling != nullptr && theta0_.valid()) {
        out.coupling->set(theta0_, o.theta0);
        out.coupling->set(theta1s_, o.theta1s);
        out.coupling->set(theta1c_, o.theta1c);
        out.coupling->set(theta_tr_, o.theta_tr);
        out.coupling->set(stab_, o.stabilator_incidence);
    }
    const auto run = [&](const ServoConfig& s, double u, std::size_t i) {
        servo(s, u, x[i], x[i + 1], out.xdot0[i], out.xdot0[i + 1]);
    };
    // Primary servos (text p. 5.5-3), on the mixer outputs.
    run(config_.primary_servo, o.theta0_cmd, kServoTheta0);
    run(config_.primary_servo, o.a1s_cmd, kServoA1s);
    run(config_.primary_servo, o.b1s_cmd, kServoB1s);
    run(config_.primary_servo, o.thettr_cmd, kServoThettr);

    // Rate gyros (Fig. 5.1.2) on the body rates.
    const BodyState& b = ctx.body;
    run(config_.rate_gyro, b.omega_b.x(), kGyroP);
    run(config_.rate_gyro, b.omega_b.y(), kGyroQ);
    run(config_.rate_gyro, b.omega_b.z(), kGyroR);

    // Lateral accelerometer (Fig. 5.1.2): the body-y specific force at the
    // transducer r, f = v̇ + ω × v + ω̇ × r + ω × (ω × r) − g, affine in
    // a = [v̇; ω̇] (D-029: the loop closes through this sensor's states).
    const Vec3& r = config_.accelerometer_b;
    const ServoConfig& acc = config_.lateral_accelerometer;
    run(acc, lateral_specific_force0(b, r), kAccelY);
    // ∂f_y/∂a: v̇_y, and (ω̇ × r)_y = ω̇_z r_x − ω̇_x r_z.
    const std::size_t row = (kAccelY + 1) * 6;
    out.G[row + 1] = 1.0 / acc.a2;
    out.G[row + 3] = -r.z() / acc.a2;
    out.G[row + 5] = r.x() / acc.a2;

    // Stabilator (Fig. 5.1.7): airspeed lag on the equivalent airspeed
    // (D-040), then the servo with its rate limit on the command.
    out.xdot0[kAirspeedLag] =
        (equivalent_airspeed(ctx) - x[kAirspeedLag]) / config_.stabilator.airspeed_lag;
    run(config_.stabilator.servo, o.stabilator_cmd, kServoStab);
}

void GenHelFcs::steady_states(const SimContext& ctx, std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);  // every velocity state at rest
    const BodyState& b = ctx.body;
    x[kGyroP] = b.omega_b.x();
    x[kGyroQ] = b.omega_b.y();
    x[kGyroR] = b.omega_b.z();
    x[kAccelY] = lateral_specific_force0(b, config_.accelerometer_b);
    x[kAirspeedLag] = equivalent_airspeed(ctx);
    // The commands read the sensors just set.
    const GenHelFcsOutputs o = evaluate(ctx, x);
    x[kServoTheta0] = o.theta0_cmd;
    x[kServoA1s] = o.a1s_cmd;
    x[kServoB1s] = o.b1s_cmd;
    x[kServoThettr] = o.thettr_cmd;
    x[kServoStab] = o.stabilator_cmd;
}

namespace {

using units::Dimension;

ServoConfig servo_from_config(const Config& node, bool rate_limited) {
    if (rate_limited) {
        node.allow_keys({"a2", "a1", "rate_limit"});
    } else {
        node.allow_keys({"a2", "a1"});
    }
    ServoConfig s;
    s.a2 = node.quantity("a2", Dimension::time_squared);
    s.a1 = node.quantity("a1", Dimension::time);
    if (rate_limited) {
        s.rate_limit = node.quantity("rate_limit", Dimension::angular_rate);
    }
    return s;
}

StickTravel travel_from_config(const Config& node) {
    node.allow_keys({"lo", "hi"});
    return {node.quantity("lo", Dimension::length), node.quantity("hi", Dimension::length)};
}

void limits_from_config(const Config& node, double& lo, double& hi) {
    node.allow_keys({"min", "max"});
    lo = node.quantity("min", Dimension::angle);
    hi = node.quantity("max", Dimension::angle);
}

GenHelRigging rigging_from_config(const Config& node) {
    node.allow_keys({"travel", "gains", "longitudinal_stick_sign", "mixing", "bias",
                     "swashplate_phase", "head_limits"});
    GenHelRigging r;
    const Config t = node.child("travel");
    t.allow_keys({"collective", "lat_cyclic", "lon_cyclic", "pedal"});
    r.collective =
        t.in_context("collective", [&] { return travel_from_config(t.child("collective")); });
    r.lat_cyclic =
        t.in_context("lat_cyclic", [&] { return travel_from_config(t.child("lat_cyclic")); });
    r.lon_cyclic =
        t.in_context("lon_cyclic", [&] { return travel_from_config(t.child("lon_cyclic")); });
    r.pedal = t.in_context("pedal", [&] { return travel_from_config(t.child("pedal")); });
    const Config g = node.child("gains");
    g.allow_keys({"collective", "lateral", "longitudinal", "pedal"});
    r.k_collective = g.quantity("collective", Dimension::angle_per_length);
    r.k_lateral = g.quantity("lateral", Dimension::angle_per_length);
    r.k_longitudinal = g.quantity("longitudinal", Dimension::angle_per_length);
    r.k_pedal = g.quantity("pedal", Dimension::angle_per_length);
    r.sign_lon_stick = node.number("longitudinal_stick_sign");
    if (r.sign_lon_stick != 1.0 && r.sign_lon_stick != -1.0) {
        node.fail("longitudinal_stick_sign", "must be 1 or -1");
    }
    const Config m = node.child("mixing");
    m.allow_keys({"lateral_collective", "longitudinal_collective", "longitudinal_pedal"});
    r.mix_lateral_collective = m.number("lateral_collective");
    r.mix_longitudinal_collective = m.number("longitudinal_collective");
    r.mix_longitudinal_pedal = m.number("longitudinal_pedal");
    const Config b = node.child("bias");
    b.allow_keys({"collective", "lateral", "longitudinal", "pedal"});
    r.bias_collective = b.quantity("collective", Dimension::angle);
    r.bias_lateral = b.quantity("lateral", Dimension::angle);
    r.bias_longitudinal = b.quantity("longitudinal", Dimension::angle);
    r.bias_pedal = b.quantity("pedal", Dimension::angle);
    r.swashplate_phase = node.quantity("swashplate_phase", Dimension::angle);
    if (node.contains("head_limits")) {
        const Config h = node.child("head_limits");
        h.allow_keys({"theta0", "a1s", "b1s"});
        h.in_context("theta0",
                     [&] { limits_from_config(h.child("theta0"), r.theta0_min, r.theta0_max); });
        h.in_context("a1s", [&] { limits_from_config(h.child("a1s"), r.a1s_min, r.a1s_max); });
        h.in_context("b1s", [&] { limits_from_config(h.child("b1s"), r.b1s_min, r.b1s_max); });
    }
    return r;
}

GenHelStabilatorLaw stabilator_from_config(const Config& node) {
    node.allow_keys({"schedule", "airspeed_lag", "pitch_rate_gain", "lateral_accel_gain", "servo"});
    GenHelStabilatorLaw l;
    l.schedule = node.in_context(
        "schedule", [&] { return incidence_schedule_from_config(node.child("schedule")); });
    l.airspeed_lag = node.quantity("airspeed_lag", Dimension::time);
    l.pitch_rate_gain = node.quantity("pitch_rate_gain", Dimension::time);
    const Config ay = node.child("lateral_accel_gain");
    ay.allow_keys({"gain", "from", "to"});
    l.ay_gain = ay.quantity("gain", Dimension::angle_per_acceleration);
    l.ay_speed_lo = ay.quantity("from", Dimension::speed);
    l.ay_speed_hi = ay.quantity("to", Dimension::speed);
    l.servo =
        node.in_context("servo", [&] { return servo_from_config(node.child("servo"), true); });
    return l;
}

}  // namespace

std::unique_ptr<GenHelFcs> GenHelFcs::from_config(const Config& node) {
    node.allow_keys({"type", "name", "rigging", "primary_servo", "rate_gyro",
                     "lateral_accelerometer", "stabilator"});
    GenHelFcsConfig c;
    c.name = node.string("name");
    c.rigging =
        node.in_context("rigging", [&] { return rigging_from_config(node.child("rigging")); });
    c.primary_servo = node.in_context(
        "primary_servo", [&] { return servo_from_config(node.child("primary_servo"), false); });
    c.rate_gyro = node.in_context(
        "rate_gyro", [&] { return servo_from_config(node.child("rate_gyro"), false); });
    const Config la = node.child("lateral_accelerometer");
    la.allow_keys({"a2", "a1", "position"});
    c.lateral_accelerometer.a2 = la.quantity("a2", Dimension::time_squared);
    c.lateral_accelerometer.a1 = la.quantity("a1", Dimension::time);
    const Config pos = la.child("position");
    pos.allow_keys({"x", "y", "z"});
    c.accelerometer_b =
        Vec3(pos.quantity("x", Dimension::length), pos.quantity("y", Dimension::length),
             pos.quantity("z", Dimension::length));
    c.stabilator = node.in_context(
        "stabilator", [&] { return stabilator_from_config(node.child("stabilator")); });
    return node.in_context("", [&] { return std::make_unique<GenHelFcs>(std::move(c)); });
}

}  // namespace eyas
