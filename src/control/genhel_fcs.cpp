#include "control/genhel_fcs.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "env/air_data.hpp"
#include "env/atmosphere.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {

namespace detail {

// The SAS and PBA at one evaluation (M8c2): the input of every first-order
// state (a lag's input, a washout's or hold's input, an actuator's command,
// the fade's target), the channel outputs [-] and the PBA's held command.
struct GenHelAfcsSignals {
    std::array<double, GenHelFcs::kNumStates> in{};
    double pitch_d = 0.0, pitch_a = 0.0, roll_d = 0.0, roll_a = 0.0, yaw_d = 0.0, yaw_a = 0.0;
    double pba_held = 0.0;  // the hold's output
};

}  // namespace detail

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

// The automatic inputs to the mixer's sums [m]: the SAS inner loop XAILS,
// XBILS, XPILS and the PBA's XBBAS (M8c2; the FPS joins in M8c3).
struct InnerLoop {
    double xa = 0.0, xb = 0.0, xp = 0.0;
};

// GenHel Figs. 5.1.6(a)–(d) (p. 5.5-14 to 5.5-17; control_rigging_5_5.txt):
//   THOSUM = KXCTH0 XCSUM,  THRSUM = KXPTHR XPSUM
//   THETA0 = THOSUM + BIAS_C
//   A1S    = KXAA1S XASUM + MIX_AC THOSUM + BIAS_A
//   B1S    = KXBB1S XBSUM + MIX_BC THOSUM + MIX_BP THRSUM + BIAS_B
//   THETTR = THRSUM + THOSUM + BIAS_P
// with XBSUM = SIGN_XB2 XB+2 + XBILS + XBBAS, XASUM = XA+2 + XAILS, XPSUM =
// XP+2 + XPILS, XCSUM = XC+2, then the rotor head limits of p. 5.5-20 on
// THETA0, A1S and B1S (D-045).
Mixer mix(const GenHelRigging& r, const Controls& c, const InnerLoop& il) noexcept {
    const double xc = inches(r.collective, c.collective, true);
    const double xa = inches(r.lat_cyclic, c.lat_cyclic, false) + il.xa;
    const double xb = r.sign_lon_stick * inches(r.lon_cyclic, c.lon_cyclic, false) + il.xb;
    const double xp = inches(r.pedal, c.pedal, false) + il.xp;
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
// limit holds the velocity at ±R. Where the command pushes it outward at or
// past the limit, only the servo's own velocity damping acts, on the excess:
// v̇ = −a1 (v ∓ R)/a2. A step that carries v past R (by up to v̇ h) then
// decays back instead of winding up, and v̇ is continuous at the limit.
void servo(const ServoConfig& s, double u, double p, double v, double& p_dot,
           double& v_dot) noexcept {
    double acc = (u - p - s.a1 * v) / s.a2;
    if (s.rate_limit > 0.0) {
        const double R = s.rate_limit;
        if (v >= R && acc > 0.0) {
            acc = -s.a1 * (v - R) / s.a2;
        } else if (v <= -R && acc < 0.0) {
            acc = -s.a1 * (v + R) / s.a2;
        }
        p_dot = std::clamp(v, -R, R);
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

// Equivalent airspeed of the body-x air velocity (D-040) [m/s].
double body_x_eas(const SimContext& ctx) noexcept {
    const BodyState& b = ctx.body;
    const double u_air = (b.vel_b - b.q_nb.conjugate() * ctx.env.wind_n).x();
    return equivalent_airspeed(ctx.air.density, u_air);
}

using Fcs = GenHelFcs;

// The fade's approach to its target: at most 1/T_fade, proportional within
// 1/(T_fade k) of it, so it settles without chattering (numerical, D-045).
constexpr double kFadeGain = 100.0;  // [1/s]

// Passes of steady_states(): each sets every state to its input, which is
// exact once it has run as many passes as the longest chain of states (7:
// the roll analog's attitude filters, shaping, derivative, lags, actuator).
constexpr int kSteadyPasses = 8;

// The SAS's 2:1 gain switch (Fig. 5.1.3(a), p. 5.5-2): ON/ON both channels at
// gain 1, ON/OFF the digital alone at 2, OFF/ON the analog alone at 2.
double digital_gain(const AfcsSwitches& s) noexcept {
    if (!s.sas_digital) {
        return 0.0;
    }
    return s.sas_analog ? 1.0 : 2.0;
}
double analog_gain(const AfcsSwitches& s) noexcept {
    if (!s.sas_analog) {
        return 0.0;
    }
    return s.sas_digital ? 1.0 : 2.0;
}

double limit(double v, double l) noexcept {
    return std::clamp(v, -l, l);
}

using AfcsSignals = detail::GenHelAfcsSignals;

// A zero-order hold as a first-order Padé of half of it (D-045):
// (1 − sT/4)/(1 + sT/4) = 2/(1 + sT/4) − 1, a lag z of T/4 with output 2z − u.
// A hold's output stays within its input's range, the Padé's doesn't (a
// reversal from +L to −L gives about 3L), so the SAS clamps it to the
// channel's limit (code review).
double pade_hold(std::span<const double> x, std::size_t i, double u, AfcsSignals& s) noexcept {
    s.in[i] = u;
    return 2.0 * x[i] - u;
}

// GenHel Figs. 5.1.3(a)–(c) (SAS) and 5.1.4 (PBA), with the text pp. 5.5-2,
// 5.5-3; signal names as fig_5_1_3_sas.txt. The sensors are the M8c1 states
// (rate gyros, lateral accelerometer), the vertical gyro the Euler attitude
// (Fig. 5.1.2: no dynamics), the airspeed VXBIKT the body-x equivalent
// airspeed (D-040).
AfcsSignals afcs_signals(const GenHelFcsConfig& c, const SimContext& ctx,
                         std::span<const double> x) noexcept {
    AfcsSignals o;
    if (!c.sas) {
        return o;
    }
    const GenHelSas& sas = *c.sas;
    const AfcsSwitches& sw = ctx.controls.afcs;
    const double gd = digital_gain(sw);
    const double ga = analog_gain(sw);
    const EulerAngles e = euler_angles(ctx.body.q_nb);
    const double v = body_x_eas(ctx);
    const bool slow = v < sas.switch_speed;
    const double ysens = x[Fcs::kAccelY];
    auto& in = o.in;

    // Pitch, Fig. 5.1.3(a). Digital: A/D filter, rate and lagged rate,
    // washout, switch, limit, hold.
    const GenHelSasPitch& p = sas.pitch;
    in[Fcs::kPitchAd] = x[Fcs::kGyroQ];
    const double q1 = x[Fcs::kPitchAd];
    in[Fcs::kPitchRateD] = q1;
    in[Fcs::kPitchLagD] = q1;
    const double pd =
        p.rate_d.gain * x[Fcs::kPitchRateD] + p.lagged_rate_d.gain * x[Fcs::kPitchLagD];
    in[Fcs::kPitchWashD] = pd;
    o.pitch_d =
        limit(pade_hold(x, Fcs::kPitchHoldD, limit(gd * (pd - x[Fcs::kPitchWashD]), p.limit_d), o),
              p.limit_d);
    // Analog: KQQPA TW s/((T s + 1)(TW s + 1)), rate and lagged rate, the .5
    // block, switch, limit.
    in[Fcs::kPitchFilterA] = x[Fcs::kGyroQ];
    const double qa = p.filter_a.gain * x[Fcs::kPitchFilterA];
    in[Fcs::kPitchWashA] = qa;
    const double q2 = qa - x[Fcs::kPitchWashA];
    in[Fcs::kPitchRateA] = q2;
    in[Fcs::kPitchLagA] = q2;
    o.pitch_a =
        limit(ga * p.scale_a *
                  (p.rate_a.gain * x[Fcs::kPitchRateA] + p.lagged_rate_a.gain * x[Fcs::kPitchLagA]),
              p.limit_a);

    // Roll, Fig. 5.1.3(b). Digital: A/D filter, rate, lagged rate and the
    // limited attitude, switch, limit, hold.
    const GenHelSasRoll& r = sas.roll;
    in[Fcs::kRollAd] = x[Fcs::kGyroP];
    const double p1 = x[Fcs::kRollAd];
    in[Fcs::kRollLagD] = p1;
    const double rd = r.rate_gain_d * p1 + r.lagged_rate_d.gain * x[Fcs::kRollLagD] +
                      r.attitude_gain_d * limit(e.roll, r.attitude_limit_d);
    o.roll_d = limit(pade_hold(x, Fcs::kRollHoldD, limit(gd * rd, r.limit_d), o), r.limit_d);
    // Analog, its rate derived from the roll attitude: f1 = KPHF1/((T1 s +
    // 1)(T2 s + 1)) φ, then KPHF2/(A2 s² + A1 s + 1), limit, and KPHF3 s/((T1 s
    // + 1)(T2 s + 1)) = p2, whose s is the second lag's rate; lags on p2, and
    // the attitude path KPHIRA limit(KPHR1 f1).
    in[Fcs::kRollAtt1] = r.attitude_filter_gain * e.roll;
    in[Fcs::kRollAtt2] = x[Fcs::kRollAtt1];
    const double f1 = x[Fcs::kRollAtt2];
    in[Fcs::kRollShape] = r.shaping_gain * f1;
    in[Fcs::kRollDer1] = limit(x[Fcs::kRollShape], r.shaping_limit);
    in[Fcs::kRollDer2] = x[Fcs::kRollDer1];
    const double p2 = r.derivative_gain * (x[Fcs::kRollDer1] - x[Fcs::kRollDer2]) / r.derivative_t2;
    in[Fcs::kRollRateA] = p2;
    in[Fcs::kRollLagA] = p2;
    const double r_a = r.rate_a.gain * x[Fcs::kRollRateA];  // "B", to the yaw analog
    const double ra = r_a + r.lagged_rate_a.gain * x[Fcs::kRollLagA] +
                      r.attitude_output_gain_a * limit(r.attitude_gain_a * f1, r.attitude_limit_a);
    o.roll_a = limit(ga * ra, r.limit_a);

    // Yaw, Fig. 5.1.3(c). Digital: A/D filter; rate, and the lagged rate below
    // the switch speed; washout; and, faded in above it, the lagged lateral
    // acceleration and the roll A/D output ("A"); switch, limit, hold.
    const GenHelSasYaw& y = sas.yaw;
    in[Fcs::kYawAd] = x[Fcs::kGyroR];
    const double r1 = x[Fcs::kYawAd];
    in[Fcs::kYawRateD] = r1;
    in[Fcs::kYawLagD] = slow ? r1 : 0.0;
    const double s1 = y.rate_d.gain * x[Fcs::kYawRateD] + y.lagged_rate_d.gain * x[Fcs::kYawLagD];
    in[Fcs::kYawWashD] = s1;
    in[Fcs::kYawAccelD] = ysens;
    in[Fcs::kYawFade] = slow ? 0.0 : 1.0;
    const double fade = std::clamp(x[Fcs::kYawFade], 0.0, 1.0);
    const double s2 =
        fade * (y.lateral_accel_d.gain * x[Fcs::kYawAccelD] + y.roll_rate_gain_d * p1);
    o.yaw_d =
        limit(pade_hold(x, Fcs::kYawHoldD, limit(gd * (s1 - x[Fcs::kYawWashD] + s2), y.limit_d), o),
              y.limit_d);
    // Analog: KRRFI TW s/((T s + 1)(TW s + 1)) = w; a lag on w; a lag on w
    // below the switch speed, on KLAYF1 Y.SENS above it; the .5 block; and,
    // faded in above it, KRPPYA on the roll channel's "B"; switch, limit.
    in[Fcs::kYawFilterA] = x[Fcs::kGyroR];
    const double ra_f = y.filter_a.gain * x[Fcs::kYawFilterA];
    in[Fcs::kYawWashA] = ra_f;
    const double w = ra_f - x[Fcs::kYawWashA];
    in[Fcs::kYawRateA] = w;
    in[Fcs::kYawLagA] = slow ? w : y.lateral_accel_gain_a * ysens;
    const double s3 =
        y.scale_a * (y.rate_a.gain * x[Fcs::kYawRateA] + y.lagged_a.gain * x[Fcs::kYawLagA]);
    o.yaw_a = limit(ga * (s3 + fade * y.roll_rate_gain_a * r_a), y.limit_a);

    // SAS actuator commands: percent → blade pitch (KPCDG*) → inches (1/KX*),
    // Figs. 5.1.6(b)–(d).
    const GenHelRigging& rig = c.rigging;
    in[Fcs::kPitchAct] = (o.pitch_d + o.pitch_a) * sas.descale_pitch / rig.k_longitudinal;
    in[Fcs::kRollAct] = (o.roll_d + o.roll_a) * sas.descale_roll / rig.k_lateral;
    in[Fcs::kYawAct] = (o.yaw_d + o.yaw_a) * sas.descale_yaw / rig.k_pedal;

    // PBA, Fig. 5.1.4: the limited attitude, conditioned pitch rate (q1) and
    // airspeed above the bias, plus the bias; the hold. Off: no command.
    if (c.pitch_bias) {
        const GenHelPitchBias& b = *c.pitch_bias;
        const double batemp =
            sw.pitch_bias
                ? b.attitude_gain * limit(e.pitch, b.attitude_limit) +
                      b.rate_gain * limit(q1, b.rate_limit_in) +
                      b.speed_gain * std::clamp(v - b.speed_from, 0.0, b.speed_range) + b.bias
                : 0.0;
        o.pba_held = pade_hold(x, Fcs::kPbaHold, batemp, o);
    }
    return o;
}

// The automatic inches into the mixer at states x.
InnerLoop inner_loop(const GenHelFcsConfig& c, std::span<const double> x) noexcept {
    InnerLoop il;
    if (c.sas) {
        il.xb = x[Fcs::kPitchAct];
        il.xa = x[Fcs::kRollAct];
        il.xp = x[Fcs::kYawAct];
        if (c.pitch_bias) {
            const double pba = limit(x[Fcs::kPba], c.pitch_bias->authority);
            il.xb += pba * c.sas->descale_pitch / c.rigging.k_longitudinal;  // XBBAS
        }
    }
    return il;
}

// The lateral-acceleration gain's ramp, 0 → 1 from ay_speed_lo to
// ay_speed_hi; checked first, so a bad range gets its own message.
Table1D ay_ramp(const GenHelStabilatorLaw& l) {
    if (!(l.ay_speed_lo < l.ay_speed_hi)) {
        throw InitError("genhel_fcs: the lateral-acceleration ramp needs from < to");
    }
    return Table1D(Axis({l.ay_speed_lo, l.ay_speed_hi}), {0.0, 1.0});
}

}  // namespace

GenHelFcs::GenHelFcs(GenHelFcsConfig config)
    : config_(std::move(config)), ay_ramp_(ay_ramp(config_.stabilator)) {
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
    const GenHelRigging& r = config_.rigging;
    if (!(r.theta0_min < r.theta0_max) || !(r.a1s_min < r.a1s_max) || !(r.b1s_min < r.b1s_max)) {
        throw InitError("genhel_fcs: each head limit's min must be below its max");
    }
    if (config_.sas) {
        const GenHelSas& s = *config_.sas;
        positive(s.actuator, "sas actuator");
        const auto time = [](double t, const char* what) {
            if (!(t > 0.0)) {
                throw InitError(std::string("genhel_fcs: sas ") + what + " must be positive");
            }
            return t;
        };
        const auto lim = [](double l, const char* what) {
            if (!(l >= 0.0)) {
                throw InitError(std::string("genhel_fcs: sas ") + what + " must be >= 0");
            }
        };
        if (!(r.k_lateral != 0.0) || !(r.k_longitudinal != 0.0) || !(r.k_pedal != 0.0)) {
            throw InitError("genhel_fcs: the sas needs nonzero rigging gains");
        }
        const double hold = time(s.hold, "hold") / 4.0;
        time(s.fade_time, "fade time");
        const GenHelSasPitch& p = s.pitch;
        tau_[kPitchAd] = time(p.ad_lag, "pitch A/D lag");
        tau_[kPitchRateD] = time(p.rate_d.tau, "pitch digital rate lag");
        tau_[kPitchLagD] = time(p.lagged_rate_d.tau, "pitch digital lagged rate lag");
        tau_[kPitchWashD] = time(p.washout_d, "pitch digital washout");
        tau_[kPitchHoldD] = hold;
        tau_[kPitchFilterA] = time(p.filter_a.tau, "pitch analog filter lag");
        tau_[kPitchWashA] = time(p.washout_a, "pitch analog washout");
        tau_[kPitchRateA] = time(p.rate_a.tau, "pitch analog rate lag");
        tau_[kPitchLagA] = time(p.lagged_rate_a.tau, "pitch analog lagged rate lag");
        lim(p.limit_d, "pitch digital limit");
        lim(p.limit_a, "pitch analog limit");
        const GenHelSasRoll& o = s.roll;
        tau_[kRollAd] = time(o.ad_lag, "roll A/D lag");
        tau_[kRollLagD] = time(o.lagged_rate_d.tau, "roll digital lagged rate lag");
        tau_[kRollHoldD] = hold;
        tau_[kRollAtt1] = time(o.attitude_filter_t1, "roll attitude filter lag 1");
        tau_[kRollAtt2] = time(o.attitude_filter_t2, "roll attitude filter lag 2");
        tau_[kRollDer1] = time(o.derivative_t1, "roll derivative lag 1");
        tau_[kRollDer2] = time(o.derivative_t2, "roll derivative lag 2");
        tau_[kRollRateA] = time(o.rate_a.tau, "roll analog rate lag");
        tau_[kRollLagA] = time(o.lagged_rate_a.tau, "roll analog lagged rate lag");
        time(o.shaping_a2, "roll shaping a2");
        time(o.shaping_a1, "roll shaping a1");
        lim(o.attitude_limit_d, "roll digital attitude limit");
        lim(o.limit_d, "roll digital limit");
        lim(o.shaping_limit, "roll shaping limit");
        lim(o.attitude_limit_a, "roll analog attitude limit");
        lim(o.limit_a, "roll analog limit");
        const GenHelSasYaw& y = s.yaw;
        tau_[kYawAd] = time(y.ad_lag, "yaw A/D lag");
        tau_[kYawRateD] = time(y.rate_d.tau, "yaw digital rate lag");
        tau_[kYawLagD] = time(y.lagged_rate_d.tau, "yaw digital lagged rate lag");
        tau_[kYawWashD] = time(y.washout_d, "yaw digital washout");
        tau_[kYawAccelD] = time(y.lateral_accel_d.tau, "yaw digital lateral acceleration lag");
        tau_[kYawHoldD] = hold;
        tau_[kYawFilterA] = time(y.filter_a.tau, "yaw analog filter lag");
        tau_[kYawWashA] = time(y.washout_a, "yaw analog washout");
        tau_[kYawRateA] = time(y.rate_a.tau, "yaw analog rate lag");
        tau_[kYawLagA] = time(y.lagged_a.tau, "yaw analog lagged lag");
        lim(y.limit_d, "yaw digital limit");
        lim(y.limit_a, "yaw analog limit");
    }
    if (config_.pitch_bias) {
        const GenHelPitchBias& b = *config_.pitch_bias;
        if (!config_.sas) {
            throw InitError("genhel_fcs: the pitch bias actuator needs the sas (its pitch rate)");
        }
        if (!(b.hold > 0.0) || !(b.actuator_gain > 0.0) || !(b.rate_limit > 0.0) ||
            !(b.authority > 0.0) || !(b.attitude_limit >= 0.0) || !(b.rate_limit_in >= 0.0) ||
            !(b.speed_range >= 0.0)) {
            throw InitError(
                "genhel_fcs: the pitch bias actuator needs hold, gain, rate limit and authority "
                "> 0 and its input limits >= 0");
        }
        tau_[kPbaHold] = b.hold / 4.0;
    }
}

void GenHelFcs::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
    const Mixer m = mix(config_.rigging, Controls{}, InnerLoop{});
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
    return evaluate(ctx, x, afcs_signals(config_, ctx, x));
}

GenHelFcsOutputs GenHelFcs::evaluate(const SimContext& ctx, std::span<const double> x,
                                     const AfcsSignals& s) const noexcept {
    const GenHelRigging& r = config_.rigging;
    const GenHelStabilatorLaw& l = config_.stabilator;
    GenHelFcsOutputs o;
    o.pitch_d = s.pitch_d;
    o.pitch_a = s.pitch_a;
    o.roll_d = s.roll_d;
    o.roll_a = s.roll_a;
    o.yaw_d = s.yaw_d;
    o.yaw_a = s.yaw_a;
    const InnerLoop il = inner_loop(config_, x);
    if (config_.sas) {
        o.xbils = x[kPitchAct];
        o.xails = x[kRollAct];
        o.xpils = x[kYawAct];
        o.xbbas = il.xb - o.xbils;
    }
    if (config_.pitch_bias) {
        o.pba = limit(x[kPba], config_.pitch_bias->authority);
    }
    const Mixer m = mix(r, ctx.controls, il);
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
    const AfcsSignals s = afcs_signals(config_, ctx, x);
    const GenHelFcsOutputs o = evaluate(ctx, x, s);
    if (out.coupling != nullptr && theta0_.valid()) {
        out.coupling->set(theta0_, o.theta0);
        out.coupling->set(theta1s_, o.theta1s);
        out.coupling->set(theta1c_, o.theta1c);
        out.coupling->set(theta_tr_, o.theta_tr);
        out.coupling->set(stab_, o.stabilator_incidence);
    }
    const auto run = [&](const ServoConfig& sv, double u, std::size_t i) {
        servo(sv, u, x[i], x[i + 1], out.xdot0[i], out.xdot0[i + 1]);
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
    out.xdot0[kAirspeedLag] = (body_x_eas(ctx) - x[kAirspeedLag]) / config_.stabilator.airspeed_lag;
    run(config_.stabilator.servo, o.stabilator_cmd, kServoStab);

    // SAS and PBA (M8c2): first-order states toward their inputs; the
    // second-order roll shaping and the SAS actuators; the fade; the PBA
    // actuator. Without their config sections they rest at zero.
    const std::span<double> afcs_rates = out.xdot0.subspan(kPitchAd, kNumStates - kPitchAd);
    std::fill(afcs_rates.begin(), afcs_rates.end(), 0.0);
    if (!config_.sas) {
        return;
    }
    for (std::size_t i = kPitchAd; i < kNumStates; ++i) {
        if (tau_[i] > 0.0) {
            out.xdot0[i] = (s.in[i] - x[i]) / tau_[i];
        }
    }
    const GenHelSas& sas = *config_.sas;
    const ServoConfig shaping{sas.roll.shaping_a2, sas.roll.shaping_a1, 0.0};
    run(shaping, s.in[kRollShape], kRollShape);
    run(sas.actuator, s.in[kPitchAct], kPitchAct);
    run(sas.actuator, s.in[kRollAct], kRollAct);
    run(sas.actuator, s.in[kYawAct], kYawAct);
    // The 2 s fade: toward its target at 1/T_fade, settling within 1/(T k).
    const double rate = 1.0 / sas.fade_time;
    out.xdot0[kYawFade] = std::clamp(kFadeGain * (s.in[kYawFade] - x[kYawFade]), -rate, rate);
    // PBA actuator (Fig. 5.1.4, p. 5.5-3): gain on the error to the held
    // command, rate limit, and travel limit, where it stops (D-045: GenHel's
    // limit after the integrator would let it wind up).
    if (config_.pitch_bias) {
        const GenHelPitchBias& pb = *config_.pitch_bias;
        double v = limit(pb.actuator_gain * (s.pba_held - x[kPba]), pb.rate_limit);
        if ((x[kPba] >= pb.authority && v > 0.0) || (x[kPba] <= -pb.authority && v < 0.0)) {
            v = 0.0;
        }
        out.xdot0[kPba] = v;
    }
}

void GenHelFcs::steady_states(const SimContext& ctx, std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);  // every velocity state at rest
    const BodyState& b = ctx.body;
    x[kGyroP] = b.omega_b.x();
    x[kGyroQ] = b.omega_b.y();
    x[kGyroR] = b.omega_b.z();
    x[kAccelY] = lateral_specific_force0(b, config_.accelerometer_b);
    x[kAirspeedLag] = body_x_eas(ctx);
    // SAS and PBA (GenHel's IC mode, p. 5.5-2: integrators at their initial
    // values): every state at its input, pass by pass down the chains, so the
    // washouts read zero and the lags their steady input; the fade at its
    // target; the PBA at its held command within its travel.
    if (config_.sas) {
        for (int pass = 0; pass < kSteadyPasses; ++pass) {
            const AfcsSignals s = afcs_signals(config_, ctx, x);
            for (std::size_t i = kPitchAd; i < kNumStates; ++i) {
                x[i] = s.in[i];
            }
            for (const std::size_t i : {kRollShape, kPitchAct, kRollAct, kYawAct}) {
                x[i + 1] = 0.0;  // the second-order states' velocities
            }
            x[kPba] = config_.pitch_bias ? limit(s.pba_held, config_.pitch_bias->authority) : 0.0;
        }
    }
    // The commands read the sensors and inner loop just set.
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

// {gain, lag}: a lag with its gain in `gain_dim`, or a plain number when
// `gain_dim` is dimensionless.
Lag lag_from_config(const Config& node, Dimension gain_dim) {
    node.allow_keys({"gain", "lag"});
    Lag l;
    l.gain = gain_dim == Dimension::dimensionless ? node.number("gain")
                                                  : node.quantity("gain", gain_dim);
    l.tau = node.quantity("lag", Dimension::time);
    return l;
}

GenHelSasPitch sas_pitch_from_config(const Config& node) {
    node.allow_keys({"digital", "analog"});
    GenHelSasPitch p;
    const Config d = node.child("digital");
    d.allow_keys({"ad_lag", "rate", "lagged_rate", "washout", "limit"});
    p.ad_lag = d.quantity("ad_lag", Dimension::time);
    p.rate_d = lag_from_config(d.child("rate"), Dimension::per_angular_rate);
    p.lagged_rate_d = lag_from_config(d.child("lagged_rate"), Dimension::per_angular_rate);
    p.washout_d = d.quantity("washout", Dimension::time);
    p.limit_d = d.quantity("limit", Dimension::dimensionless);
    const Config a = node.child("analog");
    a.allow_keys({"filter", "washout", "rate", "lagged_rate", "scale", "limit"});
    p.filter_a = lag_from_config(a.child("filter"), Dimension::dimensionless);
    p.washout_a = a.quantity("washout", Dimension::time);
    p.rate_a = lag_from_config(a.child("rate"), Dimension::per_angular_rate);
    p.lagged_rate_a = lag_from_config(a.child("lagged_rate"), Dimension::per_angular_rate);
    p.scale_a = a.number("scale");
    p.limit_a = a.quantity("limit", Dimension::dimensionless);
    return p;
}

GenHelSasRoll sas_roll_from_config(const Config& node) {
    node.allow_keys({"digital", "analog"});
    GenHelSasRoll r;
    const Config d = node.child("digital");
    d.allow_keys({"ad_lag", "rate_gain", "lagged_rate", "attitude", "limit"});
    r.ad_lag = d.quantity("ad_lag", Dimension::time);
    r.rate_gain_d = d.quantity("rate_gain", Dimension::per_angular_rate);
    r.lagged_rate_d = lag_from_config(d.child("lagged_rate"), Dimension::per_angular_rate);
    const Config da = d.child("attitude");
    da.allow_keys({"gain", "limit"});
    r.attitude_gain_d = da.quantity("gain", Dimension::per_angle);
    r.attitude_limit_d = da.quantity("limit", Dimension::angle);
    r.limit_d = d.quantity("limit", Dimension::dimensionless);
    const Config a = node.child("analog");
    a.allow_keys(
        {"attitude_filter", "shaping", "derivative", "rate", "lagged_rate", "attitude", "limit"});
    const Config f = a.child("attitude_filter");
    f.allow_keys({"gain", "lag1", "lag2"});
    r.attitude_filter_gain = f.number("gain");
    r.attitude_filter_t1 = f.quantity("lag1", Dimension::time);
    r.attitude_filter_t2 = f.quantity("lag2", Dimension::time);
    const Config s = a.child("shaping");
    s.allow_keys({"gain", "a2", "a1", "limit"});
    r.shaping_gain = s.number("gain");
    r.shaping_a2 = s.quantity("a2", Dimension::time_squared);
    r.shaping_a1 = s.quantity("a1", Dimension::time);
    r.shaping_limit = s.quantity("limit", Dimension::angle);
    const Config der = a.child("derivative");
    der.allow_keys({"gain", "lag1", "lag2"});
    r.derivative_gain = der.number("gain");
    r.derivative_t1 = der.quantity("lag1", Dimension::time);
    r.derivative_t2 = der.quantity("lag2", Dimension::time);
    r.rate_a = lag_from_config(a.child("rate"), Dimension::per_angular_rate);
    r.lagged_rate_a = lag_from_config(a.child("lagged_rate"), Dimension::per_angular_rate);
    const Config aa = a.child("attitude");
    aa.allow_keys({"gain", "limit", "output_gain"});
    r.attitude_gain_a = aa.number("gain");
    r.attitude_limit_a = aa.quantity("limit", Dimension::angle);
    r.attitude_output_gain_a = aa.quantity("output_gain", Dimension::per_angle);
    r.limit_a = a.quantity("limit", Dimension::dimensionless);
    return r;
}

GenHelSasYaw sas_yaw_from_config(const Config& node) {
    node.allow_keys({"digital", "analog"});
    GenHelSasYaw y;
    const Config d = node.child("digital");
    d.allow_keys(
        {"ad_lag", "rate", "lagged_rate", "washout", "lateral_accel", "roll_rate_gain", "limit"});
    y.ad_lag = d.quantity("ad_lag", Dimension::time);
    y.rate_d = lag_from_config(d.child("rate"), Dimension::per_angular_rate);
    y.lagged_rate_d = lag_from_config(d.child("lagged_rate"), Dimension::per_angular_rate);
    y.washout_d = d.quantity("washout", Dimension::time);
    y.lateral_accel_d = lag_from_config(d.child("lateral_accel"), Dimension::per_acceleration);
    y.roll_rate_gain_d = d.quantity("roll_rate_gain", Dimension::per_angular_rate);
    y.limit_d = d.quantity("limit", Dimension::dimensionless);
    const Config a = node.child("analog");
    a.allow_keys({"filter", "washout", "rate", "lateral_accel_gain", "lagged", "scale",
                  "roll_rate_gain", "limit"});
    y.filter_a = lag_from_config(a.child("filter"), Dimension::dimensionless);
    y.washout_a = a.quantity("washout", Dimension::time);
    y.rate_a = lag_from_config(a.child("rate"), Dimension::per_angular_rate);
    y.lateral_accel_gain_a = a.quantity("lateral_accel_gain", Dimension::rate_per_acceleration);
    y.lagged_a = lag_from_config(a.child("lagged"), Dimension::per_angular_rate);
    y.scale_a = a.number("scale");
    y.roll_rate_gain_a = a.number("roll_rate_gain");
    y.limit_a = a.quantity("limit", Dimension::dimensionless);
    return y;
}

GenHelSas sas_from_config(const Config& node) {
    node.allow_keys(
        {"hold", "switch_speed", "fade_time", "actuator", "descaling", "pitch", "roll", "yaw"});
    GenHelSas s;
    s.hold = node.quantity("hold", Dimension::time);
    s.switch_speed = node.quantity("switch_speed", Dimension::speed);
    s.fade_time = node.quantity("fade_time", Dimension::time);
    s.actuator = servo_from_config(node.child("actuator"), false);
    const Config ds = node.child("descaling");
    ds.allow_keys({"pitch", "roll", "yaw"});
    s.descale_pitch = ds.quantity("pitch", Dimension::angle);
    s.descale_roll = ds.quantity("roll", Dimension::angle);
    s.descale_yaw = ds.quantity("yaw", Dimension::angle);
    s.pitch = sas_pitch_from_config(node.child("pitch"));
    s.roll = sas_roll_from_config(node.child("roll"));
    s.yaw = sas_yaw_from_config(node.child("yaw"));
    return s;
}

GenHelPitchBias pitch_bias_from_config(const Config& node) {
    node.allow_keys({"attitude", "pitch_rate", "airspeed", "bias", "hold", "actuator"});
    GenHelPitchBias b;
    const Config at = node.child("attitude");
    at.allow_keys({"gain", "limit"});
    b.attitude_gain = at.quantity("gain", Dimension::per_angle);
    b.attitude_limit = at.quantity("limit", Dimension::angle);
    const Config pr = node.child("pitch_rate");
    pr.allow_keys({"gain", "limit"});
    b.rate_gain = pr.quantity("gain", Dimension::per_angular_rate);
    b.rate_limit_in = pr.quantity("limit", Dimension::angular_rate);
    const Config as = node.child("airspeed");
    as.allow_keys({"gain", "from", "range"});
    b.speed_gain = as.quantity("gain", Dimension::per_speed);
    b.speed_from = as.quantity("from", Dimension::speed);
    b.speed_range = as.quantity("range", Dimension::speed);
    b.bias = node.quantity("bias", Dimension::dimensionless);
    b.hold = node.quantity("hold", Dimension::time);
    const Config ac = node.child("actuator");
    ac.allow_keys({"gain", "rate_limit", "authority"});
    b.actuator_gain = ac.quantity("gain", Dimension::frequency);
    b.rate_limit = ac.quantity("rate_limit", Dimension::frequency);
    b.authority = ac.quantity("authority", Dimension::dimensionless);
    return b;
}

}  // namespace

std::unique_ptr<GenHelFcs> GenHelFcs::from_config(const Config& node) {
    node.allow_keys({"type", "name", "rigging", "primary_servo", "rate_gyro",
                     "lateral_accelerometer", "stabilator", "sas", "pitch_bias"});
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
    if (node.contains("sas")) {
        c.sas = sas_from_config(node.child("sas"));
    }
    if (node.contains("pitch_bias")) {
        c.pitch_bias = pitch_bias_from_config(node.child("pitch_bias"));
    }
    return node.in_context("", [&] { return std::make_unique<GenHelFcs>(std::move(c)); });
}

}  // namespace eyas
