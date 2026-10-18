// M8c2 (D-045): the GenHel SAS and pitch bias actuator, against GenHel's
// block diagrams written out test-side in GenHel's own units (deg, deg/s,
// ft/s², kt, %, in): Figs. 5.1.3(a)–(c) (SAS), Fig. 5.1.4 (PBA) and the
// inner-loop junctions of Figs. 5.1.6(b)–(d), with the text pp. 5.5-2 and
// 5.5-3. The zero-order holds are a first-order Padé of half the hold
// (D-045), gated here against the hold's exact frequency response.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "control/genhel_fcs.hpp"
#include "core/attitude.hpp"
#include "env/air_data.hpp"
#include "env/atmosphere.hpp"
#include "support/alloc_counter.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kDegree;
using units::kFoot;
using units::kKnot;
using Fcs = GenHelFcs;

constexpr double kInch = 0.0254;
constexpr double kPct = 0.01;  // one percent, as a fraction

// GenHel's values (data/uh60a/genhel/fig_5_1_3_sas.txt, fig_5_1_4_pba.txt,
// control_rigging_5_5.txt), in GenHel's units, test-side.
struct GenHel {
    // Pitch, Fig. 5.1.3(a).
    double P_AD = 0.026, KRQQPD = -0.5425, TRQQPD = 0.023, KLQQPD = -0.541, TLQQPD = 2.0;
    double TWQQPD = 7.0, LMPD = 5.0;
    double KQQPA = 0.1785, TQQPA = 0.022, TWQQPA = 7.0, KRQQPA = -6.194, TRQQPA = 0.024;
    double KLQQPA = -6.047, TLQQPA = 1.993, HALF = 0.5, LMPA = 5.5;
    // Roll, Fig. 5.1.3(b).
    double R_AD = 0.026, KRPPRD = -0.2085, KLPPRD = -0.1035, TLPPRD = 1.02, LMPHRD = 7.0;
    double KPHRD = -0.139, LMRD = 5.0;
    double KPHF1 = 0.0885, T1PHF1 = 0.0058, T2PHF1 = 0.00154;
    double KPHF2 = 1.06, A2PHF2 = 0.00203, A1PHF2 = 0.0655, LMPHR1 = 75.0;
    double KPHF3 = 1.24, T1PHF3 = 0.0091, T2PHF3 = 0.00195;
    double KRPHRA = -1.738, TRPHRA = 0.0112, KLPHRA = -0.847, TLPHRA = 0.996;
    double KPHR1 = -0.703, LMPHRA = 7.0, KPHIRA = 2.413, LMRA = 5.5;
    // Yaw, Fig. 5.1.3(c).
    double Y_AD = 0.026, KRRRYD = -0.557, TRRRYD = 0.047, KLRRYD = -0.901, TLRRYD = 1.0;
    double TWRRYD = 2.0, KLAYYD = -1.1, TLAYYD = 1.0, KRPPYD = 0.1378, LMYD = 5.0;
    double KRRFI = 0.072, TRRFI = 0.027, TWRRYA = 2.0, KRRRYA = -16.95, TRRRYA = 0.047;
    double KLAYF1 = 0.0876, KLAYRR = -28.11, TLAYRR = 1.045, KRPPYA = 0.296, LMYA = 5.5;
    double V_SW = 60.0, T_FADE = 2.0;
    // Common: hold, SAS actuator, descaling (p. 5.5-20), rigging gains.
    double HOLD = 0.025, ACT_A2 = 0.00032, ACT_A1 = 0.025;
    double KPCDGP = -0.283, KPCDGR = 0.16, KPCDGY = -0.298;
    double KXAA1S = 1.6, KXBB1S = -2.83, KXPTHR = -5.539;  // [deg/in]
    // PBA, Fig. 5.1.4 (LMQQBA in deg/s, human review 2026-10-18).
    double LMTHBA = 20.0, KTHBA = -0.375, LMQQBA = 34.5, KQQBA = -0.13125;
    double BIAS_VX = -80.0, LMVXBA = 100.0, KVXBA = 0.0525, BIAS_BA = -4.9;
    double ZOH_BA = 0.1, GAIN_BA = 10.0, RATE_BA = 3.0, AUTH_BA = 15.0;
};

GenHelFcsConfig config() {
    const GenHel g;
    GenHelFcsConfig c;
    c.name = "fcs";
    GenHelRigging& r = c.rigging;
    r.collective = {0.0, 10.0 * kInch};
    r.lat_cyclic = {0.0, 10.0 * kInch};
    r.lon_cyclic = {-3.0 * kInch, 7.0 * kInch};
    r.pedal = {0.0, 5.4 * kInch};
    r.k_collective = 1.6 * kDegree / kInch;
    r.k_lateral = g.KXAA1S * kDegree / kInch;
    r.k_longitudinal = g.KXBB1S * kDegree / kInch;
    r.k_pedal = g.KXPTHR * kDegree / kInch;
    r.sign_lon_stick = -1.0;
    r.mix_longitudinal_pedal = -0.2936;
    c.primary_servo = {0.00114, 0.0473, 0.0};
    c.rate_gyro = {0.00014, 0.016, 0.0};
    c.lateral_accelerometer = {0.0254, 0.233, 0.0};
    c.accelerometer_b = Vec3(1.27, 0.0, -0.47);
    GenHelStabilatorLaw& l = c.stabilator;
    l.schedule.command = Table2D(Axis({0.0, 100.0}), Axis({0.0, 1.0}), {0.1, 0.1, 0.0, 0.0});
    l.schedule.min = -8.0 * kDegree;
    l.schedule.max = 39.0 * kDegree;
    l.airspeed_lag = 0.398;
    l.ay_speed_lo = 30.0 * kKnot;
    l.ay_speed_hi = 60.0 * kKnot;
    l.servo = {0.0254, 0.223, 7.0 * kDegree};

    const double per_rate = kPct / kDegree;  // %/(deg/s) → 1/(rad/s)
    GenHelSas s;
    GenHelSasPitch& p = s.pitch;
    p.ad_lag = g.P_AD;
    p.rate_d = {g.KRQQPD * per_rate, g.TRQQPD};
    p.lagged_rate_d = {g.KLQQPD * per_rate, g.TLQQPD};
    p.washout_d = g.TWQQPD;
    p.limit_d = g.LMPD * kPct;
    p.filter_a = {g.KQQPA, g.TQQPA};
    p.washout_a = g.TWQQPA;
    p.rate_a = {g.KRQQPA * per_rate, g.TRQQPA};
    p.lagged_rate_a = {g.KLQQPA * per_rate, g.TLQQPA};
    p.scale_a = g.HALF;
    p.limit_a = g.LMPA * kPct;
    GenHelSasRoll& o = s.roll;
    o.ad_lag = g.R_AD;
    o.rate_gain_d = g.KRPPRD * per_rate;
    o.lagged_rate_d = {g.KLPPRD * per_rate, g.TLPPRD};
    o.attitude_gain_d = g.KPHRD * kPct / kDegree;
    o.attitude_limit_d = g.LMPHRD * kDegree;
    o.limit_d = g.LMRD * kPct;
    o.attitude_filter_gain = g.KPHF1;
    o.attitude_filter_t1 = g.T1PHF1;
    o.attitude_filter_t2 = g.T2PHF1;
    o.shaping_gain = g.KPHF2;
    o.shaping_a2 = g.A2PHF2;
    o.shaping_a1 = g.A1PHF2;
    o.shaping_limit = g.LMPHR1 * kDegree;
    o.derivative_gain = g.KPHF3;
    o.derivative_t1 = g.T1PHF3;
    o.derivative_t2 = g.T2PHF3;
    o.rate_a = {g.KRPHRA * per_rate, g.TRPHRA};
    o.lagged_rate_a = {g.KLPHRA * per_rate, g.TLPHRA};
    o.attitude_gain_a = g.KPHR1;
    o.attitude_limit_a = g.LMPHRA * kDegree;
    o.attitude_output_gain_a = g.KPHIRA * kPct / kDegree;
    o.limit_a = g.LMRA * kPct;
    GenHelSasYaw& y = s.yaw;
    y.ad_lag = g.Y_AD;
    y.rate_d = {g.KRRRYD * per_rate, g.TRRRYD};
    y.lagged_rate_d = {g.KLRRYD * per_rate, g.TLRRYD};
    y.washout_d = g.TWRRYD;
    y.lateral_accel_d = {g.KLAYYD * kPct / kFoot, g.TLAYYD};
    y.roll_rate_gain_d = g.KRPPYD * per_rate;
    y.limit_d = g.LMYD * kPct;
    y.filter_a = {g.KRRFI, g.TRRFI};
    y.washout_a = g.TWRRYA;
    y.rate_a = {g.KRRRYA * per_rate, g.TRRRYA};
    y.lateral_accel_gain_a = g.KLAYF1 * kDegree / kFoot;
    y.lagged_a = {g.KLAYRR * per_rate, g.TLAYRR};
    y.scale_a = g.HALF;
    y.roll_rate_gain_a = g.KRPPYA;
    y.limit_a = g.LMYA * kPct;
    s.hold = g.HOLD;
    s.switch_speed = g.V_SW * kKnot;
    s.fade_time = g.T_FADE;
    s.actuator = {g.ACT_A2, g.ACT_A1, 0.0};
    s.descale_pitch = g.KPCDGP * kDegree / kPct;
    s.descale_roll = g.KPCDGR * kDegree / kPct;
    s.descale_yaw = g.KPCDGY * kDegree / kPct;
    c.sas = s;

    GenHelPitchBias b;
    b.attitude_gain = g.KTHBA * kPct / kDegree;
    b.attitude_limit = g.LMTHBA * kDegree;
    b.rate_gain = g.KQQBA * per_rate;
    b.rate_limit_in = g.LMQQBA * kDegree;
    b.speed_gain = g.KVXBA * kPct / kKnot;
    b.speed_from = -g.BIAS_VX * kKnot;
    b.speed_range = g.LMVXBA * kKnot;
    b.bias = g.BIAS_BA * kPct;
    b.hold = g.ZOH_BA;
    b.actuator_gain = g.GAIN_BA;
    b.rate_limit = g.RATE_BA * kPct;
    b.authority = g.AUTH_BA * kPct;
    c.pitch_bias = b;
    return c;
}

// The GenHel unit of each state, as its SI value per GenHel unit: lags of a
// rate in deg/s, of an attitude in deg, of an acceleration in ft/s², of a
// channel signal in %; actuators in inches and in/s.
std::array<double, Fcs::kNumStates> genhel_units() {
    std::array<double, Fcs::kNumStates> u{};
    u.fill(std::numeric_limits<double>::quiet_NaN());
    for (const std::size_t i :
         {Fcs::kPitchAd, Fcs::kPitchRateD, Fcs::kPitchLagD, Fcs::kPitchFilterA, Fcs::kPitchWashA,
          Fcs::kPitchRateA, Fcs::kPitchLagA, Fcs::kRollAd, Fcs::kRollLagD, Fcs::kRollShape + 1,
          Fcs::kRollRateA, Fcs::kRollLagA, Fcs::kYawAd, Fcs::kYawRateD, Fcs::kYawLagD,
          Fcs::kYawFilterA, Fcs::kYawWashA, Fcs::kYawRateA, Fcs::kYawLagA}) {
        u[i] = kDegree;  // deg/s
    }
    for (const std::size_t i :
         {Fcs::kRollAtt1, Fcs::kRollAtt2, Fcs::kRollShape, Fcs::kRollDer1, Fcs::kRollDer2}) {
        u[i] = kDegree;  // deg
    }
    for (const std::size_t i : {Fcs::kPitchWashD, Fcs::kPitchHoldD, Fcs::kRollHoldD, Fcs::kYawWashD,
                                Fcs::kYawHoldD, Fcs::kPbaHold, Fcs::kPba}) {
        u[i] = kPct;
    }
    u[Fcs::kYawAccelD] = kFoot;
    u[Fcs::kYawFade] = 1.0;
    for (const std::size_t i : {Fcs::kPitchAct, Fcs::kRollAct, Fcs::kYawAct}) {
        u[i] = kInch;
        u[i + 1] = kInch;
    }
    return u;
}

// The signals the SAS and PBA read, in GenHel's units.
struct Inputs {
    double p = 0.0, q = 0.0, r = 0.0;  // rate gyro outputs PSENSR, QSENSR, RSENSR [deg/s]
    double ysens = 0.0;                // Y.SENS [ft/s²]
    double phi = 0.0, theta = 0.0;     // PHIB, THETAB [deg]
    double v_kt = 0.0;                 // VXBIKT [kt]
};

double lim(double v, double l) {
    return std::clamp(v, -l, l);
}

// The rates of the SAS and PBA states, in GenHel's units per second, at
// states s (GenHel's units) and the switches; written from the diagrams.
// Also the channel outputs, after the holds [%], and the PBA output [%].
struct Reference {
    std::array<double, Fcs::kNumStates> rate{};
    double pd = 0, pa = 0, rd = 0, ra = 0, yd = 0, ya = 0, pba = 0;
};

Reference reference(const GenHel& g, const Inputs& in, const std::array<double, Fcs::kNumStates>& s,
                    const AfcsSwitches& sw) {
    Reference o;
    auto& d = o.rate;
    // The 2:1 gain switch (Fig. 5.1.3(a)): ON/ON 1, one channel alone 2.
    const double sw_d = sw.sas_digital ? (sw.sas_analog ? 1.0 : 2.0) : 0.0;
    const double sw_a = sw.sas_analog ? (sw.sas_digital ? 1.0 : 2.0) : 0.0;
    // A Padé (1 − sT/4)/(1 + sT/4) of a hold T: lag z of T/4, output 2z − u.
    const auto hold = [&](std::size_t i, double u, double T) {
        d[i] = (u - s[i]) / (T / 4.0);
        return 2.0 * s[i] - u;
    };
    const bool slow = in.v_kt < g.V_SW;

    // Pitch digital: A/D, rate + lagged rate, washout, switch, limit, hold.
    d[Fcs::kPitchAd] = (in.q - s[Fcs::kPitchAd]) / g.P_AD;
    const double q1 = s[Fcs::kPitchAd];
    d[Fcs::kPitchRateD] = (q1 - s[Fcs::kPitchRateD]) / g.TRQQPD;
    d[Fcs::kPitchLagD] = (q1 - s[Fcs::kPitchLagD]) / g.TLQQPD;
    const double pd_sum = g.KRQQPD * s[Fcs::kPitchRateD] + g.KLQQPD * s[Fcs::kPitchLagD];
    d[Fcs::kPitchWashD] = (pd_sum - s[Fcs::kPitchWashD]) / g.TWQQPD;
    const double pd_in = lim(sw_d * (pd_sum - s[Fcs::kPitchWashD]), g.LMPD);
    o.pd = hold(Fcs::kPitchHoldD, pd_in, g.HOLD);
    // Pitch analog: KQQPA (7s)/((.022s+1)(7s+1)), rate + lagged rate, .5, switch, limit.
    d[Fcs::kPitchFilterA] = (in.q - s[Fcs::kPitchFilterA]) / g.TQQPA;
    const double qa = g.KQQPA * s[Fcs::kPitchFilterA];
    d[Fcs::kPitchWashA] = (qa - s[Fcs::kPitchWashA]) / g.TWQQPA;
    const double q2 = qa - s[Fcs::kPitchWashA];
    d[Fcs::kPitchRateA] = (q2 - s[Fcs::kPitchRateA]) / g.TRQQPA;
    d[Fcs::kPitchLagA] = (q2 - s[Fcs::kPitchLagA]) / g.TLQQPA;
    o.pa = lim(sw_a * g.HALF * (g.KRQQPA * s[Fcs::kPitchRateA] + g.KLQQPA * s[Fcs::kPitchLagA]),
               g.LMPA);

    // Roll digital: A/D, rate + lagged rate + limited attitude, switch, limit, hold.
    d[Fcs::kRollAd] = (in.p - s[Fcs::kRollAd]) / g.R_AD;
    const double p1 = s[Fcs::kRollAd];
    d[Fcs::kRollLagD] = (p1 - s[Fcs::kRollLagD]) / g.TLPPRD;
    const double rd_temp =
        g.KRPPRD * p1 + g.KLPPRD * s[Fcs::kRollLagD] + g.KPHRD * lim(in.phi, g.LMPHRD);
    o.rd = hold(Fcs::kRollHoldD, lim(sw_d * rd_temp, g.LMRD), g.HOLD);
    // Roll analog: φ → KPHF1/((T1 s+1)(T2 s+1)) = f1 → KPHF2/(A2 s² + A1 s + 1),
    // limit ±75° → KPHF3 s/((T1 s+1)(T2 s+1)) = p2 → KRPHRA, KLPHRA lags; and
    // f1 → KPHR1, limit ±7°, KPHIRA.
    d[Fcs::kRollAtt1] = (g.KPHF1 * in.phi - s[Fcs::kRollAtt1]) / g.T1PHF1;
    d[Fcs::kRollAtt2] = (s[Fcs::kRollAtt1] - s[Fcs::kRollAtt2]) / g.T2PHF1;
    const double f1 = s[Fcs::kRollAtt2];
    d[Fcs::kRollShape] = s[Fcs::kRollShape + 1];
    d[Fcs::kRollShape + 1] =
        (g.KPHF2 * f1 - s[Fcs::kRollShape] - g.A1PHF2 * s[Fcs::kRollShape + 1]) / g.A2PHF2;
    const double f2 = lim(s[Fcs::kRollShape], g.LMPHR1);
    d[Fcs::kRollDer1] = (f2 - s[Fcs::kRollDer1]) / g.T1PHF3;
    d[Fcs::kRollDer2] = (s[Fcs::kRollDer1] - s[Fcs::kRollDer2]) / g.T2PHF3;
    const double p2 = g.KPHF3 * d[Fcs::kRollDer2];
    d[Fcs::kRollRateA] = (p2 - s[Fcs::kRollRateA]) / g.TRPHRA;
    d[Fcs::kRollLagA] = (p2 - s[Fcs::kRollLagA]) / g.TLPHRA;
    const double r_a = g.KRPHRA * s[Fcs::kRollRateA];  // "B", to the yaw analog channel
    const double ra_temp =
        r_a + g.KLPHRA * s[Fcs::kRollLagA] + g.KPHIRA * lim(g.KPHR1 * f1, g.LMPHRA);
    o.ra = lim(sw_a * ra_temp, g.LMRA);

    // Yaw digital: A/D; rate, plus the lagged rate below 60 kt; washout; plus,
    // faded in above 60 kt, lagged lateral acceleration and the roll A/D
    // output ("A"); switch, limit, hold.
    d[Fcs::kYawAd] = (in.r - s[Fcs::kYawAd]) / g.Y_AD;
    const double r1 = s[Fcs::kYawAd];
    d[Fcs::kYawRateD] = (r1 - s[Fcs::kYawRateD]) / g.TRRRYD;
    d[Fcs::kYawLagD] = ((slow ? r1 : 0.0) - s[Fcs::kYawLagD]) / g.TLRRYD;
    const double s1 = g.KRRRYD * s[Fcs::kYawRateD] + g.KLRRYD * s[Fcs::kYawLagD];
    d[Fcs::kYawWashD] = (s1 - s[Fcs::kYawWashD]) / g.TWRRYD;
    d[Fcs::kYawAccelD] = (in.ysens - s[Fcs::kYawAccelD]) / g.TLAYYD;
    const double fade = s[Fcs::kYawFade];
    const double s2 = fade * (g.KLAYYD * s[Fcs::kYawAccelD] + g.KRPPYD * p1);
    o.yd = hold(Fcs::kYawHoldD, lim(sw_d * (s1 - s[Fcs::kYawWashD] + s2), g.LMYD), g.HOLD);
    // Yaw analog: KRRFI (2s)/((.027s+1)(2s+1)) = w; KRRRYA lag on w; KLAYRR lag
    // on w below 60 kt, on KLAYF1·Y.SENS above; .5; plus, faded in above
    // 60 kt, KRPPYA on the roll channel's "B"; switch, limit.
    d[Fcs::kYawFilterA] = (in.r - s[Fcs::kYawFilterA]) / g.TRRFI;
    const double ra_f = g.KRRFI * s[Fcs::kYawFilterA];
    d[Fcs::kYawWashA] = (ra_f - s[Fcs::kYawWashA]) / g.TWRRYA;
    const double w = ra_f - s[Fcs::kYawWashA];
    d[Fcs::kYawRateA] = (w - s[Fcs::kYawRateA]) / g.TRRRYA;
    const double u = slow ? w : g.KLAYF1 * in.ysens;
    d[Fcs::kYawLagA] = (u - s[Fcs::kYawLagA]) / g.TLAYRR;
    const double s3 = g.HALF * (g.KRRRYA * s[Fcs::kYawRateA] + g.KLAYRR * s[Fcs::kYawLagA]);
    o.ya = lim(sw_a * (s3 + fade * g.KRPPYA * r_a), g.LMYA);

    // SAS actuators on the inches: % → deg (KPCDG*) → in (1/KX*).
    const auto act = [&](std::size_t i, double pct, double kpcdg, double kx) {
        const double cmd = pct * kpcdg / kx;
        d[i] = s[i + 1];
        d[i + 1] = (cmd - s[i] - g.ACT_A1 * s[i + 1]) / g.ACT_A2;
    };
    act(Fcs::kPitchAct, o.pd + o.pa, g.KPCDGP, g.KXBB1S);
    act(Fcs::kRollAct, o.rd + o.ra, g.KPCDGR, g.KXAA1S);
    act(Fcs::kYawAct, o.yd + o.ya, g.KPCDGY, g.KXPTHR);

    // PBA: limited attitude, conditioned rate and airspeed, bias; hold;
    // actuator gain 10, rate ±3 %/s, travel ±15 % (stops there).
    const double batemp =
        sw.pitch_bias ? g.KTHBA * lim(in.theta, g.LMTHBA) + g.KQQBA * lim(q1, g.LMQQBA) +
                            g.KVXBA * std::clamp(in.v_kt + g.BIAS_VX, 0.0, g.LMVXBA) + g.BIAS_BA
                      : 0.0;
    const double held = hold(Fcs::kPbaHold, batemp, g.ZOH_BA);
    double v = lim(g.GAIN_BA * (held - s[Fcs::kPba]), g.RATE_BA);
    if ((s[Fcs::kPba] >= g.AUTH_BA && v > 0.0) || (s[Fcs::kPba] <= -g.AUTH_BA && v < 0.0)) {
        v = 0.0;
    }
    d[Fcs::kPba] = v;
    o.pba = lim(s[Fcs::kPba], g.AUTH_BA);
    return o;
}

struct Eval {
    std::vector<double> xdot0 = std::vector<double>(Fcs::kNumStates, 0.0);
    std::vector<double> G = std::vector<double>(Fcs::kNumStates * 6, 0.0);
};

Eval derivs(const Fcs& f, const SimContext& ctx, std::span<const double> x) {
    Eval e;
    ComponentDerivs out{e.xdot0, e.G, Wrench{}, Mat66::Zero(), true, nullptr};
    f.derivatives(ctx, x, out);
    return e;
}

SimContext random_context(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    SimContext ctx;
    ctx.body.q_nb = quaternion(EulerAngles{0.3 * u(rng), 0.3 * u(rng), 3.0 * u(rng)});
    ctx.body.vel_b = Vec3(40.0 + 35.0 * u(rng), 5.0 * u(rng), 3.0 * u(rng));
    ctx.body.omega_b = Vec3(0.3 * u(rng), 0.3 * u(rng), 0.3 * u(rng));
    ctx.air = isa_atmosphere(1000.0 * (1.0 + u(rng)), 0.0);
    ctx.controls.collective = 0.5 + 0.5 * u(rng);
    ctx.controls.lat_cyclic = u(rng);
    ctx.controls.lon_cyclic = u(rng);
    ctx.controls.pedal = u(rng);
    ctx.controls.afcs.sas_digital = u(rng) > -0.5;
    ctx.controls.afcs.sas_analog = u(rng) > -0.5;
    ctx.controls.afcs.pitch_bias = u(rng) > -0.5;
    return ctx;
}

// Random states in GenHel's units: rates within ±20 deg/s, attitudes ±20°,
// channel signals ±8 % (some beyond the limits), actuators ±0.5 in.
std::array<double, Fcs::kNumStates> random_genhel_states(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    const auto units = genhel_units();
    std::array<double, Fcs::kNumStates> s{};
    for (std::size_t i = Fcs::kPitchAd; i < Fcs::kNumStates; ++i) {
        const double scale = units[i] == kPct ? 8.0 : units[i] == kInch ? 0.5 : 20.0;
        s[i] = scale * u(rng);
    }
    s[Fcs::kYawFade] = 0.5 * (1.0 + u(rng));
    s[Fcs::kPba] = 16.0 * u(rng);  // past the ±15 % stop at times
    return s;
}

// Every SAS and PBA state's rate, and every channel output, is GenHel's
// diagram (above, in GenHel's units) at random states, flight conditions and
// switch positions.
TEST(GenHelSas, ChannelsAreGenHelsDiagrams) {
    const GenHel g;
    const Fcs f(config());
    const auto units = genhel_units();
    std::mt19937_64 rng(8201);
    int slow = 0;
    for (int n = 0; n < 400; ++n) {
        SimContext ctx = random_context(rng);
        const auto s = random_genhel_states(rng);
        std::vector<double> x(Fcs::kNumStates, 0.0);
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        x[Fcs::kGyroP] = 0.3 * u(rng);
        x[Fcs::kGyroQ] = 0.3 * u(rng);
        x[Fcs::kGyroR] = 0.3 * u(rng);
        x[Fcs::kAccelY] = 3.0 * u(rng);
        x[Fcs::kAirspeedLag] = 40.0;
        for (std::size_t i = Fcs::kPitchAd; i < Fcs::kNumStates; ++i) {
            x[i] = s[i] * units[i];
        }
        const EulerAngles e = euler_angles(ctx.body.q_nb);
        Inputs in;
        in.p = x[Fcs::kGyroP] / kDegree;
        in.q = x[Fcs::kGyroQ] / kDegree;
        in.r = x[Fcs::kGyroR] / kDegree;
        in.ysens = x[Fcs::kAccelY] / kFoot;
        in.phi = e.roll / kDegree;
        in.theta = e.pitch / kDegree;
        const Vec3 air_b = ctx.body.vel_b - ctx.body.q_nb.conjugate() * ctx.env.wind_n;
        in.v_kt = equivalent_airspeed(ctx.air.density, air_b.x()) / kKnot;
        if (std::abs(in.v_kt - g.V_SW) < 1e-6) {
            continue;
        }
        slow += in.v_kt < g.V_SW ? 1 : 0;
        const Reference want = reference(g, in, s, ctx.controls.afcs);
        const Eval got = derivs(f, ctx, x);
        for (std::size_t i = Fcs::kPitchAd; i < Fcs::kNumStates; ++i) {
            if (i == Fcs::kYawFade) {
                continue;  // below
            }
            const double w = want.rate[i];
            EXPECT_NEAR(got.xdot0[i] / units[i], w, 1e-9 * (1.0 + std::abs(w))) << "state " << i;
            for (std::size_t k = 0; k < 6; ++k) {
                EXPECT_EQ(got.G[i * 6 + k], 0.0) << "state " << i;
            }
        }
        const GenHelFcsOutputs o = f.evaluate(ctx, x);
        EXPECT_NEAR(o.pitch_d / kPct, want.pd, 1e-10);
        EXPECT_NEAR(o.pitch_a / kPct, want.pa, 1e-10);
        EXPECT_NEAR(o.roll_d / kPct, want.rd, 1e-10);
        EXPECT_NEAR(o.roll_a / kPct, want.ra, 1e-10);
        EXPECT_NEAR(o.yaw_d / kPct, want.yd, 1e-10);
        EXPECT_NEAR(o.yaw_a / kPct, want.ya, 1e-10);
        EXPECT_NEAR(o.pba / kPct, want.pba, 1e-10);
        // The inner-loop inches are the actuator positions, and the PBA's
        // KPCDGP/KXBB1S times its position (Fig. 5.1.4).
        EXPECT_NEAR(o.xbils / kInch, s[Fcs::kPitchAct], 1e-12);
        EXPECT_NEAR(o.xails / kInch, s[Fcs::kRollAct], 1e-12);
        EXPECT_NEAR(o.xpils / kInch, s[Fcs::kYawAct], 1e-12);
        EXPECT_NEAR(o.xbbas / kInch, want.pba * g.KPCDGP / g.KXBB1S, 1e-12);
    }
    EXPECT_GT(slow, 40);  // both sides of the 60 kt switches
    EXPECT_LT(slow, 360);
}

// The inner-loop inches join the mixer's sums (Figs. 5.1.6(b)–(d)): XAILS
// into XASUM, XBILS and XBBAS into XBSUM, XPILS into XPSUM, which reaches
// B1S as well through MIX_BP. Seen as changes in the servo commands.
TEST(GenHelSas, InnerLoopInchesJoinTheMixer) {
    const GenHel g;
    const Fcs f(config());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.controls.collective = 0.5;
    std::vector<double> x(Fcs::kNumStates, 0.0);
    const GenHelFcsOutputs base = f.evaluate(ctx, x);
    x[Fcs::kRollAct] = 0.3 * kInch;
    x[Fcs::kPitchAct] = -0.2 * kInch;
    x[Fcs::kYawAct] = 0.1 * kInch;
    x[Fcs::kPba] = 4.0 * kPct;
    const GenHelFcsOutputs o = f.evaluate(ctx, x);
    const double xbbas = 4.0 * g.KPCDGP / g.KXBB1S;  // [in]
    const double thrsum = g.KXPTHR * 0.1;            // [deg]
    EXPECT_NEAR((o.a1s_cmd - base.a1s_cmd) / kDegree, g.KXAA1S * 0.3, 1e-12);
    EXPECT_NEAR((o.b1s_cmd - base.b1s_cmd) / kDegree, g.KXBB1S * (-0.2 + xbbas) - 0.2936 * thrsum,
                1e-12);
    EXPECT_NEAR((o.thettr_cmd - base.thettr_cmd) / kDegree, thrsum, 1e-12);
    EXPECT_NEAR(o.theta0_cmd, base.theta0_cmd, 1e-15);
    // The PBA's 4 % is KPCDGP · 4 = −1.13° of B1S, as the figure's B1SBA.
    EXPECT_NEAR(g.KXBB1S * xbbas, g.KPCDGP * 4.0, 1e-12);
}

// The 2 s fade (Fig. 5.1.3(c)): toward 1 above 60 kt and 0 below, at most
// 1/(2 s), settling on its target without passing it.
TEST(GenHelSas, YawFadeRunsOverTwoSeconds) {
    const GenHel g;
    const Fcs f(config());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    std::vector<double> x(Fcs::kNumStates, 0.0);
    x[Fcs::kAirspeedLag] = 40.0;
    const auto rate = [&](double kt, double fade) {
        ctx.body.vel_b = Vec3(kt * kKnot, 0.0, 0.0);
        x[Fcs::kYawFade] = fade;
        return derivs(f, ctx, x).xdot0[Fcs::kYawFade];
    };
    EXPECT_NEAR(rate(80.0, 0.0), 1.0 / g.T_FADE, 1e-12);
    EXPECT_NEAR(rate(80.0, 0.5), 1.0 / g.T_FADE, 1e-12);
    EXPECT_NEAR(rate(40.0, 1.0), -1.0 / g.T_FADE, 1e-12);
    EXPECT_NEAR(rate(40.0, 0.5), -1.0 / g.T_FADE, 1e-12);
    EXPECT_EQ(rate(80.0, 1.0), 0.0);
    EXPECT_EQ(rate(40.0, 0.0), 0.0);
    // Near the target it slows; it never points past it.
    EXPECT_GT(rate(80.0, 0.999), 0.0);
    EXPECT_LT(rate(80.0, 0.999), 1.0 / g.T_FADE);
    EXPECT_LT(rate(80.0, 1.001), 0.0);
    // Under RK4 at 1 ms: from 0 it reaches 0.5 at 1 s and stays within [0, 1].
    ctx.body.vel_b = Vec3(80.0 * kKnot, 0.0, 0.0);
    double y = 0.0;
    const double h = 1e-3;
    const auto fdot = [&](double v) { return rate(80.0, v); };
    for (int n = 1; n <= 3000; ++n) {
        const double k1 = fdot(y);
        const double k2 = fdot(y + 0.5 * h * k1);
        const double k3 = fdot(y + 0.5 * h * k2);
        const double k4 = fdot(y + h * k3);
        y += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        if (n == 1000) {
            EXPECT_NEAR(y, 0.5, 1e-9);
        }
        EXPECT_LE(y, 1.0 + 1e-12);
    }
    EXPECT_NEAR(y, 1.0, 1e-6);
}

// The PBA actuator (Fig. 5.1.4, p. 5.5-3): it slews at ±3 %/s toward the held
// command, stops at ±15 % without winding up, and with its switch off returns
// to centre at the same rate.
TEST(GenHelSas, PitchBiasActuatorLimits) {
    const GenHel g;
    const Fcs f(config());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.body.vel_b = Vec3(150.0 * kKnot, 0.0, 0.0);  // command 3.7 − 4.9 % at level pitch
    std::vector<double> x(Fcs::kNumStates, 0.0);
    x[Fcs::kAirspeedLag] = 70.0;
    ctx.controls.afcs.pitch_bias = true;
    const double cmd = g.KVXBA * 70.0 + g.BIAS_BA;  // [%]
    x[Fcs::kPbaHold] = cmd * kPct;
    x[Fcs::kPba] = 10.0 * kPct;  // far above: slews down at the limit
    EXPECT_NEAR(derivs(f, ctx, x).xdot0[Fcs::kPba] / kPct, -g.RATE_BA, 1e-12);
    x[Fcs::kPba] = (cmd + 0.1) * kPct;  // close: gain 10
    EXPECT_NEAR(derivs(f, ctx, x).xdot0[Fcs::kPba] / kPct, -g.GAIN_BA * 0.1, 1e-9);
    // At the stop, pushed outward: held there.
    x[Fcs::kPbaHold] = -30.0 * kPct;
    x[Fcs::kPba] = -15.0 * kPct;
    ctx.body.q_nb = quaternion(EulerAngles{0.0, 20.0 * kDegree, 0.0});
    EXPECT_EQ(derivs(f, ctx, x).xdot0[Fcs::kPba], 0.0);
    // Off: the command is zero, the hold follows it and the actuator centres.
    ctx.controls.afcs.pitch_bias = false;
    x[Fcs::kPbaHold] = 0.0;
    x[Fcs::kPba] = -5.0 * kPct;
    EXPECT_NEAR(derivs(f, ctx, x).xdot0[Fcs::kPba] / kPct, g.RATE_BA, 1e-12);
}

// D-045: each zero-order hold is a first-order Padé of half the hold,
// (1 − sT/4)/(1 + sT/4). Accuracy gate against the hold's exact frequency
// response, a delay of T/2 and the gain sin(ωT/2)/(ωT/2): within 1° of phase
// and 3 % of gain up to 5 Hz for the SAS's 25 ms (the body modes are below
// 1 Hz, the SAS actuator's corner is 9 Hz), and up to 1 Hz for the PBA's
// 0.1 s (its actuator moves at most 3 %/s).
TEST(GenHelSas, HoldPadeMatchesTheZeroOrderHold) {
    const GenHel g;
    for (const auto& [T, f_max] : {std::pair{g.HOLD, 5.0}, std::pair{g.ZOH_BA, 1.0}}) {
        double worst_phase = 0.0;
        double worst_gain = 0.0;
        for (int k = 1; k <= static_cast<int>(std::lround(f_max * 100.0)); ++k) {
            const double f = 0.01 * k;
            const double w = 2.0 * std::numbers::pi * f;
            const std::complex<double> s(0.0, w);
            const std::complex<double> pade = (1.0 - s * T / 4.0) / (1.0 + s * T / 4.0);
            const double x = w * T / 2.0;
            const std::complex<double> zoh = std::polar(std::sin(x) / x, -x);
            worst_phase = std::max(worst_phase, std::abs(std::arg(pade / zoh)) / kDegree);
            worst_gain = std::max(worst_gain, std::abs(std::abs(pade) / std::abs(zoh) - 1.0));
        }
        EXPECT_LT(worst_phase, 1.0) << T;
        EXPECT_LT(worst_gain, 0.03) << T;
    }
    // And the component's holds are that Padé: z' = (u − z)/(T/4), y = 2z − u
    // (ChannelsAreGenHelsDiagrams checks every one).
}

// GenHel's IC mode (p. 5.5-2) as steady_states(): every SAS and PBA state at
// its held-body equilibrium, any switches; washed-out paths read zero.
TEST(GenHelSas, SteadyStatesAreAHeldBodyEquilibrium) {
    const Fcs f(config());
    std::mt19937_64 rng(8202);
    for (int n = 0; n < 200; ++n) {
        const SimContext ctx = random_context(rng);
        std::vector<double> x(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        f.steady_states(ctx, x);
        for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
            ASSERT_TRUE(std::isfinite(x[i])) << i;
        }
        const Eval e = derivs(f, ctx, x);
        for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
            EXPECT_NEAR(e.xdot0[i], 0.0, 1e-9) << i;
        }
        // Rate-only channels read zero in steady flight; the pitch SAS has no
        // other input, so its output is zero.
        const GenHelFcsOutputs o = f.evaluate(ctx, x);
        EXPECT_NEAR(o.pitch_d, 0.0, 1e-15);
        EXPECT_NEAR(o.pitch_a, 0.0, 1e-15);
    }
}

// Switched off, the SAS and PBA put nothing into the mixer, whatever their
// filters hold: the M8c1 aircraft.
TEST(GenHelSas, SwitchedOffAddsNothing) {
    const Fcs f(config());
    GenHelFcsConfig bare = config();
    bare.sas.reset();
    bare.pitch_bias.reset();
    const Fcs m8c1(bare);
    std::mt19937_64 rng(8203);
    for (int n = 0; n < 100; ++n) {
        SimContext ctx = random_context(rng);
        ctx.controls.afcs = AfcsSwitches{};
        std::vector<double> x(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        f.steady_states(ctx, x);
        std::vector<double> y(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        m8c1.steady_states(ctx, y);
        for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
            if (i < Fcs::kPitchAd || i == Fcs::kPitchAct || i == Fcs::kRollAct ||
                i == Fcs::kYawAct || i == Fcs::kPba) {
                EXPECT_EQ(x[i], y[i]) << i;
            }
        }
        // Without the sections every new state is zero.
        for (std::size_t i = Fcs::kPitchAd; i < Fcs::kNumStates; ++i) {
            EXPECT_EQ(y[i], 0.0) << i;
        }
        const Eval e = derivs(m8c1, ctx, y);
        for (std::size_t i = Fcs::kPitchAd; i < Fcs::kNumStates; ++i) {
            EXPECT_EQ(e.xdot0[i], 0.0) << i;
        }
    }
}

// No allocation in derivatives() or steady_states() (spec §7).
TEST(GenHelSas, DoesNotAllocate) {
    const Fcs f(config());
    std::mt19937_64 rng(8204);
    const SimContext ctx = random_context(rng);
    std::vector<double> x(Fcs::kNumStates, 0.0);
    Eval e;
    ComponentDerivs out{e.xdot0, e.G, Wrench{}, Mat66::Zero(), true, nullptr};
    const test::AllocationCounter count;
    f.steady_states(ctx, x);
    f.derivatives(ctx, x, out);
    (void)f.evaluate(ctx, x);
    EXPECT_EQ(count.count(), 0U);
}

// The flying UH-60A's SAS and PBA have GenHel's values, read in GenHel's
// units (the tests above check the equations, this one the data).
TEST(GenHelSas, Uh60aConfigIsGenHels) {
    const test::Uh60aHarness ac;
    const GenHelFcsConfig& got = ac.get<GenHelFcs>("fcs").config();
    const GenHelFcsConfig want = config();
    ASSERT_TRUE(got.sas.has_value());
    ASSERT_TRUE(got.pitch_bias.has_value());
    const auto near = [](double a, double b, const char* what) {
        EXPECT_NEAR(a, b, 1e-12 * (1.0 + std::abs(b))) << what;
    };
    const auto lag = [&](const Lag& a, const Lag& b, const char* what) {
        near(a.gain, b.gain, what);
        near(a.tau, b.tau, what);
    };
    const GenHelSas& s = *got.sas;
    const GenHelSas& w = *want.sas;
    near(s.pitch.ad_lag, w.pitch.ad_lag, "P_AD");
    lag(s.pitch.rate_d, w.pitch.rate_d, "KRQQPD");
    lag(s.pitch.lagged_rate_d, w.pitch.lagged_rate_d, "KLQQPD");
    near(s.pitch.washout_d, w.pitch.washout_d, "TWQQPD");
    near(s.pitch.limit_d, w.pitch.limit_d, "LMPD");
    lag(s.pitch.filter_a, w.pitch.filter_a, "KQQPA");
    near(s.pitch.washout_a, w.pitch.washout_a, "TWQQPA");
    lag(s.pitch.rate_a, w.pitch.rate_a, "KRQQPA");
    lag(s.pitch.lagged_rate_a, w.pitch.lagged_rate_a, "KLQQPA");
    near(s.pitch.scale_a, w.pitch.scale_a, "pitch .5");
    near(s.pitch.limit_a, w.pitch.limit_a, "LMPA");
    near(s.roll.ad_lag, w.roll.ad_lag, "R_AD");
    near(s.roll.rate_gain_d, w.roll.rate_gain_d, "KRPPRD");
    lag(s.roll.lagged_rate_d, w.roll.lagged_rate_d, "KLPPRD");
    near(s.roll.attitude_gain_d, w.roll.attitude_gain_d, "KPHRD");
    near(s.roll.attitude_limit_d, w.roll.attitude_limit_d, "LMPHRD");
    near(s.roll.limit_d, w.roll.limit_d, "LMRD");
    near(s.roll.attitude_filter_gain, w.roll.attitude_filter_gain, "KPHF1");
    near(s.roll.attitude_filter_t1, w.roll.attitude_filter_t1, "T1PHF1");
    near(s.roll.attitude_filter_t2, w.roll.attitude_filter_t2, "T2PHF1");
    near(s.roll.shaping_gain, w.roll.shaping_gain, "KPHF2");
    near(s.roll.shaping_a2, w.roll.shaping_a2, "A2PHF2");
    near(s.roll.shaping_a1, w.roll.shaping_a1, "A1PHF2");
    near(s.roll.shaping_limit, w.roll.shaping_limit, "LMPHR1");
    near(s.roll.derivative_gain, w.roll.derivative_gain, "KPHF3");
    near(s.roll.derivative_t1, w.roll.derivative_t1, "T1PHF3");
    near(s.roll.derivative_t2, w.roll.derivative_t2, "T2PHF3");
    lag(s.roll.rate_a, w.roll.rate_a, "KRPHRA");
    lag(s.roll.lagged_rate_a, w.roll.lagged_rate_a, "KLPHRA");
    near(s.roll.attitude_gain_a, w.roll.attitude_gain_a, "KPHR1");
    near(s.roll.attitude_limit_a, w.roll.attitude_limit_a, "LMPHRA");
    near(s.roll.attitude_output_gain_a, w.roll.attitude_output_gain_a, "KPHIRA");
    near(s.roll.limit_a, w.roll.limit_a, "LMRA");
    near(s.yaw.ad_lag, w.yaw.ad_lag, "Y_AD");
    lag(s.yaw.rate_d, w.yaw.rate_d, "KRRRYD");
    lag(s.yaw.lagged_rate_d, w.yaw.lagged_rate_d, "KLRRYD");
    near(s.yaw.washout_d, w.yaw.washout_d, "TWRRYD");
    lag(s.yaw.lateral_accel_d, w.yaw.lateral_accel_d, "KLAYYD");
    near(s.yaw.roll_rate_gain_d, w.yaw.roll_rate_gain_d, "KRPPYD");
    near(s.yaw.limit_d, w.yaw.limit_d, "LMYD");
    lag(s.yaw.filter_a, w.yaw.filter_a, "KRRFI");
    near(s.yaw.washout_a, w.yaw.washout_a, "TWRRYA");
    lag(s.yaw.rate_a, w.yaw.rate_a, "KRRRYA");
    near(s.yaw.lateral_accel_gain_a, w.yaw.lateral_accel_gain_a, "KLAYF1");
    lag(s.yaw.lagged_a, w.yaw.lagged_a, "KLAYRR");
    near(s.yaw.scale_a, w.yaw.scale_a, "yaw .5");
    near(s.yaw.roll_rate_gain_a, w.yaw.roll_rate_gain_a, "KRPPYA");
    near(s.yaw.limit_a, w.yaw.limit_a, "LMYA");
    near(s.hold, w.hold, "SAS hold");
    near(s.switch_speed, w.switch_speed, "VG60SW");
    near(s.fade_time, w.fade_time, "fade");
    near(s.actuator.a2, w.actuator.a2, "SAS actuator a2");
    near(s.actuator.a1, w.actuator.a1, "SAS actuator a1");
    EXPECT_EQ(s.actuator.rate_limit, 0.0);
    near(s.descale_pitch, w.descale_pitch, "KPCDGP");
    near(s.descale_roll, w.descale_roll, "KPCDGR");
    near(s.descale_yaw, w.descale_yaw, "KPCDGY");
    const GenHelPitchBias& b = *got.pitch_bias;
    const GenHelPitchBias& v = *want.pitch_bias;
    near(b.attitude_gain, v.attitude_gain, "KTHBA");
    near(b.attitude_limit, v.attitude_limit, "LMTHBA");
    near(b.rate_gain, v.rate_gain, "KQQBA");
    near(b.rate_limit_in, v.rate_limit_in, "LMQQBA");
    near(b.speed_gain, v.speed_gain, "KVXBA");
    near(b.speed_from, v.speed_from, "PBA airspeed bias");
    near(b.speed_range, v.speed_range, "LMVXBA");
    near(b.bias, v.bias, "BIAS_BA");
    near(b.hold, v.hold, "PBA hold");
    near(b.actuator_gain, v.actuator_gain, "PBA gain");
    near(b.rate_limit, v.rate_limit, "PBA rate");
    near(b.authority, v.authority, "PBA authority");
}

// Invalid SAS and PBA configurations are rejected at init.
TEST(GenHelSas, RejectsInvalidConfigs) {
    const auto bad = [](auto change) {
        GenHelFcsConfig c = config();
        change(c);
        EXPECT_THROW(Fcs{c}, InitError);
    };
    bad([](GenHelFcsConfig& c) { c.sas->pitch.ad_lag = 0.0; });
    bad([](GenHelFcsConfig& c) { c.sas->roll.lagged_rate_a.tau = -1.0; });
    bad([](GenHelFcsConfig& c) { c.sas->roll.shaping_a2 = 0.0; });
    bad([](GenHelFcsConfig& c) { c.sas->yaw.washout_a = 0.0; });
    bad([](GenHelFcsConfig& c) { c.sas->hold = 0.0; });
    bad([](GenHelFcsConfig& c) { c.sas->fade_time = 0.0; });
    bad([](GenHelFcsConfig& c) { c.sas->actuator.a1 = 0.0; });
    bad([](GenHelFcsConfig& c) { c.sas->yaw.limit_d = -0.01; });
    bad([](GenHelFcsConfig& c) { c.pitch_bias->hold = 0.0; });
    bad([](GenHelFcsConfig& c) { c.pitch_bias->rate_limit = 0.0; });
    bad([](GenHelFcsConfig& c) { c.pitch_bias->authority = 0.0; });
    bad([](GenHelFcsConfig& c) { c.pitch_bias->actuator_gain = 0.0; });
    EXPECT_NO_THROW(Fcs{config()});
}

}  // namespace
}  // namespace eyas
