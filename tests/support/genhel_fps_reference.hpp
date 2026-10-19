#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

// GenHel's FPS (NASA CR-166309 Figs. 5.1.5(a)–(c), text p. 5.5-3; M8c3,
// D-045) written out test-side in GenHel's own units: deg, deg/s, kt, ft/s²,
// % of authority, and the collective stick in %. An exact-equation reference
// for the genhel_fcs component's FPS states: no Eyas code, only the figures
// (data/uh60a/genhel/fig_5_1_5_fps.txt) and the D-045 M8c3 choices, each
// marked where it is used:
// - a synchronizer's reference follows its input as a lag of kSyncTau while it
//   tracks (output 0), and stays while it holds (output input − reference);
// - every synchronizer also tracks with the FPS off or the trim release held,
//   the integrators run only with the FPS on and the trim release up, and the
//   trim actuators stand still while the trim release is held with the FPS
//   on (off, they drive back to centre);
// - the zero-order holds are a Padé (1 − sT/4)/(1 + sT/4);
// - the yaw integrator's input per the wiring (physical or as printed);
// - the airspeed rate limiter approaches its input at gain kRateGain;
// - the trim actuators stop at their travel.
namespace eyas::test::fps {

constexpr double kSyncTau = 0.01;    // [s]
constexpr double kRateGain = 100.0;  // [1/s]

// fig_5_1_5_fps.txt, in its units.
struct Constants {
    double ZOH = 0.10, TA_GAIN = 24.1, TA_RATE = 10.0, TA_AUTH = 100.0;
    double KRQQPO = -0.336, KSTHPO = -1.42, KSVXPO = 0.22, LMVX = 1.6, KIVXPO = 0.0188;
    double V_HOLD = 30.0;
    double KRPRO = -0.189, KPHRO1 = -0.75, KPHRO2 = -0.375, BPPHRO = 7.9, KIPHRO = -0.0868;
    double KRPPYO = 0.11, KSPSYO = -2.2, DBPSYO = 1.0, VGPSYO = -0.348, KIAYYO = -0.656;
    double TLAYYO = 1.0, KLAYYO = -2.38, KSXCYO = -0.625, KLVXYO = 10.0;
    double LMIXYO_V0 = 40.0, LMIXYO_V1 = 100.0;
    bool as_printed = false;  // the yaw integrator's wiring
};

// The signals the FPS reads.
struct Inputs {
    double theta = 0.0, phi = 0.0, psi = 0.0;  // THETAB, PHIB, PSIB [deg]
    double v_kt = 0.0;                         // VXBIKT [kt]
    double q1 = 0.0, p1 = 0.0;                 // conditioned rates [deg/s]
    double ysens = 0.0;                        // Y.SENS [ft/s²]
    double xc = 0.0;                           // XCPC, collective stick [%]
    bool fps = false, turn = false, release = false;
};

// The FPS states, in the order the component keeps them, each in its unit
// above.
enum : std::size_t {
    kThetaRef,
    kSpeedRef,
    kSpeedInt,
    kPitchHold,
    kPitchAct,  // deg, kt, %, %, %
    kPhiRef,
    kRollInt,
    kRollHold,
    kRollAct,  // deg, %, %, %
    kPsiRef,
    kYawInt,
    kAccelLag,
    kCollRef,
    kSpeedRl,  // deg, %, ft/s², %, kt
    kYawHold,
    kYawAct,  // %, %
    kCount
};
using States = std::array<double, kCount>;

struct Result {
    States rate{};                                    // per second
    double potemp = 0.0, rotemp = 0.0, yotemp = 0.0;  // [%]
    double e_pitch = 0.0, e_roll = 0.0, e_yaw = 0.0;  // the trim actuators' commands [%]
};

inline Result reference(const Constants& g, const Inputs& in, const States& s) {
    Result o;
    auto& d = o.rate;
    const bool tracking = !in.fps || in.release;
    const bool running = in.fps && !in.release;
    const auto sync = [&](std::size_t i, double diff, bool track) {
        d[i] = track ? diff / kSyncTau : 0.0;
        return track ? 0.0 : diff;
    };
    const auto lim = [](double v, double l) { return std::clamp(v, -l, l); };

    // Pitch, Fig. 5.1.5(a): KRQQPO q₁ + KSTHPO sync(θ) + limit(KSVXPO dv,
    // ±LMVX) + ∫KIVXPO dv, the airspeed sync tracking below 30 kt (VG30SW).
    const double dth = sync(kThetaRef, in.theta - s[kThetaRef], tracking);
    const double dv = sync(kSpeedRef, in.v_kt - s[kSpeedRef], tracking || in.v_kt < g.V_HOLD);
    d[kSpeedInt] = running ? g.KIVXPO * dv : 0.0;
    o.potemp = g.KRQQPO * in.q1 + g.KSTHPO * dth + lim(g.KSVXPO * dv, g.LMVX) + s[kSpeedInt];

    // Roll, Fig. 5.1.5(b): KRPRO p₁ + G(sync(φ)) + ∫KIPHRO sync(φ), the
    // integrator held by NTRNSW = 0 (turn switch on).
    const double dph = sync(kPhiRef, in.phi - s[kPhiRef], tracking);
    const double g_phi = std::abs(dph) <= g.BPPHRO
                             ? g.KPHRO1 * dph
                             : (dph > 0.0 ? 1.0 : -1.0) *
                                   (g.KPHRO1 * g.BPPHRO + g.KPHRO2 * (std::abs(dph) - g.BPPHRO));
    d[kRollInt] = running && !in.turn ? g.KIPHRO * dph : 0.0;
    o.rotemp = g.KRPRO * in.p1 + g_phi + s[kRollInt];

    // Yaw, Fig. 5.1.5(c): the heading sync tracks with NTRNSW = 0; its
    // difference taken the short way round.
    double dpsi_in = std::fmod(in.psi - s[kPsiRef], 360.0);
    if (dpsi_in > 180.0) {
        dpsi_in -= 360.0;
    } else if (dpsi_in < -180.0) {
        dpsi_in += 360.0;
    }
    const double dpsi = sync(kPsiRef, dpsi_in, tracking || in.turn);
    d[kAccelLag] = (in.ysens - s[kAccelLag]) / g.TLAYYO;
    const double ay = s[kAccelLag];
    double db = 0.0;
    if (dpsi > g.DBPSYO) {
        db = dpsi - g.DBPSYO;
    } else if (dpsi < -g.DBPSYO) {
        db = dpsi + g.DBPSYO;
    }
    // Physical: heading path with the turn switch off; as printed: with it on.
    const bool heading_path = g.as_printed ? in.turn : !in.turn;
    d[kYawInt] = running ? (heading_path ? g.VGPSYO * db : g.KIAYYO * ay) : 0.0;
    const double dxc = sync(kCollRef, in.xc - s[kCollRef], tracking);
    d[kSpeedRl] = lim(kRateGain * (in.v_kt - s[kSpeedRl]), g.KLVXYO);
    double lmix = (g.LMIXYO_V1 - s[kSpeedRl]) / (g.LMIXYO_V1 - g.LMIXYO_V0);
    lmix = std::clamp(lmix, 0.0, 1.0);
    o.yotemp = g.KSPSYO * dpsi + s[kYawInt] + (in.turn ? g.KRPPYO * in.p1 + g.KLAYYO * ay : 0.0) +
               g.KSXCYO * dxc * lmix;

    // Each channel: hold, FPSSW, trim actuator (24.1, ±10 %/s, ±100 %).
    const auto channel = [&](std::size_t hold, std::size_t act, double temp, double& e) {
        d[hold] = (temp - s[hold]) / (g.ZOH / 4.0);
        e = in.fps ? 2.0 * s[hold] - temp : 0.0;
        double a = in.fps && in.release ? 0.0 : lim(g.TA_GAIN * (e - s[act]), g.TA_RATE);
        if ((s[act] >= g.TA_AUTH && a > 0.0) || (s[act] <= -g.TA_AUTH && a < 0.0)) {
            a = 0.0;
        }
        d[act] = a;
    };
    channel(kPitchHold, kPitchAct, o.potemp, o.e_pitch);
    channel(kRollHold, kRollAct, o.rotemp, o.e_roll);
    channel(kYawHold, kYawAct, o.yotemp, o.e_yaw);
    return o;
}

}  // namespace eyas::test::fps
