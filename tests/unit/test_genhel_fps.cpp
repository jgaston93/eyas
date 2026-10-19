// M8c3 (D-045): the GenHel FPS, against GenHel's block diagrams written out
// test-side in GenHel's own units (support/genhel_fps_reference.hpp; Figs.
// 5.1.5(a)–(c), text p. 5.5-3), and the trim actuators' inches into the
// mixer. The UH-60A config's FPS is checked against the transcription here
// too; the FPS's behaviour on the flying aircraft is in test_uh60a_fps.cpp.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "control/genhel_fcs.hpp"
#include "core/attitude.hpp"
#include "env/air_data.hpp"
#include "env/atmosphere.hpp"
#include "support/alloc_counter.hpp"
#include "support/genhel_fps_reference.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kDegree;
using units::kFoot;
using units::kKnot;
using Fcs = GenHelFcs;
namespace ref = test::fps;

constexpr double kInch = 0.0254;
constexpr double kPct = 0.01;  // one percent, as a fraction

// The flying UH-60A's flight control system (SAS, PBA and FPS).
const GenHelFcsConfig& uh60a() {
    static const test::Uh60aHarness ac;
    return ac.get<GenHelFcs>("fcs").config();
}

GenHelFcsConfig with_wiring(YawIntegratorWiring w) {
    GenHelFcsConfig c = uh60a();
    c.fps->yaw.wiring = w;
    return c;
}

// The SI value of one GenHel unit of each FPS state (index from kFpsBegin).
std::array<double, ref::kCount> fps_units() {
    std::array<double, ref::kCount> u{};
    u.fill(kPct);  // integrators, holds, actuators
    u[ref::kThetaRef] = kDegree;
    u[ref::kPhiRef] = kDegree;
    u[ref::kPsiRef] = kDegree;
    u[ref::kSpeedRef] = kKnot;
    u[ref::kSpeedRl] = kKnot;
    u[ref::kAccelLag] = kFoot;
    u[ref::kCollRef] = kPct;  // the collective [0, 1] as XCPC [%]
    return u;
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

// Random flight conditions from 4 to 150 kt, attitudes within ±17°, every
// heading, and random switches.
SimContext random_context(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    SimContext ctx;
    ctx.body.q_nb = quaternion(EulerAngles{0.3 * u(rng), 0.3 * u(rng), 3.2 * u(rng)});
    ctx.body.vel_b = Vec3(40.0 + 38.0 * u(rng), 5.0 * u(rng), 3.0 * u(rng));
    ctx.body.omega_b = Vec3(0.3 * u(rng), 0.3 * u(rng), 0.3 * u(rng));
    ctx.air = isa_atmosphere(1000.0 * (1.0 + u(rng)), 0.0);
    ctx.controls.collective = 0.5 + 0.5 * u(rng);
    ctx.controls.lat_cyclic = u(rng);
    ctx.controls.lon_cyclic = u(rng);
    ctx.controls.pedal = u(rng);
    AfcsSwitches& s = ctx.controls.afcs;
    s.sas_digital = u(rng) > -0.5;
    s.sas_analog = u(rng) > -0.5;
    s.pitch_bias = u(rng) > -0.5;
    s.fps = u(rng) > -0.5;
    s.fps_turn = u(rng) > 0.0;
    s.trim_release = u(rng) > 0.5;
    return ctx;
}

// `steps` RK4 steps of 1 ms of the component alone, the context at time t
// given by at(t).
template <class At>
void rk4(const Fcs& f, At&& at, std::vector<double>& x, int steps) {
    const double h = 1e-3;
    std::vector<double> xs(x.size());
    for (int n = 0; n < steps; ++n) {
        const double t = n * h;
        const auto stage = [&](double ts, const Eval& from, double scale) {
            for (std::size_t i = 0; i < x.size(); ++i) {
                xs[i] = x[i] + scale * h * from.xdot0[i];
            }
            return derivs(f, at(ts), xs);
        };
        const Eval a = derivs(f, at(t), x);
        const Eval b = stage(t + 0.5 * h, a, 0.5);
        const Eval c = stage(t + 0.5 * h, b, 0.5);
        const Eval d = stage(t + h, c, 1.0);
        for (std::size_t i = 0; i < x.size(); ++i) {
            x[i] += h / 6.0 * (a.xdot0[i] + 2.0 * b.xdot0[i] + 2.0 * c.xdot0[i] + d.xdot0[i]);
        }
    }
}

// The FPS's inputs at ctx and x, in GenHel's units.
ref::Inputs inputs(const SimContext& ctx, std::span<const double> x) {
    const EulerAngles e = euler_angles(ctx.body.q_nb);
    ref::Inputs in;
    in.theta = e.pitch / kDegree;
    in.phi = e.roll / kDegree;
    in.psi = e.heading / kDegree;
    const Vec3 air_b = ctx.body.vel_b - ctx.body.q_nb.conjugate() * ctx.env.wind_n;
    in.v_kt = equivalent_airspeed(ctx.air.density, air_b.x()) / kKnot;
    in.q1 = x[Fcs::kPitchAd] / kDegree;
    in.p1 = x[Fcs::kRollAd] / kDegree;
    in.ysens = x[Fcs::kAccelY] / kFoot;
    in.xc = ctx.controls.collective / kPct;
    in.fps = ctx.controls.afcs.fps;
    in.turn = ctx.controls.afcs.fps_turn;
    in.release = ctx.controls.afcs.trim_release;
    return in;
}

// Random states: SAS and sensors as the SAS test's, the FPS's in GenHel's
// units: references near their inputs (within ±20° and ±20 kt, any heading
// difference), integrators and holds ±30 %, actuators ±110 % (past the stops
// at times), the lag ±10 ft/s².
std::vector<double> random_states(std::mt19937_64& rng, const ref::Inputs& in0, ref::States& s) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::vector<double> x(Fcs::kNumStates, 0.0);
    for (std::size_t i = Fcs::kPitchAd; i < Fcs::kFpsBegin; ++i) {
        x[i] = 0.05 * u(rng);
    }
    x[Fcs::kGyroP] = 0.3 * u(rng);
    x[Fcs::kGyroQ] = 0.3 * u(rng);
    x[Fcs::kGyroR] = 0.3 * u(rng);
    x[Fcs::kAccelY] = 3.0 * u(rng);
    x[Fcs::kAirspeedLag] = 40.0;
    x[Fcs::kYawFade] = 0.5 * (1.0 + u(rng));
    s[ref::kThetaRef] = in0.theta + 20.0 * u(rng);
    s[ref::kSpeedRef] = in0.v_kt + 20.0 * u(rng);
    s[ref::kPhiRef] = in0.phi + 20.0 * u(rng);
    s[ref::kPsiRef] = 360.0 * u(rng);
    s[ref::kCollRef] = 50.0 + 50.0 * u(rng);
    s[ref::kSpeedRl] = 70.0 + 70.0 * u(rng);
    s[ref::kAccelLag] = 10.0 * u(rng);
    for (const std::size_t i : {ref::kSpeedInt, ref::kRollInt, ref::kYawInt, ref::kPitchHold,
                                ref::kRollHold, ref::kYawHold}) {
        s[i] = 30.0 * u(rng);
    }
    for (const std::size_t i : {ref::kPitchAct, ref::kRollAct, ref::kYawAct}) {
        s[i] = 110.0 * u(rng);
    }
    const auto units = fps_units();
    for (std::size_t j = 0; j < ref::kCount; ++j) {
        x[Fcs::kFpsBegin + j] = s[j] * units[j];
    }
    return x;
}

// Every FPS state's rate, the channel sums and the trim actuators' inches are
// GenHel's diagrams (support/genhel_fps_reference.hpp) at random states,
// flight conditions and switches, for both yaw integrator wirings.
TEST(GenHelFps, ChannelsAreGenHelsDiagrams) {
    const auto units = fps_units();
    for (const auto wiring : {YawIntegratorWiring::physical, YawIntegratorWiring::as_printed}) {
        const Fcs f(with_wiring(wiring));
        ref::Constants g;
        g.as_printed = wiring == YawIntegratorWiring::as_printed;
        std::mt19937_64 rng(8301);
        int slow = 0;
        int faded = 0;
        for (int n = 0; n < 600; ++n) {
            const SimContext ctx = random_context(rng);
            std::vector<double> x0(Fcs::kNumStates, 0.0);
            ref::States s{};
            std::vector<double> x = random_states(rng, inputs(ctx, x0), s);
            const ref::Inputs in = inputs(ctx, x);
            if (std::abs(in.v_kt - g.V_HOLD) < 1e-6) {
                continue;
            }
            slow += in.v_kt < g.V_HOLD ? 1 : 0;
            faded += s[ref::kSpeedRl] > g.LMIXYO_V0 && s[ref::kSpeedRl] < g.LMIXYO_V1 ? 1 : 0;
            const ref::Result want = ref::reference(g, in, s);
            const Eval got = derivs(f, ctx, x);
            for (std::size_t j = 0; j < ref::kCount; ++j) {
                const double w = want.rate[j];
                EXPECT_NEAR(got.xdot0[Fcs::kFpsBegin + j] / units[j], w, 1e-9 * (1.0 + std::abs(w)))
                    << "FPS state " << j;
                for (std::size_t k = 0; k < 6; ++k) {
                    EXPECT_EQ(got.G[(Fcs::kFpsBegin + j) * 6 + k], 0.0);
                }
            }
            const GenHelFcsOutputs o = f.evaluate(ctx, x);
            EXPECT_NEAR(o.potemp / kPct, want.potemp, 1e-10);
            EXPECT_NEAR(o.rotemp / kPct, want.rotemp, 1e-10);
            EXPECT_NEAR(o.yotemp / kPct, want.yotemp, 1e-10);
            // The trim actuators' positions within their travel, as inches
            // (% → deg by KPCDG*, → in by 1/KX*; p. 5.5-20).
            const auto pos = [&](std::size_t j) { return std::clamp(s[j], -g.TA_AUTH, g.TA_AUTH); };
            EXPECT_NEAR(o.fps_pitch / kPct, pos(ref::kPitchAct), 1e-12);
            EXPECT_NEAR(o.xbols / kInch, pos(ref::kPitchAct) * -0.283 / -2.83, 1e-12);
            EXPECT_NEAR(o.xaols / kInch, pos(ref::kRollAct) * 0.16 / 1.6, 1e-12);
            EXPECT_NEAR(o.xpols / kInch, pos(ref::kYawAct) * -0.298 / -5.539, 1e-12);
        }
        EXPECT_GT(slow, 20);  // both sides of VG30SW
        EXPECT_GT(faded, 100);
    }
}

// The trim actuators' inches join the mixer's sums next to the SAS's
// (Figs. 5.1.5, 5.1.6(b)–(d)): XAOLS into XASUM, XBOLS into XBSUM, XPOLS
// into XPSUM, which reaches B1S as well through MIX_BP.
TEST(GenHelFps, TrimActuatorInchesJoinTheMixer) {
    const Fcs f(uh60a());
    const GenHelRigging& rig = uh60a().rigging;
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.controls.collective = 0.5;
    std::vector<double> x(Fcs::kNumStates, 0.0);
    const GenHelFcsOutputs base = f.evaluate(ctx, x);
    x[Fcs::kFpsPitchAct] = 20.0 * kPct;
    x[Fcs::kFpsRollAct] = -30.0 * kPct;
    x[Fcs::kFpsYawAct] = 10.0 * kPct;
    const GenHelFcsOutputs o = f.evaluate(ctx, x);
    const double deg_b1s = -0.283 * 20.0;  // KPCDG* × % [deg]
    const double deg_a1s = 0.16 * -30.0;
    const double deg_thr = -0.298 * 10.0;
    EXPECT_NEAR((o.a1s_cmd - base.a1s_cmd) / kDegree, deg_a1s, 1e-10);
    EXPECT_NEAR((o.b1s_cmd - base.b1s_cmd) / kDegree,
                deg_b1s + rig.mix_longitudinal_pedal * deg_thr, 1e-10);
    EXPECT_NEAR((o.thettr_cmd - base.thettr_cmd) / kDegree, deg_thr, 1e-10);
    EXPECT_NEAR(o.theta0_cmd, base.theta0_cmd, 1e-15);
    // Past the stop the actuator counts as at it (its inches; −16° of A1S
    // is beyond the flying aircraft's head limits).
    x[Fcs::kFpsRollAct] = -130.0 * kPct;
    EXPECT_NEAR(f.evaluate(ctx, x).xaols / kInch, -100.0 * 0.16 / 1.6, 1e-12);
}

// A synchronizer that has been tracking a steadily turning heading holds,
// when the FPS engages, the heading less its rate times kSyncTau: at a
// standard-rate turn (3°/s) 0.03°, 0.066 % of yaw command (KSPSYO −2.2
// %/deg), against the trim actuator's 100 %. RK4 at 1 ms on the component's
// own rates.
TEST(GenHelFps, EngagingGivesNoVisibleKick) {
    const Fcs f(uh60a());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.body.vel_b = Vec3(80.0 * kKnot, 0.0, 0.0);
    ctx.controls.collective = 0.5;
    const double rate = 3.0 * kDegree;  // [rad/s]
    std::vector<double> x(Fcs::kNumStates, 0.0);
    f.steady_states(ctx, x);
    const auto at = [&](double t) {
        SimContext c = ctx;
        c.body.q_nb = quaternion(EulerAngles{0.0, 0.0, 0.5 + rate * t});
        return c;
    };
    rk4(f, at, x, 1000);
    SimContext on = at(1.0);
    on.controls.afcs.fps = true;
    const GenHelFcsOutputs o = f.evaluate(on, x);
    const double held = (euler_angles(on.body.q_nb).heading - x[Fcs::kFpsPsiRef]) / kDegree;
    EXPECT_NEAR(held, 3.0 * ref::kSyncTau, 1e-4);  // rate × τ [deg]
    EXPECT_LT(std::abs(o.yotemp) / kPct, 0.1);
}

// The trim release (an Eyas assumption, D-045 M8c3): held, the references
// follow the aircraft as with the FPS off, the integrators stop and the trim
// actuators stand still; let go, the FPS holds the new attitude with the
// actuators where they were.
TEST(GenHelFps, TrimReleaseResynchronizes) {
    const Fcs f(uh60a());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.body.vel_b = Vec3(100.0 * kKnot, 0.0, 0.0);
    ctx.controls.collective = 0.5;
    ctx.controls.afcs = {true, true, true, true, false, false};
    std::vector<double> x(Fcs::kNumStates, 0.0);
    f.steady_states(ctx, x);
    x[Fcs::kFpsPitchAct] = 12.0 * kPct;  // trimmed somewhere
    x[Fcs::kFpsSpeedInt] = 3.0 * kPct;
    // The pilot pitches 5° nose down with the release held: no integration,
    // no actuator motion, the reference follows.
    const auto at = [&](double /*t*/) {
        SimContext c = ctx;
        c.body.q_nb = quaternion(EulerAngles{0.0, -5.0 * kDegree, 0.0});
        c.controls.afcs.trim_release = true;
        return c;
    };
    const Eval held = derivs(f, at(0.0), x);
    EXPECT_EQ(held.xdot0[Fcs::kFpsPitchAct], 0.0);
    EXPECT_EQ(held.xdot0[Fcs::kFpsSpeedInt], 0.0);
    EXPECT_NEAR(held.xdot0[Fcs::kFpsThetaRef], -5.0 * kDegree / ref::kSyncTau, 1e-9);
    rk4(f, at, x, 200);
    EXPECT_NEAR(x[Fcs::kFpsThetaRef], -5.0 * kDegree, 1e-9);
    EXPECT_EQ(x[Fcs::kFpsPitchAct], 12.0 * kPct);
    EXPECT_EQ(x[Fcs::kFpsSpeedInt], 3.0 * kPct);
    // Let go: the attitude error is the new reference's, zero.
    SimContext go = at(0.0);
    go.controls.afcs.trim_release = false;
    EXPECT_NEAR(f.evaluate(go, x).potemp / kPct, 3.0, 1e-6);  // the integral alone
}

// The turn switch (Fig. 5.1.5(c), p. 5.5-3): on, the heading reference
// tracks, the roll integrator stops, and the roll rate and lateral
// acceleration join the yaw channel; the yaw integrator takes the lateral
// acceleration (physical wiring) or the inert heading path (as printed).
TEST(GenHelFps, TurnSwitch) {
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.body.vel_b = Vec3(100.0 * kKnot, 0.0, 0.0);
    ctx.body.q_nb = quaternion(EulerAngles{20.0 * kDegree, 0.0, 1.0});
    ctx.controls.collective = 0.5;
    ctx.controls.afcs = {true, true, true, true, true, false};
    for (const auto wiring : {YawIntegratorWiring::physical, YawIntegratorWiring::as_printed}) {
        const Fcs f(with_wiring(wiring));
        std::vector<double> x(Fcs::kNumStates, 0.0);
        f.steady_states(ctx, x);
        x[Fcs::kFpsPsiRef] = 0.9;            // 5.7° off the heading
        x[Fcs::kFpsPhiRef] = 0.0;            // 20° off the bank
        x[Fcs::kFpsAccelLag] = 0.5 * kFoot;  // [m/s²]
        const Eval d = derivs(f, ctx, x);
        EXPECT_NEAR(d.xdot0[Fcs::kFpsPsiRef], 0.1 / ref::kSyncTau, 1e-9);
        EXPECT_EQ(d.xdot0[Fcs::kFpsRollInt], 0.0);
        // Heading path: deadband(0) = 0 while the reference tracks.
        const double want = wiring == YawIntegratorWiring::physical ? -0.656 * 0.5 * kPct : 0.0;
        EXPECT_NEAR(d.xdot0[Fcs::kFpsYawInt], want, 1e-15);
    }
}

// GenHel's IC mode (p. 5.5-2) as steady_states(): every FPS reference at its
// input, integrators zero, holds, lag and rate limiter at their inputs, the
// trim actuators at their commands, so every state is at rest but the yaw
// integrator in a turn-switch-on flight, which integrates the steady lateral
// acceleration (what TrimResult::held_drift_rate reports). The FPS outputs
// nothing then (its references hold the trim): POTEMP, ROTEMP are the rate
// terms, zero, and so is YOTEMP with the turn switch off.
TEST(GenHelFps, SteadyStatesAreAHeldBodyEquilibrium) {
    const Fcs f(uh60a());
    std::mt19937_64 rng(8302);
    for (int n = 0; n < 300; ++n) {
        SimContext ctx = random_context(rng);
        ctx.body.omega_b.setZero();
        std::vector<double> x(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        f.steady_states(ctx, x);
        for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
            ASSERT_TRUE(std::isfinite(x[i])) << i;
        }
        const AfcsSwitches& sw = ctx.controls.afcs;
        const Eval e = derivs(f, ctx, x);
        for (std::size_t i = Fcs::kFpsBegin; i < Fcs::kNumStates; ++i) {
            if (i == Fcs::kFpsYawInt && sw.fps && sw.fps_turn && !sw.trim_release) {
                EXPECT_NEAR(e.xdot0[i], -0.656 * kPct / kFoot * x[Fcs::kAccelY], 1e-12);
                continue;
            }
            EXPECT_NEAR(e.xdot0[i], 0.0, 1e-9) << i;
        }
        for (const std::size_t i : {Fcs::kFpsSpeedInt, Fcs::kFpsRollInt, Fcs::kFpsYawInt}) {
            EXPECT_EQ(x[i], 0.0);
        }
        const GenHelFcsOutputs o = f.evaluate(ctx, x);
        EXPECT_NEAR(o.potemp, 0.0, 1e-15);
        EXPECT_NEAR(o.rotemp, 0.0, 1e-15);
        if (!sw.fps_turn) {
            EXPECT_NEAR(o.yotemp, 0.0, 1e-15);
        }
        if (!sw.fps) {
            EXPECT_EQ(o.fps_yaw, 0.0);
        }
    }
}

// Trim holds the references and integrators (GenHel's IC mode) and starts
// the rest on their free-flight orbit; without the FPS section every FPS
// state is held at zero.
TEST(GenHelFps, SteadyKinds) {
    std::vector<SteadyKind> k(Fcs::kNumStates);
    Fcs(uh60a()).steady_kinds(k);
    for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
        const bool held = i == Fcs::kFpsThetaRef || i == Fcs::kFpsSpeedRef ||
                          i == Fcs::kFpsSpeedInt || i == Fcs::kFpsPhiRef || i == Fcs::kFpsRollInt ||
                          i == Fcs::kFpsPsiRef || i == Fcs::kFpsYawInt || i == Fcs::kFpsCollRef;
        EXPECT_EQ(k[i], held ? SteadyKind::held : SteadyKind::orbit) << i;
    }
    GenHelFcsConfig bare = uh60a();
    bare.fps.reset();
    Fcs(bare).steady_kinds(k);
    for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
        EXPECT_EQ(k[i], i >= Fcs::kFpsBegin ? SteadyKind::held : SteadyKind::orbit) << i;
    }
}

// Switched off, the FPS puts nothing into the mixer at its steady states,
// whatever the other switches: the M8c2 aircraft. Without the section its
// states stay at zero.
TEST(GenHelFps, SwitchedOffAddsNothing) {
    const Fcs f(uh60a());
    GenHelFcsConfig bare = uh60a();
    bare.fps.reset();
    const Fcs m8c2(bare);
    std::mt19937_64 rng(8303);
    for (int n = 0; n < 100; ++n) {
        SimContext ctx = random_context(rng);
        ctx.controls.afcs.fps = false;
        std::vector<double> x(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        f.steady_states(ctx, x);
        std::vector<double> y(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        m8c2.steady_states(ctx, y);
        for (std::size_t i = 0; i < Fcs::kFpsBegin; ++i) {
            EXPECT_EQ(x[i], y[i]) << i;
        }
        const GenHelFcsOutputs a = f.evaluate(ctx, x);
        const GenHelFcsOutputs b = m8c2.evaluate(ctx, y);
        EXPECT_EQ(a.a1s_cmd, b.a1s_cmd);
        EXPECT_EQ(a.b1s_cmd, b.b1s_cmd);
        EXPECT_EQ(a.thettr_cmd, b.thettr_cmd);
        for (std::size_t i = Fcs::kFpsBegin; i < Fcs::kNumStates; ++i) {
            EXPECT_EQ(y[i], 0.0) << i;
        }
        const Eval e = derivs(m8c2, ctx, y);
        for (std::size_t i = Fcs::kFpsBegin; i < Fcs::kNumStates; ++i) {
            EXPECT_EQ(e.xdot0[i], 0.0) << i;
        }
    }
}

// The trim actuators (Fig. 5.1.5, p. 5.5-3): gain 24.1 on the error, ±10 %/s,
// stopping at ±100 % without winding up; held by the trim release with the
// FPS on; with it off they drive back to centre at the rate limit.
TEST(GenHelFps, TrimActuatorLimits) {
    const Fcs f(uh60a());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.body.vel_b = Vec3(100.0 * kKnot, 0.0, 0.0);
    ctx.controls.collective = 0.5;
    ctx.controls.afcs.sas_digital = true;
    ctx.controls.afcs.fps = true;
    std::vector<double> x(Fcs::kNumStates, 0.0);
    f.steady_states(ctx, x);
    const auto pitch_rate = [&](double hold_pct, double act_pct) {
        // The hold at rest at POTEMP + its error: the command is the hold.
        std::vector<double> y = x;
        y[Fcs::kFpsPitchHold] = hold_pct * kPct;
        y[Fcs::kFpsPitchAct] = act_pct * kPct;
        y[Fcs::kFpsSpeedInt] = hold_pct * kPct;  // POTEMP = the integral
        return derivs(f, ctx, y).xdot0[Fcs::kFpsPitchAct] / kPct;
    };
    EXPECT_NEAR(pitch_rate(5.0, 0.0), 10.0, 1e-9);        // rate limited
    EXPECT_NEAR(pitch_rate(5.0, 4.9), 24.1 * 0.1, 1e-9);  // linear
    EXPECT_EQ(pitch_rate(150.0, 100.0), 0.0);             // at the stop
    EXPECT_NEAR(pitch_rate(150.0, 99.9), 10.0, 1e-9);     // reaching it
    ctx.controls.afcs.trim_release = true;
    EXPECT_EQ(pitch_rate(5.0, 30.0), 0.0);  // on, trim release held: it stays
    ctx.controls.afcs.fps = false;
    EXPECT_NEAR(pitch_rate(5.0, 30.0), -10.0, 1e-9);  // off: back to centre,
    ctx.controls.afcs.trim_release = false;           // trim release or not
    EXPECT_NEAR(pitch_rate(5.0, 30.0), -10.0, 1e-9);  // (code review)
    EXPECT_NEAR(pitch_rate(5.0, 0.1), -24.1 * 0.1, 1e-9);
}

// No allocation in derivatives() or steady_states() (spec §7).
TEST(GenHelFps, DoesNotAllocate) {
    const Fcs f(uh60a());
    std::mt19937_64 rng(8304);
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

// The flying UH-60A's FPS has GenHel's values (fig_5_1_5_fps.txt), read in
// GenHel's units and converted here (the tests above check the equations
// against the same values).
TEST(GenHelFps, Uh60aConfigIsGenHels) {
    const ref::Constants g;
    ASSERT_TRUE(uh60a().fps.has_value());
    const GenHelFps& f = *uh60a().fps;
    const auto near = [](double a, double b, const char* what) {
        EXPECT_NEAR(a, b, 1e-12 * (1.0 + std::abs(b))) << what;
    };
    const double per_rate = kPct / kDegree;
    near(f.hold, g.ZOH, "ZOH");
    near(f.actuator_gain, g.TA_GAIN, "trim actuator gain");
    near(f.rate_limit, g.TA_RATE * kPct, "trim actuator rate");
    near(f.authority, g.TA_AUTH * kPct, "trim actuator authority");
    near(f.pitch.rate_gain, g.KRQQPO * per_rate, "KRQQPO");
    near(f.pitch.attitude_gain, g.KSTHPO * kPct / kDegree, "KSTHPO");
    near(f.pitch.speed_gain, g.KSVXPO * kPct / kKnot, "KSVXPO");
    near(f.pitch.speed_limit, g.LMVX * kPct, "LMVX");
    near(f.pitch.speed_integral_gain, g.KIVXPO * kPct / kKnot, "KIVXPO");
    near(f.pitch.speed_hold_above, g.V_HOLD * kKnot, "VG30SW");
    near(f.roll.rate_gain, g.KRPRO * per_rate, "KRPRO");
    near(f.roll.attitude_gain, g.KPHRO1 * kPct / kDegree, "KPHRO1");
    near(f.roll.attitude_gain_beyond, g.KPHRO2 * kPct / kDegree, "KPHRO2");
    near(f.roll.attitude_breakpoint, g.BPPHRO * kDegree, "BPPHRO");
    near(f.roll.integral_gain, g.KIPHRO * kPct / kDegree, "KIPHRO");
    near(f.yaw.heading_gain, g.KSPSYO * kPct / kDegree, "KSPSYO");
    near(f.yaw.heading_integral_gain, g.VGPSYO * kPct / kDegree, "VGPSYO");
    near(f.yaw.heading_deadband, g.DBPSYO * kDegree, "DBPSYO");
    near(f.yaw.accel_lag, g.TLAYYO, "TLAYYO");
    near(f.yaw.accel_gain, g.KLAYYO * kPct / kFoot, "KLAYYO");
    near(f.yaw.accel_integral_gain, g.KIAYYO * kPct / kFoot, "KIAYYO");
    near(f.yaw.roll_rate_gain, g.KRPPYO * per_rate, "KRPPYO");
    near(f.yaw.collective_gain, g.KSXCYO, "KSXCYO");
    near(f.yaw.speed_rate_limit, g.KLVXYO * kKnot, "KLVXYO");
    near(f.yaw.collective_fade_from, g.LMIXYO_V0 * kKnot, "LMIXYO from");
    near(f.yaw.collective_fade_to, g.LMIXYO_V1 * kKnot, "LMIXYO to");
    EXPECT_EQ(f.yaw.wiring, YawIntegratorWiring::physical);
}

// Invalid FPS configurations are rejected at init.
TEST(GenHelFps, RejectsInvalidConfigs) {
    const auto bad = [](auto change) {
        GenHelFcsConfig c = uh60a();
        change(c);
        EXPECT_THROW(Fcs{c}, InitError);
    };
    bad([](GenHelFcsConfig& c) { c.sas.reset(), c.pitch_bias.reset(); });
    bad([](GenHelFcsConfig& c) { c.fps->hold = 0.0; });
    bad([](GenHelFcsConfig& c) { c.fps->actuator_gain = 0.0; });
    bad([](GenHelFcsConfig& c) { c.fps->rate_limit = 0.0; });
    bad([](GenHelFcsConfig& c) { c.fps->authority = 0.0; });
    bad([](GenHelFcsConfig& c) { c.fps->yaw.accel_lag = 0.0; });
    bad([](GenHelFcsConfig& c) { c.fps->yaw.speed_rate_limit = 0.0; });
    bad([](GenHelFcsConfig& c) { c.fps->pitch.speed_limit = -0.01; });
    bad([](GenHelFcsConfig& c) { c.fps->roll.attitude_breakpoint = -0.01; });
    bad([](GenHelFcsConfig& c) { c.fps->yaw.heading_deadband = -0.01; });
    bad([](GenHelFcsConfig& c) {
        c.fps->yaw.collective_fade_to = c.fps->yaw.collective_fade_from;
    });
    EXPECT_NO_THROW(Fcs{uh60a()});
}

}  // namespace
}  // namespace eyas
