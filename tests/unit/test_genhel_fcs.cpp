// M8c1 (D-045): the GenHel flight control system's dynamic parts, against
// GenHel's block diagrams written out test-side in GenHel's own units: the
// primary servos (text p. 5.5-3), the Fig. 5.1.2 sensors, the Fig. 5.1.7
// stabilator network and servo. The mixer and head limits are checked
// against GenHel's rigging in Uh60aAircraft.ControlMapsAreGenHelsRigging.

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "control/genhel_fcs.hpp"
#include "core/attitude.hpp"
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
constexpr double kG = units::kStandardGravity;

// GenHel's values (data/uh60a/genhel/fig_5_1_2_sensors.txt,
// fig_5_5_1_7_stabilator.txt, control_rigging_5_5.txt), test-side.
struct GenHel {
    double RG_A2 = 0.00014, RG_A1 = 0.016;   // rate gyros [s², s]
    double LA_A2 = 0.0254, LA_A1 = 0.233;    // lateral accelerometer
    double PS_A2 = 0.00114, PS_A1 = 0.0473;  // primary servos
    double SV_A2 = 0.0254, SV_A1 = 0.223;    // stabilator servo
    double RATE = 7.0;                       // [deg/s] stabilator servo rate limit
    double TAU = 0.398;                      // [s] airspeed lag
    double KVTDT = 0.16;                     // [deg per deg/s]
    double KVYDT = -0.0433;                  // [deg per (ft/s²) per kt]
    double FSPS = 307.0, WLPS = 263.0;       // [in] transducer station
    // [in] the airframe CG of data/uh60a/uh60a.yaml (GenHel's, less the blades
    // and unsprung gear: D-002, D-034), the origin of its positions
    double FS_CG = 356.9298, WL_CG = 244.5933;
    double LO = -8.0, HI = 39.0;  // [deg] stabilator travel
};

// A synthetic schedule [deg] against equivalent airspeed [kt] and collective.
double schedule_deg(double kt, double coll) {
    return 40.0 - 0.3 * kt + 4.0 * coll;
}

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
    r.k_lateral = 1.6 * kDegree / kInch;
    r.k_longitudinal = -2.83 * kDegree / kInch;
    r.k_pedal = -5.539 * kDegree / kInch;
    r.sign_lon_stick = -1.0;
    r.bias_collective = 9.9 * kDegree;
    r.bias_lateral = -8.0 * kDegree;
    r.bias_longitudinal = -9.7754 * kDegree;
    r.bias_pedal = 21.9 * kDegree;
    c.primary_servo = {g.PS_A2, g.PS_A1, 0.0};
    c.rate_gyro = {g.RG_A2, g.RG_A1, 0.0};
    c.lateral_accelerometer = {g.LA_A2, g.LA_A1, 0.0};
    // Body x forward, z down: FS grows aft, WL up.
    c.accelerometer_b = Vec3(-(g.FSPS - g.FS_CG) * kInch, 0.0, -(g.WLPS - g.WL_CG) * kInch);
    GenHelStabilatorLaw& l = c.stabilator;
    const std::vector<double> kt = {0.0, 200.0};
    const std::vector<double> coll = {0.0, 1.0};
    std::vector<double> v;
    for (const double s : kt) {
        for (const double k : coll) {
            v.push_back(schedule_deg(s, k) * kDegree);
        }
    }
    l.schedule.command = Table2D(Axis({0.0, 200.0 * kKnot}), Axis(coll), v);
    l.schedule.min = g.LO * kDegree;
    l.schedule.max = g.HI * kDegree;
    l.airspeed_lag = g.TAU;
    l.pitch_rate_gain = g.KVTDT;
    l.ay_gain = g.KVYDT * 30.0 * kDegree / kFoot;
    l.ay_speed_lo = 30.0 * kKnot;
    l.ay_speed_hi = 60.0 * kKnot;
    l.servo = {g.SV_A2, g.SV_A1, g.RATE * kDegree};
    return c;
}

struct Eval {
    std::vector<double> xdot0 = std::vector<double>(Fcs::kNumStates, 0.0);
    std::vector<double> G = std::vector<double>(Fcs::kNumStates * 6, 0.0);
};

Eval derivs(const Fcs& f, const SimContext& ctx, const std::vector<double>& x) {
    Eval e;
    ComponentDerivs out{e.xdot0, e.G, Wrench{}, Mat66::Zero(), true, nullptr};
    f.derivatives(ctx, x, out);
    return e;
}

SimContext random_context(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    SimContext ctx;
    ctx.body.q_nb = quaternion(EulerAngles{0.4 * u(rng), 0.3 * u(rng), 3.0 * u(rng)});
    ctx.body.vel_b = Vec3(40.0 + 30.0 * u(rng), 5.0 * u(rng), 3.0 * u(rng));
    ctx.body.omega_b = Vec3(0.3 * u(rng), 0.3 * u(rng), 0.3 * u(rng));
    ctx.air = isa_atmosphere(1000.0 * (1.0 + u(rng)), 0.0);
    ctx.env.wind_n = Vec3(5.0 * u(rng), 5.0 * u(rng), 0.0);
    ctx.controls.collective = 0.5 + 0.5 * u(rng);
    ctx.controls.lat_cyclic = u(rng);
    ctx.controls.lon_cyclic = u(rng);
    ctx.controls.pedal = u(rng);
    return ctx;
}

std::vector<double> random_states(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::vector<double> x(Fcs::kNumStates);
    for (double& v : x) {
        v = 0.2 * u(rng);
    }
    x[Fcs::kAirspeedLag] = 40.0 + 30.0 * u(rng);  // [m/s]
    x[Fcs::kServoStab] = 10.0 * kDegree * u(rng);
    x[Fcs::kServoStab + 1] = 3.0 * kDegree * u(rng);  // inside the 7°/s rate limit
    return x;
}

// ẍ of 1/(a2 s² + a1 s + 1) at (p, v) on input u, GenHel's form.
double second_order(double a2, double a1, double u, double p, double v) {
    return (u - p - a1 * v) / a2;
}

// Every servo and sensor is GenHel's transfer function on its input: the
// position state's rate is the velocity state, the velocity's rate is
// (u − p − a1 v)/a2, and the airspeed lag is (V_e − V_F)/τ. Inputs: the
// mixer commands (primary servos), the body rates (gyros), the equivalent
// airspeed u √(ρ/ρ0) in the air-relative body x (D-040), and IHT0 (Fig. 5.1.7,
// in GenHel's degrees and knots) for the stabilator servo.
TEST(GenHelFcs, ServosAndSensorsAreGenHelsTransferFunctions) {
    const GenHel g;
    const Fcs f(config());
    std::mt19937_64 rng(8101);
    for (int n = 0; n < 300; ++n) {
        const SimContext ctx = random_context(rng);
        const std::vector<double> x = random_states(rng);
        const Eval e = derivs(f, ctx, x);
        const GenHelFcsOutputs o = f.evaluate(ctx, x);
        const auto check = [&](std::size_t i, double a2, double a1, double u, const char* what) {
            SCOPED_TRACE(what);
            EXPECT_DOUBLE_EQ(e.xdot0[i], x[i + 1]);
            const double want = second_order(a2, a1, u, x[i], x[i + 1]);
            EXPECT_NEAR(e.xdot0[i + 1], want, 1e-9 * (1.0 + std::abs(want)));
        };
        check(Fcs::kServoTheta0, g.PS_A2, g.PS_A1, o.theta0_cmd, "theta0 servo");
        check(Fcs::kServoA1s, g.PS_A2, g.PS_A1, o.a1s_cmd, "A1S servo");
        check(Fcs::kServoB1s, g.PS_A2, g.PS_A1, o.b1s_cmd, "B1S servo");
        check(Fcs::kServoThettr, g.PS_A2, g.PS_A1, o.thettr_cmd, "THETTR servo");
        check(Fcs::kGyroP, g.RG_A2, g.RG_A1, ctx.body.omega_b.x(), "p gyro");
        check(Fcs::kGyroQ, g.RG_A2, g.RG_A1, ctx.body.omega_b.y(), "q gyro");
        check(Fcs::kGyroR, g.RG_A2, g.RG_A1, ctx.body.omega_b.z(), "r gyro");

        // Airspeed lag.
        const Vec3 air_b = ctx.body.vel_b - ctx.body.q_nb.conjugate() * ctx.env.wind_n;
        const double ve = air_b.x() * std::sqrt(ctx.air.density / isa::kSeaLevelDensity);
        EXPECT_NEAR(e.xdot0[Fcs::kAirspeedLag], (ve - x[Fcs::kAirspeedLag]) / g.TAU, 1e-9);

        // Stabilator command, Fig. 5.1.7 in degrees and knots, away from the
        // ramp's rounded corners (D-045).
        const double vf_kt = x[Fcs::kAirspeedLag] / kKnot;
        if (std::abs(vf_kt - 30.0) > 3.5 && std::abs(vf_kt - 60.0) > 3.5) {
            const double qs_deg = x[Fcs::kGyroQ] / kDegree;
            const double ys_ft = x[Fcs::kAccelY] / kFoot;
            const double iht0 =
                std::clamp(schedule_deg(vf_kt, ctx.controls.collective) + g.KVTDT * qs_deg +
                               ys_ft * g.KVYDT * std::clamp(vf_kt - 30.0, 0.0, 30.0),
                           g.LO, g.HI);
            EXPECT_NEAR(o.stabilator_cmd / kDegree, iht0, 1e-9);
            check(Fcs::kServoStab, g.SV_A2, g.SV_A1, iht0 * kDegree, "stabilator servo");
        }
    }
}

// The lateral accelerometer reads the body-y specific force at its station,
// f = a_P − g with a_P the transducer's inertial acceleration (D-045):
// written here as d/dt(v + ω × r) in the body frame plus ω × (v + ω × r).
// The component returns it as xdot0 + G a, affine in a = [v̇; ω̇].
TEST(GenHelFcs, AccelerometerSensesTheSpecificForceAtItsStation) {
    const GenHel g;
    const GenHelFcsConfig cfg = config();
    const Fcs f(cfg);
    const Vec3 r = cfg.accelerometer_b;
    std::mt19937_64 rng(8102);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int n = 0; n < 300; ++n) {
        const SimContext ctx = random_context(rng);
        const std::vector<double> x = random_states(rng);
        const Vec3 vdot(3.0 * u(rng), 3.0 * u(rng), 3.0 * u(rng));
        const Vec3 wdot(0.5 * u(rng), 0.5 * u(rng), 0.5 * u(rng));
        const BodyState& b = ctx.body;
        const Vec3 vp = b.vel_b + b.omega_b.cross(r);
        const Vec3 ap = vdot + wdot.cross(r) + b.omega_b.cross(vp);
        const Vec3 g_b = b.q_nb.conjugate() * Vec3(0.0, 0.0, kG);
        const double fy = (ap - g_b).y();
        const Eval e = derivs(f, ctx, x);
        const std::size_t i = Fcs::kAccelY + 1;
        double got = e.xdot0[i];
        for (int k = 0; k < 3; ++k) {
            got += e.G[i * 6 + static_cast<std::size_t>(k)] * vdot(k);
            got += e.G[i * 6 + 3 + static_cast<std::size_t>(k)] * wdot(k);
        }
        const double want = second_order(g.LA_A2, g.LA_A1, fy, x[Fcs::kAccelY], x[i]);
        EXPECT_NEAR(got, want, 1e-9 * (1.0 + std::abs(want)));
        EXPECT_DOUBLE_EQ(e.xdot0[Fcs::kAccelY], x[i]);
    }
    // Anchor: at rest, rolled right by φ, it reads −g sin φ (right wing low:
    // the reaction to gravity points up and to the left).
    SimContext ctx;
    const double phi = 10.0 * kDegree;
    ctx.body.q_nb = quaternion(EulerAngles{phi, 0.0, 0.0});
    ctx.air = isa_atmosphere(0.0, 0.0);
    std::vector<double> x(Fcs::kNumStates, 0.0);
    f.steady_states(ctx, x);
    EXPECT_NEAR(x[Fcs::kAccelY], -kG * std::sin(phi), 1e-12);
}

// The stabilator servo's rate limit (Fig. 5.1.7, 7°/s): at the limit the
// velocity state stops growing outward and the position moves at the limit.
TEST(GenHelFcs, StabilatorServoRateLimit) {
    const GenHel g;
    const Fcs f(config());
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    ctx.body.vel_b = Vec3(50.0 * kKnot, 0.0, 0.0);
    std::vector<double> x(Fcs::kNumStates, 0.0);
    f.steady_states(ctx, x);
    const double R = g.RATE * kDegree;
    const std::size_t s = Fcs::kServoStab;
    x[s] = g.LO * kDegree;  // far below the command: pushes up
    for (const double v : {R, 1.5 * R}) {
        x[s + 1] = v;
        const Eval e = derivs(f, ctx, x);
        EXPECT_DOUBLE_EQ(e.xdot0[s], R);
        EXPECT_DOUBLE_EQ(e.xdot0[s + 1], 0.0);
    }
    x[s + 1] = 0.5 * R;  // inside: free
    EXPECT_GT(derivs(f, ctx, x).xdot0[s + 1], 0.0);
    x[s + 1] = R;
    x[s] = g.HI * kDegree;  // above the command: may slow down
    EXPECT_LT(derivs(f, ctx, x).xdot0[s + 1], 0.0);
}

// steady_states() (D-045, GenHel's IC mode) is an equilibrium of the held
// body: with a = 0 every state's rate is zero, for any body state and
// controls, and every state is set.
TEST(GenHelFcs, SteadyStatesAreAHeldBodyEquilibrium) {
    const Fcs f(config());
    std::mt19937_64 rng(8103);
    for (int n = 0; n < 200; ++n) {
        const SimContext ctx = random_context(rng);
        std::vector<double> x(Fcs::kNumStates, std::numeric_limits<double>::quiet_NaN());
        f.steady_states(ctx, x);
        for (const double v : x) {
            ASSERT_TRUE(std::isfinite(v));
        }
        const Eval e = derivs(f, ctx, x);
        for (std::size_t i = 0; i < Fcs::kNumStates; ++i) {
            EXPECT_NEAR(e.xdot0[i], 0.0, 1e-9) << i;
        }
    }
}

// No allocation in derivatives() (spec §7).
TEST(GenHelFcs, DerivativesDoNotAllocate) {
    const Fcs f(config());
    std::mt19937_64 rng(8104);
    const SimContext ctx = random_context(rng);
    const std::vector<double> x = random_states(rng);
    Eval e;
    ComponentDerivs out{e.xdot0, e.G, Wrench{}, Mat66::Zero(), true, nullptr};
    const test::AllocationCounter count;
    f.derivatives(ctx, x, out);
    (void)f.evaluate(ctx, x);
    EXPECT_EQ(count.count(), 0U);
}

// The flying UH-60A's flight control system has GenHel's values, read in
// GenHel's units (the tests above take their values from the same struct,
// so they check the equations, this one the data).
TEST(GenHelFcs, Uh60aConfigIsGenHels) {
    const GenHel g;
    const test::Uh60aHarness ac;
    const GenHelFcsConfig& c = ac.get<GenHelFcs>("fcs").config();
    const auto servo = [](const ServoConfig& s, double a2, double a1, double rate_deg) {
        EXPECT_NEAR(s.a2, a2, 1e-15);
        EXPECT_NEAR(s.a1, a1, 1e-15);
        EXPECT_NEAR(s.rate_limit / kDegree, rate_deg, 1e-12);
    };
    servo(c.primary_servo, g.PS_A2, g.PS_A1, 0.0);
    servo(c.rate_gyro, g.RG_A2, g.RG_A1, 0.0);
    servo(c.lateral_accelerometer, g.LA_A2, g.LA_A1, 0.0);
    servo(c.stabilator.servo, g.SV_A2, g.SV_A1, g.RATE);
    // The CG above is printed to 1e-4 in.
    EXPECT_NEAR(c.accelerometer_b.x() / kInch, g.FS_CG - g.FSPS, 1e-4);
    EXPECT_NEAR(c.accelerometer_b.y(), 0.0, 1e-15);
    EXPECT_NEAR(c.accelerometer_b.z() / kInch, -(g.WLPS - g.WL_CG), 1e-4);
    const GenHelStabilatorLaw& l = c.stabilator;
    EXPECT_NEAR(l.airspeed_lag, g.TAU, 1e-15);
    EXPECT_NEAR(l.pitch_rate_gain, g.KVTDT, 1e-15);
    // KVYDT · limit(V_F − 30, 0, 30): full gain 30 KVYDT from 60 kt.
    EXPECT_NEAR(l.ay_gain / (kDegree / kFoot), 30.0 * g.KVYDT, 1e-12);
    EXPECT_NEAR(l.ay_speed_lo / kKnot, 30.0, 1e-12);
    EXPECT_NEAR(l.ay_speed_hi / kKnot, 60.0, 1e-12);
    EXPECT_NEAR(l.schedule.min / kDegree, g.LO, 1e-12);
    EXPECT_NEAR(l.schedule.max / kDegree, g.HI, 1e-12);
}

// Invalid configurations are rejected at init.
TEST(GenHelFcs, RejectsInvalidConfigs) {
    const auto bad = [](auto change) {
        GenHelFcsConfig c = config();
        change(c);
        EXPECT_THROW(Fcs{c}, InitError);
    };
    bad([](GenHelFcsConfig& c) { c.name.clear(); });
    bad([](GenHelFcsConfig& c) { c.primary_servo.a2 = 0.0; });
    bad([](GenHelFcsConfig& c) { c.rate_gyro.a1 = -1.0; });
    bad([](GenHelFcsConfig& c) { c.lateral_accelerometer.a2 = 0.0; });
    bad([](GenHelFcsConfig& c) { c.stabilator.servo.rate_limit = -1.0; });
    bad([](GenHelFcsConfig& c) { c.stabilator.airspeed_lag = 0.0; });
    bad([](GenHelFcsConfig& c) { c.stabilator.ay_speed_hi = c.stabilator.ay_speed_lo; });
    bad([](GenHelFcsConfig& c) { c.rigging.b1s_min = c.rigging.b1s_max = 0.0; });
    EXPECT_NO_THROW(Fcs{config()});
}

}  // namespace
}  // namespace eyas
