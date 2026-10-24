// M9: Ballin's component-level T700 (NASA TM-100991; D-054). The gas path
// against Ballin's printed steady states (Table B.1), the curve-family
// lookup, and the config.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "analysis/trim.hpp"
#include "core/coupling.hpp"
#include "core/rigid_body.hpp"
#include "engine/ballin_t700.hpp"
#include "env/atmosphere.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kFoot;
using units::kPoundForce;
using units::kPoundMass;
using units::kPsi;
using units::kRankine;
using units::kRpm;

constexpr double kFtLbf = kFoot * kPoundForce;  // [N·m]
constexpr double kLbmHr = kPoundMass / 3600.0;  // [kg/s]
constexpr double kHp = 550.0 * kFtLbf;          // [W]

std::unique_ptr<BallinT700> load_engine() {
    const Config c = Config::from_file(std::string(EYAS_VEHICLE_DATA_DIR) +
                                       "/uh60a/engine_ballin/ballin_t700.yaml");
    return BallinT700::from_config(c);
}

// The MAP reading (TM-83579 App. C): along each curve, then linear between
// the two around the parameter, clamped beyond the outermost.
TEST(BallinT700, CurveFamilyReadsAlongThenAcross) {
    std::vector<Table1D> curves;
    curves.emplace_back(Axis({0.0, 1.0}), std::vector<double>{0.0, 10.0});
    curves.emplace_back(Axis({0.0, 2.0}), std::vector<double>{4.0, 4.0});
    const CurveFamily f({1.0, 3.0}, std::move(curves));
    CurveFamily::Cursor c = f.make_cursor();
    EXPECT_DOUBLE_EQ(f(1.0, 0.5, c), 5.0);
    EXPECT_DOUBLE_EQ(f(3.0, 0.5, c), 4.0);
    EXPECT_DOUBLE_EQ(f(2.0, 0.5, c), 4.5);
    EXPECT_DOUBLE_EQ(f(2.0, 5.0, c), 7.0);  // each curve clamped: 10 and 4
    EXPECT_DOUBLE_EQ(f(0.0, 0.5, c), 5.0);  // the parameter clamped
    EXPECT_DOUBLE_EQ(f(9.0, 0.5, c), 4.0);
}

// Table B.1 (p. 53): the engine's steady states in hover, at 80 kt and in an
// 80 kt descent, open loop at the printed fuel flow and power-turbine speed
// on a standard day. The gas path is solved with the gas generator's torques
// balanced and the heat sink at rest. Tolerances: the maps are read to about
// 0.25 % of each axis (DATA_GAPS.md) and the state passes through several in
// series (f1, f2 and the bleeds into the flows, f7-f9 into the turbines'
// work), and Table B.1 prints 4-5 figures: 0.5 % on NG and the temperatures,
// 1 % on the pressures, 2 % on the torque and power, which take the product
// of two maps' errors (W45 and ΔH_PT). With eq. (43) as printed (− W_f) the
// pressures miss by 7-9 % and the power by 13-23 % (D-054 item 5).
TEST(BallinT700, ReproducesTableB1SteadyStates) {
    const auto engine = load_engine();
    struct Row {
        double fuel_lbm_hr, np_rpm, ng_rpm, ps3_psi, p41_psi, t41_r, p45_psi, t45_r, q_ftlbf, hp;
    };
    const std::array<Row, 3> rows{{
        {476.3, 20895.0, 41638.0, 176.34, 174.28, 2292.0, 37.42, 1632.0, 229.0, 911.1},
        {349.3, 20895.0, 39768.0, 142.13, 140.26, 2102.0, 30.66, 1501.0, 138.9, 552.6},
        {267.7, 20895.0, 38072.0, 114.27, 112.77, 1982.0, 25.54, 1424.0, 76.06, 302.6},
    }};
    for (const Row& r : rows) {
        SCOPED_TRACE(r.fuel_lbm_hr);
        double ng = 0.0;
        BallinT700Gas g;
        const double np = r.np_rpm * kRpm;
        ASSERT_TRUE(engine->steady_gas(r.fuel_lbm_hr * kLbmHr, np, 14.696 * kPsi, 518.67 * kRankine,
                                       ng, g));
        const auto rel = [](double a, double b) { return std::abs(a / b - 1.0); };
        EXPECT_LT(rel(ng / kRpm, r.ng_rpm), 0.005);
        EXPECT_LT(rel(g.ps3 / kPsi, r.ps3_psi), 0.01);
        EXPECT_LT(rel(g.p41 / kPsi, r.p41_psi), 0.01);
        EXPECT_LT(rel(g.p45 / kPsi, r.p45_psi), 0.01);
        EXPECT_LT(rel(g.t41 / kRankine, r.t41_r), 0.005);
        EXPECT_LT(rel(g.t45 / kRankine, r.t45_r), 0.005);
        EXPECT_LT(rel(g.q_pt / kFtLbf, r.q_ftlbf), 0.02);
        EXPECT_LT(rel(g.q_pt * np / kHp, r.hp), 0.02);
        // The heat sink at rest passes T41ns through.
        EXPECT_NEAR(g.t41, g.t41ns, 1e-9 * g.t41);
    }
}

// The quasi-steady gas path (D-054 item 4): at a steady point it reproduces
// the steady solution (the gas generator's torques balanced, T41 = T41ns at
// rest), and from its state-only start it converges well away from the
// steady line, within kMaxIter, in a handful of iterations.
TEST(BallinT700, QuasiSteadyGasPathConverges) {
    const auto engine = load_engine();
    const double p2 = 14.696 * kPsi;
    const double t2 = 518.67 * kRankine;
    const double np = 20895.0 * kRpm;
    double ng = 0.0;
    BallinT700Gas s;
    ASSERT_TRUE(engine->steady_gas(476.3 * kLbmHr, np, p2, t2, ng, s));
    const BallinT700Gas g = engine->gas(ng, np, s.t41ns, 476.3 * kLbmHr, p2, t2);
    ASSERT_TRUE(g.converged);
    EXPECT_NEAR(g.p3, s.p3, 1e-8 * s.p3);
    EXPECT_NEAR(g.t41, s.t41ns, 1e-8 * s.t41ns);
    EXPECT_NEAR(g.q_gt, g.q_c, 1e-7 * g.q_c);
    int worst = 0;
    for (const double fuel_scale : {0.5, 0.8, 1.25, 1.6}) {
        for (const double ng_scale : {0.92, 1.0, 1.05}) {
            for (const double metal_shift : {-300.0, 0.0, 300.0}) {
                const BallinT700Gas o = engine->gas(ng * ng_scale, np, s.t41ns + metal_shift,
                                                    476.3 * kLbmHr * fuel_scale, p2, t2);
                EXPECT_TRUE(o.converged) << fuel_scale << " " << ng_scale << " " << metal_shift;
                worst = std::max(worst, o.iterations);
            }
        }
    }
    EXPECT_LE(worst, 12);
}

// App. B's extracted linear models (Figs. B1-B11, Table 1), their
// engine-only entries: b1 = ∂ṄG/∂W_f and A11 = ∂ṄG/∂NG of the 2-state models
// (quasi-steady volumes, no heat sink), A21/b2 = (∂Q_PT/∂NG)/(∂Q_PT/∂W_f), in
// which the load terms of eqs. (60)-(62) cancel, and the gas generator's and
// heat sink's eigenvalues of the 3-state models (coordinate-free). Ballin
// extracts by central differences of ±2 % of the equilibrium values (p. 14);
// so does this test. On the maps' broken lines the step matters: ∂ṄG/∂NG
// at trim 1 is −2.34 (tangent), −2.80 (±1 %) and −2.98/s (±2 %), a 27 %
// spread, which bounds what a comparison with these extractions can
// resolve; b1 moves 1.5 % over the same steps. Gates: b1 within 10 % (the
// maps' slopes, D-054); A11, A21/b2 and the gas generator's mode within 25 %
// (the step's spread), against a gross error. The heat sink's mode comes out
// 10-16 % faster at every step, a systematic difference reported in
// VALIDATION.md (M9 Stage B1′).
TEST(BallinT700, LinearizedGasGeneratorAgreesWithAppendixB) {
    const auto engine = load_engine();
    const double p2 = 14.696 * kPsi;
    const double t2 = 518.67 * kRankine;
    const double np = 20895.0 * kRpm;
    const double j_gt = engine->config().j_gt;
    struct Row {
        double fuel_lbm_hr, a11, b1, a21, b2, eig_ng, eig_hs;
    };
    // Figs. B1, B3, B5 (2-state) and B7, B9, B11 (3-state, eigenvalues).
    const std::array<Row, 3> rows{{
        {476.3, -0.2693e1, 0.1296e6, 0.3865e0, 0.2105e5, -2.229, -0.378},
        {349.3, -0.2233e1, 0.1494e6, 0.3128e0, 0.1663e5, -1.814, -0.353},
        {267.7, -0.1816e1, 0.1531e6, 0.3615e0, 0.1435e5, -1.267, -0.358},
    }};
    for (const Row& r : rows) {
        SCOPED_TRACE(r.fuel_lbm_hr);
        const double wf = r.fuel_lbm_hr * kLbmHr;
        double ng = 0.0;
        BallinT700Gas s;
        ASSERT_TRUE(engine->steady_gas(wf, np, p2, t2, ng, s));
        const auto ngdot = [&](double n, double w, double metal) {
            const BallinT700Gas g = engine->gas(n, np, metal, w, p2, t2);
            return (g.q_gt - g.q_c) / j_gt;  // [rad/s²]
        };
        const auto qpt = [&](double n, double w) {
            return engine->gas(n, np, -1.0, w, p2, t2).q_pt;
        };
        const double hn = 0.02 * ng;
        const double hw = 0.02 * wf;
        const double a11 = (ngdot(ng + hn, wf, -1.0) - ngdot(ng - hn, wf, -1.0)) / (2.0 * hn);
        const double b1 = (ngdot(ng, wf + hw, -1.0) - ngdot(ng, wf - hw, -1.0)) / (2.0 * hw) *
                          kPoundMass / kRpm;  // [rpm/s per lbm/s]
        const double ratio = (qpt(ng + hn, wf) - qpt(ng - hn, wf)) / (2.0 * hn) * kRpm /
                             ((qpt(ng, wf + hw) - qpt(ng, wf - hw)) / (2.0 * hw) * kPoundMass);
        const auto rel = [](double a, double b) { return std::abs(a / b - 1.0); };
        EXPECT_LT(rel(b1, r.b1), 0.10) << b1;
        EXPECT_LT(rel(a11, r.a11), 0.25) << a11;
        EXPECT_LT(rel(ratio, r.a21 / r.b2), 0.25) << ratio;
        // The gas generator with the heat sink: (NG, T_m).
        const double tm = s.t41ns;
        const double ht = 0.02 * tm;
        const auto rates = [&](double n, double m) {
            const BallinT700Gas g = engine->gas(n, np, m, wf, p2, t2);
            return Eigen::Vector2d((g.q_gt - g.q_c) / j_gt, (g.t41ns - m) / g.metal_lag);
        };
        Eigen::Matrix2d jac;
        jac.col(0) = (rates(ng + hn, tm) - rates(ng - hn, tm)) / (2.0 * hn);
        jac.col(1) = (rates(ng, tm + ht) - rates(ng, tm - ht)) / (2.0 * ht);
        const Eigen::Vector2cd ev = jac.eigenvalues();
        const double fast = std::min(ev(0).real(), ev(1).real());
        const double slow = std::max(ev(0).real(), ev(1).real());
        EXPECT_LT(std::abs(ev(0).imag()), 1e-9);
        EXPECT_LT(rel(fast, r.eig_ng), 0.25) << fast;
        EXPECT_LT(rel(slow, r.eig_hs), 0.25) << slow;
    }
}

// The component alone on shaft "main" at the governed 27 rad/s, reading the
// demand of "rotor" from a coupling store, sea level ISA.
class Harness {
public:
    Harness() {
        BallinT700Config c = load_engine()->config();
        c.shaft = "main";
        c.rotor = "rotor";
        engine_ = std::make_unique<BallinT700>(std::move(c));
        slot_ = registry_.add("rotor.shaft_torque", "rotor");
        store_ = CouplingOutputs(registry_.size());
        engine_->bind(registry_);
        const std::vector<ShaftInfo> info = {{"main", kGoverned}};
        engine_->bind_shafts(info);
    }
    static constexpr double kGoverned = 27.0;  // [rad/s]
    const BallinT700& engine() const { return *engine_; }

    SimContext context(double omega, double collective, double demand) const {
        SimContext ctx;
        ctx.num_shafts = 1;
        ctx.shafts[0] = {0.0, omega};
        ctx.controls.collective = collective;
        ctx.air = isa_atmosphere(0.0, 0.0);
        store_.begin_evaluation();
        store_.set(slot_, demand);
        ctx.coupling = &store_;
        return ctx;
    }
    // ẋ at a = 0, Ω̇ = 0; `q0` the shaft torque [N·m].
    void evaluate(std::span<const double> x, const SimContext& ctx, std::vector<double>& xdot,
                  double& q0) const {
        xdot.assign(BallinT700::kNumStates, 0.0);
        g_.assign(6 * BallinT700::kNumStates, 0.0);
        ComponentDerivs d{xdot, g_, Wrench{}, Mat66::Zero()};
        engine_->derivatives(ctx, x, d);
        q0 = d.Q0(0);
    }

private:
    std::unique_ptr<BallinT700> engine_;
    CouplingRegistry registry_;
    CouplingSlot slot_;
    mutable CouplingOutputs store_;
    mutable std::vector<double> g_;
};

// The trim seed (D-054): for the rotor's demand the engines deliver it plus
// the drive train's load, and every state is at rest: the gas path steady, the
// sensors at their inputs, the ECU and the torque motor holding the fuel the
// torque needs, inside their limits and stops. At Table B.1's three powers
// (229, 138.9 and 76.06 ft·lbf per engine at the turbine), each with
// collectives that go with it. The ECU's authority is bounded: through its
// P+I integrator's range (−0.3 to 3.5 V) and the torque motor it takes 0.30
// to 6.10 lbm/(hr·psi) off the HMU's demand, while the load-demand spindle
// anticipates the power from the collective; a collective far from the power
// (76 ft·lbf at 70 %) asks for more than the authority (TrimSeedHoldsTheEcu-
// AtItsLimitOutsideItsAuthority).
TEST(BallinT700, TrimSeedIsAtRest) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    struct Case {
        double q_pt_ftlbf;
        std::array<double, 3> collectives;
    };
    for (const Case& k : {Case{229.0, {0.5, 0.6, 0.7}}, Case{138.9, {0.35, 0.45, 0.55}},
                          Case{76.06, {0.25, 0.35, 0.45}}}) {
        const double q_pt_ftlbf = k.q_pt_ftlbf;
        for (const double collective : k.collectives) {
            SCOPED_TRACE(q_pt_ftlbf);
            SCOPED_TRACE(collective);
            const double demand = c.engines * c.gear * q_pt_ftlbf * kFtLbf -
                                  h.engine().drive_train_load(Harness::kGoverned);
            const SimContext ctx = h.context(Harness::kGoverned, collective, demand);
            std::vector<double> x(BallinT700::kNumStates);
            h.engine().trim_seed(ctx, x);
            std::vector<double> xdot;
            double q0 = 0.0;
            h.evaluate(x, ctx, xdot, q0);
            EXPECT_NEAR(q0, demand, 1e-6 * demand);
            for (std::size_t i = 0; i < xdot.size(); ++i) {
                EXPECT_NEAR(xdot[i], 0.0, 1e-6) << "state " << i;
            }
            const BallinT700Signals s = h.engine().signals(x, Harness::kGoverned, collective,
                                                           ctx.air.pressure, ctx.air.temperature);
            EXPECT_NEAR(s.gas.q_pt / kFtLbf, q_pt_ftlbf, 1e-6 * q_pt_ftlbf);
            EXPECT_GT(x[BallinT700::kTmPos], c.xlolim);
            EXPECT_LT(x[BallinT700::kTmPos], c.xhilim);
            EXPECT_GT(x[BallinT700::kZ], s.zlolim);
            EXPECT_LT(x[BallinT700::kZ], c.zhilim);
            EXPECT_EQ(s.tm_current, 0.0);  // the torque motor at rest
            EXPECT_DOUBLE_EQ(s.b4, 1.0);   // flight torque: the high-gain switch on
        }
    }
}

// Outside the ECU's authority there is no governed equilibrium: the seed
// holds the P+I integrator at the limit (here the most fuel it can take off,
// ZHILIM), and the gas path still delivers the demand.
TEST(BallinT700, TrimSeedHoldsTheEcuAtItsLimitOutsideItsAuthority) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const double demand =
        c.engines * c.gear * 76.06 * kFtLbf - h.engine().drive_train_load(Harness::kGoverned);
    const SimContext ctx = h.context(Harness::kGoverned, 0.7, demand);
    std::vector<double> x(BallinT700::kNumStates);
    h.engine().trim_seed(ctx, x);
    EXPECT_EQ(x[BallinT700::kZ], c.zhilim);
    std::vector<double> xdot;
    double q0 = 0.0;
    h.evaluate(x, ctx, xdot, q0);
    EXPECT_NEAR(q0, demand, 1e-6 * demand);
}

// Isochronous governing (Ballin p. 5; the T700's ECU, App. C): on a shaft
// with the UH-60A's rotor and drive-train inertia, a step from Ballin's 80 kt
// power to his hover power (138.9 to 229 ft·lbf per engine at the turbine)
// with the collective raised to match (0.45 to 0.6) droops the rotor, and the
// ECU's integrator brings it back to the reference. Settled, the torque
// motor's dead band (±2 mA, Table C.1) and the integrator hunt: a limit cycle
// of about ±0.035 % NP with a period near 4 s, its mean at the reference
// within ±0.006 % over any 5 s. RK4 at 1 ms, as the executive's step.
TEST(BallinT700, GovernsTheShaftIsochronously) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const double slug_ft2 = units::kSlug * kFoot * kFoot;
    // Blades 4 (I + 2eS + me²) = 6967 slug·ft² (D-053 item 5) and the drive
    // train's TM-88360 parts at rotor speed (D-054 item 3); the engines'
    // output sides are the component's.
    const double j_rest =
        (6967.0 + 208.4 + 41.85 + 19.69 * std::pow(124.62 / 27.0, 2) + 0.004 * 51.4 * 51.4) *
        slug_ft2;
    const double j = j_rest + c.engines * c.output_inertia * c.gear * c.gear;
    const auto demand_for = [&](double q_pt_ftlbf) {
        return c.engines * c.gear * q_pt_ftlbf * kFtLbf -
               h.engine().drive_train_load(Harness::kGoverned);
    };
    const double before = demand_for(138.9);
    const double after = demand_for(229.0);
    std::vector<double> x(BallinT700::kNumStates + 1);
    {
        std::vector<double> e(BallinT700::kNumStates);
        h.engine().trim_seed(h.context(Harness::kGoverned, 0.45, before), e);
        std::copy(e.begin(), e.end(), x.begin());
        x.back() = Harness::kGoverned;
    }
    const auto rates = [&](double t, const std::vector<double>& s) {
        const bool stepped = t >= 1.0;
        const double demand = stepped ? after : before;
        const SimContext ctx = h.context(s.back(), stepped ? 0.6 : 0.45, demand);
        std::vector<double> xdot;
        double q0 = 0.0;
        h.evaluate(std::span<const double>(s.data(), BallinT700::kNumStates), ctx, xdot, q0);
        xdot.push_back((q0 - demand) / j);
        return xdot;
    };
    const double dt = 0.001;
    double low = Harness::kGoverned;
    double tail_sum = 0.0;
    double tail_lo = 1e9;
    double tail_hi = 0.0;
    int tail_n = 0;
    for (int n = 0; n < 40000; ++n) {
        const double t = n * dt;
        const std::vector<double> k1 = rates(t, x);
        std::vector<double> s(x.size());
        for (std::size_t i = 0; i < x.size(); ++i)
            s[i] = x[i] + 0.5 * dt * k1[i];
        const std::vector<double> k2 = rates(t + 0.5 * dt, s);
        for (std::size_t i = 0; i < x.size(); ++i)
            s[i] = x[i] + 0.5 * dt * k2[i];
        const std::vector<double> k3 = rates(t + 0.5 * dt, s);
        for (std::size_t i = 0; i < x.size(); ++i)
            s[i] = x[i] + dt * k3[i];
        const std::vector<double> k4 = rates(t + dt, s);
        for (std::size_t i = 0; i < x.size(); ++i) {
            x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        low = std::min(low, x.back());
        if (n >= 30000) {
            tail_sum += x.back();
            tail_lo = std::min(tail_lo, x.back());
            tail_hi = std::max(tail_hi, x.back());
            ++tail_n;
        }
    }
    // A transient droop, then the reference on average over the last 10 s,
    // with the dead band's small limit cycle.
    EXPECT_LT(low, 0.99 * Harness::kGoverned);
    EXPECT_GT(low, 0.95 * Harness::kGoverned);
    EXPECT_NEAR(tail_sum / tail_n, Harness::kGoverned, 2e-4 * Harness::kGoverned);
    EXPECT_LT(tail_hi - tail_lo, 2e-3 * Harness::kGoverned);
}

// The executive sets the initial states before it binds the shafts, so they
// can't depend on the shaft (code review of PR #71): the NP reference is the
// design speed until bound, and the initial state, evaluated off any shaft
// (where the engine runs at its design speed and drives nothing), is at rest.
TEST(BallinT700, InitialStatesBeforeBindingAreAtRest) {
    const auto engine = load_engine();
    EXPECT_EQ(engine->np_reference(), 1.0);
    std::vector<double> x(BallinT700::kNumStates);
    engine->initial_states(x);
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    std::vector<double> xdot(BallinT700::kNumStates);
    std::vector<double> g(6 * BallinT700::kNumStates);
    ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
    engine->derivatives(ctx, x, d);
    EXPECT_TRUE(d.converged);
    for (std::size_t i = 0; i < xdot.size(); ++i) {
        EXPECT_NEAR(xdot[i], 0.0, 1e-6) << "state " << i;
    }
    const BallinT700Config& c = engine->config();
    EXPECT_GT(x[BallinT700::kZ], c.zlolim_high);
    EXPECT_LT(x[BallinT700::kZ], c.zhilim);
    EXPECT_NEAR(x[BallinT700::kSpdsf], 0.0, 1e-12);  // no speed error
}

// Bound to a shaft governed off the engine's design speed (27 rad/s against
// NP_des/gear = 27.006 rad/s), the initial state is the rest point at the
// shaft's speed: every rate zero, no torque at the shaft, no speed error
// (code review of PR #71).
TEST(BallinT700, InitialStatesOnTheBoundShaftAreAtRest) {
    const Harness h;
    std::vector<double> x(BallinT700::kNumStates);
    h.engine().initial_states(x);
    std::vector<double> xdot;
    double q0 = 0.0;
    h.evaluate(x, h.context(Harness::kGoverned, 0.0, 0.0), xdot, q0);
    for (std::size_t i = 0; i < xdot.size(); ++i) {
        EXPECT_NEAR(xdot[i], 0.0, 1e-6) << "state " << i;
    }
    EXPECT_NEAR(q0, 0.0, 1e-6);
    EXPECT_NEAR(x[BallinT700::kSpdsf], 0.0, 1e-12);
}

// The constants the engine divides by or takes a root of are checked at
// init, as its lags are (code review of PR #71).
TEST(BallinT700, RejectsNonPositiveDivisors) {
    const BallinT700Config base = load_engine()->config();
    for (double BallinT700Config::*field :
         {&BallinT700Config::tm_gain, &BallinT700Config::tmlvg, &BallinT700Config::tmlg,
          &BallinT700Config::k_dpb, &BallinT700Config::tc_t41, &BallinT700Config::k_wgt}) {
        BallinT700Config c = base;
        c.*field = 0.0;
        EXPECT_THROW(BallinT700{std::move(c)}, InitError);
    }
    BallinT700Config c = base;
    c.wfmin = -1e-3;
    EXPECT_THROW(BallinT700{std::move(c)}, InitError);
}

// A state where the gas path has no solution (here NaN) is reported:
// derivatives() clears the convergence flag (§7), as the rotors do.
TEST(BallinT700, ReportsAnUnconvergedGasPath) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const SimContext ctx = h.context(Harness::kGoverned, 0.55, c.engines * c.gear * 229.0 * kFtLbf);
    std::vector<double> x(BallinT700::kNumStates);
    h.engine().trim_seed(ctx, x);
    std::vector<double> xdot(BallinT700::kNumStates);
    std::vector<double> g(6 * BallinT700::kNumStates);
    {
        ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
        h.engine().derivatives(ctx, x, d);
        EXPECT_TRUE(d.converged);
    }
    x[BallinT700::kNg] = std::numeric_limits<double>::quiet_NaN();
    ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
    h.engine().derivatives(ctx, x, d);
    EXPECT_FALSE(d.converged);
    EXPECT_TRUE(std::isfinite(xdot[BallinT700::kMetal]));  // the stand-ins keep it finite
}

// A demand no steady state delivers (ten times the hover torque; the steady
// solve fails from twice it) seeds a finite, fuelled state: the seed falls
// back to the rest point's fuel, not the failed solve's zero fuel, nor 0/0
// where no steady state at zero fuel exists (code review of PR #71).
TEST(BallinT700, TrimSeedPastTheMapsIsFinite) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const SimContext ctx =
        h.context(Harness::kGoverned, 0.9, 10.0 * c.engines * c.gear * 229.0 * kFtLbf);
    std::vector<double> x(BallinT700::kNumStates);
    h.engine().trim_seed(ctx, x);
    for (std::size_t i = 0; i < x.size(); ++i) {
        EXPECT_TRUE(std::isfinite(x[i])) << "state " << i;
    }
    EXPECT_GT(x[BallinT700::kValve] * BallinT700::kFuelScale, 100.0 * kLbmHr);
    EXPECT_GT(x[BallinT700::kNg], 0.65);
}

// Past the maps at altitude the seed is still a steady gas path at the
// inlet's pressure and temperature, not sea level's: the gas generator and
// the PS3 sensor at rest (code review of PR #71).
TEST(BallinT700, TrimSeedPastTheMapsIsAtTheInlet) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    for (const double feet : {0.0, 10000.0, 20000.0}) {
        SCOPED_TRACE(feet);
        SimContext ctx =
            h.context(Harness::kGoverned, 0.9, 10.0 * c.engines * c.gear * 229.0 * kFtLbf);
        ctx.air = isa_atmosphere(feet * kFoot, 0.0);
        std::vector<double> x(BallinT700::kNumStates);
        h.engine().trim_seed(ctx, x);
        std::vector<double> xdot(BallinT700::kNumStates);
        std::vector<double> g(6 * BallinT700::kNumStates);
        ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
        h.engine().derivatives(ctx, x, d);
        EXPECT_TRUE(d.converged);
        EXPECT_NEAR(xdot[BallinT700::kNg], 0.0, 1e-6);
        EXPECT_NEAR(xdot[BallinT700::kPs3Lag], 0.0, 1e-6);
    }
}

// Linearization holds the flapper only where it doesn't move: at rest in the
// dead band (the trim seed), or pressed on a stop; a current that drives it
// mid-travel leaves it free (code review of PR #71).
TEST(BallinT700, PinsTheFlapperOnlyWhereItDoesntMove) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const SimContext ctx = h.context(Harness::kGoverned, 0.55, c.engines * c.gear * 229.0 * kFtLbf);
    std::vector<double> x(BallinT700::kNumStates);
    h.engine().trim_seed(ctx, x);
    std::vector<unsigned char> pinned(BallinT700::kNumStates);
    h.engine().pinned_states(ctx, x, pinned);
    EXPECT_EQ(pinned[BallinT700::kTmPos], 1);
    EXPECT_EQ(pinned[BallinT700::kPs3Hys], 1);
    // A current: SPDG half a volt up, its lead-lag settled, the flapper mid-travel.
    x[BallinT700::kSpdg] += 0.5;
    x[BallinT700::kTm1] += 0.5;
    h.engine().pinned_states(ctx, x, pinned);
    EXPECT_EQ(pinned[BallinT700::kTmPos], 0);
    // The same current with the flapper on the stop it drives toward.
    x[BallinT700::kTmPos] = c.xlolim;
    x[BallinT700::kTm1] = x[BallinT700::kSpdg] + c.tm_bias - c.tmlvg * c.xlolim;
    const BallinT700Signals s =
        h.engine().signals(x, Harness::kGoverned, 0.55, ctx.air.pressure, ctx.air.temperature);
    h.engine().pinned_states(ctx, x, pinned);
    EXPECT_EQ(pinned[BallinT700::kTmPos], s.tm_current < 0.0 ? 1 : 0);
}

// Real-time rules (§7): derivatives() allocates nothing, and depends only on
// its inputs (the lookups' cursors are hints): evaluating another state in
// between leaves the result bit for bit.
TEST(BallinT700, DerivativesArePureAndAllocateNothing) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const double demand = c.engines * c.gear * 229.0 * kFtLbf;
    const SimContext ctx = h.context(Harness::kGoverned, 0.55, demand);
    std::vector<double> x(BallinT700::kNumStates);
    h.engine().trim_seed(ctx, x);
    x[BallinT700::kValve] *= 1.2;  // off equilibrium
    std::vector<double> a;
    std::vector<double> b;
    double q = 0.0;
    h.evaluate(x, ctx, a, q);
    std::vector<double> other = x;
    other[BallinT700::kNg] *= 0.9;
    other[BallinT700::kMetal] *= 1.1;
    std::vector<double> scratch;
    h.evaluate(other, ctx, scratch, q);
    h.evaluate(x, ctx, b, q);
    EXPECT_EQ(a, b);
    std::vector<double> xdot(BallinT700::kNumStates);
    std::vector<double> g(6 * BallinT700::kNumStates);
    ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
    const test::AllocationCounter counter;
    h.engine().derivatives(ctx, x, d);
    EXPECT_EQ(counter.count(), 0U);
}

// A reset starts the aircraft where init does, the engines at rest on the
// bound shaft both times (code review of PR #71).
TEST(BallinT700Aircraft, ResetStartsWhereInitDoes) {
    const Config cfg =
        Config::from_file(std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/engine_ballin/uh60a.yaml");
    Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);
    const std::vector<double> at_init(ex.state().begin(), ex.state().end());
    ASSERT_TRUE(ex.reset(BodyState{}));
    for (std::size_t i = 0; i < at_init.size(); ++i) {
        EXPECT_EQ(ex.state()[i], at_init[i]) << "state " << i;
    }
}

// The UH-60A on Ballin's engines (data/uh60a/engine_ballin/uh60a_table_b1.yaml,
// Table B.1's loading, sea level ISA, D-054) trims at Table B.1's three
// conditions, hover, 80 kt and 80 kt at −1000 ft/min (γ = −7.08°), within
// the M7a sweep's 15 iterations: the governor holds the rotor at 27 rad/s
// (isochronous, its integrator inside its range) and the revolution closes.
// The comparison with Table B.1's engine values is in VALIDATION.md (M9
// Stage B1′); Eyas's tail rotor takes no torque from the shaft yet (#40).
TEST(BallinT700Aircraft, TrimsAtTableB1Conditions) {
    const Config cfg = Config::from_file(std::string(EYAS_VEHICLE_DATA_DIR) +
                                         "/uh60a/engine_ballin/uh60a_table_b1.yaml");
    const Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);
    const BallinT700* engine = nullptr;
    std::size_t first = 0;
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* e = dynamic_cast<const BallinT700*>(&ex.component(i))) {
            engine = e;
            first = ex.component_offset(i) - body_layout::kSize;
        }
    }
    ASSERT_NE(engine, nullptr);
    const std::size_t omega_at = ex.shaft_offset(0) + 1 - body_layout::kSize;
    const double governed = ex.shafts()[0].speed;
    std::vector<double> x;
    for (const auto& [knots, gamma_deg] :
         {std::pair{0.0, 0.0}, std::pair{80.0, 0.0}, std::pair{80.0, -7.08}}) {
        SCOPED_TRACE(knots);
        SCOPED_TRACE(gamma_deg);
        TrimCondition c;
        c.airspeed = knots * units::kKnot;
        c.flight_path_angle = gamma_deg * units::kDegree;
        const TrimResult r = solve_trim(ex, c, nullptr, x);
        ASSERT_TRUE(r.converged);
        EXPECT_LE(r.iterations, 15);
        const double omega = r.component_states[omega_at];
        EXPECT_NEAR(omega, governed, 1e-6 * governed);
        EXPECT_NEAR(omega * r.period, 2.0 * std::numbers::pi, 1e-5);
        const double z = r.component_states[first + BallinT700::kZ];
        EXPECT_GT(z, engine->config().zlolim_high);
        EXPECT_LT(z, engine->config().zhilim);
    }
}

}  // namespace
}  // namespace eyas
