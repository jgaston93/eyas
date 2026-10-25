// M9: Ballin's component-level T700 (NASA TM-100991; D-054). The gas path
// against Ballin's printed steady states (Table B.1), the curve-family
// lookup, and the config.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "analysis/held_orbit.hpp"
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
    explicit Harness(const std::function<void(BallinT700Config&)>& edit = {}) {
        BallinT700Config c = load_engine()->config();
        if (edit) {
            edit(c);
        }
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
    // Each engine's fuel shutoff in every context from here on (D-056).
    void set_fuel(std::array<bool, 2> on) { fuel_ = on; }
    std::size_t size() const { return engine_->num_states(); }
    // Engine 0's signals at x.
    BallinT700Signals signals(const std::vector<double>& x, const SimContext& ctx) const {
        return engine_->signals(std::span<const double>(x).first(BallinT700::kEngineStates),
                                engine_->inputs(ctx, 0));
    }

    SimContext context(double omega, double collective, double demand,
                       std::array<double, 2> levers = {1.0, 1.0}) const {
        SimContext ctx;
        ctx.num_shafts = 1;
        ctx.shafts[0] = {0.0, omega};
        ctx.controls.collective = collective;
        ctx.controls.engines[0].power_lever = levers[0];
        ctx.controls.engines[1].power_lever = levers[1];
        ctx.controls.engines[0].fuel_on = fuel_[0];
        ctx.controls.engines[1].fuel_on = fuel_[1];
        ctx.air = isa_atmosphere(0.0, 0.0);
        store_.begin_evaluation();
        store_.set(slot_, demand);
        ctx.coupling = &store_;
        return ctx;
    }
    // ẋ at a = 0, Ω̇ = 0; `q0` the shaft torque [N·m].
    void evaluate(std::span<const double> x, const SimContext& ctx, std::vector<double>& xdot,
                  double& q0) const {
        xdot.assign(engine_->num_states(), 0.0);
        g_.assign(6 * engine_->num_states(), 0.0);
        ComponentDerivs d{xdot, g_, Wrench{}, Mat66::Zero()};
        engine_->derivatives(ctx, x, d);
        q0 = d.Q0(0);
    }

    // One RK4 step of `dt` [s] of the engines' states and the shaft's speed
    // (x's last entry) on a drive of inertia J_d (the config's) loaded by the
    // rotor's `demand` [N·m], or with the shaft held at its speed. Body at
    // rest.
    void step(std::vector<double>& x, double dt, double collective, std::array<double, 2> levers,
              double demand, bool hold) const {
        const std::size_t n = size();
        const auto rates = [&](const std::vector<double>& s) {
            const SimContext ctx = context(s.back(), collective, demand, levers);
            std::vector<double> xdot(n, 0.0);
            g_.assign(6 * n, 0.0);
            gs_.assign(n * kMaxShafts, 0.0);
            ComponentDerivs d{xdot, g_, Wrench{}, Mat66::Zero()};
            d.Gs = gs_;
            engine_->derivatives(ctx, std::span<const double>(s.data(), n), d);
            // J_d Ω̇ = Q0 + K_s Ω̇ − demand (the engaged engines' inertia in K_s).
            const double wdot =
                hold ? 0.0 : (d.Q0(0) - demand) / (engine_->config().drive_inertia - d.Ks(0, 0));
            for (std::size_t i = 0; i < n; ++i) {
                xdot[i] += gs_[i * kMaxShafts] * wdot;
            }
            xdot.push_back(wdot);
            return xdot;
        };
        const std::vector<double> k1 = rates(x);
        std::vector<double> s(x.size());
        for (std::size_t i = 0; i < x.size(); ++i)
            s[i] = x[i] + 0.5 * dt * k1[i];
        const std::vector<double> k2 = rates(s);
        for (std::size_t i = 0; i < x.size(); ++i)
            s[i] = x[i] + 0.5 * dt * k2[i];
        const std::vector<double> k3 = rates(s);
        for (std::size_t i = 0; i < x.size(); ++i)
            s[i] = x[i] + dt * k3[i];
        const std::vector<double> k4 = rates(s);
        for (std::size_t i = 0; i < x.size(); ++i) {
            x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
    // Engine e's signals in a flight state (the shaft's speed last).
    BallinT700Signals signals_at(const std::vector<double>& x, std::size_t e, double collective,
                                 std::array<double, 2> levers) const {
        const SimContext ctx = context(x.back(), collective, 0.0, levers);
        return engine_->signals(std::span<const double>(x).subspan(e * BallinT700::kEngineStates,
                                                                   BallinT700::kEngineStates),
                                engine_->inputs(ctx, static_cast<int>(e)));
    }
    // The engines' trim seed for the demand, then the shaft at kGoverned.
    std::vector<double> seeded(double collective, double demand) const {
        std::vector<double> x(size());
        engine_->trim_seed(context(kGoverned, collective, demand), x);
        x.push_back(kGoverned);
        return x;
    }

private:
    std::unique_ptr<BallinT700> engine_;
    CouplingRegistry registry_;
    CouplingSlot slot_;
    mutable CouplingOutputs store_;
    mutable std::vector<double> g_;
    mutable std::vector<double> gs_;
    std::array<bool, 2> fuel_{true, true};
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
            std::vector<double> x(h.size());
            h.engine().trim_seed(ctx, x);
            std::vector<double> xdot;
            double q0 = 0.0;
            h.evaluate(x, ctx, xdot, q0);
            EXPECT_NEAR(q0, demand, 1e-6 * demand);
            for (std::size_t i = 0; i < xdot.size(); ++i) {
                EXPECT_NEAR(xdot[i], 0.0, 1e-6) << "state " << i;
            }
            const BallinT700Signals s = h.signals(x, ctx);
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
    std::vector<double> x(h.size());
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
    std::vector<double> x(h.size() + 1);
    {
        std::vector<double> e(h.size());
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
        h.evaluate(std::span<const double>(s.data(), h.size()), ctx, xdot, q0);
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
    std::vector<double> x(engine->num_states());
    engine->initial_states(x);
    SimContext ctx;
    ctx.air = isa_atmosphere(0.0, 0.0);
    std::vector<double> xdot(engine->num_states());
    std::vector<double> g(6 * engine->num_states());
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
    std::vector<double> x(h.size());
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
    std::vector<double> x(h.size());
    h.engine().trim_seed(ctx, x);
    std::vector<double> xdot(h.size());
    std::vector<double> g(6 * h.size());
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
    std::vector<double> x(h.size());
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
        std::vector<double> x(h.size());
        h.engine().trim_seed(ctx, x);
        std::vector<double> xdot(h.size());
        std::vector<double> g(6 * h.size());
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
    std::vector<double> x(h.size());
    h.engine().trim_seed(ctx, x);
    std::vector<unsigned char> pinned(h.size());
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
    const BallinT700Signals s = h.signals(x, ctx);
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
    std::vector<double> x(h.size());
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
    std::vector<double> xdot(h.size());
    std::vector<double> g(6 * h.size());
    ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
    const test::AllocationCounter counter;
    h.engine().derivatives(ctx, x, d);
    EXPECT_EQ(counter.count(), 0U);
}

// ---- Stage B2a (D-055) ----------------------------------------------------------

// The power lever's rigging (D-055 item 2): linear between the stops, clamped,
// NaN at fly.
TEST(BallinT700, PowerLeverRigsTheSpindle) {
    const auto engine = load_engine();
    const BallinT700Config& c = engine->config();
    EXPECT_DOUBLE_EQ(engine->spindle(0.0), c.pas_idle);
    EXPECT_DOUBLE_EQ(engine->spindle(1.0), c.pas_fly);
    EXPECT_DOUBLE_EQ(engine->spindle(0.25), c.pas_idle + 0.25 * (c.pas_fly - c.pas_idle));
    EXPECT_DOUBLE_EQ(engine->spindle(-3.0), c.pas_idle);
    EXPECT_DOUBLE_EQ(engine->spindle(7.0), c.pas_fly);
    EXPECT_DOUBLE_EQ(engine->spindle(std::numeric_limits<double>::quiet_NaN()), c.pas_fly);
    EXPECT_NEAR(c.pas_fly, 120.0 * units::kDegree, 1e-12);
}

// Fig. C30 as read (D-055 item 6): the family is read along each T2 curve and
// across them, from the digitized points (60 % up).
TEST(BallinT700, AccelerationLimitReadsFigC30) {
    const auto engine = load_engine();
    const CurveFamily& f = *engine->config().fhm7;
    CurveFamily::Cursor k = f.make_cursor();
    const double ratio = kLbmHr / kPsi;  // lbm*in^2/(lbf*hr)
    ASSERT_EQ(f.params().size(), 7U);
    EXPECT_NEAR(f.params()[0], 395.0 * kRankine, 1e-9);
    EXPECT_NEAR(f.params()[6], 590.0 * kRankine, 1e-9);
    // Readings (data/uh60a/ballin_t700/C30.csv): T2 = 519 °R at 79.92 % and
    // 395 °R on its floor past 94.6 %.
    EXPECT_NEAR(f(519.0 * kRankine, 0.79923, k) / ratio, 3.4745, 1e-3);
    EXPECT_NEAR(f(395.0 * kRankine, 0.96, k) / ratio, 0.99, 0.01);
    // Below the first column (60 %) the curves are clamped.
    EXPECT_DOUBLE_EQ(f(519.0 * kRankine, 0.55, k), f(519.0 * kRankine, 0.5999, k));
}

// At the idle stop (D-055 item 2) the idle schedule holds the gas generator
// (Fig. C20): steady, the fuel ratio sits on WFIDM = KNDRP (PCNGI − PCNGHL) +
// WFIRF, so NG = PCNGI + (WFIRF − W_f/P_s3)/KNDRP, a little above the
// schedule's PCNGI (68.9 % at 518.67 °R, Fig. C29). Shaft held at its speed,
// sea level, collective down, from the 80 kt trim.
TEST(BallinT700, IdleStopHoldsTheIdleSchedule) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    for (int n = 0; n < 30000; ++n) {
        h.step(x, 0.001, 0.0, {0.0, 0.0}, 0.0, true);
    }
    const BallinT700Signals s = h.signals_at(x, 0, 0.0, {0.0, 0.0});
    EXPECT_EQ(s.hmusel, s.wfidm);  // the idle floor selected
    const double t2 = isa_atmosphere(0.0, 0.0).temperature;
    const double pcngi = (*c.fhm6)(t2);
    const double ratio = s.fuel / s.ps3l;
    const double ng = pcngi + ((*c.fhm5)(t2)-ratio) / c.kndrp;
    EXPECT_NEAR(x[BallinT700::kNgSensor], ng, 2e-4);  // steady: the ratio is WFIDM
    EXPECT_GT(x[BallinT700::kNg], pcngi);
    EXPECT_LT(x[BallinT700::kNg], pcngi + 0.02);
    // Both engines alike, bit for bit.
    for (std::size_t i = 0; i < BallinT700::kEngineStates; ++i) {
        EXPECT_EQ(x[i], x[BallinT700::kEngineStates + i]) << i;
    }
}

// From idle, the levers to fly and the collective to hover: the governor
// asks for far more fuel than the acceleration schedule allows (Fig. C30), so
// the fuel ratio rides WFPAC (Fig. C18's MIN) while the gas generator spools
// up. The shaft is held at its speed (the engines alone, as Ballin's
// open-loop runs). Without the limit the ratio would follow WFPDM, about
// three times it.
TEST(BallinT700, AccelerationFromIdleRidesTheLimit) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    for (int n = 0; n < 30000; ++n) {
        h.step(x, 0.001, 0.0, {0.0, 0.0}, 0.0, true);
    }
    const double ng_idle = x[BallinT700::kNg];
    int on_limit = 0;
    double above = 0.0;  // the most WFPDM exceeded the limit
    for (int n = 0; n < 2000; ++n) {
        h.step(x, 0.001, 0.6, {1.0, 1.0}, 0.0, true);
        if (n % 100 == 99) {
            const BallinT700Signals s = h.signals_at(x, 0, 0.6, {1.0, 1.0});
            on_limit += s.hmusel == s.wfpac ? 1 : 0;
            above = std::max(above, s.wfpdm / s.wfpac);
        }
    }
    EXPECT_GE(on_limit, 15);  // of 20 samples in the first 2 s
    EXPECT_GT(above, 2.0);
    EXPECT_GT(x[BallinT700::kNg], ng_idle + 0.1);
}

// The clutches' rule (D-055 item 5; TM-88360 App. A p. 21): the engines at
// zero slip engage together unless one alone would accelerate slower than the
// drive does with the set engaged, (Σ Q_e − L)/(J_d + Σ J_e).
TEST(BallinT700, ClutchesEngageTheEnginesThatDrive) {
    const auto engine = load_engine();
    const BallinT700Config& c = engine->config();
    const double je = c.output_inertia * c.gear * c.gear;
    const double jd = c.drive_inertia;
    const double load = 20000.0;  // [N·m]
    const auto mask = [&](double q0, double q1, double s0, double s1, double l) {
        const std::array<double, 2> q{q0, q1};
        const std::array<double, 2> s{s0, s1};
        return engine->engaged(q, s, l);
    };
    // Both driving: both engaged, whatever the load.
    EXPECT_EQ(mask(12000.0, 12000.0, 0.0, 0.0, load), 3U);
    EXPECT_EQ(mask(12000.0, 12000.0, 0.0, 0.0, -load), 3U);
    // Slipping: out.
    EXPECT_EQ(mask(12000.0, 12000.0, 0.0, 1e-3, load), 1U);
    // An engine whose own deceleration is faster than the drive's: out. One
    // with a small drag torque decelerates slower than the loaded drive, so
    // its spool's inertia still drives through the clutch: in.
    EXPECT_EQ(mask(12000.0, -20000.0, 0.0, 0.0, 1000.0), 1U);
    EXPECT_EQ(mask(12000.0, -50.0, 0.0, 0.0, load), 3U);
    // The rotor driving the shaft (negative load, autorotation): an engine
    // whose own acceleration is below the drive's is out. With engine 0 alone
    // the drive accelerates at (q0 + 30000)/(J_d + J_e); engine 1 stays in
    // just above it, drops out just below it.
    const double q0 = 500.0;
    const double common = (q0 + 30000.0) / (jd + je);
    EXPECT_EQ(mask(q0, je * common * 1.5, 0.0, 0.0, -30000.0) & 2U, 2U);
    // Below the drive's acceleration, both out: removing one only raises it.
    EXPECT_EQ(mask(q0, q0, 0.0, 0.0, -30000.0), 0U);
}

// A disengaged engine (slip > 0) turns on its own (D-055 item 5): its torque
// and inertia leave the shaft, ṡ = Ω̇ + n·ω̇/n_E − Q_e/J_e, and the other
// engine still drives. Its torque at the shaft and inertia are exactly one
// engine's.
TEST(BallinT700, DisengagedEngineTurnsOnItsOwn) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const double demand = c.engines * c.gear * 229.0 * kFtLbf - h.engine().drive_train_load(27.0);
    const SimContext ctx = h.context(Harness::kGoverned, 0.6, demand);
    std::vector<double> x(h.size());
    h.engine().trim_seed(ctx, x);
    const std::size_t n = h.size();
    const auto eval = [&](const std::vector<double>& s, std::vector<double>& xdot,
                          std::vector<double>& gs, ComponentDerivs& out) {
        xdot.assign(n, 0.0);
        gs.assign(n * kMaxShafts, 0.0);
        out.xdot0 = xdot;
        out.Gs = gs;
        h.engine().derivatives(ctx, s, out);
    };
    std::vector<double> g(6 * n);
    std::vector<double> xd0;
    std::vector<double> gs0;
    ComponentDerivs both{xd0, g, Wrench{}, Mat66::Zero()};
    eval(x, xd0, gs0, both);
    std::vector<double> y = x;
    y[BallinT700::kEngineStates + BallinT700::kSlip] = 0.5;  // engine 1 behind the drive
    std::vector<double> xd1;
    std::vector<double> gs1;
    std::vector<double> g1(6 * n);
    ComponentDerivs one{xd1, g1, Wrench{}, Mat66::Zero()};
    eval(y, xd1, gs1, one);
    const BallinT700Signals s1 = h.engine().signals(
        std::span<const double>(y).subspan(BallinT700::kEngineStates, BallinT700::kEngineStates),
        h.engine().inputs(ctx, 1));
    const double je = c.output_inertia * c.gear * c.gear;
    const std::size_t slip = BallinT700::kEngineStates + BallinT700::kSlip;
    const BallinT700Signals s0 = h.engine().signals(
        std::span<const double>(x).first(BallinT700::kEngineStates), h.engine().inputs(ctx, 0));
    // Engine 0 alone on the shaft: half the identical pair's torque.
    EXPECT_NEAR(one.Q0(0), both.Q0(0) - s0.q_engine, 1e-9 * std::abs(both.Q0(0)));
    EXPECT_NEAR(one.Ks(0, 0), both.Ks(0, 0) + je, 1e-9 * je);
    EXPECT_DOUBLE_EQ(xd1[slip], -s1.q_engine / je);
    EXPECT_EQ(gs1[slip * kMaxShafts], 1.0);
    EXPECT_DOUBLE_EQ(g1[slip * 6 + 3], c.pt_axis_b.x() / c.gear);
    // Engaged, the slip doesn't move.
    EXPECT_EQ(xd0[slip], 0.0);
    EXPECT_EQ(gs0[slip * kMaxShafts], 0.0);
    // The disengaged engine's NP sensor reads its own, slower turbine.
    EXPECT_LT(s1.engine_speed, Harness::kGoverned);
}

// Load share (Fig. C2; D-055 item 4). The engines governing one shaft share
// its speed error, so a difference between their trims (here engine 1's P+I
// integrator started 0.6 V lower: more fuel) persists without load share.
// With it the engine delivering less sees an underspeed of up to 2.3 % NP
// and adds fuel until the sensed torques differ by no more than the dead
// zone DBIAS/1.1 = 2.95 ft·lbf. At 80 kt power on the UH-60A's drive, 60 s.
TEST(BallinT700, LoadShareBringsTheEnginesTogether) {
    const auto run = [](bool share) {
        const Harness h([share](BallinT700Config& c) {
            if (!share) {
                c.zk8 = 0.0;
            }
        });
        const BallinT700Config& c = h.engine().config();
        const double demand =
            c.engines * c.gear * 138.9 * kFtLbf - h.engine().drive_train_load(Harness::kGoverned);
        std::vector<double> x = h.seeded(0.45, demand);
        x[BallinT700::kEngineStates + BallinT700::kZ] -= 0.6;
        for (int n = 0; n < 60000; ++n) {
            h.step(x, 0.001, 0.45, {1.0, 1.0}, demand, false);
        }
        return (x[BallinT700::kTorqueSensed] -
                x[BallinT700::kEngineStates + BallinT700::kTorqueSensed]) /
               kFtLbf;
    };
    const double with = run(true);
    const double without = run(false);
    EXPECT_LT(std::abs(with), 2.95 * 1.05) << with;
    EXPECT_GT(std::abs(without), 3.0 * std::abs(with) + 5.0) << without;
}

// More engines than Controls::engines holds, or than a Ballin component's
// load share pairs, are refused (D-055 items 1, 4).
TEST(BallinT700, ConfigLimitsTheEngines) {
    const std::string dir = std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/engine_ballin/";
    const Config c = Config::from_file(dir + "ballin_t700.yaml");
    EXPECT_EQ(BallinT700::from_config(c, 2)->config().first_engine, 2);
    std::ifstream in(dir + "ballin_t700.yaml");
    std::stringstream buffer;
    buffer << in.rdbuf();
    // The tables by absolute path, and three engines.
    std::string text =
        std::regex_replace(buffer.str(), std::regex(R"(([\s{])(\w+\.csv))"), "$1" + dir + "$2");
    const std::string from = "engines: 2";
    ASSERT_NE(text.find(from), std::string::npos);
    EXPECT_NO_THROW(BallinT700::from_config(Config::from_string(text)));
    text.replace(text.find(from), from.size(), "engines: 3");
    EXPECT_THROW(BallinT700::from_config(Config::from_string(text)), InitError);
}

// ---- Stage B2b (D-056) ----------------------------------------------------------

// The maps' floor (D-056 item 3): the highest first corrected speed of the
// maps read by NGC (f1, f3, f10, f_hs), so every one has data above it, as a
// fraction of NG_des at the inlet's temperature (NGC = NG/(NG_des √θ2)).
TEST(BallinT700, MapsFloorIsTheHighestFirstSpeed) {
    const auto engine = load_engine();
    const BallinT700Config& c = engine->config();
    const double floor =
        std::max({c.f1->params().front(), c.f3->x_axis().breakpoints().front(),
                  c.f10->x_axis().breakpoints().front(), c.f_hs->x_axis().breakpoints().front()});
    EXPECT_DOUBLE_EQ(engine->maps_floor(), floor);
    EXPECT_NEAR(floor, 0.65048, 1e-5);  // f3's (data/uh60a/engine_ballin/f3_seal_bleed.csv)
    const double t2 = 250.0;
    EXPECT_DOUBLE_EQ(engine->ng_floor(t2), floor * std::sqrt(t2 / BallinT700::kTstd));
}

// The shutoff (D-056 item 2): with the fuel off the metering valve's command
// is zero, below the valve's minimum flow WFMIN; with it on, at least WFMIN.
TEST(BallinT700, FuelShutoffClosesTheValve) {
    Harness h;
    const BallinT700Config& c = h.engine().config();
    const std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    const BallinT700Signals on = h.signals_at(x, 1, 0.0, {0.0, 0.0});
    EXPECT_GE(on.fuel_command, c.wfmin);
    h.set_fuel({true, false});
    const BallinT700Signals off = h.signals_at(x, 1, 0.0, {0.0, 0.0});
    EXPECT_EQ(off.fuel_command, 0.0);
    EXPECT_GT(off.fuel, 0.0);  // the valve and the line still deliver, for now
    EXPECT_EQ(h.signals_at(x, 0, 0.0, {0.0, 0.0}).fuel_command, on.fuel_command);
}

// A shutdown at 80 kt power (D-056 items 2, 3), the shaft held at its speed:
// the delivered fuel follows the valve's lag and the line's delay down (the
// delay's Padé approximant lifts it by 2.5e-7 of NG in the first 5 ms, its
// inverse response; D-054 item 7); Ballin's gas path then runs the gas
// generator down through the maps, steadily, its power turbine's torque
// falling to zero within 2 s, and the clutch lets go once it drags. At the
// maps' floor the engine is out: the gas generator held there, no torque.
// The other engine is untouched.
TEST(BallinT700, ShutdownRunsDownToTheMapsFloor) {
    Harness h;
    const BallinT700Config& c = h.engine().config();
    std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    const std::vector<double> start = x;
    const double t2 = isa_atmosphere(0.0, 0.0).temperature;
    const double floor = h.engine().ng_floor(t2);
    const double q_start = h.signals_at(x, 1, 0.45, {1.0, 1.0}).q_engine;
    const double fuel_start = h.signals_at(x, 1, 0.45, {1.0, 1.0}).fuel;
    h.set_fuel({true, false});
    double t_fuel = -1.0;    // when the delivered fuel fell below 1 % of its start [s]
    double t_torque = -1.0;  // when the torque fell below 2 % of its start [s]
    double t_floor = -1.0;   // when NG reached the floor [s]
    double ng_prev = x[BallinT700::kEngineStates + BallinT700::kNg];
    bool monotonic = true;
    for (int n = 1; n <= 20000; ++n) {
        h.step(x, 0.001, 0.45, {1.0, 1.0}, 0.0, true);
        const double t = n * 0.001;
        const BallinT700Signals s = h.signals_at(x, 1, 0.45, {1.0, 1.0});
        const double ng = x[BallinT700::kEngineStates + BallinT700::kNg];
        if (t_fuel < 0.0 && s.fuel < 0.01 * fuel_start) {
            t_fuel = t;
        }
        if (t_torque < 0.0 && std::abs(s.q_engine) < 0.02 * q_start) {
            t_torque = t;
        }
        if (t_floor < 0.0 && ng <= floor + 1e-6) {
            t_floor = t;  // a step's overshoot below it returns (the limiter)
        }
        if (t > 0.1 && t_floor < 0.0 && ng > ng_prev + 1e-12 && monotonic) {
            monotonic = false;
            ADD_FAILURE() << "NG rose at t = " << t << " by " << ng - ng_prev << " at " << ng
                          << " (floor " << floor << ")";
        }
        ng_prev = ng;
        for (const double v : x) {
            ASSERT_TRUE(std::isfinite(v)) << t;
        }
    }
    EXPECT_GT(t_fuel, 0.0);
    EXPECT_LT(t_fuel, 0.2);
    EXPECT_GT(t_torque, 0.0);
    EXPECT_LT(t_torque, 2.0);
    EXPECT_GT(t_floor, t_torque);
    std::cout << "fuel below 1 % at " << t_fuel << " s, torque below 2 % at " << t_torque
              << " s, NG on the floor at " << t_floor << " s\n";
    const BallinT700Signals s = h.signals_at(x, 1, 0.45, {1.0, 1.0});
    EXPECT_TRUE(s.out);
    EXPECT_EQ(s.q_engine, 0.0);
    EXPECT_NEAR(x[BallinT700::kEngineStates + BallinT700::kNg], floor, 1e-6);
    EXPECT_GT(x[BallinT700::kEngineStates + BallinT700::kSlip], 0.0);
    // Engine 0 runs on as it was (the shaft held, its sensed torque only through
    // the load share's limited, lagged trim).
    EXPECT_NEAR(x[BallinT700::kNg], start[BallinT700::kNg], 0.02);
    EXPECT_FALSE(h.signals_at(x, 0, 0.45, {1.0, 1.0}).out);
}

// An out engine (D-056 item 3) has no gas path: no flow, no torque, inlet
// pressure and temperature throughout, the solve not run (converged). Every
// state but the clutch's slip is held (the fuel control has nothing to
// control), and the clutch is open even at zero slip: the engine's output
// inertia stays off the shaft, its slip follows the shaft (ṡ = Ω̇). Just
// above the floor, or with the fuel on at the floor, it isn't out.
TEST(BallinT700, OutEngineHasNoGasPath) {
    Harness h;
    const BallinT700Config& c = h.engine().config();
    std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    const std::size_t o = BallinT700::kEngineStates;
    const double t2 = isa_atmosphere(0.0, 0.0).temperature;
    x[o + BallinT700::kNg] = h.engine().ng_floor(t2);
    h.set_fuel({true, false});
    const SimContext ctx = h.context(Harness::kGoverned, 0.45, 0.0);
    const BallinT700Signals s = h.signals_at(x, 1, 0.45, {1.0, 1.0});
    EXPECT_TRUE(s.out);
    EXPECT_TRUE(s.gas.converged);
    EXPECT_EQ(s.gas.w41, 0.0);
    EXPECT_EQ(s.gas.w45, 0.0);
    EXPECT_EQ(s.gas.q_pt, 0.0);
    EXPECT_EQ(s.q_engine, 0.0);
    EXPECT_EQ(s.gas.p3, ctx.air.pressure);
    EXPECT_EQ(s.gas.t45, t2);
    x[o + BallinT700::kSlip] = 0.0;
    // The drive decelerating (the rotor's demand three times engine 0's torque),
    // where the ordinary rule would engage a zero-torque engine at zero slip.
    const SimContext slowing =
        h.context(Harness::kGoverned, 0.45, 3.0 * h.signals(x, ctx).q_engine);
    std::vector<double> xdot(h.size(), 0.0);
    std::vector<double> g(6 * h.size(), 0.0);
    std::vector<double> gs(h.size() * kMaxShafts, 0.0);
    ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
    d.Gs = gs;
    {
        const test::AllocationCounter counter;  // the out path allocates nothing either (§7)
        h.engine().derivatives(slowing, std::span<const double>(x.data(), h.size()), d);
        EXPECT_EQ(counter.count(), 0U);
    }
    // Held: every state but the slip and the torque sensor, which reads the
    // engine's zero torque through its lags (the other engine's load share
    // reads it); the gas generator doesn't follow the airframe's rotation
    // either, so no airframe motion lifts it off the floor (code review of
    // PR #77).
    for (std::size_t i = 0; i < BallinT700::kEngineStates; ++i) {
        if (i != BallinT700::kSlip && i != BallinT700::kTorqueLag &&
            i != BallinT700::kTorqueSensed) {
            EXPECT_EQ(xdot[o + i], 0.0) << "state " << i;
        }
        for (std::size_t j = 0; j < 6; ++j) {
            if (i != BallinT700::kSlip) {
                EXPECT_EQ(g[(o + i) * 6 + j], 0.0) << "state " << i << ", a" << j;
            }
        }
    }
    EXPECT_DOUBLE_EQ(xdot[o + BallinT700::kTorqueLag],
                     -x[o + BallinT700::kTorqueLag] / c.torque_lag1);
    EXPECT_DOUBLE_EQ(
        xdot[o + BallinT700::kTorqueSensed],
        (x[o + BallinT700::kTorqueLag] - x[o + BallinT700::kTorqueSensed]) / c.torque_lag2);
    EXPECT_NE(x[o + BallinT700::kTorqueLag], 0.0);  // the 80 kt torque, decaying
    // Free of the airframe's rotation: the live gas generator, and the out
    // engine's power turbine behind its open clutch; not the out gas generator.
    EXPECT_DOUBLE_EQ(d.H(3, 3), c.j_gt * c.gg_axis_b.x() * c.gg_axis_b.x() +
                                    c.output_inertia * c.pt_axis_b.x() * c.pt_axis_b.x());
    // Linearization holds the out engine but its torque sensor.
    std::vector<unsigned char> pinned(h.size());
    h.engine().pinned_states(slowing, std::span<const double>(x.data(), h.size()), pinned);
    for (std::size_t i = 0; i < BallinT700::kEngineStates; ++i) {
        const bool sensor = i == BallinT700::kTorqueLag || i == BallinT700::kTorqueSensed;
        EXPECT_EQ(pinned[o + i], sensor ? 0 : 1) << "state " << i;
    }
    EXPECT_EQ(xdot[o + BallinT700::kSlip], 0.0);  // no torque: ṡ = Ω̇ only
    EXPECT_EQ(gs[(o + BallinT700::kSlip) * kMaxShafts], 1.0);
    const double je = c.output_inertia * c.gear * c.gear;
    EXPECT_NEAR(d.Ks(0, 0), -je, 1e-9 * je);  // engine 0's inertia alone
    for (const double v : xdot) {
        EXPECT_TRUE(std::isfinite(v));
    }
    // Just above the floor: Ballin's gas path, running down.
    std::vector<double> above = x;
    above[o + BallinT700::kNg] += 1e-4;
    EXPECT_FALSE(h.signals_at(above, 1, 0.45, {1.0, 1.0}).out);
    // At the floor with the fuel on: lit.
    h.set_fuel({true, true});
    EXPECT_FALSE(h.signals_at(x, 1, 0.45, {1.0, 1.0}).out);
}

// Trim leaves an out engine as it is (D-056 item 3): steady_states writes
// only a lit engine's held states, so a trim's collective doesn't move an
// out engine's load-demand spindle or torque motor (code review of PR #77).
TEST(BallinT700, SteadyStatesLeaveAnOutEngine) {
    Harness h;
    const BallinT700Config& c = h.engine().config();
    h.set_fuel({true, false});
    std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    x.pop_back();
    const std::vector<double> before = x;
    h.engine().steady_states(h.context(Harness::kGoverned, 0.7, 0.0), x);
    for (std::size_t i = 0; i < BallinT700::kEngineStates; ++i) {
        EXPECT_EQ(x[BallinT700::kEngineStates + i], before[BallinT700::kEngineStates + i])
            << "state " << i;
    }
    EXPECT_NE(x[BallinT700::kLdsHys], before[BallinT700::kLdsHys]);  // the lit engine's moves
}

// The trim seed in a descent the rotor drives (D-056 item 5): the lit engines
// run free at the reference speed only when the shaft turns faster than it;
// at a slower shaft they are seeded engaged, at the shaft's speed, with no
// speed error between their state and the speed they turn at (code review
// of PR #77).
TEST(BallinT700, OverrunSeedTurnsAtTheShaftBelowTheReference) {
    const Harness h;
    const BallinT700Config& c = h.engine().config();
    const double load = -0.5 * c.engines * c.gear * 138.9 * kFtLbf;
    for (const double ratio : {0.97, 1.03}) {
        SCOPED_TRACE(ratio);
        const double omega = ratio * Harness::kGoverned;
        const SimContext ctx = h.context(omega, 0.2, load);
        std::vector<double> x(h.size());
        h.engine().trim_seed(ctx, x);
        const double free = h.engine().np_reference() * c.np_des / c.gear;
        for (std::size_t e = 0; e < 2; ++e) {
            const std::size_t o = e * BallinT700::kEngineStates;
            const double turns = omega - x[o + BallinT700::kSlip];
            EXPECT_NEAR(turns, omega > free ? free : omega, 1e-12 * omega);
            EXPECT_NEAR(x[o + BallinT700::kNpSensor], c.gear * turns / c.np_des, 1e-12);
        }
    }
}

// The fuel back on at the floor relights at once (D-056 item 4; no starter):
// the idle schedule and the acceleration limit take the gas generator back
// through idle to flight power within 10 s, the shaft held at its speed.
TEST(BallinT700, FuelBackOnRelights) {
    Harness h;
    const BallinT700Config& c = h.engine().config();
    std::vector<double> x = h.seeded(0.45, c.engines * c.gear * 138.9 * kFtLbf);
    const double ng_flight = x[BallinT700::kEngineStates + BallinT700::kNg];
    h.set_fuel({true, false});
    for (int n = 0; n < 10000; ++n) {
        h.step(x, 0.001, 0.45, {1.0, 1.0}, 0.0, true);
    }
    ASSERT_TRUE(h.signals_at(x, 1, 0.45, {1.0, 1.0}).out);
    h.set_fuel({true, true});
    for (int n = 0; n < 10000; ++n) {
        h.step(x, 0.001, 0.45, {1.0, 1.0}, 0.0, true);
    }
    const BallinT700Signals s = h.signals_at(x, 1, 0.45, {1.0, 1.0});
    EXPECT_FALSE(s.out);
    EXPECT_GT(x[BallinT700::kEngineStates + BallinT700::kNg], 0.9 * ng_flight);
    EXPECT_GT(s.gas.q_pt, 0.0);
}

// One engine shut down at 80 kt power on the UH-60A's drive (D-056): the rotor
// droops, the live engine takes the whole load and its governor brings the
// rotor back to the reference (isochronous), the dead engine disengaged.
TEST(BallinT700, OneEngineCarriesTheLoad) {
    Harness h;
    const BallinT700Config& c = h.engine().config();
    const double demand =
        c.engines * c.gear * 138.9 * kFtLbf - h.engine().drive_train_load(Harness::kGoverned);
    std::vector<double> x = h.seeded(0.45, demand);
    h.set_fuel({true, false});
    double low = Harness::kGoverned;
    for (int n = 0; n < 40000; ++n) {
        h.step(x, 0.001, 0.45, {1.0, 1.0}, demand, false);
        low = std::min(low, x.back());
    }
    EXPECT_LT(low, 0.995 * Harness::kGoverned);
    EXPECT_GT(low, 0.95 * Harness::kGoverned);
    EXPECT_NEAR(x.back(), Harness::kGoverned, 1e-3 * Harness::kGoverned);
    const BallinT700Signals live = h.signals_at(x, 0, 0.45, {1.0, 1.0});
    EXPECT_NEAR(live.q_engine, demand + h.engine().drive_train_load(x.back()), 0.01 * demand);
    EXPECT_TRUE(h.signals_at(x, 1, 0.45, {1.0, 1.0}).out);
    EXPECT_GT(x[BallinT700::kEngineStates + BallinT700::kSlip], 0.0);
}

// The clutches' drive inertia J_d (config, D-055 item 5) is the shaft the
// executive integrates: the parts no component models and every component's
// own inertia on it (the blades; K_s at the initial state, blades unflapped),
// the engines' excepted. Each of the UH-60A's Ballin's-engine configs (code
// review of PR #73: two copies of one quantity, held equal here).
TEST(BallinT700Aircraft, DriveInertiaIsTheShafts) {
    for (const char* file :
         {"uh60a.yaml", "uh60a_table_b1.yaml", "uh60a_ballin.yaml", "uh60a_fletcher.yaml"}) {
        SCOPED_TRACE(file);
        const Config cfg =
            Config::from_file(std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/engine_ballin/" + file);
        const Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);
        ASSERT_EQ(ex.shafts().size(), 1U);
        double j = 0.0;
        for (const ShaftPart& p : ex.shafts()[0].parts) {
            j += p.inertia;
        }
        // The coupling store at this state first (Executive::context).
        std::vector<double> rates(ex.num_states());
        (void)ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, rates);
        const SimContext ctx = ex.context(0.0, ex.state(), Controls{}, Environment{});
        const BallinT700* engine = nullptr;
        for (const std::size_t i : ex.evaluation_order()) {  // producers first (D-003)
            const DynamicComponent& c = ex.component(i);
            if (const auto* e = dynamic_cast<const BallinT700*>(&c)) {
                engine = e;
                continue;
            }
            const std::size_t n = ex.component_size(i);
            std::vector<double> xdot(n);
            std::vector<double> g(6 * n);
            std::vector<double> gs(n * kMaxShafts);
            ComponentDerivs d{xdot, g, Wrench{}, Mat66::Zero()};
            d.Gs = gs;
            c.derivatives(ctx, ex.state().subspan(ex.component_offset(i), n), d);
            j -= d.Ks(0, 0);
        }
        ASSERT_NE(engine, nullptr);
        EXPECT_NEAR(engine->config().drive_inertia, j, 1e-6 * j);
    }
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

// The cockpit's engine-speed trim (D-055 item 1): the UH-60A on Ballin's
// engines trims at 80 kt with the reference at 0.97 and 1.02 of the governed
// speed, the governor isochronous there, the revolution's period following
// the rotor (D-053 amendment 2), and the trim reports the reference in its
// controls. The trimmed state is the revolution's start, ψ = 0, on the
// shaft's 4/rev speed ripple from the rotor's torque (about 1e-5 of the speed;
// there the NP sensor reads 8e-6 above the reference while the P+I
// integrator's periodicity closes to 2e-9 V): the bound is 2e-5, against a
// reference ignored (2-3e-2).
TEST(BallinT700Aircraft, TrimsAtTheSpeedReference) {
    const Config cfg = Config::from_file(std::string(EYAS_VEHICLE_DATA_DIR) +
                                         "/uh60a/engine_ballin/uh60a_table_b1.yaml");
    const Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);
    const std::size_t omega_at = ex.shaft_offset(0) + 1 - body_layout::kSize;
    const double governed = ex.shafts()[0].speed;
    std::vector<double> x;
    for (const double ref : {0.97, 1.02}) {
        SCOPED_TRACE(ref);
        TrimCondition c;
        c.airspeed = 80.0 * units::kKnot;
        c.rotor_speed_reference = ref;
        const TrimResult r = solve_trim(ex, c, nullptr, x);
        ASSERT_TRUE(r.converged);
        EXPECT_EQ(r.controls.rotor_speed_reference, ref);
        const double omega = r.component_states[omega_at];
        EXPECT_NEAR(omega, ref * governed, 2e-5 * governed);
        EXPECT_NEAR(omega * r.period / (2.0 * std::numbers::pi), 1.0, 2e-5);
    }
}

// The UH-60A on Ballin's engines at Table B.1's loading, with what a test
// reads off a trim: each engine's signals at the trimmed state (D-056).
class Uh60aOnBallinEngines {
public:
    Uh60aOnBallinEngines() : ex_(build_executive(cfg(), read_origin(cfg()).origin().altitude)) {
        for (std::size_t i = 0; i < ex_.num_components(); ++i) {
            if (const auto* e = dynamic_cast<const BallinT700*>(&ex_.component(i))) {
                engine_ = e;
                offset_ = ex_.component_offset(i);
            }
        }
    }
    const Executive& ex() const { return ex_; }
    const BallinT700& engine() const { return *engine_; }
    double governed() const { return ex_.shafts()[0].speed; }
    double omega(const TrimResult& r) const {
        return r.component_states[ex_.shaft_offset(0) + 1 - body_layout::kSize];
    }
    // Engine e's state in the trim's state x and its signals there.
    std::span<const double> states(const std::vector<double>& x, std::size_t e) const {
        return std::span<const double>(x).subspan(offset_ + e * BallinT700::kEngineStates,
                                                  BallinT700::kEngineStates);
    }
    BallinT700Signals signals(const TrimCondition& c, const TrimResult& r,
                              const std::vector<double>& x, std::size_t e) const {
        const SimContext ctx = ex_.context(0.0, x, r.controls, trim_environment(c));
        return engine_->signals(states(x, e), engine_->inputs(ctx, static_cast<int>(e)));
    }
    TrimResult trim(const TrimCondition& c, const TrimResult* start, std::vector<double>& x) const {
        return solve_trim(ex_, c, start, x);
    }

private:
    static const Config& cfg() {
        static const Config c = Config::from_file(std::string(EYAS_VEHICLE_DATA_DIR) +
                                                  "/uh60a/engine_ballin/uh60a_table_b1.yaml");
        return c;
    }
    Executive ex_;
    const BallinT700* engine_ = nullptr;
    std::size_t offset_ = 0;
};

// One engine out at 80 kt (D-056): the trim converges with the dead engine
// out on the maps' floor, its clutch open, and the live engine alone
// delivers what the two did, since the rotor's demand is the same: its
// torque at the shaft equals the pair's sum, to the trims' tolerance. The
// governor holds the reference.
TEST(BallinT700Aircraft, TrimsWithOneEngineOut) {
    const Uh60aOnBallinEngines a;
    std::vector<double> x;
    TrimCondition c;
    c.airspeed = 80.0 * units::kKnot;
    const TrimResult both = a.trim(c, nullptr, x);
    ASSERT_TRUE(both.converged);
    const double pair = a.signals(c, both, x, 0).q_engine + a.signals(c, both, x, 1).q_engine;
    const double ng_both = a.states(x, 0)[BallinT700::kNg];
    c.engines[1].fuel_on = false;
    const TrimResult one = a.trim(c, nullptr, x);
    ASSERT_TRUE(one.converged);
    EXPECT_LE(one.iterations, 15);
    EXPECT_EQ(one.controls.engines[1].fuel_on, false);
    const BallinT700Signals live = a.signals(c, one, x, 0);
    const BallinT700Signals dead = a.signals(c, one, x, 1);
    EXPECT_FALSE(live.out);
    EXPECT_TRUE(dead.out);
    EXPECT_EQ(dead.q_engine, 0.0);
    EXPECT_NEAR(a.states(x, 1)[BallinT700::kNg],
                a.engine().ng_floor(isa_atmosphere(0.0, 0.0).temperature), 1e-9);
    EXPECT_NEAR(live.q_engine, pair, 1e-4 * pair);
    EXPECT_GT(a.states(x, 0)[BallinT700::kNg], ng_both + 0.02);  // the live engine works harder
    EXPECT_NEAR(a.omega(one), a.governed(), 1e-6 * a.governed());
}

// A descent the rotor autorotates in with the engines lit (D-056 item 5): at
// 100 kt, continued from level flight, the clutches let go between −2750
// and −3000 ft/min, and the rotor turns faster than the governed speed,
// while the engines run free at no load, governed at the reference
// (isochronous: their NP sensors at it). The rotor and airframe then trim
// exactly as with both engines out, since free-running lit engines and dead
// ones put nothing on the shaft: the collective, attitude and rotor speed
// agree to the trims' tolerance.
TEST(BallinT700Aircraft, OverrunsInASteepDescentAsInAutorotation) {
    const Uh60aOnBallinEngines a;
    std::vector<double> x;
    TrimCondition c;
    c.airspeed = 100.0 * units::kKnot;
    TrimResult r = a.trim(c, nullptr, x);
    ASSERT_TRUE(r.converged);
    for (const double fpm : {-1500.0, -2500.0, -2750.0, -3000.0, -3250.0}) {
        SCOPED_TRACE(fpm);
        c.flight_path_angle = std::asin(fpm * units::kFoot / 60.0 / c.airspeed);
        const TrimResult next = a.trim(c, &r, x);
        ASSERT_TRUE(next.converged);
        r = next;
        const double slip = a.states(x, 0)[BallinT700::kSlip];
        if (fpm >= -2750.0) {
            EXPECT_EQ(slip, 0.0);
            // The start of the revolution is on the shaft's 4/rev ripple, about
            // 1e-5 of the speed (TrimsAtTheSpeedReference).
            EXPECT_NEAR(a.omega(r), a.governed(), 2e-5 * a.governed());
        } else {
            EXPECT_GT(slip, 0.0);
            EXPECT_GT(a.omega(r), 1.01 * a.governed());
        }
    }
    for (std::size_t e = 0; e < 2; ++e) {
        const BallinT700Signals s = a.signals(c, r, x, e);
        EXPECT_FALSE(s.out);
        EXPECT_NEAR(s.q_engine, 0.0, 1e-6 * 1000.0);
        EXPECT_NEAR(a.states(x, e)[BallinT700::kNpSensor], a.engine().np_reference(), 1e-5);
    }
    TrimCondition off = c;
    off.engines[0].fuel_on = off.engines[1].fuel_on = false;
    std::vector<double> y;
    const TrimResult dead = a.trim(off, &r, y);
    ASSERT_TRUE(dead.converged);
    EXPECT_NEAR(dead.controls.collective, r.controls.collective, 1e-6);
    EXPECT_NEAR(dead.pitch, r.pitch, 1e-6);
    EXPECT_NEAR(dead.roll, r.roll, 1e-6);
    EXPECT_NEAR(a.omega(dead), a.omega(r), 1e-6 * a.governed());
}

}  // namespace
}  // namespace eyas
