// M9: Ballin's component-level T700 (NASA TM-100991; D-054). The gas path
// against Ballin's printed steady states (Table B.1), the curve-family
// lookup, and the config.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "engine/ballin_t700.hpp"
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
        ASSERT_TRUE(engine->steady_gas(r.fuel_lbm_hr * kLbmHr, np, 14.696 * kPsi,
                                       518.67 * kRankine, ng, g));
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
        const auto qpt = [&](double n, double w) { return engine->gas(n, np, -1.0, w, p2, t2).q_pt; };
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

}  // namespace
}  // namespace eyas
