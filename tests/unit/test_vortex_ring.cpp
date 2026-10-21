// M8d vortex-ring-state correction (D-049; SPEC_REVIEW #21): Johnson,
// NASA/TP-2005-213477, Tables 3 and 4. Layer 1, the production curves
// against an independent transcription (tests/support/vrs_reference.hpp);
// layer 2, the properties Johnson builds the model on (the specified
// inflow peak and dip with zero slope on the stability boundary, the joins,
// momentum theory outside the region) and the ones the integration relies
// on (unique thrust inversion); the rotor with each inflow model; the heave
// damping sign change at the boundary; and the real-time rules.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>
#include <vector>

#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/vortex_ring.hpp"
#include "sim/build.hpp"
#include "support/alloc_counter.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/vrs_reference.hpp"
#include "util/config.hpp"

namespace eyas {
namespace {

using P = JohnsonVrsParameters;
using test::airfoil_from;
using test::sea_level_context;
using test::synthetic_airfoil_csv;
using test::test_rotor;

constexpr double kDeg = std::numbers::pi / 180.0;

// Johnson's stability boundary (Table 3 step 3.1): the minimum N and the
// maximum X of the inflow curve at in-plane speed vx < v_xM.
double boundary_n(double vx) {
    const double s = 1.0 - (vx / P::vx_m) * (vx / P::vx_m);
    return 0.5 * (P::vz_n + P::vz_x) + 0.5 * (P::vz_n - P::vz_x) * std::pow(s, 0.2);
}
double boundary_x(double vx) {
    const double s = 1.0 - (vx / P::vx_m) * (vx / P::vx_m);
    return 0.5 * (P::vz_n + P::vz_x) - 0.5 * (P::vz_n - P::vz_x) * std::pow(s, 1.5);
}

// ---- Momentum roots --------------------------------------------------------------------

// Axial flow has closed forms (Johnson TP-2005 p. 2): the normal branch
// v = −vz/2 + √(vz²/4 + 1) and, for vz < −2, the windmill-brake branch
// v = −vz/2 − √(vz²/4 − 1). Edgewise flow: the companion-matrix roots.
TEST(VortexRing, MomentumRootsMatchClosedFormsAndTheReference) {
    for (double vz = -5.0; vz <= 3.0; vz += 0.0625) {
        EXPECT_NEAR(momentum_normal(0.0, vz).v, -0.5 * vz + std::sqrt(0.25 * vz * vz + 1.0), 1e-13)
            << vz;
        if (vz < -2.0) {
            EXPECT_NEAR(momentum_windmill(0.0, vz).v, -0.5 * vz - std::sqrt(0.25 * vz * vz - 1.0),
                        1e-12)
                << vz;
        }
    }
    for (double vx = 0.0; vx <= 3.0; vx += 0.05) {
        // Offset so no point lands on a double root (vx = 0, vz = −2), where
        // any solver resolves the root only to about √ε.
        for (double vz = -6.0123; vz <= 2.0; vz += 0.05) {
            const auto n = momentum_normal(vx, vz);
            const auto w = momentum_windmill(vx, vz);
            const auto rn = test::ref_normal(vx, vz);
            const auto rw = test::ref_windmill(vx, vz);
            EXPECT_NEAR(n.v, rn.v, 1e-12 * rn.v) << vx << ", " << vz;
            EXPECT_NEAR(w.v, rw.v, 1e-12 * rw.v) << vx << ", " << vz;
            EXPECT_NEAR(n.dv_dvz, rn.dv, 1e-9 * (1.0 + std::abs(rn.dv))) << vx << ", " << vz;
        }
    }
}

// ---- Layer 1: production against the independent transcription -----------------------

TEST(VortexRing, CurvesMatchJohnsonTable3Transcription) {
    for (double vx = 0.0; vx <= 1.3; vx += 0.01) {
        for (double vz = -3.5; vz <= 0.5; vz += 0.01) {
            const auto ref_b = test::ref_baseline(vx, vz);
            const auto ref_v = test::ref_vrs(vx, vz);
            const auto b = johnson_inflow(vx, vz, VrsCurve::baseline);
            const auto v = johnson_inflow(vx, vz, VrsCurve::vortex_ring);
            EXPECT_NEAR(b.v, ref_b.v, 1e-11) << vx << ", " << vz;
            EXPECT_NEAR(v.v, ref_v.v, 1e-11) << vx << ", " << vz;
            EXPECT_NEAR(v.dv_dvz, ref_v.dv, 1e-8 * (1.0 + std::abs(ref_v.dv))) << vx << ", " << vz;
        }
    }
}

// ---- Layer 2: the model's defining properties ---------------------------------------

// Table 4: in axial flow the total inflow vz + v has its minimum 0.85 at
// vz = −0.45 and its maximum 1.25 at vz = −1.5, with zero slope at both
// (the stability boundary). At each vx < v_xM the slope is zero at the
// boundary points N(vx) and X(vx).
TEST(VortexRing, InflowHasZeroSlopeOnTheStabilityBoundary) {
    const auto n = johnson_inflow(0.0, P::vz_n, VrsCurve::vortex_ring);
    const auto x = johnson_inflow(0.0, P::vz_x, VrsCurve::vortex_ring);
    EXPECT_NEAR(P::vz_n + n.v, P::total_n, 1e-13);
    EXPECT_NEAR(P::vz_x + x.v, P::total_x, 1e-13);
    for (double vx = 0.0; vx <= P::vx_m - kVortexRingFadeWidth + 1e-12; vx += 0.05) {
        for (double vz : {boundary_n(vx), boundary_x(vx)}) {
            const auto c = johnson_inflow(vx, vz, VrsCurve::vortex_ring);
            EXPECT_NEAR(1.0 + c.dv_dvz, 0.0, 1e-10) << vx << ", " << vz;
            // And the slope is the curve's own (central difference).
            const double h = 1e-6;
            const double fd = (johnson_inflow(vx, vz + h, VrsCurve::vortex_ring).v -
                               johnson_inflow(vx, vz - h, VrsCurve::vortex_ring).v) /
                              (2.0 * h);
            // 1e-5: at vx = 0, X = A, where the curvature jumps.
            EXPECT_NEAR(c.dv_dvz, fd, 1e-5) << vx << ", " << vz;
        }
    }
    // Between N and X the total inflow falls as the descent grows: the
    // unstable band (d(vz + v)/dvz < 0).
    for (double vz = -1.45; vz < -0.5; vz += 0.05) {
        EXPECT_LT(1.0 + johnson_inflow(0.0, vz, VrsCurve::vortex_ring).dv_dvz, 0.0) << vz;
    }
    // The baseline has no such band (Johnson Fig. 52: stable in VRS).
    for (double vz = -2.5; vz < 0.0; vz += 0.01) {
        EXPECT_GT(1.0 + johnson_inflow(0.0, vz, VrsCurve::baseline).dv_dvz, 0.0) << vz;
    }
}

// The joins: value and slope continuous at A (baseline) and D (VRS), value
// continuous at B and E. Momentum theory outside the region, to the bit.
TEST(VortexRing, CurvesJoinMomentumTheoryWhereJohnsonJoinsThem) {
    const double h = 1e-9;
    for (double vx = 0.0; vx < P::vx_c; vx += 0.05) {
        const double r = vx / P::vx_c;
        const double a = P::vz_a + 0.2 * r * r;
        double b = P::vz_b + 0.2 * r * r;
        if (r > 0.5) {
            b += 0.7 * (a - b) * std::pow(2.0 * r - 1.0, 3);
        }
        const auto am = johnson_inflow(vx, a - h, VrsCurve::baseline);
        const auto ap = johnson_inflow(vx, a + h, VrsCurve::baseline);
        EXPECT_NEAR(am.v, ap.v, 1e-8) << vx;
        EXPECT_NEAR(am.dv_dvz, ap.dv_dvz, 1e-6) << vx;
        EXPECT_NEAR(johnson_inflow(vx, b - h, VrsCurve::baseline).v,
                    johnson_inflow(vx, b + h, VrsCurve::baseline).v, 1e-7)
            << vx;
    }
    for (double vx = 0.0; vx < P::vx_m; vx += 0.05) {
        const double e = P::vz_e + (boundary_x(vx) - P::vz_x);
        for (double z : {P::vz_d, e}) {
            const auto m = johnson_inflow(vx, z - h, VrsCurve::vortex_ring);
            const auto p = johnson_inflow(vx, z + h, VrsCurve::vortex_ring);
            EXPECT_NEAR(m.v, p.v, 1e-7) << vx << ", " << z;
        }
        const auto dm = johnson_inflow(vx, P::vz_d - h, VrsCurve::vortex_ring);
        const auto dp = johnson_inflow(vx, P::vz_d + h, VrsCurve::vortex_ring);
        EXPECT_NEAR(dm.dv_dvz, dp.dv_dvz, 1e-6) << vx;
    }
    for (double vx : {0.0, 0.5, 0.95, 2.0}) {
        for (double vz : {-0.2, -0.1, 0.0, 0.7}) {
            const auto m = momentum_normal(vx, vz);
            EXPECT_EQ(johnson_inflow(vx, vz, VrsCurve::vortex_ring).v, m.v);
            EXPECT_EQ(johnson_inflow(vx, vz, VrsCurve::baseline).v, m.v);
        }
    }
}

// Johnson's model jumps at vx = v_xC and v_xM; Eyas fades each departure
// out below its edge (D-049), so the curves are continuous in vx, and the
// stability boundary is Johnson's for vx ≤ v_xM − width.
TEST(VortexRing, FadesMakeTheCurvesContinuousInEdgewiseSpeed) {
    for (VrsCurve curve : {VrsCurve::baseline, VrsCurve::vortex_ring}) {
        for (double edge : {P::vx_c, P::vx_m}) {
            for (double vz = -2.4; vz < -0.2; vz += 0.01) {
                const double below = johnson_inflow(edge * (1.0 - 1e-12), vz, curve).v;
                const double at = johnson_inflow(edge, vz, curve).v;
                EXPECT_NEAR(below, at, 1e-9) << edge << ", " << vz;
            }
        }
    }
    // Unfaded, the published model's jumps: compare the reference without
    // the fade.
    test::VrsRefParams johnson;
    johnson.fade = 1e-12;
    double jump_m = 0.0;
    double jump_c = 0.0;
    for (double vz = -2.4; vz < -0.2; vz += 0.01) {
        jump_m = std::max(jump_m, std::abs(test::ref_vrs(P::vx_m - 1e-9, vz, johnson).v -
                                           test::ref_vrs(P::vx_m, vz, johnson).v));
        jump_c = std::max(jump_c, std::abs(test::ref_baseline(P::vx_c - 1e-9, vz, johnson).v -
                                           test::ref_baseline(P::vx_c, vz, johnson).v));
    }
    EXPECT_GT(jump_m, 0.10);
    EXPECT_LT(jump_m, 0.16);
    EXPECT_GT(jump_c, 0.002);
    EXPECT_LT(jump_c, 0.005);
}

// ---- Layer 2: the thrust inversion -----------------------------------------------------

// λ0 rises strictly with λ_h at fixed (μ, μ_z) on both curves: with the
// homogeneity λ0 = λ_h f(μ/λ_h, vz/λ_h), dλ0/dλ_h = f − x f_x − z f_z.
// This is what makes the inversion unique and the inflow equations' steady
// state single-valued (D-049).
TEST(VortexRing, InducedVelocityRisesWithThrust) {
    const double h = 1e-6;
    for (VrsCurve curve : {VrsCurve::baseline, VrsCurve::vortex_ring}) {
        double worst = 1e9;
        std::vector<double> xs;
        for (double x = 0.0; x <= 1.2; x += 0.02) {
            xs.push_back(x);
        }
        for (double x = 0.69; x <= 0.96; x += 0.0025) {
            xs.push_back(x);  // the fade bands, finer
        }
        for (double x : xs) {
            for (double z = -3.0; z < -0.01; z += 0.01) {
                const double f = johnson_inflow(x, z, curve).v;
                const double xl = std::max(0.0, x - h);
                const double fx =
                    (johnson_inflow(x + h, z, curve).v - johnson_inflow(xl, z, curve).v) /
                    (x + h - xl);
                const double fz = johnson_inflow(x, z, curve).dv_dvz;
                worst = std::min(worst, f - x * fx - z * fz);
            }
        }
        EXPECT_GT(worst, 0.4) << static_cast<int>(curve);
    }
}

// Round trip: λ0 on each curve at a known λ_h, inverted, gives the same
// thrust; negative thrust mirrors. Outside the region nothing is solved and
// the thrust is the momentum expression exactly.
TEST(VortexRing, ThrustInversionRecoversTheCurveThrust) {
    std::mt19937 rng(20261020);
    std::uniform_real_distribution<double> ux(0.0, 1.1);
    std::uniform_real_distribution<double> uz(-3.0, 0.1);
    std::uniform_real_distribution<double> ul(0.02, 0.12);
    int active = 0;
    for (int i = 0; i < 2000; ++i) {
        const double lh = ul(rng);
        const double x = ux(rng);
        const double z = uz(rng);
        const double mu = x * lh;
        const double mu_z = -z * lh;
        for (double sign : {1.0, -1.0}) {
            const double l_vrs = sign * lh * johnson_inflow(x, z, VrsCurve::vortex_ring).v;
            const VortexRingThrust t = vortex_ring_thrust(l_vrs, mu, sign * mu_z);
            EXPECT_TRUE(t.converged);
            EXPECT_LE(t.iterations, 2 * kVortexRingMaxIterations);
            EXPECT_NEAR(t.ct_vrs, sign * 2.0 * lh * lh, 1e-12 * lh * lh) << x << ", " << z;
            const double l_base = sign * lh * johnson_inflow(x, z, VrsCurve::baseline).v;
            const VortexRingThrust tb = vortex_ring_thrust(l_base, mu, sign * mu_z);
            EXPECT_NEAR(tb.ct_base, sign * 2.0 * lh * lh, 1e-12 * lh * lh) << x << ", " << z;
            active += t.active ? 1 : 0;
        }
    }
    EXPECT_GT(active, 1000);
    // Momentum region: hover, climb, fast forward flight.
    for (const auto& [l0, mu, mu_z] :
         {std::tuple{0.07, 0.0, 0.0}, std::tuple{0.05, 0.0, -0.02}, std::tuple{0.02, 0.3, 0.01},
          std::tuple{-0.03, 0.05, 0.0}, std::tuple{0.0, 0.1, 0.05}}) {
        const VortexRingThrust t = vortex_ring_thrust(l0, mu, mu_z);
        const double d = l0 - mu_z;
        EXPECT_FALSE(t.active);
        EXPECT_EQ(t.ct_vrs, 2.0 * l0 * std::sqrt(mu * mu + d * d));
        EXPECT_EQ(t.ct_base, t.ct_vrs);
        EXPECT_EQ(t.iterations, 0);
    }
}

// Bounded everywhere, including the extremes (tiny λ0, fast descent, high μ).
TEST(VortexRing, InversionIsBoundedAndFinite) {
    for (double l0 : {1e-9, 1e-5, 0.01, 0.08, 0.3}) {
        for (double mu : {0.0, 1e-4, 0.01, 0.05, 0.5}) {
            for (double mu_z : {-0.5, -0.01, 0.0, 0.01, 0.03, 0.06, 0.1, 0.2, 1.0, 5.0}) {
                const VortexRingThrust t = vortex_ring_thrust(l0, mu, mu_z);
                EXPECT_TRUE(t.converged) << l0 << ", " << mu << ", " << mu_z;
                EXPECT_TRUE(std::isfinite(t.ct_vrs) && std::isfinite(t.ct_base));
                EXPECT_GT(t.ct_vrs, 0.0);
            }
        }
    }
}

// ---- The rotor ---------------------------------------------------------------------------

SwashplatePitch pitch(double theta0_deg) {
    SwashplatePitch p;
    p.theta0 = theta0_deg * kDeg;
    return p;
}

BladeElementRotorConfig vrs_rotor(InflowModel inflow, VortexRingModel vrs, double theta0_deg,
                                  bool stall = true) {
    // The synthetic airfoil stalls; the linear one (a = 5.73, c_d0 = 0.01)
    // doesn't.
    BladeElementRotorConfig c = test_rotor(
        airfoil_from(stall ? synthetic_airfoil_csv() : test::linear_airfoil_csv(5.73, 0.01)),
        pitch(theta0_deg), 20);
    c.inflow = inflow;
    c.vortex_ring = vrs;
    return c;
}

// The uniform solve in axial descent at fixed collective: with the
// correction it lands on Johnson's curve, continuously: refining the sweep
// 4× shrinks the largest thrust step about 4× (the curve is steep, not
// broken, near ideal autorotation). Linear airfoil: past stall (about 40 m/s
// here with the synthetic airfoil) the blades' thrust can rise with the
// inflow, which folds the solution whatever the inflow model. Deep in
// descent (vz < −2 v_h) the corrected rotor is in the windmill-brake state,
// with the flow up through the disc. Uncorrected, momentum theory stays on
// its normal-working branch until it jumps to the windmill branch at about
// 3 v_h (SPEC_REVIEW #21).
double largest_step(const BladeElementRotor& rotor, double dw, bool check_curve) {
    double prev_ct = 0.0;
    double worst = 0.0;
    const int n = static_cast<int>(std::lround(75.0 / dw));
    for (int i = 0; i <= n; ++i) {
        SimContext ctx = sea_level_context(0.0);
        ctx.body.vel_b = Vec3(0.0, 0.0, dw * i);  // descent 0–75 m/s, past 3 v_h
        const RotorLoads l = rotor.evaluate(ctx);
        if (check_curve) {
            EXPECT_TRUE(l.converged) << dw * i;
            EXPECT_LE(l.passes, BladeElementRotor::kMaxPasses);
            const VortexRingThrust t = vortex_ring_thrust(l.lambda_i, l.mu, l.mu_z);
            EXPECT_NEAR(t.ct_vrs, l.ct, 1e-11) << dw * i;
        }
        if (i > 0) {
            worst = std::max(worst, std::abs(l.ct - prev_ct));
        }
        prev_ct = l.ct;
    }
    return worst;
}

TEST(VortexRing, UniformSolveFollowsJohnsonsCurveWithoutJumps) {
    const BladeElementRotor rotor(
        vrs_rotor(InflowModel::uniform, VortexRingModel::johnson_2005, 22.0, false));
    const double coarse = largest_step(rotor, 0.1, true);
    const double fine = largest_step(rotor, 0.025, true);
    EXPECT_LT(fine, 0.4 * coarse);
    for (double w : {60.0, 75.0}) {
        SimContext ctx = sea_level_context(0.0);
        ctx.body.vel_b = Vec3(0.0, 0.0, w);
        const RotorLoads l = rotor.evaluate(ctx);
        ASSERT_LT(-l.mu_z / std::sqrt(0.5 * l.ct), -2.0) << w;
        EXPECT_LT(l.lambda_i - l.mu_z, 0.0) << w;  // windmill brake: up through the disc
    }
    // Uncorrected, the M3 root switch: a jump that refining doesn't shrink.
    const BladeElementRotor plain(
        vrs_rotor(InflowModel::uniform, VortexRingModel::none, 22.0, false));
    const double plain_coarse = largest_step(plain, 0.1, false);
    const double plain_fine = largest_step(plain, 0.025, false);
    EXPECT_GT(plain_fine, 0.9 * plain_coarse);
    EXPECT_GT(plain_fine, 5.0 * fine);
}

// Heave damping at fixed collective, uniform inflow, axial descent: with
// thrust from the blades a function of θ and the total inflow λ only,
// sign(dT/dw) = sign(d(vz + v)/dvz at fixed T), so it changes sign exactly
// where Johnson's curve has zero slope: vz/v_h = −0.45 and −1.5 (Johnson
// Fig. 52; Table 4). Found by bisection on dC_T/dw; v_h from the thrust
// there.
TEST(VortexRing, HeaveDampingChangesSignOnTheBoundary) {
    const BladeElementRotor rotor(
        vrs_rotor(InflowModel::uniform, VortexRingModel::johnson_2005, 22.0));
    const auto ct_at = [&](double w) {
        SimContext ctx = sea_level_context(0.0);
        ctx.body.vel_b = Vec3(0.0, 0.0, w);
        return rotor.evaluate(ctx).ct;
    };
    const auto slope = [&](double w) {
        const double h = 1e-4;
        return (ct_at(w + h) - ct_at(w - h)) / (2.0 * h);
    };
    const auto vz_over_vh = [&](double w) {
        const double lh = std::sqrt(0.5 * ct_at(w));
        return -(w / rotor.tip_speed()) / lh;
    };
    // Scan for sign changes of dC_T/dw over 0–30 m/s.
    std::vector<double> crossings;
    double w0 = 0.5;
    double s0 = slope(w0);
    for (double w = 1.0; w <= 30.0; w += 0.5) {
        const double s = slope(w);
        if ((s > 0.0) != (s0 > 0.0)) {
            double a = w0;
            double b = w;
            for (int k = 0; k < 50; ++k) {
                const double m = 0.5 * (a + b);
                ((slope(m) > 0.0) == (s0 > 0.0) ? a : b) = m;
            }
            crossings.push_back(vz_over_vh(0.5 * (a + b)));
        }
        w0 = w;
        s0 = s;
    }
    ASSERT_EQ(crossings.size(), 2U);
    EXPECT_NEAR(crossings[0], P::vz_n, 1e-4);
    EXPECT_NEAR(crossings[1], P::vz_x, 1e-4);
}

// Pitt–Peters with the correction: its steady state in axial descent is
// the uniform solve's (Johnson's curve), with D at its quasi-static value,
// and away from the region the rates equal the uncorrected rotor's to the
// bit.
TEST(VortexRing, PittPetersSteadyStateIsJohnsonsCurve) {
    const BladeElementRotor uni(
        vrs_rotor(InflowModel::uniform, VortexRingModel::johnson_2005, 22.0));
    const BladeElementRotor pp(
        vrs_rotor(InflowModel::pitt_peters, VortexRingModel::johnson_2005, 22.0));
    ASSERT_EQ(pp.num_states(), 4U);
    ASSERT_EQ(pp.vortex_ring_offset(), 3U);
    for (double w : {3.0, 6.0, 10.0, 14.0, 18.0, 22.0, 26.0}) {
        SimContext ctx = sea_level_context(0.0);
        ctx.body.vel_b = Vec3(0.0, 0.0, w);
        const RotorLoads lu = uni.evaluate(ctx);
        const VortexRingThrust t = vortex_ring_thrust(lu.lambda_i, lu.mu, lu.mu_z);
        std::vector<double> x = {lu.lambda_i, 0.0, 0.0, t.ct_vrs - t.ct_base};
        std::vector<double> xd(4, 0.0);
        std::vector<double> G(24, 0.0);
        ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
        pp.derivatives(ctx, x, d);
        EXPECT_TRUE(d.converged);
        // λ̇0 ~ Ω C_T / M̃₀ ≈ 0.4 /s per unit thrust error; the residual is the
        // uniform solve's tolerance.
        EXPECT_NEAR(xd[0], 0.0, 1e-9) << w;
        EXPECT_NEAR(xd[1], 0.0, 1e-12) << w;
        EXPECT_NEAR(xd[2], 0.0, 1e-12) << w;
        EXPECT_NEAR(xd[3], 0.0, 1e-14) << w;
    }
    // Outside the region (hover, climb, forward flight) with D = 0.
    const BladeElementRotor plain(vrs_rotor(InflowModel::pitt_peters, VortexRingModel::none, 22.0));
    for (const Vec3& v : {Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -8.0), Vec3(40.0, 3.0, 4.0)}) {
        SimContext ctx = sea_level_context(0.013);
        ctx.body.vel_b = v;
        std::vector<double> x = {0.06, 0.004, -0.007, 0.0};
        std::vector<double> xd(4, 0.0);
        std::vector<double> xd_plain(3, 0.0);
        std::vector<double> G(24, 0.0);
        ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
        ComponentDerivs dp{xd_plain, std::span<double>(G).first(18), Wrench{}, Mat66::Zero()};
        pp.derivatives(ctx, x, d);
        plain.derivatives(ctx, std::span<const double>(x).first(3), dp);
        for (std::size_t k = 0; k < 3; ++k) {
            EXPECT_EQ(xd[k], xd_plain[k]) << k;
        }
        EXPECT_EQ(xd[3], 0.0);
    }
}

// The deficit lags its quasi-static value with τ = 0.7/λ_h revolutions
// (Johnson TP-2005 p. 16).
TEST(VortexRing, DeficitLagsWithJohnsonsTimeConstant) {
    const BladeElementRotor pp(
        vrs_rotor(InflowModel::pitt_peters, VortexRingModel::johnson_2005, 22.0));
    SimContext ctx = sea_level_context(0.0);
    ctx.body.vel_b = Vec3(0.0, 0.0, 12.0);
    const double l0 = 0.09;
    const VortexRingThrust t = vortex_ring_thrust(l0, 0.0, 12.0 / pp.tip_speed());
    ASSERT_TRUE(t.active);
    std::vector<double> x = {l0, 0.0, 0.0, 0.0};
    std::vector<double> xd(4, 0.0);
    std::vector<double> G(24, 0.0);
    ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
    pp.derivatives(ctx, x, d);
    const double tau = 0.7 / t.lambda_h * 2.0 * std::numbers::pi / 27.0;
    EXPECT_NEAR(xd[3], (t.ct_vrs - t.ct_base) / tau, 1e-15);
}

// ---- The UH-60A ----------------------------------------------------------------------

// One vertical-descent trim of the UH-60A: the main rotor's descent and
// edgewise speed over its own v_h (from its thrust), and the condensed
// model's heave damping Z_w (the VRS deficit quasi-static, as Johnson's
// "qs inflow"; the lag doesn't move the sign change, Johnson Fig. 52).
struct DescentPoint {
    double vz = 0.0;  // vz / v_h [-]
    double vx = 0.0;  // vx / v_h [-]
    double zw = 0.0;  // [1/s]
};

DescentPoint uh60a_descent(const Executive& ex, std::size_t mr, double rate) {
    const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(mr));
    TrimCondition c;
    c.airspeed = rate;
    c.flight_path_angle = -0.5 * std::numbers::pi;
    std::vector<double> x0;
    const TrimResult r = solve_trim(ex, c, nullptr, x0);
    EXPECT_TRUE(r.converged) << rate;
    const std::vector<double> x = held_orbit_start(ex, c, r);
    const SimContext ctx = ex.context(0.0, x, r.controls, trim_environment(c));
    const RotorLoads l = rotor.evaluate(
        ctx, std::span<const double>(x).subspan(ex.component_offset(mr), ex.component_size(mr)));
    const double lh = std::sqrt(0.5 * l.ct);
    LinearizeOptions o;
    o.azimuths = 8;  // averaged A; a 4-blade rotor's 4/rev is sampled twice per cycle
    o.height = false;
    const CondensedModel cm = condense(linearize(ex, c, r, o));
    return {-l.mu_z / lh, l.mu / lh, cm.A(2, 2)};
}

// The whole aircraft (bare, collective fixed) in vertical descent: the
// heave damping changes sign where Johnson's curve has zero slope, at the
// main rotor's edgewise speed (the shaft tilt gives vx/v_h ≈ 0.02 to 0.09):
// N(vx) and X(vx) of Table 3 step 3.1. The aircraft adds what the axial
// rotor of HeaveDampingChangesSignOnTheBoundary lacks: flapping, the
// fuselage's vertical drag, the edgewise flow. Each crossing by bisection
// from a stable-unstable bracket of descent rates (from the M8d sweep), by
// two regula-falsi steps; gated at 0.05 v_h (measured: VALIDATION.md M8d).
TEST(Uh60aVortexRing, HeaveDampingChangesSignOnTheBoundary) {
    const Config cfg = Config::from_file(test::uh60a_config_path());
    const Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);
    std::size_t mr = 0;
    while (dynamic_cast<const BladeElementRotor*>(&ex.component(mr)) == nullptr) {
        ++mr;
    }
    const auto crossing = [&](double lo, double hi) {
        DescentPoint a = uh60a_descent(ex, mr, lo);
        DescentPoint b = uh60a_descent(ex, mr, hi);
        EXPECT_NE(a.zw > 0.0, b.zw > 0.0) << lo << ", " << hi;
        // Regula falsi in the descent rate, two steps (Z_w is smooth here).
        for (int k = 0; k < 2; ++k) {
            const double mid = lo + a.zw / (a.zw - b.zw) * (hi - lo);
            const DescentPoint m = uh60a_descent(ex, mr, mid);
            if ((m.zw > 0.0) == (a.zw > 0.0)) {
                lo = mid;
                a = m;
            } else {
                hi = mid;
                b = m;
            }
        }
        const double t = a.zw / (a.zw - b.zw);
        std::printf("crossing at vz/v_h = %.4f (vx/v_h = %.4f)\n", a.vz + t * (b.vz - a.vz),
                    a.vx + t * (b.vx - a.vx));
        return DescentPoint{a.vz + t * (b.vz - a.vz), a.vx + t * (b.vx - a.vx), 0.0};
    };
    const DescentPoint n = crossing(3.5, 7.5);
    EXPECT_NEAR(n.vz, boundary_n(n.vx), 0.05) << n.vx;
    const DescentPoint x = crossing(15.5, 19.5);
    EXPECT_NEAR(x.vz, boundary_x(x.vx), 0.05) << x.vx;
}

// ---- Real-time rules -------------------------------------------------------------------

TEST(VortexRing, AllocatesNothing) {
    const BladeElementRotor pp(
        vrs_rotor(InflowModel::pitt_peters, VortexRingModel::johnson_2005, 22.0));
    const BladeElementRotor uni(
        vrs_rotor(InflowModel::uniform, VortexRingModel::johnson_2005, 22.0));
    SimContext ctx = sea_level_context(0.0);
    ctx.body.vel_b = Vec3(2.0, 0.0, 12.0);
    std::vector<double> x = {0.09, 0.0, 0.0, 0.001};
    std::vector<double> xd(4, 0.0);
    std::vector<double> G(24, 0.0);
    ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
    test::AllocationCounter counter;
    for (int i = 0; i < 10; ++i) {
        pp.derivatives(ctx, x, d);
        (void)uni.evaluate(ctx);
        (void)vortex_ring_thrust(0.05, 0.01, 0.04);
    }
    EXPECT_EQ(counter.count(), 0U);
}

}  // namespace
}  // namespace eyas
