// M3 acceptance tests for the blade-element rotor (spec §9 M3, D-007, D-024):
// layer 1, the production code against an independent exact-equation
// reference; layer 2, the hover closed form of Leishman 2nd ed. §3.2; loads
// steady across azimuth in hover; and the conventions, purity and real-time
// rules the rotor must keep.

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/bet_reference.hpp"
#include "support/m3_fixtures.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using test::airfoil_from;
using test::linear_airfoil_csv;
using test::reference_condition;
using test::reference_loads;
using test::reference_loads_at;
using test::reference_rotor;
using test::RefLoads;
using test::sea_level_context;
using test::synthetic_airfoil_csv;
using test::test_rotor;

constexpr double kDeg = std::numbers::pi / 180.0;

SwashplatePitch pitch(double theta0_deg, double theta1s_deg = 0.0, double theta1c_deg = 0.0) {
    return {theta0_deg * kDeg, theta1s_deg * kDeg, theta1c_deg * kDeg};
}

// The six hub-axis load components and λ_i, for refinement studies.
struct LoadVector {
    std::array<double, 7> v{};
};

LoadVector as_vector(const RotorLoads& l) {
    return {{l.F_h.x(), l.F_h.y(), l.F_h.z(), l.M_h.x(), l.M_h.y(), l.M_h.z(), l.lambda_i}};
}

LoadVector as_vector(const RefLoads& l) {
    return {{l.F_h.x(), l.F_h.y(), l.F_h.z(), l.M_h.x(), l.M_h.y(), l.M_h.z(), l.lambda_i}};
}

// Force and moment scales for relative errors: thrust, and thrust × R.
struct Scales {
    double force;
    double moment;
};

// Asserts that the production loads converge to the reference at second
// order (midpoint rule) and that their Richardson extrapolation from N and 2N
// elements agrees with it to `tol` relative. `make` builds the rotor with a
// given element count; `eval` evaluates it.
template <class Make, class Eval>
void expect_converges_to(const Make& make, const Eval& eval, const LoadVector& ref, int n0,
                         Scales sc, double lambda_scale, double tol) {
    std::vector<LoadVector> e;
    for (int n = n0; n <= 8 * n0; n *= 2) {
        const BladeElementRotor rotor = make(n);
        e.push_back(as_vector(eval(rotor)));
    }
    for (std::size_t i = 0; i < 7; ++i) {
        double scale = lambda_scale;
        if (i < 3) {
            scale = sc.force;
        } else if (i < 6) {
            scale = sc.moment;
        }
        const double e0 = e[1].v[i] - e[0].v[i];
        const double e1 = e[2].v[i] - e[1].v[i];
        const double e2 = e[3].v[i] - e[2].v[i];
        // Second-order convergence: each halving of the element width cuts
        // the change by a ratio approaching 4 (skip components that are
        // already at round-off). Equal-area spacing has a larger h⁴ term, so
        // the ratio approaches 4 from further away.
        if (std::abs(e1) > 1e-9 * scale) {
            EXPECT_NEAR(e0 / e1, 4.0, 0.5) << "component " << i;
            EXPECT_NEAR(e1 / e2, 4.0, 0.15) << "component " << i;
            EXPECT_LT(std::abs(e1 / e2 - 4.0), std::abs(e0 / e1 - 4.0) + 1e-3) << "component " << i;
        }
        const double extrapolated = (4.0 * e[3].v[i] - e[2].v[i]) / 3.0;
        EXPECT_NEAR(extrapolated, ref.v[i], tol * scale) << "component " << i;
    }
}

// ---- D-007 layer 1: production code vs. the exact-equation reference ------------

// Hover, linear airfoil, at a fixed λ_i: the blade-element pass alone.
TEST(BladeElementRotor, HoverPassMatchesExactReference) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    const SimContext ctx = sea_level_context(0.013);
    for (const ElementSpacing sp : {ElementSpacing::uniform, ElementSpacing::equal_area}) {
        const auto make = [&](int n) {
            return BladeElementRotor(test_rotor(lin, pitch(18.0), n, sp));
        };
        const BladeElementRotor r0 = make(10);
        const RefLoads ref =
            reference_loads_at(reference_rotor(r0), reference_condition(r0, ctx), 0.06);
        const Scales sc{ref.thrust, ref.thrust * 8.0};
        expect_converges_to(
            make, [&](const BladeElementRotor& r) { return r.evaluate_at_inflow(ctx, 0.06); },
            as_vector(ref), 160, sc, 0.06, 1e-9);
    }
}

// Hover with the inflow solved: Eyas's secant iteration and the reference's
// bisection must find the same λ_i as the elements are refined.
TEST(BladeElementRotor, HoverWithSolvedInflowMatchesExactReference) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    const SimContext ctx = sea_level_context(0.0);
    const auto make = [&](int n) { return BladeElementRotor(test_rotor(lin, pitch(18.0), n)); };
    const BladeElementRotor r0 = make(10);
    const RefLoads ref = reference_loads(reference_rotor(r0), reference_condition(r0, ctx));
    ASSERT_GT(ref.thrust, 0.0);
    const Scales sc{ref.thrust, ref.thrust * 8.0};
    expect_converges_to(
        make, [&](const BladeElementRotor& r) { return r.evaluate(ctx); }, as_vector(ref), 160, sc,
        ref.lambda_i, 1e-9);
}

// Tip loss: two meshes with an edge exactly at B·R. With r_root = 0.1R and
// B = 0.97, N = 30k gives k tip elements, so the refinement stays regular.
TEST(BladeElementRotor, TipLossMatchesExactReference) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    const SimContext ctx = sea_level_context(0.0);
    const auto make = [&](int n) {
        BladeElementRotorConfig c = test_rotor(lin, pitch(18.0), n, ElementSpacing::equal_area);
        c.root_cutout = 0.8;
        c.tip_loss = 0.97;
        return BladeElementRotor(c);
    };
    const BladeElementRotor r0 = make(30);
    ASSERT_EQ(r0.elements().size(), 30U);
    EXPECT_EQ(r0.elements().back().lift, 0.0);
    EXPECT_EQ(r0.elements()[28].lift, 1.0);
    EXPECT_DOUBLE_EQ(r0.elements()[28].r + 0.5 * r0.elements()[28].dr, 0.97 * 8.0);
    const RefLoads ref = reference_loads(reference_rotor(r0), reference_condition(r0, ctx));
    const Scales sc{ref.thrust, ref.thrust * 8.0};
    expect_converges_to(
        make, [&](const BladeElementRotor& r) { return r.evaluate(ctx); }, as_vector(ref), 240, sc,
        ref.lambda_i, 1e-9);
}

// Forward flight with everything on: in-plane and vertical hub velocity,
// sideslip, body rates, cyclic, hub tilt and offset, both rotation
// directions. μ ≈ 0.2 with r_root = 0.3R keeps the blades out of reverse
// flow: there the linear test airfoil's α wraps from π to −π and C_l jumps,
// so the integrand would not be smooth (reverse flow is tested next, with an
// airfoil that is continuous at ±180°).
TEST(BladeElementRotor, ForwardFlightMatchesExactReference) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    for (const RotationDirection dir :
         {RotationDirection::counter_clockwise, RotationDirection::clockwise}) {
        SimContext ctx = sea_level_context(0.0371);
        ctx.body.vel_b = Vec3(42.0, -6.0, 3.0);
        ctx.body.omega_b = Vec3(0.3, -0.2, 0.15);
        ctx.body.q_nb =
            Quat(Eigen::AngleAxisd(0.4, Vec3::UnitZ()) * Eigen::AngleAxisd(-0.1, Vec3::UnitY()));
        ctx.env.wind_n = Vec3(-5.0, 2.0, 0.5);
        const auto make = [&](int n) {
            BladeElementRotorConfig c = test_rotor(lin, pitch(14.0, -6.0, 2.5), n);
            c.direction = dir;
            c.root_cutout = 2.4;
            c.hub_position_b = Vec3(0.3, 0.05, -2.1);
            c.hub_pitch = -3.0 * kDeg;
            c.hub_roll = 1.0 * kDeg;
            return BladeElementRotor(c);
        };
        const BladeElementRotor r0 = make(10);
        const RefLoads ref = reference_loads(reference_rotor(r0), reference_condition(r0, ctx));
        ASSERT_GT(ref.thrust, 0.0);
        const Scales sc{ref.thrust, ref.thrust * 8.0};
        expect_converges_to(
            make, [&](const BladeElementRotor& r) { return r.evaluate(ctx); }, as_vector(ref), 160,
            sc, ref.lambda_i, 1e-9);
    }
}

// The nonlinear synthetic airfoil at μ ≈ 0.45: reverse flow on the retreating
// side, stall, Mach effects up to the table edge. The tables are piecewise
// linear, so the integrand has kinks and Richardson extrapolation doesn't
// apply; instead the error against the reference must shrink with N and be
// small at fine resolution.
TEST(BladeElementRotor, ReverseFlowAndStallMatchExactReference) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    // Downflow through the disc, then upflow (descending): with upflow in the
    // reverse-flow region φ is near −π, so α = θ − φ passes π and must wrap.
    for (const Vec3& vel : {Vec3(97.0, 4.0, -2.0), Vec3(90.0, -3.0, 22.0)}) {
        SimContext ctx = sea_level_context(0.0517);
        ctx.body.vel_b = vel;
        ctx.body.omega_b = Vec3(-0.1, 0.25, 0.05);
        const auto make = [&](int n) {
            BladeElementRotorConfig c = test_rotor(syn, pitch(16.0, -9.0, 1.5), n);
            c.root_cutout = 0.6;
            c.tip_loss = 0.97;
            return BladeElementRotor(c);
        };
        const BladeElementRotor r0 = make(10);
        const RefLoads ref = reference_loads(reference_rotor(r0), reference_condition(r0, ctx));
        const double fscale = std::abs(ref.thrust);
        ASSERT_GT(fscale, 1000.0);
        double prev_err = 0.0;
        for (const int n : {50, 200, 800, 3200}) {
            const RotorLoads l = make(n).evaluate(ctx);
            const LoadVector e = as_vector(l);
            const LoadVector rv = as_vector(ref);
            double err = 0.0;
            for (std::size_t i = 0; i < 6; ++i) {
                const double scale = i < 3 ? fscale : fscale * 8.0;
                err = std::max(err, std::abs(e.v[i] - rv.v[i]) / scale);
            }
            if (n > 50) {
                EXPECT_LT(err, 0.35 * prev_err) << "N = " << n;  // shrinks with N
            }
            prev_err = err;
        }
        EXPECT_LT(prev_err, 2e-6) << "vel_b = " << vel.transpose();
    }
}

// ---- D-007 layer 2: the hover closed form (Leishman 2nd ed. §3.2) ----------------

// Leishman 2nd ed. Eq. 3.25 with Eq. 3.32 (λ = √(C_T/2)), solved together:
//   2λ² + (σa/4)λ − (σa/2)(θ0/3 + θ_tw/4) = 0,
// and Eq. 3.31, C_P = λC_T + σC_d0/8.
struct ClosedForm {
    double ct;
    double cp;
    double lambda;
};

ClosedForm leishman_hover(double sigma, double a, double theta0, double theta_tw, double cd0) {
    const double k = sigma * a / 4.0;
    const double lam = (-k + std::sqrt(k * k + 16.0 * k * (theta0 / 3.0 + theta_tw / 4.0))) / 4.0;
    const double ct = 2.0 * lam * lam;
    return {ct, lam * ct + sigma * cd0 / 8.0, lam};
}

// The closed form's assumptions: constant chord, C_l = aα, C_d = C_d0,
// linear twist, uniform inflow, no root cutout, no tip loss (D-007 layer 2).
// Its small-angle approximations (φ ≈ λ/r, U ≈ U_T, D sin φ ≈ 0) leave a
// residual: relative to C_T, lift terms of O(λ²) and a drag term
// −σC_d0/(8λ). The refinement study in VALIDATION.md (M3) measures it
// (≤ 0.6% in C_T, ≤ 1.0% in C_P over C_T up to 0.009, with discretization
// error below 1e-5); the tolerances below are set from it.
TEST(BladeElementRotor, HoverMatchesLeishmanClosedForm) {
    const double a = 5.73;
    const double cd0 = 0.01;
    const auto lin = airfoil_from(linear_airfoil_csv(a, cd0));
    const SimContext ctx = sea_level_context(0.0);
    const double sigma = 4.0 * 0.53 / (std::numbers::pi * 8.0);
    for (const double tw : {0.0, -10.0}) {
        for (const double th75 : {4.0, 8.0, 12.0}) {
            const double th = th75 - 0.75 * tw;  // θ0 at the hub center
            BladeElementRotorConfig c = test_rotor(lin, pitch(th), 400);
            c.root_cutout = 0.0;
            c.twist = Table1D(Axis({0.0, 1.0}), {0.0, tw * kDeg});
            const RotorLoads l = BladeElementRotor(c).evaluate(ctx);
            const ClosedForm cf = leishman_hover(sigma, a, th * kDeg, tw * kDeg, cd0);
            ASSERT_TRUE(l.converged);
            EXPECT_NEAR(l.ct / cf.ct, 1.0, 1e-2) << "θ0 = " << th << ", θtw = " << tw;
            EXPECT_NEAR(l.cq / cf.cp, 1.0, 1.5e-2) << "θ0 = " << th << ", θtw = " << tw;
            EXPECT_NEAR(l.lambda_i / cf.lambda, 1.0, 5e-3) << "θ0 = " << th << ", θtw = " << tw;
        }
    }
}

// The residual comes from the small-angle approximations, so it must vanish
// as they become exact. With C_d0 = 0 and no twist, expanding the exact lift
// integrand a r √(r² + λ²)(θ − atan(λ/r)) for r ≫ λ gives
// a(θr² − λr + θλ²/2 − λ³/(6r) + …): at fixed λ, C_T rises by a relative
// (3/2)λ² over Leishman 2nd ed. Eq. 3.22. Solving the inflow with the thrust scales
// that by 4λ/(4λ + σa/4) < 1, so |ε| < (3/2)λ² up to O(λ³ ln λ).
TEST(BladeElementRotor, ClosedFormResidualVanishesWithInflowAngle) {
    const double a = 5.73;
    const auto lin = airfoil_from(linear_airfoil_csv(a, 0.0));
    const SimContext ctx = sea_level_context(0.0);
    const double sigma = 4.0 * 0.53 / (std::numbers::pi * 8.0);
    for (const double th : {16.0, 12.0, 6.0, 3.0, 1.5}) {
        BladeElementRotorConfig c = test_rotor(lin, pitch(th), 800);
        c.root_cutout = 0.0;
        c.twist = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
        const RotorLoads l = BladeElementRotor(c).evaluate(ctx);
        const ClosedForm cf = leishman_hover(sigma, a, th * kDeg, 0.0, 0.0);
        const double eps = l.ct / cf.ct - 1.0;
        EXPECT_LT(std::abs(eps), 1.5 * cf.lambda * cf.lambda) << "θ0 = " << th;
    }
}

// ---- Spec M3: steady across azimuth in hover --------------------------------------

// In hover the flow is axisymmetric, so thrust, torque and power at constant
// collective don't depend on azimuth, and the in-plane forces and hub moments
// of the balanced rotor vanish (to round-off).
TEST(BladeElementRotor, HoverLoadsAreSteadyAcrossAzimuth) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    for (const int nb : {2, 3, 4, 5}) {
        BladeElementRotorConfig c = test_rotor(syn, pitch(14.0), 24);
        c.num_blades = nb;
        const BladeElementRotor rotor(c);
        const RotorLoads l0 = rotor.evaluate(sea_level_context(0.0));
        ASSERT_GT(l0.thrust, 0.0);
        for (int i = 1; i < 97; ++i) {
            const double t = i * 0.0021;  // azimuth steps of 3.2°, over 1.2 rev
            const RotorLoads l = rotor.evaluate(sea_level_context(t));
            EXPECT_NEAR(l.thrust, l0.thrust, 1e-12 * l0.thrust);
            EXPECT_NEAR(l.torque, l0.torque, 1e-12 * l0.torque);
            EXPECT_NEAR(l.power, l0.power, 1e-12 * l0.power);
            EXPECT_NEAR(l.F_h.head<2>().norm(), 0.0, 1e-12 * l0.thrust);
            EXPECT_NEAR(l.M_h.head<2>().norm(), 0.0, 1e-12 * l0.thrust * 8.0);
        }
    }
}

// In forward flight the loads repeat every 1/N_b of a revolution: advancing
// the azimuth by 2π/N_b just relabels the blades.
TEST(BladeElementRotor, ForwardFlightLoadsHaveBladePassagePeriod) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    const BladeElementRotor rotor(test_rotor(syn, pitch(22.0, -3.0), 24));
    const double period = 2.0 * std::numbers::pi / (4.0 * 27.0);
    for (int i = 0; i < 10; ++i) {
        SimContext a = sea_level_context(0.003 * i);
        a.body.vel_b = Vec3(60.0, 0.0, 0.0);
        SimContext b = a;
        b.t += period;
        const RotorLoads la = rotor.evaluate(a);
        const RotorLoads lb = rotor.evaluate(b);
        ASSERT_GT(la.thrust, 1000.0);
        EXPECT_NEAR((la.F_h - lb.F_h).norm(), 0.0, 1e-9 * la.thrust);
        EXPECT_NEAR((la.M_h - lb.M_h).norm(), 0.0, 1e-9 * la.thrust * 8.0);
    }
}

// ---- Conventions -------------------------------------------------------------------

// A clockwise rotor is the mirror image (in the x–z plane) of a
// counter-clockwise one: Y force, roll and yaw moments change sign; X, Z and
// pitch don't; torque and power are the same.
TEST(BladeElementRotor, ClockwiseRotorMirrorsCounterClockwise) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    BladeElementRotorConfig c = test_rotor(syn, pitch(14.0, -5.0, 0.0), 24);
    const BladeElementRotor ccw(c);
    c.direction = RotationDirection::clockwise;
    const BladeElementRotor cw(c);
    SimContext ctx = sea_level_context(0.017);
    ctx.body.vel_b = Vec3(50.0, 0.0, 2.0);
    ctx.body.omega_b = Vec3(0.0, 0.1, 0.0);  // pitch rate is symmetric under the mirror
    const RotorLoads a = ccw.evaluate(ctx);
    const RotorLoads b = cw.evaluate(ctx);
    const double tol = 1e-10 * a.thrust;
    EXPECT_NEAR(a.F_h.x(), b.F_h.x(), tol);
    EXPECT_NEAR(a.F_h.y(), -b.F_h.y(), tol);
    EXPECT_NEAR(a.F_h.z(), b.F_h.z(), tol);
    EXPECT_NEAR(a.M_h.x(), -b.M_h.x(), 8.0 * tol);
    EXPECT_NEAR(a.M_h.y(), b.M_h.y(), 8.0 * tol);
    EXPECT_NEAR(a.M_h.z(), -b.M_h.z(), 8.0 * tol);
    EXPECT_NEAR(a.torque, b.torque, 8.0 * tol);
}

// Physical signs for a counter-clockwise rotor (UH-60 sense):
// - hover: thrust up (F_z < 0), torque positive, and the drag reaction yaws
//   the airframe nose right (M_z > 0 about body z, down);
// - forward flight with no cyclic and no flapping: the advancing side (right)
//   lifts more, so the rotor rolls the airframe left (M_x < 0).
TEST(BladeElementRotor, CounterClockwiseRotorLoadSigns) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    const BladeElementRotor rotor(test_rotor(syn, pitch(14.0), 24));
    const RotorLoads hover = rotor.evaluate(sea_level_context(0.0));
    EXPECT_LT(hover.F_h.z(), 0.0);
    EXPECT_GT(hover.torque, 0.0);
    EXPECT_GT(hover.M_h.z(), 0.0);
    EXPECT_GT(hover.lambda_i, 0.0);
    SimContext fwd = sea_level_context(0.0);
    fwd.body.vel_b = Vec3(40.0, 0.0, 0.0);
    // Average over a blade passage to remove the 4/rev content.
    double mx = 0.0;
    for (int i = 0; i < 16; ++i) {
        fwd.t = i * (2.0 * std::numbers::pi / (4.0 * 27.0)) / 16.0;
        mx += rotor.evaluate(fwd).M_h.x();
    }
    EXPECT_LT(mx, 0.0);
}

// Azimuth from time (D-024), and Padfield's blade numbering (Eq. 3.49).
TEST(BladeElementRotor, AzimuthFromTime) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    const BladeElementRotor rotor(test_rotor(lin, pitch(10.0), 10));
    EXPECT_EQ(rotor.azimuth(0.0), 0.0);
    EXPECT_NEAR(rotor.azimuth(0.01), 0.27, 1e-15);
    // About an hour of revolutions, then a quarter turn: ψ = π/2 up to the
    // rounding of Ω t (≈ 1e5 rad, so ~1e-11 rad).
    const double rev = 2.0 * std::numbers::pi / 27.0;
    EXPECT_NEAR(rotor.azimuth(15000.0 * rev + 0.25 * rev), 0.5 * std::numbers::pi, 1e-9);
    EXPECT_NEAR(rotor.azimuth(-0.25 * rev), 1.5 * std::numbers::pi, 1e-12);
    EXPECT_DOUBLE_EQ(rotor.blade_azimuth(1.0, 1), 1.0 - 0.5 * std::numbers::pi);
}

// The hub frame for a forward shaft tilt γ (hub pitch −γ): the shaft axis z_h
// is (−sin γ, 0, cos γ) in body axes, so thrust tilts forward; and the body
// wrench is the hub wrench rotated and moved to the CG.
TEST(BladeElementRotor, HubFrameAndBodyTransform) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    BladeElementRotorConfig c = test_rotor(lin, pitch(14.0), 20);
    const double gamma = 3.0 * kDeg;
    c.hub_pitch = -gamma;
    c.hub_position_b = Vec3(0.4, 0.0, -2.0);
    const BladeElementRotor rotor(c);
    const Vec3 z_h_in_b = rotor.C_hb().transpose() * Vec3::UnitZ();
    EXPECT_NEAR(z_h_in_b.x(), -std::sin(gamma), 1e-15);
    EXPECT_NEAR(z_h_in_b.y(), 0.0, 1e-15);
    EXPECT_NEAR(z_h_in_b.z(), std::cos(gamma), 1e-15);
    const RotorLoads l = rotor.evaluate(sea_level_context(0.0));
    EXPECT_GT(l.body.F.x(), 0.0);  // forward component of thrust
    EXPECT_NEAR(l.body.F.x(), l.thrust * std::sin(gamma), 1e-9 * l.thrust);
    const Vec3 M_expected = rotor.C_hb().transpose() * l.M_h + c.hub_position_b.cross(l.body.F);
    EXPECT_NEAR((l.body.M - M_expected).norm(), 0.0, 1e-9 * l.thrust);
    // Hover: the hub force is the thrust along −z_h, and the hub moment is
    // only the torque reaction about z_h, which has no body-y component. So
    // M_y = r_z F_x − r_x F_z = T (0.4 cos γ − 2 sin γ): the thrust 0.4 m
    // ahead of the CG pitches the nose up, its forward tilt 2 m above the CG
    // pitches it down.
    EXPECT_NEAR(l.body.M.y(), l.thrust * (0.4 * std::cos(gamma) - 2.0 * std::sin(gamma)),
                1e-9 * l.thrust);
}

// ---- Inflow solve -----------------------------------------------------------------

// Hover, climb, windmill-brake descent (clear of the vortex ring), negative
// thrust and forward flight all converge, in a few passes, to a root of
// Padfield 3rd ed. Eq. 3.139. With θ0 = 22° (14° at 0.75R) the hover induced
// velocity is about 13 m/s.
TEST(BladeElementRotor, InflowSolveConvergesAcrossConditions) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    struct Case {
        double theta0;
        Vec3 vel_b;
    };
    for (const Case& cs : {Case{22.0, Vec3::Zero()}, Case{22.0, Vec3(0.0, 0.0, -10.0)},
                           Case{22.0, Vec3(0.0, 0.0, 45.0)}, Case{-2.0, Vec3::Zero()},
                           Case{16.0, Vec3(80.0, 0.0, 5.0)}, Case{0.0, Vec3(30.0, 0.0, 0.0)}}) {
        const BladeElementRotor rotor(test_rotor(syn, pitch(cs.theta0), 20));
        SimContext ctx = sea_level_context(0.004);
        ctx.body.vel_b = cs.vel_b;
        const RotorLoads l = rotor.evaluate(ctx);
        EXPECT_TRUE(l.converged) << "θ0 = " << cs.theta0;
        EXPECT_LE(l.passes, 10) << "θ0 = " << cs.theta0;
        const double d = l.lambda_i - l.mu_z;
        EXPECT_NEAR(l.lambda_i * 2.0 * std::sqrt(l.mu * l.mu + d * d), l.ct, 1e-12)
            << "θ0 = " << cs.theta0;
    }
}

// In the vortex-ring state (descent near the hover induced velocity) momentum
// theory may have several roots (Padfield 3rd ed. §3.2.1). The solve must
// still stay within its pass limit, return finite loads, and either meet its
// tolerance or say it didn't (D-023).
TEST(BladeElementRotor, InflowSolveIsBoundedInVortexRing) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    const BladeElementRotor rotor(test_rotor(syn, pitch(22.0), 20));
    for (int i = 0; i <= 40; ++i) {
        SimContext ctx = sea_level_context(0.0);
        ctx.body.vel_b = Vec3(0.0, 0.0, 0.75 * i);  // descent 0–30 m/s
        const RotorLoads l = rotor.evaluate(ctx);
        EXPECT_LE(l.passes, BladeElementRotor::kMaxPasses);
        EXPECT_TRUE(std::isfinite(l.thrust) && std::isfinite(l.lambda_i));
        if (l.converged) {
            const double d = l.lambda_i - l.mu_z;
            EXPECT_NEAR(l.lambda_i * 2.0 * std::sqrt(d * d), l.ct, 1e-11) << "descent " << 0.75 * i;
        }
    }
}

// ---- Real-time rules and purity ----------------------------------------------------

ExecutiveConfig executive_config() {
    ExecutiveConfig ec;
    ec.mass = make_mass_properties(7000.0, 6000.0, 40000.0, 37000.0, 1900.0);
    return ec;
}

std::vector<std::unique_ptr<DynamicComponent>> one_rotor(std::shared_ptr<const Airfoil> af) {
    std::vector<std::unique_ptr<DynamicComponent>> v;
    BladeElementRotorConfig c = test_rotor(std::move(af), pitch(14.0, -3.0), 20);
    c.hub_position_b = Vec3(0.0, 0.0, -2.0);
    v.push_back(std::make_unique<BladeElementRotor>(c));
    return v;
}

// The cached table-search hints must not change results: the same
// (t, x, inputs) gives bit-identical derivatives with other evaluations in
// between (spec §5).
TEST(BladeElementRotor, DerivativesArePureWithRotor) {
    Executive ex(executive_config(), one_rotor(airfoil_from(synthetic_airfoil_csv())));
    std::vector<double> x(ex.state().begin(), ex.state().end());
    x[body_layout::kVel] = 45.0;
    std::vector<double> y = x;
    y[body_layout::kVel] = -20.0;
    y[body_layout::kOmega + 1] = 0.4;
    std::vector<double> d1(x.size());
    std::vector<double> d2(x.size());
    std::vector<double> scratch(x.size());
    EXPECT_TRUE(ex.derivatives(0.0123, x, Controls{}, Environment{}, d1));
    EXPECT_TRUE(ex.derivatives(1.7, y, Controls{}, Environment{}, scratch));
    EXPECT_TRUE(ex.derivatives(0.0123, x, Controls{}, Environment{}, d2));
    EXPECT_EQ(std::memcmp(d1.data(), d2.data(), d1.size() * sizeof(double)), 0);
}

TEST(BladeElementRotor, AdvanceDoesNotAllocateAndConverges) {
    Executive ex(executive_config(), one_rotor(airfoil_from(synthetic_airfoil_csv())));
    BodyState b;
    b.vel_b = Vec3(30.0, 0.0, 0.0);
    ex.set_body_state(b);
    test::AllocationCounter count;
    for (int i = 0; i < 20; ++i) {
        const AdvanceStatus st = ex.advance(Controls{}, Environment{}, 16'666'667);
        ASSERT_FALSE(st.non_finite);
        EXPECT_EQ(ex.unconverged_evaluations(), 0);
    }
    EXPECT_EQ(count.count(), 0U);
}

// ---- Construction errors -----------------------------------------------------------

TEST(BladeElementRotor, RejectsInvalidConfiguration) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    const auto bad = [&](auto&& edit) {
        BladeElementRotorConfig c = test_rotor(lin, pitch(10.0), 20);
        edit(c);
        EXPECT_THROW(BladeElementRotor{c}, InitError);
    };
    bad([](BladeElementRotorConfig& c) { c.num_blades = 0; });
    bad([](BladeElementRotorConfig& c) { c.radius = 0.0; });
    bad([](BladeElementRotorConfig& c) { c.rotor_speed = -27.0; });
    bad([](BladeElementRotorConfig& c) { c.tip_loss = 0.0; });
    bad([](BladeElementRotorConfig& c) { c.tip_loss = 1.01; });
    bad([](BladeElementRotorConfig& c) { c.root_cutout = 8.0; });
    bad([](BladeElementRotorConfig& c) { c.num_elements = 0; });
    bad([](BladeElementRotorConfig& c) { c.airfoil = nullptr; });
    bad([](BladeElementRotorConfig& c) { c.controls = nullptr; });
    bad([](BladeElementRotorConfig& c) { c.chord = Table1D(Axis({0.0, 1.0}), {0.5, -0.1}); });
    bad([](BladeElementRotorConfig& c) {
        c.hub_position_b.x() = std::numeric_limits<double>::quiet_NaN();
    });
}

}  // namespace
}  // namespace eyas
