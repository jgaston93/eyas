// M4 literature anchors (D-026, layer 3): flap frequency, hub-motion forcing,
// coning, flapping against advance ratio, cyclic phase lag, δ₃ and the
// hub force from disc tilt. Fixtures match each closed form's assumptions;
// the tolerances cover its small-angle and first-harmonic approximations and
// were set from the measured residuals recorded in VALIDATION.md.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include "rotor/blade_element_rotor.hpp"
#include "rotor/controls.hpp"
#include "support/flap_closed_forms.hpp"
#include "support/m4_fixtures.hpp"

namespace eyas {
namespace {

using test::kDeg;

using test::kLiftSlope;

// Measures the free flap frequency [rad/s] of blade 1 in vacuum, hub at rest,
// from upward crossings of its mean over `periods` periods.
double free_flap_frequency(const BladeElementRotor& rotor, double guess, int periods) {
    SimContext ctx = test::sea_level_context();
    std::vector<double> x(rotor.num_states(), 0.0);
    for (std::size_t i = 0; i < x.size(); i += 2) {
        x[i] = 0.01;
    }
    const double h = 1e-3;
    const int steps = static_cast<int>(periods * 2.0 * std::numbers::pi / guess / h);
    std::vector<double> t;
    std::vector<double> b;
    test::whirl(rotor, ctx, x, h, steps, [&](double ts, const std::vector<double>& xs) {
        t.push_back(ts);
        b.push_back(xs[0]);
    });
    double mean = 0.0;
    for (double v : b) {
        mean += v;
    }
    mean /= static_cast<double>(b.size());
    std::vector<double> crossings;
    for (std::size_t i = 1; i < b.size(); ++i) {
        if (b[i - 1] < mean && b[i] >= mean) {
            const double f = (mean - b[i - 1]) / (b[i] - b[i - 1]);
            crossings.push_back(t[i - 1] + f * (t[i] - t[i - 1]));
        }
    }
    const double period =
        (crossings.back() - crossings.front()) / static_cast<double>(crossings.size() - 1);
    return 2.0 * std::numbers::pi / period;
}

// Rotating flap frequency Ω ν: the spec's uniform-blade formula
// ν² = 1 + (3/2) e/(1 − e) + K/(IΩ²) (Johnson 2013 Eq. 6.262 with a spring)
// and, for a non-uniform blade, ν² = 1 + e S_β/I_β + K_β/(I_β Ω²)
// (Johnson 2013 Eq. 6.261). Non-rotating: √(K_β/I_β) (D-009).
TEST(FlapAnchors, FreeFlapFrequencyMatchesNu) {
    const auto vacuum = test::airfoil_from(test::zero_airfoil_csv());
    {
        SCOPED_TRACE("uniform blade, spec formula");
        const double e = 0.6;
        const BladeElementRotorConfig c =
            test::flap_rotor(vacuum, {}, test::uniform_beads(100.0, 8.0 - e, 2000), e, 4);
        const double eb = e / c.radius;
        // The spec's e is the hinge offset over the blade length from the centre.
        const double nu2 = 1.0 + 1.5 * eb / (1.0 - eb);
        const double w = free_flap_frequency(BladeElementRotor(c), 27.0, 30);
        EXPECT_NEAR(w / (c.rotor_speed * std::sqrt(nu2)), 1.0, 1e-4);
    }
    {
        SCOPED_TRACE("non-uniform blade with a spring, Johnson Eq. 6.261");
        BladeElementRotorConfig c =
            test::flap_rotor(vacuum, {}, test::nonuniform_beads(7.6, 12.0), 0.4, 4);
        c.flap->spring = 0.15 * c.flap->inertia * 27.0 * 27.0;
        const double w = free_flap_frequency(BladeElementRotor(c), 27.0, 30);
        EXPECT_NEAR(w / (c.rotor_speed * std::sqrt(test::nu2(c))), 1.0, 1e-4);
    }
    {
        SCOPED_TRACE("non-rotating, √(K/I)");
        BladeElementRotorConfig c =
            test::flap_rotor(vacuum, {}, test::nonuniform_beads(7.6, 12.0), 0.4, 4);
        c.rotor_speed = 1e-3;
        c.flap->spring = 400.0 * c.flap->inertia;  // 20 rad/s
        const double w = free_flap_frequency(BladeElementRotor(c), 20.0, 30);
        EXPECT_NEAR(w / 20.0, 1.0, 1e-4);
    }
}

// Linear hub-motion forcing at β = β̇ = 0, hub at the CG, in vacuum. Padfield
// 3rd ed. Eq. 3.20 (e = 0, anticlockwise rotor) gives the flapwise
// acceleration 2Ω(p cos ψ − q sin ψ) + (q̇ cos ψ + ṗ sin ψ); Johnson 2013
// Eq. 16.240 gives the weights for an offset hinge: ∫ r η m dr = I_β + e S_β
// on the angular terms and ∫ η m dr = S_β on the vertical hub acceleration
// (η = r − e, the hinge-angle mode, §6.15). So
//   ∂β̈/∂ṗ = k sin ψ, ∂β̈/∂q̇ = k cos ψ, ∂β̈/∂p = 2Ωk cos ψ, ∂β̈/∂q = −2Ωk sin ψ,
//   ∂β̈/∂ẇ = S_β/I_β, with k = (I_β + e S_β)/I_β; no u̇, v̇ or ṙ terms.
TEST(FlapAnchors, HubMotionForcingMatchesJohnsonAndPadfield) {
    const auto vacuum = test::airfoil_from(test::zero_airfoil_csv());
    for (const double e : {0.0, 0.4}) {
        SCOPED_TRACE(e);
        BladeElementRotorConfig c =
            test::flap_rotor(vacuum, {}, test::nonuniform_beads(8.0 - e, 12.0), e, 4);
        c.hub_position_b = Vec3::Zero();
        const BladeElementRotor rotor(c);
        const FlapHinge& f = *c.flap;
        const double k = (f.inertia + e * f.first_moment) / f.inertia;
        const double W = c.rotor_speed;
        const std::vector<double> x(8, 0.0);
        for (const double t : {0.0, 0.013, 0.031, 0.05}) {
            SimContext ctx = test::sea_level_context(t);
            const double psi = rotor.azimuth(t);
            const auto bdd = [&](const SimContext& cx, std::vector<double>* G = nullptr) {
                std::vector<double> xd(8);
                std::vector<double> g(48);
                ComponentDerivs d{xd, g, Wrench{}, Mat66::Zero()};
                rotor.derivatives(cx, x, d);
                if (G != nullptr) {
                    *G = g;
                }
                return xd[1];
            };
            std::vector<double> G;
            (void)bdd(ctx, &G);
            const double* g = G.data() + 6;  // row of β̈ for blade 1
            EXPECT_NEAR(g[0], 0.0, 1e-12);
            EXPECT_NEAR(g[1], 0.0, 1e-12);
            EXPECT_NEAR(g[2], f.first_moment / f.inertia, 1e-12);
            EXPECT_NEAR(g[3], k * std::sin(psi), 1e-12);
            EXPECT_NEAR(g[4], k * std::cos(psi), 1e-12);
            EXPECT_NEAR(g[5], 0.0, 1e-12);
            const double dw = 1e-4;
            for (int axis = 0; axis < 3; ++axis) {
                SimContext up = ctx;
                SimContext dn = ctx;
                up.body.omega_b(axis) = dw;
                dn.body.omega_b(axis) = -dw;
                const double slope = (bdd(up) - bdd(dn)) / (2.0 * dw);
                const double expect = axis == 0   ? 2.0 * W * k * std::cos(psi)
                                      : axis == 1 ? -2.0 * W * k * std::sin(psi)
                                                  : 0.0;
                EXPECT_NEAR(slope, expect, 1e-7) << "axis " << axis;
            }
        }
    }
}

BladeElementRotorConfig anchor_rotor(double e, SwashplatePitch pitch) {
    return test::anchor_flap_rotor(e, pitch);
}

// Steady periodic flapping of an anchor rotor at advance ratio μ, compared
// with a closed form evaluated at Eyas's own inflow.
struct FlapComparison {
    test::Harmonics eyas;
    test::FlapClosedForm closed;
};

template <class Closed>
FlapComparison compare_flapping(const BladeElementRotorConfig& c, double mu, Closed closed) {
    const BladeElementRotor rotor(c);
    SimContext ctx = test::sea_level_context();
    ctx.body.vel_b = Vec3(mu * c.rotor_speed * c.radius, 0.0, 0.0);
    std::vector<double> x(rotor.num_states(), 0.0);
    FlapComparison out;
    out.eyas = test::periodic_flapping(rotor, ctx, x, 12, 3);
    const RotorLoads l = rotor.evaluate(ctx, x);
    test::FlapCondition k;
    k.rho = ctx.air.density;
    k.lift_slope = kLiftSlope;
    k.mu = l.mu;
    k.lambda = l.lambda_i - l.mu_z;
    const SwashplatePitch p = c.controls->pitch({});
    k.theta0 = p.theta0;
    k.theta1c = p.theta1c;
    k.theta1s = p.theta1s;
    k.theta_tw = c.twist(1.0);
    out.closed = closed(c, k);
    return out;
}

// The closed forms drop 2/rev and higher flapping and expand to O(μ²), so
// the residual grows with μ (measured, VALIDATION.md M4): ≤ 1.1% of the disc
// tilt up to μ = 0.1 and ≤ 2% at μ = 0.2. Gates: 1.5% and 3%. At μ = 0.3
// the comparison is reported only.
void expect_flapping_close(double mu, const FlapComparison& r) {
    const double tilt = std::hypot(r.closed.beta1c, r.closed.beta1s);
    const double tol = (mu <= 0.1 ? 0.015 : 0.03) * tilt;
    EXPECT_NEAR(r.eyas.b0, r.closed.beta0, tol) << "mu " << mu;
    EXPECT_NEAR(r.eyas.b1c, r.closed.beta1c, tol) << "mu " << mu;
    EXPECT_NEAR(r.eyas.b1s, r.closed.beta1s, tol) << "mu " << mu;
}

// Hover coning against collective (and so against thrust): Johnson 2013
// Eq. 6.277 at μ = 0 plus the weight moment, Eq. 6.208.
TEST(FlapAnchors, HoverConingMatchesJohnson) {
    for (const double t0 : {4.0, 8.0, 12.0, 16.0}) {
        const BladeElementRotorConfig c = anchor_rotor(0.4, {t0 * kDeg, 0.0, 0.0});
        const FlapComparison r = compare_flapping(c, 0.0, test::johnson_offset_hinge);
        EXPECT_NEAR(r.eyas.b0, r.closed.beta0, 0.01 * std::abs(r.closed.beta0) + 0.002 * kDeg);
    }
}

// Flapping against advance ratio, centre-spring rotor: Padfield 3rd ed.
// Eqs. 3.57, 3.58 and 3.64.
TEST(FlapAnchors, CentreSpringFlappingMatchesPadfield) {
    for (const double mu : {0.0, 0.05, 0.1, 0.2}) {
        BladeElementRotorConfig c = anchor_rotor(0.0, {8 * kDeg, -2 * kDeg, 1 * kDeg});
        c.root_cutout = 0.0;
        c.flap->spring = 0.12 * c.flap->inertia * c.rotor_speed * c.rotor_speed;
        const FlapComparison r = compare_flapping(c, mu, test::padfield_centre_spring);
        expect_flapping_close(mu, r);
    }
}

// Flapping against advance ratio, offset hinge: Johnson 2013 Eqs. 6.277–6.279.
TEST(FlapAnchors, OffsetHingeFlappingMatchesJohnson) {
    for (const double mu : {0.0, 0.05, 0.1, 0.2}) {
        const BladeElementRotorConfig c = anchor_rotor(0.4, {8 * kDeg, -2 * kDeg, 1 * kDeg});
        const FlapComparison r = compare_flapping(c, mu, test::johnson_offset_hinge);
        expect_flapping_close(mu, r);
    }
}

// Cyclic step in hover: the disc settles to the steady response with a phase
// lag of 90° − atan(N*/d1) behind the pitch input (Johnson 2013 Eqs.
// 6.278–6.279 at μ = 0; for the centre spring, Padfield 3rd ed. Eq. 3.40), and
// amplitude ratio c2/√(d1² + N*²) in Johnson's normalization.
TEST(FlapAnchors, CyclicStepPhaseLag) {
    struct Rotor {
        const char* name;
        double e;
        double spring_ratio;  // K_β / (I_β Ω²)
    };
    for (const Rotor& rc : {Rotor{"offset hinge", 0.4, 0.0}, Rotor{"centre spring", 0.0, 0.12}}) {
        SCOPED_TRACE(rc.name);
        BladeElementRotorConfig c = anchor_rotor(rc.e, {});
        c.flap->spring = rc.spring_ratio * c.flap->inertia * c.rotor_speed * c.rotor_speed;
        if (rc.e == 0.0) {
            c.root_cutout = 0.0;
        }
        // Longitudinal stick: θ1s = −3° per unit lon_cyclic.
        c.controls = std::make_shared<const LinearControlMap>(
            LinearPitchRow{10 * kDeg}, LinearPitchRow{0.0, 0.0, -3 * kDeg}, LinearPitchRow{});
        const BladeElementRotor rotor(c);
        SimContext ctx = test::sea_level_context();
        std::vector<double> x(rotor.num_states(), 0.0);
        const test::Harmonics before = test::periodic_flapping(rotor, ctx, x, 10, 2);
        ctx.controls.lon_cyclic = 1.0;  // the step (pitch phase −90°: θ1s < 0)
        ctx.t = 0.0;
        const test::Harmonics after = test::periodic_flapping(rotor, ctx, x, 12, 3);
        const double d1c = after.b1c - before.b1c;
        const double d1s = after.b1s - before.b1s;
        // Phase of θ = θ1s sin ψ with θ1s < 0 is −90°; flap phase atan2(β1s, β1c).
        const double lag = std::atan2(d1s, d1c) - (-0.5 * std::numbers::pi);
        const double expect = test::johnson_hover_phase_lag(c, ctx.air.density, kLiftSlope);
        RecordProperty("lag_deg", test::fmt17(lag / kDeg));
        RecordProperty("lag_expected_deg", test::fmt17(expect / kDeg));
        EXPECT_NEAR(lag, expect, 0.3 * kDeg);
        // Amplitude, in the hinge angle.
        const double R = c.radius;
        const double e = rc.e / R;
        const double L = R - rc.e;
        const double gamma =
            ctx.air.density * kLiftSlope * c.chord(0.5) * R * R * L * L / c.flap->inertia;
        const double N = (test::nu2(c) - 1.0) / (gamma / 8.0);
        const double d1 = 1.0 - (2.0 * e + e * e) / 3.0;
        const double c2 = 1.0 - (e + e * e + e * e * e) / 3.0;
        const double ratio = c2 / std::hypot(d1, N) / (1.0 - e);
        EXPECT_NEAR(std::hypot(d1c, d1s) / (3 * kDeg), ratio, 0.01 * ratio);
    }
}

// Pitch–flap coupling adds an aerodynamic spring (γ/8) K_P to the flap
// stiffness, K_P = tan δ₃ (Johnson 2013 Eqs. 6.303–6.304, hover, e = 0):
// ∂M_aero/∂β changes by −I_β Ω² (γ/8) tan δ₃. Checked at fixed inflow.
TEST(FlapAnchors, Delta3AddsAerodynamicSpring) {
    const auto slope = [](double delta3) {
        BladeElementRotorConfig c = anchor_rotor(0.0, {10 * kDeg, 0.0, 0.0});
        c.root_cutout = 0.0;
        c.flap->delta3 = delta3;
        const BladeElementRotor rotor(c);
        const SimContext ctx = test::sea_level_context();
        const double lambda = rotor.evaluate(ctx).lambda_i;
        const double h = 1e-4;
        std::vector<double> up(8, 0.0);
        std::vector<double> dn(8, 0.0);
        up[0] = h;
        dn[0] = -h;
        return (rotor.evaluate_at_inflow(ctx, lambda, up).flap_moment[0] -
                rotor.evaluate_at_inflow(ctx, lambda, dn).flap_moment[0]) /
               (2.0 * h);
    };
    const BladeElementRotorConfig c = anchor_rotor(0.0, {});
    const double gamma =
        1.225 * kLiftSlope * c.chord(0.5) * std::pow(c.radius, 4) / c.flap->inertia;
    for (const double d3 : {15 * kDeg, 30 * kDeg, -20 * kDeg}) {
        const double expect =
            -c.flap->inertia * c.rotor_speed * c.rotor_speed * gamma / 8.0 * std::tan(d3);
        EXPECT_NEAR(slope(d3) - slope(0.0), expect, 0.01 * std::abs(expect)) << d3 / kDeg;
    }
}

// In hover the in-plane hub force is the thrust tilted with the disc:
// C_x = C_T β1c and C_y = −C_T β1s (Padfield 3rd ed. Eqs. 3.100–3.101,
// e = 0). Mean over a revolution, including the blade inertial forces.
TEST(FlapAnchors, HoverHubForceFollowsDiscTilt) {
    BladeElementRotorConfig c = anchor_rotor(0.0, {10 * kDeg, -2 * kDeg, 1.5 * kDeg});
    c.root_cutout = 0.0;
    c.hub_position_b = Vec3::Zero();
    const BladeElementRotor rotor(c);
    SimContext ctx = test::sea_level_context();
    std::vector<double> x(rotor.num_states(), 0.0);
    const test::Harmonics hm = test::periodic_flapping(rotor, ctx, x, 12, 1);
    Vec3 F = Vec3::Zero();
    double T = 0.0;
    int n = 0;
    std::vector<double> xd(8);
    std::vector<double> G(48);
    test::whirl(rotor, ctx, x, 1e-3, 233, [&](double t, const std::vector<double>& xs) {
        SimContext cx = ctx;
        cx.t = t;
        ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
        rotor.derivatives(cx, xs, d);
        F += d.load0_b.F;
        T += rotor.evaluate(cx, xs).thrust;
        ++n;
    });
    F /= n;
    T /= n;
    RecordProperty("Fx_over_T", test::fmt17(F.x() / T));
    RecordProperty("beta1c", test::fmt17(hm.b1c));
    const double tol = 0.03 * std::hypot(hm.b1c, hm.b1s);
    EXPECT_NEAR(F.x() / T, hm.b1c, tol);
    EXPECT_NEAR(F.y() / T, -hm.b1s, tol);
}

}  // namespace
}  // namespace eyas
