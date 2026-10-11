#pragma once

// Synthetic flapping rotors and helpers for the M4 tests and validation cases
// (D-026). Test fixtures, not aircraft data.

#include <cmath>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "core/component.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "support/flap_reference.hpp"
#include "support/m3_fixtures.hpp"

namespace eyas::test {

inline constexpr double kDeg = std::numbers::pi / 180.0;

// C_l = C_d = 0 everywhere: a rotor in vacuum.
inline std::string zero_airfoil_csv() {
    return "# source: synthetic test airfoil, no aerodynamic force (tests only)\n"
           "alpha[deg],mach[1],cl[1],cd[1]\n"
           "-180,0,0,0\n180,0,0,0\n-180,1,0,0\n180,1,0,0\n";
}

// n beads of equal mass on [0, L] outboard of the hinge (midpoints): a
// uniform blade of mass m and length L.
inline Beads uniform_beads(double m, double L, int n) {
    Beads b;
    for (int i = 0; i < n; ++i) {
        b.rho.push_back(L * (i + 0.5) / n);
        b.mass.push_back(m / n);
    }
    return b;
}

// A non-uniform blade: heavier inboard (root fittings) and a tip weight, so
// S_β/m and I_β differ from a uniform blade's.
// `scale` multiplies every mass: 1 gives a light blade (≈ 9 kg) for the
// mechanics tests, 12 a blade of realistic Lock number (γ ≈ 8 on the test
// rotor) for the aerodynamic anchors.
inline Beads nonuniform_beads(double L, double scale = 1.0) {
    Beads b;
    const int n = 25;
    for (int i = 0; i < n; ++i) {
        const double x = (i + 0.5) / n;
        b.rho.push_back(L * x);
        b.mass.push_back(scale * 6.0 * (1.6 - x) / n);  // [kg]
    }
    b.rho.push_back(0.97 * L);  // tip weight
    b.mass.push_back(scale * 2.0);
    return b;
}

inline FlapHinge hinge_from(const Beads& b, double offset) {
    FlapHinge h;
    h.offset = offset;
    h.mass = b.m();
    h.first_moment = b.S();
    h.inertia = b.I();
    return h;
}

// The M3 test rotor (R = 8 m, Ω = 27 rad/s, 4 blades, c = 0.53 m) with a
// flap hinge at `offset` and the root cutout at the hinge.
inline BladeElementRotorConfig flap_rotor(std::shared_ptr<const Airfoil> airfoil,
                                          SwashplatePitch pitch, const Beads& beads, double offset,
                                          int elements = 20) {
    BladeElementRotorConfig c = test_rotor(std::move(airfoil), pitch, elements);
    c.root_cutout = std::max(offset, 0.0);
    c.flap = hinge_from(beads, offset);
    return c;
}

// Anchor fixture for the closed forms (D-026 layer 3): realistic Lock number
// (γ ≈ 8), linear airfoil without drag (C_l = 5.73 α), constant chord, linear
// twist −8°, lift from the hinge to the tip (B = 1), 100 elements.
inline constexpr double kLiftSlope = 5.73;  // [1/rad]
inline BladeElementRotorConfig anchor_flap_rotor(double e, SwashplatePitch pitch) {
    const auto lin = airfoil_from(linear_airfoil_csv(kLiftSlope, 0.0));
    BladeElementRotorConfig c = flap_rotor(lin, pitch, nonuniform_beads(8.0 - e, 12.0), e, 100);
    c.twist = Table1D(Axis({0.0, 1.0}), {0.0, -8.0 * kDeg});
    return c;
}

// Lock number about the hinge, γ = ρ a c R⁴ / I_β (Johnson 2013 Eq. 6.125),
// and the rotating flap frequency ν² = 1 + e S_β/I_β + K_β/(I_β Ω²)
// (Johnson 2013 Eq. 6.261, dimensional form).
inline double nu2(const BladeElementRotorConfig& c) {
    const FlapHinge& f = *c.flap;
    return 1.0 + f.offset * f.first_moment / f.inertia +
           f.spring / (f.inertia * c.rotor_speed * c.rotor_speed);
}

// Integrates the rotor states alone, with the body held in the motion given
// by ctx (a = 0: a whirl tower, or a hub in steady translation), by RK4 with
// step h [s]. ctx.t advances. `sample` is called after every step.
template <class Sample>
void whirl(const BladeElementRotor& rotor, SimContext& ctx, std::vector<double>& x, double h,
           int steps, Sample&& sample) {
    const std::size_t n = x.size();
    std::vector<double> xdot0(n);
    std::vector<double> G(n * 6);
    const auto f = [&](double t, const std::vector<double>& xs, std::vector<double>& out) {
        SimContext c = ctx;
        c.t = t;
        ComponentDerivs d{xdot0, G, Wrench{}, Mat66::Zero()};
        rotor.derivatives(c, xs, d);
        out = xdot0;
    };
    std::vector<double> k1(n);
    std::vector<double> k2(n);
    std::vector<double> k3(n);
    std::vector<double> k4(n);
    std::vector<double> tmp(n);
    for (int s = 0; s < steps; ++s) {
        const double t = ctx.t;
        f(t, x, k1);
        for (std::size_t i = 0; i < n; ++i) {
            tmp[i] = x[i] + 0.5 * h * k1[i];
        }
        f(t + 0.5 * h, tmp, k2);
        for (std::size_t i = 0; i < n; ++i) {
            tmp[i] = x[i] + 0.5 * h * k2[i];
        }
        f(t + 0.5 * h, tmp, k3);
        for (std::size_t i = 0; i < n; ++i) {
            tmp[i] = x[i] + h * k3[i];
        }
        f(t + h, tmp, k4);
        for (std::size_t i = 0; i < n; ++i) {
            x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        ctx.t = t + h;
        sample(ctx.t, x);
    }
}

// Least-squares fit β(ψ) = β0 + Σ_{n=1..4} (β_nc cos nψ + β_ns sin nψ).
struct Harmonics {
    double b0 = 0.0;
    double b1c = 0.0;
    double b1s = 0.0;
};
inline Harmonics fit_harmonics(const std::vector<double>& psi, const std::vector<double>& beta) {
    const auto m = static_cast<Eigen::Index>(psi.size());
    Eigen::MatrixXd A(m, 9);
    Eigen::VectorXd y(m);
    for (Eigen::Index i = 0; i < m; ++i) {
        const double p = psi[static_cast<std::size_t>(i)];
        A(i, 0) = 1.0;
        for (int k = 1; k <= 4; ++k) {
            A(i, 2 * static_cast<Eigen::Index>(k) - 1) = std::cos(k * p);
            A(i, 2 * static_cast<Eigen::Index>(k)) = std::sin(k * p);
        }
        y(i) = beta[static_cast<std::size_t>(i)];
    }
    const Eigen::VectorXd c = A.colPivHouseholderQr().solve(y);
    return {c(0), c(1), c(2)};
}

// Runs a whirl tower to a periodic state and fits blade 1 over the last
// `fit_revs` revolutions. h = 1 ms.
inline Harmonics periodic_flapping(const BladeElementRotor& rotor, SimContext ctx,
                                   std::vector<double>& x, int settle_revs = 25, int fit_revs = 4) {
    const double W = rotor.config().rotor_speed;
    const double h = 1e-3;
    const int per_rev = static_cast<int>(std::lround(2.0 * std::numbers::pi / W / h));
    whirl(rotor, ctx, x, h, settle_revs * per_rev, [](double, const std::vector<double>&) {});
    std::vector<double> psi;
    std::vector<double> beta;
    whirl(rotor, ctx, x, h, fit_revs * per_rev, [&](double t, const std::vector<double>& xs) {
        psi.push_back(rotor.azimuth(t));
        beta.push_back(xs[0]);
    });
    return fit_harmonics(psi, beta);
}

}  // namespace eyas::test
