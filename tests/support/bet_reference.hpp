#pragma once

// Exact-equation reference for the M3 blade-element rotor (D-007 layer 1).
//
// Deliberately simple and slow, and written independently of
// src/rotor/blade_element_rotor.cpp:
// - element velocities from scalar expressions derived in the rotating frame,
//   not from the production vector algebra;
// - lift and drag resolved with Leishman 2nd ed. Eq. 3.5 using cos φ and sin φ
//   of φ = atan2(U_P, U_T), not the production vector form;
// - moments as generic cross products r × dF;
// - span integrals by adaptive Gauss–Kronrod (7–15) quadrature instead of
//   midpoint-rule elements;
// - the uniform inflow by bisection to machine precision instead of the
//   production secant iteration.
// It shares only the conventions (frames, azimuth, the airfoil table) with
// the production code.

#include <array>
#include <cmath>
#include <functional>
#include <numbers>

#include "core/types.hpp"
#include "rotor/airfoil.hpp"
#include "rotor/controls.hpp"

namespace eyas::test {

struct RefRotor {
    int num_blades = 4;
    double radius = 0.0;                  // R [m]
    double omega = 0.0;                   // Ω [rad/s]
    double s = 1.0;                       // +1 counter-clockwise from above, −1 clockwise
    double root_cutout = 0.0;             // [m]
    double tip_loss = 1.0;                // B
    std::function<double(double)> chord;  // [m] against r/R
    std::function<double(double)> twist;  // [rad] against r/R
    const Airfoil* airfoil = nullptr;
};

struct RefCondition {
    Vec3 v_hub_h = Vec3::Zero();  // [m/s] hub velocity relative to the air, hub axes
    Vec3 omega_h = Vec3::Zero();  // [rad/s] body rate, hub axes
    double psi = 0.0;             // [rad] azimuth of blade 1
    SwashplatePitch pitch;
    double rho = 1.225;  // [kg/m³]
    double a = 340.294;  // [m/s]
};

struct RefLoads {
    Vec3 F_h = Vec3::Zero();
    Vec3 M_h = Vec3::Zero();
    double thrust = 0.0;
    double torque = 0.0;
    double ct = 0.0;
    double cq = 0.0;
    double lambda_i = 0.0;
};

namespace detail {

using Vec6r = std::array<double, 6>;  // F_h (3), M_h (3) per unit span

// Gauss–Kronrod 7–15 nodes and weights on [−1, 1] (standard QUADPACK values).
inline constexpr std::array<double, 8> kXgk{
    0.991455371120812639206854697526329, 0.949107912342758524526189684047851,
    0.864864423359769072789712788640926, 0.741531185599394439863864773280788,
    0.586087235467691130294144845693013, 0.405845151377397166906606412076961,
    0.207784955007898467600689403773245, 0.000000000000000000000000000000000};
inline constexpr std::array<double, 8> kWgk{
    0.022935322010529224963732008058970, 0.063092092629978553290700663189204,
    0.104790010322250183839876322541518, 0.140653259715525918745189590510238,
    0.169004726639267902826583426598550, 0.190350578064785409913256402421014,
    0.204432940075298892414161999234649, 0.209482141084727828012999174891714};
inline constexpr std::array<double, 4> kWg{
    0.129484966168869693270611432679082, 0.279705391489276667901467771423780,
    0.381830050505118944950369775488975, 0.417959183673469387755102040816327};

template <class F>
void gk15(const F& f, double a, double b, Vec6r& kronrod, double& err) {
    const double c = 0.5 * (a + b);
    const double h = 0.5 * (b - a);
    Vec6r k{};
    Vec6r g{};
    const Vec6r fc = f(c);
    for (std::size_t i = 0; i < 6; ++i) {
        k[i] = kWgk[7] * fc[i];
        g[i] = kWg[3] * fc[i];
    }
    for (std::size_t j = 0; j < 7; ++j) {
        const Vec6r f1 = f(c - h * kXgk[j]);
        const Vec6r f2 = f(c + h * kXgk[j]);
        for (std::size_t i = 0; i < 6; ++i) {
            k[i] += kWgk[j] * (f1[i] + f2[i]);
            if (j % 2 == 1) {
                g[i] += kWg[j / 2] * (f1[i] + f2[i]);
            }
        }
    }
    err = 0.0;
    for (std::size_t i = 0; i < 6; ++i) {
        kronrod[i] = h * k[i];
        err = std::max(err, std::abs(h * (k[i] - g[i])));
    }
}

template <class F>
void adaptive(const F& f, double a, double b, double tol, int depth, Vec6r& sum) {
    Vec6r v{};
    double err = 0.0;
    gk15(f, a, b, v, err);
    if (err <= tol || depth >= 40) {
        for (std::size_t i = 0; i < 6; ++i) {
            sum[i] += v[i];
        }
        return;
    }
    const double m = 0.5 * (a + b);
    adaptive(f, a, m, 0.5 * tol, depth + 1, sum);
    adaptive(f, m, b, 0.5 * tol, depth + 1, sum);
}

}  // namespace detail

// Loads at a given uniform induced inflow λ_i [-].
inline RefLoads reference_loads_at(const RefRotor& rotor, const RefCondition& cond,
                                   double lambda_i) {
    using detail::Vec6r;
    const double R = rotor.radius;
    const double W = rotor.omega;
    const double s = rotor.s;
    const double u = cond.v_hub_h.x();
    const double v = cond.v_hub_h.y();
    const double w = cond.v_hub_h.z();
    const double p = cond.omega_h.x();
    const double q = cond.omega_h.y();
    const double r_rate = cond.omega_h.z();
    const double v_i = lambda_i * W * R;

    RefLoads out;
    double thrust = 0.0;
    double torque = 0.0;
    for (int k = 0; k < rotor.num_blades; ++k) {
        const double psi =
            cond.psi - 2.0 * std::numbers::pi * k / static_cast<double>(rotor.num_blades);
        const double cp = std::cos(psi);
        const double sp = std::sin(psi);
        const Vec3 e_r(-cp, s * sp, 0.0);
        const Vec3 e_t(sp, s * cp, 0.0);
        const Vec3 e_z(0.0, 0.0, 1.0);
        const double theta_c =
            cond.pitch.theta0 + cond.pitch.theta1c * cp + cond.pitch.theta1s * sp;

        // Per unit span: F_h (3), M_h (3); plus thrust and torque tallies
        // (Leishman 2nd ed. Eq. 3.6) carried in separate integrals below.
        const auto section = [&](double r, bool lifting, double& f_up, double& f_x) {
            // In-plane and normal velocities from the rotating-frame scalar
            // derivation (D-024 conventions): U_T = u sin ψ + s v cos ψ +
            // (Ω − s r_b) r and U_P = −w + v_i − r (s p sin ψ + q cos ψ).
            const double u_t = u * sp + s * v * cp + (W - s * r_rate) * r;
            const double u_p = -w + v_i - r * (s * p * sp + q * cp);
            const double phi = std::atan2(u_p, u_t);
            const double alpha =
                std::remainder(theta_c + rotor.twist(r / R) - phi, 2.0 * std::numbers::pi);
            const double u2 = u_t * u_t + u_p * u_p;
            TableCursor<2> cur;
            const AirfoilCoefficients c = (*rotor.airfoil)(alpha, std::sqrt(u2) / cond.a, cur);
            const double cl = lifting ? c.cl : 0.0;
            // Leishman 2nd ed. Eq. 3.4
            const double dl = 0.5 * cond.rho * u2 * rotor.chord(r / R) * cl;
            const double dd = 0.5 * cond.rho * u2 * rotor.chord(r / R) * c.cd;
            f_up = dl * std::cos(phi) - dd * std::sin(phi);  // Leishman 2nd ed. Eq. 3.5
            f_x = dl * std::sin(phi) + dd * std::cos(phi);
        };
        const auto load = [&](bool lifting) {
            return [&, lifting](double r) {
                double f_up = 0.0;
                double f_x = 0.0;
                section(r, lifting, f_up, f_x);
                const Vec3 F = -f_x * e_t - f_up * e_z;  // f_x opposes rotation; z down
                const Vec3 M = (r * e_r).cross(F);
                return Vec6r{F.x(), F.y(), F.z(), M.x(), M.y(), M.z()};
            };
        };
        const auto tq = [&](bool lifting) {
            return [&, lifting](double r) {
                double f_up = 0.0;
                double f_x = 0.0;
                section(r, lifting, f_up, f_x);
                return Vec6r{f_up, r * f_x, 0.0, 0.0, 0.0, 0.0};
            };
        };
        const double tol = 1e-12 * cond.rho * std::pow(W * R, 2) * R * R;
        Vec6r fm{};
        Vec6r tt{};
        const double r_lift = rotor.tip_loss * R;
        detail::adaptive(load(true), rotor.root_cutout, r_lift, tol, 0, fm);
        detail::adaptive(tq(true), rotor.root_cutout, r_lift, tol, 0, tt);
        if (rotor.tip_loss < 1.0) {
            detail::adaptive(load(false), r_lift, R, tol, 0, fm);
            detail::adaptive(tq(false), r_lift, R, tol, 0, tt);
        }
        out.F_h += Vec3(fm[0], fm[1], fm[2]);
        out.M_h += Vec3(fm[3], fm[4], fm[5]);
        thrust += tt[0];
        torque += tt[1];
    }
    const double q_disk = cond.rho * std::numbers::pi * R * R * std::pow(W * R, 2);
    out.thrust = thrust;
    out.torque = torque;
    out.ct = thrust / q_disk;
    out.cq = torque / (q_disk * R);
    out.lambda_i = lambda_i;
    return out;
}

// Loads with λ_i solved from Padfield 3rd ed. Eq. 3.139 by bisection on
// g(λ) = 2λ√(μ² + (λ − μ_z)²) − C_T(λ) over a bracket found by expanding
// [−0.5, 0.5].
inline RefLoads reference_loads(const RefRotor& rotor, const RefCondition& cond) {
    const double vr = rotor.omega * rotor.radius;
    const double mu = std::hypot(cond.v_hub_h.x(), cond.v_hub_h.y()) / vr;
    const double mu_z = cond.v_hub_h.z() / vr;
    const auto g = [&](double lam) {
        const double d = lam - mu_z;
        return 2.0 * lam * std::sqrt(mu * mu + d * d) - reference_loads_at(rotor, cond, lam).ct;
    };
    double lo = -0.5;
    double hi = 0.5;
    while (g(lo) > 0.0) {
        lo *= 2.0;
    }
    while (g(hi) < 0.0) {
        hi *= 2.0;
    }
    for (int i = 0; i < 200 && hi - lo > 0.0; ++i) {
        const double m = 0.5 * (lo + hi);
        if (m <= lo || m >= hi) {
            break;
        }
        (g(m) < 0.0 ? lo : hi) = m;
    }
    return reference_loads_at(rotor, cond, 0.5 * (lo + hi));
}

}  // namespace eyas::test
