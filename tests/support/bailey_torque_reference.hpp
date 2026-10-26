#pragma once

// Layer-1 reference for the Bailey tail-rotor torque (D-058): Bailey's
// assumptions (NACA Report 716 pp. 209–213) integrated numerically, with none
// of the closed form's truncation. Blades hinged on the axis, no cyclic
// pitch, lift c_l = aα inboard of BR, drag c_d = δ₀ + δ₁α + δ₂α² out to the
// tip, first-harmonic flapping β = a₀ − a₁cos ψ − b₁sin ψ, the weight moment
// dropped. All speeds over ΩR, x = r/R, ψ from downwind:
//   u_T = x + μ sin ψ,  u_P = λ − x β′ − μ β cos ψ,  θ = θ₀ + x θ_tw.
// Small angles: the lift per span ∝ |u_T|(u_P + u_T θ), normal to the disk;
// its in-plane part, which drives the rotor, ∝ sgn(u_T)(u_P² + u_T u_P θ),
// so the reversed-velocity region (u_T < 0) is kept everywhere. The drag
// opposes the flow, ∝ sgn(u_T) u_T² c_d, with Bailey's α′ = −α where the flow
// is reversed (p. 213). The flapping solves β″ + β = (γ/2)∫₀^B x · lift by
// harmonic balance. The closed form keeps terms to μ², so the two agree to
// O(μ³).
//
// Quadrature: Gauss–Legendre on each smooth piece, x split at the reversed
// region's edge −μ sin ψ and ψ at π, so the integrals are exact to rounding
// for these piecewise polynomials and smooth trigonometric pieces.

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include <Eigen/Dense>

namespace eyas::test {

struct BaileyTorqueCase {
    double B = 1.0;       // tip loss [-]
    double gamma = 0.0;   // Lock number ρacR⁴/I₁ [-]
    double mu = 0.0;      // advance ratio [-]
    double lambda = 0.0;  // inflow, positive up through the disk [-]
    double theta = 0.0;   // pitch at the root [rad]
    double twist = 0.0;   // linear twist, root to tip [rad]
    double d0 = 0.0;      // δ₀ [-]
    double d1 = 0.0;      // δ₁ [1/rad]
    double d2 = 0.0;      // δ₂ [1/rad²]
};

struct BaileyTorqueResult {
    double a0 = 0.0, a1 = 0.0, b1 = 0.0;  // flapping [rad]
    double accelerating = 0.0;            // (1/a)(2C_Qa/σ)
    double decelerating = 0.0;            // 2C_Qd/σ
    // C_Q = (σ/2)(decelerating − a accelerating).
    double cq(double sigma, double a) const {
        return sigma / 2.0 * (decelerating - a * accelerating);
    }
};

namespace bailey_detail {

constexpr int kNodes = 24;

// Gauss–Legendre nodes and weights on [−1, 1] by Newton's method on P_n.
inline const std::array<std::array<double, 2>, kNodes>& gauss_legendre() {
    static const auto rule = [] {
        std::array<std::array<double, 2>, kNodes> r{};
        for (int i = 0; i < kNodes; ++i) {
            double z = std::cos(std::numbers::pi * (i + 0.75) / (kNodes + 0.5));
            double dp = 0.0;
            for (int it = 0; it < 100; ++it) {
                double p0 = 1.0;
                double p1 = z;
                for (int k = 2; k <= kNodes; ++k) {
                    const double p2 = ((2.0 * k - 1.0) * z * p1 - (k - 1.0) * p0) / k;
                    p0 = p1;
                    p1 = p2;
                }
                dp = kNodes * (z * p1 - p0) / (z * z - 1.0);
                const double dz = p1 / dp;
                z -= dz;
                if (std::abs(dz) < 1e-16) {
                    break;
                }
            }
            r[static_cast<std::size_t>(i)] = {z, 2.0 / ((1.0 - z * z) * dp * dp)};
        }
        return r;
    }();
    return rule;
}

// ∫_lo^hi f(t) dt.
template <class F>
double integrate(double lo, double hi, const F& f) {
    if (!(hi > lo)) {
        return 0.0;
    }
    const double h = 0.5 * (hi - lo);
    const double m = 0.5 * (hi + lo);
    double s = 0.0;
    for (const auto& [z, w] : gauss_legendre()) {
        s += w * f(m + h * z);
    }
    return h * s;
}

// ∫_0^end f(x) dx split at the reversed region's edge x_b = −μ sin ψ.
template <class F>
double radial(double mu, double psi, double end, const F& f) {
    const double xb = std::clamp(-mu * std::sin(psi), 0.0, end);
    return integrate(0.0, xb, f) + integrate(xb, end, f);
}

// (1/2π)∫_0^2π g(ψ) dψ, split at π (where the reversed region starts).
template <class G>
double mean(const G& g) {
    return (integrate(0.0, std::numbers::pi, g) +
            integrate(std::numbers::pi, 2.0 * std::numbers::pi, g)) /
           (2.0 * std::numbers::pi);
}

}  // namespace bailey_detail

inline BaileyTorqueResult bailey_torque_reference(const BaileyTorqueCase& c) {
    using namespace bailey_detail;
    const auto speeds = [&](const Eigen::Vector3d& f, double x, double psi, double& ut,
                            double& up) {
        const double beta = f(0) - f(1) * std::cos(psi) - f(2) * std::sin(psi);
        const double dbeta = f(1) * std::sin(psi) - f(2) * std::cos(psi);
        ut = x + c.mu * std::sin(psi);
        up = c.lambda - x * dbeta - c.mu * beta * std::cos(psi);
    };
    // Flapping moment over IΩ² at azimuth ψ, (γ/2)∫₀^B x|u_T|(u_P + u_T θ) dx.
    const auto moment = [&](const Eigen::Vector3d& f, double psi) {
        return 0.5 * c.gamma * radial(c.mu, psi, c.B, [&](double x) {
                   double ut = 0.0;
                   double up = 0.0;
                   speeds(f, x, psi, ut, up);
                   return x * std::abs(ut) * (up + ut * (c.theta + x * c.twist));
               });
    };
    // Harmonic balance, affine in (a₀, a₁, b₁): mean(m) − a₀ = 0 and m's first
    // harmonics zero.
    const auto residual = [&](const Eigen::Vector3d& f) {
        return Eigen::Vector3d(mean([&](double p) { return moment(f, p); }) - f(0),
                               mean([&](double p) { return moment(f, p) * std::cos(p); }),
                               mean([&](double p) { return moment(f, p) * std::sin(p); }));
    };
    const Eigen::Vector3d r0 = residual(Eigen::Vector3d::Zero());
    Eigen::Matrix3d J;
    for (int j = 0; j < 3; ++j) {
        J.col(j) = residual(Eigen::Vector3d::Unit(j)) - r0;
    }
    const Eigen::Vector3d f = J.fullPivLu().solve(-r0);

    BaileyTorqueResult out;
    out.a0 = f(0);
    out.a1 = f(1);
    out.b1 = f(2);
    out.accelerating = mean([&](double psi) {
        return radial(c.mu, psi, c.B, [&](double x) {
            double ut = 0.0;
            double up = 0.0;
            speeds(f, x, psi, ut, up);
            const double sgn = ut < 0.0 ? -1.0 : 1.0;
            return x * sgn * (up * up + ut * up * (c.theta + x * c.twist));
        });
    });
    out.decelerating = mean([&](double psi) {
        return radial(c.mu, psi, 1.0, [&](double x) {
            double ut = 0.0;
            double up = 0.0;
            speeds(f, x, psi, ut, up);
            const double sgn = ut < 0.0 ? -1.0 : 1.0;
            const double wn = up + ut * (c.theta + x * c.twist);  // u_T α
            // sgn u_T² c_d(sgn α): δ₀ and δ₂ flip with the flow, δ₁ doesn't.
            return x * (sgn * c.d0 * ut * ut + c.d1 * ut * wn + sgn * c.d2 * wn * wn);
        });
    });
    return out;
}

}  // namespace eyas::test
