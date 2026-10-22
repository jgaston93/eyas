#include "rotor/pitt_peters.hpp"

#include <cmath>
#include <numbers>

namespace eyas {
namespace {

constexpr double kSkew = 15.0 * std::numbers::pi / 64.0;  // Pitt & Peters 1980 Table 4

}  // namespace

WakeSkew wake_skew(double mu, double lambda) noexcept {
    WakeSkew w;
    w.V = std::sqrt(mu * mu + lambda * lambda);
    // Johnson 2013 Eq. 11.132: tan(χ/2) = μ / (√(μ² + λ²) + |λ|).
    const double d = w.V + std::abs(lambda);
    w.X = d > 0.0 ? mu / d : 0.0;
    return w;
}

Mat3 pitt_peters_gain(double X) noexcept {
    Mat3 L;
    L << 0.5, 0.0, -kSkew * X, 0.0, 2.0 * (1.0 + X * X), 0.0, kSkew * X, 0.0, 2.0 * (1.0 - X * X);
    return L;
}

Vec3 pitt_peters_rates(const Vec3& lambda, const Vec3& loads, double mu, double mu_z, double cos_w,
                       double sin_w, double omega, const std::optional<VortexRingRow>& vrs,
                       double ground, double tip) noexcept {
    const WakeSkew w = wake_skew(mu, lambda.x() - mu_z);
    // To wind axes: (c, s)_w = (c cos ψ_w + s sin ψ_w, −c sin ψ_w + s cos ψ_w).
    const auto to_wind = [&](const Vec3& v) {
        return Vec3(v.x(), -v.z() * sin_w + v.y() * cos_w, v.z() * cos_w + v.y() * sin_w);
    };
    const Vec3 lw = to_wind(lambda);
    const Vec3 cw = to_wind(loads);
    // V L̂⁻¹ λ_w, with L̂ inverted in closed form: a 2×2 block coupling
    // (λ0, λ1c) and a scalar for λ1s. det > 0 for X in [0, 1].
    const Mat3 L = pitt_peters_gain(w.X);
    const double a = L(0, 0);
    const double b = L(0, 2);
    const double c = L(2, 0);
    const double d = L(2, 2);
    const double det = a * d - b * c;
    // The thrust row at the out-of-ground-effect inflow λ0/κ_g (D-050). Its
    // mass flow is V at λ0/κ_g: at κ_g = 1 the same operations as w.V, so the
    // same bits.
    const double l0 = lw.x() / ground;
    const double lt = l0 - mu_z;
    const double v_thrust = vrs && vrs->mass_flow ? *vrs->mass_flow : std::sqrt(mu * mu + lt * lt);
    // With tip loss (D-051) λ1c enters the effective disc's row times B: its
    // radial weighting is r/R_e = (r/R)/B while λ0 is scaled by 1/B.
    const double r0 = v_thrust * (d * l0 - b * (tip * lw.z())) / det;
    const double rc = w.V * (-c * lw.x() + a * lw.z()) / det;
    const double rs = w.V * lw.y() / L(1, 1);
    // M̃ dλ_w/dτ = C_w − V L̂⁻¹ λ_w, τ = Ω t; the thrust row over the disc of
    // radius tip·R (D-051), divided by tip².
    const double ct_disc = cw.x() / (tip * tip);
    const double ct = vrs ? ct_disc - vrs->deficit : ct_disc;
    const double d0 = omega * (ct - r0) / (tip * PittPetersMass::m0);
    const double ds = omega * (cw.y() - rs) / PittPetersMass::m1;
    const double dc = omega * (cw.z() - rc) / PittPetersMass::m1;
    // Back to hub axes.
    return {d0, ds * cos_w + dc * sin_w, dc * cos_w - ds * sin_w};
}

}  // namespace eyas
