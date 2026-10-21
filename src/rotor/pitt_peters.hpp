#pragma once

#include <numbers>
#include <optional>

#include "core/types.hpp"

// Pitt–Peters dynamic inflow (spec §6.5, D-027). Pure functions, called from
// advance(): no allocation, no exceptions.
//
// Notation (D-027). Inflow states λ = (λ0, λ1s, λ1c) [-], with the induced
// velocity λ(r, ψ) = λ0 + (r/R)(λ1s sin ψ + λ1c cos ψ), positive down along
// the shaft, ψ the blade azimuth (D-024). Loads C = (C_T, C_1s, C_1c) [-]:
// aerodynamic thrust and the azimuth-weighted moments of the aerodynamic
// force normal to the hub plane (positive up),
//   C_1s = Σ dF_up (r/R) sin ψ / (ρA(ΩR)²),  C_1c = Σ dF_up (r/R) cos ψ / (ρA(ΩR)²).
// Pitt & Peters 1980 (6th ERF Paper 47) eqs. 5b–5c have C_L = −C_1s and
// C_M = −C_1c.
namespace eyas {

// Apparent masses M̃ = diag(128/75π, 16/45π, 16/45π): Pitt & Peters 1980
// Table 4 (p. 47-13), signs absorbed by C_1s and C_1c; Johnson 2013 p. 437
// for the moment terms (not Eq. 11.146; SPEC_REVIEW #10).
struct PittPetersMass {
    static constexpr double m0 = 128.0 / (75.0 * std::numbers::pi);
    static constexpr double m1 = 16.0 / (45.0 * std::numbers::pi);
};

// Mass-flow speed and wake skew for a total flow λ = λ0 − μ_z down through
// the disc and an in-plane speed μ (both / ΩR):
//   V = √(μ² + λ²),  X = tan(χ/2) = μ / (V + |λ|)   (Johnson 2013 Eq. 11.132),
// with X = 0 when V = 0. |λ| keeps X in [0, 1] when the flow is up through
// the disc (D-027, "Upflow").
struct WakeSkew {
    double V = 0.0;  // [-]
    double X = 0.0;  // [-] in [0, 1]
};
WakeSkew wake_skew(double mu, double lambda) noexcept;

// L̂(X) in wind axes, for states (λ0, λ1s, λ1c) and loads (C_T, C_1s, C_1c),
// so that the steady inflow is λ = L̂ C / V (D-027):
//   [[1/2, 0, −(15π/64)X], [0, 2(1+X²), 0], [(15π/64)X, 0, 2(1−X²)]].
// Pitt & Peters 1980 Table 4 times V, with columns 2–3 negated and
// 1+sin α, sin α written through X (Johnson 2013 Eqs. 11.133–11.134).
Mat3 pitt_peters_gain(double X) noexcept;

// Vortex-ring correction of the thrust row (D-049): the row's mass flow
// V_T (the baseline curve's C_T,base/(2λ0) in place of V; V itself when
// empty, outside the region) and a thrust deficit D [-] subtracted from C_T
// (the lagged VRS increment).
struct VortexRingRow {
    std::optional<double> mass_flow;  // V_T [-]
    double deficit = 0.0;             // D [-]
};

// dλ/dt [1/s] in hub axes (D-027):
//   M̃ dλ_w/dτ + V L̂(X)⁻¹ λ_w = C_w,  τ = Ω t,
// solved in wind axes w and rotated back. mu, mu_z as RotorLoads; (cos_w,
// sin_w) is the downwind azimuth ψ_w (cos ψ_w = v_x/|v_xy|, sin ψ_w =
// −s v_y/|v_xy|; (1, 0) when |v_xy| = 0); omega = Ω [rad/s]. With vrs, the
// thrust row is M̃₀ dλ0/dτ + V_T (L̂⁻¹ λ_w)₀ = C_T − D (D-049); the moment
// rows and the wake skew X keep V. Without it the result is unchanged.
Vec3 pitt_peters_rates(const Vec3& lambda, const Vec3& loads, double mu, double mu_z, double cos_w,
                       double sin_w, double omega,
                       const std::optional<VortexRingRow>& vrs = std::nullopt) noexcept;

}  // namespace eyas
