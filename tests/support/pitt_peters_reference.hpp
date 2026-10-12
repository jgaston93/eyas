#pragma once

// Independent transcriptions of the Pitt–Peters equations for the M5 layer 1
// tests (D-027). Each is written in its source's own notation, state order
// and load signs, so agreement with the production form (src/rotor/
// pitt_peters.cpp) tests the sign reconciliation of SPEC_REVIEW #10.
//
// Both use the D-027 choices: the nonlinear total-state form, one mass-flow
// speed V = √(μ² + λ²) for every column, the moment apparent mass 16/45π
// (Pitt & Peters 1980 Table 3/4; Johnson 2013 p. 437), and wind axes aligned
// with the in-plane hub velocity, done here with complex numbers rather than
// rotation matrices.

#include <cmath>
#include <complex>
#include <numbers>

#include <Eigen/Dense>

namespace eyas::test {

// Inputs in hub axes. States (λ0, λ1s, λ1c) as in D-027; loads are given in
// each source's convention by the functions below.
struct PPInputs {
    double l0 = 0.0;
    double l1s = 0.0;
    double l1c = 0.0;
    double mu = 0.0;     // in-plane hub speed / ΩR
    double mu_z = 0.0;   // hub speed along z_h (down) / ΩR
    double psi_w = 0.0;  // [rad] downwind azimuth
    double omega = 1.0;  // Ω [rad/s]
};

struct PPRates {
    double l0 = 0.0;  // dλ0/dt [1/s]
    double l1s = 0.0;
    double l1c = 0.0;
};

// Pitt & Peters 1980 (6th ERF Paper 47), Table 4 (p. 47-13) and eq. 2a:
//   [M]{λ̇} + [L]⁻¹{λ} = {C_T, C_L, C_M},  (λ0, λs, λc), λ̇ = dλ/d(Ωt),
// L = (1/V) [[1/2, 0, (15π/64)√((1−sin α)/(1+sin α))],
//            [0, −4/(1+sin α), 0],
//            [(15π/64)√((1−sin α)/(1+sin α)), 0, −4 sin α/(1+sin α)]],
// M = diag(128/75π, −16/45π, −16/45π).
// C_L, C_M: eqs. 5b–5c (−r sin ψ, −r cos ψ weightings). α is the disc angle
// of attack, tan α = (λ̄ + ν̄)/μ (eq. 23c). Eq. 23c is signed, but the paper
// derives L only for 0° ≤ α ≤ 90° (p. 47-10); like production, this uses
// sin α = |λ|/V (Johnson 2013 Eq. 11.132), so it doesn't test that choice
// (D-027, "Upflow").
inline PPRates pitt_peters_1980(const PPInputs& in, double c_t, double c_l, double c_m) {
    const double pi = std::numbers::pi;
    const double lam = in.l0 - in.mu_z;
    const double V = std::sqrt(in.mu * in.mu + lam * lam);
    const double sa = V > 0.0 ? std::abs(lam) / V : 1.0;
    const double k = 15.0 * pi / 64.0 * std::sqrt((1.0 - sa) / (1.0 + sa));
    // L·V (so L⁻¹ = V (L·V)⁻¹ has no singularity at V = 0).
    Eigen::Matrix3d LV;
    LV << 0.5, 0.0, k, 0.0, -4.0 / (1.0 + sa), 0.0, k, 0.0, -4.0 * sa / (1.0 + sa);
    const Eigen::Vector3d Mdiag(128.0 / (75.0 * pi), -16.0 / (45.0 * pi), -16.0 / (45.0 * pi));

    // Wind axes: a first harmonic a cos ψ + b sin ψ is a_w cos(ψ − ψ_w) +
    // b_w sin(ψ − ψ_w) with a_w + i b_w = (a + i b) e^{−iψ_w}.
    const std::complex<double> rot = std::polar(1.0, -in.psi_w);
    const std::complex<double> lam_w = std::complex<double>(in.l1c, in.l1s) * rot;
    const std::complex<double> load_w = std::complex<double>(c_m, c_l) * rot;
    const Eigen::Vector3d x(in.l0, lam_w.imag(), lam_w.real());  // (λ0, λs, λc)
    const Eigen::Vector3d c(c_t, load_w.imag(), load_w.real());  // (C_T, C_L, C_M)
    const Eigen::Vector3d rhs = c - V * LV.inverse() * x;
    const Eigen::Vector3d xdot = in.omega * rhs.cwiseQuotient(Mdiag);
    // Back to hub axes.
    const std::complex<double> d1 = std::complex<double>(xdot(2), xdot(1)) / rot;
    return {xdot(0), d1.imag(), d1.real()};
}

// Johnson 2013 §11.3, Eqs. 11.142–11.143 with M from p. 437:
//   LM (λ̇0, λ̇c, λ̇s) + (λ0, λc, λs) = L (C_T, −C_My, C_Mx),
// L = (1/V) [[1/2, −(15π/64)√((1−cos χ)/(1+cos χ)), 0],
//            [(15π/64)√((1−cos χ)/(1+cos χ)), 4 cos χ/(1+cos χ), 0],
//            [0, 0, 4/(1+cos χ)]],
// M = diag(128/75π, 16/45π, 16/45π), cos χ = |λ|/√(μ² + λ²) (Eq. 11.132 ff.).
// −C_My and C_Mx: Eqs. 11.150–11.151 (+r cos ψ and +r sin ψ weightings).
inline PPRates johnson_2013(const PPInputs& in, double c_t, double minus_c_my, double c_mx) {
    const double pi = std::numbers::pi;
    const double lam = in.l0 - in.mu_z;
    const double V = std::sqrt(in.mu * in.mu + lam * lam);
    const double cx = V > 0.0 ? std::abs(lam) / V : 1.0;
    const double k = 15.0 * pi / 64.0 * std::sqrt((1.0 - cx) / (1.0 + cx));
    Eigen::Matrix3d LV;
    LV << 0.5, -k, 0.0, k, 4.0 * cx / (1.0 + cx), 0.0, 0.0, 0.0, 4.0 / (1.0 + cx);
    const Eigen::Matrix3d M =
        Eigen::Vector3d(128.0 / (75.0 * pi), 16.0 / (45.0 * pi), 16.0 / (45.0 * pi)).asDiagonal();

    const std::complex<double> rot = std::polar(1.0, -in.psi_w);
    const std::complex<double> lam_w = std::complex<double>(in.l1c, in.l1s) * rot;
    const std::complex<double> load_w = std::complex<double>(minus_c_my, c_mx) * rot;
    const Eigen::Vector3d x(in.l0, lam_w.real(), lam_w.imag());  // (λ0, λc, λs)
    const Eigen::Vector3d c(c_t, load_w.real(), load_w.imag());  // (C_T, −C_My, C_Mx)
    // L M λ̇ = L C − λ  ⇒  λ̇ = M⁻¹ (C − L⁻¹ λ), L⁻¹ = V (L V)⁻¹.
    const Eigen::Vector3d xdot = in.omega * M.inverse() * (c - V * LV.inverse() * x);
    const std::complex<double> d1 = std::complex<double>(xdot(1), xdot(2)) / rot;
    return {xdot(0), d1.imag(), d1.real()};
}

}  // namespace eyas::test
