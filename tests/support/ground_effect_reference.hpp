#pragma once

// Independent transcription of the two ground-effect models (D-050), for the
// layer-1 gate. It shares no code with src/rotor/ground_effect.cpp and
// follows each source's own variables: GenHel's λ/μ_TOT and Johnson's thrust
// ratio f_g with z√(1 + (μ/λ)²) for z.

#include <algorithm>
#include <cmath>
#include <limits>

namespace eyas::test {

// GenHel CR-166309 Eq. 5.8.2-1 (p. 5.8-3): K_GE = [1 + .115 (R/Z)² (λ/μ_TOT)]^(−2/3),
// μ_TOT = (μ_xs² + μ_ys² + λ²)^(1/2), with λ the induced inflow, as a
// magnitude (D-050). z_over_r is held at 0.3 (D-050).
inline double genhel_kge(double z_over_r, double mu, double lambda) {
    const double z = std::max(z_over_r, 0.3);
    const double mu_tot = std::hypot(mu, lambda);
    const double ratio = mu_tot == 0.0 ? 1.0 : std::fabs(lambda) / mu_tot;
    return 1.0 / std::cbrt(std::pow(1.0 + 0.115 * ratio / (z * z), 2.0));
}

// Johnson 2013 Eq. 4.71, T/T∞ = f_g = [1 − 1/(4z/R)²]^(−1), with z replaced
// by z√(1 + (μ/λ)²) in forward flight (Eq. 5.76); the inflow ratio at
// constant thrust is κ_g = f_g^(−3/2) (§4.8). z_over_r held at 0.3 (D-050).
inline double cheeseman_bennett_kappa(double z_over_r, double mu, double lambda) {
    const double z = std::max(z_over_r, 0.3);
    double z_wake = z;
    if (mu != 0.0) {
        z_wake = lambda == 0.0 ? std::numeric_limits<double>::infinity()
                               : z * std::sqrt(1.0 + (mu / lambda) * (mu / lambda));
    }
    const double f = 1.0 / (1.0 - 1.0 / ((4.0 * z_wake) * (4.0 * z_wake)));
    return std::pow(f, -1.5);
}

}  // namespace eyas::test
