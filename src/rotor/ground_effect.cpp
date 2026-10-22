#include "rotor/ground_effect.hpp"

#include <algorithm>
#include <cmath>

namespace eyas {

double ground_effect_cos_chi(double lambda0, double mu) noexcept {
    const double v = std::sqrt(mu * mu + lambda0 * lambda0);
    return v > 0.0 ? std::abs(lambda0) / v : 1.0;
}

double ground_effect_factor(GroundEffectModel model, double height, double cos_chi) noexcept {
    if (model == GroundEffectModel::none || std::isnan(height)) {
        return 1.0;
    }
    const double h = std::max(height, kGroundEffectMinHeight);
    if (model == GroundEffectModel::genhel) {
        // CR-166309 Eq. 5.8.2-1 (p. 5.8-3).
        return std::pow(1.0 + 0.115 / (h * h) * cos_chi, -2.0 / 3.0);
    }
    // Johnson 2013 Eq. 4.71 with z/cos χ for z (Eq. 5.76): f_g = 1/(1 − q),
    // q = (R cos χ / 4z)², and κ_g = f_g^(−3/2) (§4.8). q ≤ (1/1.2)² < 1.
    const double s = cos_chi / (4.0 * h);
    const double q = s * s;
    return std::pow(1.0 - q, 1.5);
}

}  // namespace eyas
