#include "env/air_data.hpp"

#include <algorithm>
#include <cmath>

#include "env/atmosphere.hpp"

namespace eyas {

namespace {
constexpr double kGamma = isa::kGamma;
constexpr double kExponent = kGamma / (kGamma - 1.0);  // γ/(γ−1)
}  // namespace

double impact_pressure(double pressure, double density, double true_airspeed) noexcept {
    // Gracey Eq. 3.14
    const double x =
        (kGamma - 1.0) / (2.0 * kGamma) * density / pressure * true_airspeed * true_airspeed;
    return pressure * (std::pow(1.0 + x, kExponent) - 1.0);
}

double calibrated_airspeed(double impact_pressure) noexcept {
    using namespace isa;
    // Gracey Eq. 3.16 solved for V_c.
    const double qc = std::max(impact_pressure, 0.0);
    const double s = std::pow(qc / kSeaLevelPressure + 1.0, 1.0 / kExponent) - 1.0;
    return std::sqrt(2.0 * kExponent * kSeaLevelPressure / kSeaLevelDensity * s);
}

double equivalent_airspeed(double density, double true_airspeed) noexcept {
    return true_airspeed * std::sqrt(density / isa::kSeaLevelDensity);  // Gracey Eq. 3.20
}

}  // namespace eyas
