#include "env/atmosphere.hpp"

#include <cmath>

#include "util/units.hpp"

namespace eyas {

double geopotential_altitude(double altitude) noexcept {
    return isa::kEarthRadius * altitude / (isa::kEarthRadius + altitude);  // USSA Eq. 18
}

bool isa_out_of_range(double altitude) noexcept {
    const double H = geopotential_altitude(altitude);
    // Written so that a NaN altitude counts as out of range.
    // NOLINTNEXTLINE(readability-simplify-boolean-expr)
    return !(H >= isa::kMinAltitude && H <= isa::kMaxAltitude);
}

bool isa_delta_T_valid(double delta_T) noexcept {
    // Same operations as the tropopause temperature T1 in isa_atmosphere(), so
    // the two agree to the last bit.
    const double T0 = isa::kSeaLevelTemperature + delta_T;
    return std::isfinite(delta_T) && T0 + isa::kLapseRate * isa::kTropopause > 0.0;
}

AirState isa_atmosphere(double altitude, double delta_T) noexcept {
    using namespace isa;
    // g0' M0 / R*  [K/m']; g0' is numerically g0 (USSA Table 2, p. 3).
    constexpr double k = units::kStandardGravity * kMolecularWeight / kGasConstant;

    AirState air;
    air.altitude = altitude;
    const double H = geopotential_altitude(altitude);
    air.geopotential_altitude = H;
    air.out_of_range = isa_out_of_range(altitude);

    // Hydrostatic integration in geopotential altitude, g0' dH = g dZ (USSA
    // Eq. 16), through the profile T(H) = T_std(H) + ΔT. With a constant
    // gradient L this gives Eq. 33a, P = Pb [Tb / (Tb + L (H − Hb))]^(k/L), and
    // with L = 0 Eq. 33b, P = Pb exp(−k (H − Hb) / Tb), where Tb includes ΔT.
    const double T0 = kSeaLevelTemperature + delta_T;
    if (H <= kTropopause) {
        air.temperature = T0 + kLapseRate * H;
        air.pressure = kSeaLevelPressure * std::pow(T0 / air.temperature, k / kLapseRate);
    } else {
        const double T1 = T0 + kLapseRate * kTropopause;
        const double P1 = kSeaLevelPressure * std::pow(T0 / T1, k / kLapseRate);
        air.temperature = T1;
        air.pressure = P1 * std::exp(-k * (H - kTropopause) / T1);
    }
    air.density = air.pressure * kMolecularWeight / (kGasConstant * air.temperature);  // Eq. 42
    air.speed_of_sound =
        std::sqrt(kGamma * kGasConstant * air.temperature / kMolecularWeight);  // Eq. 50
    return air;
}

}  // namespace eyas
