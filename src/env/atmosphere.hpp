#pragma once

// ISA atmosphere (spec §6.2) per D-015: the U.S. Standard Atmosphere 1976
// (USSA; identical to the ICAO ISA up to 32 km), layers 0 and 1, evaluated on
// geopotential altitude, with a hydrostatic temperature offset. Called in every
// derivative evaluation (D-016): no allocation, no exceptions.
namespace eyas {

namespace isa {
// USSA Table 2 (book p. 2) unless noted. Table 2 misprints the exponents of R*
// and r0; the values here are from the text (D-015, DATA_GAPS.md).
inline constexpr double kSeaLevelTemperature = 288.15;  // T0 [K]
inline constexpr double kSeaLevelPressure = 101325.0;   // P0 [Pa]
inline constexpr double kEarthRadius = 6356766.0;       // r0 [m], text p. 8
inline constexpr double kGasConstant = 8.31432e3;       // R* [N·m/(kmol·K)], text p. 3
inline constexpr double kMolecularWeight = 28.9644;     // M0 [kg/kmol], p. 9
inline constexpr double kGamma = 1.40;                  // [-]
// ρ0 [kg/m³], from the sea-level constants through the equation of state
// (Eq. 42): 1.2250 kg/m³, the standard's tabulated value.
inline constexpr double kSeaLevelDensity =
    kSeaLevelPressure * kMolecularWeight / (kGasConstant * kSeaLevelTemperature);
// USSA Table 4 (p. 3): base heights and gradients of layers 0 and 1.
inline constexpr double kTropopause = 11000.0;  // H1 [m']
inline constexpr double kLapseRate = -6.5e-3;   // L_M,0 [K/m']; L_M,1 = 0
// Range covered by layers 0 and 1 (D-015; USSA p. 12 allows −5 km').
inline constexpr double kMinAltitude = -5000.0;  // [m']
inline constexpr double kMaxAltitude = 20000.0;  // [m']
}  // namespace isa

// Atmosphere at one point.
struct AirState {
    double altitude = 0.0;               // [m]  geometric, above MSL
    double geopotential_altitude = 0.0;  // [m'] USSA Eq. 18
    double temperature = 0.0;            // [K]
    double pressure = 0.0;               // [Pa]
    double density = 0.0;                // [kg/m^3]
    double speed_of_sound = 0.0;         // [m/s]
    bool out_of_range = false;           // H outside [−5, 20] km' (or NaN): profile extended
};

// True if geometric altitude Z [m above MSL] is outside the range of layers 0
// and 1, H in [−5, 20] km' (D-015), or NaN. Same test as AirState::out_of_range.
bool isa_out_of_range(double altitude) noexcept;

// True if ΔT [K] is finite and keeps the temperature positive everywhere in the
// profile: the coldest point is the isothermal layer at 216.65 K (USSA Table 4,
// p. 3), so ΔT > −216.65 K (D-019).
bool isa_delta_T_valid(double delta_T) noexcept;

// Geopotential altitude H [m'] from geometric altitude Z [m]:
// H = r0 Z / (r0 + Z) (USSA Eq. 18, Γ = 1 m'/m).
double geopotential_altitude(double altitude) noexcept;

// Atmosphere at geometric altitude Z [m above MSL] on an ISA + ΔT day [K].
// ΔT shifts the temperature profile; sea-level pressure stays P0 and pressure
// follows the hydrostatic equation through the shifted profile (D-015). With
// ΔT = 0 this is USSA Eqs. 33a/33b (pressure), 42 (density) and 50 (speed of
// sound), with T_M = T below 80 km (p. 9).
AirState isa_atmosphere(double altitude, double delta_T) noexcept;

// Pressure altitude [m'] of static pressure P [Pa]: the geopotential altitude
// at which the standard atmosphere (ΔT = 0) has pressure P. Altimeters are
// calibrated in geopotential altitude (Gracey, NASA RP-1046, p. 12). The
// inverse of USSA Eqs. 33a (layer 0) and 33b (layer 1), extended beyond them
// as isa_atmosphere() is; NaN for P ≤ 0 or NaN.
double pressure_altitude(double pressure) noexcept;

}  // namespace eyas
