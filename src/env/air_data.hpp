#pragma once

// Air-data relations (D-043): what a pitot-static system with no position or
// instrument error would read. Gracey, NASA RP-1046 (1980), Ch. III. Called
// from advance() through Model's outputs: no allocation, no exceptions.
namespace eyas {

// Impact pressure q_c = p_t − p [Pa] of true airspeed V [m/s] in air of static
// pressure P [Pa] and density ρ [kg/m³], subsonic (V ≤ a): Gracey Eq. 3.14,
//   q_c = P [(1 + (γ−1)/(2γ) (ρ/P) V²)^(γ/(γ−1)) − 1].
double impact_pressure(double pressure, double density, double true_airspeed) noexcept;

// Calibrated airspeed V_c [m/s] of impact pressure q_c [Pa]: Gracey Eq. 3.16
// (standard sea-level P0, ρ0) solved for V_c,
//   V_c = √(2γ/(γ−1) (P0/ρ0) [(q_c/P0 + 1)^((γ−1)/γ) − 1]).
// Eq. 3.16 holds for V_c ≤ a0 (661 kt); above it Eq. 3.17 applies, which Eyas
// doesn't need. q_c < 0 gives 0.
double calibrated_airspeed(double impact_pressure) noexcept;

// Equivalent airspeed V_e = V √(ρ/ρ0) [m/s] (Gracey Eq. 3.20; ρ0 from
// isa::kSeaLevelDensity, as D-040).
double equivalent_airspeed(double density, double true_airspeed) noexcept;

}  // namespace eyas
