#pragma once

#include <cstdint>

// Ground effect on the main rotor's mean induced inflow (M9, D-050; spec
// §6.5). Near the ground the induced inflow is κ_g times its value out of
// ground effect at the same thrust and speeds, κ_g ≤ 1: the inflow equations
// are solved for λ0/κ_g (Johnson's κ in v = κ(v_base + f Δv_vrs), NASA/TP-
// 2005-213477 p. 14; D-049). Pure functions, called from advance(): no
// allocation, no exceptions.
namespace eyas {

enum class GroundEffectModel : std::uint8_t {
    none,               // κ_g = 1
    genhel,             // GenHel CR-166309 Eq. 5.8.2-1 (the UH-60A's)
    cheeseman_bennett,  // Johnson 2013 Eqs. 4.71 and 5.76, κ_g = f_g^(−3/2)
};

// The lowest hub height used, z/R [-] (D-050): Johnson 2013 §4.8 gives the
// empirical forms for z/R ≥ 0.3, and Cheeseman–Bennett's κ_g is zero at
// z/R = 1/4. Lower heights (the hub nearer the ground than the airframe
// allows on its wheels, about 0.45R on the UH-60A) are held here.
inline constexpr double kGroundEffectMinHeight = 0.3;

// Wake angle from the shaft, as cos χ = |λ0| / √(μ² + λ0²) [-], from the
// induced inflow λ0 and the in-plane speed μ (both / ΩR): Padfield 3rd ed.
// Eq. 3.218's (V/v_i)² as 1/cos²χ − 1. 1 when both are zero (hover). D-050:
// the induced inflow, not the total flow λ0 − μ_z, keeps λ0/κ_g rising with
// λ0, so the uniform inflow solve keeps a unique root.
double ground_effect_cos_chi(double lambda0, double mu) noexcept;

// κ_g [-] in (0, 1] for a hub height z/R [-] above the ground (NaN: no
// ground; held at kGroundEffectMinHeight or above) and cos χ in [0, 1]:
//   genhel:            [1 + 0.115 (R/z)² cos χ]^(−2/3)
//                      (CR-166309 p. 5.8-3; |λ|/μ_TOT read as cos χ, D-050),
//   cheeseman_bennett: [1 − (R/(4 z_w))²]^(3/2), z_w = z / cos χ
//                      (Johnson 2013 Eqs. 4.71, 5.76; κ_g = f_g^(−3/2), §4.8).
// GenHel switches its factor off above z = 5R; Eyas doesn't (D-050).
double ground_effect_factor(GroundEffectModel model, double height, double cos_chi) noexcept;

}  // namespace eyas
