#pragma once

#include "core/component.hpp"

// Gyroscopic moment of parts spinning at constant speed relative to the
// airframe (engine spools, shafts, a rotor hub; GenHel CR-166309 p. 5.10-4,
// D-048). Their angular momentum relative to the body, H_b = Σ J_i Ω_i along
// each part's axis, is constant, so the moment on the airframe is
//   M_b = −ω_b × H_b,
// GenHel's L_GY = H_Y r − H_Z q, M_GY = −H_X r + H_Z p, N_GY = H_X q − H_Y p.
// The parts' mass and inertia are the airframe's; only their spin is added.
// Selectable from config as `spinning_parts`.
namespace eyas {

class SpinningParts : public ForceModel {
public:
    // H_b [kg·m²/s], body axes. Throws InitError unless finite.
    explicit SpinningParts(Vec3 angular_momentum_b);

    // F_b = 0; M_b = −ω_b × H_b [N·m].
    Wrench compute(const SimContext& ctx) const override;

    const Vec3& angular_momentum_b() const noexcept { return h_b_; }

private:
    Vec3 h_b_;
};

}  // namespace eyas
