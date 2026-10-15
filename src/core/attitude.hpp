#pragma once

#include "core/types.hpp"

// Euler angles for output and for setting up states (spec §4: attitude is a
// quaternion; trim, D-035, builds one from θ and φ).
namespace eyas {

// Padfield's yaw–pitch–roll Euler angles (3rd ed. Eqs. 3A.34–3A.37).
struct EulerAngles {
    double roll = 0.0;     // φ   [rad] (−π, π]
    double pitch = 0.0;    // θ   [rad] [−π/2, π/2]
    double heading = 0.0;  // ψ_h [rad] [0, 2π)
};

// Extracts the angles from C_bn = Γ = Φ Θ Ψ (Padfield 3rd ed. Eq. 3A.40):
//   θ = −asin Γ₁₃,  φ = atan2(Γ₂₃, Γ₃₃),  ψ_h = atan2(Γ₁₂, Γ₁₁).
// At θ = ±π/2 roll and heading are not separable; the result is finite.
EulerAngles euler_angles(const Quat& q_nb) noexcept;

// The attitude with these angles, the inverse of euler_angles(): q_nb maps
// body to NED, C_nb = Γᵀ = Ψᵀ Θᵀ Φᵀ (Padfield 3rd ed. Eqs. 3A.34–3A.40), i.e.
// heading about z, then pitch about the new y, then roll about x. Unit.
Quat quaternion(const EulerAngles& e) noexcept;

}  // namespace eyas
