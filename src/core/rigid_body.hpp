#pragma once

#include <cstddef>
#include <span>

#include "core/types.hpp"
#include "util/units.hpp"

// Six-DOF rigid-body equations of the airframe (spec §6.1). The airframe
// excludes the rotor blades modeled as components (D-002).
namespace eyas {

// Mass properties of the airframe about its own CG, in body axes.
struct MassProperties {
    double mass = 0.0;              // [kg]
    Mat3 inertia_b = Mat3::Zero();  // [kg·m²] full tensor
};

// Builds the tensor from Padfield's moments and product of inertia (3rd ed.
// Eqs. 3A.29–3A.32: Ixz = ∫ x z dm). With that definition the off-diagonal
// entries are −Ixz, which reproduces Padfield Eq. 3A.33. Ixy = Iyz = 0
// (lateral symmetry). [kg], [kg·m²]
MassProperties make_mass_properties(double mass, double Ixx, double Iyy, double Izz, double Ixz);

// Gravity in NED, flat earth. [m/s²]
inline Vec3 gravity_n() {
    return {0.0, 0.0, units::kStandardGravity};
}

// Body states at the front of the global state vector.
namespace body_layout {
inline constexpr std::size_t kPos = 0;     // pos_n   (3)
inline constexpr std::size_t kVel = 3;     // vel_b   (3)
inline constexpr std::size_t kQuat = 6;    // q_nb    (4): w, x, y, z
inline constexpr std::size_t kOmega = 10;  // omega_b (3)
inline constexpr std::size_t kSize = 13;
}  // namespace body_layout

// Unpacks the body states. The quaternion is taken as stored: it is unit
// between steps, and slightly off unit inside RK4 stages.
BodyState unpack_body(std::span<const double> x) noexcept;
void pack_body(const BodyState& b, std::span<double> x) noexcept;

// Diagonal blocks m·I₃ and I_b.
Mat66 rigid_body_mass_matrix(const MassProperties& mp) noexcept;

// Right-hand side of the body equations without applied loads:
//   [m C_bn g_n − m ω × v;  −ω × (I_b ω)]
// so that M_rb a = rigid_body_bias + applied loads, with a = [v̇_b; ω̇_b].
// Padfield 3rd ed. Eqs. 3A.24 and 3A.33 (general tensor form); gravity Eq. 3A.43.
Vec6 rigid_body_bias(const MassProperties& mp, const BodyState& b) noexcept;

// Writes the body part of ẋ given the solved acceleration a:
//   ṗ_n = C_nb v_b,  v̇_b = a[0:3],  q̇_nb = ½ q_nb ⊗ [0, ω_b],  ω̇_b = a[3:6]
// The quaternion equation is from spec §6.1 (not in the D-001 references);
// it is verified against its closed-form solution in tests (D-013).
void body_kinematics(const BodyState& b, const Vec6& a, std::span<double> xdot) noexcept;

}  // namespace eyas
