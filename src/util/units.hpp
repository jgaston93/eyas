#pragma once

#include <numbers>
#include <optional>
#include <string_view>

// SI conversion factors for the units found in source data. Eyas is SI
// throughout; imperial data is converted once, at load time (spec §3).
namespace eyas::units {

inline constexpr double kFoot = 0.3048;              // [m]  international foot, exact
inline constexpr double kInch = 0.0254;              // [m]  exact
inline constexpr double kStandardGravity = 9.80665;  // [m/s^2] exact (CGPM 1901)
inline constexpr double kPoundMass = 0.45359237;     // [kg] exact
inline constexpr double kPoundForce = kPoundMass * kStandardGravity;  // [N]
inline constexpr double kSlug = kPoundForce / kFoot;                  // [kg] 1 lbf·s²/ft
inline constexpr double kKnot = 1852.0 / 3600.0;                      // [m/s] exact
inline constexpr double kDegree = std::numbers::pi / 180.0;           // [rad]
inline constexpr double kRpm = 2.0 * std::numbers::pi / 60.0;         // [rad/s]

// Factor that converts a value expressed in `unit` to SI, or nullopt if the
// unit is not registered. Only multiplicative units are supported (no
// offsets such as °F). The registry is an explicit list so every accepted
// unit is auditable; see units.cpp. "lb" is rejected as ambiguous; write
// "lbf" (force) or "lbm" (mass).
std::optional<double> si_scale(std::string_view unit) noexcept;

}  // namespace eyas::units
