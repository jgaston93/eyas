#pragma once

#include <cstdint>
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
inline constexpr double kPsi = kPoundForce / (kInch * kInch);         // [Pa] lbf/in²

// Physical dimension of a unit. Config quantities are read with the dimension
// they must have, so a value in a unit of the wrong kind ("1000 s" for a rate)
// is rejected at load. Frequency [Hz, cycles/s] and angular rate [rad/s] are
// kept apart because converting between them takes a factor of 2π.
enum class Dimension : std::uint8_t {
    dimensionless,
    length,
    time,
    frequency,
    mass,
    force,
    angle,
    per_angle,  // e.g. a lift-curve slope [1/rad]
    speed,
    angular_rate,
    acceleration,
    density,
    moment,                // force × length
    mass_moment,           // first moment, mass × length
    inertia,               // second moment, mass × length²
    rotational_stiffness,  // moment per angle, e.g. a hinge spring
    rotational_damping,    // moment per angular rate, e.g. a hinge damper
    area,
    volume,           // e.g. a moment per unit dynamic pressure [m³]
    angle_per_force,  // compliance, e.g. a coning angle per unit thrust
    pressure,
    stiffness,          // force per length, e.g. a tire spring
    damping,            // force per speed, e.g. a linear damper
    quadratic_damping,  // force per speed², e.g. an oleo orifice
};

// Name used in error messages, e.g. "length".
std::string_view dimension_name(Dimension d) noexcept;

struct Unit {
    double si_scale;  // factor to SI
    Dimension dimension;
};

// The registered unit called `unit`, or nullopt.
std::optional<Unit> find_unit(std::string_view unit) noexcept;

// Factor that converts a value expressed in `unit` to SI, or nullopt if the
// unit is not registered. Only multiplicative units are supported (no
// offsets such as °F). The registry is an explicit list so every accepted
// unit is auditable; see units.cpp. "lb" is rejected as ambiguous; write
// "lbf" (force) or "lbm" (mass).
std::optional<double> si_scale(std::string_view unit) noexcept;

}  // namespace eyas::units
