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
inline constexpr double kRankine = 5.0 / 9.0;                         // [K] exact
// International Table Btu: 1 Btu/lbm = 2326 J/kg exactly, so 1 Btu = 2326 kPoundMass J.
inline constexpr double kBtu = 2326.0 * kPoundMass;  // [J]

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
    stiffness,               // force per length, e.g. a tire spring
    damping,                 // force per speed, e.g. a linear damper
    quadratic_damping,       // force per speed², e.g. an oleo orifice
    angle_per_length,        // e.g. a control rigging gain [deg/in]
    angle_per_acceleration,  // e.g. a lateral-acceleration feedback gain
    time_squared,            // e.g. the s² coefficient of a servo transfer function
    // AFCS gains (GenHel §5.5), output a fraction of authority ("%"):
    per_angular_rate,       // e.g. a rate gain [%/(deg/s)]
    per_speed,              // e.g. an airspeed gain [%/kt]
    per_acceleration,       // e.g. a lateral-acceleration gain [%/(ft/s²)]
    rate_per_acceleration,  // an acceleration scaled into a rate path [(deg/s)/(ft/s²)]
    per_angle_time,         // an attitude integrator's gain [%/(deg s)]
    per_length,             // an airspeed integrator's gain [%/(kt s)]
    // Engines and their fuel control (GenHel §5.6; D-053). Speeds in % are
    // fractions of the 100 % speed, so "V/%" is a voltage.
    mass_flow,               // fuel flow [kg/s]
    pressure_per_mass_flow,  // ∂P3/∂W_f [Pa·s/kg]
    moment_per_mass_flow,    // ∂Q/∂W_f [N·m·s/kg]
    mass_flow_per_pressure,  // a fuel ratio W_f/P3 [kg/(s·Pa)]
    per_moment_time,         // a spool's acceleration per torque [1/(N·m·s)], speed a fraction
    voltage,                 // ECU signals [V]
    voltage_per_moment,      // [V/(N·m)]
    current_per_voltage,     // [A/V]
    length_per_current,      // [m/A]
    voltage_per_length,      // [V/m]
    // Ballin's component-level T700 (NASA TM-100991; D-054).
    temperature,                      // absolute [K]
    per_temperature,                  // [1/K]
    specific_energy,                  // enthalpy [J/kg]
    specific_heat,                    // [J/(kg·K)]
    temperature_per_specific_energy,  // [K·kg/J]
    burner_resistance,                // K_dpb, eq. 18 [Pa²·s²/(kg²·K)]
    heat_sink_constant,               // TC_T41, eq. 51 [kg^0.8·s^0.2/K^0.5]
    current,                          // [A]
    length_per_current_time,          // a torque motor's sensitivity [m/(A·s)]
    mass_flow_per_pressure_length,    // a linkage gain on a fuel ratio [kg/(s·Pa·m)]
    flow_parameter,                   // a turbine's W √T / P [kg·K^0.5/(s·Pa)]
    moment_time,                      // a torque integrated over time [N·m·s]
    power,                            // [W]
    voltage_per_temperature,          // [V/K]
    voltage_time,                     // a rate network's gain [V·s per unit input]
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
