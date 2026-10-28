#pragma once

#include <array>
#include <cstddef>
#include <limits>

#include "eyas/aircraft_state.hpp"
#include "eyas/types.hpp"

// Readouts (D-059, SPEC_REVIEW #32): what a component reports for the
// outputs' Tier 4 and flight-control fields, written during the output
// evaluation's derivative call. A component declares the role it reports in
// (DynamicComponent::readout_role()); the executive passes the record to the
// first component of each role in config order, and to every landing gear and
// every engine component (D-060). Fields keep NaN (false) unless a component
// of their role writes them.
namespace eyas {

enum class ReadoutRole : unsigned char {
    none,             // reports nothing (the default)
    main_rotor,       // the main_rotor_* fields, rotor speed, flapping
    tail_rotor,       // the tail_rotor_* fields
    landing_gear,     // weight on wheels (every gear, OR-ed)
    airframe,         // the stabilator's incidence
    flight_controls,  // the blade pitch after the servos and head limits
    engines,          // its own entries of `engines` (every engine component, D-060)
};
inline constexpr std::size_t kNumReadoutRoles = 7;

// Whether every component of `role` reports, not only the first (D-059, D-060).
constexpr bool every_component_reports(ReadoutRole role) noexcept {
    return role == ReadoutRole::landing_gear || role == ReadoutRole::engines;
}

struct Readouts {
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    // Main rotor (units, frames and signs as AircraftState Tier 4)
    double rotor_speed_percent = kNan;  // [%] of the configured speed
    double main_rotor_thrust = kNan;    // [N]
    double main_rotor_torque = kNan;    // [N·m]
    double main_rotor_power = kNan;     // [W]
    double coning = kNan;               // [rad]
    double flap_1c = kNan;              // [rad]
    double flap_1s = kNan;              // [rad]
    double main_rotor_azimuth = kNan;   // [rad] [0, 2π)
    // Tail rotor
    double tail_rotor_thrust = kNan;   // [N]
    double tail_rotor_azimuth = kNan;  // [rad] [0, 2π)
    double tail_rotor_torque = kNan;   // [N·m] (D-060)
    double tail_rotor_power = kNan;    // [W]
    // Engines, Controls::engines' order (D-060)
    std::array<EngineState, kMaxEngines> engines{};
    // Landing gear
    bool weight_on_wheels = false;
    // Airframe
    double stabilator_incidence = kNan;  // [rad]
    // Flight controls
    double main_rotor_theta0 = kNan;   // [rad]
    double main_rotor_theta1s = kNan;  // [rad]
    double main_rotor_theta1c = kNan;  // [rad]
    double tail_rotor_theta0 = kNan;   // [rad]
};

}  // namespace eyas
