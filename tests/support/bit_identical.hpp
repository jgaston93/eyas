#pragma once

#include <cstring>
#include <span>
#include <vector>

#include "eyas/aircraft_state.hpp"

// Bit-for-bit comparison for the determinism tests (spec §7 rule 4).
namespace eyas::test {

// Equality of two arrays of doubles: unlike ==, it tells −0 from +0 and
// compares NaN payloads.
inline bool bit_identical(std::span<const double> a, std::span<const double> b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

// Every physical output field (Tiers 1–4), in a fixed order. Excludes the
// per-call diagnostics, last_advance and quat_norm_error, which by design
// describe the call that produced them.
inline std::vector<double> physical_outputs(const AircraftState& o) {
    std::vector<double> v = {static_cast<double>(o.sim_time_ns),
                             o.sim_time,
                             o.latitude,
                             o.longitude,
                             o.altitude,
                             o.q_nb.w(),
                             o.q_nb.x(),
                             o.q_nb.y(),
                             o.q_nb.z(),
                             o.roll,
                             o.pitch,
                             o.heading,
                             o.load_factor,
                             o.ground_speed,
                             o.track,
                             o.true_airspeed,
                             o.equivalent_airspeed,
                             o.calibrated_airspeed,
                             o.indicated_airspeed,
                             o.angle_of_attack,
                             o.sideslip,
                             o.pressure_altitude,
                             o.vertical_speed,
                             o.outside_air_temperature,
                             o.rotor_speed_percent,
                             o.main_rotor_thrust,
                             o.main_rotor_torque,
                             o.main_rotor_power,
                             o.coning,
                             o.flap_1c,
                             o.flap_1s,
                             o.main_rotor_azimuth,
                             o.tail_rotor_thrust,
                             o.tail_rotor_azimuth,
                             o.weight_on_wheels ? 1.0 : 0.0};
    for (const Vec3* w :
         {&o.pos_n, &o.vel_n, &o.vel_b, &o.omega_b, &o.accel_b, &o.specific_force_b}) {
        v.insert(v.end(), w->data(), w->data() + 3);
    }
    return v;
}

}  // namespace eyas::test
