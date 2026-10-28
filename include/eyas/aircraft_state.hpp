#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "eyas/types.hpp"

// The output record of the Eyas library (spec §6.10). It grows in tiers;
// existing fields never change meaning, units or frame. Tiers 2–4 (D-043) are
// evaluated once per advance() at its final state, with that call's controls
// and environment (after trim(): the trimmed controls and the condition's
// environment; restore() brings back those of the snapshot; reset() keeps
// the last ones until the next advance()). "The CG" is the airframe CG
// (D-002).
namespace eyas {

namespace detail {
inline constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
}  // namespace detail

// One engine's readouts (M9, D-060). Every field is NaN (out: false) for an
// entry past the aircraft's engines, and NaN where its engine model has no
// such quantity. Torque and power are the engine's output shaft's, at the
// power-turbine speed, as a flight test's engine torque sensor measures them.
struct EngineState {
    double gas_generator_speed_percent = detail::kNan;  // NG [%] of its 100 % speed
    double power_turbine_speed_percent = detail::kNan;  // NP [%] of its 100 % speed
    double torque = detail::kNan;                       // [N·m] at the output shaft
    double power = detail::kNan;                        // [W] = torque × NP
    double fuel_flow = detail::kNan;                    // W_f [kg/s]
    double turbine_gas_temperature = detail::kNan;      // T4.5 [K], the gas's
    bool out = false;  // flamed out: no gas path, no torque (D-056)
};

struct AircraftState {
    // ---- Tier 1: pose (M2) --------------------------------------------------
    std::int64_t sim_time_ns = 0;  // [ns] n_steps × physics step, exact
    double sim_time = 0.0;         // [s]  sim_time_ns × 1e-9

    // Geodetic position of the CG on the flat-earth map (D-017).
    double latitude = 0.0;   // [rad] WGS-84 geodetic
    double longitude = 0.0;  // [rad] (−π, π]
    double altitude = 0.0;   // [m] above MSL

    // Attitude. q_nb maps body (FRD) vectors to NED; unit length.
    Quat q_nb = Quat::Identity();
    // Padfield 3rd ed. Euler angles (yaw–pitch–roll order), output only.
    double roll = 0.0;     // φ   [rad] (−π, π]
    double pitch = 0.0;    // θ   [rad] [−π/2, π/2]
    double heading = 0.0;  // ψ_h [rad] [0, 2π), from north; true heading

    // ---- Tier 2: inertial and navigation state (M8a, D-043) ------------------
    Vec3 pos_n = Vec3::Zero();    // [m] CG position, NED from the geodetic origin
    Vec3 vel_n = Vec3::Zero();    // [m/s] CG inertial velocity, NED
    Vec3 vel_b = Vec3::Zero();    // [m/s] CG inertial velocity, body axes
    Vec3 omega_b = Vec3::Zero();  // [rad/s] body angular rate, body axes
    // [m/s²] CG inertial acceleration, body axes: v̇_b + ω × v_b.
    Vec3 accel_b = Vec3::Zero();
    // [m/s²] specific force at the CG, body axes: accel_b − g (what an
    // accelerometer at the CG reads; −g0 along z_b at rest, level).
    Vec3 specific_force_b = Vec3::Zero();
    double load_factor = 0.0;   // n_z [-] = −specific_force_b.z / g0; 1 in level flight
    double ground_speed = 0.0;  // [m/s] horizontal inertial speed
    // [rad] [0, 2π), from north: direction of the horizontal inertial
    // velocity; NaN at zero ground speed. Reported at any ground speed, so it
    // is noise at low speed (a hover's 4/rev vibration moves the CG by
    // ~1e-6 m/s): a display blanks it below its own threshold (D-043).
    double track = detail::kNan;

    // ---- Tier 3: air data (M8a, D-043) ---------------------------------------
    // From the velocity relative to the air at the CG, v_air_b = v_b − C_bn
    // wind_n, and the atmosphere at the CG. An ideal pitot-static system: no
    // position or instrument error.
    double true_airspeed = 0.0;        // V [m/s] = |v_air_b|
    double equivalent_airspeed = 0.0;  // V_e [m/s] = V √(ρ/ρ0)
    double calibrated_airspeed = 0.0;  // V_c [m/s] from the impact pressure, subsonic
    double indicated_airspeed = 0.0;   // [m/s] = V_c: no position-error data (DATA_GAPS)
    double angle_of_attack = 0.0;      // α [rad] = atan2(w_air, u_air)
    // β [rad] = asin(v_air / V), + wind from the right (as TrimCondition); 0 at V = 0.
    double sideslip = 0.0;
    double pressure_altitude = 0.0;        // [m'] standard-atmosphere geopotential altitude
    double vertical_speed = 0.0;           // [m/s] inertial, + up (−vel_n.z)
    double outside_air_temperature = 0.0;  // [K] static air temperature at the CG

    // ---- Tier 4: rotor (M8a, D-043) -------------------------------------------
    // The main rotor is the config's first blade-element rotor, the tail rotor
    // its first Bailey rotor. Fields of a rotor the config lacks are NaN.
    double rotor_speed_percent = detail::kNan;  // N_R [%]; 100 while Ω is fixed (until M9)
    double main_rotor_thrust = detail::kNan;    // T [N], along the shaft, up
    double main_rotor_torque = detail::kNan;    // Q [N·m] aerodynamic shaft torque
    double main_rotor_power = detail::kNan;     // P [W] = Q Ω
    // Multiblade flapping (Padfield 3rd ed. Eqs. 3.42–3.45) [rad]; 0 for
    // rigid blades.
    double coning = detail::kNan;              // β0
    double flap_1c = detail::kNan;             // β1c, + disc tilted back (up at ψ = 0, aft)
    double flap_1s = detail::kNan;             // β1s, + up at ψ = 90°
    double main_rotor_azimuth = detail::kNan;  // ψ [rad] [0, 2π), ψ = 0 aft (D-024)
    double tail_rotor_thrust = detail::kNan;   // T [N], along −z of its hub
    // [rad] [0, 2π): Ω_TR t, for display; the tail rotor has no blade states.
    double tail_rotor_azimuth = detail::kNan;
    bool weight_on_wheels = false;  // some gear carries a normal load

    // ---- Tier 4: drive (M9, D-060) -------------------------------------------
    // The tail rotor's aerodynamic shaft torque and power, as the main rotor's;
    // NaN for a tail rotor without a torque model (D-058).
    double tail_rotor_torque = detail::kNan;  // Q [N·m]
    double tail_rotor_power = detail::kNan;   // P [W] = Q Ω_TR
    // The engines, in Controls::engines' order (config order). A model that
    // runs several engines as one reports each with the same values.
    std::array<EngineState, kMaxEngines> engines{};

    // ---- Flight controls (M8c, D-045) -----------------------------------------
    // Stabilator: trailing edge down positive, from the airframe (scheduled,
    // held or from the flight control system); NaN without an airframe.
    // Blade pitch: the flight control system's (genhel_fcs) outputs after its
    // servos and the rotor head limits, whether or not a rotor takes them; NaN
    // without one, also when a rotor flies on its own control map. Main rotor
    // θ = θ0 + θ1c cos ψ + θ1s sin ψ at the hub centre (Padfield 3rd ed.
    // Eq. 3.59), tail rotor its collective before the rotor's own pitch bias.
    double stabilator_incidence = detail::kNan;  // [rad]
    double main_rotor_theta0 = detail::kNan;     // [rad]
    double main_rotor_theta1s = detail::kNan;    // [rad] + nose-down disc tilt needs θ1s < 0
    double main_rotor_theta1c = detail::kNan;    // [rad]
    double tail_rotor_theta0 = detail::kNan;     // [rad]

    // ---- Tier 5: diagnostics ------------------------------------------------
    AdvanceStatus last_advance;  // status of the most recent advance()
    // [-] max |‖q‖ − 1| before renormalization over the last advance() that
    // ran; a rejected dt leaves it unchanged.
    double quat_norm_error = 0.0;
};

}  // namespace eyas
