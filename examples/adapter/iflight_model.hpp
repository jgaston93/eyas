#pragma once

#include <cstdint>
#include <string>

// A toy host-side flight-model interface (spec §6.10, "Adapter pattern"). The
// real one lives in the host application's codebase; this stands in for it
// so the example can show what an adapter does. Host conventions, chosen to
// differ from Eyas's on purpose: degrees, feet, knots, feet per minute,
// degrees Celsius, and pedal positive left.
namespace host {

struct PilotInputs {
    double collective = 0.0;    // [0, 1]  up
    double lateral = 0.0;       // [-1, 1] right stick
    double longitudinal = 0.0;  // [-1, 1] forward stick
    double pedal = 0.0;         // [-1, 1] LEFT pedal (host convention)
    bool brakes = false;
    bool sas = false;                // cockpit switch: stability augmentation, both channels
    bool pitch_bias = false;         // cockpit switch: pitch bias actuator
    bool fps = false;                // cockpit switch: flight path stabilization
    bool turn_coordination = false;  // the FPS turn mode, latched by the host
                                     // (e.g. with examples::latch_turn_switch)
    bool trim_release = false;       // cyclic button, held
};

// Where to start: steady level flight (airspeed > 0 or hover).
struct InitialCondition {
    double airspeed_kt = 0.0;  // true airspeed
    double altitude_ft = 0.0;  // MSL
    double heading_deg = 0.0;  // true
    bool sas = false;          // the switches the aircraft is trimmed with
    bool pitch_bias = false;
    bool fps = false;
};

struct Pose {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_ft = 0.0;  // MSL
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double heading_deg = 0.0;  // true
};

struct AirData {
    double indicated_airspeed_kt = 0.0;
    double true_airspeed_kt = 0.0;
    double pressure_altitude_ft = 0.0;
    double vertical_speed_fpm = 0.0;
    double angle_of_attack_deg = 0.0;
    double sideslip_deg = 0.0;
    double outside_air_temperature_c = 0.0;
};

struct RotorData {
    double rotor_speed_percent = 0.0;
    double main_rotor_torque_ftlbf = 0.0;
    bool weight_on_wheels = false;
};

class IFlightModel {
public:
    virtual ~IFlightModel() = default;
    // Loads the aircraft and puts it at the initial condition. Returns false
    // (with a reason in `error`) if it can't.
    virtual bool initialize(const std::string& aircraft, const InitialCondition& ic,
                            std::string& error) = 0;
    // The pilot inputs that hold the initial condition: the host drives its
    // cockpit controls (e.g. force-feel trim) there before the first update.
    virtual PilotInputs trim_inputs() const = 0;
    // One host frame of dt_ns nanoseconds. Real time: must not block or allocate.
    virtual void update(const PilotInputs& in, std::int64_t dt_ns) = 0;
    virtual Pose pose() const = 0;
    virtual AirData air_data() const = 0;
    virtual RotorData rotor() const = 0;
    // False once the model has hit a fault the host should show (e.g. a
    // non-finite state, or frames being dropped).
    virtual bool healthy() const = 0;
};

}  // namespace host
