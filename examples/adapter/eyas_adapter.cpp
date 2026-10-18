#include "eyas_adapter.hpp"

#include <exception>
#include <numbers>

namespace {

constexpr double kDeg = 180.0 / std::numbers::pi;        // [deg/rad]
constexpr double kFoot = 0.3048;                         // [m]
constexpr double kKnot = 1852.0 / 3600.0;                // [m/s]
constexpr double kFootPound = 0.3048 * 4.4482216152605;  // [N·m] per ft·lbf

}  // namespace

bool EyasAdapter::initialize(const std::string& aircraft, const host::InitialCondition& ic,
                             std::string& error) {
    // Init-time: Eyas throws eyas::InitError on a bad config; the host's
    // interface reports errors by value, so catch here.
    try {
        model_.emplace(eyas::Model::from_file(aircraft));
        eyas::TrimCondition c;
        c.airspeed = ic.airspeed_kt * kKnot;
        c.altitude = ic.altitude_ft * kFoot;
        c.heading = ic.heading_deg / kDeg;
        c.afcs = {ic.sas, ic.sas, ic.pitch_bias};
        const eyas::TrimResult r = model_->trim(c);
        if (!r.converged) {
            error = "trim did not converge";
            return false;
        }
        env_ = eyas::Environment{c.isa_delta_T, c.wind_n};  // the trim's: ISA, no wind
        trim_.collective = r.controls.collective;
        trim_.lateral = r.controls.lat_cyclic;
        trim_.longitudinal = r.controls.lon_cyclic;
        trim_.pedal = -r.controls.pedal;  // to host convention, + left
        trim_.brakes = false;
        trim_.sas = ic.sas;
        trim_.pitch_bias = ic.pitch_bias;
    } catch (const std::exception& e) {
        error = e.what();
        model_.reset();
        return false;
    }
    healthy_ = true;
    return true;
}

void EyasAdapter::update(const host::PilotInputs& in, std::int64_t dt_ns) {
    if (!model_) {
        return;
    }
    eyas::Controls c;
    c.collective = in.collective;
    c.lat_cyclic = in.lateral;
    c.lon_cyclic = in.longitudinal;
    c.pedal = -in.pedal;  // host: + left pedal; Eyas: + right pedal (D-024)
    c.brake_left = c.brake_right = in.brakes ? 1.0 : 0.0;
    c.afcs = {in.sas, in.sas, in.pitch_bias};  // aircraft without them ignore them
    // Real time from here: advance() doesn't allocate, throw or do I/O.
    const eyas::AdvanceStatus s = model_->advance(c, env_, dt_ns);
    // A capped frame dropped sim time (a host hitch); a non-finite step kept
    // the last good state; a rejected call did nothing. Any of these is
    // worth showing the host.
    if (s.non_finite || s.rejected_dt || s.rejected_env || s.capped) {
        healthy_ = false;
    }
}

host::Pose EyasAdapter::pose() const {
    host::Pose p;
    if (!model_) {
        return p;
    }
    const eyas::AircraftState& o = model_->outputs();
    p.latitude_deg = o.latitude * kDeg;
    p.longitude_deg = o.longitude * kDeg;
    p.altitude_ft = o.altitude / kFoot;
    p.roll_deg = o.roll * kDeg;
    p.pitch_deg = o.pitch * kDeg;
    p.heading_deg = o.heading * kDeg;
    return p;
}

host::AirData EyasAdapter::air_data() const {
    host::AirData a;
    if (!model_) {
        return a;
    }
    const eyas::AircraftState& o = model_->outputs();
    a.indicated_airspeed_kt = o.indicated_airspeed / kKnot;
    a.true_airspeed_kt = o.true_airspeed / kKnot;
    a.pressure_altitude_ft = o.pressure_altitude / kFoot;
    a.vertical_speed_fpm = o.vertical_speed / kFoot * 60.0;
    a.angle_of_attack_deg = o.angle_of_attack * kDeg;
    a.sideslip_deg = o.sideslip * kDeg;
    a.outside_air_temperature_c = o.outside_air_temperature - 273.15;
    return a;
}

host::RotorData EyasAdapter::rotor() const {
    host::RotorData r;
    if (!model_) {
        return r;
    }
    const eyas::AircraftState& o = model_->outputs();
    r.rotor_speed_percent = o.rotor_speed_percent;
    r.main_rotor_torque_ftlbf = o.main_rotor_torque / kFootPound;
    r.weight_on_wheels = o.weight_on_wheels;
    return r;
}
