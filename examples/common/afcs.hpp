#pragma once

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <string_view>

#include "common/units.hpp"
#include "eyas/aircraft_state.hpp"
#include "eyas/types.hpp"

// The flight control system's switches as text: a list of sas (both SAS
// channels), sas_digital, sas_analog, pba, fps, fps_turn and trim_release, or
// none. --afcs in eyas_drive and the pilot station separates them with
// commas (M8c2); the control files' and state histories' afcs column with
// '+', so the CSV keeps its commas (M8c3). Also GenHel's FPS turn-switch
// latch, which a host runs itself (the library leaves the switch to it).
namespace examples {

inline std::string afcs_text(const eyas::AfcsSwitches& s, char separator = ',') {
    std::string t;
    const auto add = [&](const char* name) {
        if (!t.empty()) {
            t += separator;
        }
        t += name;
    };
    if (s.sas_digital && s.sas_analog) {
        add("sas");
    } else if (s.sas_digital) {
        add("sas_digital");
    } else if (s.sas_analog) {
        add("sas_analog");
    }
    if (s.pitch_bias) {
        add("pba");
    }
    if (s.fps) {
        add("fps");
    }
    if (s.fps_turn) {
        add("fps_turn");
    }
    if (s.trim_release) {
        add("trim_release");
    }
    return t.empty() ? "none" : t;
}

// Throws std::runtime_error on an unknown name.
inline eyas::AfcsSwitches parse_afcs(std::string_view text, char separator = ',') {
    eyas::AfcsSwitches s;
    if (text == "none") {
        return s;
    }
    while (!text.empty()) {
        const std::size_t end = text.find(separator);
        const std::string_view name = text.substr(0, end);
        if (name == "sas") {
            s.sas_digital = s.sas_analog = true;
        } else if (name == "sas_digital") {
            s.sas_digital = true;
        } else if (name == "sas_analog") {
            s.sas_analog = true;
        } else if (name == "pba") {
            s.pitch_bias = true;
        } else if (name == "fps") {
            s.fps = true;
        } else if (name == "fps_turn") {
            s.fps_turn = true;
        } else if (name == "trim_release") {
            s.trim_release = true;
        } else {
            throw std::runtime_error("unknown AFCS switch '" + std::string(name) +
                                     "' (sas, sas_digital, sas_analog, pba, fps, fps_turn, "
                                     "trim_release or none)");
        }
        text = end == std::string_view::npos ? std::string_view() : text.substr(end + 1);
    }
    return s;
}

// What GenHel's FPS turn-switch latch reads of the aircraft (CR-166309 p.
// 5.5-3), from the outputs of the frame before.
struct TurnSwitchInputs {
    double airspeed_kt = 0.0;  // VXBIKT: the body-x equivalent airspeed [kt]
    double roll = 0.0;         // φ [rad]
};

// VXBIKT from the public outputs [kt]: the body-x equivalent airspeed, as the
// library's flight control system reads it (genhel_fcs, D-040). u_air = V
// cos α cos β, so it is the equivalent airspeed times cos α cos β.
inline double vxbikt_kt(const eyas::AircraftState& o) noexcept {
    return o.equivalent_airspeed * std::cos(o.angle_of_attack) * std::cos(o.sideslip) / kKnot;
}

// The latch (GenHel p. 5.5-3; D-045 M8c3), for a host to run with its
// switches: latched on with the FPS on, VXBIKT above 60 kt, the trim release
// pressed and more than 2° of bank; off with the feet on the pedals and the
// trim release pressed, or with the FPS off. Returns the new switch position.
inline bool latch_turn_switch(bool on, const eyas::AfcsSwitches& s, bool feet_on_pedals,
                              const TurnSwitchInputs& f) noexcept {
    constexpr double kLatchSpeed = 60.0;                           // [kt]
    constexpr double kLatchBank = 2.0 * std::numbers::pi / 180.0;  // [rad]
    if (!s.fps) {
        return false;
    }
    if (s.trim_release && feet_on_pedals) {
        return false;
    }
    if (s.trim_release && f.airspeed_kt > kLatchSpeed && std::abs(f.roll) > kLatchBank) {
        return true;
    }
    return on;
}

}  // namespace examples
