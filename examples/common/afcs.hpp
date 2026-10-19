#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

#include "eyas/types.hpp"

// The flight control system's switches as text: a list of sas (both SAS
// channels), sas_digital, sas_analog, pba, fps, fps_turn and trim_release, or
// none. --afcs in eyas_drive and the pilot station separates them with
// commas (M8c2); the control files' and state histories' afcs column with
// '+', so the CSV keeps its commas (M8c3).
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

inline bool same_switches(const eyas::AfcsSwitches& a, const eyas::AfcsSwitches& b) noexcept {
    return a.sas_digital == b.sas_digital && a.sas_analog == b.sas_analog &&
           a.pitch_bias == b.pitch_bias && a.fps == b.fps && a.fps_turn == b.fps_turn &&
           a.trim_release == b.trim_release;
}

}  // namespace examples
