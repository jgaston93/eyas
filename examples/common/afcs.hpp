#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

#include "eyas/types.hpp"

// The flight control system's switches as command-line text (--afcs in
// eyas_drive and the pilot station; M8c2): a comma list of sas (both SAS
// channels), sas_digital, sas_analog and pba, or none.
namespace examples {

inline std::string afcs_text(const eyas::AfcsSwitches& s) {
    std::string t;
    const auto add = [&t](const char* name) { t += (t.empty() ? "" : ",") + std::string(name); };
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
    return t.empty() ? "none" : t;
}

// Throws std::runtime_error on an unknown name.
inline eyas::AfcsSwitches parse_afcs(std::string_view text) {
    eyas::AfcsSwitches s;
    if (text == "none") {
        return s;
    }
    while (!text.empty()) {
        const std::size_t comma = text.find(',');
        const std::string_view name = text.substr(0, comma);
        if (name == "sas") {
            s.sas_digital = s.sas_analog = true;
        } else if (name == "sas_digital") {
            s.sas_digital = true;
        } else if (name == "sas_analog") {
            s.sas_analog = true;
        } else if (name == "pba") {
            s.pitch_bias = true;
        } else {
            throw std::runtime_error("--afcs: unknown switch '" + std::string(name) +
                                     "' (sas, sas_digital, sas_analog, pba or none)");
        }
        text = comma == std::string_view::npos ? std::string_view() : text.substr(comma + 1);
    }
    return s;
}

}  // namespace examples
