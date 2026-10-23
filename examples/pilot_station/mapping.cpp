#include "mapping.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace station {

InputSource parse_source(std::string_view name) {
    static constexpr std::array<std::pair<std::string_view, InputSource>, 6> kNames{{
        {"gamepad", InputSource::gamepad},
        {"joystick", InputSource::joystick},
        {"mouse", InputSource::mouse_keyboard},
        {"fg", InputSource::flightgear},
        {"script", InputSource::script},
        {"none", InputSource::none},
    }};
    for (const auto& [n, s] : kNames) {
        if (n == name) {
            return s;
        }
    }
    throw std::runtime_error("unknown input '" + std::string(name) +
                             "' (gamepad, joystick, mouse, fg, script, none)");
}

std::string_view source_name(InputSource s) noexcept {
    switch (s) {
        case InputSource::gamepad:
            return "gamepad";
        case InputSource::joystick:
            return "joystick";
        case InputSource::mouse_keyboard:
            return "mouse";
        case InputSource::flightgear:
            return "fg";
        case InputSource::script:
            return "script";
        case InputSource::none:
            return "none";
    }
    return "?";
}

namespace {

AxisMap axis(int index, bool invert, AxisMode mode, double gain, double deadband = 0.05) {
    AxisMap m;
    m.axis = index;
    m.invert = invert;
    m.mode = mode;
    m.gain = gain;
    m.deadband = deadband;
    return m;
}

// Collective rate at full stick [1/s]: full travel in 2 s.
constexpr double kCollectiveRate = 0.5;

}  // namespace

ControlsMapConfig default_map(InputSource s) noexcept {
    ControlsMapConfig m;
    switch (s) {
        case InputSource::gamepad:
            m.collective = axis(1, true, AxisMode::rate, kCollectiveRate, 0.1);
            m.pedal = axis(0, false, AxisMode::absolute, 1.0, 0.1);
            m.lat_cyclic = axis(2, false, AxisMode::absolute, 1.0);
            m.lon_cyclic = axis(3, true, AxisMode::absolute, 1.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 3;
            m.sas_button = 4;
            m.pba_button = 5;
            m.fps_button = 2;
            break;
        case InputSource::flightgear:
            m.collective = axis(1, true, AxisMode::rate, kCollectiveRate, 0.1);
            m.pedal = axis(0, false, AxisMode::absolute, 1.0, 0.1);
            m.lat_cyclic = axis(4, false, AxisMode::absolute, 1.0);
            m.lon_cyclic = axis(3, true, AxisMode::absolute, 1.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 3;
            m.sas_button = 4;
            m.pba_button = 5;
            m.fps_button = 2;
            break;
        case InputSource::joystick:
            m.lat_cyclic = axis(0, false, AxisMode::absolute, 1.0);
            m.lon_cyclic = axis(1, true, AxisMode::absolute, 1.0);
            m.pedal = axis(2, false, AxisMode::absolute, 1.0);
            m.collective = axis(3, true, AxisMode::absolute, 1.0, 0.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 2;
            m.sas_button = 3;
            m.pba_button = 4;
            m.fps_button = 5;
            break;
        case InputSource::mouse_keyboard:
            m.lat_cyclic = axis(0, false, AxisMode::absolute, 1.0, 0.0);
            m.lon_cyclic = axis(1, true, AxisMode::absolute, 1.0, 0.0);
            m.collective = axis(2, false, AxisMode::rate, 0.2, 0.0);
            m.pedal = axis(3, false, AxisMode::absolute, 0.5, 0.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 2;
            m.sas_button = 3;
            m.pba_button = 4;
            m.fps_button = 5;
            break;
        case InputSource::script:
        case InputSource::none:
            break;
    }
    return m;
}

namespace {

std::string_view trim(std::string_view s) {
    const auto b = s.find_first_not_of(" \t\r");
    if (b == std::string_view::npos) {
        return {};
    }
    const auto e = s.find_last_not_of(" \t\r");
    return s.substr(b, e - b + 1);
}

[[noreturn]] void fail(std::string_view origin, int line, const std::string& what) {
    throw std::runtime_error(std::string(origin) + ":" + std::to_string(line) + ": " + what);
}

double to_double(std::string_view v, std::string_view origin, int line) {
    double d = 0.0;
    const auto [p, ec] = std::from_chars(v.data(), v.data() + v.size(), d);
    if (ec != std::errc{} || p != v.data() + v.size()) {
        fail(origin, line, "expected a number, got '" + std::string(v) + "'");
    }
    return d;
}

int to_int(std::string_view v, std::string_view origin, int line) {
    int i = 0;
    const auto [p, ec] = std::from_chars(v.data(), v.data() + v.size(), i);
    if (ec != std::errc{} || p != v.data() + v.size()) {
        fail(origin, line, "expected an integer, got '" + std::string(v) + "'");
    }
    return i;
}

bool to_bool(std::string_view v, std::string_view origin, int line) {
    if (v == "true") {
        return true;
    }
    if (v == "false") {
        return false;
    }
    fail(origin, line, "expected true or false, got '" + std::string(v) + "'");
}

// "d:i" (device d, index i) or "i" (device 0) into a flat index, or −1
// for "-1" (none).
int to_index(std::string_view v, int per_device, std::string_view what, std::string_view origin,
             int line) {
    const auto colon = v.find(':');
    const int device =
        colon == std::string_view::npos ? 0 : to_int(v.substr(0, colon), origin, line);
    const int index =
        to_int(colon == std::string_view::npos ? v : v.substr(colon + 1), origin, line);
    if (colon == std::string_view::npos && index == -1) {
        return -1;
    }
    if (device < 0 || device >= kMaxDevices || index < 0 || index >= per_device) {
        fail(origin, line,
             std::string(what) + " must be <device>:<index> with device 0-" +
                 std::to_string(kMaxDevices - 1) + " and index 0-" +
                 std::to_string(per_device - 1) + ", or -1 for none; got '" + std::string(v) + "'");
    }
    return device * per_device + index;
}

std::string index_text(int flat, int per_device) {
    if (flat < 0) {
        return "-1";
    }
    return std::to_string(flat / per_device) + ":" + std::to_string(flat % per_device);
}

AxisMap* control(ControlsMapConfig& m, std::string_view name) {
    if (name == "collective") {
        return &m.collective;
    }
    if (name == "lat_cyclic") {
        return &m.lat_cyclic;
    }
    if (name == "lon_cyclic") {
        return &m.lon_cyclic;
    }
    if (name == "pedal") {
        return &m.pedal;
    }
    if (name == "power_lever") {
        return &m.power_lever;
    }
    return nullptr;
}

}  // namespace

ControlsMapConfig parse_map(std::string_view text, const ControlsMapConfig& base,
                            std::string_view origin) {
    ControlsMapConfig m = base;
    int n = 0;
    while (!text.empty()) {
        ++n;
        const auto eol = text.find('\n');
        std::string_view line = text.substr(0, eol);
        text = eol == std::string_view::npos ? std::string_view{} : text.substr(eol + 1);
        line = trim(line.substr(0, line.find('#')));
        if (line.empty()) {
            continue;
        }
        const auto eq = line.find('=');
        if (eq == std::string_view::npos) {
            fail(origin, n, "expected key = value");
        }
        const std::string_view key = trim(line.substr(0, eq));
        const std::string_view value = trim(line.substr(eq + 1));
        if (key == "trim_button") {
            m.trim_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "trim_reset_button") {
            m.trim_reset_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "brake_button") {
            m.brake_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "sas_button") {
            m.sas_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "pba_button") {
            m.pba_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "fps_button") {
            m.fps_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "engine_select_button") {
            m.engine_select_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "rpm_up_button") {
            m.rpm_up_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "rpm_down_button") {
            m.rpm_down_button = to_index(value, kButtonsPerDevice, "a button", origin, n);
        } else if (key == "rpm_rate") {
            m.rpm_rate = to_double(value, origin, n);
            if (!(m.rpm_rate >= 0.0) || !std::isfinite(m.rpm_rate)) {
                fail(origin, n, "rpm_rate must be finite and non-negative [1/s]");
            }
        } else if (key == "rpm_low" || key == "rpm_high") {
            (key == "rpm_low" ? m.rpm_low : m.rpm_high) = to_double(value, origin, n);
            if (std::isnan(m.rpm_low) || m.rpm_low <= 0.0 || m.rpm_low > 1.0 || m.rpm_high < 1.0 ||
                !std::isfinite(m.rpm_high)) {
                fail(origin, n, "rpm_low must be in (0, 1] and rpm_high finite and at least 1");
            }
        } else if (key == "feet_on_pedals") {
            m.feet_on_pedals = to_double(value, origin, n);
            if (std::isnan(m.feet_on_pedals) || m.feet_on_pedals < 0.0 || m.feet_on_pedals > 1.0) {
                fail(origin, n, "feet_on_pedals must be in [0, 1] (fraction of the pedal travel)");
            }
        } else if (key == "recentre") {
            m.recentre = to_double(value, origin, n);
        } else if (key == "pickup") {
            m.pickup = to_double(value, origin, n);
            if (std::isnan(m.pickup) || m.pickup < 0.0 || m.pickup > 0.5) {
                fail(origin, n, "pickup must be in [0, 0.5] (fraction of the lever's travel)");
            }
        } else {
            const auto dot = key.find('.');
            AxisMap* a = dot == std::string_view::npos ? nullptr : control(m, key.substr(0, dot));
            if (a == nullptr) {
                fail(origin, n, "unknown key '" + std::string(key) + "'");
            }
            const std::string_view field = key.substr(dot + 1);
            if (field == "axis") {
                a->axis = to_index(value, kAxesPerDevice, "an axis", origin, n);
            } else if (field == "invert") {
                a->invert = to_bool(value, origin, n);
            } else if (field == "deadband") {
                a->deadband = to_double(value, origin, n);
            } else if (field == "expo") {
                a->expo = to_double(value, origin, n);
            } else if (field == "gain") {
                a->gain = to_double(value, origin, n);
            } else if (field == "mode") {
                if (value == "absolute") {
                    a->mode = AxisMode::absolute;
                } else if (value == "rate") {
                    a->mode = AxisMode::rate;
                } else {
                    fail(origin, n, "mode must be absolute or rate");
                }
            } else {
                fail(origin, n, "unknown key '" + std::string(key) + "'");
            }
        }
    }
    return m;
}

ControlsMapConfig read_map_file(const std::string& path, const ControlsMapConfig& base) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::stringstream s;
    s << in.rdbuf();
    return parse_map(s.str(), base, path);
}

void write_map(std::ostream& out, const ControlsMapConfig& map) {
    const auto write_axis = [&](std::string_view name, const AxisMap& a) {
        out << name << ".axis = " << index_text(a.axis, kAxesPerDevice) << '\n'
            << name << ".invert = " << (a.invert ? "true" : "false") << '\n'
            << name << ".deadband = " << a.deadband << '\n'
            << name << ".expo = " << a.expo << '\n'
            << name << ".mode = " << (a.mode == AxisMode::rate ? "rate" : "absolute") << '\n'
            << name << ".gain = " << a.gain << '\n';
    };
    out.precision(17);
    write_axis("collective", map.collective);
    write_axis("lat_cyclic", map.lat_cyclic);
    write_axis("lon_cyclic", map.lon_cyclic);
    write_axis("pedal", map.pedal);
    write_axis("power_lever", map.power_lever);
    out << "trim_button = " << index_text(map.trim_button, kButtonsPerDevice) << '\n'
        << "trim_reset_button = " << index_text(map.trim_reset_button, kButtonsPerDevice) << '\n'
        << "brake_button = " << index_text(map.brake_button, kButtonsPerDevice) << '\n'
        << "sas_button = " << index_text(map.sas_button, kButtonsPerDevice) << '\n'
        << "pba_button = " << index_text(map.pba_button, kButtonsPerDevice) << '\n'
        << "fps_button = " << index_text(map.fps_button, kButtonsPerDevice) << '\n'
        << "recentre = " << map.recentre << '\n'
        << "pickup = " << map.pickup << '\n'
        << "feet_on_pedals = " << map.feet_on_pedals << '\n'
        << "engine_select_button = " << index_text(map.engine_select_button, kButtonsPerDevice)
        << '\n'
        << "rpm_up_button = " << index_text(map.rpm_up_button, kButtonsPerDevice) << '\n'
        << "rpm_down_button = " << index_text(map.rpm_down_button, kButtonsPerDevice) << '\n'
        << "rpm_rate = " << map.rpm_rate << '\n'
        << "rpm_low = " << map.rpm_low << '\n'
        << "rpm_high = " << map.rpm_high << '\n';
}

}  // namespace station
