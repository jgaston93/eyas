#include "mapping.hpp"

#include <array>
#include <charconv>
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
            break;
        case InputSource::flightgear:
            m.collective = axis(1, true, AxisMode::rate, kCollectiveRate, 0.1);
            m.pedal = axis(0, false, AxisMode::absolute, 1.0, 0.1);
            m.lat_cyclic = axis(4, false, AxisMode::absolute, 1.0);
            m.lon_cyclic = axis(3, true, AxisMode::absolute, 1.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 3;
            break;
        case InputSource::joystick:
            m.lat_cyclic = axis(0, false, AxisMode::absolute, 1.0);
            m.lon_cyclic = axis(1, true, AxisMode::absolute, 1.0);
            m.pedal = axis(2, false, AxisMode::absolute, 1.0);
            m.collective = axis(3, true, AxisMode::absolute, 1.0, 0.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 2;
            break;
        case InputSource::mouse_keyboard:
            m.lat_cyclic = axis(0, false, AxisMode::absolute, 1.0, 0.0);
            m.lon_cyclic = axis(1, true, AxisMode::absolute, 1.0, 0.0);
            m.collective = axis(2, false, AxisMode::rate, 0.2, 0.0);
            m.pedal = axis(3, false, AxisMode::absolute, 0.5, 0.0);
            m.trim_button = 0;
            m.brake_button = 1;
            m.trim_reset_button = 2;
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
            m.trim_button = to_int(value, origin, n);
        } else if (key == "trim_reset_button") {
            m.trim_reset_button = to_int(value, origin, n);
        } else if (key == "brake_button") {
            m.brake_button = to_int(value, origin, n);
        } else if (key == "recentre") {
            m.recentre = to_double(value, origin, n);
        } else {
            const auto dot = key.find('.');
            AxisMap* a = dot == std::string_view::npos ? nullptr : control(m, key.substr(0, dot));
            if (a == nullptr) {
                fail(origin, n, "unknown key '" + std::string(key) + "'");
            }
            const std::string_view field = key.substr(dot + 1);
            if (field == "axis") {
                a->axis = to_int(value, origin, n);
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
            if (a->axis >= kMaxAxes) {
                fail(origin, n, "axis must be below " + std::to_string(kMaxAxes));
            }
        }
        for (const int b : {m.trim_button, m.trim_reset_button, m.brake_button}) {
            if (b >= kMaxButtons) {
                fail(origin, n, "buttons must be below " + std::to_string(kMaxButtons));
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
        out << name << ".axis = " << a.axis << '\n'
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
    out << "trim_button = " << map.trim_button << '\n'
        << "trim_reset_button = " << map.trim_reset_button << '\n'
        << "brake_button = " << map.brake_button << '\n'
        << "recentre = " << map.recentre << '\n';
}

}  // namespace station
