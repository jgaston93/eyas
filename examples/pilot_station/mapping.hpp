#pragma once

#include <iosfwd>
#include <string>
#include <string_view>

#include "controls_map.hpp"

// Control-mapping files for the pilot station (D-044). Init time: these
// allocate and throw std::runtime_error on bad input.
//
// A mapping is plain text, one `key = value` per line, `#` starting a
// comment. Keys are `<control>.<field>` for the controls collective,
// lat_cyclic, lon_cyclic and pedal, with the AxisMap fields axis, invert
// (true/false), deadband, expo, mode (absolute/rate) and gain, plus
// trim_button, trim_reset_button, brake_button and recentre. A file
// overrides the input source's defaults key by key; `--print-map` writes
// the full current mapping as a starting point.
namespace station {

enum class InputSource {
    gamepad,         // GLFW gamepad (SDL layout: Xbox-style pads)
    joystick,        // GLFW raw joystick (stick, throttle, twist or pedals)
    mouse_keyboard,  // the station window
    flightgear,      // the stick FlightGear reads, forwarded over UDP
    script,          // an eyas_drive control file
    none,            // the start controls, held
};

InputSource parse_source(std::string_view name);
std::string_view source_name(InputSource s) noexcept;

// The built-in mapping for each source. Axis and button numbers:
//  gamepad (GLFW_GAMEPAD_*): axes 0 left X, 1 left Y (+ down), 2 right X,
//    3 right Y (+ down), 4 left trigger, 5 right trigger; buttons 0 A, 1 B,
//    2 X, 3 Y. Right stick cyclic, left stick Y rate collective, left
//    stick X pedals; A trim, B brakes, Y trim reset.
//  flightgear: FlightGear's joystick[0] axes and buttons as FG numbers them.
//    The defaults assume an Xbox pad on Windows (axes 0 left X, 1 left Y,
//    3 right Y, 4 right X, as FG's own Xbox 360 binding file reads them);
//    check them with FG's joystick dialog and override as needed.
//  joystick: 0 roll, 1 pitch (+ aft), 2 twist (pedals), 3 throttle lever
//    (absolute collective, + down); buttons 0 trim, 1 brakes, 2 trim reset.
//  mouse_keyboard: the window fills axes 0 mouse X, 1 mouse Y (+ down) from
//    the pointer, 2 the collective keys and 3 the pedal keys; buttons 0 trim,
//    1 brakes, 2 trim reset (see the station's key help).
ControlsMapConfig default_map(InputSource s) noexcept;

// Applies a mapping file's text over `base`. `origin` names it in errors.
ControlsMapConfig parse_map(std::string_view text, const ControlsMapConfig& base,
                            std::string_view origin);
ControlsMapConfig read_map_file(const std::string& path, const ControlsMapConfig& base);

// Writes `map` in the file format; parse_map(written, anything) gives it back.
void write_map(std::ostream& out, const ControlsMapConfig& map);

}  // namespace station
