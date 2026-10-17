#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include "controls_map.hpp"
#include "eyas/aircraft_state.hpp"
#include "mapping.hpp"

// The pilot station's window (D-044): GLFW and fixed-function OpenGL 2.1.
// It draws a simple world (sky, a gridded ground, the pad, a runway and
// scattered blocks for scale and parallax), the aircraft as a symbol, a HUD
// and an engineering readout, and reads the GLFW devices and the keyboard.
// Built only when GLFW and OpenGL are found.
namespace station {

// Commands from the keyboard, collected between polls.
struct ViewCommands {
    bool quit = false;
    bool pause = false;    // toggle
    bool restart = false;  // the current start point again
    int start = -1;        // a start point by index (keys 1–4), or −1
    bool help = false;     // toggle
};

// Everything the HUD shows, filled by the main loop each frame.
struct HudInfo {
    const eyas::AircraftState* state = nullptr;
    eyas::Controls applied;
    eyas::Controls trim_point;  // start + the force-trim point
    // A lever collective waiting for pickup, and where the lever is.
    bool collective_held = false;
    double collective_lever = 0.0;
    double ground_height = 0.0;  // [m] MSL
    std::string_view source;
    std::string_view start;
    bool paused = false;
    bool trim_held = false;
    bool faulted = false;
    bool crashed = false;
    // The input is stale (no sample within the timeout, or no device):
    // waiting = none yet, lost = it stopped. The station is paused meanwhile.
    bool input_waiting = false;
    bool input_lost = false;
    double realtime_ratio = 1.0;  // sim time / wall time over the last second
    double advance_us = 0.0;
    double worst_advance_us = 0.0;
    std::int64_t overruns = 0;
    std::int64_t fg_lines = -1;  // eyas_ctrl lines received; −1: not listening
    bool fg_sending = false;
    int placeholders = 0;
    int recording_segment = 0;  // 0: not recording
};

class GlView {
public:
    // Opens the window; throws std::runtime_error if GLFW or the window
    // can't start. `source` selects which GLFW device poll() reads.
    // `help`: show the key help at start (H toggles it).
    GlView(InputSource source, bool help);
    ~GlView();
    GlView(const GlView&) = delete;
    GlView& operator=(const GlView&) = delete;

    // Processes window events and fills `raw` from the selected source
    // (gamepad, joystick, or mouse and keyboard; other sources leave it
    // alone). Returns the keyboard commands since the last call.
    ViewCommands poll(RawInput& raw);
    void draw(const HudInfo& hud);
    // Writes the next drawn frame to `path` as a binary PPM (for docs).
    void screenshot(std::string path) noexcept;
    // The camera: 0 chase, 1 eye at the CG, 2 ground observer.
    void set_camera(int camera) noexcept;
    // The name of the device being read, or a note that none is connected.
    std::string_view device() const noexcept;
    // For the gamepad and joystick sources: whether the device was there at
    // the last poll().
    bool device_connected() const noexcept;

    struct Impl;

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace station
