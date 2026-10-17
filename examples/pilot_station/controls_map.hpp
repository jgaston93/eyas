#pragma once

#include <array>

#include "eyas/types.hpp"

// Pilot-station control mapping (D-044): raw device axes and buttons to
// eyas::Controls. Real time: no allocation, no exceptions.
//
// The map works in increments from the start controls (the trim the station
// started from), the same form eyas_drive's control files use, so a recorded
// run replays there exactly: the applied controls are start + delta,
// computed the same way in both programs.
namespace station {

// Up to kMaxDevices devices at once (a HOTAS is two: stick and throttle),
// each with up to kAxesPerDevice axes and kButtonsPerDevice buttons. They
// are stored flat, device by device: axis i of device d is axes[d *
// kAxesPerDevice + i], and the same for buttons. Map files write them d:i.
inline constexpr int kMaxDevices = 4;
inline constexpr int kAxesPerDevice = 12;
inline constexpr int kButtonsPerDevice = 32;
inline constexpr int kMaxAxes = kMaxDevices * kAxesPerDevice;
inline constexpr int kMaxButtons = kMaxDevices * kButtonsPerDevice;

constexpr int axis_index(int device, int axis) noexcept {
    return device * kAxesPerDevice + axis;
}
constexpr int button_index(int device, int button) noexcept {
    return device * kButtonsPerDevice + button;
}

// One sample of every device. Axes in [-1, 1] as the device reports them;
// buttons pressed or not. Absent devices, axes and buttons stay at 0.
struct RawInput {
    std::array<double, kMaxAxes> axes{};
    std::array<bool, kMaxButtons> buttons{};
};

enum class AxisMode {
    absolute,  // the control follows the axis (spring-centred cyclic, lever collective)
    rate,      // the axis moves the control at `rate` per second; it holds when centred
};

struct AxisMap {
    int axis = -1;           // flat RawInput axis index; < 0: unmapped (control stays at its trim)
    bool invert = false;     // flip the device's sign before anything else
    double deadband = 0.05;  // [-] of the axis travel, removed and the rest rescaled to [-1, 1]
    double expo = 0.0;       // [0, 1] blend of a cubic: 0 linear, 1 pure cubic
    AxisMode mode = AxisMode::absolute;
    // absolute: control travel per unit axis deflection, from the trim
    // point (cyclic and pedal: full axis = ±gain of the [-1, 1] travel;
    // collective: full axis = ±gain of the [0, 1] travel, from 0.5 + trim).
    // rate: control travel per second at full axis deflection [1/s].
    double gain = 1.0;
};

struct ControlsMapConfig {
    AxisMap collective;
    AxisMap lat_cyclic;
    AxisMap lon_cyclic;
    AxisMap pedal;
    // Flat RawInput button indices; < 0: none.
    int trim_button = -1;        // held: force trim release (cyclic and pedal)
    int trim_reset_button = -1;  // pressed: trim points back to the start controls
    int brake_button = -1;       // pressed: toggles the wheel brakes, set or released
    // After a trim release the control holds at its trim point until the
    // shaped axis value comes back within this of centre (a spring-centred
    // stick returns into its deadband), so letting go of the stick doesn't
    // move the control; resuming then moves it by at most gain × recentre.
    double recentre = 0.02;
    // A lever collective takes over once within this fraction of its travel
    // of the held value (or when it passes it): pickup, so that a start
    // doesn't jump the collective to the lever.
    double pickup = 0.02;
};

// Increments from the start controls (start + delta = applied).
struct ControlDeltas {
    double collective = 0.0;
    double lat_cyclic = 0.0;
    double lon_cyclic = 0.0;
    double pedal = 0.0;
};

// Deadband, then expo: odd, continuous, 0 inside the deadband, ±1 at ±1.
double shape(double raw, double deadband, double expo) noexcept;

class ControlsMap {
public:
    ControlsMap() = default;
    ControlsMap(const ControlsMapConfig& config, const eyas::Controls& start) noexcept;

    // Starts again from `start`: trim points at it, deltas zero, the brakes
    // as `start` has them.
    void restart(const eyas::Controls& start) noexcept;

    // One host frame of dt seconds. Returns the applied controls, each
    // within its D-024 range.
    eyas::Controls update(const RawInput& in, double dt) noexcept;

    const ControlDeltas& deltas() const noexcept { return delta_; }
    // The trim points (increments from start) for the display.
    const ControlDeltas& trim_points() const noexcept { return trim_; }
    const eyas::Controls& start() const noexcept { return start_; }
    bool trim_held() const noexcept { return trim_held_; }
    // An absolute collective (a lever) holds the start value after each
    // start until the lever is brought to it (pickup). While held, the HUD
    // shows where the lever is: collective_lever() (applied units).
    bool collective_held() const noexcept {
        return config_.collective.axis >= 0 && config_.collective.mode == AxisMode::absolute &&
               lever_held_;
    }
    double collective_lever() const noexcept { return start_.collective + lever_target_; }
    bool brakes_set() const noexcept { return brakes_; }

private:
    struct Channel {
        double* delta;
        double* trim;
        bool* latched;
        double lo;  // control range
        double hi;
        double start;
        bool trimmable;  // follows the force trim (cyclic, pedal)
        // For a lever (absolute, not trimmable): the pickup state.
        bool* lever_held = nullptr;
        double* lever_prev = nullptr;
        double* lever_target = nullptr;
    };
    // Writes through `ch`'s pointers into this map's state.
    void update_channel(const AxisMap& m, const Channel& ch, const RawInput& in, double dt,
                        bool released) const noexcept;

    ControlsMapConfig config_;
    eyas::Controls start_;
    ControlDeltas delta_;
    ControlDeltas trim_;
    std::array<bool, 4> latched_{};  // collective, lat, lon, pedal
    bool trim_held_ = false;
    bool brakes_ = false;
    bool brake_was_pressed_ = false;
    bool reset_was_pressed_ = false;
    bool lever_held_ = true;
    double lever_prev_ = 0.0;
    double lever_target_ = 0.0;
};

// The applied controls for a delta: start + delta, the sum eyas_drive makes,
// so both programs compute the same doubles. ControlsMap keeps each delta
// at clamp(start + d) − start, which puts the sum within its range to an
// ulp. The brakes are the start's.
eyas::Controls apply(const eyas::Controls& start, const ControlDeltas& d) noexcept;

}  // namespace station
