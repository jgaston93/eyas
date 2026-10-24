#pragma once

#include <array>
#include <limits>

#include "common/afcs.hpp"
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
    int sas_button = -1;         // pressed: toggles the SAS, both channels together (M8c2)
    int pba_button = -1;         // pressed: toggles the pitch bias actuator
    int fps_button = -1;         // pressed: toggles the FPS (M8c3)
    // After a trim release the control holds at its trim point until the
    // shaped axis value comes back within this of centre (a spring-centred
    // stick returns into its deadband), so letting go of the stick doesn't
    // move the control; resuming then moves it by at most gain × recentre.
    double recentre = 0.02;
    // "Feet on the pedals" for the FPS turn switch (GenHel p. 5.5-3): the
    // pedal axis, shaped, beyond this fraction of its travel (a station
    // assumption, D-045 M8c3). [0, 1]
    double feet_on_pedals = 0.1;
    // A lever collective takes over once within this fraction of its travel
    // of the held value (or when it passes it in a small step): pickup, so
    // that a start doesn't jump the collective to the lever. [0, 0.5].
    double pickup = 0.02;
    // Engine controls (D-055). The power lever: an absolute lever (mode
    // absolute only), full travel from the idle stop (−1) to fly (+1) at
    // gain 1, the gain scaling it about the middle as the collective's; it
    // picks up as the collective does (`pickup`), starting at the start's
    // lever positions.
    // Unmapped, the engines stay at the start's (fly).
    AxisMap power_lever;
    // Pressed: cycles which engines the lever moves: all, engine 1, engine 2,
    // all (the others hold where they are). < 0: all, always.
    int engine_select_button = -1;
    // Held: the engine-speed trim (Controls::rotor_speed_reference) moves at
    // rpm_rate [1/s] within [rpm_low, rpm_high], a station assumption around
    // the 94.6-102.7 % of rotor speed USAAEFA flew the UH-60A at (ADA180181
    // Table 1). < 0: none.
    int rpm_up_button = -1;
    int rpm_down_button = -1;
    double rpm_rate = 0.01;
    double rpm_low = 0.94;
    double rpm_high = 1.03;
};

// How many engines the station's power lever reaches (Controls::engines).
inline constexpr int kStationEngines = 2;  // ControlsMap::levers_ starts each at fly

// Increments from the start controls (start + delta = applied).
struct ControlDeltas {
    double collective = 0.0;
    double lat_cyclic = 0.0;
    double lon_cyclic = 0.0;
    double pedal = 0.0;
};

// Deadband, then expo: odd, continuous, 0 inside the deadband, ±1 at ±1.
double shape(double raw, double deadband, double expo) noexcept;

// The FPS turn switch's latch (common/afcs.hpp).
using examples::latch_turn_switch;
using examples::TurnSwitchInputs;

class ControlsMap {
public:
    ControlsMap() = default;
    ControlsMap(const ControlsMapConfig& config, const eyas::Controls& start) noexcept;

    // Starts again from `start`: trim points at it, deltas zero, the brakes
    // and the flight control system's switches as `start` has them.
    void restart(const eyas::Controls& start) noexcept;

    // One host frame of dt seconds. Returns the applied controls, each
    // within its D-024 range. The trim button is also the AFCS's trim
    // release (held: AfcsSwitches::trim_release), and `f` drives the FPS
    // turn switch's latch (latch_turn_switch()).
    eyas::Controls update(const RawInput& in, double dt, const TurnSwitchInputs& f = {}) noexcept;

    const ControlDeltas& deltas() const noexcept { return delta_; }
    // The trim points (increments from start) for the display.
    const ControlDeltas& trim_points() const noexcept { return trim_; }
    const eyas::Controls& start() const noexcept { return start_; }
    bool trim_held() const noexcept { return trim_held_; }
    // An absolute collective (a lever) holds its value after each start, and
    // after rearm_pickup(), until the lever is brought to it (pickup). While
    // held, the HUD shows where the lever is: collective_lever() (applied
    // units), once collective_lever_known() (a sample has arrived).
    bool collective_held() const noexcept {
        return config_.collective.axis >= 0 && config_.collective.mode == AxisMode::absolute &&
               lever_held_;
    }
    double collective_lever() const noexcept { return start_.collective + lever_target_; }
    bool collective_lever_known() const noexcept { return lever_seen_; }
    // Holds the collective where it is until the lever is brought to it
    // again: after the input was lost, a device may come back reading
    // something else (FlightGear restarted reads an untouched axis as 0).
    void rearm_pickup() noexcept;
    // The largest lever step between two samples that still counts as
    // passing the held value [fraction of travel]: a bigger one is a jump
    // (e.g. FlightGear's first real reading of an untouched axis), not a pass.
    static constexpr double kPickupStep = 0.05;
    bool brakes_set() const noexcept { return brakes_; }
    const eyas::AfcsSwitches& afcs() const noexcept { return afcs_; }
    // Engine controls (D-055): each engine's lever, which engines the lever
    // moves (0 all, else engine k), whether engine e's lever is held until
    // the lever is brought to it, and the engine-speed trim.
    double power_lever(int e) const noexcept { return levers_[static_cast<std::size_t>(e)]; }
    int engine_selection() const noexcept { return selection_; }
    bool power_lever_held(int e) const noexcept {
        return config_.power_lever.axis >= 0 && levers_held_[static_cast<std::size_t>(e)];
    }
    double rotor_speed_reference() const noexcept { return speed_reference_; }

private:
    // A trimmable control (cyclic, pedal) or a rate collective.
    struct Channel {
        double* delta;
        double* trim;
        bool* latched;
        double lo;  // control range
        double hi;
        double start;
    };
    void update_channel(const AxisMap& m, const Channel& ch, const RawInput& in, double dt,
                        bool released) noexcept;
    // An absolute collective: a lever, with pickup.
    void update_lever(const AxisMap& m, const RawInput& in) noexcept;
    // The power lever, the engine selection and the speed trim (D-055).
    void update_engines(const RawInput& in, double dt) noexcept;
    void hold_levers() noexcept;

    ControlsMapConfig config_;
    eyas::Controls start_;
    ControlDeltas delta_;
    ControlDeltas trim_;
    std::array<bool, 4> latched_{};  // collective, lat, lon, pedal
    bool trim_held_ = false;
    bool brakes_ = false;
    bool brake_was_pressed_ = false;
    eyas::AfcsSwitches afcs_;
    bool sas_was_pressed_ = false;
    bool pba_was_pressed_ = false;
    bool fps_was_pressed_ = false;
    bool reset_was_pressed_ = false;
    bool lever_held_ = true;
    double lever_prev_ = std::numeric_limits<double>::quiet_NaN();
    double lever_target_ = 0.0;
    bool lever_seen_ = false;
    std::array<double, kStationEngines> levers_{1.0, 1.0};  // fly, as EngineControls
    std::array<bool, kStationEngines> levers_held_{};
    double levers_prev_ = std::numeric_limits<double>::quiet_NaN();
    int selection_ = 0;
    bool select_was_pressed_ = false;
    double speed_reference_ = 1.0;
};

// The applied controls for a delta: start + delta, the sum eyas_drive makes,
// so both programs compute the same doubles. ControlsMap keeps each delta
// at clamp(start + d) − start, which puts the sum within its range to an
// ulp. The brakes, the flight control system's switches and the engine
// controls are the start's.
eyas::Controls apply(const eyas::Controls& start, const ControlDeltas& d) noexcept;

}  // namespace station
