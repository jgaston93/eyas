#include "controls_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace station {

double shape(double raw, double deadband, double expo) noexcept {
    if (!std::isfinite(raw)) {
        return 0.0;
    }
    const double a = std::clamp(std::abs(raw), 0.0, 1.0);
    const double db = std::clamp(deadband, 0.0, 0.99);
    const double x = a <= db ? 0.0 : (a - db) / (1.0 - db);
    const double e = std::clamp(expo, 0.0, 1.0);
    const double y = (1.0 - e) * x + e * x * x * x;
    return raw < 0.0 ? -y : y;
}

bool latch_turn_switch(bool on, const eyas::AfcsSwitches& s, bool feet_on_pedals,
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

ControlsMap::ControlsMap(const ControlsMapConfig& config, const eyas::Controls& start) noexcept
    : config_(config) {
    restart(start);
}

void ControlsMap::restart(const eyas::Controls& start) noexcept {
    start_ = start;
    delta_ = {};
    trim_ = {};
    latched_ = {};
    trim_held_ = false;
    brakes_ = start.brake_left > 0.5 || start.brake_right > 0.5;
    afcs_ = start.afcs;
    // A lever holds the start until it is brought to it (pickup).
    rearm_pickup();
}

namespace {

bool pressed(const RawInput& in, int button) noexcept {
    return button >= 0 && button < kMaxButtons && in.buttons[static_cast<std::size_t>(button)];
}

double axis_value(const RawInput& in, const AxisMap& m) noexcept {
    if (m.axis < 0 || m.axis >= kMaxAxes) {
        return 0.0;
    }
    const double raw = in.axes[static_cast<std::size_t>(m.axis)];
    return m.invert ? -raw : raw;
}

}  // namespace

// Not const: it changes this map's state, through `ch`'s pointers.
// NOLINTNEXTLINE(readability-make-member-function-const)
void ControlsMap::update_channel(const AxisMap& m, const Channel& ch, const RawInput& in, double dt,
                                 bool released) noexcept {
    if (m.axis < 0) {
        return;
    }
    const double raw = axis_value(in, m);
    const double s = shape(raw, m.deadband, m.expo);
    // The delta range that keeps start + delta within [lo, hi].
    const double dlo = ch.lo - ch.start;
    const double dhi = ch.hi - ch.start;
    double d = *ch.delta;
    if (m.mode == AxisMode::rate) {
        d += m.gain * s * dt;
    } else {
        // About the trim point, with force trim.
        const double half = 0.5 * (ch.hi - ch.lo);
        if (released) {
            // Force trim: the control stays where the pilot left it.
            *ch.trim = std::clamp(*ch.delta, dlo, dhi);
            *ch.latched = true;
        }
        if (*ch.latched && std::abs(s) <= config_.recentre) {
            *ch.latched = false;
        }
        d = *ch.latched ? *ch.trim : *ch.trim + m.gain * half * s;
    }
    *ch.delta = std::clamp(d, dlo, dhi);
}

void ControlsMap::update_lever(const AxisMap& m, const RawInput& in) noexcept {
    if (m.axis < 0) {
        return;
    }
    // The collective's range is [0, 1]; the lever's middle is its middle.
    const double s = shape(axis_value(in, m), m.deadband, m.expo);
    const double dlo = -start_.collective;
    const double dhi = 1.0 - start_.collective;
    const double target = std::clamp(0.5 + m.gain * 0.5 * s - start_.collective, dlo, dhi);
    lever_target_ = target;
    lever_seen_ = true;
    if (lever_held_) {
        // Pickup: hold the collective until the lever reaches it, or passes
        // it in a small step between two samples, so a start never jumps
        // the collective to wherever the lever happens to be. A big step
        // isn't a pass: through FlightGear an untouched axis reads 0 (mid
        // travel) until it moves, then jumps to the real value (code review).
        const double d = delta_.collective;
        const double gap = target - d;
        const double prev_gap = lever_prev_ - d;  // NaN before the first sample
        const bool near = std::abs(gap) <= config_.pickup;
        const bool crossed = prev_gap * gap < 0.0 && std::abs(target - lever_prev_) <= kPickupStep;
        if (near || crossed) {
            lever_held_ = false;
        }
    }
    lever_prev_ = target;
    if (!lever_held_) {
        delta_.collective = target;
    }
}

void ControlsMap::rearm_pickup() noexcept {
    lever_held_ = true;
    lever_prev_ = std::numeric_limits<double>::quiet_NaN();
    lever_target_ = 0.0;
    lever_seen_ = false;
}

eyas::Controls ControlsMap::update(const RawInput& in, double dt,
                                   const TurnSwitchInputs& f) noexcept {
    const bool trim_now = pressed(in, config_.trim_button);
    const bool released = trim_held_ && !trim_now;
    trim_held_ = trim_now;

    const bool reset_now = pressed(in, config_.trim_reset_button);
    if (reset_now && !reset_was_pressed_) {
        trim_ = {};
        latched_ = {};
    }
    reset_was_pressed_ = reset_now;

    const bool brake_now = pressed(in, config_.brake_button);
    if (brake_now && !brake_was_pressed_) {
        brakes_ = !brakes_;
    }
    brake_was_pressed_ = brake_now;

    // The SAS toggles both channels: off if either is on, else both on.
    const bool sas_now = pressed(in, config_.sas_button);
    if (sas_now && !sas_was_pressed_) {
        const bool on = !(afcs_.sas_digital || afcs_.sas_analog);
        afcs_.sas_digital = on;
        afcs_.sas_analog = on;
    }
    sas_was_pressed_ = sas_now;
    const bool pba_now = pressed(in, config_.pba_button);
    if (pba_now && !pba_was_pressed_) {
        afcs_.pitch_bias = !afcs_.pitch_bias;
    }
    pba_was_pressed_ = pba_now;
    const bool fps_now = pressed(in, config_.fps_button);
    if (fps_now && !fps_was_pressed_) {
        afcs_.fps = !afcs_.fps;
    }
    fps_was_pressed_ = fps_now;
    // The force trim release is the AFCS's trim release too (D-045 M8c3),
    // and with the pedals it works the FPS turn switch's latch.
    afcs_.trim_release = trim_now;
    const double pedal =
        shape(axis_value(in, config_.pedal), config_.pedal.deadband, config_.pedal.expo);
    const bool feet = config_.pedal.axis >= 0 && std::abs(pedal) > config_.feet_on_pedals;
    afcs_.fps_turn = latch_turn_switch(afcs_.fps_turn, afcs_, feet, f);

    if (config_.collective.mode == AxisMode::absolute) {
        update_lever(config_.collective, in);
    } else {
        update_channel(
            config_.collective,
            {&delta_.collective, &trim_.collective, latched_.data(), 0.0, 1.0, start_.collective},
            in, dt, released);
    }
    update_channel(
        config_.lat_cyclic,
        {&delta_.lat_cyclic, &trim_.lat_cyclic, latched_.data() + 1, -1.0, 1.0, start_.lat_cyclic},
        in, dt, released);
    update_channel(
        config_.lon_cyclic,
        {&delta_.lon_cyclic, &trim_.lon_cyclic, latched_.data() + 2, -1.0, 1.0, start_.lon_cyclic},
        in, dt, released);
    update_channel(config_.pedal,
                   {&delta_.pedal, &trim_.pedal, latched_.data() + 3, -1.0, 1.0, start_.pedal}, in,
                   dt, released);

    eyas::Controls c = apply(start_, delta_);
    c.brake_left = brakes_ ? 1.0 : 0.0;
    c.brake_right = c.brake_left;
    c.afcs = afcs_;
    return c;
}

eyas::Controls apply(const eyas::Controls& start, const ControlDeltas& d) noexcept {
    eyas::Controls c = start;
    c.collective += d.collective;
    c.lat_cyclic += d.lat_cyclic;
    c.lon_cyclic += d.lon_cyclic;
    c.pedal += d.pedal;
    return c;
}

}  // namespace station
