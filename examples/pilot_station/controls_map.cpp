#include "controls_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
    // A lever holds the start until it is brought to it (pickup).
    lever_held_ = true;
    lever_prev_ = std::numeric_limits<double>::quiet_NaN();
    lever_target_ = 0.0;
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

void ControlsMap::update_channel(const AxisMap& m, const Channel& ch, const RawInput& in, double dt,
                                 bool released) const noexcept {
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
        const double half = 0.5 * (ch.hi - ch.lo);
        if (ch.trimmable) {
            if (released) {
                // Force trim: the control stays where the pilot left it.
                *ch.trim = std::clamp(*ch.delta, dlo, dhi);
                *ch.latched = true;
            }
            if (*ch.latched && std::abs(s) <= config_.recentre) {
                *ch.latched = false;
            }
            d = *ch.latched ? *ch.trim : *ch.trim + m.gain * half * s;
        } else {
            // A lever: its middle is the middle of the control's range.
            const double target =
                std::clamp(0.5 * (ch.lo + ch.hi) + m.gain * half * s - ch.start, dlo, dhi);
            *ch.lever_target = target;
            if (*ch.lever_held) {
                // Pickup: hold the control until the lever reaches it, or
                // passes it between two samples, so a start never jumps the
                // control to wherever the lever happens to be.
                const double gap = target - d;
                const double prev_gap = *ch.lever_prev - d;  // NaN before the first sample
                const bool near = std::abs(gap) <= config_.pickup * (ch.hi - ch.lo);
                const bool crossed = prev_gap * gap < 0.0;
                if (near || crossed) {
                    *ch.lever_held = false;
                }
            }
            *ch.lever_prev = target;
            if (!*ch.lever_held) {
                d = target;
            }
        }
    }
    *ch.delta = std::clamp(d, dlo, dhi);
}

eyas::Controls ControlsMap::update(const RawInput& in, double dt) noexcept {
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

    update_channel(config_.collective,
                   {&delta_.collective, &trim_.collective, latched_.data(), 0.0, 1.0,
                    start_.collective, false, &lever_held_, &lever_prev_, &lever_target_},
                   in, dt, released);
    update_channel(config_.lat_cyclic,
                   {&delta_.lat_cyclic, &trim_.lat_cyclic, latched_.data() + 1, -1.0, 1.0,
                    start_.lat_cyclic, true},
                   in, dt, released);
    update_channel(config_.lon_cyclic,
                   {&delta_.lon_cyclic, &trim_.lon_cyclic, latched_.data() + 2, -1.0, 1.0,
                    start_.lon_cyclic, true},
                   in, dt, released);
    update_channel(
        config_.pedal,
        {&delta_.pedal, &trim_.pedal, latched_.data() + 3, -1.0, 1.0, start_.pedal, true}, in, dt,
        released);

    eyas::Controls c = apply(start_, delta_);
    c.brake_left = brakes_ ? 1.0 : 0.0;
    c.brake_right = c.brake_left;
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
