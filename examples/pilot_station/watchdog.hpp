#pragma once

#include <chrono>

// Input watchdog for the pilot station (D-044): a live device must keep
// delivering samples. When none has arrived for `timeout`, or the device is
// gone, the input is stale and the station pauses: the bare aircraft is
// unstable, so flying on with frozen controls is worse than stopping. It
// resumes when samples return. Real time: no allocation.
namespace station {

class InputWatchdog {
public:
    using Clock = std::chrono::steady_clock;

    explicit InputWatchdog(Clock::duration timeout = std::chrono::milliseconds(250)) noexcept
        : timeout_(timeout) {}

    // A sample arrived (or, for a polled device, the device is connected).
    void fresh(Clock::time_point now) noexcept {
        last_ = now;
        seen_ = true;
    }
    // Stale: nothing yet, or nothing for longer than the timeout.
    bool stale(Clock::time_point now) const noexcept { return !seen_ || now - last_ > timeout_; }
    // Whether any sample ever arrived (to tell "waiting" from "lost").
    bool seen() const noexcept { return seen_; }

private:
    Clock::duration timeout_;
    Clock::time_point last_{};
    bool seen_ = false;
};

}  // namespace station
