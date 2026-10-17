#pragma once

#include <cmath>
#include <cstdint>

// Host-frame arithmetic shared by eyas_drive and the pilot station, so a
// recorded run replays with exactly the same frames (D-006, D-044).
namespace examples {

// The nominal integer frame period [ns] for a host rate [Hz], rounded.
inline std::int64_t nominal_frame_ns(int rate) noexcept {
    return (1'000'000'000 + rate / 2) / rate;
}

// The number of frames of `frame_ns` that make up `duration` seconds.
inline std::int64_t frames_for(double duration, std::int64_t frame_ns) noexcept {
    return static_cast<std::int64_t>(std::llround(duration * 1e9 / static_cast<double>(frame_ns)));
}

}  // namespace examples
