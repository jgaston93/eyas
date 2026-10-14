#pragma once

#include <algorithm>

// Small numeric helpers shared by the force models. No allocation, no
// exceptions (spec §7).
namespace eyas {

// smoothstep(t) = t²(3 − 2t) on t clamped to [0, 1]: 0 and 1 at the ends,
// zero slope at both. Used for the fades that keep a derivative continuous
// (D-031, D-032 item 2).
inline double smoothstep(double t) noexcept {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace eyas
