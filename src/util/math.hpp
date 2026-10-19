#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

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

// An angle wrapped into (−π, π] [rad], e.g. a heading difference taken the
// short way round.
inline double wrap_pi(double a) noexcept {
    a = std::remainder(a, 2.0 * std::numbers::pi);
    return a == -std::numbers::pi ? std::numbers::pi : a;
}

}  // namespace eyas
