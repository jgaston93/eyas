#pragma once

#include <algorithm>

// Control-system blocks shared by the engine models (D-053, D-054).
namespace eyas::engine {

// The output of (1 + a s)/(1 + b s) u realized with state z, ż = (u − z)/b.
inline double lead_lag(double z, double u, double a, double b) noexcept {
    return z + (a / b) * (u - z);
}

// A limited integrator's rate (GenHel's limiters, pp. 5.6-12, 5.6-13; Ballin's,
// TM-100991 Figs. C3, C7, C10): its input inside the range and at a limit it
// pulls off, zero at a limit it pushes against. Past a limit (an integration
// step's overshoot) it returns to the limit within kLimitReturn, so that the
// limit is the only state there that stays: everywhere past it the model reads
// the limit, and trim would otherwise face a flat run of equivalent values
// with no derivative to follow (D-053 amendment 2).
inline constexpr double kLimitReturn = 0.05;  // [s]

inline double limited_rate(double x, double rate, double lo, double hi) noexcept {
    if (x > hi) {
        return std::min(rate, 0.0) + (hi - x) / kLimitReturn;
    }
    if (x < lo) {
        return std::max(rate, 0.0) + (lo - x) / kLimitReturn;
    }
    if ((x == hi && rate > 0.0) || (x == lo && rate < 0.0)) {
        return 0.0;
    }
    return rate;
}

// True when a limited integrator sits on a limit its input pushes against.
inline bool pushes_on_limit(double x, double rate, double lo, double hi) noexcept {
    return (x >= hi && rate > 0.0) || (x <= lo && rate < 0.0);
}

}  // namespace eyas::engine
