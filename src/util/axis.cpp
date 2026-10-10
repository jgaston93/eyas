#include "util/axis.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "util/error.hpp"

namespace eyas {

Axis::Axis(std::vector<double> breakpoints) : bp_(std::move(breakpoints)) {
    if (bp_.size() < 2) {
        throw InitError("axis needs at least 2 breakpoints, got " + std::to_string(bp_.size()));
    }
    for (std::size_t i = 0; i < bp_.size(); ++i) {
        if (!std::isfinite(bp_[i])) {
            throw InitError("axis breakpoint " + std::to_string(i) + " is not finite");
        }
        if (i > 0 && !(bp_[i] > bp_[i - 1])) {
            throw InitError("axis breakpoints must be strictly increasing (breakpoint " +
                            std::to_string(i) + ")");
        }
    }
}

AxisPosition Axis::at_interval(std::size_t i, double x) const noexcept {
    return {i, (x - bp_[i]) / (bp_[i + 1] - bp_[i])};
}

// Handles NaN and out-of-range queries. Sets `done` when the returned
// position is final.
AxisPosition Axis::clamp_or_nan(double x, bool& done) const noexcept {
    const std::size_t last = bp_.size() - 2;
    done = true;
    if (std::isnan(x)) {
        return {0, std::numeric_limits<double>::quiet_NaN()};
    }
    if (x <= bp_.front()) {
        return {0, 0.0};
    }
    if (x >= bp_.back()) {
        return {last, 1.0};
    }
    done = false;
    return {};
}

// For interior x, both searches return the unique interval i with
// bp[i] <= x < bp[i+1], so results never depend on the search path.
AxisPosition Axis::locate(double x) const noexcept {
    bool done = false;
    const AxisPosition edge = clamp_or_nan(x, done);
    if (done) {
        return edge;
    }
    const auto it = std::upper_bound(bp_.begin(), bp_.end(), x);
    const auto i = static_cast<std::size_t>(it - bp_.begin()) - 1;
    return at_interval(i, x);
}

AxisPosition Axis::locate(double x, std::size_t& hint) const noexcept {
    const std::size_t last = bp_.size() - 2;
    bool done = false;
    const AxisPosition edge = clamp_or_nan(x, done);
    if (done) {
        hint = edge.index;
        return edge;
    }

    std::size_t lo = std::min(hint, last);
    std::size_t hi = lo + 1;
    if (bp_[lo] <= x && x < bp_[hi]) {
        hint = lo;
        return at_interval(lo, x);
    }

    // Hunt: grow a bracket [lo, hi] with bp[lo] <= x < bp[hi] by doubling
    // steps away from the hint, then bisect it.
    std::size_t step = 1;
    if (x >= bp_[hi]) {
        lo = hi;
        while (true) {
            hi = std::min(lo + step, bp_.size() - 1);
            if (x < bp_[hi]) {
                break;
            }
            lo = hi;
            step *= 2;
        }
    } else {
        hi = lo;
        while (true) {
            lo = hi > step ? hi - step : 0;
            if (bp_[lo] <= x) {
                break;
            }
            hi = lo;
            step *= 2;
        }
    }
    while (hi - lo > 1) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (bp_[mid] <= x) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    hint = lo;
    return at_interval(lo, x);
}

}  // namespace eyas
