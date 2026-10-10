#pragma once

#include <cstddef>
#include <span>
#include <vector>

namespace eyas {

// Position of a query point on an axis: the interval
// [breakpoint(index), breakpoint(index + 1)] and the fraction t in [0, 1]
// within it. Queries outside the axis clamp to the end points (no
// extrapolation). A NaN query gives t = NaN, which propagates to the result.
struct AxisPosition {
    std::size_t index;
    double t;
};

// Strictly increasing, finite breakpoints (at least two). Built at init time.
class Axis {
public:
    // Throws InitError if the breakpoints are invalid.
    explicit Axis(std::vector<double> breakpoints);

    std::size_t size() const noexcept { return bp_.size(); }
    double breakpoint(std::size_t i) const noexcept { return bp_[i]; }
    std::span<const double> breakpoints() const noexcept { return bp_; }

    // Binary search.
    AxisPosition locate(double x) const noexcept;

    // Hunt search starting from the interval in `hint`, which is updated to
    // the interval found. The result never depends on the hint, only the
    // search time does, so a cached hint does not break the purity of
    // derivative evaluations (spec §6, DynamicComponent).
    AxisPosition locate(double x, std::size_t& hint) const noexcept;

private:
    AxisPosition clamp_or_nan(double x, bool& done) const noexcept;
    AxisPosition at_interval(std::size_t i, double x) const noexcept;

    std::vector<double> bp_;
};

}  // namespace eyas
