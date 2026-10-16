#include "util/table.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "util/error.hpp"

namespace eyas {
namespace detail {

void check_table_values(const std::vector<double>& values, std::size_t expected, const char* kind) {
    if (values.size() != expected) {
        throw InitError(std::string(kind) + ": expected " + std::to_string(expected) +
                        " values for the grid, got " + std::to_string(values.size()));
    }
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (!std::isfinite(values[i])) {
            throw InitError(std::string(kind) + ": value " + std::to_string(i) + " is not finite");
        }
    }
}

}  // namespace detail

Table1D::Table1D(Axis x, std::vector<double> values) : x_(std::move(x)), v_(std::move(values)) {
    detail::check_table_values(v_, x_.size(), "Table1D");
}

Table2D::Table2D(Axis x, Axis y, std::vector<double> values)
    : x_(std::move(x)), y_(std::move(y)), v_(std::move(values)) {
    detail::check_table_values(v_, x_.size() * y_.size(), "Table2D");
}

Table3D::Table3D(Axis x, Axis y, Axis z, std::vector<double> values)
    : x_(std::move(x)), y_(std::move(y)), z_(std::move(z)), v_(std::move(values)) {
    detail::check_table_values(v_, x_.size() * y_.size() * z_.size(), "Table3D");
}

double rounded(const Table1D& t, double x, double fraction, TableCursor<1>& cursor) noexcept {
    const double linear = t(x, cursor);
    const std::span<const double> bp = t.x_axis().breakpoints();
    const std::size_t n = bp.size();
    if (n < 2 || !(fraction > 0.0)) {
        return linear;
    }
    // The breakpoint nearest x (the end one beyond the ends).
    const AxisPosition p = t.x_axis().locate(x, cursor.hint[0]);
    const std::size_t j = p.t < 0.5 ? p.index : p.index + 1;
    const double xj = bp[j];
    const double vj = t(xj);
    const double inf = std::numeric_limits<double>::infinity();
    const double hl = j > 0 ? xj - bp[j - 1] : inf;
    const double hr = j + 1 < n ? bp[j + 1] - xj : inf;
    const double sl = j > 0 ? (vj - t(bp[j - 1])) / hl : 0.0;
    const double sr = j + 1 < n ? (t(bp[j + 1]) - vj) / hr : 0.0;
    const double d = fraction * std::min(hl, hr);
    const double u = x - xj;
    if (!(std::abs(u) < d) || sl == sr) {
        return linear;
    }
    // q(u) = v_j + s_l u + (s_r − s_l)(u + δ)²/(4δ): q and q′ equal the left
    // piece at u = −δ and the right piece at u = +δ.
    return vj + sl * u + (sr - sl) * (u + d) * (u + d) / (4.0 * d);
}

}  // namespace eyas
