#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <utility>
#include <vector>

#include "util/axis.hpp"

// Linear interpolation tables in 1, 2 and 3 dimensions. Tables are built at
// init time; lookups never allocate, throw or do I/O (spec §7). Queries
// outside the breakpoints clamp to the edge values.
namespace eyas {

// Per-caller hunt-search hints, one per axis. Any hint values give identical
// results; they only make repeated nearby lookups faster.
template <std::size_t N>
struct TableCursor {
    std::array<std::size_t, N> hint{};
};

namespace detail {

// Exact at t = 0 and t = 1, so node values are reproduced bit-for-bit.
inline double lerp(double v0, double v1, double t) noexcept {
    return (1.0 - t) * v0 + t * v1;
}

// Throws InitError unless `values` has `expected` entries, all finite.
void check_table_values(const std::vector<double>& values, std::size_t expected, const char* kind);

}  // namespace detail

class Table1D {
public:
    // values[i] = f(x_i). Throws InitError on a size mismatch or non-finite value.
    Table1D(Axis x, std::vector<double> values);

    double operator()(double x) const noexcept { return eval(x_.locate(x)); }
    double operator()(double x, TableCursor<1>& cursor) const noexcept {
        return eval(x_.locate(x, cursor.hint[0]));
    }

    const Axis& x_axis() const noexcept { return x_; }

private:
    double eval(AxisPosition px) const noexcept {
        return detail::lerp(v_[px.index], v_[px.index + 1], px.t);
    }

    Axis x_;
    std::vector<double> v_;
};

// A 1-D table read with its corners rounded (D-038): within δ_j of each
// breakpoint x_j where the slope changes (the constant extension beyond the
// ends counts as a piece), the parabola tangent to both neighbouring pieces
// at x_j ± δ_j, δ_j = fraction × the shorter neighbouring segment, fraction
// in (0, 0.5] so the bands don't overlap. The result is C¹ and equals the
// linear table outside the bands; at x_j it differs from the node value by
// (s_right − s_left) δ_j / 4. A trim on a breakpoint then has one slope
// (as D-036 for the flow-angle tables).
double rounded(const Table1D& t, double x, double fraction, TableCursor<1>& cursor) noexcept;

// The smallest value the table can return: interpolation and the clamped
// ends never go below the smallest node value.
inline double min_value(const Table1D& t) noexcept {
    const std::span<const double> bp = t.x_axis().breakpoints();
    double m = t(bp[0]);
    for (const double x : bp) {
        m = t(x) < m ? t(x) : m;
    }
    return m;
}

class Table2D {
public:
    // Row-major: values[i * ny + j] = f(x_i, y_j).
    Table2D(Axis x, Axis y, std::vector<double> values);

    double operator()(double x, double y) const noexcept {
        return eval(x_.locate(x), y_.locate(y));
    }
    double operator()(double x, double y, TableCursor<2>& cursor) const noexcept {
        return eval(x_.locate(x, cursor.hint[0]), y_.locate(y, cursor.hint[1]));
    }

    const Axis& x_axis() const noexcept { return x_; }
    const Axis& y_axis() const noexcept { return y_; }

private:
    double eval(AxisPosition px, AxisPosition py) const noexcept {
        const std::size_t i = px.index;
        const std::size_t j = py.index;
        const double v0 = detail::lerp(at(i, j), at(i, j + 1), py.t);
        const double v1 = detail::lerp(at(i + 1, j), at(i + 1, j + 1), py.t);
        return detail::lerp(v0, v1, px.t);
    }
    double at(std::size_t i, std::size_t j) const noexcept { return v_[i * y_.size() + j]; }

    Axis x_;
    Axis y_;
    std::vector<double> v_;
};

// N quantities tabulated on one (x, y) grid, e.g. airfoil C_l and C_d. One
// lookup locates both axes once and interpolates every quantity, so they
// can't be looked up on mismatched axes. Each result equals what a Table2D of
// that quantity alone would return, bit for bit.
template <std::size_t N>
class MultiTable2D {
public:
    // values[k] is quantity k, row-major: values[k][i * ny + j] = f_k(x_i, y_j).
    // Throws InitError on a size mismatch or non-finite value.
    MultiTable2D(Axis x, Axis y, const std::array<std::vector<double>, N>& values)
        : x_(std::move(x)), y_(std::move(y)), v_(x_.size() * y_.size() * N) {
        const std::size_t n = x_.size() * y_.size();
        for (std::size_t k = 0; k < N; ++k) {
            detail::check_table_values(values[k], n, "MultiTable2D");
            for (std::size_t p = 0; p < n; ++p) {
                v_[p * N + k] = values[k][p];  // interleaved: one grid point's N values together
            }
        }
    }

    std::array<double, N> operator()(double x, double y) const noexcept {
        return eval(x_.locate(x), y_.locate(y));
    }
    std::array<double, N> operator()(double x, double y, TableCursor<2>& cursor) const noexcept {
        return eval(x_.locate(x, cursor.hint[0]), y_.locate(y, cursor.hint[1]));
    }

    const Axis& x_axis() const noexcept { return x_; }
    const Axis& y_axis() const noexcept { return y_; }

private:
    std::array<double, N> eval(AxisPosition px, AxisPosition py) const noexcept {
        const double* v00 = at(px.index, py.index);
        const double* v01 = at(px.index, py.index + 1);
        const double* v10 = at(px.index + 1, py.index);
        const double* v11 = at(px.index + 1, py.index + 1);
        std::array<double, N> out{};
        for (std::size_t k = 0; k < N; ++k) {  // same operation order as Table2D::eval
            const double v0 = detail::lerp(v00[k], v01[k], py.t);
            const double v1 = detail::lerp(v10[k], v11[k], py.t);
            out[k] = detail::lerp(v0, v1, px.t);
        }
        return out;
    }
    const double* at(std::size_t i, std::size_t j) const noexcept {
        return v_.data() + (i * y_.size() + j) * N;
    }

    Axis x_;
    Axis y_;
    std::vector<double> v_;  // interleaved, (i * ny + j) * N + k
};

class Table3D {
public:
    // Row-major: values[(i * ny + j) * nz + k] = f(x_i, y_j, z_k).
    Table3D(Axis x, Axis y, Axis z, std::vector<double> values);

    double operator()(double x, double y, double z) const noexcept {
        return eval(x_.locate(x), y_.locate(y), z_.locate(z));
    }
    double operator()(double x, double y, double z, TableCursor<3>& cursor) const noexcept {
        return eval(x_.locate(x, cursor.hint[0]), y_.locate(y, cursor.hint[1]),
                    z_.locate(z, cursor.hint[2]));
    }

    const Axis& x_axis() const noexcept { return x_; }
    const Axis& y_axis() const noexcept { return y_; }
    const Axis& z_axis() const noexcept { return z_; }

private:
    double at(std::size_t i, std::size_t j, std::size_t k) const noexcept {
        return v_[(i * y_.size() + j) * z_.size() + k];
    }
    double eval_xy(std::size_t i, AxisPosition py, AxisPosition pz) const noexcept {
        const std::size_t j = py.index;
        const std::size_t k = pz.index;
        const double v0 = detail::lerp(at(i, j, k), at(i, j, k + 1), pz.t);
        const double v1 = detail::lerp(at(i, j + 1, k), at(i, j + 1, k + 1), pz.t);
        return detail::lerp(v0, v1, py.t);
    }
    double eval(AxisPosition px, AxisPosition py, AxisPosition pz) const noexcept {
        return detail::lerp(eval_xy(px.index, py, pz), eval_xy(px.index + 1, py, pz), px.t);
    }

    Axis x_;
    Axis y_;
    Axis z_;
    std::vector<double> v_;
};

}  // namespace eyas
