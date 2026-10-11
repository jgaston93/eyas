#pragma once

#include <array>
#include <cstddef>
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

    // Value at positions already located on this table's axes, so several
    // tables on the same axes can share one search (e.g. airfoil C_l and C_d).
    double eval(AxisPosition px, AxisPosition py) const noexcept {
        const std::size_t i = px.index;
        const std::size_t j = py.index;
        const double v0 = detail::lerp(at(i, j), at(i, j + 1), py.t);
        const double v1 = detail::lerp(at(i + 1, j), at(i + 1, j + 1), py.t);
        return detail::lerp(v0, v1, px.t);
    }

private:
    double at(std::size_t i, std::size_t j) const noexcept { return v_[i * y_.size() + j]; }

    Axis x_;
    Axis y_;
    std::vector<double> v_;
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
