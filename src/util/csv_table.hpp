#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "util/table.hpp"
#include "util/units.hpp"

// Table files in "long" CSV form, one row per grid point, loaded at init time:
//
//     # source: GenHel NASA CR-166309, Fig. 1.5.2(a), p. 5.1-51 (PDF p. 71)
//     # status: placeholder
//     alpha[deg],mach[1],cl[1]
//     -180,0.0,0.0
//     ...
//
// - Lines starting with '#' are comments. "# key: value" comments are
//   metadata; a non-empty "source" is mandatory, so no table enters the model
//   without provenance. "status: placeholder" marks a data gap (DATA_GAPS.md).
// - The first non-comment line is the header. Every column is "name[unit]",
//   and values are converted to SI on load (units::si_scale).
// - Grid tables may list their rows in any order, but every grid point must
//   appear exactly once.
namespace eyas {

struct CsvTable {
    std::string origin;                        // file path or test label
    std::map<std::string, std::string> meta;   // "# key: value" lines
    std::vector<std::string> names;            // column names, without units
    std::vector<std::vector<double>> columns;  // SI values, one vector per column
    std::vector<units::Dimension> dimensions;  // of each column's unit

    const std::string& source() const { return meta.at("source"); }
    bool is_placeholder() const;

    // Throws InitError if there is no column with this name, or if names,
    // columns and dimensions differ in length (a hand-built table).
    const std::vector<double>& column(std::string_view name) const;

    // Throws InitError, naming the column and its unit's dimension, unless the
    // column exists and its unit has dimension `d` (D-021).
    void require_dimension(std::string_view name, units::Dimension d) const;

private:
    std::size_t index(std::string_view name) const;  // checked, as column()
};

// Throw InitError, naming the origin and line, on any format problem.
CsvTable read_csv_table(const std::filesystem::path& path);
CsvTable parse_csv_table(std::string_view text, std::string_view origin);

Table1D make_table1d(const CsvTable& csv, std::string_view x, std::string_view value);
Table2D make_table2d(const CsvTable& csv, std::string_view x, std::string_view y,
                     std::string_view value);
Table3D make_table3d(const CsvTable& csv, std::string_view x, std::string_view y,
                     std::string_view z, std::string_view value);

namespace detail {

// The axes and row-major value grid for any number of axis columns. Rows may
// come in any order; each grid point must appear exactly once.
struct Grid {
    std::vector<Axis> axes;
    std::vector<double> values;
};
Grid build_grid(const CsvTable& csv, const std::vector<std::string_view>& axis_names,
                std::string_view value_name);

}  // namespace detail

// Several value columns on one (x, y) grid, e.g. airfoil C_l and C_d.
template <std::size_t N>
MultiTable2D<N> make_multi_table2d(const CsvTable& csv, std::string_view x, std::string_view y,
                                   const std::array<std::string_view, N>& values) {
    static_assert(N > 0);
    std::array<std::vector<double>, N> v;
    detail::Grid g = detail::build_grid(csv, {x, y}, values[0]);
    v[0] = std::move(g.values);
    for (std::size_t k = 1; k < N; ++k) {
        // Same axis columns, so the same axes as values[0].
        v[k] = detail::build_grid(csv, {x, y}, values[k]).values;
    }
    return MultiTable2D<N>(std::move(g.axes[0]), std::move(g.axes[1]), v);
}

}  // namespace eyas
