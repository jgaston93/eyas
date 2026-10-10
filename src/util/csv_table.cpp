#include "util/csv_table.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include "util/error.hpp"
#include "util/parse.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

std::vector<std::string_view> split(std::string_view line, char sep) {
    std::vector<std::string_view> out;
    std::size_t start = 0;
    while (true) {
        const std::size_t pos = line.find(sep, start);
        out.push_back(detail::trim(line.substr(start, pos - start)));
        if (pos == std::string_view::npos) {
            return out;
        }
        start = pos + 1;
    }
}

// A metadata key is a single word; other comments are free text.
bool is_meta_key(std::string_view key) {
    return !key.empty() && std::all_of(key.begin(), key.end(), [](char c) {
        return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_' || c == '-';
    });
}

[[noreturn]] void fail(std::string_view origin, std::size_t line, const std::string& what) {
    std::string msg(origin);
    if (line > 0) {
        msg += ": line " + std::to_string(line);
    }
    throw InitError(msg + ": " + what);
}

// Sorted, de-duplicated values of a grid axis column.
std::vector<double> unique_sorted(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

// Builds the axes and row-major value grid for any number of axis columns.
// Rows may come in any order; each grid point must appear exactly once.
struct Grid {
    std::vector<Axis> axes;
    std::vector<double> values;
};

Grid build_grid(const CsvTable& csv, const std::vector<std::string_view>& axis_names,
                std::string_view value_name) {
    const std::vector<double>& value_col = csv.column(value_name);
    std::vector<const std::vector<double>*> axis_cols;
    std::vector<std::vector<double>> breakpoints;
    std::size_t total = 1;
    for (const std::string_view name : axis_names) {
        axis_cols.push_back(&csv.column(name));
        breakpoints.push_back(unique_sorted(csv.column(name)));
        total *= breakpoints.back().size();
    }

    std::vector<double> values(total, std::numeric_limits<double>::quiet_NaN());
    std::vector<bool> filled(total, false);
    for (std::size_t row = 0; row < value_col.size(); ++row) {
        std::size_t flat = 0;
        for (std::size_t a = 0; a < axis_cols.size(); ++a) {
            const auto& bp = breakpoints[a];
            const double x = (*axis_cols[a])[row];
            const auto idx =
                static_cast<std::size_t>(std::lower_bound(bp.begin(), bp.end(), x) - bp.begin());
            flat = flat * bp.size() + idx;
        }
        if (filled[flat]) {
            fail(csv.origin, 0,
                 "data row " + std::to_string(row + 1) + " repeats a grid point already given");
        }
        filled[flat] = true;
        values[flat] = value_col[row];
    }
    const auto missing = std::find(filled.begin(), filled.end(), false);
    if (missing != filled.end()) {
        fail(csv.origin, 0,
             "grid is incomplete: " +
                 std::to_string(std::count(filled.begin(), filled.end(), false)) + " of " +
                 std::to_string(total) + " grid points have no row");
    }

    Grid grid;
    try {
        for (auto& bp : breakpoints) {
            grid.axes.emplace_back(std::move(bp));
        }
    } catch (const InitError& e) {
        fail(csv.origin, 0, e.what());
    }
    grid.values = std::move(values);
    return grid;
}

}  // namespace

bool CsvTable::is_placeholder() const {
    const auto it = meta.find("status");
    return it != meta.end() && it->second == "placeholder";
}

const std::vector<double>& CsvTable::column(std::string_view name) const {
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (names[i] == name) {
            return columns[i];
        }
    }
    throw InitError(origin + ": no column named '" + std::string(name) + "'");
}

CsvTable parse_csv_table(std::string_view text, std::string_view origin) {
    CsvTable csv;
    csv.origin = std::string(origin);
    std::vector<double> scales;
    std::size_t line_no = 0;
    std::size_t rows = 0;
    std::size_t start = 0;

    while (start < text.size()) {
        const std::size_t nl = text.find('\n', start);
        const std::string_view line =
            detail::trim(text.substr(start, nl == std::string_view::npos ? nl : nl - start));
        start = nl == std::string_view::npos ? text.size() : nl + 1;
        ++line_no;

        if (line.empty()) {
            continue;
        }
        if (line.front() == '#') {
            const std::string_view body = detail::trim(line.substr(1));
            const std::size_t colon = body.find(':');
            if (colon == std::string_view::npos) {
                continue;
            }
            const std::string key(detail::trim(body.substr(0, colon)));
            if (!is_meta_key(key)) {
                continue;
            }
            if (csv.meta.contains(key)) {
                fail(origin, line_no, "metadata key '" + key + "' given twice");
            }
            csv.meta[key] = std::string(detail::trim(body.substr(colon + 1)));
            continue;
        }

        const std::vector<std::string_view> cells = split(line, ',');
        if (csv.names.empty()) {
            for (const std::string_view cell : cells) {
                const std::size_t open = cell.find('[');
                if (open == std::string_view::npos || cell.back() != ']') {
                    fail(origin, line_no,
                         "header cell '" + std::string(cell) +
                             "' must be name[unit], e.g. alpha[deg]");
                }
                const std::string name(detail::trim(cell.substr(0, open)));
                const std::string_view unit = cell.substr(open + 1, cell.size() - open - 2);
                const auto scale = units::si_scale(unit);
                if (name.empty()) {
                    fail(origin, line_no, "empty column name in '" + std::string(cell) + "'");
                }
                if (!scale) {
                    fail(origin, line_no,
                         "unknown unit '" + std::string(unit) + "' in column '" + name +
                             "' (use lbf or lbm, not lb)");
                }
                if (std::find(csv.names.begin(), csv.names.end(), name) != csv.names.end()) {
                    fail(origin, line_no, "duplicate column '" + name + "'");
                }
                csv.names.push_back(name);
                scales.push_back(*scale);
            }
            csv.columns.resize(csv.names.size());
            continue;
        }

        if (cells.size() != csv.names.size()) {
            fail(origin, line_no,
                 "expected " + std::to_string(csv.names.size()) + " values, got " +
                     std::to_string(cells.size()));
        }
        for (std::size_t c = 0; c < cells.size(); ++c) {
            const auto value = detail::parse_finite_double(cells[c]);
            if (!value) {
                fail(origin, line_no,
                     "column '" + csv.names[c] + "': '" + std::string(cells[c]) +
                         "' is not a finite number");
            }
            csv.columns[c].push_back(*value * scales[c]);
        }
        ++rows;
    }

    const auto source = csv.meta.find("source");
    if (source == csv.meta.end() || source->second.empty()) {
        fail(origin, 0, "missing '# source: ...' provenance line");
    }
    if (csv.names.empty()) {
        fail(origin, 0, "missing header line");
    }
    if (rows == 0) {
        fail(origin, 0, "no data rows");
    }
    return csv;
}

CsvTable read_csv_table(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw InitError(path.string() + ": cannot open table file");
    }
    std::ostringstream text;
    text << in.rdbuf();
    return parse_csv_table(text.str(), path.string());
}

Table1D make_table1d(const CsvTable& csv, std::string_view x, std::string_view value) {
    Grid g = build_grid(csv, {x}, value);
    return Table1D(std::move(g.axes[0]), std::move(g.values));
}

Table2D make_table2d(const CsvTable& csv, std::string_view x, std::string_view y,
                     std::string_view value) {
    Grid g = build_grid(csv, {x, y}, value);
    return Table2D(std::move(g.axes[0]), std::move(g.axes[1]), std::move(g.values));
}

Table3D make_table3d(const CsvTable& csv, std::string_view x, std::string_view y,
                     std::string_view z, std::string_view value) {
    Grid g = build_grid(csv, {x, y, z}, value);
    return Table3D(std::move(g.axes[0]), std::move(g.axes[1]), std::move(g.axes[2]),
                   std::move(g.values));
}

}  // namespace eyas
