#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "util/table.hpp"

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

    const std::string& source() const { return meta.at("source"); }
    bool is_placeholder() const;

    // Throws InitError if there is no column with this name.
    const std::vector<double>& column(std::string_view name) const;
};

// Throw InitError, naming the origin and line, on any format problem.
CsvTable read_csv_table(const std::filesystem::path& path);
CsvTable parse_csv_table(std::string_view text, std::string_view origin);

Table1D make_table1d(const CsvTable& csv, std::string_view x, std::string_view value);
Table2D make_table2d(const CsvTable& csv, std::string_view x, std::string_view y,
                     std::string_view value);
Table3D make_table3d(const CsvTable& csv, std::string_view x, std::string_view y,
                     std::string_view z, std::string_view value);

}  // namespace eyas
