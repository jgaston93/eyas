#include "util/table.hpp"

#include <cmath>
#include <string>

#include "util/error.hpp"

namespace eyas {
namespace {

void check_values(const std::vector<double>& values, std::size_t expected, const char* kind) {
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

}  // namespace

Table1D::Table1D(Axis x, std::vector<double> values) : x_(std::move(x)), v_(std::move(values)) {
    check_values(v_, x_.size(), "Table1D");
}

Table2D::Table2D(Axis x, Axis y, std::vector<double> values)
    : x_(std::move(x)), y_(std::move(y)), v_(std::move(values)) {
    check_values(v_, x_.size() * y_.size(), "Table2D");
}

Table3D::Table3D(Axis x, Axis y, Axis z, std::vector<double> values)
    : x_(std::move(x)), y_(std::move(y)), z_(std::move(z)), v_(std::move(values)) {
    check_values(v_, x_.size() * y_.size() * z_.size(), "Table3D");
}

}  // namespace eyas
