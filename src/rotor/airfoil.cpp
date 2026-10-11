#include "rotor/airfoil.hpp"

#include <numbers>
#include <utility>

#include "util/error.hpp"

namespace eyas {
namespace {

// 180° in a table converts to π within a few ulps (deg × π/180).
constexpr double kAlphaCoverageTolerance = 1e-9;  // [rad]

}  // namespace

Airfoil::Airfoil(Table2D cl, Table2D cd, std::string source, bool placeholder)
    : cl_(std::move(cl)),
      cd_(std::move(cd)),
      source_(std::move(source)),
      placeholder_(placeholder) {}

Airfoil Airfoil::from_csv(const CsvTable& csv) {
    csv.require_dimension("alpha", units::Dimension::angle);
    csv.require_dimension("mach", units::Dimension::dimensionless);
    csv.require_dimension("cl", units::Dimension::dimensionless);
    csv.require_dimension("cd", units::Dimension::dimensionless);
    Table2D cl = make_table2d(csv, "alpha", "mach", "cl");
    Table2D cd = make_table2d(csv, "alpha", "mach", "cd");
    const Axis& alpha = cl.x_axis();
    if (alpha.breakpoint(0) > -std::numbers::pi + kAlphaCoverageTolerance ||
        alpha.breakpoint(alpha.size() - 1) < std::numbers::pi - kAlphaCoverageTolerance) {
        throw InitError(csv.origin + ": alpha must cover [-180, 180] deg for reverse flow");
    }
    return Airfoil(std::move(cl), std::move(cd), csv.source(), csv.is_placeholder());
}

}  // namespace eyas
