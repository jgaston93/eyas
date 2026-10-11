#include "rotor/airfoil.hpp"

#include <numbers>
#include <utility>

#include "util/error.hpp"

namespace eyas {
namespace {

// 180° in a table converts to π within a few ulps (deg × π/180).
constexpr double kAlphaCoverageTolerance = 1e-9;  // [rad]

}  // namespace

Airfoil::Airfoil(MultiTable2D<2> coefficients, std::string source, bool placeholder)
    : coefficients_(std::move(coefficients)),
      source_(std::move(source)),
      placeholder_(placeholder) {}

Airfoil Airfoil::from_csv(const CsvTable& csv) {
    csv.require_dimension("alpha", units::Dimension::angle);
    csv.require_dimension("mach", units::Dimension::dimensionless);
    csv.require_dimension("cl", units::Dimension::dimensionless);
    csv.require_dimension("cd", units::Dimension::dimensionless);
    MultiTable2D<2> coefficients = make_multi_table2d<2>(csv, "alpha", "mach", {"cl", "cd"});
    const Axis& alpha = coefficients.x_axis();
    if (alpha.breakpoint(0) > -std::numbers::pi + kAlphaCoverageTolerance ||
        alpha.breakpoint(alpha.size() - 1) < std::numbers::pi - kAlphaCoverageTolerance) {
        throw InitError(csv.origin + ": alpha must cover [-180, 180] deg for reverse flow");
    }
    return Airfoil(std::move(coefficients), csv.source(), csv.is_placeholder());
}

}  // namespace eyas
