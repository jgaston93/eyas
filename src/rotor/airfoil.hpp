#pragma once

#include <string>

#include "util/csv_table.hpp"
#include "util/table.hpp"

// Two-dimensional airfoil section coefficients C_l(α, M) and C_d(α, M) from
// tables (spec §6.4 step 5). Built at init time; lookups never allocate,
// throw or do I/O (spec §7).
namespace eyas {

struct AirfoilCoefficients {
    double cl = 0.0;  // [-] section lift coefficient
    double cd = 0.0;  // [-] section drag coefficient
};

class Airfoil {
public:
    // From a long-form CSV table (D-012) with columns alpha [angle],
    // mach [1], cl [1] and cd [1]. The alpha breakpoints must cover
    // [−180°, 180°], so reverse flow is looked up rather than clamped
    // (spec §6.4). Throws InitError naming the file on any problem.
    static Airfoil from_csv(const CsvTable& csv);

    // α [rad] in [−π, π]; M [-]. Queries outside the Mach range clamp to its
    // edges (D-012). `cursor` is a per-caller hunt-search hint that never
    // changes the result.
    AirfoilCoefficients operator()(double alpha, double mach,
                                   TableCursor<2>& cursor) const noexcept {
        const AxisPosition pa = cl_.x_axis().locate(alpha, cursor.hint[0]);
        const AxisPosition pm = cl_.y_axis().locate(mach, cursor.hint[1]);
        return {cl_.eval(pa, pm), cd_.eval(pa, pm)};
    }

    const std::string& source() const noexcept { return source_; }
    bool is_placeholder() const noexcept { return placeholder_; }

private:
    Airfoil(Table2D cl, Table2D cd, std::string source, bool placeholder);

    Table2D cl_;  // axes (α [rad], M [-]); cd_ has identical axes
    Table2D cd_;
    std::string source_;
    bool placeholder_;
};

}  // namespace eyas
