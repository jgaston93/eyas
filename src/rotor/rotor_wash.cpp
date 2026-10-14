#include "rotor/rotor_wash.hpp"

#include <array>
#include <cmath>
#include <utility>

#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"

namespace eyas {

RotorWash::RotorWash(std::string rotor, MultiTable2D<2> map)
    : rotor_(std::move(rotor)), map_(std::move(map)) {
    if (rotor_.empty()) {
        throw InitError("rotor wash: the rotor name must not be empty");
    }
}

RotorWash RotorWash::from_config(const Config& node) {
    node.allow_keys({"rotor", "table"});
    std::string rotor = node.string("rotor");
    MultiTable2D<2> map = node.in_context("table", [&] {
        const CsvTable csv = node.csv_table("table");
        csv.require_dimension("chi", units::Dimension::angle);
        csv.require_dimension("a1", units::Dimension::angle);
        csv.require_dimension("ekx", units::Dimension::dimensionless);
        csv.require_dimension("ekz", units::Dimension::dimensionless);
        return make_multi_table2d<2>(csv, "chi", "a1", {"ekx", "ekz"});
    });
    return node.in_context("rotor", [&] { return RotorWash(std::move(rotor), std::move(map)); });
}

std::vector<std::string> RotorWash::consumes() const {
    return {rotor_ + ".lambda0", rotor_ + ".mu_x", rotor_ + ".mu_z", rotor_ + ".beta_1c",
            rotor_ + ".tip_speed"};
}

void RotorWash::bind(const CouplingRegistry& registry, std::string_view consumer) {
    lambda0_ = registry.find(rotor_ + ".lambda0", consumer);
    mu_x_ = registry.find(rotor_ + ".mu_x", consumer);
    mu_z_ = registry.find(rotor_ + ".mu_z", consumer);
    beta_1c_ = registry.find(rotor_ + ".beta_1c", consumer);
    tip_speed_ = registry.find(rotor_ + ".tip_speed", consumer);
}

RotorWash::Result RotorWash::evaluate(const CouplingOutputs& coupling) const noexcept {
    return evaluate(coupling.get(lambda0_), coupling.get(mu_x_), coupling.get(mu_z_),
                    coupling.get(beta_1c_), coupling.get(tip_speed_));
}

RotorWash::Result RotorWash::evaluate(double lambda0, double mu_x, double mu_z, double beta_1c,
                                      double tip_speed) const noexcept {
    Result r;
    // GenHel CR-166309 p. 5.1-37 (wake skew) and p. 5.1-18 (a₁). atan2 gives
    // GenHel's atan(μ_x/|λ|) and stays defined at λ = 0.
    r.a1 = -beta_1c;
    r.chi = std::atan2(mu_x, std::abs(lambda0 - mu_z)) + r.a1;
    const std::array<double, 2> e = map_(r.chi, r.a1, cursor_);
    // pp. 5.3-5, 5.4-4, 5.4-5.
    const double wash = lambda0 * tip_speed;
    r.velocity_b = Vec3(wash * e[0], 0.0, -wash * e[1]);
    return r;
}

}  // namespace eyas
