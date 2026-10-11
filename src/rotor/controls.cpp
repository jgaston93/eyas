#include "rotor/controls.hpp"

#include <cmath>
#include <string_view>

#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

bool finite(const LinearPitchRow& r) {
    return std::isfinite(r.offset) && std::isfinite(r.collective) && std::isfinite(r.lon_cyclic) &&
           std::isfinite(r.lat_cyclic) && std::isfinite(r.pedal);
}

LinearPitchRow read_row(const Config& node, std::string_view key) {
    LinearPitchRow row;
    if (!node.contains(key)) {
        return row;
    }
    const Config r = node.child(key);
    r.allow_keys({"offset", "collective", "lon_cyclic", "lat_cyclic", "pedal"});
    const auto angle = [&](std::string_view k) {
        return r.contains(k) ? r.quantity(k, units::Dimension::angle) : 0.0;
    };
    row.offset = angle("offset");
    row.collective = angle("collective");
    row.lon_cyclic = angle("lon_cyclic");
    row.lat_cyclic = angle("lat_cyclic");
    row.pedal = angle("pedal");
    return row;
}

}  // namespace

LinearControlMap::LinearControlMap(LinearPitchRow theta0, LinearPitchRow theta1s,
                                   LinearPitchRow theta1c)
    : theta0_(theta0), theta1s_(theta1s), theta1c_(theta1c) {
    if (!finite(theta0_) || !finite(theta1s_) || !finite(theta1c_)) {
        throw InitError("control map: offsets and gains must be finite");
    }
}

std::unique_ptr<LinearControlMap> LinearControlMap::from_config(const Config& node) {
    node.allow_keys({"theta0", "theta1s", "theta1c"});
    return std::make_unique<LinearControlMap>(read_row(node, "theta0"), read_row(node, "theta1s"),
                                              read_row(node, "theta1c"));
}

}  // namespace eyas
