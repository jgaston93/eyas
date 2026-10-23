#include "rotor/shaft_link.hpp"

#include <cmath>
#include <cstddef>

#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {

ShaftLink read_shaft_link(const Config& node) {
    node.allow_keys({"name", "gear"});
    ShaftLink link;
    link.name = node.string("name");
    link.gear = node.contains("gear") ? node.number("gear") : 1.0;
    if (link.name.empty()) {
        node.fail("name", "must not be empty");
    }
    if (!(link.gear > 0.0) || !std::isfinite(link.gear)) {
        node.fail("gear", "must be positive and finite");
    }
    return link;
}

int bind_shaft_link(const ShaftLink& link, std::span<const ShaftInfo> shafts, double rotor_speed,
                    const std::string& who) {
    for (std::size_t k = 0; k < shafts.size(); ++k) {
        if (shafts[k].name == link.name) {
            const double want = link.gear * shafts[k].speed;
            if (std::abs(rotor_speed - want) > 1e-9 * want) {
                throw InitError(who + ": rotor_speed must be gear × shaft '" + shafts[k].name +
                                "' speed (D-052)");
            }
            return static_cast<int>(k);
        }
    }
    throw InitError(who + ": no shaft named '" + link.name + "' (D-052)");
}

}  // namespace eyas
