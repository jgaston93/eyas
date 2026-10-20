#include "core/spinning_parts.hpp"

#include <utility>

#include "util/error.hpp"

namespace eyas {

SpinningParts::SpinningParts(Vec3 angular_momentum_b) : h_b_(std::move(angular_momentum_b)) {
    if (!h_b_.allFinite()) {
        throw InitError("spinning_parts: the angular momentum must be finite");
    }
}

Wrench SpinningParts::compute(const SimContext& ctx) const {
    Wrench w;
    w.M = -ctx.body.omega_b.cross(h_b_);  // GenHel p. 5.10-4
    return w;
}

}  // namespace eyas
