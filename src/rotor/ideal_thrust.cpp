#include "rotor/ideal_thrust.hpp"

#include <cmath>

#include "util/error.hpp"

namespace eyas {

IdealThrust::IdealThrust(double max_thrust) : max_thrust_(max_thrust) {
    if (!(max_thrust > 0.0) || !std::isfinite(max_thrust)) {
        throw InitError("ideal_thrust: max_thrust must be positive and finite");
    }
}

Wrench IdealThrust::compute(const SimContext& ctx) const {
    Wrench w;
    w.F = Vec3(0.0, 0.0, -max_thrust_ * ctx.controls.collective);
    return w;
}

}  // namespace eyas
