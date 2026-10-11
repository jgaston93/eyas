#include "util/units.hpp"

#include <array>
#include <utility>

namespace eyas::units {
namespace {

// Every unit accepted in config and table files. Add entries here as source
// data needs them; each must be an exact product of the definitions in
// units.hpp.
// clang-format off
constexpr std::array<std::pair<std::string_view, double>, 32> kRegistry{{
    {"1", 1.0},
    // length
    {"m", 1.0},
    {"ft", kFoot},
    {"in", kInch},
    // time
    {"s", 1.0},
    // frequency
    {"Hz", 1.0},
    // mass
    {"kg", 1.0},
    {"slug", kSlug},
    {"lbm", kPoundMass},
    // force
    {"N", 1.0},
    {"lbf", kPoundForce},
    // angle
    {"rad", 1.0},
    {"deg", kDegree},
    {"1/rad", 1.0},
    {"1/deg", 1.0 / kDegree},
    // speed and rate
    {"m/s", 1.0},
    {"ft/s", kFoot},
    {"kt", kKnot},
    {"rad/s", 1.0},
    {"deg/s", kDegree},
    {"rpm", kRpm},
    // acceleration
    {"m/s^2", 1.0},
    {"ft/s^2", kFoot},
    // density
    {"kg/m^3", 1.0},
    {"slug/ft^3", kSlug / (kFoot * kFoot * kFoot)},
    // moment and torque
    {"N*m", 1.0},
    {"ft*lbf", kFoot * kPoundForce},
    // first and second mass moments
    {"slug*ft", kSlug * kFoot},
    {"kg*m", 1.0},
    {"kg*m^2", 1.0},
    {"slug*ft^2", kSlug * kFoot * kFoot},
    {"lbf*in*s^2", kPoundForce * kInch},
}};
// clang-format on

}  // namespace

std::optional<double> si_scale(std::string_view unit) noexcept {
    for (const auto& [name, scale] : kRegistry) {
        if (name == unit) {
            return scale;
        }
    }
    return std::nullopt;
}

}  // namespace eyas::units
