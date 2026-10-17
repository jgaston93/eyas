#pragma once

#include <cmath>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>

#include "eyas/types.hpp"

namespace examples {

// Parses "<latitude deg>,<longitude deg>,<altitude m MSL>" (e.g. the
// --origin option) into a GeodeticOrigin [rad, rad, m]. Init time: throws
// std::runtime_error. Eyas checks the range when it builds the model.
inline eyas::GeodeticOrigin parse_origin(const std::string& text) {
    std::istringstream s(text);
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    char c1 = 0;
    char c2 = 0;
    if (!(s >> lat >> c1 >> lon >> c2 >> alt) || c1 != ',' || c2 != ',' || !s.eof()) {
        throw std::runtime_error("--origin: expected <lat deg>,<lon deg>,<alt m>, got " + text);
    }
    constexpr double kDeg = std::numbers::pi / 180.0;
    return {lat * kDeg, lon * kDeg, alt};
}

}  // namespace examples
