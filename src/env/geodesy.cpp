#include "env/geodesy.hpp"

#include <cmath>
#include <numbers>

#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {

double meridian_radius(double latitude) noexcept {
    using namespace wgs84;
    const double s = std::sin(latitude);
    const double w = 1.0 - kEccentricitySquared * s * s;
    return kSemiMajorAxis * (1.0 - kEccentricitySquared) / (w * std::sqrt(w));
}

double prime_vertical_radius(double latitude) noexcept {
    using namespace wgs84;
    const double s = std::sin(latitude);
    return kSemiMajorAxis / std::sqrt(1.0 - kEccentricitySquared * s * s);
}

FlatEarthMap::FlatEarthMap(const GeodeticPosition& origin) : origin_(origin) {
    if (!std::isfinite(origin.latitude) || !std::isfinite(origin.longitude) ||
        !std::isfinite(origin.altitude)) {
        throw InitError("geodetic origin: latitude, longitude and altitude must be finite");
    }
    if (std::abs(origin.latitude) > 89.0 * units::kDegree) {
        throw InitError("geodetic origin: |latitude| must be at most 89 deg (D-017)");
    }
    if (std::abs(origin.longitude) > std::numbers::pi) {
        throw InitError("geodetic origin: |longitude| must be at most 180 deg");
    }
    const double h0 = origin.altitude;
    rad_per_m_north_ = 1.0 / (meridian_radius(origin.latitude) + h0);
    rad_per_m_east_ =
        1.0 / ((prime_vertical_radius(origin.latitude) + h0) * std::cos(origin.latitude));
}

GeodeticPosition FlatEarthMap::to_geodetic(const Vec3& pos_n) const noexcept {
    constexpr double two_pi = 2.0 * std::numbers::pi;
    GeodeticPosition g;
    g.latitude = origin_.latitude + pos_n.x() * rad_per_m_north_;
    // remainder() maps to [−π, π] and leaves values already there unchanged.
    double lon = std::remainder(origin_.longitude + pos_n.y() * rad_per_m_east_, two_pi);
    if (lon <= -std::numbers::pi) {
        lon += two_pi;
    }
    g.longitude = lon;
    g.altitude = origin_.altitude - pos_n.z();
    return g;
}

}  // namespace eyas
