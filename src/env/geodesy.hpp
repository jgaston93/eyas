#pragma once

#include "core/types.hpp"

// NED-to-geodetic map for the Tier 1 pose output (D-017). Flat-earth dynamics
// are mapped onto the WGS-84 ellipsoid by arc length at the origin: level
// flight in NED stays at constant geodetic altitude.
namespace eyas {

namespace wgs84 {
// NGA.STND.0036_1.0.0_WGS84, Eq. 3-2 (p. 3-2): the defining a and 1/f.
inline constexpr double kSemiMajorAxis = 6378137.0;          // a [m]
inline constexpr double kInverseFlattening = 298.257223563;  // 1/f [-]
inline constexpr double kFlattening = 1.0 / kInverseFlattening;
inline constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);  // e² [-]
}  // namespace wgs84

struct GeodeticPosition {
    double latitude = 0.0;   // [rad] WGS-84 geodetic
    double longitude = 0.0;  // [rad] (−π, π]
    double altitude = 0.0;   // [m] above MSL (geoid separation not modelled; D-017)
};

// Radii of curvature of the WGS-84 ellipsoid at geodetic latitude φ [rad], [m]:
//   meridian (north–south)   R_M = a (1 − e²) / (1 − e² sin²φ)^{3/2}
//   prime vertical (east–west) R_N = a / √(1 − e² sin²φ)
// NGA.STND.0036 doesn't print these; they are verified against its Table 3.5
// (equator and poles) and an independent ellipse construction (D-017, tests).
double meridian_radius(double latitude) noexcept;
double prime_vertical_radius(double latitude) noexcept;

class FlatEarthMap {
public:
    // Throws InitError unless the origin is finite, |latitude| ≤ 89° and
    // |longitude| ≤ 180°.
    explicit FlatEarthMap(const GeodeticPosition& origin);

    // Geodetic position of the NED point pos_n [m] (D-017):
    //   lat = lat0 + N / (R_M + h0),  lon = lon0 + E / ((R_N + h0) cos lat0),
    //   alt = h0 − D,  with R_M, R_N at lat0.
    // Meant for the flat-earth operating area (tens of km); not valid across a pole.
    GeodeticPosition to_geodetic(const Vec3& pos_n) const noexcept;

    const GeodeticPosition& origin() const noexcept { return origin_; }

private:
    GeodeticPosition origin_;
    double rad_per_m_north_;  // [rad/m]
    double rad_per_m_east_;   // [rad/m]
};

}  // namespace eyas
