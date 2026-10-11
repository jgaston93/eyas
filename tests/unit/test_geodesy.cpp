// M2 acceptance tests for the Tier 1 NED-to-geodetic map (D-017): WGS-84
// radii of curvature against NGA.STND.0036 tabulated values and against an
// independent construction of the meridian ellipse, and the flat-earth map at
// several origins.

#include <array>
#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "env/geodesy.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using std::numbers::pi;
constexpr double kDeg = units::kDegree;

// NGA.STND.0036 Table 3.5 (p. 3-9), human-verified 2026-10-10.
constexpr double kSemiMinorAxis = 6356752.3142;  // b [m], printed to 0.1 mm
constexpr double kPolarRadius = 6399593.6258;    // R_p [m], printed to 0.1 mm

// ---- Independent ellipse geometry ---------------------------------------------
// The meridian is the ellipse x = a cos β, z = b sin β (β: reduced latitude).
// The geodetic latitude φ of a point is the angle of the ellipse normal there:
// tan φ = (a/b) tan β. Nothing here uses the radius formulas under test.
constexpr double a_ = wgs84::kSemiMajorAxis;
const double b_ = a_ * (1.0 - wgs84::kFlattening);

double reduced_latitude(double phi) {
    return std::atan2(b_ * std::sin(phi), a_ * std::cos(phi));
}

// Point on the meridian ellipse at geodetic latitude φ: (x, z) [m].
void meridian_point(double phi, double& x, double& z) {
    const double beta = reduced_latitude(phi);
    x = a_ * std::cos(beta);
    z = b_ * std::sin(beta);
}

// Meridian arc length from φ1 to φ2 [m], by summing short chords. With n chords
// over Δφ the chord-vs-arc error is ~ (Δφ/n)² / 24 relative.
double meridian_arc(double phi1, double phi2, int n = 20000) {
    double s = 0.0;
    double x0 = 0.0;
    double z0 = 0.0;
    meridian_point(phi1, x0, z0);
    for (int i = 1; i <= n; ++i) {
        double x1 = 0.0;
        double z1 = 0.0;
        meridian_point(phi1 + (phi2 - phi1) * i / n, x1, z1);
        s += std::hypot(x1 - x0, z1 - z0);
        x0 = x1;
        z0 = z1;
    }
    return phi2 >= phi1 ? s : -s;
}

// ---- Radii of curvature ---------------------------------------------------------

// Anchors from the NGA tables: at the equator R_N = a and R_M = b²/a; at the
// poles R_M = R_N = R_p. Tolerances: b and R_p are printed to 0.1 mm, so
// |Δ(b²/a)| ≤ 2b·0.05e-3/a ≈ 1e-4 m, and R_p ± 5e-5 m.
TEST(Geodesy, RadiiMatchNgaTabulatedValues) {
    EXPECT_EQ(wgs84::kSemiMajorAxis, 6378137.0);
    EXPECT_NEAR(1.0 / wgs84::kFlattening, 298.257223563, 1e-9);
    EXPECT_NEAR(b_, kSemiMinorAxis, 5e-5);

    EXPECT_NEAR(prime_vertical_radius(0.0), wgs84::kSemiMajorAxis, 1e-6);
    EXPECT_NEAR(meridian_radius(0.0), kSemiMinorAxis * kSemiMinorAxis / wgs84::kSemiMajorAxis,
                1.1e-4);
    for (const double pole : {pi / 2.0, -pi / 2.0}) {
        EXPECT_NEAR(meridian_radius(pole), kPolarRadius, 6e-5);
        EXPECT_NEAR(prime_vertical_radius(pole), kPolarRadius, 6e-5);
    }
}

// R_M is the meridian's radius of curvature, ds/dφ. Compare with the arc of the
// independently constructed ellipse over ±1e-4 rad (chord error ~1e-10).
TEST(Geodesy, MeridianRadiusIsCurvatureOfMeridianEllipse) {
    constexpr double d = 1e-4;
    for (const double lat_deg : {-75.0, -33.9, 0.0, 12.5, 45.0, 60.0, 80.0}) {
        SCOPED_TRACE(lat_deg);
        const double phi = lat_deg * kDeg;
        const double R_ref = meridian_arc(phi - d, phi + d, 200) / (2.0 * d);
        EXPECT_NEAR(meridian_radius(phi), R_ref, 1e-8 * R_ref);
    }
}

// R_N is the length of the normal from the surface to the polar axis:
// R_N = x / cos φ for the ellipse point at geodetic latitude φ. Round-off in x
// (~1e-9 m) is magnified by 1/cos φ (≈ 6 at 80°), hence 1e-7 m.
TEST(Geodesy, PrimeVerticalRadiusIsNormalLengthToAxis) {
    for (const double lat_deg : {-75.0, -33.9, 0.0, 12.5, 45.0, 60.0, 80.0}) {
        SCOPED_TRACE(lat_deg);
        const double phi = lat_deg * kDeg;
        double x = 0.0;
        double z = 0.0;
        meridian_point(phi, x, z);
        EXPECT_NEAR(prime_vertical_radius(phi), x / std::cos(phi), 1e-7);
    }
}

// ---- Flat-earth map (D-017) -----------------------------------------------------

struct Origin {
    double lat_deg;
    double lon_deg;
    double alt;  // [m] MSL
};

constexpr std::array<Origin, 5> kOrigins = {{
    {0.0, 0.0, 0.0},         // equator, prime meridian
    {37.41, -122.05, 10.0},  // mid northern latitude, western hemisphere
    {-33.95, 151.18, 6.0},   // southern hemisphere, eastern
    {64.8, -147.9, 1500.0},  // high latitude, elevated origin
    {51.0, 179.99, 0.0},     // next to the antimeridian
}};

GeodeticPosition origin_of(const Origin& o) {
    return {o.lat_deg * kDeg, o.lon_deg * kDeg, o.alt};
}

TEST(Geodesy, OriginMapsToItself) {
    for (const Origin& o : kOrigins) {
        const FlatEarthMap map(origin_of(o));
        const GeodeticPosition p = map.to_geodetic(Vec3::Zero());
        EXPECT_EQ(p.latitude, o.lat_deg * kDeg);
        EXPECT_DOUBLE_EQ(p.longitude, o.lon_deg * kDeg);
        EXPECT_EQ(p.altitude, o.alt);
    }
}

// Altitude is h0 − D exactly, independent of the horizontal position (level
// flight in NED stays level geodetically; D-017).
TEST(Geodesy, AltitudeIsOriginHeightMinusDown) {
    for (const Origin& o : kOrigins) {
        const FlatEarthMap map(origin_of(o));
        for (const Vec3& p_n :
             {Vec3(0.0, 0.0, -300.0), Vec3(25e3, -40e3, -300.0), Vec3(-8e3, 3e3, 120.0)}) {
            EXPECT_EQ(map.to_geodetic(p_n).altitude, o.alt - p_n.z());
        }
    }
}

// Reference calculation at several origins. Northward: the map keeps arc length
// along the meridian with the origin's curvature, so the true meridian arc
// (independent ellipse, scaled to height h0) from lat0 to the mapped latitude
// must equal N up to the change of R_M over the arc:
//   |arc − N| ≲ |dR_M/dφ| Δφ² / 2,  with |dR_M/dφ| ≤ 3 e² a (|sin 2φ|/2) / (1−e²)^{3/2}.
// Eastward: the map keeps arc length along the origin's parallel, whose radius
// is the distance to the polar axis, x(φ0) + h0 cos φ0 (independent ellipse).
TEST(Geodesy, MatchesEllipsoidArcLengthsAtSeveralOrigins) {
    const double e2 = wgs84::kEccentricitySquared;
    for (const Origin& o : kOrigins) {
        SCOPED_TRACE(o.lat_deg);
        const GeodeticPosition g0 = origin_of(o);
        const FlatEarthMap map(g0);
        for (const double N : {-20e3, -1e3, 500.0, 15e3}) {
            const GeodeticPosition p = map.to_geodetic(Vec3(N, 0.0, 0.0));
            EXPECT_DOUBLE_EQ(p.longitude, g0.longitude);
            // Arc at height h0: scale each chord's curvature radius R by (R + h)/R.
            // Over a short arc this is arc_surface + h·Δφ.
            const double dphi = p.latitude - g0.latitude;
            const double arc = meridian_arc(g0.latitude, p.latitude) + g0.altitude * dphi;
            const double dRdphi = 3.0 * e2 * wgs84::kSemiMajorAxis *
                                  std::abs(std::sin(2.0 * g0.latitude)) / 2.0 /
                                  std::pow(1.0 - e2, 1.5);
            const double bound = 0.5 * dRdphi * dphi * dphi + 1e-6 * std::abs(N) + 1e-6;
            EXPECT_NEAR(arc, N, bound);
        }
        double x = 0.0;
        double z = 0.0;
        meridian_point(g0.latitude, x, z);
        const double parallel_radius = x + g0.altitude * std::cos(g0.latitude);
        for (const double E : {-30e3, 2e3, 12e3}) {
            const GeodeticPosition p = map.to_geodetic(Vec3(0.0, E, 0.0));
            EXPECT_EQ(p.latitude, g0.latitude);
            double dlon = p.longitude - g0.longitude;
            dlon = std::remainder(dlon, 2.0 * pi);  // across the antimeridian
            EXPECT_NEAR(dlon * parallel_radius, E, 1e-6 * std::abs(E));
        }
    }
}

// Longitude is wrapped to (−π, π].
TEST(Geodesy, LongitudeWrapsAtAntimeridian) {
    const FlatEarthMap map({51.0 * kDeg, 179.99 * kDeg, 0.0});
    const double lon = map.to_geodetic(Vec3(0.0, 5e3, 0.0)).longitude;
    EXPECT_LT(lon, -179.9 * kDeg);
    EXPECT_GT(lon, -pi);
    const FlatEarthMap west({51.0 * kDeg, -179.99 * kDeg, 0.0});
    EXPECT_GT(west.to_geodetic(Vec3(0.0, -5e3, 0.0)).longitude, 179.9 * kDeg);
}

TEST(Geodesy, RejectsInvalidOrigins) {
    EXPECT_THROW(FlatEarthMap({89.5 * kDeg, 0.0, 0.0}), InitError);
    EXPECT_THROW(FlatEarthMap({-90.0 * kDeg, 0.0, 0.0}), InitError);
    EXPECT_THROW(FlatEarthMap({0.0, 181.0 * kDeg, 0.0}), InitError);
    EXPECT_THROW(FlatEarthMap({std::nan(""), 0.0, 0.0}), InitError);
    EXPECT_NO_THROW(FlatEarthMap({89.0 * kDeg, 180.0 * kDeg, -400.0}));
}

}  // namespace
}  // namespace eyas
