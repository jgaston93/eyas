// M2: Euler angles for the Tier 1 output (D-018), checked against the rotation
// matrix built from Padfield's single-axis rotations (3rd ed. Eqs. 3A.34–3A.36).

#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "core/attitude.hpp"

namespace eyas {
namespace {

using std::numbers::pi;

// Γ = Φ Θ Ψ maps earth (NED) components to body components (Padfield 3rd ed.
// Eqs. 3A.34–3A.37), so Γ = C_bn and C_nb = Γᵀ. Each factor is written out as
// printed, independently of the code under test.
Mat3 padfield_gamma(double phi, double theta, double psi) {
    Mat3 Psi;
    Psi << std::cos(psi), std::sin(psi), 0.0,  //
        -std::sin(psi), std::cos(psi), 0.0,    //
        0.0, 0.0, 1.0;                         // Eq. 3A.34
    Mat3 Theta;
    Theta << std::cos(theta), 0.0, -std::sin(theta),  //
        0.0, 1.0, 0.0,                                //
        std::sin(theta), 0.0, std::cos(theta);        // Eq. 3A.35
    Mat3 Phi;
    Phi << 1.0, 0.0, 0.0,                    //
        0.0, std::cos(phi), std::sin(phi),   //
        0.0, -std::sin(phi), std::cos(phi);  // Eq. 3A.36
    return Phi * Theta * Psi;
}

Quat q_from_euler(double phi, double theta, double psi) {
    return Quat(Mat3(padfield_gamma(phi, theta, psi).transpose()));
}

// Heading is compared as an angle: at ψ_h = 0 round-off can give 2π − 4e-16,
// which is the same direction and inside [0, 2π).
double angle_difference(double a, double b) {
    return std::remainder(a - b, 2.0 * pi);
}

// Round trip over a grid that covers every quadrant of roll and heading and
// pitch up to ±85°.
TEST(Attitude, EulerAnglesRecoverPadfieldRotation) {
    const double r = pi / 180.0;
    for (int i = 0; i < 11; ++i) {
        const double phi = (-175.0 + 35.0 * i) * r;
        for (int j = 0; j < 11; ++j) {
            const double theta = (-85.0 + 17.0 * j) * r;
            for (int k = 0; k < 9; ++k) {
                const double psi = 40.0 * k * r;
                const EulerAngles e = euler_angles(q_from_euler(phi, theta, psi));
                SCOPED_TRACE(testing::Message() << phi << " " << theta << " " << psi);
                EXPECT_NEAR(e.roll, phi, 1e-12);
                EXPECT_NEAR(e.pitch, theta, 1e-12);
                EXPECT_NEAR(angle_difference(e.heading, psi), 0.0, 1e-12);
                EXPECT_GE(e.heading, 0.0);
                EXPECT_LT(e.heading, 2.0 * pi);
            }
        }
    }
}

// quaternion() is the inverse: it reproduces Padfield's Γ (test-side,
// Eqs. 3A.34–3A.36) on the same grid.
TEST(Attitude, QuaternionFromEulerIsPadfieldRotation) {
    const double r = pi / 180.0;
    for (int i = 0; i < 11; ++i) {
        for (int j = 0; j < 11; ++j) {
            for (int k = 0; k < 9; ++k) {
                const double phi = (-175.0 + 35.0 * i) * r;
                const double theta = (-85.0 + 17.0 * j) * r;
                const double psi = 40.0 * k * r;
                const Quat q = quaternion(EulerAngles{phi, theta, psi});
                EXPECT_NEAR(q.norm(), 1.0, 1e-15);
                const Mat3 expect = padfield_gamma(phi, theta, psi).transpose();
                EXPECT_LT((q.toRotationMatrix() - expect).norm(), 1e-14);
            }
        }
    }
}

// Independent physical anchors for the quaternion convention (q_nb maps body
// vectors to NED): heading 90° points the nose east; pitch +30° points it up
// (negative down); roll +90° points the right wing down.
TEST(Attitude, AnglesHaveExpectedPhysicalMeaning) {
    const double r = pi / 180.0;
    EXPECT_TRUE((q_from_euler(0.0, 0.0, 90.0 * r) * Vec3::UnitX()).isApprox(Vec3::UnitY(), 1e-15));
    EXPECT_TRUE((q_from_euler(0.0, 30.0 * r, 0.0) * Vec3::UnitX())
                    .isApprox(Vec3(std::cos(30.0 * r), 0.0, -0.5), 1e-15));
    EXPECT_TRUE((q_from_euler(90.0 * r, 0.0, 0.0) * Vec3::UnitY()).isApprox(Vec3::UnitZ(), 1e-15));
}

// Heading is reported in [0, 2π); roll and pitch keep their signs.
TEST(Attitude, HeadingRange) {
    const double r = pi / 180.0;
    const EulerAngles west = euler_angles(q_from_euler(0.0, 0.0, -90.0 * r));
    EXPECT_NEAR(west.heading, 270.0 * r, 1e-12);
    const EulerAngles north = euler_angles(Quat::Identity());
    EXPECT_EQ(north.heading, 0.0);
    EXPECT_GE(euler_angles(q_from_euler(0.0, 0.0, -1e-17)).heading, 0.0);
    EXPECT_LT(euler_angles(q_from_euler(0.0, 0.0, -1e-17)).heading, 2.0 * pi);
}

// Roll is reported in (−π, π]. An inverted attitude whose quaternion has
// signed zeros gives Γ23 = −0.0, and atan2(−0.0, −1) is −π, so it must fold.
TEST(Attitude, RollRangeExcludesMinusPi) {
    const Quat inverted(-0.0, 1.0, -0.0, 0.0);  // (w, x, y, z): 180° about x_b
    const EulerAngles e = euler_angles(inverted);
    EXPECT_EQ(e.roll, pi);
    EXPECT_EQ(euler_angles(Quat(0.0, 1.0, 0.0, 0.0)).roll, pi);
}

// At pitch ±90° roll and heading are not separable. The output must stay finite,
// with pitch exactly ±π/2 (clamped against round-off) and the same nose
// direction as the input.
TEST(Attitude, GimbalLockStaysFinite) {
    const double r = pi / 180.0;
    for (const double theta : {90.0, -90.0}) {
        const Quat q = q_from_euler(20.0 * r, theta * r, 50.0 * r);
        const EulerAngles e = euler_angles(q);
        EXPECT_TRUE(std::isfinite(e.roll));
        EXPECT_TRUE(std::isfinite(e.heading));
        EXPECT_NEAR(e.pitch, theta * r, 1e-7);
        EXPECT_LE(std::abs(e.pitch), pi / 2.0);
        const Quat back = q_from_euler(e.roll, e.pitch, e.heading);
        EXPECT_TRUE((back * Vec3::UnitX()).isApprox(q * Vec3::UnitX(), 1e-7));
    }
}

}  // namespace
}  // namespace eyas
