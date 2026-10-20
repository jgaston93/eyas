// D-048: the gyroscopic moment of GenHel's spinning components (CR-166309
// p. 5.10-4, data p. 5.10-19):
// - layer 1: SpinningParts against GenHel's printed component equations;
// - the config reader sums the parts' J Ω per axis, and the executive adds the
//   moment to the body equations.

#include <random>
#include <span>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/rigid_body.hpp"
#include "core/spinning_parts.hpp"
#include "sim/build.hpp"
#include "util/config.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

// GenHel p. 5.10-4: L_GY = H_Y r − H_Z q, M_GY = −H_X r + H_Z p,
// N_GY = H_X q − H_Y p.
Vec3 genhel_gyroscopic(const Vec3& h, const Vec3& omega) {
    const double p = omega.x();
    const double q = omega.y();
    const double r = omega.z();
    return {h.y() * r - h.z() * q, -h.x() * r + h.z() * p, h.x() * q - h.y() * p};
}

TEST(SpinningParts, MatchesGenHel) {
    std::mt19937_64 rng(48);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int i = 0; i < 50; ++i) {
        const Vec3 h(900.0 * u(rng), 120.0 * u(rng), 700.0 * u(rng));
        SimContext ctx;
        ctx.body.vel_b = Vec3(60.0, 3.0, -2.0);
        ctx.body.omega_b = Vec3(0.8 * u(rng), 0.8 * u(rng), 0.8 * u(rng));
        const Wrench w = SpinningParts(h).compute(ctx);
        EXPECT_EQ(w.F, Vec3::Zero());
        EXPECT_NEAR((w.M - genhel_gyroscopic(h, ctx.body.omega_b)).norm(), 0.0, 1e-12 * h.norm());
    }
    EXPECT_THROW(SpinningParts(Vec3(0.0, std::numeric_limits<double>::infinity(), 0.0)), InitError);
}

std::string body_only(const std::string& force_models) {
    return R"(
airframe:
  mass: "15000 lbm"
  inertia: {ixx: "4659 slug*ft^2", iyy: "38512 slug*ft^2", izz: "36796 slug*ft^2", ixz: "1882 slug*ft^2"}
)" + force_models;
}

Vec3 omega_dot(const Executive& ex, const BodyState& b) {
    std::vector<double> x(ex.num_states(), 0.0);
    pack_body(b, x);
    std::vector<double> xdot(ex.num_states(), 0.0);
    EXPECT_TRUE(ex.derivatives(0.0, x, Controls{}, Environment{}, xdot));
    return {xdot[body_layout::kOmega], xdot[body_layout::kOmega + 1],
            xdot[body_layout::kOmega + 2]};
}

// The reader sums J Ω per body axis (GenHel's H = Σ J W), and the executive
// adds −ω × H to the body moments: the change in ω̇ is I⁻¹ times GenHel's
// moment.
TEST(SpinningParts, ReadFromConfigAndAddedToTheBody) {
    const std::string parts = R"(
force_models:
  - type: spinning_parts
    parts:
      - {axis: x, inertia: "0.116 slug*ft^2", speed: "4188.0 rad/s"}
      - {axis: x, inertia: "0.124 slug*ft^2", speed: "2094.0 rad/s"}
      - {axis: y, inertia: "0.715 slug*ft^2", speed: "127.1 rad/s"}
      - {axis: z, inertia: "18.0 slug*ft^2", speed: "-27.0 rad/s"}
)";
    const Executive with = build_executive(Config::from_string(body_only(parts)), 0.0);
    const Executive without = build_executive(Config::from_string(body_only("")), 0.0);
    BodyState b;
    b.vel_b = Vec3(60.0, 2.0, 1.0);
    b.omega_b = Vec3(0.3, -0.2, 0.25);
    const double si = units::kSlug * units::kFoot * units::kFoot;  // slug·ft² → kg·m²
    const Vec3 h = Vec3(0.116 * 4188.0 + 0.124 * 2094.0, 0.715 * 127.1, 18.0 * -27.0) * si;
    const Mat3 inertia =
        rigid_body_mass_matrix(make_mass_properties(15000.0 * units::kPoundMass, 4659.0 * si,
                                                    38512.0 * si, 36796.0 * si, 1882.0 * si))
            .bottomRightCorner<3, 3>();
    const Vec3 expected = inertia.inverse() * genhel_gyroscopic(h, b.omega_b);
    const Vec3 got = omega_dot(with, b) - omega_dot(without, b);
    EXPECT_NEAR((got - expected).norm(), 0.0, 1e-9 * expected.norm());

    // A part along another axis is an error, not ignored.
    EXPECT_THROW((void)build_executive(Config::from_string(body_only(R"(
force_models:
  - type: spinning_parts
    parts:
      - {axis: w, inertia: "1 slug*ft^2", speed: "1 rad/s"}
)")),
                                       0.0),
                 InitError);
}

}  // namespace
}  // namespace eyas
