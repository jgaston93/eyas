// Yawed-flow drag (D-039; Johnson 2013 §6.22–6.23): the section drag acts
// along the full local flow, spanwise velocity u_R included.
// - Layer 1: the rotor's profile H-force and torque in edgewise flight, with
//   a constant drag coefficient and no lift, equal Johnson's integrals
//   (Eqs. 6.418–6.419, reverse flow included), evaluated here by independent
//   quadrature, to the element mesh's error.
// - Layer 2: the profile power C_Q + μ C_H against Johnson's approximation
//   Eq. 6.423, accurate to 1 % up to μ = 1, and against the familiar
//   1 + 4.65 μ² (Eq. 6.424) at μ ≤ 0.35.
// - With no spanwise flow (hover, unflapped) the loads are unchanged, bit for
//   bit, and the switch off reproduces the drag from U alone.

#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "rotor/blade_element_rotor.hpp"
#include "support/m3_fixtures.hpp"

namespace eyas {
namespace {

using test::airfoil_from;
using test::linear_airfoil_csv;
using test::sea_level_context;
using test::test_rotor;

constexpr double kCd0 = 0.01;

// No lift, constant drag, no root cutout or twist: Johnson's profile-drag
// rotor (§6.23). Uniform elements, `n` of them.
BladeElementRotorConfig profile_rotor(int n, bool yawed) {
    BladeElementRotorConfig c =
        test_rotor(airfoil_from(linear_airfoil_csv(0.0, kCd0)), SwashplatePitch{}, n);
    c.root_cutout = 0.0;
    c.twist = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
    c.yawed_flow_drag = yawed;
    return c;
}

struct Profile {
    double fh = 0.0;  // C_H / (σ c_d0 / 8), rearward positive
    double fq = 0.0;  // C_Q / (σ c_d0 / 8)
};

// Rotor loads at advance ratio μ (edgewise, no inflow), averaged over a
// blade passage, scaled as Johnson's F_H and F_Q.
Profile eyas_profile(const BladeElementRotor& rotor, double mu) {
    const BladeElementRotorConfig& c = rotor.config();
    const double sigma = c.num_blades * 0.53 / (std::numbers::pi * c.radius);
    const double scale = sigma * kCd0 / 8.0;
    const int M = 64;
    const double passage = 2.0 * std::numbers::pi / (c.num_blades * c.rotor_speed);
    Profile p;
    for (int j = 0; j < M; ++j) {
        SimContext ctx = sea_level_context(passage * (j + 0.5) / M);
        ctx.body.vel_b = Vec3(mu * rotor.tip_speed(), 0.0, 0.0);
        const RotorLoads l = rotor.evaluate_at_inflow(ctx, 0.0);
        const double q =
            ctx.air.density * rotor.disk_area() * rotor.tip_speed() * rotor.tip_speed();
        p.fh += -l.F_h.x() / q / scale / M;
        p.fq += l.cq / scale / M;
    }
    return p;
}

// Johnson's integrals for edgewise flight with u_P = 0 (Eqs. 6.418–6.419):
// F_H = (1/2π)∮∫ 4 W (r sin ψ + μ) dr dψ, F_Q = (1/2π)∮∫ 4 W u_T r dr dψ,
// u_T = r + μ sin ψ, u_R = μ cos ψ, W = √(u_T² + u_R²); the drag force has
// the sign of u_T, so reverse flow is included. Midpoint rule, fine grid.
Profile johnson_profile(double mu) {
    const int nr = 4000;
    const int npsi = 1440;
    Profile p;
    for (int i = 0; i < npsi; ++i) {
        const double psi = 2.0 * std::numbers::pi * (i + 0.5) / npsi;
        for (int k = 0; k < nr; ++k) {
            const double r = (k + 0.5) / nr;
            const double ut = r + mu * std::sin(psi);
            const double ur = mu * std::cos(psi);
            const double W = std::sqrt(ut * ut + ur * ur);
            p.fh += 4.0 * W * (r * std::sin(psi) + mu) / nr / npsi;
            p.fq += 4.0 * W * ut * r / nr / npsi;
        }
    }
    return p;
}

TEST(YawedFlowDrag, ProfileLoadsAreJohnsonsIntegrals) {
    const BladeElementRotor rotor(profile_rotor(400, true));
    for (const double mu : {0.1, 0.25, 0.4}) {
        const Profile e = eyas_profile(rotor, mu);
        const Profile j = johnson_profile(mu);
        EXPECT_NEAR(e.fh, j.fh, 1e-3 * j.fh) << "mu " << mu;
        EXPECT_NEAR(e.fq, j.fq, 1e-3 * j.fq) << "mu " << mu;
    }
}

// F_P = F_Q + μ F_H against Johnson's Eq. 6.423 (1 % to μ = 1) and Eq. 6.424
// (1 % to μ = 0.35). Without yawed-flow drag the radial-drag share (μ² in
// F_H, Johnson p. 233) and the W/U growth are missing, so the power is lower.
TEST(YawedFlowDrag, ProfilePowerMatchesJohnsonsApproximations) {
    const BladeElementRotor on(profile_rotor(400, true));
    const BladeElementRotor off(profile_rotor(400, false));
    for (const double mu : {0.1, 0.2, 0.3, 0.35, 0.5, 0.8}) {
        const Profile e = eyas_profile(on, mu);
        const double fp = e.fq + mu * e.fh;
        const double eq_6_423 = 1.0 + 4.5 * mu * mu + 1.61 * std::pow(mu, 3.7);
        EXPECT_NEAR(fp, eq_6_423, 0.011 * eq_6_423) << "mu " << mu;
        if (mu <= 0.35) {
            EXPECT_NEAR(fp, 1.0 + 4.65 * mu * mu, 0.011 * (1.0 + 4.65 * mu * mu)) << "mu " << mu;
        }
        const Profile f = eyas_profile(off, mu);
        EXPECT_LT(f.fq + mu * f.fh, fp) << "mu " << mu;
    }
}

// No spanwise flow in hover with the blades unflapped: identical loads.
TEST(YawedFlowDrag, NoSpanwiseFlowNoChange) {
    const auto syn = airfoil_from(test::synthetic_airfoil_csv());
    const double d = std::numbers::pi / 180.0;
    BladeElementRotorConfig c = test_rotor(syn, SwashplatePitch{12.0 * d, 0.0, 0.0}, 24);
    const BladeElementRotor off(c);
    c.yawed_flow_drag = true;
    const BladeElementRotor on(c);
    const SimContext ctx = sea_level_context(0.011);
    const RotorLoads a = off.evaluate(ctx);
    const RotorLoads b = on.evaluate(ctx);
    EXPECT_EQ(a.F_h, b.F_h);
    EXPECT_EQ(a.M_h, b.M_h);
    EXPECT_EQ(a.power, b.power);
}

}  // namespace
}  // namespace eyas
