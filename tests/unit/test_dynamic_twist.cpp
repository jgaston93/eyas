// GenHel's elastic blade twist with a lagged tip twist (D-038; CR-166309
// pp. 5.1-24, 5.1-25, 5.1-49):
// - the twist is GenHel's: at tip twist θ_e the loads equal those of the
//   rigid rotor with θ_e m(r/R) added to its twist table (exact);
// - the state equation is dθ_e/dt = (K(V) F − θ_e)/τ, F the mean over blades
//   of |(F_t, F_n)|, checked on a one-blade rotor against forces taken from
//   the rotor's total loads, independently of the per-blade sums;
// - on the UH-60A, the trim is GenHel's fixed point (the revolution mean of
//   θ_e equals that of K F), the tip twist is GenHel's size, and the trim
//   doesn't depend on the lag τ, a numerical choice.

#include <cmath>
#include <fstream>
#include <numbers>
#include <regex>
#include <span>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "sim/build.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/config.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using test::airfoil_from;
using test::sea_level_context;
using test::synthetic_airfoil_csv;
using test::test_rotor;

constexpr double kDeg = std::numbers::pi / 180.0;

SwashplatePitch pitch(double t0, double t1c, double t1s) {
    return SwashplatePitch{t0 * kDeg, t1c * kDeg, t1s * kDeg};
}

DynamicTwist twist(Table1D gain, Table1D mode) {
    DynamicTwist d;
    d.gain = std::move(gain);
    d.mode = std::move(mode);
    d.lag = 0.01;
    return d;
}

// At tip twist θ_e the loads are those of the rigid rotor whose twist table
// has θ_e m(r/R) added. With a linear mode and a linear preset twist the
// sum is linear too, so the comparison is exact to rounding.
TEST(DynamicTwist, IsTheTwistTableIncrement) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    const double theta_e = -1.7 * kDeg;
    const double m0 = 0.28;
    BladeElementRotorConfig a = test_rotor(syn, pitch(14.0, -5.0, 2.0), 24);
    a.dynamic_twist =
        twist(Table1D(Axis({0.0, 100.0}), {-1e-6, -2e-6}), Table1D(Axis({0.0, 1.0}), {m0, 1.0}));
    BladeElementRotorConfig b = test_rotor(syn, pitch(14.0, -5.0, 2.0), 24);
    b.twist = Table1D(Axis({0.0, 1.0}), {m0 * theta_e, -16.0 * kDeg + theta_e});
    const BladeElementRotor ra(a);
    const BladeElementRotor rb(b);
    ASSERT_EQ(ra.num_states(), 1u);
    ASSERT_EQ(rb.num_states(), 0u);
    SimContext ctx = sea_level_context(0.019);
    ctx.body.vel_b = Vec3(45.0, 4.0, 1.5);
    const std::vector<double> x{theta_e};
    const RotorLoads la = ra.evaluate(ctx, x);
    const RotorLoads lb = rb.evaluate(ctx);
    const double tol = 1e-11 * std::abs(lb.thrust);
    EXPECT_NEAR(la.F_h.x(), lb.F_h.x(), tol);
    EXPECT_NEAR(la.F_h.y(), lb.F_h.y(), tol);
    EXPECT_NEAR(la.F_h.z(), lb.F_h.z(), tol);
    EXPECT_NEAR(la.M_h.x(), lb.M_h.x(), 10.0 * tol);
    EXPECT_NEAR(la.M_h.y(), lb.M_h.y(), 10.0 * tol);
    EXPECT_NEAR(la.M_h.z(), lb.M_h.z(), 10.0 * tol);
    EXPECT_NEAR(la.blade_force, lb.blade_force, tol);
    // The twist matters: zero twist state gives different loads.
    EXPECT_GT(std::abs(ra.evaluate(ctx, std::vector<double>{0.0}).thrust - la.thrust), 1e3 * tol);
}

// dθ_e/dt = (K(V) F − θ_e)/τ. On one blade with no flapping the blade's
// aerodynamic force is the rotor's: F_n = F_h·z_h (the blade normal), F_t =
// F_h·e_t with e_t = (sin ψ, cos ψ, 0) for a counter-clockwise rotor, so F
// follows from evaluate()'s totals. V is the CG true airspeed.
TEST(DynamicTwist, StateFollowsTheBladeLoad) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    BladeElementRotorConfig c = test_rotor(syn, pitch(12.0, -4.0, 3.0), 24);
    c.num_blades = 1;
    c.dynamic_twist =
        twist(Table1D(Axis({0.0, 50.0}), {-1e-6, -3e-6}), Table1D(Axis({0.0, 1.0}), {0.28, 1.0}));
    const BladeElementRotor rotor(c);
    SimContext ctx = sea_level_context(0.0137);
    ctx.body.vel_b = Vec3(29.0, -5.0, 2.0);  // V far from the gain table's corners
    const double V = ctx.body.vel_b.norm();
    const double K = -1e-6 - 2e-6 * V / 50.0;
    const double theta_e = -0.02;
    const std::vector<double> x{theta_e};
    std::vector<double> xdot(1);
    std::vector<double> G(6);
    ComponentDerivs out{xdot, G, Wrench{}, Mat66::Zero()};
    rotor.derivatives(ctx, x, out);
    const RotorLoads l = rotor.evaluate(ctx, x);
    const double psi = rotor.azimuth(ctx.t);
    const Vec3 e_t(std::sin(psi), std::cos(psi), 0.0);
    const double F = std::hypot(l.F_h.dot(e_t), l.F_h.z());
    EXPECT_NEAR(l.blade_force, F, 1e-12 * F);
    EXPECT_NEAR(xdot[0], (K * F - theta_e) / c.dynamic_twist->lag,
                1e-10 * std::abs(K * F / c.dynamic_twist->lag));
    for (const double g : G) {
        EXPECT_EQ(g, 0.0);  // independent of the body acceleration
    }
}

TEST(DynamicTwist, RejectsANonPositiveLag) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    BladeElementRotorConfig c = test_rotor(syn, pitch(12.0, 0.0, 0.0), 12);
    c.dynamic_twist =
        twist(Table1D(Axis({0.0, 1.0}), {-1e-6, -1e-6}), Table1D(Axis({0.0, 1.0}), {0.28, 1.0}));
    c.dynamic_twist->lag = 0.0;
    EXPECT_THROW(BladeElementRotor{c}, InitError);
}

// ---- The UH-60A ---------------------------------------------------------------------

// The UH-60A config with its table paths made absolute and the twist lag
// replaced, so it can be loaded from a string.
Executive uh60a_with_lag(const std::string& lag) {
    std::ifstream f(test::uh60a_config_path());
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    const std::string dir = std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/";
    text =
        std::regex_replace(text, std::regex(R"(([ {:])([A-Za-z0-9_/]+\.csv))"), "$1" + dir + "$2");
    const std::string from = "lag: \"0.01 s\"";
    const std::size_t at = text.find(from);
    EXPECT_NE(at, std::string::npos);
    text.replace(at, from.size(), "lag: \"" + lag + "\"");
    const Config cfg = Config::from_string(text);
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

// Trims at 100 kt; returns the trim and the revolution means of θ_e and of
// K(V) F along the held orbit.
struct TwistTrim {
    TrimResult trim;
    double theta_mean = 0.0;
    double kf_mean = 0.0;
};

TwistTrim trim_100kt(const Executive& ex) {
    TrimCondition c;
    c.airspeed = 100.0 * units::kKnot;
    std::vector<double> x;
    TwistTrim out;
    out.trim = solve_trim(ex, c, nullptr, x);
    std::size_t k = 0;
    while (dynamic_cast<const BladeElementRotor*>(&ex.component(k)) == nullptr) {
        ++k;
    }
    const auto& mr = dynamic_cast<const BladeElementRotor&>(ex.component(k));
    const DynamicTwist& d = *mr.config().dynamic_twist;
    TableCursor<1> cursor;
    const double K = rounded(d.gain, c.airspeed, d.gain_corner_fraction, cursor);
    const int N = 256;
    const HeldOrbit orbit = held_orbit(ex, c, out.trim, N, N);
    const double T = trim_period(ex);
    for (int j = 0; j < N; ++j) {
        const std::vector<double>& xj = orbit.states[static_cast<std::size_t>(j)];
        const SimContext ctx = ex.context(T * j / N, xj, out.trim.controls, Environment{});
        const RotorLoads l = mr.evaluate(
            ctx, std::span<const double>(xj).subspan(ex.component_offset(k), mr.num_states()));
        out.theta_mean += xj[ex.component_offset(k) + mr.twist_offset()] / N;
        out.kf_mean += K * l.blade_force / N;
    }
    return out;
}

// The trim is GenHel's fixed point θ_e = K F (as revolution means), with a
// tip twist of GenHel's size at 100 kt (−0.0003 deg/lbf × about 3800 lbf
// per blade ≈ −1.15°; the preview probe gave −1.22° with F ≈ thrust/4), and
// halving the lag τ changes the trim only through the 4/rev ripple of θ_e.
TEST(DynamicTwist, Uh60aTrimIsGenHelsFixedPointAndIndependentOfTheLag) {
    const TwistTrim a = trim_100kt(uh60a_with_lag("0.01 s"));
    ASSERT_TRUE(a.trim.converged);
    // Periodicity makes the means equal (the mean of dθ_e/dt is zero) up to
    // the trim's periodic tolerance and the sampled revolution's step error,
    // measured 7e-8 relative; 1e-6 still catches a different load (dropping
    // the tangential force changes K F by about 1e-4).
    EXPECT_NEAR(a.theta_mean, a.kf_mean, 1e-6 * std::abs(a.kf_mean));
    EXPECT_LT(a.theta_mean / kDeg, -1.0);
    EXPECT_GT(a.theta_mean / kDeg, -1.4);
    const TwistTrim b = trim_100kt(uh60a_with_lag("0.005 s"));
    ASSERT_TRUE(b.trim.converged);
    EXPECT_NEAR(b.theta_mean, a.theta_mean, 1e-5 * std::abs(a.theta_mean));
    EXPECT_NEAR(b.trim.controls.collective, a.trim.controls.collective, 1e-5);
    EXPECT_NEAR(b.trim.controls.lon_cyclic, a.trim.controls.lon_cyclic, 1e-5);
    EXPECT_NEAR(b.trim.controls.lat_cyclic, a.trim.controls.lat_cyclic, 1e-5);
    EXPECT_NEAR(b.trim.controls.pedal, a.trim.controls.pedal, 1e-5);
    EXPECT_NEAR(b.trim.pitch, a.trim.pitch, 1e-6);
}

}  // namespace
}  // namespace eyas
