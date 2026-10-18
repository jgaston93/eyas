// M7b acceptance tests for the linearization (D-035 and its amendment of
// 2026-10-15, SPEC_REVIEW #29): the reduced coordinates (round trip, the
// multiblade transform, and the rates as the exact time derivative of the
// coordinates), the gravity and kinematic entries of A (L1), the symmetric
// hover fixture's constant coefficients and Floquet exponents (L2 gate), and
// the removal of states that feed nothing.

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <memory>
#include <numbers>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "env/atmosphere.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/multiblade.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7_fixtures.hpp"
#include "support/m7b_support.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;
using RC = ReducedCoordinates;

constexpr double kKnot = 1852.0 / 3600.0;  // [m/s]

Executive uh60a_executive() {
    const Config cfg = Config::from_file(test::uh60a_config_path());
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

TrimCondition level(double knots) {
    TrimCondition c;
    c.airspeed = knots * kKnot;
    return c;
}

TrimResult trim(const Executive& ex, const TrimCondition& c) {
    std::vector<double> x;
    TrimResult r = solve_trim(ex, c, nullptr, x);
    EXPECT_TRUE(r.converged);
    return r;
}

// A random full state: body velocity, rate and attitude, every component
// state, all of realistic size.
std::vector<double> random_state(const Executive& ex, std::mt19937& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    std::vector<double> x(ex.num_states(), 0.0);
    BodyState b;
    b.pos_n = Vec3(10.0 * u(rng), 10.0 * u(rng), -100.0 + 10.0 * u(rng));
    b.vel_b = Vec3(40.0 * u(rng), 5.0 * u(rng), 5.0 * u(rng));
    b.omega_b = Vec3(0.5 * u(rng), 0.5 * u(rng), 0.5 * u(rng));
    b.q_nb = quaternion(EulerAngles{0.5 * u(rng), 0.5 * u(rng), 3.0 + 3.0 * u(rng)});
    pack_body(b, x);
    for (std::size_t i = body_layout::kSize; i < x.size(); ++i) {
        x[i] = 0.05 * u(rng);
    }
    return x;
}

// ---- Reduced coordinates ---------------------------------------------------------

// to_full(to_reduced(x)) = x for the UH-60A and for 3-, 4- and 5-bladed
// fixture rotors (the general multiblade transform, Johnson 2013 Eq. 15.77).
TEST(ReducedCoordinates, RoundTrip) {
    std::mt19937 rng(7);
    std::vector<Executive> exs;
    exs.push_back(uh60a_executive());
    for (int nb : {3, 4, 5}) {
        exs.push_back(test::symmetric_hover_executive(nb));
    }
    for (const Executive& ex : exs) {
        const RC rc(ex, true);
        for (int trial = 0; trial < 20; ++trial) {
            const std::vector<double> x = random_state(ex, rng);
            const double t = 0.37 * trial;
            const VecX xr = rc.to_reduced(t, x, x);
            std::vector<double> back(x.size());
            rc.to_full(t, xr, x, back);
            for (std::size_t i = 0; i < x.size(); ++i) {
                if (i >= body_layout::kQuat && i < body_layout::kQuat + 4) {
                    continue;  // compared as rotations below
                }
                EXPECT_NEAR(back[i], x[i], 1e-12 * std::max(1.0, std::abs(x[i]))) << i;
            }
            const Mat3 C = unpack_body(x).q_nb.toRotationMatrix();
            const Mat3 C_back = unpack_body(back).q_nb.toRotationMatrix();
            EXPECT_LT((C - C_back).cwiseAbs().maxCoeff(), 1e-14);
            // And the other way round.
            const VecX xr2 = rc.to_reduced(t, back, x);
            EXPECT_LT((xr2 - xr).cwiseAbs().maxCoeff(), 1e-12);
        }
    }
}

// The UH-60A's multiblade coordinates are multiblade_flap()'s (Padfield 3rd
// ed. Eqs. 3.42–3.45), in the order β0, β1c, β1s, β_d.
TEST(ReducedCoordinates, MatchesMultibladeFlap) {
    const Executive ex = uh60a_executive();
    const RC rc(ex, true);
    std::mt19937 rng(3);
    std::size_t main = 0;
    while (dynamic_cast<const BladeElementRotor*>(&ex.component(main)) == nullptr) {
        ++main;
    }
    const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(main));
    const std::size_t off = ex.component_offset(main);
    const auto& names = rc.names();
    const std::size_t first = static_cast<std::size_t>(
        std::find(names.begin(), names.end(), "main_rotor.beta_0") - names.begin());
    ASSERT_LT(first + 4, names.size());
    EXPECT_EQ(rc.names()[first + 3], "main_rotor.beta_d");
    EXPECT_EQ(rc.names()[first + 4], "main_rotor.beta_0_dot");
    for (int trial = 0; trial < 10; ++trial) {
        const std::vector<double> x = random_state(ex, rng);
        const double t = 0.11 * trial;
        const VecX xr = rc.to_reduced(t, x, x);
        const MultibladeFlap m =
            multiblade_flap(std::span<const double>(x).subspan(off, 8), rotor.azimuth(t), 4);
        const auto at = [&](std::size_t i) { return xr(static_cast<Eigen::Index>(first + i)); };
        EXPECT_NEAR(at(0), m.beta0, 1e-15);
        EXPECT_NEAR(at(1), m.beta1c, 1e-15);
        EXPECT_NEAR(at(2), m.beta1s, 1e-15);
        EXPECT_NEAR(at(3), m.beta0d, 1e-15);
    }
}

// Gate (D-035 amendment): the reduced rates are the time derivative of the
// reduced coordinates along the free flow, d/dt to_reduced(t, x(t)) with
// ẋ = f(t, x), by central differences. Checks the Euler rates, ḣ and the
// multiblade Coriolis and centrifugal terms for 3, 4 and 5 blades.
TEST(ReducedCoordinates, RatesAreTheTimeDerivative) {
    std::mt19937 rng(11);
    std::vector<Executive> exs;
    exs.push_back(uh60a_executive());
    for (int nb : {3, 4, 5}) {
        exs.push_back(test::symmetric_hover_executive(nb));
    }
    Environment env;
    env.ground_height = std::numeric_limits<double>::quiet_NaN();
    Controls u;
    u.collective = 0.5;
    for (const Executive& ex : exs) {
        const RC rc(ex, true);
        std::vector<double> xd(ex.num_states());
        std::vector<double> xp(ex.num_states());
        std::vector<double> xm(ex.num_states());
        double worst = 0.0;
        for (int trial = 0; trial < 10; ++trial) {
            const std::vector<double> x = random_state(ex, rng);
            const double t = 0.093 * trial;
            EXPECT_TRUE(ex.derivatives(t, x, u, env, xd));
            const double d = 1e-6;
            for (std::size_t i = 0; i < x.size(); ++i) {
                xp[i] = x[i] + d * xd[i];
                xm[i] = x[i] - d * xd[i];
            }
            const VecX numeric =
                (rc.to_reduced(t + d, xp, x) - rc.to_reduced(t - d, xm, x)) / (2 * d);
            const VecX exact = rc.rates(t, x, xd);
            for (Eigen::Index i = 0; i < exact.size(); ++i) {
                worst = std::max(
                    worst, std::abs(numeric(i) - exact(i)) / std::max(1.0, std::abs(exact(i))));
            }
        }
        EXPECT_LT(worst, 1e-7);
    }
}

// ---- L1: gravity and kinematics ---------------------------------------------------

// Gate L1 (D-035): in still air the aerodynamic loads don't depend on the
// attitude, and uniform gravity accelerates every particle alike, so the φ
// and θ columns of A hold gravity alone (Padfield 3rd ed. Eq. 4.36):
// ∂u̇/∂θ = −g cos θ, ∂v̇/∂θ = −g sin θ sin φ, ∂v̇/∂φ = g cos θ cos φ,
// ∂ẇ/∂θ = −g sin θ cos φ, ∂ẇ/∂φ = −g cos θ sin φ, ḣ's
// (u sin θ − v sin φ cos θ − w cos φ cos θ) derivatives, and zero elsewhere
// (the blade flap equations and the moments included); the ψ_h column is
// zero; and with ω = 0 the Euler-rate rows are Eq. 4.37's ∂/∂(p, q, r).
// At every azimuth, so for the averaged model too.
TEST(Linearize, GravityAndKinematicEntriesAreExact) {
    const Executive ex = uh60a_executive();
    const double g = units::kStandardGravity;
    for (double knots : {0.0, 60.0, 120.0}) {
        const TrimCondition c = level(knots);
        const TrimResult r = trim(ex, c);
        LinearizeOptions o;
        o.azimuths = 8;
        const LinearModel m = linearize(ex, c, r, o);
        EXPECT_EQ(m.unconverged, 0U);
        const double th = r.pitch;
        const double ph = r.roll;
        const BodyState b = unpack_body(held_orbit_start(ex, c, r));
        const double U = b.vel_b.x();
        const double V = b.vel_b.y();
        const double W = b.vel_b.z();
        const auto n = static_cast<Eigen::Index>(m.states.size());
        for (const MatX& A : m.A_periodic) {
            VecX col_th = VecX::Zero(n);
            VecX col_ph = VecX::Zero(n);
            col_th(0) = -g * std::cos(th);
            col_th(1) = -g * std::sin(th) * std::sin(ph);
            col_th(2) = -g * std::sin(th) * std::cos(ph);
            col_ph(1) = g * std::cos(th) * std::cos(ph);
            col_ph(2) = -g * std::cos(th) * std::sin(ph);
            col_th(RC::kHeight) = U * std::cos(th) + V * std::sin(ph) * std::sin(th) +
                                  W * std::cos(ph) * std::sin(th);
            col_ph(RC::kHeight) =
                -V * std::cos(ph) * std::cos(th) + W * std::sin(ph) * std::cos(th);
            // FD precision: measured ≤ 2.5e-9, from rounding in the stiff gear
            // rows (truncation at 1e-5 rad is ~1e-10 g).
            EXPECT_LT((A.col(RC::kTheta) - col_th).cwiseAbs().maxCoeff(), 1e-7) << knots;
            EXPECT_LT((A.col(RC::kPhi) - col_ph).cwiseAbs().maxCoeff(), 1e-7) << knots;
            EXPECT_LT(A.col(RC::kPsi).cwiseAbs().maxCoeff(), 1e-9) << knots;
            // Euler-rate rows at ω = 0.
            const double tt = std::tan(th);
            const double sc = 1.0 / std::cos(th);
            const Eigen::Matrix3d E{{1.0, std::sin(ph) * tt, std::cos(ph) * tt},
                                    {0.0, std::cos(ph), -std::sin(ph)},
                                    {0.0, std::sin(ph) * sc, std::cos(ph) * sc}};
            EXPECT_LT((A.block(RC::kPhi, RC::kP, 3, 3) - E).cwiseAbs().maxCoeff(), 1e-9);
            MatX rest = A.middleRows(RC::kPhi, 3);
            rest.middleCols(RC::kP, 3).setZero();
            EXPECT_LT(rest.cwiseAbs().maxCoeff(), 1e-9);
            // ḣ row: ∂/∂(u, v, w).
            const Vec3 dh(std::sin(th), -std::sin(ph) * std::cos(th), -std::cos(ph) * std::cos(th));
            for (int k = 0; k < 3; ++k) {
                EXPECT_NEAR(A(RC::kHeight, k), dh(k), 1e-9);
            }
        }
    }
}

// ---- L2: the symmetric hover fixture -------------------------------------------

// Gate L2 (D-035 amendment): on the symmetric hover fixture the multiblade
// Jacobians agree at every azimuth, and the averaged model's eigenvalues
// equal the Floquet exponents from the free flow map, both to
// finite-difference precision.
TEST(Linearize, SymmetricHoverIsConstantCoefficient) {
    const Executive ex = test::symmetric_hover_executive(4);
    const TrimCondition c = level(0.0);
    const TrimResult r = trim(ex, c);
    EXPECT_LT(std::abs(r.pitch), 1e-9);
    EXPECT_LT(std::abs(r.roll), 1e-9);
    const LinearModel m = linearize(ex, c, r);
    double var = 0.0;
    for (const MatX& A : m.A_periodic) {
        var = std::max(var, (A - m.A).cwiseAbs().maxCoeff());
    }
    const double scale = m.A.cwiseAbs().maxCoeff();
    // Measured 9e-11 of the largest entry.
    EXPECT_LT(var / scale, 1e-8) << "largest |A_j − Ā| " << var;

    const Monodromy M = monodromy(ex, c, r);
    EXPECT_EQ(M.unconverged, 0U);
    const Eigen::VectorXcd lam = m.A.eigenvalues();
    const FloquetModes f = floquet(M.matrix, m.period, lam);
    double worst = 0.0;
    int matched = 0;
    for (Eigen::Index i = 0; i < f.exponents.size(); ++i) {
        const auto k = f.match[static_cast<std::size_t>(i)];
        if (k < 0) {
            continue;
        }
        ++matched;
        worst =
            std::max(worst, std::abs(f.exponents(i) - lam(k)) / std::max(1.0, std::abs(lam(k))));
    }
    // Every mode is matched (none vanishes within a revolution here), each
    // to 1.7e-8 relative (measured): the flow map's RK4 at T/256 and the
    // central differences.
    EXPECT_EQ(matched, f.exponents.size());
    EXPECT_LT(worst, 1e-7);
}

// ---- State elimination ---------------------------------------------------------

// The states condense() removes feed nothing else, so the eigenvalues of A
// (without h) are those of the rest together with theirs (block triangular).
TEST(Linearize, FeedNothingStatesSplitTheEigenvalues) {
    const Executive ex = uh60a_executive();
    for (double knots : {0.0, 120.0}) {
        const TrimCondition c = level(knots);
        const TrimResult r = trim(ex, c);
        LinearizeOptions o;
        o.height = false;
        o.azimuths = 16;
        const LinearModel m = linearize(ex, c, r, o);
        const std::vector<std::size_t> s = feed_nothing_states(m.A, 9);
        EXPECT_FALSE(s.empty());
        std::vector<std::size_t> keep;
        for (std::size_t i = 0; i < m.states.size(); ++i) {
            if (std::find(s.begin(), s.end(), i) == s.end()) {
                keep.push_back(i);
            }
        }
        const auto sub = [&](const std::vector<std::size_t>& idx) {
            MatX S(static_cast<Eigen::Index>(idx.size()), static_cast<Eigen::Index>(idx.size()));
            for (std::size_t a = 0; a < idx.size(); ++a) {
                for (std::size_t bb = 0; bb < idx.size(); ++bb) {
                    S(static_cast<Eigen::Index>(a), static_cast<Eigen::Index>(bb)) =
                        m.A(static_cast<Eigen::Index>(idx[a]), static_cast<Eigen::Index>(idx[bb]));
                }
            }
            return S;
        };
        std::vector<std::complex<double>> parts;
        for (const MatX& S : {sub(keep), sub(s)}) {
            const Eigen::VectorXcd e = S.eigenvalues();
            parts.insert(parts.end(), e.begin(), e.end());
        }
        const Eigen::VectorXcd all = m.A.eigenvalues();
        ASSERT_EQ(parts.size(), static_cast<std::size_t>(all.size()));
        for (const auto& z : parts) {
            double best = 1e300;
            for (const auto& w : all) {
                best = std::min(best, std::abs(z - w) / std::max(1.0, std::abs(w)));
            }
            EXPECT_LT(best, 1e-8) << knots << " kt: " << z;
        }
    }
}

// ---- Acceleration-coupled component states -----------------------------------------

// A first-order lateral accelerometer at r from the CG (D-029, D-045): its
// state y follows the body-y specific force, ẏ = (f_y − y)/τ, with
// f = v̇ + ω × v + ω̇ × r + ω × (ω × r) − g affine in a = [v̇; ω̇] through G,
// as the flight control system's sensor. Publishes probe.y.
class Probe : public DynamicComponent {
public:
    static constexpr double kTau = 0.3;                  // [s]
    static Vec3 position() { return {1.2, 0.0, -0.5}; }  // [m]

    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override { x[0] = 0.0; }
    std::string_view name() const noexcept override { return "probe"; }
    std::vector<std::string> produces() const override { return {"probe.y"}; }
    void bind(const CouplingRegistry& registry) override {
        slot_ = registry.find("probe.y", "probe");
    }
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override {
        if (out.coupling != nullptr && slot_.valid()) {
            out.coupling->set(slot_, x[0]);
        }
        const BodyState& b = ctx.body;
        const Vec3& w = b.omega_b;
        const Vec3 r = position();
        const Vec3 g_b = b.q_nb.conjugate() * gravity_n();
        const double f0 = w.cross(b.vel_b).y() + w.cross(w.cross(r)).y() - g_b.y();
        out.xdot0[0] = (f0 - x[0]) / kTau;
        out.G[1] = 1.0 / kTau;     // v̇_y
        out.G[3] = -r.z() / kTau;  // (ω̇ × r)_y = ω̇_z r_x − ω̇_x r_z
        out.G[5] = r.x() / kTau;
    }

private:
    CouplingSlot slot_;
};

// Linear damping on every body axis, the controls on X, Y, Z and N, and
// pitching and yawing moments from the probe's reading (as the stabilator's
// lateral-acceleration feedback moves the aircraft in pitch).
class Plant : public DynamicComponent {
public:
    static constexpr double kMy = 2000.0;   // [N·m per m/s²]
    static constexpr double kNy = -1500.0;  // [N·m per m/s²]
    explicit Plant(double weight) : weight_(weight) {}

    std::size_t num_states() const override { return 0; }
    void initial_states(std::span<double> /*x*/) const override {}
    std::string_view name() const noexcept override { return "plant"; }
    std::vector<std::string> consumes() const override { return {"probe.y"}; }
    void bind(const CouplingRegistry& registry) override {
        slot_ = registry.find("probe.y", "plant");
    }
    void derivatives(const SimContext& ctx, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        const double y = ctx.coupling != nullptr ? ctx.coupling->get(slot_) : 0.0;
        const Vec3& v = ctx.body.vel_b;
        const Vec3& w = ctx.body.omega_b;
        const Controls& c = ctx.controls;
        out.load0_b.F =
            Vec3(-200.0 * v.x() + 2e4 * c.lon_cyclic, -300.0 * v.y() + 2e4 * c.lat_cyclic,
                 -2.0 * weight_ * c.collective - 500.0 * v.z());
        out.load0_b.M = Vec3(-3000.0 * w.x(), -5000.0 * w.y() + kMy * y,
                             -4000.0 * w.z() + kNy * y + 1e4 * c.pedal);
    }

private:
    double weight_;
    CouplingSlot slot_;
};

// The linearization of a component state that depends on the body
// acceleration (the G path of D-004): at a trim with the body at rest
// rotationally (ω = 0), velocity (U, 0, W) and attitude (θ, φ), the probe's
// row of A is exactly
//   (A_v + U e_r − W e_p + r_x A_r − r_z A_p − g cos θ cos φ e_φ
//    + g sin θ sin φ e_θ − e_y)/τ
// from the body rows (f_y = v̇_y + (ω × v)_y + (ω̇ × r)_y − g_y, linearized),
// and the body's moment rows take the probe through I⁻¹ (0, M_y, N_y).
// In hover, at 60 and at 150 kt.
TEST(Linearize, AccelerationCoupledStatesAreExact) {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(1000.0, 500.0, 800.0, 900.0, 100.0);
    const double weight = cfg.mass.mass * units::kStandardGravity;
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<Plant>(weight));
    comps.push_back(std::make_unique<Probe>());
    const Executive ex(cfg, std::move(comps));
    const Vec3 rp = Probe::position();
    const double g = units::kStandardGravity;
    const Eigen::Vector3d dM(0.0, Plant::kMy, Plant::kNy);
    const Eigen::Vector3d dw = cfg.mass.inertia_b.inverse() * dM;
    for (double knots : {0.0, 60.0, 150.0}) {
        SCOPED_TRACE(knots);
        const TrimCondition c = level(knots);
        const TrimResult r = trim(ex, c);
        LinearizeOptions o;
        o.azimuths = 4;
        const LinearModel m = linearize(ex, c, r, o);
        const auto& names = m.states;
        const auto iy = static_cast<Eigen::Index>(
            std::find(names.begin(), names.end(), "probe[0]") - names.begin());
        ASSERT_LT(static_cast<std::size_t>(iy), names.size());
        const BodyState b = unpack_body(held_orbit_start(ex, c, r));
        const double U = b.vel_b.x();
        const double W = b.vel_b.z();
        const double th = r.pitch;
        const double ph = r.roll;
        constexpr auto kV = static_cast<Eigen::Index>(RC::kU) + 1;
        constexpr auto kPp = static_cast<Eigen::Index>(RC::kP);
        constexpr auto kRr = kPp + 2;
        for (const MatX& A : m.A_periodic) {
            VecX expect = (A.row(kV) + rp.x() * A.row(kRr) - rp.z() * A.row(kPp)).transpose();
            expect(kRr) += U;
            expect(kPp) -= W;
            expect(RC::kPhi) -= g * std::cos(th) * std::cos(ph);
            expect(RC::kTheta) += g * std::sin(th) * std::sin(ph);
            expect(iy) -= 1.0;
            expect /= Probe::kTau;
            const double scale = std::max(1.0, expect.cwiseAbs().maxCoeff());
            EXPECT_LT((A.row(iy).transpose() - expect).cwiseAbs().maxCoeff() / scale, 1e-7)
                << "row " << A.row(iy) << "\nexpect " << expect.transpose();
            for (int k = 0; k < 3; ++k) {
                EXPECT_NEAR(A(kPp + k, iy), dw(k), 1e-7 * std::max(1.0, dw.cwiseAbs().maxCoeff()));
            }
        }
    }
}

// ---- L4: Padfield's closed forms on fixtures ------------------------------------

// Gate L4 (D-035): hover heave damping against Padfield 3rd ed. Table 4.2,
// Z_w = −ρ(ΩR)πR² (∂C_T/∂μ_z)/M_a with ∂C_T/∂μ_z = 2aσλ0/(16λ0 + aσ) per
// rotor, on the symmetric fixture with the formula's assumptions: centre
// hinge, no root cutout, B = 1, linear airfoil without drag, constant chord,
// linear twist, Pitt–Peters inflow (its steady state is momentum theory).
// M_a includes the blades. The condensed (quasi-static inflow) model's Z_w
// must match to 0.5 % (measured 0.20 % and 0.09 % at the two weights: the
// small-angle and coning terms the closed form drops). The condensed heave
// root is Z_w (heave decouples in symmetric hover); the retained model's
// heave root differs by the inflow lag (7 % and 14 %; VALIDATION.md M7b).
TEST(Linearize, HoverHeaveDampingMatchesPadfield) {
    for (const double mass : {4000.0, 2000.0}) {
        test::SymmetricFixture f;
        f.hinge_offset = 0.0;
        f.cd0 = 0.0;
        f.mass = mass;
        const Executive ex = test::symmetric_hover_executive(f);
        const TrimResult r = trim(ex, level(0.0));
        LinearizeOptions o;
        o.azimuths = 16;
        o.height = false;
        const LinearModel m = linearize(ex, level(0.0), r, o);
        const CondensedModel cm = condense(m);
        const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(0));
        const BladeElementRotorConfig& cfg = rotor.config();
        const double lambda0 = r.component_states[rotor.inflow_offset()];
        const double a = test::kLiftSlope;
        const double sigma = cfg.num_blades * cfg.chord(0.5) / (std::numbers::pi * cfg.radius);
        const double Ma = mass + 2.0 * cfg.num_blades * cfg.flap->mass;
        const double rho = isa_atmosphere(0.0, 0.0).density;
        const double dct = 2.0 * a * sigma * lambda0 / (16.0 * lambda0 + a * sigma);
        const double Zw = -rho * cfg.rotor_speed * cfg.radius * std::numbers::pi * cfg.radius *
                          cfg.radius * 2.0 * dct / Ma;
        EXPECT_NEAR(cm.A(2, 2) / Zw, 1.0, 5e-3) << mass;
        const Eigen::VectorXcd ev = cm.A.eigenvalues();
        double nearest = 1e300;
        for (const auto& z : ev) {
            nearest = std::min(nearest, std::abs(z - cm.A(2, 2)));
        }
        EXPECT_LT(nearest, 1e-9 * std::abs(Zw)) << mass;
    }
}

// Gate L4 (D-035): hover flap-rate and rate-damping derivatives against
// Padfield 3rd ed. on a centre-spring rotor with uniform inflow (the
// symmetric fixture: centre hinge with spring K_β, no root cutout, B = 1,
// linear airfoil without drag), counter-clockwise rotor:
//   ∂β1c/∂q = ∂β1s/∂p = (S_β + 16/γ) / ((1 + S_β²) Ω)      (Eq. 4.84),
//   ∂β1c/∂p = −∂β1s/∂q = (S_β 16/γ − 1) / ((1 + S_β²) Ω)   (Eq. 4.85),
// S_β = 8(λ_β² − 1)/γ (Eq. 4.59), γ = ρacR⁴/I_β, quasi-steady with the body
// held; and the condensed model's M_q against Eq. 4.81 with Eq. 4.84,
// M_q I_yy = −Σ (N_b K_β/2 + T h_R) ∂β1c/∂q, on the stiff hub (S_β = 0.3)
// with a heavy airframe (blade inertia and the acceleration coupling below
// 0.2 %). Tolerances from the measured residuals: the flap derivatives
// 0.5 % for every S_β (coning and small angles), gated at 1 %; M_q 1.1 % at
// S_β = 0.3, gated at 2 %. Eq. 4.81 drops the in-plane rotor loads (Padfield
// p. 197), which grow as the hub softens: 2.7 % at S_β = 0.15, and the sign
// reverses at S_β = 0 (reported in VALIDATION.md M7b, not gated).
TEST(Linearize, HoverRateDerivativesMatchPadfield) {
    for (const double Sb : {0.0, 0.15, 0.3}) {
        test::SymmetricFixture f;
        f.hinge_offset = 0.0;
        f.cd0 = 0.0;
        f.inflow = InflowModel::uniform;
        f.hub_height = 0.5;
        f.inertia = Vec3(1e6, 1e6, 1e6);
        const BladeElementRotorConfig cfg =
            test::symmetric_rotor(f, RotationDirection::counter_clockwise);
        const double rho = isa_atmosphere(0.0, 0.0).density;
        const double W = cfg.rotor_speed;
        const double Ib = cfg.flap->inertia;
        const double gamma = rho * test::kLiftSlope * cfg.chord(0.5) * std::pow(cfg.radius, 4) / Ib;
        f.spring = Sb * gamma * Ib * W * W / 8.0;
        const Executive ex = test::symmetric_hover_executive(f);
        const TrimResult r = trim(ex, level(0.0));
        const test::QuasiSteady q = test::quasi_steady(ex, level(0.0), r);
        EXPECT_EQ(q.unconverged, 0U) << Sb;
        const double direct = (Sb + 16.0 / gamma) / ((1.0 + Sb * Sb) * W);
        const double coupling = (Sb * 16.0 / gamma - 1.0) / ((1.0 + Sb * Sb) * W);
        // Rotor 1's β1c and β1s are its reduced coordinates 1 and 2 after the body.
        EXPECT_NEAR(q.flap(1, 1) / direct, 1.0, 1e-2) << Sb;        // ∂β1c/∂q
        EXPECT_NEAR(q.flap(2, 0) / direct, 1.0, 1e-2) << Sb;        // ∂β1s/∂p
        EXPECT_NEAR(q.flap(1, 0), coupling, 1e-2 * direct) << Sb;   // ∂β1c/∂p
        EXPECT_NEAR(q.flap(2, 1), -coupling, 1e-2 * direct) << Sb;  // ∂β1s/∂q
        if (Sb < 0.3) {
            continue;
        }
        LinearizeOptions o;
        o.azimuths = 16;
        o.height = false;
        const CondensedModel cm = condense(linearize(ex, level(0.0), r, o));
        const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(0));
        const std::vector<double> x = held_orbit_start(ex, level(0.0), r);
        const SimContext ctx = ex.context(0.0, x, r.controls, trim_environment(level(0.0)));
        const double T = rotor
                             .evaluate(ctx, std::span<const double>(x).subspan(
                                                ex.component_offset(0), ex.component_size(0)))
                             .thrust;
        const double Mq = -2.0 * (cfg.num_blades * f.spring / 2.0 + T * f.hub_height) * direct;
        EXPECT_NEAR(cm.A(4, 4) * f.inertia.y() / Mq, 1.0, 2e-2);
        EXPECT_NEAR(cm.A(3, 3) * f.inertia.x() / Mq, 1.0, 2e-2);  // L_p: the same in hover
    }
}

// ---- Modes -----------------------------------------------------------------------

// modes(): frequency and damping of each root, one per conjugate pair, and
// the labels on decoupled synthetic models (states u v w p q r φ θ ψ_h).
TEST(Linearize, ModesAndLabels) {
    CondensedModel m;
    m.A = MatX::Zero(9, 9);
    m.A(2, 2) = -0.7;  // heave subsidence
    m.A(3, 3) = -5.0;  // roll subsidence, φ̇ = p
    m.A(6, 3) = 1.0;
    m.A(0, 0) = -0.02;  // phugoid in u, θ: ω_n² = 9.81 × 0.02, lightly damped
    m.A(0, 7) = -9.81;
    m.A(7, 0) = 0.02;
    m.A(5, 5) = -0.3;  // yaw subsidence
    m.A(4, 4) = -1.2;  // pitch subsidence, θ coupled only through u
    const std::vector<Mode> md = modes(m, 0.0);
    const auto find = [&](const std::string& label) {
        return std::count_if(md.begin(), md.end(), [&](const Mode& x) { return x.label == label; });
    };
    EXPECT_EQ(find("heave subsidence"), 1);
    EXPECT_EQ(find("roll subsidence"), 1);
    EXPECT_EQ(find("phugoid"), 1);
    EXPECT_EQ(find("yaw subsidence"), 1);
    EXPECT_EQ(find("pitch subsidence"), 1);
    for (const Mode& x : md) {
        EXPECT_GE(x.eigenvalue.imag(), 0.0);
        EXPECT_NEAR(x.frequency, std::abs(x.eigenvalue), 1e-15);
        if (x.label == "phugoid") {
            // λ² + 0.02 λ + 9.81 × 0.02 = 0.
            EXPECT_NEAR(x.frequency, std::sqrt(9.81 * 0.02), 1e-12);
            EXPECT_NEAR(x.damping, 0.02 / (2.0 * std::sqrt(9.81 * 0.02)), 1e-12);
        }
        if (x.label == "heave subsidence") {
            EXPECT_NEAR(x.eigenvalue.real(), -0.7, 1e-12);
            EXPECT_NEAR(x.damping, 1.0, 1e-15);
            EXPECT_GT(x.longitudinal, 0.99);
        }
    }
    // Slowest first.
    for (std::size_t i = 1; i < md.size(); ++i) {
        EXPECT_LE(md[i - 1].frequency, md[i].frequency);
    }
}

// ---- L3: linear against nonlinear ----------------------------------------------

// The frequency response is the transfer function on the imaginary axis:
// against the closed forms of a first-order roll response, p/δ = L_δ/(iω −
// L_p) from ṗ − L_p p = L_δ δ (Padfield 3rd ed. Eq. 6.2), and a second-order system, x/u =
// ω_n²/(ω_n² − ω² + 2iζω_nω), over three decades; and it refuses ω on an eigenvalue.
TEST(Linearize, FrequencyResponseMatchesClosedForms) {
    const double Lp = -3.5;
    const double Ld = 0.8;
    const double wn = 2.3;
    const double zeta = 0.15;
    MatX A1(1, 1);
    A1 << Lp;
    MatX B1(1, 1);
    B1 << Ld;
    MatX A2(2, 2);
    A2 << 0.0, 1.0, -wn * wn, -2.0 * zeta * wn;
    MatX B2(2, 1);
    B2 << 0.0, wn * wn;
    for (double w = 0.01; w <= 10.0; w *= 1.2) {
        const std::complex<double> iw(0.0, w);
        const Eigen::MatrixXcd H1 = frequency_response(A1, B1, w);
        EXPECT_NEAR(std::abs(H1(0, 0) - Ld / (iw - Lp)), 0.0, 1e-14);
        const Eigen::MatrixXcd H2 = frequency_response(A2, B2, w);
        const std::complex<double> x = wn * wn / (wn * wn - w * w + 2.0 * zeta * wn * iw);
        EXPECT_NEAR(std::abs(H2(0, 0) - x), 0.0, 1e-12 * std::abs(x));
        EXPECT_NEAR(std::abs(H2(1, 0) - iw * x), 0.0, 1e-12 * std::abs(iw * x));
    }
    MatX A0 = MatX::Zero(1, 1);
    EXPECT_THROW((void)frequency_response(A0, B1, 0.0), InitError);
    EXPECT_THROW((void)frequency_response(A2, B1, 1.0), InitError);
}

// Gate L3 (D-035 amendment, option A of 2026-10-15): small initial
// perturbations of each body coordinate and small control doublets, flown
// for 3 s, against the periodic linear model (both stepped at T/256 up to
// 60 kt, T/512 above), in hover, at 60 kt and at 150 kt; eyas_m7b_cases
// covers every 10 kt (VALIDATION.md M7b). The relative error must be the sum
// of a part proportional to the amplitude (the fit c0 + c1 a describes the
// four amplitudes to within 15 %), which proves the absolute error second
// order, and a leftover c0 below 1e-4 up to 60 kt and 1e-3 above. Half of
// the high-speed leftover is the held-body linearization (the free aircraft
// vibrates at 4/rev), half the steps. In hover the averaged model is gated
// too (the multiblade coefficients are almost constant there, SPEC_REVIEW
// #29).
// The flying UH-60A, flight control system and the stabilator's lateral-
// acceleration feedback included (D-045): the free flights start with the
// flight control system's sensors and servos on their free-flight 4/rev
// orbit (solve_trim, D-045 amendment). A negative leftover is a fit
// artifact, so |c0| is gated.
TEST(Linearize, PeriodicModelMatchesTheNonlinearResponse) {
    const Executive ex = uh60a_executive();
    TrimResult previous;
    for (double knots : {0.0, 60.0, 150.0}) {
        const TrimCondition c = level(knots);
        std::vector<double> start;
        const TrimResult r = solve_trim(ex, c, knots == 0.0 ? nullptr : &previous, start);
        ASSERT_TRUE(r.converged) << knots;
        previous = r;
        // Both models step at 2T/J: T/256 up to 60 kt, T/512 above, where the
        // rotor and inflow responses to collective need the finer step.
        LinearizeOptions o;
        o.azimuths = knots <= 60.0 ? 512 : 1024;
        o.steps_per_period = o.azimuths;
        const LinearModel m = linearize(ex, c, r, o);
        EXPECT_EQ(m.unconverged, 0U);
        const double floor = test::l3_floor(knots);
        for (const test::L3Case& k : test::l3_cases()) {
            SCOPED_TRACE(testing::Message()
                         << knots << " kt, state " << k.state << ", control " << k.control);
            const test::L3Errors e = test::l3_errors(ex, c, r, start, m, k, 3.0);
            EXPECT_EQ(e.unconverged, 0U);
            const test::L3Fit f = test::fit_l3(e.periodic, k.amplitude, 0.5 * floor);
            EXPECT_LT(f.misfit, 0.15);
            EXPECT_LT(std::abs(f.c0), floor);
            if (knots == 0.0) {
                const test::L3Fit fa = test::fit_l3(e.averaged, k.amplitude, 0.5 * 2e-3);
                EXPECT_LT(fa.misfit, 0.15) << "averaged";
                EXPECT_LT(fa.c0, 2e-3) << "averaged";
            }
        }
    }
}

}  // namespace
}  // namespace eyas
