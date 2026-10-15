// M7b acceptance tests for the linearization (D-035 and its amendment of
// 2026-10-15, SPEC_REVIEW #29): the reduced coordinates (round trip, the
// multiblade transform, and the rates as the exact time derivative of the
// coordinates), the gravity and kinematic entries of A (L1), the symmetric
// hover fixture's constant coefficients and Floquet exponents (L2 gate), and
// the removal of states that feed nothing.

#include <algorithm>
#include <cmath>
#include <complex>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/multiblade.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7_fixtures.hpp"
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
    const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(0));
    const std::size_t off = ex.component_offset(0);
    const std::size_t first = rc.body_size();
    EXPECT_EQ(rc.names()[first], "main_rotor.beta_0");
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
            (void)ex.derivatives(t, x, u, env, xd);
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

    const MatX M = monodromy(ex, c, r);
    const Eigen::VectorXcd lam = m.A.eigenvalues();
    const FloquetModes f = floquet(M, m.period, lam);
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

}  // namespace
}  // namespace eyas
