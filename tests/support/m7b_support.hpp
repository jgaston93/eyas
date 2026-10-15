#pragma once

// Shared by the M7b linearization tests and the M7b validation generator
// (D-035 and its amendment): the L3 comparison of linear and nonlinear
// responses and the L4 quasi-steady rotor derivatives.

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Dense>

#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "sim/executive.hpp"
#include "support/m7_fixtures.hpp"

namespace eyas::test {

// One L3 case: a reduced body coordinate perturbed at t = 0, or a control
// given a one-cycle sine doublet (1 s; continuous, so RK4 stays accurate
// across it). `amplitude` is the largest of the four amplitudes, which halve.
struct L3Case {
    int state = -1;
    int control = -1;
    double amplitude = 0.0;  // [coordinate or Controls units]
};

// Every body coordinate (u v w p q r φ θ) and every control, with
// amplitudes small enough that the largest stays clear of the UH-60A's
// nearby table nodes (VALIDATION.md M7b).
inline std::vector<L3Case> l3_cases() {
    return {{0, -1, 0.0625}, {1, -1, 0.0625}, {2, -1, 0.0625}, {3, -1, 0.005},
            {4, -1, 0.005},  {5, -1, 0.005},  {6, -1, 0.0025}, {7, -1, 0.0025},
            {-1, 0, 0.0025}, {-1, 1, 0.0025}, {-1, 2, 0.0025}, {-1, 3, 0.0025}};
}

// The relative errors of the periodic and the averaged linear models'
// responses against the nonlinear one at each amplitude: all as changes
// from the unperturbed free flight, compared every linear step over
// `seconds`, over the body coordinates weighted by typical sizes (1 m/s,
// 0.1 rad/s, 0.1 rad, 1 m), as the largest error over the largest linear
// response. The free flights start from `start` (the trim's state with its
// vibration offset, as Model::trim installs) by RK4 at T/J; the linear
// models step at 2T/J (linear_response), and so do the free flights, by
// RK4, compared at every step.
struct L3Errors {
    std::vector<double> periodic;
    std::vector<double> averaged;
};

inline L3Errors l3_errors(const Executive& ex, const TrimCondition& c, const TrimResult& r,
                          const std::vector<double>& start, const LinearModel& m, const L3Case& k,
                          double seconds) {
    using VecX = Eigen::VectorXd;
    const ReducedCoordinates rc(ex, true);
    const Environment env = trim_environment(c);
    const auto per_rev = static_cast<int>(m.A_periodic.size()) / 2;  // steps per revolution
    const double dt = m.period / per_rev;
    const int steps = static_cast<int>(std::ceil(seconds / m.period)) * per_rev;
    const auto doublet = [](double t) {
        return t < 1.0 ? std::sin(2.0 * std::numbers::pi * t) : 0.0;
    };
    const auto base = free_flight(ex, env, start, [&](double) { return r.controls; }, dt, steps);
    const VecX x0 = rc.to_reduced(0.0, start, start);
    VecX w = VecX::Ones(10);
    w.segment(3, 6).setConstant(10.0);
    L3Errors out;
    for (int level = 0; level < 4; ++level) {
        const double a = k.amplitude / std::pow(2.0, level);
        VecX xr = x0;
        VecX dx0 = VecX::Zero(static_cast<Eigen::Index>(rc.size()));
        if (k.state >= 0) {
            dx0(k.state) = a;
            xr(k.state) += a;
        }
        std::vector<double> x1(start.size());
        rc.to_full(0.0, xr, start, x1);
        const auto controls_at = [&](double t) {
            Controls u = r.controls;
            if (k.control >= 0) {
                const std::array<double*, 4> p{&u.collective, &u.lat_cyclic, &u.lon_cyclic,
                                               &u.pedal};
                *p[static_cast<std::size_t>(k.control)] += a * doublet(t);
            }
            return u;
        };
        const auto input = [&](double t) {
            Eigen::Vector4d v = Eigen::Vector4d::Zero();
            if (k.control >= 0) {
                v(k.control) = a * doublet(t);
            }
            return v;
        };
        const auto pert = free_flight(ex, env, x1, controls_at, dt, steps);
        for (const bool averaged : {false, true}) {
            const std::vector<VecX> lin = linear_response(m, dx0, input, steps, averaged);
            double err = 0.0;
            double size = 0.0;
            for (int n = 0; n <= steps; ++n) {
                const double t = dt * n;
                const auto i = static_cast<std::size_t>(n);
                const VecX& l = lin[static_cast<std::size_t>(n)];
                const VecX dnl =
                    rc.to_reduced(t, pert[i], start) - rc.to_reduced(t, base[i], start);
                err = std::max(err, (dnl.head(10) - l.head(10)).cwiseProduct(w).norm());
                size = std::max(size, l.head(10).cwiseProduct(w).norm());
            }
            (averaged ? out.averaged : out.periodic).push_back(err / size);
        }
    }
    return out;
}

// e(a) ≈ c0 + c1 a: c1 a is the part of a linear model's relative error that
// shrinks with the amplitude (an absolute error of second order), c0 what
// doesn't. Least squares in relative terms. `misfit` is the largest
// deviation of the fit from the data beyond `noise`, relative to the data.
struct L3Fit {
    double c0 = 0.0;
    double c1 = 0.0;
    double misfit = 0.0;
};

inline L3Fit fit_l3(const std::vector<double>& e, double amplitude, double noise) {
    Eigen::MatrixXd M(static_cast<Eigen::Index>(e.size()), 2);
    Eigen::VectorXd y = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(e.size()));
    for (std::size_t i = 0; i < e.size(); ++i) {
        const double a = amplitude / std::pow(2.0, static_cast<double>(i));
        M(static_cast<Eigen::Index>(i), 0) = 1.0 / e[i];
        M(static_cast<Eigen::Index>(i), 1) = a / e[i];
    }
    const Eigen::Vector2d c = M.colPivHouseholderQr().solve(y);
    L3Fit f{c(0), c(1), 0.0};
    for (std::size_t i = 0; i < e.size(); ++i) {
        const double a = amplitude / std::pow(2.0, static_cast<double>(i));
        const double dev = std::max(std::abs(c(0) + c(1) * a - e[i]) - noise, 0.0);
        f.misfit = std::max(f.misfit, dev / e[i]);
    }
    return f;
}

// The L3 leftover limits (D-035 amendment, option A): 1e-4 up to 60 kt,
// 1e-3 above.
inline double l3_floor(double knots) {
    return knots <= 60.0 ? 1e-4 : 1e-3;
}

// The quasi-steady response of the rotor states to steady p and q with the
// body held (Padfield's quasi-steady derivatives: the disc settles before
// the fuselage moves), from the held derivative function, and the held net
// moment derivatives with that response.
struct QuasiSteady {
    Eigen::MatrixXd flap;    // rotor reduced coordinates × (p, q)
    Eigen::MatrixXd moment;  // net (L, M, N) × (p, q) [N·m·s/rad]
};

inline QuasiSteady quasi_steady(const Executive& ex, const TrimCondition& c, const TrimResult& r) {
    using VecX = Eigen::VectorXd;
    using RC = ReducedCoordinates;
    const RC rc(ex, false);
    const std::vector<double> ref = held_orbit_start(ex, c, r);
    const VecX x0 = rc.to_reduced(0.0, ref, ref);
    const Environment env = trim_environment(c);
    const auto nr = static_cast<Eigen::Index>(rc.size() - 9);
    std::vector<double> x(ex.num_states());
    std::vector<double> xd(ex.num_states());
    const auto g = [&](const VecX& xr) {
        rc.to_full(0.0, xr, ref, x);
        std::fill(xd.begin(), xd.end(), 0.0);
        Vec6 net;
        (void)ex.held_derivatives(0.0, x, r.controls, env, xd, net);
        VecX out(nr + 3);
        out.head(nr) = rc.rates(0.0, x, xd).tail(nr);
        out.tail(3) = net.tail<3>();
        return out;
    };
    Eigen::MatrixXd J(nr + 3, nr + 2);
    constexpr auto kP = static_cast<Eigen::Index>(RC::kP);
    for (Eigen::Index k = 0; k < nr + 2; ++k) {
        const Eigen::Index col = k < nr ? 9 + k : (k == nr ? kP : kP + 1);
        const double h = 1e-6;
        VecX xp = x0;
        VecX xm = x0;
        xp(col) += h;
        xm(col) -= h;
        J.col(k) = (g(xp) - g(xm)) / (2.0 * h);
    }
    QuasiSteady q;
    q.flap = -J.topLeftCorner(nr, nr).fullPivLu().solve(J.topRightCorner(nr, 2));
    q.moment = J.bottomRightCorner(3, 2) + J.bottomLeftCorner(3, nr) * q.flap;
    return q;
}

}  // namespace eyas::test
