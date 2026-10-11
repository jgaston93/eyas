#pragma once

// Exact-equation reference for the M4 flapping rotor (D-026, layer 1).
//
// Written independently of the production derivation in
// src/rotor/blade_element_rotor.cpp:
// - each blade is a set of point masses ("beads") on its span line;
// - every point's inertial position in NED is written as an explicit function
//   of time, under a prescribed body motion with the given acceleration a and
//   a prescribed flap motion with a trial β̈, and differentiated exactly by
//   second-order forward-mode automatic differentiation (Taylor jets). No
//   transport theorem, Coriolis or centrifugal terms are written out;
// - directions on the blade come from derivatives of the position: e_t from
//   ∂P/∂ψ, the blade normal from −∂P/∂β, the span from ∂P/∂ρ;
// - the flap equation is d'Alembert's principle for the hinge coordinate:
//   Σ (F − m P̈)·∂P/∂β − K_β β − C_β β̇ = 0, which is affine in β̈ and solved
//   from two trial values;
// - the load on the airframe is Σ (F − m P̈) over every bead and element,
//   with its moment about the airframe CG;
// - aerodynamics as in the M3 reference (Leishman 2nd ed. Eq. 3.5 with
//   φ = atan2(U_P, U_T)), with velocities from the jets, and the uniform
//   inflow by bisection on Padfield 3rd ed. Eq. 3.139.
// It shares the conventions (frames, azimuth, blade numbering, β positive
// up, δ₃ sign), the airfoil table and the production element mesh, which
// the M3 tests verify on their own.

#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <vector>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"

namespace eyas::test {

// Value, first and second derivative with respect to one variable.
struct Jet {
    double v = 0.0;
    double d1 = 0.0;
    double d2 = 0.0;
};
inline Jet operator+(Jet a, Jet b) {
    return {a.v + b.v, a.d1 + b.d1, a.d2 + b.d2};
}
inline Jet operator-(Jet a, Jet b) {
    return {a.v - b.v, a.d1 - b.d1, a.d2 - b.d2};
}
inline Jet operator-(Jet a) {
    return {-a.v, -a.d1, -a.d2};
}
inline Jet operator*(Jet a, Jet b) {
    return {a.v * b.v, a.d1 * b.v + a.v * b.d1, a.d2 * b.v + 2.0 * a.d1 * b.d1 + a.v * b.d2};
}
inline Jet operator*(double k, Jet a) {
    return {k * a.v, k * a.d1, k * a.d2};
}
inline Jet jsin(Jet a) {
    const double s = std::sin(a.v);
    const double c = std::cos(a.v);
    return {s, c * a.d1, c * a.d2 - s * a.d1 * a.d1};
}
inline Jet jcos(Jet a) {
    const double s = std::sin(a.v);
    const double c = std::cos(a.v);
    return {c, -s * a.d1, -s * a.d2 - c * a.d1 * a.d1};
}
inline Jet constant(double v) {
    return {v, 0.0, 0.0};
}

using JVec = std::array<Jet, 3>;
inline JVec operator+(const JVec& a, const JVec& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
inline JVec operator*(Jet k, const JVec& a) {
    return {k * a[0], k * a[1], k * a[2]};
}
inline JVec jcross(const JVec& a, const JVec& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline JVec jconst(const Vec3& v) {
    return {constant(v.x()), constant(v.y()), constant(v.z())};
}
// Constant matrix times a jet vector.
inline JVec jmul(const Mat3& m, const JVec& a) {
    JVec r;
    for (int i = 0; i < 3; ++i) {
        r[static_cast<std::size_t>(i)] = m(i, 0) * a[0] + m(i, 1) * a[1] + m(i, 2) * a[2];
    }
    return r;
}
inline Vec3 value(const JVec& a) {
    return {a[0].v, a[1].v, a[2].v};
}
inline Vec3 d1(const JVec& a) {
    return {a[0].d1, a[1].d1, a[2].d1};
}
inline Vec3 d2(const JVec& a) {
    return {a[0].d2, a[1].d2, a[2].d2};
}

// A blade as point masses at distances rho [m] outboard of the hinge.
struct Beads {
    std::vector<double> rho;   // [m]
    std::vector<double> mass;  // [kg]
    double m() const {
        double s = 0.0;
        for (double v : mass) {
            s += v;
        }
        return s;
    }
    double S() const {
        double s = 0.0;
        for (std::size_t i = 0; i < mass.size(); ++i) {
            s += mass[i] * rho[i];
        }
        return s;
    }
    double I() const {
        double s = 0.0;
        for (std::size_t i = 0; i < mass.size(); ++i) {
            s += mass[i] * rho[i] * rho[i];
        }
        return s;
    }
};

// Prescribed motion around the evaluation instant t0 (τ = t − t0).
struct Motion {
    BodyState body;             // at τ = 0; pos_n is the CG position
    Vec6 accel = Vec6::Zero();  // a = [v̇_b; ω̇_b] at τ = 0
};

class FlapReference {
public:
    FlapReference(const BladeElementRotor& rotor, Beads beads)
        : rotor_(rotor), c_(rotor.config()), f_(*rotor.config().flap), beads_(std::move(beads)) {
        s_ = c_.direction == RotationDirection::counter_clockwise ? 1.0 : -1.0;
        C_bh_ = rotor.C_hb().transpose();
    }

    // Inertial position (NED) of the point at distance ρ outboard of the
    // hinge of a blade at azimuth ψ and flap β, all as jets in one variable.
    // Body attitude C_nb(τ) = C_nb(0) exp([φ(τ)]×) with φ = ω τ + ½ ω̇ τ²,
    // whose body rate and its derivative at τ = 0 are ω and ω̇; exp is
    // truncated at second order, which is exact for 2-jets since φ(0) = 0.
    // CG: X(τ) = X0 + C0 v τ + ½ C0 (v̇ + ω × v) τ², whose body-axis velocity
    // and its derivative at τ = 0 are v and v̇.
    JVec point(const Motion& mo, Jet tau, Jet psi, Jet beta, Jet rho) const {
        const Mat3 C0 = mo.body.q_nb.toRotationMatrix();
        const Vec3 w = mo.body.omega_b;
        const Vec3 wd = mo.accel.tail<3>();
        const Vec3 v = mo.body.vel_b;
        const Vec3 vd = mo.accel.head<3>();
        const Jet tau2 = 0.5 * (tau * tau);
        JVec phi;
        for (std::size_t i = 0; i < 3; ++i) {
            phi[i] =
                w(static_cast<Eigen::Index>(i)) * tau + wd(static_cast<Eigen::Index>(i)) * tau2;
        }
        // Blade point in hub axes, then body axes from the CG.
        const Jet cp = jcos(psi);
        const Jet sp = jsin(psi);
        const JVec e_r{-cp, s_ * sp, constant(0.0)};
        const JVec e_z{constant(0.0), constant(0.0), constant(1.0)};
        const JVec e_s = jcos(beta) * e_r + (-jsin(beta)) * e_z;
        const JVec p_h = constant(f_.offset) * e_r + rho * e_s;
        const JVec p_b = jconst(c_.hub_position_b) + jmul(C_bh_, p_h);
        // exp([φ]×) p ≈ p + φ × p + ½ φ × (φ × p)
        const JVec phip = jcross(phi, p_b);
        const JVec rot = p_b + phip + 0.5 * constant(1.0) * jcross(phi, phip);
        const JVec p_n = jmul(C0, rot);
        const Vec3 a0 = vd + w.cross(v);
        JVec X;
        for (std::size_t i = 0; i < 3; ++i) {
            const auto ii = static_cast<Eigen::Index>(i);
            X[i] = constant(mo.body.pos_n(ii)) + (C0 * v)(ii)*tau + (C0 * a0)(ii)*tau2;
        }
        return X + p_n;
    }

    struct Result {
        std::vector<double> beta_ddot;  // per blade [rad/s²]
        Wrench load_b;                  // on the airframe, body axes, about the CG
        double lambda_i = 0.0;
    };

    // ctx: the evaluation context (t, body state, controls, env, air);
    // x: rotor states; a: the body acceleration.
    Result evaluate(const SimContext& ctx, std::span<const double> x, const Vec6& a) const {
        Motion mo{ctx.body, a};
        mo.body.pos_n = Vec3::Zero();  // moments about the CG
        const double lambda = solve_inflow(ctx, x, mo);
        Result out;
        out.lambda_i = lambda;
        const int nb = c_.num_blades;
        for (int k = 0; k < nb; ++k) {
            // Residual of d'Alembert's principle at two trial β̈; affine in β̈.
            BladeSum s0 = blade(ctx, x, mo, lambda, k, 0.0);
            BladeSum s1 = blade(ctx, x, mo, lambda, k, 1.0);
            const double bdd = -s0.residual / (s1.residual - s0.residual);
            const BladeSum sb = blade(ctx, x, mo, lambda, k, bdd);
            out.beta_ddot.push_back(bdd);
            out.load_b.F += sb.F_n;
            out.load_b.M += sb.M_n;
        }
        const Mat3 C_bn = ctx.body.q_nb.toRotationMatrix().transpose();
        out.load_b.F = C_bn * out.load_b.F;
        out.load_b.M = C_bn * out.load_b.M;
        return out;
    }

private:
    struct BladeSum {
        double residual = 0.0;    // generalized force on β including inertia [N·m]
        Vec3 F_n = Vec3::Zero();  // load on the airframe, NED
        Vec3 M_n = Vec3::Zero();  // about the CG, NED
        double thrust = 0.0;      // aerodynamic force along −z_h [N]
    };

    double azimuth(const SimContext& ctx, int k) const {
        return std::fmod(c_.rotor_speed * ctx.t, 2.0 * std::numbers::pi) -
               2.0 * std::numbers::pi * k / static_cast<double>(c_.num_blades);
    }

    // Jets in time (seeded on τ) for blade k with flap β(τ) = β + β̇ τ + ½ β̈ τ².
    JVec point_t(const SimContext& ctx, std::span<const double> x, const Motion& mo, int k,
                 double bdd, double rho) const {
        const std::size_t ib = 2 * static_cast<std::size_t>(k);
        const Jet tau{0.0, 1.0, 0.0};
        const Jet psi = constant(azimuth(ctx, k)) + c_.rotor_speed * tau;
        const Jet beta = Jet{x[ib], x[ib + 1], bdd};
        return point(mo, tau, psi, beta, constant(rho));
    }
    // ∂P/∂β, ∂P/∂ψ at the instant.
    Vec3 dP_dbeta(const SimContext& ctx, std::span<const double> x, const Motion& mo, int k,
                  double rho) const {
        const std::size_t ib = 2 * static_cast<std::size_t>(k);
        return d1(point(mo, constant(0.0), constant(azimuth(ctx, k)), Jet{x[ib], 1.0, 0.0},
                        constant(rho)));
    }
    Vec3 dP_dpsi(const SimContext& ctx, std::span<const double> x, const Motion& mo, int k,
                 double rho) const {
        const std::size_t ib = 2 * static_cast<std::size_t>(k);
        return d1(point(mo, constant(0.0), Jet{azimuth(ctx, k), 1.0, 0.0}, constant(x[ib]),
                        constant(rho)));
    }

    BladeSum blade(const SimContext& ctx, std::span<const double> x, const Motion& mo,
                   double lambda, int k, double bdd) const {
        const std::size_t ib = 2 * static_cast<std::size_t>(k);
        const Vec3 g_n = gravity_n();
        BladeSum out;
        // Beads: gravity and inertia.
        for (std::size_t i = 0; i < beads_.rho.size(); ++i) {
            const JVec P = point_t(ctx, x, mo, k, bdd, beads_.rho[i]);
            const Vec3 f = beads_.mass[i] * (g_n - d2(P));
            out.residual += f.dot(dP_dbeta(ctx, x, mo, k, beads_.rho[i]));
            out.F_n += f;
            out.M_n += value(P).cross(f);
        }
        // Aerodynamics on the production elements.
        const Mat3 C_nh = ctx.body.q_nb.toRotationMatrix() * C_bh_;
        const Vec3 shaft_n = C_nh * Vec3(0.0, 0.0, 1.0);  // z_h, down the shaft
        const double v_i = lambda * c_.rotor_speed * c_.radius;
        const SwashplatePitch pitch = c_.controls->pitch(ctx.controls);
        const double psi = azimuth(ctx, k);
        const double beta = x[ib];
        for (const BladeElement& el : rotor_.elements()) {
            const double rho = el.r - f_.offset;
            const JVec P = point_t(ctx, x, mo, k, bdd, rho);
            const Vec3 v = d1(P) - ctx.env.wind_n;  // element velocity relative to the air
            const Vec3 e_t = dP_dpsi(ctx, x, mo, k, rho).normalized();
            const Vec3 n = -dP_dbeta(ctx, x, mo, k, rho) / rho;
            const double u_t = v.dot(e_t);
            const double u_p = -v.dot(n) + v_i * shaft_n.dot(n);
            const double phi = std::atan2(u_p, u_t);
            const double theta = pitch.theta0 + pitch.theta1c * std::cos(psi) +
                                 pitch.theta1s * std::sin(psi) + el.twist -
                                 std::tan(f_.delta3) * beta;
            const double alpha = std::remainder(theta - phi, 2.0 * std::numbers::pi);
            const double u2 = u_t * u_t + u_p * u_p;
            TableCursor<2> cur;
            const AirfoilCoefficients cf =
                (*c_.airfoil)(alpha, std::sqrt(u2) / ctx.air.speed_of_sound, cur);
            const double dl = 0.5 * ctx.air.density * u2 * el.chord * el.dr * el.lift * cf.cl;
            const double dd = 0.5 * ctx.air.density * u2 * el.chord * el.dr * cf.cd;
            const double f_up = dl * std::cos(phi) - dd * std::sin(phi);  // Leishman Eq. 3.5
            const double f_x = dl * std::sin(phi) + dd * std::cos(phi);
            const Vec3 F = -f_x * e_t - f_up * n;
            out.residual += F.dot(dP_dbeta(ctx, x, mo, k, rho));
            out.F_n += F;
            out.M_n += value(P).cross(F);
            out.thrust -= F.dot(shaft_n);
        }
        out.residual -= f_.spring * x[ib] + f_.damper * x[ib + 1];
        return out;
    }

    double thrust(const SimContext& ctx, std::span<const double> x, const Motion& mo,
                  double lambda) const {
        double t = 0.0;
        for (int k = 0; k < c_.num_blades; ++k) {
            t += blade(ctx, x, mo, lambda, k, 0.0).thrust;
        }
        return t;
    }

    double solve_inflow(const SimContext& ctx, std::span<const double> x, const Motion& mo) const {
        // Hub velocity relative to the air, hub axes, from the hub-centre jet.
        const Jet tau{0.0, 1.0, 0.0};
        Motion m0 = mo;
        const Mat3 C_hn = (ctx.body.q_nb.toRotationMatrix() * C_bh_).transpose();
        // The hub centre is the point with e = 0, ρ = 0; use ρ = −e on blade 0
        // at β = 0, which lands on it.
        const JVec hub = point(m0, tau, constant(0.0), constant(0.0), constant(-f_.offset));
        const Vec3 v_h = C_hn * (d1(hub) - ctx.env.wind_n);
        const double vr = c_.rotor_speed * c_.radius;
        const double mu = std::hypot(v_h.x(), v_h.y()) / vr;
        const double mu_z = v_h.z() / vr;
        const double q = ctx.air.density * std::numbers::pi * c_.radius * c_.radius * vr * vr;
        const auto g = [&](double lam) {
            const double d = lam - mu_z;
            return 2.0 * lam * std::sqrt(mu * mu + d * d) - thrust(ctx, x, mo, lam) / q;
        };
        double lo = -0.5;
        double hi = 0.5;
        while (g(lo) > 0.0) {
            lo *= 2.0;
        }
        while (g(hi) < 0.0) {
            hi *= 2.0;
        }
        for (int i = 0; i < 200; ++i) {
            const double m = 0.5 * (lo + hi);
            if (m <= lo || m >= hi) {
                break;
            }
            (g(m) < 0.0 ? lo : hi) = m;
        }
        return 0.5 * (lo + hi);
    }

    const BladeElementRotor& rotor_;
    const BladeElementRotorConfig& c_;
    FlapHinge f_;
    Beads beads_;
    double s_ = 1.0;
    Mat3 C_bh_;
};

}  // namespace eyas::test
