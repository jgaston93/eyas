#include "rotor/vortex_ring.hpp"

#include <Eigen/Core>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>

#include "util/math.hpp"

namespace eyas {
namespace {

using P = JohnsonVrsParameters;

// Glauert's momentum equation multiplied out, F(v) = v²(vx² + (vz + v)²) − 1,
// and F'(v) = 2v(2v² + 3vz v + vz² + vx²).
double momentum_f(double v, double vx, double vz) noexcept {
    const double u = vz + v;
    return v * v * (vx * vx + u * u) - 1.0;
}
double momentum_df(double v, double vx, double vz) noexcept {
    return 2.0 * v * (2.0 * v * v + 3.0 * vz * v + vz * vz + vx * vx);
}

MomentumRoot with_slope(double v, double vx, double vz) noexcept {
    // dv/dvz = −F_vz / F_v = −v(vz + v) / (vx² + (vz + v)² + v(vz + v)).
    const double u = vz + v;
    return {v, -v * u / (vx * vx + u * u + v * u)};
}

// The root of F on [lo, hi], where F rises from ≤ 0 to ≥ 0: Newton steps,
// falling back to bisection whenever a step leaves the bracket. The bracket
// shrinks every step, so the limit is never reached in practice; converged
// reports it if it is.
constexpr int kMomentumMaxIterations = 200;
MomentumRoot momentum_root_in(double lo, double hi, double vx, double vz) noexcept {
    double x = 0.5 * (lo + hi);
    for (int i = 0; i < kMomentumMaxIterations; ++i) {
        const double f = momentum_f(x, vx, vz);
        if (f == 0.0) {
            return with_slope(x, vx, vz);
        }
        if (f < 0.0) {
            lo = x;
        } else {
            hi = x;
        }
        const double df = momentum_df(x, vx, vz);
        double next = df > 0.0 ? x - f / df : 0.5 * (lo + hi);
        if (next <= lo || next >= hi) {
            next = 0.5 * (lo + hi);
        }
        if (std::abs(next - x) <= 4.0 * std::numeric_limits<double>::epsilon() * next) {
            return with_slope(next, vx, vz);
        }
        x = next;
    }
    MomentumRoot r = with_slope(x, vx, vz);
    r.converged = false;
    return r;
}

// The roots of F: F(0) = −1, and F turns at v = 0 and at the roots c1 ≤ c2
// of 2v² + 3vz v + vz² + vx², which are real and positive only for vz < 0
// and vz² > 8vx². F rises on (0, c1), falls on (c1, c2) and rises beyond
// c2. F ≥ 0 at v = max(1, 1 − vz) (there v ≥ 1 and |vz + v| ≥ 1).
MomentumRoot momentum_root(double vx, double vz, bool largest) noexcept {
    const double top = std::max(1.0, 1.0 - vz);
    const double disc = vz * vz - 8.0 * vx * vx;
    if (vz >= 0.0 || disc <= 0.0) {
        return momentum_root_in(0.0, top, vx, vz);
    }
    const double s = std::sqrt(disc);
    const double c1 = (-3.0 * vz - s) / 4.0;
    const double c2 = (-3.0 * vz + s) / 4.0;
    const bool low_root = momentum_f(c1, vx, vz) >= 0.0;   // a root in (0, c1]
    const bool high_root = momentum_f(c2, vx, vz) <= 0.0;  // a root in [c2, top)
    const bool high = largest ? high_root : !low_root;
    return high ? momentum_root_in(c2, top, vx, vz) : momentum_root_in(0.0, c1, vx, vz);
}

// Cubic p(z) = p0 + p1 z + p2 z² + p3 z³ and its slope.
struct Cubic {
    Eigen::Vector4d p = Eigen::Vector4d::Zero();
    double value(double z) const noexcept { return p[0] + z * (p[1] + z * (p[2] + z * p[3])); }
    double slope(double z) const noexcept { return p[1] + z * (2.0 * p[2] + 3.0 * z * p[3]); }
};
Eigen::RowVector4d value_row(double z) noexcept {
    return {1.0, z, z * z, z * z * z};
}
Eigen::RowVector4d slope_row(double z) noexcept {
    return {0.0, 1.0, 2.0 * z, 3.0 * z * z};
}
Cubic solve_cubic(const Eigen::Matrix4d& A, const Eigen::Vector4d& b) noexcept {
    return {A.partialPivLu().solve(b)};
}

// Table 3 step 2: the baseline curve.
MomentumRoot baseline(double vx, double vz) noexcept {
    // Step 1: momentum theory in climb and above v_xC (one root there).
    if (vz >= 0.0 || vx >= P::vx_c) {
        return momentum_normal(vx, vz);
    }
    // Step 2.1: the end points move right with vx.
    const double r = vx / P::vx_c;
    const double a = P::vz_a + 0.2 * r * r;
    double b = P::vz_b + 0.2 * r * r;
    if (r > 0.5) {
        const double t = 2.0 * r - 1.0;
        b += 0.7 * (a - b) * t * t * t;
    }
    // Step 2.2: v = b vz + c vz² + d vz³ (no constant term), matching
    // momentum v and dv/dvz at A and momentum v at B. A is on the normal
    // branch, B on the windmill branch.
    if (b < vz && vz < a) {
        const MomentumRoot ra = momentum_normal(vx, a);
        const MomentumRoot rb = momentum_windmill(vx, b);
        Eigen::Matrix4d A;
        A << value_row(a), slope_row(a), value_row(b), Eigen::RowVector4d(1.0, 0.0, 0.0, 0.0);
        const Cubic c = solve_cubic(A, Eigen::Vector4d(ra.v, ra.dv_dvz, rb.v, 0.0));
        // Slope b + 2c vz + 3d vz². MISPRINT CORRECTED (DATA_GAPS.md): Table 3
        // step 3.2.4 prints b + 2c vz + 2c vz².
        const MomentumRoot cubic{c.value(vz), c.slope(vz), ra.converged && rb.converged};
        // Eyas's fade into momentum theory below v_xC (D-049); one root there.
        const double w = smoothstep((P::vx_c - vx) / kVortexRingFadeWidth);
        if (w == 1.0) {
            return cubic;
        }
        const MomentumRoot m = momentum_normal(vx, vz);
        return {m.v + w * (cubic.v - m.v), m.dv_dvz + w * (cubic.dv_dvz - m.dv_dvz),
                m.converged && cubic.converged};
    }
    // Step 2.3: momentum theory, on the branch each end point joins.
    return vz >= a ? momentum_normal(vx, vz) : momentum_windmill(vx, vz);
}

// Momentum v at (vx = 0, vz) on the normal branch: v = −vz/2 + √(vz²/4 + 1)
// (Johnson TP-2005 p. 2, continued into descent).
double hover_normal(double vz) noexcept {
    return -0.5 * vz + std::sqrt(0.25 * vz * vz + 1.0);
}

// Table 3 step 3: the VRS increment Δ(vz + v) and its slope, zero outside
// E < vz < D.
MomentumRoot vrs_increment(double vx, double vz) noexcept {
    if (vz >= 0.0 || vx >= P::vx_m) {
        return {};
    }
    // Step 3.1: the stability boundary, N and X closing on each other as vx
    // approaches v_xM.
    const double q = vx / P::vx_m;
    const double s = 1.0 - q * q;
    const double mid = 0.5 * (P::vz_n + P::vz_x);
    const double half = 0.5 * (P::vz_n - P::vz_x);
    const double d = P::vz_d;
    const double n = mid + half * std::pow(s, 0.2);
    const double x = mid - half * std::pow(s, 1.5);
    const double e = P::vz_e + (x - P::vz_x);
    if (vz <= e || vz >= d) {
        return {};
    }
    // Step 3.2: the increments at N and X (relative to momentum theory at
    // vx = 0), scaled down with vx; zero total slope d(vz + v)/dvz there.
    const double q3 = q * q * q;
    const double amp = std::sqrt(1.0 - q3 * q3);
    const double dn = (P::total_n - (P::vz_n + hover_normal(P::vz_n))) * amp;
    const double dx = (P::total_x - (P::vz_x + hover_normal(P::vz_x))) * amp;
    const MomentumRoot bn = baseline(vx, n);
    const MomentumRoot bx = baseline(vx, x);
    const double sn = -(1.0 + bn.dv_dvz);
    const double sx = -(1.0 + bx.dv_dvz);
    Eigen::Matrix4d A;
    Eigen::Vector4d rhs;
    if (vz >= n) {
        // Step 3.2.1: D to N; value and slope zero at D.
        A << value_row(d), slope_row(d), value_row(n), slope_row(n);
        rhs << 0.0, 0.0, dn, sn;
    } else if (vz >= x) {
        // Step 3.2.2: N to X.
        A << value_row(n), slope_row(n), value_row(x), slope_row(x);
        rhs << dn, sn, dx, sx;
    } else {
        // Step 3.2.3: X to E; value zero at E, slope free, so the constant
        // term is zero. MISPRINT CORRECTED (DATA_GAPS.md): Table 3 prints
        // "match at V_zDID" for this end; the segment ends at E.
        A << value_row(x), slope_row(x), value_row(e), Eigen::RowVector4d(1.0, 0.0, 0.0, 0.0);
        rhs << dx, sx, 0.0, 0.0;
    }
    const Cubic c = solve_cubic(A, rhs);
    // Eyas's fade to zero below v_xM (D-049).
    const double w = smoothstep((P::vx_m - vx) / kVortexRingFadeWidth);
    return {w * c.value(vz), w * c.slope(vz), bn.converged && bx.converged};
}

// λ_h with λ0 = λ_h f(μ/λ_h, vz/λ_h) on one curve, λ0 > 0, from the start
// λ_h = start > 0: bracket by doubling or halving, then Brent's method
// (util/math.hpp). converged is false if the iteration limit is hit or a
// momentum root in a curve evaluation did not converge.
double solve_lambda_h(double lambda0, double mu, double vz, double start, VrsCurve curve,
                      int& count, bool& converged) noexcept {
    bool roots_converged = true;
    const auto h = [&](double lh) {
        const MomentumRoot r = johnson_inflow(mu / lh, vz / lh, curve);
        roots_converged = roots_converged && r.converged;
        ++count;
        return lh * r.v - lambda0;
    };
    double a = start;
    double fa = h(a);
    if (fa == 0.0) {
        converged = roots_converged;
        return a;
    }
    // λ_h f → 0 as λ_h → 0 and → λ_h as λ_h → ∞ (hover), and rises
    // strictly between, so a bracket exists on one side of the start.
    const double factor = fa < 0.0 ? 2.0 : 0.5;
    double b = a * factor;
    double fb = h(b);
    while ((fb < 0.0) == (fa < 0.0) && fb != 0.0 && count < kVortexRingMaxIterations) {
        a = b;
        fa = fb;
        b *= factor;
        fb = h(b);
    }
    if (fb == 0.0) {
        converged = roots_converged;
        return b;
    }
    if ((fb < 0.0) == (fa < 0.0)) {
        converged = false;
        return b;
    }
    const double lh = brent(h, a, fa, b, fb, 1e-16, kVortexRingMaxIterations, count, converged);
    converged = converged && roots_converged;
    return lh;
}

}  // namespace

MomentumRoot momentum_normal(double vx, double vz) noexcept {
    return momentum_root(vx, vz, true);
}

MomentumRoot momentum_windmill(double vx, double vz) noexcept {
    return momentum_root(vx, vz, false);
}

MomentumRoot johnson_inflow(double vx, double vz, VrsCurve curve) noexcept {
    const MomentumRoot base = baseline(vx, vz);
    if (curve == VrsCurve::baseline) {
        return base;
    }
    const MomentumRoot inc = vrs_increment(vx, vz);
    // Δ is an increment of (vz + v), so of v at fixed vz.
    return {base.v + inc.v, base.dv_dvz + inc.dv_dvz, base.converged && inc.converged};
}

VortexRingThrust vortex_ring_thrust(double lambda0, double mu, double mu_z,
                                    bool baseline) noexcept {
    VortexRingThrust out;
    // Momentum theory, as the inflow solves write it (D-024, D-027).
    const double d = lambda0 - mu_z;
    const double ct_mom = 2.0 * lambda0 * std::sqrt(mu * mu + d * d);
    out.ct_vrs = ct_mom;
    out.ct_base = ct_mom;
    out.lambda_h = std::sqrt(0.5 * std::abs(ct_mom));
    if (lambda0 == 0.0) {
        return out;  // no induced flow: zero thrust on every curve
    }
    // Mirror negative thrust onto positive.
    const double sign = lambda0 > 0.0 ? 1.0 : -1.0;
    const double l0 = sign * lambda0;
    const double vz = sign * -mu_z;  // climb positive, Johnson's V_z
    // Johnson's curves are momentum theory, with one root, for vz ≥ v_zD or
    // vx ≥ v_xM (three roots need vx ≤ 0.62 and vz ≤ −1.77): there the
    // momentum λ_h is the answer. At ideal autorotation in axial flow (μ = 0,
    // λ0 = μ_z) momentum theory gives zero thrust, the singularity the
    // baseline removes, so that point is solved too.
    const double lh_mom = out.lambda_h;
    if (lh_mom > 0.0 && (vz / lh_mom >= P::vz_d || mu / lh_mom >= P::vx_m)) {
        return out;
    }
    const double start = lh_mom > 0.0 ? lh_mom : l0;
    out.active = true;
    bool conv_vrs = false;
    const double lh_vrs =
        solve_lambda_h(l0, mu, vz, start, VrsCurve::vortex_ring, out.iterations, conv_vrs);
    out.converged = conv_vrs;
    out.ct_vrs = sign * 2.0 * lh_vrs * lh_vrs;
    out.lambda_h = lh_vrs;
    if (!baseline) {
        out.ct_base = std::numeric_limits<double>::quiet_NaN();
        return out;
    }
    bool conv_base = false;
    int base_count = 0;
    const double lh_base =
        solve_lambda_h(l0, mu, vz, start, VrsCurve::baseline, base_count, conv_base);
    out.iterations += base_count;
    out.converged = conv_vrs && conv_base;
    out.ct_base = sign * 2.0 * lh_base * lh_base;
    return out;
}

}  // namespace eyas
