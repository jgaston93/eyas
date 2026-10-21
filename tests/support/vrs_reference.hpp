#pragma once

// Independent transcription of Johnson's vortex-ring-state model (NASA/TP-
// 2005-213477, Table 3 with the Table 4 parameters; D-049), for the M8d
// layer-1 gate. It shares no code with src/rotor/vortex_ring.cpp: momentum
// roots come from the eigenvalues of the quartic's companion matrix (not a
// bracketed Newton solve), and each cubic is built by Hermite interpolation
// with Newton divided differences (not a linear solve on the monomial basis).
// Velocities are scaled by v_h; vz is positive in climb (Johnson's V_z).

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

#include <Eigen/Dense>

namespace eyas::test {

struct VrsRefParams {
    double vz_a = -1.5, vz_b = -2.1, vx_c = 0.75;      // Table 4, baseline
    double vz_d = -0.2, vz_n = -0.45, total_n = 0.85;  // Table 4, VRS model
    double vz_x = -1.5, total_x = 1.25, vz_e = -2.0, vx_m = 0.95;
    double fade = 0.05;  // D-049: departures faded out below v_xC and v_xM
};

// D-049's fade weight: 3t² − 2t³ of t = (edge − vx)/width, clamped to [0, 1].
inline double ref_fade(double edge, double vx, double width) {
    const double t = std::clamp((edge - vx) / width, 0.0, 1.0);
    return 3.0 * t * t - 2.0 * t * t * t;
}

// Positive real roots of v⁴ + 2vz v³ + (vx² + vz²) v² − 1 = 0 (Glauert's
// v √(vx² + (vz + v)²) = 1, squared), ascending.
inline std::vector<double> ref_momentum_roots(double vx, double vz) {
    Eigen::Matrix4d C = Eigen::Matrix4d::Zero();
    // Companion matrix of v⁴ + a3 v³ + a2 v² + a1 v + a0.
    const double a3 = 2.0 * vz;
    const double a2 = vx * vx + vz * vz;
    const double a1 = 0.0;
    const double a0 = -1.0;
    C(0, 0) = -a3;
    C(0, 1) = -a2;
    C(0, 2) = -a1;
    C(0, 3) = -a0;
    C(1, 0) = 1.0;
    C(2, 1) = 1.0;
    C(3, 2) = 1.0;
    const Eigen::EigenSolver<Eigen::Matrix4d> es(C, false);
    std::vector<double> roots;
    for (int i = 0; i < 4; ++i) {
        const std::complex<double> z = es.eigenvalues()[i];
        if (std::abs(z.imag()) < 1e-7 && z.real() > 0.0) {
            // Polish with Newton on the unsquared residual's square.
            double v = z.real();
            for (int k = 0; k < 4; ++k) {
                const double f = ((v + a3) * v + a2) * v * v - 1.0;
                const double df = ((4.0 * v + 3.0 * a3) * v + 2.0 * a2) * v;
                v -= f / df;
            }
            roots.push_back(v);
        }
    }
    std::sort(roots.begin(), roots.end());
    return roots;
}

struct RefVal {
    double v;
    double dv;
};

// dv/dvz from F(v, vz) = v²(vx² + (vz + v)²) − 1 = 0.
inline RefVal ref_with_slope(double v, double vx, double vz) {
    const double Fv = 2.0 * v * (vx * vx + (vz + v) * (vz + v)) + 2.0 * v * v * (vz + v);
    const double Fz = 2.0 * v * v * (vz + v);
    return {v, -Fz / Fv};
}
inline RefVal ref_normal(double vx, double vz) {
    return ref_with_slope(ref_momentum_roots(vx, vz).back(), vx, vz);
}
inline RefVal ref_windmill(double vx, double vz) {
    return ref_with_slope(ref_momentum_roots(vx, vz).front(), vx, vz);
}

// Hermite interpolating cubic through four conditions, each a value at a
// node or a slope at a node given right after that node's value (repeated
// node). Newton divided differences.
struct HermiteCubic {
    std::array<double, 4> z{};  // nodes, repeated for slope conditions
    std::array<double, 4> c{};  // Newton coefficients
    double value(double t) const {
        return c[0] + (t - z[0]) * (c[1] + (t - z[1]) * (c[2] + (t - z[2]) * c[3]));
    }
    double slope(double t) const {
        // d/dt of the nested Newton form.
        const double p3 = c[3];
        const double p2 = c[2] + (t - z[2]) * p3;
        const double dp2 = p3;
        const double p1 = c[1] + (t - z[1]) * p2;
        const double dp1 = p2 + (t - z[1]) * dp2;
        return p1 + (t - z[0]) * dp1;
    }
};
// is_slope[i] marks data[i] as the slope at nodes[i] == nodes[i − 1]. Built
// for the two patterns Table 3 needs: [value, slope, value, slope] and
// [value, slope, value, value].
inline HermiteCubic ref_hermite(const std::array<double, 4>& nodes,
                                const std::array<double, 4>& data,
                                const std::array<bool, 4>& is_slope) {
    // Divided-difference table; a slope entry supplies f[z_i, z_i].
    HermiteCubic h;
    h.z = nodes;
    std::array<double, 4> col{};
    for (std::size_t i = 0; i < 4; ++i) {
        col[i] = is_slope[i] ? col[i - 1] : data[i];
    }
    h.c[0] = col[0];
    for (std::size_t order = 1; order < 4; ++order) {
        std::array<double, 4> next{};
        for (std::size_t i = 0; i + order < 4; ++i) {
            const double dz = nodes[i + order] - nodes[i];
            // A repeated pair (order 1 only) takes the slope given there.
            next[i] = dz == 0.0 ? data[i + 1] : (col[i + 1] - col[i]) / dz;
        }
        col = next;
        h.c[order] = col[0];
    }
    return h;
}

// Table 3 step 2, the baseline curve.
inline RefVal ref_baseline(double vx, double vz, const VrsRefParams& p = {}) {
    if (vz >= 0.0 || vx >= p.vx_c) {
        return ref_normal(vx, vz);  // step 1
    }
    const double a = p.vz_a + 0.2 * std::pow(vx / p.vx_c, 2);  // step 2.1
    double b = p.vz_b + 0.2 * std::pow(vx / p.vx_c, 2);
    if (vx / p.vx_c > 0.5) {
        b = b + 0.7 * (a - b) * std::pow(2.0 * vx / p.vx_c - 1.0, 3);
    }
    if (b < vz && vz < a) {  // step 2.2: v(A), v'(A), v(B), and v(0) = 0
        const RefVal ra = ref_normal(vx, a);
        const RefVal rb = ref_windmill(vx, b);
        const HermiteCubic h =
            ref_hermite({a, a, b, 0.0}, {ra.v, ra.dv, rb.v, 0.0}, {false, true, false, false});
        const RefVal m = ref_normal(vx, vz);  // D-049 fade toward momentum theory
        const double w = ref_fade(p.vx_c, vx, p.fade);
        return {(1.0 - w) * m.v + w * h.value(vz), (1.0 - w) * m.dv + w * h.slope(vz)};
    }
    return vz >= a ? ref_normal(vx, vz) : ref_windmill(vx, vz);  // step 2.3
}

// Table 3, the VRS model (step 3 added to the baseline).
inline RefVal ref_vrs(double vx, double vz, const VrsRefParams& p = {}) {
    const RefVal base = ref_baseline(vx, vz, p);
    if (!(vz < 0.0 && vx < p.vx_m)) {
        return base;
    }
    const double s = 1.0 - std::pow(vx / p.vx_m, 2);  // step 3.1
    const double D = p.vz_d;
    const double N = 0.5 * (p.vz_n + p.vz_x) + 0.5 * (p.vz_n - p.vz_x) * std::pow(s, 0.2);
    const double X = 0.5 * (p.vz_n + p.vz_x) - 0.5 * (p.vz_n - p.vz_x) * std::pow(s, 1.5);
    const double E = p.vz_e + (X - p.vz_x);
    if (!(E < vz && vz < D)) {
        return base;
    }
    const double amp = std::pow(1.0 - std::pow(vx / p.vx_m, 6), 0.5);
    const double vn_mom = ref_normal(0.0, p.vz_n).v;
    const double vx_mom = ref_normal(0.0, p.vz_x).v;
    const double dN = (p.total_n - (p.vz_n + vn_mom)) * amp;
    const double dX = (p.total_x - (p.vz_x + vx_mom)) * amp;
    // Step 3.2.4 (with 3d vz², DATA_GAPS.md).
    const double sN = -(1.0 + ref_baseline(vx, N, p).dv);
    const double sX = -(1.0 + ref_baseline(vx, X, p).dv);
    HermiteCubic h;
    if (N <= vz && vz < D) {  // step 3.2.1
        h = ref_hermite({D, D, N, N}, {0.0, 0.0, dN, sN}, {false, true, false, true});
    } else if (X <= vz && vz < N) {  // step 3.2.2
        h = ref_hermite({N, N, X, X}, {dN, sN, dX, sX}, {false, true, false, true});
    } else {  // step 3.2.3, ending at E (DATA_GAPS.md), constant term zero
        h = ref_hermite({X, X, E, 0.0}, {dX, sX, 0.0, 0.0}, {false, true, false, false});
    }
    const double w = ref_fade(p.vx_m, vx, p.fade);  // D-049
    return {base.v + w * h.value(vz), base.dv + w * h.slope(vz)};
}

}  // namespace eyas::test
