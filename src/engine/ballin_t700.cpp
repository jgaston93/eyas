#include "engine/ballin_t700.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/Dense>

#include "engine/control_blocks.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using engine::lead_lag;
using engine::limited_rate;
using engine::pushes_on_limit;

// Ballin's dead bands are symmetric (Figs. C3, C10).
double dead_band(double u, double w) noexcept {
    return engine::dead_band(u, w, w);
}

// A play operator of full width w (Figs. C11, C13, C15; D-054 item 7): the
// output h stays put while the input u is within w/2 of it, else follows it
// at w/2's distance, with lag tau.
double play_rate(double u, double h, double w, double tau) noexcept {
    double target = h;
    if (u - h > 0.5 * w) {
        target = u - 0.5 * w;
    } else if (u - h < -0.5 * w) {
        target = u + 0.5 * w;
    }
    return (target - h) / tau;
}

// Newton's method on N unknowns with a forward-difference Jacobian and a
// backtracking line search on the residual's norm; `feasible` rejects steps
// leaving the residual's domain. Deterministic, bounded (spec §7). Returns
// converged; `iterations` the iterations used.
template <int N, typename F, typename Ok>
bool newton(F&& residual, Ok&& feasible, Eigen::Matrix<double, N, 1>& y, int max_iter, double tol,
            int& iterations) noexcept {
    using Vec = Eigen::Matrix<double, N, 1>;
    using Mat = Eigen::Matrix<double, N, N>;
    Vec r;
    if (!residual(y, r)) {
        iterations = 0;
        return false;
    }
    for (int it = 1; it <= max_iter; ++it) {
        Mat j;
        for (int k = 0; k < N; ++k) {
            const double h = 1e-7 * std::max(std::abs(y(k)), 1.0);
            Vec yp = y;
            yp(k) += h;
            Vec rp;
            if (!residual(yp, rp)) {
                yp(k) = y(k) - h;
                if (!residual(yp, rp)) {
                    iterations = it;
                    return false;
                }
                j.col(k) = (r - rp) / h;
            } else {
                j.col(k) = (rp - r) / h;
            }
        }
        const Vec dy = j.fullPivLu().solve(-r);
        if (!dy.allFinite()) {
            iterations = it;
            return false;
        }
        double lambda = 1.0;
        Vec yn = y;
        Vec rn = r;
        bool stepped = false;
        for (int k = 0; k < 30; ++k) {
            lambda = std::ldexp(1.0, -k);  // 1, 1/2, 1/4, ...
            yn = y + lambda * dy;
            if (feasible(yn) && residual(yn, rn) && rn.norm() <= (1.0 - 1e-4 * lambda) * r.norm()) {
                stepped = true;
                break;
            }
        }
        if (!stepped) {
            iterations = it;
            return r.norm() < tol;
        }
        y = yn;
        r = rn;
        if ((lambda * dy).cwiseAbs().maxCoeff() < tol && r.norm() < 1e3 * tol) {
            iterations = it;
            return true;
        }
    }
    iterations = max_iter;
    return r.norm() < tol;
}

constexpr int kMaxIter = 40;
constexpr double kTol = 1e-11;

void require_positive(double v, const char* what) {
    engine::require_positive(v, "ballin_t700", what);
}

}  // namespace

// ---- CurveFamily -------------------------------------------------------------

CurveFamily::CurveFamily(std::vector<double> params, std::vector<Table1D> curves)
    : params_(Axis(std::move(params))), curves_(std::move(curves)) {
    if (curves_.size() != params_.breakpoints().size() || curves_.size() < 2) {
        throw InitError("curve family: needs one curve per parameter value, at least two");
    }
}

double CurveFamily::operator()(double p, double x, Cursor& cursor) const noexcept {
    const AxisPosition at = params_.locate(p, cursor.param);
    const double lo = curves_[at.index](x, cursor.curves[at.index]);
    const double hi = curves_[at.index + 1](x, cursor.curves[at.index + 1]);
    return detail::lerp(lo, hi, at.t);
}

// ---- The gas path --------------------------------------------------------------

double BallinT700::residuals(const double* y, double ng, double np, double metal, double fuel,
                             double p2, double t2, double* r, BallinT700Gas* out) const noexcept {
    const BallinT700Config& c = config_;
    Cursors& k = cursors_;
    const double p3 = y[0] * p2;
    const double p41 = y[1] * p2;
    const double p45 = y[2] * p2;
    const double t41 = y[3] * t2;
    // Inlet and compressor (TM-100991 eqs. 1-11).
    const double h2 = c.k_h2 * t2;                           // (3)
    const double ps3 = c.k_ps3 * p3;                         // (4)
    const double theta2 = t2 / kTstd;                        // (5)
    const double ngc = ng / (c.ng_des * std::sqrt(theta2));  // (6), a fraction
    const double wa2c = (*c.f1)(ngc, ps3 / p2, k.f1);        // (7)
    const double delta2 = p2 / kPstd;                        // (8)
    const double wa2 = wa2c * delta2 / std::sqrt(theta2);    // (9)
    const double t3 = t2 * (*c.f2)(ps3 / p2, k.t[0]);        // (10)
    const double h3 = c.k_h3_1 * t3 + c.k_h3_2;              // (11)
    // Bleeds and combustor (eqs. 12-24).
    const double b1 = (*c.f3)(ngc, k.t[1]);    // (12)
    const double b2 = (*c.f4)(wa2c, k.t[2]);   // (13)
    const double b3 = (*c.f5)(wa2c, k.t[3]);   // (14)
    const double wa24bl = wa2 * (b1 + b2);     // (15)
    const double wa3bl = wa2 * (b3 + c.k_b3);  // (16)
    const double wa3 = wa2 - wa24bl;           // (17)
    const double drop = p3 * (p3 - p41);
    if (!(drop > 0.0) || !(t3 > 0.0) || !(t41 > 0.0) || !(wa2 > 0.0)) {
        return -1.0;
    }
    const double wa31 = std::sqrt(drop / (c.k_dpb * t3));         // (18)
    const double far = fuel / wa31;                               // (19)
    const double eta = (*c.f6)(far, k.t[4]);                      // (20)
    const double h41ns = (h3 + eta * far * c.hvf) / (1.0 + far);  // (21)
    const double t41ns = c.k_t41_1 * h41ns + c.k_t41_2;           // (22)
    const double h41 = c.k_h41_1 * t41 + c.k_h41_2;               // (24)
    // Turbines (eqs. 25-38).
    const double th41 = c.k_th41_1 * t41 + c.k_th41_2;  // (25)
    if (!(th41 > 0.0) || !(p41 > 0.0) || !(p45 > 0.0)) {
        return -1.0;
    }
    const double dh_gt = th41 * (*c.f7)(p45 / p41, k.t[5]);  // (26)
    const double h44 = h41 - dh_gt;                          // (27)
    const double w41 = c.k_wgt * p41 / std::sqrt(th41);      // (28)
    const double h45 = c.k_h45 * h44;                        // (29)
    const double t45 = c.k_t45_1 * h45 + c.k_t45_2;          // (30)
    const double th45 = c.k_th45_1 * t45 + c.k_th45_2;       // (31)
    if (!(th45 > 0.0)) {
        return -1.0;
    }
    const double ps9 = p2;                                                  // (37)
    const double p49 = ps9 * (*c.f10)(ngc, k.t[6]);                         // (38)
    const double dh_pt = th45 * (*c.f8)(p49 / p45, k.t[7]);                 // (32)
    const double w45 = (*c.f9)(ps9 / p45, k.t[8]) * p45 / std::sqrt(th45);  // (33), (34)
    // The heat sink's gas side (eqs. 50-53): T41 = T41ns − (τ3/τ2)(T41ns − T_m).
    const double tau2 = c.tc_t41 * std::sqrt(t41) / std::pow(w41, 0.8);  // (51)
    const double tau3 = (*c.f_hs)(ngc, k.t[9]) / w41;                    // (52), (53)
    const double m = metal > 0.0 ? metal : t41ns;  // metal ≤ 0: the heat sink at rest
    const double t41_out = t41ns - (tau3 / tau2) * (t41ns - m);
    // Continuity of volumes 3, 41, 45 (eqs. 42-44 at equilibrium; eq. 43 with
    // + W_f, D-054 item 5), relative to the inlet flow, and the heat sink.
    r[0] = (wa3 - wa3bl - wa31) / wa2;
    r[1] = (wa31 + fuel - w41) / wa2;
    r[2] = (w41 - w45 + b3 * c.k_bl * wa2) / wa2;
    r[3] = (t41 - t41_out) / t2;
    if (out != nullptr) {
        BallinT700Gas& g = *out;
        g.p3 = p3;
        g.ps3 = ps3;
        g.p41 = p41;
        g.p45 = p45;
        g.p49 = p49;
        g.t3 = t3;
        g.t41 = t41;
        g.t41ns = t41ns;
        g.t45 = t45;
        g.wa2 = wa2;
        g.wa2c = wa2c;
        g.wa3 = wa3;
        g.wa31 = wa31;
        g.w41 = w41;
        g.w45 = w45;
        g.far = far;
        g.ngc = ngc;
        // Torques (eqs. 39-41), Ballin's 778.12 (60/2π)/N in SI: power over speed.
        g.q_c = (wa2 * (c.k_qc_1 * h3 - h2) + wa3 * c.k_qc_2 * h3) / ng;  // (39)
        g.q_gt = w41 * dh_gt / ng;                                        // (40)
        g.q_pt = w45 * dh_pt / np - c.k_damp * (np - c.np_des);           // (41)
        g.metal_lag = tau2;
    }
    return 0.0;
}

BallinT700Gas BallinT700::gas(double ng, double np, double metal, double fuel, double p2,
                              double t2) const noexcept {
    using Vec = Eigen::Matrix<double, 4, 1>;
    const double ngc = ng / (config_.ng_des * std::sqrt(t2 / kTstd));
    // The start: the steady line's pressure ratios at this corrected speed and
    // the gas at the metal's temperature (D-054 item 4).
    Vec y(start_p3_->operator()(ngc, cursors_.t[14]), start_p41_->operator()(ngc, cursors_.t[15]),
          start_p45_->operator()(ngc, cursors_.t[16]), metal / t2);
    if (!(metal > 0.0)) {
        // The heat sink at rest (T41 = T41ns): the gas temperature's start by
        // fixed point from the start's pressures.
        y(3) = (config_.k_t41_2 + 1200.0) / t2;
        for (int k = 0; k < 4; ++k) {
            BallinT700Gas g;
            Vec r;
            if (residuals(y.data(), ng, np, metal, fuel, p2, t2, r.data(), &g) == 0.0) {
                y(3) = g.t41ns / t2;
            }
        }
    }
    const auto f = [&](const Vec& v, Vec& r) {
        return residuals(v.data(), ng, np, metal, fuel, p2, t2, r.data(), nullptr) == 0.0;
    };
    const auto ok = [](const Vec& v) {
        return v(0) > v(1) && v(1) > v(2) && v(2) > 0.0 && v(3) > 0.0;
    };
    BallinT700Gas g;
    int iterations = 0;
    g.converged = newton<4>(f, ok, y, kMaxIter, kTol, iterations);
    Vec r;
    if (residuals(y.data(), ng, np, metal, fuel, p2, t2, r.data(), &g) != 0.0) {
        // No gas path at this state (outside the maps' domain, or NaN): finite
        // stand-ins, so the rates stay finite, flagged unconverged (§7).
        g = BallinT700Gas{};
        g.p3 = g.ps3 = g.p41 = g.p45 = g.p49 = p2;
        g.t3 = g.t41 = g.t41ns = g.t45 = t2;
        g.metal_lag = 1.0;
        g.converged = false;
    }
    g.iterations = iterations;
    return g;
}

bool BallinT700::steady_gas(double fuel, double np, double p2, double t2, double& ng,
                            BallinT700Gas& out) const noexcept {
    // Unknowns P3, P41, P45, T41 over P2, T2 and NG over NG_des; the heat sink
    // at rest and the gas generator's torques balanced (eq. 45).
    using Vec = Eigen::Matrix<double, 5, 1>;
    const BallinT700Config& c = config_;
    const double root = std::sqrt(t2 / kTstd);
    const double n0 = ng > 0.0 ? ng / (c.ng_des * root) : 0.9;
    Vec y((*start_p3_)(n0), (*start_p41_)(n0), (*start_p45_)(n0), 0.0, n0);
    y(3) = (c.k_t41_2 + 1200.0) / t2;  // a mid-range gas temperature; refined below
    const auto f = [&](const Vec& v, Vec& r) {
        BallinT700Gas g;
        const double n = v(4) * c.ng_des * root;
        if (!(n > 0.0) || residuals(v.data(), n, np, -1.0, fuel, p2, t2, r.data(), &g) != 0.0) {
            return false;
        }
        r(4) = (g.q_gt - g.q_c) / std::max(std::abs(g.q_c), 1.0);
        return true;
    };
    const auto ok = [](const Vec& v) {
        return v(0) > v(1) && v(1) > v(2) && v(2) > 0.0 && v(3) > 0.0 && v(4) > 0.0;
    };
    int iterations = 0;
    // The gas temperature's start from the fuel: two Newton passes, the first
    // with the temperature found by fixed point from the start's pressures.
    for (int k = 0; k < 4; ++k) {
        BallinT700Gas g;
        Eigen::Matrix<double, 4, 1> r;
        if (residuals(y.data(), y(4) * c.ng_des * root, np, -1.0, fuel, p2, t2, r.data(), &g) ==
            0.0) {
            y(3) = g.t41ns / t2;
        }
    }
    const bool converged = newton<5>(f, ok, y, kMaxIter, kTol, iterations);
    ng = y(4) * c.ng_des * root;
    Eigen::Matrix<double, 4, 1> r;
    residuals(y.data(), ng, np, -1.0, fuel, p2, t2, r.data(), &out);
    out.iterations = iterations;
    out.converged = converged;
    return converged;
}

bool BallinT700::steady_for_torque(double shaft_torque, double omega, double p2, double t2,
                                   double& fuel, double& ng, BallinT700Gas& out,
                                   int engines) const noexcept {
    // Secant iteration on the fuel flow over steady_gas, which carries its own
    // Newton: the engines' torque at the shaft rises steadily with the fuel.
    const BallinT700Config& c = config_;
    const double np = c.gear * omega;
    const auto torque = [&](double wf, double& n, BallinT700Gas& g) {
        if (!steady_gas(wf, np, p2, t2, n, g)) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return (engines > 0 ? engines : c.engines) * c.gear * g.q_pt - shaft_torque;
    };
    double w0 = fuel > 0.0 ? fuel : 0.06 * p2 / kPstd;
    double n1 = ng;
    double r0 = torque(w0, n1, out);
    double w1 = 1.05 * w0;
    double r1 = torque(w1, n1, out);
    for (int it = 0; it < 60; ++it) {
        if (!std::isfinite(r0) || !std::isfinite(r1)) {
            return false;
        }
        if (std::abs(r1) < 1e-9 * std::max(std::abs(shaft_torque), 1.0)) {
            fuel = w1;
            ng = n1;
            return true;  // `out` holds the gas path at w1
        }
        const double slope = (r1 - r0) / (w1 - w0);
        if (!(slope > 0.0)) {
            return false;
        }
        double w2 = w1 - r1 / slope;
        w2 = std::clamp(w2, 0.5 * w1, 2.0 * w1);
        w0 = w1;
        r0 = r1;
        w1 = w2;
        r1 = torque(w1, n1, out);  // NG warm-started from the last solve
    }
    return false;
}

// ---- The component -----------------------------------------------------------

BallinT700::BallinT700(BallinT700Config config) : config_(std::move(config)) {
    const BallinT700Config& c = config_;
    if (c.shaft.empty()) {
        throw InitError("ballin_t700: needs a shaft (D-054)");
    }
    if (c.rotor.empty()) {
        throw InitError("ballin_t700: needs the rotor whose demand seeds trim (D-054)");
    }
    if (c.engines < 1) {
        throw InitError("ballin_t700: needs at least one engine");
    }
    for (const auto& t : {c.f2, c.f3, c.f4, c.f5, c.f6, c.f7, c.f8, c.f9, c.f10, c.f_hs, c.fhm1,
                          c.fhm2, c.fhm3, c.fhm4, c.fhm5, c.fhm6}) {
        if (!t) {
            throw InitError("ballin_t700: a map is missing");
        }
    }
    if (!c.f1 || !c.fec1 || !c.fhm7) {
        throw InitError("ballin_t700: a map is missing");
    }
    if (c.engines > static_cast<int>(kMaxEnginesPerComponent) || c.first_engine < 0) {
        throw InitError("ballin_t700: one or two engines (D-055)");
    }
    require_positive(c.gear, "gear");
    require_positive(c.ng_des, "NG_des");
    require_positive(c.np_des, "NP_des");
    require_positive(c.j_gt, "J_GT");
    require_positive(c.output_inertia, "JE");
    require_positive(c.k_dpb, "K_DPB");
    require_positive(c.k_wgt, "K_WGT");
    require_positive(c.tc_t41, "TC_T41");
    require_positive(c.tm_gain, "the torque motor's gain");
    require_positive(c.tmlvg, "the torque motor's feedback");
    require_positive(c.tmlg, "the torque motor's linkage");
    require_positive(c.drive_inertia, "the drive's inertia");
    require_positive(c.share_gain, "the load share's gain");
    if (!(c.pas_idle < c.pas_fly) || !(c.dbias < c.ce)) {
        throw InitError("ballin_t700: limits out of order");
    }
    for (const double t :
         {c.ctpl, c.ct13, c.ct14,  c.torque_lag1,    c.torque_lag2, c.ct2,   c.ct12,
          c.tlge, c.ct9,  c.t10,   c.ct16,           c.tm_lag,      c.ctps3, c.clld,
          c.cntl, c.clmv, c.delay, c.hysteresis_lag, c.ct7,         c.t17}) {
        require_positive(t, "a lag");
    }
    if (!(c.xlolim < c.xhilim) || !(c.ylolim < c.yhilim) || !(c.zlolim_low < c.zhilim) ||
        !(c.zlolim_high < c.zhilim) || !(c.wfmin >= 0.0) || !(c.wfmin < c.wfmax) ||
        !(c.wfpdcl < c.wfpdch)) {
        throw InitError("ballin_t700: limits out of order");
    }
    // The maps' floor (D-056 item 3): the highest first speed of the maps read
    // by NGC, so that every one has data above it.
    maps_floor_ =
        std::max({c.f1->params().front(), c.f3->x_axis().breakpoints().front(),
                  c.f10->x_axis().breakpoints().front(), c.f_hs->x_axis().breakpoints().front()});
    cursors_.f1 = c.f1->make_cursor();
    cursors_.fec1 = c.fec1->make_cursor();
    cursors_.fhm7 = c.fhm7->make_cursor();
    // The steady line on a standard day, from 65 % to 104 % corrected speed,
    // by continuation from the middle; its pressure ratios start the
    // quasi-steady solve (D-054 item 4). Placeholders until it is built.
    std::vector<double> n{0.6, 1.1};
    start_p3_ = std::make_shared<const Table1D>(Axis(n), std::vector<double>{6.0, 6.0});
    start_p41_ = std::make_shared<const Table1D>(Axis(n), std::vector<double>{5.7, 5.7});
    start_p45_ = std::make_shared<const Table1D>(Axis(n), std::vector<double>{1.7, 1.7});
    std::vector<double> ngc;
    std::vector<double> p3;
    std::vector<double> p41;
    std::vector<double> p45;
    const auto line = [&](int from, int to, int step) {  // corrected speeds in %
        double fuel = 0.06;
        double ng = 0.01 * from * c.ng_des;
        for (int pct = from; step > 0 ? pct <= to : pct >= to; pct += step) {
            const double v = 0.01 * pct;
            // At a given NG: secant on the fuel for the torque balance.
            double target = v * c.ng_des;
            BallinT700Gas g;
            double w0 = fuel;
            double n0 = ng;
            bool ok = steady_gas(w0, c.np_des, kPstd, kTstd, n0, g);
            double w1 = w0 * 1.02;
            double n1 = n0;
            ok = ok && steady_gas(w1, c.np_des, kPstd, kTstd, n1, g);
            for (int it = 0; ok && it < 50 && std::abs(n1 - target) > 1e-10 * c.ng_des; ++it) {
                const double w2 = w1 + (target - n1) * (w1 - w0) / (n1 - n0);
                w0 = w1;
                n0 = n1;
                w1 = std::clamp(w2, 0.5 * w1, 2.0 * w1);
                ok = steady_gas(w1, c.np_des, kPstd, kTstd, n1, g);
            }
            if (!ok) {
                break;
            }
            fuel = w1;
            ng = n1;
            ngc.push_back(v);
            p3.push_back(g.p3 / kPstd);
            p41.push_back(g.p41 / kPstd);
            p45.push_back(g.p45 / kPstd);
        }
    };
    line(93, 104, 1);
    const std::vector<double> up_n = ngc;
    const std::vector<double> up3 = p3;
    const std::vector<double> up41 = p41;
    const std::vector<double> up45 = p45;
    ngc.clear();
    p3.clear();
    p41.clear();
    p45.clear();
    line(92, 65, -1);
    if (ngc.size() + up_n.size() < 10) {
        throw InitError("ballin_t700: the steady line couldn't be built from the maps");
    }
    std::reverse(ngc.begin(), ngc.end());
    std::reverse(p3.begin(), p3.end());
    std::reverse(p41.begin(), p41.end());
    std::reverse(p45.begin(), p45.end());
    ngc.insert(ngc.end(), up_n.begin(), up_n.end());
    p3.insert(p3.end(), up3.begin(), up3.end());
    p41.insert(p41.end(), up41.begin(), up41.end());
    p45.insert(p45.end(), up45.begin(), up45.end());
    start_p3_ = std::make_shared<const Table1D>(Axis(ngc), std::move(p3));
    start_p41_ = std::make_shared<const Table1D>(Axis(ngc), std::move(p41));
    start_p45_ = std::make_shared<const Table1D>(Axis(ngc), std::move(p45));
    // The rest point at the design speed until a shaft is bound.
    find_rest(c.np_des / c.gear);
}

void BallinT700::find_rest(double omega) {
    // The rest point: zero torque at the shaft at speed omega, sea level.
    if (!steady_for_torque(drive_train_load(omega), omega, kPstd, kTstd, rest_fuel_, rest_ng_,
                           rest_gas_)) {
        throw InitError("ballin_t700: no steady state at zero shaft torque from the maps");
    }
    rest_omega_ = omega;
}

double BallinT700::rest_fuel_at(double p2) const noexcept {
    return rest_fuel_ * p2 / kPstd;
}

double BallinT700::drive_train_load(double omega) const noexcept {
    // Gearbox drag DAMPBX Ω (TM-88360 eq. A2) and the accessories, ACCHP at
    // the governed speed with 10 % per 100 % of speed (eqs. A11, A12).
    const BallinT700Config& c = config_;
    const double reference = governed_ > 0.0 ? governed_ : c.np_des / c.gear;
    const double accessories =
        c.accessory_power / reference * (1.0 + 0.1 * (omega / reference - 1.0));
    return c.gearbox_damping * omega + accessories;
}

double BallinT700::rotor_demand(const SimContext& ctx) const noexcept {
    return ctx.coupling != nullptr && demand_.valid() ? ctx.coupling->get(demand_) : 0.0;
}

std::vector<std::string> BallinT700::consumes() const {
    return {config_.rotor + ".shaft_torque"};
}

void BallinT700::bind(const CouplingRegistry& registry) {
    demand_ = registry.find(config_.rotor + ".shaft_torque", config_.name);
}

void BallinT700::bind_shafts(std::span<const ShaftInfo> shafts) {
    for (std::size_t k = 0; k < shafts.size(); ++k) {
        if (shafts[k].name == config_.shaft) {
            shaft_index_ = static_cast<int>(k);
            governed_ = shafts[k].speed;
            // The NP reference is the shaft's governed speed (D-054 item 2).
            np_reference_ = config_.gear * governed_ / config_.np_des;
            find_rest(governed_);  // the initial state at rest on this shaft
            return;
        }
    }
    throw InitError("ballin_t700 '" + config_.name + "': no shaft named '" + config_.shaft +
                    "' (D-054)");
}

bool BallinT700::on_shaft(const SimContext& ctx) const noexcept {
    return shaft_index_ >= 0 && shaft_index_ < ctx.num_shafts;
}

double BallinT700::shaft_speed(const SimContext& ctx) const noexcept {
    return on_shaft(ctx) ? ctx.shafts[static_cast<std::size_t>(shaft_index_)].omega
                         : config_.np_des / config_.gear;
}

double BallinT700::xldsa(double collective) const noexcept {
    // XLDSA = 0.914 XCPC + 5.34 deg, XCPC = 100 × collective (Fig. C13).
    return config_.rig_gain * collective + config_.rig_bias;
}

double BallinT700::spindle(double lever) const noexcept {
    // The power lever's rigging to the power-available spindle (D-055 item 2).
    const double u = std::isnan(lever) ? 1.0 : std::clamp(lever, 0.0, 1.0);
    return config_.pas_idle + u * (config_.pas_fly - config_.pas_idle);
}

double BallinT700::load_share(double share_lag) const noexcept {
    // Fig. C2: the lagged 1.1 TRQER limited to [DBIAS, CE], less DBIAS.
    return std::clamp(share_lag, config_.dbias, config_.ce) - config_.dbias;
}

double BallinT700::gain_switch(double spder, double torque_int) const noexcept {
    // B4 (Fig. C3): the relay's output (outside the speed band) plus the
    // torque integrator, against CORR. With the relay's gain far above CORR
    // it is on outside the band, or once the integrator passes CORR.
    const BallinT700Config& c = config_;
    const bool relay = spder < c.band_low || spder > c.band_high;
    const double input = c.relay_gain * c.b6 * (relay ? 1.0 : 0.0) + torque_int;
    return input > c.corr ? 1.0 : 0.0;
}

double BallinT700::zlolim(double trql) const noexcept {
    // Fig. C8: the P+I integrator's floor rises above the switch torque.
    const BallinT700Config& c = config_;
    return trql < c.zlolim_torque ? c.zlolim_low : c.zlolim_high;
}

double BallinT700::dwfp(double xldsh) const noexcept {
    // Fig. C17: KNDRP (NGREF − PNG) + (3.26 − WFQPS3).
    const BallinT700Config& c = config_;
    const double png = (*c.fhm3)(xldsh, cursors_.t[17]);
    const double wfqps3 = (*c.fhm4)(xldsh, cursors_.t[18]);
    return c.kndrp * (c.ngref - png) + (c.lds_fuel_bias - wfqps3);
}

double BallinT700::motor_input(double spdg, double pos) const noexcept {
    // Fig. C10: SPDG − 0.44 V − TMLVG × position.
    return spdg + config_.tm_bias - config_.tmlvg * pos;
}

double BallinT700::flapper_at_rest(double spdg) const noexcept {
    // The position where the torque motor's current is zero (the dead band's
    // middle): 564 (SPDG − 0.44 − TMLVG x) − 31 mA = 0, within the stops.
    const BallinT700Config& c = config_;
    return std::clamp((spdg + c.tm_bias + c.tm_offset / c.tm_gain) / c.tmlvg, c.xlolim, c.xhilim);
}

double BallinT700::ng_floor(double t2) const noexcept {
    return maps_floor_ * std::sqrt(t2 / kTstd);  // NGC = NG/(NG_des √θ2), eq. (6)
}

bool BallinT700::is_out(double ng, const BallinT700Inputs& in) const noexcept {
    return !in.fuel_on && ng <= ng_floor(in.t2);
}

BallinT700Inputs BallinT700::inputs(const SimContext& ctx, int e) const noexcept {
    BallinT700Inputs in;
    in.omega = shaft_speed(ctx);
    in.collective = ctx.controls.collective;
    const auto k = static_cast<std::size_t>(config_.first_engine) + static_cast<std::size_t>(e);
    const bool listed = k < ctx.controls.engines.size();
    in.power_lever = listed ? ctx.controls.engines[k].power_lever : 1.0;
    in.fuel_on = listed ? ctx.controls.engines[k].fuel_on : true;
    // The engine-speed trim scales the reference (D-055 item 1).
    in.np_reference = np_reference_ * speed_reference(ctx.controls.rotor_speed_reference);
    in.p2 = ctx.air.pressure;
    in.t2 = ctx.air.temperature;
    return in;
}

BallinT700Signals BallinT700::signals(std::span<const double> x,
                                      const BallinT700Inputs& in) const noexcept {
    const BallinT700Config& c = config_;
    Cursors& k = cursors_;
    BallinT700Signals s;
    const double p2 = in.p2;
    const double t2 = in.t2;
    // The metering valve's delivery: the valve's lag through the delay's Padé
    // approximant, y = u − T q̇ (D-054 item 7).
    s.fuel = std::max((x[kValve] - c.delay * x[kDelay2]) * kFuelScale, 0.0);
    const double ng = x[kNg] * c.ng_des;
    s.out = is_out(x[kNg], in);
    // The engine side turns at the shaft's speed less the clutch's slip; an
    // out engine's open clutch lets it run on at its own (D-056 item 3).
    s.engine_speed = in.omega - (s.out ? x[kSlip] : std::max(x[kSlip], 0.0));
    const double np = c.gear * s.engine_speed;
    if (s.out) {
        // Out (D-056 item 3): no gas path below the maps, so no flow and no
        // torque; the stations at the inlet's pressure and temperature.
        BallinT700Gas& g = s.gas;
        g.p3 = g.ps3 = g.p41 = g.p45 = g.p49 = p2;
        g.t3 = g.t41 = g.t41ns = g.t45 = t2;
        g.ngc = ng / (c.ng_des * std::sqrt(t2 / kTstd));
        g.metal_lag = 1.0;
        g.converged = true;
    } else {
        s.gas = gas(ng, np, x[kMetal] * kTstd, s.fuel, p2, t2);
    }
    const BallinT700Gas& g = s.gas;
    // ECU (Figs. C1-C8): speed error less the load share's trim, rate
    // compensation and gain switch.
    s.spder = x[kNpSensor] - in.np_reference - x[kXqlo];
    s.rate_input = dead_band(s.spder, c.cb);
    const double rate =
        c.zk9 * s.spder + c.zk10 * s.rate_input + c.zk5 / c.ct14 * (x[kRate1] - x[kRate2]);
    s.b4 = gain_switch(s.spder, std::clamp(x[kTorqueInt], c.ylolim, c.yhilim));
    s.spds1 = rate * (c.zk7 + s.b4 * c.zk1);
    s.spdsf = x[kSpdsf];
    // T4.5 compensation (C6) and the selector (C1; D-054 item 6(a)).
    s.tsig = lead_lag(x[kTsig], x[kTemp1], c.t8, c.t10);
    s.spdss = std::max(s.spdsf, s.tsig);
    // P+I compensation (C7, C8).
    s.zlolim = zlolim(x[kTorqueSensed]);
    s.spdsp = c.xkprop * s.spdss + std::clamp(x[kZ], s.zlolim, c.zhilim);
    // Torque motor (C10): the flapper's position trims the HMU's demand.
    s.motor_in = motor_input(x[kSpdg], x[kTmPos]);
    s.tm_current = dead_band(
        c.tm_gain * lead_lag(x[kTm1], s.motor_in, c.tm_lead, c.tm_lag) + c.tm_offset, c.tmdb);
    s.tmru = c.tmlg * std::clamp(x[kTmPos], c.xlolim, c.xhilim);
    // HMU (C9, C13-C22, C30).
    s.ps3l = x[kPs3Hys] * kPstd;
    s.xldsa = xldsa(in.collective);
    s.dwfp = dwfp(x[kLdsHys]);
    s.pas = spindle(in.power_lever);
    const double pcnghl = x[kNgSensor];
    const double rulds = std::max(x[kDwfpl] * kRatioScale + s.tmru, 0.0);
    s.wfpdm =
        (*c.fhm1)(t2, k.t[10]) + c.kndrp * (c.ngref - pcnghl) - (*c.fhm2)(s.pas, k.t[11]) - rulds;
    s.wfidm = c.kndrp * ((*c.fhm6)(t2, k.t[12]) - pcnghl) + (*c.fhm5)(t2, k.t[13]);  // + WFIRF
    s.wfpac = (*c.fhm7)(t2, pcnghl, k.fhm7);
    s.wfpdc = std::clamp(c.awfp * pcnghl + c.bwfp, c.wfpdcl, c.wfpdch);
    // Fig. C18: the idle floor, the acceleration ceiling, the deceleration floor.
    s.hmusel = std::max(std::min(std::max(s.wfpdm, s.wfidm), s.wfpac), s.wfpdc);
    // The shutoff closed, the valve's command is zero, below its minimum (D-056 item 2).
    s.fuel_command = in.fuel_on ? std::clamp(s.hmusel * s.ps3l, c.wfmin, c.wfmax) : 0.0;
    s.q_engine = c.gear * g.q_pt;
    return s;
}

unsigned BallinT700::engaged(std::span<const double> q, std::span<const double> slip,
                             double load) const noexcept {
    // The engines at zero slip engage together unless one alone would
    // accelerate slower than the drive does with the set engaged (D-055 item
    // 5; TM-88360 App. A p. 21). Removing an engine only raises the drive's
    // acceleration, so at most N passes.
    const BallinT700Config& c = config_;
    const double j_e = c.output_inertia * c.gear * c.gear;  // one engine's, at the shaft
    unsigned set = 0;
    for (std::size_t e = 0; e < q.size(); ++e) {
        if (slip[e] <= 0.0) {
            set |= 1U << e;
        }
    }
    for (std::size_t pass = 0; pass < q.size(); ++pass) {
        double torque = -load;
        double inertia = c.drive_inertia;
        for (std::size_t e = 0; e < q.size(); ++e) {
            if ((set & (1U << e)) != 0U) {
                torque += q[e];
                inertia += j_e;
            }
        }
        const double common = torque / inertia;
        unsigned next = set;
        for (std::size_t e = 0; e < q.size(); ++e) {
            if ((set & (1U << e)) != 0U && q[e] / j_e < common) {
                next &= ~(1U << e);
            }
        }
        if (next == set) {
            break;
        }
        set = next;
    }
    return set;
}

void BallinT700::derivatives(const SimContext& ctx, std::span<const double> x,
                             ComponentDerivs& out) const {
    const BallinT700Config& c = config_;
    const auto n_e = static_cast<std::size_t>(c.engines);
    const double omega = shaft_speed(ctx);
    // Each engine's signals (D-055 item 3).
    std::array<BallinT700Signals, kMaxEnginesPerComponent> sig;
    std::array<double, kMaxEnginesPerComponent> q{};
    std::array<double, kMaxEnginesPerComponent> slip{};
    for (std::size_t e = 0; e < n_e; ++e) {
        const std::span<const double> xe = x.subspan(e * kEngineStates, kEngineStates);
        sig[e] = signals(xe, inputs(ctx, static_cast<int>(e)));
        out.converged = out.converged && sig[e].gas.converged;  // the quasi-steady solve (§7)
        q[e] = sig[e].q_engine;
        // An out engine's clutch stays open (D-056 item 3).
        slip[e] = sig[e].out ? std::numeric_limits<double>::infinity() : xe[kSlip];
    }
    const double drive_load = drive_train_load(omega);
    const unsigned mask =
        engaged(std::span<const double>(q.data(), n_e), std::span<const double>(slip.data(), n_e),
                rotor_demand(ctx) + drive_load);

    const double j_e = c.output_inertia * c.gear * c.gear;  // one engine's, at the shaft
    const double h_e = j_e / c.gear;                        // its momentum per unit Ω_e
    double q_engaged = 0.0;
    double j_engaged = 0.0;
    double h_engaged = 0.0;
    Vec3 h = Vec3::Zero();       // the spools' momentum relative to the airframe
    Vec3 m_gg = Vec3::Zero();    // the gas generators' spin-up reaction
    Vec3 m_free = Vec3::Zero();  // the disengaged power turbines' spin-up reaction
    double h_free = 0.0;
    int live_spools = 0;  // gas generators free of the airframe's rotation
    for (std::size_t e = 0; e < n_e; ++e) {
        const std::size_t o = e * kEngineStates;
        const std::span<const double> xe = x.subspan(o, kEngineStates);
        const BallinT700Signals& s = sig[e];
        const BallinT700Gas& g = s.gas;
        const auto set_g = [&](std::size_t row, const Vec3& w) {  // ∂ẋ_row/∂ω̇
            for (std::size_t j = 0; j < 3; ++j) {
                out.G[(o + row) * 6 + 3 + j] = w(static_cast<Eigen::Index>(j));
            }
        };
        double* d = out.xdot0.data() + o;
        // Out (D-056 item 3), the fuel control has nothing to control and is
        // held, to resume from there at a relight; the gas generator stays on
        // the maps' floor.
        const bool live = !s.out;

        // Gas generator (eq. 45), stopped at the maps' floor (D-056 item 3), its
        // speed relative to the airframe (D-053 item 5), and the heat sink's
        // metal (eqs. 48, 51), held while out.
        const double ng_rate =
            limited_rate(xe[kNg], (g.q_gt - g.q_c) / (c.j_gt * c.ng_des),
                         ng_floor(ctx.air.temperature), std::numeric_limits<double>::infinity());
        d[kNg] = ng_rate;
        if (live) {
            set_g(kNg, -c.gg_axis_b / c.ng_des);  // out, it stays on the floor (D-056 item 3)
        }
        d[kMetal] = s.out ? 0.0 : (g.t41ns / kTstd - xe[kMetal]) / g.metal_lag;
        // ECU.
        d[kNpSensor] = (c.gear * s.engine_speed / c.np_des - xe[kNpSensor]) / c.ctpl;
        d[kRate1] = (s.rate_input - xe[kRate1]) / c.ct13;
        d[kRate2] = (xe[kRate1] - xe[kRate2]) / c.ct14;
        d[kTorqueInt] = limited_rate(xe[kTorqueInt], xe[kTorqueSensed] - c.cr, c.ylolim, c.yhilim);
        d[kTorqueLag] = (g.q_pt - xe[kTorqueLag]) / c.torque_lag1;
        d[kTorqueSensed] = (xe[kTorqueLag] - xe[kTorqueSensed]) / c.torque_lag2;
        d[kGov1] = (s.spds1 - xe[kGov1]) / c.ct2;
        d[kSpdsf] = (lead_lag(xe[kGov1], s.spds1, c.gov_lead, c.ct2) - xe[kSpdsf]) / c.ct12;
        d[kT45l] = ((g.t45 + c.t45cor) / kTstd - xe[kT45l]) / c.tlge;
        const double t45l = xe[kT45l] * kTstd;
        const double w45r = g.w45 * std::sqrt(t45l) / g.p45;  // Fig. C5
        const double tau45 = (*c.fec1)(t45l, w45r, cursors_.fec1);
        d[kT45el] = (xe[kT45l] - xe[kT45el]) / std::max(tau45, 1e-3);
        d[kTemp1] = (c.zk3 * (xe[kT45el] * kTstd - c.t45ref) - xe[kTemp1]) / c.ct9;
        d[kTsig] = (xe[kTemp1] - xe[kTsig]) / c.t10;
        d[kZ] = limited_rate(xe[kZ], c.xkintg * s.spdss, s.zlolim, c.zhilim);
        d[kSpdg] = (s.spdsp - xe[kSpdg]) / c.ct16;
        // Load share (Fig. C2): TRQER = TRQL(other) − TRQL(own); one engine,
        // the switch open.
        const double trqer =
            n_e == 2 ? x[(1 - e) * kEngineStates + kTorqueSensed] - xe[kTorqueSensed] : 0.0;
        d[kShareLag] = (c.share_gain * trqer - xe[kShareLag]) / c.ct7;
        d[kXqlo] = (c.zk8 * load_share(xe[kShareLag]) - xe[kXqlo]) / c.t17;
        // HMU.
        d[kTm1] = (s.motor_in - xe[kTm1]) / c.tm_lag;
        d[kTmPos] = limited_rate(xe[kTmPos], c.tmgn * s.tm_current, c.xlolim, c.xhilim);
        d[kPs3Lag] = (g.ps3 / kPstd - xe[kPs3Lag]) / c.ctps3;
        d[kPs3Hys] = play_rate(xe[kPs3Lag], xe[kPs3Hys], c.ps3hys / kPstd, c.hysteresis_lag);
        d[kLdsHys] = play_rate(s.xldsa, xe[kLdsHys], c.xldhys, c.hysteresis_lag);
        d[kDwfpl] = (s.dwfp / kRatioScale - xe[kDwfpl]) / c.clld;
        d[kNgHys] = play_rate(xe[kNg], xe[kNgHys], c.ch, c.hysteresis_lag);
        d[kNgSensor] = (xe[kNgHys] - xe[kNgSensor]) / c.cntl;
        d[kValve] = (s.fuel_command / kFuelScale - xe[kValve]) / c.clmv;
        // The delay's Padé approximant: (T²/12) q̈ + (T/2) q̇ + q = u.
        d[kDelay1] = xe[kDelay2];
        d[kDelay2] =
            (xe[kValve] - xe[kDelay1] - 0.5 * c.delay * xe[kDelay2]) / (c.delay * c.delay / 12.0);
        if (!live) {
            // The torque sensor still reads the engine's (zero) torque, for
            // the other engine's load share (code review of PR #77).
            const double lag = d[kTorqueLag];
            const double sensed = d[kTorqueSensed];
            std::fill(d + kMetal, d + kSlip, 0.0);
            std::fill(d + kShareLag, d + kEngineStates, 0.0);
            d[kTorqueLag] = lag;
            d[kTorqueSensed] = sensed;
        }

        // The clutch (D-055 item 5).
        if ((mask & (1U << e)) != 0U) {
            q_engaged += s.q_engine;
            j_engaged += j_e;
            h_engaged += h_e;
            // A step's overshoot below zero slip returns within kLimitReturn,
            // as a limiter's does (D-053 amendment 5).
            d[kSlip] = xe[kSlip] < 0.0 ? -xe[kSlip] / engine::kLimitReturn : 0.0;
        } else {
            // J_e (Ω̇_e + n·ω̇/n_E) = Q_e; ṡ = Ω̇ − Ω̇_e.
            d[kSlip] = -s.q_engine / j_e;
            set_g(kSlip, c.pt_axis_b / c.gear);
            if (on_shaft(ctx) && !out.Gs.empty()) {
                out.Gs[(o + kSlip) * kMaxShafts + static_cast<std::size_t>(shaft_index_)] = 1.0;
            }
            // The airframe: −(J_e/n_E) Ω̇_e n.
            m_free += (s.q_engine / c.gear) * c.pt_axis_b;
            h_free += h_e / c.gear;
        }
        h += h_e * s.engine_speed * c.pt_axis_b + c.j_gt * xe[kNg] * c.ng_des * c.gg_axis_b;
        m_gg += c.j_gt * c.ng_des * ng_rate * c.gg_axis_b;
        live_spools += live ? 1 : 0;
    }

    // The shaft: the drive train's load, and the engaged engines' torque and
    // output inertia (D-054 item 3, D-055 item 5); the spools' momentum to the
    // airframe (D-053 item 5). Not on a shaft in ctx (a context built by
    // hand), the engines run at the design speed and drive nothing.
    if (on_shaft(ctx)) {
        const auto k = static_cast<Eigen::Index>(shaft_index_);
        out.Q0(k) += q_engaged - drive_load;
        out.Ks(k, k) -= j_engaged;
        out.Ka.block<1, 3>(k, 3) -= h_engaged * c.pt_axis_b.transpose();
        out.Hs.block<3, 1>(3, k) -= h_engaged * c.pt_axis_b;
    }
    out.load0_b.M -= ctx.body.omega_b.cross(h);
    out.load0_b.M -= m_gg;
    out.load0_b.M -= m_free;
    out.H.block<3, 3>(3, 3) += (live_spools * c.j_gt) * c.gg_axis_b * c.gg_axis_b.transpose();
    if (h_free != 0.0) {
        out.H.block<3, 3>(3, 3) += h_free * c.pt_axis_b * c.pt_axis_b.transpose();
    }
}

// ---- Trim ------------------------------------------------------------------------

void BallinT700::equilibrium(double fuel, const BallinT700Inputs& in,
                             std::span<double> x) const noexcept {
    const BallinT700Config& c = config_;
    const double p2 = in.p2;
    const double t2 = in.t2;
    std::fill(x.begin(), x.end(), 0.0);
    double ng = 0.0;
    BallinT700Gas g;
    if (!steady_gas(fuel, c.gear * in.omega, p2, t2, ng, g)) {
        // No steady gas path for this fuel: the rest point's fuel scaled to the
        // inlet's pressure, at the inlet; failing that, the sea-level rest
        // point itself, its sensors then off the inlet's values. Trim moves on
        // from either.
        fuel = rest_fuel_at(p2);
        if (!steady_gas(fuel, c.gear * in.omega, p2, t2, ng, g)) {
            fuel = rest_fuel_;
            ng = rest_ng_;
            g = rest_gas_;
        }
    }
    x[kNg] = ng / c.ng_des;
    x[kMetal] = g.t41ns / kTstd;
    // ECU at rest: sensors at their inputs, lags at their inputs; engaged,
    // the load share at rest (the other engine's torque sensed at this one's).
    const double pcnp = c.gear * in.omega / c.np_des;
    x[kNpSensor] = pcnp;
    const double spder = pcnp - in.np_reference;
    const double u1 = dead_band(spder, c.cb);
    x[kRate1] = u1;
    x[kRate2] = u1;
    x[kTorqueLag] = g.q_pt;
    x[kTorqueSensed] = g.q_pt;
    x[kTorqueInt] = g.q_pt > c.cr ? c.yhilim : c.ylolim;
    const double spds1 =
        (c.zk9 * spder + c.zk10 * u1) * (c.zk7 + gain_switch(spder, x[kTorqueInt]) * c.zk1);
    x[kGov1] = spds1;
    x[kSpdsf] = spds1;
    x[kT45l] = (g.t45 + c.t45cor) / kTstd;
    x[kT45el] = x[kT45l];
    const double tsig = c.zk3 * (x[kT45el] * kTstd - c.t45ref);
    x[kTemp1] = tsig;
    x[kTsig] = tsig;
    const double spdss = std::max(spds1, tsig);
    // HMU at rest: the sensors at their inputs, the hystereses in the middle
    // of their bands (D-054 item 7).
    x[kPs3Lag] = g.ps3 / kPstd;
    x[kPs3Hys] = x[kPs3Lag];
    x[kLdsHys] = xldsa(in.collective);
    const double command = dwfp(x[kLdsHys]);
    x[kDwfpl] = command / kRatioScale;
    x[kNgHys] = x[kNg];
    x[kNgSensor] = x[kNg];
    // The torque motor trims the demand down to the fuel's ratio: the flapper
    // where it must be, at rest (the dead band's middle) for its SPDG, and the
    // P+I integrator holding that SPDG, as far as the stops and limits allow.
    const double ratio = fuel / g.ps3;
    const double before =
        (*c.fhm1)(t2) + c.kndrp * (c.ngref - x[kNgSensor]) - (*c.fhm2)(spindle(in.power_lever));
    const double tmru = before - ratio - command;
    const double pos = std::clamp(tmru / c.tmlg, c.xlolim, c.xhilim);
    x[kTmPos] = pos;
    const double spdg = c.tmlvg * pos - c.tm_bias - c.tm_offset / c.tm_gain;  // flapper_at_rest⁻¹
    x[kZ] = std::clamp(spdg - c.xkprop * spdss, zlolim(x[kTorqueSensed]), c.zhilim);
    x[kSpdg] = c.xkprop * spdss + x[kZ];
    x[kTm1] = motor_input(x[kSpdg], pos);
    x[kValve] = fuel / kFuelScale;
    x[kDelay1] = x[kValve];
    x[kDelay2] = 0.0;
    x[kSlip] = 0.0;
    x[kShareLag] = 0.0;
    x[kXqlo] = c.zk8 * load_share(0.0);
}

void BallinT700::initial_states(std::span<double> x) const {
    // The rest point (sea level, zero torque at the shaft) with the collective
    // down and the levers at fly: at the bound shaft's governed speed, no
    // speed error, or before binding at the design speed.
    BallinT700Inputs in;
    in.omega = rest_omega_;
    in.np_reference = np_reference_;
    in.p2 = kPstd;
    in.t2 = kTstd;
    for (std::size_t e = 0; e < static_cast<std::size_t>(config_.engines); ++e) {
        equilibrium(rest_fuel_, in, x.subspan(e * kEngineStates, kEngineStates));
    }
}

void BallinT700::steady_states(const SimContext& ctx, std::span<double> x) const {
    // Held in trim (D-054 item 7): the torque integrator at the limit its input
    // drives it to; the play operators at their inputs; the flapper at rest
    // for SPDG (no current), within its stops; the clutch's slip where it is
    // steady (D-056 item 5). An out engine is left as it is.
    const BallinT700Config& c = config_;
    for (std::size_t e = 0; e < static_cast<std::size_t>(c.engines); ++e) {
        const std::span<double> xe = x.subspan(e * kEngineStates, kEngineStates);
        const BallinT700Inputs in = inputs(ctx, static_cast<int>(e));
        if (is_out(xe[kNg], in)) {
            continue;  // held where it is, its clutch at zero slip (D-056 item 3)
        }
        xe[kTorqueInt] = xe[kTorqueSensed] > c.cr ? c.yhilim : c.ylolim;
        xe[kPs3Hys] = xe[kPs3Lag];
        xe[kLdsHys] = xldsa(ctx.controls.collective);
        xe[kNgHys] = xe[kNg];
        xe[kTmPos] = flapper_at_rest(xe[kSpdg]);
        xe[kSlip] = steady_slip(ctx, xe, in);
    }
}

double BallinT700::steady_slip(const SimContext& ctx, std::span<const double> x,
                               const BallinT700Inputs& in) const noexcept {
    // Steady, the drive's mean acceleration is zero, so a lit engine is
    // engaged where its torque at the shaft's speed is not negative (D-055 item
    // 5's rule). Otherwise its power turbine runs free where its torque is
    // zero: eq. (41), Q_PT = A/NP − K_damp (NP − NP_des) with A = W45 ΔH_PT,
    // which doesn't depend on NP, is zero at the positive root of
    // K_damp NP² − K_damp NP_des NP − A = 0. At Q_PT = 0 that root is the
    // shaft's own speed, so the slip is continuous. An out engine's open
    // clutch has no steady slip of its own; it starts at zero (D-056 item 5).
    const BallinT700Config& c = config_;
    std::array<double, kEngineStates> engaged{};
    std::copy(x.begin(), x.end(), engaged.begin());
    engaged[kSlip] = 0.0;
    const BallinT700Signals s = signals(engaged, in);
    if (s.out || !(s.gas.q_pt < 0.0) || !on_shaft(ctx)) {
        return 0.0;
    }
    const double np = c.gear * in.omega;
    const double a = np * (s.gas.q_pt + c.k_damp * (np - c.np_des));
    const double disc = c.np_des * c.np_des + 4.0 * a / c.k_damp;
    if (!(disc >= 0.0)) {
        return in.omega;  // no speed holds it: at rest
    }
    const double np_free = 0.5 * (c.np_des + std::sqrt(disc));
    return std::max(in.omega - np_free / c.gear, 0.0);
}

void BallinT700::steady_kinds(std::span<SteadyKind> kinds) const {
    std::fill(kinds.begin(), kinds.end(), SteadyKind::free);
    for (std::size_t e = 0; e < static_cast<std::size_t>(config_.engines); ++e) {
        for (const std::size_t i : {kTorqueInt, kPs3Hys, kLdsHys, kNgHys, kTmPos, kSlip}) {
            kinds[e * kEngineStates + i] = SteadyKind::held;
        }
    }
}

void BallinT700::trim_seed(const SimContext& ctx, std::span<double> x) const {
    // The engines with their fuel on share the rotor's demand and the drive
    // train's load equally; those with it off are out (D-056).
    const double omega = shaft_speed(ctx);
    const double load = rotor_demand(ctx) + drive_train_load(omega);
    int lit = 0;
    for (int e = 0; e < config_.engines; ++e) {
        lit += inputs(ctx, e).fuel_on ? 1 : 0;
    }
    // A rotor that drives the drive (the load negative: a descent it
    // autorotates in) overruns the clutches: the lit engines then run free,
    // governed at their reference speed with no load (D-056 item 5).
    // Only a shaft faster than the reference runs ahead of them; at a slower
    // one they are engaged with no load (code review of PR #77).
    const double reference = inputs(ctx, 0).np_reference * config_.np_des / config_.gear;
    const bool overrun = !(load > 0.0) && omega > reference;
    double free_speed = omega;  // the engine side's speed at the shaft [rad/s]
    if (overrun && lit > 0) {
        free_speed = reference;
    }
    double fuel = 0.0;
    double ng = 0.0;
    BallinT700Gas g;
    if (lit > 0 && !steady_for_torque(load > 0.0 ? load : 0.0, free_speed, ctx.air.pressure,
                                      ctx.air.temperature, fuel, ng, g, lit)) {
        // Past the maps (a demand no steady state delivers): seed from the rest
        // point's fuel, scaled to the inlet's pressure; trim moves on from it.
        fuel = rest_fuel_at(ctx.air.pressure);
    }
    for (std::size_t e = 0; e < static_cast<std::size_t>(config_.engines); ++e) {
        BallinT700Inputs in = inputs(ctx, static_cast<int>(e));
        const std::span<double> xe = x.subspan(e * kEngineStates, kEngineStates);
        if (in.fuel_on) {
            in.omega = free_speed;
            equilibrium(fuel, in, xe);
            xe[kSlip] = std::max(omega - free_speed, 0.0);
        } else {
            out_rest(in, xe);
        }
    }
}

void BallinT700::out_rest(const BallinT700Inputs& in, std::span<double> x) const noexcept {
    // An out engine at rest (D-056 item 3): its fuel control as at the rest
    // point, which it holds while out; the gas generator on the maps' floor,
    // its metal at the inlet's temperature, no fuel in the valve or the line,
    // no torque sensed, the clutch at zero slip.
    const BallinT700Config& c = config_;
    equilibrium(rest_fuel_at(in.p2), in, x);
    x[kNg] = ng_floor(in.t2);
    x[kMetal] = in.t2 / kTstd;
    x[kTorqueLag] = 0.0;
    x[kTorqueSensed] = 0.0;
    x[kTorqueInt] = c.ylolim;
    x[kValve] = 0.0;
    x[kDelay1] = 0.0;
    x[kDelay2] = 0.0;
    x[kSlip] = 0.0;
}

void BallinT700::pinned_states(const SimContext& ctx, std::span<const double> x,
                               std::span<unsigned char> pinned) const {
    // Held for small perturbations: the play operators inside their bands, the
    // flapper inside its dead band (no current) or pressed on a stop, the
    // torque integrator and the P+I integrator on a limit their inputs push
    // against, and the slip of an engaged clutch.
    const BallinT700Config& c = config_;
    const auto n_e = static_cast<std::size_t>(c.engines);
    std::fill(pinned.begin(), pinned.end(), static_cast<unsigned char>(0));
    std::array<double, kMaxEnginesPerComponent> q{};
    std::array<double, kMaxEnginesPerComponent> slip{};
    std::array<bool, kMaxEnginesPerComponent> outs{};
    const auto in_band = [](double u, double h, double w) { return std::abs(u - h) <= 0.5 * w; };
    for (std::size_t e = 0; e < n_e; ++e) {
        const std::span<const double> xe = x.subspan(e * kEngineStates, kEngineStates);
        const std::span<unsigned char> pe = pinned.subspan(e * kEngineStates, kEngineStates);
        const BallinT700Signals s = signals(xe, inputs(ctx, static_cast<int>(e)));
        q[e] = s.q_engine;
        outs[e] = s.out;
        slip[e] = s.out ? std::numeric_limits<double>::infinity() : xe[kSlip];
        pe[kPs3Hys] = in_band(xe[kPs3Lag], xe[kPs3Hys], c.ps3hys / kPstd) ? 1 : 0;
        pe[kLdsHys] = in_band(s.xldsa, xe[kLdsHys], c.xldhys) ? 1 : 0;
        pe[kNgHys] = in_band(xe[kNg], xe[kNgHys], c.ch) ? 1 : 0;
        pe[kTmPos] = s.tm_current == 0.0 ||
                             pushes_on_limit(xe[kTmPos], c.tmgn * s.tm_current, c.xlolim, c.xhilim)
                         ? 1
                         : 0;
        pe[kTorqueInt] =
            pushes_on_limit(xe[kTorqueInt], xe[kTorqueSensed] - c.cr, c.ylolim, c.yhilim) ? 1 : 0;
        pe[kZ] = pushes_on_limit(xe[kZ], c.xkintg * s.spdss, s.zlolim, c.zhilim) ? 1 : 0;
        // The gas generator on the maps' floor, and an out engine's metal (D-056).
        const double ng_rate = (s.gas.q_gt - s.gas.q_c) / (c.j_gt * c.ng_des);
        const double floor = ng_floor(ctx.air.temperature);
        pe[kNg] = s.out || pushes_on_limit(xe[kNg], ng_rate, floor,
                                           std::numeric_limits<double>::infinity())
                      ? 1
                      : 0;
        pe[kMetal] = s.out ? 1 : 0;
    }
    const unsigned mask =
        engaged(std::span<const double>(q.data(), n_e), std::span<const double>(slip.data(), n_e),
                rotor_demand(ctx) + drive_train_load(shaft_speed(ctx)));
    for (std::size_t e = 0; e < n_e; ++e) {
        pinned[e * kEngineStates + kSlip] = (mask & (1U << e)) != 0U ? 1 : 0;
    }
    // An out engine is held but its torque sensor (D-056 item 3): its fuel
    // control and gas generator don't move, and its open clutch's slip only
    // follows the shaft; the sensor's lags decay to its zero torque.
    for (std::size_t e = 0; e < n_e; ++e) {
        if (outs[e]) {
            std::fill_n(pinned.begin() + static_cast<std::ptrdiff_t>(e * kEngineStates),
                        kEngineStates, static_cast<unsigned char>(1));
            pinned[e * kEngineStates + kTorqueLag] = 0;
            pinned[e * kEngineStates + kTorqueSensed] = 0;
        }
    }
}

}  // namespace eyas

// ---- Config ------------------------------------------------------------------

namespace eyas {

std::unique_ptr<BallinT700> BallinT700::from_config(const Config& node, int first_engine) {
    using units::Dimension;
    node.allow_keys({"type", "name", "shaft", "rotor", "engines", "gear", "gas_path", "output",
                     "gas_generator_spool", "maps", "ecu", "hmu", "drive_train", "clutch"});
    BallinT700Config c;
    c.name = node.string("name");
    if (c.name.empty()) {
        node.fail("name", "must not be empty");
    }
    const Config sh = node.child("shaft");
    sh.allow_keys({"name"});
    c.shaft = sh.string("name");
    c.rotor = node.string("rotor");
    const std::int64_t engines = node.integer("engines");
    if (engines < 1 || engines > static_cast<std::int64_t>(kMaxEnginesPerComponent)) {
        node.fail("engines", "must be 1 or 2 (Fig. C2's load share pairs two; D-055)");
    }
    c.engines = static_cast<int>(engines);
    c.first_engine = first_engine;
    c.gear = node.number("gear");

    const auto affine = [](const Config& n, std::string_view key, Dimension slope, Dimension offset,
                           double& a, double& b) {
        const Config v = n.child(key);
        v.allow_keys({"slope", "offset"});
        a = v.quantity("slope", slope);
        b = v.quantity("offset", offset);
    };
    const Config g = node.child("gas_path");
    g.allow_keys({"hvf",   "j_gt",   "k_bl",   "k_b3",  "k_damp", "k_dpb",  "k_h2",
                  "k_h3",  "k_h41",  "k_h45",  "k_ps3", "k_qc_1", "k_qc_2", "k_t41",
                  "k_t45", "k_th41", "k_th45", "k_wgt", "ng_des", "np_des", "tc_t41"});
    c.hvf = g.quantity("hvf", Dimension::specific_energy);
    c.j_gt = g.quantity("j_gt", Dimension::inertia);
    c.k_bl = g.number("k_bl");
    c.k_b3 = g.number("k_b3");
    c.k_damp = g.quantity("k_damp", Dimension::rotational_damping);
    c.k_dpb = g.quantity("k_dpb", Dimension::burner_resistance);
    c.k_h2 = g.quantity("k_h2", Dimension::specific_heat);
    affine(g, "k_h3", Dimension::specific_heat, Dimension::specific_energy, c.k_h3_1, c.k_h3_2);
    affine(g, "k_h41", Dimension::specific_heat, Dimension::specific_energy, c.k_h41_1, c.k_h41_2);
    c.k_h45 = g.number("k_h45");
    c.k_ps3 = g.number("k_ps3");
    c.k_qc_1 = g.number("k_qc_1");
    c.k_qc_2 = g.number("k_qc_2");
    affine(g, "k_t41", Dimension::temperature_per_specific_energy, Dimension::temperature,
           c.k_t41_1, c.k_t41_2);
    affine(g, "k_t45", Dimension::temperature_per_specific_energy, Dimension::temperature,
           c.k_t45_1, c.k_t45_2);
    affine(g, "k_th41", Dimension::per_temperature, Dimension::dimensionless, c.k_th41_1,
           c.k_th41_2);
    affine(g, "k_th45", Dimension::per_temperature, Dimension::dimensionless, c.k_th45_1,
           c.k_th45_2);
    c.k_wgt = g.quantity("k_wgt", Dimension::mass_flow_per_pressure);
    c.ng_des = g.quantity("ng_des", Dimension::angular_rate);
    c.np_des = g.quantity("np_des", Dimension::angular_rate);
    c.tc_t41 = g.quantity("tc_t41", Dimension::heat_sink_constant);

    const auto axis = [](const Config& n) {
        const Config a = n.child("axis");
        a.allow_keys({"x", "y", "z"});
        const Vec3 v(a.number("x"), a.number("y"), a.number("z"));
        if (!(v.norm() > 0.0) || !v.allFinite()) {
            n.fail("axis", "must be a nonzero, finite direction");
        }
        return Vec3(v.normalized());
    };
    const Config o = node.child("output");
    o.allow_keys({"inertia", "axis"});
    c.output_inertia = o.quantity("inertia", Dimension::inertia);
    c.pt_axis_b = axis(o);
    const Config gs = node.child("gas_generator_spool");
    gs.allow_keys({"axis"});
    c.gg_axis_b = axis(gs);

    const Config maps = node.child("maps");
    maps.allow_keys({"f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "f9", "f10", "f_hs", "fec1",
                     "fhm1", "fhm2", "fhm3", "fhm4", "fhm5", "fhm6", "fhm7"});
    const auto table = [&](std::string_view key, std::string_view x, Dimension xd,
                           std::string_view v, Dimension vd) {
        return maps.in_context(key, [&] {
            const CsvTable csv = maps.csv_table(key);
            csv.require_dimension(x, xd);
            csv.require_dimension(v, vd);
            return std::make_shared<const Table1D>(make_table1d(csv, x, v));
        });
    };
    const auto family = [&](std::string_view key, std::string_view p, Dimension pd,
                            std::string_view x, Dimension xd, std::string_view v, Dimension vd) {
        return maps.in_context(key, [&] {
            const CsvTable csv = maps.csv_table(key);
            csv.require_dimension(p, pd);
            csv.require_dimension(x, xd);
            csv.require_dimension(v, vd);
            const std::vector<double>& pc = csv.column(p);
            const std::vector<double>& xc = csv.column(x);
            const std::vector<double>& vc = csv.column(v);
            std::vector<double> params;
            std::vector<Table1D> curves;
            for (std::size_t i = 0; i < pc.size();) {
                std::size_t j = i;
                std::vector<double> xs;
                std::vector<double> vs;
                for (; j < pc.size() && pc[j] == pc[i]; ++j) {
                    xs.push_back(xc[j]);
                    vs.push_back(vc[j]);
                }
                params.push_back(pc[i]);
                curves.emplace_back(Axis(std::move(xs)), std::move(vs));
                i = j;
            }
            return std::make_shared<const CurveFamily>(std::move(params), std::move(curves));
        });
    };
    c.f1 = family("f1", "ngc", Dimension::dimensionless, "pr", Dimension::dimensionless, "wa2c",
                  Dimension::mass_flow);
    c.f2 = table("f2", "pr", Dimension::dimensionless, "t3_t2", Dimension::dimensionless);
    c.f3 = table("f3", "ngc", Dimension::dimensionless, "b1", Dimension::dimensionless);
    c.f4 = table("f4", "wa2c", Dimension::mass_flow, "b2", Dimension::dimensionless);
    c.f5 = table("f5", "wa2c", Dimension::mass_flow, "b3", Dimension::dimensionless);
    c.f6 = table("f6", "far", Dimension::dimensionless, "eta", Dimension::dimensionless);
    c.f7 = table("f7", "pr", Dimension::dimensionless, "dh_theta", Dimension::specific_energy);
    c.f8 = table("f8", "pr", Dimension::dimensionless, "dh_theta", Dimension::specific_energy);
    c.f9 = table("f9", "pr", Dimension::dimensionless, "w45c", Dimension::mass_flow_per_pressure);
    c.f10 = table("f10", "ngc", Dimension::dimensionless, "p49_ps9", Dimension::dimensionless);
    c.f_hs = table("f_hs", "ngc", Dimension::dimensionless, "t41sgn", Dimension::mass);
    c.fec1 = family("fec1", "t45l", Dimension::temperature, "w45r", Dimension::flow_parameter,
                    "tau45", Dimension::time);
    c.fhm1 =
        table("fhm1", "t2", Dimension::temperature, "wfptp", Dimension::mass_flow_per_pressure);
    c.fhm2 = table("fhm2", "pas", Dimension::angle, "wfprf", Dimension::mass_flow_per_pressure);
    c.fhm3 = table("fhm3", "xldsa", Dimension::angle, "png", Dimension::dimensionless);
    c.fhm4 = table("fhm4", "xldsa", Dimension::angle, "wfqps3", Dimension::mass_flow_per_pressure);
    c.fhm5 =
        table("fhm5", "t2", Dimension::temperature, "wfirf", Dimension::mass_flow_per_pressure);
    c.fhm6 = table("fhm6", "t2", Dimension::temperature, "pcngi", Dimension::dimensionless);
    c.fhm7 = family("fhm7", "t2", Dimension::temperature, "pcnghl", Dimension::dimensionless,
                    "wfpac", Dimension::mass_flow_per_pressure);

    // ECU (Figs. C1-C8, Table C.1). Speed errors are fractions, so a gain
    // printed per % NP reads "V/%".
    const Config e = node.child("ecu");
    e.allow_keys({"np_sensor_lag", "dead_band", "rate_lags", "gains", "relay", "torque_switch",
                  "torque_sensor_lags", "governor", "harness", "temperature", "integrator",
                  "load_share"});
    c.ctpl = e.quantity("np_sensor_lag", Dimension::time);
    c.cb = e.quantity("dead_band", Dimension::dimensionless);
    const Config rl = e.child("rate_lags");
    rl.allow_keys({"ct13", "ct14"});
    c.ct13 = rl.quantity("ct13", Dimension::time);
    c.ct14 = rl.quantity("ct14", Dimension::time);
    const Config gn = e.child("gains");
    gn.allow_keys({"zk1", "zk5", "zk7", "zk9", "zk10"});
    c.zk1 = gn.number("zk1");
    c.zk5 = gn.quantity("zk5", Dimension::voltage_time);
    c.zk7 = gn.number("zk7");
    c.zk9 = gn.quantity("zk9", Dimension::voltage);
    c.zk10 = gn.quantity("zk10", Dimension::voltage);
    const Config re = e.child("relay");
    re.allow_keys({"low", "high", "gain", "b6"});
    c.band_low = re.quantity("low", Dimension::dimensionless);
    c.band_high = re.quantity("high", Dimension::dimensionless);
    c.relay_gain = re.quantity("gain", Dimension::moment_time);
    c.b6 = re.number("b6");
    const Config ts = e.child("torque_switch");
    ts.allow_keys({"threshold", "lower", "upper", "switch_point"});
    c.cr = ts.quantity("threshold", Dimension::moment);
    c.ylolim = ts.quantity("lower", Dimension::moment_time);
    c.yhilim = ts.quantity("upper", Dimension::moment_time);
    c.corr = ts.quantity("switch_point", Dimension::moment_time);
    const Config tl = e.child("torque_sensor_lags");
    tl.allow_keys({"tl1", "tl2"});
    c.torque_lag1 = tl.quantity("tl1", Dimension::time);
    c.torque_lag2 = tl.quantity("tl2", Dimension::time);
    const Config gv = e.child("governor");
    gv.allow_keys({"t11", "ct2", "ct12"});
    c.gov_lead = gv.quantity("t11", Dimension::time);
    c.ct2 = gv.quantity("ct2", Dimension::time);
    c.ct12 = gv.quantity("ct12", Dimension::time);
    const Config ha = e.child("harness");
    ha.allow_keys({"t45cor", "tlge"});
    c.t45cor = ha.quantity("t45cor", Dimension::temperature);
    c.tlge = ha.quantity("tlge", Dimension::time);
    const Config te = e.child("temperature");
    te.allow_keys({"t45ref", "zk3", "ct9", "t8", "t10"});
    c.t45ref = te.quantity("t45ref", Dimension::temperature);
    c.zk3 = te.quantity("zk3", Dimension::voltage_per_temperature);
    c.ct9 = te.quantity("ct9", Dimension::time);
    c.t8 = te.quantity("t8", Dimension::time);
    c.t10 = te.quantity("t10", Dimension::time);
    const Config pi = e.child("integrator");
    pi.allow_keys({"xkprop", "xkintg", "zhilim", "zlolim", "ct16"});
    c.xkprop = pi.number("xkprop");
    c.xkintg = pi.quantity("xkintg", Dimension::frequency);
    c.zhilim = pi.quantity("zhilim", Dimension::voltage);
    const Config zl = pi.child("zlolim");
    zl.allow_keys({"below", "above", "torque"});
    c.zlolim_low = zl.quantity("below", Dimension::voltage);
    c.zlolim_high = zl.quantity("above", Dimension::voltage);
    c.zlolim_torque = zl.quantity("torque", Dimension::moment);
    c.ct16 = pi.quantity("ct16", Dimension::time);
    const Config ls = e.child("load_share");
    ls.allow_keys({"gain", "ct7", "lower", "upper", "zk8", "t17"});
    c.share_gain = ls.quantity("gain", Dimension::per_moment);
    c.ct7 = ls.quantity("ct7", Dimension::time);
    c.dbias = ls.quantity("lower", Dimension::dimensionless);
    c.ce = ls.quantity("upper", Dimension::dimensionless);
    c.zk8 = ls.number("zk8");
    c.t17 = ls.quantity("t17", Dimension::time);

    // HMU (Figs. C9-C22, Table C.1).
    const Config h = node.child("hmu");
    h.allow_keys({"torque_motor", "ps3_sensor", "load_demand", "droop", "ng_sensor",
                  "power_available", "deceleration", "valve", "hysteresis_lag"});
    const Config tm = h.child("torque_motor");
    tm.allow_keys({"bias", "lead", "lag", "gain", "offset", "dead_band", "sensitivity", "stops",
                   "feedback", "linkage"});
    c.tm_bias = tm.quantity("bias", Dimension::voltage);
    c.tm_lead = tm.quantity("lead", Dimension::time);
    c.tm_lag = tm.quantity("lag", Dimension::time);
    c.tm_gain = tm.quantity("gain", Dimension::current_per_voltage);
    c.tm_offset = tm.quantity("offset", Dimension::current);
    c.tmdb = tm.quantity("dead_band", Dimension::current);
    c.tmgn = tm.quantity("sensitivity", Dimension::length_per_current_time);
    const Config st = tm.child("stops");
    st.allow_keys({"lower", "upper"});
    c.xlolim = st.quantity("lower", Dimension::length);
    c.xhilim = st.quantity("upper", Dimension::length);
    c.tmlvg = tm.quantity("feedback", Dimension::voltage_per_length);
    c.tmlg = tm.quantity("linkage", Dimension::mass_flow_per_pressure_length);
    const Config ps = h.child("ps3_sensor");
    ps.allow_keys({"lag", "hysteresis"});
    c.ctps3 = ps.quantity("lag", Dimension::time);
    c.ps3hys = ps.quantity("hysteresis", Dimension::pressure);
    const Config ld = h.child("load_demand");
    ld.allow_keys({"rigging_gain", "rigging_bias", "hysteresis", "fuel_bias", "lag"});
    c.rig_gain = ld.quantity("rigging_gain", Dimension::angle);
    c.rig_bias = ld.quantity("rigging_bias", Dimension::angle);
    c.xldhys = ld.quantity("hysteresis", Dimension::angle);
    c.lds_fuel_bias = ld.quantity("fuel_bias", Dimension::mass_flow_per_pressure);
    c.clld = ld.quantity("lag", Dimension::time);
    const Config dr = h.child("droop");
    dr.allow_keys({"slope", "reference"});
    c.kndrp = dr.quantity("slope", Dimension::mass_flow_per_pressure);
    c.ngref = dr.quantity("reference", Dimension::dimensionless);
    const Config ns = h.child("ng_sensor");
    ns.allow_keys({"hysteresis", "lag"});
    c.ch = ns.quantity("hysteresis", Dimension::dimensionless);
    c.cntl = ns.quantity("lag", Dimension::time);
    const Config pa = h.child("power_available");
    pa.allow_keys({"idle", "fly"});
    c.pas_idle = pa.quantity("idle", Dimension::angle);
    c.pas_fly = pa.quantity("fly", Dimension::angle);
    const Config de = h.child("deceleration");
    de.allow_keys({"gain", "bias", "lower", "upper"});
    c.awfp = de.quantity("gain", Dimension::mass_flow_per_pressure);
    c.bwfp = de.quantity("bias", Dimension::mass_flow_per_pressure);
    c.wfpdcl = de.quantity("lower", Dimension::mass_flow_per_pressure);
    c.wfpdch = de.quantity("upper", Dimension::mass_flow_per_pressure);
    const Config va = h.child("valve");
    va.allow_keys({"min", "max", "lag", "delay"});
    c.wfmin = va.quantity("min", Dimension::mass_flow);
    c.wfmax = va.quantity("max", Dimension::mass_flow);
    c.clmv = va.quantity("lag", Dimension::time);
    c.delay = va.quantity("delay", Dimension::time);
    c.hysteresis_lag = h.quantity("hysteresis_lag", Dimension::time);

    const Config dt = node.child("drive_train");
    dt.allow_keys({"gearbox_damping", "accessory_power"});
    c.gearbox_damping = dt.quantity("gearbox_damping", Dimension::rotational_damping);
    c.accessory_power = dt.quantity("accessory_power", Dimension::power);
    const Config cl = node.child("clutch");
    cl.allow_keys({"drive_inertia"});
    c.drive_inertia = cl.quantity("drive_inertia", Dimension::inertia);

    return node.in_context("", [&] { return std::make_unique<BallinT700>(std::move(c)); });
}

}  // namespace eyas
