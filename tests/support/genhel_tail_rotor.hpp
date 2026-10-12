#pragma once

// Exact-equation reference for the M6a tail rotor (D-028, layer 1).
//
// A literal transcription of GenHel CR-166309 §5.4.2 (pp. 5.4-4 to 5.4-8),
// in GenHel's notation, units and conventions: inches and feet, lbf, slug/ft³,
// degrees; stations FS positive aft, WL positive up, BL positive to port
// (DATA_GAPS.md, "B_LTR sign"); velocities are the aircraft's relative to the
// air, in body axes. Nothing is shared with the production code.
//
// GenHel evaluates the inflow and the δ₃ pitch with values from the previous
// pass (DWSHTR uses λ_TR(t−1), θ_TR uses T_TR(t−1)). Its steady state is the
// fixed point of those passes; this reference iterates them to convergence,
// which is what production's single-state model must reproduce when its
// inflow state sits at equilibrium (D-028).
//
// Two deliberate differences from the page: degrees are converted exactly
// (GenHel writes 1/57.3, a 7e-5 rounding, not part of the model), and the
// drag X_TR = −C_D ½ρ V_XTR² is only compared for V_XTR > 0 (production uses
// V|V| so the drag reverses in rearward flight).

#include <cmath>

namespace eyas::test {

struct GenHelTailRotorInput {
    // Geometry [in] (p. 5.4-4).
    double FSTR = 0.0, WLTR = 0.0, BLTR = 0.0;
    double FSCGB = 0.0, WLCGB = 0.0, BLCGB = 0.0;
    // Rotor (p. 5.4-14 names).
    double RTR = 0.0;     // [ft]
    double OMEGTR = 0.0;  // [rad/s]
    double BTR = 0.0;     // blades
    double CHRDTR = 0.0;  // [ft]
    double ATR = 0.0;     // [1/rad]
    double BTLTR = 1.0;   // tip loss B
    double TWSTTR = 0.0;  // [deg/R]
    double GAMTR = 0.0;   // cant Γ [deg]
    double DEL3TR = 0.0;  // [deg]
    double DELTTR = 0.0;  // ∂a₀/∂T [deg/lbf]
    double BIASTR = 0.0;  // [deg]
    double CDTR = 0.0;    // [ft²]
    double KBVTTR = 1.0;  // blockage factor
    double VBVTTR = 0.0;  // [kt]
    double RHO = 0.0;     // [slug/ft³]
    // Flight state.
    double VXB = 0.0, VYB = 0.0, VZB = 0.0;  // [ft/s]
    double P = 0.0, Q = 0.0, R = 0.0;        // [rad/s]
    double VKT = 0.0;                        // flight-path airspeed [kt]
    double THETTR = 0.0;                     // commanded pitch [deg]
    // Total interference velocities (p. 5.4-6), [ft/s].
    double VXITR = 0.0, VYITR = 0.0, VZITR = 0.0;
};

struct GenHelTailRotorOutput {
    double XTR = 0.0, YTR = 0.0, ZTR = 0.0;  // [lbf]
    double LTR = 0.0, MTR = 0.0, NTR = 0.0;  // [ft·lbf]
    double TTR = 0.0;                        // [lbf]
    double DWSHTR = 0.0;                     // uniform downwash [-]
    double THETA = 0.0;                      // actual blade pitch [rad]
    double MUZTR = 0.0, MUTR = 0.0;
    bool converged = false;
};

inline GenHelTailRotorOutput genhel_tail_rotor(const GenHelTailRotorInput& in) {
    const double d2r = 1.0 / 57.29577951308232;
    GenHelTailRotorOutput o;
    // Geometry, p. 5.4-4.
    const double FTR = (in.FSTR - in.FSCGB) / 12.0;
    const double WTR = (in.WLTR - in.WLCGB) / 12.0;
    const double BTR_ = (in.BLTR - in.BLCGB) / 12.0;
    // Tail rotor velocities in the body axes, p. 5.4-6 (gust 0, K_QTR = 1).
    const double KQTR = 1.0;
    const double VXTRB = in.VXB * KQTR - in.Q * WTR + in.R * BTR_ + in.VXITR;
    const double VYTRB = in.VYB * KQTR - in.R * FTR + in.P * WTR + in.VYITR;
    const double VZTRB = in.VZB * KQTR + in.Q * FTR - in.P * BTR_ + in.VZITR;
    // Shaft axes.
    const double G_ = in.GAMTR * d2r;
    const double VXTR = VXTRB;
    const double VYTR = VYTRB * std::cos(G_) + VZTRB * std::sin(G_);
    const double VZTR = -VYTRB * std::sin(G_) + VZTRB * std::cos(G_);
    const double OR = in.OMEGTR * in.RTR;
    const double MUXTR = VXTR / OR;
    const double MUYTR = VYTR / OR;
    const double MUZTR = VZTR / OR;
    const double MUTR = std::sqrt(MUXTR * MUXTR + MUYTR * MUYTR);
    // Bailey coefficients, p. 5.4-7.
    const double B = in.BTLTR;
    const double t31 = B * B / 2.0 + MUTR * MUTR / 4.0;
    const double t32 = B * B * B / 3.0 + B / 2.0 * MUTR * MUTR;
    const double t33 = B * B * B * B / 4.0 + B * B / 4.0 * MUTR * MUTR;
    const double G = in.ATR / 2.0 * (in.BTR * in.CHRDTR / (3.141592653589793 * in.RTR));
    // Blockage, p. 5.4-8.
    double KBLKTR = 1.0;
    if (in.VKT < in.VBVTTR) {
        KBLKTR = (1.0 - in.KBVTTR) * in.VKT * in.VKT / (in.VBVTTR * in.VBVTTR) + in.KBVTTR;
    }
    // Passes with the previous-pass λ_TR and T_TR, to the fixed point.
    double lambda_prev = MUZTR;  // λ_TR(t−1), starting from zero downwash
    double T_prev = 0.0;         // T_TR(t−1)
    double DWSHTR = 0.0;
    double THETA = 0.0;
    double TTR = 0.0;
    for (int pass = 0; pass < 100000; ++pass) {
        THETA = d2r * (in.THETTR - T_prev * in.DELTTR * std::tan(in.DEL3TR * d2r) + in.BIASTR);
        DWSHTR = G * (MUZTR * t31 + THETA * t32 + in.TWSTTR * d2r * t33) /
                 (2.0 * std::sqrt(MUTR * MUTR + lambda_prev * lambda_prev) + G * t31);
        const double lambda = MUZTR - DWSHTR;
        TTR = 2.0 * in.RHO * 3.141592653589793 * std::pow(in.RTR, 4) * DWSHTR *
              std::sqrt(MUTR * MUTR + lambda * lambda) * in.OMEGTR * in.OMEGTR * KBLKTR;
        const bool done = std::abs(lambda - lambda_prev) < 1e-15 &&
                          std::abs(TTR - T_prev) <= 1e-13 * std::abs(TTR);
        lambda_prev = lambda;
        T_prev = TTR;
        if (done) {
            o.converged = true;
            break;
        }
    }
    // Forces and moments at the CG, body axes, p. 5.4-8.
    o.XTR = -in.CDTR * 0.5 * in.RHO * VXTR * VXTR;
    o.YTR = TTR * std::sin(G_);
    o.ZTR = -TTR * std::cos(G_);
    o.LTR = o.YTR * WTR - o.ZTR * BTR_;
    o.MTR = o.ZTR * FTR - o.XTR * WTR;
    o.NTR = o.XTR * BTR_ - o.YTR * FTR;
    o.TTR = TTR;
    o.DWSHTR = DWSHTR;
    o.THETA = THETA;
    o.MUZTR = MUZTR;
    o.MUTR = MUTR;
    return o;
}

}  // namespace eyas::test
