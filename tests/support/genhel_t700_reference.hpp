#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <span>

// GenHel's T700 engine and fuel control (NASA CR-166309 §5.6, pp. 5.6-9 to
// 5.6-18) written out test-side as printed: GenHel's own units (%, rpm,
// lbm/hr, psi, ft·lbf, V, in, mA), every quantity an increment from the trim
// it is anchored at, and the time-varying coefficients looked up at the
// current N_GG (p. 5.6-11, "update all time varying coefficients"). An
// exact-equation reference for the genhel_t700 component (D-053): no Eyas
// code, only the equation pages, the constants (p. 5.6-24) and Tables 6.5.1 to
// 6.5.6 as transcribed in data/uh60a/genhel/, with these readings, each
// marked where it is used:
// - GenHel's previous-pass values (t − 1) are the current ones (continuous
//   time), its maps linear between their rows and held beyond them;
// - V_Q ≡ 0 (human-reviewed) and Q_INT = (0.7/s)(K_4 Q1 − 20) (D-053 item 2);
//   Q_INT unlimited, B its value clipped to [0, 40], as printed (p. 5.6-12);
// - the dead band, switch logic and limited V_SS as printed (pp. 5.6-11 to 13).
// The rotor's speed comes in as an input; GenHel's own rotor degree of
// freedom (p. 5.6-17) is in rotor_acceleration() for closed-loop runs.
namespace eyas::test::t700 {

// Piecewise-linear lookup, held beyond the ends.
template <std::size_t N>
double interp(const std::array<double, N>& xs, const std::array<double, N>& ys, double x) {
    if (x <= xs.front()) {
        return ys.front();
    }
    if (x >= xs.back()) {
        return ys.back();
    }
    std::size_t i = 1;
    while (xs[i] < x) {
        ++i;
    }
    const double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[i - 1] + t * (ys[i] - ys[i - 1]);
}

// Tables 6.5.1 to 6.5.6 (pp. 5.6-25 to 5.6-28) and the constants (p. 5.6-24),
// as printed. A test may replace the tables (e.g. with self-consistent ones).
struct Data {
    // 6.5.1: (Q_REQ)_TRIM / N_E [ft·lbf] → (N_GG)_TRIM [%]
    std::array<double, 7> q_trim = {0, 6000, 12000, 18000, 24000, 30000, 36000};
    std::array<double, 7> ngg_trim = {70, 84.6, 89.9, 94.0, 97.2, 100.1, 102.6};
    // 6.5.2: X_C [%] → LDS [deg]
    std::array<double, 11> xc = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    std::array<double, 11> lds = {0, 8.5, 17.0, 25.5, 34.0, 42.6, 51.0, 59.5, 68.0, 76.5, 85.0};
    // 6.5.3: LDS [deg] → N_GG,LDS [%]
    std::array<double, 6> cam_lds = {0, 20, 40, 60, 80, 100};
    std::array<double, 6> cam_ngg = {78.6, 85.0, 91.3, 97.8, 100.0, 100.0};
    // 6.5.4: N_GG [%] → steady fuel per engine [lbm/hr]
    std::array<double, 8> fuel_ngg = {65, 70, 75, 80, 85, 90, 95, 100};
    std::array<double, 8> fuel = {120, 140, 166, 210, 280, 390, 575, 790};
    // 6.5.5 and 6.5.6 on N_GG = 70, 75, ..., 100 [%]
    std::array<double, 7> ngg = {70, 75, 80, 85, 90, 95, 100};
    std::array<double, 7> p3_ss = {69, 78, 94, 118, 149, 184, 226};               // [psi]
    std::array<double, 7> dp3_dwf = {.051, .071, .078, .072, .089, .080, .075};   // psi/(lbm/hr)
    std::array<double, 7> dp3_dngg = {5.10, 4.30, 3.97, 4.20, 4.73, 4.97, 1.71};  // psi/%
    std::array<double, 7> dwf_dp3 = {2.60, 2.50, 2.40, 2.32, 2.49, 2.75, 3.20};   // (lbm/hr)/psi
    std::array<double, 7> dqgg_dwf = {.29, .285, .283, .273, .261, .200, .173};  // ft·lbf/(lbm/hr)
    std::array<double, 7> dqgg_dngg = {-1.10, -2.1, -3.1, -4.3, -5.65, -7.20, -4.62};  // ft·lbf/%
    std::array<double, 7> dqpt_dwf = {.24, .325, .358, .363, .387, .380, .331};  // ft·lbf/(lbm/hr)
    std::array<double, 7> dqpt_dnpt = {.0045, .0065, .0086, .0094, .0107, .0132, .0151};  // /rpm
    std::array<double, 7> dqpt_dngg = {0, 4.8, 7.0, 8.3, 9.9, 11.8, 5.5};  // ft·lbf/%

    double KQ = 0.1, NE = 2.0, GE = 76.0216, NPT100 = 20000.0;
    double K1 = 3.0, K3 = 0.6, K4 = 0.059, K6 = 0.2, K9 = 0.08, K10 = 0.020408, K11 = 0.1;
    double a = 0.4, b = 0.8, t1 = 0.55, t2 = 0.107, t3 = 0.08, t4 = 1.58, AU = 0.0, AL = -3.5;
    double G4 = 564.0, G5 = 0.0159, G6 = 56.0, G7 = 336.0, T2 = 0.2, T3 = 0.04;
    double TNGG = 0.02, DWP3_DNGG = 0.25, TP3 = 0.01, KCF_JGG = 0.479, KHS = 0.65, TMV = 0.004;
    double JTOTE = 690.6, JREST = 943.9;  // [slug·ft²]
};

// The trim GenHel anchors at (pp. 5.6-9, 5.6-10).
struct Trim {
    double q_req = 0.0;     // (Q̄_REQ)_TRIM, total at the rotor [ft·lbf]
    double ngg = 0.0;       // (N_GG)_TRIM [%]
    double ngg_lds = 0.0;   // (N_GGLDS)_TRIM [%]
    double lds_e = 0.0;     // (L_DSE)_TRIM [%]
    double omega_s0 = 0.0;  // Ω_S0 [rad/s]
    double npt_ref = 0.0;   // (N_PTP)_REF [%]
};

// The incremental states, GenHel's units.
inline constexpr std::size_t kDngg = 0;  // ΔN_GG [%]
inline constexpr std::size_t kDwf = 1;   // ΔW_f′ [lbm/hr], metered
inline constexpr std::size_t kP3Fb =
    2;  // the lagged P3 feedback, {[ΔW_f/ΔP3] ΔP3}/(T_P3 s + 1) [lbm/hr]
inline constexpr std::size_t kDnggMeas = 3;  // ΔN_GG/(T_NGG s + 1) [%]
inline constexpr std::size_t kIdb = 4;       // I_DB1 [V]
inline constexpr std::size_t kIdbRate = 5;   // [V/s]
inline constexpr std::size_t kQint = 6;      // Q_INT [V]
inline constexpr std::size_t kVsZ = 7;       // V_S lead-lag state [V]
inline constexpr std::size_t kVss = 8;       // V_SS [V]
inline constexpr std::size_t kVdmdZ = 9;     // V_DMD lead-lag state [V]
inline constexpr std::size_t kMZ = 10;       // M lead-lag state [V]
inline constexpr std::size_t kIns = 11;      // I_NS [in]
inline constexpr std::size_t kNumStates = 12;

struct Signals {
    double ngg = 0.0;       // N_GG [%]
    double dqpt = 0.0;      // ΔQ_PT per engine [ft·lbf]
    double qe = 0.0;        // Q_E total at the rotor [ft·lbf] (p. 5.6-16)
    double ecu = 0.0;       // ECU(s) [%]
    double nptdp = 0.0;     // N_PTDP [%]
    double vg = 0.0;        // V_G [V]
    bool open = false;      // switch open (high gain)
    double dngg_ref = 0.0;  // ΔN_GG,REF [%]
};

struct Reference {
    Data d;
    Trim trim;

    double lds_ngg(double xc_pct) const {
        return interp(d.cam_lds, d.cam_ngg, interp(d.xc, d.lds, xc_pct));
    }

    // Trimming mode (pp. 5.6-9, 5.6-10) at the rotor's torque Q_HMR [ft·lbf],
    // collective X_C [%] and shaft speed Ω_S0 [rad/s]; returns the states at
    // trim (all increments zero, Q_INT at its trim value).
    std::array<double, kNumStates> anchor(double q_hmr, double xc_pct, double omega_s0) {
        trim.q_req = q_hmr * (1.0 + d.KQ);
        trim.ngg = interp(d.q_trim, d.ngg_trim, trim.q_req / d.NE);
        trim.ngg_lds = lds_ngg(xc_pct);
        trim.lds_e = trim.ngg_lds - trim.ngg;
        trim.omega_s0 = omega_s0;
        trim.npt_ref = omega_s0 * 60.0 * d.GE / (2.0 * std::numbers::pi) * 100.0 / d.NPT100;
        std::array<double, kNumStates> x{};
        // "Q_INT must be initialised at trim value of Q_INT, i.e. ΔQ_PT = 0"
        // (p. 5.6-12): its input's sign decides the limit it sits at.
        x[kQint] = d.K4 * trim.q_req / (d.NE * d.GE) - 20.0 > 0.0 ? 40.0 : 0.0;
        return x;
    }

    Signals signals(std::span<const double> x, double omega, double xc_pct) const {
        Signals s;
        s.ngg = trim.ngg + x[kDngg];
        const double n = s.ngg;
        // Power turbine speed error (p. 5.6-11).
        const double npt_pct = omega * 60.0 * d.GE / (2.0 * std::numbers::pi) * 100.0 / d.NPT100;
        s.nptdp = trim.npt_ref - npt_pct;
        const double dnpt = s.nptdp * d.NPT100 / 100.0;  // ΔN_PT [rpm] (p. 5.6-16)
        s.dqpt = interp(d.ngg, d.dqpt_dwf, n) * x[kDwf] + interp(d.ngg, d.dqpt_dnpt, n) * dnpt +
                 interp(d.ngg, d.dqpt_dngg, n) * x[kDngg];
        s.qe = d.NE * d.GE * s.dqpt + trim.q_req;
        // ECU (pp. 5.6-11 to 5.6-13).
        const double idb = x[kIdb];
        double vhg = 0.0;
        if (idb <= -d.a) {
            vhg = idb + d.a;
        } else if (idb >= d.b) {
            vhg = idb - d.b;
        }
        const double vlg = d.K3 * s.nptdp;
        const double A = (s.nptdp >= -4.0 && s.nptdp <= 1.0) ? 0.0 : 40.0;
        const double B = std::clamp(x[kQint], 0.0, 40.0);
        s.open = A >= 20.0 || B >= 20.0;
        s.vg = s.open ? vhg : vlg;  // V_Q ≡ 0
        s.ecu = d.G7 * x[kIns];
        const double ldsei = lds_ngg(xc_pct) - trim.ngg - trim.lds_e;
        s.dngg_ref = s.ecu + ldsei;
        return s;
    }

    // ẋ at x with the shaft at `omega` [rad/s] and collective X_C [%].
    std::array<double, kNumStates> rates(std::span<const double> x, double omega,
                                         double xc_pct) const {
        const Signals s = signals(x, omega, xc_pct);
        const double n = s.ngg;
        std::array<double, kNumStates> r{};
        // ECU.
        r[kIdb] = x[kIdbRate];
        r[kIdbRate] = (d.K1 * s.nptdp - x[kIdb] - d.K11 * x[kIdbRate]) / d.K10;
        r[kQint] = 0.7 * (d.K4 * (s.dqpt + trim.q_req / (d.NE * d.GE)) - 20.0);  // D-053
        r[kVsZ] = (s.vg - x[kVsZ]) / d.t2;
        const double vs = x[kVsZ] + d.t1 / d.t2 * (s.vg - x[kVsZ]);
        double vss_rate = d.K9 * vs;
        if ((x[kVss] >= d.AU && vss_rate > 0.0) || (x[kVss] <= d.AL && vss_rate < 0.0)) {
            vss_rate = 0.0;
        }
        r[kVss] = vss_rate;
        // "IF V_SS > A_U SET V_SS = A_U" (p. 5.6-13): the value used is clipped.
        const double vdm = std::clamp(x[kVss], d.AL, d.AU) + d.K6 * vs;
        r[kVdmdZ] = (vdm - x[kVdmdZ]) / d.t4;
        const double vdmd = x[kVdmdZ] + d.t3 / d.t4 * (vdm - x[kVdmdZ]);
        const double m_in = (vdmd - d.G6 * x[kIns]) * d.G4;
        r[kMZ] = (m_in - x[kMZ]) / d.T2;
        const double m = x[kMZ] + d.T3 / d.T2 * (m_in - x[kMZ]);
        r[kIns] = d.G5 * m;
        // Gas generator speed error and fuel demand (p. 5.6-14).
        r[kDnggMeas] = (x[kDngg] - x[kDnggMeas]) / d.TNGG;
        const double dngge = s.dngg_ref - x[kDnggMeas];
        const double dwf = interp(d.ngg, d.p3_ss, n) * d.DWP3_DNGG * dngge;
        // Compressor pressure, fuel feedback and metering (p. 5.6-15).
        const double dp3 =
            interp(d.ngg, d.dp3_dwf, n) * x[kDwf] + interp(d.ngg, d.dp3_dngg, n) * x[kDngg];
        r[kP3Fb] = (interp(d.ngg, d.dwf_dp3, n) * dp3 - x[kP3Fb]) / d.TP3;
        const double dwf_eff = dwf + x[kP3Fb];
        r[kDwf] = (dwf_eff - x[kDwf]) / d.TMV;
        // Gas generator (pp. 5.6-15, 5.6-16).
        const double dqgg =
            interp(d.ngg, d.dqgg_dwf, n) * x[kDwf] + interp(d.ngg, d.dqgg_dngg, n) * x[kDngg];
        r[kDngg] = d.KHS * d.KCF_JGG * dqgg;
        return r;
    }

    // GenHel's rotor speed degree of freedom, clutch engaged (p. 5.6-17):
    // Ω̇_S = (Q_E − Q_REQ)/(J_TOTE + J_REST) [rad/s²], torques in ft·lbf.
    double rotor_acceleration(std::span<const double> x, double omega, double xc_pct,
                              double q_hmr) const {
        const double qe = signals(x, omega, xc_pct).qe;
        return (qe - q_hmr * (1.0 + d.KQ)) / (d.JTOTE + d.JREST);
    }
};

}  // namespace eyas::test::t700
