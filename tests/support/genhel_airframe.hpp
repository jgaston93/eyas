#pragma once

// Exact-equation reference for the M6b airframe aerodynamics (D-028, layer 1).
//
// A literal transcription of GenHel CR-166309 §5.2.2 (fuselage, pp. 5.2-5 to
// 5.2-9) and §5.3.2 (empennage, pp. 5.3-5 to 5.3-12), in GenHel's notation,
// units and conventions: inches and feet, lbf, slug/ft³, degrees; stations FS
// positive aft, WL positive up, BL positive to port (DATA_GAPS.md, "B_LTR
// sign"); velocities are the aircraft's relative to the air, in body axes.
//
// GenHel's tables are read from the as-printed transcriptions in
// data/uh60a/genhel/ and looked up the way GenHel's map routines do, low and
// high maps kept separate. Production reads the merged CSVs that
// tools/build_uh60a_airframe.py writes, so the comparison also checks the
// merge. Nothing else is shared with the production code.
//
// Departures from the page, each deliberate:
// - Degrees are converted exactly (GenHel writes 57.3).
// - The fin's main-rotor downwash takes the minus sign the stabilator and
//   tail rotor have; the fin page prints none (p. 5.3-9; DATA_GAPS.md M6
//   item 1, D-028 "Data rulings").
// - DLQD, DMQD, DNQD (p. 5.2-7) are zero (DATA_GAPS.md M6 item 4).
// - A single map (UVR) holds its end value outside its range; GenHel doesn't
//   say what its routine does there (DATA_GAPS.md).
// - GenHel delays the fuselage downwash at the stabilator and the sidewash at
//   the fin. This reference takes the delayed values as inputs; NaN means
//   "steady state", i.e. the current value (D-028).
// - Only V_X > 0 at every surface is compared: GenHel builds its wind axes
//   from atan(V_Z/|V_X|), which points the drag forward in rearward flight
//   (D-030).
//
// The low-speed filter (p. 5.2-10) is not transcribed: "not being executed on
// PDP10", and its parameters aren't given (D-028).

#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace eyas::test {

// ---- GenHel's maps ------------------------------------------------------------

// The numbers of one transcription file, by ';'-labelled block.
inline std::map<std::string, std::vector<double>> read_genhel_blocks(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error("cannot open " + path);
    }
    std::map<std::string, std::vector<double>> out;
    std::string line;
    std::string label;
    while (std::getline(f, line)) {
        const auto first = line.find_first_not_of(" \t");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        if (line[first] == ';') {
            label = line.substr(line.find_first_not_of(" \t", first + 1));
            out[label] = {};
            continue;
        }
        std::istringstream s(line);
        double v = 0.0;
        while (s >> v) {
            out[label].push_back(v);
        }
    }
    return out;
}

// Linear interpolation on an evenly spaced map, holding the end values.
inline double genhel_interp(const std::vector<double>& y, double x0, double dx, double x) {
    const double s = (x - x0) / dx;
    if (s <= 0.0) {
        return y.front();
    }
    const auto n = static_cast<double>(y.size() - 1);
    if (s >= n) {
        return y.back();
    }
    const auto i = static_cast<std::size_t>(s);
    const double f = s - static_cast<double>(i);
    return y[i] + f * (y[i + 1] - y[i]);
}

// UVRUVR: low map on [lo0, lo0 + n·dlo] inside a high map.
struct GenHelLowHighMap {
    std::vector<double> low, high;
    double lo0 = 0.0, dlo = 0.0, hi0 = 0.0, dhi = 0.0;
    double operator()(double x) const {
        const double lo1 = lo0 + dlo * static_cast<double>(low.size() - 1);
        if (high.empty() || (x >= lo0 && x <= lo1)) {
            return genhel_interp(low, lo0, dlo, x);
        }
        return genhel_interp(high, hi0, dhi, x);
    }
};

// BIV: E(χ, a₁) on χ = 0..100 step 10 and a₁ = −6, 0, 6, bilinear.
struct GenHelWashMap {
    std::array<std::vector<double>, 3> block;
    double operator()(double chi, double a1) const {
        const double s = std::fmin(std::fmax((a1 + 6.0) / 6.0, 0.0), 2.0);
        const std::size_t j = s >= 2.0 ? 1 : static_cast<std::size_t>(s);
        const double f = s - static_cast<double>(j);
        const double e0 = genhel_interp(block[j], 0.0, 10.0, chi);
        const double e1 = genhel_interp(block[j + 1], 0.0, 10.0, chi);
        return e0 + f * (e1 - e0);
    }
};

struct GenHelAirframeTables {
    GenHelWashMap EKXWF, EKZWF, EKXH1, EKZH1;  // Tables 2.5.1, 2.5.2, 3.5.1, 3.5.2
    GenHelLowHighMap DQF, DDQF, YQF, LQF, DLQF, RQF, MQF, DMQF, NQF;  // 2.5.3–2.5.11
    GenHelLowHighMap QH1, EPH1, CLH1, CDH1, QV1, SGV1, CLV1, CDV1;    // 3.5.3–3.5.10
};

inline GenHelAirframeTables read_genhel_airframe_tables(const std::string& dir) {
    const auto wash = [&](const std::string& file) {
        auto b = read_genhel_blocks(dir + "/" + file);
        GenHelWashMap m;
        m.block[0] = b.at("AA1FMR = -6");
        m.block[1] = b.at("AA1FMR = 0");
        m.block[2] = b.at("AA1FMR = 6");
        return m;
    };
    // lo0, dlo, hi0, dhi as printed on each table's EXP limit lines.
    const auto map = [&](const std::string& file, double lo0, double dlo, double hi0 = 0.0,
                         double dhi = 0.0) {
        auto b = read_genhel_blocks(dir + "/" + file);
        GenHelLowHighMap m;
        m.low = b.at("low");
        if (b.contains("high")) {
            m.high = b.at("high");
        }
        m.lo0 = lo0;
        m.dlo = dlo;
        m.hi0 = hi0;
        m.dhi = dhi;
        return m;
    };
    GenHelAirframeTables t;
    t.EKXWF = wash("table_2_5_1_ekx_fuselage.txt");
    t.EKZWF = wash("table_2_5_2_ekz_fuselage.txt");
    t.EKXH1 = wash("table_3_5_1_ekx_tail.txt");
    t.EKZH1 = wash("table_3_5_2_ekz_tail.txt");
    t.DQF = map("table_2_5_3_dqf.txt", -30, 5, -90, 10);
    t.DDQF = map("table_2_5_4_ddqf.txt", 0, 5, 30, 10);
    t.YQF = map("table_2_5_5_yqf.txt", 0, 5, 30, 10);
    t.LQF = map("table_2_5_6_lqf.txt", -30, 5, -90, 10);
    t.DLQF = map("table_2_5_7_dlqf.txt", -30, 5);
    t.RQF = map("table_2_5_8_rqf.txt", 0, 5, 30, 10);
    t.MQF = map("table_2_5_9_mqf.txt", -30, 5, -90, 10);
    t.DMQF = map("table_2_5_10_dmqf.txt", 0, 5);
    t.NQF = map("table_2_5_11_nqf.txt", -30, 5, -90, 10);
    t.QH1 = map("table_3_5_3_qh1.txt", -30, 5);
    t.EPH1 = map("table_3_5_4_eph1.txt", -30, 5, -90, 10);
    t.CLH1 = map("table_3_5_5_clh1.txt", 0, 5, 30, 10);
    t.CDH1 = map("table_3_5_6_cdh1.txt", 0, 5, 30, 10);
    t.QV1 = map("table_3_5_7_qv1.txt", 0, 5);
    t.SGV1 = map("table_3_5_8_sgv1.txt", 0, 5, 0, 30);
    t.CLV1 = map("table_3_5_9_clv1.txt", -30, 5, -90, 10);
    t.CDV1 = map("table_3_5_10_cdv1.txt", -30, 5, -90, 10);
    return t;
}

// ---- The airframe -------------------------------------------------------------

struct GenHelAirframeInput {
    // Geometry [in] (pp. 5.2-15, 5.3-19); areas [ft²]; incidences [deg].
    double FSCGB = 0.0, WLCGB = 0.0, BLCGB = 0.0;
    double FSWF = 0.0, WLWF = 0.0, BLWF = 0.0;
    double FSH1 = 0.0, WLH1 = 0.0, BLH1 = 0.0, SAH1 = 0.0, IH1 = 0.0;
    double FSV1 = 0.0, WLV1 = 0.0, BLV1 = 0.0, SAV1 = 0.0, IV1 = 0.0;
    double RHO = 0.0;  // [slug/ft³]
    // Flight state, relative to the air (no gust).
    double VXB = 0.0, VYB = 0.0, VZB = 0.0;  // [ft/s]
    double P = 0.0, Q = 0.0, R = 0.0;        // [rad/s]
    // Main rotor (p. 5.2-11): uniform downwash, Ω, R, wake skew and flapping [deg].
    double DWSHMR = 0.0, OMGTMR = 0.0, RMR = 0.0, CHIPMR = 0.0, A1FMR = 0.0;
    // Delayed fuselage downwash at the stabilator and sidewash at the fin
    // [deg]; NaN for the steady state (the current value).
    double EPSH1_DELAYED = std::numeric_limits<double>::quiet_NaN();
    double SIGV1_DELAYED = std::numeric_limits<double>::quiet_NaN();
};

struct GenHelLoads {
    double X = 0.0, Y = 0.0, Z = 0.0;  // [lbf]
    double L = 0.0, M = 0.0, N = 0.0;  // [ft·lbf]
};

struct GenHelAirframeOutput {
    GenHelLoads WF, H1, V1;
    double ALFWF = 0.0, BETAWF = 0.0, PSIWF = 0.0, QWF = 0.0;  // [deg], [lbf/ft²]
    double ALFH1 = 0.0, ALFHH1 = 0.0, BETAH1 = 0.0, QH1 = 0.0;
    double ALFV1 = 0.0, BETAV1 = 0.0, BETVV1 = 0.0, QV1 = 0.0;
    double EPSH1 = 0.0, SIGV1 = 0.0;  // undelayed [deg]
    double KQH1 = 0.0, KQV1 = 0.0;
};

// The wind-to-body matrix GenHel uses for every panel (pp. 5.2-9, 5.3-8,
// 5.3-12), applied to a wind-axis vector w.
using GenHelVec = std::array<double, 3>;
inline GenHelVec genhel_wind_to_body(double alpha, double beta, const GenHelVec& w) {
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double cb = std::cos(beta);
    const double sb = std::sin(beta);
    return {ca * cb * w[0] + ca * sb * w[1] - sa * w[2], sb * w[0] - cb * w[1],
            sa * cb * w[0] + sa * sb * w[1] + ca * w[2]};
}

inline GenHelAirframeOutput genhel_airframe(const GenHelAirframeTables& t,
                                            const GenHelAirframeInput& in) {
    constexpr double r2d = 57.29577951308232;
    GenHelAirframeOutput o;
    const double wash = in.DWSHMR * in.OMGTMR * in.RMR;

    // ---- Fuselage, pp. 5.2-5 to 5.2-9 ----
    const double VXIWF = t.EKXWF(in.CHIPMR, in.A1FMR) * wash;
    const double VZIWF = -t.EKZWF(in.CHIPMR, in.A1FMR) * wash;
    const double VXWF = in.VXB + VXIWF;
    const double VYWF = in.VYB;  // EKYWF: no input for the UH-60A
    const double VZWF = in.VZB + VZIWF;
    o.ALFWF = std::atan(VZWF / std::abs(VXWF)) * r2d;
    o.BETAWF = std::atan(VYWF / std::sqrt(VXWF * VXWF + VZWF * VZWF)) * r2d;
    o.PSIWF = -o.BETAWF;  // p. 5.2-12: PSIWF = −BETAWF
    o.QWF = 0.5 * in.RHO * (VXWF * VXWF + VYWF * VYWF + VZWF * VZWF);
    const double psi = o.PSIWF;
    const double apsi = std::abs(psi);
    const double sgn = psi > 0.0 ? 1.0 : (psi < 0.0 ? -1.0 : 0.0);
    // p. 5.2-7.
    const double DQFTOT = t.DQF(o.ALFWF) + t.DDQF(apsi);
    const double YQFTOT = sgn * t.YQF(apsi);
    const double LQFTOT = t.LQF(o.ALFWF) + t.DLQF(psi);
    const double RQFTOT = sgn * t.RQF(apsi);
    const double MQFTOT = t.MQF(o.ALFWF) + sgn * t.DMQF(apsi);
    const double NQFTOT = t.NQF(psi);
    // p. 5.2-9.
    const double a = o.ALFWF / r2d;
    const double b = o.BETAWF / r2d;
    const GenHelVec f =
        genhel_wind_to_body(a, b, {-DQFTOT * o.QWF, -YQFTOT * o.QWF, -LQFTOT * o.QWF});
    const GenHelVec m =
        genhel_wind_to_body(a, b, {RQFTOT * o.QWF, -MQFTOT * o.QWF, NQFTOT * o.QWF});
    const double FWT = (in.FSCGB - in.FSWF) / 12.0;  // p. 5.2-8
    const double WWT = (in.WLCGB - in.WLWF) / 12.0;
    const double BWT = (in.BLCGB - in.BLWF) / 12.0;
    o.WF = {f[0],
            f[1],
            f[2],
            m[0] - f[1] * WWT + f[2] * BWT,
            m[1] - f[2] * FWT + f[0] * WWT,
            m[2] + f[1] * FWT - f[0] * BWT};

    // ---- Horizontal tail, pp. 5.3-5 to 5.3-8 ----
    {
        const double F = (in.FSH1 - in.FSCGB) / 12.0;
        const double W = (in.WLH1 - in.WLCGB) / 12.0;
        const double B = (in.BLH1 - in.BLCGB) / 12.0;
        const double VXMRH1 = wash * t.EKXH1(in.CHIPMR, in.A1FMR);
        const double VZMRH1 = -wash * t.EKZH1(in.CHIPMR, in.A1FMR);
        o.KQH1 = std::sqrt(t.QH1(o.ALFWF));
        o.EPSH1 = t.EPH1(o.ALFWF);
        const double eps = std::isnan(in.EPSH1_DELAYED) ? o.EPSH1 : in.EPSH1_DELAYED;
        const double VZWFH1 = -in.VXB / r2d * eps * o.KQH1;
        const double VXIH1 = VXMRH1;
        const double VYIH1 = 0.0;  // EKYH1: no input for the UH-60A
        const double VZIH1 = VZMRH1 + VZWFH1;
        const double VXH1 = in.VXB * o.KQH1 - in.Q * W + in.R * B + VXIH1;
        const double VYH1 = in.VYB * o.KQH1 + in.P * W - in.R * F + VYIH1;
        const double VZH1 = in.VZB * o.KQH1 + in.Q * F - in.P * B + VZIH1;
        o.QH1 = 0.5 * in.RHO * (VXH1 * VXH1 + VYH1 * VYH1 + VZH1 * VZH1);
        o.ALFH1 = std::atan(VZH1 / std::abs(VXH1)) * r2d;
        o.ALFHH1 = in.IH1 + o.ALFH1;
        o.BETAH1 = std::atan(VYH1 / std::sqrt(VXH1 * VXH1 + VZH1 * VZH1)) * r2d;
        // CLH1 is odd (UVSUVS), CDH1 is looked up on |ALFHH1| (p. 5.3-8).
        const double ah = o.ALFHH1;
        const double CLH1 = (ah < 0.0 ? -1.0 : 1.0) * t.CLH1(std::abs(ah));
        const double CDH1 = t.CDH1(std::abs(ah));
        const GenHelVec fh = genhel_wind_to_body(
            o.ALFH1 / r2d, o.BETAH1 / r2d, {-o.QH1 * in.SAH1 * CDH1, 0.0, -o.QH1 * in.SAH1 * CLH1});
        o.H1 = {fh[0],
                fh[1],
                fh[2],
                fh[1] * W - fh[2] * B,
                -fh[0] * W + fh[2] * F,
                -fh[1] * F + fh[0] * B};
    }

    // ---- Vertical tail, pp. 5.3-9 to 5.3-12 ----
    {
        const double F = (in.FSV1 - in.FSCGB) / 12.0;
        const double W = (in.WLV1 - in.WLCGB) / 12.0;
        const double B = (in.BLV1 - in.BLCGB) / 12.0;
        const double VXMRV1 = wash * t.EKXH1(in.CHIPMR, in.A1FMR);
        const double VZMRV1 = -wash * t.EKZH1(in.CHIPMR, in.A1FMR);  // sign: see top
        o.KQV1 = std::sqrt(t.QV1(apsi));
        o.SIGV1 = sgn * t.SGV1(apsi);  // UVSUVS: odd in PSIWF
        const double sig = std::isnan(in.SIGV1_DELAYED) ? o.SIGV1 : in.SIGV1_DELAYED;
        const double VYWFV1 = -in.VXB / r2d * sig * o.KQV1;
        const double VXV1 = in.VXB * o.KQV1 - in.Q * W + in.R * B + VXMRV1;
        const double VYV1 = in.VYB * o.KQV1 - in.R * F + in.P * W + VYWFV1;
        const double VZV1 = in.VZB * o.KQV1 + in.Q * F - in.P * B + VZMRV1;
        o.QV1 = 0.5 * in.RHO * (VXV1 * VXV1 + VYV1 * VYV1 + VZV1 * VZV1);
        o.ALFV1 = std::atan(VZV1 / std::abs(VXV1)) * r2d;
        o.BETAV1 = std::atan(VYV1 / std::sqrt(VXV1 * VXV1 + VZV1 * VZV1)) * r2d;
        o.BETVV1 = in.IV1 + o.BETAV1;
        const double CLV1 = t.CLV1(o.BETVV1);
        const double CDV1 = t.CDV1(o.BETVV1);
        const GenHelVec fv = genhel_wind_to_body(
            o.ALFV1 / r2d, o.BETAV1 / r2d, {-CDV1 * in.SAV1 * o.QV1, CLV1 * in.SAV1 * o.QV1, 0.0});
        o.V1 = {fv[0],
                fv[1],
                fv[2],
                fv[1] * W - fv[2] * B,
                fv[2] * F - fv[0] * W,
                -fv[1] * F + fv[0] * B};
    }
    return o;
}

// GenHel's stabilator control system (Fig. 5.1.7, p. 5.5-18), steady
// state: airspeed lag settled, no pitch rate or lateral acceleration, servo
// at unit gain. Constants read from data/uh60a/genhel/fig_5_5_1_7_stabilator.txt.
struct GenHelStabilator {
    std::map<std::string, std::vector<double>> k;
    // Commanded incidence IHT0 [deg] at airspeed VXBIKT [kt] and collective
    // XCPC [percent, assumed: DATA_GAPS.md].
    double command(double VXBIKT, double XCPC) const {
        const auto lim = [](double x, const std::vector<double>& l) {
            return std::fmin(std::fmax(x, l[0]), l[1]);
        };
        const double PR50 = lim(VXBIKT + k.at("BIAS_VX1")[0], k.at("LMP50"));
        const double PR51 = lim(VXBIKT + k.at("BIAS_VX2")[0], k.at("LMP51"));
        const double C2 = lim(k.at("BIAS_C2")[0] - XCPC, k.at("LMC2"));
        const double C1 = lim(k.at("BIAS_C1")[0] - XCPC, k.at("LMC1"));
        const double KVX1 = -(k.at("KCO")[0] * 1.0 + k.at("KC2")[0] * C2 + k.at("KC1")[0] * C1);
        return k.at("BIAS_ITAIL0")[0] + KVX1 * PR50 + k.at("KVX2")[0] * PR51;
    }
    // IH1 [deg]: the command within the travel limits.
    double incidence(double VXBIKT, double XCPC) const {
        return std::fmin(std::fmax(command(VXBIKT, XCPC), k.at("LMSTAB")[0]), k.at("LMSTAB")[1]);
    }
};

inline GenHelStabilator read_genhel_stabilator(const std::string& dir) {
    std::ifstream f(dir + "/fig_5_5_1_7_stabilator.txt");
    if (!f) {
        throw std::runtime_error("cannot open the stabilator transcription");
    }
    GenHelStabilator s;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream in(line);
        std::string name;
        in >> name;
        double v = 0.0;
        while (in >> v) {
            s.k[name].push_back(v);
        }
    }
    return s;
}

// The fuselage wake at the tail rotor (p. 5.4-5): K_QTR from Table 3.5.7 on
// |PSIWF|, sidewash from Table 3.5.8 on PSIWF and downwash from Table 3.5.4
// on ALFWF, each delayed; NaN delayed values mean the steady state.
struct GenHelTailRotorWake {
    double KQTR = 1.0;
    double SIGTR = 0.0, EPSTR = 0.0;    // undelayed [deg]
    double VYWFTR = 0.0, VZWFTR = 0.0;  // [ft/s]
};

inline GenHelTailRotorWake genhel_tail_rotor_wake(const GenHelAirframeTables& t, double ALFWF,
                                                  double PSIWF, double VXB, double SIGTR_DELAYED,
                                                  double EPSTR_DELAYED) {
    constexpr double r2d = 57.29577951308232;
    GenHelTailRotorWake w;
    const double sgn = PSIWF > 0.0 ? 1.0 : (PSIWF < 0.0 ? -1.0 : 0.0);
    w.KQTR = std::sqrt(t.QV1(std::abs(PSIWF)));
    w.SIGTR = sgn * t.SGV1(std::abs(PSIWF));
    w.EPSTR = t.EPH1(ALFWF);
    const double sig = std::isnan(SIGTR_DELAYED) ? w.SIGTR : SIGTR_DELAYED;
    const double eps = std::isnan(EPSTR_DELAYED) ? w.EPSTR : EPSTR_DELAYED;
    w.VYWFTR = -VXB / r2d * sig * w.KQTR;
    w.VZWFTR = -VXB / r2d * eps * w.KQTR;
    return w;
}

}  // namespace eyas::test
