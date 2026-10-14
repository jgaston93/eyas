#pragma once

// Exact-equation reference for the M6c landing gear (D-032, layer 1).
//
// A literal transcription of GenHel CR-166309 §5.7.2 (pp. 5.7-9 to 5.7-26) in
// GenHel's notation, units and conventions: inches for stations and tire
// sizes, feet, lbf, slugs, psi; FS positive aft, WL positive up, BL positive
// to starboard in this module (the right main gear is at +BLMGA, the left at
// −BLMGA: L_LG = Y_RMGA (Z_FGRR − Z_FGRL), p. 5.7-26); space axes x north,
// y east, z down; the ground plane at height WLFD [ft], level.
//
// GenHel's tables are read from the as-printed transcriptions in
// data/uh60a/genhel/ and interpolated here (bilinear in deflection or load and
// tire pressure, clamped at the ends). Production reads the CSVs that
// tools/build_uh60a_gear.py writes and slices them at the tire pressure, so
// the comparison also checks both. Nothing else is shared with production.
//
// Inputs that GenHel keeps from frame to frame are inputs here:
// - the retained contact point (XID, YID) [ft, space axes] (p. 5.7-16);
// - the tire's filtered deflection rates DXF, DYF, DZF [ft/s, body axes]
//   (p. 5.7-17: a frame difference through a 0.01 s lag);
// - the strut stroke DL [ft] and its rate DLD [ft/s] (p. 5.7-23).
//
// Departures from the page, each deliberate:
// - The tire can't pull: −Z_FTH ≥ 0 (D-032, implementation notes).
// - GenHel enters its tire tables with |ΔZ|; production has no vertical load
//   for ΔZ ≤ 0, which happens only when a barely touching tire's tilted
//   in-plane deflection outweighs δ_T. The tests compare ΔZ > 0 only.
// - GenHel's friction limit uses |Z_FTD| (p. 5.7-18); production uses the
//   normal load only while it pushes. With a barely loaded tire the tilted
//   trial in-plane forces can make Z_FTD > 0; the tests skip those states.
// - K′ (Tables 7.5.4, 7.5.6) is looked up on the spring part of the vertical
//   load, which GenHel's frame order makes from the previous pass (D-032).
// - Only the stroking branch of the strut (p. 5.7-23, "Z_FTA ≤ −F_PRE") is
//   transcribed; the locked preload branch is replaced by a stop (D-032
//   item 6) and tested in layer 2.
// - The vertical gear-load filter 1/(1 + T_FGR s) (p. 5.7-25) is at unity:
//   T_FGR is not given (D-032 item 8). STLDSW = 0 (strut loads used).
// - Degrees aren't involved; the attitude is in radians.

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace eyas::test {

// ---- GenHel's tables ---------------------------------------------------------

// Numeric rows of a transcription file, comments stripped.
inline std::vector<std::vector<double>> read_genhel_rows(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        throw std::runtime_error("cannot open " + path);
    }
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(f, line)) {
        line = line.substr(0, line.find('#'));
        std::istringstream ss(line);
        std::vector<double> r;
        double v = 0.0;
        while (ss >> v) {
            r.push_back(v);
        }
        if (!r.empty()) {
            rows.push_back(r);
        }
    }
    return rows;
}

// A GenHel map: rows of (x, value at each pressure), pressures given.
struct GenHelMap {
    std::vector<std::vector<double>> rows;
    std::vector<double> pressures;  // empty: one column

    double operator()(double x, double p) const {
        const auto lerp1 = [&](std::size_t col) {
            if (x <= rows.front()[0]) {
                return rows.front()[col];
            }
            if (x >= rows.back()[0]) {
                return rows.back()[col];
            }
            std::size_t i = 0;
            while (rows[i + 1][0] < x) {
                ++i;
            }
            const double t = (x - rows[i][0]) / (rows[i + 1][0] - rows[i][0]);
            return rows[i][col] + t * (rows[i + 1][col] - rows[i][col]);
        };
        if (pressures.empty()) {
            return lerp1(1);
        }
        const double pc = std::clamp(p, pressures.front(), pressures.back());
        std::size_t j = 0;
        while (j + 2 < pressures.size() && pressures[j + 1] < pc) {
            ++j;
        }
        const double t = (pc - pressures[j]) / (pressures[j + 1] - pressures[j]);
        return lerp1(j + 1) + t * (lerp1(j + 2) - lerp1(j + 1));
    }
};

// ---- Inputs ------------------------------------------------------------------

struct GenHelGear {
    // Geometry: strut station, waterline of the axle at free extension, butt line.
    double FSGA = 0.0, WLGAO = 0.0, BLGA = 0.0;               // [in]
    double RT = 0.0, DT = 0.0;                                // tire radius, diameter [in]
    double P = 0.0, PR = 0.0;                                 // tire and rated pressure [psi]
    double NT = 1.0;                                          // wheels
    double M = 0.0;                                           // unsprung mass [slug]
    double GD = 0.0;                                          // tire damping factor [s]
    double CPV = 0.0, FPRE = 0.0;                             // air spring [ft·lbf], preload [lbf]
    double SD_BRK = 0.0;                                      // damper break speed [ft/s]
    double C_LO = 0.0, F0_LO = 0.0, C_HI = 0.0, F0_HI = 0.0;  // [lbf·s²/ft²], [lbf]
    bool braked = false;    // false: K_μB = N_BRK always (tail wheel)
    bool main_tire = true;  // lateral stiffness from Table 7.5.2; else 19.7 √Z (p. 5.7-46)
    GenHelMap vertical;     // Table 7.5.3 or 7.5.5: Z_FTH [lbf] against 12·ΔZ [in]
    GenHelMap damping;      // Table 7.5.4 or 7.5.6: K′ [lbf/in] against −Z_FTH [lbf]
    GenHelMap lateral;      // Table 7.5.2: K_TY [lbf/in] against 12·|ΔZ| [in]

    // Frame-to-frame values.
    double XID = 0.0, YID = 0.0;             // retained contact point [ft]
    double DL = 0.0, DLD = 0.0;              // stroke [ft], rate [ft/s]
    double DXF = 0.0, DYF = 0.0, DZF = 0.0;  // filtered deflection rates [ft/s]
};

struct GenHelGearInput {
    double THETA = 0.0, PHI = 0.0, PSI = 0.0;  // [rad]
    double XSH = 0.0, YSH = 0.0, ZSH = 0.0;    // CG, space axes [ft]
    double FSCGB = 0.0, WLCGB = 0.0;           // CG [in]
    double WLFD = 0.0;                         // ground plane height [ft]
    double MU = 0.0, BRK = 0.0, NBRK = 0.0, KSLIP = 0.0;
    // GenHel's brakes are set or off (p. 5.7-18). Eyas blends linearly for a
    // brake input b in [0, 1] (D-032), which this reference applies too; at
    // b = 0 and 1 it is GenHel exactly.
    double brake = 0.0;
    std::vector<GenHelGear> gears;
};

struct GenHelGearOut {
    bool contact = false;
    double DLT = 0.0;                   // δ_T, tire radial deflection [ft]
    std::array<double, 3> DH{};         // tire deflection, body axes [ft]
    std::array<double, 3> FTD_trial{};  // trial tire force, ground axes ψ = 0 [lbf]
    std::array<double, 3> FTH{};        // corrected tire force, body axes [lbf]
    std::array<bool, 2> slip{};
    double FSTR = 0.0;            // strut force [lbf]
    double DLDD = 0.0;            // GenHel's δ̈ [ft/s²]
    std::array<double, 6> FGR{};  // X, Y, Z, L, M, N at the gear reference point
};

struct GenHelGearResult {
    std::vector<GenHelGearOut> gears;
    std::array<double, 6> LG{};  // XLG, YLG, ZLG [lbf], LLG, MLG, NLG [ft·lbf]
};

// ---- Equations -----------------------------------------------------------------

using M3 = std::array<std::array<double, 3>, 3>;

// A_HBS, body to space (columns: body axes in space axes); p. 5.7-13 gives its
// third column.
inline M3 genhel_ahbs(double th, double ph, double ps) {
    const double ct = std::cos(th), st = std::sin(th), cp = std::cos(ph), sp = std::sin(ph);
    const double cs = std::cos(ps), ss = std::sin(ps);
    return {{{ct * cs, sp * st * cs - cp * ss, cp * st * cs + sp * ss},
             {ct * ss, sp * st * ss + cp * cs, cp * st * ss - sp * cs},
             {-st, sp * ct, cp * ct}}};
}

inline std::array<double, 3> mul(const M3& a, const std::array<double, 3>& v) {
    return {a[0][0] * v[0] + a[0][1] * v[1] + a[0][2] * v[2],
            a[1][0] * v[0] + a[1][1] * v[1] + a[1][2] * v[2],
            a[2][0] * v[0] + a[2][1] * v[1] + a[2][2] * v[2]};
}

inline std::array<double, 3> mul_t(const M3& a, const std::array<double, 3>& v) {
    return {a[0][0] * v[0] + a[1][0] * v[1] + a[2][0] * v[2],
            a[0][1] * v[0] + a[1][1] * v[1] + a[2][1] * v[2],
            a[0][2] * v[0] + a[1][2] * v[1] + a[2][2] * v[2]};
}

// Body coordinates of the axle at free extension [ft] (pp. 5.7-9, 5.7-10).
inline std::array<double, 3> genhel_gear_axle(const GenHelGearInput& in, const GenHelGear& g) {
    return {(in.FSCGB - g.FSGA) / 12.0, g.BLGA / 12.0, (in.WLCGB - g.WLGAO) / 12.0};
}

// Axle reference point in space axes (p. 5.7-11) and the gear line's
// intersection with the ground (p. 5.7-15) [ft].
inline std::array<double, 3> genhel_gear_axle_space(const GenHelGearInput& in,
                                                    const GenHelGear& g) {
    const M3 A = genhel_ahbs(in.THETA, in.PHI, in.PSI);
    const auto a = genhel_gear_axle(in, g);
    const auto s = mul(A, {a[0], a[1], a[2] - g.DL});
    return {s[0] + in.XSH, s[1] + in.YSH, s[2] + in.ZSH};
}

inline std::array<double, 3> genhel_gear_intersection(const GenHelGearInput& in,
                                                      const GenHelGear& g) {
    const M3 A = genhel_ahbs(in.THETA, in.PHI, in.PSI);
    const auto as = genhel_gear_axle_space(in, g);
    // The gear reference point (p. 5.7-12) is on the same line, one foot up
    // the strut here; only the line's direction (A's third column) matters.
    const auto a = genhel_gear_axle(in, g);
    const auto r = mul(A, {a[0], a[1], a[2] - g.DL - 1.0});
    const std::array<double, 3> gs{r[0] + in.XSH, r[1] + in.YSH, r[2] + in.ZSH};
    const double aa = (gs[0] - as[0]) / (gs[2] - as[2]);
    const double bb = as[0] - aa * as[2];
    const double cc = (gs[1] - as[1]) / (gs[2] - as[2]);
    const double dd = as[1] - cc * as[2];
    const double zint = -in.WLFD;  // fixed, level ground plane
    return {aa * zint + bb, cc * zint + dd, zint};
}

inline GenHelGearResult genhel_gear(const GenHelGearInput& in) {
    GenHelGearResult res;
    const M3 A = genhel_ahbs(in.THETA, in.PHI, in.PSI);
    const M3 A0 = genhel_ahbs(in.THETA, in.PHI, 0.0);                // "temporarily set ψ = 0"
    const double cos_gamma = std::cos(in.THETA) * std::cos(in.PHI);  // p. 5.7-13
    for (const GenHelGear& g : in.gears) {
        GenHelGearOut o;
        const auto ga = genhel_gear_axle(in, g);
        const auto as = genhel_gear_axle_space(in, g);
        // p. 5.7-14: δ_T = R/12 + DNA/cos γ, DNA = Z_AS + WLFD.
        const double DNA = as[2] + in.WLFD;
        o.DLT = g.RT / 12.0 + DNA / cos_gamma;
        o.contact = o.DLT >= 0.0;
        double ZFTH = 0.0, XFTH = 0.0, YFTH = 0.0;
        if (o.contact) {
            // p. 5.7-16: deflections in space axes; p. 5.7-17: body axes.
            const auto I = genhel_gear_intersection(in, g);
            const std::array<double, 3> DS{I[0] - g.XID, I[1] - g.YID, 0.0};
            const auto DB = mul_t(A, DS);
            o.DH = {DB[0], DB[1], DB[2] + o.DLT};
            const double dz_in = 12.0 * std::abs(o.DH[2]);
            // Vertical: K_TZ ΔZ = map output (p. 5.7-17 note); damping with K′.
            const double z_spring = g.vertical(dz_in, g.P);
            const double ktz_d = 12.0 * g.damping(z_spring, g.P);  // [lbf/ft]
            ZFTH = -g.NT * (z_spring + g.GD * ktz_d * g.DZF);
            ZFTH = std::min(ZFTH, 0.0);  // no adhesion (departure)
            // In-plane stiffnesses (pp. 5.7-42, 5.7-46) [lbf/ft].
            const double KTX = 0.53 * g.DT * (g.P + 4.0 * g.PR) *
                               std::cbrt(12.0 * std::abs(o.DH[2]) / g.DT) * 12.0;
            const double KTY =
                g.main_tire ? 12.0 * g.lateral(dz_in, g.P) : 19.7 * std::sqrt(-ZFTH / g.NT) * 12.0;
            XFTH = -g.NT * (KTX * o.DH[0] + g.GD * KTX * g.DXF);
            YFTH = -g.NT * (KTY * o.DH[1] + g.GD * KTY * g.DYF);
            // p. 5.7-18: ground plane, ψ = 0; brakes; maximum X reaction.
            const auto FTD = mul(A0, {XFTH, YFTH, ZFTH});
            o.FTD_trial = FTD;
            const double KMUB = g.braked ? in.NBRK + (in.BRK - in.NBRK) * in.brake : in.NBRK;
            const double XM = KMUB * in.MU * std::abs(FTD[2]);
            const double YM = in.MU * std::abs(FTD[2]);
            std::array<double, 3> F = FTD;
            // p. 5.7-19: longitudinal and lateral slip.
            if (std::abs(F[0]) > XM) {
                F[0] = (F[0] < 0.0 ? -1.0 : 1.0) * in.KSLIP * XM;
                o.slip[0] = true;
            }
            if (std::abs(F[1]) > YM) {
                F[1] = (F[1] < 0.0 ? -1.0 : 1.0) * in.KSLIP * YM;
                o.slip[1] = true;
            }
            // p. 5.7-20: corrected loads in body axes.
            const auto FH = mul_t(A0, F);
            XFTH = FH[0];
            YFTH = FH[1];
            ZFTH = FH[2];
        }
        o.FTH = {XFTH, YFTH, ZFTH};
        // p. 5.7-20: loads at the axle reference point.
        const double R12 = g.RT / 12.0;
        const double LFTA = ZFTH * (-o.DH[1]) - YFTH * (R12 - o.DH[2]);
        const double MFTA = ZFTH * o.DH[0] + XFTH * (R12 - o.DH[2]);
        const double NFTA = YFTH * (-o.DH[0]) - XFTH * (-o.DH[1]);
        // pp. 5.7-23, 5.7-24: strut, stroking branch.
        const bool lo = std::abs(g.DLD) <= g.SD_BRK;
        const double C = lo ? g.C_LO : g.C_HI;
        const double F0 = lo ? g.F0_LO : g.F0_HI;
        const double sgn = g.DLD < 0.0 ? -1.0 : 1.0;
        const double FSTF = g.CPV / std::max(g.CPV / g.FPRE - g.DL, 0.01);
        o.FSTR = sgn * (C * g.DLD * g.DLD + F0) + FSTF;
        o.DLDD = -(1.0 / g.M) * (ZFTH + o.FSTR);
        // p. 5.7-25: gear loads at the gear reference point.
        o.FGR = {XFTH, YFTH, -o.FSTR, -YFTH * (ga[2] - g.DL) + LFTA, XFTH * (ga[2] - g.DL) + MFTA,
                 NFTA};
        res.gears.push_back(o);
    }
    // p. 5.7-26: loads at the CG, written for any number of gears with signed
    // butt lines (GenHel's Y_RMGA(Z_FGRR − Z_FGRL) and (X_FTAL − X_FTAR)Y_RMGA).
    for (std::size_t i = 0; i < in.gears.size(); ++i) {
        const auto ga = genhel_gear_axle(in, in.gears[i]);
        const auto& f = res.gears[i].FGR;
        res.LG[0] += f[0];
        res.LG[1] += f[1];
        res.LG[2] += f[2];
        res.LG[3] += ga[1] * f[2] + f[3];
        res.LG[4] += -f[2] * ga[0] + f[4];
        res.LG[5] += f[1] * ga[0] - f[0] * ga[1] + f[5];
    }
    return res;
}

}  // namespace eyas::test
