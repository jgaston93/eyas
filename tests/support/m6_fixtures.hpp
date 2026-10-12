#pragma once

// Fixtures for the M6 tests (D-028). The tail-rotor constants are GenHel's
// UH-60A values (CR-166309 p. 5.4-14), verified by human review
// (DATA_GAPS.md); the CG station is GenHel's (p. 5.1-49). Everything else
// here is synthetic test data.

#include <cmath>
#include <memory>
#include <numbers>
#include <string>
#include <utility>

#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/controls.hpp"
#include "support/genhel_tail_rotor.hpp"
#include "support/m3_fixtures.hpp"
#include "util/units.hpp"

namespace eyas::test {

// GenHel input for the UH-60A tail rotor, flight state zero.
inline GenHelTailRotorInput genhel_uh60a_tail_rotor() {
    GenHelTailRotorInput in;
    in.FSTR = 732.0;
    in.WLTR = 324.7;
    in.BLTR = -14.0;
    in.FSCGB = 355.9;
    in.WLCGB = 248.2;
    in.BLCGB = 0.0;
    in.RTR = 5.5;
    in.OMEGTR = 124.62;
    in.BTR = 4.0;
    in.CHRDTR = 0.81;
    in.ATR = 5.73;
    in.BTLTR = 0.92;
    in.TWSTTR = -18.0;
    in.GAMTR = 70.0;
    in.DEL3TR = 35.0;
    in.DELTTR = 0.001455;
    in.BIASTR = 6.0;
    in.CDTR = 0.0;
    in.KBVTTR = 0.796;
    in.VBVTTR = 30.0;
    in.RHO = 0.0023769;
    return in;
}

// θ_cmd = pedal [rad]: the commanded pitch is set through Controls::pedal.
inline std::shared_ptr<const PitchSchedule> pedal_is_pitch() {
    LinearPitchRow theta0;
    theta0.pedal = 1.0;
    return std::make_shared<const LinearControlMap>(theta0, LinearPitchRow{}, LinearPitchRow{});
}

// The production configuration of the same rotor. Body axes from GenHel
// stations: x = −ΔFS, y = −ΔBL (buttlines positive to port), z = −ΔWL.
inline BaileyRotorConfig bailey_from_genhel(const GenHelTailRotorInput& in) {
    using namespace units;
    BaileyRotorConfig c;
    c.name = "tail_rotor";
    c.num_blades = static_cast<int>(in.BTR);
    c.radius = in.RTR * kFoot;
    c.rotor_speed = in.OMEGTR;
    c.chord = in.CHRDTR * kFoot;
    c.lift_slope = in.ATR;
    c.tip_loss = in.BTLTR;
    c.twist = in.TWSTTR * kDegree;
    c.delta3 = in.DEL3TR * kDegree;
    c.coning_per_thrust = in.DELTTR * kDegree / kPoundForce;
    c.pitch_bias = in.BIASTR * kDegree;
    c.drag_area = in.CDTR * kFoot * kFoot;
    c.blockage_factor = in.KBVTTR;
    c.blockage_speed = in.VBVTTR * kKnot;
    c.hub_position_b =
        Vec3(-(in.FSTR - in.FSCGB), -(in.BLTR - in.BLCGB), -(in.WLTR - in.WLCGB)) * kInch;
    c.hub_roll = in.GAMTR * kDegree;
    c.controls = pedal_is_pitch();
    return c;
}

// Synthetic main-rotor wash table (not aircraft data): E_KX and E_KZ on a
// (χ, a₁) grid, smooth and nonzero so every interpolation path is used.
inline std::string synthetic_wash_csv() {
    std::string s =
        "# source: synthetic main-rotor wash test table (tests only, not aircraft data)\n"
        "chi[deg],a1[deg],ekx[1],ekz[1]\n";
    for (int chi = 0; chi <= 100; chi += 25) {
        for (int a1 = -6; a1 <= 6; a1 += 6) {
            const double ekx = 0.01 * chi - 0.03 * a1;
            const double ekz = 1.2 - 0.004 * chi + 0.02 * a1;
            s += std::to_string(chi) + "," + std::to_string(a1) + "," + std::to_string(ekx) + "," +
                 std::to_string(ekz) + "\n";
        }
    }
    return s;
}

// δ₃ = 0, no blockage, no drag: the bare Bailey rotor used by the anchors.
BaileyRotorConfig bare(double tip_loss, double twist) {
    GenHelTailRotorInput in = genhel_uh60a_tail_rotor();
    BaileyRotorConfig c = bailey_from_genhel(in);
    c.tip_loss = tip_loss;
    c.twist = twist;
    c.delta3 = 0.0;
    c.coning_per_thrust = 0.0;
    c.pitch_bias = 0.0;
    c.blockage_speed = 0.0;
    c.hub_position_b = Vec3::Zero();
    c.hub_roll = 0.0;
    return c;
}

// The thin-plate small-angle airfoil: C_l = a·w(α) with w the angle to the
// chord line taken from whichever edge faces the wind, i.e. α wrapped to
// [−90°, 90°) (π-periodic); C_d = 0. In reverse flow the production rotor's
// lift direction flips with the wind (D-024), so this gives lift
// ∝ U_T|U_T| θ − |U_T| U_P, the small-angle reverse-flow lift Bailey's
// coefficients integrate (Johnson 2013 §6.21, Eq. 6.219).
std::string thin_plate_csv(double a) {
    const double e = 1e-6;
    const double d = std::numbers::pi / 180.0;
    std::string s =
        "# source: synthetic thin-plate test airfoil, C_l = a*alpha wrapped to [-90, 90) deg "
        "(tests only)\nalpha[deg],mach[1],cl[1],cd[1]\n";
    const double pts[][2] = {{-180.0, 0.0},        {-90.0 - e, 90.0 - e}, {-90.0 + e, -90.0 + e},
                             {90.0 - e, 90.0 - e}, {90.0 + e, -90.0 + e}, {180.0, 0.0}};
    for (const char* m : {"0", "1"}) {
        for (const auto& p : pts) {
            s += fmt17(p[0]) + "," + m + "," + fmt17(a * p[1] * d) + ",0\n";
        }
    }
    return s;
}

// The production blade-element rotor with the same geometry, flapping held at
// zero (M3), at a uniform inflow; thrust averaged over 1/N_b rev.
double blade_element_ct(const BaileyRotorConfig& b, double theta0, double lambda_i, double mu) {
    BladeElementRotorConfig c;
    c.num_blades = b.num_blades;
    c.radius = b.radius;
    c.rotor_speed = b.rotor_speed;
    c.root_cutout = 0.0;
    c.tip_loss = b.tip_loss;
    c.chord = Table1D(Axis({0.0, 1.0}), {b.chord, b.chord});
    c.twist = Table1D(Axis({0.0, 1.0}), {0.0, b.twist});
    c.num_elements = 4000;
    c.spacing = ElementSpacing::uniform;
    c.airfoil = airfoil_from(thin_plate_csv(b.lift_slope));
    c.controls = std::make_shared<const FixedPitch>(SwashplatePitch{theta0, 0.0, 0.0});
    const BladeElementRotor rotor(std::move(c));
    SimContext ctx;
    ctx.air.density = 1.2;
    ctx.air.speed_of_sound = 1e9;  // no Mach effect (the airfoil has none anyway)
    ctx.body.vel_b = Vec3(mu * rotor.tip_speed(), 0.0, 0.0);
    const int n = 64;
    double sum = 0.0;
    for (int k = 0; k < n; ++k) {
        ctx.t = (2.0 * std::numbers::pi / b.num_blades) * k / n / b.rotor_speed;
        sum += rotor.evaluate_at_inflow(ctx, lambda_i).ct;
    }
    return sum / n;
}

double bailey_ct(const BaileyRotorConfig& b, double theta0, double lambda_i, double mu) {
    const BaileyRotor rotor(b);
    SimContext ctx;
    ctx.air.density = 1.2;
    ctx.body.vel_b = Vec3(mu * rotor.tip_speed(), 0.0, 0.0);
    ctx.controls.pedal = theta0;
    return rotor.evaluate(ctx, lambda_i, Vec3::Zero()).ct;
}

// Small-angle thrust integrals over r in [0, B], ψ in [0, 2π), by midpoint
// quadrature (independent of both rotors): with U_T = r + μ sin ψ,
//   C_T/(σa/2) = θ₀ E[∫ U_T u dr] + θ_tw E[∫ r U_T u dr] − λ_i E[∫ |U_T| dr]
// (hover, μ_z = 0), where u = |U_T| with reverse flow on the pitch terms and
// u = U_T without it. Lift on the thin-plate airfoil is ∝ U_T|U_T|θ −
// |U_T|U_P (blade_element_ct).
struct ThrustIntegrals {
    double theta = 0.0;   // E[∫ U_T u dr]
    double twist = 0.0;   // E[∫ r U_T u dr]
    double inflow = 0.0;  // E[∫ |U_T| dr]
};
ThrustIntegrals thrust_integrals(double B, double mu, bool reverse) {
    const int nr = 2000;
    const int np = 720;
    ThrustIntegrals t;
    for (int j = 0; j < np; ++j) {
        const double sp = std::sin(2.0 * std::numbers::pi * (j + 0.5) / np);
        for (int i = 0; i < nr; ++i) {
            const double r = B * (i + 0.5) / nr;
            const double ut = r + mu * sp;
            const double u = reverse ? std::abs(ut) : ut;
            t.theta += ut * u;
            t.twist += r * ut * u;
            t.inflow += std::abs(ut);
        }
    }
    const double w = B / nr / np;
    t.theta *= w;
    t.twist *= w;
    t.inflow *= w;
    return t;
}

}  // namespace eyas::test
