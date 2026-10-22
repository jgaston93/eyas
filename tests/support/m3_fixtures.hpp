#pragma once

// Synthetic test rotors and airfoils for the M3 tests and validation cases.
// These are test fixtures, not aircraft data: they are chosen to exercise
// the code (and, for the linear airfoil, to match the assumptions of the
// closed-form oracle, D-007), and never go into data/.

#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "core/component.hpp"
#include "env/atmosphere.hpp"
#include "rotor/airfoil.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/controls.hpp"
#include "support/bet_reference.hpp"
#include "util/csv_table.hpp"

namespace eyas::test {

inline std::string fmt17(double v) {
    std::array<char, 40> b{};
    (void)std::snprintf(b.data(), b.size(), "%.17g", v);
    return b.data();
}

// C_l = a α, C_d = C_d0 over α in [−180°, 180°], independent of Mach. Two
// α breakpoints, so linear interpolation reproduces the line exactly (to
// round-off). The assumptions of Leishman 2nd ed. §3.2 (D-007 layer 2).
inline std::string linear_airfoil_csv(double a, double cd0) {
    const double c = a * std::numbers::pi;
    std::string s =
        "# source: synthetic test airfoil, C_l = a*alpha, C_d = C_d0 (tests only)\n"
        "alpha[deg],mach[1],cl[1],cd[1]\n";
    for (const char* m : {"0", "1"}) {
        s += std::string("-180,") + m + "," + fmt17(-c) + "," + fmt17(cd0) + "\n";
        s += std::string("180,") + m + "," + fmt17(c) + "," + fmt17(cd0) + "\n";
    }
    return s;
}

// A nonlinear synthetic airfoil shaped like a real one, to exercise what
// real tables do: Mach dependence (Prandtl–Glauert slope), stall near 14°,
// flat-plate behaviour at large angles, and continuous reverse flow at ±180°.
// Shapes are invented for testing only.
inline double synthetic_cl(double alpha, double mach) {
    const double beta = std::sqrt(1.0 - std::min(mach, 0.9) * std::min(mach, 0.9));
    const double a = 5.9 / beta;
    const double stall = (14.0 - 4.0 * mach) * std::numbers::pi / 180.0;
    const double attached = a * alpha;
    const double plate = 1.1 * std::sin(2.0 * alpha);
    const double x = std::abs(alpha);
    if (x <= stall) {
        return attached;
    }
    const double blend = std::exp(-(x - stall) / 0.12);
    return blend * a * std::copysign(stall, alpha) + (1.0 - blend) * plate;
}

inline double synthetic_cd(double alpha, double mach) {
    const double drag_rise = mach > 0.75 ? 0.4 * (mach - 0.75) * (mach - 0.75) : 0.0;
    return 0.008 + 0.01 * alpha * alpha + drag_rise + 1.8 * std::pow(std::sin(alpha), 2);
}

inline std::string synthetic_airfoil_csv() {
    std::vector<double> alphas;
    for (int d = -180; d <= -30; d += 10) {
        alphas.push_back(d);
    }
    for (int d = -28; d <= 28; d += 2) {
        alphas.push_back(d);
    }
    for (int d = 30; d <= 180; d += 10) {
        alphas.push_back(d);
    }
    std::string s =
        "# source: synthetic nonlinear test airfoil (tests only)\n"
        "alpha[deg],mach[1],cl[1],cd[1]\n";
    for (const double d : alphas) {
        for (int m = 0; m <= 10; ++m) {
            const double mach = 0.1 * m;
            const double a = d * std::numbers::pi / 180.0;
            s += fmt17(d) + "," + fmt17(mach) + "," + fmt17(synthetic_cl(a, mach)) + "," +
                 fmt17(synthetic_cd(a, mach)) + "\n";
        }
    }
    return s;
}

inline std::shared_ptr<const Airfoil> airfoil_from(const std::string& csv) {
    return std::make_shared<const Airfoil>(Airfoil::from_csv(parse_csv_table(csv, "test airfoil")));
}

// Zero swashplate map plus fixed pitch; the controls are ignored.
class FixedPitch : public PitchSchedule {
public:
    explicit FixedPitch(SwashplatePitch p) : p_(p) {}
    SwashplatePitch pitch(const Controls& /*c*/) const noexcept override { return p_; }

private:
    SwashplatePitch p_;
};

// A test rotor of UH-60 size (not UH-60 data): R = 8 m, Ω = 27 rad/s, 4 blades,
// constant chord 0.53 m (σ = 0.0843), linear twist, given airfoil.
inline BladeElementRotorConfig test_rotor(std::shared_ptr<const Airfoil> airfoil,
                                          SwashplatePitch pitch, int elements,
                                          ElementSpacing spacing = ElementSpacing::uniform) {
    BladeElementRotorConfig c;
    c.num_blades = 4;
    c.radius = 8.0;
    c.rotor_speed = 27.0;
    c.root_cutout = 1.2;
    c.tip_loss = 1.0;
    c.chord = Table1D(Axis({0.0, 1.0}), {0.53, 0.53});
    c.twist = Table1D(Axis({0.0, 1.0}), {0.0, -16.0 * std::numbers::pi / 180.0});
    c.num_elements = elements;
    c.spacing = spacing;
    c.airfoil = std::move(airfoil);
    c.controls = std::make_shared<const FixedPitch>(pitch);
    return c;
}

// Sea-level ISA context, level attitude, at rest, at time t.
inline SimContext sea_level_context(double t = 0.0) {
    SimContext ctx;
    ctx.t = t;
    ctx.air = isa_atmosphere(0.0, 0.0);
    return ctx;
}

// The reference rotor and condition matching a production rotor and context.
// The hub-axis velocity uses the production C_hb, which is tested on its own.
inline RefRotor reference_rotor(const BladeElementRotor& rotor) {
    const BladeElementRotorConfig& c = rotor.config();
    RefRotor r;
    r.num_blades = c.num_blades;
    r.radius = c.radius;
    r.omega = c.rotor_speed;
    r.s = c.direction == RotationDirection::counter_clockwise ? 1.0 : -1.0;
    r.root_cutout = c.root_cutout;
    r.tip_loss = c.tip_loss;
    r.momentum_radius = c.inflow_tip_loss == InflowTipLoss::effective_disc ? c.tip_loss : 1.0;
    const Table1D chord = c.chord;
    const Table1D twist = c.twist;
    r.chord = [chord](double x) { return chord(x); };
    r.twist = [twist](double x) { return twist(x); };
    r.airfoil = c.airfoil.get();
    return r;
}

inline RefCondition reference_condition(const BladeElementRotor& rotor, const SimContext& ctx) {
    const BladeElementRotorConfig& c = rotor.config();
    const Vec3 v_air_b = ctx.body.vel_b - ctx.body.q_nb.conjugate() * ctx.env.wind_n;
    const Vec3 v_hub_b = v_air_b + ctx.body.omega_b.cross(c.hub_position_b);
    RefCondition k;
    k.v_hub_h = rotor.C_hb() * v_hub_b;
    k.omega_h = rotor.C_hb() * ctx.body.omega_b;
    k.psi = std::fmod(c.rotor_speed * ctx.t, 2.0 * std::numbers::pi);
    k.pitch = c.controls->pitch(ctx.controls);
    k.rho = ctx.air.density;
    k.a = ctx.air.speed_of_sound;
    return k;
}

}  // namespace eyas::test
