// Writes the M3 cases to CSV for the VALIDATION.md plots (tools/plot_m3.py).
// Same production code paths, reference and fixtures as
// tests/unit/test_blade_element_rotor.cpp.
//
// Usage: eyas_m3_cases <output-dir>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "sim/executive.hpp"
#include "support/bet_reference.hpp"
#include "support/frame_status.hpp"
#include "support/m3_fixtures.hpp"

namespace {

using namespace eyas;
using namespace eyas::test;

constexpr double kDeg = std::numbers::pi / 180.0;

std::ofstream open_csv(const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path.string());
    }
    out.precision(17);
    return out;
}

// Largest relative error of the six hub load components against the
// reference, forces scaled by |T| and moments by |T| R.
double max_error(const RotorLoads& l, const RefLoads& r, double R) {
    const double f = std::abs(r.thrust);
    double e = (l.F_h - r.F_h).cwiseAbs().maxCoeff() / f;
    return std::max(e, (l.M_h - r.M_h).cwiseAbs().maxCoeff() / (f * R));
}

double max_error(const RotorLoads& a, const RotorLoads& b, const RefLoads& r, double R) {
    // Richardson extrapolation (4 b − a)/3 from N (a) and 2N (b) elements.
    RotorLoads x = b;
    x.F_h = (4.0 * b.F_h - a.F_h) / 3.0;
    x.M_h = (4.0 * b.M_h - a.M_h) / 3.0;
    return max_error(x, r, R);
}

// D-007 layer 1: error against the exact-equation reference as the elements
// are refined, raw and Richardson-extrapolated.
void refinement(const std::filesystem::path& dir) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.9, 0.01));
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    struct Case {
        const char* name;
        std::shared_ptr<const Airfoil> airfoil;
        SwashplatePitch pitch;
        Vec3 vel_b;
        Vec3 omega_b;
        double root_cutout;
        double tip_loss;
        ElementSpacing spacing;
    };
    const std::vector<Case> cases{
        {"hover_uniform",
         lin,
         {18.0 * kDeg, 0.0, 0.0},
         Vec3::Zero(),
         Vec3::Zero(),
         1.2,
         1.0,
         ElementSpacing::uniform},
        {"hover_equal_area",
         lin,
         {18.0 * kDeg, 0.0, 0.0},
         Vec3::Zero(),
         Vec3::Zero(),
         1.2,
         1.0,
         ElementSpacing::equal_area},
        {"tip_loss",
         lin,
         {18.0 * kDeg, 0.0, 0.0},
         Vec3::Zero(),
         Vec3::Zero(),
         0.8,
         0.97,
         ElementSpacing::equal_area},
        {"forward",
         lin,
         {14.0 * kDeg, -6.0 * kDeg, 2.5 * kDeg},
         Vec3(42.0, -6.0, 3.0),
         Vec3(0.3, -0.2, 0.15),
         2.4,
         1.0,
         ElementSpacing::uniform},
        {"reverse_stall",
         syn,
         {16.0 * kDeg, -9.0 * kDeg, 1.5 * kDeg},
         Vec3(97.0, 4.0, -2.0),
         Vec3(-0.1, 0.25, 0.05),
         0.6,
         0.97,
         ElementSpacing::uniform},
    };
    std::ofstream out = open_csv(dir / "refinement.csv");
    out << "case,N,error,richardson_error\n";
    for (const Case& c : cases) {
        SimContext ctx = sea_level_context(0.0517);
        ctx.body.vel_b = c.vel_b;
        ctx.body.omega_b = c.omega_b;
        const auto make = [&](int n) {
            BladeElementRotorConfig cfg = test_rotor(c.airfoil, c.pitch, n, c.spacing);
            cfg.root_cutout = c.root_cutout;
            cfg.tip_loss = c.tip_loss;
            return BladeElementRotor(cfg);
        };
        const BladeElementRotor r0 = make(30);
        const RefLoads ref = reference_loads(reference_rotor(r0), reference_condition(r0, ctx));
        RotorLoads prev;
        for (int n = 15; n <= 3840; n *= 2) {
            const RotorLoads l = make(n).evaluate(ctx);
            out << c.name << ',' << n << ',' << max_error(l, ref, 8.0) << ',';
            if (n > 15) {
                out << max_error(prev, l, ref, 8.0);
            }
            out << '\n';
            prev = l;
        }
    }
}

// D-007 layer 2 (and the setup for layer 3): C_T and C_P against collective
// for the four Knight & Hefner solidities (Leishman 2nd ed. Fig. 3.2):
// untwisted, constant chord, C_lα = 5.73/rad (Leishman p. 120), no root
// cutout or tip loss, uniform inflow.
void closed_form(const std::filesystem::path& dir) {
    const double a = 5.73;
    const double cd0 = 0.01;
    const auto lin = airfoil_from(linear_airfoil_csv(a, cd0));
    std::ofstream out = open_csv(dir / "closed_form.csv");
    out << "sigma,theta0_deg,ct,ct_cf,cp,cp_cf,lambda,lambda_cf\n";
    for (const double sigma : {0.042, 0.064, 0.085, 0.106}) {
        for (int i = 0; i <= 30; ++i) {
            const double th = 0.5 * i;
            BladeElementRotorConfig c = test_rotor(lin, {th * kDeg, 0.0, 0.0}, 400);
            const double chord = sigma * std::numbers::pi * c.radius / c.num_blades;
            c.chord = Table1D(Axis({0.0, 1.0}), {chord, chord});
            c.twist = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
            c.root_cutout = 0.0;
            const RotorLoads l = BladeElementRotor(c).evaluate(sea_level_context(0.0));
            // Leishman 2nd ed. Eqs. 3.25 and 3.32 solved together; Eq. 3.31.
            const double k = sigma * a / 4.0;
            const double lam = (-k + std::sqrt(k * k + 16.0 * k * (th * kDeg / 3.0))) / 4.0;
            const double ct = 2.0 * lam * lam;
            out << sigma << ',' << th << ',' << l.ct << ',' << ct << ',' << l.cq << ','
                << lam * ct + sigma * cd0 / 8.0 << ',' << l.lambda_i << ',' << lam << '\n';
        }
    }
}

// D-007 layer 3: C_T against collective for the Knight & Hefner solidities,
// with the same assumptions as Leishman's theory curves in Fig. 3.2 (B = 1),
// and with an effective-radius tip loss B = 0.97 to show its effect. Compared
// with tests/validation/reference/knight_hefner_1937.csv in tools/plot_m3.py.
void knight_hefner(const std::filesystem::path& dir) {
    const auto lin = airfoil_from(linear_airfoil_csv(5.73, 0.01));
    std::ofstream out = open_csv(dir / "knight_hefner.csv");
    out << "sigma,theta0_deg,ct_b100,ct_b097\n";
    for (const double sigma : {0.042, 0.064, 0.085, 0.106}) {
        for (int i = 0; i <= 60; ++i) {
            const double th = 0.25 * i;
            out << sigma << ',' << th;
            for (const double b : {1.0, 0.97}) {
                BladeElementRotorConfig c = test_rotor(lin, {th * kDeg, 0.0, 0.0}, 400);
                const double chord = sigma * std::numbers::pi * c.radius / c.num_blades;
                c.chord = Table1D(Axis({0.0, 1.0}), {chord, chord});
                c.twist = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
                c.root_cutout = 0.0;
                c.tip_loss = b;
                out << ',' << BladeElementRotor(c).evaluate(sea_level_context(0.0)).ct;
            }
            out << '\n';
        }
    }
}

// Spec M3: loads against azimuth in hover (steady) and at μ ≈ 0.2 (4/rev).
void azimuth(const std::filesystem::path& dir) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    // Hover at constant collective (spec M3); forward flight with some
    // forward cyclic.
    const BladeElementRotor hover(test_rotor(syn, {22.0 * kDeg, 0.0, 0.0}, 20));
    const BladeElementRotor fwd(test_rotor(syn, {22.0 * kDeg, -3.0 * kDeg, 0.0}, 20));
    std::ofstream out = open_csv(dir / "azimuth.csv");
    out << "psi_deg,T_hover,Q_hover,T_fwd,Q_fwd,Mx_fwd,My_fwd\n";
    for (int i = 0; i <= 360; ++i) {
        const double t = (i * kDeg) / 27.0;
        SimContext h = sea_level_context(t);
        SimContext f = h;
        f.body.vel_b = Vec3(43.0, 0.0, 0.0);
        const RotorLoads lh = hover.evaluate(h);
        const RotorLoads lf = fwd.evaluate(f);
        out << i << ',' << lh.thrust << ',' << lh.torque << ',' << lf.thrust << ',' << lf.torque
            << ',' << lf.M_h.x() << ',' << lf.M_h.y() << '\n';
    }
}

// Inflow solve across vertical speed, through the vortex-ring region.
void inflow(const std::filesystem::path& dir) {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    const BladeElementRotor rotor(test_rotor(syn, {22.0 * kDeg, 0.0, 0.0}, 20));
    std::ofstream out = open_csv(dir / "inflow.csv");
    out << "descent,lambda_i,mu_z,ct,passes,converged\n";
    for (int i = -40; i <= 120; ++i) {
        SimContext ctx = sea_level_context(0.0);
        ctx.body.vel_b = Vec3(0.0, 0.0, 0.5 * i);
        const RotorLoads l = rotor.evaluate(ctx);
        out << 0.5 * i << ',' << l.lambda_i << ',' << l.mu_z << ',' << l.ct << ',' << l.passes
            << ',' << (l.converged ? 1 : 0) << '\n';
    }
}

// Cost of the rotor (not the D-005 benchmark, which comes in M8): one
// evaluate() and one physics step with the rotor, 4 blades × 20 elements, in
// forward flight. Only meaningful from a Release build.
void timing(const std::filesystem::path& dir) {
    using clock = std::chrono::steady_clock;
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    BladeElementRotorConfig c = test_rotor(syn, {16.0 * kDeg, -4.0 * kDeg, 0.0}, 20);
    c.hub_position_b = Vec3(0.0, 0.0, -2.0);
    const BladeElementRotor rotor(c);
    SimContext ctx = sea_level_context(0.0);
    ctx.body.vel_b = Vec3(50.0, 0.0, 0.0);
    const int n_eval = 20000;
    double sink = 0.0;
    long passes = 0;
    auto t0 = clock::now();
    for (int i = 0; i < n_eval; ++i) {
        ctx.t = 1e-3 * i;
        const RotorLoads l = rotor.evaluate(ctx);
        sink += l.thrust;
        passes += l.passes;
    }
    const double eval_us =
        std::chrono::duration<double, std::micro>(clock::now() - t0).count() / n_eval;

    ExecutiveConfig ec;
    ec.mass = make_mass_properties(7000.0, 6000.0, 40000.0, 37000.0, 1900.0);
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<BladeElementRotor>(c));
    Executive ex(ec, std::move(comps));
    BodyState b;
    b.vel_b = Vec3(50.0, 0.0, 0.0);
    ex.set_body_state(b);
    const int n_steps = 20000;
    t0 = clock::now();
    for (int i = 0; i < n_steps / 20; ++i) {
        test::require_frame(ex.advance(Controls{}, Environment{}, 20'000'000),
                            "m3_cases");  // 20 steps
    }
    const double step_us =
        std::chrono::duration<double, std::micro>(clock::now() - t0).count() / n_steps;

    std::ofstream out = open_csv(dir / "timing.csv");
    out << "evaluate_us,mean_passes,step_us,sink\n";
    out << eval_us << ',' << static_cast<double>(passes) / n_eval << ',' << step_us << ',' << sink
        << '\n';
    std::cout << "evaluate(): " << eval_us << " us (" << static_cast<double>(passes) / n_eval
              << " passes); step(): " << step_us << " us\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m3_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        refinement(dir);
        closed_form(dir);
        knight_hefner(dir);
        azimuth(dir);
        inflow(dir);
        timing(dir);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
