// Writes the M6a cases to CSV for the VALIDATION.md plots (tools/plot_m6.py):
// - the Bailey tail-rotor thrust against the production blade-element rotor
//   and the small-angle integrals, over advance ratio and over inflow
//   (D-028 layer 2; the residual study behind the gate tolerances);
// - a tail-rotor pedal step in hover: Eyas's inflow state (D-028) against
//   GenHel's previous-pass inflow and δ₃ (CR-166309 p. 5.4-7) run at 1 kHz and
//   at 60 Hz, showing that GenHel's lag depends on the frame rate.
//
// The tail rotor is GenHel's UH-60A tail rotor (p. 5.4-14) at sea level. The
// anchor cases use the bare rotor (no δ₃, blockage or bias) and the
// thin-plate test airfoil (tests/support/m6_fixtures.hpp).
//
// Usage: eyas_m6_cases <output-dir>

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "rotor/bailey_rotor.hpp"
#include "support/m6_fixtures.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;
using namespace eyas::test;
using units::kDegree;

std::ofstream open_csv(const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path.string());
    }
    out.precision(17);
    return out;
}

double small_angle_ct(const BaileyRotorConfig& b, double th, double lam_i, double mu,
                      bool reverse) {
    const ThrustIntegrals t = thrust_integrals(b.tip_loss, mu, reverse);
    const double G = b.lift_slope * BaileyRotor(b).solidity() / 2.0;
    return G * (th * t.theta + b.twist * t.twist - lam_i * t.inflow);
}

void anchor_cases(const std::filesystem::path& dir) {
    const BaileyRotorConfig b = bare(0.92, -18.0 * kDegree);
    const double th = 20.0 * kDegree;
    {
        std::ofstream out = open_csv(dir / "m6a_anchor_mu.csv");
        out << "mu,ct_blade_element,ct_bailey,ct_small_angle\n";
        const double lam_i = 0.05;
        for (int i = 0; i <= 14; ++i) {
            const double mu = 0.025 * i;
            out << mu << "," << blade_element_ct(b, th, lam_i, mu) << ","
                << bailey_ct(b, th, lam_i, mu) << "," << small_angle_ct(b, th, lam_i, mu, true)
                << "\n";
        }
    }
    {
        std::ofstream out = open_csv(dir / "m6a_anchor_lambda.csv");
        out << "lambda_i,ct_blade_element,ct_bailey\n";
        for (int i = 1; i <= 10; ++i) {
            const double lam_i = 0.01 * i;
            out << lam_i << "," << blade_element_ct(b, th, lam_i, 0.0) << ","
                << bailey_ct(b, th, lam_i, 0.0) << "\n";
        }
    }
}

// GenHel's per-pass scheme for one frame: inflow and δ₃ from the previous
// pass (p. 5.4-7), hover, no blockage.
struct GenHelPass {
    double lambda_prev = 0.0;
    double t_prev = 0.0;
};

double genhel_frame(const GenHelTailRotorInput& in, GenHelPass& s) {
    const double d2r = kDegree;
    const double B = in.BTLTR;
    const double G = in.ATR / 2.0 * (in.BTR * in.CHRDTR / (3.141592653589793 * in.RTR));
    const double t31 = B * B / 2.0;
    const double t32 = B * B * B / 3.0;
    const double t33 = B * B * B * B / 4.0;
    const double theta =
        d2r * (in.THETTR - s.t_prev * in.DELTTR * std::tan(in.DEL3TR * d2r) + in.BIASTR);
    const double dw =
        G * (theta * t32 + in.TWSTTR * d2r * t33) / (2.0 * std::abs(s.lambda_prev) + G * t31);
    const double lambda = -dw;
    const double thrust = 2.0 * in.RHO * 3.141592653589793 * std::pow(in.RTR, 4) * dw *
                          std::abs(lambda) * in.OMEGTR * in.OMEGTR;
    s.lambda_prev = lambda;
    s.t_prev = thrust;
    return thrust;  // [lb]
}

void pedal_step(const std::filesystem::path& dir) {
    GenHelTailRotorInput in = genhel_uh60a_tail_rotor();
    in.KBVTTR = 1.0;              // no blockage in hover for this comparison
    const double theta_a = 16.0;  // [deg] commanded; hover thrust ≈ 1600–2000 lbf
    const double theta_b = 18.0;

    BaileyRotorConfig c = bailey_from_genhel(in);
    c.blockage_speed = 0.0;
    const BaileyRotor rotor(c);
    SimContext ctx;
    ctx.air.density = in.RHO * units::kSlug / std::pow(units::kFoot, 3);

    // Steady start at θ_a, both models.
    in.THETTR = theta_a;
    const GenHelTailRotorOutput start = genhel_tail_rotor(in);
    double lam = start.DWSHTR;

    std::ofstream out = open_csv(dir / "m6a_pedal_step.csv");
    out << "model,t,thrust_lbf\n";
    // Eyas: RK4 on the inflow state at 1 kHz, step at t = 0.
    ctx.controls.pedal = theta_b * kDegree;
    const double h = 1e-3;
    const auto f = [&](double l) { return rotor.evaluate(ctx, l, Vec3::Zero()).lambda_dot; };
    for (int n = 0; n <= 100; ++n) {
        const double thrust = rotor.evaluate(ctx, lam, Vec3::Zero()).thrust / units::kPoundForce;
        out << "eyas," << n * h << "," << thrust << "\n";
        const double k1 = f(lam);
        const double k2 = f(lam + 0.5 * h * k1);
        const double k3 = f(lam + 0.5 * h * k2);
        const double k4 = f(lam + h * k3);
        lam += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    // GenHel: one pass per frame, at 1 kHz and at 60 Hz.
    for (const double frame : {1e-3, 1.0 / 60.0}) {
        GenHelPass s{-start.DWSHTR, start.TTR};
        in.THETTR = theta_b;
        const char* name = frame < 0.01 ? "genhel_1khz" : "genhel_60hz";
        for (int n = 0; n * frame <= 0.1 + 1e-12; ++n) {
            out << name << "," << n * frame << "," << genhel_frame(in, s) << "\n";
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m6_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        anchor_cases(dir);
        pedal_step(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m6_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
