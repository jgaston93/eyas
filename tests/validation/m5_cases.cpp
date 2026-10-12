// Writes the M5 cases to CSV for the VALIDATION.md plots (tools/plot_m5.py):
// the Carpenter & Fridovich (NACA TN 3044, 1953) collective ramps on a
// full-scale rotor in hover, through the production rotor with Pitt–Peters
// inflow (D-027), against tests/validation/reference/carpenter_fridovich_1953.csv.
//
// Rotor B of NACA TN 1698 (p. 5): 3 blades, R = 19 ft, σ = 0.042, NACA 23015.
// Assumptions (VALIDATION.md, DATA_GAPS.md):
// - constant chord from σ (the planform taper is not given), no root cutout,
//   tip loss B = 0.97 (TN 3044 p. 13 integrates thrust to 0.97R);
// - linear airfoil, no drag, lift slope 0.105–0.110 /deg from NACA Report
//   824 p. 147 (NACA 23015; reading confirmed by human review);
// - twist 0° (TN 3044 p. 4) or −8° linear (TN 1698 p. 5), with the pitch
//   taken at 0.75R for the twisted blade (the reference station isn't given);
// - massless blades: the flap inertia isn't given, and the ramps used here are
//   the slow ones, where TN 3044 p. 14 puts blade inertia at a few percent of
//   thrust; the flap hinge is on the shaft axis, so coning doesn't change the
//   hover thrust;
// - Ω constant at the case's initial speed (Eyas has no rotor-speed state yet);
// - sea-level ISA (C_T doesn't depend on density for a linear airfoil).
//
// Usage: eyas_m5_cases <output-dir>

#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "rotor/blade_element_rotor.hpp"
#include "rotor/controls.hpp"
#include "support/m3_fixtures.hpp"

namespace {

using namespace eyas;
using namespace eyas::test;

constexpr double kFt = 0.3048;
constexpr double kRpm = 2.0 * std::numbers::pi / 60.0;
constexpr double kDegRad = std::numbers::pi / 180.0;

std::ofstream open_csv(const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path.string());
    }
    out.precision(17);
    return out;
}

// A TN 3044 case: the measured pitch ramp (piecewise linear) and initial Ω.
struct Ramp {
    const char* name;
    std::vector<std::array<double, 2>> theta;  // (t [s], θ [deg])
    double rpm;
    double t_end;  // [s]

    double operator()(double t) const {
        if (t <= theta.front()[0]) {
            return theta.front()[1];
        }
        for (std::size_t i = 1; i < theta.size(); ++i) {
            if (t <= theta[i][0]) {
                const double f = (t - theta[i - 1][0]) / (theta[i][0] - theta[i - 1][0]);
                return theta[i - 1][1] + f * (theta[i][1] - theta[i - 1][1]);
            }
        }
        return theta.back()[1];
    }
};

// Readings confirmed by human review, as in the reference CSV.
std::vector<Ramp> ramps() {
    return {
        {"b", {{0.0, 0.0}, {0.1, 5.0}, {0.2, 9.5}, {0.25, 12.0}}, 220.0, 1.5},
        {"c", {{0.0, 0.0}, {0.6, 12.0}}, 217.0, 1.5},
        {"d", {{0.0, 0.0}, {2.05, 12.0}}, 230.0, 3.0},
    };
}

// θ0 = collective input [rad] (gain 1 rad per unit input).
BladeElementRotorConfig rotor_b(double lift_slope_per_deg, double twist_deg, double rpm,
                                InflowModel inflow) {
    BladeElementRotorConfig c;
    c.num_blades = 3;
    c.radius = 19.0 * kFt;
    c.rotor_speed = rpm * kRpm;
    c.root_cutout = 0.0;
    c.tip_loss = 0.97;
    const double chord = 0.042 * std::numbers::pi * c.radius / 3.0;  // from σ = N c/(πR)
    c.chord = Table1D(Axis({0.0, 1.0}), {chord, chord});
    // Linear washout, zero at 0.75R.
    c.twist = Table1D(Axis({0.0, 1.0}), {-0.75 * twist_deg * kDegRad, 0.25 * twist_deg * kDegRad});
    c.num_elements = 40;
    c.spacing = ElementSpacing::uniform;
    c.airfoil = airfoil_from(linear_airfoil_csv(lift_slope_per_deg / kDegRad, 0.0));
    c.controls = std::make_shared<const LinearControlMap>(LinearPitchRow{0.0, 1.0, 0.0, 0.0, 0.0},
                                                          LinearPitchRow{}, LinearPitchRow{});
    c.inflow = inflow;
    return c;
}

// Time history at 1 kHz, RK4, with the pitch evaluated at each stage time.
// The hub is held fixed (tower). Writes t, θ, C_T, λ0.
void run(std::ofstream& out, const Ramp& ramp, double a, double twist, InflowModel inflow) {
    const BladeElementRotor rotor(rotor_b(a, twist, ramp.rpm, inflow));
    SimContext ctx = sea_level_context(0.0);
    const auto at = [&](double t) {
        SimContext c = ctx;
        c.t = t;
        c.controls.collective = ramp(t) * kDegRad;
        return c;
    };
    const char* model = inflow == InflowModel::pitt_peters ? "pitt_peters" : "uniform";
    const auto write = [&](double t, std::span<const double> x) {
        const RotorLoads l = rotor.evaluate(at(t), x);
        out << ramp.name << ',' << model << ',' << a << ',' << twist << ',' << t << ',' << ramp(t)
            << ',' << l.ct << ',' << l.lambda_i << '\n';
    };
    const double h = 1e-3;
    const int steps = static_cast<int>(std::lround(ramp.t_end / h));
    const std::size_t n = rotor.num_states();
    std::vector<double> x(n, 0.0);  // no inflow at θ = 0, as in the test
    std::vector<double> xd(n);
    std::vector<double> G(n * 6);
    const auto f = [&](double t, const std::vector<double>& xs, std::vector<double>& k) {
        ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
        rotor.derivatives(at(t), xs, d);
        k = xd;
    };
    std::vector<double> k1(n);
    std::vector<double> k2(n);
    std::vector<double> k3(n);
    std::vector<double> k4(n);
    std::vector<double> tmp(n);
    write(0.0, x);
    for (int s = 0; s < steps; ++s) {
        const double t = s * h;
        if (n > 0) {
            f(t, x, k1);
            for (std::size_t i = 0; i < n; ++i) {
                tmp[i] = x[i] + 0.5 * h * k1[i];
            }
            f(t + 0.5 * h, tmp, k2);
            for (std::size_t i = 0; i < n; ++i) {
                tmp[i] = x[i] + 0.5 * h * k2[i];
            }
            f(t + 0.5 * h, tmp, k3);
            for (std::size_t i = 0; i < n; ++i) {
                tmp[i] = x[i] + h * k3[i];
            }
            f(t + h, tmp, k4);
            for (std::size_t i = 0; i < n; ++i) {
                x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
        }
        write((s + 1) * h, x);
    }
}

void collective_ramps(const std::filesystem::path& dir) {
    std::ofstream out = open_csv(dir / "ramps.csv");
    out << "case,model,a_per_deg,twist_deg,t,theta_deg,ct,lambda0\n";
    for (const Ramp& r : ramps()) {
        for (const double twist : {0.0, -8.0}) {
            for (const double a : {0.105, 0.1075, 0.110}) {
                run(out, r, a, twist, InflowModel::pitt_peters);
            }
        }
        run(out, r, 0.1075, 0.0, InflowModel::uniform);
    }
}

// D-027 layer 2, plotted: a small collective step on the closed-form
// fixture of tests/unit/test_pitt_peters.cpp (StepCollectiveOvershoot), hub
// fixed: C_T jumps by (σa/6)δθ0, then relaxes at rate s0 to C′ times the jump
// (Johnson 2013 Eqs. 11.148–11.149).
void step_anchor(const std::filesystem::path& dir) {
    const double a = 5.73;
    const auto make = [&](double theta0) {
        BladeElementRotorConfig c =
            test_rotor(airfoil_from(linear_airfoil_csv(a, 0.0)), {theta0, 0.0, 0.0}, 200);
        c.root_cutout = 0.0;
        c.twist = Table1D(Axis({0.0, 1.0}), {0.0, -8.0 * kDegRad});
        c.inflow = InflowModel::pitt_peters;
        return c;
    };
    const double theta0 = 12.0 * kDegRad;
    const double step = 0.25 * kDegRad;
    const BladeElementRotor before(make(theta0));
    const BladeElementRotor after(make(theta0 + step));
    const SimContext ctx = sea_level_context();
    const double l0 = before.evaluate_uniform(ctx).lambda_i;
    std::vector<double> x{l0, 0.0, 0.0};
    const double ct0 = before.evaluate(ctx, x).ct;
    const BladeElementRotorConfig& c = after.config();
    const double sa = c.num_blades * c.chord(0.5) / (std::numbers::pi * c.radius) * a;
    const double W = c.rotor_speed;
    const double jump = sa / 6.0 * step;
    const double cprime = 1.0 / (1.0 + sa / (16.0 * l0));
    const double s0 = -(W / (128.0 / (75.0 * std::numbers::pi))) * (4.0 * l0 + sa / 4.0);

    std::ofstream out = open_csv(dir / "step_anchor.csv");
    out << "t,dct,closed_form\n";
    const auto closed = [&](double t) {
        return jump * (cprime + (1.0 - cprime) * std::exp(s0 * t));
    };
    const double h = 1e-4;
    std::vector<double> xd(3);
    std::vector<double> G(18);
    std::vector<double> k1(3);
    std::vector<double> k2(3);
    std::vector<double> k3(3);
    std::vector<double> k4(3);
    std::vector<double> tmp(3);
    const auto f = [&](const std::vector<double>& xs, std::vector<double>& k) {
        ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
        after.derivatives(ctx, xs, d);
        k = xd;
    };
    for (int s = 0; s <= 3000; ++s) {
        if (s % 10 == 0) {
            const double t = s * h;
            out << t << ',' << after.evaluate(ctx, x).ct - ct0 << ',' << closed(t) << '\n';
        }
        f(x, k1);
        for (std::size_t i = 0; i < 3; ++i) {
            tmp[i] = x[i] + 0.5 * h * k1[i];
        }
        f(tmp, k2);
        for (std::size_t i = 0; i < 3; ++i) {
            tmp[i] = x[i] + 0.5 * h * k2[i];
        }
        f(tmp, k3);
        for (std::size_t i = 0; i < 3; ++i) {
            tmp[i] = x[i] + h * k3[i];
        }
        f(tmp, k4);
        for (std::size_t i = 0; i < 3; ++i) {
            x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m5_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        collective_ramps(dir);
        step_anchor(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m5_cases: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
