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
// And the M6b cases (D-028, "Check (M6b)" and "Data rulings"):
// - the lag that stands in for GenHel's transport delay of the fuselage
//   downwash at the stabilator, against the exact delay, in a sinusoidal
//   heave from 20 to 150 kt and 0.5 to 10 rad/s: the delayed downwash and
//   the resulting stabilator lift, both through the production airframe;
// - the sensitivity of the stabilator to a lag on GenHel's undefined
//   AA1FMR: the lift per degree of a₁ against the lift per degree of angle
//   of attack, and the gain and phase a first-order lag of time constant τ
//   on a₁ would add.
// The airframe is the UH-60A's (GenHel stations, tables and stabilator
// schedule) at sea level, collective 50 %.
//
// Usage: eyas_m6_cases <output-dir>

#include <algorithm>
#include <cmath>
#include <complex>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/rotor_wash.hpp"
#include "support/m6_fixtures.hpp"
#include "support/m6b_fixtures.hpp"
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

// ---- M6b ------------------------------------------------------------------------

using Complex = std::complex<double>;

AirframeAeroConfig uh60a_airframe_scheduled() {
    AirframeAeroConfig c = uh60a_airframe(uh60a_geometry());
    c.stabilator.schedule = uh60a_stabilator_schedule();
    return c;
}

SimContext sea_level() {
    SimContext ctx;
    ctx.air.density = 1.225;
    ctx.air.speed_of_sound = 340.3;
    ctx.controls.collective = 0.5;
    return ctx;
}

// Complex amplitude at ω of a periodic signal sampled over whole periods.
struct Fourier {
    double omega;
    Complex sum{0.0, 0.0};
    void add(double t, double y, double h) { sum += y * std::exp(Complex(0.0, -omega * t)) * h; }
    Complex amplitude(double span) const { return 2.0 * sum / span; }
};

struct HeaveResponse {
    Complex downwash;  // ε_d [rad]
    Complex lift;      // stabilator lift, −F_z [N]
};

// Heave w(t) = w0 sin ωt at body speed u: the stabilator's delayed downwash
// and lift, with the production lag state (RK4) or with the exact delay
// ε_d(t) = ε(t − l/u) fed in as the state.
HeaveResponse heave(const AirframeAero& aero, double u, double omega, bool exact) {
    const double l = -aero.config().stabilator.position_b.x();
    const double tau = l / u;
    const double w0 = 0.02 * u;  // ±1.1° angle of attack: linear range
    const double period = 2.0 * std::numbers::pi / omega;
    const double h_max = std::min(2e-3, period / 400.0);
    const int n_meas = static_cast<int>(std::ceil(3.0 * period / h_max));
    const double h = 3.0 * period / n_meas;
    const int n_settle = static_cast<int>(std::ceil(std::max(12.0 * tau, period) / h));
    const SimContext base = sea_level();
    const auto at = [&](double t) {
        SimContext c = base;
        c.body.vel_b = Vec3(u, 0.0, w0 * std::sin(omega * t));
        return c;
    };
    const Vec3 zero = Vec3::Zero();
    const auto undelayed = [&](double t) {
        const std::vector<double> x0 = {0.0, 0.0};
        return aero.evaluate(at(t), x0, zero, zero).downwash;
    };
    std::vector<double> x = {undelayed(0.0), 0.0};
    const auto rate = [&](double t, double e) {
        const std::vector<double> xs = {e, 0.0};
        return aero.evaluate(at(t), xs, zero, zero).downwash_rate;
    };
    Fourier fd{omega};
    Fourier fl{omega};
    const int n_total = n_settle + n_meas;
    for (int n = 0; n < n_total; ++n) {
        const double t = n * h;
        if (exact) {
            x[0] = undelayed(t - tau);
        }
        if (n >= n_settle) {
            const double lift = -aero.evaluate(at(t), x, zero, zero).stabilator.F.z();
            fd.add(t, x[0], h);
            fl.add(t, lift, h);
        }
        if (!exact) {
            const double k1 = rate(t, x[0]);
            const double k2 = rate(t + 0.5 * h, x[0] + 0.5 * h * k1);
            const double k3 = rate(t + 0.5 * h, x[0] + 0.5 * h * k2);
            const double k4 = rate(t + h, x[0] + h * k3);
            x[0] += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        }
    }
    const double span = n_meas * h;
    return {fd.amplitude(span), fl.amplitude(span)};
}

double phase_deg(Complex z) {
    return std::arg(z) / kDegree;
}

void lag_vs_delay(const std::filesystem::path& dir) {
    const AirframeAero aero(uh60a_airframe_scheduled());
    const double l = -aero.config().stabilator.position_b.x();
    std::ofstream out = open_csv(dir / "m6b_lag_vs_delay.csv");
    out << "speed_kt,omega,tau,wash_gain,wash_phase_deg,wash_gain_analytic,"
           "wash_phase_deg_analytic,lift_gain,lift_phase_deg\n";
    for (int kt = 20; kt <= 150; kt += 10) {
        const double u = kt * units::kKnot;
        for (int k = 0; k <= 20; ++k) {
            const double omega = 0.5 * std::pow(20.0, k / 20.0);  // 0.5 to 10 rad/s
            const HeaveResponse lag = heave(aero, u, omega, false);
            const HeaveResponse del = heave(aero, u, omega, true);
            const Complex rw = lag.downwash / del.downwash;
            const Complex rl = lag.lift / del.lift;
            const double tau = l / u;
            // Analytic: (1/(1 + jωτ)) / e^{−jωτ}.
            const Complex ra = std::exp(Complex(0.0, omega * tau)) / Complex(1.0, omega * tau);
            out << kt << ',' << omega << ',' << tau << ',' << std::abs(rw) << ',' << phase_deg(rw)
                << ',' << std::abs(ra) << ',' << phase_deg(ra) << ',' << std::abs(rl) << ','
                << phase_deg(rl) << '\n';
        }
    }
}

// Uniform inflow of the UH-60A main rotor in level flight at μ (momentum
// theory, Glauert, μ_z = 0): an operating point for the wash maps.
double momentum_inflow(double ct, double mu) {
    double lam = std::sqrt(ct / 2.0);
    for (int i = 0; i < 200; ++i) {
        lam = ct / (2.0 * std::sqrt(mu * mu + lam * lam));
    }
    return lam;
}

void a1_sensitivity(const std::filesystem::path& dir) {
    const AirframeAero aero(uh60a_airframe_scheduled());
    const RotorWash fus = uh60a_wash("fuselage_wash.csv");
    const RotorWash tail = uh60a_wash("tail_wash.csv");
    const double tip = 27.0 * 26.83 * units::kFoot;  // GenHel p. 5.1-49
    const double area = std::numbers::pi * std::pow(26.83 * units::kFoot, 2);
    const double ct = 16638.0 * units::kPoundForce / (1.225 * area * tip * tip);
    const double a10 = 2.0 * kDegree;

    // Lift at a₁ (fed to both wash maps) and heave w, lag states at equilibrium.
    const auto lift = [&](double u, double lam, double a1, double w) {
        SimContext c = sea_level();
        c.body.vel_b = Vec3(u, 0.0, w);
        const double mu = u / tip;
        const Vec3 wf = fus.evaluate(lam, mu, 0.0, -a1, tip).velocity_b;
        const Vec3 wt = tail.evaluate(lam, mu, 0.0, -a1, tip).velocity_b;
        std::vector<double> x = {0.0, 0.0};
        const AirframeAeroLoads l0 = aero.evaluate(c, x, wf, wt);
        x = {l0.downwash, l0.sidewash};
        return -aero.evaluate(c, x, wf, wt).stabilator.F.z();
    };
    {
        std::ofstream out = open_csv(dir / "m6b_a1_derivatives.csv");
        out << "speed_kt,mu,lambda0,chi_deg,dlift_da1,dlift_dalpha,ratio\n";
        for (int kt = 20; kt <= 150; kt += 10) {
            const double u = kt * units::kKnot;
            const double mu = u / tip;
            const double lam = momentum_inflow(ct, mu);
            const double d = 0.25 * kDegree;
            const double da1 = (lift(u, lam, a10 + d, 0.0) - lift(u, lam, a10 - d, 0.0)) / 0.5;
            const double dal =
                (lift(u, lam, a10, u * std::tan(d)) - lift(u, lam, a10, -u * std::tan(d))) / 0.5;
            const double chi = (std::atan2(mu, lam) + a10) / kDegree;
            out << kt << ',' << mu << ',' << lam << ',' << chi << ',' << da1 << ',' << dal << ','
                << da1 / dal << '\n';
        }
    }
    // The lag itself, through the production path at 100 kt: a₁ = a₁₀ +
    // 0.5° sin ωt through dã/dt = (a₁ − ã)/τ, ã fed to both wash maps.
    // The airframe is linear in ã here, so the ratio to τ = 0 must be
    // 1/(1 + jωτ).
    std::ofstream out = open_csv(dir / "m6b_a1_lag.csv");
    out << "tau,omega,gain,phase_deg,gain_analytic,phase_deg_analytic\n";
    const double u = 100.0 * units::kKnot;
    const double lam = momentum_inflow(ct, u / tip);
    for (const double tau : {0.0175, 0.05, 0.3}) {
        for (const double omega : {0.5, 1.0, 2.0, 3.0, 5.0, 10.0}) {
            const auto response = [&](double ta) {
                const double period = 2.0 * std::numbers::pi / omega;
                const int n_meas = static_cast<int>(std::ceil(3.0 * period / 1e-3));
                const double h = 3.0 * period / n_meas;
                const int n_settle = static_cast<int>(std::ceil(12.0 * std::max(ta, 0.1) / h));
                double af = a10;
                Fourier f{omega};
                const auto a1 = [&](double t) { return a10 + 0.5 * kDegree * std::sin(omega * t); };
                for (int n = 0; n < n_settle + n_meas; ++n) {
                    const double t = n * h;
                    const double used = ta > 0.0 ? af : a1(t);
                    if (n >= n_settle) {
                        f.add(t, lift(u, lam, used, 0.0), h);
                    }
                    if (ta > 0.0) {
                        const auto r = [&](double tt, double a) { return (a1(tt) - a) / ta; };
                        const double k1 = r(t, af);
                        const double k2 = r(t + 0.5 * h, af + 0.5 * h * k1);
                        const double k3 = r(t + 0.5 * h, af + 0.5 * h * k2);
                        const double k4 = r(t + h, af + h * k3);
                        af += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
                    }
                }
                return f.amplitude(n_meas * h);
            };
            const Complex ratio = response(tau) / response(0.0);
            const Complex analytic = 1.0 / Complex(1.0, omega * tau);
            out << tau << ',' << omega << ',' << std::abs(ratio) << ',' << phase_deg(ratio) << ','
                << std::abs(analytic) << ',' << phase_deg(analytic) << '\n';
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
        lag_vs_delay(dir);
        a1_sensitivity(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m6_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
