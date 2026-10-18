// Writes the M7b linearization cases to CSV for VALIDATION.md
// (tools/plot_m7b.py; D-035 and its amendment, D-036):
// - the UH-60A from 0 to 150 kt every 10 kt (ISA sea level, 16 638 lbm): the
//   condensed model's modes and derivatives, the averaged model against the
//   Floquet exponents of the free flow map, the multiblade Jacobians'
//   variation around the revolution, and the L3 comparison with the
//   nonlinear model;
// - the Floquet-to-averaged gap against the periodic part scaled by s;
// - the step-size study;
// - the held-body linearization against one about the free (vibrating)
//   flight;
// - the L4 fixtures against Padfield's closed forms.
//
// Usage: eyas_m7b_cases <output-dir>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "env/atmosphere.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7_fixtures.hpp"
#include "support/m7b_support.hpp"

namespace {

using namespace eyas;
using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;
using CVec = Eigen::VectorXcd;

constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kBand = 3.0;  // [rad/s] the flight-dynamic band compared

const std::array<const char*, 6> kRows = {"X", "Y", "Z", "L", "M", "N"};
const std::array<const char*, 9> kCols = {"u", "v", "w", "p", "q", "r", "phi", "theta", "psi"};
const std::array<const char*, 4> kInputs = {"col", "lat", "lon", "ped"};

// Throws if any evaluation behind a result had an unconverged iteration
// (D-023): such a result would go into VALIDATION.md unflagged.
void require_converged(std::size_t unconverged, const std::string& what, int kt) {
    if (unconverged != 0) {
        throw std::runtime_error(what + ": " + std::to_string(unconverged) +
                                 " unconverged evaluations at " + std::to_string(kt) + " kt");
    }
}

// The nearest element of `set` to z.
std::complex<double> nearest(const CVec& set, std::complex<double> z) {
    std::complex<double> best = set(0);
    for (const auto& w : set) {
        if (std::abs(w - z) < std::abs(best - z)) {
            best = w;
        }
    }
    return best;
}

// The largest Floquet-to-averaged gap in the flight-dynamic band for the
// periodic model with its periodic part scaled by s, the monodromy built
// from the samples by midpoint matrix exponentials (pairs of samples).
double scaled_gap(const LinearModel& m, double s) {
    const auto J = static_cast<int>(m.A_periodic.size());
    const auto n = m.A.rows();
    MatX phi = MatX::Identity(n, n);
    for (int i = 0; i < J / 2; ++i) {
        const MatX A = m.A + s * (m.A_periodic[static_cast<std::size_t>(2 * i + 1)] - m.A);
        phi = (A * (2.0 * m.period / J)).exp() * phi;
    }
    const CVec lam = m.A.eigenvalues();
    const FloquetModes f = floquet(phi, m.period, lam);
    double gap = 0.0;
    for (Eigen::Index i = 0; i < f.exponents.size(); ++i) {
        const auto k = f.match[static_cast<std::size_t>(i)];
        if (k >= 0 && std::abs(lam(k)) < kBand) {
            gap = std::max(gap, std::abs(f.exponents(i) - lam(k)));
        }
    }
    return gap;
}

// The averaged linear model about the free flight `traj` (sampled every
// `every` steps of dt from step `first`, J samples) instead of the held
// orbit, with linearize()'s steps.
LinearModel around_free(const Executive& ex, const std::vector<std::vector<double>>& traj,
                        int first, int J, int every, double dt, const Controls& u,
                        const Environment& env, const LinearModel& like) {
    const ReducedCoordinates rc(ex, like.body_size == 10);
    LinearModel m = like;
    m.unconverged = 0;
    m.A.setZero();
    m.B.setZero();
    m.A_periodic.clear();
    m.B_periodic.clear();
    std::vector<double> x(ex.num_states());
    std::vector<double> xd(ex.num_states());
    const auto n = static_cast<Eigen::Index>(rc.size());
    const LinearizeOptions o;
    const std::vector<double> steps = perturbation_steps(rc, o);
    for (int j = 0; j < J; ++j) {
        const int s = first + j * every;
        const double t = s * dt;
        const std::vector<double>& ref = traj[static_cast<std::size_t>(s)];
        const VecX x0 = rc.to_reduced(t, ref, ref);
        const auto f = [&](const VecX& xr, const Controls& c) {
            rc.to_full(t, xr, ref, x);
            if (!ex.derivatives(t, x, c, env, xd)) {
                ++m.unconverged;
            }
            return VecX(rc.rates(t, x, xd));
        };
        MatX A(n, n);
        MatX B(n, 4);
        for (Eigen::Index k = 0; k < n; ++k) {
            const double h = steps[static_cast<std::size_t>(k)];
            VecX xp = x0;
            VecX xm = x0;
            xp(k) += h;
            xm(k) -= h;
            A.col(k) = (f(xp, u) - f(xm, u)) / (2.0 * h);
        }
        for (int k = 0; k < 4; ++k) {
            Controls up = u;
            Controls um = u;
            const std::array<double*, 4> pp{&up.collective, &up.lat_cyclic, &up.lon_cyclic,
                                            &up.pedal};
            const std::array<double*, 4> pm{&um.collective, &um.lat_cyclic, &um.lon_cyclic,
                                            &um.pedal};
            *pp[static_cast<std::size_t>(k)] += o.step_control;
            *pm[static_cast<std::size_t>(k)] -= o.step_control;
            B.col(k) = (f(x0, up) - f(x0, um)) / (2.0 * o.step_control);
        }
        m.A += A / J;
        m.B += B / J;
    }
    return m;
}

double derivative(const CondensedModel& cm, int row, int col) {
    return col < 9 ? cm.A(row, col) : cm.B(row, col - 9);
}

std::string derivative_name(int row, int col) {
    return std::string(kRows[static_cast<std::size_t>(row)]) + "_" +
           (col < 9 ? kCols[static_cast<std::size_t>(col)]
                    : kInputs[static_cast<std::size_t>(col - 9)]);
}

// The key derivatives reported in the step-size and held/free studies.
const std::array<std::array<int, 2>, 14> kKey = {{{0, 0},
                                                  {2, 2},
                                                  {4, 4},
                                                  {3, 3},
                                                  {5, 5},
                                                  {4, 0},
                                                  {4, 2},
                                                  {3, 1},
                                                  {5, 1},
                                                  {1, 1},
                                                  {2, 9},
                                                  {4, 11},
                                                  {3, 10},
                                                  {5, 12}}};

void write_uh60a(const std::filesystem::path& dir) {
    const test::Uh60aHarness ac;
    const Executive& ex = ac.executive();
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const double tip_speed = mr.config().rotor_speed * mr.config().radius;

    std::ofstream modes_csv(dir / "m7b_modes.csv");
    modes_csv.precision(12);
    modes_csv << "# UH-60A condensed 6-DOF modes, ISA sea level, 16638 lbm (M7b, D-035)\n"
              << "knots,label,re,im,wn,zeta,longitudinal\n";
    std::ofstream der_csv(dir / "m7b_derivatives.csv");
    der_csv.precision(12);
    der_csv << "# UH-60A condensed derivatives (Padfield App. 4B form: gravity and kinematics "
               "included; L, N primed), SI, controls per unit of Controls (M7b)\nknots";
    for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 13; ++c) {
            der_csv << "," << derivative_name(r, c);
        }
    }
    der_csv << "\n";
    std::ofstream floq_csv(dir / "m7b_floquet.csv");
    floq_csv.precision(12);
    floq_csv << "# Averaged model against Floquet exponents of the free flow map, |lambda| < 3 "
                "rad/s (M7b, D-035 L2)\nknots,mu,avg_re,avg_im,floq_re,floq_im,gap\n";
    std::ofstream per_csv(dir / "m7b_periodicity.csv");
    per_csv.precision(12);
    per_csv << "# Variation of the multiblade Jacobian around the revolution, body rows x (body "
               "+ flap) columns, J = 64 (M7b)\nknots,mu,variation,variation_without_beta_d\n";
    std::ofstream l3_csv(dir / "m7b_l3.csv");
    l3_csv.precision(12);
    l3_csv << "# L3: relative error fit c0 + c1 a against the nonlinear response, periodic and "
              "averaged models, steps T/256 to 60 kt, T/512 above (M7b)\n"
           << "knots,state,control,c0_periodic,c1a_periodic,misfit_periodic,c0_averaged,"
              "c1a_averaged,misfit_averaged\n";
    std::ofstream hf_csv(dir / "m7b_heldfree.csv");
    hf_csv.precision(12);
    hf_csv << "# Held-orbit linear model against one about the free flight (first and third "
              "revolution) (M7b)\nknots,quantity,held,free1,free3\n";
    std::ofstream ss_csv(dir / "m7b_sscaling.csv");
    ss_csv.precision(12);
    ss_csv << "# Floquet-to-averaged gap with the periodic part scaled by s, J = 1024 (M7b)\n"
           << "knots,mu,s,gap\n";
    std::ofstream step_csv(dir / "m7b_stepsize.csv");
    step_csv.precision(12);
    step_csv << "# Step-size study: all central-difference steps scaled by factor (M7b)\n"
             << "knots,factor,quantity,value\n";

    TrimResult previous;
    for (int kt = 0; kt <= 150; kt += 10) {
        TrimCondition c;
        c.airspeed = kt * kKnot;
        std::vector<double> start;
        const TrimResult r = solve_trim(ex, c, kt == 0 ? nullptr : &previous, start);
        previous = r;
        if (!r.converged) {
            throw std::runtime_error("trim did not converge at " + std::to_string(kt) + " kt");
        }
        const double mu = c.airspeed / tip_speed;
        const LinearModel m = linearize(ex, c, r);
        require_converged(m.unconverged, "linearize", kt);
        const CondensedModel cm = condense(m);
        std::cerr << kt << " kt\n";

        for (const Mode& md : modes(cm, r.vel_b.x())) {
            modes_csv << kt << "," << md.label << "," << md.eigenvalue.real() << ","
                      << md.eigenvalue.imag() << "," << md.frequency << "," << md.damping << ","
                      << md.longitudinal << "\n";
        }
        der_csv << kt;
        for (int row = 0; row < 6; ++row) {
            for (int col = 0; col < 13; ++col) {
                der_csv << "," << derivative(cm, row, col);
            }
        }
        der_csv << "\n";

        // Periodicity of the multiblade Jacobian.
        double var = 0.0;
        double var_nd = 0.0;
        const auto beta_d = [&](Eigen::Index k) {
            return m.states[static_cast<std::size_t>(k)].find("beta_d") != std::string::npos;
        };
        MatX Mbar = m.A.topLeftCorner(10, 18);
        MatX Mbar_nd = Mbar;
        for (Eigen::Index k = 0; k < 18; ++k) {
            if (beta_d(k)) {
                Mbar_nd.col(k).setZero();
            }
        }
        for (const MatX& A : m.A_periodic) {
            MatX D = A.topLeftCorner(10, 18) - Mbar;
            var = std::max(var, D.norm() / Mbar.norm());
            for (Eigen::Index k = 0; k < 18; ++k) {
                if (beta_d(k)) {
                    D.col(k).setZero();
                }
            }
            var_nd = std::max(var_nd, D.norm() / Mbar_nd.norm());
        }
        per_csv << kt << "," << mu << "," << var << "," << var_nd << "\n";

        // Floquet exponents of the free flow map against the averaged roots.
        const CVec lam = m.A.eigenvalues();
        const Monodromy M = monodromy(ex, c, r);
        require_converged(M.unconverged, "monodromy", kt);
        const FloquetModes f = floquet(M.matrix, m.period, lam);
        for (Eigen::Index i = 0; i < f.exponents.size(); ++i) {
            const auto k = f.match[static_cast<std::size_t>(i)];
            if (k < 0 || std::abs(lam(k)) >= kBand || lam(k).imag() < 0.0) {
                continue;
            }
            floq_csv << kt << "," << mu << "," << lam(k).real() << "," << lam(k).imag() << ","
                     << f.exponents(i).real() << "," << f.exponents(i).imag() << ","
                     << std::abs(f.exponents(i) - lam(k)) << "\n";
        }

        // L3 at every speed (gated at 0, 35, 60, 90, 120 and 150 kt).
        LinearizeOptions o512;
        o512.azimuths = kt <= 60 ? 512 : 1024;  // steps T/256 and T/512, as the L3 test
        o512.steps_per_period = o512.azimuths;
        const LinearModel m512 = linearize(ex, c, r, o512);
        require_converged(m512.unconverged, "linearize (L3)", kt);
        for (const test::L3Case& k : test::l3_cases()) {
            const test::L3Errors e = test::l3_errors(ex, c, r, start, m512, k, 3.0);
            require_converged(e.unconverged, "L3 free flights", kt);
            const double floor = test::l3_floor(kt);
            const test::L3Fit fp = test::fit_l3(e.periodic, k.amplitude, 0.1 * floor);
            const test::L3Fit fa = test::fit_l3(e.averaged, k.amplitude, 0.1 * floor);
            l3_csv << kt << "," << k.state << "," << k.control << "," << fp.c0 << ","
                   << fp.c1 * k.amplitude << "," << fp.misfit << "," << fa.c0 << ","
                   << fa.c1 * k.amplitude << "," << fa.misfit << "\n";
        }

        if (kt == 0 || kt == 60 || kt == 120 || kt == 150) {
            // Held orbit against the free flight.
            const Environment env = trim_environment(c);
            const double dt = m.period / 256;
            std::size_t missed = 0;
            const auto traj = test::free_flight(
                ex, env, start, [&](double) { return r.controls; }, dt, 3 * 256, &missed);
            require_converged(missed, "free flight", kt);
            const LinearModel f1 = around_free(ex, traj, 0, 64, 4, dt, r.controls, env, m);
            const LinearModel f3 = around_free(ex, traj, 512, 64, 4, dt, r.controls, env, m);
            require_converged(f1.unconverged + f3.unconverged, "linearize (free flight)", kt);
            const CondensedModel c1 = condense(f1);
            const CondensedModel c3 = condense(f3);
            const CVec l1 = f1.A.eigenvalues();
            const CVec l3 = f3.A.eigenvalues();
            for (const auto& z : lam) {
                if (std::abs(z) >= kBand || z.imag() < 0.0 || std::abs(z) < 1e-6) {
                    continue;
                }
                const auto a1 = nearest(l1, z);
                const auto a3 = nearest(l3, z);
                hf_csv << kt << ",eig_re," << z.real() << "," << a1.real() << "," << a3.real()
                       << "\n"
                       << kt << ",eig_im," << z.imag() << "," << a1.imag() << "," << a3.imag()
                       << "\n";
            }
            for (const auto& key : kKey) {
                hf_csv << kt << "," << derivative_name(key[0], key[1]) << ","
                       << derivative(cm, key[0], key[1]) << "," << derivative(c1, key[0], key[1])
                       << "," << derivative(c3, key[0], key[1]) << "\n";
            }
        }
        if (kt == 60 || kt == 120 || kt == 150) {
            LinearizeOptions o1024;
            o1024.azimuths = 1024;
            o1024.steps_per_period = 1024;
            const LinearModel m1024 = linearize(ex, c, r, o1024);
            require_converged(m1024.unconverged, "linearize (scaling)", kt);
            for (const double s : {0.125, 0.25, 0.5, 1.0}) {
                ss_csv << kt << "," << mu << "," << s << "," << scaled_gap(m1024, s) << "\n";
            }
        }
        if (kt == 0 || kt == 60 || kt == 150) {
            for (const double factor : {0.1, 0.3, 1.0, 3.0, 10.0}) {
                LinearizeOptions o;
                o.step_velocity *= factor;
                o.step_rate *= factor;
                o.step_angle *= factor;
                o.step_height *= factor;
                o.step_flap *= factor;
                o.step_flap_rate *= factor;
                o.step_other *= factor;
                o.step_control *= factor;
                const LinearModel ms = linearize(ex, c, r, o);
                require_converged(ms.unconverged, "linearize (step size)", kt);
                const CondensedModel cs = condense(ms);
                for (const auto& key : kKey) {
                    step_csv << kt << "," << factor << "," << derivative_name(key[0], key[1]) << ","
                             << derivative(cs, key[0], key[1]) << "\n";
                }
                int i = 0;
                for (const Mode& md : modes(cs, r.vel_b.x())) {
                    step_csv << kt << "," << factor << ",mode" << i << "_re,"
                             << md.eigenvalue.real() << "\n"
                             << kt << "," << factor << ",mode" << i << "_im,"
                             << md.eigenvalue.imag() << "\n";
                    ++i;
                }
            }
        }
    }
}

void write_l4(const std::filesystem::path& dir) {
    std::ofstream out(dir / "m7b_l4.csv");
    out.precision(12);
    out << "# L4 fixtures against Padfield 3rd ed. closed forms, hover (M7b, D-035)\n"
        << "case,quantity,model,closed\n";
    const double rho = isa_atmosphere(0.0, 0.0).density;
    const double pi = std::numbers::pi;
    TrimCondition hover;
    for (const double mass : {4000.0, 2000.0}) {
        test::SymmetricFixture f;
        f.hinge_offset = 0.0;
        f.cd0 = 0.0;
        f.mass = mass;
        const Executive ex = test::symmetric_hover_executive(f);
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, hover, nullptr, x);
        if (!r.converged) {
            throw std::runtime_error("L4 trim did not converge (heave, mass " +
                                     std::to_string(mass) + " kg)");
        }
        LinearizeOptions o;
        o.azimuths = 16;
        o.height = false;
        const LinearModel m = linearize(ex, hover, r, o);
        require_converged(m.unconverged, "linearize (L4)", 0);
        const CondensedModel cm = condense(m);
        const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(0));
        const BladeElementRotorConfig& cfg = rotor.config();
        const double lambda0 = r.component_states[rotor.inflow_offset()];
        const double a = test::kLiftSlope;
        const double sigma = cfg.num_blades * cfg.chord(0.5) / (pi * cfg.radius);
        const double Ma = mass + 2.0 * cfg.num_blades * cfg.flap->mass;
        const double dct = 2.0 * a * sigma * lambda0 / (16.0 * lambda0 + a * sigma);
        const double Zw =
            -rho * cfg.rotor_speed * cfg.radius * pi * cfg.radius * cfg.radius * 2.0 * dct / Ma;
        const CVec ev = m.A.eigenvalues();
        const std::string name = "heave_mass" + std::to_string(static_cast<int>(mass));
        out << name << ",Z_w," << cm.A(2, 2) << "," << Zw << "\n"
            << name << ",retained_heave_root," << nearest(ev, Zw).real() << "," << Zw << "\n"
            << name << ",lambda0," << lambda0 << "," << lambda0 << "\n";
    }
    for (const double Sb : {0.0, 0.15, 0.3}) {
        test::SymmetricFixture f;
        f.hinge_offset = 0.0;
        f.cd0 = 0.0;
        f.inflow = InflowModel::uniform;
        f.hub_height = 0.5;
        f.inertia = Vec3(1e6, 1e6, 1e6);
        const BladeElementRotorConfig cfg =
            test::symmetric_rotor(f, RotationDirection::counter_clockwise);
        const double W = cfg.rotor_speed;
        const double Ib = cfg.flap->inertia;
        const double gamma = rho * test::kLiftSlope * cfg.chord(0.5) * std::pow(cfg.radius, 4) / Ib;
        f.spring = Sb * gamma * Ib * W * W / 8.0;
        const Executive ex = test::symmetric_hover_executive(f);
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, hover, nullptr, x);
        if (!r.converged) {
            throw std::runtime_error("L4 trim did not converge (stiffness " + std::to_string(Sb) +
                                     ")");
        }
        const test::QuasiSteady q = test::quasi_steady(ex, hover, r);
        require_converged(q.unconverged, "quasi-steady derivatives", 0);
        const double direct = (Sb + 16.0 / gamma) / ((1.0 + Sb * Sb) * W);
        const double coupling = (Sb * 16.0 / gamma - 1.0) / ((1.0 + Sb * Sb) * W);
        LinearizeOptions o;
        o.azimuths = 16;
        o.height = false;
        const LinearModel lm = linearize(ex, hover, r, o);
        require_converged(lm.unconverged, "linearize (L4)", 0);
        const CondensedModel cm = condense(lm);
        const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(0));
        const std::vector<double> xs = held_orbit_start(ex, hover, r);
        const SimContext ctx = ex.context(0.0, xs, r.controls, trim_environment(hover));
        const double T = rotor
                             .evaluate(ctx, std::span<const double>(xs).subspan(
                                                ex.component_offset(0), ex.component_size(0)))
                             .thrust;
        const double Mq = -2.0 * (cfg.num_blades * f.spring / 2.0 + T * f.hub_height) * direct;
        const std::string name = "stiffness" + std::to_string(Sb).substr(0, 4);
        out << name << ",dbeta1c_dq," << q.flap(1, 1) << "," << direct << "\n"
            << name << ",dbeta1s_dp," << q.flap(2, 0) << "," << direct << "\n"
            << name << ",dbeta1c_dp," << q.flap(1, 0) << "," << coupling << "\n"
            << name << ",dbeta1s_dq," << q.flap(2, 1) << "," << -coupling << "\n"
            << name << ",Mq_Iyy," << cm.A(4, 4) * f.inertia.y() << "," << Mq << "\n"
            << name << ",held_dM_dq," << q.moment(1, 1) << "," << Mq << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m7b_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write_l4(dir);
        write_uh60a(dir);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
