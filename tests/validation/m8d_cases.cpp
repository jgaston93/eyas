// Writes the M8d data to CSV for VALIDATION.md (tools/plot_m8d.py; D-049):
// - Johnson's curves (NASA/TP-2005-213477 Table 3) as Eyas has them, on the
//   axes of Johnson's Figs. 38 and 39, and the published model without
//   Eyas's fades near the two edges where it jumps;
// - the UH-60A (data/uh60a/uh60a.yaml, ISA sea level, bare, collective
//   fixed) in vertical descent, 0.5 to 22 m/s: trimmed collective, the main
//   rotor's descent and edgewise speed over its v_h, the condensed heave
//   damping Z_w with the VRS deficit quasi-static (Johnson's "qs inflow")
//   and the heave root with it kept dynamic (the lag), and the same without
//   the correction (momentum theory);
// - a vortex-ring entry: from a trimmed 1 m/s vertical descent with the
//   whole AFCS on, the collective lowered in three steps, flown free for
//   100 s with and without the correction (the shape of Johnson's D6075
//   case, Figs. 56–57; no UH-60A flight data).
//
// Usage: eyas_m8d_cases <output-dir>

#include <cmath>
#include <complex>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "analysis/free_flight.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/linearize.hpp"
#include "analysis/trim.hpp"
#include "core/rk4.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/vortex_ring.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "support/vrs_reference.hpp"

namespace {

using namespace eyas;

void require(bool ok, const std::string& what) {
    if (!ok) {
        throw std::runtime_error(what);
    }
}

std::string config_text(bool vrs) {
    const std::string text = test::ballin::config_text(test::uh60a_config_path());
    const std::regex key(R"(vortex_ring: johnson_2005)");
    require(std::distance(std::sregex_iterator(text.begin(), text.end(), key),
                          std::sregex_iterator()) == 1,
            "expected exactly one vortex_ring: johnson_2005");
    return vrs ? text : std::regex_replace(text, key, "vortex_ring: none");
}

Executive executive(bool vrs) {
    const Config cfg = Config::from_string(config_text(vrs), vrs ? "uh60a" : "uh60a, no VRS");
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

std::size_t main_rotor(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const BladeElementRotor*>(&ex.component(i)) != nullptr) {
            return i;
        }
    }
    throw std::runtime_error("no blade_element_rotor");
}

TrimCondition vertical_descent(double rate) {
    TrimCondition c;
    c.airspeed = rate;
    c.flight_path_angle = -0.5 * std::numbers::pi;
    return c;
}

void curves(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8d_curves.csv");
    f.precision(12);
    f << "# Johnson NASA/TP-2005-213477 Table 3: induced velocity v/v_h against vz/v_h (D-049)\n"
      << "vx,vz,v_baseline,v_vrs,v_baseline_unfaded,v_vrs_unfaded\n";
    test::VrsRefParams johnson;
    johnson.fade = 1e-12;
    for (const double vx : {0.0, 0.4, 0.6, 0.749, 0.75, 0.8, 0.949, 0.95, 1.0, 1.4, 2.0, 4.0}) {
        for (int i = 0; i <= 1750; ++i) {
            const double vz = -3.0 + 0.002 * i;
            f << vx << ',' << vz << ',' << johnson_inflow(vx, vz, VrsCurve::baseline).v << ','
              << johnson_inflow(vx, vz, VrsCurve::vortex_ring).v << ','
              << test::ref_baseline(vx, vz, johnson).v << ',' << test::ref_vrs(vx, vz, johnson).v
              << '\n';
        }
    }
    std::cout << "curves done\n";
}

void heave(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8d_heave.csv");
    f.precision(12);
    f << "# UH-60A vertical descent, ISA sea level, bare, collective fixed (D-049)\n"
      << "vrs,rate_mps,collective,vz_vh,vx_vh,lambda0,deficit,zw_qs,heave_lag\n";
    for (const bool vrs : {true, false}) {
        const Executive ex = executive(vrs);
        const std::size_t mr = main_rotor(ex);
        const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(mr));
        for (int i = 1; i <= 44; ++i) {
            const double rate = 0.5 * i;
            const TrimCondition c = vertical_descent(rate);
            std::vector<double> x0;
            const TrimResult r = solve_trim(ex, c, nullptr, x0);
            require(r.converged, "trim did not converge at " + std::to_string(rate) + " m/s");
            const std::vector<double> x = held_orbit_start(ex, c, r);
            const std::span<const double> xr(x.data() + ex.component_offset(mr),
                                             ex.component_size(mr));
            const SimContext ctx = ex.context(0.0, x, r.controls, trim_environment(c));
            const RotorLoads l = rotor.evaluate(ctx, xr);
            const double lh = std::sqrt(0.5 * l.ct);
            LinearizeOptions o;
            o.azimuths = 16;
            o.height = false;
            const LinearModel m = linearize(ex, c, r, o);
            const double zw = condense(m).A(2, 2);
            // With the deficit kept dynamic: the root nearest Z_w, its real
            // part (near the region's edges the heave and lag roots pair up).
            double heave_lag = std::numeric_limits<double>::quiet_NaN();
            if (vrs) {
                const std::string name =
                    "main_rotor[" + std::to_string(rotor.vortex_ring_offset()) + "]";
                std::size_t idx = 0;
                while (idx < m.states.size() && m.states[idx] != name) {
                    ++idx;
                }
                require(idx < m.states.size(), "no state " + name);
                const CondensedModel cm = condense(m, {idx});
                const Eigen::VectorXcd ev = cm.A.eigenvalues();
                double best = 1e300;
                for (const auto& z : ev) {
                    if (std::abs(z - zw) < best) {
                        best = std::abs(z - zw);
                        heave_lag = z.real();
                    }
                }
            }
            f << (vrs ? 1 : 0) << ',' << rate << ',' << r.controls.collective << ',' << -l.mu_z / lh
              << ',' << l.mu / lh << ',' << xr[rotor.inflow_offset()] << ','
              << (vrs ? xr[rotor.vortex_ring_offset()] : 0.0) << ',' << zw << ',' << heave_lag
              << '\n';
        }
        std::cout << "heave (" << (vrs ? "VRS" : "momentum") << ") done\n";
    }
}

void encounter(const std::filesystem::path& dir) {
    std::ofstream f(dir / "m8d_encounter.csv");
    f.precision(10);
    f << "# UH-60A vortex-ring entry from a 1 m/s vertical descent, whole AFCS on (D-049); "
         "vz climb positive\n"
      << "vrs,t,collective,vz_mps,vx_mps,vz_vh,lambda0,deficit,thrust_n\n";
    // Johnson's D6075 vertical case lowers the collective in three steps
    // (Fig. 56, about 0.7° in all); here three steps of −0.015 (Controls
    // units: 0.15 in of the 10 in travel at 1.6°/in, 0.24° of θ0 each), each
    // ramped over 1 s, at 5, 15 and 25 s.
    const auto collective_change = [](double t) {
        double d = 0.0;
        for (const double t0 : {5.0, 15.0, 25.0}) {
            d -= 0.015 * std::clamp(t - t0, 0.0, 1.0);
        }
        return d;
    };
    for (const bool vrs : {true, false}) {
        const Executive ex = executive(vrs);
        const std::size_t mr = main_rotor(ex);
        const auto& rotor = dynamic_cast<const BladeElementRotor&>(ex.component(mr));
        TrimCondition c = vertical_descent(1.0);
        c.afcs = {true, true, true, true, false, false};
        std::vector<double> x;
        const TrimResult r = solve_trim(ex, c, nullptr, x);
        require(r.converged, "encounter trim did not converge");
        const Environment env = trim_environment(c);
        const auto controls = [&](double t) {
            Controls k = r.controls;
            k.collective += collective_change(t);
            return k;
        };
        Rk4 rk4(x.size());
        const double dt = 1e-3;
        const int n = 100000;
        for (int i = 0; i <= n; ++i) {
            const double t = dt * i;
            if (i % 50 == 0) {
                const BodyState b = unpack_body(x);
                const Vec3 v_n = b.q_nb * b.vel_b;
                const std::span<const double> xr(x.data() + ex.component_offset(mr),
                                                 ex.component_size(mr));
                const SimContext ctx = ex.context(t, x, controls(t), env);
                const RotorLoads l = rotor.evaluate(ctx, xr);
                const double vh = std::sqrt(0.5 * l.ct) * rotor.tip_speed();
                f << (vrs ? 1 : 0) << ',' << t << ',' << controls(t).collective << ',' << -v_n.z()
                  << ',' << std::hypot(v_n.x(), v_n.y()) << ',' << -v_n.z() / vh << ','
                  << xr[rotor.inflow_offset()] << ','
                  << (vrs ? xr[rotor.vortex_ring_offset()] : 0.0) << ',' << l.thrust << '\n';
            }
            if (i < n) {
                (void)free_step(ex, rk4, controls, env, t, dt, x);
            }
        }
        std::cout << "encounter (" << (vrs ? "VRS" : "momentum") << ") done\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m8d_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        curves(dir);
        heave(dir);
        encounter(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m8d_cases: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
