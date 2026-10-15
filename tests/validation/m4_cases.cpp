// Writes the M4 cases to CSV for the VALIDATION.md plots (tools/plot_m4.py).
// Same production code paths, reference, closed forms and fixtures as
// tests/unit/test_flapping_rotor.cpp and test_flap_anchors.cpp.
//
// Usage: eyas_m4_cases <output-dir>

#include <chrono>
#include <cmath>
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
#include "rotor/controls.hpp"
#include "rotor/multiblade.hpp"
#include "sim/executive.hpp"
#include "support/flap_closed_forms.hpp"
#include "support/flap_momentum.hpp"
#include "support/frame_status.hpp"
#include "support/m4_fixtures.hpp"

namespace {

using namespace eyas;
using namespace eyas::test;

std::ofstream open_csv(const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path.string());
    }
    out.precision(17);
    return out;
}

FlapCondition condition_from(const BladeElementRotor& rotor, const SimContext& ctx,
                             std::span<const double> x) {
    const RotorLoads l = rotor.evaluate(ctx, x);
    const SwashplatePitch p = rotor.config().controls->pitch(ctx.controls);
    FlapCondition k;
    k.rho = ctx.air.density;
    k.lift_slope = kLiftSlope;
    k.mu = l.mu;
    k.lambda = l.lambda_i - l.mu_z;
    k.theta0 = p.theta0;
    k.theta1c = p.theta1c;
    k.theta1s = p.theta1s;
    k.theta_tw = rotor.config().twist(1.0);
    return k;
}

// Flapping against advance ratio, both anchor rotors, against Padfield
// (centre spring) and Johnson (offset hinge).
void flapping_mu(const std::filesystem::path& dir) {
    std::ofstream out = open_csv(dir / "flapping_mu.csv");
    out << "rotor,mu,b0,b1c,b1s,cf_b0,cf_b1c,cf_b1s\n";
    for (const char* name : {"centre", "offset"}) {
        const bool centre = std::string(name) == "centre";
        BladeElementRotorConfig c =
            anchor_flap_rotor(centre ? 0.0 : 0.4, {8 * kDeg, -2 * kDeg, 1 * kDeg});
        if (centre) {
            c.root_cutout = 0.0;
            c.flap->spring = 0.12 * c.flap->inertia * c.rotor_speed * c.rotor_speed;
        }
        const BladeElementRotor rotor(c);
        for (int i = 0; i <= 14; ++i) {
            const double mu = 0.025 * i;
            SimContext ctx = sea_level_context();
            ctx.body.vel_b = Vec3(mu * c.rotor_speed * c.radius, 0.0, 0.0);
            std::vector<double> x(rotor.num_states(), 0.0);
            const Harmonics h = periodic_flapping(rotor, ctx, x, 12, 3);
            const FlapCondition k = condition_from(rotor, ctx, x);
            const FlapClosedForm f =
                centre ? padfield_centre_spring(c, k) : johnson_offset_hinge(c, k);
            out << name << ',' << mu << ',' << h.b0 << ',' << h.b1c << ',' << h.b1s << ','
                << f.beta0 << ',' << f.beta1c << ',' << f.beta1s << '\n';
        }
    }
}

// Hover coning against collective and thrust, offset hinge, against Johnson.
void coning(const std::filesystem::path& dir) {
    std::ofstream out = open_csv(dir / "coning.csv");
    out << "theta0_deg,ct,b0,cf_b0\n";
    for (int t = 2; t <= 18; t += 2) {
        const BladeElementRotorConfig c = anchor_flap_rotor(0.4, {t * kDeg, 0.0, 0.0});
        const BladeElementRotor rotor(c);
        SimContext ctx = sea_level_context();
        std::vector<double> x(rotor.num_states(), 0.0);
        const Harmonics h = periodic_flapping(rotor, ctx, x, 12, 1);
        const FlapCondition k = condition_from(rotor, ctx, x);
        out << t << ',' << rotor.evaluate(ctx, x).ct << ',' << h.b0 << ','
            << johnson_offset_hinge(c, k).beta0 << '\n';
    }
}

// Longitudinal cyclic step in hover (θ1s 0 → −3°), offset hinge: disc tilt
// in multiblade coordinates against time, and the steady closed form.
void cyclic_step(const std::filesystem::path& dir) {
    BladeElementRotorConfig c = anchor_flap_rotor(0.4, {});
    c.controls = std::make_shared<const LinearControlMap>(
        LinearPitchRow{10 * kDeg}, LinearPitchRow{0.0, 0.0, -3 * kDeg}, LinearPitchRow{});
    const BladeElementRotor rotor(c);
    SimContext ctx = sea_level_context();
    std::vector<double> x(rotor.num_states(), 0.0);
    (void)periodic_flapping(rotor, ctx, x, 10, 1);
    ctx.t = 0.0;
    ctx.controls.lon_cyclic = 1.0;
    FlapCondition k = condition_from(rotor, ctx, x);
    const FlapClosedForm f = johnson_offset_hinge(c, k);
    std::ofstream out = open_csv(dir / "cyclic_step.csv");
    out << "t,b0,b1c,b1s,cf_b1c,cf_b1s,lag_deg\n";
    const double lag = johnson_hover_phase_lag(c, ctx.air.density, kLiftSlope) / kDeg;
    out << 0.0 << ',' << multiblade_flap(x, 0.0, 4).beta0 << ",0,0," << f.beta1c << ',' << f.beta1s
        << ',' << lag << '\n';
    whirl(rotor, ctx, x, 1e-3, 2000, [&](double t, const std::vector<double>& xs) {
        const MultibladeFlap m = multiblade_flap(xs, rotor.azimuth(t), 4);
        out << t << ',' << m.beta0 << ',' << m.beta1c << ',' << m.beta1s << ',' << f.beta1c << ','
            << f.beta1s << ',' << lag << '\n';
    });
}

// Momentum drift of the coupled airframe + blades against step size
// (FlappingRotor.CoupledSystemConservesMomentum, over a range of steps).
void conservation(const std::filesystem::path& dir) {
    const auto airfoil = airfoil_from(zero_airfoil_csv());
    const double e = 0.4;
    const Beads beads = nonuniform_beads(8.0 - e);
    BladeElementRotorConfig c = flap_rotor(airfoil, {}, beads, e, 8);
    c.hub_position_b = Vec3(0.3, 0.1, -2.0);
    c.hub_pitch = -4 * kDeg;
    c.flap->spring = 5e3;
    c.flap->damper = 200.0;
    const MassProperties mp = make_mass_properties(900.0, 600.0, 1500.0, 1400.0, 120.0);
    std::ofstream out = open_csv(dir / "conservation.csv");
    out << "step_ms,dH,dP\n";
    for (const std::int64_t step_ns : {4'000'000, 2'000'000, 1'000'000, 500'000, 250'000}) {
        ExecutiveConfig ec;
        ec.mass = mp;
        ec.step_ns = step_ns;
        ec.max_substeps = 100000;
        std::vector<std::unique_ptr<DynamicComponent>> comps;
        comps.push_back(std::make_unique<BladeElementRotor>(c));
        const auto* rotor = dynamic_cast<const BladeElementRotor*>(comps.back().get());
        Executive ex(ec, std::move(comps));
        BodyState b;
        b.vel_b = Vec3(3.0, -1.0, 2.0);
        b.omega_b = Vec3(0.6, -0.4, 0.8);
        b.q_nb = Quat(Eigen::AngleAxisd(0.3, Vec3(1, 2, 3).normalized()));
        ex.set_body_state(b);
        std::vector<double> xs(ex.state().begin(), ex.state().end());
        for (std::size_t k = 0; k < 4; ++k) {
            xs[body_layout::kSize + 2 * k] = 0.05 * (1.0 + static_cast<double>(k));
            xs[body_layout::kSize + 2 * k + 1] = 0.3 - 0.2 * static_cast<double>(k);
        }
        (void)ex.restore(xs, 0, 0);
        const FlapReference ref(*rotor, beads);
        const Momentum m0 = system_momentum(ex, *rotor, ref, beads, mp);
        test::require_frame(ex.advance({}, {}, 1'000'000'000), "m4_cases");
        const Momentum m1 = system_momentum(ex, *rotor, ref, beads, mp);
        const double dP = (m1.P_n - (m0.P_n + m0.mass * gravity_n())).norm() / (m0.mass * 10.0);
        const double dH = (m1.H_cg_n - m0.H_cg_n).norm() / m0.H_cg_n.norm();
        out << static_cast<double>(step_ns) * 1e-6 << ',' << dH << ',' << dP << '\n';
    }
}

// Cost of one rotor derivative evaluation, with and without the flap hinge
// (Release build; not the D-005 benchmark).
void timing() {
    const auto syn = airfoil_from(synthetic_airfoil_csv());
    const SwashplatePitch p{8 * kDeg, -2 * kDeg, 1 * kDeg};
    BladeElementRotorConfig c3 = test_rotor(syn, p, 20);
    BladeElementRotorConfig c4 = flap_rotor(syn, p, nonuniform_beads(7.6, 12.0), 0.4, 20);
    c4.root_cutout = 1.2;
    for (const auto* c : {&c3, &c4}) {
        const BladeElementRotor rotor(*c);
        SimContext ctx = sea_level_context(0.1);
        ctx.body.vel_b = Vec3(50.0, 0.0, 1.0);
        std::vector<double> x(rotor.num_states(), 0.03);
        std::vector<double> xd(x.size());
        std::vector<double> G(x.size() * 6);
        const int n = 20000;
        const auto t0 = std::chrono::steady_clock::now();
        double sink = 0.0;
        for (int i = 0; i < n; ++i) {
            ctx.t = 1e-4 * i;
            ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
            rotor.derivatives(ctx, x, d);
            sink += d.load0_b.F.z();
        }
        const auto t1 = std::chrono::steady_clock::now();
        std::cout << (c->flap ? "flapping" : "M3 rigid")
                  << " rotor: " << std::chrono::duration<double, std::micro>(t1 - t0).count() / n
                  << " us per derivative evaluation (sink " << sink << ")\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m4_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir = argv[1];
        std::filesystem::create_directories(dir);
        flapping_mu(dir);
        coning(dir);
        cyclic_step(dir);
        conservation(dir);
        timing();
    } catch (const std::exception& e) {
        std::cerr << "eyas_m4_cases: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
