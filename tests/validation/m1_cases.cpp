// Writes time histories of the M1 acceptance cases to CSV for the
// VALIDATION.md plots (tools/plot_m1.py). Same cases and production code
// paths as tests/unit/test_rigid_body.cpp and test_coupled_solve.cpp.
//
// Usage: eyas_m1_cases <output-dir>

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "sim/recorder.hpp"
#include "support/spring_mass.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;

constexpr std::int64_t kStepNs = 1'000'000;

Executive make(const MassProperties& mp, std::vector<std::unique_ptr<DynamicComponent>> c = {}) {
    ExecutiveConfig cfg;
    cfg.mass = mp;
    cfg.step_ns = kStepNs;
    return Executive(cfg, std::move(c));
}

void run_steps(Executive& ex, std::int64_t n) {
    for (std::int64_t i = 0; i < n; ++i) {
        (void)ex.advance(Controls{}, Environment{}, kStepNs);
    }
}

// Same tensor as the unit tests: GenHel UH-60A airframe inertias (D-002).
MassProperties helicopter_like_body() {
    const double s = units::kSlug * units::kFoot * units::kFoot;
    return make_mass_properties(1000.0, 4659.0 * s, 38512.0 * s, 36796.0 * s, 1882.0 * s);
}

void torque_free(const std::filesystem::path& dir) {
    const MassProperties mp = helicopter_like_body();
    Executive ex = make(mp);
    BodyState b0;
    b0.q_nb = Quat(Eigen::AngleAxisd(-0.4, Vec3::UnitZ()) * Eigen::AngleAxisd(0.1, Vec3::UnitY()) *
                   Eigen::AngleAxisd(0.2, Vec3::UnitX()));
    b0.omega_b = Vec3(0.6, -0.5, 0.8);
    ex.set_body_state(b0);
    const double E0 = 0.5 * b0.omega_b.dot(mp.inertia_b * b0.omega_b);
    const Vec3 H0 = b0.q_nb * (mp.inertia_b * b0.omega_b);
    Recorder rec({"dE_rel", "dH_rel"}, 601);
    for (int i = 0; i <= 600; ++i) {
        const BodyState b = ex.body_state();
        const double E = 0.5 * b.omega_b.dot(mp.inertia_b * b.omega_b);
        const Vec3 H = b.q_nb * (mp.inertia_b * b.omega_b);
        const std::array<double, 2> row{(E - E0) / E0, (H - H0).norm() / H0.norm()};
        (void)rec.record(ex.sim_time_ns(), row);
        run_steps(ex, 100);
    }
    rec.write_csv(dir / "torque_free.csv");
}

void intermediate_axis(const std::filesystem::path& dir) {
    const double I1 = 1.0;
    const double I2 = 2.0;
    const double I3 = 3.0;
    const double W = 1.0;  // [rad/s]
    const double eps = 1e-8;
    Executive ex = make(make_mass_properties(1.0, I1, I2, I3, 0.0));
    const double lambda = W * std::sqrt((I3 - I2) * (I2 - I1) / (I1 * I3));
    BodyState b0;
    b0.omega_b = Vec3(eps, W, -lambda * eps / ((I3 - I2) * W / I1));
    ex.set_body_state(b0);
    Recorder rec({"w1", "w2", "w3", "w1_linear"}, 1201);
    for (int i = 0; i <= 1200; ++i) {
        const Vec3 w = ex.body_state().omega_b;
        const std::array<double, 4> row{w.x(), w.y(), w.z(),
                                        eps * std::exp(lambda * ex.sim_time())};
        (void)rec.record(ex.sim_time_ns(), row);
        run_steps(ex, 50);
    }
    rec.write_csv(dir / "intermediate_axis.csv");
}

void oscillator(const std::filesystem::path& dir) {
    const double M = 900.0;  // [kg]
    const double m = 100.0;  // [kg]
    const double wn = 10.0;  // [rad/s]
    const double s0 = 0.05;  // [m]
    const double mu = M * m / (M + m);
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<test::SpringMassOnRail>(
        test::Rail{m, wn * wn * mu, 0.0, Vec3::Zero(), Vec3::UnitX()}, s0, 0.0));
    Executive ex = make(make_mass_properties(M, 700.0, 2100.0, 1800.0, 0.0), std::move(comps));
    ex.set_body_state(BodyState{});
    Recorder rec({"s", "s_exact"}, 1001);
    for (int i = 0; i <= 1000; ++i) {
        const std::array<double, 2> row{ex.state()[body_layout::kSize],
                                        s0 * std::cos(wn * ex.sim_time())};
        (void)rec.record(ex.sim_time_ns(), row);
        run_steps(ex, 10);
    }
    rec.write_csv(dir / "oscillator.csv");
}

// Per-step quaternion norm error vs body rate, with RK4's prediction
// 1 − sqrt(1 − θ⁶/72 + θ⁸/576), θ = |ω| h / 2 (see test_rigid_body.cpp).
void quat_norm_sweep(const std::filesystem::path& dir) {
    std::ofstream out(dir / "quat_norm_sweep.csv");
    out.precision(17);
    out << "omega_rad_s,measured,predicted\n";
    for (int i = 0; i <= 40; ++i) {
        const double w = 0.5 * std::pow(10.0, 0.06 * i);  // 0.5 to ~126 rad/s
        Executive ex = make(make_mass_properties(1.0, 1.0, 2.0, 3.0, 0.0));
        BodyState b0;
        b0.omega_b = Vec3(w, 0.0, 0.0);
        ex.set_body_state(b0);
        run_steps(ex, 1);
        const double th = w * 1e-3 / 2.0;
        const double u = std::pow(th, 6) / 72.0 - std::pow(th, 8) / 576.0;
        out << w << ',' << ex.quat_norm_error() << ',' << (u / 2.0 + u * u / 8.0) << '\n';
    }
    if (!out) {
        throw std::runtime_error("cannot write quat_norm_sweep.csv");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m1_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        torque_free(dir);
        intermediate_axis(dir);
        oscillator(dir);
        quat_norm_sweep(dir);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
