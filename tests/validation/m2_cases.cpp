// Writes the M2 acceptance cases to CSV for the VALIDATION.md plots
// (tools/plot_m2.py). Same cases and production code paths as
// tests/unit/test_atmosphere.cpp and test_model.cpp.
//
// Usage: eyas_m2_cases <output-dir>

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

#include "env/atmosphere.hpp"
#include "eyas/model.hpp"
#include "sim/executive.hpp"
#include "sim/recorder.hpp"
#include "support/frame_status.hpp"
#include "support/m2_fixtures.hpp"

namespace {

using namespace eyas;

constexpr std::int64_t kFrameNs = 16'666'667;  // 60 Hz host frame

std::ofstream open_csv(const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot write " + path.string());
    }
    out.precision(17);
    return out;
}

// ISA profile for ΔT = −20, 0, +20 K, from −1 km to 20 km.
void isa_profile(const std::filesystem::path& dir) {
    std::ofstream out = open_csv(dir / "isa_profile.csv");
    out << "Z,T_m20,T_0,T_p20,P_m20,P_0,P_p20,rho_m20,rho_0,rho_p20\n";
    for (int i = -10; i <= 200; ++i) {
        const double Z = 100.0 * i;
        const AirState m = isa_atmosphere(Z, -20.0);
        const AirState s = isa_atmosphere(Z, 0.0);
        const AirState p = isa_atmosphere(Z, 20.0);
        out << Z << ',' << m.temperature << ',' << s.temperature << ',' << p.temperature << ','
            << m.pressure << ',' << s.pressure << ',' << p.pressure << ',' << m.density << ','
            << s.density << ',' << p.density << '\n';
    }
}

// Residuals against the USSA Table I rows (human-verified; DATA_GAPS.md), in
// units of the last printed digit.
void isa_table_check(const std::filesystem::path& dir) {
    struct Row {
        bool geometric;  // true: Z table, false: H table
        double altitude, T, P_mb, P_unit, rho, rho_unit;
    };
    const std::array<Row, 7> rows = {{
        {true, 0.0, 288.150, 1013.25, 0.01, 1.2250, 1e-4},
        {true, 1000.0, 281.651, 898.76, 0.01, 1.1117, 1e-4},
        {true, 5000.0, 255.676, 540.48, 0.01, 0.73643, 1e-5},
        {true, 11000.0, 216.774, 226.99, 0.01, 0.36480, 1e-5},
        {false, 1000.0, 281.650, 898.74, 0.01, 1.1116, 1e-4},
        {false, 5000.0, 255.650, 540.19, 0.01, 0.73612, 1e-5},
        {false, 11000.0, 216.650, 226.32, 0.01, 0.36392, 1e-5},
    }};
    std::ofstream out = open_csv(dir / "isa_table_check.csv");
    out << "geometric,altitude,dT_lsd,dP_lsd,drho_lsd\n";
    const double r0 = isa::kEarthRadius;
    for (const Row& r : rows) {
        const double Z = r.geometric ? r.altitude : r0 * r.altitude / (r0 - r.altitude);
        const AirState a = isa_atmosphere(Z, 0.0);
        out << (r.geometric ? 1 : 0) << ',' << r.altitude << ',' << (a.temperature - r.T) / 1e-3
            << ',' << (a.pressure / 100.0 - r.P_mb) / r.P_unit << ','
            << (a.density - r.rho) / r.rho_unit << '\n';
    }
}

// Equilibrium drift over 60 s: the spec's level case through eyas::Model, and
// the D-018 tilted, translating case through the executive.
void equilibrium(const std::filesystem::path& dir) {
    Model level = Model::from_string(test::m2_yaml(), "m2_cases");
    Controls hover;
    hover.collective = 0.5;

    ExecutiveConfig cfg;
    cfg.mass = test::m2_mass_properties();
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::make_unique<test::InertialWeightBalance>());
    Executive tilted(cfg, {}, std::move(forces));
    BodyState b0;
    b0.q_nb = Quat(Eigen::AngleAxisd(2.1, Vec3::UnitZ()) * Eigen::AngleAxisd(-0.3, Vec3::UnitY()) *
                   Eigen::AngleAxisd(0.45, Vec3::UnitX()));
    b0.vel_b = Vec3(9.0, -2.5, 1.2);
    tilted.set_body_state(b0);
    const BodyState start = tilted.body_state();

    Recorder rec({"level_pos", "level_vel", "tilted_pos", "tilted_vel"}, 601);
    for (int i = 0; i <= 600; ++i) {
        if (i > 0) {
            for (int k = 0; k < 100; ++k) {
                test::require_frame(tilted.advance(Controls{}, Environment{}, 1'000'000),
                                    "m2_cases");
            }
            for (int k = 0; k < 6; ++k) {
                test::require_frame(level.advance(hover, Environment{}, kFrameNs), "m2_cases");
            }
        }
        const BodyState l = level.body_state();
        const BodyState t = tilted.body_state();
        const Vec3 expected = start.q_nb * start.vel_b * tilted.sim_time();
        const std::array<double, 4> row = {l.pos_n.norm(), l.vel_b.norm(),
                                           (t.pos_n - expected).norm(),
                                           (t.vel_b - start.vel_b).norm()};
        (void)rec.record(tilted.sim_time_ns(), row);
    }
    rec.write_csv(dir / "equilibrium.csv");
}

// Collective step 0.5 → 0.6 at t = 1 s: a = −0.2 g along D.
void collective_step(const std::filesystem::path& dir) {
    Model m = Model::from_string(test::m2_yaml(), "m2_cases");
    Recorder rec({"collective", "w", "D", "altitude"}, 200);
    Controls c;
    for (int i = 0; i < 180; ++i) {  // 3 s
        c.collective = m.outputs().sim_time_ns < 1'000'000'000 ? 0.5 : 0.6;
        test::require_frame(m.advance(c, Environment{}, kFrameNs), "m2_cases");
        const BodyState b = m.body_state();
        const std::array<double, 4> row = {c.collective, b.vel_b.z(), b.pos_n.z(),
                                           m.outputs().altitude};
        (void)rec.record(m.outputs().sim_time_ns, row);
    }
    rec.write_csv(dir / "collective_step.csv");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m2_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        isa_profile(dir);
        isa_table_check(dir);
        equilibrium(dir);
        collective_step(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m2_cases: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
