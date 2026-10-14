// Writes the M6c landing-gear cases to CSV for the VALIDATION.md plots
// (tools/plot_m6c.py; D-032):
// - a drop of the full gear from 0.3 m, damping on: CG height, pitch, strut
//   strokes and normal loads;
// - energy conservation of one gear under the CG with damping and friction
//   off, against the physics step;
// - a slope below atan μ (hold) and above it (slide), every wheel braked;
// - a brakes-off roll from 2 m/s on level ground;
// - the sliding blend speed v_s swept for a braked stop from 4 ft/s.
// The gear is GenHel's UH-60A gear (tests/support/m6c_fixtures.hpp); the
// airframe mass is the tests' synthetic 7000 kg.
//
// Usage: eyas_m6c_cases <output-dir>

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "airframe/landing_gear.hpp"
#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "support/m6c_fixtures.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;
using units::kStandardGravity;

constexpr double kAirframeMass = 7000.0;  // [kg] synthetic, without the wheels
constexpr std::size_t kN = LandingGear::kStatesPerGear;

class Lift : public ForceModel {
public:
    explicit Lift(double f) : f_(f) {}
    Wrench compute(const SimContext& ctx) const override {
        Wrench w;
        w.F = ctx.body.q_nb.toRotationMatrix().transpose() * Vec3(0.0, 0.0, -f_);
        return w;
    }

private:
    double f_;
};

struct Rig {
    LandingGear* gear = nullptr;
    std::unique_ptr<Executive> ex;
    Environment env;
    Controls controls = [] {
        Controls c;
        c.brake_left = 1.0;  // brakes set unless a case releases them
        c.brake_right = 1.0;
        return c;
    }();

    std::vector<double> gear_states() const {
        const auto s = ex->state().subspan(body_layout::kSize);
        return {s.begin(), s.end()};
    }
    GearState state(std::size_t i) const {
        SimContext c;
        c.body = ex->body_state();
        c.env = env;
        c.controls = controls;
        c.air.altitude = -c.body.pos_n.z();
        return gear->gear_state(c, gear_states(), i);
    }
    void step(double dt) {
        (void)ex->advance(controls, env, static_cast<std::int64_t>(std::llround(dt * 1e9)));
    }
};

Rig make_rig(LandingGearConfig gc, double lift, double gap, double pitch,
             std::int64_t step_ns = 1'000'000) {
    Rig r;
    auto gear = std::make_unique<LandingGear>(std::move(gc));
    r.gear = gear.get();
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::move(gear));
    std::vector<std::unique_ptr<ForceModel>> forces;
    if (lift != 0.0) {
        forces.push_back(std::make_unique<Lift>(lift));
    }
    ExecutiveConfig ec;
    ec.mass = make_mass_properties(kAirframeMass, 6000.0, 50000.0, 46000.0, 2000.0);
    ec.step_ns = step_ns;
    ec.max_substeps = 1'000'000;
    r.ex = std::make_unique<Executive>(ec, std::move(comps), std::move(forces));
    r.env.ground_height = 0.0;
    BodyState b;
    b.q_nb = Quat(Eigen::AngleAxisd(pitch, Vec3::UnitY()));
    const Mat3 C_nb = b.q_nb.toRotationMatrix();
    const Vec3 u = r.gear->config().ground_normal_n;
    double lowest = 1e9;
    for (const GearConfig& g : r.gear->config().gears) {
        lowest = std::min(lowest, u.dot(C_nb * (g.axle_b + g.tire.radius * Vec3::UnitZ())));
    }
    b.pos_n = u * (gap - lowest);
    r.ex->set_body_state(b);
    return r;
}

LandingGearConfig uh60a(const test::GearYamlOptions& o = {}) {
    return LandingGear::from_config(Config::from_string(test::uh60a_gear_yaml(o)))->config();
}

std::ofstream open(const std::filesystem::path& p) {
    std::ofstream f(p);
    if (!f) {
        throw std::runtime_error("cannot write " + p.string());
    }
    f.precision(10);
    return f;
}

void drop(const std::filesystem::path& dir) {
    Rig r = make_rig(uh60a(), 0.0, 0.3, 0.0);
    std::ofstream f = open(dir / "m6c_drop.csv");
    f << "t,height,pitch_deg,vz,s0,s1,s2,n0,n1,n2\n";
    for (int k = 0; k <= 3000; ++k) {
        if (k % 5 == 0) {
            const BodyState b = r.ex->body_state();
            const auto x = r.gear_states();
            f << k * 1e-3 << ',' << -b.pos_n.z() << ','
              << euler_angles(b.q_nb).pitch / units::kDegree << ','
              << (b.q_nb.toRotationMatrix() * b.vel_b).z() << ',' << x[0] << ',' << x[kN] << ','
              << x[2 * kN] << ',' << r.state(0).normal_load << ',' << r.state(1).normal_load << ','
              << r.state(2).normal_load << '\n';
        }
        r.step(1e-3);
    }
}

double energy_drift(std::int64_t step_ns) {
    LandingGearConfig c = uh60a();
    GearConfig g = c.gears[0];
    g.axle_b = Vec3(0.0, 0.0, g.axle_b.z());
    g.tire.damping_stiffness = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
    g.strut.low = {};
    g.strut.high = {};
    g.strut.stop_damping = 0.0;
    c.gears = {g};
    c.friction_coefficient = 0.0;
    const double lift = 2.0 / 3.0 * (kAirframeMass + g.unsprung_mass) * kStandardGravity;
    Rig r = make_rig(c, lift, 0.3, 0.0, step_ns);
    const double m = g.unsprung_mass;
    const double k_stop = r.gear->stop_stiffness(0);
    const StrutConfig& sc = g.strut;
    const double L0 = sc.air_spring / sc.preload;
    double s0 = -(sc.preload + m * kStandardGravity) / k_stop;
    for (int it = 0; it < 50; ++it) {
        const double f = sc.air_spring / (L0 - s0) + k_stop * s0 + m * kStandardGravity;
        s0 -= f / (sc.air_spring / ((L0 - s0) * (L0 - s0)) + k_stop);
    }
    std::vector<double> x(r.ex->state().begin(), r.ex->state().end());
    x[body_layout::kSize] = s0;
    (void)r.ex->restore(x, 0, 0);
    const auto tire_energy = [&](double d) {
        double e = 0.0;
        const double step = d / 2000.0;
        for (int k = 0; k < 2000; ++k) {
            e += tire_vertical_load(g.tire.vertical_load, (k + 0.5) * step) * step;
        }
        return d > 0.0 ? e : 0.0;
    };
    const auto energy = [&] {
        const BodyState b = r.ex->body_state();
        const auto xs = r.gear_states();
        const double s = xs[0];
        const double vz_wheel = b.vel_b.z() - xs[1];
        const double z_wheel = b.pos_n.z() + g.axle_b.z() - s;
        const double strut =
            sc.air_spring * std::log(L0 / (L0 - s)) + (s < 0.0 ? 0.5 * k_stop * s * s : 0.0);
        return 0.5 * kAirframeMass * b.vel_b.squaredNorm() + 0.5 * m * vz_wheel * vz_wheel -
               kStandardGravity * (kAirframeMass * b.pos_n.z() + m * z_wheel) + lift * b.pos_n.z() +
               strut + tire_energy(r.state(0).tire_deflection);
    };
    const double e0 = energy();
    const double scale = (kAirframeMass * kStandardGravity - lift) * 0.3;
    double worst = 0.0;
    for (int k = 0; k < 300; ++k) {
        r.step(0.01);
        worst = std::max(worst, std::abs(energy() - e0));
    }
    return worst / scale;
}

void energy(const std::filesystem::path& dir) {
    std::ofstream f = open(dir / "m6c_energy.csv");
    f << "step_ms,relative_drift\n";
    for (const std::int64_t ns : {2'000'000, 1'000'000, 500'000, 250'000, 125'000, 62'500}) {
        f << static_cast<double>(ns) * 1e-6 << ',' << energy_drift(ns) << '\n';
    }
}

void slope(const std::filesystem::path& dir) {
    test::GearYamlOptions o;
    o.tail_brake = "left";
    const LandingGearConfig base = uh60a(o);
    const double mu = base.friction_coefficient;
    std::ofstream f = open(dir / "m6c_slope.csv");
    f << "case,angle_deg,t,x,v,slipping,expected_accel\n";
    int idx = 0;
    for (const double angle : {0.8 * std::atan(mu), std::atan(mu) + 3.0 * units::kDegree}) {
        LandingGearConfig c = base;
        c.ground_normal_n = Vec3(std::sin(angle), 0.0, -std::cos(angle));
        Rig r = make_rig(c, 0.0, 0.01, -angle);
        const Vec3 ds(std::cos(angle), 0.0, std::sin(angle));
        const double expected =
            angle < std::atan(mu)
                ? 0.0
                : kStandardGravity * (std::sin(angle) - c.sliding_factor * mu * std::cos(angle));
        const double x0 = ds.dot(r.ex->body_state().pos_n);
        for (int k = 0; k <= 10000; ++k) {
            if (k % 10 == 0) {
                const BodyState b = r.ex->body_state();
                int slipping = 0;
                for (std::size_t i = 0; i < 3; ++i) {
                    slipping += r.state(i).slipping[0] ? 1 : 0;
                }
                f << idx << ',' << angle / units::kDegree << ',' << k * 1e-3 << ','
                  << ds.dot(b.pos_n) - x0 << ',' << ds.dot(b.q_nb.toRotationMatrix() * b.vel_b)
                  << ',' << slipping << ',' << expected << '\n';
            }
            r.step(1e-3);
        }
        ++idx;
    }
}

// A stop from u0 on level ground, every tire fore-aft at its limit once it
// slides: returns the distance and writes the history if `out` is set.
std::pair<double, double> stop(LandingGearConfig c, double u0, std::ofstream* out, double T,
                               double brake = 1.0) {
    Rig r = make_rig(std::move(c), 0.0, 0.0, 0.0);
    r.controls.brake_left = brake;
    r.controls.brake_right = brake;
    // Settle on the ground first, then start rolling.
    for (int k = 0; k < 3000; ++k) {
        r.step(1e-3);
    }
    std::vector<double> x(r.ex->state().begin(), r.ex->state().end());
    x[body_layout::kVel] += u0;
    const double x0 = x[body_layout::kPos];
    (void)r.ex->restore(x, 0, 0);
    double t_stop = -1.0;
    for (int k = 0; k <= static_cast<int>(T * 1e3); ++k) {
        const BodyState b = r.ex->body_state();
        if (out != nullptr && k % 10 == 0) {
            *out << k * 1e-3 << ',' << b.vel_b.x() << ',' << b.pos_n.x() - x0 << '\n';
        }
        if (t_stop < 0.0 && b.vel_b.x() <= 0.0) {
            t_stop = k * 1e-3;
        }
        r.step(1e-3);
    }
    return {r.ex->body_state().pos_n.x() - x0, t_stop};
}

void roll(const std::filesystem::path& dir) {
    const LandingGearConfig c = uh60a();
    std::ofstream f = open(dir / "m6c_roll.csv");
    f << "t,u,x\n";
    (void)stop(c, 2.0, &f, 20.0, 0.0);
    // Expected: every tire at K_SLIP k_off μ N, so a = K_SLIP k_off μ g.
    std::ofstream e = open(dir / "m6c_roll_expected.csv");
    e << "decel\n"
      << c.sliding_factor * c.brake_factor_off * c.friction_coefficient * kStandardGravity << '\n';
}

void vs_sweep(const std::filesystem::path& dir) {
    std::ofstream f = open(dir / "m6c_vs_sweep.csv");
    f << "vs_ftps,distance,t_stop\n";
    for (const double vs_ft : {0.01, 0.03, 0.1, 0.3, 1.0}) {
        LandingGearConfig c = uh60a();
        c.sliding_blend_speed = vs_ft * units::kFoot;
        const auto [d, t] = stop(c, 4.0 * units::kFoot, nullptr, 3.0);
        f << vs_ft << ',' << d << ',' << t << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m6c_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        drop(dir);
        energy(dir);
        slope(dir);
        roll(dir);
        vs_sweep(dir);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
