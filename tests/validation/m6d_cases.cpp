// Writes the M6d whole-aircraft cases for the VALIDATION.md entry
// (tools/plot_m6d.py; D-028, D-034), on the UH-60A config
// data/uh60a/uh60a.yaml:
// - hover out of ground effect, body held level and at rest, cyclic centred:
//   the collective and pedal for zero vertical force and yaw moment, the yaw
//   budget and the controls in GenHel's units;
// - the ground pedal (yaw balance at collective down) and a drop from 0.3 m
//   with brakes set: time histories, the settled ground loads against weight
//   minus lift, and the slow rocking D-032 warned of;
// - the cost of one physics step of the whole aircraft (Release build).
//
// Usage: eyas_m6d_cases <output-dir>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

#include "eyas/model.hpp"
#include "support/m6d_fixtures.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;
using test::Uh60aHarness;
constexpr double kDeg = std::numbers::pi / 180.0;

// GenHel inches from a normalized input: the placeholder travels (D-034).
double xc_in(double c) {
    return 10.0 * c;
}
double xp_in(double p) {
    const double lo = (36.5 - 21.9) / -5.539;
    const double hi = (4.5 - 21.9) / -5.539;
    return 0.5 * (lo + hi) + 0.5 * (hi - lo) * p;
}

void hover(const Uh60aHarness& ac, std::ostream& out) {
    BodyState b;
    b.pos_n = Vec3(0.0, 0.0, -500.0);
    std::vector<double> x = ac.initial(b);
    double t = 0.0;
    Controls c;
    c.collective = 0.5;
    c.pedal = 0.5;
    const Environment env;
    test::HeldBalance bal;
    if (!test::find_hover(ac, x, t, c, env, bal, 0.05, 0.05, 20)) {
        throw std::runtime_error("hover: no balance");
    }
    // Aerodynamic quantities averaged over one blade passage.
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto& tr = ac.get<BaileyRotor>("tail_rotor");
    const int samples = 48;
    const double h = ac.passage_period() / samples;
    double q = 0.0, thrust = 0.0, power = 0.0, t_tr = 0.0, th_tr = 0.0;
    for (int k = 0; k < samples; ++k) {
        (void)ac.evaluate(t, x, c, env);  // fills the coupling store at (t, x)
        const SimContext ctx = ac.context(t, x, c, env);
        const RotorLoads l = mr.evaluate(ctx, ac.states(x, ac.index("main_rotor")));
        const BaileyRotorLoads lt = tr.evaluate(ctx, ac.states(x, ac.index("tail_rotor")));
        q += l.torque / samples;
        thrust += l.thrust / samples;
        power += l.power / samples;
        t_tr += lt.thrust / samples;
        th_tr += lt.pitch / samples;
        ac.hold(t, x, c, env, h, 1);
    }
    const SwashplatePitch p_mr = mr.config().controls->pitch(c);
    const double th_cmd = tr.config().controls->pitch(c).theta0;
    const double ftlbf = units::kPoundForce * units::kFoot;
    out << "# Hover OGE, ISA sea level, body held level, cyclic centred (M6d)\n";
    out << "weight_lbf," << ac.total_weight() / units::kPoundForce << "\n";
    out << "collective," << c.collective << "\n";
    out << "collective_in," << xc_in(c.collective) << "\n";
    out << "theta0_deg," << p_mr.theta0 / kDeg << "\n";
    out << "pedal," << c.pedal << "\n";
    out << "pedal_in," << xp_in(c.pedal) << "\n";
    out << "theta_tr_cmd_deg," << th_cmd / kDeg << "\n";
    out << "theta_tr_deg," << th_tr / kDeg << "\n";
    out << "mr_thrust_lbf," << thrust / units::kPoundForce << "\n";
    out << "mr_torque_ftlbf," << q / ftlbf << "\n";
    out << "mr_power_hp," << power / 745.69987158227022 << "\n";
    out << "tr_thrust_lbf," << t_tr / units::kPoundForce << "\n";
    for (std::size_t i = 0; i < ac.size(); ++i) {
        out << "yaw_" << ac.component(i).name() << "_ftlbf," << bal.loads[i].M.z() / ftlbf << "\n";
    }
    out << "yaw_sum_ftlbf," << bal.yaw / ftlbf << "\n";
    out << "vertical_residual_lbf," << bal.vertical / units::kPoundForce << "\n";
}

struct Sample {
    double t, height, pitch, roll, heading, q, p, n0, n1, n2, lift;
};

void ground(const Uh60aHarness& ac, const std::filesystem::path& dir, std::ostream& summary) {
    Controls c;
    {
        BodyState b;
        b.pos_n = Vec3(0.0, 0.0, -500.0);
        std::vector<double> x = ac.initial(b);
        double t = 0.0;
        if (!test::balance_pedal(ac, x, t, c, Environment{}, 0.05, 20)) {
            throw std::runtime_error("ground pedal: no balance");
        }
    }
    c.brake_left = 1.0;
    c.brake_right = 1.0;
    summary << "ground_pedal," << c.pedal << "\n";
    summary << "ground_pedal_in," << xp_in(c.pedal) << "\n";

    Model m = Model::from_file(test::uh60a_config_path());
    const auto& gear = ac.get<LandingGear>("gear");
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    double contact = 0.0;
    for (const auto& g : gear.config().gears) {
        contact = std::max(contact, g.axle_b.z() + g.tire.radius);
    }
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -(contact + 0.3));
    m.reset(s);
    Environment env;
    env.ground_height = 0.0;

    std::ofstream f(dir / "m6d_drop.csv");
    f << "t,height,pitch,roll,heading,q,p,n0,n1,n2,lift\n";
    const double T = 40.0;
    const int n = static_cast<int>(std::lround(T * 1000.0));
    std::vector<Sample> hist;
    for (int k = 0; k <= n; ++k) {
        if (k > 0) {
            (void)m.advance(c, env, 1'000'000);
        }
        if (k % 10 != 0) {
            continue;
        }
        const auto st = m.state();
        const std::vector<Wrench> all = ac.evaluate(m.outputs().sim_time, st, c, env);
        const SimContext ctx = ac.context(m.outputs().sim_time, st, c, env);
        const auto xg = ac.states(st, ac.index("gear"));
        const Vec3 f_aero = mr.evaluate(ctx, ac.states(st, ac.index("main_rotor"))).body.F +
                            all[ac.index("airframe")].F + all[ac.index("tail_rotor")].F;
        Sample a{};
        a.t = m.outputs().sim_time;
        a.height = -ctx.body.pos_n.z();
        a.pitch = m.outputs().pitch;
        a.roll = m.outputs().roll;
        a.heading = m.outputs().heading;
        a.q = ctx.body.omega_b.y();
        a.p = ctx.body.omega_b.x();
        a.n0 = gear.gear_state(ctx, xg, 0).normal_load;
        a.n1 = gear.gear_state(ctx, xg, 1).normal_load;
        a.n2 = gear.gear_state(ctx, xg, 2).normal_load;
        a.lift = -(ctx.body.q_nb.toRotationMatrix() * f_aero).z();
        hist.push_back(a);
        f << a.t << "," << a.height << "," << a.pitch << "," << a.roll << "," << a.heading << ","
          << a.q << "," << a.p << "," << a.n0 << "," << a.n1 << "," << a.n2 << "," << a.lift
          << "\n";
    }
    // Settled loads over the last 10 s, and the residual rocking by window.
    double nsum = 0.0, lift = 0.0;
    int cnt = 0;
    for (const Sample& a : hist) {
        if (a.t >= T - 10.0) {
            nsum += a.n0 + a.n1 + a.n2;
            lift += a.lift;
            ++cnt;
        }
    }
    nsum /= cnt;
    lift /= cnt;
    const double w = ac.total_weight();
    summary << "ground_normal_lbf," << nsum / units::kPoundForce << "\n";
    summary << "ground_lift_lbf," << lift / units::kPoundForce << "\n";
    summary << "ground_weight_minus_lift_lbf," << (w - lift) / units::kPoundForce << "\n";
    summary << "ground_balance_error_rel," << (nsum - (w - lift)) / w << "\n";
    for (double t0 = 0.0; t0 < T; t0 += 5.0) {
        double rq = 0.0, rp = 0.0;
        int k = 0;
        int crossings = 0;
        double prev = 0.0;
        for (const Sample& a : hist) {
            if (a.t >= t0 && a.t < t0 + 5.0) {
                rq += a.q * a.q;
                rp += a.p * a.p;
                if (k > 0 && (a.q > 0.0) != (prev > 0.0)) {
                    ++crossings;
                }
                prev = a.q;
                ++k;
            }
        }
        summary << "rocking_" << t0 << "_" << t0 + 5.0 << "_q_rms_degs," << std::sqrt(rq / k) / kDeg
                << ",p_rms_degs," << std::sqrt(rp / k) / kDeg << ",q_freq_hz,"
                << crossings / 2.0 / 5.0 << "\n";
    }
    const Sample& last = hist.back();
    const Sample* at5 = nullptr;
    for (const Sample& a : hist) {
        if (a.t >= 5.0) {
            at5 = &a;
            break;
        }
    }
    summary << "settled_pitch_deg," << last.pitch / kDeg << "\n";
    summary << "settled_roll_deg," << last.roll / kDeg << "\n";
    summary << "heading_change_5s_to_end_deg," << (last.heading - at5->heading) / kDeg << "\n";
}

void cost(std::ostream& summary) {
    Model m = Model::from_file(test::uh60a_config_path());
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -500.0);
    m.reset(s);
    Controls c;
    c.collective = 0.6;
    c.pedal = 0.3;
    const Environment env;
    const int steps = 20000;
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < steps; ++k) {
        (void)m.advance(c, env, 1'000'000);
    }
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    summary << "cost_us_per_step," << us / steps << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m6d_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        const Uh60aHarness ac;
        std::ofstream summary(dir / "m6d_summary.csv");
        hover(ac, summary);
        ground(ac, dir, summary);
        cost(summary);
        summary.close();
        std::ifstream in(dir / "m6d_summary.csv");
        std::cout << in.rdbuf();
    } catch (const std::exception& e) {
        std::cerr << "eyas_m6d_cases: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
