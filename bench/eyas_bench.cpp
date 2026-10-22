// Real-time timing benchmark (spec §7 rule 5, as replaced by D-005): the
// UH-60A's step() and advance() time, mean, p99 and max, in a hover trim, a
// 120 kt trim, a 10 m/s vertical descent in the vortex-ring state (D-049), a
// hover in ground effect (D-050; all four with the whole AFCS on, M8c3) and on
// the ground. D-005's rule: the worst
// advance() of a nominal host frame stays below 50 % of the frame; equivalently, the worst step() ×
// ceil(dt_host / dt_physics) does. Run the release build on the target (D-005, D-037), idle and
// with --load.
//
// Usage: eyas_bench [--config <uh60a.yaml>] [--frames N] [--rate HZ]
//                   [--load THREADS] [--csv <file>]
//
// step() is timed through the executive with a one-step dt (internal API);
// advance() through Model at the host rate, outputs included, as a host sees
// it. Each case starts from its trim (or settled ground state) and returns to
// it every 5 s of sim time, so the bare, unstable aircraft stays near the
// condition; the restore is not timed.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "eyas/model.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;
using Clock = std::chrono::steady_clock;

struct Options {
    std::string config = EYAS_BENCH_DEFAULT_CONFIG;
    int frames = 3600;  // host frames per case
    int rate = 60;      // [Hz] host rate
    int load = 0;       // busy threads
    std::string csv;
};

struct Stats {
    double mean = 0.0;  // [µs]
    double p99 = 0.0;
    double max = 0.0;
};

Stats stats(std::vector<double> us) {
    Stats s;
    if (us.empty()) {
        return s;
    }
    double sum = 0.0;
    for (const double v : us) {
        sum += v;
    }
    s.mean = sum / static_cast<double>(us.size());
    s.max = *std::max_element(us.begin(), us.end());
    const auto k = static_cast<std::size_t>(std::ceil(0.99 * static_cast<double>(us.size()))) - 1;
    std::nth_element(us.begin(), us.begin() + static_cast<std::ptrdiff_t>(k), us.end());
    s.p99 = us[k];
    return s;
}

double elapsed_us(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
}

// One benchmark condition: the inputs to hold and the state to start from,
// as a full state vector (for the executive) and a Model snapshot.
struct Case {
    std::string name;
    Controls controls;
    Environment env;
    std::vector<double> state;
    Snapshot snapshot;
};

std::vector<Case> make_cases(Model& m) {
    std::vector<Case> cases;
    struct Trim {
        const char* name;
        double airspeed;           // [m/s]
        double flight_path_angle;  // [rad]
        double ground_height;      // [m] MSL; NaN: no ground
    };
    constexpr double kNoGround = std::numeric_limits<double>::quiet_NaN();
    // The vortex-ring case descends at about 0.87 v_h, mid-region, where the
    // inflow inverts Johnson's curve every evaluation (D-049). The ground-
    // effect case hovers with the CG at MSL 0 and the ground 2.75 m below it,
    // the wheels about 3 ft up (hub z/R ≈ 0.55; D-050).
    for (const Trim& t :
         {Trim{"hover", 0.0, 0.0, kNoGround}, Trim{"120kt", 120.0 * units::kKnot, 0.0, kNoGround},
          Trim{"vrs", 10.0, -0.5 * std::numbers::pi, kNoGround}, Trim{"ige", 0.0, 0.0, -2.75}}) {
        TrimCondition c;
        c.airspeed = t.airspeed;
        c.flight_path_angle = t.flight_path_angle;
        c.ground_height = t.ground_height;
        // The whole AFCS on, as a pilot flies (SAS, PBA, FPS; M8c3). A
        // config without one ignores the switches.
        c.afcs = {true, true, true, true, false, false};
        const TrimResult r = m.trim(c);
        if (!r.converged) {
            throw std::runtime_error(std::string("trim did not converge: ") + t.name);
        }
        Environment env;
        env.ground_height = t.ground_height;
        cases.push_back({t.name, r.controls, env,
                         std::vector<double>(m.state().begin(), m.state().end()),
                         m.make_snapshot()});
    }
    // On the ground: dropped from 2.2 m, brakes set, settled for 10 s.
    Case g{"ground", {}, {}, {}, {}};
    g.controls.collective = 0.2;
    g.controls.pedal = 0.5;
    g.controls.brake_left = 1.0;
    g.controls.brake_right = 1.0;
    g.env.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);
    (void)m.reset(s);
    for (int f = 0; f < 600; ++f) {
        (void)m.advance(g.controls, g.env, 16'666'667);
    }
    if (!m.outputs().weight_on_wheels) {
        throw std::runtime_error("ground case did not settle on its gear");
    }
    g.state.assign(m.state().begin(), m.state().end());
    g.snapshot = m.make_snapshot();
    cases.push_back(std::move(g));
    return cases;
}

// One busy thread of host load. std::jthread asks it to stop when it is
// destroyed, on every way out of run(), exceptions included.
void busy(const std::stop_token& stop) {
    volatile double x = 1.0;
    while (!stop.stop_requested()) {
        x = std::sqrt(x * 1.000001 + 1.0);
    }
}

int run(const Options& opt) {
    Model model = Model::from_file(opt.config);
    const std::vector<Case> cases = make_cases(model);
    const Config cfg = Config::from_file(opt.config);
    Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);

    const std::int64_t step_ns = ex.step_ns();
    const std::int64_t frame_ns = (1'000'000'000 + opt.rate / 2) / opt.rate;
    const std::int64_t steps_per_frame = (frame_ns + step_ns - 1) / step_ns;  // ceil
    const auto restore_every = static_cast<int>(5'000'000'000 / frame_ns);    // 5 s of frames

    std::vector<std::jthread> load;
    load.reserve(static_cast<std::size_t>(opt.load));
    for (int i = 0; i < opt.load; ++i) {
        load.emplace_back([](const std::stop_token& stop) { busy(stop); });
    }

    std::ofstream csv;
    if (!opt.csv.empty()) {
        csv.open(opt.csv);
        csv << "case,kind,index,us\n";
    }
    std::printf("Eyas timing benchmark: %s\n", opt.config.c_str());
    std::printf(
        "host %d Hz (%lld ns), physics step %lld ns, %lld steps per frame, %d busy "
        "threads, %u hardware threads\n",
        opt.rate, static_cast<long long>(frame_ns), static_cast<long long>(step_ns),
        static_cast<long long>(steps_per_frame), opt.load, std::thread::hardware_concurrency());
    std::printf("%-7s %-8s %10s %10s %10s   [µs]\n", "case", "call", "mean", "p99", "max");

    bool pass = true;
    for (const Case& c : cases) {
        // step(): one-step advance() calls through the executive.
        const auto n_steps = static_cast<std::size_t>(opt.frames * steps_per_frame);
        std::vector<double> step_us;
        step_us.reserve(n_steps);
        for (std::size_t i = 0; i < n_steps + 100; ++i) {  // 100 warm-up steps
            if (i % 5000 == 0) {
                (void)ex.restore(c.state, 0, 0);
            }
            const Clock::time_point t0 = Clock::now();
            (void)ex.advance(c.controls, c.env, step_ns);
            const Clock::time_point t1 = Clock::now();
            if (i >= 100) {
                step_us.push_back(elapsed_us(t0, t1));
            }
        }
        // advance(): host frames through Model, outputs included.
        std::vector<double> frame_us;
        frame_us.reserve(static_cast<std::size_t>(opt.frames));
        for (int f = 0; f < opt.frames + 10; ++f) {  // 10 warm-up frames
            if (f % restore_every == 0) {
                (void)model.restore(c.snapshot);
            }
            const Clock::time_point t0 = Clock::now();
            (void)model.advance(c.controls, c.env, frame_ns);
            const Clock::time_point t1 = Clock::now();
            if (f >= 10) {
                frame_us.push_back(elapsed_us(t0, t1));
            }
        }
        const Stats s = stats(step_us);
        const Stats a = stats(frame_us);
        std::printf("%-7s %-8s %10.2f %10.2f %10.2f\n", c.name.c_str(), "step()", s.mean, s.p99,
                    s.max);
        std::printf("%-7s %-8s %10.2f %10.2f %10.2f\n", c.name.c_str(), "advance()", a.mean, a.p99,
                    a.max);
        // The verdict is the measured worst advance() (D-042). The worst
        // step() × steps per frame bounds it only while step times are set
        // by arithmetic; under load the worst step is a preemption, which
        // doesn't recur in every step of a frame, so the product is reported
        // for information.
        const double budget = 0.5 * static_cast<double>(frame_ns) * 1e-3;  // [µs]
        const bool ok = a.max < budget;
        pass = pass && ok;
        std::printf(
            "%-7s D-005: worst advance() %.0f µs against a budget of %.0f µs: %s "
            "(worst step() × %lld = %.0f µs)\n",
            c.name.c_str(), a.max, budget, ok ? "met" : "NOT MET",
            static_cast<long long>(steps_per_frame), s.max * static_cast<double>(steps_per_frame));
        if (csv) {
            for (std::size_t i = 0; i < step_us.size(); ++i) {
                csv << c.name << ",step," << i << ',' << step_us[i] << '\n';
            }
            for (std::size_t i = 0; i < frame_us.size(); ++i) {
                csv << c.name << ",advance," << i << ',' << frame_us[i] << '\n';
            }
        }
    }
    return pass ? 0 : 3;
}

Options parse(int argc, char** argv) {
    Options opt;
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const auto value = [&]() -> std::string {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("missing value after " + std::string(args[i]));
            }
            return std::string(args[++i]);
        };
        if (args[i] == "--config") {
            opt.config = value();
        } else if (args[i] == "--frames") {
            opt.frames = std::stoi(value());
        } else if (args[i] == "--rate") {
            opt.rate = std::stoi(value());
        } else if (args[i] == "--load") {
            opt.load = std::stoi(value());
        } else if (args[i] == "--csv") {
            opt.csv = value();
        } else {
            throw std::runtime_error("unknown option " + std::string(args[i]) +
                                     "; usage: eyas_bench [--config F] [--frames N] [--rate HZ] "
                                     "[--load THREADS] [--csv FILE]");
        }
    }
    if (opt.frames < 1 || opt.rate < 1 || opt.load < 0) {
        throw std::runtime_error("--frames and --rate must be positive, --load >= 0");
    }
    return opt;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(parse(argc, argv));
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}
