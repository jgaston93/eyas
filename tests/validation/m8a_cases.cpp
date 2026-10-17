// Writes the M8a long-run data to CSV for VALIDATION.md (tools/plot_m8a.py;
// D-042): over `minutes` (default 30) of simulated time,
// - (b) the UH-60A's held periodic orbit at hover and 120 kt: the drift of
//   the component states, revolution by revolution;
// - (c) the M2 dummy-rotor equilibrium: the largest state change (exact: 0);
// - (d) the free UH-60A parked on its gear, brakes set: position, attitude,
//   speeds and quaternion norm error once a second.
//
// Usage: eyas_m8a_cases <output-dir> [minutes]

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "support/m8a_long_runs.hpp"

namespace {

using namespace eyas;

void write(const std::filesystem::path& dir, int minutes) {
    const int seconds = minutes * 60;
    {
        std::ofstream f(dir / "m8a_held_orbit.csv");
        f << "# UH-60A held periodic orbit, ISA sea level, RK4 at T/256 (D-042 b)\n"
          << "speed_kt,revolution,time_s,deviation\n";
        f.precision(17);
        for (const double kt : {0.0, 120.0}) {
            // Revolutions covering `seconds` at the trim period.
            const double T = test::held_orbit_drift(kt, 0).period;
            const int revs = static_cast<int>(seconds / T);
            const test::HeldOrbitDrift d = test::held_orbit_drift(kt, revs);
            if (!d.converged) {
                throw std::runtime_error("trim did not converge at " + std::to_string(kt) + " kt");
            }
            std::cout << kt << " kt: " << revs << " revolutions, trim periodic residual "
                      << d.periodic_residual << ", final deviation " << d.deviation.back()
                      << ", unconverged evaluations " << d.unconverged << "\n";
            for (std::size_t n = 0; n < d.deviation.size(); ++n) {
                f << kt << ',' << n + 1 << ',' << static_cast<double>(n + 1) * d.period << ','
                  << d.deviation[n] << '\n';
            }
        }
    }
    {
        const test::DummyEquilibrium d = test::dummy_equilibrium(seconds);
        std::ofstream f(dir / "m8a_dummy.csv");
        f << "# M2 dummy-rotor equilibrium (D-042 c)\nseconds,max_change,time_exact\n"
          << seconds << ',' << d.max_change << ',' << (d.time_exact ? 1 : 0) << '\n';
        std::cout << "dummy: max change " << d.max_change << ", time exact " << d.time_exact
                  << "\n";
    }
    {
        const std::vector<test::ParkedSample> p = test::parked_run(seconds);
        std::ofstream f(dir / "m8a_parked.csv");
        f << "# UH-60A parked, brakes set, collective 0.2, pedal 0.5 (D-042 d)\n"
          << "time_s,x_m,y_m,z_m,roll_rad,pitch_rad,heading_rad,speed_mps,rate_radps,"
             "quat_norm_error,weight_on_wheels,finite\n";
        f.precision(17);
        for (const test::ParkedSample& s : p) {
            f << s.t << ',' << s.pos_n.x() << ',' << s.pos_n.y() << ',' << s.pos_n.z() << ','
              << s.roll << ',' << s.pitch << ',' << s.heading << ',' << s.speed << ',' << s.rate
              << ',' << s.quat_norm_error << ',' << (s.weight_on_wheels ? 1 : 0) << ','
              << (s.finite ? 1 : 0) << '\n';
        }
        const test::ParkedSample& e = p.back();
        std::cout << "parked: t " << e.t << " s, pos " << e.pos_n.transpose() << ", speed "
                  << e.speed << ", rate " << e.rate << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) {
        std::fprintf(stderr, "usage: %s <output-dir> [minutes]\n", argv[0]);
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write(dir, argc == 3 ? std::stoi(argv[2]) : 30);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    return 0;
}
