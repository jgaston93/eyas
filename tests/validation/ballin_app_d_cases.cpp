// Writes Eyas's responses to Ballin's App. D and E control histories (NASA
// TM-88360, 1987; D-046) for VALIDATION.md (tools/plot_ballin_app_d.py): each
// Table C1 case trimmed at its test conditions and flown through the
// digitized inputs (tests/support/ballin_app_d.hpp), once with GenHel's
// primary servos (CR-166309, App. E's Sikorsky simulation) and once with
// Ames's (Ballin App. A, App. D's simulation). A case with a stand-in config
// variant (File 210's doubtful waterline, DATA_GAPS.md) is flown with each,
// written with the variant's suffix.
//
// Usage: eyas_ballin_app_d_cases <output-dir> [figure ...]
// Exits 1 if a trim fails to converge.

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#include "support/ballin_app_d.hpp"

namespace {

using namespace eyas;
using namespace eyas::test::ballin_d;

// Returns false if the trim didn't converge.
bool write(const std::filesystem::path& dir, const Case& c, Servo servo, std::ofstream& summary) {
    const char* name = servo == Servo::ames ? "ames_servo" : "genhel_servo";
    const Run run = fly(c, servo);
    std::string fig = c.figure;
    for (auto& ch : fig) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    const std::string variant = c.variant.empty() ? "" : "_" + c.variant;
    std::ofstream out(dir / ("ballin_" + fig + "_" + name + variant + ".csv"));
    out << "# Eyas flown through Ballin Fig. " << c.figure << " (file " << c.file << variant
        << "), " << name << "; trim converged " << run.trim_converged << ", stabilator held at "
        << run.stabilator_deg << " deg, " << run.airspeed_kt << " kt true, altitude "
        << run.air.altitude << " m, ISA dT " << run.air.delta_T << " K; " << run.unconverged
        << " unconverged evaluations (D-046)\n";
    out << "t_s,collective_norm,lat_cyclic_norm,lon_cyclic_norm,pedal_norm,pitch_deg,roll_deg,"
           "heading_deg,p_dps,q_dps,r_dps,az_sensor_fps2\n";
    out.precision(10);
    for (const Sample& s : run.samples) {
        out << s.t << "," << s.u.collective << "," << s.u.lat_cyclic << "," << s.u.lon_cyclic << ","
            << s.u.pedal << "," << s.pitch << "," << s.roll << "," << s.heading << "," << s.p << ","
            << s.q << "," << s.r << "," << s.az_sensor << "\n";
    }
    summary << c.figure << "," << c.file << variant << "," << name << "," << run.trim_converged
            << "," << run.airspeed_kt << "," << run.air.altitude << "," << run.air.delta_T << ","
            << run.stabilator_deg << "," << run.trim.pitch / kDeg << "," << run.trim.roll / kDeg
            << "," << run.unconverged << "\n";
    std::cout << c.figure << variant << " " << name << ": trim "
              << (run.trim_converged ? "ok" : "FAILED") << ", stabilator " << run.stabilator_deg
              << " deg\n";
    return run.trim_converged;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: eyas_ballin_app_d_cases <output-dir> [figure ...]\n";
        return 2;
    }
    const std::filesystem::path dir = argv[1];
    std::filesystem::create_directories(dir);
    std::set<std::string> only;
    for (int i = 2; i < argc; ++i) {
        only.insert(argv[i]);
    }
    std::ofstream summary(dir / "ballin_app_d_summary.csv");
    summary << "figure,file,servo,trim_converged,true_airspeed_kt,altitude_m,isa_delta_T_K,"
               "stabilator_deg,pitch_deg,roll_deg,unconverged\n";
    bool ok = true;
    for (const Case& c : cases()) {
        if (!only.empty() && !only.contains(c.figure)) {
            continue;
        }
        for (const Case& v : with_variants(c)) {
            ok = write(dir, v, Servo::genhel, summary) && ok;
            ok = write(dir, v, Servo::ames, summary) && ok;
        }
    }
    return ok ? 0 : 1;
}
