// Writes Eyas's steady-heading sideslips for comparison with Kaplita, NASA
// CR-177360 (1985), Figs. 11-13 (D-048; VALIDATION.md, "Kaplita steady-heading
// sideslips"; plotted by tools/plot_kaplita.py over the digitized GenHel curves
// and flight test, data/uh60a/validation/kaplita_sideslip*.csv).
//
// As Kaplita's simulated runs (p. 2): SAS on, PBA disabled and centred, FPS
// and trim off, the stabilator held at its zero-sideslip trim incidence. The
// aircraft is Ballin's (uh60a_ballin.yaml) at 5250 ft, a stand-in for the
// three test flights (KaplitaSideslip test header). Two variants: the
// UH-60A as configured ("eyas") and without GenHel's downwash correction
// terms ("no_additions", the config's `additional_loads` removed), to show
// what D-048 changed.
//
// Usage: eyas_kaplita_cases <output-dir>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "control/genhel_fcs.hpp"
#include "support/m7c_ballin.hpp"
#include "support/m8c3_fps.hpp"

namespace {

using namespace eyas;
namespace b = eyas::test::ballin;

void sweep(std::ofstream& out, const char* variant, const std::string& text, int kt) {
    const AfcsSwitches sas{true, true, false, false, false, false};
    const test::Uh60aHarness law(Config::from_string(text, variant));
    TrimCondition c0 = b::condition(kt, 0.0);
    c0.afcs = sas;
    std::vector<double> x0;
    const TrimResult r0 = solve_trim(law.executive(), c0, nullptr, x0);
    if (!r0.converged) {
        throw std::runtime_error("zero-sideslip trim did not converge");
    }
    const double inc =
        x0[test::m8c3::fcs_offset(law.executive()) + GenHelFcs::kServoStab] / b::kDeg;
    const std::regex input(R"(incidence_input: [A-Za-z0-9_]+[^\n]*)");
    std::ostringstream held_inc;
    held_inc.precision(17);
    held_inc << "incidence: \"" << inc << " deg\"";
    const test::Uh60aHarness held(
        Config::from_string(std::regex_replace(text, input, held_inc.str()), variant));
    for (const std::vector<double>& betas :
         {std::vector<double>{0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20},
          std::vector<double>{-2, -4, -6, -8, -10, -12, -14, -16, -18, -20}}) {
        TrimResult prev;
        bool have = false;
        for (const double beta : betas) {
            TrimCondition c = c0;
            c.sideslip = beta * b::kDeg;
            std::vector<double> x;
            const TrimResult r = solve_trim(held.executive(), c, have ? &prev : nullptr, x);
            std::cout << variant << ", " << kt << " kt, " << beta
                      << " deg: " << (r.converged ? "converged" : "NOT CONVERGED") << "\n";
            if (!r.converged) {
                throw std::runtime_error("sideslip trim did not converge");
            }
            prev = r;
            have = true;
            out << variant << "," << kt << "," << beta << "," << inc << ","
                << b::lon_pct(r.controls) << "," << r.pitch / b::kDeg << ","
                << b::lat_pct(r.controls) << "," << r.roll / b::kDeg << ","
                << b::pedal_pct(r.controls) << "," << b::collective_pct(r.controls) << "\n";
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_kaplita_cases <output-dir>\n";
        return 2;
    }
    const std::filesystem::path dir(argv[1]);
    std::filesystem::create_directories(dir);
    const std::string text = b::config_text(b::config_path());
    const std::regex additions(R"(\n\s*additional_loads: [^\n]*)");
    const std::string bare = std::regex_replace(text, additions, "");
    if (bare == text) {
        std::cerr << "the config has no additional_loads to remove\n";
        return 1;
    }
    std::ofstream out(dir / "kaplita_sideslip.csv");
    out << "# Eyas steady-heading sideslips at Kaplita's conditions (NASA CR-177360 Figs. 11-13; "
           "D-048): uh60a_ballin.yaml, ISA 5250 ft, SAS on, stabilator held at its "
           "zero-sideslip trim incidence. Controls in Ballin's percent scales.\n"
        << "variant,knots,sideslip_deg,stabilator_deg,lon_cyclic_pct,pitch_deg,lat_cyclic_pct,"
           "roll_deg,pedal_pct,collective_pct\n";
    try {
        for (const int kt : {60, 100, 140}) {
            sweep(out, "eyas", text, kt);
            sweep(out, "no_additions", bare, kt);
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
