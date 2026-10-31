// Writes the UH-60A frequency responses at the conditions of Fletcher, 21st ERF
// (1995) Paper VII-4, for comparison with its flight data (Figs. 3, 4, 7, 13;
// data/uh60a/validation/fletcher_*.csv; D-035 M7c amendment, V2):
// data/uh60a/uh60a_fletcher.yaml (about 14350 lbm, CG FS 360 in) at sea level
// in the ISA, in hover and at 80 kt (zero sideslip), the stabilator held at the
// test's 40 deg (hover) and 4 deg (80 kt) trailing edge down (Fletcher p.
// VII-4-6), with no stability augmentation (as the SAS/FPS-off flights).
//
// The averaged (constant-coefficient) retained linear model (M7b, D-035) gives
// p, q, r per inch of control: lateral cyclic right positive, longitudinal
// cyclic aft positive (Fletcher's sign: q/delta_lon is in phase at mid
// frequency), collective up positive, with the travels of the config (10 in
// lateral and longitudinal, 10 in collective). The model includes GenHel's
// primary servos, 1/(0.00114 s^2 + 0.0473 s + 1) (CR-166309 Figs.
// 5.1.6(a)-(c), D-045), as the flight data do; the responses are also
// written without them, divided out exactly (the servos are linear and
// unit-gain), for comparison with M7c, which had none.
//
// Also writes, at both conditions, the condensed 6-DOF model's modes and
// derivatives (Padfield App. 4B form, as M7b) and the averaged model's roots
// below 3 rad/s against the Floquet exponents of the free flow map (V2 mode
// character).
//
// Usage: eyas_m7c_fletcher <output-dir> [config [ideal]]  (default
// data/uh60a/uh60a_fletcher.yaml; `ideal`: the rotor shaft held at its
// governed speed, an ideal drive, D-052, the engines kept as components: the
// M7 regression's baseline for the engines' effect, D-061)

#include <array>
#include <cmath>
#include <complex>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "analysis/linearize.hpp"
#include "support/m7c_ballin.hpp"

namespace {

using namespace eyas;
using test::ballin::kDeg;

// The aircraft: Fletcher's weight on GenHel's engines, or another config given
// on the command line (Ballin's engines, data/uh60a/engine_ballin/; D-054).
std::string g_config;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
bool g_ideal = false;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)
std::string fletcher_config() {
    return g_config.empty() ? std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/uh60a_fletcher.yaml"
                            : g_config;
}

constexpr double kBand = 3.0;  // [rad/s] the flight-dynamic band of the Floquet comparison
const std::array<const char*, 6> kRows = {"X", "Y", "Z", "L", "M", "N"};
const std::array<const char*, 13> kCols = {"u",     "v",   "w",   "p",   "q",   "r",  "phi",
                                           "theta", "psi", "col", "lat", "lon", "ped"};

std::complex<double> servo(double w) {
    // GenHel primary servo, 1/(0.00114 s^2 + 0.0473 s + 1); the config's.
    const std::complex<double> s(0.0, w);
    return 1.0 / (0.00114 * s * s + 0.0473 * s + 1.0);
}

void write(const std::filesystem::path& dir) {
    std::ofstream out(dir / "m7c_fletcher.csv");
    out << "# UH-60A at Fletcher's conditions (21st ERF 1995 Paper VII-4): uh60a_fletcher.yaml, "
           "ISA "
           "sea level, stabilator held (M7c, D-035 V2)\n";
    out << "condition,response,frequency_rad_s,magnitude_db,phase_deg,magnitude_db_servo,"
           "phase_deg_servo\n";
    std::ofstream modes_csv(dir / "m7c_fletcher_modes.csv");
    modes_csv.precision(12);
    modes_csv << "# Condensed 6-DOF modes at Fletcher's conditions (M7c, D-035 V2)\n"
              << "condition,label,re,im,wn,zeta,longitudinal\n";
    std::ofstream der_csv(dir / "m7c_fletcher_derivatives.csv");
    der_csv.precision(12);
    der_csv << "# Condensed derivatives (Padfield App. 4B form: gravity and kinematics included; "
               "L, N primed), SI, controls per unit of Controls (M7c, D-035 V2)\n"
            << "condition,derivative,value\n";
    std::ofstream floq_csv(dir / "m7c_fletcher_floquet.csv");
    floq_csv.precision(12);
    floq_csv << "# Averaged model against Floquet exponents of the free flow map, |lambda| < 3 "
                "rad/s (M7c, D-035 V2)\ncondition,avg_re,avg_im,floq_re,floq_im,gap\n";
    struct Case {
        const char* name;
        double kt;
        double stabilator_deg;
    };
    for (const Case& cs : {Case{"hover", 0.0, 40.0}, Case{"80kt", 80.0, 4.0}}) {
        const test::Uh60aHarness ac =
            test::ballin::fixed_stabilator(cs.stabilator_deg, fletcher_config(), g_ideal);
        TrimCondition c;
        c.airspeed = cs.kt * units::kKnot;
        c.altitude = 0.0;
        std::vector<double> x;
        const TrimResult r = solve_trim(ac.executive(), c, nullptr, x);
        if (!r.converged) {
            throw std::runtime_error(std::string("trim did not converge: ") + cs.name);
        }
        const LinearModel m = linearize(ac.executive(), c, r);
        if (m.unconverged != 0) {
            throw std::runtime_error(std::string("linearize: unconverged evaluations: ") + cs.name);
        }
        std::cout << cs.name << ": trimmed (pitch " << r.pitch / kDeg << " deg, roll "
                  << r.roll / kDeg << " deg), " << m.states.size() << " states\n";
        const CondensedModel cm = condense(m);
        for (const Mode& md : modes(cm, r.vel_b.x())) {
            modes_csv << cs.name << "," << md.label << "," << md.eigenvalue.real() << ","
                      << md.eigenvalue.imag() << "," << md.frequency << "," << md.damping << ","
                      << md.longitudinal << "\n";
        }
        for (Eigen::Index row = 0; row < 6; ++row) {
            for (Eigen::Index col = 0; col < 13; ++col) {
                der_csv << cs.name << "," << kRows[static_cast<std::size_t>(row)] << "_"
                        << kCols[static_cast<std::size_t>(col)] << ","
                        << (col < 9 ? cm.A(row, col) : cm.B(row, col - 9)) << "\n";
            }
        }
        const Eigen::VectorXcd lam = m.A.eigenvalues();
        const Monodromy M = monodromy(ac.executive(), c, r);
        if (M.unconverged != 0) {
            throw std::runtime_error(std::string("monodromy: unconverged evaluations: ") + cs.name);
        }
        const FloquetModes f = floquet(M.matrix, m.period, lam);
        for (Eigen::Index i = 0; i < f.exponents.size(); ++i) {
            const auto k = f.match[static_cast<std::size_t>(i)];
            if (k < 0 || std::abs(lam(k)) >= kBand || lam(k).imag() < 0.0) {
                continue;
            }
            floq_csv << cs.name << "," << lam(k).real() << "," << lam(k).imag() << ","
                     << f.exponents(i).real() << "," << f.exponents(i).imag() << ","
                     << std::abs(f.exponents(i) - lam(k)) << "\n";
        }
        // Controls units per inch: lateral and longitudinal ±1 over 10 in, collective 0..1
        // over 10 in; longitudinal sign flipped to aft positive.
        struct Resp {
            const char* name;
            std::size_t state;  // p, q, r = 3, 4, 5 in the reduced coordinates
            std::size_t input;  // collective, lat_cyclic, lon_cyclic, pedal
            double per_inch;
        };
        const std::array<Resp, 5> resps = {{{"p_lat", 3, 1, 1.0 / 5.0},
                                            {"p_lon", 3, 2, -1.0 / 5.0},
                                            {"q_lat", 4, 1, 1.0 / 5.0},
                                            {"q_lon", 4, 2, -1.0 / 5.0},
                                            {"r_col", 5, 0, 1.0 / 10.0}}};
        for (const Resp& rp : resps) {
            double prev = 0.0;
            double unwrap = 0.0;
            bool first = true;
            for (int k = -24; k <= 32; ++k) {  // 0.1 to 20 rad/s, 24 per decade (+ 2 rad/s)
                const double w = std::pow(10.0, k / 24.0);
                if (w > 20.0 + 1e-9) {
                    break;
                }
                const Eigen::MatrixXcd H = frequency_response(m.A, m.B, w);
                // With the servos (the model), and without them.
                const std::complex<double> hs =
                    H(static_cast<Eigen::Index>(rp.state), static_cast<Eigen::Index>(rp.input)) *
                    rp.per_inch;
                const std::complex<double> h = hs / servo(w);
                double ph = std::arg(h) / kDeg;
                if (!first) {  // unwrap
                    while (ph + unwrap - prev > 180.0) {
                        unwrap -= 360.0;
                    }
                    while (ph + unwrap - prev < -180.0) {
                        unwrap += 360.0;
                    }
                }
                ph += unwrap;
                prev = ph;
                first = false;
                const double phs = ph + std::arg(servo(w)) / kDeg;
                out << cs.name << "," << rp.name << "," << w << ","
                    << 20.0 * std::log10(std::abs(h)) << "," << ph << ","
                    << 20.0 * std::log10(std::abs(hs)) << "," << phs << "\n";
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4 || (argc == 4 && std::string(argv[3]) != "ideal")) {
        std::cerr << "usage: eyas_m7c_fletcher <output-dir> [config [ideal]]\n";
        return 2;
    }
    if (argc >= 3) {
        g_config = argv[2];
    }
    g_ideal = argc == 4;
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write(dir);
    } catch (const std::exception& e) {
        std::cerr << "eyas_m7c_fletcher: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
