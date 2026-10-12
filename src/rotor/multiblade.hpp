#pragma once

#include <cmath>
#include <numbers>
#include <span>

// Multiblade (disc) coordinates of the flap states (D-026), for outputs,
// tests and the rotor's coupling outputs (D-028). No allocation.
namespace eyas {

struct MultibladeFlap {
    double beta0 = 0.0;   // [rad] coning
    double beta0d = 0.0;  // [rad] differential coning (even N_b only, else 0)
    double beta1c = 0.0;  // [rad] longitudinal disc tilt (+: up at ψ = 0, aft)
    double beta1s = 0.0;  // [rad] lateral disc tilt (+: up at ψ = 90°)
};

// From rotor states x (x[2k] = β_k, as BladeElementRotor) at rotor azimuth
// ψ [rad], with blade k at ψ − 2π k/N_b (D-024). Padfield 3rd ed.
// Eqs. 3.42–3.45 (blade i = k + 1, (−1)^i).
inline MultibladeFlap multiblade_flap(std::span<const double> x, double psi,
                                      int num_blades) noexcept {
    MultibladeFlap m;
    const double n = num_blades;
    for (int k = 0; k < num_blades; ++k) {
        const double b = x[2 * static_cast<std::size_t>(k)];
        const double p = psi - 2.0 * std::numbers::pi * k / n;
        m.beta0 += b / n;
        m.beta0d += (k % 2 == 0 ? -b : b) / n;  // (−1)^i with i = k + 1
        m.beta1c += 2.0 * b * std::cos(p) / n;
        m.beta1s += 2.0 * b * std::sin(p) / n;
    }
    if (num_blades % 2 != 0) {
        m.beta0d = 0.0;
    }
    return m;
}

}  // namespace eyas
