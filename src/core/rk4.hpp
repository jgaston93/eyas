#pragma once

#include <cstddef>
#include <span>
#include <vector>

// Classical fourth-order Runge–Kutta for the global state vector (spec
// §6.10). Stage buffers are allocated once, in the constructor; step() never
// allocates. Another scheme (semi-implicit, symplectic) can be swapped in by
// providing the same step() signature.
namespace eyas {

class Rk4 {
public:
    explicit Rk4(std::size_t n) : k1_(n), k2_(n), k3_(n), k4_(n), tmp_(n) {}

    // Advances x in place from t to t + h. `f(t, x, xdot)` evaluates the
    // derivative; it is a template parameter (no std::function) so nothing
    // allocates. [s]
    template <class F>
    void step(F&& f, double t, double h, std::span<double> x) noexcept {
        const std::size_t n = x.size();
        f(t, std::span<const double>(x), std::span<double>(k1_));
        for (std::size_t i = 0; i < n; ++i) {
            tmp_[i] = x[i] + 0.5 * h * k1_[i];
        }
        f(t + 0.5 * h, std::span<const double>(tmp_), std::span<double>(k2_));
        for (std::size_t i = 0; i < n; ++i) {
            tmp_[i] = x[i] + 0.5 * h * k2_[i];
        }
        f(t + 0.5 * h, std::span<const double>(tmp_), std::span<double>(k3_));
        for (std::size_t i = 0; i < n; ++i) {
            tmp_[i] = x[i] + h * k3_[i];
        }
        f(t + h, std::span<const double>(tmp_), std::span<double>(k4_));
        for (std::size_t i = 0; i < n; ++i) {
            x[i] += h / 6.0 * (k1_[i] + 2.0 * k2_[i] + 2.0 * k3_[i] + k4_[i]);
        }
    }

private:
    std::vector<double> k1_, k2_, k3_, k4_, tmp_;
};

}  // namespace eyas
