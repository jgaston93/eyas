#pragma once

#include <span>

#include "core/rigid_body.hpp"
#include "core/rk4.hpp"
#include "sim/executive.hpp"

// One free step of the whole state by the scheme Executive::step uses: RK4,
// then the quaternion renormalized. Shared by the linearization's flow map
// (monodromy()) and the tests' free flights. Allocates nothing.
namespace eyas {

// Advances x (num_states() long) from t to t + dt [s], the controls given by
// controls(t) at each RK4 stage. Returns how many of the step's four
// evaluations had a bounded iteration that did not converge (D-023).
template <class ControlsAt>
[[nodiscard]] int free_step(const Executive& ex, Rk4& rk4, ControlsAt&& controls,
                            const Environment& env, double t, double dt, std::span<double> x) {
    int unconverged = 0;
    rk4.step(
        [&](double ts, std::span<const double> xs, std::span<double> xd) {
            if (!ex.derivatives(ts, xs, controls(ts), env, xd)) {
                ++unconverged;
            }
        },
        t, dt, x);
    BodyState b = unpack_body(x);
    b.q_nb.normalize();
    pack_body(b, x);
    return unconverged;
}

}  // namespace eyas
