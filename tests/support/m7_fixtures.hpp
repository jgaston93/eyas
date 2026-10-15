#pragma once

// Fixtures for the M7b linearization tests (D-035 and its amendment). Test
// fixtures, not aircraft data.

#include <memory>
#include <utility>
#include <vector>

#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/controls.hpp"
#include "sim/executive.hpp"
#include "support/m4_fixtures.hpp"

namespace eyas::test {

// Linear swashplate map: θ0 = a0 + a·collective, θ1s = −b·lon_cyclic,
// θ1c = b·lat_cyclic [rad].
class LinearPitch : public PitchSchedule {
public:
    LinearPitch(double a0, double a, double b) : a0_(a0), a_(a), b_(b) {}
    SwashplatePitch pitch(const Controls& c) const noexcept override {
        return {a0_ + a_ * c.collective, -b_ * c.lon_cyclic, b_ * c.lat_cyclic};
    }

private:
    double a0_;
    double a_;
    double b_;
};

// One rotor of the symmetric hover fixture: the M4 anchor blade (γ ≈ 8,
// linear airfoil with drag), N blades, a 0.3 m hinge, Pitt–Peters inflow,
// the hub 1.5 m above the CG on a shaft along body z.
inline BladeElementRotorConfig symmetric_rotor(int blades, RotationDirection direction) {
    const double e = 0.3;
    const auto airfoil = airfoil_from(linear_airfoil_csv(kLiftSlope, 0.01));
    BladeElementRotorConfig c = flap_rotor(airfoil, {}, nonuniform_beads(8.0 - e, 12.0), e, 40);
    c.num_blades = blades;
    c.direction = direction;
    c.twist = Table1D(Axis({0.0, 1.0}), {0.0, -8.0 * kDeg});
    c.hub_position_b = Vec3(0.0, 0.0, -1.5);
    c.controls = std::make_shared<const LinearPitch>(0.0, 0.4, 0.1);
    c.inflow = InflowModel::pitt_peters;
    return c;
}

// The symmetric hover fixture (D-035 amendment, L2): two identical flapping
// rotors on one shaft through the CG, turning in opposite directions so their
// torques cancel, over an airframe mass with no aerodynamics. In hover the
// trim needs no cyclic and no attitude, gravity lies along the shaft, and
// every blade sees the same conditions at every azimuth: the rotating-frame
// equations have constant coefficients, so the multiblade ones do too
// (Johnson 2013 pp. 556, 572).
inline Executive symmetric_hover_executive(int blades = 4) {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(4000.0, 3000.0, 10000.0, 9000.0, 0.0);
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<BladeElementRotor>(
        symmetric_rotor(blades, RotationDirection::counter_clockwise)));
    comps.push_back(
        std::make_unique<BladeElementRotor>(symmetric_rotor(blades, RotationDirection::clockwise)));
    return Executive(cfg, std::move(comps));
}

// Flies `ex` free from x at t = 0 by RK4 with step dt [s], renormalizing the
// quaternion every step as the executive does, the controls given by
// controls(t). Returns the state after every step (n = 1…steps) and x
// itself first.
template <class ControlsAt>
std::vector<std::vector<double>> free_flight(const Executive& ex, const Environment& env,
                                             std::vector<double> x, ControlsAt&& controls,
                                             double dt, int steps) {
    const std::size_t n = x.size();
    std::vector<double> k1(n);
    std::vector<double> k2(n);
    std::vector<double> k3(n);
    std::vector<double> k4(n);
    std::vector<double> tmp(n);
    std::vector<std::vector<double>> out;
    out.reserve(static_cast<std::size_t>(steps) + 1);
    out.push_back(x);
    for (int s = 0; s < steps; ++s) {
        const double t = dt * s;
        (void)ex.derivatives(t, x, controls(t), env, k1);
        for (std::size_t i = 0; i < n; ++i) {
            tmp[i] = x[i] + 0.5 * dt * k1[i];
        }
        (void)ex.derivatives(t + 0.5 * dt, tmp, controls(t + 0.5 * dt), env, k2);
        for (std::size_t i = 0; i < n; ++i) {
            tmp[i] = x[i] + 0.5 * dt * k2[i];
        }
        (void)ex.derivatives(t + 0.5 * dt, tmp, controls(t + 0.5 * dt), env, k3);
        for (std::size_t i = 0; i < n; ++i) {
            tmp[i] = x[i] + dt * k3[i];
        }
        (void)ex.derivatives(t + dt, tmp, controls(t + dt), env, k4);
        for (std::size_t i = 0; i < n; ++i) {
            x[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        }
        BodyState b = unpack_body(x);
        b.q_nb.normalize();
        pack_body(b, x);
        out.push_back(x);
    }
    return out;
}

}  // namespace eyas::test
