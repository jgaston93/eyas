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

}  // namespace eyas::test
