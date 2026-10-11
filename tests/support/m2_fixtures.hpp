#pragma once

#include <array>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "util/units.hpp"

// Shared M2 test aircraft: used by tests/unit/test_model.cpp and the
// VALIDATION.md generator tests/validation/m2_cases.cpp, so both run the
// same case.
namespace eyas::test {

inline constexpr double kM2Mass = 8000.0;  // [kg]

inline MassProperties m2_mass_properties() {
    return make_mass_properties(kM2Mass, 6000.0, 40000.0, 37000.0, 1900.0);
}

// YAML for the M2 aircraft, with the ideal_thrust dummy rotor. Max thrust is
// 2mg, written with 17 significant digits so that it parses back to exactly
// the double 2·m·g; collective 0.5 then gives T = m·g exactly (scaling by 2
// and by 0.5 is exact).
inline std::string m2_yaml() {
    std::array<char, 64> thrust{};
    const int n = std::snprintf(thrust.data(), thrust.size(), "\"%.17g N\"",
                                2.0 * kM2Mass * units::kStandardGravity);
    if (n <= 0 || static_cast<std::size_t>(n) >= thrust.size()) {
        throw std::runtime_error("m2_yaml: formatting failed");
    }
    return std::string(R"(
simulation:
  physics_rate: "1000 Hz"
  max_substeps: 100
geodetic_origin:
  latitude: "37.41 deg"
  longitude: "-122.05 deg"
  altitude: "10 m"
airframe:
  mass: "8000 kg"
  inertia:
    ixx: "6000 kg*m^2"
    iyy: "40000 kg*m^2"
    izz: "37000 kg*m^2"
    ixz: "1900 kg*m^2"
force_models:
  - type: ideal_thrust
    max_thrust: )") +
           thrust.data() + "\n";
}

// A force fixed in inertial space that balances the M2 airframe's weight at
// any attitude: F_b = −m C_bn g_n, using the unit attitude in SimContext.
class InertialWeightBalance : public ForceModel {
public:
    Wrench compute(const SimContext& ctx) const override {
        Wrench w;
        w.F = -kM2Mass * (ctx.body.q_nb.conjugate() * gravity_n());
        return w;
    }
};

}  // namespace eyas::test
