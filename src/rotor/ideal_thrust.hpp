#pragma once

#include "core/component.hpp"

// The M2 "dummy rotor" (spec §9 M2, D-018): a massless thrust along −z_b
// proportional to collective, with no moment. Test scaffolding until the
// blade-element rotor (M3); selectable from config as `ideal_thrust`.
namespace eyas {

class IdealThrust : public ForceModel {
public:
    // Thrust at collective = 1 [N]. Throws InitError unless positive and finite.
    explicit IdealThrust(double max_thrust);

    // F_b = (0, 0, −max_thrust · collective); M_b = 0. Collective is used as
    // given (nominally [0, 1]).
    Wrench compute(const SimContext& ctx) const override;

private:
    double max_thrust_;
};

}  // namespace eyas
