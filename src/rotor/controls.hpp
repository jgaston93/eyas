#pragma once

#include <memory>

#include "core/types.hpp"

namespace eyas {
class Config;
}

// Control mapping (spec §6.3, D-011, D-024): normalized pilot inputs to a
// rotor's swashplate pitch. Each rotor owns its map. Called in every
// derivative evaluation: no allocation, no exceptions.
namespace eyas {

// Blade pitch harmonics: θ = θ0 + θ1c cos ψ + θ1s sin ψ at the hub center
// (Padfield 3rd ed. Eq. 3.59); blade twist is added by the rotor.
struct SwashplatePitch {
    double theta0 = 0.0;   // [rad] collective
    double theta1s = 0.0;  // [rad] longitudinal cyclic (sin ψ)
    double theta1c = 0.0;  // [rad] lateral cyclic (cos ψ)
};

// Interface, so a nonlinear or mode-dependent mixer can replace the linear
// map later (D-011).
class PitchSchedule {
public:
    PitchSchedule() = default;
    PitchSchedule(const PitchSchedule&) = delete;
    PitchSchedule& operator=(const PitchSchedule&) = delete;
    virtual ~PitchSchedule() = default;

    virtual SwashplatePitch pitch(const Controls& controls) const noexcept = 0;
};

// One output of the linear map: offset + Σ gain × input.
struct LinearPitchRow {
    double offset = 0.0;      // [rad]
    double collective = 0.0;  // [rad] per unit input
    double lon_cyclic = 0.0;  // [rad] per unit input
    double lat_cyclic = 0.0;  // [rad] per unit input
    double pedal = 0.0;       // [rad] per unit input

    double operator()(const Controls& c) const noexcept {
        return offset + collective * c.collective + lon_cyclic * c.lon_cyclic +
               lat_cyclic * c.lat_cyclic + pedal * c.pedal;
    }
};

// Linear gearing with mixing (spec §6.3): each pitch output is an offset plus
// a gain on every input, so mechanical mixing is a nonzero cross gain.
class LinearControlMap : public PitchSchedule {
public:
    LinearControlMap(LinearPitchRow theta0, LinearPitchRow theta1s, LinearPitchRow theta1c);

    // Reads the map from config, e.g.
    //   theta0:  {offset: "1 deg", collective: "16 deg"}
    //   theta1s: {lon_cyclic: "-10 deg"}
    //   theta1c: {lat_cyclic: "8 deg"}
    // Every row and every gain is optional and defaults to zero; all are
    // angles. Throws InitError naming the key.
    static std::unique_ptr<LinearControlMap> from_config(const Config& node);

    SwashplatePitch pitch(const Controls& controls) const noexcept override {
        return {theta0_(controls), theta1s_(controls), theta1c_(controls)};
    }

private:
    LinearPitchRow theta0_;
    LinearPitchRow theta1s_;
    LinearPitchRow theta1c_;
};

}  // namespace eyas
