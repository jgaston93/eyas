#pragma once

#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "util/table.hpp"

namespace eyas {
class Config;
}

// Main-rotor wash at a point on the airframe, GenHel style (CR-166309
// §5.3.2 p. 5.3-5 for the stabilator and fin, §5.4.2 pp. 5.4-4 to 5.4-5 for
// the tail rotor; D-028): the rotor's uniform downwash scaled by empirical
// factors tabulated against wake skew χ and longitudinal flapping a₁,
//   V_X = λ0 ΩR E_KX(χ, a₁),   V_Z = −λ0 ΩR E_KZ(χ, a₁)   (body axes),
// added to the point's velocity relative to the air. With GenHel's velocities
// being the aircraft's velocity relative to the air, V_Z < 0 is air moving
// down past the point. GenHel has no lateral factor for the UH-60A (E_KY, "no
// input"), so there is none here.
//
// The rotor's outputs come from the coupling store (D-003): lambda0, mu_x,
// mu_z, beta_1c and tip_speed of the named rotor instance. GenHel's wake skew
// and flapping (pp. 5.1-37 and 5.1-18) are
//   χ = atan(μ_x / |λ|) + a₁,   a₁ = −(2/N_b) Σ β_k cos ψ_k = −β_1c,
// with λ the total flow through the disc, λ0 − μ_z here (D-027), and μ_x the
// hub velocity along the shaft x axis / ΩR. Lookups never allocate or throw.
namespace eyas {

class RotorWash {
public:
    // map: (E_KX, E_KZ) [-] against (χ, a₁) [rad].
    RotorWash(std::string rotor, MultiTable2D<2> map);

    // Reads {rotor: <instance name>, table: <CSV with columns chi, a1 (angles)
    // and ekx, ekz (dimensionless)>}.
    static RotorWash from_config(const Config& node);

    std::vector<std::string> consumes() const;
    void bind(const CouplingRegistry& registry, std::string_view consumer);

    struct Result {
        Vec3 velocity_b = Vec3::Zero();  // [m/s] added to the point's velocity relative to the air
        double chi = 0.0;                // χ [rad], GenHel's wake skew
        double a1 = 0.0;                 // a₁ [rad], GenHel's longitudinal flapping
    };
    // From the bound rotor's outputs in this evaluation.
    Result evaluate(const CouplingOutputs& coupling) const noexcept;
    // From given rotor outputs (tests).
    Result evaluate(double lambda0, double mu_x, double mu_z, double beta_1c,
                    double tip_speed) const noexcept;

    const std::string& rotor() const noexcept { return rotor_; }

private:
    std::string rotor_;
    MultiTable2D<2> map_;
    CouplingSlot lambda0_;
    CouplingSlot mu_x_;
    CouplingSlot mu_z_;
    CouplingSlot beta_1c_;
    CouplingSlot tip_speed_;
    mutable TableCursor<2> cursor_;  // search hint only (spec §6)
};

}  // namespace eyas
