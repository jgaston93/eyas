#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "rotor/rotor_wash.hpp"
#include "util/table.hpp"
#include "util/units.hpp"

namespace eyas {
class Config;
}

// Airframe aerodynamics, GenHel style (CR-166309 §5.2.2 fuselage, §5.3.2
// empennage; spec §6.7, §6.8; D-028): a fuselage from wind-tunnel tables, one
// stabilator panel and one fin panel.
//
// Fuselage (pp. 5.2-5 to 5.2-9). Velocity relative to the air at the CG plus
// main-rotor wash (no body-rate terms, as GenHel). GenHel's tunnel angles
//   α_WF = atan(V_Z/|V_X|),  β_WF = atan(V_Y/√(V_X² + V_Z²)),  ψ_WF = −β_WF;
// six coefficients per unit dynamic pressure from tables in α_WF and ψ_WF,
// in wind axes at the tunnel mounting point, moved to the CG.
//
// Panels (pp. 5.3-5 to 5.3-12). Local velocity with body rates, main-rotor
// wash, the fuselage's dynamic-pressure ratio K_Q² and its delayed downwash
// (stabilator) or sidewash (fin):
//   V = K_Q v_b + ω × r + v_wash + v_fuselage-wake,
//   v_wake = −u ε_d K_Q (stabilator, along z) or −u σ_d K_Q (fin, along y),
// u the body x velocity; lift and drag from tables in the panel's angle plus
// its incidence.
//
// Wind axes (D-030): table lookups use GenHel's angles, but the wind-to-body
// rotation uses the flow's actual direction, α' = atan2(V_Z, V_X), so that
// drag always opposes the flow. For V_X > 0 this is GenHel exactly.
//
// Sideward fade (D-031, SPEC_REVIEW #27): on the line V_X = V_Z = 0 the wind
// axes are singular. Within `sideward_fade` of it (|β| > 90° − c) every
// lookup angle derived from α is multiplied by
//   s = smoothstep(cos β / sin c),  smoothstep(t) = t²(3 − 2t), s = 1 for t ≥ 1,
// and of each wind-axis force or moment w, w_x stays along the flow while
// (0, w_y, w_z) is s·R(α', β) + (1 − s)·R(0, β) applied to it. The loads are
// then continuous across the line, and drag still opposes the flow. GenHel's
// own low-speed filter (p. 5.2-10, not executed) is the model for the fade.
//
// States (D-028): the delayed fuselage downwash at the stabilator and
// sidewash at the fin, each a first-order lag standing in for GenHel's
// transport delay l/u: dε_d/dt = (|u|/l)(ε − ε_d), l the panel's distance aft
// of the CG. G = H = 0.
//
// Coupling (D-003): produces <name>.alpha_wf and <name>.psi_wf [rad] for the
// tail rotor; consumes the main rotor's outputs through the wash maps.
namespace eyas {

// GenHel Tables 2.5.3–2.5.11 as tools/build_uh60a_airframe.py merges them.
// Per unit dynamic pressure: forces [m²], moments [m³], in GenHel's wind axes
// (drag aft, side force, lift up; rolling, pitching, yawing moment).
struct FuselageTables {
    // Against α_WF [rad].
    Table1D drag{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D lift{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D pitch{Axis({0.0, 1.0}), {0.0, 0.0}};
    // Against ψ_WF [rad]; side, roll and pitch_increment are odd, drag_increment even.
    Table1D drag_increment{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D side{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D lift_increment{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D roll{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D pitch_increment{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D yaw{Axis({0.0, 1.0}), {0.0, 0.0}};
};

// One empennage panel. The stabilator's lift acts in the x–z plane and its
// angle is α_H = atan(V_Z/|V_X|) + incidence; the fin's lift acts along y and
// its angle is β_V = atan(V_Y/√(V_X² + V_Z²)) + incidence.
// The stabilator's incidence schedule (GenHel Fig. 5.1.7, steady state:
// airspeed lag settled, no pitch-rate or lateral-acceleration feedback, servo
// at unit gain; D-028): i = clamp(command(u, collective), min, max), with u
// the body x velocity relative to the air. The dynamic law is the AFCS's
// (D-029).
struct IncidenceSchedule {
    // [rad] against airspeed u [m/s] and collective [0, 1].
    Table2D command{Axis({0.0, 1.0}), Axis({0.0, 1.0}), {0.0, 0.0, 0.0, 0.0}};
    double min = 0.0;  // [rad] travel limits
    double max = 0.0;
};

struct PanelConfig {
    Vec3 position_b = Vec3::Zero();  // [m] from the airframe CG, body axes; x < 0 (aft)
    double area = 0.0;               // S [m²]
    double incidence = 0.0;          // [rad] fixed (stabilator: GenHel's STBSET override)
    std::optional<IncidenceSchedule> schedule;  // stabilator: replaces `incidence`
    Table1D cl{Axis({0.0, 1.0}), {0.0, 0.0}};   // against the panel angle [rad]
    Table1D cd{Axis({0.0, 1.0}), {0.0, 0.0}};
    // Fuselage wake, against α_WF (stabilator) or ψ_WF (fin) [rad]:
    // dynamic-pressure ratio K_Q² [-] and wake angle ε or σ [rad].
    Table1D q_ratio{Axis({0.0, 1.0}), {1.0, 1.0}};
    Table1D wake_angle{Axis({0.0, 1.0}), {0.0, 0.0}};
};

struct AirframeAeroConfig {
    std::string name;                      // instance name (D-003); may be empty
    Vec3 fuselage_point_b = Vec3::Zero();  // [m] tunnel mounting point from the CG
    FuselageTables fuselage;
    std::optional<RotorWash> fuselage_wash;  // GenHel Tables 2.5.1–2.5.2
    std::optional<RotorWash> tail_wash;      // Tables 3.5.1–3.5.2, stabilator and fin
    PanelConfig stabilator;
    PanelConfig fin;
    // c [rad] in [0, 90°): half-width of the fade about the sideward line
    // (D-031); 0 turns the fade off.
    double sideward_fade = 20.0 * units::kDegree;
};

struct AirframeAeroLoads {
    Wrench body;  // total, body axes, about the airframe CG
    Wrench fuselage, stabilator, fin;
    double alpha_wf = 0.0, psi_wf = 0.0;  // [rad] GenHel's tunnel angles
    double q_wf = 0.0;                    // [Pa]
    double alpha_h = 0.0, q_h = 0.0;      // stabilator angle incl. incidence [rad], [Pa]
    double stabilator_incidence = 0.0;    // [rad] fixed or scheduled
    double beta_v = 0.0, q_v = 0.0;       // fin angle incl. incidence [rad], [Pa]
    double downwash = 0.0;                // ε at the stabilator, undelayed [rad]
    double sidewash = 0.0;                // σ at the fin, undelayed [rad]
    double downwash_rate = 0.0;           // dε_d/dt [rad/s]
    double sidewash_rate = 0.0;           // dσ_d/dt [rad/s]
    // Local velocity relative to the air at each panel, body axes [m/s]
    // (V_Z < 0: the air moves down past the panel).
    Vec3 v_stabilator_b = Vec3::Zero();
    Vec3 v_fin_b = Vec3::Zero();
};

class AirframeAero : public DynamicComponent {
public:
    // Throws InitError on an invalid configuration.
    explicit AirframeAero(AirframeAeroConfig config);

    // Reads a `type: airframe_aero` config node.
    static std::unique_ptr<AirframeAero> from_config(const Config& node);

    // States: x[0] = ε_d, x[1] = σ_d [rad], starting at zero.
    std::size_t num_states() const override { return 2; }
    void initial_states(std::span<double> x) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;

    std::string_view name() const noexcept override { return config_.name; }
    std::vector<std::string> produces() const override;
    std::vector<std::string> consumes() const override;
    void bind(const CouplingRegistry& registry) override;

    // Loads at lag states x. Main-rotor wash comes from ctx.coupling, or is
    // zero when that is null (D-003); the second form takes it given (tests).
    AirframeAeroLoads evaluate(const SimContext& ctx, std::span<const double> x) const noexcept;
    AirframeAeroLoads evaluate(const SimContext& ctx, std::span<const double> x,
                               const Vec3& fuselage_wash_b, const Vec3& tail_wash_b) const noexcept;

    const AirframeAeroConfig& config() const noexcept { return config_; }

private:
    AirframeAeroConfig config_;
    CouplingSlot alpha_wf_;
    CouplingSlot psi_wf_;
};

}  // namespace eyas
