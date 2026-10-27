#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "rotor/controls.hpp"
#include "rotor/rotation_direction.hpp"
#include "rotor/rotor_wash.hpp"
#include "rotor/shaft_link.hpp"

namespace eyas {
class Config;
}

// Closed-form (Bailey) rotor: the GenHel tail rotor (CR-166309 §5.4.2,
// pp. 5.4-4 to 5.4-8; spec §6.6; D-028). Thrust, no in-plane H force, as in
// GenHel; the torque optionally (below, D-058). One state, the uniform
// induced inflow λ_i.
//
// Frames: hub frame h as the blade-element rotor (D-024): z_h down the shaft,
// opposite the thrust, from hub Euler angles (pitch, roll) relative to body.
// GenHel's tail-rotor shaft axes are this frame with roll = 90° − cant, i.e.
// Γ_TR = 70° for the UH-60A (p. 5.4-6: V_YTR = V_YTRB cos Γ + V_ZTRB sin Γ,
// V_ZTR = −V_YTRB sin Γ + V_ZTRB cos Γ, thrust along −z_TR, p. 5.4-8).
//
// Model, per evaluation:
//   μ, μ_z    hub velocity relative to the air in hub axes / ΩR, including
//             body rates and optional main-rotor wash (p. 5.4-6);
//   C_T       = (aσ/2)(λ t₃₁ + θ t₃₂ + θ_tw t₃₃), λ = μ_z − λ_i, with
//             t₃₁ = B²/2 + μ²/4, t₃₂ = B³/3 + Bμ²/2, t₃₃ = B⁴/4 + B²μ²/4
//             (p. 5.4-7). At B = 1 this is Johnson (2013) Eq. 6.219, whose
//             μ²λ term is the reverse-flow contribution; Johnson's tip loss
//             puts B³μ² and B⁴μ² where GenHel has Bμ² and B²μ²;
//   θ         = θ_cmd + bias − (∂a₀/∂T) T tan δ₃ (p. 5.4-7), with T the
//             blocked thrust of this evaluation (D-028; GenHel uses the
//             previous pass). T is linear in θ, so this is solved in closed
//             form;
//   T         = K_BLK ρπR²(ΩR)² C_T, K_BLK the fin blockage (p. 5.4-8);
//   dλ_i/dt   = (Ω/M₀)(C_T − 2λ_i√(μ² + λ²)), M₀ = 128/75π: the Pitt–Peters
//             thrust row (D-027), replacing GenHel's previous-pass inflow
//             (D-028). Its steady state is GenHel's algebraic inflow.
// Loads: thrust along −z_h and GenHel's drag −½ρ S v_x|v_x| along x_b, at
// the hub, moved to the CG. G = H = 0.
//
// Torque (optional, D-058; GenHel has none, p. 5.4-2): Bailey's (NACA
// Report 716 Eqs. 9 and 11, the torque Ames added to GenHel's tail rotor,
// Ballin NASA TM-88360 App. A), C_Q = C_Qd − C_Qa with
//   (1/a)(2C_Qa/σ) = t₄₁λ² + t₄₂λθ + t₄₃λθ_tw + t₄₄θ² + t₄₅θθ_tw + t₄₆θ_tw²,
//   2C_Qd/σ        = δ₀t₅₁ + δ₁(t₅₂λ + t₅₃θ + t₅₄θ_tw)
//                    + δ₂(t₅₅λ² + t₅₆λθ + t₅₇λθ_tw + t₅₈θ² + t₅₉θθ_tw + t₅,₁₀θ_tw²),
// c_d = δ₀ + δ₁α + δ₂α², the t's to μ² (as t₃₁…t₃₃), with the blades'
// flapping through the Lock number γ = ρacR⁴/I₁ (pp. 210, 223–224, the
// weight-moment terms dropped as Bailey does, p. 212), derived and checked
// against Bailey's appendix by tools/derive_bailey_torque.py; θ after δ₃,
// whose cyclic feathering Bailey's non-feathering rotor doesn't have. The
// torque Q = ρπR²(ΩR)²R C_Q, unblocked, opposes the rotation: on the
// airframe −Q along the rotation axis, on the shaft −gear × Q (D-052),
// published as the demand <name>.shaft_torque = gear × Q [N·m].
//
// Fuselage wake (optional, p. 5.4-5; M6b): GenHel reads the fin's and the
// stabilator's wake maps here (QV1MP, SGV1MP, EPH1MP), so the airframe looks
// them up once and publishes them (D-003): the dynamic-pressure factor K
// scales the body velocity, and the sidewash σ and downwash ε enter as
// −u σ_d K along y_b and −u ε_d K along z_b, u the body x velocity relative
// to the air. σ_d, ε_d lag σ, ε with GenHel's transport delay l/u replaced
// by dσ_d/dt = (|u|/l)(σ − σ_d), l the hub's distance aft of the CG (D-028);
// two more states. The main-rotor wash stays the tail rotor's own: GenHel
// gives it its own maps, equal to the stabilator's "for Black Hawk"
// (p. 5.4-4).
namespace eyas {

// The airframe's wake at the tail rotor (GenHel p. 5.4-5).
struct FuselageWake {
    // Instance producing <airframe>.fin_q_factor, .sidewash and .downwash.
    std::string airframe;
};

// The airframe's wake values and the lag states, for one evaluation.
struct FuselageWakeInput {
    double q_factor = 1.0;    // K [-]
    double sidewash = 0.0;    // σ [rad], undelayed
    double downwash = 0.0;    // ε [rad], undelayed
    double sidewash_d = 0.0;  // σ_d [rad], lagged
    double downwash_d = 0.0;  // ε_d [rad], lagged
};

// Bailey's torque (D-058).
struct BaileyTorque {
    RotationDirection direction = RotationDirection::counter_clockwise;
    double flap_inertia = 0.0;  // I₁ [kg·m²], a blade's about its flapping hinge
    double d0 = 0.0;            // δ₀ [-]
    double d1 = 0.0;            // δ₁ [1/rad]
    double d2 = 0.0;            // δ₂ [1/rad²]
};

// Bailey's t's for the torque (D-058), t₄₁…t₄₆ then t₅₁…t₅₁₀, split as
// t = t0 + (t1 + γ² t2) μ²: the parts the tip loss fixes, computed once.
struct BaileyTorqueTerms {
    std::array<double, 16> t0{};
    std::array<double, 16> t1{};
    std::array<double, 16> t2{};
};

struct BaileyRotorConfig {
    std::string name;                // instance name (D-003); may be empty
    int num_blades = 0;              // b
    double radius = 0.0;             // R [m]
    double rotor_speed = 0.0;        // Ω [rad/s], positive
    double chord = 0.0;              // c [m]
    double lift_slope = 0.0;         // a [1/rad]
    double tip_loss = 1.0;           // B [-] in (0, 1]
    double twist = 0.0;              // θ_tw [rad]: linear, root (r = 0) to tip
    double delta3 = 0.0;             // δ₃ [rad], not negative when coning_per_thrust > 0
    double coning_per_thrust = 0.0;  // ∂a₀/∂T [rad/N]
    double pitch_bias = 0.0;         // [rad] added to the commanded θ (GenHel BIASTR)
    double drag_area = 0.0;          // S [m²] (GenHel C_DTR)
    // Fin blockage (p. 5.4-8): K_BLK = 1 at airspeed V ≥ V_B, else
    // (1 − K)(V/V_B)² + K. No blockage when blockage_speed = 0.
    double blockage_factor = 1.0;        // K [-]
    double blockage_speed = 0.0;         // V_B [m/s]
    Vec3 hub_position_b = Vec3::Zero();  // [m] hub center from the airframe CG, body axes
    double hub_pitch = 0.0;              // [rad]
    double hub_roll = 0.0;               // [rad]
    std::shared_ptr<const PitchSchedule> controls;  // θ_cmd = theta0
    // Instead of `controls`: the instance whose <pitch_input>.theta_tr [rad]
    // is θ_cmd (D-003), e.g. a flight control system (D-045). Empty: `controls`.
    std::string pitch_input;
    std::optional<RotorWash> wash;              // main-rotor wash at the hub
    std::optional<FuselageWake> fuselage_wake;  // airframe wake at the hub (M6b)
    // The shaft it turns on (D-052): speed = gear × the shaft's; rotor_speed
    // must equal gear × the shaft's governed speed. None: constant rotor_speed.
    using ShaftLink = eyas::ShaftLink;  // rotor/shaft_link.hpp
    std::optional<ShaftLink> shaft;
    std::optional<BaileyTorque> torque;  // none: thrust only, as GenHel (D-058)
};

struct BaileyRotorLoads {
    Wrench body;                 // body axes, about the airframe CG
    double thrust = 0.0;         // T [N], blocked, along −z_h
    double ct = 0.0;             // C_T [-], before blockage
    double pitch = 0.0;          // θ [rad], after δ₃
    double blockage = 1.0;       // K_BLK [-]
    double mu = 0.0;             // μ [-]
    double mu_z = 0.0;           // μ_z [-], along z_h
    double lambda_i = 0.0;       // λ_i [-] induced inflow / ΩR, along +z_h (opposite the thrust)
    double lambda_dot = 0.0;     // dλ_i/dt [1/s]
    double cq = 0.0;             // C_Q [-] (D-058; zero without the torque)
    double torque = 0.0;         // Q [N·m], positive opposing the rotation
    Vec3 wash_b = Vec3::Zero();  // [m/s] main-rotor wash velocity used
    // Fuselage wake (zero/one without it).
    double q_factor = 1.0;       // K [-]
    double sidewash = 0.0;       // σ [rad], undelayed
    double downwash = 0.0;       // ε [rad], undelayed
    double sidewash_rate = 0.0;  // dσ_d/dt [rad/s]
    double downwash_rate = 0.0;  // dε_d/dt [rad/s]
};

class BaileyRotor : public DynamicComponent {
public:
    // Throws InitError on an invalid configuration.
    explicit BaileyRotor(BaileyRotorConfig config);

    // Reads a `type: bailey_rotor` config node.
    static std::unique_ptr<BaileyRotor> from_config(const Config& node);

    // States: x[0] = λ_i [-]; with a fuselage wake also x[1] = σ_d and
    // x[2] = ε_d [rad]; on a shaft, last, its own azimuth ψ [rad], ψ̇ = gear
    // × the shaft's Ω (D-053 amendment 3: as gear × the shaft's wrapped
    // azimuth it would jump at each wrap unless the gear were whole). All
    // start at zero.
    std::size_t num_states() const override {
        return std::size_t{config_.fuselage_wake ? 3U : 1U} + (config_.shaft ? 1U : 0U);
    }
    void initial_states(std::span<double> x) const override;
    // Trim (D-053 amendment 3): its azimuth starts at zero, a phase.
    void steady_states(const SimContext& ctx, std::span<double> x) const override;
    void steady_kinds(std::span<SteadyKind> kinds) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;
    // D-059: reports its Readouts fields in derivatives().
    ReadoutRole readout_role() const noexcept override { return ReadoutRole::tail_rotor; }

    std::string_view name() const noexcept override { return config_.name; }
    // <name>.shaft_torque [N·m] with the torque on a shaft (D-058).
    std::vector<std::string> produces() const override;
    std::vector<std::string> consumes() const override;
    void bind(const CouplingRegistry& registry) override;
    void bind_shafts(std::span<const ShaftInfo> shafts) override;

    // Rotor speed Ω [rad/s] in this context: the shaft's × gear (D-052), or
    // the constant rotor_speed.
    double rotor_speed(const SimContext& ctx) const noexcept;
    // Its azimuth [rad, in [0, 2π)] at states x: its own state on a shaft
    // (D-053 amendment 3), else Ω t.
    double azimuth(const SimContext& ctx, std::span<const double> x) const noexcept;

    // Loads at states x. The main-rotor wash and the airframe's wake values
    // come from ctx.coupling; without it the wash is zero and the wake
    // neutral (K = 1, σ = ε = 0; the component evaluated on its own, D-003).
    // The second form takes them given (tests).
    BaileyRotorLoads evaluate(const SimContext& ctx, std::span<const double> x) const noexcept;
    // `wake` null: no fuselage wake (K = 1).
    BaileyRotorLoads evaluate(const SimContext& ctx, double lambda_i, const Vec3& wash_b,
                              const FuselageWakeInput* wake = nullptr) const noexcept;

    const BaileyRotorConfig& config() const noexcept { return config_; }
    double solidity() const noexcept { return solidity_; }     // σ = bc/(πR)
    double disk_area() const noexcept { return area_; }        // πR² [m²]
    double tip_speed() const noexcept { return tip_speed_; }   // nominal ΩR [m/s]
    int shaft_index() const noexcept { return shaft_index_; }  // D-052; −1: none

private:
    BaileyRotorConfig config_;
    double solidity_;
    double area_;
    double tip_speed_;
    double lift_factor_;   // G = aσ/2
    double coning_pitch_;  // (∂a₀/∂T) tan δ₃ [rad/N]
    Mat3 C_hb_;
    Mat3 C_bh_;
    CouplingSlot pitch_;
    CouplingSlot q_factor_;
    CouplingSlot sidewash_;
    CouplingSlot downwash_;
    CouplingSlot shaft_torque_;  // D-058
    Vec3 spin_axis_b_;           // unit, along the rotation (D-058)
    double lock_per_density_;    // γ/ρ = acR⁴/I₁ [m³/kg] (D-058; zero without the torque)
    BaileyTorqueTerms torque_terms_;  // its t's parts fixed by the tip loss (D-058)
    int shaft_index_ = -1;            // D-052
    double gear_ = 1.0;               // the shaft link's gear, set with shaft_index_
};

}  // namespace eyas
