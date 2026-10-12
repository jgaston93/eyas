#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "rotor/controls.hpp"
#include "rotor/rotor_wash.hpp"

namespace eyas {
class Config;
}

// Closed-form (Bailey) rotor: the GenHel tail rotor (CR-166309 §5.4.2,
// pp. 5.4-4 to 5.4-8; spec §6.6; D-028). Thrust only: no torque and no
// in-plane H force, as in GenHel. One state, the uniform induced inflow λ_i.
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
namespace eyas {

struct BaileyRotorConfig {
    std::string name;                // instance name (D-003); may be empty
    int num_blades = 0;              // b
    double radius = 0.0;             // R [m]
    double rotor_speed = 0.0;        // Ω [rad/s], positive
    double chord = 0.0;              // c [m]
    double lift_slope = 0.0;         // a [1/rad]
    double tip_loss = 1.0;           // B [-] in (0, 1]
    double twist = 0.0;              // θ_tw [rad]: linear, root (r = 0) to tip
    double delta3 = 0.0;             // δ₃ [rad]
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
    std::optional<RotorWash> wash;                  // main-rotor wash at the hub
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
    Vec3 wash_b = Vec3::Zero();  // [m/s] main-rotor wash velocity used
};

class BaileyRotor : public DynamicComponent {
public:
    // Throws InitError on an invalid configuration.
    explicit BaileyRotor(BaileyRotorConfig config);

    // Reads a `type: bailey_rotor` config node.
    static std::unique_ptr<BaileyRotor> from_config(const Config& node);

    // State: x[0] = λ_i [-], starting at zero.
    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;

    std::string_view name() const noexcept override { return config_.name; }
    std::vector<std::string> consumes() const override;
    void bind(const CouplingRegistry& registry) override;

    // Loads at inflow state λ_i. With main-rotor wash configured, ctx.coupling
    // must hold the rotor's outputs; `wash` overrides them (tests).
    BaileyRotorLoads evaluate(const SimContext& ctx, double lambda_i) const noexcept;
    BaileyRotorLoads evaluate(const SimContext& ctx, double lambda_i,
                              const Vec3& wash_b) const noexcept;

    const BaileyRotorConfig& config() const noexcept { return config_; }
    double solidity() const noexcept { return solidity_; }  // σ = bc/(πR)
    double disk_area() const noexcept { return area_; }     // πR² [m²]
    double tip_speed() const noexcept { return tip_speed_; }

private:
    BaileyRotorConfig config_;
    double solidity_;
    double area_;
    double tip_speed_;
    Mat3 C_hb_;
    Mat3 C_bh_;
};

}  // namespace eyas
