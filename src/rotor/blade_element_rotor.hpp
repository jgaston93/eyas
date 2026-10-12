#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "core/component.hpp"
#include "rotor/airfoil.hpp"
#include "rotor/controls.hpp"
#include "util/table.hpp"

namespace eyas {
class Config;
}

// Blade-element rotor with rigid blades (spec §6.4, §6.5; D-024, D-026,
// D-027). The induced inflow is uniform momentum inflow, solved in every
// evaluation (D-024), or Pitt–Peters dynamic inflow with three states
// (D-027), chosen by config. Two blade forms, also chosen by config:
// - no flap hinge (M3): massless blades with flapping held at zero; no
//   states, aerodynamic loads only (G = H = 0);
// - flap hinge (M4, D-026): rigid blades with mass, hinged at an offset e,
//   with states [β_k, β̇_k] per blade. The flap equation and the hub loads
//   are exact rigid-body mechanics, affine in the body acceleration
//   a = [v̇_b; ω̇_b], and returned as values at a = 0 plus G and H.
//
// Frames (D-024): hub frame h has its origin at the hub center, z_h down
// along the shaft, x_h forward and y_h right; with zero hub angles it is
// parallel to body axes. Blade k points along e_r = (−cos ψ_k, s sin ψ_k, 0)
// in hub axes, where s = +1 for a rotor turning counter-clockwise viewed from
// above and −1 for clockwise, so ψ_k = 0 points aft and ψ_k increases with
// rotation (spec §4).
namespace eyas {

enum class RotationDirection : std::uint8_t {
    counter_clockwise,  // viewed from above (UH-60, spec §4)
    clockwise,
};

enum class InflowModel : std::uint8_t {
    uniform,      // momentum, solved each evaluation, no states (D-024)
    pitt_peters,  // dynamic, states (λ0, λ1s, λ1c) (D-027)
};

// Linear induced inflow λ(r, ψ) = l0 + (r/R)(l1s sin ψ + l1c cos ψ) [-],
// positive down along the shaft, ψ the blade azimuth, r the radius in the
// hub plane (Pitt & Peters 1980 eq. 1; D-027).
struct LinearInflow {
    double l0 = 0.0;
    double l1s = 0.0;
    double l1c = 0.0;
};

enum class ElementSpacing : std::uint8_t {
    uniform,     // equal widths
    equal_area,  // annuli of equal area (spec §6.4)
};

// Rigid flapping blade on an offset hinge (D-026). The blade is a line of
// mass along its span, so its inertia is fully described by the mass and its
// first and second moments about the hinge (Johnson 2013 §6.15: ν² = 1 +
// e S_β/I_β + K_β/(I_β Ω²), Eq. 6.261 in dimensional form). The hinge axis is
// perpendicular to the span, in the hub plane; β is positive up (toward −z_h).
struct FlapHinge {
    double offset = 0.0;        // e [m] hinge distance from the hub center
    double mass = 0.0;          // m_b [kg] outboard of the hinge
    double first_moment = 0.0;  // S_β [kg·m] about the hinge
    double inertia = 0.0;       // I_β [kg·m²] about the hinge
    double spring = 0.0;        // K_β [N·m/rad] about β = 0
    double damper = 0.0;        // C_β [N·m·s/rad]
    double delta3 = 0.0;  // δ₃ [rad]: pitch change −tan δ₃ · β (Johnson 2013 §6.18)
};

struct BladeElementRotorConfig {
    int num_blades = 0;        // N_b
    double radius = 0.0;       // R [m]
    double rotor_speed = 0.0;  // Ω [rad/s], positive
    RotationDirection direction = RotationDirection::counter_clockwise;
    Vec3 hub_position_b = Vec3::Zero();  // [m] hub center from the airframe CG, body axes
    double hub_pitch = 0.0;              // [rad] hub-frame Euler angles relative to body
    double hub_roll = 0.0;               //       (a forward shaft tilt is a negative pitch)
    double root_cutout = 0.0;            // [m] from the hub center; no loads inboard
    double tip_loss = 1.0;               // B [-] in (0, 1]; no lift outboard of B·R
    // Against r/R [-]. Chord [m]; twist θ_tw(r) [rad], added to θ0, which is
    // the pitch at the hub center (Leishman 2nd ed. Eq. 3.25 convention).
    Table1D chord{Axis({0.0, 1.0}), {0.0, 0.0}};
    Table1D twist{Axis({0.0, 1.0}), {0.0, 0.0}};
    int num_elements = 20;  // N, spanwise, total (spec §6.4)
    ElementSpacing spacing = ElementSpacing::equal_area;
    std::shared_ptr<const Airfoil> airfoil;
    std::shared_ptr<const PitchSchedule> controls;
    std::optional<FlapHinge> flap;  // none: M3 rotor, flapping held at zero
    InflowModel inflow = InflowModel::uniform;
};

inline constexpr int kMaxBlades = 64;

// Rotor loads from one evaluation.
struct RotorLoads {
    // Aerodynamic loads only. The blade inertia and weight of a flapping
    // rotor are added by derivatives().
    Vec3 F_h = Vec3::Zero();  // [N]   aerodynamic force on the rotor, hub axes
    Vec3 M_h = Vec3::Zero();  // [N·m] about the hub center, hub axes
    Wrench body;              // the same, body axes, about the airframe CG
    double thrust = 0.0;      // T [N] = −F_h,z (along the shaft, upward)
    double torque = 0.0;      // Q [N·m] shaft torque needed to hold Ω (positive)
    double power = 0.0;       // P [W] = Q Ω
    double ct = 0.0;          // C_T = T / (ρ π R² (ΩR)²)
    double cq = 0.0;          // C_Q = C_P = Q / (ρ π R² (ΩR)² R)
    double lambda_i = 0.0;    // λ_i [-] uniform induced inflow (λ0) / ΩR, positive down
    double lambda_1s = 0.0;   // first-harmonic inflow [-] (D-027; 0 for uniform inflow)
    double lambda_1c = 0.0;
    // Azimuth-weighted moments of the aerodynamic force normal to the hub
    // plane, Σ dF_up (r/R)(sin ψ, cos ψ) / (ρA(ΩR)²) (D-027).
    double c1s = 0.0;
    double c1c = 0.0;
    double mu = 0.0;        // μ [-] in-plane hub speed through the air / ΩR
    double mu_z = 0.0;      // μ_z [-] hub speed along z_h (down) / ΩR
    int passes = 0;         // blade-element passes used by the inflow solve
    bool converged = true;  // inflow solve met its tolerance (D-023)
    // Aerodynamic moment about each blade's flap hinge [N·m], positive flap
    // up; zero without a flap hinge.
    std::array<double, kMaxBlades> flap_moment{};
};

// One spanwise element, midpoint rule.
struct BladeElement {
    double r = 0.0;      // [m] span position of the element center from the hub center
    double dr = 0.0;     // [m] width
    double chord = 0.0;  // [m]
    double twist = 0.0;  // [rad] θ_tw(r)
    double lift = 1.0;   // 1 inboard of B·R, 0 outboard (tip loss)
};

class BladeElementRotor : public DynamicComponent {
public:
    // Throws InitError on an invalid configuration.
    explicit BladeElementRotor(BladeElementRotorConfig config);

    // Reads a `type: blade_element_rotor` config node (see data docs in
    // DECISIONS D-024). Table files are resolved by Config::path.
    static std::unique_ptr<BladeElementRotor> from_config(const Config& node);

    // States: with a flap hinge, x[2k] = β_k [rad], x[2k+1] = β̇_k [rad/s]
    // for blade k = 0…N_b−1, numbered as in blade_azimuth() (D-026); then,
    // with Pitt–Peters inflow, (λ0, λ1s, λ1c) [-] from inflow_offset()
    // (D-027). All start at zero.
    std::size_t num_states() const override;
    void initial_states(std::span<double> x) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;

    // Aerodynamic loads at rotor states x (empty: every state zero), with the
    // configured inflow: uniform inflow solved for this evaluation (D-024),
    // or the Pitt–Peters states in x (D-027).
    RotorLoads evaluate(const SimContext& ctx, std::span<const double> x = {}) const noexcept;
    // Loads with the uniform momentum inflow solved (D-024), whatever the
    // configured model: the steady Pitt–Peters inflow in hover, and a start
    // for its states. Flap states are read from x as in evaluate().
    RotorLoads evaluate_uniform(const SimContext& ctx,
                                std::span<const double> x = {}) const noexcept;
    // Loads at a given induced inflow (one blade-element pass, no inflow
    // solve). converged and passes describe that single pass.
    RotorLoads evaluate_at_inflow(const SimContext& ctx, double lambda_i,
                                  std::span<const double> x = {}) const noexcept;
    RotorLoads evaluate_at_inflow(const SimContext& ctx, const LinearInflow& lambda,
                                  std::span<const double> x = {}) const noexcept;

    // Index of λ0 in the states (Pitt–Peters only): 2 N_b with a flap hinge, else 0.
    std::size_t inflow_offset() const noexcept;

    // Rotor azimuth ψ [rad] in [0, 2π) at time t [s]: ψ = Ω t (D-024).
    double azimuth(double t) const noexcept;
    // Azimuth of blade k = 0…N_b−1: ψ − 2π k / N_b (Padfield 3rd ed. Eq. 3.49).
    double blade_azimuth(double psi, int k) const noexcept;

    const BladeElementRotorConfig& config() const noexcept { return config_; }
    std::span<const BladeElement> elements() const noexcept { return elements_; }
    double disk_area() const noexcept { return area_; }       // π R² [m²]
    double tip_speed() const noexcept { return tip_speed_; }  // ΩR [m/s]
    const Mat3& C_hb() const noexcept { return C_hb_; }

    // Iteration limit and tolerance of the inflow solve.
    static constexpr int kMaxPasses = 30;
    static constexpr double kInflowTolerance = 1e-13;  // on λ_i [-]

private:
    // Hinge offset e [m]; 0 without a flap hinge.
    double hinge_offset() const noexcept { return config_.flap ? config_.flap->offset : 0.0; }

    // Per-evaluation inputs to a blade-element pass, in hub axes.
    struct Kinematics {
        Vec3 v_hub_h;  // [m/s] hub velocity relative to the air
        Vec3 omega_h;  // [rad/s] body angular rate
        double psi;    // [rad] rotor azimuth
        SwashplatePitch pitch;
        double rho;    // [kg/m³]
        double a;      // [m/s] speed of sound
        double mu;     // μ [-] in-plane hub speed through the air / ΩR
        double mu_z;   // μ_z [-] hub speed along z_h (down) / ΩR
        double cos_w;  // downwind azimuth ψ_w (D-027): cos ψ_w = v_x/|v_xy|,
        double sin_w;  //   sin ψ_w = −s v_y/|v_xy|; (1, 0) at |v_xy| = 0
    };

    Kinematics kinematics(const SimContext& ctx) const noexcept;
    RotorLoads pass(const Kinematics& k, const LinearInflow& lambda,
                    std::span<const double> x) const noexcept;
    RotorLoads solve_uniform(const Kinematics& k, std::span<const double> x) const noexcept;
    void finish(RotorLoads& loads, const Kinematics& k) const noexcept;

    BladeElementRotorConfig config_;
    std::vector<BladeElement> elements_;
    double s_;          // +1 counter-clockwise, −1 clockwise
    double area_;       // [m²]
    double tip_speed_;  // [m/s]
    Mat3 C_hb_;         // body → hub
    Mat3 C_bh_;
    // Hunt-search hints, one per (blade, element). They change only the
    // search time, never the result, so derivatives() stays pure (spec §6).
    mutable std::vector<TableCursor<2>> cursors_;
};

}  // namespace eyas
