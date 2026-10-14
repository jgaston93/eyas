#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "util/table.hpp"
#include "util/units.hpp"

namespace eyas {
class Config;
}

// Landing gear, GenHel style (CR-166309 §5.7; spec §6.7; D-028, D-032): per
// gear a tire in series with an oleo strut carrying an unsprung mass
// (Figure 7.1.1, p. 5.7-5). The strut slides along body z.
//
// Geometry (pp. 5.7-9 to 5.7-15). The axle sits at r_ax = r_a − s·e_z (body,
// s the stroke, positive compressed). The gear line (body z) meets the ground
// plane at distance L = h/cos γ from the axle, h the axle's height above the
// plane and cos γ = −u·(C_nb e_z), u the plane's upward normal. The tire's
// radial deflection is δ_T = R − L (p. 5.7-14); the tire is in contact when
// δ_T > 0.
//
// Tire (pp. 5.7-16 to 5.7-20). The deflection in body axes is
//   Δ_b = C_bg (δx, δy, 0) + (0, 0, δ_T),
// with (δx, δy) the in-plane deflection state in ground axes g (the ground
// plane, x along the body's heading; GenHel's space axes with ψ = 0). Forces
// on the wheel, body axes:
//   X = −K_X (ΔX + G ΔẊ),  Y = −K_Y (ΔY + G ΔẎ),
//   Z = −(F_z(ΔZ) + G K′(F_z) ż_f)   (no adhesion: Z ≤ 0),
// with K_X = 0.53·D·(P + 4 P_R)·(ΔZ/D)^(1/3) (p. 5.7-42), K_Y from a table or
// K_ref √(F/F_ref) (p. 5.7-46), F_z the vertical load table, K′ the damping
// slope table and ż_f the vertical deflection rate through GenHel's lag
// (D-032 item 3). All are multiplied by the number of wheels.
//
// Friction (pp. 5.7-18 to 5.7-22; D-032 items 1, 2). In ground axes the limits
// are L_x = k_B μ N and L_y = μ N, N the trial normal load, k_B from the
// wheel's brake input (Controls::brake_left, brake_right), each times
// 1 − (1 − K_SLIP)·smoothstep(|v_c|/v_s) of its own axis's contact-point
// speed. On an axis where the trial (sticking) force exceeds its limit the
// force is held at ±L and the deflection relaxes towards it through the
// tire's damper:
//   stick: δ' = v_c;  slip: δ' = (q − δ)/G, q the deflection that gives ±L.
// Out of contact the unloaded tire relaxes, δ' = −δ/G.
//
// Strut (pp. 5.7-23, 5.7-24; D-032 items 5, 6, 7). Force on the wheel, down:
//   F_S = sgn(ṡ)(C ṡ² + F_0) + C_PV / max(C_PV/F_PRE − s, l_min)
//         + [s < 0] (k_stop s + c_stop ṡ),
// and the unsprung mass m obeys, along body z,
//   s̈ = [v̇ + ω̇ × r_ax]_z + [ω × v + ω × (ω × r_ax) + 2 ω × ṙ_ax]_z
//       − (Z + F_S)/m − g_z.
//
// Loads on the airframe about its CG (D-002: the wheels' weight is included):
// the in-plane tire forces at the contact point r_c = r_ax + (−ΔX, −ΔY, R − ΔZ),
// the tire's vertical force about the axle (moment only), −F_S along body z at
// the axle, and the wheel's weight minus its inertial force in x and y.
//
// States per gear (D-032): s [m], ṡ [m/s], δx, δy [m], ż_f [m/s].
// Coupling: none.
namespace eyas {

struct TireConfig {
    double radius = 0.0;          // R [m]; the diameter D = 2R
    int wheels = 1;               // N_T
    double pressure = 0.0;        // P [Pa]
    double rated_pressure = 0.0;  // P_R [Pa]
    // Vertical load [N] against the radial deflection [m], at `pressure`,
    // extended beyond its last point with the last segment's slope.
    Table1D vertical_load{Axis({0.0, 1.0}), {0.0, 0.0}};
    // K′ [N/m] against the vertical load [N], at `pressure` (damping only).
    Table1D damping_stiffness{Axis({0.0, 1.0}), {0.0, 0.0}};
    // Lateral stiffness: a table [N/m] against the deflection [m] at
    // `pressure`, or K_ref √(F/F_ref) with F the vertical load.
    std::optional<Table1D> lateral_stiffness;
    double lateral_reference_load = 0.0;       // F_ref [N]
    double lateral_reference_stiffness = 0.0;  // K_ref [N/m]
    double fore_aft_coefficient = 0.0;         // 0.53 in GenHel [-]
    double damping_factor = 0.0;               // G [s]
    double damping_lag = 0.0;                  // τ [s], vertical only
};

struct StrutDamperRegime {
    double coefficient = 0.0;  // C [N·s²/m²]
    double offset = 0.0;       // F_0 [N]
};

struct StrutConfig {
    double preload = 0.0;                         // F_PRE [N]
    double air_spring = 0.0;                      // C_PV [N·m], isothermal pV
    double min_gas_length = 0.01 * units::kFoot;  // l_min [m], GenHel p. 5.7-23
    double break_speed = 0.0;                     // [m/s]; |ṡ| ≤ it: `low`
    StrutDamperRegime low, high;
    // Top stop (D-032 item 6). Unset: k_stop = 10 × the tire's slope at zero
    // deflection, c_stop = 2√(k_stop m).
    std::optional<double> stop_stiffness;  // [N/m]
    std::optional<double> stop_damping;    // [N·s/m]
};

// Which brake input acts on a wheel (Controls::brake_left, brake_right). None:
// k_B is the brakes-off factor always (GenHel's tail wheel, p. 5.7-18).
enum class BrakeSide : std::uint8_t { none, left, right };

struct GearConfig {
    std::string name;
    Vec3 axle_b = Vec3::Zero();  // [m] axle at full extension, from the CG, body axes
    double unsprung_mass = 0.0;  // m [kg]
    BrakeSide brake = BrakeSide::none;
    TireConfig tire;
    StrutConfig strut;
};

struct LandingGearConfig {
    std::string name;
    std::vector<GearConfig> gears;
    double friction_coefficient = 0.0;  // μ
    // k_B = off + (set − off)·b for a brake input b in [0, 1] (D-032): GenHel's
    // two values at the ends, linear between (no data).
    double brake_factor_set = 0.0;                    // k_B at b = 1 (GenHel B_RK)
    double brake_factor_off = 0.0;                    // k_B at b = 0 (GenHel N_BRK)
    double sliding_factor = 0.0;                      // K_SLIP
    double sliding_blend_speed = 0.1 * units::kFoot;  // v_s [m/s] (D-032 item 2)
    // Upward normal of the ground plane, NED (D-032 item 9). The plane passes
    // through the point on the NED origin's vertical at Environment::ground_height
    // above MSL: z_n = origin altitude − ground_height.
    Vec3 ground_normal_n = Vec3(0.0, 0.0, -1.0);
};

// One gear's results for one evaluation (tests, recorder).
struct GearState {
    bool in_contact = false;
    double tire_deflection = 0.0;              // δ_T [m]
    double vertical_deflection = 0.0;          // ΔZ [m]
    double normal_load = 0.0;                  // N [N]
    Vec3 tire_force_b = Vec3::Zero();          // on the wheel [N]
    Vec3 tire_force_g = Vec3::Zero();          // the same in ground axes [N]
    std::array<bool, 2> slipping{};            // x, y in ground axes
    std::array<double, 2> limit{};             // L_x, L_y [N]
    std::array<double, 2> contact_velocity{};  // v_c, ground axes [m/s]
    std::array<double, 2> stick_rate{};        // δ' while sticking, ground axes [m/s]
    double strut_force = 0.0;                  // F_S [N], on the wheel, down
    Vec3 contact_point_b = Vec3::Zero();       // r_c [m]
    Vec3 axle_b = Vec3::Zero();                // r_ax [m]
};

class LandingGear : public DynamicComponent {
public:
    static constexpr std::size_t kStatesPerGear = 5;

    // Throws InitError on an invalid configuration.
    explicit LandingGear(LandingGearConfig config);

    // Reads a `type: landing_gear` config node.
    static std::unique_ptr<LandingGear> from_config(const Config& node);

    std::size_t num_states() const override { return kStatesPerGear * config_.gears.size(); }
    // Every gear at full extension, undeflected, at rest.
    void initial_states(std::span<double> x) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;
    std::string_view name() const noexcept override { return config_.name; }

    // Gear i's results at states x (all gears' states).
    GearState gear_state(const SimContext& ctx, std::span<const double> x,
                         std::size_t i) const noexcept;

    const LandingGearConfig& config() const noexcept { return config_; }
    // The top stop actually used (configured or default) [N/m], [N·s/m].
    double stop_stiffness(std::size_t i) const noexcept { return stop_k_[i]; }
    double stop_damping(std::size_t i) const noexcept { return stop_c_[i]; }

private:
    struct GearOut;
    struct Frame;
    Frame frame(const SimContext& ctx) const noexcept;
    void evaluate(const SimContext& ctx, const Frame& f, std::span<const double> x, std::size_t i,
                  GearOut& out) const noexcept;

    LandingGearConfig config_;
    std::vector<double> stop_k_, stop_c_;
};

// F_z(Δ) of a tire table, extended linearly beyond its last point (D-032).
double tire_vertical_load(const Table1D& table, double deflection) noexcept;

}  // namespace eyas
