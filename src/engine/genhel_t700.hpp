#pragma once

#include <array>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "util/table.hpp"

namespace eyas {
class Config;
}

// GenHel's T700 engines, governor and clutch (CR-166309 §5.6, pp. 5.6-8 to
// 5.6-18; D-053), re-anchored on GenHel's steady-state maps: N_E identical
// engines driving one shaft through one clutch, as GenHel models them.
//
// GenHel writes every engine quantity as its trim value plus partial
// derivatives times departures from trim. Here each is the steady map's value
// at the gas-generator speed plus GenHel's partial times the departure from
// that steady line (D-053 item 1), so nothing is remembered from a trim:
//   P3   = P3_ss(N) + ∂P3/∂W_f (W_f − W_ss(N))                      (p. 5.6-15)
//   Q_GG = ∂Q_GG/∂W_f (W_f − W_ss(N)),  dN/dt = K_HS (K_CF/J_GG) Q_GG (pp. 5.6-15, 16)
//   Q_PT = Q_PT,ss(N) + ∂Q_PT/∂W_f (W_f − W_ss(N)) + ∂Q_PT/∂N_PT (N_PT,ref − N_PT)
//                                                                     (p. 5.6-16)
// GenHel's ΔN_PT is the speed error N_PTDP in rpm, reference less actual
// (p. 5.6-16), so with its positive partials the torque rises as the turbine
// slows, as a turbine's does.
// with W_ss, P3_ss from Tables 6.5.4 and 6.5.5, Q_PT,ss the inverse of Table
// 6.5.1 over G_E, and the partials from Table 6.5.6, all at N = N_GG.
//
// Fuel control (hydromechanical unit, p. 5.6-14 and 5.6-15; D-053 item 2 and
// its amendment): W_f lags W_cmd = R · P3′ by T_MV, with P3′ = P3 lagged by
// T_P3 and the ratio R = W_ss/P3_ss at N_GG,ref + (∂(W_f/P3)/∂N_GG) e, where
// e = N_GG,ref − N_GG,meas and N_GG,meas lags N_GG by T_NGG. Isochronous on
// the steady maps. W_cmd ≥ 0.
//
// Demanded gas-generator speed: N_GG,ref = N_GG,LDS(X_C) + ECU, the load-demand
// spindle's schedule (Tables 6.5.2, 6.5.3) on the collective stick plus the
// electronic control's output, which can only trim it down (A_U = 0).
//
// Electronic control unit (Fig. 6.1.3, pp. 5.6-11 to 5.6-13), in volts, with
// the power-turbine speed error N_PTD = (N_PT,ref − N_PT)/N_PT100 and N_PT,ref
// the shaft's governed speed times G_E (p. 5.6-10):
//   I_DB1 = K_1 N_PTD / (K_10 s² + K_11 s + 1);  V_HG = deadband(I_DB1; a, b)
//   V_LG  = K_3 N_PTD
//   V_G   = V_HG if A ≥ 20 or B ≥ 20 (switch open, high gain), else V_LG
//   A     = 40 (Q_INT's limit) outside −4 % ≤ N_PTD ≤ 1 %, else 0
//   B     = Q_INT, dQ_INT/dt = 0.7 (K_4 Q_PT − 20), limited to [0, 40] (D-053)
//   V_S   = (1 + t_1 s)/(1 + t_2 s) V_G;  dV_SS/dt = K_9 V_S, limited to [A_L, A_U]
//   V_DMD = (1 + t_3 s)/(1 + t_4 s) (V_SS + K_6 V_S)
//   M     = G_4 (1 + T_3 s)/(1 + T_2 s) (V_DMD − G_6 INS);  dINS/dt = G_5 M
//   ECU   = G_7 INS
//
// Clutch (Fig. 6.1.4, pp. 5.6-17, 18; D-053 item 3): a slip state s ≥ 0, the
// drive's speed less the engine side's (both referred to the shaft). At zero
// slip it is engaged iff Q_E/J_TOTE + FLQREQ/J_d > 0: the engine alone would
// accelerate at least as fast as the drive alone, FLQREQ the 0.3 s lag of the
// main rotor's demand times (1 + K_Q). Engaged, the engine side is part of the
// shaft: its torque Q_E = N_E G_E Q_PT and inertia J_TOTE act on it. Free, it
// runs on its own: J_TOTE (dΩ_e/dt + n·ω̇/G_E) = Q_E, ds/dt = Ω̇ − dΩ_e/dt.
//
// Losses (D-053 item 4): K_Q times the main rotor's demand on the shaft, both
// modes.
//
// Inertial terms (D-053 item 5): the engine side spins at G_E Ω_e about n_pt
// with angular momentum (J_TOTE/G_E) Ω_e n_pt; the gas-generator spools at
// N_GG × their 100 % speed about n_gg with inertia J_GG, whose spool speed
// relative to the airframe changes as J_GG (ω̇_gg + n_gg·ω̇) = torque. Each
// gives the airframe −(its momentum's rate relative to the airframe) and the
// gyroscopic −ω × momentum; the engine's own torques are internal (D-052).
//
// Units: SI, speeds in N_GG and N_PT as fractions of their 100 % speeds, ECU
// signals in volts, INS in metres. Inputs: the collective stick
// (Controls::collective, X_C = 100 × it), the shaft's speed, and the main
// rotor's demand `<rotor>.shaft_torque` and its body-acceleration terms
// `<rotor>.shaft_torque_a0` … `_a5` (D-003, D-053 amendment 2).
namespace eyas {

struct GenHelT700Config {
    std::string name;   // instance name (D-003)
    std::string shaft;  // the shaft it drives; GenHel's inertias and G_E are referred to it
    std::string rotor;  // the instance publishing <rotor>.shaft_torque [N·m]
    int engines = 2;    // N_E
    double gear = 0.0;  // G_E: power-turbine speed over shaft speed [-]
    double power_turbine_100 = 0.0;  // N_PT100 [rad/s]
    double gas_generator_100 = 0.0;  // N_GG100 [rad/s]
    double loss_factor = 0.0;        // K_Q [-]

    // Maps (Tables 6.5.1 to 6.5.6), axes N_GG as a fraction unless noted.
    std::shared_ptr<const Table1D> torque_ngg;   // per-engine demand at the shaft [N·m] → N_GG
    std::shared_ptr<const Table1D> lds_rigging;  // collective [fraction] → LDS [rad]
    std::shared_ptr<const Table1D> lds_cam;      // LDS [rad] → N_GG,LDS
    std::shared_ptr<const Table1D> fuel;         // W_ss [kg/s], per engine
    std::shared_ptr<const Table1D> p3;           // P3_ss [Pa]
    std::shared_ptr<const Table1D> dp3_dwf;      // [Pa·s/kg]
    std::shared_ptr<const Table1D> dqgg_dwf;     // [N·m·s/kg]
    std::shared_ptr<const Table1D> dqpt_dwf;     // [N·m·s/kg], per engine
    std::shared_ptr<const Table1D> dqpt_dnpt;    // [N·m·s/rad], per engine

    // ECU (Fig. 6.1.3, p. 5.6-24).
    double k1 = 0.0;                                // [V] per unit N_PTD
    double k3 = 0.0;                                // [V] per unit N_PTD
    double k4 = 0.0;                                // [V/(N·m)]
    double k6 = 0.0;                                // [-]
    double k9 = 0.0;                                // [1/s]
    double k10 = 0.0;                               // [s²]
    double k11 = 0.0;                               // [s]
    double dead_band_a = 0.0;                       // [V]
    double dead_band_b = 0.0;                       // [V]
    double t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0;  // [s]
    double upper = 0.0;                             // A_U [V]
    double lower = 0.0;                             // A_L [V]
    double g4 = 0.0;                                // [A/V]
    double g5 = 0.0;                                // [m/A]
    double g6 = 0.0;                                // [V/m]
    double g7 = 0.0;                                // [1/m] fraction of N_GG per metre of INS
    double T2 = 0.0, T3 = 0.0;                      // [s]
    // The high/low-gain switch (p. 5.6-12): speed band, Q_INT's gain,
    // threshold and limit, as printed.
    double band_low = -0.04;   // [fraction]
    double band_high = 0.01;   // [fraction]
    double qint_gain = 0.7;    // [1/s]
    double threshold = 20.0;   // [V]
    double qint_limit = 40.0;  // [V]

    // Fuel control and gas generator (p. 5.6-24).
    double speed_lag = 0.0;     // T_NGG [s]
    double ratio_gain = 0.0;    // ∂(W_f/P3)/∂N_GG [kg/(s·Pa)] per unit N_GG
    double p3_lag = 0.0;        // T_P3 [s]
    double valve_lag = 0.0;     // T_MV [s]
    double acceleration = 0.0;  // K_CF/J_GG [1/(N·m·s)], N_GG a fraction
    double heat_sink = 0.0;     // K_HS [-]

    // Clutch and inertias (pp. 5.6-17, 18, 24; D-053).
    double demand_lag = 0.3;               // FLQREQ's lag [s]
    double drive_inertia = 0.0;            // J_d [kg·m²], the drive's referred to the shaft
    double engine_inertia = 0.0;           // J_TOTE [kg·m²], referred to the shaft
    Vec3 pt_axis_b = Vec3(1.0, 0.0, 0.0);  // power turbines' spin axis (unit)
    double gg_inertia = 0.0;               // J_GG [kg·m²], the spools' own, all engines
    Vec3 gg_axis_b = Vec3(1.0, 0.0, 0.0);  // gas generators' spin axis (unit)
};

// The engine's signals at one state (outputs and tests).
struct GenHelT700Signals {
    double n_gg = 0.0;          // N_GG [fraction]
    double n_gg_ref = 0.0;      // N_GG,ref [fraction]
    double n_gg_lds = 0.0;      // N_GG,LDS [fraction]
    double ecu = 0.0;           // [fraction of N_GG]
    double n_ptd = 0.0;         // N_PTD [fraction]
    double fuel = 0.0;          // W_f per engine [kg/s]
    double fuel_command = 0.0;  // W_cmd [kg/s]
    double p3 = 0.0;            // [Pa]
    double q_gg = 0.0;          // gas-generator accelerating torque [N·m]
    double q_pt = 0.0;          // power-turbine torque per engine [N·m]
    double q_engine = 0.0;      // Q_E, all engines referred to the shaft [N·m]
    double v_g = 0.0;           // [V]
    double v_s = 0.0;           // [V]
    bool high_gain = false;     // the switch open
    bool engaged = true;        // the clutch
    double engine_speed = 0.0;  // Ω_e [rad/s], the engine side referred to the shaft
    double rotor_demand = 0.0;  // the main rotor's demand [N·m]
};

class GenHelT700 : public DynamicComponent {
public:
    // State indices.
    static constexpr std::size_t kNgg = 0;   // N_GG [fraction]
    static constexpr std::size_t kFuel = 1;  // W_f per engine [kg/s]
    static constexpr std::size_t kP3Lag =
        2;  // P3′/p₀ [-], p₀ = 101325 Pa (a pressure ratio, so trim's tolerance fits)
    static constexpr std::size_t kNggMeas = 3;  // N_GG,meas [fraction]
    static constexpr std::size_t kIdb = 4;      // I_DB1 [V]
    static constexpr std::size_t kIdbRate = 5;  // dI_DB1/dt [V/s]
    static constexpr std::size_t kQint = 6;     // Q_INT [V]
    static constexpr std::size_t kVs = 7;       // V_S lead-lag state [V]
    static constexpr std::size_t kVss = 8;      // V_SS [V]
    static constexpr std::size_t kVdmd = 9;     // V_DMD lead-lag state [V]
    static constexpr std::size_t kMotor = 10;   // the torque motor's lead-lag state [V]
    static constexpr std::size_t kIns = 11;     // INS [m]
    static constexpr std::size_t kDemand =
        12;  // FLQREQ/J_d [rad/s²]: the drive's deceleration were it unpowered
    static constexpr std::size_t kSlip = 13;  // s [rad/s]
    static constexpr std::size_t kNumStates = 14;

    // Throws InitError on an invalid configuration.
    explicit GenHelT700(GenHelT700Config config);

    // Reads a `type: genhel_t700` config node; tables relative to `base`.
    static std::unique_ptr<GenHelT700> from_config(const Config& node);

    std::size_t num_states() const override { return kNumStates; }
    // The equilibrium at zero demand and zero collective.
    void initial_states(std::span<double> x) const override;
    // Trim (D-053 item 6 and amendment): the clutch engaged, its slip held at
    // zero; Q_INT held at the limit the engine's torque drives it to; every
    // other state free.
    void steady_states(const SimContext& ctx, std::span<double> x) const override;
    void steady_kinds(std::span<SteadyKind> kinds) const override;
    // Trim: the equilibrium for the main rotor's demand in ctx's coupling
    // store, the engines delivering it with losses (D-053 item 6).
    bool has_trim_seed() const noexcept override { return true; }
    void trim_seed(const SimContext& ctx, std::span<double> x) const override;
    // Linearization (D-053 amendment 2): the slip while engaged, and Q_INT
    // and V_SS at a limit their inputs push against.
    void pinned_states(const SimContext& ctx, std::span<const double> x,
                       std::span<unsigned char> pinned) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;

    std::string_view name() const noexcept override { return config_.name; }
    std::vector<std::string> consumes() const override;
    void bind(const CouplingRegistry& registry) override;
    void bind_shafts(std::span<const ShaftInfo> shafts) override;

    // The signals at x with the shaft at speed `omega` [rad/s], the collective
    // stick at `collective` [0, 1] and the main rotor's demand `rotor_demand`
    // [N·m]. Pure.
    GenHelT700Signals signals(std::span<const double> x, double omega, double collective,
                              double rotor_demand) const noexcept;

    // The equilibrium states with the shaft at its governed speed, the engines
    // delivering `engine_torque` [N·m, referred to the shaft], the main rotor
    // demanding `rotor_demand` [N·m] and the stick at `collective` [0, 1]
    // (D-053 item 6): on the steady maps, speed error zero, the ECU holding
    // N_GG,ref − N_GG,LDS (V_SS clamped to [A_L, A_U]), Q_INT at the limit its
    // input drives it to, the clutch engaged.
    void equilibrium(double engine_torque, double rotor_demand, double collective,
                     std::span<double> x) const noexcept;

    // The main rotor's demand [N·m] in the coupling store of ctx (0 without
    // one), as derivatives() reads it.
    double rotor_demand(const SimContext& ctx) const noexcept;

    const GenHelT700Config& config() const noexcept { return config_; }
    int shaft_index() const noexcept { return shaft_index_; }
    double governed_speed() const noexcept { return governed_; }  // [rad/s]

private:
    double lds_speed(double collective) const noexcept;
    // Q_PT,ss(N): per engine at the power turbine [N·m].
    double steady_torque(double n_gg) const noexcept;

    // Table lookups with a cached search hint each (spec §7).
    static constexpr std::size_t kTorque = 0;
    static constexpr std::size_t kRigging = 1;
    static constexpr std::size_t kCam = 2;
    static constexpr std::size_t kFuelMap = 3;
    static constexpr std::size_t kP3Map = 4;
    static constexpr std::size_t kDp3 = 5;
    static constexpr std::size_t kDqgg = 6;
    static constexpr std::size_t kDqpt = 7;
    static constexpr std::size_t kDqptN = 8;
    static constexpr std::size_t kTorqueInv = 9;
    static constexpr std::size_t kFuelRef = 10;
    static constexpr std::size_t kP3Ref = 11;
    static constexpr std::size_t kNumLookups = 12;
    double lookup(std::size_t which, const Table1D& t, double x) const noexcept {
        return t(x, cursors_[which]);
    }

    GenHelT700Config config_;
    std::shared_ptr<const Table1D> ngg_torque_;  // Table 6.5.1 inverted: N_GG → per-engine demand
    mutable std::array<TableCursor<1>, kNumLookups> cursors_{};  // search hints only (spec §6)
    CouplingSlot demand_;
    std::array<CouplingSlot, 6> demand_a_;  // its sensitivity to a = [v̇_b; ω̇_b]
    int shaft_index_ = -1;
    double governed_ = 0.0;
};

}  // namespace eyas
