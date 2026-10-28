#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "core/component.hpp"
#include "util/axis.hpp"
#include "util/table.hpp"

namespace eyas {
class Config;

// A family of curves y(x), one at each value p_k of a parameter (Ballin's
// Figs. A1 and C23), read as Ballin's and Hart & Wenzel's MAP routine reads
// them (TM-83579 App. C): along each curve in x, clamped at its ends, then
// linearly between the two curves around p, clamped beyond the outermost.
class CurveFamily {
public:
    // params increasing, one curve each, at least two. Throws InitError.
    CurveFamily(std::vector<double> params, std::vector<Table1D> curves);

    // Search hints, one per curve and one for the parameter; sized at init.
    struct Cursor {
        std::size_t param = 0;
        std::vector<TableCursor<1>> curves;
    };
    Cursor make_cursor() const { return Cursor{0, std::vector<TableCursor<1>>(curves_.size())}; }

    double operator()(double p, double x, Cursor& cursor) const noexcept;

    std::span<const double> params() const noexcept { return params_.breakpoints(); }
    const Table1D& curve(std::size_t k) const noexcept { return curves_[k]; }

private:
    Axis params_;
    std::vector<Table1D> curves_;
};

// Ballin's component-level T700 (NASA TM-100991, 1988; D-054): N_E identical
// engines on one shaft, each a gas generator and a free power turbine with the
// T700-GE-700's fuel control.
//
// Gas path (TM-100991 eqs. 1-53, pp. 8-12). States: the gas-generator speed NG
// and the station 4.1 heat sink's metal temperature T_m. The volumes' pressures
// P3, P41, P45 are quasi-steady (p. 12: "in equilibrium at all times"): with the
// gas temperature T41 they are solved every evaluation from the volumes'
// continuity, eqs. (42)-(44) with their integrands zero, and the heat sink's
// gas-side balance, by Newton's method from a start that is a function of the
// state (D-054 item 4). Eq. (43) with + W_f (D-054 item 5). The heat sink
// (eqs. 48-53) is the metal's lag, c_pm M Ṫ_m = hA (T41ns − T_m) with
// Mc_pm/hA = TC_T41 √T41 / W41^0.8, and the gas leaving it at
// T41 = T41ns − (T41SGN/W41)/(Mc_pm/hA) (T41ns − T_m), so that
// T41/T41ns = ((τ2 − τ3) s + 1)/(τ2 s + 1) as eq. (50) has it.
// NG: J_GT ṄG = Q_GT − Q_C (eq. 45). The power turbine's torque Q_PT (eq. 41)
// drives the shaft at NP = gear × Ω, its output inertia JE with it.
//
// Fuel control (TM-100991 App. C, Figs. C1-C22, C30; D-054 item 6), every
// signal in SI with the ECU's speed errors as fractions of NP_des (its gains
// then in volts per fraction): the ECU (load share C2, governor rate
// compensation C3, governor dynamics C4, thermocouple harness C5, T4.5
// compensation C6, selector, P+I compensation C7) drives the HMU's torque motor
// (C10), which trims the HMU's fuel-ratio demand down from the topping line plus
// the NG droop line, less the power-available and load-demand spindles (C9,
// C14-C17), limited by the idle, acceleration and deceleration schedules (C18,
// C20-C22, C30); the metering valve (C19) meters W_f = ratio × P_s3, sensed.
// Hystereses are play operators with a fast lag, the transport delay a
// second-order Padé approximant (D-054 item 7).
//
// Each engine has its own states (D-055 item 3): one block of kEngineStates
// per engine, engine e's at e × kEngineStates. Its power lever sets the
// power-available spindle through the config's rigging (D-055 item 2), the
// cockpit's engine-speed trim the NP reference. Each drives the shaft through
// an overrunning clutch (TM-88360 App. A; D-055 item 5).
//
// Engine failure is a fuel shutoff (D-056): with the fuel off the metering
// valve's command is zero and Ballin's gas path runs the gas generator down
// through the maps. Below the maps' floor (their lowest corrected speed,
// about idle; Ballin models no starts or sub-idle running, p. 24) the model has
// no data: there NG stops at the floor, and with the fuel off the engine is
// out, with no gas path, flow or torque. The fuel back on relights it at
// once.
//
// Drive train (TM-88360 App. A, eqs. A2, A5, A11, A12; D-054 item 3): the
// gearbox's oil drag DAMPBX Ω and the accessories, ACCHP at the governed speed
// falling 10 % per 100 % of speed, load the shaft.
// The tail rotor's torque, when its model has one (D-058), loads it through
// the executive's shaft row; the clutches' rule and trim's seed read it
// from the tail rotor, as the main rotor's.
//
// Inputs: the collective stick (Controls::collective, XCPC = 100 × it), each
// engine's power lever and the engine-speed trim (Controls::engines from
// `first_engine`, Controls::rotor_speed_reference), the shaft's speed, the
// atmosphere at the aircraft (P2, T2). Outputs: the shaft's torque and
// inertia; the spools' angular momentum to the airframe.
struct BallinT700Config {
    std::string name;   // instance name (D-003)
    std::string shaft;  // the shaft it drives
    std::string rotor;  // the instance publishing <rotor>.shaft_torque [N·m] (trim's seed)
    // The tail rotor publishing <tail_rotor>.shaft_torque [N·m], added to the
    // rotor's in the clutches' rule and trim's seed (D-058); empty: none.
    std::string tail_rotor;
    int engines = 2;       // N_E: 1 or 2 (Fig. C2's load share pairs two)
    int first_engine = 0;  // this component's first entry in Controls::engines
    double gear = 0.0;     // n_E: power-turbine speed over the shaft's [-]

    // Table A.1 (p. 41), SI.
    double hvf = 0.0;             // fuel's heating value [J/kg]
    double j_gt = 0.0;            // gas-generator spool [kg·m²]
    double k_bl = 0.0;            // share of the cooling bleed returned at 4.5 [-]
    double k_b3 = 0.0;            // station 3 leakage fraction [-]
    double k_damp = 0.0;          // power turbine's damping about its design speed [N·m·s/rad]
    double k_dpb = 0.0;           // burner pressure-drop constant [Pa²·s²/(kg²·K)]
    double k_h2 = 0.0;            // [J/(kg·K)]
    double k_h3_1 = 0.0;          // [J/(kg·K)]
    double k_h3_2 = 0.0;          // [J/kg]
    double k_h41_1 = 0.0;         // [J/(kg·K)]
    double k_h41_2 = 0.0;         // [J/kg]
    double k_h45 = 0.0;           // [-]
    double k_ps3 = 0.0;           // P_s3 / P3 [-]
    double k_qc_1 = 0.0;          // [-]
    double k_qc_2 = 0.0;          // [-]
    double k_t41_1 = 0.0;         // [K·kg/J]
    double k_t41_2 = 0.0;         // [K]
    double k_t45_1 = 0.0;         // [K·kg/J]
    double k_t45_2 = 0.0;         // [K]
    double k_th41_1 = 0.0;        // [1/K]
    double k_th41_2 = 0.0;        // [-]
    double k_th45_1 = 0.0;        // [1/K]
    double k_th45_2 = 0.0;        // [-]
    double k_wgt = 0.0;           // gas-generator turbine nozzle flow constant [kg/(s·Pa)]
    double ng_des = 0.0;          // [rad/s]
    double np_des = 0.0;          // [rad/s]
    double tc_t41 = 0.0;          // heat sink [kg^0.8·s^0.2/K^0.5]
    double output_inertia = 0.0;  // JE, each engine's output side at its speed [kg·m²]
    Vec3 pt_axis_b = Vec3(1.0, 0.0, 0.0);  // power turbines' spin axis (unit)
    Vec3 gg_axis_b = Vec3(1.0, 0.0, 0.0);  // gas generators' spin axis (unit)

    // Maps f1-f11 (Figs. A1-A11), SI; corrected speeds as fractions of NG_des.
    std::shared_ptr<const CurveFamily> f1;  // (NGC, P_s3/P2) → WA2C [kg/s]
    std::shared_ptr<const Table1D> f2;      // P_s3/P2 → T3/T2
    std::shared_ptr<const Table1D> f3;      // NGC → B1
    std::shared_ptr<const Table1D> f4;      // WA2C [kg/s] → B2
    std::shared_ptr<const Table1D> f5;      // WA2C [kg/s] → B3
    std::shared_ptr<const Table1D> f6;      // FAR → η
    std::shared_ptr<const Table1D> f7;      // P45/P41 → ΔH_GT/θ41 [J/kg]
    std::shared_ptr<const Table1D> f8;      // P49/P45 → ΔH_PT/θ45 [J/kg]
    std::shared_ptr<const Table1D> f9;    // P_s9/P45 → W45C [kg/(s·Pa)] (W45 = W45C P45/√θ45)
    std::shared_ptr<const Table1D> f10;   // NGC → P49/P_s9
    std::shared_ptr<const Table1D> f_hs;  // NGC → T41SGN [kg]

    // Fuel control: Table C.1 (pp. 69-70) and the figures' own constants, SI.
    // ECU.
    double ctpl = 0.0;              // NP sensor lag [s]
    double cb = 0.0;                // speed-error dead band [fraction]
    double ct13 = 0.0, ct14 = 0.0;  // rate compensation lags [s]
    // C3's gains: ZK9, ZK10 [V per fraction] and ZK5 [V·s per fraction] turn the
    // speed error into volts; ZK7 and ZK1 [-] scale the sum (H&W B64, B65).
    double zk1 = 0.0, zk5 = 0.0, zk7 = 0.0, zk9 = 0.0, zk10 = 0.0;
    double band_low = 0.0, band_high = 0.0;         // C3's relay breakpoints [fraction]
    double relay_gain = 0.0;                        // C3's 1000 [N·m·s]
    double b6 = 0.0;                                // [-]
    double cr = 0.0;                                // torque threshold [N·m]
    double ylolim = 0.0, yhilim = 0.0, corr = 0.0;  // the torque integrator [N·m·s]
    double torque_lag1 = 0.0, torque_lag2 = 0.0;    // TL1, TL2: torque sensor lags [s]
    double gov_lead = 0.0, ct2 = 0.0, ct12 = 0.0;   // T11, CT2, CT12: governor dynamics [s]
    double t45cor = 0.0;                            // [K]
    double tlge = 0.0;                              // harness lag [s]
    std::shared_ptr<const CurveFamily> fec1;  // (T45L [K], W45R [kg·K^0.5/(s·Pa)]) → TAU45 [s]
    double t45ref = 0.0;                      // [K]
    double zk3 = 0.0;                         // [V/K]
    double ct9 = 0.0, t8 = 0.0, t10 = 0.0;    // [s]
    double xkprop = 0.0, xkintg = 0.0;        // [-], [1/s]
    double zhilim = 0.0;                      // [V]
    double zlolim_low = 0.0, zlolim_high = 0.0;  // C8, below and above the switch torque [V]
    double zlolim_torque = 0.0;                  // C8's switch torque [N·m]
    double ct16 = 0.0;                           // [s]
    // HMU.
    double tm_bias = 0.0;                  // C10's −0.44 V [V]
    double tm_lead = 0.0, tm_lag = 0.0;    // C10's lead-lag [s]
    double tm_gain = 0.0;                  // C10's 564 [A/V]
    double tm_offset = 0.0;                // C10's −31 mA [A]
    double tmdb = 0.0;                     // [A]
    double tmgn = 0.0;                     // [m/(A·s)]
    double xlolim = 0.0, xhilim = 0.0;     // the flapper's stops [m] (D-054 item 6(b))
    double tmlvg = 0.0;                    // [V/m]
    double tmlg = 0.0;                     // [kg/(s·Pa·m)]
    double ctps3 = 0.0;                    // [s]
    double ps3hys = 0.0;                   // [Pa]
    double rig_gain = 0.0;                 // C13's 0.914 [rad per fraction of collective]
    double rig_bias = 0.0;                 // C13's 5.34 deg [rad]
    double xldhys = 0.0;                   // [rad]
    double kndrp = 0.0;                    // droop slope [kg/(s·Pa) per fraction of NG]
    double ngref = 0.0;                    // [fraction]
    double lds_fuel_bias = 0.0;            // C17's 3.26 [kg/(s·Pa)]
    double clld = 0.0;                     // CLLDS [s]
    double ch = 0.0;                       // NG sensor hysteresis [fraction]
    double cntl = 0.0;                     // [s]
    double pas_idle = 0.0, pas_fly = 0.0;  // the spindle at the lever's stops [rad] (D-055)
    double awfp = 0.0, bwfp = 0.0, wfpdch = 0.0, wfpdcl = 0.0;  // deceleration schedule
    double wfmax = 0.0, wfmin = 0.0;                            // [kg/s]
    double clmv = 0.0;                                          // [s]
    double delay = 0.0;                                         // the valve's transport delay [s]
    double hysteresis_lag = 0.0;              // the play operators' lag [s] (D-054 item 7)
    std::shared_ptr<const Table1D> fhm1;      // T2 [K] → WFPTP [kg/(s·Pa)]
    std::shared_ptr<const Table1D> fhm2;      // PAS [rad] → WFPRF
    std::shared_ptr<const Table1D> fhm3;      // XLDSH [rad] → PNG [fraction]
    std::shared_ptr<const Table1D> fhm4;      // XLDSH [rad] → WFQPS3
    std::shared_ptr<const Table1D> fhm5;      // T2 [K] → WFIRF
    std::shared_ptr<const Table1D> fhm6;      // T2 [K] → PCNGI [fraction]
    std::shared_ptr<const CurveFamily> fhm7;  // (T2 [K], PCNGHL) → WFPAC [kg/(s·Pa)]

    // Load share (Fig. C2, Table C.1; D-055 item 4).
    double share_gain = 0.0;  // 1.1 %NP per ft·lbf [1/(N·m)]
    double ct7 = 0.0;         // [s]
    double dbias = 0.0;       // the limiter's floor and the bias [fraction]
    double ce = 0.0;          // the limiter's ceiling [fraction]
    double zk8 = 0.0;         // [-]
    double t17 = 0.0;         // [s]

    // Clutch (D-055 item 5): the drive's inertia with the blades, at the shaft.
    double drive_inertia = 0.0;  // [kg·m²]

    // Drive train (TM-88360 App. A).
    double gearbox_damping = 0.0;  // DAMPBX [N·m·s/rad] at the shaft
    double accessory_power = 0.0;  // ACCHP [W] at the governed speed
};

// The gas path at one state: the quasi-steady solution and what follows from it.
struct BallinT700Gas {
    double p3 = 0.0, ps3 = 0.0, p41 = 0.0, p45 = 0.0, p49 = 0.0;                // [Pa]
    double t3 = 0.0, t41 = 0.0, t41ns = 0.0, t45 = 0.0;                         // [K]
    double wa2 = 0.0, wa2c = 0.0, wa3 = 0.0, wa31 = 0.0, w41 = 0.0, w45 = 0.0;  // [kg/s]
    double far = 0.0;                          // fuel-air ratio [-]
    double ngc = 0.0;                          // corrected NG [fraction]
    double q_c = 0.0, q_gt = 0.0, q_pt = 0.0;  // per engine [N·m]
    double metal_lag = 0.0;                    // Mc_pm/hA [s]
    int iterations = 0;
    bool converged = false;
};

// The fuel control's signals at one state (outputs and tests).
struct BallinT700Signals {
    BallinT700Gas gas;
    double fuel = 0.0;          // W_f [kg/s]
    double engine_speed = 0.0;  // the engine side's speed referred to the shaft [rad/s]
    double spder = 0.0;         // speed error [fraction]
    double rate_input = 0.0;    // the speed error past C3's dead band [fraction]
    double spds1 = 0.0, spdsf = 0.0, tsig = 0.0, spdss = 0.0, spdsp = 0.0;  // [V]
    double b4 = 0.0;          // the gain switch [0 or 1]
    double zlolim = 0.0;      // [V]
    double motor_in = 0.0;    // the torque motor's input, SPDG − 0.44 V − feedback [V]
    double tm_current = 0.0;  // [A], after the dead band
    double tmru = 0.0;        // the torque motor's trim [kg/(s·Pa)]
    double pas = 0.0;         // [rad]
    double wfpdm = 0.0, wfidm = 0.0, wfpac = 0.0, wfpdc = 0.0, hmusel = 0.0;  // [kg/(s·Pa)]
    double ps3l = 0.0;                                                        // [Pa]
    double xldsa = 0.0;                                                       // [rad]
    double dwfp = 0.0;          // the load-demand spindle's command [kg/(s·Pa)]
    double fuel_command = 0.0;  // WFMV, limited [kg/s]
    double q_engine = 0.0;      // the engine's torque referred to the shaft [N·m]
    bool out = false;           // flamed out at the maps' floor: no gas path (D-056)
};

// What one engine's fuel control reads besides its states (D-055).
struct BallinT700Inputs {
    double omega = 0.0;         // the shaft's speed [rad/s]
    double collective = 0.0;    // [0, 1]
    double power_lever = 1.0;   // [0, 1], 0 the idle stop, 1 fly
    double np_reference = 1.0;  // PCPRF [fraction of NP_des]
    double p2 = 0.0;            // [Pa]
    double t2 = 0.0;            // [K]
    bool fuel_on = true;        // the shutoff open (D-056)
};

class BallinT700 : public DynamicComponent {
public:
    static constexpr std::size_t kMaxEnginesPerComponent = 2;

    // State indices within an engine's block.
    static constexpr std::size_t kNg = 0;            // NG / NG_des
    static constexpr std::size_t kMetal = 1;         // T_m / T_std
    static constexpr std::size_t kNpSensor = 2;      // PCNP lagged [fraction]
    static constexpr std::size_t kRate1 = 3;         // C3's CT13 lag [fraction]
    static constexpr std::size_t kRate2 = 4;         // C3's CT14 lag [fraction]
    static constexpr std::size_t kTorqueInt = 5;     // C3's ∫(TRQL − CR) [N·m·s]
    static constexpr std::size_t kTorqueLag = 6;     // TL1 lag [N·m]
    static constexpr std::size_t kTorqueSensed = 7;  // TRQL [N·m]
    static constexpr std::size_t kGov1 = 8;          // C4's lead-lag [V]
    static constexpr std::size_t kSpdsf = 9;         // SPDSF [V]
    static constexpr std::size_t kT45l = 10;         // T45L / T_std
    static constexpr std::size_t kT45el = 11;        // T45EL / T_std
    static constexpr std::size_t kTemp1 = 12;        // C6's CT9 lag [V]
    static constexpr std::size_t kTsig = 13;         // C6's lead-lag [V]
    static constexpr std::size_t kZ = 14;            // C7's integrator [V]
    static constexpr std::size_t kSpdg = 15;         // SPDG [V]
    static constexpr std::size_t kTm1 = 16;          // C10's lead-lag [V]
    static constexpr std::size_t kTmPos = 17;        // the flapper's position [m]
    static constexpr std::size_t kPs3Lag = 18;       // P_s3 lagged / P_std
    static constexpr std::size_t kPs3Hys = 19;       // PS3L / P_std
    static constexpr std::size_t kLdsHys = 20;       // XLDSH [rad]
    static constexpr std::size_t kDwfpl = 21;        // DWFPL / ratio scale
    static constexpr std::size_t kNgHys = 22;        // PCNG after hysteresis [fraction]
    static constexpr std::size_t kNgSensor = 23;     // PCNGHL [fraction]
    static constexpr std::size_t kValve = 24;        // the metering valve's lag / fuel scale
    static constexpr std::size_t kDelay1 = 25;       // Padé states / fuel scale
    static constexpr std::size_t kDelay2 = 26;
    static constexpr std::size_t kSlip = 27;      // the clutch's slip, at the shaft [rad/s]
    static constexpr std::size_t kShareLag = 28;  // C2's CT7 lag, 1.1 TRQER [fraction]
    static constexpr std::size_t kXqlo = 29;      // C2's T17 lag, XQLO [fraction]
    static constexpr std::size_t kEngineStates = 30;

    // Scales of the states held as ratios (so that trim's tolerances fit).
    static constexpr double kTstd = 288.15;      // [K]
    static constexpr double kPstd = 101325.0;    // [Pa]
    static constexpr double kFuelScale = 0.1;    // [kg/s]
    static constexpr double kRatioScale = 1e-6;  // [kg/(s·Pa)], about 0.24 lbm/(hr·psi)

    // Throws InitError on an invalid configuration.
    explicit BallinT700(BallinT700Config config);

    // `first_engine`: the first of Controls::engines this component's engines take.
    static std::unique_ptr<BallinT700> from_config(const Config& node, int first_engine = 0);

    std::size_t num_states() const override {
        return kEngineStates * static_cast<std::size_t>(config_.engines);
    }
    void initial_states(std::span<double> x) const override;
    void steady_states(const SimContext& ctx, std::span<double> x) const override;
    void steady_kinds(std::span<SteadyKind> kinds) const override;
    bool has_trim_seed() const noexcept override { return true; }
    void trim_seed(const SimContext& ctx, std::span<double> x) const override;
    void pinned_states(const SimContext& ctx, std::span<const double> x,
                       std::span<unsigned char> pinned) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;

    std::string_view name() const noexcept override { return config_.name; }
    std::vector<std::string> consumes() const override;
    void bind(const CouplingRegistry& registry) override;
    void bind_shafts(std::span<const ShaftInfo> shafts) override;
    ReadoutRole readout_role() const noexcept override { return ReadoutRole::engines; }

    // One engine's readouts (D-060) at its states `x` (kEngineStates) and
    // its signals there.
    EngineState readout(std::span<const double> x, const BallinT700Signals& s) const noexcept;

    // The rotors' demand at the shaft [N·m] in the coupling store of ctx, the
    // main rotor's and any tail rotor's (D-058); 0 without a store.
    double rotor_demand(const SimContext& ctx) const noexcept;

    // The gas path at gas-generator speed `ng` and power-turbine speed `np`
    // [rad/s], metal temperature `metal` [K], fuel flow `fuel` per engine
    // [kg/s], inlet P2 [Pa], T2 [K]. Pure.
    BallinT700Gas gas(double ng, double np, double metal, double fuel, double p2,
                      double t2) const noexcept;

    // The steady gas path (NG on its own, the heat sink at rest: T_m = T41ns)
    // at fuel flow `fuel` per engine [kg/s] and power-turbine speed `np`
    // [rad/s]; `ng` returns the gas generator's speed [rad/s]. Init and trim
    // time. Returns converged.
    bool steady_gas(double fuel, double np, double p2, double t2, double& ng,
                    BallinT700Gas& out) const noexcept;

    // The steady fuel flow per engine [kg/s] for which `engines` engines (0:
    // all) deliver `shaft_torque` [N·m] to the shaft at speed `omega` [rad/s],
    // with the gas path at steady state. Trim time. Returns converged.
    bool steady_for_torque(double shaft_torque, double omega, double p2, double t2, double& fuel,
                           double& ng, BallinT700Gas& out, int engines = 0) const noexcept;

    // Engine `e`'s inputs in ctx (D-055 item 1).
    BallinT700Inputs inputs(const SimContext& ctx, int e) const noexcept;

    // One engine's signals at its states `x` (kEngineStates) and inputs. Pure.
    BallinT700Signals signals(std::span<const double> x, const BallinT700Inputs& in) const noexcept;

    // The lowest corrected gas-generator speed every map read by it covers
    // [fraction of NG_des] (D-056 item 3).
    double maps_floor() const noexcept { return maps_floor_; }
    // NG at the maps' floor at inlet temperature `t2` [K] [fraction of NG_des].
    double ng_floor(double t2) const noexcept;
    // Out (D-056 item 3): the fuel off and NG [fraction of NG_des] at or below
    // the floor.
    bool is_out(double ng, const BallinT700Inputs& in) const noexcept;

    // The drive train's torque on the shaft at speed `omega` [rad/s] [N·m, a load].
    double drive_train_load(double omega) const noexcept;

    // One engine's equilibrium states `x` (kEngineStates), engaged, at its
    // inputs, its gas path at fuel flow `fuel` [kg/s], the other engine's
    // torque sensed at its own (no load share).
    void equilibrium(double fuel, const BallinT700Inputs& in, std::span<double> x) const noexcept;

    // A lit engine's clutch slip in a steady state at its states `x`
    // (kEngineStates; its own slip ignored) and inputs [rad/s, ≥ 0] (D-056
    // item 5): zero where it drives, else where its power turbine's torque is
    // zero. Trim time.
    double steady_slip(const SimContext& ctx, std::span<const double> x,
                       const BallinT700Inputs& in) const noexcept;
    // An out engine's states `x` (kEngineStates) at rest (D-056 item 3).
    void out_rest(const BallinT700Inputs& in, std::span<double> x) const noexcept;

    // The power-available spindle's angle for a power lever at `lever` (D-055
    // item 2) [rad]; clamped to the stops, NaN reads as fly.
    double spindle(double lever) const noexcept;

    // Which engines the clutches engage (D-055 item 5), bit e for engine e,
    // given each engine's torque at the shaft `q`, slip `slip` and the drive's
    // load `load` [N·m]. Pure.
    unsigned engaged(std::span<const double> q, std::span<const double> slip,
                     double load) const noexcept;

    const BallinT700Config& config() const noexcept { return config_; }
    int shaft_index() const noexcept { return shaft_index_; }
    double governed_speed() const noexcept { return governed_; }  // [rad/s]
    // The ECU's NP reference PCPRF [fraction of NP_des]: the shaft's governed
    // speed once bound (D-054 item 2), the design speed until then.
    double np_reference() const noexcept { return np_reference_; }

private:
    // Lookup hints (spec §7).
    struct Cursors {
        CurveFamily::Cursor f1, fec1;
        CurveFamily::Cursor fhm7;
        std::array<TableCursor<1>, 19>
            t{};  // f2-f_hs 0-9, F_HM1, 2, 6, 5 10-13, starts 14-16, F_HM3, 4 17-18
    };
    // Shared by the dynamics, trim and linearization, so they can't disagree.
    double xldsa(double collective) const noexcept;      // Fig. C13 [rad]
    double load_share(double share_lag) const noexcept;  // Fig. C2's limiter less DBIAS [fraction]
    double gain_switch(double spder, double torque_int) const noexcept;  // B4, Fig. C3
    double zlolim(double trql) const noexcept;                           // Fig. C8 [V]
    double dwfp(double xldsh) const noexcept;                            // Fig. C17
    double motor_input(double spdg, double pos) const noexcept;          // Fig. C10 [V]
    double flapper_at_rest(double spdg) const noexcept;                  // zero current [m]
    // The shaft's speed, or the design speed if the shaft isn't in ctx [rad/s].
    double shaft_speed(const SimContext& ctx) const noexcept;
    bool on_shaft(const SimContext& ctx) const noexcept;

    double residuals(const double* y, double ng, double np, double metal, double fuel, double p2,
                     double t2, double* r, BallinT700Gas* out) const noexcept;
    // Init-time: the rest point at shaft speed omega [rad/s]; throws InitError
    // if the maps give none.
    void find_rest(double omega);
    // The rest point's fuel scaled to inlet pressure p2 [Pa]: trim's fallback [kg/s].
    double rest_fuel_at(double p2) const noexcept;

    BallinT700Config config_;
    std::shared_ptr<const Table1D> start_p3_, start_p41_, start_p45_;  // the steady line by NGC
    mutable Cursors cursors_;
    CouplingSlot demand_;
    CouplingSlot tail_demand_;  // D-058
    double maps_floor_ = 0.0;   // NGC [fraction] (D-056 item 3)
    // A steady state known at init (sea level, zero shaft torque, at the bound
    // shaft's governed speed or before binding the design speed): the initial
    // state, and where a trim-time solve that fails falls back.
    double rest_omega_ = 0.0;  // [rad/s]
    double rest_fuel_ = 0.0;
    double rest_ng_ = 0.0;
    BallinT700Gas rest_gas_;
    double np_reference_ = 1.0;
    int shaft_index_ = -1;
    double governed_ = 0.0;
};

}  // namespace eyas
