#pragma once

#include <array>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "core/component.hpp"
#include "core/coupling.hpp"
#include "util/table.hpp"

namespace eyas {
class Config;
}

// The UH-60A flight control system: GenHel CR-166309 §5.5 (pp. 5.5-2 to
// 5.5-20), M8c (D-029, D-045). One component, as GenHel's module is one:
//   pilot inputs [in] → mixer (Figs. 5.1.6(a)–(d)) → primary servos → rotor
//   head limits → blade pitch for the main and tail rotors;
//   airspeed, collective, pitch rate and lateral acceleration → stabilator
//   network (Fig. 5.1.7) → stabilator servo → incidence.
// The sensors are those of Fig. 5.1.2. The SAS, pitch bias actuator and FPS
// add their inch inputs ahead of the mixer (M8c2, M8c3).
//
// Coupling (D-003): produces <name>.theta0, <name>.theta1s, <name>.theta1c
// (main-rotor swashplate pitch at the hub centre, Padfield 3rd ed. Eq. 3.59)
// and <name>.theta_tr (tail-rotor collective) [rad], and
// <name>.stabilator_incidence [rad]. All are functions of the component's own
// states, so it consumes nothing and runs first.
namespace eyas {

// A second-order servo 1/(a2 s² + a1 s + 1) with an optional rate limit
// (D-045), which saturates the servo's velocity state. States: [position,
// velocity]. It has no position limits of its own: where a channel has them
// (the rotor head limits, the stabilator travel), the component clamps that
// channel's command and its output.
struct ServoConfig {
    double a2 = 0.0;          // [s²]
    double a1 = 0.0;          // [s]
    double rate_limit = 0.0;  // [unit/s] of the position; 0: none
};

// Pilot stick travel: inches at the low and high end of the normalized input
// (collective 0..1, the others −1..1), GenHel's XC+2, XA+2, XB+2, XP+2.
struct StickTravel {
    double lo = 0.0;  // [m]
    double hi = 0.0;  // [m]
};

// GenHel's mechanical control system, Figs. 5.1.6(a)–(d) and p. 5.5-20, and
// the swashplate phase (p. 5.1-24). Gains [rad/m], biases and limits [rad].
struct GenHelRigging {
    StickTravel collective, lat_cyclic, lon_cyclic, pedal;
    double k_collective = 0.0;                 // KXCTH0
    double k_lateral = 0.0;                    // KXAA1S
    double k_longitudinal = 0.0;               // KXBB1S
    double k_pedal = 0.0;                      // KXPTHR
    double sign_lon_stick = 0.0;               // SIGN_XB2: XB = SIGN_XB2 · XB+2 + ...
    double mix_lateral_collective = 0.0;       // MIX_AC [-]
    double mix_longitudinal_collective = 0.0;  // MIX_BC [-]
    double mix_longitudinal_pedal = 0.0;       // MIX_BP [-]
    double bias_collective = 0.0;              // BIAS_C
    double bias_lateral = 0.0;                 // BIAS_A
    double bias_longitudinal = 0.0;            // BIAS_B
    double bias_pedal = 0.0;                   // BIAS_P
    double swashplate_phase = 0.0;             // DSP
    // Rotor head travel limits {min, max} (p. 5.5-20), "for piloted
    // simulation"; none by default (GenHel's analysis runs, D-045).
    double theta0_min = -kNoLimit, theta0_max = kNoLimit;  // THETA0 at the cuff
    double a1s_min = -kNoLimit, a1s_max = kNoLimit;
    double b1s_min = -kNoLimit, b1s_max = kNoLimit;
    static constexpr double kNoLimit = std::numeric_limits<double>::infinity();
};

// Fig. 5.1.7, with the steady part as Eyas's existing schedule table (D-028,
// D-040): IHT0 = table(V_F, collective) + K_q q_s + K_ay(V_F) a_ys, where V_F
// is the equivalent airspeed through the lag 1/(τ s + 1) and K_ay ramps
// linearly from 0 at ay_speed_lo to ay_gain at ay_speed_hi (Fig. 5.1.7's
// KVYDT·limit(V_F − 30, 0, 30) written as a gain), its corners rounded over
// a tenth of the ramp (D-045, as D-038).
struct GenHelStabilatorLaw {
    IncidenceSchedule schedule;    // table and travel limits
    double airspeed_lag = 0.0;     // τ [s]
    double pitch_rate_gain = 0.0;  // K_q, KVTDT [s] (rad per rad/s)
    double ay_gain = 0.0;          // [rad/(m/s²)] at and above ay_speed_hi
    double ay_speed_lo = 0.0;      // [m/s]
    double ay_speed_hi = 0.0;      // [m/s]
    ServoConfig servo;             // Fig. 5.1.7 servo and rate limit
};

// A first-order lag K/(τ s + 1); K in the units of its block.
struct Lag {
    double gain = 0.0;
    double tau = 0.0;  // [s]
};

// The SAS (Figs. 5.1.3(a)–(c), text p. 5.5-2; M8c2), each axis a digital and
// an analog channel, in SI: rates [rad/s], angles [rad], acceleration
// [m/s²], airspeed [m/s] (equivalent, D-040), outputs a fraction of
// authority (1 = 100 %). Names are GenHel's (fig_5_1_3_sas.txt).
struct GenHelSasPitch {
    double ad_lag = 0.0;        // P_AD [s], A/D filter (also the PBA's pitch rate)
    Lag rate_d, lagged_rate_d;  // KRQQPD, KLQQPD [1/(rad/s)]
    double washout_d = 0.0;     // TWQQPD [s]
    double limit_d = 0.0;       // LMPD [-]
    Lag filter_a;               // KQQPA [-], TQQPA
    double washout_a = 0.0;     // TWQQPA [s]
    Lag rate_a, lagged_rate_a;  // KRQQPA, KLQQPA [1/(rad/s)]
    double scale_a = 0.0;       // the analog ".5 %/%" block [-]
    double limit_a = 0.0;       // LMPA [-]
};

struct GenHelSasRoll {
    double ad_lag = 0.0;            // R_AD [s]
    double rate_gain_d = 0.0;       // KRPPRD [1/(rad/s)]
    Lag lagged_rate_d;              // KLPPRD [1/(rad/s)]
    double attitude_gain_d = 0.0;   // KPHRD [1/rad]
    double attitude_limit_d = 0.0;  // LMPHRD [rad]
    double limit_d = 0.0;           // LMRD [-]
    // Analog: roll rate derived from the vertical gyro's roll attitude.
    double attitude_filter_gain = 0.0;                // KPHF1 [-]
    double attitude_filter_t1 = 0.0;                  // T1PHF1 [s]
    double attitude_filter_t2 = 0.0;                  // T2PHF1 [s]
    double shaping_gain = 0.0;                        // KPHF2 [-]
    double shaping_a2 = 0.0, shaping_a1 = 0.0;        // A2PHF2 [s²], A1PHF2 [s]
    double shaping_limit = 0.0;                       // LMPHR1 [rad]
    double derivative_gain = 0.0;                     // KPHF3 [-] (its s is the derivative)
    double derivative_t1 = 0.0, derivative_t2 = 0.0;  // T1PHF3, T2PHF3 [s]
    Lag rate_a, lagged_rate_a;                        // KRPHRA, KLPHRA [1/(rad/s)]
    double attitude_gain_a = 0.0;                     // KPHR1 [-]
    double attitude_limit_a = 0.0;                    // LMPHRA [rad]
    double attitude_output_gain_a = 0.0;              // KPHIRA [1/rad]
    double limit_a = 0.0;                             // LMRA [-]
};

struct GenHelSasYaw {
    double ad_lag = 0.0;            // Y_AD [s]
    Lag rate_d;                     // KRRRYD [1/(rad/s)]
    Lag lagged_rate_d;              // KLRRYD [1/(rad/s)], below the switch speed only
    double washout_d = 0.0;         // TWRRYD [s]
    Lag lateral_accel_d;            // KLAYYD [1/(m/s²)], faded in above the switch speed
    double roll_rate_gain_d = 0.0;  // KRPPYD [1/(rad/s)], on the digital roll A/D output
    double limit_d = 0.0;           // LMYD [-]
    Lag filter_a;                   // KRRFI [-], TRRFI
    double washout_a = 0.0;         // TWRRYA [s]
    Lag rate_a;                     // KRRRYA [1/(rad/s)]
    // KLAYF1 [(rad/s)/(m/s²)]: the lateral acceleration into the lagged path
    // above the switch speed (its 0.0001 s lag is below the step: unity, p. 5.5-2).
    double lateral_accel_gain_a = 0.0;
    Lag lagged_a;                   // KLAYRR [1/(rad/s)], TLAYRR
    double scale_a = 0.0;           // the analog ".5 %/%" block [-]
    double roll_rate_gain_a = 0.0;  // KRPPYA [-], on the analog roll rate path's output
    double limit_a = 0.0;           // LMYA [-]
};

struct GenHelSas {
    GenHelSasPitch pitch;
    GenHelSasRoll roll;
    GenHelSasYaw yaw;
    double hold = 0.0;          // digital zero-order hold [s] (a Padé of half of it, D-045)
    double switch_speed = 0.0;  // VG60SW [m/s]
    double fade_time = 0.0;     // [s] "2 SEC FADE"
    ServoConfig actuator;       // SAS actuator, on the inches (Figs. 5.1.6(b)–(d))
    // Percent to blade pitch, KPCDGP, KPCDGR, KPCDGY [rad per unit fraction];
    // divided by the rigging gain into the inner-loop inches XBILS, XAILS, XPILS.
    double descale_pitch = 0.0, descale_roll = 0.0, descale_yaw = 0.0;
};

// The pitch bias actuator, Fig. 5.1.4 and text p. 5.5-3 (M8c2).
struct GenHelPitchBias {
    double attitude_gain = 0.0;   // KTHBA [1/rad]
    double attitude_limit = 0.0;  // LMTHBA [rad]
    double rate_gain = 0.0;       // KQQBA [1/(rad/s)], on the pitch SAS A/D output
    double rate_limit_in = 0.0;   // LMQQBA [rad/s]
    double speed_gain = 0.0;      // KVXBA [s/m]
    double speed_from = 0.0;      // [m/s] the −80 kt bias
    double speed_range = 0.0;     // [m/s] LMVXBA's upper limit
    double bias = 0.0;            // BIAS_BA [-]
    double hold = 0.0;            // [s] zero-order hold (a Padé of half of it, D-045)
    double actuator_gain = 0.0;   // [1/s]
    double rate_limit = 0.0;      // [1/s] actuator rate
    double authority = 0.0;       // [-] actuator travel, ±
};

struct GenHelFcsConfig {
    std::string name;  // instance name (D-003); required
    GenHelRigging rigging;
    ServoConfig primary_servo;            // 1/(.00114 s² + .0473 s + 1), all four channels
    ServoConfig rate_gyro;                // Fig. 5.1.2, p, q, r
    ServoConfig lateral_accelerometer;    // Fig. 5.1.2
    Vec3 accelerometer_b = Vec3::Zero();  // [m] transducer from the CG, body axes
    GenHelStabilatorLaw stabilator;
    // Optional (M8c2): without them their states stay at zero and the
    // switches (Controls::afcs) do nothing.
    std::optional<GenHelSas> sas;
    std::optional<GenHelPitchBias> pitch_bias;
};

// Signals of one evaluation, for outputs and tests.
struct GenHelFcsOutputs {
    double theta0 = 0.0, theta1s = 0.0, theta1c = 0.0;  // [rad] main rotor, Eq. 3.59
    double theta_tr = 0.0;                              // [rad] THETTR
    double a1s = 0.0, b1s = 0.0, theta0_cuff = 0.0;     // [rad] GenHel's A1S, B1S, THETA0
    double stabilator_incidence = 0.0;                  // [rad]
    // Mixer outputs (servo commands), GenHel A1SMIX, B1SMIX, THOMIX, THRMIX,
    // within the head limits [rad].
    double a1s_cmd = 0.0, b1s_cmd = 0.0, theta0_cmd = 0.0, thettr_cmd = 0.0;
    double stabilator_cmd = 0.0;  // IHT0 within the travel limits [rad]
    // SAS channel outputs PDSASO, PASASO, RDSASO, RASASO, YDSASO, YASASO [-]
    // (fractions; the digital ones after the hold), the PBA position [-], and
    // the inner-loop inches XBILS, XAILS, XPILS, XBBAS [m].
    double pitch_d = 0.0, pitch_a = 0.0, roll_d = 0.0, roll_a = 0.0, yaw_d = 0.0, yaw_a = 0.0;
    double pba = 0.0;
    double xbils = 0.0, xails = 0.0, xpils = 0.0, xbbas = 0.0;
};

namespace detail {
struct GenHelAfcsSignals;  // one evaluation's SAS and PBA signals (genhel_fcs.cpp)
}

class GenHelFcs : public DynamicComponent {
public:
    // State layout.
    static constexpr std::size_t kServoTheta0 = 0;  // each servo: position, velocity
    static constexpr std::size_t kServoA1s = 2;
    static constexpr std::size_t kServoB1s = 4;
    static constexpr std::size_t kServoThettr = 6;
    static constexpr std::size_t kGyroP = 8;  // each sensor: value, rate
    static constexpr std::size_t kGyroQ = 10;
    static constexpr std::size_t kGyroR = 12;
    static constexpr std::size_t kAccelY = 14;
    static constexpr std::size_t kAirspeedLag = 16;  // V_F [m/s]
    static constexpr std::size_t kServoStab = 17;
    // SAS (M8c2). First-order states hold a lag's output (washouts: the lag
    // their output subtracts from the input; holds: the Padé's lag); the
    // actuators are position and velocity [m].
    static constexpr std::size_t kPitchAd = 19;       // q₁, the A/D filter output
    static constexpr std::size_t kPitchRateD = 20;    // KRQQPD's lag
    static constexpr std::size_t kPitchLagD = 21;     // KLQQPD's lag
    static constexpr std::size_t kPitchWashD = 22;    // WSQQPD
    static constexpr std::size_t kPitchHoldD = 23;    // zero-order hold
    static constexpr std::size_t kPitchFilterA = 24;  // KQQPA's lag
    static constexpr std::size_t kPitchWashA = 25;    // WSQQPA
    static constexpr std::size_t kPitchRateA = 26;    // KRQQPA's lag
    static constexpr std::size_t kPitchLagA = 27;     // KLQQPA's lag
    static constexpr std::size_t kPitchAct = 28;      // SAS actuator → XBILS
    static constexpr std::size_t kRollAd = 30;        // p₁
    static constexpr std::size_t kRollLagD = 31;      // KLPPRD's lag
    static constexpr std::size_t kRollHoldD = 32;
    static constexpr std::size_t kRollAtt1 = 33;  // KPHF1's lags, in series
    static constexpr std::size_t kRollAtt2 = 34;
    static constexpr std::size_t kRollShape = 35;  // KPHF2's second order (2 states)
    static constexpr std::size_t kRollDer1 = 37;   // KPHF3's lags, in series
    static constexpr std::size_t kRollDer2 = 38;
    static constexpr std::size_t kRollRateA = 39;  // KRPHRA's lag
    static constexpr std::size_t kRollLagA = 40;   // KLPHRA's lag
    static constexpr std::size_t kRollAct = 41;    // → XAILS
    static constexpr std::size_t kYawAd = 43;      // r₁
    static constexpr std::size_t kYawRateD = 44;   // KRRRYD's lag
    static constexpr std::size_t kYawLagD = 45;    // KLRRYD's lag
    static constexpr std::size_t kYawWashD = 46;   // WSRRYD
    static constexpr std::size_t kYawAccelD = 47;  // KLAYYD's lag
    static constexpr std::size_t kYawFade = 48;    // the 2 s fade, 0..1
    static constexpr std::size_t kYawHoldD = 49;
    static constexpr std::size_t kYawFilterA = 50;  // KRRFI's lag
    static constexpr std::size_t kYawWashA = 51;    // WSRRYA
    static constexpr std::size_t kYawRateA = 52;    // KRRRYA's lag
    static constexpr std::size_t kYawLagA = 53;     // KLAYRR's lag
    static constexpr std::size_t kYawAct = 54;      // → XPILS
    // Pitch bias actuator (M8c2).
    static constexpr std::size_t kPbaHold = 56;  // zero-order hold
    static constexpr std::size_t kPba = 57;      // actuator position [-]
    static constexpr std::size_t kNumStates = 58;

    explicit GenHelFcs(GenHelFcsConfig config);

    // Reads a `type: genhel_fcs` config node.
    static std::unique_ptr<GenHelFcs> from_config(const Config& node);

    std::size_t num_states() const override { return kNumStates; }
    // Sensors and airspeed lag at zero, each servo at rest at its command for
    // zero pilot inputs and zero airspeed. Trim and the host's start solve them.
    void initial_states(std::span<double> x) const override;
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override;
    // Every state: each servo at rest at its command, each sensor reading its
    // input with the body acceleration zero, the airspeed lag at the
    // equivalent airspeed (D-045).
    void steady_states(const SimContext& ctx, std::span<double> x) const override;

    std::string_view name() const noexcept override { return config_.name; }
    std::vector<std::string> produces() const override;
    void bind(const CouplingRegistry& registry) override;

    // The signals at states x (no derivatives).
    GenHelFcsOutputs evaluate(const SimContext& ctx, std::span<const double> x) const noexcept;

    const GenHelFcsConfig& config() const noexcept { return config_; }

private:
    // evaluate() with the SAS and PBA signals already computed, so that
    // derivatives() computes them once.
    GenHelFcsOutputs evaluate(const SimContext& ctx, std::span<const double> x,
                              const detail::GenHelAfcsSignals& s) const noexcept;

    GenHelFcsConfig config_;
    // Time constant of each first-order SAS and PBA state [s]; 0 for the rest.
    std::array<double, kNumStates> tau_{};
    Table1D ay_ramp_;  // 0 → 1 over the lateral-acceleration ramp, corners rounded
    mutable TableCursor<1> ay_cursor_;  // search hint only (spec §6)
    CouplingSlot theta0_, theta1s_, theta1c_, theta_tr_, stab_;
};

}  // namespace eyas
