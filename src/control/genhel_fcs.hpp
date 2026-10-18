#pragma once

#include <limits>
#include <memory>
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

// A second-order servo 1/(a2 s² + a1 s + 1) with optional rate and position
// limits (D-045). The command is clamped to the position limits and the
// position output again, so the limits hold without windup; the rate limit
// saturates the servo's velocity state. States: [position, velocity].
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

struct GenHelFcsConfig {
    std::string name;  // instance name (D-003); required
    GenHelRigging rigging;
    ServoConfig primary_servo;            // 1/(.00114 s² + .0473 s + 1), all four channels
    ServoConfig rate_gyro;                // Fig. 5.1.2, p, q, r
    ServoConfig lateral_accelerometer;    // Fig. 5.1.2
    Vec3 accelerometer_b = Vec3::Zero();  // [m] transducer from the CG, body axes
    GenHelStabilatorLaw stabilator;
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
};

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
    static constexpr std::size_t kNumStates = 19;

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
    GenHelFcsConfig config_;
    Table1D ay_ramp_;  // 0 → 1 over the lateral-acceleration ramp, corners rounded
    mutable TableCursor<1> ay_cursor_;  // search hint only (spec §6)
    CouplingSlot theta0_, theta1s_, theta1c_, theta_tr_, stab_;
};

}  // namespace eyas
