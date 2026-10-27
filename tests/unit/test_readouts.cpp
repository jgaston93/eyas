// The readouts (D-059, SPEC_REVIEW #32): the components report Tier 4 and
// the flight-control outputs during the output evaluation. Checked here
// against the components' own queries, made the way Model made them before
// D-059: the executive's evaluation at the final state, then each
// component's evaluate() and state queries, found by type.

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "airframe/airframe_aero.hpp"
#include "airframe/landing_gear.hpp"
#include "control/genhel_fcs.hpp"
#include "core/rigid_body.hpp"
#include "eyas/model.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/multiblade.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

std::string config_path(const std::string& name) {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/" + name;
}

// The first component of type T, config order; kNone if there is none.
constexpr std::size_t kNone = static_cast<std::size_t>(-1);
template <class T>
std::size_t first(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const T*>(&ex.component(i)) != nullptr) {
            return i;
        }
    }
    return kNone;
}

// Tier 4 and the flight-control fields as Model computed them before D-059.
AircraftState queried(const Executive& ex, const Controls& c, const Environment& env) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    AircraftState o;
    o.rotor_speed_percent = o.main_rotor_thrust = o.main_rotor_torque = o.main_rotor_power = nan;
    o.coning = o.flap_1c = o.flap_1s = o.main_rotor_azimuth = nan;
    o.tail_rotor_thrust = o.tail_rotor_azimuth = nan;
    o.stabilator_incidence = nan;
    o.main_rotor_theta0 = o.main_rotor_theta1s = o.main_rotor_theta1c = o.tail_rotor_theta0 = nan;
    Vec6 a;
    (void)ex.body_acceleration(c, env, a);
    const std::span<const double> x = ex.state();
    const SimContext ctx = ex.context(ex.sim_time(), x, c, env);
    const auto states = [&](std::size_t i) {
        return x.subspan(ex.component_offset(i), ex.component_size(i));
    };
    if (const std::size_t i = first<BladeElementRotor>(ex); i != kNone) {
        const auto& rotor = static_cast<const BladeElementRotor&>(ex.component(i));
        const RotorLoads loads = rotor.evaluate(ctx, states(i));
        const double psi = rotor.azimuth(ctx);
        o.rotor_speed_percent = 100.0 * rotor.rotor_speed(ctx) / rotor.config().rotor_speed;
        o.main_rotor_thrust = loads.thrust;
        o.main_rotor_torque = loads.torque;
        o.main_rotor_power = loads.power;
        o.main_rotor_azimuth = psi;
        MultibladeFlap m;
        if (rotor.config().flap) {
            m = multiblade_flap(states(i), psi, rotor.config().num_blades);
        }
        o.coning = m.beta0;
        o.flap_1c = m.beta1c;
        o.flap_1s = m.beta1s;
    }
    if (const std::size_t i = first<BaileyRotor>(ex); i != kNone) {
        const auto& rotor = static_cast<const BaileyRotor&>(ex.component(i));
        o.tail_rotor_thrust = rotor.evaluate(ctx, states(i)).thrust;
        o.tail_rotor_azimuth = rotor.azimuth(ctx, states(i));
    }
    o.weight_on_wheels = false;
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* gear = dynamic_cast<const LandingGear*>(&ex.component(i))) {
            for (std::size_t k = 0; k < gear->config().gears.size(); ++k) {
                o.weight_on_wheels =
                    o.weight_on_wheels || gear->gear_state(ctx, states(i), k).normal_load > 0.0;
            }
        }
    }
    if (const std::size_t i = first<AirframeAero>(ex); i != kNone) {
        o.stabilator_incidence = static_cast<const AirframeAero&>(ex.component(i))
                                     .evaluate(ctx, states(i))
                                     .stabilator_incidence;
    }
    if (const std::size_t i = first<GenHelFcs>(ex); i != kNone) {
        const GenHelFcsOutputs f =
            static_cast<const GenHelFcs&>(ex.component(i)).evaluate(ctx, states(i));
        o.main_rotor_theta0 = f.theta0;
        o.main_rotor_theta1s = f.theta1s;
        o.main_rotor_theta1c = f.theta1c;
        o.tail_rotor_theta0 = f.theta_tr;
    }
    return o;
}

// Bit for bit, NaN equal to NaN.
void expect_same(double got, double want, const char* field) {
    EXPECT_EQ(std::bit_cast<std::uint64_t>(got), std::bit_cast<std::uint64_t>(want))
        << field << ": got " << got << ", want " << want;
}

void expect_readouts(const Model& m, const Controls& c, const Environment& env,
                     const std::string& config) {
    const Config cfg = Config::from_file(config);
    Executive ex = build_executive(cfg, read_origin(cfg).origin().altitude);
    const std::span<const double> x = m.state();
    const std::int64_t steps = m.outputs().sim_time_ns / ex.step_ns();
    ASSERT_TRUE(ex.restore(x, steps, m.remainder_ns()));
    const AircraftState want = queried(ex, c, env);
    const AircraftState& o = m.outputs();
    SCOPED_TRACE(o.sim_time);
    expect_same(o.rotor_speed_percent, want.rotor_speed_percent, "rotor_speed_percent");
    expect_same(o.main_rotor_thrust, want.main_rotor_thrust, "main_rotor_thrust");
    expect_same(o.main_rotor_torque, want.main_rotor_torque, "main_rotor_torque");
    expect_same(o.main_rotor_power, want.main_rotor_power, "main_rotor_power");
    expect_same(o.coning, want.coning, "coning");
    expect_same(o.flap_1c, want.flap_1c, "flap_1c");
    expect_same(o.flap_1s, want.flap_1s, "flap_1s");
    expect_same(o.main_rotor_azimuth, want.main_rotor_azimuth, "main_rotor_azimuth");
    expect_same(o.tail_rotor_thrust, want.tail_rotor_thrust, "tail_rotor_thrust");
    expect_same(o.tail_rotor_azimuth, want.tail_rotor_azimuth, "tail_rotor_azimuth");
    EXPECT_EQ(o.weight_on_wheels, want.weight_on_wheels);
    expect_same(o.stabilator_incidence, want.stabilator_incidence, "stabilator_incidence");
    expect_same(o.main_rotor_theta0, want.main_rotor_theta0, "main_rotor_theta0");
    expect_same(o.main_rotor_theta1s, want.main_rotor_theta1s, "main_rotor_theta1s");
    expect_same(o.main_rotor_theta1c, want.main_rotor_theta1c, "main_rotor_theta1c");
    expect_same(o.tail_rotor_theta0, want.tail_rotor_theta0, "tail_rotor_theta0");
}

// A doublet of ±0.05 starting at frame `start`, 15 frames each way.
double doublet(int frame, int start) {
    if (frame < start || frame >= start + 30) {
        return 0.0;
    }
    return frame < start + 15 ? 0.05 : -0.05;
}

// In flight: from an 80 kt trim (SAS, PBA and FPS on), a lateral and a
// longitudinal doublet, on GenHel's engine and on Ballin's.
TEST(Readouts, MatchTheComponentsOwnQueriesInFlight) {
    for (const char* name : {"uh60a.yaml", "engine_ballin/uh60a.yaml"}) {
        SCOPED_TRACE(name);
        Model m = Model::from_file(config_path(name));
        TrimCondition tc;
        tc.airspeed = 80.0 * units::kKnot;
        tc.altitude = 1000.0;
        tc.afcs.sas_digital = tc.afcs.sas_analog = tc.afcs.pitch_bias = tc.afcs.fps = true;
        const TrimResult r = m.trim(tc);
        ASSERT_TRUE(r.converged);
        Environment env;
        env.isa_delta_T = tc.isa_delta_T;
        expect_readouts(m, r.controls, env, config_path(name));
        for (int f = 0; f < 60; ++f) {
            Controls c = r.controls;
            c.lat_cyclic += doublet(f, 0);
            c.lon_cyclic += doublet(f, 30);
            (void)m.advance(c, env, 16'666'667);
            if (f % 10 == 9) {
                expect_readouts(m, c, env, config_path(name));
            }
        }
    }
}

// On the ground: weight on wheels both ways, from a drop onto the gear.
TEST(Readouts, MatchTheComponentsOwnQueriesOnTheGround) {
    Model m = Model::from_file(config_path("uh60a.yaml"));
    Controls c;
    c.collective = 0.2;
    c.brake_left = c.brake_right = 1.0;
    Environment env;
    env.ground_height = 0.0;
    BodyState s;
    s.pos_n = Vec3(0.0, 0.0, -2.2);  // first contact at about 0.27 s
    ASSERT_TRUE(m.reset(s));
    for (int f = 0; f < 60; ++f) {
        (void)m.advance(c, env, 16'666'667);
        if (f % 6 == 5) {
            expect_readouts(m, c, env, config_path("uh60a.yaml"));
        }
    }
    EXPECT_TRUE(m.outputs().weight_on_wheels);
}

// A stand-in component of any role that reports `value` in its role's
// fields (gear: weight on wheels when value > 0).
class Reporter : public DynamicComponent {
public:
    Reporter(ReadoutRole role, double value) : role_(role), value_(value) {}
    std::size_t num_states() const override { return 0; }
    void initial_states(std::span<double> /*x*/) const override {}
    ReadoutRole readout_role() const noexcept override { return role_; }
    void derivatives(const SimContext& /*ctx*/, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        if (out.readouts == nullptr) {
            return;
        }
        Readouts& r = *out.readouts;
        switch (role_) {
            case ReadoutRole::main_rotor:
                r.main_rotor_thrust = value_;
                break;
            case ReadoutRole::tail_rotor:
                r.tail_rotor_thrust = value_;
                break;
            case ReadoutRole::landing_gear:
                r.weight_on_wheels = r.weight_on_wheels || value_ > 0.0;
                break;
            default:
                break;
        }
    }

private:
    ReadoutRole role_;
    double value_;
};

// Who reports (D-059): the first component of each role in config order,
// every landing gear, none of a component without a role; a role nobody
// fills stays NaN. A component type Model knows nothing about reports, so a
// new rotor model needs no change to Model.
TEST(Readouts, FirstOfEachRoleAndEveryGearReport) {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(1000.0, 800.0, 2500.0, 2200.0, 120.0);
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<Reporter>(ReadoutRole::none, 7.0));
    comps.push_back(std::make_unique<Reporter>(ReadoutRole::main_rotor, 1.0));
    comps.push_back(std::make_unique<Reporter>(ReadoutRole::landing_gear, 0.0));
    comps.push_back(std::make_unique<Reporter>(ReadoutRole::main_rotor, 2.0));
    comps.push_back(std::make_unique<Reporter>(ReadoutRole::landing_gear, 1.0));
    const Executive ex(cfg, std::move(comps));
    Readouts r;
    r.tail_rotor_thrust = 5.0;  // reset by the evaluation
    Vec6 a;
    ASSERT_TRUE(ex.body_acceleration(Controls{}, Environment{}, a, r));
    EXPECT_EQ(r.main_rotor_thrust, 1.0);
    EXPECT_TRUE(r.weight_on_wheels);  // the second gear's
    EXPECT_TRUE(std::isnan(r.tail_rotor_thrust));
    EXPECT_TRUE(std::isnan(r.coning));  // filled by none of these
}

}  // namespace
}  // namespace eyas
