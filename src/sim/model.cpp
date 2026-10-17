#include "eyas/model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "airframe/landing_gear.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "env/air_data.hpp"
#include "env/atmosphere.hpp"
#include "env/geodesy.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/multiblade.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {

namespace {
constexpr std::size_t kNone = static_cast<std::size_t>(-1);
}  // namespace

struct Model::Impl {
    FlatEarthMap map;  // declared before executive, which is built from its origin
    Executive executive;
    std::vector<std::string> placeholders;  // data gaps, from Config (D-025)

    // The components the Tier 4 outputs read (D-043), as component indices
    // (config order); kNone when the config has none.
    std::size_t main_rotor = kNone;  // the first blade-element rotor
    std::size_t tail_rotor = kNone;  // the first Bailey rotor
    std::vector<std::size_t> gears;  // every landing-gear component

    // The inputs the outputs are evaluated with: those of the last advance()
    // that ran, or the trimmed ones (D-043).
    Controls controls;
    Environment env;
};

namespace {

// `origin`, when given, replaces the config's geodetic_origin (D-044). The
// config's block is then optional, but still checked when present, so a
// file is valid or invalid the same way whichever call loads it.
std::unique_ptr<Model::Impl> build(const Config& config, const GeodeticOrigin* origin) {
    config.allow_keys({"simulation", "geodetic_origin", "airframe", "components", "force_models"});
    if (origin != nullptr && config.contains("geodetic_origin")) {
        (void)read_origin(config);
    }
    FlatEarthMap map =
        origin == nullptr
            ? read_origin(config)
            : FlatEarthMap(GeodeticPosition{origin->latitude, origin->longitude, origin->altitude});
    const double origin_altitude = map.origin().altitude;
    Executive executive = build_executive(config, origin_altitude);
    // Read after the build, so that placeholder data files noted by the
    // component readers are included.
    auto impl = std::make_unique<Model::Impl>(
        Model::Impl{map, std::move(executive), config.placeholders(), kNone, kNone, {}, {}, {}});
    const Executive& ex = impl->executive;
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        const DynamicComponent* c = &ex.component(i);
        if (impl->main_rotor == kNone && dynamic_cast<const BladeElementRotor*>(c) != nullptr) {
            impl->main_rotor = i;
        } else if (impl->tail_rotor == kNone && dynamic_cast<const BaileyRotor*>(c) != nullptr) {
            impl->tail_rotor = i;
        } else if (dynamic_cast<const LandingGear*>(c) != nullptr) {
            impl->gears.push_back(i);
        }
    }
    return impl;
}

// Angle in [0, 2π).
double wrap_2pi(double a) noexcept {
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    double w = std::fmod(a, kTwoPi);
    if (w < 0.0) {
        w += kTwoPi;
    }
    return w >= kTwoPi ? 0.0 : w;
}

}  // namespace

Model::Model(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {
    update_outputs();
}

Model::Model(Model&&) noexcept = default;
Model& Model::operator=(Model&&) noexcept = default;
Model::~Model() = default;

Model Model::from_file(const std::filesystem::path& path) {
    return Model(build(Config::from_file(path), nullptr));
}

Model Model::from_file(const std::filesystem::path& path, const GeodeticOrigin& origin) {
    return Model(build(Config::from_file(path), &origin));
}

Model Model::from_string(std::string_view yaml, std::string_view origin) {
    return Model(build(Config::from_string(yaml, origin), nullptr));
}

const std::vector<std::string>& Model::placeholders() const noexcept {
    return impl_->placeholders;
}

AdvanceStatus Model::advance(const Controls& controls, const Environment& env,
                             std::int64_t dt_host_ns) noexcept {
    const AdvanceStatus status = impl_->executive.advance(controls, env, dt_host_ns);
    if (!status.rejected_dt && !status.rejected_env) {
        impl_->controls = controls;
        impl_->env = env;
    }
    outputs_.last_advance = status;
    update_outputs();
    return status;
}

bool Model::reset(const BodyState& body) noexcept {
    if (!impl_->executive.reset(body)) {
        return false;
    }
    outputs_.last_advance = AdvanceStatus{};
    update_outputs();
    return true;
}

TrimResult Model::trim(const TrimCondition& condition) {
    return trim_from(condition, nullptr);
}

TrimResult Model::trim(const TrimCondition& condition, const TrimResult& start) {
    return trim_from(condition, &start);
}

TrimResult Model::trim_from(const TrimCondition& condition, const TrimResult* start) {
    std::vector<double> x;
    TrimResult r = solve_trim(impl_->executive, condition, start, x);
    if (!r.converged) {
        return r;
    }
    // solve_trim checks the state is finite, so restore() accepts it; if it
    // ever doesn't, the model is unchanged and the trim must not claim success.
    if (!impl_->executive.restore(x, 0, 0)) {
        r.converged = false;
        return r;
    }
    impl_->controls = r.controls;
    impl_->env = Environment{condition.isa_delta_T, condition.wind_n};  // no ground, as trimmed
    outputs_.last_advance = AdvanceStatus{};
    update_outputs();
    return r;
}

Snapshot Model::make_snapshot() const {
    // Filled with the current state, so restoring a buffer that was never
    // passed to snapshot() is still valid.
    Snapshot s;
    s.x_.assign(impl_->executive.num_states(), 0.0);
    (void)snapshot(s);
    return s;
}

bool Model::snapshot(Snapshot& out) const noexcept {
    const Executive& ex = impl_->executive;
    const std::span<const double> x = ex.state();
    if (out.x_.size() != x.size()) {
        return false;
    }
    std::copy(x.begin(), x.end(), out.x_.begin());
    out.n_steps_ = ex.n_steps();
    out.remainder_ns_ = ex.remainder_ns();
    out.controls_ = impl_->controls;
    out.env_ = impl_->env;
    return true;
}

bool Model::restore(const Snapshot& in) noexcept {
    if (!impl_->executive.restore(in.x_, in.n_steps_, in.remainder_ns_)) {
        return false;
    }
    // The outputs' inputs too, so they come back as they were (D-043).
    impl_->controls = in.controls_;
    impl_->env = in.env_;
    outputs_.last_advance = AdvanceStatus{};
    update_outputs();
    return true;
}

BodyState Model::body_state() const noexcept {
    return impl_->executive.body_state();
}

std::span<const double> Model::state() const noexcept {
    return impl_->executive.state();
}

std::int64_t Model::remainder_ns() const noexcept {
    return impl_->executive.remainder_ns();
}

void Model::update_outputs() noexcept {
    const Executive& ex = impl_->executive;
    const BodyState b = ex.body_state();
    AircraftState& o = outputs_;
    o.sim_time_ns = ex.sim_time_ns();
    o.sim_time = ex.sim_time();
    const GeodeticPosition g = impl_->map.to_geodetic(b.pos_n);
    o.latitude = g.latitude;
    o.longitude = g.longitude;
    o.altitude = g.altitude;
    o.q_nb = b.q_nb;
    const EulerAngles e = euler_angles(b.q_nb);
    o.roll = e.roll;
    o.pitch = e.pitch;
    o.heading = e.heading;
    o.quat_norm_error = ex.quat_norm_error();

    // Tiers 2–4 (D-043): one evaluation at the final state. It leaves the
    // coupling store consistent for the component queries below.
    const Controls& c = impl_->controls;
    const Environment& env = impl_->env;
    Vec6 a;
    (void)ex.body_acceleration(c, env, a);
    const double t = ex.sim_time();
    const std::span<const double> x = ex.state();
    const SimContext ctx = ex.context(t, x, c, env);
    const Mat3 C_nb = ctx.body.q_nb.toRotationMatrix();
    const double g0 = units::kStandardGravity;

    // Tier 2
    o.pos_n = b.pos_n;
    o.vel_b = b.vel_b;
    o.omega_b = b.omega_b;
    o.vel_n = C_nb * b.vel_b;
    o.accel_b = a.head<3>() + b.omega_b.cross(b.vel_b);
    o.specific_force_b = o.accel_b - C_nb.transpose() * Vec3(0.0, 0.0, g0);
    o.load_factor = -o.specific_force_b.z() / g0;
    o.ground_speed = std::hypot(o.vel_n.x(), o.vel_n.y());
    o.track = o.ground_speed > 0.0 ? wrap_2pi(std::atan2(o.vel_n.y(), o.vel_n.x()))
                                   : std::numeric_limits<double>::quiet_NaN();

    // Tier 3 (Gracey, NASA RP-1046, Ch. III)
    const AirState& air = ctx.air;
    const Vec3 v_air_b = b.vel_b - C_nb.transpose() * env.wind_n;
    const double V = v_air_b.norm();
    o.true_airspeed = V;
    o.equivalent_airspeed = equivalent_airspeed(air.density, V);
    o.calibrated_airspeed = calibrated_airspeed(impact_pressure(air.pressure, air.density, V));
    o.indicated_airspeed = o.calibrated_airspeed;
    o.angle_of_attack = std::atan2(v_air_b.z(), v_air_b.x());
    o.sideslip = V > 0.0 ? std::asin(std::clamp(v_air_b.y() / V, -1.0, 1.0)) : 0.0;
    o.pressure_altitude = pressure_altitude(air.pressure);
    o.vertical_speed = -o.vel_n.z();
    o.outside_air_temperature = air.temperature;

    // Tier 4
    const auto states = [&](std::size_t i) {
        return x.subspan(ex.component_offset(i), ex.component_size(i));
    };
    if (impl_->main_rotor != kNone) {
        const auto& rotor = static_cast<const BladeElementRotor&>(ex.component(impl_->main_rotor));
        const std::span<const double> xr = states(impl_->main_rotor);
        const RotorLoads loads = rotor.evaluate(ctx, xr);
        const double psi = rotor.azimuth(t);
        o.rotor_speed_percent = 100.0;  // Ω fixed until M9
        o.main_rotor_thrust = loads.thrust;
        o.main_rotor_torque = loads.torque;
        o.main_rotor_power = loads.power;
        o.main_rotor_azimuth = psi;
        MultibladeFlap m;
        if (rotor.config().flap) {
            m = multiblade_flap(xr, psi, rotor.config().num_blades);
        }
        o.coning = m.beta0;
        o.flap_1c = m.beta1c;
        o.flap_1s = m.beta1s;
    }
    if (impl_->tail_rotor != kNone) {
        const auto& rotor = static_cast<const BaileyRotor&>(ex.component(impl_->tail_rotor));
        o.tail_rotor_thrust = rotor.evaluate(ctx, states(impl_->tail_rotor)).thrust;
        o.tail_rotor_azimuth = wrap_2pi(rotor.config().rotor_speed * t);
    }
    bool wow = false;
    for (const std::size_t i : impl_->gears) {
        const auto& gear = static_cast<const LandingGear&>(ex.component(i));
        for (std::size_t k = 0; k < gear.config().gears.size(); ++k) {
            wow = wow || gear.gear_state(ctx, states(i), k).normal_load > 0.0;
        }
    }
    o.weight_on_wheels = wow;
}

}  // namespace eyas
