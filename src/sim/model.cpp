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

#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "env/air_data.hpp"
#include "env/atmosphere.hpp"
#include "env/geodesy.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/error.hpp"
#include "util/math.hpp"
#include "util/units.hpp"

namespace eyas {

struct Model::Impl {
    FlatEarthMap map;  // declared before executive, which is built from its origin
    Executive executive;
    std::vector<std::string> placeholders;  // data gaps, from Config (D-025)

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
    config.allow_keys(
        {"simulation", "geodetic_origin", "airframe", "shafts", "components", "force_models"});
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
    return std::make_unique<Model::Impl>(
        Model::Impl{map, std::move(executive), config.placeholders(), {}, {}});
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
    impl_->env = trim_environment(condition);  // as trimmed, ground included (D-050)
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

    // Tiers 2–4 (D-043): one evaluation at the final state, in which the
    // components report their readouts (D-059).
    const Controls& c = impl_->controls;
    const Environment& env = impl_->env;
    Vec6 a;
    Readouts r;
    (void)ex.body_acceleration(c, env, a, r);
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

    // Tier 4 and the flight controls (D-045), as the components reported them
    o.rotor_speed_percent = r.rotor_speed_percent;
    o.main_rotor_thrust = r.main_rotor_thrust;
    o.main_rotor_torque = r.main_rotor_torque;
    o.main_rotor_power = r.main_rotor_power;
    o.coning = r.coning;
    o.flap_1c = r.flap_1c;
    o.flap_1s = r.flap_1s;
    o.main_rotor_azimuth = r.main_rotor_azimuth;
    o.tail_rotor_thrust = r.tail_rotor_thrust;
    o.tail_rotor_azimuth = r.tail_rotor_azimuth;
    o.weight_on_wheels = r.weight_on_wheels;
    o.tail_rotor_torque = r.tail_rotor_torque;
    o.tail_rotor_power = r.tail_rotor_power;
    o.engines = r.engines;
    o.stabilator_incidence = r.stabilator_incidence;
    o.main_rotor_theta0 = r.main_rotor_theta0;
    o.main_rotor_theta1s = r.main_rotor_theta1s;
    o.main_rotor_theta1c = r.main_rotor_theta1c;
    o.tail_rotor_theta0 = r.tail_rotor_theta0;
}

}  // namespace eyas
