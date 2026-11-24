#include "eyas/model.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
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
    std::vector<EngineInfo> engines;        // in config order (D-066)

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
    // The engines in config order: each component's after the ones before
    // it, as read_components numbers them (D-055).
    std::vector<EngineInfo> engines;
    for (std::size_t i = 0; i < executive.num_components(); ++i) {
        const DynamicComponent& c = executive.component(i);
        engines.insert(engines.end(), static_cast<std::size_t>(c.engine_count()), c.engine_info());
    }
    return std::make_unique<Model::Impl>(
        Model::Impl{map, std::move(executive), config.placeholders(), std::move(engines), {}, {}});
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

std::span<const EngineInfo> Model::engines() const noexcept {
    return impl_->engines;
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

namespace {

// D-067: a snapshot's bytes. Native byte order; each field written alone.
constexpr std::uint32_t kSnapshotTag = 0x4E535945U;  // "EYSN" read little-endian
constexpr std::uint32_t kSnapshotVersion = 2U;       // 2: Controls::trim_on_controls (D-068)
// The controls' and environment's sizes as written: 6 doubles, 6 switches,
// per engine a double and a switch, a double; 5 doubles.
constexpr std::size_t kControlsBytes = (std::size_t{6} * 8) + 7 + (kMaxEngines * (8 + 1)) + 8;
constexpr std::size_t kEnvironmentBytes = std::size_t{5} * 8;
constexpr std::size_t kHeaderBytes = 4 + 4 + 8;

class ByteWriter {
public:
    explicit ByteWriter(std::span<std::byte> out) noexcept : out_(out) {}
    template <typename T>
    void put(const T& v) noexcept {
        std::memcpy(out_.data() + at_, &v, sizeof(T));
        at_ += sizeof(T);
    }
    void flag(bool b) noexcept { put(static_cast<std::uint8_t>(b ? 1 : 0)); }

private:
    std::span<std::byte> out_;
    std::size_t at_ = 0;
};

class ByteReader {
public:
    explicit ByteReader(std::span<const std::byte> in) noexcept : in_(in) {}
    template <typename T>
    T get() noexcept {
        T v{};
        std::memcpy(&v, in_.data() + at_, sizeof(T));
        at_ += sizeof(T);
        return v;
    }
    // A switch: 0 or 1; anything else marks the bytes bad.
    bool flag() noexcept {
        const auto b = get<std::uint8_t>();
        ok_ = ok_ && b <= 1;
        return b == 1;
    }
    [[nodiscard]] bool ok() const noexcept { return ok_; }

private:
    std::span<const std::byte> in_;
    std::size_t at_ = 0;
    bool ok_ = true;
};

}  // namespace

std::size_t Snapshot::byte_size() const noexcept {
    return kHeaderBytes + (x_.size() * sizeof(double)) + (2 * sizeof(std::int64_t)) +
           kControlsBytes + kEnvironmentBytes;
}

bool Snapshot::to_bytes(std::span<std::byte> out) const noexcept {
    if (out.size() != byte_size()) {
        return false;
    }
    ByteWriter w(out);
    w.put(kSnapshotTag);
    w.put(kSnapshotVersion);
    w.put(static_cast<std::uint64_t>(x_.size()));
    for (const double v : x_) {
        w.put(v);
    }
    w.put(n_steps_);
    w.put(remainder_ns_);
    const Controls& c = controls_;
    for (const double v :
         {c.collective, c.lat_cyclic, c.lon_cyclic, c.pedal, c.brake_left, c.brake_right}) {
        w.put(v);
    }
    for (const bool b : {c.afcs.sas_digital, c.afcs.sas_analog, c.afcs.pitch_bias, c.afcs.fps,
                         c.afcs.fps_turn, c.afcs.trim_release, c.trim_on_controls}) {
        w.flag(b);
    }
    for (const EngineControls& e : c.engines) {
        w.put(e.power_lever);
        w.flag(e.fuel_on);
    }
    w.put(c.rotor_speed_reference);
    w.put(env_.isa_delta_T);
    w.put(env_.wind_n.x());
    w.put(env_.wind_n.y());
    w.put(env_.wind_n.z());
    w.put(env_.ground_height);
    return true;
}

bool Snapshot::from_bytes(std::span<const std::byte> in) noexcept {
    if (in.size() != byte_size()) {
        return false;
    }
    ByteReader r(in);
    if (r.get<std::uint32_t>() != kSnapshotTag || r.get<std::uint32_t>() != kSnapshotVersion ||
        r.get<std::uint64_t>() != x_.size()) {
        return false;
    }
    // Read whole before anything changes: bad bytes change nothing.
    Controls c;
    Environment env;
    std::int64_t n_steps = 0;
    std::int64_t remainder_ns = 0;
    {
        ByteReader probe = r;
        for (std::size_t k = 0; k < x_.size(); ++k) {
            (void)probe.get<double>();
        }
        n_steps = probe.get<std::int64_t>();
        remainder_ns = probe.get<std::int64_t>();
        for (double* v : {&c.collective, &c.lat_cyclic, &c.lon_cyclic, &c.pedal, &c.brake_left,
                          &c.brake_right}) {
            *v = probe.get<double>();
        }
        for (bool* b : {&c.afcs.sas_digital, &c.afcs.sas_analog, &c.afcs.pitch_bias, &c.afcs.fps,
                        &c.afcs.fps_turn, &c.afcs.trim_release, &c.trim_on_controls}) {
            *b = probe.flag();
        }
        for (EngineControls& e : c.engines) {
            e.power_lever = probe.get<double>();
            e.fuel_on = probe.flag();
        }
        c.rotor_speed_reference = probe.get<double>();
        env.isa_delta_T = probe.get<double>();
        const auto wx = probe.get<double>();
        const auto wy = probe.get<double>();
        const auto wz = probe.get<double>();
        env.wind_n = Vec3(wx, wy, wz);
        env.ground_height = probe.get<double>();
        if (!probe.ok()) {
            return false;
        }
    }
    for (double& v : x_) {
        v = r.get<double>();
    }
    n_steps_ = n_steps;
    remainder_ns_ = remainder_ns;
    controls_ = c;
    env_ = env;
    return true;
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
    o.fps_trim_lon_cyclic = r.fps_trim_lon_cyclic;
    o.fps_trim_lat_cyclic = r.fps_trim_lat_cyclic;
    o.fps_trim_pedal = r.fps_trim_pedal;
}

}  // namespace eyas
