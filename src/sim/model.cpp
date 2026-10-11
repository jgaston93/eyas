#include "eyas/model.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "core/attitude.hpp"
#include "env/geodesy.hpp"
#include "rotor/ideal_thrust.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {

struct Model::Impl {
    FlatEarthMap map;  // declared before executive, which is built from its origin
    Executive executive;
};

namespace {

using units::Dimension;

constexpr std::int64_t kNsPerSecond = 1'000'000'000;

// Runs `f`, re-raising its InitError against `key` of `node`, so that errors
// from validation inside a constructor still name the file and key path.
template <class F>
auto in_context(const Config& node, std::string_view key, F&& f) -> decltype(f()) {
    try {
        return std::forward<F>(f)();
    } catch (const InitError& e) {
        node.fail(key, e.what());
    }
}

FlatEarthMap read_origin(const Config& config) {
    const Config o = config.child("geodetic_origin");
    o.allow_keys({"latitude", "longitude", "altitude"});
    const GeodeticPosition origin{o.quantity("latitude", Dimension::angle),
                                  o.quantity("longitude", Dimension::angle),
                                  o.quantity("altitude", Dimension::length)};
    return in_context(config, "geodetic_origin", [&] { return FlatEarthMap(origin); });
}

// Physics step from `simulation.physics_rate` [Hz]: the rate must be a whole
// number that divides 1 s into whole nanoseconds (spec §6.10).
std::int64_t read_step_ns(const Config& sim) {
    const double rate = sim.quantity("physics_rate", Dimension::frequency);
    if (!(rate >= 1.0) || rate > 1e9 || rate != std::floor(rate)) {
        sim.fail("physics_rate", "must be a whole number of Hz, got " + std::to_string(rate));
    }
    const auto r = static_cast<std::int64_t>(rate);
    if (kNsPerSecond % r != 0) {
        sim.fail("physics_rate",
                 std::to_string(r) + " Hz does not divide 1 s into whole nanoseconds");
    }
    return kNsPerSecond / r;
}

std::vector<std::unique_ptr<ForceModel>> read_force_models(const Config& config) {
    std::vector<std::unique_ptr<ForceModel>> forces;
    if (!config.contains("force_models")) {
        return forces;
    }
    const Config list = config.child("force_models");
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Config f = list[i];
        const std::string type = f.string("type");
        if (type == "ideal_thrust") {
            f.allow_keys({"type", "max_thrust"});
            const double max_thrust = f.quantity("max_thrust", Dimension::force);
            forces.push_back(in_context(f, "max_thrust",
                                        [&] { return std::make_unique<IdealThrust>(max_thrust); }));
        } else {
            f.fail("type", "unknown force model '" + type + "'");
        }
    }
    return forces;
}

Executive build_executive(const Config& config, double origin_altitude) {
    ExecutiveConfig ec;
    const Config af = config.child("airframe");
    af.allow_keys({"mass", "inertia"});
    const Config in = af.child("inertia");
    in.allow_keys({"ixx", "iyy", "izz", "ixz"});
    ec.mass = make_mass_properties(
        af.quantity("mass", Dimension::mass), in.quantity("ixx", Dimension::inertia),
        in.quantity("iyy", Dimension::inertia), in.quantity("izz", Dimension::inertia),
        in.quantity("ixz", Dimension::inertia));
    ec.origin_altitude = origin_altitude;
    if (config.contains("simulation")) {
        const Config sim = config.child("simulation");
        sim.allow_keys({"physics_rate", "max_substeps"});
        if (sim.contains("physics_rate")) {
            ec.step_ns = read_step_ns(sim);
        }
        if (sim.contains("max_substeps")) {
            ec.max_substeps = sim.integer("max_substeps");
            if (ec.max_substeps <= 0) {
                sim.fail("max_substeps", "must be positive");
            }
        }
    }
    auto forces = read_force_models(config);
    // Step, substeps and origin are checked above, so what the executive can
    // still reject is the airframe's mass properties.
    return in_context(config, "airframe", [&] { return Executive(ec, {}, std::move(forces)); });
}

std::unique_ptr<Model::Impl> build(const Config& config) {
    config.allow_keys({"simulation", "geodetic_origin", "airframe", "force_models"});
    FlatEarthMap map = read_origin(config);
    const double origin_altitude = map.origin().altitude;
    return std::make_unique<Model::Impl>(
        Model::Impl{map, build_executive(config, origin_altitude)});
}

}  // namespace

Model::Model(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {
    update_outputs();
}

Model::Model(Model&&) noexcept = default;
Model& Model::operator=(Model&&) noexcept = default;
Model::~Model() = default;

Model Model::from_file(const std::filesystem::path& path) {
    return Model(build(Config::from_file(path)));
}

Model Model::from_string(std::string_view yaml, std::string_view origin) {
    return Model(build(Config::from_string(yaml, origin)));
}

AdvanceStatus Model::advance(const Controls& controls, const Environment& env,
                             std::int64_t dt_host_ns) noexcept {
    const AdvanceStatus status = impl_->executive.advance(controls, env, dt_host_ns);
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
    return true;
}

bool Model::restore(const Snapshot& in) noexcept {
    if (!impl_->executive.restore(in.x_, in.n_steps_, in.remainder_ns_)) {
        return false;
    }
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
}

}  // namespace eyas
