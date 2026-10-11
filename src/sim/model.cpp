#include "eyas/model.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "core/attitude.hpp"
#include "rotor/ideal_thrust.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

constexpr std::int64_t kNsPerSecond = 1'000'000'000;

GeodeticPosition read_origin(const Config& config) {
    const Config o = config.child("geodetic_origin");
    return {o.quantity("latitude"), o.quantity("longitude"), o.quantity("altitude")};
}

// Physics step from `simulation.physics_rate` [Hz]: the rate must be a whole
// number that divides 1 s into whole nanoseconds (spec §6.10).
std::int64_t read_step_ns(const Config& sim) {
    const double rate = sim.quantity("physics_rate");
    if (!(rate >= 1.0) || rate > 1e9 || rate != std::floor(rate)) {
        throw InitError("simulation.physics_rate: must be a whole number of Hz, got " +
                        std::to_string(rate));
    }
    const auto r = static_cast<std::int64_t>(rate);
    if (kNsPerSecond % r != 0) {
        throw InitError("simulation.physics_rate: " + std::to_string(r) +
                        " Hz does not divide 1 s into whole nanoseconds");
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
            forces.push_back(std::make_unique<IdealThrust>(f.quantity("max_thrust")));
        } else {
            throw InitError("force_models[" + std::to_string(i) + "].type: unknown force model '" +
                            type + "'");
        }
    }
    return forces;
}

Executive build_executive(const Config& config, double origin_altitude) {
    ExecutiveConfig ec;
    const Config af = config.child("airframe");
    const Config in = af.child("inertia");
    ec.mass = make_mass_properties(af.quantity("mass"), in.quantity("ixx"), in.quantity("iyy"),
                                   in.quantity("izz"), in.quantity("ixz"));
    ec.origin_altitude = origin_altitude;
    if (config.contains("simulation")) {
        const Config sim = config.child("simulation");
        if (sim.contains("physics_rate")) {
            ec.step_ns = read_step_ns(sim);
        }
        if (sim.contains("max_substeps")) {
            ec.max_substeps = sim.integer("max_substeps");
        }
    }
    return Executive(ec, {}, read_force_models(config));
}

}  // namespace

Model Model::from_file(const std::filesystem::path& path) {
    return Model(Config::from_file(path));
}

Model Model::from_string(std::string_view yaml, std::string_view origin) {
    return Model(Config::from_string(yaml, origin));
}

Model::Model(const Config& config)
    : map_(read_origin(config)), executive_(build_executive(config, map_.origin().altitude)) {
    update_outputs();
}

AdvanceStatus Model::advance(const Controls& controls, const Environment& env,
                             std::int64_t dt_host_ns) noexcept {
    const AdvanceStatus status = executive_.advance(controls, env, dt_host_ns);
    outputs_.last_advance = status;
    update_outputs();
    return status;
}

bool Model::reset(const BodyState& body) noexcept {
    if (!executive_.reset(body)) {
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
    s.x_.assign(executive_.num_states(), 0.0);
    (void)snapshot(s);
    return s;
}

bool Model::snapshot(Snapshot& out) const noexcept {
    const std::span<const double> x = executive_.state();
    if (out.x_.size() != x.size()) {
        return false;
    }
    std::copy(x.begin(), x.end(), out.x_.begin());
    out.n_steps_ = executive_.n_steps();
    out.remainder_ns_ = executive_.remainder_ns();
    return true;
}

bool Model::restore(const Snapshot& in) noexcept {
    if (!executive_.restore(in.x_, in.n_steps_, in.remainder_ns_)) {
        return false;
    }
    outputs_.last_advance = AdvanceStatus{};
    update_outputs();
    return true;
}

void Model::update_outputs() noexcept {
    const BodyState b = executive_.body_state();
    AircraftState& o = outputs_;
    o.sim_time_ns = executive_.sim_time_ns();
    o.sim_time = executive_.sim_time();
    const GeodeticPosition g = map_.to_geodetic(b.pos_n);
    o.latitude = g.latitude;
    o.longitude = g.longitude;
    o.altitude = g.altitude;
    o.q_nb = b.q_nb;
    const EulerAngles e = euler_angles(b.q_nb);
    o.roll = e.roll;
    o.pitch = e.pitch;
    o.heading = e.heading;
    o.quat_norm_error = executive_.quat_norm_error();
}

}  // namespace eyas
