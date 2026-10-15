#include "eyas/model.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "env/geodesy.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {

struct Model::Impl {
    FlatEarthMap map;  // declared before executive, which is built from its origin
    Executive executive;
    std::vector<std::string> placeholders;  // data gaps, from Config (D-025)
};

namespace {

std::unique_ptr<Model::Impl> build(const Config& config) {
    config.allow_keys({"simulation", "geodetic_origin", "airframe", "components", "force_models"});
    FlatEarthMap map = read_origin(config);
    const double origin_altitude = map.origin().altitude;
    Executive executive = build_executive(config, origin_altitude);
    // Read after the build, so that placeholder data files noted by the
    // component readers are included.
    return std::make_unique<Model::Impl>(
        Model::Impl{map, std::move(executive), config.placeholders()});
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

const std::vector<std::string>& Model::placeholders() const noexcept {
    return impl_->placeholders;
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

TrimResult Model::trim(const TrimCondition& condition) {
    std::vector<double> x;
    TrimResult r = solve_trim(impl_->executive, condition, nullptr, x);
    if (r.converged && impl_->executive.restore(x, 0, 0)) {
        outputs_.last_advance = AdvanceStatus{};
        update_outputs();
    }
    return r;
}

TrimResult Model::trim(const TrimCondition& condition, const TrimResult& start) {
    std::vector<double> x;
    TrimResult r = solve_trim(impl_->executive, condition, &start, x);
    if (r.converged && impl_->executive.restore(x, 0, 0)) {
        outputs_.last_advance = AdvanceStatus{};
        update_outputs();
    }
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
