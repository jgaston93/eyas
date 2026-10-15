#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "eyas/aircraft_state.hpp"
#include "eyas/error.hpp"
#include "eyas/trim.hpp"
#include "eyas/types.hpp"

// The Eyas library API (spec §6.10, D-018). One Model is one aircraft. It is
// single-threaded and synchronous: every call runs on the caller's thread.
//
// Real-time calls (no allocation, no exceptions, no I/O; spec §7): advance(),
// outputs(), reset(), snapshot(), restore(). Everything else is init-time,
// including trim().
//
// Only the headers in include/eyas/ are the API (D-020); the implementation is
// behind a pointer, so internal changes don't reach host builds.
namespace eyas {

// A saved simulation state for snapshot()/restore(). Create it with
// Model::make_snapshot(), which sizes it once; reusing it never allocates.
class Snapshot {
public:
    Snapshot() = default;

private:
    friend class Model;
    std::vector<double> x_;
    std::int64_t n_steps_ = 0;
    std::int64_t remainder_ns_ = 0;
};

class Model {
public:
    // Builds the aircraft from a YAML config (D-012 units). Throws InitError
    // naming the file and key on any error.
    static Model from_file(const std::filesystem::path& path);
    static Model from_string(std::string_view yaml, std::string_view origin = "<string>");

    // Movable, not copyable. A moved-from Model may only be destroyed or
    // assigned to.
    Model(Model&&) noexcept;
    Model& operator=(Model&&) noexcept;
    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;
    ~Model();

    // Advances by one host frame of dt_host_ns [ns] (D-006). The aircraft
    // starts at the NED origin, level, at rest, until reset() says otherwise.
    AdvanceStatus advance(const Controls& controls, const Environment& env,
                          std::int64_t dt_host_ns) noexcept;

    // The output record, updated by advance(), reset() and restore().
    const AircraftState& outputs() const noexcept { return outputs_; }

    // Starts again from `body` at t = 0, with every other state at its initial
    // value. Returns false, changing nothing, if `body` is not finite or its
    // quaternion is zero; the quaternion is normalized.
    bool reset(const BodyState& body) noexcept;

    // Init-time (allocates; spec §6.8, D-035): trims for steady straight
    // flight at `condition`, from a cold start or from `start`, a converged
    // trim at a nearby condition (faster and more robust, e.g. in a speed
    // sweep). On convergence the model is reset to t = 0 at the trimmed state,
    // so advance() flies on from trim; otherwise it is unchanged. Check
    // `converged`. Throws InitError on an invalid condition, or a `start`
    // from a different model.
    TrimResult trim(const TrimCondition& condition);
    TrimResult trim(const TrimCondition& condition, const TrimResult& start);

    // Init-time: a snapshot buffer sized for this model, holding its current state.
    Snapshot make_snapshot() const;
    // Saves into / restores from a buffer from make_snapshot(). Return false,
    // changing nothing, if the buffer doesn't fit this model or (restore)
    // holds a non-finite state or a zero quaternion.
    bool snapshot(Snapshot& out) const noexcept;
    bool restore(const Snapshot& in) noexcept;

    // Init-time: every data gap the aircraft was built with (D-012, D-025),
    // one entry per gap, naming the config file's key path:
    //   "components[0].blade.mass"   (a value tagged !placeholder)
    //   "components[0].blade.twist_table: <csv file> (# status: placeholder)"
    // Empty when every value and table is sourced. Eyas does no I/O, so the
    // host decides whether to log, display or refuse a model with gaps.
    const std::vector<std::string>& placeholders() const noexcept;

    // Engineering access to the full state (body states first; spec §5).
    BodyState body_state() const noexcept;
    std::span<const double> state() const noexcept;
    std::int64_t remainder_ns() const noexcept;

    // The implementation; opaque to the host (defined in model.cpp).
    struct Impl;

private:
    explicit Model(std::unique_ptr<Impl> impl);
    TrimResult trim_from(const TrimCondition& condition, const TrimResult* start);
    void update_outputs() noexcept;

    std::unique_ptr<Impl> impl_;
    AircraftState outputs_;
};

}  // namespace eyas
