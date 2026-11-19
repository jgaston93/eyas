#pragma once

#include <cstddef>
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
// It also holds the controls and environment the outputs were evaluated
// with, so a restore gives back the same outputs (D-043).
class Snapshot {
public:
    Snapshot() = default;

    // As bytes (D-067), for a host that records a session and restores it
    // later: to_bytes() writes byte_size() bytes; from_bytes() reads them into
    // a buffer from make_snapshot() of the same aircraft config. The bytes are
    // valid only for the same build and platform (native byte order; the
    // state's layout is the build's), carry a tag, a version and the state's
    // length, and are the same for the same snapshot, bit for bit (every field
    // written explicitly: no padding). Both are real-time safe (no allocation).
    [[nodiscard]] std::size_t byte_size() const noexcept;
    // False, writing nothing, if `out` isn't byte_size() long.
    bool to_bytes(std::span<std::byte> out) const noexcept;
    // False, changing nothing, if `in` isn't byte_size() long, its tag or
    // version isn't this build's, or its state's length isn't this buffer's.
    // The bytes don't name their aircraft: two configs with states of the same
    // length take each other's, so the host restores only into the config that
    // made them (D-067). It doesn't judge the state: restore() does.
    bool from_bytes(std::span<const std::byte> in) noexcept;

private:
    friend class Model;
    std::vector<double> x_;
    std::int64_t n_steps_ = 0;
    std::int64_t remainder_ns_ = 0;
    Controls controls_;
    Environment env_;
};

class Model {
public:
    // Builds the aircraft from a YAML config (D-012 units). Throws InitError
    // naming the file and key on any error.
    static Model from_file(const std::filesystem::path& path);
    static Model from_string(std::string_view yaml, std::string_view origin = "<string>");
    // As from_file, with `origin` in place of the config's geodetic_origin,
    // which is scenario data, not aircraft data (D-044): the host places the
    // aircraft, e.g. at its airport. The config's block may then be left out;
    // if present it is still checked. Throws InitError if the origin is not
    // finite, |latitude| > 89 deg or |longitude| > 180 deg (D-017).
    static Model from_file(const std::filesystem::path& path, const GeodeticOrigin& origin);

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
    // quaternion is zero; the quaternion is normalized. Until the next
    // advance(), the Tier 2–4 outputs are evaluated with the controls and
    // environment of the last advance() or trim() (D-043): e.g. reset onto
    // the ground after flying with no ground reads no weight on wheels until
    // then.
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
    // holds a non-finite state or a zero quaternion. A restore also brings
    // back the controls and environment current at the snapshot, so the
    // outputs equal those at the snapshot.
    bool snapshot(Snapshot& out) const noexcept;
    bool restore(const Snapshot& in) noexcept;

    // Init-time: every data gap the aircraft was built with (D-012, D-025),
    // one entry per gap, naming the config file's key path:
    //   "components[0].blade.mass"   (a value tagged !placeholder)
    //   "components[0].blade.twist_table: <csv file> (# status: placeholder)"
    // Empty when every value and table is sourced. Eyas does no I/O, so the
    // host decides whether to log, display or refuse a model with gaps.
    const std::vector<std::string>& placeholders() const noexcept;

    // The aircraft's engines, in config order: engine k takes
    // Controls::engines[k] (D-055). Each says which of those controls it
    // honours (D-066), e.g. for a host offering an engine's fuel shutoff only
    // where it does something. Empty for an aircraft without engines.
    std::span<const EngineInfo> engines() const noexcept;

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
