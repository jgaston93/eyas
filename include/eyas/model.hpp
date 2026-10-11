#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string_view>
#include <vector>

#include "env/geodesy.hpp"
#include "eyas/aircraft_state.hpp"
#include "sim/executive.hpp"

// The Eyas library API (spec §6.10, D-018). One Model is one aircraft. It is
// single-threaded and synchronous: every call runs on the caller's thread.
//
// Real-time calls (no allocation, no exceptions, no I/O; spec §7): advance(),
// outputs(), reset(), snapshot(), restore(). Everything else is init-time.
// trim() arrives with M7.
namespace eyas {

class Config;

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
    explicit Model(const Config& config);

    // Advances by one host frame of dt_host_ns [ns] (D-006). The aircraft
    // starts at the NED origin, level, at rest, until reset() says otherwise.
    AdvanceStatus advance(const Controls& controls, const Environment& env,
                          std::int64_t dt_host_ns) noexcept;

    // The output record, updated by advance(), reset() and restore().
    const AircraftState& outputs() const noexcept { return outputs_; }

    // Starts again from `body` at t = 0 (see Executive::reset). Returns false,
    // changing nothing, if the state is invalid.
    bool reset(const BodyState& body) noexcept;

    // Init-time: a snapshot buffer sized for this model.
    Snapshot make_snapshot() const;
    // Saves into / restores from a buffer from make_snapshot(). Return false,
    // changing nothing, if the buffer doesn't fit this model.
    bool snapshot(Snapshot& out) const noexcept;
    bool restore(const Snapshot& in) noexcept;

    // Engineering access to the full state (body states first; spec §5).
    BodyState body_state() const noexcept { return executive_.body_state(); }
    std::span<const double> state() const noexcept { return executive_.state(); }
    std::int64_t remainder_ns() const noexcept { return executive_.remainder_ns(); }

private:
    void update_outputs() noexcept;

    FlatEarthMap map_;  // declared before executive_, which is built from its origin
    Executive executive_;
    AircraftState outputs_;
};

}  // namespace eyas
