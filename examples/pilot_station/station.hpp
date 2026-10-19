#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "eyas/model.hpp"

// The pilot station's simulation side (D-044): the model, its start points
// and the frame step, with no window, device or socket, so the tests can
// drive it. The station is host code: it uses only include/eyas.
namespace station {

struct StationOptions {
    std::string config;                          // aircraft YAML
    std::optional<eyas::GeodeticOrigin> origin;  // replaces the config's (D-044)
    // [m] above the ground: the airborne starts' height. Must be positive and
    // leave the gear clear of the ground (checked at init).
    double hover_height = 30.48;
    int rate = 60;  // [Hz] host frame rate
    // The flight control system's switches at every start: the SAS and PBA
    // on, as the aircraft flies (M8c2, D-045). The pilot toggles them in flight.
    eyas::AfcsSwitches afcs{true, true, true};
};

// A place to fly from, prepared at init so a reposition is a restore()
// (a real-time call; D-043's reset(body, controls, env) isn't needed).
struct StartPoint {
    std::string name;         // "pad", "hover", "60", "120"
    eyas::Snapshot snap;      // the state to restore
    eyas::Controls controls;  // the controls that hold it: the trim, or the parked controls
    // Airborne trims replay in eyas_drive (--trim-speed, --altitude); the
    // pad start (a settled drop onto the gear) doesn't.
    bool replayable = false;
    double trim_speed_kt = 0.0;  // [kt] true airspeed
    double altitude = 0.0;       // [m] MSL of the trim
};

class Station {
public:
    // Init time: builds the model and prepares every start point (three
    // cold trims and a settled drop onto the pad). Throws eyas::InitError or
    // std::runtime_error (a trim that doesn't converge).
    explicit Station(const StationOptions& options);

    // Restores start point `index` and zeroes the frame statistics' fault
    // flags. Real time. Returns false if the restore failed.
    bool start(std::size_t index) noexcept;

    // One host frame with `controls`: advance() by the nominal frame period.
    // Real time. Times the call. After a crash it does nothing until the
    // next start().
    eyas::AdvanceStatus frame(const eyas::Controls& controls) noexcept;

    // The station's crash rule (D-044; SPEC_REVIEW #34): a frame failed, a
    // wheel is on the ground at more than 30° of bank or pitch, |n_z| passed
    // 10 g, or the CG went below the ground. These are display heuristics
    // for a desktop station, not aircraft limits: past them the GenHel gear
    // is outside its validity and the state can diverge before it overflows.
    bool crashed() const noexcept { return crashed_; }

    const eyas::AircraftState& outputs() const noexcept { return model_.outputs(); }
    // [s] sim time since the current start point: what scripts and
    // recordings are timed by. The pad's snapshot carries 40 s of settling;
    // the trims start at 0, so for them it equals the sim time exactly.
    double flight_time() const noexcept { return model_.outputs().sim_time - start_time_; }
    const eyas::Environment& environment() const noexcept { return env_; }
    const std::vector<StartPoint>& starts() const noexcept { return starts_; }
    std::size_t current_start() const noexcept { return current_; }
    std::int64_t frame_ns() const noexcept { return frame_ns_; }
    double ground_height() const noexcept { return env_.ground_height; }  // [m] MSL
    const eyas::GeodeticOrigin& origin() const noexcept { return origin_; }
    const std::vector<std::string>& placeholders() const noexcept { return model_.placeholders(); }

    // Since the last start(): the worst and the last advance() wall time
    // [ns], the frames run and whether any frame failed (non_finite,
    // capped or rejected; D-014).
    std::int64_t worst_advance_ns() const noexcept { return worst_ns_; }
    std::int64_t last_advance_ns() const noexcept { return last_ns_; }
    std::int64_t frames() const noexcept { return frames_; }
    bool faulted() const noexcept { return faulted_; }

private:
    eyas::Model model_;
    eyas::GeodeticOrigin origin_;
    eyas::Environment env_;
    std::int64_t frame_ns_;
    std::vector<StartPoint> starts_;
    std::size_t current_ = 0;
    double start_time_ = 0.0;  // [s] sim time at the current start
    std::int64_t worst_ns_ = 0;
    std::int64_t last_ns_ = 0;
    std::int64_t frames_ = 0;
    bool faulted_ = false;
    bool crashed_ = false;
};

}  // namespace station
