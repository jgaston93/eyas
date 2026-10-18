#pragma once

#include <fstream>
#include <string>
#include <string_view>

#include "controls_map.hpp"
#include "eyas/aircraft_state.hpp"
#include "station.hpp"

// Records a piloted run (D-044) as an eyas_drive control file: the
// increments from the start controls, a row each time they change, timed
// from the start point, headed by comment lines giving the eyas_drive
// command that replays it. A reposition starts a new segment file (<base>,
// <base>.2.csv, ...), since eyas_drive flies from one trim. eyas_drive has
// no brake input and holds its --afcs switches, so once the brakes or the
// AFCS switches differ from the start's, a comment line marks the rest of
// the segment as not replayable. Optionally also
// writes the state history in eyas_drive's --out format, one row per frame.
//
// Host code with file I/O through buffered streams; it never touches Eyas's
// real-time path.
namespace station {

// Quotes `s` for a POSIX shell when it needs it (single quotes), so a
// replay command can be pasted or split by CMake's separate_arguments.
std::string shell_quote(std::string_view s);

class Recorder {
public:
    // Empty paths: that output is off. `config` and `origin` (the --origin
    // text, empty for the config's own) go into the replay command as given,
    // so eyas_drive rebuilds exactly the same model.
    Recorder(std::string controls_path, std::string state_path, std::string config,
             std::string origin, int rate) noexcept;

    // Starts a segment at `start` (call after Station::start). `state` is the
    // output at the segment's start (written as the --out row at t0).
    void begin(const Station& station, const eyas::AircraftState& state);
    // One frame: `t` is the time since the start point that the controls
    // apply from (before advance()), `d` the increments and `c` the applied
    // controls, brakes included; `after` the outputs after advance().
    void frame(double t, const ControlDeltas& d, const eyas::Controls& c,
               const eyas::AircraftState& after);
    void end();

    bool recording() const noexcept { return controls_.is_open() || state_.is_open(); }
    int segment() const noexcept { return segment_; }

private:
    std::string controls_path_;
    std::string state_path_;
    std::string config_;
    std::string origin_;
    int rate_;
    int segment_ = 0;
    std::ofstream controls_;
    std::ofstream state_;
    ControlDeltas last_;
    bool first_ = true;
    eyas::Controls start_;       // the segment's start controls
    bool brakes_noted_ = false;  // the brakes have left the start's
    bool afcs_noted_ = false;    // the AFCS switches have left the start's
};

}  // namespace station
