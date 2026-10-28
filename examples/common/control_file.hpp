#pragma once

#include <array>
#include <cstddef>
#include <iosfwd>
#include <string>
#include <vector>

#include "eyas/types.hpp"

// eyas_drive's control files, shared by eyas_drive and the pilot station
// (which plays them with --input script and writes them with --record):
//
//   time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal
//   0.0,0,0,0,0
//   1.0,0,0,0.03,0
//
// Increments from the start controls, each held from its row's time until
// the next row (zero-order hold). Lines starting with '#' are comments. An
// optional column, afcs, gives the flight control system's switches from
// that row on ('+'-separated names, common/afcs.hpp; M8c3); without it the
// start's switches hold throughout. After it, optional engine columns give
// the engine controls from that row on, as values, not increments (D-060):
// each engine's power lever, the engine-speed trim (rotor_speed_reference)
// and each engine's fuel shutoff (1 open, 0 closed), kMaxEngines of each.
// The pilot station records all of them.
namespace examples {

struct ControlRow {
    double t = 0.0;  // [s] sim time the row applies from
    double d_collective = 0.0;
    double d_lat = 0.0;
    double d_lon = 0.0;
    double d_pedal = 0.0;
    bool has_afcs = false;     // the file has the afcs column
    eyas::AfcsSwitches afcs;   // the switches from this row on, if it has
    bool has_engines = false;  // the file has the engine columns (and afcs)
    // The engine controls from this row on, if it has (D-060).
    std::array<eyas::EngineControls, eyas::kMaxEngines> engines{};
    double rotor_speed_reference = 1.0;
};

inline constexpr const char* kControlHeader =
    "time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal";
inline constexpr const char* kControlHeaderAfcs =
    "time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal,afcs";
inline constexpr const char* kControlHeaderEngines =
    "time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal,afcs,"
    "power_lever_1,power_lever_2,power_lever_3,power_lever_4,rotor_speed_reference,"
    "fuel_on_1,fuel_on_2,fuel_on_3,fuel_on_4";
static_assert(eyas::kMaxEngines == 4, "kControlHeaderEngines names four engines");

// Init time: throws std::runtime_error naming the file and line.
std::vector<ControlRow> read_controls(const std::string& path);

// Writes one row (17 significant digits, so it reads back exactly), with its
// afcs column and engine columns if it has them.
void write_control_row(std::ostream& out, const ControlRow& r);

// Plays rows in sim-time order. Real time (no allocation): the rows are
// owned by the caller and must outlive the schedule.
class ControlSchedule {
public:
    ControlSchedule() = default;
    explicit ControlSchedule(const std::vector<ControlRow>& rows) noexcept : rows_(&rows) {}

    // The controls at sim time t: start plus the last row at or before t
    // (start alone before the first row), with that row's switches and
    // engine controls if the file has them. t must not decrease between calls.
    eyas::Controls at(const eyas::Controls& start, double t) noexcept;
    // The last row at or before the last t passed to at(); null before the first.
    const ControlRow* current() const noexcept;

private:
    const std::vector<ControlRow>* rows_ = nullptr;
    std::size_t next_ = 0;  // rows before this index are at or before the last t
};

}  // namespace examples
