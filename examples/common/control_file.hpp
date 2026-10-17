#pragma once

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
// the next row (zero-order hold). Lines starting with '#' are comments.
namespace examples {

struct ControlRow {
    double t = 0.0;  // [s] sim time the row applies from
    double d_collective = 0.0;
    double d_lat = 0.0;
    double d_lon = 0.0;
    double d_pedal = 0.0;
};

inline constexpr const char* kControlHeader =
    "time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal";

// Init time: throws std::runtime_error naming the file and line.
std::vector<ControlRow> read_controls(const std::string& path);

// Writes one row (17 significant digits, so it reads back exactly).
void write_control_row(std::ostream& out, const ControlRow& r);

// Plays rows in sim-time order. Real time (no allocation): the rows are
// owned by the caller and must outlive the schedule.
class ControlSchedule {
public:
    ControlSchedule() = default;
    explicit ControlSchedule(const std::vector<ControlRow>& rows) noexcept : rows_(&rows) {}

    // The controls at sim time t: start plus the last row at or before t
    // (start alone before the first row). t must not decrease between calls.
    eyas::Controls at(const eyas::Controls& start, double t) noexcept;
    // The last row at or before the last t passed to at(); null before the first.
    const ControlRow* current() const noexcept;

private:
    const std::vector<ControlRow>* rows_ = nullptr;
    std::size_t next_ = 0;  // rows before this index are at or before the last t
};

}  // namespace examples
