#include "recorder.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "common/control_file.hpp"
#include "common/state_csv.hpp"

namespace station {

namespace {

std::string segment_path(const std::string& base, int segment) {
    return segment <= 1 ? base : base + "." + std::to_string(segment) + ".csv";
}

// %.17g: reads back to the same double.
std::string exact(double v) {
    std::array<char, 32> b{};
    (void)std::snprintf(b.data(), b.size(), "%.17g", v);
    return b.data();
}

}  // namespace

Recorder::Recorder(std::string controls_path, std::string state_path, std::string config,
                   std::string origin, int rate) noexcept
    : controls_path_(std::move(controls_path)),
      state_path_(std::move(state_path)),
      config_(std::move(config)),
      origin_(std::move(origin)),
      rate_(rate) {}

void Recorder::begin(const Station& station, const eyas::AircraftState& state) {
    end();
    ++segment_;
    first_ = true;
    const StartPoint& sp = station.starts()[station.current_start()];
    if (!controls_path_.empty()) {
        const std::string path = segment_path(controls_path_, segment_);
        controls_.open(path);
        if (!controls_) {
            throw std::runtime_error("cannot write " + path);
        }
        controls_ << "# Eyas pilot station recording (D-044), start '" << sp.name << "'\n";
        if (sp.replayable) {
            controls_ << "# Replay: eyas_drive --config " << config_ << " --controls " << path
                      << " --out <out.csv> --rate " << rate_ << " --trim-speed "
                      << exact(sp.trim_speed_kt) << " --altitude " << exact(sp.altitude)
                      << (origin_.empty() ? "" : " --origin " + origin_) << " --ground "
                      << exact(station.ground_height()) << " --duration <s>\n";
        } else {
            controls_ << "# Not replayable in eyas_drive: it starts from a trim, not on the pad.\n";
        }
        controls_ << examples::kControlHeader << '\n';
    }
    if (!state_path_.empty()) {
        const std::string path = segment_path(state_path_, segment_);
        state_.open(path);
        if (!state_) {
            throw std::runtime_error("cannot write " + path);
        }
        state_.precision(17);
        examples::write_state_header(state_);
        examples::write_state_row(state_, sp.controls, state);
    }
}

void Recorder::frame(double t, const ControlDeltas& d, const eyas::Controls& c,
                     const eyas::AircraftState& after) {
    if (controls_.is_open() &&
        (first_ || d.collective != last_.collective || d.lat_cyclic != last_.lat_cyclic ||
         d.lon_cyclic != last_.lon_cyclic || d.pedal != last_.pedal)) {
        examples::write_control_row(controls_,
                                    {t, d.collective, d.lat_cyclic, d.lon_cyclic, d.pedal});
        last_ = d;
        first_ = false;
    }
    if (state_.is_open()) {
        examples::write_state_row(state_, c, after);
    }
}

void Recorder::end() {
    if (controls_.is_open()) {
        controls_.close();
    }
    if (state_.is_open()) {
        state_.close();
    }
}

}  // namespace station
