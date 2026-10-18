#include "recorder.hpp"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "common/afcs.hpp"
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

std::string shell_quote(std::string_view s) {
    const bool plain = !s.empty() && s.find_first_not_of(
                                         "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz"
                                         "0123456789_./:+=,-") == std::string_view::npos;
    if (plain) {
        return std::string(s);
    }
    std::string q = "'";
    for (const char c : s) {
        if (c == '\'') {
            q += "'\\''";
        } else {
            q += c;
        }
    }
    return q + "'";
}

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
    brakes_noted_ = false;
    afcs_noted_ = false;
    const StartPoint& sp = station.starts()[station.current_start()];
    start_ = sp.controls;
    if (!controls_path_.empty()) {
        const std::string path = segment_path(controls_path_, segment_);
        controls_.open(path);
        if (!controls_) {
            throw std::runtime_error("cannot write " + path);
        }
        controls_ << "# Eyas pilot station recording (D-044), start '" << sp.name << "'\n";
        if (sp.replayable) {
            controls_ << "# Replays exactly unless a later comment says the brakes or the AFCS "
                         "switches changed.\n";
            controls_ << "# Replay: eyas_drive --config " << shell_quote(config_) << " --controls "
                      << shell_quote(path) << " --out <out.csv> --rate " << rate_
                      << " --trim-speed " << exact(sp.trim_speed_kt) << " --altitude "
                      << exact(sp.altitude)
                      << (origin_.empty() ? "" : " --origin " + shell_quote(origin_))
                      << " --ground " << exact(station.ground_height()) << " --afcs "
                      << examples::afcs_text(sp.controls.afcs) << " --duration <s>\n";
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
    if (controls_.is_open() && !brakes_noted_ &&
        (c.brake_left != start_.brake_left || c.brake_right != start_.brake_right)) {
        controls_ << "# NOT REPLAYABLE from t = " << exact(t)
                  << " s: the brakes changed, and eyas_drive has no brake input\n";
        brakes_noted_ = true;
    }
    const eyas::AfcsSwitches& a = c.afcs;
    const eyas::AfcsSwitches& a0 = start_.afcs;
    if (controls_.is_open() && !afcs_noted_ &&
        (a.sas_digital != a0.sas_digital || a.sas_analog != a0.sas_analog ||
         a.pitch_bias != a0.pitch_bias)) {
        controls_ << "# NOT REPLAYABLE from t = " << exact(t)
                  << " s: the AFCS switches changed, and eyas_drive holds its --afcs\n";
        afcs_noted_ = true;
    }
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
