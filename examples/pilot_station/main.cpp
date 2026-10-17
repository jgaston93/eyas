// Eyas desktop pilot station (spec §9 M8, D-042 M8b, D-044). Host code: it
// uses only the public API (include/eyas), as eyas_drive does.
//
// Usage:
//   eyas_pilot_station --config <aircraft.yaml>
//       [--input gamepad|joystick|mouse|fg|script|none] [--map <file>] [--print-map]
//       [--controls <csv>]            (for --input script: an eyas_drive control file)
//       [--start pad|hover|60|120] [--origin <lat deg>,<lon deg>,<alt m>]
//       [--hover-height <m>] [--rate <Hz>]
//       [--fg <host:port>] [--listen <port>]
//       [--record <csv>] [--out <csv>] [--duration <s>] [--headless] [--no-pace]
//       [--view chase|cg|observer] [--screenshot <ppm>] [--no-help]
//
// It flies the aircraft at a fixed host rate (default 60 Hz), paced to the
// wall clock, from the chosen input, and shows it in a window and/or sends
// it to FlightGear (--fg, with flightgear/eyas_fdm.xml); --listen receives
// FlightGear's joystick (flightgear/eyas_ctrl.xml) for --input fg. The start
// points are prepared at init: on the pad (settled on the gear, brakes set)
// and trimmed level flight in hover, at 60 kt and 120 kt, --hover-height
// (default 100 ft) above the ground, which is level at the origin's altitude.
// --record writes an eyas_drive control file that replays the flight (see
// its header); --out writes the state history in eyas_drive's format.
// --headless runs without a window (FlightGear, a script or none);
// --no-pace runs as fast as it can, for tests. --duration stops after that
// much sim time from the start (default: until the window closes).
// --screenshot saves the last frame drawn (a binary PPM), for the docs.
//
// A crash (Station::crashed(): a failed frame, a wheel down at more than 30°
// of bank or pitch, more than 10 g, or the CG below the ground) freezes the
// aircraft until a restart; headless or with --duration, it ends the run.
//
// Exit codes: 0 done; 1 a frame failed (non-finite, capped or rejected);
// 2 usage or input error; 3 init error (config, trim); 4 crashed.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "common/control_file.hpp"
#include "common/frames.hpp"
#include "common/origin.hpp"
#include "controls_map.hpp"
#include "eyas/error.hpp"
#include "fg_link.hpp"
#include "mapping.hpp"
#include "recorder.hpp"
#include "station.hpp"
#ifdef EYAS_STATION_GL
#include "gl_view.hpp"
#endif

namespace {

using station::InputSource;

struct Options {
    station::StationOptions station;
    std::string origin_text;
    InputSource input = InputSource::mouse_keyboard;
    bool input_given = false;
    std::string map;
    bool print_map = false;
    std::string controls;
    std::string start = "hover";
    std::string fg;
    int listen = station::FgLink::kNoListen;
    std::string record;
    std::string out;
    double duration = -1.0;  // [s]; < 0: until quit
    bool headless = false;
    bool pace = true;
    int view = 0;
    std::string screenshot;
    bool help = true;
};

struct UsageError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

Options parse(int argc, char** argv) {
    Options opt;
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const auto value = [&]() -> std::string {
            if (i + 1 >= args.size()) {
                throw UsageError("missing value after " + std::string(args[i]));
            }
            return std::string(args[++i]);
        };
        const std::string_view a = args[i];
        if (a == "--config") {
            opt.station.config = value();
        } else if (a == "--input") {
            opt.input = station::parse_source(value());
            opt.input_given = true;
        } else if (a == "--map") {
            opt.map = value();
        } else if (a == "--print-map") {
            opt.print_map = true;
        } else if (a == "--controls") {
            opt.controls = value();
        } else if (a == "--start") {
            opt.start = value();
        } else if (a == "--origin") {
            opt.origin_text = value();
            opt.station.origin = examples::parse_origin(opt.origin_text);
        } else if (a == "--hover-height") {
            opt.station.hover_height = std::stod(value());
            if (!(opt.station.hover_height > 0.0)) {
                throw UsageError("--hover-height must be positive");
            }
        } else if (a == "--rate") {
            opt.station.rate = std::stoi(value());
        } else if (a == "--fg") {
            opt.fg = value();
        } else if (a == "--listen") {
            opt.listen = std::stoi(value());
            if (opt.listen < 1 || opt.listen > 65535) {
                throw UsageError("--listen: a UDP port, 1-65535");
            }
        } else if (a == "--record") {
            opt.record = value();
        } else if (a == "--out") {
            opt.out = value();
        } else if (a == "--duration") {
            opt.duration = std::stod(value());
        } else if (a == "--headless") {
            opt.headless = true;
        } else if (a == "--no-pace") {
            opt.pace = false;
        } else if (a == "--view") {
            const std::string v = value();
            if (v == "chase") {
                opt.view = 0;
            } else if (v == "cg") {
                opt.view = 1;
            } else if (v == "observer") {
                opt.view = 2;
            } else {
                throw UsageError("--view: chase, cg or observer");
            }
        } else if (a == "--screenshot") {
            opt.screenshot = value();
        } else if (a == "--no-help") {
            opt.help = false;
        } else {
            throw UsageError("unknown option " + std::string(a));
        }
    }
    if (!opt.input_given && opt.headless) {
        opt.input = InputSource::none;
    }
    if (opt.station.config.empty() && !opt.print_map) {
        throw UsageError("--config is required");
    }
    if (opt.station.rate < 1) {
        throw UsageError("--rate must be positive");
    }
    if ((opt.input == InputSource::script) != !opt.controls.empty()) {
        throw UsageError("--input script and --controls go together");
    }
    if (opt.input == InputSource::flightgear && opt.listen == station::FgLink::kNoListen) {
        throw UsageError("--input fg needs --listen <port>");
    }
    const bool needs_window = opt.input == InputSource::gamepad ||
                              opt.input == InputSource::joystick ||
                              opt.input == InputSource::mouse_keyboard;
    if (opt.headless && needs_window) {
        throw UsageError("--input " + std::string(station::source_name(opt.input)) +
                         " needs the window (drop --headless)");
    }
#ifndef EYAS_STATION_GL
    if (!opt.headless) {
        throw UsageError("built without GLFW/OpenGL: use --headless");
    }
#endif
    return opt;
}

std::size_t start_index(const station::Station& st, const std::string& name) {
    for (std::size_t i = 0; i < st.starts().size(); ++i) {
        if (st.starts()[i].name == name) {
            return i;
        }
    }
    throw UsageError("unknown start '" + name + "' (pad, hover, 60, 120)");
}

int run(const Options& opt) {
    station::ControlsMapConfig map_config = station::default_map(opt.input);
    if (!opt.map.empty()) {
        map_config = station::read_map_file(opt.map, map_config);
    }
    if (opt.print_map) {
        station::write_map(std::cout, map_config);
        if (opt.station.config.empty()) {
            return 0;
        }
    }
    const std::vector<examples::ControlRow> rows = opt.controls.empty()
                                                       ? std::vector<examples::ControlRow>{}
                                                       : examples::read_controls(opt.controls);

    std::optional<station::Station> st_holder;
    try {
        std::cerr << "preparing the start points (trims and the pad)...\n";
        st_holder.emplace(opt.station);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 3;
    }
    station::Station& st = *st_holder;
    for (const std::string& gap : st.placeholders()) {
        std::cerr << "note: data gap: " << gap << '\n';
    }
    std::size_t current = start_index(st, opt.start);

    station::FgLink fg;
    fg.open(opt.fg, opt.listen);
    station::Recorder rec(opt.record, opt.out, opt.station.config, opt.origin_text,
                          opt.station.rate);
#ifdef EYAS_STATION_GL
    std::unique_ptr<station::GlView> view;
    if (!opt.headless) {
        view = std::make_unique<station::GlView>(opt.input, opt.help);
        view->set_camera(opt.view);
        if (!view->device().empty()) {
            std::cerr << "input device: " << view->device() << '\n';
        }
    }
#endif

    station::ControlsMap map(map_config, st.starts()[current].controls);
    examples::ControlSchedule schedule(rows);
    std::int64_t frames_flown = 0;
    bool reported_crash = false;
    const auto begin = [&](std::size_t index) {
        if (!st.start(index)) {
            return;
        }
        current = index;
        map.restart(st.starts()[current].controls);
        schedule = examples::ControlSchedule(rows);
        frames_flown = 0;
        reported_crash = false;
        rec.begin(st, st.outputs());
    };
    begin(current);

    const std::int64_t frame_ns = st.frame_ns();
    const double dt = static_cast<double>(frame_ns) * 1e-9;
    const std::int64_t frame_limit =
        opt.duration < 0.0 ? -1 : examples::frames_for(opt.duration, frame_ns);

    using Clock = std::chrono::steady_clock;
    const auto period = std::chrono::nanoseconds(frame_ns);
    auto deadline = Clock::now();
    auto ratio_wall = Clock::now();
    double ratio_sim = st.outputs().sim_time;
    double realtime_ratio = 1.0;
    std::int64_t overruns = 0;
    bool paused = false;
    bool faulted = false;
#ifdef EYAS_STATION_GL
    const bool view_open = view != nullptr;
#else
    const bool view_open = false;
#endif
    station::RawInput raw;

    for (;;) {
        bool quit = false;
#ifdef EYAS_STATION_GL
        if (view) {
            const station::ViewCommands cmd = view->poll(raw);
            quit = cmd.quit;
            paused = cmd.pause ? !paused : paused;
            if (cmd.restart) {
                begin(current);
            } else if (cmd.start >= 0) {
                begin(static_cast<std::size_t>(cmd.start));
            }
        }
#endif
        if (opt.input == InputSource::flightgear) {
            (void)fg.poll(raw);  // keeps the newest line; the last one holds otherwise
        }
        const bool last_frame = frame_limit >= 0 && frames_flown + 1 >= frame_limit;
        if (quit || (frame_limit >= 0 && frames_flown >= frame_limit)) {
            break;
        }

        eyas::Controls applied = st.starts()[current].controls;
        if (st.crashed() && !reported_crash) {
            std::cerr << "crashed at t = " << st.outputs().sim_time << " s"
                      << (view_open ? " (R or 1-4 to fly again)" : "") << '\n';
            reported_crash = true;
            if (!view_open || frame_limit >= 0) {  // a timed run ends at a crash
                break;
            }
        }
        if (!paused && !st.crashed()) {
            const double t = st.flight_time();  // scripts and recordings run from the start
            station::ControlDeltas d;
            if (opt.input == InputSource::script) {
                applied = schedule.at(st.starts()[current].controls, t);
                if (const examples::ControlRow* r = schedule.current(); r != nullptr) {
                    d = {r->d_collective, r->d_lat, r->d_lon, r->d_pedal};
                }
            } else {
                applied = map.update(raw, dt);
                d = map.deltas();
            }
            (void)st.frame(applied);
            ++frames_flown;
            rec.frame(t, d, applied, st.outputs());
            if (st.faulted() && !faulted) {
                std::cerr << "warning: a frame failed at t = " << st.outputs().sim_time
                          << " s (non-finite, capped or rejected)\n";
            }
            faulted = faulted || st.faulted();
        } else {
            applied = station::apply(map.start(), map.deltas());
            applied.brake_left = applied.brake_right = map.brakes_set() ? 1.0 : 0.0;
        }
        fg.send(st.outputs());

        const auto now = Clock::now();
        if (now - ratio_wall >= std::chrono::seconds(1)) {
            const double wall = std::chrono::duration<double>(now - ratio_wall).count();
            realtime_ratio = (st.outputs().sim_time - ratio_sim) / wall;
            ratio_wall = now;
            ratio_sim = st.outputs().sim_time;
        }
#ifdef EYAS_STATION_GL
        if (view) {
            station::HudInfo hud;
            hud.state = &st.outputs();
            hud.applied = applied;
            hud.trim_point = station::apply(map.start(), map.trim_points());
            hud.ground_height = st.ground_height();
            hud.source = station::source_name(opt.input);
            hud.start = st.starts()[current].name;
            hud.paused = paused;
            hud.trim_held = map.trim_held();
            hud.faulted = st.faulted();
            hud.crashed = st.crashed();
            hud.realtime_ratio = realtime_ratio;
            hud.advance_us = static_cast<double>(st.last_advance_ns()) * 1e-3;
            hud.worst_advance_us = static_cast<double>(st.worst_advance_ns()) * 1e-3;
            hud.overruns = overruns;
            hud.fg_lines = fg.receiving() ? fg.lines_received() : -1;
            hud.fg_sending = fg.sending();
            hud.placeholders = static_cast<int>(st.placeholders().size());
            hud.recording_segment = rec.recording() ? rec.segment() : 0;
            if (last_frame && !opt.screenshot.empty()) {
                view->screenshot(opt.screenshot);
            }
            view->draw(hud);
        }
#endif

        if (opt.pace) {
            // Nominal frames (D-006): a late frame still advances by one
            // period, so the run stays reproducible; after five periods
            // behind, the clock resyncs instead of catching up.
            deadline += period;
            const auto late = Clock::now();
            if (late > deadline) {
                ++overruns;
                if (late - deadline > 5 * period) {
                    deadline = late;
                }
            } else {
                std::this_thread::sleep_until(deadline);
            }
        }
    }
    rec.end();
    std::cerr << "flew " << frames_flown << " frames; worst advance() "
              << static_cast<double>(st.worst_advance_ns()) * 1e-3 << " us; overruns " << overruns
              << "; real time ratio " << realtime_ratio;
    if (fg.receiving()) {
        std::cerr << "; FlightGear lines " << fg.lines_received() << " (rejected "
                  << fg.lines_rejected() << ')';
    }
    std::cerr << '\n';
    if (faulted) {
        return 1;
    }
    return st.crashed() ? 4 : 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(parse(argc, argv));
    } catch (const UsageError& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    } catch (const std::exception& e) {  // eyas::InitError, files, sockets
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}
