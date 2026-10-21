// Scripted test driver (spec §9 M8): flies an aircraft through Eyas's public
// API, as a host would (include/eyas only), from a control history in CSV to
// a time history of AircraftState in CSV.
//
// Usage:
//   eyas_drive --config <aircraft.yaml> --out <out.csv>
//              [--controls <in.csv>] [--trim-speed <kt>] [--altitude <m>]
//              [--rate <Hz>] [--duration <s>]
//              [--origin <lat deg>,<lon deg>,<alt m>] [--ground <m>]
//              [--afcs <sas,pba,fps|...|none>]
//
// The aircraft is trimmed in level flight at --trim-speed (true airspeed,
// default 0: hover) and --altitude (default 0 m MSL, ISA, no wind), then
// flown at the host rate (default 60 Hz) with integer-nanosecond frames
// (D-006). The control file holds increments from the trimmed controls,
// held from each row's time until the next (zero-order hold):
//
//   time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal
//   0.0,0,0,0,0
//   1.0,0,0,0.03,0
//
// The run lasts --duration seconds (default: the last row's time, or 10 s
// without a control file). The output has one row at t = 0 and one per host
// frame: the controls applied and every AircraftState field (SI units,
// frames as documented in include/eyas/aircraft_state.hpp). --origin
// replaces the config's geodetic origin (D-044) and --ground sets a level
// ground at that height above MSL (default: no ground), so a run recorded
// by the pilot station (--record) replays exactly. --afcs sets the flight
// control system's switches for the trim and the run (a list of sas,
// sas_digital, sas_analog, pba, fps, fps_turn, trim_release; default: the
// control file's at t = 0, else none); a control file's optional afcs column
// changes them from its rows on, as the station records them (M8c3). Exit
// codes: 0
// done; 1 a frame failed (AdvanceStatus non_finite, rejected or capped);
// 2 usage or input error; 3 trim did not converge.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "common/afcs.hpp"
#include "common/control_file.hpp"
#include "common/frames.hpp"
#include "common/origin.hpp"
#include "common/state_csv.hpp"
#include "common/units.hpp"
#include "eyas/model.hpp"

namespace {

using examples::kKnot;

struct Options {
    std::string config;
    std::string out;
    std::string controls;
    double trim_speed_kt = 0.0;
    double altitude = 0.0;                                     // [m]
    int rate = 60;                                             // [Hz]
    double duration = -1.0;                                    // [s]; < 0: from the control file
    std::string origin;                                        // empty: the config's
    double ground = std::numeric_limits<double>::quiet_NaN();  // [m] MSL; NaN: none
    std::optional<eyas::AfcsSwitches> afcs;                    // --afcs; none: see run()
};

Options parse(int argc, char** argv) {
    Options opt;
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const auto value = [&]() -> std::string {
            if (i + 1 >= args.size()) {
                throw std::runtime_error("missing value after " + std::string(args[i]));
            }
            return std::string(args[++i]);
        };
        if (args[i] == "--config") {
            opt.config = value();
        } else if (args[i] == "--out") {
            opt.out = value();
        } else if (args[i] == "--controls") {
            opt.controls = value();
        } else if (args[i] == "--trim-speed") {
            opt.trim_speed_kt = std::stod(value());
        } else if (args[i] == "--altitude") {
            opt.altitude = std::stod(value());
        } else if (args[i] == "--rate") {
            opt.rate = std::stoi(value());
        } else if (args[i] == "--duration") {
            opt.duration = std::stod(value());
        } else if (args[i] == "--origin") {
            opt.origin = value();
        } else if (args[i] == "--ground") {
            opt.ground = std::stod(value());
        } else if (args[i] == "--afcs") {
            opt.afcs = examples::parse_afcs(value());
        } else {
            throw std::runtime_error("unknown option " + std::string(args[i]));
        }
    }
    if (opt.config.empty() || opt.out.empty()) {
        throw std::runtime_error("--config and --out are required");
    }
    if (opt.rate < 1) {
        throw std::runtime_error("--rate must be positive");
    }
    return opt;
}

int run(const Options& opt) {
    const std::vector<examples::ControlRow> rows = opt.controls.empty()
                                                       ? std::vector<examples::ControlRow>{}
                                                       : examples::read_controls(opt.controls);
    // The trim's switches: --afcs, else the control file's switches at
    // t = 0 (its afcs column), so a file run without --afcs doesn't start
    // from a trim without its switches (code review), else none. With both,
    // --afcs trims and the rows take over from t = 0, as the station flew it
    // (a recording's first row can differ, e.g. the trim release held).
    eyas::AfcsSwitches afcs = opt.afcs.value_or(eyas::AfcsSwitches{});
    if (!opt.afcs && !rows.empty() && rows.front().t <= 0.0 && rows.front().has_afcs) {
        afcs = rows.front().afcs;
    }
    double duration = opt.duration;
    if (duration < 0.0) {
        duration = rows.empty() ? 10.0 : rows.back().t;
    }

    eyas::Model model =
        opt.origin.empty() ? eyas::Model::from_file(opt.config)
                           : eyas::Model::from_file(opt.config, examples::parse_origin(opt.origin));
    for (const std::string& gap : model.placeholders()) {
        std::cerr << "note: data gap: " << gap << '\n';
    }
    eyas::TrimCondition condition;
    condition.airspeed = opt.trim_speed_kt * kKnot;
    condition.altitude = opt.altitude;
    condition.ground_height = opt.ground;  // trimmed in ground effect, as flown (D-050)
    condition.afcs = afcs;                 // the trimmed controls carry them through the run
    const eyas::TrimResult trim = model.trim(condition);
    if (!trim.converged) {
        std::cerr << "error: trim did not converge\n";
        return 3;
    }
    // Fly on in the trim's environment (ISA, no wind, --ground).
    const eyas::Environment env{condition.isa_delta_T, condition.wind_n, opt.ground};

    std::ofstream out(opt.out);
    if (!out) {
        throw std::runtime_error("cannot write " + opt.out);
    }
    out.precision(17);
    examples::write_state_header(out);
    examples::write_state_row(out, trim.controls, model.outputs());

    // Nominal integer frame period (D-006): the remainder carry keeps Eyas's
    // clock exact over any number of frames.
    const std::int64_t frame_ns = examples::nominal_frame_ns(opt.rate);
    const std::int64_t frames = examples::frames_for(duration, frame_ns);
    examples::ControlSchedule schedule(rows);
    for (std::int64_t f = 0; f < frames; ++f) {
        const eyas::Controls c = schedule.at(trim.controls, model.outputs().sim_time);
        const eyas::AdvanceStatus s = model.advance(c, env, frame_ns);
        examples::write_state_row(out, c, model.outputs());
        if (s.non_finite || s.rejected_dt || s.rejected_env || s.capped) {
            std::cerr << "error: frame " << f << " failed (non_finite " << s.non_finite
                      << ", rejected_dt " << s.rejected_dt << ", rejected_env " << s.rejected_env
                      << ", capped " << s.capped << ")\n";
            return 1;
        }
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(parse(argc, argv));
    } catch (const std::exception& e) {  // eyas::InitError included
        std::cerr << "error: " << e.what() << '\n';
        return 2;
    }
}
