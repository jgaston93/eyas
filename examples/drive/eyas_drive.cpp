// Scripted test driver (spec §9 M8): flies an aircraft through Eyas's public
// API, as a host would (include/eyas only), from a control history in CSV to
// a time history of AircraftState in CSV.
//
// Usage:
//   eyas_drive --config <aircraft.yaml> --out <out.csv>
//              [--controls <in.csv>] [--trim-speed <kt>] [--altitude <m>]
//              [--rate <Hz>] [--duration <s>]
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
// frames as documented in include/eyas/aircraft_state.hpp). Exit codes: 0
// done; 1 a frame failed (AdvanceStatus non_finite, rejected or capped);
// 2 usage or input error; 3 trim did not converge.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "eyas/model.hpp"

namespace {

constexpr double kKnot = 1852.0 / 3600.0;  // [m/s]

struct Options {
    std::string config;
    std::string out;
    std::string controls;
    double trim_speed_kt = 0.0;
    double altitude = 0.0;   // [m]
    int rate = 60;           // [Hz]
    double duration = -1.0;  // [s]; < 0: from the control file
};

// One row of the control file: increments from trim from time t on.
struct ControlRow {
    double t = 0.0;
    double d_collective = 0.0;
    double d_lat = 0.0;
    double d_lon = 0.0;
    double d_pedal = 0.0;
};

std::vector<ControlRow> read_controls(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::string line;
    std::getline(in, line);
    if (!line.starts_with("time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal")) {
        throw std::runtime_error(path +
                                 ": expected header time_s,d_collective,d_lat_cyclic,"
                                 "d_lon_cyclic,d_pedal");
    }
    std::vector<ControlRow> rows;
    int n = 1;
    while (std::getline(in, line)) {
        ++n;
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream s(line);
        ControlRow r;
        char c1 = 0;
        char c2 = 0;
        char c3 = 0;
        char c4 = 0;
        if (!(s >> r.t >> c1 >> r.d_collective >> c2 >> r.d_lat >> c3 >> r.d_lon >> c4 >>
              r.d_pedal) ||
            c1 != ',' || c2 != ',' || c3 != ',' || c4 != ',') {
            throw std::runtime_error(path + ":" + std::to_string(n) + ": bad row");
        }
        if (!rows.empty() && r.t < rows.back().t) {
            throw std::runtime_error(path + ":" + std::to_string(n) + ": time goes backwards");
        }
        rows.push_back(r);
    }
    return rows;
}

// The controls at time t: trim plus the last row at or before t.
eyas::Controls controls_at(const eyas::Controls& trim, const std::vector<ControlRow>& rows,
                           double t) {
    eyas::Controls c = trim;
    const ControlRow* row = nullptr;
    for (const ControlRow& r : rows) {
        if (r.t <= t) {
            row = &r;
        }
    }
    if (row != nullptr) {
        c.collective += row->d_collective;
        c.lat_cyclic += row->d_lat;
        c.lon_cyclic += row->d_lon;
        c.pedal += row->d_pedal;
    }
    return c;
}

void write_header(std::ostream& out) {
    out << "time_s,collective,lat_cyclic,lon_cyclic,pedal,"
           "latitude_rad,longitude_rad,altitude_m,qw,qx,qy,qz,roll_rad,pitch_rad,heading_rad,"
           "pos_n_x_m,pos_n_y_m,pos_n_z_m,vel_n_x_mps,vel_n_y_mps,vel_n_z_mps,"
           "vel_b_x_mps,vel_b_y_mps,vel_b_z_mps,p_radps,q_radps,r_radps,"
           "accel_b_x_mps2,accel_b_y_mps2,accel_b_z_mps2,"
           "specific_force_b_x_mps2,specific_force_b_y_mps2,specific_force_b_z_mps2,"
           "load_factor,ground_speed_mps,track_rad,"
           "true_airspeed_mps,equivalent_airspeed_mps,calibrated_airspeed_mps,"
           "indicated_airspeed_mps,angle_of_attack_rad,sideslip_rad,pressure_altitude_m,"
           "vertical_speed_mps,outside_air_temperature_K,"
           "rotor_speed_percent,main_rotor_thrust_N,main_rotor_torque_Nm,main_rotor_power_W,"
           "coning_rad,flap_1c_rad,flap_1s_rad,main_rotor_azimuth_rad,tail_rotor_thrust_N,"
           "tail_rotor_azimuth_rad,weight_on_wheels,quat_norm_error\n";
}

void write_row(std::ostream& out, const eyas::Controls& c, const eyas::AircraftState& o) {
    const auto v3 = [&](const eyas::Vec3& v) {
        out << ',' << v.x() << ',' << v.y() << ',' << v.z();
    };
    out << o.sim_time << ',' << c.collective << ',' << c.lat_cyclic << ',' << c.lon_cyclic << ','
        << c.pedal << ',' << o.latitude << ',' << o.longitude << ',' << o.altitude << ','
        << o.q_nb.w() << ',' << o.q_nb.x() << ',' << o.q_nb.y() << ',' << o.q_nb.z() << ','
        << o.roll << ',' << o.pitch << ',' << o.heading;
    v3(o.pos_n);
    v3(o.vel_n);
    v3(o.vel_b);
    v3(o.omega_b);
    v3(o.accel_b);
    v3(o.specific_force_b);
    out << ',' << o.load_factor << ',' << o.ground_speed << ',' << o.track << ',' << o.true_airspeed
        << ',' << o.equivalent_airspeed << ',' << o.calibrated_airspeed << ','
        << o.indicated_airspeed << ',' << o.angle_of_attack << ',' << o.sideslip << ','
        << o.pressure_altitude << ',' << o.vertical_speed << ',' << o.outside_air_temperature << ','
        << o.rotor_speed_percent << ',' << o.main_rotor_thrust << ',' << o.main_rotor_torque << ','
        << o.main_rotor_power << ',' << o.coning << ',' << o.flap_1c << ',' << o.flap_1s << ','
        << o.main_rotor_azimuth << ',' << o.tail_rotor_thrust << ',' << o.tail_rotor_azimuth << ','
        << (o.weight_on_wheels ? 1 : 0) << ',' << o.quat_norm_error << '\n';
}

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
    const std::vector<ControlRow> rows =
        opt.controls.empty() ? std::vector<ControlRow>{} : read_controls(opt.controls);
    double duration = opt.duration;
    if (duration < 0.0) {
        duration = rows.empty() ? 10.0 : rows.back().t;
    }

    eyas::Model model = eyas::Model::from_file(opt.config);
    for (const std::string& gap : model.placeholders()) {
        std::cerr << "note: data gap: " << gap << '\n';
    }
    eyas::TrimCondition condition;
    condition.airspeed = opt.trim_speed_kt * kKnot;
    condition.altitude = opt.altitude;
    const eyas::TrimResult trim = model.trim(condition);
    if (!trim.converged) {
        std::cerr << "error: trim did not converge\n";
        return 3;
    }
    // Fly on in the trim's environment: ISA, no wind, no ground.
    const eyas::Environment env{condition.isa_delta_T, condition.wind_n};

    std::ofstream out(opt.out);
    if (!out) {
        throw std::runtime_error("cannot write " + opt.out);
    }
    out.precision(17);
    write_header(out);
    write_row(out, trim.controls, model.outputs());

    // Nominal integer frame period (D-006): the remainder carry keeps Eyas's
    // clock exact over any number of frames.
    const std::int64_t frame_ns = (1'000'000'000 + opt.rate / 2) / opt.rate;
    const auto frames =
        static_cast<std::int64_t>(std::llround(duration * 1e9 / static_cast<double>(frame_ns)));
    for (std::int64_t f = 0; f < frames; ++f) {
        const eyas::Controls c = controls_at(trim.controls, rows, model.outputs().sim_time);
        const eyas::AdvanceStatus s = model.advance(c, env, frame_ns);
        write_row(out, c, model.outputs());
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
