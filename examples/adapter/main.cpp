// The host side of the adapter example: code written only against
// host::IFlightModel, so swapping in another adapter changes one line (the
// factory). Flies 5 s from a 60 kt trim at 60 Hz with the controls held at trim, and
// prints the pose and air data once a second.
//
// Usage: eyas_adapter_example <aircraft.yaml>

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "eyas_adapter.hpp"
#include "iflight_model.hpp"

namespace {

// The only line that names the flight model.
std::unique_ptr<host::IFlightModel> make_flight_model() {
    return std::make_unique<EyasAdapter>();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " <aircraft.yaml>\n";
        return 2;
    }
    std::unique_ptr<host::IFlightModel> fm = make_flight_model();
    std::string error;
    host::InitialCondition ic;
    ic.airspeed_kt = 60.0;
    ic.altitude_ft = 1000.0;
    ic.sas = true;  // as the UH-60A flies (M8c2, M8c3)
    ic.pitch_bias = true;
    ic.fps = true;
    if (!fm->initialize(argv[1], ic, error)) {
        std::cerr << "error: " << error << '\n';
        return 1;
    }
    // Start the cockpit controls at the trim and hold them.
    const host::PilotInputs in = fm->trim_inputs();
    const std::int64_t frame_ns = 16'666'667;  // 60 Hz; Eyas carries the remainder (D-006)
    std::printf("%5s %9s %9s %8s %7s %7s %8s %8s %6s\n", "t[s]", "IAS[kt]", "TAS[kt]", "Hp[ft]",
                "VS[fpm]", "pitch", "roll", "Q[ft-lb]", "NR%");
    for (int f = 1; f <= 300; ++f) {
        fm->update(in, frame_ns);
        if (!fm->healthy()) {
            std::cerr << "flight model fault at frame " << f << '\n';
            return 1;
        }
        if (f % 60 == 0) {
            const host::Pose p = fm->pose();
            const host::AirData a = fm->air_data();
            const host::RotorData r = fm->rotor();
            std::printf("%5.1f %9.2f %9.2f %8.1f %7.0f %7.2f %8.2f %8.0f %6.1f\n", f / 60.0,
                        a.indicated_airspeed_kt, a.true_airspeed_kt, a.pressure_altitude_ft,
                        a.vertical_speed_fpm, p.pitch_deg, p.roll_deg, r.main_rotor_torque_ftlbf,
                        r.rotor_speed_percent);
        }
    }
    return 0;
}
