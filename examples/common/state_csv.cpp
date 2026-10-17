#include "state_csv.hpp"

#include <ostream>

namespace examples {

void write_state_header(std::ostream& out) {
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

void write_state_row(std::ostream& out, const eyas::Controls& c, const eyas::AircraftState& o) {
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

}  // namespace examples
