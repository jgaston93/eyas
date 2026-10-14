#pragma once

// Fixtures for the M6c landing-gear tests (D-032). The gear data are GenHel's
// UH-60A values (CR-166309 §5.7.5, pp. 5.7-40 to 5.7-47;
// data/uh60a/genhel/gear_constants_5_7_5.txt, DATA_GAPS.md "UH-60A landing
// gear (M6c)"); the CG station is GenHel's (p. 5.1-49).

#include <cmath>
#include <sstream>
#include <string>

#include "support/genhel_gear.hpp"

namespace eyas::test {

inline std::string uh60a_gear_dir() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/gear";
}

inline std::string genhel_dir() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/genhel";
}

// GenHel's CG (p. 5.1-49) [in].
inline constexpr double kFSCGB = 355.9;
inline constexpr double kWLCGB = 248.2;

// The tail tire's K = 19.7 √Z lbf/in (Z in lbf, p. 5.7-46) as K_ref √(Z/Z_ref).
inline constexpr double kTailLateralRefLoad = 1000.0;  // [lbf]
inline double tail_lateral_ref_stiffness() {           // [lbf/in]
    return 19.7 * std::sqrt(kTailLateralRefLoad);
}

struct GearYamlOptions {
    double sliding_factor = 0.85;  // K_SLIP
    double mu = 0.6;
    std::string tail_brake = "none";  // GenHel's tail wheel: never braked
    std::string extra_strut;          // appended to every strut block
    std::string extra_friction;       // appended to the friction block
};

inline std::string fmt_num(double v) {
    std::ostringstream ss;
    ss.precision(17);
    ss << v;
    return ss.str();
}

// GenHel's three UH-60A gears as a `type: landing_gear` node.
inline std::string uh60a_gear_yaml(const GearYamlOptions& o = {}) {
    const std::string d = uh60a_gear_dir();
    const auto main = [&](const std::string& name, double bl, const std::string& side) {
        return "  - name: " + name + R"(
    axle: {x: ")" +
               fmt_num(kFSCGB - 297.43) + R"( in", y: ")" + fmt_num(bl) + R"( in", z: ")" +
               fmt_num(kWLCGB - 184.8) + R"( in"}
    unsprung_mass: "2.6708 slug"
    brake: )" + side +
               R"(
    tire:
      radius: "12.65 in"
      wheels: 1
      pressure: "140 psi"
      rated_pressure: "110 psi"
      vertical_load: )" +
               d + R"(/main_tire_vertical.csv
      damping_stiffness: )" +
               d + R"(/main_tire_damping_stiffness.csv
      lateral_stiffness: )" +
               d + R"(/main_tire_lateral.csv
      fore_aft_coefficient: 0.53
      damping_factor: "0.003 s"
      damping_lag: "0.01 s"
    strut:
      preload: "3000 lbf"
      air_spring: "3465 ft*lbf"
      damper:
        break_speed: "1.4167 ft/s"
        low: {coefficient: "1382.4 lbf*s^2/ft^2", offset: "0 lbf"}
        high: {coefficient: "216.0 lbf*s^2/ft^2", offset: "2340.98 lbf"}
)" + o.extra_strut;
    };
    return R"(type: landing_gear
name: gear
friction:
  coefficient: )" +
           fmt_num(o.mu) + R"(
  brake_factor_set: 1.0
  brake_factor_off: 0.01
  sliding_factor: )" +
           fmt_num(o.sliding_factor) + "\n" + o.extra_friction + "gears:\n" +
           main("left_main", -53.2, "left") + main("right_main", 53.2, "right") + R"(  - name: tail
    axle: {x: ")" +
           fmt_num(kFSCGB - 644.62) + R"( in", y: "0 in", z: ")" + fmt_num(kWLCGB - 180.4) +
           R"( in"}
    unsprung_mass: "1.0 slug"
    brake: )" +
           o.tail_brake + R"(
    tire:
      radius: "7.5 in"
      wheels: 1
      pressure: "90 psi"
      rated_pressure: "110 psi"
      vertical_load: )" +
           d + R"(/tail_tire_vertical.csv
      damping_stiffness: )" +
           d + R"(/tail_tire_damping_stiffness.csv
      lateral_stiffness_law: {reference_load: ")" +
           fmt_num(kTailLateralRefLoad) + R"( lbf", reference_stiffness: ")" +
           fmt_num(tail_lateral_ref_stiffness()) + R"( lbf/in"}
      fore_aft_coefficient: 0.53
      damping_factor: "0.003 s"
      damping_lag: "0.01 s"
    strut:
      preload: "1251.0 lbf"
      air_spring: "2208.0 ft*lbf"
      damper:
        break_speed: "1 ft/s"
        low: {coefficient: "66.72 lbf*s^2/ft^2", offset: "0 lbf"}
        high: {coefficient: "66.72 lbf*s^2/ft^2", offset: "0 lbf"}
)" + o.extra_strut;
}

// The same three gears for the GenHel reference, tables read from the
// as-printed transcriptions.
inline GenHelGearInput genhel_uh60a_gear(double kslip = 0.85, double brake = 1.0) {
    const std::string g = genhel_dir();
    const std::vector<double> p3{90.0, 120.0, 150.0};
    GenHelMap main_v{read_genhel_rows(g + "/table_7_5_3_main_tire_vertical.txt"), p3};
    GenHelMap main_d{read_genhel_rows(g + "/table_7_5_4_main_tire_damping_stiffness.txt"), p3};
    GenHelMap main_l{read_genhel_rows(g + "/table_7_5_2_main_tire_lateral.txt"), p3};
    GenHelMap tail_v{read_genhel_rows(g + "/table_7_5_5_tail_tire_vertical.txt"), p3};
    GenHelMap tail_d{read_genhel_rows(g + "/table_7_5_6_tail_tire_damping_stiffness.txt"), {}};

    GenHelGearInput in;
    in.FSCGB = kFSCGB;
    in.WLCGB = kWLCGB;
    in.MU = 0.6;
    in.BRK = 1.0;
    in.NBRK = 0.01;
    in.KSLIP = kslip;
    in.brake = brake;
    for (const double bl : {-53.2, 53.2}) {
        GenHelGear m;
        m.FSGA = 297.43;
        m.WLGAO = 184.8;
        m.BLGA = bl;
        m.RT = 12.65;
        m.DT = 25.3;
        m.P = 140.0;
        m.PR = 110.0;
        m.M = 2.6708;
        m.GD = 0.003;
        m.CPV = 3465.0;
        m.FPRE = 3000.0;
        m.SD_BRK = 1.4167;
        m.C_LO = 1382.4;
        m.C_HI = 216.0;
        m.F0_HI = 2340.98;
        m.braked = true;
        m.main_tire = true;
        m.vertical = main_v;
        m.damping = main_d;
        m.lateral = main_l;
        in.gears.push_back(m);
    }
    GenHelGear t;
    t.FSGA = 644.62;
    t.WLGAO = 180.4;
    t.RT = 7.5;
    t.DT = 15.0;
    t.P = 90.0;
    t.PR = 110.0;
    t.M = 1.0;
    t.GD = 0.003;
    t.CPV = 2208.0;
    t.FPRE = 1251.0;
    t.SD_BRK = 1.0;
    t.C_LO = 66.72;
    t.C_HI = 66.72;
    t.braked = false;
    t.main_tire = false;
    t.vertical = tail_v;
    t.damping = tail_d;
    in.gears.push_back(t);
    return in;
}

}  // namespace eyas::test
