#pragma once

// Fixtures for the M6b airframe tests (D-028). The tables are GenHel's UH-60A
// data (CR-166309 Tables 2.5.1–3.5.10), accepted by human review on
// 2026-10-13 (DATA_GAPS.md) and merged by tools/build_uh60a_airframe.py.
// The geometry here is synthetic: GenHel's airframe stations (pp. 5.2-15,
// 5.3-19) are still to be reviewed, and layer 1 is a stronger test with
// random geometry anyway.

#include <random>
#include <string>

#include "airframe/airframe_aero.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/rotor_wash.hpp"
#include "support/genhel_airframe.hpp"
#include "util/csv_table.hpp"
#include "util/units.hpp"

namespace eyas::test {

inline std::string uh60a_airframe_dir() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/airframe";
}
inline std::string uh60a_genhel_dir() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/genhel";
}

inline Table1D uh60a_column(const std::string& file, const char* x, const char* value,
                            units::Dimension d) {
    const CsvTable csv = read_csv_table(uh60a_airframe_dir() + "/" + file);
    csv.require_dimension(x, units::Dimension::angle);
    csv.require_dimension(value, d);
    return make_table1d(csv, x, value);
}

inline RotorWash uh60a_wash(const std::string& file) {
    const CsvTable csv = read_csv_table(uh60a_airframe_dir() + "/" + file);
    return RotorWash("main_rotor", make_multi_table2d<2>(csv, "chi", "a1", {"ekx", "ekz"}));
}

// GenHel's geometry and incidences, in GenHel's units.
struct GenHelAirframeGeometry {
    double FSCGB, WLCGB, BLCGB;
    double FSWF, WLWF, BLWF;
    double FSH1, WLH1, BLH1, SAH1, IH1;
    double FSV1, WLV1, BLV1, SAV1, IV1;
};

// Random stations: the panels 25–33 ft aft of the CG, anywhere ±3 ft
// vertically and ±1 ft laterally; the mounting point within ±2 ft of the CG.
inline GenHelAirframeGeometry random_geometry(std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    GenHelAirframeGeometry g{};
    g.FSCGB = 355.9 + 10.0 * u(rng);
    g.WLCGB = 248.2 + 10.0 * u(rng);
    g.BLCGB = 2.0 * u(rng);
    g.FSWF = g.FSCGB + 24.0 * u(rng);
    g.WLWF = g.WLCGB + 24.0 * u(rng);
    g.BLWF = g.BLCGB + 6.0 * u(rng);
    g.FSH1 = g.FSCGB + 348.0 + 48.0 * u(rng);
    g.WLH1 = g.WLCGB + 36.0 * u(rng);
    g.BLH1 = 12.0 * u(rng);
    g.SAH1 = 45.0 + 10.0 * u(rng);
    g.IH1 = 20.0 * u(rng);
    g.FSV1 = g.FSCGB + 348.0 + 48.0 * u(rng);
    g.WLV1 = g.WLCGB + 36.0 * u(rng);
    g.BLV1 = 12.0 * u(rng);
    g.SAV1 = 32.3 + 10.0 * u(rng);
    g.IV1 = 5.0 * u(rng);
    return g;
}

inline void apply(const GenHelAirframeGeometry& g, GenHelAirframeInput& in) {
    in.FSCGB = g.FSCGB;
    in.WLCGB = g.WLCGB;
    in.BLCGB = g.BLCGB;
    in.FSWF = g.FSWF;
    in.WLWF = g.WLWF;
    in.BLWF = g.BLWF;
    in.FSH1 = g.FSH1;
    in.WLH1 = g.WLH1;
    in.BLH1 = g.BLH1;
    in.SAH1 = g.SAH1;
    in.IH1 = g.IH1;
    in.FSV1 = g.FSV1;
    in.WLV1 = g.WLV1;
    in.BLV1 = g.BLV1;
    in.SAV1 = g.SAV1;
    in.IV1 = g.IV1;
}

// Body axes from GenHel stations: x = −ΔFS, y = −ΔBL (buttlines positive to
// port), z = −ΔWL.
inline Vec3 station_to_body(double fs, double bl, double wl, const GenHelAirframeGeometry& g) {
    return Vec3(-(fs - g.FSCGB), -(bl - g.BLCGB), -(wl - g.WLCGB)) * units::kInch;
}

// The production configuration with the UH-60A tables and geometry g.
inline AirframeAeroConfig uh60a_airframe(const GenHelAirframeGeometry& g) {
    using units::Dimension;
    using units::kDegree;
    using units::kFoot;
    AirframeAeroConfig c;
    c.name = "airframe";
    c.fuselage_point_b = station_to_body(g.FSWF, g.BLWF, g.WLWF, g);
    FuselageTables& f = c.fuselage;
    f.drag = uh60a_column("fuselage_alpha.csv", "alpha_wf", "drag", Dimension::area);
    f.lift = uh60a_column("fuselage_alpha.csv", "alpha_wf", "lift", Dimension::area);
    f.pitch = uh60a_column("fuselage_alpha.csv", "alpha_wf", "pitch", Dimension::volume);
    f.drag_increment =
        uh60a_column("fuselage_psi.csv", "psi_wf", "drag_increment", Dimension::area);
    f.side = uh60a_column("fuselage_psi.csv", "psi_wf", "side", Dimension::area);
    f.roll = uh60a_column("fuselage_psi.csv", "psi_wf", "roll", Dimension::volume);
    f.yaw = uh60a_column("fuselage_psi.csv", "psi_wf", "yaw", Dimension::volume);
    f.lift_increment =
        uh60a_column("fuselage_psi_low.csv", "psi_wf", "lift_increment", Dimension::area);
    f.pitch_increment =
        uh60a_column("fuselage_psi_low.csv", "psi_wf", "pitch_increment", Dimension::volume);
    c.fuselage_wash = uh60a_wash("fuselage_wash.csv");
    c.tail_wash = uh60a_wash("tail_wash.csv");

    PanelConfig& h = c.stabilator;
    h.position_b = station_to_body(g.FSH1, g.BLH1, g.WLH1, g);
    h.area = g.SAH1 * kFoot * kFoot;
    h.incidence = g.IH1 * kDegree;
    h.cl = uh60a_column("stabilator_coefficients.csv", "alpha", "cl", Dimension::dimensionless);
    h.cd = uh60a_column("stabilator_coefficients.csv", "alpha", "cd", Dimension::dimensionless);
    h.q_ratio =
        uh60a_column("stabilator_q_ratio.csv", "alpha_wf", "q_ratio", Dimension::dimensionless);
    h.wake_angle =
        uh60a_column("stabilator_downwash.csv", "alpha_wf", "downwash", Dimension::angle);

    PanelConfig& v = c.fin;
    v.position_b = station_to_body(g.FSV1, g.BLV1, g.WLV1, g);
    v.area = g.SAV1 * kFoot * kFoot;
    v.incidence = g.IV1 * kDegree;
    v.cl = uh60a_column("fin_coefficients.csv", "beta", "cl", Dimension::dimensionless);
    v.cd = uh60a_column("fin_coefficients.csv", "beta", "cd", Dimension::dimensionless);
    v.q_ratio = uh60a_column("fin_q_ratio.csv", "psi_wf", "q_ratio", Dimension::dimensionless);
    v.wake_angle = uh60a_column("fin_sidewash.csv", "psi_wf", "sidewash", Dimension::angle);
    return c;
}

// The airframe's wake at the UH-60A tail rotor: GenHel's fin and stabilator
// tables (p. 5.4-5).
inline FuselageWake uh60a_tail_rotor_wake() {
    using units::Dimension;
    FuselageWake w;
    w.airframe = "airframe";
    w.q_ratio = uh60a_column("fin_q_ratio.csv", "psi_wf", "q_ratio", Dimension::dimensionless);
    w.sidewash = uh60a_column("fin_sidewash.csv", "psi_wf", "sidewash", Dimension::angle);
    w.downwash = uh60a_column("stabilator_downwash.csv", "alpha_wf", "downwash", Dimension::angle);
    return w;
}

}  // namespace eyas::test
