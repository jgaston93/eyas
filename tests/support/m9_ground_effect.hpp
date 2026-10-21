#pragma once

// M9 ground effect on the UH-60A (D-050): the aircraft with either model, trims
// at a wheel height, and USAAEFA's hover fits. Shared by the gate
// (tests/unit/test_ground_effect.cpp) and the generator
// (tests/validation/m9_ground_effect_cases.cpp).

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include "airframe/landing_gear.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "env/atmosphere.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "util/units.hpp"

namespace eyas::test::m9 {

inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The UH-60A with `model` ("genhel", "cheeseman_bennett" or "none").
inline Uh60aHarness aircraft(const std::string& model) {
    const std::string text = ballin::config_text(uh60a_config_path());
    const std::regex key(R"(ground_effect: genhel)");
    if (std::distance(std::sregex_iterator(text.begin(), text.end(), key),
                      std::sregex_iterator()) != 1) {
        throw std::runtime_error("expected exactly one ground_effect: genhel");
    }
    return Uh60aHarness(Config::from_string(
        std::regex_replace(text, key, "ground_effect: " + model), "uh60a, ground_effect " + model));
}

// Gross mass [kg]: the airframe, the blades and the gear's unsprung masses
// (D-002, D-034).
inline double gross_mass(const Uh60aHarness& ac) {
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    double m = ac.airframe_mass().mass + mr.config().num_blades * mr.config().flap->mass;
    for (const GearConfig& g : ac.get<LandingGear>("gear").config().gears) {
        m += g.unsprung_mass;
    }
    return m;
}

struct Point {
    bool converged = false;
    TrimResult trim;
    double power = 0.0;        // [W] main rotor, mean over the revolution
    double hub_height = kNaN;  // z/R [-] at the trimmed attitude
    double gear_load = 0.0;    // [N] largest gear normal load over the revolution
};

// The main rotor's mean power and the largest gear normal load over the
// trimmed revolution, the body held (the trim's own orbit and step).
inline void revolution(const Uh60aHarness& ac, std::vector<double> x, const Controls& c,
                       const Environment& env, Point& p) {
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const auto& gear = ac.get<LandingGear>("gear");
    const std::size_t im = ac.index("main_rotor");
    const std::size_t ig = ac.index("gear");
    const double T = trim_period(ac.executive());
    const int N = 256;
    double t = 0.0;
    for (int k = 0; k < N; ++k) {
        const SimContext ctx = ac.context(t, x, c, env);
        p.power += mr.evaluate(ctx, ac.states(x, im)).power / N;
        for (std::size_t g = 0; g < gear.config().gears.size(); ++g) {
            p.gear_load =
                std::max(p.gear_load, gear.gear_state(ctx, ac.states(x, ig), g).normal_load);
        }
        ac.hold(t, x, c, env, T / N, 1);
    }
}

// One trim at airspeed [m/s] and CG altitude [m], with the ground `wheel`
// [m] below the bottom of the left main wheel (USAAEFA's measure; NaN: no
// ground). The ground height depends on the trimmed attitude, so the trim is
// repeated with the attitude it found until the wheel height holds to 1 mm.
inline Point trim_near_ground(const Uh60aHarness& ac, double airspeed, double altitude,
                              double wheel, const TrimResult* start) {
    const Executive& ex = ac.executive();
    const GearConfig& left = ac.get<LandingGear>("gear").config().gears.front();
    if (left.name != "left_main") {
        throw std::runtime_error("expected the left main gear first");
    }
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    TrimCondition c;
    c.airspeed = airspeed;
    c.altitude = altitude;
    Point p;
    double pitch = start != nullptr ? start->pitch : 0.0;
    double roll = start != nullptr ? start->roll : 0.0;
    std::vector<double> x;
    for (int it = 0; it < 6; ++it) {
        const Quat q = quaternion(EulerAngles{roll, pitch, 0.0});
        const double wheel_bottom = altitude - (q * left.axle_b).z() - left.tire.radius;
        const double ground = std::isnan(wheel) ? kNaN : wheel_bottom - wheel;
        const bool settled =
            it > 0 && (std::isnan(wheel) || std::abs(ground - c.ground_height) < 1e-3);
        c.ground_height = ground;
        if (settled) {
            break;
        }
        p.trim = solve_trim(ex, c, it > 0 ? &p.trim : start, x);
        if (!p.trim.converged) {
            return p;
        }
        pitch = p.trim.pitch;
        roll = p.trim.roll;
    }
    p.converged = p.trim.converged;
    revolution(ac, x, p.trim.controls, trim_environment(c), p);
    const Quat q = quaternion(EulerAngles{roll, pitch, 0.0});
    const double hub = altitude - (q * mr.config().hub_position_b).z();
    p.hub_height = (hub - c.ground_height) / mr.config().radius;
    return p;
}

// USAAEFA's UH-60A tethered hover fits, C_P against C_T (ADA180181 App. E;
// coefficients as printed, DATA_GAPS.md). C_P is engine power.
// Fig. 2, 2 ft wheel height.
inline double usaaefa_cp_2ft(double ct) {
    return 8.06470e-05 + 7.55353e-01 * std::pow(ct, 1.5) + 3.96385e+01 * ct * ct * ct;
}
// Fig. 3, 100 ft wheel height: two segments meeting at C_T = 0.0070.
inline double usaaefa_cp_100ft(double ct) {
    if (ct < 0.0070) {
        return 8.06470e-05 + 1.02707 * std::pow(ct, 1.5);
    }
    return 2.57292e-04 + 4.23840e-01 * std::pow(ct, 1.5) + 5.15000e+02 * ct * ct * ct;
}

// The hover power ratio P(2 ft)/P(100 ft) at one density altitude [m], ISA,
// and the weight coefficient C_W there. At 2 ft Eyas's tail wheel grazes the
// ground (D-050), so P(2 ft) is extrapolated linearly in wheel height from
// 2.5 and 3 ft, where no wheel touches. `ok` is false if a trim didn't
// converge or a wheel carried load.
struct HoverRatio {
    bool ok = false;
    double cw = 0.0;
    double ratio = 0.0;  // extrapolated to 2 ft
    double ratio_2p5 = 0.0;
};
inline HoverRatio hover_ratio(const Uh60aHarness& ac, double altitude) {
    const auto& mr = ac.get<BladeElementRotor>("main_rotor");
    const Point p25 = trim_near_ground(ac, 0.0, altitude, 2.5 * units::kFoot, nullptr);
    const Point p3 = trim_near_ground(ac, 0.0, altitude, 3.0 * units::kFoot, &p25.trim);
    const Point p100 = trim_near_ground(ac, 0.0, altitude, 100.0 * units::kFoot, &p3.trim);
    HoverRatio r;
    r.ok = p25.converged && p3.converged && p100.converged && p25.gear_load == 0.0 &&
           p3.gear_load == 0.0 && p100.gear_load == 0.0;
    const double rho = isa_atmosphere(altitude, 0.0).density;
    const double vt = mr.tip_speed();
    r.cw = gross_mass(ac) * units::kStandardGravity / (rho * mr.disk_area() * vt * vt);
    r.ratio = (2.0 * p25.power - p3.power) / p100.power;
    r.ratio_2p5 = p25.power / p100.power;
    return r;
}

}  // namespace eyas::test::m9
