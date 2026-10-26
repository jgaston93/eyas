// Writes the M9 Stage B1′ data (Ballin's T700, D-054) to CSV for VALIDATION.md
// (tools/plot_m9_ballin.py):
// - m9_ballin_table_b1.csv: the UH-60A on Ballin's engines at Table B.1's
//   loading (data/uh60a/engine_ballin/uh60a_table_b1.yaml; 16 825 lbm, CG FS
//   355.0, WL 248.2 in), sea level ISA, bare, trimmed at Table B.1's
//   conditions (hover; 80 kt level; 80 kt at -1000 ft/min, gamma -7.08 deg):
//   the main rotor's torque, the engine's torque, power, fuel flow, speeds,
//   pressures and temperatures, beside Table B.1's (TM-100991 p. 53);
// - m9_ballin_step.csv: from the sea-level and 5250 ft hover trims (GenHel's
//   16 638 lbm), a 0.05 collective step held for 10 s, the body free, no
//   stability augmentation: on Ballin's engines, on GenHel's (D-053), and with
//   the rotor speed held (an ideal drive): rotor speed, the engines' torque
//   referred to the rotor, NG, the fuel flow and the yaw rate, every 0.05 s;
// - m9_ballin_lever.csv (Stage B2a, D-055): the UH-60A on Ballin's engines
//   (GenHel's loading) trimmed at 80 kt, sea level, SAS and FPS on; at 2 s
//   engine 2's power lever goes to the idle stop, at 22 s back to fly, the
//   other controls held: rotor speed, and per engine the turbine's torque,
//   NG, the fuel ratio selected (HMUSEL) with the acceleration limit (WFPAC),
//   the idle and deceleration floors (WFIDM, WFPDC) and the demand (WFPDM), and
//   the clutch's slip, every 0.05 s.
// - m9_ballin_failure.csv (Stage B2b, D-056): from the same trim, engine 2's
//   fuel off at 2 s (case one_out); and both off at 2 s with the collective
//   lowered from 3.5 s to 4.5 s to the 80 kt autorotation trim's (case
//   both_out), the cyclic and pedal held: rotor speed, rate of climb,
//   airspeed, and per engine the turbine's torque, NG, the slip and whether
//   it is out, every 0.05 s;
// - m9_ballin_autorotation.csv (Stage B2b): the UH-60A at Table B.1's loading,
//   sea level, bare, at 60-120 kt: the level-flight power, and the descent
//   rate at which the rotor needs no engine power at the governed speed (the
//   clutches' transition in the powered descent, by bisection; there the
//   aircraft trims with both engines out too), beside Johnson's
//   V_d = P_level/W (2013 Eq. 5.19, §7.2.3), with the powered descent's power
//   on the way;
// - m9_ballin_climbs.csv (Stage B2b): Ballin's climbs and descents (TM-88360
//   Figs. B2-B4, 60, 100 and 137.5 kt, -3000 to +3000 ft/min, each from the
//   previous trim) on Ballin's engines and with the rotor speed held:
//   collective, rotor speed and the clutches' slip.
// - m9_ballin_tail_rotor.csv (D-058): the UH-60A at Table B.1's loading, sea
//   level, bare, in hover and level flight to 160 kt, with the tail rotor
//   turning either way (its sense is a placeholder): the main and tail
//   rotors' power, the engines', and the trim controls.
//
// Usage: eyas_m9_ballin_engine_cases <output-dir>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "analysis/free_flight.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"
#include "engine/ballin_t700.hpp"
#include "engine/genhel_t700.hpp"
#include "env/atmosphere.hpp"
#include "rotor/bailey_rotor.hpp"
#include "sim/build.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m7c_ballin.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace {

using namespace eyas;

constexpr double kFtLbf = units::kFoot * units::kPoundForce;  // [N·m]
constexpr double kLbmHr = units::kPoundMass / 3600.0;         // [kg/s]
constexpr double kHp = 550.0 * kFtLbf;                        // [W]

std::string ballin_config(const std::string& name) {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/engine_ballin/" + name;
}

Executive build(const Config& cfg) {
    return build_executive(cfg, read_origin(cfg).origin().altitude);
}

template <class E>
std::pair<const E*, std::size_t> find(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (const auto* e = dynamic_cast<const E*>(&ex.component(i))) {
            return {e, ex.component_offset(i)};
        }
    }
    return {nullptr, 0};
}

// The tail rotor's torque at its hub [N·m] and its speed [rad/s] at state x
// (D-058): the coupled evaluation first, so that it reads this state's
// main-rotor wash and airframe wake from the store.
std::pair<double, double> tail_rotor_torque(const Executive& ex, const std::vector<double>& x,
                                            const Controls& u, const Environment& env) {
    std::vector<double> xdot(x.size());
    (void)ex.derivatives(0.0, x, u, env, xdot);
    const auto [tail, at] = find<BaileyRotor>(ex);
    const SimContext ctx = ex.context(0.0, x, u, env);
    const BaileyRotorLoads l =
        tail->evaluate(ctx, std::span<const double>(x).subspan(at, tail->num_states()));
    return {l.torque, tail->rotor_speed(ctx)};
}

void write_table_b1(const std::filesystem::path& dir) {
    const Executive ex = build(Config::from_file(ballin_config("uh60a_table_b1.yaml")));
    const auto [engine, offset] = find<BallinT700>(ex);
    struct Case {
        const char* name;
        double kt, gamma_deg;
        // Table B.1: required torque (hub), engine torque (shaft), hp per engine, fuel per
        // engine, NG, P_s3, P41, T41, P45, T45.
        std::array<double, 10> b1;
    };
    const std::array<Case, 3> cases{{
        {"hover",
         0.0,
         0.0,
         {32865., 229.0, 911.1, 476.3, 41638., 176.34, 174.28, 2292., 37.42, 1632.}},
        {"80kt",
         80.0,
         0.0,
         {20747., 138.9, 552.6, 349.3, 39768., 142.13, 140.26, 2102., 30.66, 1501.}},
        {"80kt_descent",
         80.0,
         -7.08,
         {10792., 76.06, 302.6, 267.7, 38072., 114.27, 112.77, 1982., 25.54, 1424.}},
    }};
    std::ofstream f(dir / "m9_ballin_table_b1.csv");
    f.precision(8);
    f << "# The UH-60A on Ballin's T700s at Table B.1's loading and conditions (D-054): Eyas "
         "and Ballin, NASA TM-100991 Table B.1 (p. 53)\n";
    f << "case,quantity,eyas,ballin\n";
    std::vector<double> x;
    for (const Case& c : cases) {
        TrimCondition tc;
        tc.airspeed = c.kt * units::kKnot;
        tc.flight_path_angle = c.gamma_deg * units::kDegree;
        const TrimResult r = solve_trim(ex, tc, nullptr, x);
        if (!r.converged) {
            throw std::runtime_error(std::string("Table B.1 trim did not converge: ") + c.name);
        }
        const double omega = x[ex.shaft_offset(0) + 1];
        const std::span<const double> xe(x.data() + offset, BallinT700::kEngineStates);
        const AirState air = isa_atmosphere(0.0, 0.0);
        const BallinT700Signals s =
            engine->signals(xe, {omega, r.controls.collective, 1.0, engine->np_reference(),
                                 air.pressure, air.temperature});
        const BallinT700Config& k = engine->config();
        // The engines less the drive train and the tail rotor (D-058): the main
        // rotor's demand, Table B.1's "required torque".
        const auto [q_tr, omega_tr] = tail_rotor_torque(ex, x, r.controls, trim_environment(tc));
        const double hub =
            k.engines * s.q_engine - engine->drive_train_load(omega) - q_tr * omega_tr / omega;
        const std::array<std::pair<const char*, double>, 14> rows{{
            {"collective_pct", 100.0 * r.controls.collective},
            {"rotor_speed_rad_s", omega},
            {"required_torque_hub_ftlbf", hub / kFtLbf},
            {"engine_torque_shaft_ftlbf", s.gas.q_pt / kFtLbf},
            {"hp_per_engine", s.gas.q_pt * k.gear * omega / kHp},
            {"fuel_per_engine_lbm_hr", s.fuel / kLbmHr},
            {"ng_rpm", xe[BallinT700::kNg] * k.ng_des / units::kRpm},
            {"ps3_psia", s.gas.ps3 / units::kPsi},
            {"p41_psia", s.gas.p41 / units::kPsi},
            {"t41_degr", s.gas.t41 / units::kRankine},
            {"p45_psia", s.gas.p45 / units::kPsi},
            {"t45_degr", s.gas.t45 / units::kRankine},
            {"tail_rotor_hp", q_tr * omega_tr / kHp},
            {"tail_over_main_rotor_power", q_tr * omega_tr / (hub * omega)},
        }};
        for (std::size_t i = 0; i < rows.size(); ++i) {
            const double b = i < 2 || i >= 12 ? std::nan("") : c.b1[i - 2];
            f << c.name << ',' << rows[i].first << ',' << rows[i].second << ',' << b << '\n';
        }
        std::cout << c.name << ": " << r.iterations << " iterations, engine torque "
                  << s.gas.q_pt / kFtLbf << " ft*lbf\n";
    }
}

// A collective step from the hover trim at `feet`, held for `seconds`.
void write_step(std::ofstream& f, const Executive& ex, const std::string& drive, double feet,
                double step, double seconds) {
    TrimCondition c;
    c.altitude = feet * units::kFoot;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    if (!r.converged) {
        throw std::runtime_error("hover trim did not converge: " + drive);
    }
    const auto [ballin, ballin_at] = find<BallinT700>(ex);
    const auto [genhel, genhel_at] = find<GenHelT700>(ex);
    const Environment env = trim_environment(c);
    const double dt = trim_period(ex, r) / 64.0;
    Controls u = r.controls;
    u.collective += step;
    Rk4 rk4(x.size());
    const double governed = ex.shafts()[0].speed;
    const AirState air = isa_atmosphere(feet * units::kFoot, 0.0);
    const int every = static_cast<int>(std::lround(0.05 / dt));
    const int steps = static_cast<int>(std::lround(seconds / dt));
    for (int n = 0; n <= steps; ++n) {
        if (n % every == 0) {
            const double omega = x[ex.shaft_offset(0) + 1];
            const BodyState b = unpack_body(x);
            double q_e = 0.0;
            double n_g = 0.0;
            double fuel = 0.0;
            if (ballin != nullptr) {
                const std::span<const double> xe(x.data() + ballin_at, BallinT700::kEngineStates);
                const BallinT700Signals s =
                    ballin->signals(xe, {omega, u.collective, 1.0, ballin->np_reference(),
                                         air.pressure, air.temperature});
                q_e = ballin->config().engines * ballin->config().gear * s.gas.q_pt;
                n_g = xe[BallinT700::kNg];
                fuel = s.fuel * ballin->config().engines;
            } else if (genhel != nullptr) {
                const std::span<const double> xe(x.data() + genhel_at, GenHelT700::kNumStates);
                const GenHelT700Signals s = genhel->signals(xe, omega, u.collective, 0.0);
                q_e = s.q_engine;
                n_g = s.n_gg;
                fuel = s.fuel * genhel->config().engines;
            }
            f << feet << ',' << drive << ',' << n * dt << ',' << 100.0 * omega / governed << ','
              << q_e / kFtLbf << ',' << 100.0 * n_g << ',' << fuel / kLbmHr << ','
              << b.omega_b.z() / units::kDegree << '\n';
        }
        if (n < steps) {
            (void)free_step(ex, rk4, [&](double) { return u; }, env, n * dt, dt, x);
        }
    }
}

// Engine 2's lever to idle and back at 80 kt (D-055).
void write_lever(const std::filesystem::path& dir, const Executive& ex) {
    TrimCondition c;
    c.airspeed = 80.0 * units::kKnot;
    c.afcs.sas_digital = true;
    c.afcs.sas_analog = true;
    c.afcs.fps = true;
    std::vector<double> x;
    const TrimResult r = solve_trim(ex, c, nullptr, x);
    if (!r.converged) {
        throw std::runtime_error("80 kt trim did not converge");
    }
    const auto [engine, at] = find<BallinT700>(ex);
    const Environment env = trim_environment(c);
    const double dt = trim_period(ex, r) / 64.0;
    Rk4 rk4(x.size());
    const double governed = ex.shafts()[0].speed;
    const AirState air = isa_atmosphere(0.0, 0.0);
    const double ratio = kLbmHr / units::kPsi;  // lbm*in^2/(lbf*hr)
    const auto controls = [&](double t) {
        Controls u = r.controls;
        u.engines[1].power_lever = (t >= 2.0 && t < 22.0) ? 0.0 : 1.0;
        return u;
    };
    std::ofstream f(dir / "m9_ballin_lever.csv");
    f.precision(8);
    f << "# UH-60A on Ballin's T700s, 80 kt, sea level, SAS and FPS on: engine 2's power lever "
         "to idle at 2 s, back to fly at 22 s (D-055)\n";
    f << "time_s,rotor_speed_pct,engine,lever,torque_ftlbf,n_g_pct,hmusel,wfpac,wfidm,wfpdc,wfpdm,"
         "slip_rad_s\n";
    const int every = static_cast<int>(std::lround(0.05 / dt));
    const int steps = static_cast<int>(std::lround(40.0 / dt));
    double low = governed;
    for (int n = 0; n <= steps; ++n) {
        const double t = n * dt;
        const double omega = x[ex.shaft_offset(0) + 1];
        low = std::min(low, omega);
        if (n % every == 0) {
            const Controls u = controls(t);
            for (int e = 0; e < engine->config().engines; ++e) {
                const std::span<const double> xe(
                    x.data() + at + static_cast<std::size_t>(e) * BallinT700::kEngineStates,
                    BallinT700::kEngineStates);
                const double lever = u.engines[static_cast<std::size_t>(e)].power_lever;
                const BallinT700Signals s =
                    engine->signals(xe, {omega, u.collective, lever, engine->np_reference(),
                                         air.pressure, air.temperature});
                f << t << ',' << 100.0 * omega / governed << ',' << e + 1 << ',' << lever << ','
                  << s.gas.q_pt / kFtLbf << ',' << 100.0 * xe[BallinT700::kNg] << ','
                  << s.hmusel / ratio << ',' << s.wfpac / ratio << ',' << s.wfidm / ratio << ','
                  << s.wfpdc / ratio << ',' << s.wfpdm / ratio << ',' << xe[BallinT700::kSlip]
                  << '\n';
            }
        }
        if (n < steps) {
            (void)free_step(ex, rk4, controls, env, t, dt, x);
        }
    }
    std::cout << "lever cut: lowest rotor speed " << 100.0 * low / governed << " %\n";
}

// A trim continued from the first of `starts` (the nearest) that converges:
// a small step can fail where a bigger one, from further back, doesn't. Then
// a cold start, kept only if its attitude continues the sweep's (within 3
// degrees: a cold start can land on another branch of the trim equations).
TrimResult continued_trim(const Executive& ex, const TrimCondition& c,
                          const std::vector<const TrimResult*>& starts, std::vector<double>& x) {
    TrimResult first;
    for (std::size_t k = 0; k < starts.size(); ++k) {
        const TrimResult r = solve_trim(ex, c, starts[k], x);
        if (r.converged) {
            return r;
        }
        if (k == 0) {
            first = r;
        }
    }
    std::vector<double> y;
    const TrimResult cold = solve_trim(ex, c, nullptr, y);
    const double tol = 3.0 * units::kDegree;
    if (cold.converged && std::abs(cold.pitch - starts[0]->pitch) < tol &&
        std::abs(cold.roll - starts[0]->roll) < tol) {
        x = y;
        return cold;
    }
    return first;
}

TrimResult continued_trim(const Executive& ex, const TrimCondition& c, const TrimResult& start,
                          std::vector<double>& x) {
    return continued_trim(ex, c, std::vector<const TrimResult*>{&start}, x);
}

// The signals of engine e at state x (the trim's or a free flight's).
BallinT700Signals engine_signals(const Executive& ex, const BallinT700& engine, std::size_t at,
                                 const std::vector<double>& x, const Controls& u,
                                 const Environment& env, int e) {
    const SimContext ctx = ex.context(0.0, x, u, env);
    const std::span<const double> xe(
        x.data() + at + static_cast<std::size_t>(e) * BallinT700::kEngineStates,
        BallinT700::kEngineStates);
    return engine.signals(xe, engine.inputs(ctx, e));
}

void write_failure(const std::filesystem::path& dir, const Executive& ex) {
    TrimCondition c;
    c.airspeed = 80.0 * units::kKnot;
    c.afcs.sas_digital = true;
    c.afcs.sas_analog = true;
    c.afcs.fps = true;
    std::vector<double> x0;
    const TrimResult r = solve_trim(ex, c, nullptr, x0);
    if (!r.converged) {
        throw std::runtime_error("80 kt trim did not converge");
    }
    // The autorotation collective at 80 kt (the rate the powered descent's
    // clutches let go at; write_autorotation), from the powered descents.
    double collective_auto = r.controls.collective;
    {
        TrimCondition d = c;
        TrimResult prev = r;
        std::vector<double> y;
        for (double fpm = -500.0; fpm >= -4000.0; fpm -= 250.0) {
            d.flight_path_angle = std::asin(fpm * units::kFoot / 60.0 / d.airspeed);
            const TrimResult next = continued_trim(ex, d, prev, y);
            if (!next.converged) {
                break;
            }
            prev = next;
            collective_auto = next.controls.collective;
            if (y[find<BallinT700>(ex).second + BallinT700::kSlip] > 0.0) {
                break;
            }
        }
    }
    const auto [engine, at] = find<BallinT700>(ex);
    const Environment env = trim_environment(c);
    const double dt = trim_period(ex, r) / 64.0;
    const double governed = ex.shafts()[0].speed;
    std::ofstream f(dir / "m9_ballin_failure.csv");
    f.precision(8);
    f << "# UH-60A on Ballin's T700s, 80 kt, sea level, SAS and FPS on: engine 2's fuel off at 2 s "
         "(one_out); both off at 2 s, the collective lowered 3.5-4.5 s to "
      << collective_auto << " (both_out) (D-056)\n";
    f << "case,time_s,rotor_speed_pct,climb_rate_m_s,airspeed_kt,collective,engine,torque_ftlbf,"
         "n_g_pct,slip_rad_s,out\n";
    for (const bool both : {false, true}) {
        std::vector<double> x = x0;
        Rk4 rk4(x.size());
        const auto controls = [&](double t) {
            Controls u = r.controls;
            if (t >= 2.0) {
                u.engines[1].fuel_on = false;
                u.engines[0].fuel_on = !both;
            }
            if (both) {
                const double k = std::clamp(t - 3.5, 0.0, 1.0);
                u.collective =
                    r.controls.collective + k * (collective_auto - r.controls.collective);
            }
            return u;
        };
        const int every = static_cast<int>(std::lround(0.05 / dt));
        const int steps = static_cast<int>(std::lround(60.0 / dt));
        double low = governed;
        double decay = 0.0;  // the rotor's deceleration just before the collective moves [1/s]
        for (int n = 0; n <= steps; ++n) {
            const double t = n * dt;
            const double omega = x[ex.shaft_offset(0) + 1];
            low = std::min(low, omega);
            const Controls u = controls(t);
            if (n % every == 0) {
                const BodyState b = unpack_body(x);
                const Vec3 v_n = b.q_nb.toRotationMatrix() * b.vel_b;
                for (int e = 0; e < engine->config().engines; ++e) {
                    const BallinT700Signals s = engine_signals(ex, *engine, at, x, u, env, e);
                    const std::size_t o =
                        at + static_cast<std::size_t>(e) * BallinT700::kEngineStates;
                    f << (both ? "both_out" : "one_out") << ',' << t << ','
                      << 100.0 * omega / governed << ',' << -v_n.z() << ','
                      << b.vel_b.norm() / units::kKnot << ',' << u.collective << ',' << e + 1 << ','
                      << s.gas.q_pt / kFtLbf << ',' << 100.0 * x[o + BallinT700::kNg] << ','
                      << x[o + BallinT700::kSlip] << ',' << (s.out ? 1 : 0) << '\n';
                }
            }
            if (both && n > 0 && std::abs(t - 3.45) < 0.5 * dt) {
                std::vector<double> xd(x.size());
                (void)ex.derivatives(t, x, u, env, xd);
                decay = xd[ex.shaft_offset(0) + 1] / governed;
            }
            if (n < steps) {
                (void)free_step(ex, rk4, controls, env, t, dt, x);
            }
        }
        std::cout << (both ? "both" : "one") << " out: lowest rotor speed "
                  << 100.0 * low / governed << " %, end "
                  << 100.0 * x[ex.shaft_offset(0) + 1] / governed << " %";
        if (both) {
            std::cout << ", decaying " << 100.0 * decay << " %/s at 3.45 s";
        }
        std::cout << '\n';
    }
}

void write_autorotation(const std::filesystem::path& dir) {
    const Executive ex = build(Config::from_file(ballin_config("uh60a_table_b1.yaml")));
    const auto [engine, at] = find<BallinT700>(ex);
    // Table B.1's aircraft weight (TM-100991 p. 53), the gross weight the
    // config splits between the airframe, the blades and the gear (D-002).
    const double weight = 16825.0 * units::kPoundMass * units::kStandardGravity;
    const double governed = ex.shafts()[0].speed;
    const std::size_t omega_at = ex.shaft_offset(0) + 1;
    std::ofstream f(dir / "m9_ballin_autorotation.csv");
    f.precision(8);
    f << "# UH-60A on Ballin's T700s, Table B.1's loading (16 825 lbm), sea level, bare: powered "
         "descents to the rate where the rotor needs no engine power at the governed speed, "
         "against Johnson's V_d = P_level/W (D-056)\n";
    f << "airspeed_kt,kind,descent_ft_min,engine_power_kw,collective,rotor_speed_pct,slip_rad_s,"
         "converged\n";
    for (const double kt : {60.0, 80.0, 100.0, 120.0}) {
        TrimCondition c;
        c.airspeed = kt * units::kKnot;
        std::vector<double> x;
        const auto power = [&](const TrimResult& r) {
            const Environment env = trim_environment(c);
            double p = 0.0;
            for (int e = 0; e < engine->config().engines; ++e) {
                p += engine_signals(ex, *engine, at, x, r.controls, env, e).q_engine * x[omega_at];
            }
            return p;
        };
        const auto row = [&](const char* kind, double fpm, const TrimResult& r) {
            f << kt << ',' << kind << ',' << -fpm << ',' << power(r) / 1e3 << ','
              << r.controls.collective << ',' << 100.0 * x[omega_at] / governed << ','
              << x[at + BallinT700::kSlip] << ',' << (r.converged ? 1 : 0) << '\n';
        };
        const auto trim = [&](double fpm, const std::vector<const TrimResult*>& starts) {
            c.flight_path_angle = std::asin(fpm * units::kFoot / 60.0 / c.airspeed);
            return starts.empty() ? solve_trim(ex, c, nullptr, x)
                                  : continued_trim(ex, c, starts, x);
        };
        TrimResult engaged = trim(0.0, {});
        if (!engaged.converged) {
            throw std::runtime_error("level trim did not converge");
        }
        const double p_level = power(engaged);
        row("powered", 0.0, engaged);
        // Down in 250 ft/min steps until the clutches let go.
        double fpm_engaged = 0.0;
        double fpm_free = 0.0;
        TrimResult free;
        std::vector<TrimResult> history{engaged};  // the engaged trims so far, nearest last
        for (double fpm = -250.0; fpm >= -6000.0; fpm -= 250.0) {
            std::vector<const TrimResult*> starts;
            for (std::size_t k = 0; k < history.size() && k < 4; ++k) {
                starts.push_back(&history[history.size() - 1 - k]);
            }
            const TrimResult r = trim(fpm, starts);
            if (!r.converged) {
                break;
            }
            row("powered", fpm, r);
            if (x[at + BallinT700::kSlip] > 0.0) {
                fpm_free = fpm;
                free = r;
                break;
            }
            engaged = r;
            history.push_back(r);
            fpm_engaged = fpm;
        }
        if (fpm_free == 0.0) {
            std::cout << kt << " kt: no overrun found\n";
            continue;
        }
        // Bisection on the transition, each trim from the engaged side.
        for (int k = 0; k < 9; ++k) {
            const double mid = 0.5 * (fpm_engaged + fpm_free);
            const TrimResult r = trim(mid, {&engaged, &free});
            if (!r.converged) {
                break;
            }
            if (x[at + BallinT700::kSlip] > 0.0) {
                fpm_free = mid;
                free = r;
            } else {
                fpm_engaged = mid;
                engaged = r;
            }
        }
        const double fpm_d = 0.5 * (fpm_engaged + fpm_free);
        const TrimResult at_d = trim(fpm_free, {&free});
        row("transition", fpm_free, at_d);
        // With both engines out there (the same rotor state, D-056 item 5).
        c.engines[0].fuel_on = c.engines[1].fuel_on = false;
        const TrimResult dead = trim(fpm_free, {&at_d});
        row("both_out", fpm_free, dead);
        c.engines[0].fuel_on = c.engines[1].fuel_on = true;
        const double vd = -fpm_d * units::kFoot / 60.0;
        std::cout << kt << " kt: P_level " << p_level / 1e3 << " kW, P/W "
                  << p_level / weight / units::kFoot * 60.0 << " ft/min; no power at " << -fpm_d
                  << " ft/min (" << vd / (p_level / weight) << " of P/W); both out there: "
                  << (dead.converged ? "converged" : "NOT converged") << ", collective "
                  << dead.controls.collective << " vs " << at_d.controls.collective << '\n';
    }
}

void write_climbs(const std::filesystem::path& dir) {
    namespace bt = test::ballin;
    const Executive engines = build(Config::from_file(ballin_config("uh60a_ballin.yaml")));
    const Executive held = build(bt::ideal_shaft_config(ballin_config("uh60a_ballin.yaml")));
    const auto [engine, at] = find<BallinT700>(engines);
    std::ofstream f(dir / "m9_ballin_climbs.csv");
    f.precision(8);
    f << "# UH-60A, Ballin's B1-B4 loading, 5250 ft ISA, Figs. B2-B4 conditions, each trim from "
         "the previous one: on Ballin's T700s and with the rotor speed held (D-056)\n";
    f << "airspeed_kt,climb_ft_min,drive,converged,collective_pct,rotor_speed_pct,slip_rad_s\n";
    for (const double kt : {60.0, 100.0, 137.5}) {
        for (const bool real : {true, false}) {
            const Executive& ex = real ? engines : held;
            const double governed = ex.shafts()[0].speed;
            // From level flight up, then down, each from its neighbour.
            for (const double sign : {1.0, -1.0}) {
                bt::Point prev = bt::genhel_trim(ex, kt, 0.0, nullptr);
                for (int k = sign > 0.0 ? 0 : 1; k <= 6; ++k) {
                    const double fpm = sign * 500.0 * k;
                    const bt::Point p = k == 0 ? prev : bt::genhel_trim(ex, kt, fpm, &prev);
                    const double omega =
                        p.trim.component_states.empty()
                            ? 0.0
                            : p.trim.component_states[ex.shaft_offset(0) + 1 - body_layout::kSize];
                    const double slip = real && !p.x.empty() ? p.x[at + BallinT700::kSlip] : 0.0;
                    f << kt << ',' << fpm << ',' << (real ? "engines" : "held") << ','
                      << (p.converged ? 1 : 0) << ',' << bt::collective_pct(p.trim.controls) << ','
                      << 100.0 * omega / governed << ',' << slip << '\n';
                    if (!p.converged) {
                        break;
                    }
                    prev = p;
                }
            }
            std::cout << kt << " kt on " << (real ? "the engines" : "a held shaft") << " done\n";
        }
    }
}

void write_tail_rotor(const std::filesystem::path& dir) {
    namespace bt = test::ballin;
    const std::string text = bt::config_text(ballin_config("uh60a_table_b1.yaml"));
    const std::string ccw = "direction: !placeholder counter_clockwise";
    const std::size_t at = text.find(ccw);
    if (at == std::string::npos) {
        throw std::runtime_error("the tail rotor's direction is not the expected placeholder");
    }
    std::string cw_text = text;
    cw_text.replace(at, ccw.size(), "direction: clockwise");
    std::ofstream f(dir / "m9_ballin_tail_rotor.csv");
    f.precision(8);
    f << "# UH-60A on Ballin's T700s, Table B.1's loading, sea level ISA, bare: the tail rotor's "
         "torque (D-058), turning either way\n";
    f << "direction,airspeed_kt,converged,main_rotor_hp,tail_rotor_hp,tail_over_main,"
         "engines_hp,collective_pct,pedal_pct,lon_pct,lat_pct\n";
    for (const bool clockwise : {false, true}) {
        const Executive ex = build(Config::from_string(clockwise ? cw_text : text, "tail rotor"));
        const auto [engine, offset] = find<BallinT700>(ex);
        std::vector<double> x;
        for (const double kt : {0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0, 160.0}) {
            TrimCondition tc;
            tc.airspeed = kt * units::kKnot;
            const TrimResult r = solve_trim(ex, tc, nullptr, x);
            f << (clockwise ? "clockwise" : "counter_clockwise") << ',' << kt << ','
              << (r.converged ? 1 : 0);
            if (!r.converged) {
                f << ",,,,,,,,\n";
                continue;
            }
            const double omega = x[ex.shaft_offset(0) + 1];
            const std::span<const double> xe(x.data() + offset, BallinT700::kEngineStates);
            const AirState air = isa_atmosphere(0.0, 0.0);
            const BallinT700Signals s =
                engine->signals(xe, {omega, r.controls.collective, 1.0, engine->np_reference(),
                                     air.pressure, air.temperature});
            const double engines = engine->config().engines * s.q_engine * omega;
            const auto [q_tr, omega_tr] =
                tail_rotor_torque(ex, x, r.controls, trim_environment(tc));
            const double tail = q_tr * omega_tr;
            const double main = engines - engine->drive_train_load(omega) * omega - tail;
            f << ',' << main / kHp << ',' << tail / kHp << ',' << tail / main << ','
              << engines / kHp << ',' << bt::collective_pct(r.controls) << ','
              << bt::pedal_pct(r.controls) << ',' << bt::lon_pct(r.controls) << ','
              << bt::lat_pct(r.controls) << '\n';
        }
        std::cout << "tail rotor turning " << (clockwise ? "clockwise" : "counter-clockwise")
                  << " done\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: eyas_m9_ballin_engine_cases <output-dir>\n";
        return 2;
    }
    try {
        const std::filesystem::path dir(argv[1]);
        std::filesystem::create_directories(dir);
        write_table_b1(dir);
        write_tail_rotor(dir);
        const Executive ballin = build(Config::from_file(ballin_config("uh60a.yaml")));
        const Executive genhel = build(Config::from_file(test::uh60a_config_path()));
        const Executive held = build(test::ballin::ideal_shaft_config());
        std::ofstream f(dir / "m9_ballin_step.csv");
        f.precision(10);
        f << "# UH-60A, 0.05 collective step from the hover trim, 10 s, no stability "
             "augmentation: on Ballin's T700s, on GenHel's, and with the rotor speed held\n";
        f << "altitude_ft,drive,time_s,rotor_speed_pct,engine_torque_ftlbf,n_g_pct,"
             "fuel_lbm_hr,yaw_rate_deg_s\n";
        for (const double feet : {0.0, 5250.0}) {
            write_step(f, ballin, "ballin", feet, 0.05, 10.0);
            write_step(f, genhel, "genhel", feet, 0.05, 10.0);
            write_step(f, held, "held", feet, 0.05, 10.0);
            std::cout << "collective step at " << feet << " ft done\n";
        }
        write_lever(dir, ballin);
        write_failure(dir, ballin);
        write_autorotation(dir);
        write_climbs(dir);
    } catch (const std::exception& err) {
        std::cerr << "eyas_m9_ballin_engine_cases: " << err.what() << '\n';
        return 1;
    }
    return 0;
}
