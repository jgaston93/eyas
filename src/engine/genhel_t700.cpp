#include "engine/genhel_t700.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "env/atmosphere.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

// GenHel's dead band (p. 5.6-11): zero for −a < u < b, else u + a or u − b.
double dead_band(double u, double a, double b) noexcept {
    if (u <= -a) {
        return u + a;
    }
    if (u >= b) {
        return u - b;
    }
    return 0.0;
}

// The output of (1 + a s)/(1 + b s) u realized with state z, ż = (u − z)/b.
double lead_lag(double z, double u, double a, double b) noexcept {
    return z + (a / b) * (u - z);
}

// A limited integrator's rate (GenHel's limiters, pp. 5.6-12, 5.6-13): its
// input inside the range and at a limit it pulls off, zero at a limit it
// pushes against. Past a limit (an integration step's overshoot) it returns
// to the limit within kLimitReturn, so that the limit is the only state there
// that stays: everywhere past it the model reads the limit, and trim would
// otherwise face a flat run of equivalent values with no derivative to follow
// (D-053 amendment 2).
constexpr double kLimitReturn = 0.05;  // [s]

double limited_rate(double x, double rate, double lo, double hi) noexcept {
    if (x > hi) {
        return std::min(rate, 0.0) + (hi - x) / kLimitReturn;
    }
    if (x < lo) {
        return std::max(rate, 0.0) + (lo - x) / kLimitReturn;
    }
    if ((x == hi && rate > 0.0) || (x == lo && rate < 0.0)) {
        return 0.0;
    }
    return rate;
}

void require_positive(double v, const char* what) {
    if (!(v > 0.0) || !std::isfinite(v)) {
        throw InitError(std::string("genhel_t700: ") + what + " must be positive and finite");
    }
}

}  // namespace

GenHelT700::GenHelT700(GenHelT700Config config) : config_(std::move(config)) {
    const GenHelT700Config& c = config_;
    if (c.shaft.empty()) {
        throw InitError("genhel_t700: needs a shaft (D-053)");
    }
    if (c.rotor.empty()) {
        throw InitError("genhel_t700: needs the rotor whose demand it reads (D-053)");
    }
    if (c.engines < 1) {
        throw InitError("genhel_t700: needs at least one engine");
    }
    for (const auto& t : {c.torque_ngg, c.lds_rigging, c.lds_cam, c.fuel, c.p3, c.dp3_dwf,
                          c.dqgg_dwf, c.dqpt_dwf, c.dqpt_dnpt}) {
        if (!t) {
            throw InitError("genhel_t700: a map is missing");
        }
    }
    require_positive(c.gear, "gear");
    require_positive(c.power_turbine_100, "the power turbine's 100 % speed");
    require_positive(c.gas_generator_100, "the gas generator's 100 % speed");
    for (const double t :
         {c.k10, c.t2, c.t4, c.T2, c.speed_lag, c.p3_lag, c.valve_lag, c.demand_lag}) {
        require_positive(t, "a lag or the ECU filter's s² term");
    }
    require_positive(c.drive_inertia, "the drive's inertia");
    require_positive(c.engine_inertia, "the engine side's inertia");
    if (!(c.lower <= c.upper) || !(c.band_low < c.band_high) || !(c.qint_limit > 0.0)) {
        throw InitError("genhel_t700: limits out of order");
    }
    if (!(c.gg_inertia >= 0.0) || std::abs(c.pt_axis_b.norm() - 1.0) > 1e-9 ||
        std::abs(c.gg_axis_b.norm() - 1.0) > 1e-9) {
        throw InitError("genhel_t700: spool axes must be unit vectors, inertias non-negative");
    }
    // Table 6.5.1 gives N_GG against the per-engine demand; its inverse is the
    // same broken line read the other way, so the N_GG values must increase.
    const std::span<const double> q = c.torque_ngg->x_axis().breakpoints();
    std::vector<double> n;
    std::vector<double> qv(q.begin(), q.end());
    for (const double qi : q) {
        n.push_back((*c.torque_ngg)(qi));
    }
    for (std::size_t i = 1; i < n.size(); ++i) {
        if (!(n[i] > n[i - 1])) {
            throw InitError("genhel_t700: Table 6.5.1's N_GG must increase with torque");
        }
    }
    ngg_torque_ = std::make_shared<const Table1D>(Axis(std::move(n)), std::move(qv));
}

void GenHelT700::initial_states(std::span<double> x) const {
    equilibrium(0.0, 0.0, 0.0, x);
}

void GenHelT700::steady_states(const SimContext& ctx, std::span<double> x) const {
    x[kSlip] = 0.0;
    // Q_INT integrates 0.7 (K_4 Q_PT − 20) between 0 and 40, so its only
    // steady values are the limits, and the input's sign picks one. Q_PT
    // follows the engine states, which barely move over a revolution, so its
    // value here is the revolution's to well within the model's accuracy
    // (D-053 amendment).
    const double omega = ctx.shafts[static_cast<std::size_t>(shaft_index_)].omega;
    const GenHelT700Signals s = signals(x, omega, ctx.controls.collective, 0.0);
    x[kQint] = config_.k4 * s.q_pt - config_.threshold > 0.0 ? config_.qint_limit : 0.0;
}

void GenHelT700::steady_kinds(std::span<SteadyKind> kinds) const {
    std::fill(kinds.begin(), kinds.end(), SteadyKind::free);
    kinds[kQint] = SteadyKind::held;
    kinds[kSlip] = SteadyKind::held;
}

void GenHelT700::pinned_states(const SimContext& ctx, std::span<const double> x,
                               std::span<unsigned char> pinned) const {
    const GenHelT700Config& c = config_;
    std::fill(pinned.begin(), pinned.end(), static_cast<unsigned char>(0));
    const double omega = ctx.shafts[static_cast<std::size_t>(shaft_index_)].omega;
    // The demand only labels the signals; the clutch reads its lagged state.
    const GenHelT700Signals s = signals(x, omega, ctx.controls.collective, 0.0);
    const auto at_limit = [](double v, double rate, double lo, double hi) {
        return (v >= hi && rate > 0.0) || (v <= lo && rate < 0.0);
    };
    pinned[kSlip] = s.engaged ? 1 : 0;
    pinned[kQint] = at_limit(x[kQint], c.k4 * s.q_pt - c.threshold, 0.0, c.qint_limit) ? 1 : 0;
    pinned[kVss] = at_limit(x[kVss], c.k9 * s.v_s, c.lower, c.upper) ? 1 : 0;
}

double GenHelT700::rotor_demand(const SimContext& ctx) const noexcept {
    return ctx.coupling != nullptr && demand_.valid() ? ctx.coupling->get(demand_) : 0.0;
}

std::vector<std::string> GenHelT700::consumes() const {
    std::vector<std::string> out = {config_.rotor + ".shaft_torque"};
    for (int j = 0; j < 6; ++j) {
        out.push_back(config_.rotor + ".shaft_torque_a" + std::to_string(j));
    }
    return out;
}

void GenHelT700::bind(const CouplingRegistry& registry) {
    demand_ = registry.find(config_.rotor + ".shaft_torque", config_.name);
    for (std::size_t j = 0; j < 6; ++j) {
        demand_a_[j] =
            registry.find(config_.rotor + ".shaft_torque_a" + std::to_string(j), config_.name);
    }
}

void GenHelT700::bind_shafts(std::span<const ShaftInfo> shafts) {
    for (std::size_t k = 0; k < shafts.size(); ++k) {
        if (shafts[k].name == config_.shaft) {
            shaft_index_ = static_cast<int>(k);
            governed_ = shafts[k].speed;
            return;
        }
    }
    throw InitError("genhel_t700 '" + config_.name + "': no shaft named '" + config_.shaft +
                    "' (D-053)");
}

double GenHelT700::lds_speed(double collective) const noexcept {
    // X_C [%] → LDS [deg] → N_GG,LDS (Tables 6.5.2, 6.5.3; p. 5.6-11).
    return lookup(kCam, *config_.lds_cam, lookup(kRigging, *config_.lds_rigging, collective));
}

double GenHelT700::steady_torque(double n_gg) const noexcept {
    return lookup(kTorqueInv, *ngg_torque_, n_gg) / config_.gear;
}

GenHelT700Signals GenHelT700::signals(std::span<const double> x, double omega, double collective,
                                      double rotor_demand) const noexcept {
    const GenHelT700Config& c = config_;
    GenHelT700Signals s;
    s.rotor_demand = rotor_demand;
    s.n_gg = x[kNgg];
    s.fuel = x[kFuel];
    s.engine_speed = omega - std::max(x[kSlip], 0.0);
    // ECU (Fig. 6.1.3; pp. 5.6-11 to 5.6-13).
    s.n_ptd = c.gear * (governed_ - s.engine_speed) / c.power_turbine_100;
    const double v_hg = dead_band(x[kIdb], c.dead_band_a, c.dead_band_b);
    const double v_lg = c.k3 * s.n_ptd;
    const double a = (s.n_ptd < c.band_low || s.n_ptd > c.band_high) ? 40.0 : 0.0;
    const double b = std::clamp(x[kQint], 0.0, c.qint_limit);
    s.high_gain = a >= c.threshold || b >= c.threshold;
    s.v_g = s.high_gain ? v_hg : v_lg;
    s.v_s = lead_lag(x[kVs], s.v_g, c.t1, c.t2);
    s.ecu = c.g7 * x[kIns];
    // Demanded and measured gas-generator speed; the fuel ratio (D-053 item 2).
    s.n_gg_lds = lds_speed(collective);
    s.n_gg_ref = s.n_gg_lds + s.ecu;
    const double error = s.n_gg_ref - x[kNggMeas];
    const double ratio = lookup(kFuelRef, *c.fuel, s.n_gg_ref) / lookup(kP3Ref, *c.p3, s.n_gg_ref) +
                         c.ratio_gain * error;
    s.fuel_command = std::max(ratio * x[kP3Lag] * isa::kSeaLevelPressure, 0.0);
    // The engine on its steady maps plus GenHel's partials (D-053 item 1).
    const double n = s.n_gg;
    const double off = s.fuel - lookup(kFuelMap, *c.fuel, n);
    s.p3 = lookup(kP3Map, *c.p3, n) + lookup(kDp3, *c.dp3_dwf, n) * off;
    s.q_gg = lookup(kDqgg, *c.dqgg_dwf, n) * off;
    s.q_pt = steady_torque(n) + lookup(kDqpt, *c.dqpt_dwf, n) * off +
             lookup(kDqptN, *c.dqpt_dnpt, n) * c.gear * (governed_ - s.engine_speed);
    s.q_engine = c.engines * c.gear * s.q_pt;
    // Clutch (D-053 item 3).
    // FLQREQ is held as the drive's unpowered deceleration FLQREQ/J_d.
    s.engaged = x[kSlip] <= 0.0 && s.q_engine / c.engine_inertia + x[kDemand] > 0.0;
    return s;
}

void GenHelT700::derivatives(const SimContext& ctx, std::span<const double> x,
                             ComponentDerivs& out) const {
    const GenHelT700Config& c = config_;
    const auto k = static_cast<Eigen::Index>(shaft_index_);
    const double omega = ctx.shafts[static_cast<std::size_t>(shaft_index_)].omega;
    const double demand = rotor_demand(ctx);
    const GenHelT700Signals s = signals(x, omega, ctx.controls.collective, demand);
    const auto ksz = static_cast<std::size_t>(shaft_index_);
    const auto set_g = [&](std::size_t row, const Vec3& w) {  // ∂ẋ_row/∂ω̇
        for (std::size_t j = 0; j < 3; ++j) {
            out.G[row * 6 + 3 + j] = w(static_cast<Eigen::Index>(j));
        }
    };

    // Gas generator (p. 5.6-16), relative to the airframe (D-053 item 5).
    const double ngg_abs_rate = c.heat_sink * c.acceleration * s.q_gg;
    out.xdot0[kNgg] = ngg_abs_rate;
    set_g(kNgg, -c.gg_axis_b / c.gas_generator_100);
    // Fuel control (pp. 5.6-14, 5.6-15).
    out.xdot0[kFuel] = (s.fuel_command - x[kFuel]) / c.valve_lag;
    out.xdot0[kP3Lag] = (s.p3 / isa::kSeaLevelPressure - x[kP3Lag]) / c.p3_lag;
    out.xdot0[kNggMeas] = (s.n_gg - x[kNggMeas]) / c.speed_lag;
    // ECU (pp. 5.6-11 to 5.6-13).
    out.xdot0[kIdb] = x[kIdbRate];
    out.xdot0[kIdbRate] = (c.k1 * s.n_ptd - x[kIdb] - c.k11 * x[kIdbRate]) / c.k10;
    out.xdot0[kQint] =
        limited_rate(x[kQint], c.qint_gain * (c.k4 * s.q_pt - c.threshold), 0.0, c.qint_limit);
    out.xdot0[kVs] = (s.v_g - x[kVs]) / c.t2;
    out.xdot0[kVss] = limited_rate(x[kVss], c.k9 * s.v_s, c.lower, c.upper);
    const double v_dm = std::clamp(x[kVss], c.lower, c.upper) + c.k6 * s.v_s;
    out.xdot0[kVdmd] = (v_dm - x[kVdmd]) / c.t4;
    const double v_dmd = lead_lag(x[kVdmd], v_dm, c.t3, c.t4);
    const double motor_in = v_dmd - c.g6 * x[kIns];
    out.xdot0[kMotor] = (motor_in - x[kMotor]) / c.T2;
    out.xdot0[kIns] = c.g5 * c.g4 * lead_lag(x[kMotor], motor_in, c.T3, c.T2);
    // Clutch demand (p. 5.6-11): FLQREQ lags the main rotor's demand with
    // losses; held over J_d.
    // The demand is affine in the body acceleration a, as the rotor's torque
    // is: gravity is in its value at a = 0, so its a terms must come with it
    // (uniform gravity changes nothing on the shaft).
    std::array<double, 6> demand_a{};
    if (ctx.coupling != nullptr) {
        for (std::size_t j = 0; j < 6; ++j) {
            demand_a[j] = ctx.coupling->get(demand_a_[j]);
        }
    }
    const double to_rate = (1.0 + c.loss_factor) / (c.drive_inertia * c.demand_lag);
    out.xdot0[kDemand] =
        ((1.0 + c.loss_factor) * demand / c.drive_inertia - x[kDemand]) / c.demand_lag;
    for (std::size_t j = 0; j < 6; ++j) {
        out.G[kDemand * 6 + j] = to_rate * demand_a[j];
    }

    // The shaft (D-052, D-053): losses always; the engine side's torque and
    // inertia when engaged, else its own speed.
    out.Q0(k) -= c.loss_factor * demand;
    for (std::size_t j = 0; j < 6; ++j) {
        out.Ka(k, static_cast<Eigen::Index>(j)) -= c.loss_factor * demand_a[j];
    }
    const double h_pt = c.engine_inertia / c.gear;  // momentum per unit Ω_e
    if (s.engaged) {
        out.Q0(k) += s.q_engine;
        out.Ks(k, k) -= c.engine_inertia;
        out.Ka.block<1, 3>(k, 3) -= h_pt * c.pt_axis_b.transpose();
        out.Hs.block<3, 1>(3, k) -= h_pt * c.pt_axis_b;
        out.xdot0[kSlip] = 0.0;
    } else {
        // J_TOTE (Ω̇_e + n·ω̇/G_E) = Q_E; ṡ = Ω̇ − Ω̇_e.
        out.xdot0[kSlip] = -s.q_engine / c.engine_inertia;
        set_g(kSlip, c.pt_axis_b / c.gear);
        if (!out.Gs.empty()) {
            out.Gs[kSlip * kMaxShafts + ksz] = 1.0;
        }
        // The airframe: −(J_TOTE/G_E) Ω̇_e n.
        out.load0_b.M -= (s.q_engine / c.gear) * c.pt_axis_b;
        out.H.block<3, 3>(3, 3) += (h_pt / c.gear) * c.pt_axis_b * c.pt_axis_b.transpose();
    }
    // Spools' momentum relative to the airframe: gyroscopic moments, and the
    // gas generators' spin-up reaction −J_GG ω̇_gg n with ω̇_gg = N_GG100 Ṅ.
    const Vec3 h = h_pt * s.engine_speed * c.pt_axis_b +
                   c.gg_inertia * c.gas_generator_100 * s.n_gg * c.gg_axis_b;
    out.load0_b.M -= ctx.body.omega_b.cross(h);
    out.load0_b.M -= c.gg_inertia * c.gas_generator_100 * ngg_abs_rate * c.gg_axis_b;
    out.H.block<3, 3>(3, 3) += c.gg_inertia * c.gg_axis_b * c.gg_axis_b.transpose();
}

void GenHelT700::equilibrium(double engine_torque, double rotor_demand, double collective,
                             std::span<double> x) const noexcept {
    const GenHelT700Config& c = config_;
    std::fill(x.begin(), x.end(), 0.0);
    const double n = lookup(kTorque, *c.torque_ngg, engine_torque / c.engines);
    x[kNgg] = n;
    x[kFuel] = lookup(kFuelMap, *c.fuel, n);
    x[kP3Lag] = lookup(kP3Map, *c.p3, n) / isa::kSeaLevelPressure;
    x[kNggMeas] = n;
    // The ECU holds N_GG,ref = N_GG: V_S = 0, so V_SS = V_DM = V_DMD = G_6 INS
    // with ECU = G_7 INS = N_GG − N_GG,LDS, as far as V_SS's limits allow.
    const double vss = std::clamp(c.g6 * (n - lds_speed(collective)) / c.g7, c.lower, c.upper);
    x[kVss] = vss;
    x[kVdmd] = vss;
    x[kIns] = vss / c.g6;
    const double q_pt = steady_torque(n);
    x[kQint] = c.k4 * q_pt - c.threshold > 0.0 ? c.qint_limit : 0.0;
    x[kDemand] = (1.0 + c.loss_factor) * rotor_demand / c.drive_inertia;
}

// ---- Config ------------------------------------------------------------------

std::unique_ptr<GenHelT700> GenHelT700::from_config(const Config& node) {
    using units::Dimension;
    node.allow_keys({"type", "name", "shaft", "rotor", "engines", "gear", "power_turbine_speed",
                     "gas_generator_speed", "loss_factor", "maps", "ecu", "switch", "fuel_control",
                     "gas_generator", "clutch", "power_turbine_spool", "gas_generator_spool"});
    GenHelT700Config c;
    c.name = node.string("name");
    if (c.name.empty()) {
        node.fail("name", "must not be empty");
    }
    const Config sh = node.child("shaft");
    sh.allow_keys({"name"});
    c.shaft = sh.string("name");
    c.rotor = node.string("rotor");
    const std::int64_t engines = node.integer("engines");
    if (engines < 1 || engines > 8) {
        node.fail("engines", "must be between 1 and 8");
    }
    c.engines = static_cast<int>(engines);
    c.gear = node.number("gear");
    c.power_turbine_100 = node.quantity("power_turbine_speed", Dimension::angular_rate);
    c.gas_generator_100 = node.quantity("gas_generator_speed", Dimension::angular_rate);
    c.loss_factor = node.number("loss_factor");

    const Config maps = node.child("maps");
    maps.allow_keys({"torque", "lds_rigging", "lds_cam", "fuel", "p3", "partials"});
    const auto table = [&](std::string_view key, std::string_view x, Dimension xd,
                           std::string_view v, Dimension vd) {
        return maps.in_context(key, [&] {
            const CsvTable csv = maps.csv_table(key);
            csv.require_dimension(x, xd);
            csv.require_dimension(v, vd);
            return std::make_shared<const Table1D>(make_table1d(csv, x, v));
        });
    };
    c.torque_ngg = table("torque", "q", Dimension::moment, "n_gg", Dimension::dimensionless);
    c.lds_rigging =
        table("lds_rigging", "collective", Dimension::dimensionless, "lds", Dimension::angle);
    c.lds_cam = table("lds_cam", "lds", Dimension::angle, "n_gg", Dimension::dimensionless);
    c.fuel = table("fuel", "n_gg", Dimension::dimensionless, "wf", Dimension::mass_flow);
    c.p3 = table("p3", "n_gg", Dimension::dimensionless, "p3", Dimension::pressure);
    c.dp3_dwf = table("partials", "n_gg", Dimension::dimensionless, "dp3_dwf",
                      Dimension::pressure_per_mass_flow);
    c.dqgg_dwf = table("partials", "n_gg", Dimension::dimensionless, "dqgg_dwf",
                       Dimension::moment_per_mass_flow);
    c.dqpt_dwf = table("partials", "n_gg", Dimension::dimensionless, "dqpt_dwf",
                       Dimension::moment_per_mass_flow);
    c.dqpt_dnpt = table("partials", "n_gg", Dimension::dimensionless, "dqpt_dnpt",
                        Dimension::rotational_damping);

    const Config e = node.child("ecu");
    e.allow_keys({"k1", "k3", "k4", "k6", "k9", "k10", "k11", "dead_band", "t1", "t2", "t3", "t4",
                  "integrator_limits", "g4", "g5", "g6", "g7", "T2", "T3"});
    c.k1 = e.quantity("k1", Dimension::voltage);
    c.k3 = e.quantity("k3", Dimension::voltage);
    c.k4 = e.quantity("k4", Dimension::voltage_per_moment);
    c.k6 = e.number("k6");
    c.k9 = e.quantity("k9", Dimension::frequency);
    c.k10 = e.quantity("k10", Dimension::time_squared);
    c.k11 = e.quantity("k11", Dimension::time);
    const Config db = e.child("dead_band");
    db.allow_keys({"a", "b"});
    c.dead_band_a = db.quantity("a", Dimension::voltage);
    c.dead_band_b = db.quantity("b", Dimension::voltage);
    c.t1 = e.quantity("t1", Dimension::time);
    c.t2 = e.quantity("t2", Dimension::time);
    c.t3 = e.quantity("t3", Dimension::time);
    c.t4 = e.quantity("t4", Dimension::time);
    const Config lim = e.child("integrator_limits");
    lim.allow_keys({"upper", "lower"});
    c.upper = lim.quantity("upper", Dimension::voltage);
    c.lower = lim.quantity("lower", Dimension::voltage);
    c.g4 = e.quantity("g4", Dimension::current_per_voltage);
    c.g5 = e.quantity("g5", Dimension::length_per_current);
    c.g6 = e.quantity("g6", Dimension::voltage_per_length);
    c.g7 = e.quantity("g7", Dimension::per_length);
    c.T2 = e.quantity("T2", Dimension::time);
    c.T3 = e.quantity("T3", Dimension::time);

    const Config sw = node.child("switch");
    sw.allow_keys({"speed_band", "torque_gain", "threshold", "limit"});
    const Config band = sw.child("speed_band");
    band.allow_keys({"low", "high"});
    c.band_low = band.quantity("low", Dimension::dimensionless);
    c.band_high = band.quantity("high", Dimension::dimensionless);
    c.qint_gain = sw.quantity("torque_gain", Dimension::frequency);
    c.threshold = sw.quantity("threshold", Dimension::voltage);
    c.qint_limit = sw.quantity("limit", Dimension::voltage);

    const Config f = node.child("fuel_control");
    f.allow_keys({"speed_lag", "ratio_gain", "p3_lag", "valve_lag"});
    c.speed_lag = f.quantity("speed_lag", Dimension::time);
    c.ratio_gain = f.quantity("ratio_gain", Dimension::mass_flow_per_pressure);
    c.p3_lag = f.quantity("p3_lag", Dimension::time);
    c.valve_lag = f.quantity("valve_lag", Dimension::time);

    const Config gg = node.child("gas_generator");
    gg.allow_keys({"acceleration", "heat_sink"});
    c.acceleration = gg.quantity("acceleration", Dimension::per_moment_time);
    c.heat_sink = gg.number("heat_sink");

    const Config cl = node.child("clutch");
    cl.allow_keys({"demand_lag", "drive_inertia", "engine_inertia"});
    c.demand_lag = cl.quantity("demand_lag", Dimension::time);
    c.drive_inertia = cl.quantity("drive_inertia", Dimension::inertia);
    c.engine_inertia = cl.quantity("engine_inertia", Dimension::inertia);

    const auto axis = [](const Config& n) {
        const Config a = n.child("axis");
        a.allow_keys({"x", "y", "z"});
        const Vec3 v(a.number("x"), a.number("y"), a.number("z"));
        if (!(v.norm() > 0.0) || !v.allFinite()) {
            n.fail("axis", "must be a nonzero, finite direction");
        }
        return Vec3(v.normalized());
    };
    const Config pt = node.child("power_turbine_spool");
    pt.allow_keys({"axis"});
    c.pt_axis_b = axis(pt);
    const Config gs = node.child("gas_generator_spool");
    gs.allow_keys({"inertia", "axis"});
    c.gg_inertia = gs.quantity("inertia", Dimension::inertia);
    c.gg_axis_b = axis(gs);

    return node.in_context("", [&] { return std::make_unique<GenHelT700>(std::move(c)); });
}

}  // namespace eyas
