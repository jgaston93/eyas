#include "airframe/airframe_aero.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>

#include "env/atmosphere.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"
#include "util/math.hpp"

namespace eyas {
namespace {

void validate_panel(const PanelConfig& p, const std::string& what) {
    if (!p.position_b.allFinite() || !(p.position_b.x() < 0.0)) {
        throw InitError("airframe aero: the " + what + " must be aft of the CG (x < 0)");
    }
    if (!(p.area > 0.0) || !std::isfinite(p.area)) {
        throw InitError("airframe aero: the " + what + " area must be positive");
    }
    if (!std::isfinite(p.incidence)) {
        throw InitError("airframe aero: the " + what + " incidence must be finite");
    }
    if (p.schedule && !(p.schedule->min < p.schedule->max)) {
        throw InitError("airframe aero: the " + what + " schedule's min must be below its max");
    }
    if (!(p.q_ratio.min_value() >= 0.0)) {
        throw InitError("airframe aero: the " + what +
                        " dynamic-pressure ratio must not be negative");
    }
}

}  // namespace

AngleTable::AngleTable(Table1D table) : t_(std::move(table)) {
    const std::span<const double> x = t_.x_axis().breakpoints();
    for (std::size_t i = 1; i + 1 < x.size(); ++i) {
        if (x[i] == 0.0 && std::abs(x[i - 1] + x[i + 1]) <= 1e-12 * x[i + 1]) {
            smooth_ = true;
            a_ = x[i + 1];
            f0_ = t_(0.0);
            e_ = 0.5 * (t_(a_) + t_(-a_));
            o_ = 0.5 * (t_(a_) - t_(-a_));
        }
    }
}

double AngleTable::min_value() const noexcept {
    double m = eyas::min_value(t_);
    const double c = e_ - f0_;  // f = f0 + o u + c u², vertex at u = −o/(2c)
    if (smooth_ && c > 0.0 && std::abs(o_) < 2.0 * c) {
        m = std::min(m, f0_ - o_ * o_ / (4.0 * c));
    }
    return m;
}

namespace {

// GenHel's wind-to-body matrix (CR-166309 pp. 5.2-9, 5.3-8, 5.3-12), applied
// to a vector in wind axes (suffix w), with the flow's actual direction α' (D-030).
Vec3 wind_to_body(double alpha, double beta, const Vec3& x_w) {
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double cb = std::cos(beta);
    const double sb = std::sin(beta);
    return {ca * cb * x_w.x() + ca * sb * x_w.y() - sa * x_w.z(), sb * x_w.x() - cb * x_w.y(),
            sa * cb * x_w.x() + sa * sb * x_w.y() + ca * x_w.z()};
}

// GenHel's flow angles at a point (pp. 5.2-6, 5.3-7, 5.3-11), with atan2 so
// that they stay defined at V_X = 0, the wind-axes angle α' (D-030) and the
// sideward fade s (D-031), already applied to α.
struct FlowAngles {
    double alpha;       // s·atan(V_Z/|V_X|): table lookups
    double beta;        // atan(V_Y/√(V_X² + V_Z²))
    double alpha_axes;  // atan2(V_Z, V_X): wind-to-body rotation
    double fade;        // s in [0, 1]; 1 away from the sideward line
};
FlowAngles flow_angles(const Vec3& v_b, double sin_fade) {
    const double xz = std::sqrt(v_b.x() * v_b.x() + v_b.z() * v_b.z());
    // t = cos β / sin c = xz / (sin c · |V|); s = 1 for t ≥ 1.
    const double edge = sin_fade * v_b.norm();
    const double s = xz < edge ? smoothstep(xz / edge) : 1.0;
    return {s * std::atan2(v_b.z(), std::abs(v_b.x())), std::atan2(v_b.y(), xz),
            std::atan2(v_b.z(), v_b.x()), s};
}

// A wind-axis force or moment in body axes (D-030, D-031): the x component
// along the flow, the others turned by R(α', β) faded into R(0, β).
Vec3 wind_to_body(const FlowAngles& fa, const Vec3& x_w) {
    if (fa.fade == 1.0) {
        return wind_to_body(fa.alpha_axes, fa.beta, x_w);
    }
    const Vec3 perp_w(0.0, x_w.y(), x_w.z());
    return wind_to_body(fa.alpha_axes, fa.beta, Vec3(x_w.x(), 0.0, 0.0)) +
           fa.fade * wind_to_body(fa.alpha_axes, fa.beta, perp_w) +
           (1.0 - fa.fade) * wind_to_body(0.0, fa.beta, perp_w);
}

}  // namespace

AirframeAero::AirframeAero(AirframeAeroConfig config) : config_(std::move(config)) {
    if (!config_.fuselage_point_b.allFinite()) {
        throw InitError("airframe aero: the fuselage mounting point must be finite");
    }
    validate_panel(config_.stabilator, "stabilator");
    validate_panel(config_.fin, "fin");
    if (config_.fin.schedule) {
        throw InitError("airframe aero: only the stabilator has an incidence schedule");
    }
    const double fade = config_.sideward_fade;
    if (!std::isfinite(fade) || fade < 0.0 || fade >= 0.5 * std::numbers::pi) {
        throw InitError("airframe aero: the sideward fade must be in [0, 90) deg");
    }
}

void AirframeAero::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
}

std::vector<std::string> AirframeAero::produces() const {
    if (config_.name.empty()) {
        return {};
    }
    const std::string& n = config_.name;
    return {n + ".alpha_wf", n + ".psi_wf", n + ".fin_q_factor", n + ".sidewash", n + ".downwash"};
}

std::vector<std::string> AirframeAero::consumes() const {
    std::vector<std::string> names;
    for (const auto* w : {&config_.fuselage_wash, &config_.tail_wash}) {
        if (*w) {
            for (std::string& n : (*w)->consumes()) {
                if (std::find(names.begin(), names.end(), n) == names.end()) {
                    names.push_back(std::move(n));
                }
            }
        }
    }
    return names;
}

void AirframeAero::bind(const CouplingRegistry& registry) {
    const std::string consumer = config_.name.empty() ? "airframe_aero" : config_.name;
    if (config_.fuselage_wash) {
        config_.fuselage_wash->bind(registry, consumer);
    }
    if (config_.tail_wash) {
        config_.tail_wash->bind(registry, consumer);
    }
    if (!config_.name.empty()) {
        alpha_wf_ = registry.find(config_.name + ".alpha_wf", consumer);
        psi_wf_ = registry.find(config_.name + ".psi_wf", consumer);
        fin_q_factor_ = registry.find(config_.name + ".fin_q_factor", consumer);
        sidewash_ = registry.find(config_.name + ".sidewash", consumer);
        downwash_ = registry.find(config_.name + ".downwash", consumer);
    }
}

AirframeAeroLoads AirframeAero::evaluate(const SimContext& ctx,
                                         std::span<const double> x) const noexcept {
    Vec3 fuselage_wash_b = Vec3::Zero();
    Vec3 tail_wash_b = Vec3::Zero();
    if (ctx.coupling != nullptr) {
        if (config_.fuselage_wash) {
            fuselage_wash_b = config_.fuselage_wash->evaluate(*ctx.coupling).velocity_b;
        }
        if (config_.tail_wash) {
            tail_wash_b = config_.tail_wash->evaluate(*ctx.coupling).velocity_b;
        }
    }
    return evaluate(ctx, x, fuselage_wash_b, tail_wash_b);
}

AirframeAeroLoads AirframeAero::evaluate(const SimContext& ctx, std::span<const double> x,
                                         const Vec3& fuselage_wash_b,
                                         const Vec3& tail_wash_b) const noexcept {
    const AirframeAeroConfig& c = config_;
    const BodyState& b = ctx.body;
    const double rho = ctx.air.density;
    AirframeAeroLoads out;
    const Vec3 v_air_b = b.vel_b - b.q_nb.conjugate() * ctx.env.wind_n;

    // ---- Fuselage (GenHel CR-166309 §5.2.2) ----
    // Velocity with main-rotor wash; no body-rate terms (p. 5.2-5).
    const Vec3 v_wf_b = v_air_b + fuselage_wash_b;
    const double sin_fade = std::sin(c.sideward_fade);
    const FlowAngles fw = flow_angles(v_wf_b, sin_fade);
    out.alpha_wf = fw.alpha;
    out.psi_wf = -fw.beta;  // p. 5.2-12: PSIWF = −BETAWF
    out.q_wf = 0.5 * rho * v_wf_b.squaredNorm();
    {
        const FuselageTables& t = c.fuselage;
        const double a = out.alpha_wf;
        const double p = out.psi_wf;
        // p. 5.2-7, with DLQD = DMQD = DNQD = 0 (DATA_GAPS.md M6 item 4). The
        // tables carry GenHel's parities in ψ_WF (tools/build_uh60a_airframe.py).
        const double drag = t.drag(a) + t.drag_increment(p);
        const double lift = t.lift(a) + t.lift_increment(p);
        const double pitch = t.pitch(a) + t.pitch_increment(p);
        const double q = out.q_wf;
        // p. 5.2-9: wind axes at the mounting point, then moved to the CG.
        const Vec3 F_b = wind_to_body(fw, Vec3(-drag, -t.side(p), -lift) * q);
        const Vec3 M_b = wind_to_body(fw, Vec3(t.roll(p), -pitch, t.yaw(p)) * q);
        out.fuselage.F = F_b;
        out.fuselage.M = M_b + c.fuselage_point_b.cross(F_b);
    }

    // ---- Empennage (§5.3.2) ----
    // The fuselage wake at a panel: K_Q = √(q ratio) scales the body velocity
    // and the delayed wake angle, −u ε_d K_Q (pp. 5.3-6, 5.3-7, 5.3-10, 5.3-11).
    const double u = v_air_b.x();
    {
        const PanelConfig& h = c.stabilator;
        const double kq = std::sqrt(h.q_ratio(out.alpha_wf));
        out.downwash = h.wake_angle(out.alpha_wf);
        const Vec3 v_b = kq * v_air_b + b.omega_b.cross(h.position_b) + tail_wash_b +
                         Vec3(0.0, 0.0, -u * x[0] * kq);
        out.v_stabilator_b = v_b;
        out.q_h = 0.5 * rho * v_b.squaredNorm();
        const FlowAngles fa = flow_angles(v_b, sin_fade);
        out.stabilator_incidence = h.incidence;
        if (h.schedule) {
            // GenHel Fig. 5.1.7, steady state (D-028), on the equivalent
            // airspeed u √(ρ/ρ0) the air-data sensors give (D-040).
            const double u_eas = u * std::sqrt(rho / isa::kSeaLevelDensity);
            out.stabilator_incidence =
                std::clamp(h.schedule->command(u_eas, ctx.controls.collective), h.schedule->min,
                           h.schedule->max);
        }
        out.alpha_h = out.stabilator_incidence + fa.alpha;
        const double qs = out.q_h * h.area;
        // p. 5.3-8: drag along the flow, lift in the x–z plane.
        const Vec3 F_b =
            wind_to_body(fa, Vec3(-qs * h.cd(out.alpha_h), 0.0, -qs * h.cl(out.alpha_h)));
        out.stabilator.F = F_b;
        out.stabilator.M = h.position_b.cross(F_b);
        out.downwash_rate = std::abs(u) / -h.position_b.x() * (out.downwash - x[0]);
    }
    {
        const PanelConfig& v_ = c.fin;
        const double kq = std::sqrt(v_.q_ratio(out.psi_wf));
        out.fin_q_factor = kq;
        out.sidewash = v_.wake_angle(out.psi_wf);
        const Vec3 v_b = kq * v_air_b + b.omega_b.cross(v_.position_b) + tail_wash_b +
                         Vec3(0.0, -u * x[1] * kq, 0.0);
        out.v_fin_b = v_b;
        out.q_v = 0.5 * rho * v_b.squaredNorm();
        const FlowAngles fa = flow_angles(v_b, sin_fade);
        out.beta_v = v_.incidence + fa.beta;
        const double qs = out.q_v * v_.area;
        // p. 5.3-12: drag along the flow, lift along the wind-axis y.
        const Vec3 F_b =
            wind_to_body(fa, Vec3(-qs * v_.cd(out.beta_v), qs * v_.cl(out.beta_v), 0.0));
        out.fin.F = F_b;
        out.fin.M = v_.position_b.cross(F_b);
        out.sidewash_rate = std::abs(u) / -v_.position_b.x() * (out.sidewash - x[1]);
    }

    out.body.F = out.fuselage.F + out.stabilator.F + out.fin.F;
    out.body.M = out.fuselage.M + out.stabilator.M + out.fin.M;
    return out;
}

void AirframeAero::derivatives(const SimContext& ctx, std::span<const double> x,
                               ComponentDerivs& out) const {
    const AirframeAeroLoads l = evaluate(ctx, x);
    out.load0_b = l.body;
    out.xdot0[0] = l.downwash_rate;
    out.xdot0[1] = l.sidewash_rate;
    if (out.coupling != nullptr && alpha_wf_.valid()) {
        out.coupling->set(alpha_wf_, l.alpha_wf);
        out.coupling->set(psi_wf_, l.psi_wf);
        out.coupling->set(fin_q_factor_, l.fin_q_factor);
        out.coupling->set(sidewash_, l.sidewash);
        out.coupling->set(downwash_, l.downwash);
    }
}

// ---- Config ------------------------------------------------------------------

namespace {

using units::Dimension;

Vec3 position(const Config& node) {
    node.allow_keys({"x", "y", "z"});
    return {node.quantity("x", Dimension::length), node.quantity("y", Dimension::length),
            node.quantity("z", Dimension::length)};
}

// One CSV, several columns against one angle column.
struct Columns {
    CsvTable csv;
    Table1D column(const std::string& name, Dimension d) const {
        csv.require_dimension(name, d);
        return make_table1d(csv, x, name);
    }
    std::string x;
};

Columns read_columns(const Config& node, const std::string& key, const std::string& x) {
    return node.in_context(key, [&] {
        CsvTable csv = node.csv_table(key);
        csv.require_dimension(x, Dimension::angle);
        return Columns{std::move(csv), x};
    });
}

// {table: CSV (airspeed, collective, incidence), min, max}.
IncidenceSchedule schedule(const Config& node) {
    node.allow_keys({"table", "min", "max"});
    IncidenceSchedule s;
    s.command = node.in_context("table", [&] {
        const CsvTable csv = node.csv_table("table");
        csv.require_dimension("airspeed", Dimension::speed);
        csv.require_dimension("collective", Dimension::dimensionless);
        csv.require_dimension("incidence", Dimension::angle);
        return make_table2d(csv, "airspeed", "collective", "incidence");
    });
    s.min = node.quantity("min", Dimension::angle);
    s.max = node.quantity("max", Dimension::angle);
    return s;
}

// `schedule` is a key of the stabilator only (Fig. 5.1.7).
PanelConfig panel(const Config& node, const std::string& angle, const std::string& wake_arg,
                  const std::string& wake_key, bool schedulable) {
    if (schedulable) {
        node.allow_keys(
            {"position", "area", "incidence", "schedule", "coefficients", "q_ratio", wake_key});
    } else {
        node.allow_keys({"position", "area", "incidence", "coefficients", "q_ratio", wake_key});
    }
    PanelConfig p;
    p.position_b = node.in_context("position", [&] { return position(node.child("position")); });
    p.area = node.quantity("area", Dimension::area);
    p.incidence = node.contains("incidence") ? node.quantity("incidence", Dimension::angle) : 0.0;
    if (node.contains("schedule")) {
        if (node.contains("incidence")) {
            node.fail("schedule", "give either incidence or schedule, not both");
        }
        p.schedule = node.in_context("schedule", [&] { return schedule(node.child("schedule")); });
    }
    const Columns coef = read_columns(node, "coefficients", angle);
    node.in_context("coefficients", [&] {
        p.cl = coef.column("cl", Dimension::dimensionless);
        p.cd = coef.column("cd", Dimension::dimensionless);
        return 0;
    });
    const Columns q = read_columns(node, "q_ratio", wake_arg);
    p.q_ratio =
        node.in_context("q_ratio", [&] { return q.column("q_ratio", Dimension::dimensionless); });
    const Columns w = read_columns(node, wake_key, wake_arg);
    p.wake_angle = node.in_context(wake_key, [&] { return w.column(wake_key, Dimension::angle); });
    return p;
}

}  // namespace

std::unique_ptr<AirframeAero> AirframeAero::from_config(const Config& node) {
    node.allow_keys(
        {"type", "name", "fuselage", "tail_wash", "stabilator", "fin", "sideward_fade"});
    AirframeAeroConfig c;
    if (node.contains("name")) {
        c.name = node.string("name");
        if (c.name.empty()) {
            node.fail("name", "must not be empty");
        }
    }
    const Config f = node.child("fuselage");
    f.allow_keys(
        {"mounting_point", "alpha_table", "psi_table", "psi_low_table", "main_rotor_wash"});
    c.fuselage_point_b =
        f.in_context("mounting_point", [&] { return position(f.child("mounting_point")); });
    const Columns ta = read_columns(f, "alpha_table", "alpha_wf");
    f.in_context("alpha_table", [&] {
        c.fuselage.drag = ta.column("drag", Dimension::area);
        c.fuselage.lift = ta.column("lift", Dimension::area);
        c.fuselage.pitch = ta.column("pitch", Dimension::volume);
        return 0;
    });
    const Columns tp = read_columns(f, "psi_table", "psi_wf");
    f.in_context("psi_table", [&] {
        c.fuselage.drag_increment = tp.column("drag_increment", Dimension::area);
        c.fuselage.side = tp.column("side", Dimension::area);
        c.fuselage.roll = tp.column("roll", Dimension::volume);
        c.fuselage.yaw = tp.column("yaw", Dimension::volume);
        return 0;
    });
    const Columns tl = read_columns(f, "psi_low_table", "psi_wf");
    f.in_context("psi_low_table", [&] {
        c.fuselage.lift_increment = tl.column("lift_increment", Dimension::area);
        c.fuselage.pitch_increment = tl.column("pitch_increment", Dimension::volume);
        return 0;
    });
    if (f.contains("main_rotor_wash")) {
        c.fuselage_wash = RotorWash::from_config(f.child("main_rotor_wash"));
    }
    if (node.contains("tail_wash")) {
        c.tail_wash = RotorWash::from_config(node.child("tail_wash"));
    }
    c.stabilator = node.in_context("stabilator", [&] {
        return panel(node.child("stabilator"), "alpha", "alpha_wf", "downwash", true);
    });
    c.fin = node.in_context(
        "fin", [&] { return panel(node.child("fin"), "beta", "psi_wf", "sidewash", false); });
    if (node.contains("sideward_fade")) {
        c.sideward_fade = node.quantity("sideward_fade", Dimension::angle);
    }
    return node.in_context("", [&] { return std::make_unique<AirframeAero>(std::move(c)); });
}

}  // namespace eyas
