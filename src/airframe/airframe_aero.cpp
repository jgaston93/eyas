#include "airframe/airframe_aero.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"

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
}

// GenHel's wind-to-body matrix (CR-166309 pp. 5.2-9, 5.3-8, 5.3-12), applied
// to a wind-axis vector w, with the flow's actual direction α' (D-030).
Vec3 wind_to_body(double alpha, double beta, const Vec3& w) {
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);
    const double cb = std::cos(beta);
    const double sb = std::sin(beta);
    return {ca * cb * w.x() + ca * sb * w.y() - sa * w.z(), sb * w.x() - cb * w.y(),
            sa * cb * w.x() + sa * sb * w.y() + ca * w.z()};
}

// GenHel's flow angles at a point (pp. 5.2-6, 5.3-7, 5.3-11), with atan2 so
// that they stay defined at V_X = 0, and the wind-axes angle α' (D-030).
struct FlowAngles {
    double alpha;       // atan(V_Z/|V_X|): table lookups
    double beta;        // atan(V_Y/√(V_X² + V_Z²))
    double alpha_axes;  // atan2(V_Z, V_X): wind-to-body rotation
};
FlowAngles flow_angles(const Vec3& v) {
    const double xz = std::sqrt(v.x() * v.x() + v.z() * v.z());
    return {std::atan2(v.z(), std::abs(v.x())), std::atan2(v.y(), xz), std::atan2(v.z(), v.x())};
}

}  // namespace

AirframeAero::AirframeAero(AirframeAeroConfig config) : config_(std::move(config)) {
    if (!config_.fuselage_point_b.allFinite()) {
        throw InitError("airframe aero: the fuselage mounting point must be finite");
    }
    validate_panel(config_.stabilator, "stabilator");
    validate_panel(config_.fin, "fin");
}

void AirframeAero::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
}

std::vector<std::string> AirframeAero::produces() const {
    if (config_.name.empty()) {
        return {};
    }
    return {config_.name + ".alpha_wf", config_.name + ".psi_wf"};
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
    const Vec3 v_wf = v_air_b + fuselage_wash_b;
    const FlowAngles fw = flow_angles(v_wf);
    out.alpha_wf = fw.alpha;
    out.psi_wf = -fw.beta;  // p. 5.2-12: PSIWF = −BETAWF
    out.q_wf = 0.5 * rho * v_wf.squaredNorm();
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
        const Vec3 F = wind_to_body(fw.alpha_axes, fw.beta, Vec3(-drag, -t.side(p), -lift) * q);
        const Vec3 M = wind_to_body(fw.alpha_axes, fw.beta, Vec3(t.roll(p), -pitch, t.yaw(p)) * q);
        out.fuselage.F = F;
        out.fuselage.M = M + c.fuselage_point_b.cross(F);
    }

    // ---- Empennage (§5.3.2) ----
    // The fuselage wake at a panel: K_Q = √(q ratio) scales the body velocity
    // and the delayed wake angle, −u ε_d K_Q (pp. 5.3-6, 5.3-7, 5.3-10, 5.3-11).
    const double u = v_air_b.x();
    {
        const PanelConfig& h = c.stabilator;
        const double kq = std::sqrt(h.q_ratio(out.alpha_wf));
        out.downwash = h.wake_angle(out.alpha_wf);
        const Vec3 v = kq * v_air_b + b.omega_b.cross(h.position_b) + tail_wash_b +
                       Vec3(0.0, 0.0, -u * x[0] * kq);
        out.v_stabilator_b = v;
        out.q_h = 0.5 * rho * v.squaredNorm();
        const FlowAngles fa = flow_angles(v);
        out.stabilator_incidence = h.incidence;
        if (h.schedule) {
            // GenHel Fig. 5.1.7, steady state (D-028).
            out.stabilator_incidence = std::clamp(h.schedule->command(u, ctx.controls.collective),
                                                  h.schedule->min, h.schedule->max);
        }
        out.alpha_h = out.stabilator_incidence + fa.alpha;
        const double qs = out.q_h * h.area;
        // p. 5.3-8: drag along the flow, lift in the x–z plane.
        const Vec3 F = wind_to_body(fa.alpha_axes, fa.beta,
                                    Vec3(-qs * h.cd(out.alpha_h), 0.0, -qs * h.cl(out.alpha_h)));
        out.stabilator.F = F;
        out.stabilator.M = h.position_b.cross(F);
        out.downwash_rate = std::abs(u) / -h.position_b.x() * (out.downwash - x[0]);
    }
    {
        const PanelConfig& v_ = c.fin;
        const double kq = std::sqrt(v_.q_ratio(out.psi_wf));
        out.sidewash = v_.wake_angle(out.psi_wf);
        const Vec3 v = kq * v_air_b + b.omega_b.cross(v_.position_b) + tail_wash_b +
                       Vec3(0.0, -u * x[1] * kq, 0.0);
        out.v_fin_b = v;
        out.q_v = 0.5 * rho * v.squaredNorm();
        const FlowAngles fa = flow_angles(v);
        out.beta_v = v_.incidence + fa.beta;
        const double qs = out.q_v * v_.area;
        // p. 5.3-12: drag along the flow, lift along the wind-axis y.
        const Vec3 F = wind_to_body(fa.alpha_axes, fa.beta,
                                    Vec3(-qs * v_.cd(out.beta_v), qs * v_.cl(out.beta_v), 0.0));
        out.fin.F = F;
        out.fin.M = v_.position_b.cross(F);
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
        CsvTable csv = read_csv_table(node.path(key));
        csv.require_dimension(x, Dimension::angle);
        if (csv.is_placeholder()) {
            node.note_placeholder(key, csv.origin + " (# status: placeholder)");
        }
        return Columns{std::move(csv), x};
    });
}

// {table: CSV (airspeed, collective, incidence), min, max}.
IncidenceSchedule schedule(const Config& node) {
    node.allow_keys({"table", "min", "max"});
    IncidenceSchedule s;
    s.command = node.in_context("table", [&] {
        const CsvTable csv = read_csv_table(node.path("table"));
        csv.require_dimension("airspeed", Dimension::speed);
        csv.require_dimension("collective", Dimension::dimensionless);
        csv.require_dimension("incidence", Dimension::angle);
        if (csv.is_placeholder()) {
            node.note_placeholder("table", csv.origin + " (# status: placeholder)");
        }
        return make_table2d(csv, "airspeed", "collective", "incidence");
    });
    s.min = node.quantity("min", Dimension::angle);
    s.max = node.quantity("max", Dimension::angle);
    return s;
}

PanelConfig panel(const Config& node, const std::string& angle, const std::string& wake_arg,
                  const std::string& wake_key) {
    node.allow_keys(
        {"position", "area", "incidence", "schedule", "coefficients", "q_ratio", wake_key});
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
    node.allow_keys({"type", "name", "fuselage", "tail_wash", "stabilator", "fin"});
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
        return panel(node.child("stabilator"), "alpha", "alpha_wf", "downwash");
    });
    c.fin = node.in_context("fin",
                            [&] { return panel(node.child("fin"), "beta", "psi_wf", "sidewash"); });
    return node.in_context("", [&] { return std::make_unique<AirframeAero>(std::move(c)); });
}

}  // namespace eyas
