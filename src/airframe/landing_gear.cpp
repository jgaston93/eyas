#include "airframe/landing_gear.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "core/rigid_body.hpp"
#include "util/config.hpp"
#include "util/csv_table.hpp"
#include "util/error.hpp"
#include "util/math.hpp"

namespace eyas {

double tire_vertical_load(const Table1D& table, double deflection) noexcept {
    const std::span<const double> bp = table.x_axis().breakpoints();
    const std::size_t n = bp.size();
    if (deflection <= bp[n - 1]) {
        return table(deflection);
    }
    // Beyond the table: the last segment's slope (D-032; DATA_GAPS.md).
    const double slope = (table(bp[n - 1]) - table(bp[n - 2])) / (bp[n - 1] - bp[n - 2]);
    return table(bp[n - 1]) + slope * (deflection - bp[n - 1]);
}

namespace {

// Below this cos γ the gear line is treated as parallel to the ground: no
// contact. A numerical guard (the aircraft would be on its side), not data.
constexpr double kMinCosGamma = 1e-6;

// An in-plane axis whose stiffness is below this fraction of the tire's total
// in-plane stiffness counts as having none. A numerical guard, not data: with
// the √F lateral law at zero load the axis is stiff only through rounding, and
// the slip law would divide by it.
constexpr double kMinAxisStiffness = 1e-9;

double sign(double v) noexcept {
    return v < 0.0 ? -1.0 : 1.0;
}

void require_positive(double v, const std::string& what) {
    if (!(v > 0.0) || !std::isfinite(v)) {
        throw InitError("landing gear: " + what + " must be positive");
    }
}

void require_non_negative(double v, const std::string& what) {
    if (!(v >= 0.0) || !std::isfinite(v)) {
        throw InitError("landing gear: " + what + " must not be negative");
    }
}

// The tire's slope at zero deflection, all wheels [N/m].
double initial_tire_slope(const TireConfig& t) {
    const std::span<const double> bp = t.vertical_load.x_axis().breakpoints();
    return t.wheels * (t.vertical_load(bp[1]) - t.vertical_load(bp[0])) / (bp[1] - bp[0]);
}

}  // namespace

struct LandingGear::GearOut {
    GearState st;
    std::array<double, kStatesPerGear> xdot{};
    std::array<double, 6> g_sddot{};  // G row of s̈
    Wrench load;
    Mat66 H = Mat66::Zero();
};

LandingGear::LandingGear(LandingGearConfig config) : config_(std::move(config)) {
    if (config_.gears.empty()) {
        throw InitError("landing gear: needs at least one gear");
    }
    require_non_negative(config_.friction_coefficient, "the friction coefficient");
    require_non_negative(config_.brake_factor_set, "the brakes-set factor");
    require_non_negative(config_.brake_factor_off, "the brakes-off factor");
    if (!(config_.sliding_factor > 0.0) || !(config_.sliding_factor <= 1.0)) {  // NaN fails too
        throw InitError("landing gear: the sliding factor must be in (0, 1]");
    }
    require_positive(config_.sliding_blend_speed, "the sliding blend speed");
    const double nn = config_.ground_normal_n.norm();
    if (!config_.ground_normal_n.allFinite() || !(nn > 0.0) ||
        !(config_.ground_normal_n.z() < 0.0)) {
        throw InitError("landing gear: the ground normal must point up (n_z < 0, NED)");
    }
    config_.ground_normal_n /= nn;
    for (const GearConfig& g : config_.gears) {
        const std::string w = "gear '" + g.name + "': ";
        if (!g.axle_b.allFinite()) {
            throw InitError("landing gear: " + w + "the axle position must be finite");
        }
        require_positive(g.unsprung_mass, w + "the unsprung mass");
        const TireConfig& t = g.tire;
        require_positive(t.radius, w + "the tire radius");
        if (t.wheels < 1) {
            throw InitError("landing gear: " + w + "needs at least one wheel");
        }
        require_positive(t.pressure, w + "the tire pressure");
        require_positive(t.rated_pressure, w + "the rated tire pressure");
        require_non_negative(t.fore_aft_coefficient, w + "the fore-aft stiffness coefficient");
        require_positive(t.damping_factor, w + "the tire damping factor");
        require_positive(t.damping_lag, w + "the tire damping lag");
        if (!(t.vertical_load.x_axis().breakpoint(0) == 0.0) || !(t.vertical_load(0.0) == 0.0)) {
            throw InitError("landing gear: " + w + "the vertical load must start at (0, 0)");
        }
        if (!(initial_tire_slope(t) > 0.0)) {
            throw InitError("landing gear: " + w + "the vertical load must rise from zero");
        }
        // Increasing throughout, since the last segment's slope continues
        // beyond the table (tire_vertical_load).
        const std::span<const double> bp = t.vertical_load.x_axis().breakpoints();
        for (std::size_t k = 1; k < bp.size(); ++k) {
            if (!(t.vertical_load(bp[k]) > t.vertical_load(bp[k - 1]))) {
                throw InitError("landing gear: " + w +
                                "the vertical load must increase with deflection");
            }
        }
        if (!(min_value(t.damping_stiffness) >= 0.0)) {
            throw InitError("landing gear: " + w + "the damping stiffness must not be negative");
        }
        if (t.lateral_stiffness) {
            if (!(min_value(*t.lateral_stiffness) >= 0.0)) {
                throw InitError("landing gear: " + w +
                                "the lateral stiffness must not be negative");
            }
        } else {
            require_positive(t.lateral_reference_load, w + "the lateral reference load");
            require_positive(t.lateral_reference_stiffness, w + "the lateral reference stiffness");
        }
        const StrutConfig& s = g.strut;
        require_positive(s.preload, w + "the strut preload");
        require_positive(s.air_spring, w + "the strut air spring constant");
        require_positive(s.min_gas_length, w + "the minimum gas length");
        require_positive(s.break_speed, w + "the damper break speed");
        require_non_negative(s.low.coefficient, w + "the low-speed damping");
        require_non_negative(s.high.coefficient, w + "the high-speed damping");
        require_non_negative(s.low.offset, w + "the low-speed damper offset");
        require_non_negative(s.high.offset, w + "the high-speed damper offset");
        const double k = s.stop_stiffness ? *s.stop_stiffness : 10.0 * initial_tire_slope(t);
        require_positive(k, w + "the top stop stiffness");
        const double c = s.stop_damping ? *s.stop_damping : 2.0 * std::sqrt(k * g.unsprung_mass);
        require_non_negative(c, w + "the top stop damping");
        stop_k_.push_back(k);
        stop_c_.push_back(c);
    }
}

void LandingGear::initial_states(std::span<double> x) const {
    std::fill(x.begin(), x.end(), 0.0);
}

// Everything that doesn't depend on the gear: attitude, gravity, the ground
// plane and the ground axes, once per evaluation.
struct LandingGear::Frame {
    Mat3 C_nb;
    Mat3 C_bg;
    Mat3 C_gb;
    Vec3 g_b;
    Vec3 xg;
    Vec3 yg;
    double z_g = 0.0;
    double omega_g = 0.0;  // Ω_g [rad/s]
    bool ground = false;
    // The gear line (body z) against the ground: cos γ, its rate, and the
    // line direction's rate, NED.
    double cg = 0.0;
    double cg_dot = 0.0;
    Vec3 wd_n;
};

LandingGear::Frame LandingGear::frame(const SimContext& ctx) const noexcept {
    Frame f;
    f.C_nb = ctx.body.q_nb.toRotationMatrix();
    const Mat3& C_nb = f.C_nb;
    const Vec3& w = ctx.body.omega_b;
    f.g_b = C_nb.transpose() * gravity_n();

    // Ground plane (D-032 item 9): through (0, 0, z_g) NED, upward normal u.
    const Vec3& u = config_.ground_normal_n;
    f.z_g = ctx.air.altitude + ctx.body.pos_n.z() - ctx.env.ground_height;
    f.ground = std::isfinite(f.z_g);

    // Ground axes g: z down the normal, x the body's heading in the plane
    // (GenHel's space axes with ψ = 0, p. 5.7-18). They turn about z_g at
    // Ω_g = y_g·ẋ_g, the heading rate in the plane (D-032, deflection axes).
    const Vec3 zg = -u;
    Vec3 xg = C_nb.col(0) - C_nb.col(0).dot(zg) * zg;
    if (xg.norm() < 1e-9) {  // nose along the normal: use the body's y
        const Vec3 py = C_nb.col(1) - C_nb.col(1).dot(zg) * zg;
        const Vec3 py_dot = C_nb * w.cross(Vec3::UnitY());
        const Vec3 yg = py.normalized();
        xg = yg.cross(zg);
        f.omega_g = -xg.dot(py_dot) / py.norm();  // ẏ_g = −Ω_g x_g
    } else {
        const Vec3 px_dot = C_nb * w.cross(Vec3::UnitX());
        const double px = xg.norm();
        xg /= px;
        f.omega_g = zg.cross(xg).dot(px_dot) / px;
    }
    f.xg = xg;
    f.yg = zg.cross(xg);
    Mat3 C_ng;
    C_ng.col(0) = f.xg;
    C_ng.col(1) = f.yg;
    C_ng.col(2) = zg;
    f.C_bg = C_nb.transpose() * C_ng;
    f.C_gb = f.C_bg.transpose();

    f.wd_n = C_nb * w.cross(Vec3::UnitZ());  // d/dt of the gear line's direction
    f.cg = -u.dot(C_nb.col(2));
    f.cg_dot = -u.dot(f.wd_n);
    return f;
}

void LandingGear::evaluate(const SimContext& ctx, const Frame& f, std::span<const double> x,
                           std::size_t i, GearOut& out) const noexcept {
    const GearConfig& gc = config_.gears[i];
    const TireConfig& tc = gc.tire;
    const StrutConfig& sc = gc.strut;
    const std::span<const double> xi = x.subspan(kStatesPerGear * i, kStatesPerGear);
    const double s = xi[0];
    const double sd = xi[1];
    const double dx = xi[2];
    const double dy = xi[3];
    const double zf = xi[4];
    const double m = gc.unsprung_mass;
    const double R = tc.radius;
    const double G = tc.damping_factor;
    const double nt = tc.wheels;

    const Mat3& C_nb = f.C_nb;
    const Mat3& C_bg = f.C_bg;
    const Mat3& C_gb = f.C_gb;
    const Vec3& g_b = f.g_b;
    const double omega_g = f.omega_g;
    const Vec3& v = ctx.body.vel_b;
    const Vec3& w = ctx.body.omega_b;
    const Vec3 ez(0.0, 0.0, 1.0);

    // Axle and wheel kinematics, body axes. ṙ_ax = −ṡ e_z relative to the body.
    const Vec3 r = gc.axle_b - s * ez;
    const Vec3 r_dot = -sd * ez;
    const Vec3 v_ax = v + w.cross(r) + r_dot;
    const Vec3 c0 = w.cross(v) + w.cross(w.cross(r)) + 2.0 * w.cross(r_dot);

    // Gear line and tire radial deflection (pp. 5.7-13 to 5.7-15).
    GearState& st = out.st;
    st.axle_b = r;
    double delta_t = 0.0;
    double delta_t_dot = 0.0;
    std::array<double, 2> vc{0.0, 0.0};
    bool line_ok = false;
    if (f.ground) {
        const double cg = f.cg;
        if (cg > kMinCosGamma) {
            line_ok = true;
            const Vec3& u = config_.ground_normal_n;
            const Vec3 p0(0.0, 0.0, f.z_g);
            const Vec3 p_ax = ctx.body.pos_n + C_nb * r;
            const double h = u.dot(p_ax - p0);
            const Vec3 v_ax_n = C_nb * v_ax;
            const double h_dot = u.dot(v_ax_n);
            const double cg_dot = f.cg_dot;
            const double L = h / cg;
            const double L_dot = (h_dot * cg - h * cg_dot) / (cg * cg);
            delta_t = R - L;
            delta_t_dot = -L_dot;
            // Velocity of the gear line's intersection with the ground: the
            // rate of GenHel's in-plane deflection while the tire sticks.
            const Vec3 v_int = v_ax_n + L * f.wd_n + L_dot * C_nb.col(2);
            vc = {f.xg.dot(v_int), f.yg.dot(v_int)};
        }
    }
    st.tire_deflection = delta_t;
    st.contact_velocity = vc;
    // While the tire sticks, δ is an earth-fixed vector written in turning
    // axes: δ' = v_c − Ω_g e_z × δ (GenHel retains the contact point in
    // space axes, p. 5.7-16).
    const std::array<double, 2> vs_stick{vc[0] + omega_g * dy, vc[1] - omega_g * dx};
    st.stick_rate = vs_stick;

    const Vec3 delta_b = C_bg * Vec3(dx, dy, 0.0) + Vec3(0.0, 0.0, delta_t);
    st.vertical_deflection = delta_b.z();
    const bool contact = line_ok && delta_t > 0.0 && delta_b.z() > 0.0;
    st.in_contact = contact;

    std::array<double, 2> dd{-dx / G, -dy / G};  // unloaded tire relaxes (D-032)
    Vec3 F_b = Vec3::Zero();
    if (contact) {
        const double dz = delta_b.z();
        // Vertical (pp. 5.7-17, 5.7-43, 5.7-44): K′ looked up on the spring load.
        const double f_spring = tire_vertical_load(tc.vertical_load, dz);
        const double k_damp = tc.damping_stiffness(f_spring);
        const double f_v = nt * std::max(0.0, f_spring + G * k_damp * zf);  // no adhesion
        // In-plane stiffnesses (pp. 5.7-42, 5.7-46).
        const double D = 2.0 * R;
        const double kx = nt * tc.fore_aft_coefficient * D *
                          (tc.pressure + 4.0 * tc.rated_pressure) * std::cbrt(dz / D);
        const double ky = tc.lateral_stiffness
                              ? nt * (*tc.lateral_stiffness)(dz)
                              : nt * tc.lateral_reference_stiffness *
                                    std::sqrt((f_v / nt) / tc.lateral_reference_load);
        // In-plane ground forces as functions of q = δ + G δ': F = −A q + b.
        const Eigen::Matrix2d M = C_gb.topLeftCorner<2, 2>();
        const Eigen::Matrix2d A = M * Eigen::Vector2d(kx, ky).asDiagonal() * M.transpose();
        const Eigen::Vector2d b = -f_v * C_gb.block<2, 1>(0, 2);
        const Eigen::Vector2d qt(dx + G * vs_stick[0], dy + G * vs_stick[1]);
        // Trial (sticking) forces, body and ground axes (pp. 5.7-17, 5.7-18).
        const Vec3 qt_b = C_bg * Vec3(qt.x(), qt.y(), 0.0);
        const Vec3 Ft_b(-kx * qt_b.x(), -ky * qt_b.y(), -f_v);
        const Vec3 Ft_g = C_gb * Ft_b;
        const double N = std::max(0.0, -Ft_g.z());
        st.normal_load = N;
        // Limits (pp. 5.7-18, 5.7-19; D-032 item 2).
        // Brake input (D-032): clamped to [0, 1], NaN as released.
        double brake = 0.0;
        if (gc.brake != BrakeSide::none) {
            const double in =
                gc.brake == BrakeSide::left ? ctx.controls.brake_left : ctx.controls.brake_right;
            brake = in > 0.0 ? std::min(in, 1.0) : 0.0;
        }
        const double kb = config_.brake_factor_off +
                          (config_.brake_factor_set - config_.brake_factor_off) * brake;
        const double mu_n = config_.friction_coefficient * N;
        const double drop = 1.0 - config_.sliding_factor;
        const double vs = config_.sliding_blend_speed;
        const std::array<double, 2> lim{kb * mu_n * (1.0 - drop * smoothstep(std::abs(vc[0]) / vs)),
                                        mu_n * (1.0 - drop * smoothstep(std::abs(vc[1]) / vs))};
        st.limit = lim;
        Vec3 F_g = Ft_g;
        const std::array<double, 2> d{dx, dy};
        for (Eigen::Index k = 0; k < 2; ++k) {
            const Eigen::Index j = 1 - k;
            const auto ku = static_cast<std::size_t>(k);
            const double ft = Ft_g(k);
            if (!(A(k, k) > kMinAxisStiffness * A.trace())) {
                // No stiffness on this axis (the √F law at zero load): no
                // force whatever δ, so it relaxes as off the ground. Sticking
                // would let δ grow with v_c and return as a stale force.
                dd[ku] = -d[ku] / G;
            } else if (std::abs(ft) > lim[ku]) {
                // Slip: the force is held at ±L; the deflection q that gives
                // it, with the other axis at its trial value (D-032 item 1).
                const double sg = sign(ft);
                const double q = (b(k) - sg * lim[ku] - A(k, j) * qt(j)) / A(k, k);
                dd[ku] = (q - d[ku]) / G;
                F_g(k) = sg * lim[ku];
                st.slipping[ku] = true;
            } else {
                dd[ku] = vs_stick[ku];
            }
        }
        F_b = C_bg * F_g;
        st.tire_force_g = F_g;
    }
    st.tire_force_b = F_b;

    // Vertical deflection rate through GenHel's lag (D-032 item 3), only in
    // contact: GenHel skips the tire while it is off the ground (p. 5.7-14),
    // so the lag starts from rest at touchdown. The exact rate of
    // ΔZ = (C_bg δ)_z + δ_T, with Ċ_bg δ = −ω × C_bg δ + C_bg (Ω_g e_z × δ).
    double zf_dot = -zf / tc.damping_lag;
    if (contact) {
        const Vec3 d_g(dx, dy, 0.0);
        const Vec3 turn_g(-omega_g * dy, omega_g * dx, 0.0);
        const double dz_dot =
            (C_bg * (Vec3(dd[0], dd[1], 0.0) + turn_g) - w.cross(C_bg * d_g)).z() + delta_t_dot;
        zf_dot = (dz_dot - zf) / tc.damping_lag;
    }

    // Strut (pp. 5.7-23, 5.7-24; D-032 items 5, 6).
    const StrutDamperRegime& reg = std::abs(sd) <= sc.break_speed ? sc.low : sc.high;
    double f_s = sign(sd) * (reg.coefficient * sd * sd + reg.offset) +
                 sc.air_spring / std::max(sc.air_spring / sc.preload - s, sc.min_gas_length);
    if (s < 0.0) {
        f_s += stop_k_[i] * s + stop_c_[i] * sd;
    }
    st.strut_force = f_s;

    // Unsprung mass along body z (D-032 item 7).
    out.xdot = {sd, c0.z() - (F_b.z() + f_s) / m - g_b.z(), dd[0], dd[1], zf_dot};
    out.g_sddot = {0.0, 0.0, 1.0, r.y(), -r.x(), 0.0};

    // Loads on the airframe about the CG.
    const Vec3 r_c =
        r + (contact ? Vec3(-delta_b.x(), -delta_b.y(), R - delta_b.z()) : Vec3(0.0, 0.0, R));
    st.contact_point_b = r_c;
    const Vec3 inertial_xy(m * (g_b.x() - c0.x()), m * (g_b.y() - c0.y()), 0.0);
    out.load.F = Vec3(F_b.x(), F_b.y(), -f_s) + inertial_xy;
    out.load.M = r_c.cross(Vec3(F_b.x(), F_b.y(), 0.0)) + (r_c - r).cross(Vec3(0.0, 0.0, F_b.z())) +
                 r.cross(Vec3(0.0, 0.0, -f_s)) + r.cross(inertial_xy);
    // ∂load/∂a: the wheel's in-plane inertial force −m (v̇ + ω̇ × r)_xy.
    Eigen::Matrix<double, 1, 6> jx;
    Eigen::Matrix<double, 1, 6> jy;
    jx << 1.0, 0.0, 0.0, 0.0, r.z(), -r.y();
    jy << 0.0, 1.0, 0.0, -r.z(), 0.0, r.x();
    out.H.row(0) = -m * jx;
    out.H.row(1) = -m * jy;
    out.H.row(3) = m * r.z() * jy;
    out.H.row(4) = -m * r.z() * jx;
    out.H.row(5) = -m * r.x() * jy + m * r.y() * jx;
}

void LandingGear::derivatives(const SimContext& ctx, std::span<const double> x,
                              ComponentDerivs& out) const {
    const Frame f = frame(ctx);
    GearOut g;
    for (std::size_t i = 0; i < config_.gears.size(); ++i) {
        g = GearOut{};
        evaluate(ctx, f, x, i, g);
        for (std::size_t k = 0; k < kStatesPerGear; ++k) {
            out.xdot0[kStatesPerGear * i + k] = g.xdot[k];
        }
        const std::size_t row = kStatesPerGear * i + 1;  // s̈
        for (std::size_t c = 0; c < 6; ++c) {
            out.G[6 * row + c] = g.g_sddot[c];
        }
        out.load0_b.F += g.load.F;
        out.load0_b.M += g.load.M;
        out.H += g.H;
    }
}

GearState LandingGear::gear_state(const SimContext& ctx, std::span<const double> x,
                                  std::size_t i) const noexcept {
    GearOut g;
    evaluate(ctx, frame(ctx), x, i, g);
    return g.st;
}

// ---- Config ------------------------------------------------------------------

namespace {

using units::Dimension;

Vec3 position(const Config& node) {
    node.allow_keys({"x", "y", "z"});
    return {node.quantity("x", Dimension::length), node.quantity("y", Dimension::length),
            node.quantity("z", Dimension::length)};
}

// A tire table against `x`, either one column per pressure (a "pressure"
// column) sliced at `pressure`, or a single curve.
Table1D tire_table(const Config& node, const std::string& key, const std::string& x,
                   Dimension x_dim, const std::string& value, Dimension value_dim,
                   double pressure) {
    return node.in_context(key, [&] {
        const CsvTable csv = node.csv_table(key);
        csv.require_dimension(x, x_dim);
        csv.require_dimension(value, value_dim);
        if (std::find(csv.names.begin(), csv.names.end(), "pressure") == csv.names.end()) {
            return make_table1d(csv, x, value);
        }
        csv.require_dimension("pressure", Dimension::pressure);
        const Table2D t = make_table2d(csv, x, "pressure", value);
        const std::span<const double> bp = t.x_axis().breakpoints();
        const std::span<const double> pp = t.y_axis().breakpoints();
        if (pressure < pp.front() || pressure > pp.back()) {
            throw InitError("the tire pressure is outside the table's pressures");
        }
        std::vector<double> xs(bp.begin(), bp.end());
        std::vector<double> vs;
        vs.reserve(xs.size());
        for (const double xi : xs) {
            vs.push_back(t(xi, pressure));
        }
        return Table1D(Axis(std::move(xs)), std::move(vs));
    });
}

TireConfig tire(const Config& node) {
    node.allow_keys({"radius", "wheels", "pressure", "rated_pressure", "vertical_load",
                     "damping_stiffness", "lateral_stiffness", "lateral_stiffness_law",
                     "fore_aft_coefficient", "damping_factor", "damping_lag"});
    TireConfig t;
    t.radius = node.quantity("radius", Dimension::length);
    if (node.contains("wheels")) {
        const std::int64_t n = node.integer("wheels");
        if (n < 1 || n > 16) {
            node.fail("wheels", "must be 1 to 16");
        }
        t.wheels = static_cast<int>(n);
    }
    t.pressure = node.quantity("pressure", Dimension::pressure);
    t.rated_pressure = node.quantity("rated_pressure", Dimension::pressure);
    t.vertical_load = tire_table(node, "vertical_load", "deflection", Dimension::length, "load",
                                 Dimension::force, t.pressure);
    t.damping_stiffness = tire_table(node, "damping_stiffness", "load", Dimension::force,
                                     "stiffness", Dimension::stiffness, t.pressure);
    if (node.contains("lateral_stiffness") == node.contains("lateral_stiffness_law")) {
        node.fail("lateral_stiffness", "give either lateral_stiffness or lateral_stiffness_law");
    }
    if (node.contains("lateral_stiffness")) {
        t.lateral_stiffness = tire_table(node, "lateral_stiffness", "deflection", Dimension::length,
                                         "stiffness", Dimension::stiffness, t.pressure);
    } else {
        const Config law = node.child("lateral_stiffness_law");
        law.allow_keys({"reference_load", "reference_stiffness"});
        t.lateral_reference_load = law.quantity("reference_load", Dimension::force);
        t.lateral_reference_stiffness = law.quantity("reference_stiffness", Dimension::stiffness);
    }
    t.fore_aft_coefficient = node.number("fore_aft_coefficient");
    t.damping_factor = node.quantity("damping_factor", Dimension::time);
    t.damping_lag = node.quantity("damping_lag", Dimension::time);
    return t;
}

StrutDamperRegime regime(const Config& node) {
    node.allow_keys({"coefficient", "offset"});
    return {node.quantity("coefficient", Dimension::quadratic_damping),
            node.quantity("offset", Dimension::force)};
}

StrutConfig strut(const Config& node) {
    node.allow_keys(
        {"preload", "air_spring", "min_gas_length", "damper", "stop_stiffness", "stop_damping"});
    StrutConfig s;
    s.preload = node.quantity("preload", Dimension::force);
    s.air_spring = node.quantity("air_spring", Dimension::moment);  // force × length (pV)
    if (node.contains("min_gas_length")) {
        s.min_gas_length = node.quantity("min_gas_length", Dimension::length);
    }
    const Config d = node.child("damper");
    d.allow_keys({"break_speed", "low", "high"});
    s.break_speed = d.quantity("break_speed", Dimension::speed);
    s.low = d.in_context("low", [&] { return regime(d.child("low")); });
    s.high = d.in_context("high", [&] { return regime(d.child("high")); });
    if (node.contains("stop_stiffness")) {
        s.stop_stiffness = node.quantity("stop_stiffness", Dimension::stiffness);
    }
    if (node.contains("stop_damping")) {
        s.stop_damping = node.quantity("stop_damping", Dimension::damping);
    }
    return s;
}

}  // namespace

std::unique_ptr<LandingGear> LandingGear::from_config(const Config& node) {
    node.allow_keys({"type", "name", "friction", "gears"});
    LandingGearConfig c;
    if (node.contains("name")) {
        c.name = node.string("name");
        if (c.name.empty()) {
            node.fail("name", "must not be empty");
        }
    }
    const Config f = node.child("friction");
    f.allow_keys({"coefficient", "brake_factor_set", "brake_factor_off", "sliding_factor",
                  "sliding_blend_speed"});
    c.friction_coefficient = f.number("coefficient");
    c.brake_factor_set = f.number("brake_factor_set");
    c.brake_factor_off = f.number("brake_factor_off");
    c.sliding_factor = f.number("sliding_factor");
    if (f.contains("sliding_blend_speed")) {
        c.sliding_blend_speed = f.quantity("sliding_blend_speed", Dimension::speed);
    }
    const Config list = node.child("gears");
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Config g = list[i];
        g.allow_keys({"name", "axle", "unsprung_mass", "brake", "tire", "strut"});
        GearConfig gc;
        gc.name = g.string("name");
        gc.axle_b = g.in_context("axle", [&] { return position(g.child("axle")); });
        gc.unsprung_mass = g.quantity("unsprung_mass", Dimension::mass);
        const std::string brake = g.string("brake");
        if (brake == "left") {
            gc.brake = BrakeSide::left;
        } else if (brake == "right") {
            gc.brake = BrakeSide::right;
        } else if (brake != "none") {
            g.fail("brake", "must be 'left', 'right' or 'none'");
        }
        gc.tire = g.in_context("tire", [&] { return tire(g.child("tire")); });
        gc.strut = g.in_context("strut", [&] { return strut(g.child("strut")); });
        c.gears.push_back(std::move(gc));
    }
    return node.in_context("", [&] { return std::make_unique<LandingGear>(std::move(c)); });
}

}  // namespace eyas
