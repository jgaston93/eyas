// M6c landing gear (D-028, D-032).

#include "airframe/landing_gear.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <string>
#include <tuple>
#include <vector>

#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/genhel_gear.hpp"
#include "support/m6c_fixtures.hpp"
#include "util/config.hpp"
#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using units::kFoot;
using units::kPoundForce;

constexpr std::size_t kN = LandingGear::kStatesPerGear;

std::unique_ptr<LandingGear> uh60a_gear(const test::GearYamlOptions& o = {}) {
    return LandingGear::from_config(Config::from_string(test::uh60a_gear_yaml(o)));
}

Quat attitude(double roll, double pitch, double heading) {
    return Quat(Eigen::AngleAxisd(heading, Vec3::UnitZ()) *
                Eigen::AngleAxisd(pitch, Vec3::UnitY()) * Eigen::AngleAxisd(roll, Vec3::UnitX()));
}

// Buffers for one derivative evaluation.
struct Derivs {
    explicit Derivs(std::size_t n) : xdot(n, 0.0), G(6 * n, 0.0) {
        out.xdot0 = xdot;
        out.G = G;
        out.H.setZero();
    }
    std::vector<double> xdot, G;
    ComponentDerivs out;
};

Derivs eval(const LandingGear& gear, const SimContext& ctx, const std::vector<double>& x) {
    Derivs d(gear.num_states());
    gear.derivatives(ctx, x, d.out);
    return d;
}

// The wheel's in-plane weight minus its inertial force at a = 0 (D-032
// item 7), and the extra terms in s̈ that GenHel leaves out.
struct D7Terms {
    Wrench load;
    std::vector<double> sddot;
};
D7Terms d7_terms(const LandingGear& gear, const SimContext& ctx, const std::vector<double>& x) {
    D7Terms t;
    const Mat3 C_nb = ctx.body.q_nb.toRotationMatrix();
    const Vec3 g_b = C_nb.transpose() * gravity_n();
    const Vec3& v = ctx.body.vel_b;
    const Vec3& w = ctx.body.omega_b;
    for (std::size_t i = 0; i < gear.config().gears.size(); ++i) {
        const GearConfig& g = gear.config().gears[i];
        const Vec3 r = g.axle_b - x[kN * i] * Vec3::UnitZ();
        const Vec3 rd = -x[kN * i + 1] * Vec3::UnitZ();
        const Vec3 c0 = w.cross(v) + w.cross(w.cross(r)) + 2.0 * w.cross(rd);
        const double m = g.unsprung_mass;
        const Vec3 f(m * (g_b.x() - c0.x()), m * (g_b.y() - c0.y()), 0.0);
        t.load.F += f;
        t.load.M += r.cross(f);
        t.sddot.push_back(c0.z() - g_b.z());
    }
    return t;
}

struct Layer1Case {
    SimContext ctx;
    std::vector<double> x;
    test::GenHelGearInput gh;
};

// A random state with the main gears' tires in contact, and GenHel's input
// for the same state.
Layer1Case random_case(std::mt19937& rng, const LandingGear& gear, test::GenHelGearInput gh) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    Layer1Case c;
    gh.THETA = 0.05 * u(rng);
    gh.PHI = 0.05 * u(rng);
    gh.PSI = std::numbers::pi * (1.0 + u(rng));
    gh.XSH = 100.0 * u(rng);
    gh.YSH = 100.0 * u(rng);
    gh.WLFD = 50.0 * u(rng);
    const std::size_t ng = gh.gears.size();
    c.x.assign(kN * ng, 0.0);
    for (std::size_t i = 0; i < ng; ++i) {
        auto& g = gh.gears[i];
        const double s = (i < 2 ? 0.4 : 0.6) * (1.0 + u(rng));  // [ft], below the gas limit
        const double sd = 3.0 * u(rng);                         // [ft/s], both damper regimes
        const double dx = 0.08 * u(rng);                        // [ft], heading axes
        const double dy = 0.08 * u(rng);
        const double zf = 1.0 * u(rng);  // [ft/s]
        c.x[kN * i] = s * kFoot;
        c.x[kN * i + 1] = sd * kFoot;
        c.x[kN * i + 2] = dx * kFoot;
        c.x[kN * i + 3] = dy * kFoot;
        c.x[kN * i + 4] = zf * kFoot;
        g.DL = s;
        g.DLD = sd;
        g.DZF = zf;
    }
    // CG height: the left main tire deflected by 0.5 to 5 in.
    {
        const double target = (2.75 + 2.25 * u(rng)) / 12.0;
        const auto A = test::genhel_ahbs(gh.THETA, gh.PHI, gh.PSI);
        const auto a = test::genhel_gear_axle(gh, gh.gears[0]);
        const double z_rel = A[2][0] * a[0] + A[2][1] * a[1] + A[2][2] * (a[2] - gh.gears[0].DL);
        const double cg = std::cos(gh.THETA) * std::cos(gh.PHI);
        gh.ZSH = (target - gh.gears[0].RT / 12.0) * cg - gh.WLFD - z_rel;
    }
    c.ctx.body.pos_n = Vec3(gh.XSH, gh.YSH, gh.ZSH) * kFoot;
    c.ctx.body.vel_b = Vec3(2.0 * u(rng), 2.0 * u(rng), 1.0 * u(rng)) * kFoot;
    c.ctx.body.omega_b = Vec3(0.2 * u(rng), 0.2 * u(rng), 0.2 * u(rng));
    c.ctx.body.q_nb = attitude(gh.PHI, gh.THETA, gh.PSI);
    c.ctx.env.ground_height = gh.WLFD * kFoot;
    // Brakes anywhere from released to full, both sides alike (D-032).
    gh.brake = 0.5 * (1.0 + u(rng));
    c.ctx.controls.brake_left = gh.brake;
    c.ctx.controls.brake_right = gh.brake;
    c.ctx.air.altitude = -c.ctx.body.pos_n.z();  // NED origin at MSL
    // GenHel's retained contact points and filtered rates for the same state.
    const Mat3 C_nb = c.ctx.body.q_nb.toRotationMatrix();
    const Mat3 C_ng = Eigen::AngleAxisd(gh.PSI, Vec3::UnitZ()).toRotationMatrix();
    const Mat3 C_bg = C_nb.transpose() * C_ng;
    for (std::size_t i = 0; i < ng; ++i) {
        auto& g = gh.gears[i];
        const Vec3 d_g(c.x[kN * i + 2] / kFoot, c.x[kN * i + 3] / kFoot, 0.0);
        const Vec3 d_s = C_ng * d_g;
        const auto I = test::genhel_gear_intersection(gh, g);
        g.XID = I[0] - d_s.x();
        g.YID = I[1] - d_s.y();
        const GearState st = gear.gear_state(c.ctx, c.x, i);
        const Vec3 rate_b =
            C_bg * Vec3(st.contact_velocity[0], st.contact_velocity[1], 0.0) / kFoot;
        g.DXF = rate_b.x();
        g.DYF = rate_b.y();
    }
    c.gh = gh;
    return c;
}

// Production against the GenHel transcription. Returns the number of gears
// in contact and of slipping axes compared.
std::pair<int, int> compare_layer1(const LandingGear& gear, const Layer1Case& c,
                                   double sliding_factor) {
    // Beyond the vertical table GenHel's map clamps and production extends
    // the last slope (D-032): compare inside the tables only.
    for (std::size_t i = 0; i < c.gh.gears.size(); ++i) {
        const GearState st = gear.gear_state(c.ctx, c.x, i);
        const double last = gear.config().gears[i].tire.vertical_load.x_axis().breakpoints().back();
        if (st.vertical_deflection > last) {
            return {-1, 0};
        }
        // GenHel takes |ΔZ|; production gives no vertical load for ΔZ ≤ 0
        // (genhel_gear.hpp, departures). Only touching tires are affected.
        if (st.tire_deflection > 0.0 && st.vertical_deflection <= 0.0) {
            return {-1, 0};
        }
    }
    const test::GenHelGearResult r = test::genhel_gear(c.gh);
    // Slip is decided on the trial force: skip a state with a trial force on
    // the edge, or in the band where K_SLIP < 1 makes production slip first
    // (D-032 item 2).
    for (std::size_t i = 0; i < c.gh.gears.size(); ++i) {
        const GearState st = gear.gear_state(c.ctx, c.x, i);
        const auto& o = r.gears[i];
        if (!o.contact || std::abs(o.DLT) < 1e-9) {
            continue;
        }
        // GenHel's friction limit uses |Z_FTD|, production only a normal load
        // that pushes (genhel_gear.hpp, departures).
        if (o.FTD_trial[2] > 0.0) {
            return {-1, 0};
        }
        const double scale = std::max(1.0, Vec3(o.FTH[0], o.FTH[1], o.FTH[2]).norm());
        for (std::size_t k = 0; k < 2; ++k) {
            const double xmax = st.limit[k] / kPoundForce / sliding_factor;
            const double ft = std::abs(o.FTD_trial[k]);
            if (std::abs(ft - xmax) < 1e-6 * scale || (ft > sliding_factor * xmax && ft <= xmax)) {
                return {-1, 0};
            }
        }
    }
    const Derivs d = eval(gear, c.ctx, c.x);
    const D7Terms d7 = d7_terms(gear, c.ctx, c.x);
    int contacts = 0;
    int slips = 0;
    for (std::size_t i = 0; i < c.gh.gears.size(); ++i) {
        const GearState st = gear.gear_state(c.ctx, c.x, i);
        const auto& o = r.gears[i];
        EXPECT_NEAR(st.tire_deflection, o.DLT * kFoot, 1e-12) << "gear " << i;
        if (std::abs(o.DLT) < 1e-9) {
            continue;
        }
        EXPECT_EQ(st.in_contact, o.contact) << "gear " << i;
        if (o.contact) {
            ++contacts;
            const double scale = std::max(1.0, Vec3(o.FTH[0], o.FTH[1], o.FTH[2]).norm());
            for (int k = 0; k < 3; ++k) {
                EXPECT_NEAR(st.tire_force_b(k) / kPoundForce, o.FTH[static_cast<std::size_t>(k)],
                            1e-9 * scale)
                    << "gear " << i << " axis " << k;
            }
            EXPECT_EQ(st.slipping[0], o.slip[0]) << "gear " << i;
            EXPECT_EQ(st.slipping[1], o.slip[1]) << "gear " << i;
            slips += static_cast<int>(o.slip[0]) + static_cast<int>(o.slip[1]);
        }
        EXPECT_NEAR(st.strut_force / kPoundForce, o.FSTR, 1e-9 * std::max(1.0, std::abs(o.FSTR)));
        const double sdd = (d.xdot[kN * i + 1] - d7.sddot[i]) / kFoot;
        EXPECT_NEAR(sdd, o.DLDD, 1e-8 * std::max(1.0, std::abs(o.DLDD))) << "gear " << i;
    }
    // Loads at the CG, D-032 item 7's terms set aside.
    const Vec3 F = (d.out.load0_b.F - d7.load.F) / kPoundForce;
    const Vec3 M = (d.out.load0_b.M - d7.load.M) / (kPoundForce * kFoot);
    const double fs = std::max(1.0, Vec3(r.LG[0], r.LG[1], r.LG[2]).norm());
    for (int k = 0; k < 3; ++k) {
        EXPECT_NEAR(F(k), r.LG[static_cast<std::size_t>(k)], 1e-9 * fs) << "force " << k;
        EXPECT_NEAR(M(k), r.LG[static_cast<std::size_t>(3 + k)], 1e-9 * fs * 30.0)
            << "moment " << k;
    }
    return {contacts, slips};
}

// Layer 1 (D-032): with K_SLIP = 1 the blend has no effect, so the force
// chain, the friction check, the strut and the CG loads are GenHel's to
// round-off in every state.
TEST(LandingGear, MatchesGenHel) {
    test::GearYamlOptions o;
    o.sliding_factor = 1.0;
    const auto gear = uh60a_gear(o);
    std::mt19937 rng(20261014);
    int contacts = 0;
    int slips = 0;
    for (int n = 0; n < 4000; ++n) {
        const Layer1Case c = random_case(rng, *gear, test::genhel_uh60a_gear(1.0));
        const auto [nc, ns] = compare_layer1(*gear, c, 1.0);
        if (nc < 0) {
            continue;
        }
        contacts += nc;
        slips += ns;
        if (::testing::Test::HasFailure()) {
            FAIL() << "case " << n;
        }
    }
    // Both regimes exercised.
    EXPECT_GT(contacts, 3000);
    EXPECT_GT(slips, 500);
}

// The same with GenHel's K_SLIP = 0.85 where every contact point moves at
// least v_s on both axes, so production's limit is the sliding one.
TEST(LandingGear, MatchesGenHelSliding) {
    const auto gear = uh60a_gear();
    std::mt19937 rng(7);
    int compared = 0;
    for (int n = 0; n < 2000; ++n) {
        Layer1Case c = random_case(rng, *gear, test::genhel_uh60a_gear(0.85));
        c.ctx.body.vel_b *= 5.0;  // fast enough that |v_c| ≥ v_s mostly
        // Rebuild GenHel's rates for the new speed.
        c = [&] {
            test::GenHelGearInput gh = c.gh;
            const Mat3 C_nb = c.ctx.body.q_nb.toRotationMatrix();
            const Mat3 C_bg =
                C_nb.transpose() * Eigen::AngleAxisd(gh.PSI, Vec3::UnitZ()).toRotationMatrix();
            bool fast = true;
            for (std::size_t i = 0; i < gh.gears.size(); ++i) {
                const GearState st = gear->gear_state(c.ctx, c.x, i);
                fast = fast &&
                       std::abs(st.contact_velocity[0]) >= gear->config().sliding_blend_speed &&
                       std::abs(st.contact_velocity[1]) >= gear->config().sliding_blend_speed;
                const Vec3 rb =
                    C_bg * Vec3(st.contact_velocity[0], st.contact_velocity[1], 0.0) / kFoot;
                gh.gears[i].DXF = rb.x();
                gh.gears[i].DYF = rb.y();
            }
            Layer1Case out = c;
            out.gh = gh;
            if (!fast) {
                out.gh.gears.clear();
            }
            return out;
        }();
        if (c.gh.gears.empty()) {
            continue;
        }
        if (compare_layer1(*gear, c, 0.85).first >= 0) {
            ++compared;
        }
        if (::testing::Test::HasFailure()) {
            FAIL() << "case " << n;
        }
    }
    EXPECT_GT(compared, 500);
}

// ---- Layer 2 (D-028, D-032) ------------------------------------------------------

// Synthetic airframe for the whole-gear tests (not UH-60A mass data): 16 000
// lbf with the wheels, which the gear carries itself (D-002).
constexpr double kAirframeMass = 7000.0;  // [kg] without the wheels
MassProperties test_mass() {
    return make_mass_properties(kAirframeMass, 6000.0, 50000.0, 46000.0, 2000.0);
}

double total_mass(const LandingGear& g) {
    double m = kAirframeMass;
    for (const GearConfig& gc : g.config().gears) {
        m += gc.unsprung_mass;
    }
    return m;
}

// A constant upward force at the CG, NED-vertical: rotor thrust's stand-in.
class Lift : public ForceModel {
public:
    explicit Lift(double f) : f_(f) {}
    Wrench compute(const SimContext& ctx) const override {
        Wrench w;
        w.F = ctx.body.q_nb.toRotationMatrix().transpose() * Vec3(0.0, 0.0, -f_);
        return w;
    }

private:
    double f_;
};

// A body dashpot for the static tests: it settles the lightly damped heave
// and rocking (D-032, "to watch") and vanishes at rest, so it can't move the
// equilibrium.
class Dashpot : public ForceModel {
public:
    Wrench compute(const SimContext& ctx) const override {
        Wrench w;
        w.F = -2.0e5 * ctx.body.vel_b;
        w.M = -2.0e6 * ctx.body.omega_b;
        return w;
    }
};

struct Rig {
    LandingGear* gear = nullptr;
    std::unique_ptr<Executive> ex;
    Environment env;
    Controls controls = [] {
        Controls c;
        c.brake_left = 1.0;  // brakes set unless a test releases them
        c.brake_right = 1.0;
        return c;
    }();

    std::span<const double> gear_states() const { return ex->state().subspan(body_layout::kSize); }
    SimContext ctx() const {
        SimContext c;
        c.body = ex->body_state();
        c.env = env;
        c.controls = controls;
        c.air.altitude = -c.body.pos_n.z();
        return c;
    }
    GearState state(std::size_t i) const {
        const std::vector<double> x(gear_states().begin(), gear_states().end());
        return gear->gear_state(ctx(), x, i);
    }
    void run(double seconds) {
        const auto n = static_cast<int>(std::lround(seconds / 0.01));
        for (int k = 0; k < n; ++k) {
            (void)ex->advance(controls, env, 10'000'000);
        }
    }
};

// The gear on level ground (or a plane through the origin with `normal`), the
// CG `gap` above the point where the lowest tire would touch, at `pitch`.
Rig make_rig(LandingGearConfig gc, double lift = 0.0, double gap = 0.02, double pitch = 0.0,
             bool dashpot = false, std::int64_t step_ns = 1'000'000) {
    Rig r;
    auto gear = std::make_unique<LandingGear>(std::move(gc));
    r.gear = gear.get();
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::move(gear));
    std::vector<std::unique_ptr<ForceModel>> forces;
    if (lift != 0.0) {
        forces.push_back(std::make_unique<Lift>(lift));
    }
    if (dashpot) {
        forces.push_back(std::make_unique<Dashpot>());
    }
    ExecutiveConfig ec;
    ec.mass = test_mass();
    ec.step_ns = step_ns;
    ec.max_substeps = static_cast<std::int64_t>(10'000'000 / step_ns);
    r.ex = std::make_unique<Executive>(ec, std::move(comps), std::move(forces));
    r.env.ground_height = 0.0;
    // Height: the lowest tire bottom `gap` above the plane, measured along the normal.
    BodyState b;
    b.q_nb = attitude(0.0, pitch, 0.0);
    const Mat3 C_nb = b.q_nb.toRotationMatrix();
    const Vec3 u = r.gear->config().ground_normal_n;
    double lowest = 1e9;
    for (const GearConfig& g : r.gear->config().gears) {
        const Vec3 bottom = C_nb * (g.axle_b + g.tire.radius * Vec3::UnitZ());
        lowest = std::min(lowest, u.dot(bottom));
    }
    b.pos_n = u * (gap - lowest);
    r.ex->set_body_state(b);
    return r;
}

// Three contact points are statically determinate: with N_i the ground's
// normal loads at the contact points (x_i, y_i) in level ground axes about
// the CG, ΣN = W − T, Σ N x = Σ m g x_ax, Σ N y = Σ m g y_ax, the wheels'
// weights acting at their axles (D-002). The test's dashpot settles the body
// in 40 s; it vanishes at rest.
TEST(LandingGear, StaticLoadSplit) {
    for (const double lift_fraction : {0.0, 0.5}) {
        const auto proto = uh60a_gear();
        const double W = total_mass(*proto) * units::kStandardGravity;
        Rig r = make_rig(proto->config(), lift_fraction * W, 0.02, 0.0, true);
        r.run(40.0);
        const Mat3 C_nb = r.ex->body_state().q_nb.toRotationMatrix();
        Eigen::Matrix3d A;
        Vec3 rhs((1.0 - lift_fraction) * W, 0.0, 0.0);
        std::array<double, 3> n{};
        for (std::size_t i = 0; i < 3; ++i) {
            const GearState st = r.state(i);
            ASSERT_TRUE(st.in_contact) << i;
            const Vec3 p = C_nb * st.contact_point_b;  // NED, from the CG
            A(0, static_cast<Eigen::Index>(i)) = 1.0;
            A(1, static_cast<Eigen::Index>(i)) = p.x();
            A(2, static_cast<Eigen::Index>(i)) = p.y();
            n[i] = st.normal_load;
            const Vec3 axle = C_nb * st.axle_b;
            const double wheel = r.gear->config().gears[i].unsprung_mass * units::kStandardGravity;
            rhs.y() += wheel * axle.x();
            rhs.z() += wheel * axle.y();
        }
        const Vec3 expected = A.colPivHouseholderQr().solve(rhs);
        for (std::size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(n[i], expected(static_cast<Eigen::Index>(i)), 1e-6 * W)
                << "gear " << i << " lift " << lift_fraction;
        }
        EXPECT_NEAR(n[0] + n[1] + n[2], (1.0 - lift_fraction) * W, 1e-6 * W);
        // At rest.
        EXPECT_LT(r.ex->body_state().vel_b.norm(), 1e-6);
    }
}

// Preload (D-032 item 6): below it the strut stays at the stop and passes the
// tire load through; above it the air spring carries it.
TEST(LandingGear, PreloadPassThrough) {
    const auto proto = uh60a_gear();
    const double W = total_mass(*proto) * units::kStandardGravity;
    {
        // 75% lift: the mains carry about 1700 lbf each, below their 3000 lbf preload.
        Rig r = make_rig(proto->config(), 0.75 * W, 0.02, 0.0, true);
        r.run(40.0);
        for (std::size_t i = 0; i < 2; ++i) {
            const GearState st = r.state(i);
            const double s = r.gear_states()[kN * i];
            const double tire = -st.tire_force_b.z();
            ASSERT_LT(tire, 3000.0 * kPoundForce);
            EXPECT_LE(s, 0.0);
            // The stop's spring carries the preload's excess: F_PRE − F_S over k_stop.
            EXPECT_GT(s, -0.1 * units::kInch);
            const double m = r.gear->config().gears[i].unsprung_mass;
            EXPECT_NEAR(st.strut_force, tire - m * units::kStandardGravity, 1e-6 * tire);
        }
    }
    {
        Rig r = make_rig(proto->config(), 0.0, 0.02, 0.0, true);
        r.run(40.0);
        for (std::size_t i = 0; i < 2; ++i) {
            const StrutConfig& sc = r.gear->config().gears[i].strut;
            const double s = r.gear_states()[kN * i];
            EXPECT_GT(s, 0.0);
            // Static: F_S = C_PV/(C_PV/F_PRE − s).
            EXPECT_NEAR(r.state(i).strut_force, sc.air_spring / (sc.air_spring / sc.preload - s),
                        1e-6 * r.state(i).strut_force);
        }
    }
}

// The coupling with the body acceleration (D-032 item 7) against
// d'Alembert's principle for the wheel written independently: load =
// F_tire + m g − m a_P with a_P from v̇, ω̇ and s̈ = s̈0 + G a.
TEST(LandingGear, AccelerationCouplingIsDAlembert) {
    const auto gear = uh60a_gear();
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    for (int n = 0; n < 200; ++n) {
        const Layer1Case c = random_case(rng, *gear, test::genhel_uh60a_gear());
        const Derivs d = eval(*gear, c.ctx, c.x);
        Vec6 a;
        for (Eigen::Index k = 0; k < 6; ++k) {
            a(k) = 5.0 * u(rng);
        }
        const Vec6 load = (Vec6() << d.out.load0_b.F, d.out.load0_b.M).finished() + d.out.H * a;
        const Mat3 C_nb = c.ctx.body.q_nb.toRotationMatrix();
        const Vec3 g_b = C_nb.transpose() * gravity_n();
        const Vec3& v = c.ctx.body.vel_b;
        const Vec3& w = c.ctx.body.omega_b;
        Vec3 F = Vec3::Zero();
        Vec3 M = Vec3::Zero();
        for (std::size_t i = 0; i < 3; ++i) {
            const GearState st = gear->gear_state(c.ctx, c.x, i);
            const double m = gear->config().gears[i].unsprung_mass;
            const Vec3 r = st.axle_b;
            const Vec3 rd(0.0, 0.0, -c.x[kN * i + 1]);
            double sdd = d.xdot[kN * i + 1];
            for (std::size_t k = 0; k < 6; ++k) {
                sdd += d.G[6 * (kN * i + 1) + k] * a(static_cast<Eigen::Index>(k));
            }
            const Vec3 a_p = a.head<3>() + w.cross(v) + a.tail<3>().cross(r) + w.cross(w.cross(r)) +
                             2.0 * w.cross(rd) - sdd * Vec3::UnitZ();
            const Vec3 f_wheel = m * (g_b - a_p);  // weight minus inertia
            F += st.tire_force_b + f_wheel;
            M += st.contact_point_b.cross(st.tire_force_b) + r.cross(f_wheel);
        }
        const double scale = std::max(1.0, F.norm());
        for (Eigen::Index k = 0; k < 3; ++k) {
            EXPECT_NEAR(load(k), F(k), 1e-9 * scale) << n;
            EXPECT_NEAR(load(3 + k), M(k), 1e-8 * scale) << n;
        }
    }
}

void drop_energy_case(std::int64_t step_ns, double bound);

// Energy (D-028 layer 2): with damping and friction off, a gear under the CG
// dropped from 0.3 m conserves kinetic + potential + strut + tire energy. The
// model is conservative; what RK4 loses comes from the kinks (touchdown, the
// tire table's breakpoints, the top stop) and falls with the step: 7.7e-4 of
// the exchanged energy at 1 kHz, 5.7e-6 at 8 kHz (VALIDATION.md M6c).
TEST(LandingGear, DropConservesEnergy) {
    for (const auto& [step_ns, bound] : {std::pair<std::int64_t, double>{1'000'000, 1e-3},
                                         std::pair<std::int64_t, double>{125'000, 1e-5}}) {
        drop_energy_case(step_ns, bound);
    }
}

void drop_energy_case(std::int64_t step_ns, double bound) {
    const auto proto = uh60a_gear();
    LandingGearConfig c = proto->config();
    GearConfig g = c.gears[0];
    g.axle_b = Vec3(0.0, 0.0, g.axle_b.z());
    g.tire.damping_stiffness = Table1D(Axis({0.0, 1.0}), {0.0, 0.0});
    g.strut.low = {};
    g.strut.high = {};
    g.strut.stop_damping = 0.0;
    c.gears = {g};
    c.friction_coefficient = 0.0;
    // Lift two thirds of the weight: the gear carries a main gear's share.
    const double lift = 2.0 / 3.0 * (kAirframeMass + g.unsprung_mass) * units::kStandardGravity;
    Rig r = make_rig(c, lift, 0.3, 0.0, false, step_ns);
    // Start the wheel hanging at its equilibrium on the stop.
    const double m = g.unsprung_mass;
    const double k_stop = r.gear->stop_stiffness(0);
    const StrutConfig& sc = g.strut;
    const auto f_strut = [&](double s) {
        return sc.air_spring / (sc.air_spring / sc.preload - s) + (s < 0.0 ? k_stop * s : 0.0);
    };
    double s0 = -(sc.preload + m * units::kStandardGravity) / k_stop;
    for (int it = 0; it < 50; ++it) {
        const double f = f_strut(s0) + m * units::kStandardGravity;
        const double df = sc.air_spring / std::pow(sc.air_spring / sc.preload - s0, 2) + k_stop;
        s0 -= f / df;
    }
    std::vector<double> x(r.ex->state().begin(), r.ex->state().end());
    x[body_layout::kSize] = s0;
    ASSERT_TRUE(r.ex->restore(x, 0, 0));
    const double L0 = sc.air_spring / sc.preload;
    const auto strut_energy = [&](double s) {
        // ∫₀ˢ F_S ds, with the air spring continued below zero.
        return sc.air_spring * std::log(L0 / (L0 - s)) + (s < 0.0 ? 0.5 * k_stop * s * s : 0.0);
    };
    const auto tire_energy = [&](double d) {
        double e = 0.0;
        const double step = d / 2000.0;
        for (int k = 0; k < 2000; ++k) {  // midpoint rule on a piecewise-linear curve
            e += tire_vertical_load(g.tire.vertical_load, (k + 0.5) * step) * step;
        }
        return d > 0.0 ? e : 0.0;
    };
    const auto energy = [&] {
        const BodyState b = r.ex->body_state();
        const double s = r.gear_states()[0];
        const double sd = r.gear_states()[1];
        const double vz_wheel = b.vel_b.z() - sd;
        const double z_wheel = b.pos_n.z() + g.axle_b.z() - s;
        return 0.5 * kAirframeMass * b.vel_b.squaredNorm() + 0.5 * m * vz_wheel * vz_wheel -
               units::kStandardGravity * (kAirframeMass * b.pos_n.z() + m * z_wheel) +
               lift * b.pos_n.z() + strut_energy(s) + tire_energy(r.state(0).tire_deflection);
    };
    const double e0 = energy();
    const double scale = (kAirframeMass * units::kStandardGravity - lift) * 0.3;
    double worst = 0.0;
    double max_tire = 0.0;
    for (int k = 0; k < 300; ++k) {
        r.run(0.01);
        worst = std::max(worst, std::abs(energy() - e0));
        max_tire = std::max(max_tire, r.state(0).tire_deflection);
    }
    EXPECT_GT(max_tire, 0.02);  // the tire was used
    EXPECT_LT(worst, bound * scale) << "worst drift " << worst << " J of " << scale;
    // Pure heave: no attitude change.
    EXPECT_LT(r.ex->body_state().omega_b.norm(), 1e-12);
}

// Slope (D-028 layer 2): every wheel braked (a synthetic layout, not the
// UH-60A's castering tail wheel), nose downhill. Below atan μ the aircraft
// holds; above it, it slides with the sliding friction K_SLIP μ N.
TEST(LandingGear, SlopeHoldAndSlide) {
    test::GearYamlOptions o;
    o.tail_brake = "left";
    const auto proto = uh60a_gear(o);
    const double mu = proto->config().friction_coefficient;
    const double kslip = proto->config().sliding_factor;
    const double g0 = units::kStandardGravity;
    for (const double angle : {0.8 * std::atan(mu), std::atan(mu) + 3.0 * units::kDegree}) {
        LandingGearConfig c = proto->config();
        // The plane falls towards +x (north): normal tilted back by `angle`.
        c.ground_normal_n = Vec3(std::sin(angle), 0.0, -std::cos(angle));
        Rig r = make_rig(c, 0.0, 0.01, -angle);
        r.run(5.0);
        const Vec3 down_slope(std::cos(angle), 0.0, std::sin(angle));
        if (angle < std::atan(mu)) {
            // The heave and rocking decay slowly (D-032, "to watch"), so compare
            // mean positions over two 5 s windows, and require every tire to stick.
            const auto window = [&] {
                double sum = 0.0;
                for (int k = 0; k < 500; ++k) {
                    r.run(0.01);
                    sum += down_slope.dot(r.ex->body_state().pos_n);
                    for (std::size_t i = 0; i < 3; ++i) {
                        EXPECT_FALSE(r.state(i).slipping[0] || r.state(i).slipping[1]) << i;
                    }
                }
                return sum / 500.0;
            };
            const double m1 = window();
            const double m2 = window();
            EXPECT_LT(std::abs(m2 - m1), 1e-4) << "creep over 5 s [m]";
            continue;
        }
        const Mat3 C_nb = r.ex->body_state().q_nb.toRotationMatrix();
        const double v1 = down_slope.dot(C_nb * r.ex->body_state().vel_b);
        const double x1 = down_slope.dot(r.ex->body_state().pos_n);
        r.run(5.0);
        const Mat3 C_nb2 = r.ex->body_state().q_nb.toRotationMatrix();
        const double v2 = down_slope.dot(C_nb2 * r.ex->body_state().vel_b);
        const double x2 = down_slope.dot(r.ex->body_state().pos_n);
        (void)x1;
        (void)x2;
        const double expected = g0 * (std::sin(angle) - kslip * mu * std::cos(angle));
        EXPECT_NEAR((v2 - v1) / 5.0, expected, 0.02 * expected);
    }
}

// Lift-off (D-032 item 1): out of contact the unloaded tire relaxes,
// δ' = −δ/G, so the in-plane deflection is gone within a few G.
TEST(LandingGear, LiftOffClearsDeflection) {
    const auto gear = uh60a_gear();
    SimContext ctx;
    ctx.env.ground_height = -100.0;  // far below
    ctx.air.altitude = 0.0;
    std::vector<double> x(gear->num_states(), 0.0);
    x[2] = 0.02;
    x[3] = -0.015;
    const double G = gear->config().gears[0].tire.damping_factor;
    const Derivs d = eval(*gear, ctx, x);
    EXPECT_DOUBLE_EQ(d.xdot[2], -0.02 / G);
    EXPECT_DOUBLE_EQ(d.xdot[3], 0.015 / G);
    EXPECT_FALSE(gear->gear_state(ctx, x, 0).in_contact);
}

// Wheel hop at 1 kHz against a fine step (D-028: h ω ≈ 0.15): the stroke
// history agrees to 0.1% of its swing over 0.2 s, body held fixed.
TEST(LandingGear, WheelHopRk4Accuracy) {
    const auto gear = uh60a_gear();
    SimContext ctx;
    ctx.env.ground_height = 0.0;
    const GearConfig& g = gear->config().gears[0];
    // Main tires 2 in into the ground, the tail clear.
    ctx.body.pos_n = Vec3(0.0, 0.0, -(g.axle_b.z() + g.tire.radius - 2.0 * units::kInch));
    ctx.air.altitude = -ctx.body.pos_n.z();
    const auto simulate = [&](double h, double T, std::vector<double>& out) {
        std::vector<double> x(gear->num_states(), 0.0);
        x[0] = 0.05;  // stroked 5 cm, released
        const auto f = [&](const std::vector<double>& s) { return eval(*gear, ctx, s).xdot; };
        const auto n = static_cast<int>(std::lround(T / h));
        const int every = static_cast<int>(std::lround(1e-3 / h));
        for (int k = 0; k < n; ++k) {
            const auto k1 = f(x);
            std::vector<double> t(x.size());
            for (std::size_t j = 0; j < x.size(); ++j)
                t[j] = x[j] + 0.5 * h * k1[j];
            const auto k2 = f(t);
            for (std::size_t j = 0; j < x.size(); ++j)
                t[j] = x[j] + 0.5 * h * k2[j];
            const auto k3 = f(t);
            for (std::size_t j = 0; j < x.size(); ++j)
                t[j] = x[j] + h * k3[j];
            const auto k4 = f(t);
            for (std::size_t j = 0; j < x.size(); ++j) {
                x[j] += h / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]);
            }
            if ((k + 1) % every == 0) {
                out.push_back(x[0]);
            }
        }
    };
    std::vector<double> coarse, fine;
    simulate(1e-3, 0.2, coarse);
    simulate(1e-5, 0.2, fine);
    ASSERT_EQ(coarse.size(), fine.size());
    double swing = 0.0;
    double err = 0.0;
    for (std::size_t k = 0; k < fine.size(); ++k) {
        swing = std::max(swing, std::abs(fine[k] - fine.back()));
        err = std::max(err, std::abs(coarse[k] - fine[k]));
    }
    EXPECT_GT(swing, 0.01);
    EXPECT_LT(err, 1e-3 * swing) << "error " << err << " m of swing " << swing;
}

// The sliding blend (D-032 item 2): breakout at rest, K_SLIP of it at and
// above v_s, smooth in between.
TEST(LandingGear, FrictionLimitBlend) {
    const auto gear = uh60a_gear();
    const LandingGearConfig& c = gear->config();
    SimContext ctx;
    ctx.env.ground_height = 0.0;
    const GearConfig& g = c.gears[0];
    ctx.body.pos_n = Vec3(0.0, 0.0, -(g.axle_b.z() + g.tire.radius - 2.0 * units::kInch));
    ctx.controls.brake_left = 1.0;
    ctx.controls.brake_right = 1.0;
    ctx.air.altitude = -ctx.body.pos_n.z();
    const std::vector<double> x(gear->num_states(), 0.0);
    double prev = 2.0;
    for (const double frac : {0.0, 0.25, 0.5, 0.75, 1.0, 2.0}) {
        ctx.body.vel_b = Vec3(frac * c.sliding_blend_speed, 0.0, 0.0);
        const GearState st = gear->gear_state(ctx, x, 0);
        const double breakout = c.brake_factor_set * c.friction_coefficient * st.normal_load;
        const double ratio = st.limit[0] / breakout;
        const double t = std::min(frac, 1.0);
        EXPECT_NEAR(ratio, 1.0 - (1.0 - c.sliding_factor) * t * t * (3.0 - 2.0 * t), 1e-12);
        EXPECT_LE(ratio, prev);
        prev = ratio;
        // The tail wheel is never braked: N_BRK.
        const GearState tail = gear->gear_state(ctx, x, 2);
        if (tail.in_contact) {
            const double tt =
                std::min(std::abs(tail.contact_velocity[0]) / c.sliding_blend_speed, 1.0);
            EXPECT_NEAR(tail.limit[0],
                        c.brake_factor_off * c.friction_coefficient * tail.normal_load *
                            (1.0 - (1.0 - c.sliding_factor) * tt * tt * (3.0 - 2.0 * tt)),
                        1e-9 * tail.normal_load);
        }
    }
}

// The default top stop (D-032 item 6): 10 × the tire's slope at zero
// deflection, critically damped with the wheel. Main tire at 140 psi:
// 3800 + (20/30)·800 = 4333.3 lbf/in (Table 7.5.3).
TEST(LandingGear, DefaultTopStop) {
    const auto gear = uh60a_gear();
    const double k = gear->stop_stiffness(0);
    EXPECT_NEAR(k / (kPoundForce / units::kInch), 43333.333333, 1e-3);
    EXPECT_NEAR(gear->stop_damping(0), 2.0 * std::sqrt(k * gear->config().gears[0].unsprung_mass),
                1e-9);
}

// Purity and real-time rules: the same inputs give bit-identical results,
// and nothing allocates.
TEST(LandingGear, PureAndAllocationFree) {
    const auto gear = uh60a_gear();
    std::mt19937 rng(3);
    const Layer1Case c = random_case(rng, *gear, test::genhel_uh60a_gear());
    Derivs a(gear->num_states());
    Derivs b(gear->num_states());
    {
        const test::AllocationCounter count;
        gear->derivatives(c.ctx, c.x, a.out);
        EXPECT_EQ(count.count(), 0U);
    }
    gear->derivatives(c.ctx, c.x, b.out);
    EXPECT_EQ(a.xdot, b.xdot);
    EXPECT_EQ(a.G, b.G);
    EXPECT_EQ(a.out.load0_b.F, b.out.load0_b.F);
    EXPECT_EQ(a.out.load0_b.M, b.out.load0_b.M);
    EXPECT_EQ(a.out.H, b.out.H);
}

// Snapshot and restore mid-slide give the same run, bit for bit.
TEST(LandingGear, RestoreMidSlideIsBitIdentical) {
    test::GearYamlOptions o;
    o.tail_brake = "left";
    LandingGearConfig c = uh60a_gear(o)->config();
    const double angle = 35.0 * units::kDegree;
    c.ground_normal_n = Vec3(std::sin(angle), 0.0, -std::cos(angle));
    Rig r = make_rig(c, 0.0, 0.01, -angle);
    r.run(1.0);
    const std::vector<double> saved(r.ex->state().begin(), r.ex->state().end());
    const auto n = r.ex->n_steps();
    r.run(0.5);
    const std::vector<double> first(r.ex->state().begin(), r.ex->state().end());
    ASSERT_TRUE(r.ex->restore(saved, n, 0));
    r.run(0.5);
    const std::vector<double> second(r.ex->state().begin(), r.ex->state().end());
    EXPECT_EQ(first, second);
    EXPECT_TRUE(r.state(0).slipping[0]);
}

TEST(LandingGear, RejectsInvalidConfig) {
    const auto base = uh60a_gear()->config();
    const auto expect_throw = [&](auto&& edit) {
        LandingGearConfig c = base;
        edit(c);
        EXPECT_THROW((void)LandingGear(c), InitError);
    };
    expect_throw([](LandingGearConfig& c) { c.gears.clear(); });
    expect_throw([](LandingGearConfig& c) { c.sliding_factor = 0.0; });
    expect_throw([](LandingGearConfig& c) { c.sliding_factor = 1.5; });
    expect_throw([](LandingGearConfig& c) { c.friction_coefficient = -0.1; });
    expect_throw([](LandingGearConfig& c) { c.sliding_blend_speed = 0.0; });
    expect_throw([](LandingGearConfig& c) { c.ground_normal_n = Vec3(0.0, 0.0, 1.0); });
    expect_throw([](LandingGearConfig& c) { c.gears[0].unsprung_mass = 0.0; });
    expect_throw([](LandingGearConfig& c) { c.gears[0].tire.radius = -1.0; });
    expect_throw([](LandingGearConfig& c) { c.gears[0].tire.damping_factor = 0.0; });
    expect_throw([](LandingGearConfig& c) { c.gears[0].strut.preload = 0.0; });
    expect_throw([](LandingGearConfig& c) {
        c.gears[0].tire.vertical_load = Table1D(Axis({0.0, 1.0}), {1.0, 2.0});
    });
    expect_throw([](LandingGearConfig& c) { c.gears[2].tire.lateral_reference_load = 0.0; });
    // Config reader: a wheel's brake is left, right or none.
    test::GearYamlOptions o;
    o.tail_brake = "both";
    EXPECT_THROW((void)uh60a_gear(o), InitError);
}

// Brake inputs (D-032): each main wheel follows its own side, k_B blends
// linearly from GenHel's N_BRK (released) to B_RK (full), inputs outside
// [0, 1] are clamped and NaN counts as released; the tail wheel has none.
TEST(LandingGear, BrakeInputs) {
    const auto gear = uh60a_gear();
    const LandingGearConfig& c = gear->config();
    SimContext ctx;
    ctx.env.ground_height = 0.0;
    const GearConfig& g = c.gears[0];
    ctx.body.pos_n = Vec3(0.0, 0.0, -(g.axle_b.z() + g.tire.radius - 2.0 * units::kInch));
    ctx.air.altitude = -ctx.body.pos_n.z();
    const std::vector<double> x(gear->num_states(), 0.0);
    const auto kb = [&](std::size_t i) {
        const GearState st = gear->gear_state(ctx, x, i);
        return st.limit[0] / (c.friction_coefficient * st.normal_load);
    };
    const double off = c.brake_factor_off;
    const double set = c.brake_factor_set;
    for (const auto& [left, right, kl, kr] :
         {std::tuple{0.0, 0.0, off, off}, std::tuple{1.0, 0.0, set, off},
          std::tuple{0.0, 1.0, off, set},
          std::tuple{0.25, 0.5, off + 0.25 * (set - off), off + 0.5 * (set - off)},
          std::tuple{2.0, -1.0, set, off},
          std::tuple{std::numeric_limits<double>::quiet_NaN(), 1.0, off, set}}) {
        ctx.controls.brake_left = left;
        ctx.controls.brake_right = right;
        EXPECT_NEAR(kb(0), kl, 1e-12) << left << ' ' << right;
        EXPECT_NEAR(kb(1), kr, 1e-12) << left << ' ' << right;
        EXPECT_NEAR(kb(2), off, 1e-12);  // tail wheel
    }
}

TEST(LandingGear, ReadsConfig) {
    const auto gear = uh60a_gear();
    const LandingGearConfig& c = gear->config();
    EXPECT_EQ(gear->name(), "gear");
    ASSERT_EQ(c.gears.size(), 3U);
    EXPECT_EQ(gear->num_states(), 15U);
    EXPECT_DOUBLE_EQ(c.sliding_blend_speed, 0.1 * kFoot);  // D-032 default
    EXPECT_NEAR(c.gears[0].axle_b.x(), (355.9 - 297.43) * units::kInch, 1e-12);
    EXPECT_NEAR(c.gears[0].axle_b.y(), -53.2 * units::kInch, 1e-12);
    EXPECT_NEAR(c.gears[2].axle_b.z(), (248.2 - 180.4) * units::kInch, 1e-12);
    EXPECT_EQ(c.gears[0].brake, BrakeSide::left);
    EXPECT_EQ(c.gears[1].brake, BrakeSide::right);
    EXPECT_EQ(c.gears[2].brake, BrakeSide::none);
    EXPECT_FALSE(c.gears[2].tire.lateral_stiffness.has_value());
    // The 140 psi slice of Table 7.5.3 is bilinear: at 1 in, 3800 + (20/30)·800 lbf.
    EXPECT_NEAR(c.gears[0].tire.vertical_load(units::kInch) / kPoundForce,
                3800.0 + 800.0 * 2.0 / 3.0, 1e-9);
}

}  // namespace
}  // namespace eyas
