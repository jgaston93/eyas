// M9 Stage A: rotor-speed degrees of freedom (D-052). The executive's shafts
// against closed forms and conservation, the rotor's shaft terms against
// their own loads, the ideal drive against the constant-speed rotor, and the
// config and real-time rules.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "core/rigid_body.hpp"
#include "eyas/model.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "support/flap_momentum.hpp"
#include "support/flap_reference.hpp"
#include "support/m3_fixtures.hpp"
#include "support/m4_fixtures.hpp"
#include "util/config.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using test::kDeg;

// An internal motor between the airframe and shaft 0: torque Q on the shaft,
// its reaction inside the airframe-plus-drivetrain system (D-052), so no
// load on the airframe.
class Motor : public DynamicComponent {
public:
    explicit Motor(double q) : q_(q) {}
    std::size_t num_states() const override { return 0; }
    void initial_states(std::span<double> /*x*/) const override {}
    void derivatives(const SimContext& /*ctx*/, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        out.Q0(0) = q_;
    }

private:
    double q_;
};

ShaftConfig free_shaft(double speed, double inertia, const Vec3& axis) {
    ShaftConfig s;
    s.name = "main";
    s.speed = speed;
    s.inertia = inertia;
    s.axis_b = axis.normalized();
    s.drive = ShaftDrive::free;
    return s;
}

// Body plus one free shaft with the motor, from rest: angular momentum about
// the CG, I_b ω + J Ω n (the parts' rigid inertia is in I_b), is conserved,
// I_b ω̇ + J Ω̇ n = 0, and the shaft's parts obey J(Ω̇ + n·ω̇) = Q. So
//   Ω̇ = Q / (J (1 − J nᵀ I_b⁻¹ n)),  ω̇ = −J Ω̇ I_b⁻¹ n,
// for any axis and inertia tensor; along a principal axis z,
// Ω̇ = Q I_zz / (J (I_zz − J)) and ṙ = −Q / (I_zz − J).
TEST(Shafts, FreeShaftWithInternalMotorMatchesClosedForm) {
    const double Q = 2500.0;
    const double J = 40.0;
    for (const Vec3& axis : {Vec3(0.0, 0.0, -1.0), Vec3(0.3, -0.2, -1.0)}) {
        ExecutiveConfig ec;
        ec.mass = make_mass_properties(5000.0, 6000.0, 30000.0, 28000.0, 1500.0);
        ec.shafts = {free_shaft(27.0, J, axis)};
        std::vector<std::unique_ptr<DynamicComponent>> comps;
        comps.push_back(std::make_unique<Motor>(Q));
        const Executive ex(ec, std::move(comps));
        std::vector<double> xd(ex.num_states(), 0.0);
        ASSERT_TRUE(ex.derivatives(0.0, ex.state(), {}, {}, xd));
        const Vec3 n = axis.normalized();
        const Mat3& I = ec.mass.inertia_b;
        const Vec3 In = I.inverse() * n;
        const double omega_dot = Q / (J * (1.0 - J * n.dot(In)));
        const Vec3 w_dot = -J * omega_dot * In;
        EXPECT_NEAR(xd[ex.shaft_offset(0) + 1], omega_dot, 1e-12 * std::abs(omega_dot));
        EXPECT_EQ(xd[ex.shaft_offset(0)], 27.0);  // ψ̇ = Ω
        for (int k = 0; k < 3; ++k) {
            EXPECT_NEAR(xd[body_layout::kOmega + static_cast<std::size_t>(k)], w_dot(k),
                        1e-12 * w_dot.norm());
        }
    }
}

// Airframe tumbling with flapping blades on a free shaft with lumped parts, in
// vacuum (D-052): no torque from outside the system, so the angular
// momentum about the system CG is conserved and only the weight changes the
// linear momentum. The blades' in-plane inertia under Ω̇ (the rotor's H_s),
// its shaft torque (Q0, K_a, K_s) and the parts' terms all enter; a wrong one
// gives an O(1) drift, RK4 truncation a drift falling as h⁴. As the M4 check
// (FlappingRotor.CoupledSystemConservesMomentum), with the shaft started off
// its governed speed so the blades accelerate relative to the hub.
TEST(Shafts, RotorOnFreeShaftConservesMomentum) {
    const auto airfoil = test::airfoil_from(test::zero_airfoil_csv());
    const double e = 0.4;
    const test::Beads beads = test::nonuniform_beads(8.0 - e);
    BladeElementRotorConfig c = test::flap_rotor(airfoil, {}, beads, e, 8);
    c.hub_position_b = Vec3(0.3, 0.1, -2.0);
    c.hub_pitch = -4 * kDeg;
    c.flap->spring = 5e3;
    c.flap->damper = 200.0;
    c.shaft = BladeElementRotorConfig::ShaftLink{"main", 1.0};
    const MassProperties mp = make_mass_properties(900.0, 600.0, 1500.0, 1400.0, 120.0);

    const auto run = [&](std::int64_t step_ns, double& dP, double& dH, double& dOmega) {
        ExecutiveConfig ec;
        ec.mass = mp;
        ec.step_ns = step_ns;
        ec.max_substeps = 100000;
        ec.shafts = {free_shaft(c.rotor_speed, 25.0, Vec3(0.1, 0.05, -1.0))};
        std::vector<std::unique_ptr<DynamicComponent>> comps;
        comps.push_back(std::make_unique<BladeElementRotor>(c));
        const auto* rotor = dynamic_cast<const BladeElementRotor*>(comps.back().get());
        Executive ex(ec, std::move(comps));
        BodyState b;
        b.vel_b = Vec3(3.0, -1.0, 2.0);
        b.omega_b = Vec3(0.6, -0.4, 0.8);
        b.q_nb = Quat(Eigen::AngleAxisd(0.3, Vec3(1, 2, 3).normalized()));
        ex.set_body_state(b);
        std::vector<double> x(ex.state().begin(), ex.state().end());
        for (std::size_t k = 0; k < 4; ++k) {
            x[body_layout::kSize + 2 * k] = 0.05 * (1.0 + static_cast<double>(k));
            x[body_layout::kSize + 2 * k + 1] = 0.3 - 0.2 * static_cast<double>(k);
        }
        x[ex.shaft_offset(0) + 1] = 0.9 * c.rotor_speed;
        ASSERT_TRUE(ex.restore(x, 0, 0));
        const test::FlapReference ref(*rotor, beads);
        const test::Momentum m0 = test::system_momentum(ex, *rotor, ref, beads, mp);
        const double omega0 = ex.state()[ex.shaft_offset(0) + 1];
        const auto st = ex.advance({}, {}, 1'000'000'000);  // 1 s
        ASSERT_FALSE(st.non_finite);
        const test::Momentum m1 = test::system_momentum(ex, *rotor, ref, beads, mp);
        dP = (m1.P_n - (m0.P_n + m0.mass * gravity_n() * 1.0)).norm() / (m0.mass * 10.0);
        dH = (m1.H_cg_n - m0.H_cg_n).norm() / m0.H_cg_n.norm();
        dOmega = ex.state()[ex.shaft_offset(0) + 1] - omega0;
    };
    double dP1 = 0.0;
    double dH1 = 0.0;
    double dO1 = 0.0;
    double dP2 = 0.0;
    double dH2 = 0.0;
    double dO2 = 0.0;
    run(1'000'000, dP1, dH1, dO1);
    run(500'000, dP2, dH2, dO2);
    // The shaft does move: the airframe's tumbling exchanges momentum with it.
    EXPECT_GT(std::abs(dO1), 1e-3);
    EXPECT_LT(dH1, 1e-7);
    EXPECT_LT(dP1, 1e-9);
    if (dH2 > 1e-13) {
        EXPECT_GT(dH1 / dH2, 10.0);
        EXPECT_LT(dH1 / dH2, 25.0);
    }
}

// The rotor's shaft terms (D-052): conservative inertial coupling makes the
// coupled matrix symmetric, so the shaft torque's sensitivity to the body
// acceleration (K_a) equals the airframe load's sensitivity to the shaft
// acceleration (H_s); its Q0 in vacuum with the body at rest is zero, and
// with air it is minus the gear times the aerodynamic torque the rotor needs
// (RotorLoads::torque), and K_s is minus the gear² times the blades'
// inertia about the shaft.
TEST(Shafts, RotorShaftTermsAreConsistent) {
    const double e = 0.4;
    const test::Beads beads = test::nonuniform_beads(8.0 - e);
    for (const bool air : {false, true}) {
        const auto airfoil = test::airfoil_from(air ? test::linear_airfoil_csv(5.7, 0.01)
                                                    : test::zero_airfoil_csv());
        BladeElementRotorConfig c = test::flap_rotor(airfoil, {12 * kDeg, 0.0, 0.0}, beads, e, 20);
        c.hub_position_b = Vec3(0.3, 0.1, -2.0);
        c.hub_pitch = -4 * kDeg;
        const double gear = 1.5;
        c.shaft = BladeElementRotorConfig::ShaftLink{"main", gear};
        BladeElementRotor rotor(c);
        const std::vector<ShaftInfo> info = {{"main", c.rotor_speed / gear}};
        rotor.bind_shafts(info);
        ASSERT_EQ(rotor.shaft_index(), 0);
        SimContext ctx = test::sea_level_context(0.0);
        ctx.num_shafts = 1;
        ctx.shafts[0] = {0.3, c.rotor_speed / gear};
        if (air) {
            ctx.body.vel_b = Vec3(20.0, 2.0, 1.0);
        }
        const std::vector<double> x = {0.05, 0.2, 0.06, -0.1, 0.04, 0.05, 0.07, 0.0};
        std::vector<double> xd(x.size(), 0.0);
        std::vector<double> G(6 * x.size(), 0.0);
        std::vector<double> Gs(kMaxShafts * x.size(), 0.0);
        ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
        d.Gs = Gs;
        rotor.derivatives(ctx, x, d);
        const double scale = d.Hs.col(0).norm();
        ASSERT_GT(scale, 0.0);
        for (int j = 0; j < 6; ++j) {
            EXPECT_NEAR(d.Ka(0, j), d.Hs(j, 0), 1e-9 * scale) << "air " << air << ", j " << j;
        }
        // −gear² Σ (S e + I cos β)(e + ρ cos β) dm about the axis, flapping
        // small: the blades' in-plane inertia, gear² (m e² + 2 S e + I).
        const FlapHinge& f = *c.flap;
        const double blades = 4.0 * (f.mass * e * e + 2.0 * f.first_moment * e + f.inertia);
        EXPECT_NEAR(-d.Ks(0, 0) / (gear * gear), blades, 0.01 * blades);
        // The flap rows don't depend on Ω̇ (n·e_t = 0, D-052).
        for (std::size_t r = 0; r < x.size(); ++r) {
            EXPECT_EQ(Gs[r * kMaxShafts], 0.0) << r;
        }
    }
    // Massless rotor in air, body at rest in hover: Q0 = −gear × torque.
    BladeElementRotorConfig c = test::test_rotor(
        test::airfoil_from(test::linear_airfoil_csv(5.7, 0.01)), {14 * kDeg, 0.0, 0.0}, 20);
    c.shaft = BladeElementRotorConfig::ShaftLink{"main", 2.0};
    BladeElementRotor rotor(c);
    const std::vector<ShaftInfo> info = {{"main", c.rotor_speed / 2.0}};
    rotor.bind_shafts(info);
    SimContext ctx = test::sea_level_context(0.0);
    ctx.num_shafts = 1;
    ctx.shafts[0] = {0.0, c.rotor_speed / 2.0};
    const RotorLoads l = rotor.evaluate(ctx);
    ASSERT_GT(l.torque, 0.0);
    ComponentDerivs d{{}, {}, Wrench{}, Mat66::Zero()};
    rotor.derivatives(ctx, {}, d);
    EXPECT_NEAR(d.Q0(0), -2.0 * l.torque, 1e-9 * l.torque);
}

// An ideal drive holds the speed (D-052): the rotor on an ideal shaft with no
// lumped inertia gives the constant-speed rotor's derivatives to the bit at
// the same azimuth, and a run of the airframe with it stays with the
// constant-speed run to round-off (ψ integrated instead of Ω t). (Lumped
// inertia adds its gyroscopic moment, a physical difference.)
TEST(Shafts, IdealDriveIsTheConstantSpeedRotor) {
    const auto airfoil = test::airfoil_from(test::linear_airfoil_csv(5.7, 0.01));
    const double e = 0.4;
    BladeElementRotorConfig c =
        test::flap_rotor(airfoil, {10 * kDeg, 0.0, 0.0}, test::nonuniform_beads(8.0 - e), e, 20);
    c.inflow = InflowModel::pitt_peters;
    const auto make = [&](bool shaft, std::int64_t& n, std::vector<double>& out) {
        ExecutiveConfig ec;
        ec.mass = make_mass_properties(5000.0, 6000.0, 30000.0, 28000.0, 1500.0);
        BladeElementRotorConfig cc = c;
        if (shaft) {
            ShaftConfig s;
            s.name = "main";
            s.speed = c.rotor_speed;
            s.drive = ShaftDrive::ideal;
            ec.shafts = {s};
            cc.shaft = BladeElementRotorConfig::ShaftLink{"main", 1.0};
        }
        std::vector<std::unique_ptr<DynamicComponent>> comps;
        comps.push_back(std::make_unique<BladeElementRotor>(cc));
        Executive ex(ec, std::move(comps));
        BodyState b;
        b.vel_b = Vec3(10.0, 0.0, 0.0);
        ex.set_body_state(b);
        // One evaluation at t = 0, where ψ = Ω t = 0 exactly.
        std::vector<double> xd(ex.num_states(), 0.0);
        (void)ex.derivatives(0.0, ex.state(), {}, {}, xd);
        out.assign(xd.begin(), xd.begin() + static_cast<std::ptrdiff_t>(ex.num_component_states() +
                                                                        body_layout::kSize));
        (void)ex.advance({}, {}, 500'000'000);
        n = ex.n_steps();
        std::vector<double> s(
            ex.state().begin(),
            ex.state().begin() +
                static_cast<std::ptrdiff_t>(ex.num_component_states() + body_layout::kSize));
        out.insert(out.end(), s.begin(), s.end());
    };
    std::int64_t n0 = 0;
    std::int64_t n1 = 0;
    std::vector<double> a;
    std::vector<double> b;
    make(false, n0, a);
    make(true, n1, b);
    ASSERT_EQ(a.size(), b.size());
    ASSERT_EQ(n0, n1);
    const std::size_t first = body_layout::kSize + 8 + 3;  // derivatives at t = 0
    for (std::size_t i = 0; i < first; ++i) {
        EXPECT_EQ(a[i], b[i]) << i;
    }
    for (std::size_t i = first; i < a.size(); ++i) {
        EXPECT_NEAR(a[i], b[i], 1e-9 * std::max(1.0, std::abs(a[i]))) << i;
    }
}

// Pitt–Peters and Bailey inflow on a shaft (D-052): the states are velocities
// over ΩR, so with the velocity held λ̇ gains −λ Ω̇/Ω, Ω̇ = gear × the
// shaft's.
TEST(Shafts, InflowStatesFollowTheTipSpeed) {
    BladeElementRotorConfig c = test::test_rotor(
        test::airfoil_from(test::linear_airfoil_csv(5.7, 0.01)), {10 * kDeg, 0.0, 0.0}, 20);
    c.inflow = InflowModel::pitt_peters;
    c.shaft = BladeElementRotorConfig::ShaftLink{"main", 1.0};
    BladeElementRotor rotor(c);
    rotor.bind_shafts(std::vector<ShaftInfo>{{"other", 3.0}, {"main", c.rotor_speed}});
    ASSERT_EQ(rotor.shaft_index(), 1);
    SimContext ctx = test::sea_level_context(0.0);
    ctx.num_shafts = 2;
    ctx.shafts[1] = {0.0, 0.9 * c.rotor_speed};
    const std::vector<double> x = {0.05, 0.01, -0.02};
    std::vector<double> xd(3, 0.0);
    std::vector<double> G(18, 0.0);
    std::vector<double> Gs(3 * kMaxShafts, 0.0);
    ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero()};
    d.Gs = Gs;
    rotor.derivatives(ctx, x, d);
    for (std::size_t j = 0; j < 3; ++j) {
        EXPECT_DOUBLE_EQ(Gs[j * kMaxShafts + 1], -x[j] / (0.9 * c.rotor_speed));
        EXPECT_EQ(Gs[j * kMaxShafts], 0.0);
    }
    // The rotor turns at the shaft's speed: its thrust coefficient uses it.
    EXPECT_DOUBLE_EQ(rotor.rotor_speed(ctx), 0.9 * c.rotor_speed);
}

// Config: the shafts list and the components' shaft keys (D-052).
TEST(Shafts, ConfigIsReadAndChecked) {
    const std::string rotor = R"(
  - type: blade_element_rotor
    blades: 4
    radius: "8 m"
    rotor_speed: "27 rad/s"
    direction: counter_clockwise
    inflow: uniform
    vortex_ring: none
    ground_effect: none
    inflow_tip_loss: none
    SHAFT
    hub:
      position: {x: "0 m", y: "0 m", z: "-2 m"}
    blade:
      root_cutout: "1.2 m"
      tip_loss: 0.97
      chord: "0.53 m"
      airfoil: )" + std::string(EYAS_TEST_DATA_DIR) +
                              R"(/linear_airfoil.csv
      twist: "-16 deg"
      elements: 20
    controls:
      theta0: {offset: "8 deg"}
)";
    const auto yaml = [&](const std::string& shafts, const std::string& link) {
        std::string r = rotor;
        r.replace(r.find("SHAFT"), 5, link);
        return R"(
geodetic_origin: {latitude: "37 deg", longitude: "-122 deg", altitude: "0 m"}
airframe:
  mass: "7000 kg"
  inertia: {ixx: "6000 kg*m^2", iyy: "40000 kg*m^2", izz: "37000 kg*m^2", ixz: "1900 kg*m^2"}
)" + shafts +
               "components:" + r;
    };
    const std::string one = R"(shafts:
  - {name: main, speed: "13.5 rad/s", inertia: "120 kg*m^2", drive: ideal}
)";
    const Model m = Model::from_string(yaml(one, "shaft: {name: main, gear: 2}"), "a.yaml");
    EXPECT_EQ(m.state().size(), 13U + 2U);
    // Errors: no such shaft, speed mismatch, bad drive, no drive, duplicate.
    EXPECT_THROW((void)Model::from_string(yaml(one, "shaft: {name: tail, gear: 2}"), "a"),
                 InitError);
    EXPECT_THROW((void)Model::from_string(yaml(one, "shaft: {name: main, gear: 1}"), "a"),
                 InitError);
    std::string bad = one;
    bad.replace(bad.find("ideal"), 5, "turbo");
    EXPECT_THROW((void)Model::from_string(yaml(bad, "shaft: {name: main, gear: 2}"), "a"),
                 InitError);
    bad = one;
    bad.replace(bad.find(", drive: ideal"), 14, "");
    EXPECT_THROW((void)Model::from_string(yaml(bad, "shaft: {name: main, gear: 2}"), "a"),
                 InitError);
    const std::string twice = one + "  - {name: main, speed: \"1 rad/s\", drive: ideal}\n";
    EXPECT_THROW((void)Model::from_string(yaml(twice, "shaft: {name: main, gear: 2}"), "a"),
                 InitError);
}

// Real-time (spec §7): advance() with a free shaft allocates nothing.
TEST(Shafts, FreeShaftAllocatesNothing) {
    const double e = 0.4;
    BladeElementRotorConfig c =
        test::flap_rotor(test::airfoil_from(test::linear_airfoil_csv(5.7, 0.01)),
                         {10 * kDeg, 0.0, 0.0}, test::nonuniform_beads(8.0 - e), e, 20);
    c.inflow = InflowModel::pitt_peters;
    c.shaft = BladeElementRotorConfig::ShaftLink{"main", 1.0};
    ExecutiveConfig ec;
    ec.mass = make_mass_properties(5000.0, 6000.0, 30000.0, 28000.0, 1500.0);
    ec.shafts = {free_shaft(c.rotor_speed, 50.0, Vec3(0.0, 0.0, -1.0))};
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<BladeElementRotor>(c));
    comps.push_back(std::make_unique<Motor>(3.0e4));
    Executive ex(ec, std::move(comps));
    (void)ex.advance({}, {}, 16'666'667);
    test::AllocationCounter count;
    for (int i = 0; i < 20; ++i) {
        (void)ex.advance({}, {}, 16'666'667);
    }
    EXPECT_EQ(count.count(), 0U);
}

}  // namespace
}  // namespace eyas
