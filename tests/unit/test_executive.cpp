// M1 acceptance tests for the simulation executive: purity of the derivative
// function, determinism, integer-nanosecond time keeping (D-006), the
// sub-step cap, and zero allocation in advance() (spec §7.1).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

// A component with nonzero G and H, so the coupled-solve buffers are exercised.
// Its state follows the body's forward acceleration through a first-order lag,
// and it adds a small fixed added-mass term to the body.
class LagOnAccel : public DynamicComponent {
public:
    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override { x[0] = 0.25; }
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override {
        out.xdot0[0] = -x[0] / 0.5 + 0.01 * ctx.body.vel_b.x();
        out.G[0] = 2.0;  // ∂ẋ/∂v̇_x
        out.load0_b.F = Vec3(-30.0 * x[0], 0.0, 0.0);
        out.load0_b.M = Vec3(0.0, 5.0 * x[0], 0.0);
        out.H(0, 0) = -20.0;  // added mass in x [kg]
        out.H(4, 4) = -3.0;   // added pitch inertia [kg·m²]
    }
};

class SteadyMoment : public ForceModel {
public:
    Wrench compute(const SimContext& ctx) const override {
        Wrench w;
        w.F = Vec3(100.0 * ctx.controls.collective, 0.0, 0.0);
        w.M = Vec3(10.0, -20.0 * ctx.controls.lon_cyclic, 5.0);
        return w;
    }
};

ExecutiveConfig test_config() {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(1000.0, 800.0, 2500.0, 2200.0, 120.0);
    cfg.step_ns = 1'000'000;
    cfg.max_substeps = 100;
    return cfg;
}

Executive make_full() {
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<LagOnAccel>());
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::make_unique<SteadyMoment>());
    Executive ex(test_config(), std::move(comps), std::move(forces));
    BodyState b;
    b.vel_b = Vec3(20.0, 1.0, -2.0);
    b.omega_b = Vec3(0.1, 0.2, -0.3);
    ex.set_body_state(b);
    return ex;
}

bool bit_identical(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

// ---- Construction ---------------------------------------------------------

TEST(Executive, RejectsInvalidConfig) {
    ExecutiveConfig bad = test_config();
    bad.step_ns = 0;
    EXPECT_THROW(Executive{bad}, InitError);
    bad = test_config();
    bad.max_substeps = 0;
    EXPECT_THROW(Executive{bad}, InitError);
    bad = test_config();
    bad.mass.mass = 0.0;
    EXPECT_THROW(Executive{bad}, InitError);
    bad = test_config();
    bad.mass.inertia_b(0, 1) = 5.0;  // not symmetric
    EXPECT_THROW(Executive{bad}, InitError);
    bad = test_config();
    bad.mass.inertia_b(2, 2) = -1.0;  // not positive definite
    EXPECT_THROW(Executive{bad}, InitError);
}

// The null check must run before anything calls through the pointer.
TEST(Executive, RejectsNullComponentAndForceModel) {
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(nullptr);
    EXPECT_THROW(Executive(test_config(), std::move(comps)), InitError);
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(nullptr);
    EXPECT_THROW(Executive(test_config(), {}, std::move(forces)), InitError);
}

TEST(Executive, StateVectorLayout) {
    const Executive ex = make_full();
    EXPECT_EQ(ex.num_states(), body_layout::kSize + 1);
    EXPECT_EQ(ex.state()[body_layout::kSize], 0.25);  // component initial state
    EXPECT_EQ(ex.state()[body_layout::kQuat], 1.0);   // identity attitude, w first
}

// ---- Purity and determinism -------------------------------------------------

// Spec M1: the full derivative function, called twice on the same
// (t, x, inputs) with other evaluations in between, is bit-identical.
TEST(Executive, DerivativeFunctionIsPure) {
    const Executive ex = make_full();
    const std::vector<double> xa(ex.state().begin(), ex.state().end());
    std::vector<double> xb = xa;
    xb[body_layout::kVel] += 3.0;
    xb[body_layout::kOmega + 2] -= 0.4;
    xb.back() = -1.0;
    Controls ua;
    ua.collective = 0.6;
    ua.lon_cyclic = -0.2;
    const Controls ub{};

    std::vector<double> d1(ex.num_states());
    std::vector<double> d2(ex.num_states());
    std::vector<double> scratch(ex.num_states());
    ex.derivatives(1.5, xa, ua, Environment{}, d1);
    ex.derivatives(7.0, xb, ub, Environment{}, scratch);
    ex.derivatives(0.0, xb, ua, Environment{}, scratch);
    ex.derivatives(1.5, xa, ua, Environment{}, d2);
    EXPECT_TRUE(bit_identical(d1, d2));
}

// The held-body evaluation (D-035) is the free one at a = 0: the free body's
// acceleration solves (M_rb − ΣH) a = net, and each component's held rate is
// its free rate less G a. Checked against LagOnAccel's known G and H.
TEST(Executive, HeldEvaluationIsTheFreeOneAtZeroAcceleration) {
    const Executive ex = make_full();
    const std::vector<double> x(ex.state().begin(), ex.state().end());
    Controls c;
    c.collective = 0.4;
    c.lon_cyclic = 0.3;
    std::vector<double> free(ex.num_states());
    std::vector<double> held(ex.num_states(), 0.0);
    ex.derivatives(2.0, x, c, Environment{}, free);
    Vec6 net;
    std::vector<Wrench> loads(1);
    ASSERT_TRUE(ex.held_derivatives(2.0, x, c, Environment{}, held, net, loads));

    Vec6 a;
    for (int k = 0; k < 3; ++k) {
        a(k) = free[body_layout::kVel + static_cast<std::size_t>(k)];
        a(3 + k) = free[body_layout::kOmega + static_cast<std::size_t>(k)];
    }
    Mat66 H = Mat66::Zero();
    H(0, 0) = -20.0;
    H(4, 4) = -3.0;
    const Vec6 expect = (rigid_body_mass_matrix(ex.airframe_mass()) - H) * a;
    EXPECT_LT((net - expect).norm(), 1e-9 * expect.norm());

    const std::size_t i = body_layout::kSize;
    EXPECT_NEAR(held[i], free[i] - 2.0 * a(0), 1e-12);
    EXPECT_DOUBLE_EQ(held[i], -x[i] / 0.5 + 0.01 * x[body_layout::kVel]);
    EXPECT_DOUBLE_EQ(loads[0].F.x(), -30.0 * x[i]);
    EXPECT_DOUBLE_EQ(loads[0].M.y(), 5.0 * x[i]);

    // A span of the wrong size is ignored; the result is the same.
    Vec6 net2;
    std::vector<Wrench> wrong(3);
    ASSERT_TRUE(ex.held_derivatives(2.0, x, c, Environment{}, held, net2, wrong));
    EXPECT_EQ(net, net2);
    EXPECT_EQ(wrong[0].F, Vec3::Zero());

    // Real-time rules: no allocation.
    const test::AllocationCounter count;
    (void)ex.held_derivatives(2.0, x, c, Environment{}, held, net2, loads);
    EXPECT_EQ(count.count(), 0U);
}

// Spec §7.4: the same inputs give bit-identical results on the same build.
TEST(Executive, RunsAreBitIdentical) {
    auto run = [] {
        Executive ex = make_full();
        Controls u;
        for (int i = 0; i < 600; ++i) {
            u.collective = 0.001 * i;
            (void)ex.advance(u, Environment{}, 16'666'667);
        }
        return std::vector<double>(ex.state().begin(), ex.state().end());
    };
    EXPECT_TRUE(bit_identical(run(), run()));
}

// ---- Time keeping (D-006) --------------------------------------------------

// Spec M1 restated by D-006: 1/60 s frames (16 666 667 ns) for 1 h give
// floor(Σ dt / step) sub-steps exactly, and sim time = n_steps × step.
TEST(Executive, OneHourAtSixtyHzGivesExactStepCount) {
    Executive ex(test_config());
    ex.set_body_state(BodyState{});
    std::int64_t total_ns = 0;
    for (int i = 0; i < 216'000; ++i) {
        const AdvanceStatus st = ex.advance(Controls{}, Environment{}, 16'666'667);
        ASSERT_FALSE(st.capped);
        total_ns += 16'666'667;
    }
    EXPECT_EQ(total_ns, 3'600'000'072'000);
    EXPECT_EQ(ex.n_steps(), 3'600'000);
    EXPECT_EQ(ex.sim_time_ns(), ex.n_steps() * 1'000'000);
    EXPECT_EQ(ex.remainder_ns(), 72'000);
}

// D-006: irregular, jittered frames. Every call's sub-step count is the
// change in floor(Σ dt / step), and nothing is lost or gained.
TEST(Executive, JitteredFramesKeepExactTime) {
    Executive ex(test_config());
    ex.set_body_state(BodyState{});
    std::mt19937_64 rng(12345);
    std::uniform_int_distribution<std::int64_t> dt(0, 40'000'000);
    std::int64_t total_ns = 0;
    for (int i = 0; i < 5'000; ++i) {
        const std::int64_t d = dt(rng);
        const std::int64_t before = total_ns / 1'000'000;
        total_ns += d;
        const AdvanceStatus st = ex.advance(Controls{}, Environment{}, d);
        EXPECT_EQ(st.substeps, total_ns / 1'000'000 - before);
    }
    EXPECT_EQ(ex.n_steps(), total_ns / 1'000'000);
    EXPECT_EQ(ex.remainder_ns(), total_ns % 1'000'000);
}

// Spec §6.10: an over-long frame runs at most max_substeps, drops the excess
// time, and raises a flag.
TEST(Executive, OverlongFrameIsCappedAndFlagged) {
    Executive ex(test_config());
    ex.set_body_state(BodyState{});
    const AdvanceStatus st = ex.advance(Controls{}, Environment{}, 5'000'300'000);  // 5 s hitch
    EXPECT_TRUE(st.capped);
    EXPECT_EQ(st.substeps, 100);
    EXPECT_EQ(ex.n_steps(), 100);
    EXPECT_EQ(ex.remainder_ns(), 300'000);  // the sub-step fraction is kept
    const AdvanceStatus next = ex.advance(Controls{}, Environment{}, 1'000'000);
    EXPECT_FALSE(next.capped);
    EXPECT_EQ(next.substeps, 1);
}

TEST(Executive, NegativeFrameIsRejectedWithoutSideEffects) {
    Executive ex(test_config());
    ex.set_body_state(BodyState{});
    (void)ex.advance(Controls{}, Environment{}, 1'500'000);
    const AdvanceStatus st = ex.advance(Controls{}, Environment{}, -1);
    EXPECT_TRUE(st.rejected_dt);
    EXPECT_EQ(st.substeps, 0);
    EXPECT_EQ(ex.n_steps(), 1);
    EXPECT_EQ(ex.remainder_ns(), 500'000);
}

// D-019: an invalid Environment is rejected like a negative frame time.
TEST(Executive, InvalidEnvironmentIsRejectedWithoutSideEffects) {
    Executive ex(test_config());
    ex.set_body_state(BodyState{});
    (void)ex.advance(Controls{}, Environment{}, 1'500'000);
    const std::vector<double> before(ex.state().begin(), ex.state().end());

    std::vector<Environment> bad(4);
    bad[0].isa_delta_T = std::numeric_limits<double>::quiet_NaN();
    bad[1].isa_delta_T = -300.0;  // a °C/K mix-up
    bad[2].wind_n.y() = std::numeric_limits<double>::infinity();
    bad[3].wind_n.x() = std::numeric_limits<double>::quiet_NaN();
    for (const Environment& env : bad) {
        const AdvanceStatus st = ex.advance(Controls{}, env, 5'000'000);
        EXPECT_TRUE(st.rejected_env);
        EXPECT_FALSE(st.rejected_dt);
        EXPECT_EQ(st.substeps, 0);
    }
    EXPECT_TRUE(std::equal(before.begin(), before.end(), ex.state().begin()));
    EXPECT_EQ(ex.n_steps(), 1);
    EXPECT_EQ(ex.remainder_ns(), 500'000);

    Environment ok;
    ok.isa_delta_T = -60.0;
    const AdvanceStatus st = ex.advance(Controls{}, ok, 1'000'000);
    EXPECT_FALSE(st.rejected_env);
    EXPECT_EQ(st.substeps, 1);
}

// D-019: leaving the tabulated atmosphere is reported per call, from the
// committed state. Origin at 0 m MSL, so altitude = −D.
TEST(Executive, AtmosphereOutOfRangeIsReported) {
    Executive ex(test_config());
    BodyState b;
    b.pos_n.z() = -19'000.0;
    ex.set_body_state(b);
    EXPECT_FALSE(ex.advance(Controls{}, Environment{}, 5'000'000).atmosphere_out_of_range);

    b.pos_n.z() = -25'000.0;
    ASSERT_TRUE(ex.reset(b));
    EXPECT_TRUE(ex.advance(Controls{}, Environment{}, 5'000'000).atmosphere_out_of_range);
    // No sub-step committed: nothing to report.
    EXPECT_FALSE(ex.advance(Controls{}, Environment{}, 0).atmosphere_out_of_range);
}

// The ground plane reaches the components in NED (D-033): its z is the
// origin's altitude less Environment::ground_height, whatever the body's
// position; NaN (no ground) passes through.
class GroundProbe : public ForceModel {
public:
    explicit GroundProbe(double* seen) : seen_(seen) {}
    Wrench compute(const SimContext& ctx) const override {
        *seen_ = ctx.ground_z_n;
        return {};
    }

private:
    double* seen_;
};

TEST(Executive, GroundPlaneIsGivenInNed) {
    double seen = 0.0;
    ExecutiveConfig cfg = test_config();
    cfg.origin_altitude = 1234.5;
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::make_unique<GroundProbe>(&seen));
    const Executive ex(cfg, {}, std::move(forces));
    std::vector<double> x(ex.state().begin(), ex.state().end());
    x[body_layout::kPos + 2] = -321.0;
    std::vector<double> xdot(x.size());
    Environment env;
    env.ground_height = 1000.0;
    ASSERT_TRUE(ex.derivatives(0.0, x, Controls{}, env, xdot));
    EXPECT_EQ(seen, 234.5);
    env.ground_height = std::numeric_limits<double>::quiet_NaN();
    ASSERT_TRUE(ex.derivatives(0.0, x, Controls{}, env, xdot));
    EXPECT_TRUE(std::isnan(seen));
}

// restore() is the other way to install a whole state, so it applies the same
// checks as reset(): a non-finite value or a zero quaternion is refused and
// nothing changes.
TEST(Executive, RestoreRejectsInvalidStateWithoutSideEffects) {
    Executive ex(test_config());
    ex.set_body_state(BodyState{});
    (void)ex.advance(Controls{}, Environment{}, 1'500'000);
    const std::vector<double> before(ex.state().begin(), ex.state().end());

    const std::vector<double> zeros(before.size(), 0.0);  // q_nb = 0
    EXPECT_FALSE(ex.restore(zeros, 0, 0));
    std::vector<double> nan_state = before;
    nan_state[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(ex.restore(nan_state, 0, 0));

    EXPECT_TRUE(std::equal(before.begin(), before.end(), ex.state().begin()));
    EXPECT_EQ(ex.n_steps(), 1);
    EXPECT_EQ(ex.remainder_ns(), 500'000);
    EXPECT_TRUE(ex.restore(before, 1, 500'000));
}

// ---- Failure reporting ------------------------------------------------------

// H equal to the airframe mass matrix makes (M_rb − H) zero, so the coupled
// solve is singular from the first evaluation.
class CancelsAirframeMass : public DynamicComponent {
public:
    explicit CancelsAirframeMass(const MassProperties& mp) : m_(rigid_body_mass_matrix(mp)) {}
    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override { x[0] = 0.0; }
    void derivatives(const SimContext&, std::span<const double>,
                     ComponentDerivs& out) const override {
        out.load0_b.F = Vec3(1.0, 0.0, 0.0);
        out.H = m_;
    }

private:
    Mat66 m_;
};

// Returns NaN from t = t_bad on, so the first steps succeed.
class NanAfter : public DynamicComponent {
public:
    explicit NanAfter(double t_bad) : t_bad_(t_bad) {}
    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override { x[0] = 0.0; }
    void derivatives(const SimContext& ctx, std::span<const double>,
                     ComponentDerivs& out) const override {
        out.xdot0[0] = ctx.t >= t_bad_ ? std::numeric_limits<double>::quiet_NaN() : 1.0;
    }

private:
    double t_bad_;
};

TEST(Executive, SingularCoupledSolveIsFlaggedAndStateKept) {
    const ExecutiveConfig cfg = test_config();
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<CancelsAirframeMass>(cfg.mass));
    Executive ex(cfg, std::move(comps));
    BodyState b;
    b.vel_b = Vec3(10.0, 0.0, 0.0);
    ex.set_body_state(b);
    const std::vector<double> before(ex.state().begin(), ex.state().end());

    const AdvanceStatus st = ex.advance(Controls{}, Environment{}, 5'000'000);
    EXPECT_TRUE(st.non_finite);
    EXPECT_EQ(st.substeps, 0);
    EXPECT_EQ(ex.n_steps(), 0);
    EXPECT_EQ(std::memcmp(ex.state().data(), before.data(), before.size() * sizeof(double)), 0);
    EXPECT_EQ(ex.quat_norm_error(), 0.0);  // no step was committed
}

TEST(Executive, NanMidFrameKeepsLastGoodStep) {
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    // Step 11 (t = 10 → 11 ms) has stages at 10.5 ms, so steps 1–10 succeed.
    comps.push_back(std::make_unique<NanAfter>(0.0105));
    Executive ex(test_config(), std::move(comps));
    ex.set_body_state(BodyState{});

    const AdvanceStatus st = ex.advance(Controls{}, Environment{}, 20'000'000);
    EXPECT_TRUE(st.non_finite);
    EXPECT_EQ(st.substeps, 10);
    EXPECT_EQ(ex.n_steps(), 10);
    for (const double v : ex.state()) {
        EXPECT_TRUE(std::isfinite(v));
    }
    EXPECT_NEAR(ex.state()[body_layout::kSize], 0.010, 1e-15);
    EXPECT_LT(ex.quat_norm_error(), 1e-15);  // NaN must not hide behind std::max

    const AdvanceStatus again = ex.advance(Controls{}, Environment{}, 1'000'000);
    EXPECT_TRUE(again.non_finite);
    EXPECT_EQ(ex.n_steps(), 10);
}

// ---- Convergence diagnostic (D-023) ------------------------------------------

// A stateless component whose bounded iteration "fails" when collective > 0.5.
class ConvergesBelowHalfCollective : public DynamicComponent {
public:
    std::size_t num_states() const override { return 0; }
    void initial_states(std::span<double> /*x*/) const override {}
    void derivatives(const SimContext& ctx, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        out.converged = ctx.controls.collective <= 0.5;
    }
};

// Counts evaluations in which any component failed to converge, four per RK4
// step, and starts again from zero at each advance(), reset() and restore().
// A converged component listed after the failing one must not mask it.
TEST(Executive, UnconvergedEvaluationsAreCountedPerAdvance) {
    std::vector<std::unique_ptr<DynamicComponent>> comps;
    comps.push_back(std::make_unique<ConvergesBelowHalfCollective>());
    comps.push_back(std::make_unique<LagOnAccel>());  // always converged
    Executive ex(test_config(), std::move(comps));
    Controls fails;
    fails.collective = 1.0;
    const Controls converges{};
    constexpr std::int64_t kThreeSteps = 3'000'000;  // 3 × 1 ms

    std::vector<double> xdot(ex.num_states());
    EXPECT_FALSE(ex.derivatives(0.0, ex.state(), fails, Environment{}, xdot));
    EXPECT_TRUE(ex.derivatives(0.0, ex.state(), converges, Environment{}, xdot));

    ex.advance(fails, Environment{}, kThreeSteps);
    EXPECT_EQ(ex.unconverged_evaluations(), 12);
    ex.advance(converges, Environment{}, kThreeSteps);
    EXPECT_EQ(ex.unconverged_evaluations(), 0);

    ex.advance(fails, Environment{}, kThreeSteps);
    const std::vector<double> x(ex.state().begin(), ex.state().end());
    ASSERT_TRUE(ex.reset(BodyState{}));
    EXPECT_EQ(ex.unconverged_evaluations(), 0);

    ex.advance(fails, Environment{}, kThreeSteps);
    ASSERT_TRUE(ex.restore(x, 3, 0));
    EXPECT_EQ(ex.unconverged_evaluations(), 0);
}

// ---- Real-time rules ----------------------------------------------------------

// Spec §7.1: advance() never allocates, with components and force models in.
TEST(Executive, AdvanceDoesNotAllocate) {
    Executive ex = make_full();
    (void)ex.advance(Controls{}, Environment{}, 16'666'667);  // warm-up outside the counter
    const eyas::test::AllocationCounter counter;
    for (int i = 0; i < 120; ++i) {
        (void)ex.advance(Controls{}, Environment{}, 16'666'667);
    }
    (void)ex.advance(Controls{}, Environment{}, 1'000'000'000);  // capped path
    EXPECT_EQ(counter.count(), 0U);
}

}  // namespace
}  // namespace eyas
