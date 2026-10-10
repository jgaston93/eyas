// M1 acceptance tests for the simulation executive: purity of the derivative
// function, determinism, integer-nanosecond time keeping (D-006), the
// sub-step cap, and zero allocation in advance() (spec §7.1).

#include <cstdint>
#include <cstring>
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
