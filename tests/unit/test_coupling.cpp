// D-003 data path between components (first used in M6a, D-028): producers
// are evaluated before consumers whatever the config order, values reach the
// consumer in the same evaluation, init errors name the problem, and the
// derivative function stays pure and allocation-free.

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "core/component.hpp"
#include "core/coupling.hpp"
#include "core/rigid_body.hpp"
#include "sim/executive.hpp"
#include "support/alloc_counter.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

// Produces "<name>.value" = 3 x + u (u the forward speed); ẋ = −x.
class Producer : public DynamicComponent {
public:
    explicit Producer(std::string name) : name_(std::move(name)) {}
    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override { x[0] = 2.0; }
    std::string_view name() const noexcept override { return name_; }
    std::vector<std::string> produces() const override { return {name_ + ".value"}; }
    void bind(const CouplingRegistry& r) override { slot_ = r.find(name_ + ".value", name_); }
    void derivatives(const SimContext& ctx, std::span<const double> x,
                     ComponentDerivs& out) const override {
        out.xdot0[0] = -x[0];
        if (out.coupling != nullptr) {
            out.coupling->set(slot_, 3.0 * x[0] + ctx.body.vel_b.x());
        }
    }

private:
    std::string name_;
    CouplingSlot slot_;
};

// Consumes "<source>.value"; ẋ = value. Optionally produces its own output.
class Consumer : public DynamicComponent {
public:
    Consumer(std::string name, std::string source, bool declare = true, bool produce = false)
        : name_(std::move(name)),
          source_(std::move(source)),
          declare_(declare),
          produce_(produce) {}
    std::size_t num_states() const override { return 1; }
    void initial_states(std::span<double> x) const override { x[0] = 0.0; }
    std::string_view name() const noexcept override { return name_; }
    std::vector<std::string> produces() const override {
        return produce_ ? std::vector<std::string>{name_ + ".value"} : std::vector<std::string>{};
    }
    std::vector<std::string> consumes() const override {
        return declare_ ? std::vector<std::string>{source_ + ".value"} : std::vector<std::string>{};
    }
    void bind(const CouplingRegistry& r) override {
        in_ = r.find(source_ + ".value", name_);
        if (produce_) {
            out_ = r.find(name_ + ".value", name_);
        }
    }
    void derivatives(const SimContext& ctx, std::span<const double> /*x*/,
                     ComponentDerivs& out) const override {
        const double v = ctx.coupling->get(in_);
        out.xdot0[0] = v;
        if (produce_) {
            out.coupling->set(out_, v);
        }
    }

private:
    std::string name_;
    std::string source_;
    bool declare_;
    bool produce_;
    CouplingSlot in_;
    CouplingSlot out_;
};

ExecutiveConfig config() {
    ExecutiveConfig cfg;
    cfg.mass = make_mass_properties(1000.0, 800.0, 2500.0, 2200.0, 120.0);
    return cfg;
}

template <class... T>
std::vector<std::unique_ptr<DynamicComponent>> list(std::unique_ptr<T>... c) {
    std::vector<std::unique_ptr<DynamicComponent>> v;
    (v.push_back(std::move(c)), ...);
    return v;
}

std::string init_error(std::vector<std::unique_ptr<DynamicComponent>> components) {
    try {
        Executive ex(config(), std::move(components));
    } catch (const InitError& e) {
        return e.what();
    }
    return "";
}

TEST(Coupling, ProducerRunsFirstWhateverTheConfigOrder) {
    // Config order: consumer, unrelated producer, producer. Evaluation order:
    // the consumer's producer first; ties keep config order.
    Executive ex(config(), list(std::make_unique<Consumer>("c", "p"),
                                std::make_unique<Producer>("q"), std::make_unique<Producer>("p")));
    const auto order = ex.evaluation_order();
    ASSERT_EQ(order.size(), 3U);
    EXPECT_EQ(order[0], 1U);  // q: ready, first in config order
    EXPECT_EQ(order[1], 2U);  // p
    EXPECT_EQ(order[2], 0U);  // c after p

    BodyState b;
    b.vel_b = Vec3(5.0, 0.0, 0.0);
    ex.set_body_state(b);
    std::vector<double> xdot(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, xdot));
    const std::size_t c = body_layout::kSize;  // consumer's state: first in layout
    EXPECT_EQ(xdot[c], 3.0 * 2.0 + 5.0);       // p's value from this evaluation
}

TEST(Coupling, ChainsAcrossComponents) {
    Executive ex(config(), list(std::make_unique<Consumer>("b", "a"),
                                std::make_unique<Consumer>("a", "p", true, true),
                                std::make_unique<Producer>("p")));
    const auto order = ex.evaluation_order();
    EXPECT_EQ(order[0], 2U);
    EXPECT_EQ(order[1], 1U);
    EXPECT_EQ(order[2], 0U);
    std::vector<double> xdot(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, xdot));
    EXPECT_EQ(xdot[body_layout::kSize], 6.0);
    EXPECT_EQ(xdot[body_layout::kSize + 1], 6.0);
}

TEST(Coupling, InitErrorsNameTheProblem) {
    const std::string missing = init_error(list(std::make_unique<Consumer>("c", "nobody")));
    EXPECT_NE(missing.find("'c' consumes 'nobody.value'"), std::string::npos) << missing;

    const std::string twice =
        init_error(list(std::make_unique<Producer>("p"), std::make_unique<Producer>("p")));
    EXPECT_NE(twice.find("produced by both"), std::string::npos) << twice;

    const std::string cycle = init_error(list(std::make_unique<Consumer>("a", "b", true, true),
                                              std::make_unique<Consumer>("b", "a", true, true)));
    EXPECT_NE(cycle.find("cycle"), std::string::npos) << cycle;
    EXPECT_NE(cycle.find("a, b"), std::string::npos) << cycle;
}

TEST(Coupling, DerivativesStayPure) {
    Executive ex(config(),
                 list(std::make_unique<Consumer>("c", "p"), std::make_unique<Producer>("p")));
    std::vector<double> x(ex.state().begin(), ex.state().end());
    std::vector<double> other = x;
    other[body_layout::kSize + 1] = -7.0;  // producer state
    std::vector<double> a(ex.num_states());
    std::vector<double> b(ex.num_states());
    std::vector<double> junk(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, x, Controls{}, Environment{}, a));
    EXPECT_TRUE(ex.derivatives(0.0, other, Controls{}, Environment{}, junk));
    EXPECT_TRUE(ex.derivatives(0.0, x, Controls{}, Environment{}, b));
    EXPECT_EQ(std::memcmp(a.data(), b.data(), a.size() * sizeof(double)), 0);
    EXPECT_NE(junk[body_layout::kSize], a[body_layout::kSize]);
}

// A diverging RK4 stage makes a producer write NaN. The read must not abort
// a debug build: the executive rejects the non-finite step, as in release.
TEST(Coupling, ProducedNaNReachesTheConsumer) {
    Executive ex(config(),
                 list(std::make_unique<Consumer>("c", "p"), std::make_unique<Producer>("p")));
    std::vector<double> x(ex.state().begin(), ex.state().end());
    x[body_layout::kSize + 1] = std::numeric_limits<double>::quiet_NaN();  // producer state
    std::vector<double> xdot(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, x, Controls{}, Environment{}, xdot));
    EXPECT_TRUE(std::isnan(xdot[body_layout::kSize]));
}

// Force models run before the store is filled, so they get none (D-003).
class StoreProbe : public ForceModel {
public:
    explicit StoreProbe(bool* saw_store) : saw_store_(saw_store) {}
    Wrench compute(const SimContext& ctx) const override {
        *saw_store_ = *saw_store_ || ctx.coupling != nullptr;
        return Wrench{};
    }

private:
    bool* saw_store_;
};

TEST(Coupling, ForceModelsGetNoStore) {
    bool saw_store = false;
    std::vector<std::unique_ptr<ForceModel>> forces;
    forces.push_back(std::make_unique<StoreProbe>(&saw_store));
    Executive ex(config(),
                 list(std::make_unique<Consumer>("c", "p"), std::make_unique<Producer>("p")),
                 std::move(forces));
    std::vector<double> xdot(ex.num_states());
    EXPECT_TRUE(ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, xdot));
    EXPECT_FALSE(saw_store);
}

TEST(Coupling, AdvanceDoesNotAllocate) {
    Executive ex(config(),
                 list(std::make_unique<Consumer>("c", "p"), std::make_unique<Producer>("p")));
    (void)ex.advance(Controls{}, Environment{}, 16'666'667);
    const test::AllocationCounter count;
    (void)ex.advance(Controls{}, Environment{}, 16'666'667);
    EXPECT_EQ(count.count(), 0U);
}

#ifndef NDEBUG
// A component that reads a slot it never declared isn't ordered after the
// producer; the debug written flag catches the read before the write (D-003).
TEST(CouplingDeathTest, UndeclaredReadAssertsInDebug) {
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    Executive ex(config(), list(std::make_unique<Consumer>("c", "p", /*declare=*/false),
                                std::make_unique<Producer>("p")));
    std::vector<double> xdot(ex.num_states());
    EXPECT_DEATH((void)ex.derivatives(0.0, ex.state(), Controls{}, Environment{}, xdot), "");
}
#endif

}  // namespace
}  // namespace eyas
