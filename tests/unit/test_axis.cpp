#include "util/axis.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "support/alloc_counter.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

Axis make_axis() {
    return Axis({-2.0, 0.0, 1.0, 4.0, 10.0});
}

TEST(Axis, RejectsInvalidBreakpoints) {
    EXPECT_THROW(Axis({}), InitError);
    EXPECT_THROW(Axis({1.0}), InitError);
    EXPECT_THROW(Axis({0.0, 0.0}), InitError);
    EXPECT_THROW(Axis({0.0, 2.0, 1.0}), InitError);
    EXPECT_THROW(Axis({0.0, kNaN}), InitError);
    EXPECT_THROW(Axis({0.0, kInf}), InitError);
}

TEST(Axis, LocatesInteriorPoints) {
    const Axis a = make_axis();
    const AxisPosition p = a.locate(2.5);
    EXPECT_EQ(p.index, 2U);
    EXPECT_DOUBLE_EQ(p.t, 0.5);
}

TEST(Axis, BreakpointsMapToTheIntervalTheyStart) {
    const Axis a = make_axis();
    for (std::size_t i = 0; i + 1 < a.size(); ++i) {
        const AxisPosition p = a.locate(a.breakpoint(i));
        EXPECT_EQ(p.index, i);
        EXPECT_EQ(p.t, 0.0);
    }
}

TEST(Axis, ClampsOutsideTheRange) {
    const Axis a = make_axis();
    AxisPosition lo = a.locate(-100.0);
    EXPECT_EQ(lo.index, 0U);
    EXPECT_EQ(lo.t, 0.0);
    AxisPosition hi = a.locate(100.0);
    EXPECT_EQ(hi.index, 3U);
    EXPECT_EQ(hi.t, 1.0);
    AxisPosition last = a.locate(10.0);
    EXPECT_EQ(last.index, 3U);
    EXPECT_EQ(last.t, 1.0);
    EXPECT_EQ(a.locate(-kInf).t, 0.0);
    EXPECT_EQ(a.locate(kInf).t, 1.0);
}

TEST(Axis, NanPropagates) {
    const Axis a = make_axis();
    EXPECT_TRUE(std::isnan(a.locate(kNaN).t));
    std::size_t hint = 2;
    EXPECT_TRUE(std::isnan(a.locate(kNaN, hint).t));
    EXPECT_LT(hint, a.size() - 1);
}

TEST(Axis, TwoPointAxis) {
    const Axis a({0.0, 2.0});
    EXPECT_EQ(a.locate(1.0).index, 0U);
    EXPECT_DOUBLE_EQ(a.locate(1.0).t, 0.5);
}

// Purity requirement: the hunt result must be bit-identical to the binary
// search for every query and every starting hint, including stale or
// out-of-range hints.
TEST(Axis, HuntResultIndependentOfHint) {
    std::vector<double> bp;
    for (int i = 0; i < 200; ++i) {
        bp.push_back(static_cast<double>(i) * 0.37 + std::sin(i) * 0.1);
    }
    const Axis a(bp);
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> x_dist(-5.0, 80.0);
    std::uniform_int_distribution<std::size_t> hint_dist(0, 400);
    for (int n = 0; n < 20000; ++n) {
        const double x = x_dist(rng);
        std::size_t hint = hint_dist(rng);
        const AxisPosition ref = a.locate(x);
        const AxisPosition hunt = a.locate(x, hint);
        ASSERT_EQ(hunt.index, ref.index) << "x=" << x;
        ASSERT_EQ(std::bit_cast<std::uint64_t>(hunt.t), std::bit_cast<std::uint64_t>(ref.t));
        ASSERT_EQ(hint, ref.index);
    }
}

TEST(Axis, HuntFollowsSweepsInBothDirections) {
    const Axis a = make_axis();
    std::size_t hint = 0;
    for (double x = -3.0; x <= 11.0; x += 0.01) {
        ASSERT_EQ(a.locate(x, hint).index, a.locate(x).index);
    }
    for (double x = 11.0; x >= -3.0; x -= 0.01) {
        ASSERT_EQ(a.locate(x, hint).index, a.locate(x).index);
    }
}

TEST(Axis, LookupsDoNotAllocate) {
    const Axis a = make_axis();
    std::size_t hint = 0;
    test::AllocationCounter counter;
    double sink = 0.0;
    for (double x = -3.0; x <= 11.0; x += 0.1) {
        sink += a.locate(x).t + a.locate(x, hint).t;
    }
    EXPECT_EQ(counter.count(), 0U);
    EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
}  // namespace eyas
