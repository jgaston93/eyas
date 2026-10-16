#include "util/table.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "support/alloc_counter.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

const std::vector<double> kXs{-1.0, 0.0, 2.0, 5.0};
const std::vector<double> kYs{0.0, 0.5, 1.5};
const std::vector<double> kZs{10.0, 20.0};

// Multilinear test functions: (bi/tri)linear interpolation reproduces them
// everywhere inside the grid, up to rounding.
double f1(double x) {
    return 3.0 - 2.0 * x;
}
double f2(double x, double y) {
    return 1.0 + 2.0 * x - 3.0 * y + 0.5 * x * y;
}
double f3(double x, double y, double z) {
    return f2(x, y) + 0.25 * z - 0.1 * x * z + 0.01 * x * y * z;
}

Table1D make1() {
    std::vector<double> v;
    for (double x : kXs) {
        v.push_back(f1(x));
    }
    return Table1D(Axis(kXs), v);
}

Table2D make2() {
    std::vector<double> v;
    for (double x : kXs) {
        for (double y : kYs) {
            v.push_back(f2(x, y));
        }
    }
    return Table2D(Axis(kXs), Axis(kYs), v);
}

Table3D make3() {
    std::vector<double> v;
    for (double x : kXs) {
        for (double y : kYs) {
            for (double z : kZs) {
                v.push_back(f3(x, y, z));
            }
        }
    }
    return Table3D(Axis(kXs), Axis(kYs), Axis(kZs), v);
}

TEST(Table, RejectsSizeMismatchAndNonFinite) {
    EXPECT_THROW(Table1D(Axis(kXs), {1.0, 2.0}), InitError);
    EXPECT_THROW(Table1D(Axis({0.0, 1.0}), {1.0, kNaN}), InitError);
    EXPECT_THROW(Table2D(Axis(kXs), Axis(kYs), std::vector<double>(11, 0.0)), InitError);
    EXPECT_THROW(Table3D(Axis(kXs), Axis(kYs), Axis(kZs), std::vector<double>(23, 0.0)), InitError);
}

TEST(Table, ReproducesNodeValuesExactly) {
    const Table1D t1 = make1();
    const Table2D t2 = make2();
    const Table3D t3 = make3();
    for (double x : kXs) {
        EXPECT_EQ(t1(x), f1(x));
        for (double y : kYs) {
            EXPECT_EQ(t2(x, y), f2(x, y));
            for (double z : kZs) {
                EXPECT_EQ(t3(x, y, z), f3(x, y, z));
            }
        }
    }
}

TEST(Table, ReproducesMultilinearFunctionsBetweenNodes) {
    const Table1D t1 = make1();
    const Table2D t2 = make2();
    const Table3D t3 = make3();
    for (double x = -1.0; x <= 5.0; x += 0.37) {
        EXPECT_NEAR(t1(x), f1(x), 1e-12);
        for (double y = 0.0; y <= 1.5; y += 0.11) {
            EXPECT_NEAR(t2(x, y), f2(x, y), 1e-12);
            for (double z = 10.0; z <= 20.0; z += 1.3) {
                EXPECT_NEAR(t3(x, y, z), f3(x, y, z), 1e-12);
            }
        }
    }
}

TEST(Table, ClampsEachAxisIndependently) {
    const Table2D t2 = make2();
    EXPECT_EQ(t2(-50.0, 0.5), f2(-1.0, 0.5));
    EXPECT_EQ(t2(2.0, 99.0), f2(2.0, 1.5));
    EXPECT_EQ(t2(99.0, -99.0), f2(5.0, 0.0));
    const Table3D t3 = make3();
    EXPECT_EQ(t3(2.0, 0.5, 0.0), f3(2.0, 0.5, 10.0));
    EXPECT_EQ(t3(2.0, 0.5, 100.0), f3(2.0, 0.5, 20.0));
}

TEST(Table, NanQueryGivesNan) {
    EXPECT_TRUE(std::isnan(make1()(kNaN)));
    EXPECT_TRUE(std::isnan(make2()(1.0, kNaN)));
    EXPECT_TRUE(std::isnan(make3()(kNaN, 1.0, 15.0)));
}

TEST(Table, CursorAndBinaryLookupsAgreeExactly) {
    const Table3D t3 = make3();
    TableCursor<3> cursor;
    for (double x = -2.0; x <= 6.0; x += 0.13) {
        for (double y = -0.5; y <= 2.0; y += 0.29) {
            for (double z = 5.0; z <= 25.0; z += 3.7) {
                ASSERT_EQ(t3(x, y, z, cursor), t3(x, y, z));
            }
        }
    }
}

// MultiTable2D: each quantity matches a Table2D of it alone, bit for bit,
// inside the grid, on nodes and clamped outside; with and without a cursor.
TEST(Table, MultiTable2DMatchesSeparateTablesExactly) {
    std::vector<double> a;
    std::vector<double> b;
    for (double x : kXs) {
        for (double y : kYs) {
            a.push_back(f2(x, y));
            b.push_back(std::sin(3.0 * x) * std::exp(y));  // not multilinear
        }
    }
    const Table2D ta(Axis(kXs), Axis(kYs), a);
    const Table2D tb(Axis(kXs), Axis(kYs), b);
    const MultiTable2D<2> m(Axis(kXs), Axis(kYs), {a, b});
    TableCursor<2> cursor;
    for (double x = -2.0; x <= 6.0; x += 0.0625 * 1.3) {
        for (double y = -0.5; y <= 2.0; y += 0.0625 * 0.7) {
            const auto v = m(x, y);
            const auto vc = m(x, y, cursor);
            EXPECT_EQ(v[0], ta(x, y)) << x << ", " << y;
            EXPECT_EQ(v[1], tb(x, y)) << x << ", " << y;
            EXPECT_EQ(vc, v);
        }
    }
    for (double x : kXs) {
        for (double y : kYs) {
            EXPECT_EQ(m(x, y)[1], tb(x, y));
        }
    }
}

TEST(Table, MultiTable2DRejectsBadValuesInAnyQuantity) {
    const std::vector<double> good(12, 1.0);
    std::vector<double> nan = good;
    nan[7] = kNaN;
    EXPECT_NO_THROW(MultiTable2D<2>(Axis(kXs), Axis(kYs), {good, good}));
    EXPECT_THROW(MultiTable2D<2>(Axis(kXs), Axis(kYs), {good, std::vector<double>(11, 1.0)}),
                 InitError);
    EXPECT_THROW(MultiTable2D<2>(Axis(kXs), Axis(kYs), {good, nan}), InitError);
}

TEST(Table, LookupsDoNotAllocate) {
    const Table1D t1 = make1();
    const Table2D t2 = make2();
    const Table3D t3 = make3();
    const MultiTable2D<2> m2(Axis(kXs), Axis(kYs),
                             {std::vector<double>(12, 1.0), std::vector<double>(12, 2.0)});
    TableCursor<1> c1;
    TableCursor<2> c2;
    TableCursor<3> c3;
    test::AllocationCounter counter;
    double sink = 0.0;
    for (double x = -2.0; x <= 6.0; x += 0.1) {
        sink += t1(x) + t1(x, c1) + t2(x, 0.7) + t2(x, 0.7, c2) + t3(x, 0.7, 12.0) +
                t3(x, 0.7, 12.0, c3) + m2(x, 0.7)[1] + m2(x, 0.7, c2)[0];
    }
    EXPECT_EQ(counter.count(), 0U);
    EXPECT_TRUE(std::isfinite(sink));
}

// rounded() (D-038): C¹ through every corner, including the clamped ends;
// equal to the linear table outside the bands; tangent to both pieces at the
// band edges; the deviation at a corner is (s_r − s_l) δ / 4.
TEST(Table, RoundedCornersAreSmoothAndLocal) {
    // GenHel's K_FP0 schedule shape: flat, a ramp, flat beyond the end.
    const Table1D t(Axis({0.0, 100.0, 150.0}), {-3.0, -3.0, -5.2});
    const double f = 0.1;  // δ = 5 at both corners
    TableCursor<1> c;
    const double s = -2.2 / 50.0;  // ramp slope
    // Outside the bands: exactly linear.
    for (double x : {-10.0, 0.0, 50.0, 94.9, 105.1, 125.0, 144.9, 155.1, 400.0}) {
        EXPECT_EQ(rounded(t, x, f, c), t(x)) << x;
    }
    // At the corners: the predicted deviation.
    EXPECT_NEAR(rounded(t, 100.0, f, c) - t(100.0), (s - 0.0) * 5.0 / 4.0, 1e-14);
    EXPECT_NEAR(rounded(t, 150.0, f, c) - t(150.0), (0.0 - s) * 5.0 / 4.0, 1e-14);
    // C¹: value and slope continuous everywhere, checked by central
    // differences across a fine sweep; slopes within the pieces' range. In a
    // band the slope changes by |s| over 2δ = 10, so 0.001 |s| per 0.01 step;
    // the bound is twice that (a corner would jump by |s|).
    const double h = 1e-4;
    double prev = (rounded(t, 80.0 + h, f, c) - rounded(t, 80.0 - h, f, c)) / (2 * h);
    for (double x = 80.0; x <= 170.0; x += 0.01) {
        const double d = (rounded(t, x + h, f, c) - rounded(t, x - h, f, c)) / (2 * h);
        EXPECT_LE(std::abs(d - prev), 2e-3 * std::abs(s)) << x;
        EXPECT_LE(d, 1e-9) << x;
        EXPECT_GE(d, s - 1e-9) << x;
        prev = d;
    }
    // A straight run has no corner; fraction 0 is the linear table.
    const Table1D line(Axis({0.0, 1.0, 2.0}), {0.0, 1.0, 2.0});
    EXPECT_EQ(rounded(line, 1.0, f, c), 1.0);
    EXPECT_EQ(rounded(t, 100.0, 0.0, c), t(100.0));
}

}  // namespace
}  // namespace eyas
