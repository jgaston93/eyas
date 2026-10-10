#include <Eigen/Dense>

#include <gtest/gtest.h>

namespace eyas {
namespace {

using Mat66 = Eigen::Matrix<double, 6, 6>;
using Vec6 = Eigen::Matrix<double, 6, 1>;

// Verifies that Eigen builds cleanly under the project's -Werror flags as a
// SYSTEM include (D-010), using the fixed-size 6x6 solve the executive will
// need for the coupled body-acceleration solve (spec §5).
TEST(EigenSmoke, FixedSizeSolve) {
    Mat66 m = Mat66::Identity() * 4.0;
    m(0, 5) = 1.0;
    m(5, 0) = 1.0;
    const Vec6 b = Vec6::Constant(1.0);
    const Vec6 x = m.partialPivLu().solve(b);
    EXPECT_LT((m * x - b).norm(), 1e-14);
}

}  // namespace
}  // namespace eyas
