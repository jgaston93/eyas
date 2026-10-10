#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "support/alloc_counter.hpp"

namespace eyas::test {
namespace {

// The zero-allocation tests are only meaningful if the counter really sees
// allocations.
TEST(AllocationCounter, CountsHeapAllocations) {
    AllocationCounter counter;
    auto p = std::make_unique<int>(1);
    std::vector<double> v(16);
    EXPECT_EQ(counter.count(), 2U);
}

TEST(AllocationCounter, IgnoresAllocationsOutsideItsScope) {
    auto before = std::make_unique<int>(1);
    AllocationCounter counter;
    EXPECT_EQ(counter.count(), 0U);
}

}  // namespace
}  // namespace eyas::test
