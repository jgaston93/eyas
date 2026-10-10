#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "support/alloc_counter.hpp"

namespace eyas::test {
namespace {

// Makes a pointer visible to the compiler as used. Since C++14 an unobserved
// new/delete pair may be elided (Clang does this at -O1), and then nothing is
// allocated, so there would be nothing to count.
void escape(const void* p) {
    __asm__ __volatile__("" : : "r"(p) : "memory");
}

// The zero-allocation tests are only meaningful if the counter really sees
// allocations.
TEST(AllocationCounter, CountsHeapAllocations) {
    AllocationCounter counter;
    auto p = std::make_unique<int>(1);
    std::vector<double> v(16);
    escape(p.get());
    escape(v.data());
    EXPECT_EQ(counter.count(), 2U);
}

TEST(AllocationCounter, IgnoresAllocationsOutsideItsScope) {
    auto before = std::make_unique<int>(1);
    AllocationCounter counter;
    EXPECT_EQ(counter.count(), 0U);
}

}  // namespace
}  // namespace eyas::test
