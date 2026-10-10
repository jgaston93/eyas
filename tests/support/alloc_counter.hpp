#pragma once

#include <cstdint>

// Counts heap allocations made on the current thread while an
// AllocationCounter is alive. Backed by replacements of the global
// operator new family in alloc_counter.cpp; used to enforce the
// no-allocation rule for everything reachable from advance() (spec §7.1).
namespace eyas::test {

class AllocationCounter {
public:
    AllocationCounter();
    ~AllocationCounter();
    AllocationCounter(const AllocationCounter&) = delete;
    AllocationCounter& operator=(const AllocationCounter&) = delete;

    std::uint64_t count() const;

private:
    std::uint64_t start_;
    bool was_armed_;
};

}  // namespace eyas::test
