#include "support/alloc_counter.hpp"

#include <cstddef>
#include <cstdlib>
#include <new>

namespace {

thread_local bool g_armed = false;
thread_local std::uint64_t g_count = 0;

void* counted_alloc(std::size_t size) {
    if (g_armed) {
        ++g_count;
    }
    return std::malloc(size == 0 ? 1 : size);
}

void* counted_aligned_alloc(std::size_t size, std::align_val_t align) {
    if (g_armed) {
        ++g_count;
    }
    const auto a = static_cast<std::size_t>(align);
    const std::size_t rounded = ((size == 0 ? 1 : size) + a - 1) / a * a;
    return std::aligned_alloc(a, rounded);
}

}  // namespace

namespace eyas::test {

AllocationCounter::AllocationCounter() : start_(g_count), was_armed_(g_armed) {
    g_armed = true;
}

AllocationCounter::~AllocationCounter() {
    g_armed = was_armed_;
}

std::uint64_t AllocationCounter::count() const {
    return g_count - start_;
}

}  // namespace eyas::test

// Global replacements. Throwing forms throw std::bad_alloc on failure, as the
// standard requires.
void* operator new(std::size_t size) {
    if (void* p = counted_alloc(size)) {
        return p;
    }
    throw std::bad_alloc();
}
void* operator new[](std::size_t size) {
    if (void* p = counted_alloc(size)) {
        return p;
    }
    throw std::bad_alloc();
}
void* operator new(std::size_t size, std::align_val_t align) {
    if (void* p = counted_aligned_alloc(size, align)) {
        return p;
    }
    throw std::bad_alloc();
}
void* operator new[](std::size_t size, std::align_val_t align) {
    if (void* p = counted_aligned_alloc(size, align)) {
        return p;
    }
    throw std::bad_alloc();
}
void* operator new(std::size_t size, const std::nothrow_t&) noexcept {
    return counted_alloc(size);
}
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept {
    return counted_alloc(size);
}

void operator delete(void* p) noexcept {
    std::free(p);
}
void operator delete[](void* p) noexcept {
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept {
    std::free(p);
}
void operator delete(void* p, std::align_val_t) noexcept {
    std::free(p);
}
void operator delete[](void* p, std::align_val_t) noexcept {
    std::free(p);
}
void operator delete(void* p, std::size_t, std::align_val_t) noexcept {
    std::free(p);
}
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept {
    std::free(p);
}
void operator delete(void* p, const std::nothrow_t&) noexcept {
    std::free(p);
}
void operator delete[](void* p, const std::nothrow_t&) noexcept {
    std::free(p);
}
