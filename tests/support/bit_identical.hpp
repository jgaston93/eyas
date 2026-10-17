#pragma once

#include <cstring>
#include <span>

// Bit-for-bit equality of two arrays of doubles, for the determinism tests
// (spec §7 rule 4): unlike ==, it tells −0 from +0 and compares NaN payloads.
namespace eyas::test {

inline bool bit_identical(std::span<const double> a, std::span<const double> b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

}  // namespace eyas::test
