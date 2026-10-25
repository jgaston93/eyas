#pragma once

#include <stdexcept>
#include <string>

#include "eyas/types.hpp"

namespace eyas::test {

// Whether an advance() call ran its whole frame: not rejected, not capped,
// and no sub-step discarded as non-finite. atmosphere_out_of_range and
// shaft_on_floor are information, not faults.
inline bool frame_ok(const AdvanceStatus& s) noexcept {
    return !s.rejected_dt && !s.rejected_env && !s.capped && !s.non_finite;
}

// For the validation generators: throws unless the frame ran whole, so a
// discarded or dropped step can't reach VALIDATION.md unflagged.
inline void require_frame(const AdvanceStatus& s, const std::string& where) {
    if (!frame_ok(s)) {
        throw std::runtime_error(where + ": advance() did not run its whole frame");
    }
}

}  // namespace eyas::test
