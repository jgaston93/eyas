#pragma once

#include <cstdint>

namespace eyas {

// A rotor's sense of rotation, viewed from the −z_h side of its hub frame
// (D-024: z_h points down the shaft, opposite the thrust), so from the side
// its thrust points to: from above for a main rotor (spec §4). Counter-
// clockwise: the rotor's angular velocity is along −z_h.
enum class RotationDirection : std::uint8_t {
    counter_clockwise,  // the UH-60's main rotor, viewed from above (spec §4)
    clockwise,
};

}  // namespace eyas
