#pragma once

#include <cmath>

#include "core/types.hpp"

namespace eyas {

// Hub-frame Euler angles (pitch θ_h, roll φ_h; no yaw) to C_hb = Φ Θ, the
// same rotation sequence as the body attitude (Padfield 3rd ed. Eq. 3A.37
// with ψ = 0). Hub frame h (D-024): z_h down the shaft, opposite the thrust;
// a forward shaft tilt is a negative pitch.
inline Mat3 hub_rotation(double pitch, double roll) noexcept {
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    Mat3 theta;
    theta << cp, 0.0, -sp, 0.0, 1.0, 0.0, sp, 0.0, cp;
    Mat3 phi;
    phi << 1.0, 0.0, 0.0, 0.0, cr, sr, 0.0, -sr, cr;
    return phi * theta;
}

}  // namespace eyas
