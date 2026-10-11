#pragma once

#include <cstdint>

#include "core/types.hpp"

// The output record of the Eyas library (spec §6.10). It grows in tiers;
// existing fields never change meaning, units or frame.
namespace eyas {

struct AircraftState {
    // ---- Tier 1: pose (M2) --------------------------------------------------
    std::int64_t sim_time_ns = 0;  // [ns] n_steps × physics step, exact
    double sim_time = 0.0;         // [s]  sim_time_ns × 1e-9

    // Geodetic position of the CG on the flat-earth map (D-017).
    double latitude = 0.0;   // [rad] WGS-84 geodetic
    double longitude = 0.0;  // [rad] (−π, π]
    double altitude = 0.0;   // [m] above MSL

    // Attitude. q_nb maps body (FRD) vectors to NED; unit length.
    Quat q_nb = Quat::Identity();
    // Padfield 3rd ed. Euler angles (yaw–pitch–roll order), output only.
    double roll = 0.0;     // φ   [rad] (−π, π]
    double pitch = 0.0;    // θ   [rad] [−π/2, π/2]
    double heading = 0.0;  // ψ_h [rad] [0, 2π), from north

    // ---- Tier 5: diagnostics ------------------------------------------------
    AdvanceStatus last_advance;  // status of the most recent advance()
    // [-] max |‖q‖ − 1| before renormalization over the last advance() that
    // ran; a rejected dt leaves it unchanged.
    double quat_norm_error = 0.0;
};

}  // namespace eyas
