#pragma once

// System momentum of an airframe with one flapping rotor, from the executive
// state, for the M4 conservation checks (D-026 layer 2). Blade kinematics
// come from the exact reference (support/flap_reference.hpp), independent of
// the production loads.

#include <span>
#include <vector>

#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "sim/executive.hpp"
#include "support/flap_reference.hpp"

namespace eyas::test {

// System momentum of airframe + blades, from the state, computed with the
// reference bead kinematics (independent of the production loads).
struct Momentum {
    Vec3 P_n;     // linear [kg·m/s]
    Vec3 H_cg_n;  // angular about the system CG [kg·m²/s]
    double mass;  // [kg]
};

inline Momentum system_momentum(const Executive& ex, const BladeElementRotor& rotor,
                                const FlapReference& ref, const Beads& beads,
                                const MassProperties& mp) {
    const BodyState b = ex.body_state();
    const std::span<const double> x = ex.state().subspan(body_layout::kSize);
    const double t = ex.sim_time();
    const Mat3 C = b.q_nb.toRotationMatrix();
    struct Pt {
        double m;
        Vec3 p;
        Vec3 v;
    };
    std::vector<Pt> pts{{mp.mass, b.pos_n, C * b.vel_b}};
    Motion mo{b, Vec6::Zero()};
    const int nb = rotor.config().num_blades;
    for (int k = 0; k < nb; ++k) {
        const std::size_t ib = 2 * static_cast<std::size_t>(k);
        const double psi = rotor.blade_azimuth(rotor.azimuth(t), k);
        for (std::size_t i = 0; i < beads.rho.size(); ++i) {
            const Jet tau{0.0, 1.0, 0.0};
            const JVec P = ref.point(mo, tau, constant(psi) + rotor.config().rotor_speed * tau,
                                     Jet{x[ib], x[ib + 1], 0.0}, constant(beads.rho[i]));
            pts.push_back({beads.mass[i], value(P), d1(P)});
        }
    }
    Momentum out{Vec3::Zero(), Vec3::Zero(), 0.0};
    Vec3 mx = Vec3::Zero();
    for (const Pt& p : pts) {
        out.mass += p.m;
        out.P_n += p.m * p.v;
        mx += p.m * p.p;
    }
    const Vec3 xc = mx / out.mass;
    const Vec3 vc = out.P_n / out.mass;
    for (const Pt& p : pts) {
        out.H_cg_n += p.m * (p.p - xc).cross(p.v - vc);
    }
    out.H_cg_n += C * (mp.inertia_b * b.omega_b);  // airframe spin
    return out;
}

}  // namespace eyas::test
