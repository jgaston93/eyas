#pragma once

// Closed-form flapping results used as M4 literature anchors (D-026, layer
// 3). Each assumes what its source assumes: linear lift (C_l = a α), no
// drag in the flap moment, constant chord, linear twist from the hub centre,
// uniform inflow, lift from the hinge (Johnson) or the centre (Padfield) to
// the tip, small angles, and first-harmonic flapping only.

#include <cmath>

#include <Eigen/Dense>

#include "core/rigid_body.hpp"
#include "rotor/blade_element_rotor.hpp"

namespace eyas::test {

struct FlapClosedForm {
    double beta0 = 0.0;   // [rad] about the hinge
    double beta1c = 0.0;  // [rad]
    double beta1s = 0.0;  // [rad]
};

// Rotor operating point for the closed forms.
struct FlapCondition {
    double rho = 1.225;     // [kg/m³]
    double lift_slope = 0;  // a [1/rad]
    double mu = 0.0;        // advance ratio, along +x_h (no sideslip)
    double lambda = 0.0;    // inflow through the hub plane, positive down (λ_i − μ_z)
    double theta0 = 0.0;    // [rad] at the hub centre
    double theta_tw = 0.0;  // [rad] linear twist, θ = θ0 + θ_tw r/R
    double theta1c = 0.0;   // [rad]
    double theta1s = 0.0;   // [rad]
};

// Centre-spring rotor (e = 0, K_β ≥ 0): Padfield 3rd ed. Eqs. 3.55, 3.57,
// 3.58 and 3.64 (quasi-steady multiblade solution, differential coning
// dropped), with no body rates and uniform inflow (λ1 = 0), plus the blade
// weight moment (Johnson 2013 Eqs. 6.208–6.209) on the coning row. The same
// equations are Johnson 2013 Eqs. 6.231–6.233.
inline FlapClosedForm padfield_centre_spring(const BladeElementRotorConfig& c,
                                             const FlapCondition& k) {
    const FlapHinge& f = *c.flap;
    const double W = c.rotor_speed;
    const double R = c.radius;
    const double chord = c.chord(0.5);
    const double gamma = k.rho * k.lift_slope * chord * std::pow(R, 4) / f.inertia;  // Eq. 3.32
    const double lb2 = 1.0 + f.spring / (f.inertia * W * W);                         // Eq. 3.32
    const double mu = k.mu;
    const double m2 = mu * mu;
    // D_M0 and H_M0 without the common factor γ/8; rows β0, β1c, β1s.
    Eigen::Matrix3d D;
    D << 8.0 * lb2 / gamma, 0.0, 0.0,                               //
        4.0 / 3.0 * mu, 8.0 * (lb2 - 1.0) / gamma, 1.0 + m2 / 2.0,  //
        0.0, -(1.0 - m2 / 2.0), 8.0 * (lb2 - 1.0) / gamma;
    const double weight = units::kStandardGravity * f.first_moment / (f.inertia * W * W);
    Eigen::Vector3d H;
    H << k.theta0 * (1.0 + m2) + 4.0 * k.theta_tw * (0.2 + m2 / 6.0) + 4.0 / 3.0 * mu * k.theta1s -
             4.0 / 3.0 * k.lambda - 8.0 / gamma * weight,
        k.theta1c * (1.0 + m2 / 2.0),
        8.0 / 3.0 * mu * k.theta0 + 2.0 * mu * k.theta_tw + k.theta1s * (1.0 + 1.5 * m2) -
            2.0 * mu * k.lambda;
    const Eigen::Vector3d b = D.partialPivLu().solve(H);
    return {b(0), b(1), b(2)};
}

// Offset-hinge rotor (K_β = 0): Johnson 2013 Eqs. 6.269–6.279, with lift
// from the hinge to the tip. Johnson's β is the angle of the line from the
// rotor centre to the blade tip, so the hinge angle is β/(1 − e); his Lock
// number uses the generalized mass of the mode η = (r − e)/(1 − e),
// I_b = R² I_β / (R − e)² (dimensional), so γ = ρ a c R² (R − e)² / I_β.
// The blade weight moment (Eqs. 6.208–6.209) is added to the coning.
inline FlapClosedForm johnson_offset_hinge(const BladeElementRotorConfig& c,
                                           const FlapCondition& k) {
    const FlapHinge& f = *c.flap;
    const double W = c.rotor_speed;
    const double R = c.radius;
    const double L = R - f.offset;
    const double e = f.offset / R;
    const double e2 = e * e;
    const double e3 = e2 * e;
    const double e4 = e3 * e;
    const double chord = c.chord(0.5);
    const double gamma = k.rho * k.lift_slope * chord * R * R * L * L / f.inertia;
    const double nu2 = 1.0 + f.offset * f.first_moment / f.inertia;  // Eq. 6.261
    const double c0 = 1.0 - e;                                       // Eqs. 6.269–6.276
    const double c1 = 1.0 - (e + e2) / 2.0;
    const double c2 = 1.0 - (e + e2 + e3) / 3.0;
    const double c3 = 1.0 - (e + e2 + e3 + e4) / 4.0;
    const double d0 = 1.0 - e;
    const double d1 = 1.0 - (2.0 * e + e2) / 3.0;
    const double f0 = 1.0;
    const double f1 = 1.0 + e / 2.0;
    const double mu = k.mu;
    const double m2 = mu * mu;
    const double lnfp = k.lambda - mu * k.theta1s;  // λ_NFP = λ − μ θ1s (Eq. 6.138)
    const double t75 = k.theta0 + 0.75 * k.theta_tw;
    const double t8 = k.theta0 + 0.8 * k.theta_tw;
    const double N = (nu2 - 1.0) / (gamma / 8.0);
    // Unknowns (β0, β1c, β1s) in Johnson's normalization.
    Eigen::Matrix3d A;
    Eigen::Vector3d b;
    // Eq. 6.277
    A.row(0) << 1.0, -gamma / nu2 * mu / 12.0 * (d0 - f1), 0.0;
    b(0) = gamma / nu2 *
           (t8 / 8.0 * (c2 + m2 * c0) + k.theta_tw / 10.0 * (c3 - c2 + m2 * (5.0 / 6.0 * c1 - c0)) -
            lnfp / 6.0 * c1);
    // Eq. 6.278
    A.row(1) << 4.0 / 3.0 * mu * f1, N, d1 + 0.5 * m2 * f0;
    b(1) = (c2 + 0.5 * m2 * c0) * k.theta1c;
    // Eq. 6.279
    A.row(2) << 0.0, -(d1 - 0.5 * m2 * f0), N;
    b(2) = (c2 - 0.5 * m2 * c0) * k.theta1s +
           8.0 / 3.0 * mu * (t75 * c1 + 0.75 * k.theta_tw * (c2 - c1) - 0.75 * lnfp);
    const Eigen::Vector3d bJ = A.partialPivLu().solve(b);
    const double weight =
        units::kStandardGravity * f.first_moment / (f.inertia * W * W * nu2);  // hinge angle
    return {bJ(0) / (1.0 - e) - weight, bJ(1) / (1.0 - e), bJ(2) / (1.0 - e)};
}

// Hover phase lag of the flap response to cyclic pitch [rad]: 90° minus
// atan(N*/d1) (Johnson 2013 Eqs. 6.278–6.279 at μ = 0; Eq. 6.242 for e = 0,
// which is Padfield 3rd ed. Eq. 3.40). γ and d1 as above.
inline double johnson_hover_phase_lag(const BladeElementRotorConfig& c, double rho,
                                      double lift_slope) {
    const FlapHinge& f = *c.flap;
    const double W = c.rotor_speed;
    const double R = c.radius;
    const double L = R - f.offset;
    const double e = f.offset / R;
    const double gamma = rho * lift_slope * c.chord(0.5) * R * R * L * L / f.inertia;
    const double nu2 = 1.0 + f.offset * f.first_moment / f.inertia + f.spring / (f.inertia * W * W);
    const double d1 = 1.0 - (2.0 * e + e * e) / 3.0;
    return 0.5 * std::numbers::pi - std::atan((nu2 - 1.0) / (gamma / 8.0) / d1);
}

}  // namespace eyas::test
