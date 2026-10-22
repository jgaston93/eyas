#pragma once

// Vortex-ring-state (VRS) correction to the mean induced inflow (M8d,
// D-049; SPEC_REVIEW #21): Johnson, NASA/TP-2005-213477 (2005), Table 3
// (the algorithm) and Table 4 (its parameters), the model Johnson 2013 §4.2
// refers to. An empirical extension of momentum theory, built so that the
// inflow curve has zero slope on a flight-test stability boundary. Pure
// functions, called from advance(): no allocation, no exceptions, bounded
// iterations.
//
// Notation (Johnson's): velocities scaled by v_h = √(T/2ρA), the ideal hover
// induced velocity. vx ≥ 0 is the speed in the disc plane, vz the speed
// along the shaft, positive in climb (Eyas's μ_z is positive in descent, so
// vz = −μ_z/λ_h), v the induced velocity, positive down through the disc.
namespace eyas {

// Johnson's two curves: the baseline (momentum theory with the ideal-
// autorotation singularity removed, Table 3 step 2) and the VRS model
// (baseline plus the VRS increment, step 3; f = 1, κ = 1 in v = κ(v_base +
// f Δv_vrs), p. 14).
enum class VrsCurve : unsigned char {
    baseline,
    vortex_ring,
};

// Table 4 (p. 24): points of Fig. 37.
struct JohnsonVrsParameters {
    static constexpr double vz_a = -1.5;     // baseline: momentum v, dv/dvz matched here
    static constexpr double vz_b = -2.1;     //           momentum v matched here
    static constexpr double vx_c = 0.75;     //           momentum theory for vx ≥ this
    static constexpr double vz_d = -0.2;     // VRS: joins the baseline (value and slope)
    static constexpr double vz_n = -0.45;    //      inflow minimum (stability boundary)
    static constexpr double total_n = 0.85;  // (vz + v) at N
    static constexpr double vz_x = -1.5;     //      inflow maximum (stability boundary)
    static constexpr double total_x = 1.25;  // (vz + v) at X
    static constexpr double vz_e = -2.0;     //      joins the baseline (value only)
    static constexpr double vx_m = 0.95;     //      no VRS increment for vx ≥ this
};

// Eyas's fades (D-049, a departure from Table 3). Johnson's model jumps at
// vx = v_xC (the baseline cubic meets momentum theory with an offset of up
// to 0.003) and at vx = v_xM (the VRS increment keeps its zero-slope
// conditions while its size goes to zero, an offset of up to 0.14). A jump
// makes the thrust inversion multi-valued. Each departure from momentum
// theory is faded to zero with smoothstep over this width below its edge:
// the baseline's over 0.70–0.75, the increment's over 0.90–0.95, where
// momentum theory has a single root. The stability boundary is Johnson's
// for vx ≤ 0.90; its nose moves from about 0.933 to about 0.917.
inline constexpr double kVortexRingFadeWidth = 0.05;

// A root of Glauert's momentum equation v √(vx² + (vz + v)²) = 1 (Johnson
// TP-2005 p. 3; Johnson 2013 Eq. 5.9 scaled by λ_h), with its slope
// dv/dvz at fixed vx (implicit differentiation). For vx ≤ 0.62 and
// vz ≤ −1.77 the equation has three positive roots; elsewhere one.
struct MomentumRoot {
    double v = 0.0;
    double dv_dvz = 0.0;
    bool converged = true;  // every momentum root it uses met its tolerance
};
// The largest root: the normal working branch, continued into descent.
MomentumRoot momentum_normal(double vx, double vz) noexcept;
// The smallest root: the windmill-brake branch where three roots exist.
MomentumRoot momentum_windmill(double vx, double vz) noexcept;

// Induced velocity v on Johnson's curve, with its slope dv/dvz.
MomentumRoot johnson_inflow(double vx, double vz, VrsCurve curve) noexcept;

// The thrust that puts the inflow state on each curve. Johnson's curves give
// v from (vx, vz, v_h); the inflow equations need the reverse, the thrust
// for a given induced inflow λ0 at the hub's (μ, μ_z). Each curve is
// homogeneous of degree one, λ0 = λ_h f(μ/λ_h, −μ_z/λ_h), and λ0 rises
// strictly with λ_h at fixed (μ, μ_z) (D-049 checks f − x f_x − z f_z ≥ 0.49
// over the region), so λ_h is unique; found with Brent's method.
//
// Outside the region, where both curves are momentum theory with a unique
// root (vz ≥ v_zD or vx ≥ v_xM at the momentum λ_h), nothing is solved:
// active = false and both thrusts are the momentum value 2λ0√(μ² + (λ0 −
// μ_z)²), computed exactly as the inflow solves compute it. Negative λ0
// (negative thrust) mirrors the rotor: λ0 → −λ0, μ_z → −μ_z, C_T → −C_T.
//
// With baseline = false only the VRS curve is solved (the uniform inflow
// solve needs no more); ct_base is then NaN wherever active.
struct VortexRingThrust {
    double ct_vrs = 0.0;    // C_T on the VRS curve [-]
    double ct_base = 0.0;   // C_T on the baseline curve [-]
    double lambda_h = 0.0;  // √(|ct_vrs|/2) [-]
    bool active = false;    // false: momentum theory, nothing solved
    bool converged = true;  // every solve met its tolerance
    int iterations = 0;     // curve evaluations used, all solves
};
// lambda0 = λ0, mu = μ ≥ 0, mu_z = μ_z (positive down), all / ΩR.
VortexRingThrust vortex_ring_thrust(double lambda0, double mu, double mu_z,
                                    bool baseline = true) noexcept;

// The VRS lag (D-049): the increment builds up and decays with time constant
// τ = τ_rev 2π/Ω, τ_rev = 0.7/λ_h: Johnson TP-2005 p. 16 gives 14 revolutions
// for the D6075 flight test at λ_h = 0.05; the scaling with R/v_h is Eyas's
// (D-049). With tip loss on the inflow the radius is B·R (D-051).
inline constexpr double kVortexRingLagCoefficient = 0.7;

// Iteration limit of each solve in vortex_ring_thrust().
inline constexpr int kVortexRingMaxIterations = 100;

}  // namespace eyas
