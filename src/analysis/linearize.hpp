#pragma once

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "eyas/trim.hpp"
#include "sim/executive.hpp"

// Linearization about a periodic trim (spec §6.9; D-035 and its amendment
// of 2026-10-15, SPEC_REVIEW #1 and #29). Internal: used by the tests and
// the validation generators. Init-time: allocates.
namespace eyas {

class BladeElementRotor;

// Reduced coordinates. Body: u, v, w [m/s] (body axes), p, q, r [rad/s],
// φ, θ, ψ_h [rad] (Padfield 3rd ed. Eqs. 3A.34–3A.37), then optionally the
// height change h [m] (positive up). x_n and y_n are left out (they enter
// nothing while the ground is absent and the wind uniform). Then every
// component's states in state order, except that each flapping
// blade-element rotor's 2 N_b flap states are replaced by its N_b multiblade
// coordinates q (Johnson 2013 Eq. 15.77: β0, β_nc, β_ns for n = 1…(N_b−1)/2,
// and β_d with (−1)^(k+1) for even N_b, blade k = 0…N_b−1 at
// blade_azimuth(), as multiblade_flap()) followed by their time derivatives
// q̇. The parts of the state that aren't coordinates (x_n, y_n, and z_n
// without h) come from a reference state.
class ReducedCoordinates {
public:
    static constexpr std::size_t kU = 0;  // u v w p q r φ θ ψ_h follow
    static constexpr std::size_t kP = 3;
    static constexpr std::size_t kPhi = 6;
    static constexpr std::size_t kTheta = 7;
    static constexpr std::size_t kPsi = 8;
    static constexpr std::size_t kHeight = 9;  // if with_height

    ReducedCoordinates(const Executive& ex, bool with_height);

    std::size_t size() const noexcept { return names_.size(); }
    std::size_t body_size() const noexcept { return with_height_ ? 10 : 9; }
    bool with_height() const noexcept { return with_height_; }
    const std::vector<std::string>& names() const noexcept { return names_; }
    // Whether reduced coordinate i is a multiblade flap coordinate q, one of
    // their rates q̇, or neither.
    enum class FlapKind : std::uint8_t { none, coordinate, rate };
    FlapKind flap_kind(std::size_t i) const noexcept;

    // Full state for reduced coordinates xr at time t [s]: the quaternion
    // from (φ, θ, ψ_h), z_n = z_ref − h, blade states from the multiblade
    // coordinates; the rest from `ref`.
    void to_full(double t, const Eigen::VectorXd& xr, std::span<const double> ref,
                 std::span<double> x) const;
    // Reduced coordinates of the full state x at time t; ψ_h is taken within
    // ±π of ref's heading, so it is continuous across north.
    Eigen::VectorXd to_reduced(double t, std::span<const double> x,
                               std::span<const double> ref) const;
    // ẋr at (t, x) given the full ẋ: Euler rates (Padfield 3rd ed.
    // Eq. 4.37), ḣ = −ż_n and the multiblade rates q̈ = a − 2Ω D q̇ − Ω² D² q,
    // a the multiblade transform of the blade accelerations and D the
    // azimuth derivative of the basis (Johnson 2013 Eq. 15.80 for the flap
    // equation).
    Eigen::VectorXd rates(double t, std::span<const double> x, std::span<const double> xdot) const;

private:
    struct Rotor {
        const BladeElementRotor* rotor = nullptr;  // in the executive, which must outlive this
        std::size_t full = 0;                      // first flap state in the full state
        std::size_t reduced = 0;                   // first multiblade coordinate
        int blades = 0;
        double omega = 0.0;  // [rad/s]
    };
    // Basis value φ_i and its azimuth derivative for blade k of rotor r at t.
    static void basis(const Rotor& r, double t, int k, std::span<double> phi,
                      std::span<double> dphi);
    // (D q)_i: the coefficients of Σ_j q_j φ_j′ (cos nψ → −n sin nψ).
    static void derivative(int blades, std::span<const double> q, std::span<double> out);

    bool with_height_;
    std::vector<std::string> names_;
    std::vector<Rotor> rotors_;
    // Full index of every other reduced coordinate after the body block,
    // with its reduced index.
    std::vector<std::pair<std::size_t, std::size_t>> copied_;
};

struct LinearizeOptions {
    int azimuths = 64;           // J samples over the revolution
    int steps_per_period = 256;  // RK4 steps of the held orbit (J must divide it)
    bool height = true;          // h in the retained model
    // Central-difference half-steps, by coordinate kind (the step-size study,
    // VALIDATION.md M7b).
    double step_velocity = 1e-4;   // [m/s]
    double step_rate = 1e-5;       // [rad/s]
    double step_angle = 1e-5;      // [rad]
    double step_height = 1e-2;     // [m]
    double step_flap = 1e-6;       // [rad] multiblade coordinates
    double step_flap_rate = 1e-5;  // [rad/s] their rates
    double step_other = 1e-6;      // other component states, in their own units
    double step_control = 1e-5;    // [-] Controls units
};

// The retained linear model about a trim, in reduced coordinates.
struct LinearModel {
    std::vector<std::string> states;
    std::vector<std::string> inputs;  // collective, lat_cyclic, lon_cyclic, pedal
    std::size_t body_size = 9;        // leading body coordinates (10 with h)
    double period = 0.0;              // T [s]
    // The periodic (LTP) model: A(t_j), B(t_j) at t_j = jT/J, j = 0…J−1.
    std::vector<Eigen::MatrixXd> A_periodic;
    std::vector<Eigen::MatrixXd> B_periodic;
    // Their averages: the constant-coefficient approximation, averaged in
    // the non-rotating frame (Johnson 2013 §15.4).
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    // Evaluations with an unconverged iteration (D-023), in the held orbit
    // and in the perturbations about it.
    std::size_t unconverged = 0;
};

// The central-difference half-step of each coordinate of `rc`, by kind.
std::vector<double> perturbation_steps(const ReducedCoordinates& rc, const LinearizeOptions& o);

// Linearizes `ex` about trim `r` for `c` along the held orbit (D-035).
// Throws InitError if the trim doesn't fit `ex`.
LinearModel linearize(const Executive& ex, const TrimCondition& c, const TrimResult& r,
                      const LinearizeOptions& options = {});

// Response of the linear model to x(0) = x0 and inputs u(t) [Controls
// units, the change from trim], by RK4 with step 2T/J, every stage taking
// the sampled matrices at its own time (J even), so the step and its
// midpoint fall on samples: the periodic model with A(t_j), B(t_j); with
// `averaged`, the constant A, B. Returns x at t_n = 2nT/J, n = 0…steps.
std::vector<Eigen::VectorXd> linear_response(const LinearModel& m, const Eigen::VectorXd& x0,
                                             const std::function<Eigen::Vector4d(double)>& u,
                                             int steps, bool averaged = false);

// The condensed model (D-035 amendment): h removed (as Padfield 3rd ed.
// Eq. 4.44), then every set of non-body states that feeds no other state
// removed (block triangular, so no other eigenvalue changes), then the rest
// quasi-static: A_bb − A_br A_rr⁻¹ A_rb, B_b − A_br A_rr⁻¹ B_r. The body
// states are u, v, w, p, q, r, φ, θ, ψ_h; the moment rows are Padfield's
// primed derivatives (Eqs. 4.47–4.48), and gravity and the kinematic terms
// are included, as in Padfield's App. 4B charts (p. 187).
// `retained` (indices into m.states, none of them body states) are kept as
// dynamic states after the body states, in that order, and never removed:
// for slow non-body states that quasi-static elimination would misstate, e.g.
// a flight control system's washouts, whose output it sets to zero (M8c2).
struct CondensedModel {
    std::vector<std::string> states;  // the 9 body states, then the retained ones
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    std::vector<std::size_t> dropped;  // retained-model indices removed as feeding nothing
    Eigen::VectorXcd dropped_eigenvalues;
};

// Throws InitError if the quasi-static block is singular, naming the state
// when one has no dynamics of its own (a zero row, e.g. an engaged FPS
// synchronizer's reference, a pure memory): keep such states in `retained`.
CondensedModel condense(const LinearModel& m, const std::vector<std::size_t>& retained = {});

// The monodromy matrix Φ(T) of the free system over one revolution, in
// reduced coordinates with h (options.height must be true, else InitError):
// central differences of the flow map from the held orbit's start (t = 0),
// with the free body integrated by RK4 at options.steps_per_period steps per
// revolution and the quaternion renormalized every step, as the executive
// does (free_step()). Independent of the sampled Jacobians (D-035
// amendment, L2).
struct Monodromy {
    Eigen::MatrixXd matrix;       // Φ(T)
    std::size_t unconverged = 0;  // evaluations with an unconverged iteration (D-023)
};
Monodromy monodromy(const Executive& ex, const TrimCondition& c, const TrimResult& r,
                    const LinearizeOptions& options = {});

// Floquet exponents (Johnson 2013 §15.6): λ = ln Λ / T for each multiplier
// Λ of the monodromy matrix, with the branch of the logarithm (λ + i n 2π/T,
// Eq. 15.148) chosen by continuity from the constant-coefficient root it
// is matched to (Johnson pp. 570–571): pairs are matched nearest first, one
// to one. Multipliers with |Λ| < min_multiplier (modes that vanish within a
// revolution) are left unmatched.
struct FloquetModes {
    Eigen::VectorXcd multipliers;
    Eigen::VectorXcd exponents;         // on the chosen branch (principal if unmatched) [1/s]
    std::vector<std::ptrdiff_t> match;  // index into `reference`, or −1
};
FloquetModes floquet(const Eigen::MatrixXd& monodromy, double period,
                     const Eigen::VectorXcd& reference, double min_multiplier = 1e-6);

// A mode of the condensed model: eigenvalue, undamped natural frequency
// ω_n = |λ| [rad/s] and damping ratio ζ = −Re λ/|λ| (±1 for a real root),
// its longitudinal share, and a label (Padfield 3rd ed. §4.3.2 names). The
// label is a heuristic for reports, and only indicative where the modes
// couple strongly (the UH-60 in hover). The eigenvector is weighted by
// typical sizes (1 m/s, 0.1 rad/s, 0.1 rad) and split into longitudinal
// (u, w, q, θ) and lateral (v, p, r, φ, ψ_h) shares. A pair oscillates if
// ζ < 0.9. Longitudinal: phugoid (oscillating, ω_n < 1 rad/s) or short
// period; heave, pitch or speed subsidence (w, q and θ, or u dominant).
// Lateral: Dutch roll (oscillating, u > 15 m/s, ω_n ≥ 0.3 rad/s), lateral
// phugoid (oscillating, u ≤ 15 m/s), lateral oscillation (other
// oscillating); roll subsidence (p and φ dominant, ω_n > 0.5 rad/s), yaw
// subsidence (r dominant, ω_n > 0.1 rad/s), spiral (other). ω_n < 1e-6:
// heading (neutral).
// With retained states, `retained_weights` gives each one's weight (one over
// its typical size); body is the body states' weighted share of the
// eigenvector (0 for a mode with no body motion), and a mode with body < 0.5
// is labelled "control system".
struct Mode {
    std::complex<double> eigenvalue;
    double frequency = 0.0;     // ω_n [rad/s]
    double damping = 0.0;       // ζ [-]
    double longitudinal = 0.0;  // weighted longitudinal share of the body part [-]
    double body = 1.0;          // weighted body share of the eigenvector [-]
    std::string label;
};

// The modes of a condensed model about a trim with body x velocity u [m/s],
// one per conjugate pair (Im λ ≥ 0), slowest first. Throws InitError unless
// retained_weights has one weight per retained state (none for a 9-state
// model).
std::vector<Mode> modes(const CondensedModel& m, double u,
                        const std::vector<double>& retained_weights = {});

// Frequency response of ẋ = A x + B u at ω [rad/s]: H(iω) = (iωI − A)⁻¹ B,
// states by inputs, in the units of x per unit of u (the Laplace transfer
// function of the state-space model on the imaginary axis, s = iω). For the
// constant-coefficient models (the averaged
// retained model or the condensed one). Throws InitError if iωI − A is
// singular (ω on an eigenvalue) or the sizes don't fit.
Eigen::MatrixXcd frequency_response(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                    double omega);

// Padfield's state ordering (3rd ed. Eq. 4.44): u, w, q, θ, v, p, φ, r, then
// ψ_h, as indices into the condensed states (u, v, w, p, q, r, φ, θ, ψ_h).
inline constexpr std::array<std::size_t, 9> kPadfieldOrder = {0, 2, 4, 7, 1, 3, 6, 5, 8};

// The indices (into A) of the states removed by condense() as feeding
// nothing: the largest set S of non-body states with A[i, j] = 0 exactly
// for every i ∉ S, j ∈ S (`first` leading states are never removed).
std::vector<std::size_t> feed_nothing_states(const Eigen::MatrixXd& A, std::size_t first);

}  // namespace eyas
