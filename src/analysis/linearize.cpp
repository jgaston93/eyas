#include "analysis/linearize.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <numbers>

#include "analysis/free_flight.hpp"
#include "analysis/held_orbit.hpp"
#include "analysis/trim.hpp"
#include "core/attitude.hpp"
#include "core/rigid_body.hpp"
#include "core/rk4.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "util/error.hpp"
#include "util/math.hpp"

namespace eyas {
namespace {

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

constexpr double kPi = std::numbers::pi;

// Name of multiblade coordinate i of an N-bladed rotor (Johnson 2013 Eq. 15.77).
std::string multiblade_name(int blades, int i) {
    if (i == 0) {
        return "beta_0";
    }
    if (blades % 2 == 0 && i == blades - 1) {
        return "beta_d";
    }
    const int n = (i + 1) / 2;
    return "beta_" + std::to_string(n) + (i % 2 == 1 ? "c" : "s");
}

}  // namespace

// ---- Reduced coordinates -------------------------------------------------------

ReducedCoordinates::ReducedCoordinates(const Executive& ex, bool with_height, double period)
    : with_height_(with_height) {
    names_ = {"u", "v", "w", "p", "q", "r", "phi", "theta", "psi_h"};
    if (with_height_) {
        names_.emplace_back("h");
    }
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        const DynamicComponent& c = ex.component(i);
        const std::string prefix =
            c.name().empty() ? "component" + std::to_string(i) : std::string(c.name());
        const std::size_t off = ex.component_offset(i);
        std::size_t first = 0;  // first state copied as is
        const auto* b = dynamic_cast<const BladeElementRotor*>(&c);
        if (b != nullptr && b->config().flap.has_value()) {
            Rotor r;
            r.rotor = b;
            r.full = off;
            r.reduced = names_.size();
            r.blades = b->config().num_blades;
            r.omega = period > 0.0 ? 2.0 * kPi / period : b->config().rotor_speed;
            const auto& link = b->config().shaft;
            if (b->shaft_index() >= 0 && link.has_value() &&
                ex.shafts()[static_cast<std::size_t>(b->shaft_index())].drive == ShaftDrive::free) {
                r.on_free_shaft = true;
                r.shaft = ex.shaft_offset(static_cast<std::size_t>(b->shaft_index()));
                r.gear = link->gear;
            }
            for (int k = 0; k < r.blades; ++k) {
                names_.push_back(prefix + "." + multiblade_name(r.blades, k));
            }
            for (int k = 0; k < r.blades; ++k) {
                names_.push_back(prefix + "." + multiblade_name(r.blades, k) + "_dot");
            }
            rotors_.push_back(r);
            first = 2 * static_cast<std::size_t>(r.blades);
        }
        for (std::size_t j = first; j < ex.component_size(i); ++j) {
            // A rotor's own azimuth is a phase, as a free shaft's (D-053
            // amendment 3).
            if (ex.steady_kinds()[off + j] == SteadyKind::phase) {
                azimuths_.emplace_back(off + j, names_.size());
            } else {
                copied_.emplace_back(off + j, names_.size());
            }
            names_.push_back(prefix + "[" + std::to_string(j) + "]");
        }
    }
    // A free shaft's azimuth and speed are degrees of freedom (D-052,
    // D-053): a change of speed shifts the rotor's phase on its orbit, and
    // with it the phase of every load periodic in it.
    for (std::size_t k = 0; k < ex.shafts().size(); ++k) {
        if (ex.shafts()[k].drive == ShaftDrive::free) {
            azimuths_.emplace_back(ex.shaft_offset(k), names_.size());
            names_.push_back(ex.shafts()[k].name + ".psi");
            copied_.emplace_back(ex.shaft_offset(k) + 1, names_.size());
            names_.push_back(ex.shafts()[k].name + ".omega");
        }
    }
}

ReducedCoordinates::Phase ReducedCoordinates::phase(const Rotor& r, double t,
                                                    std::span<const double> x) {
    Phase p;
    if (r.on_free_shaft) {
        // The rotor's own azimuth, so the multiblade coordinates turn with it
        // and the shaft's azimuth is cyclic (D-053 amendment 2).
        p.psi = r.gear * x[r.shaft];
        p.rate = r.gear * x[r.shaft + 1];
    } else {
        p.psi = r.omega * t;
        p.rate = r.omega;
    }
    p.psi = std::fmod(p.psi, 2.0 * kPi);
    if (p.psi < 0.0) {
        p.psi += 2.0 * kPi;
    }
    return p;
}

void ReducedCoordinates::basis(const Rotor& r, double psi_rotor, int k, std::span<double> phi,
                               std::span<double> dphi) {
    const double psi = r.rotor->blade_azimuth(psi_rotor, k);
    const auto n_b = static_cast<std::size_t>(r.blades);
    phi[0] = 1.0;
    dphi[0] = 0.0;
    for (std::size_t i = 1; i < n_b; ++i) {
        if (n_b % 2 == 0 && i == n_b - 1) {
            phi[i] = k % 2 == 0 ? -1.0 : 1.0;  // (−1)^(k+1), as multiblade_flap()
            dphi[i] = 0.0;
            continue;
        }
        const std::size_t harmonic = (i + 1) / 2;  // cos at odd i, sin at even i
        const auto n = static_cast<double>(harmonic);
        if (i % 2 == 1) {
            phi[i] = std::cos(n * psi);
            dphi[i] = -n * std::sin(n * psi);
        } else {
            phi[i] = std::sin(n * psi);
            dphi[i] = n * std::cos(n * psi);
        }
    }
}

ReducedCoordinates::FlapKind ReducedCoordinates::flap_kind(std::size_t i) const noexcept {
    for (const Rotor& r : rotors_) {
        const auto n_b = static_cast<std::size_t>(r.blades);
        if (i >= r.reduced && i < r.reduced + n_b) {
            return FlapKind::coordinate;
        }
        if (i >= r.reduced + n_b && i < r.reduced + 2 * n_b) {
            return FlapKind::rate;
        }
    }
    return FlapKind::none;
}

std::size_t ReducedCoordinates::reduced_of(std::size_t full) const noexcept {
    for (const auto& [f, red] : copied_) {
        if (f == full) {
            return red;
        }
    }
    return size();
}

std::vector<std::size_t> ReducedCoordinates::azimuth_states() const {
    std::vector<std::size_t> out;
    out.reserve(azimuths_.size());
    for (const auto& [full, red] : azimuths_) {
        out.push_back(red);
    }
    return out;
}

void ReducedCoordinates::derivative(int blades, std::span<const double> q, std::span<double> out) {
    std::fill(out.begin(), out.end(), 0.0);
    // Σ_j q_j φ_j′ = Σ_n n (q_ns cos nψ − q_nc sin nψ), n = 1…(N_b−1)/2.
    for (int n = 1; n <= (blades - 1) / 2; ++n) {
        const auto ic = static_cast<std::size_t>(2 * n - 1);
        out[ic] = n * q[ic + 1];
        out[ic + 1] = -n * q[ic];
    }
}

void ReducedCoordinates::to_full(double t, const VecX& xr, std::span<const double> ref,
                                 std::span<double> x) const {
    std::copy(ref.begin(), ref.end(), x.begin());
    BodyState b = unpack_body(ref);
    b.vel_b = Vec3(xr(0), xr(1), xr(2));
    b.omega_b = Vec3(xr(3), xr(4), xr(5));
    b.q_nb = quaternion(EulerAngles{xr(kPhi), xr(kTheta), xr(kPsi)});
    if (with_height_) {
        b.pos_n.z() -= xr(kHeight);
    }
    pack_body(b, x);
    // The shafts first: the blades' basis turns with their rotor.
    for (const auto& [full, red] : copied_) {
        x[full] = xr(static_cast<Eigen::Index>(red));
    }
    for (const auto& [full, red] : azimuths_) {
        x[full] = xr(static_cast<Eigen::Index>(red));
    }
    std::array<double, kMaxBlades> phi{};
    std::array<double, kMaxBlades> dphi{};
    for (const Rotor& r : rotors_) {
        const auto n_b = static_cast<std::size_t>(r.blades);
        const auto q = [&](std::size_t i) { return xr(static_cast<Eigen::Index>(r.reduced + i)); };
        const auto qd = [&](std::size_t i) {
            return xr(static_cast<Eigen::Index>(r.reduced + n_b + i));
        };
        const Phase ph = phase(r, t, x);
        for (int k = 0; k < r.blades; ++k) {
            basis(r, ph.psi, k, phi, dphi);
            double beta = 0.0;
            double rate = 0.0;
            for (std::size_t i = 0; i < n_b; ++i) {
                beta += q(i) * phi[i];
                rate += qd(i) * phi[i] + ph.rate * q(i) * dphi[i];
            }
            const std::size_t j = r.full + 2 * static_cast<std::size_t>(k);
            x[j] = beta;
            x[j + 1] = rate;
        }
    }
}

VecX ReducedCoordinates::to_reduced(double t, std::span<const double> x,
                                    std::span<const double> ref) const {
    VecX xr = VecX::Zero(static_cast<Eigen::Index>(size()));
    BodyState b = unpack_body(x);
    b.q_nb.normalize();
    const BodyState b_ref = unpack_body(ref);
    for (int i = 0; i < 3; ++i) {
        xr(i) = b.vel_b(i);
        xr(3 + i) = b.omega_b(i);
    }
    const EulerAngles e = euler_angles(b.q_nb);
    const double heading_ref = euler_angles(b_ref.q_nb).heading;
    xr(kPhi) = e.roll;
    xr(kTheta) = e.pitch;
    xr(kPsi) = heading_ref + wrap_pi(e.heading - heading_ref);
    if (with_height_) {
        xr(kHeight) = b_ref.pos_n.z() - b.pos_n.z();
    }
    std::array<double, kMaxBlades> phi{};
    std::array<double, kMaxBlades> dphi{};
    std::array<double, kMaxBlades> q{};
    std::array<double, kMaxBlades> m{};
    std::array<double, kMaxBlades> dq{};
    for (const Rotor& r : rotors_) {
        const auto n_b = static_cast<std::size_t>(r.blades);
        std::fill(q.begin(), q.end(), 0.0);
        std::fill(m.begin(), m.end(), 0.0);
        const Phase ph = phase(r, t, x);
        for (int k = 0; k < r.blades; ++k) {
            basis(r, ph.psi, k, phi, dphi);
            const std::size_t j = r.full + 2 * static_cast<std::size_t>(k);
            for (std::size_t i = 0; i < n_b; ++i) {
                // Inverse weights: 1/N for β0 and β_d, 2/N for the harmonics.
                const bool single = i == 0 || (n_b % 2 == 0 && i == n_b - 1);
                const double w = (single ? 1.0 : 2.0) / static_cast<double>(n_b);
                q[i] += w * phi[i] * x[j];
                m[i] += w * phi[i] * x[j + 1];
            }
        }
        // m = q̇ + ψ̇ D q.
        derivative(r.blades, q, dq);
        for (std::size_t i = 0; i < n_b; ++i) {
            xr(static_cast<Eigen::Index>(r.reduced + i)) = q[i];
            xr(static_cast<Eigen::Index>(r.reduced + n_b + i)) = m[i] - ph.rate * dq[i];
        }
    }
    for (const auto& [full, red] : copied_) {
        xr(static_cast<Eigen::Index>(red)) = x[full];
    }
    // Within ±π of ref's, as ψ_h: continuous across the wrap.
    for (const auto& [full, red] : azimuths_) {
        xr(static_cast<Eigen::Index>(red)) = ref[full] + wrap_pi(x[full] - ref[full]);
    }
    return xr;
}

VecX ReducedCoordinates::rates(double t, std::span<const double> x,
                               std::span<const double> xdot) const {
    const VecX xr = to_reduced(t, x, x);
    VecX f = VecX::Zero(static_cast<Eigen::Index>(size()));
    for (std::size_t i = 0; i < 3; ++i) {
        f(static_cast<Eigen::Index>(i)) = xdot[body_layout::kVel + i];
        f(static_cast<Eigen::Index>(3 + i)) = xdot[body_layout::kOmega + i];
    }
    // Padfield 3rd ed. Eq. 4.37.
    const double phi = xr(kPhi);
    const double theta = xr(kTheta);
    const double p = xr(3);
    const double q = xr(4);
    const double r = xr(5);
    f(kPhi) = p + q * std::sin(phi) * std::tan(theta) + r * std::cos(phi) * std::tan(theta);
    f(kTheta) = q * std::cos(phi) - r * std::sin(phi);
    f(kPsi) = (q * std::sin(phi) + r * std::cos(phi)) / std::cos(theta);
    if (with_height_) {
        f(kHeight) = -xdot[body_layout::kPos + 2];
    }
    std::array<double, kMaxBlades> phi_k{};
    std::array<double, kMaxBlades> dphi{};
    std::array<double, kMaxBlades> a{};
    std::array<double, kMaxBlades> qv{};
    std::array<double, kMaxBlades> qdv{};
    std::array<double, kMaxBlades> d1{};
    std::array<double, kMaxBlades> d2{};
    for (const Rotor& rot : rotors_) {
        const auto n_b = static_cast<std::size_t>(rot.blades);
        std::fill(a.begin(), a.end(), 0.0);
        const Phase ph = phase(rot, t, x);
        for (int k = 0; k < rot.blades; ++k) {
            basis(rot, ph.psi, k, phi_k, dphi);
            const std::size_t j = rot.full + 2 * static_cast<std::size_t>(k);
            for (std::size_t i = 0; i < n_b; ++i) {
                const bool single = i == 0 || (n_b % 2 == 0 && i == n_b - 1);
                const double w = (single ? 1.0 : 2.0) / static_cast<double>(n_b);
                a[i] += w * phi_k[i] * xdot[j + 1];
            }
        }
        for (std::size_t i = 0; i < n_b; ++i) {
            qv[i] = xr(static_cast<Eigen::Index>(rot.reduced + i));
            qdv[i] = xr(static_cast<Eigen::Index>(rot.reduced + n_b + i));
        }
        derivative(rot.blades, qdv, d1);  // D q̇
        derivative(rot.blades, qv, d2);   // D q
        std::array<double, kMaxBlades> dd{};
        derivative(rot.blades, d2, dd);  // D² q
        // q̈ = a − 2ψ̇ D q̇ − ψ̇² D² q − ψ̈ D q; ψ̈ = 0 with a uniform rotation,
        // gear × Ω̇ on a free shaft.
        const double w = ph.rate;
        const double wdot = rot.on_free_shaft ? rot.gear * xdot[rot.shaft + 1] : 0.0;
        for (std::size_t i = 0; i < n_b; ++i) {
            f(static_cast<Eigen::Index>(rot.reduced + i)) = qdv[i];
            f(static_cast<Eigen::Index>(rot.reduced + n_b + i)) =
                a[i] - 2.0 * w * d1[i] - w * w * dd[i] - wdot * d2[i];
        }
    }
    for (const auto& [full, red] : copied_) {
        f(static_cast<Eigen::Index>(red)) = xdot[full];
    }
    for (const auto& [full, red] : azimuths_) {
        f(static_cast<Eigen::Index>(red)) = xdot[full];
    }
    return f;
}

// ---- Linearization --------------------------------------------------------------

std::vector<double> perturbation_steps(const ReducedCoordinates& rc, const LinearizeOptions& o) {
    std::vector<double> h(rc.size(), o.step_other);
    for (std::size_t i = 0; i < 3; ++i) {
        h[i] = o.step_velocity;
        h[3 + i] = o.step_rate;
        h[6 + i] = o.step_angle;
    }
    if (rc.with_height()) {
        h[ReducedCoordinates::kHeight] = o.step_height;
    }
    for (std::size_t i = rc.body_size(); i < rc.size(); ++i) {
        switch (rc.flap_kind(i)) {
            case ReducedCoordinates::FlapKind::coordinate:
                h[i] = o.step_flap;
                break;
            case ReducedCoordinates::FlapKind::rate:
                h[i] = o.step_flap_rate;
                break;
            case ReducedCoordinates::FlapKind::none:
                break;
        }
    }
    return h;
}

namespace {

// The reduced coordinates pinned on a limit or a switch at any of `states`
// (sampled evenly over the period T): DynamicComponent::pinned_states().
std::vector<bool> pinned_reduced(const Executive& ex, const ReducedCoordinates& rc,
                                 const std::vector<std::vector<double>>& states, double T,
                                 const Controls& u, const Environment& env) {
    std::vector<unsigned char> pinned;
    std::vector<bool> frozen(rc.size(), false);
    for (std::size_t j = 0; j < states.size(); ++j) {
        const double t = T * static_cast<double>(j) / static_cast<double>(states.size());
        const SimContext ctx = ex.context(t, states[j], u, env);
        for (std::size_t i = 0; i < ex.num_components(); ++i) {
            pinned.assign(ex.component_size(i), 0);
            ex.component(i).pinned_states(ctx,
                                          std::span<const double>(states[j]).subspan(
                                              ex.component_offset(i), ex.component_size(i)),
                                          pinned);
            for (std::size_t k = 0; k < pinned.size(); ++k) {
                const std::size_t red = rc.reduced_of(ex.component_offset(i) + k);
                if (pinned[k] != 0 && red < rc.size()) {
                    frozen[red] = true;
                }
            }
        }
    }
    return frozen;
}

}  // namespace

LinearModel linearize(const Executive& ex, const TrimCondition& c, const TrimResult& r,
                      const LinearizeOptions& o) {
    if (!o.height && !std::isnan(c.ground_height)) {
        throw InitError(
            "linearize: with a ground the height is a coordinate (ground effect, D-050); set "
            "options.height");
    }
    const double T = trim_period(ex, r);
    const ReducedCoordinates rc(ex, o.height, T);
    const HeldOrbit held = held_orbit(ex, c, r, o.azimuths, o.steps_per_period);
    const std::vector<std::vector<double>>& orbit = held.states;
    const Environment env = trim_environment(c);
    const auto n = static_cast<Eigen::Index>(rc.size());
    const std::vector<double> h = perturbation_steps(rc, o);

    LinearModel m;
    m.states = rc.names();
    m.inputs = {"collective", "lat_cyclic", "lon_cyclic", "pedal"};
    m.body_size = rc.body_size();
    m.period = T;
    m.A = MatX::Zero(n, n);
    m.B = MatX::Zero(n, 4);
    m.unconverged = held.unconverged;

    std::vector<double> x(ex.num_states(), 0.0);
    std::vector<double> xd(ex.num_states(), 0.0);
    const auto f = [&](double t, const VecX& xr, const Controls& u,
                       const std::vector<double>& ref) {
        rc.to_full(t, xr, ref, x);
        if (!ex.derivatives(t, x, u, env, xd)) {
            ++m.unconverged;
        }
        return rc.rates(t, x, xd);
    };
    for (std::size_t j = 0; j < orbit.size(); ++j) {
        const double t = T * static_cast<double>(j) / static_cast<double>(orbit.size());
        const std::vector<double>& ref = orbit[j];
        const VecX x0 = rc.to_reduced(t, ref, ref);
        MatX A(n, n);
        MatX B(n, 4);
        for (Eigen::Index k = 0; k < n; ++k) {
            const double hk = h[static_cast<std::size_t>(k)];
            VecX xp = x0;
            VecX xm = x0;
            xp(k) += hk;
            xm(k) -= hk;
            A.col(k) = (f(t, xp, r.controls, ref) - f(t, xm, r.controls, ref)) / (2.0 * hk);
        }
        for (int k = 0; k < 4; ++k) {
            Controls up = r.controls;
            Controls um = r.controls;
            const std::array<double*, 4> pp{&up.collective, &up.lat_cyclic, &up.lon_cyclic,
                                            &up.pedal};
            const std::array<double*, 4> pm{&um.collective, &um.lat_cyclic, &um.lon_cyclic,
                                            &um.pedal};
            *pp[static_cast<std::size_t>(k)] += o.step_control;
            *pm[static_cast<std::size_t>(k)] -= o.step_control;
            B.col(k) = (f(t, x0, up, ref) - f(t, x0, um, ref)) / (2.0 * o.step_control);
        }
        m.A += A / static_cast<double>(orbit.size());
        m.B += B / static_cast<double>(orbit.size());
        m.A_periodic.push_back(std::move(A));
        m.B_periodic.push_back(std::move(B));
    }
    // States pinned on a limit or a switch anywhere on the orbit are held
    // (D-053 amendment 2): a central difference across the switch is no
    // derivative, and small perturbations leave them where they are.
    const std::vector<bool> frozen = pinned_reduced(ex, rc, orbit, T, r.controls, env);
    for (std::size_t i = 0; i < frozen.size(); ++i) {
        if (!frozen[i]) {
            continue;
        }
        const auto k = static_cast<Eigen::Index>(i);
        for (MatX& A : m.A_periodic) {
            A.row(k).setZero();
            A.col(k).setZero();
        }
        for (MatX& B : m.B_periodic) {
            B.row(k).setZero();
        }
        m.A.row(k).setZero();
        m.A.col(k).setZero();
        m.B.row(k).setZero();
    }
    // A free shaft's azimuth is cyclic in the averaged model: its column is
    // the mean of a periodic function's derivative over the revolution, zero
    // but for sampling and rounding, so it is set to zero and the azimuth
    // feeds nothing (D-053 amendment 2). The periodic model keeps it.
    for (const std::size_t i : rc.azimuth_states()) {
        m.A.col(static_cast<Eigen::Index>(i)).setZero();
    }
    return m;
}

std::vector<VecX> linear_response(const LinearModel& m, const VecX& x0,
                                  const std::function<Eigen::Vector4d(double)>& u, int steps,
                                  bool averaged) {
    const std::size_t J = m.A_periodic.size();
    if (J == 0 || J % 2 != 0) {
        throw InitError("linear response: needs an even number of azimuth samples");
    }
    const double h = 2.0 * m.period / static_cast<double>(J);
    const auto f = [&](std::size_t sample, double t, const VecX& x) {
        const std::size_t j = sample % J;
        const MatX& A = averaged ? m.A : m.A_periodic[j];
        const MatX& B = averaged ? m.B : m.B_periodic[j];
        return VecX(A * x + B * u(t));
    };
    std::vector<VecX> out;
    out.reserve(static_cast<std::size_t>(steps) + 1);
    out.push_back(x0);
    VecX x = x0;
    for (int n = 0; n < steps; ++n) {
        const auto s0 = 2 * static_cast<std::size_t>(n);
        const double t = h * n;
        const VecX k1 = f(s0, t, x);
        const VecX k2 = f(s0 + 1, t + 0.5 * h, x + 0.5 * h * k1);
        const VecX k3 = f(s0 + 1, t + 0.5 * h, x + 0.5 * h * k2);
        const VecX k4 = f(s0 + 2, t + h, x + h * k3);
        x += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        out.push_back(x);
    }
    return out;
}

// ---- Floquet ---------------------------------------------------------------------

Monodromy monodromy(const Executive& ex, const TrimCondition& c, const TrimResult& r,
                    const LinearizeOptions& o) {
    if (!o.height) {
        throw InitError("monodromy: the flow map needs h (density changes with height)");
    }
    const double T = trim_period(ex, r);
    const ReducedCoordinates rc(ex, true, T);
    const std::vector<double> start = held_orbit_start(ex, c, r);
    const Environment env = trim_environment(c);
    const int steps = o.steps_per_period;
    const double dt = T / steps;
    std::vector<double> x(ex.num_states());
    Rk4 rk4(x.size());
    Monodromy out;
    const auto controls = [&](double) { return r.controls; };
    const auto flow = [&](const VecX& xr0) {
        rc.to_full(0.0, xr0, start, x);
        for (int s = 0; s < steps; ++s) {
            const double t = T * static_cast<double>(s) / steps;
            out.unconverged +=
                static_cast<std::size_t>(free_step(ex, rk4, controls, env, t, dt, x));
        }
        return rc.to_reduced(T, x, start);
    };
    const VecX x0 = rc.to_reduced(0.0, start, start);
    const std::vector<double> h = perturbation_steps(rc, o);
    const auto n = static_cast<Eigen::Index>(rc.size());
    // States pinned on a limit or a switch stay where they are and move
    // nothing (D-053 amendment 2): their rows and columns are the identity's.
    const std::vector<bool> frozen = pinned_reduced(ex, rc, {start}, T, r.controls, env);
    out.matrix.resize(n, n);
    for (Eigen::Index k = 0; k < n; ++k) {
        if (frozen[static_cast<std::size_t>(k)]) {
            out.matrix.col(k).setZero();
            continue;
        }
        const double hk = h[static_cast<std::size_t>(k)];
        VecX xp = x0;
        VecX xm = x0;
        xp(k) += hk;
        xm(k) -= hk;
        out.matrix.col(k) = (flow(xp) - flow(xm)) / (2.0 * hk);
    }
    for (Eigen::Index k = 0; k < n; ++k) {
        if (frozen[static_cast<std::size_t>(k)]) {
            out.matrix.row(k).setZero();
            out.matrix(k, k) = 1.0;
        }
    }
    return out;
}

FloquetModes floquet(const MatX& monodromy, double period, const Eigen::VectorXcd& reference,
                     double min_multiplier) {
    FloquetModes out;
    out.multipliers = monodromy.eigenvalues();
    const Eigen::Index n = out.multipliers.size();
    out.exponents.resize(n);
    out.match.assign(static_cast<std::size_t>(n), -1);
    const double w = 2.0 * kPi / period;  // fundamental frequency of the system
    struct Pair {
        double distance;
        Eigen::Index floquet;
        Eigen::Index ref;
        std::complex<double> lambda;
    };
    std::vector<Pair> pairs;
    for (Eigen::Index i = 0; i < n; ++i) {
        const std::complex<double> principal = std::log(out.multipliers(i)) / period;
        out.exponents(i) = principal;
        if (std::abs(out.multipliers(i)) < min_multiplier) {
            continue;
        }
        for (Eigen::Index k = 0; k < reference.size(); ++k) {
            const double branch = std::round((reference(k).imag() - principal.imag()) / w);
            const std::complex<double> l = principal + std::complex<double>(0.0, branch * w);
            pairs.push_back({std::abs(l - reference(k)), i, k, l});
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.distance < b.distance; });
    std::vector<bool> ref_used(static_cast<std::size_t>(reference.size()), false);
    for (const Pair& p : pairs) {
        auto& m = out.match[static_cast<std::size_t>(p.floquet)];
        if (m >= 0 || ref_used[static_cast<std::size_t>(p.ref)]) {
            continue;
        }
        m = p.ref;
        ref_used[static_cast<std::size_t>(p.ref)] = true;
        out.exponents(p.floquet) = p.lambda;
    }
    return out;
}

// ---- Modes -----------------------------------------------------------------------

Eigen::MatrixXcd frequency_response(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                                    double omega) {
    if (A.rows() != A.cols() || B.rows() != A.rows() || !std::isfinite(omega)) {
        throw InitError("frequency response: A must be square, B fit it, and ω be finite");
    }
    const Eigen::Index n = A.rows();
    const Eigen::MatrixXcd M = std::complex<double>(0.0, omega) * Eigen::MatrixXcd::Identity(n, n) -
                               A.cast<std::complex<double>>();
    const Eigen::FullPivLU<Eigen::MatrixXcd> lu(M);
    if (!lu.isInvertible()) {
        throw InitError("frequency response: iωI − A is singular (ω on an eigenvalue)");
    }
    return lu.solve(B.cast<std::complex<double>>());
}

std::vector<Mode> modes(const CondensedModel& m, double u,
                        const std::vector<double>& retained_weights) {
    const auto extra = static_cast<std::size_t>(m.A.rows()) - 9;
    if (retained_weights.size() != extra) {
        // Weighing retained states zero would label their own roots as body
        // modes (code review), so a caller must say how much each counts.
        throw InitError("modes: one weight per retained state");
    }
    const Eigen::EigenSolver<MatX> es(m.A);
    // Weights: 1 m/s, 0.1 rad/s, 0.1 rad (states u v w p q r φ θ ψ_h).
    const std::array<double, 9> w = {1.0, 1.0, 1.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    std::vector<Mode> out;
    for (Eigen::Index i = 0; i < es.eigenvalues().size(); ++i) {
        const std::complex<double> l = es.eigenvalues()(i);
        if (l.imag() < 0.0) {
            continue;
        }
        Mode md;
        md.eigenvalue = l;
        md.frequency = std::abs(l);
        md.damping = md.frequency > 0.0 ? -l.real() / md.frequency : 0.0;
        std::array<double, 9> e{};
        for (std::size_t k = 0; k < 9; ++k) {
            e[k] = std::norm(es.eigenvectors()(static_cast<Eigen::Index>(k), i) * w[k]);
        }
        const double lon = e[0] + e[2] + e[4] + e[7];
        const double lat = e[1] + e[3] + e[5] + e[6] + e[8];
        md.longitudinal = lon + lat > 0.0 ? lon / (lon + lat) : 0.0;
        double rest = 0.0;
        for (std::size_t k = 0; k < retained_weights.size(); ++k) {
            rest += std::norm(es.eigenvectors()(static_cast<Eigen::Index>(9 + k), i) *
                              retained_weights[k]);
        }
        // No body motion at all (a retained state's own root): all control system.
        md.body = lon + lat > 0.0 ? (lon + lat) / (lon + lat + rest) : 0.0;
        const bool oscillatory = l.imag() > 1e-9 && md.damping < 0.9;
        if (md.body < 0.5) {
            md.label = "control system";
        } else if (md.frequency < 1e-6) {
            md.label = "heading";
        } else if (lon >= lat) {
            if (oscillatory) {
                md.label = md.frequency < 1.0 ? "phugoid" : "short period";
            } else if (e[2] >= std::max(e[4] + e[7], e[0])) {
                md.label = "heave subsidence";
            } else if (e[4] + e[7] >= e[0]) {
                md.label = "pitch subsidence";
            } else {
                md.label = "speed subsidence";
            }
        } else if (oscillatory) {
            if (u <= 15.0) {
                md.label = "lateral phugoid";
            } else {
                md.label = md.frequency >= 0.3 ? "Dutch roll" : "lateral oscillation";
            }
        } else if (e[3] + e[6] >= e[5] && md.frequency > 0.5) {
            md.label = "roll subsidence";
        } else if (e[5] > e[3] + e[6] && md.frequency > 0.1) {
            md.label = "yaw subsidence";
        } else {
            md.label = "spiral";
        }
        out.push_back(md);
    }
    std::sort(out.begin(), out.end(),
              [](const Mode& a, const Mode& b) { return a.frequency < b.frequency; });
    return out;
}

// ---- Condensation ---------------------------------------------------------------

std::vector<std::size_t> feed_nothing_states(const MatX& A, std::size_t first) {
    // The states that reach a leading state through the influence graph
    // (j → i where A(i, j) ≠ 0) feed something; the rest form the largest
    // set that feeds nothing outside itself.
    const auto n = static_cast<std::size_t>(A.rows());
    std::vector<bool> reaches(n, false);
    std::vector<std::size_t> stack;
    for (std::size_t i = 0; i < first; ++i) {
        reaches[i] = true;
        stack.push_back(i);
    }
    while (!stack.empty()) {
        const std::size_t i = stack.back();
        stack.pop_back();
        for (std::size_t j = 0; j < n; ++j) {
            if (!reaches[j] && j != i &&
                A(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) != 0.0) {
                reaches[j] = true;
                stack.push_back(j);
            }
        }
    }
    std::vector<std::size_t> s;
    for (std::size_t j = first; j < n; ++j) {
        if (!reaches[j]) {
            s.push_back(j);
        }
    }
    return s;
}

CondensedModel condense(const LinearModel& m, const std::vector<std::size_t>& retained) {
    constexpr std::size_t nb = 9;  // u v w p q r φ θ ψ_h
    const auto n = static_cast<std::size_t>(m.A.rows());
    std::vector<bool> is_retained(n, false);
    for (const std::size_t i : retained) {
        if (i < m.body_size || i >= n || is_retained[i]) {
            throw InitError("condense: retained states must be distinct non-body states");
        }
        is_retained[i] = true;
    }
    // Keep everything but h (as Padfield's Eq. 4.44): the body states, the
    // retained ones, then the rest.
    std::vector<std::size_t> keep;
    for (std::size_t i = 0; i < nb; ++i) {
        keep.push_back(i);
    }
    keep.insert(keep.end(), retained.begin(), retained.end());
    for (std::size_t i = m.body_size; i < n; ++i) {
        if (!is_retained[i]) {
            keep.push_back(i);
        }
    }
    const std::size_t lead = nb + retained.size();  // never removed nor condensed
    const auto sub = [](const MatX& M, const std::vector<std::size_t>& rows,
                        const std::vector<std::size_t>& cols) {
        MatX S(static_cast<Eigen::Index>(rows.size()), static_cast<Eigen::Index>(cols.size()));
        for (std::size_t a = 0; a < rows.size(); ++a) {
            for (std::size_t b = 0; b < cols.size(); ++b) {
                S(static_cast<Eigen::Index>(a), static_cast<Eigen::Index>(b)) =
                    M(static_cast<Eigen::Index>(rows[a]), static_cast<Eigen::Index>(cols[b]));
            }
        }
        return S;
    };
    const MatX A1 = sub(m.A, keep, keep);
    CondensedModel out;
    std::vector<std::size_t> drop_local = feed_nothing_states(A1, lead);
    std::vector<bool> dropped(keep.size(), false);
    for (std::size_t j : drop_local) {
        dropped[j] = true;
        out.dropped.push_back(keep[j]);
    }
    const MatX Ad = sub(A1, drop_local, drop_local);
    out.dropped_eigenvalues =
        Ad.size() > 0 ? Eigen::VectorXcd(Ad.eigenvalues()) : Eigen::VectorXcd();
    std::vector<std::size_t> body;
    std::vector<std::size_t> rest;
    for (std::size_t i = 0; i < keep.size(); ++i) {
        if (i < lead) {
            body.push_back(keep[i]);
        } else if (!dropped[i]) {
            rest.push_back(keep[i]);
        }
    }
    std::vector<std::size_t> inputs = {0, 1, 2, 3};
    const MatX Abb = sub(m.A, body, body);
    const MatX Abr = sub(m.A, body, rest);
    const MatX Arb = sub(m.A, rest, body);
    const MatX Arr = sub(m.A, rest, rest);
    const MatX Bb = sub(m.B, body, inputs);
    const MatX Br = sub(m.B, rest, inputs);
    out.states.assign(m.states.begin(), m.states.begin() + static_cast<std::ptrdiff_t>(nb));
    for (const std::size_t i : retained) {
        out.states.push_back(m.states[i]);
    }
    if (rest.empty()) {
        out.A = Abb;
        out.B = Bb;
        return out;
    }
    // A state with no dynamics of its own (a zero row) has no quasi-static
    // value; say which (code review, M8c3).
    for (std::size_t a = 0; a < rest.size(); ++a) {
        if (Arr.row(static_cast<Eigen::Index>(a)).isZero(0.0)) {
            throw InitError("condense: " + m.states[rest[a]] +
                            " has no dynamics of its own (e.g. an engaged FPS reference); keep it "
                            "dynamic with condense(m, retained)");
        }
    }
    const Eigen::FullPivLU<MatX> lu(Arr);
    if (!lu.isInvertible()) {
        throw InitError("condense: the quasi-static rotor and inflow block is singular");
    }
    out.A = Abb - Abr * lu.solve(Arb);
    out.B = Bb - Abr * lu.solve(Br);
    return out;
}

}  // namespace eyas
