#pragma once

// Fixtures for the M6d whole-aircraft tests (D-028, D-034): the UH-60A
// config in data/uh60a/uh60a.yaml (built by tools/build_uh60a_config.py from
// GenHel CR-166309), and a harness that evaluates it through the production
// executive with the body held (a = 0, D-035), reading each component's
// loads, which is where the D-028 whole-aircraft checks are stated.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "airframe/landing_gear.hpp"
#include "core/component.hpp"
#include "core/rigid_body.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "sim/build.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"
#include "util/units.hpp"

namespace eyas::test {

inline std::string uh60a_config_path() {
    return std::string(EYAS_VEHICLE_DATA_DIR) + "/uh60a/uh60a.yaml";
}

// GenHel's whole-aircraft weight and CG (p. 5.1-49).
inline constexpr double kGenHelWeight = 16638.0;  // [lbm]
inline constexpr double kFSCG = 355.9;            // [in]
inline constexpr double kWLCG = 248.2;            // [in]

// The travels (D-035), from GenHel Figs. 6.3.1 and 6.4.1 (pp. 6.16-6.17),
// written out here from the figure rather than taken from the generator.
// Normalized input → GenHel inches. Shared by the tests and the validation
// run; if the travels change in tools/build_uh60a_config.py,
// ControlMapsAreGenHelsRigging fails until these follow.
struct Travels {
    static double xc(double c) { return 10.0 * c; }       // stroke 10.0 in, 0 = full down
    static double xa(double l) { return 5.0 + 5.0 * l; }  // stroke 10.0 in, neutral 5.0 in
    // Stroke 10.0 in about neutral cyclic. The figure's three rows imply three
    // neutrals (pedal at neutral; they spread by 0.81 in, DATA_GAPS.md); neutral
    // is the pinned row's, the largest, the end nearest the fit to Ballin's
    // GenHel longitudinal stick (D-035 M7c amendment).
    static double xb2(double f) {
        const double b_rest_low = -0.2936 * -5.539 * kXpNeutral - 9.7754;
        const double neutral = (-11.0 - b_rest_low) / 2.83 + 3.89;
        return neutral + 5.0 * f;
    }
    // Stroke with overtravel 5.38 in about the pinned pedal, which gives 15.0°
    // at mid collective: (21.9 + 1.6 × 5 − 15.0) / 5.539 in. Right pedal: less thrust.
    static constexpr double kXpNeutral = (21.9 + 8.0 - 15.0) / 5.539;
    static double xp(double p) { return kXpNeutral + 2.69 * p; }
};

// The UH-60A, built from the config by the same reader as Model::from_file
// (sim/build.hpp), and evaluated through the executive with the body held
// (Executive::held_derivatives, D-035), so a test can read each component's
// loads.
class Uh60aHarness {
public:
    explicit Uh60aHarness(const std::string& path = uh60a_config_path())
        : Uh60aHarness(Config::from_file(path)) {}
    // From a config already read (e.g. edited text through Config::from_string,
    // with absolute table paths).
    explicit Uh60aHarness(Config cfg)
        : cfg_(std::move(cfg)), ex_(build_executive(cfg_, read_origin(cfg_).origin().altitude)) {}

    const Config& config() const { return cfg_; }
    const Executive& executive() const { return ex_; }
    const MassProperties& airframe_mass() const { return ex_.airframe_mass(); }
    std::size_t num_states() const { return ex_.num_states(); }
    std::size_t size() const { return ex_.num_components(); }
    std::size_t offset(std::size_t i) const { return ex_.component_offset(i); }
    const DynamicComponent& component(std::size_t i) const { return ex_.component(i); }
    std::size_t index(std::string_view name) const {
        for (std::size_t i = 0; i < size(); ++i) {
            if (component(i).name() == name) {
                return i;
            }
        }
        throw std::runtime_error("no component " + std::string(name));
    }
    template <class T>
    const T& get(std::string_view name) const {
        return dynamic_cast<const T&>(component(index(name)));
    }
    std::span<const double> states(std::span<const double> x, std::size_t i) const {
        return x.subspan(offset(i), ex_.component_size(i));
    }

    // Initial state: body at `body`, every component at its initial state.
    std::vector<double> initial(const BodyState& body) const {
        std::vector<double> x(num_states(), 0.0);
        pack_body(body, x);
        for (std::size_t i = 0; i < size(); ++i) {
            component(i).initial_states(
                std::span<double>(x).subspan(offset(i), ex_.component_size(i)));
        }
        ex_.initial_shaft_states(x);  // D-052
        return x;
    }

    // The executive's context at (t, x); its coupling store holds the latest
    // evaluation, so evaluate at (t, x) before reading coupled inputs.
    SimContext context(double t, std::span<const double> x, const Controls& controls,
                       const Environment& env) const {
        return ex_.context(t, x, controls, env);
    }

    // Every component's load0 (body axes, about the airframe CG) and xdot0 at
    // (t, x): the loads with the body acceleration zero. xdot0 is written
    // into xdot (component states only; the body part is left as is). Throws
    // if a component's bounded iteration did not converge (D-023), so no
    // check passes on unconverged loads.
    std::vector<Wrench> evaluate(double t, std::span<const double> x, const Controls& controls,
                                 const Environment& env, std::span<double> xdot = {}) const {
        std::vector<double> xd(num_states(), 0.0);
        std::vector<Wrench> loads(size());
        Vec6 net;
        if (!ex_.held_derivatives(t, x, controls, env, xd, net, loads)) {
            throw std::runtime_error("unconverged component evaluation");
        }
        if (!xdot.empty()) {
            std::copy(xd.begin() + static_cast<std::ptrdiff_t>(body_layout::kSize), xd.end(),
                      xdot.begin() + static_cast<std::ptrdiff_t>(body_layout::kSize));
        }
        return loads;
    }

    // Integrates the component states with the body held at its state in x
    // (a = 0, so ẋ_c = xdot0 exactly), RK4 with step h [s], from t.
    void hold(double& t, std::vector<double>& x, const Controls& controls, const Environment& env,
              double h, int steps) const {
        const std::size_t n = x.size();
        std::vector<double> k1(n, 0.0);
        std::vector<double> k2(n, 0.0);
        std::vector<double> k3(n, 0.0);
        std::vector<double> k4(n, 0.0);
        std::vector<double> tmp(x);  // the body part stays x's
        const auto stage = [&](const std::vector<double>& k, double scale) {
            for (std::size_t i = body_layout::kSize; i < n; ++i) {
                tmp[i] = x[i] + scale * k[i];
            }
        };
        for (int s = 0; s < steps; ++s) {
            (void)evaluate(t, x, controls, env, k1);
            stage(k1, 0.5 * h);
            (void)evaluate(t + 0.5 * h, tmp, controls, env, k2);
            stage(k2, 0.5 * h);
            (void)evaluate(t + 0.5 * h, tmp, controls, env, k3);
            stage(k3, h);
            (void)evaluate(t + h, tmp, controls, env, k4);
            for (std::size_t i = body_layout::kSize; i < n; ++i) {
                x[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
            t += h;
        }
    }

    // The main rotor's blade-passage period 2π/(N_b Ω) [s].
    double passage_period() const {
        const auto& mr = get<BladeElementRotor>("main_rotor").config();
        return 2.0 * std::numbers::pi / (mr.num_blades * mr.rotor_speed);
    }

    // Loads averaged over one blade passage while the body is held: the
    // component states keep integrating (the rotor is periodic at N_b/rev in
    // a held hover). Rectangle rule over `samples` equal steps of exactly one
    // period, exact for harmonics below `samples` per passage.
    std::vector<Wrench> passage_average(double& t, std::vector<double>& x, const Controls& controls,
                                        const Environment& env, int samples = 48) const {
        const double h = passage_period() / samples;
        std::vector<Wrench> avg(size());
        for (int k = 0; k < samples; ++k) {
            const std::vector<Wrench> w = evaluate(t, x, controls, env);
            for (std::size_t i = 0; i < w.size(); ++i) {
                avg[i].F += w[i].F / samples;
                avg[i].M += w[i].M / samples;
            }
            hold(t, x, controls, env, h, 1);
        }
        return avg;
    }

    // Total weight [N]: airframe, blades and unsprung masses (D-002).
    double total_weight() const {
        const auto& mr = get<BladeElementRotor>("main_rotor").config();
        double m = airframe_mass().mass + mr.num_blades * mr.flap->mass;
        for (const auto& g : get<LandingGear>("gear").config().gears) {
            m += g.unsprung_mass;
        }
        return m * units::kStandardGravity;
    }

private:
    Config cfg_;
    Executive ex_;
};

// The held-body balance, body level and at rest (the D-028 hover check):
// every component's load averaged over one blade passage after `settle`
// seconds of held flight, with the sums the checks need.
struct HeldBalance {
    double vertical = 0.0;      // Σ F_z + airframe weight [N], body z (down); 0 when balanced
    double yaw = 0.0;           // Σ N [N·m] about the airframe CG
    std::vector<Wrench> loads;  // per component, passage-averaged
};

inline HeldBalance held_balance(const Uh60aHarness& ac, std::vector<double>& x, double& t,
                                const Controls& c, const Environment& env, double settle = 3.0) {
    ac.hold(t, x, c, env, 1e-3, static_cast<int>(std::lround(settle * 1000.0)));
    double ta = t;
    std::vector<double> xa = x;
    HeldBalance b;
    b.loads = ac.passage_average(ta, xa, c, env);
    b.vertical = ac.airframe_mass().mass * units::kStandardGravity;
    for (const Wrench& w : b.loads) {
        b.vertical += w.F.z();
        b.yaw += w.M.z();
    }
    return b;
}

// The pedal giving Σ N = 0 with the body held, other controls fixed:
// secant iteration from pedal 0 and 0.5. Returns false if it doesn't reach
// |Σ N| < tol [N·m] in `iterations`.
inline bool balance_pedal(const Uh60aHarness& ac, std::vector<double>& x, double& t, Controls& c,
                          const Environment& env, double tol = 0.5, int iterations = 12) {
    double p0 = 0.0;
    c.pedal = p0;
    double n0 = held_balance(ac, x, t, c, env).yaw;
    double p1 = 0.5;
    c.pedal = p1;
    double n1 = held_balance(ac, x, t, c, env).yaw;
    for (int k = 0; k < iterations; ++k) {
        if (std::abs(n1) < tol) {
            return true;
        }
        const double p2 = p1 - n1 * (p1 - p0) / (n1 - n0);
        p0 = p1;
        n0 = n1;
        p1 = p2;
        c.pedal = p1;
        n1 = held_balance(ac, x, t, c, env).yaw;
    }
    return std::abs(n1) < tol;
}

// Hover with the body held level and at rest, cyclic as given: the
// collective and pedal giving zero vertical force and zero yaw moment
// (Newton, finite-difference Jacobian refreshed every iteration). Returns
// false if it doesn't reach |vertical| < tol_f [N] and |Σ N| < tol_m [N·m].
inline bool find_hover(const Uh60aHarness& ac, std::vector<double>& x, double& t, Controls& c,
                       const Environment& env, HeldBalance& out, double tol_f = 0.5,
                       double tol_m = 0.5, int iterations = 10) {
    const double d = 1e-3;
    for (int k = 0; k < iterations; ++k) {
        out = held_balance(ac, x, t, c, env);
        if (std::abs(out.vertical) < tol_f && std::abs(out.yaw) < tol_m) {
            return true;
        }
        Controls cc = c;
        cc.collective += d;
        std::vector<double> xc = x;
        double tc = t;
        const HeldBalance bc = held_balance(ac, xc, tc, cc, env);
        Controls cp = c;
        cp.pedal += d;
        std::vector<double> xp = x;
        double tp = t;
        const HeldBalance bp = held_balance(ac, xp, tp, cp, env);
        const double a = (bc.vertical - out.vertical) / d;
        const double b = (bp.vertical - out.vertical) / d;
        const double e = (bc.yaw - out.yaw) / d;
        const double f = (bp.yaw - out.yaw) / d;
        const double det = a * f - b * e;
        c.collective -= (f * out.vertical - b * out.yaw) / det;
        c.pedal -= (-e * out.vertical + a * out.yaw) / det;
    }
    out = held_balance(ac, x, t, c, env);
    return std::abs(out.vertical) < tol_f && std::abs(out.yaw) < tol_m;
}

}  // namespace eyas::test
