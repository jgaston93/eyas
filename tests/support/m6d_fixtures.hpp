#pragma once

// Fixtures for the M6d whole-aircraft tests (D-028, D-034): the UH-60A
// config in data/uh60a/uh60a.yaml (built by tools/build_uh60a_config.py from
// GenHel CR-166309), and a test-side harness that evaluates its components
// one by one, so a test can read each component's loads. The production
// executive solves the coupled accelerations and doesn't keep per-component
// loads; the harness evaluates the same components, from the same config, in
// a producer-first order (D-003) with the body held (a = 0), which is where
// the D-028 whole-aircraft checks are stated.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "airframe/landing_gear.hpp"
#include "core/component.hpp"
#include "core/coupling.hpp"
#include "core/rigid_body.hpp"
#include "env/atmosphere.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
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

// The UH-60A components, built from the config, evaluated outside the
// executive.
class Uh60aHarness {
public:
    explicit Uh60aHarness(const std::string& path = uh60a_config_path())
        : cfg_(Config::from_file(path)) {
        const Config o = cfg_.child("geodetic_origin");
        origin_altitude_ = o.quantity("altitude", units::Dimension::length);
        const Config af = cfg_.child("airframe");
        const Config in = af.child("inertia");
        using units::Dimension;
        mass_ = make_mass_properties(
            af.quantity("mass", Dimension::mass), in.quantity("ixx", Dimension::inertia),
            in.quantity("iyy", Dimension::inertia), in.quantity("izz", Dimension::inertia),
            in.quantity("ixz", Dimension::inertia));
        const Config list = cfg_.child("components");
        std::size_t offset = body_layout::kSize;
        for (std::size_t i = 0; i < list.size(); ++i) {
            const Config c = list[i];
            const std::string type = c.string("type");
            if (type == "blade_element_rotor") {
                comps_.push_back(BladeElementRotor::from_config(c));
            } else if (type == "bailey_rotor") {
                comps_.push_back(BaileyRotor::from_config(c));
            } else if (type == "airframe_aero") {
                comps_.push_back(AirframeAero::from_config(c));
            } else if (type == "landing_gear") {
                comps_.push_back(LandingGear::from_config(c));
            } else {
                throw std::runtime_error("unknown component " + type);
            }
            offsets_.push_back(offset);
            offset += comps_.back()->num_states();
        }
        num_states_ = offset;
        CouplingRegistry registry;
        for (const auto& c : comps_) {
            for (const std::string& n : c->produces()) {
                registry.add(n, c->name());
            }
        }
        for (const auto& c : comps_) {
            c->bind(registry);
        }
        coupling_ = CouplingOutputs(registry.size());
        // Producers first: repeatedly take the first component whose inputs
        // are all produced by components already taken (D-003).
        std::vector<bool> taken(comps_.size(), false);
        std::vector<std::string> available;
        while (order_.size() < comps_.size()) {
            bool progress = false;
            for (std::size_t i = 0; i < comps_.size(); ++i) {
                if (taken[i]) {
                    continue;
                }
                bool ready = true;
                for (const std::string& n : comps_[i]->consumes()) {
                    ready = ready &&
                            std::find(available.begin(), available.end(), n) != available.end();
                }
                if (ready) {
                    taken[i] = true;
                    order_.push_back(i);
                    for (const std::string& n : comps_[i]->produces()) {
                        available.push_back(n);
                    }
                    progress = true;
                    break;
                }
            }
            if (!progress) {
                throw std::runtime_error("coupling cycle");
            }
        }
    }

    const Config& config() const { return cfg_; }
    const MassProperties& airframe_mass() const { return mass_; }
    std::size_t num_states() const { return num_states_; }
    std::size_t size() const { return comps_.size(); }
    std::size_t offset(std::size_t i) const { return offsets_[i]; }
    const DynamicComponent& component(std::size_t i) const { return *comps_[i]; }
    std::size_t index(std::string_view name) const {
        for (std::size_t i = 0; i < comps_.size(); ++i) {
            if (comps_[i]->name() == name) {
                return i;
            }
        }
        throw std::runtime_error("no component " + std::string(name));
    }
    template <class T>
    const T& get(std::string_view name) const {
        return dynamic_cast<const T&>(*comps_[index(name)]);
    }
    std::span<const double> states(std::span<const double> x, std::size_t i) const {
        return x.subspan(offsets_[i], comps_[i]->num_states());
    }

    // Initial state: body at `body`, every component at its initial state.
    std::vector<double> initial(const BodyState& body) const {
        std::vector<double> x(num_states_, 0.0);
        pack_body(body, x);
        for (std::size_t i = 0; i < comps_.size(); ++i) {
            comps_[i]->initial_states(
                std::span<double>(x).subspan(offsets_[i], comps_[i]->num_states()));
        }
        return x;
    }

    SimContext context(double t, std::span<const double> x, const Controls& controls,
                       const Environment& env) const {
        SimContext ctx;
        ctx.t = t;
        ctx.body = unpack_body(x);
        ctx.body.q_nb.normalize();
        ctx.controls = controls;
        ctx.env = env;
        ctx.air = isa_atmosphere(origin_altitude_ - ctx.body.pos_n.z(), env.isa_delta_T);
        ctx.ground_z_n = origin_altitude_ - env.ground_height;
        ctx.coupling = &coupling_;
        return ctx;
    }

    // Every component's load0 (body axes, about the airframe CG) and xdot0 at
    // (t, x): the loads with the body acceleration zero. xdot0 is written
    // into xdot (component states only; the body part is left as is).
    std::vector<Wrench> evaluate(double t, std::span<const double> x, const Controls& controls,
                                 const Environment& env, std::span<double> xdot = {}) const {
        const SimContext ctx = context(t, x, controls, env);
        coupling_.begin_evaluation();
        std::vector<Wrench> loads(comps_.size());
        for (const std::size_t i : order_) {
            const std::size_t n = comps_[i]->num_states();
            std::vector<double> xd(n, 0.0);
            std::vector<double> G(n * 6, 0.0);
            ComponentDerivs d{xd, G, Wrench{}, Mat66::Zero(), true, &coupling_};
            comps_[i]->derivatives(ctx, x.subspan(offsets_[i], n), d);
            loads[i] = d.load0_b;
            if (!xdot.empty()) {
                std::copy(xd.begin(), xd.end(),
                          xdot.begin() + static_cast<std::ptrdiff_t>(offsets_[i]));
            }
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
        std::vector<Wrench> avg(comps_.size());
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
        double m = mass_.mass + mr.num_blades * mr.flap->mass;
        for (const auto& g : get<LandingGear>("gear").config().gears) {
            m += g.unsprung_mass;
        }
        return m * units::kStandardGravity;
    }

private:
    Config cfg_;
    double origin_altitude_ = 0.0;
    MassProperties mass_;
    std::vector<std::unique_ptr<DynamicComponent>> comps_;
    std::vector<std::size_t> offsets_;
    std::vector<std::size_t> order_;
    std::size_t num_states_ = 0;
    mutable CouplingOutputs coupling_;
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
