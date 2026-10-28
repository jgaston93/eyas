#include "sim/build.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "airframe/airframe_aero.hpp"
#include "airframe/landing_gear.hpp"
#include "control/genhel_fcs.hpp"
#include "core/spinning_parts.hpp"
#include "engine/ballin_t700.hpp"
#include "engine/genhel_t700.hpp"
#include "rotor/bailey_rotor.hpp"
#include "rotor/blade_element_rotor.hpp"
#include "rotor/ideal_thrust.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

using units::Dimension;

constexpr std::int64_t kNsPerSecond = 1'000'000'000;

// Physics step from `simulation.physics_rate` [Hz]: the rate must be a whole
// number that divides 1 s into whole nanoseconds (spec §6.10).
std::int64_t read_step_ns(const Config& sim) {
    const double rate = sim.quantity("physics_rate", Dimension::frequency);
    if (!(rate >= 1.0) || rate > 1e9 || rate != std::floor(rate)) {
        sim.fail("physics_rate", "must be a whole number of Hz, got " + std::to_string(rate));
    }
    const auto r = static_cast<std::int64_t>(rate);
    if (kNsPerSecond % r != 0) {
        sim.fail("physics_rate",
                 std::to_string(r) + " Hz does not divide 1 s into whole nanoseconds");
    }
    return kNsPerSecond / r;
}

std::vector<std::unique_ptr<ForceModel>> read_force_models(const Config& config) {
    std::vector<std::unique_ptr<ForceModel>> forces;
    if (!config.contains("force_models")) {
        return forces;
    }
    const Config list = config.child("force_models");
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Config f = list[i];
        const std::string type = f.string("type");
        if (type == "ideal_thrust") {
            f.allow_keys({"type", "max_thrust"});
            const double max_thrust = f.quantity("max_thrust", Dimension::force);
            forces.push_back(f.in_context(
                "max_thrust", [&] { return std::make_unique<IdealThrust>(max_thrust); }));
        } else if (type == "spinning_parts") {
            f.allow_keys({"type", "parts"});
            const Config parts = f.child("parts");
            Vec3 h = Vec3::Zero();
            for (std::size_t k = 0; k < parts.size(); ++k) {
                const Config p = parts[k];
                p.allow_keys({"axis", "inertia", "speed"});
                const std::string axis = p.string("axis");
                int a = -1;
                if (axis == "x") {
                    a = 0;
                } else if (axis == "y") {
                    a = 1;
                } else if (axis == "z") {
                    a = 2;
                }
                if (a < 0) {
                    p.fail("axis", "must be x, y or z (body axes), got '" + axis + "'");
                }
                h[a] += p.quantity("inertia", Dimension::inertia) *
                        p.quantity("speed", Dimension::angular_rate);
            }
            forces.push_back(
                f.in_context("parts", [&] { return std::make_unique<SpinningParts>(h); }));
        } else {
            f.fail("type", "unknown force model '" + type + "'");
        }
    }
    return forces;
}

// A Bailey rotor with torque on a shaft (D-058) loads that shaft, so every
// engine on it must count its demand: Ballin's names it as its tail_rotor;
// GenHel's can't, and already takes the tail rotor's power in its loss factor
// K_Q (CR-166309 p. 5.6-20), so the torque there would count it twice.
void check_tail_rotor_torque(const Config& list,
                             const std::vector<std::unique_ptr<DynamicComponent>>& components) {
    for (const auto& rotor : components) {
        const auto* tail = dynamic_cast<const BaileyRotor*>(rotor.get());
        if (tail == nullptr) {
            continue;
        }
        const BaileyRotorConfig& tc = tail->config();
        if (!tc.torque || !tc.shaft) {
            continue;
        }
        const std::string& shaft = tc.shaft->name;
        for (std::size_t i = 0; i < components.size(); ++i) {
            if (const auto* e = dynamic_cast<const BallinT700*>(components[i].get());
                e != nullptr && e->config().shaft == shaft && e->config().tail_rotor != tc.name) {
                list[i].fail("tail_rotor", "the Bailey rotor '" + tc.name +
                                               "' has torque on shaft '" + shaft +
                                               "': name it here (D-058)");
            }
            if (const auto* e = dynamic_cast<const GenHelT700*>(components[i].get());
                e != nullptr && e->config().shaft == shaft) {
                list[i].fail("shaft", "the Bailey rotor '" + tc.name + "' has torque on shaft '" +
                                          shaft +
                                          "', which GenHel's engine already counts in K_Q "
                                          "(D-058)");
            }
        }
    }
}

// Components with states or acceleration coupling, in config order. The
// executive evaluates them producers first (D-003).
std::vector<std::unique_ptr<DynamicComponent>> read_components(const Config& config) {
    std::vector<std::unique_ptr<DynamicComponent>> components;
    if (!config.contains("components")) {
        return components;
    }
    const Config list = config.child("components");
    int engines = 0;  // the engines so far, each taking a Controls::engines entry (D-055)
    const auto take_engines = [&](const Config& c, int n) {
        if (engines + n > static_cast<int>(kMaxEngines)) {
            c.fail("engines", "more than " + std::to_string(kMaxEngines) +
                                  " engines in the aircraft (Controls::engines, D-055)");
        }
        engines += n;
    };
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Config c = list[i];
        const std::string type = c.string("type");
        if (type == "airframe_aero") {
            components.push_back(AirframeAero::from_config(c));
        } else if (type == "bailey_rotor") {
            components.push_back(BaileyRotor::from_config(c));
        } else if (type == "blade_element_rotor") {
            components.push_back(BladeElementRotor::from_config(c));
        } else if (type == "landing_gear") {
            components.push_back(LandingGear::from_config(c));
        } else if (type == "genhel_fcs") {
            components.push_back(GenHelFcs::from_config(c));
        } else if (type == "genhel_t700") {
            auto e = GenHelT700::from_config(c, engines);
            take_engines(c, e->config().engines);
            components.push_back(std::move(e));
        } else if (type == "ballin_t700") {
            const int first = engines;
            auto e = BallinT700::from_config(c, first);
            take_engines(c, e->config().engines);
            components.push_back(std::move(e));
        } else {
            c.fail("type", "unknown component '" + type + "'");
        }
    }
    check_tail_rotor_torque(list, components);
    return components;
}

// The rotor-speed degrees of freedom (D-052), in config order.
std::vector<ShaftConfig> read_shafts(const Config& config) {
    std::vector<ShaftConfig> shafts;
    if (!config.contains("shafts")) {
        return shafts;
    }
    const Config list = config.child("shafts");
    for (std::size_t i = 0; i < list.size(); ++i) {
        const Config s = list[i];
        s.allow_keys({"name", "speed", "parts", "drive"});
        ShaftConfig sh;
        sh.name = s.string("name");
        sh.speed = s.quantity("speed", Dimension::angular_rate);
        if (!(sh.speed > 0.0) || !std::isfinite(sh.speed)) {
            s.fail("speed", "must be positive and finite");
        }
        // Each part: its own inertia about its axis and its speed at the
        // shaft's governed speed (D-053), as the spinning parts (D-048); the
        // executive takes gear = speed / shaft speed, J = inertia × gear². A
        // part given by its inertia alone is that inertia referred to the
        // shaft, with no angular momentum (D-053 amendment 4).
        if (s.contains("parts")) {
            const Config parts = s.child("parts");
            for (std::size_t k = 0; k < parts.size(); ++k) {
                const Config p = parts[k];
                p.allow_keys({"inertia", "speed", "axis"});
                if (!p.contains("speed") && !p.contains("axis")) {
                    ShaftPart part;
                    part.inertia = p.quantity("inertia", Dimension::inertia);
                    part.axis_b = Vec3::Zero();
                    sh.parts.push_back(part);
                    continue;
                }
                const Config a = p.child("axis");
                a.allow_keys({"x", "y", "z"});
                const Vec3 v(a.number("x"), a.number("y"), a.number("z"));
                if (!(v.norm() > 0.0) || !v.allFinite()) {
                    p.fail("axis", "must be a nonzero, finite direction");
                }
                const double speed = p.quantity("speed", Dimension::angular_rate);
                if (!(speed != 0.0) || !std::isfinite(speed)) {
                    p.fail("speed", "must be nonzero and finite");
                }
                ShaftPart part;
                part.gear = speed / sh.speed;
                part.inertia = p.quantity("inertia", Dimension::inertia) * part.gear * part.gear;
                part.axis_b = v.normalized();
                sh.parts.push_back(part);
            }
        }
        // Required, no default (D-021, D-052).
        const std::string drive = s.string("drive");
        if (drive == "ideal") {
            sh.drive = ShaftDrive::ideal;
        } else if (drive == "free") {
            sh.drive = ShaftDrive::free;
        } else {
            s.fail("drive", "must be ideal or free");
        }
        shafts.push_back(sh);
    }
    return shafts;
}

}  // namespace

FlatEarthMap read_origin(const Config& config) {
    const Config o = config.child("geodetic_origin");
    o.allow_keys({"latitude", "longitude", "altitude"});
    const GeodeticPosition origin{o.quantity("latitude", Dimension::angle),
                                  o.quantity("longitude", Dimension::angle),
                                  o.quantity("altitude", Dimension::length)};
    return config.in_context("geodetic_origin", [&] { return FlatEarthMap(origin); });
}

Executive build_executive(const Config& config, double origin_altitude) {
    ExecutiveConfig ec;
    const Config af = config.child("airframe");
    af.allow_keys({"mass", "inertia"});
    const Config in = af.child("inertia");
    in.allow_keys({"ixx", "iyy", "izz", "ixz"});
    ec.mass = make_mass_properties(
        af.quantity("mass", Dimension::mass), in.quantity("ixx", Dimension::inertia),
        in.quantity("iyy", Dimension::inertia), in.quantity("izz", Dimension::inertia),
        in.quantity("ixz", Dimension::inertia));
    ec.origin_altitude = origin_altitude;
    if (config.contains("simulation")) {
        const Config sim = config.child("simulation");
        sim.allow_keys({"physics_rate", "max_substeps"});
        if (sim.contains("physics_rate")) {
            ec.step_ns = read_step_ns(sim);
        }
        if (sim.contains("max_substeps")) {
            ec.max_substeps = sim.integer("max_substeps");
            if (ec.max_substeps <= 0) {
                sim.fail("max_substeps", "must be positive");
            }
        }
    }
    ec.shafts = read_shafts(config);
    auto components = read_components(config);
    auto forces = read_force_models(config);
    // Step, substeps and origin are checked above, and the components and
    // force models by their readers, so what the executive can still reject
    // is the airframe's mass properties, the shafts and their bindings.
    return config.in_context(
        "airframe", [&] { return Executive(ec, std::move(components), std::move(forces)); });
}

}  // namespace eyas
