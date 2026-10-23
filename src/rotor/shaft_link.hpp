#pragma once

#include <span>
#include <string>

#include "core/component.hpp"

namespace eyas {
class Config;
}

// A rotor's link to a shaft (D-052): the shaft it turns on and its speed over
// the shaft's. Shared by the rotor models. Init-time only.
namespace eyas {

struct ShaftLink {
    std::string name;
    double gear = 1.0;  // rotor speed / shaft speed [-], positive
};

// Reads a rotor's `shaft: {name, gear}` node (gear optional, 1 by default).
// Fails the node if the name is empty or the gear isn't positive and finite.
ShaftLink read_shaft_link(const Config& node);

// The index of link's shaft in `shafts`. Throws InitError, naming `who`, if
// there is no such shaft or if `rotor_speed` [rad/s] isn't gear × its
// governed speed.
int bind_shaft_link(const ShaftLink& link, std::span<const ShaftInfo> shafts, double rotor_speed,
                    const std::string& who);

}  // namespace eyas
