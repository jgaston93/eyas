#pragma once

#include "env/geodesy.hpp"
#include "sim/executive.hpp"
#include "util/config.hpp"

// Building an aircraft from its config (D-012, D-021): the readers behind
// Model::from_file, shared with the tests and the analysis tools so they
// evaluate exactly the aircraft the Model flies. Init-time; throw InitError.
namespace eyas {

// The NED origin from `geodetic_origin` (D-017).
FlatEarthMap read_origin(const Config& config);

// The executive for the whole config: airframe mass properties, simulation
// settings, components and force models.
Executive build_executive(const Config& config, double origin_altitude);

}  // namespace eyas
