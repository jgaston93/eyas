#pragma once

#include <iosfwd>

#include "eyas/aircraft_state.hpp"
#include "eyas/types.hpp"

// The time-history CSV that eyas_drive and the pilot station write: the
// controls applied and every AircraftState field, SI units, frames as
// documented in include/eyas/aircraft_state.hpp, then the flight control
// system's switches ('+'-separated, common/afcs.hpp; M8c3), then the engine
// controls (each engine's power lever, the rotor speed reference; D-055;
// each engine's fuel shutoff, 1 open; D-056).
// Set the stream's precision (eyas_drive uses 17) before writing.
namespace examples {

void write_state_header(std::ostream& out);
void write_state_row(std::ostream& out, const eyas::Controls& c, const eyas::AircraftState& o);

}  // namespace examples
