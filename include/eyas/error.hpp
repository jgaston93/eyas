#pragma once

#include <stdexcept>

namespace eyas {

// Error raised by init-time code: config and table loading, setup validation.
// Never thrown from anything reachable from advance() (spec §7 rule 2).
class InitError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

}  // namespace eyas
