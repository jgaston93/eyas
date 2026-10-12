#include "core/coupling.hpp"

#include <algorithm>
#include <cassert>

#include "util/error.hpp"

namespace eyas {

void CouplingOutputs::begin_evaluation() noexcept {
#ifndef NDEBUG
    std::fill(written_.begin(), written_.end(), 0);
#endif
}

double CouplingOutputs::get(CouplingSlot s) const noexcept {
    assert(s.index < values_.size());
    // Read before the producer wrote it this evaluation (D-003).
    assert(written_[s.index] != 0);
    return values_[s.index];
}

void CouplingOutputs::set(CouplingSlot s, double v) noexcept {
    assert(s.index < values_.size());
    values_[s.index] = v;
#ifndef NDEBUG
    written_[s.index] = 1;
#endif
}

CouplingSlot CouplingRegistry::add(const std::string& name, std::string_view producer) {
    const auto it = std::find(names_.begin(), names_.end(), name);
    if (it != names_.end()) {
        const auto i = static_cast<std::size_t>(it - names_.begin());
        throw InitError("coupling output '" + name + "' is produced by both '" + producers_[i] +
                        "' and '" + std::string(producer) + "'");
    }
    names_.push_back(name);
    producers_.emplace_back(producer);
    return CouplingSlot{names_.size() - 1};
}

CouplingSlot CouplingRegistry::find(const std::string& name, std::string_view consumer) const {
    const auto it = std::find(names_.begin(), names_.end(), name);
    if (it == names_.end()) {
        throw InitError("component '" + std::string(consumer) + "' consumes '" + name +
                        "', which no component produces");
    }
    return CouplingSlot{static_cast<std::size_t>(it - names_.begin())};
}

}  // namespace eyas
