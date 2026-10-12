#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

// Per-evaluation data path between components (D-003). A component declares
// the scalar outputs it produces and the ones it consumes, by name
// "<instance>.<field>", e.g. "main_rotor.lambda0". At init the executive
// gives every produced name a slot in one pre-allocated store, orders the
// components so producers run before consumers, and hands each component the
// slots it needs. In every derivative evaluation producers write their slots
// and consumers read them; nothing is kept between evaluations, so the
// derivative function stays pure (spec §5).
namespace eyas {

// Index of one scalar in the store, resolved at init.
struct CouplingSlot {
    std::size_t index = std::numeric_limits<std::size_t>::max();
    bool valid() const noexcept { return index != std::numeric_limits<std::size_t>::max(); }
};

// The values of one derivative evaluation. Called from advance(): no
// allocation, no exceptions.
class CouplingOutputs {
public:
    CouplingOutputs() = default;
    explicit CouplingOutputs(std::size_t size) : values_(size, 0.0) {}

    // Debug builds set every value to NaN at the start of an evaluation, so
    // a read before the producer's write asserts (D-003). Release: no-op.
    void begin_evaluation() noexcept;

    double get(CouplingSlot s) const noexcept;
    void set(CouplingSlot s, double v) noexcept;

    std::size_t size() const noexcept { return values_.size(); }

private:
    std::vector<double> values_;
};

// Name → slot table, built by the executive at init and passed to each
// component's bind(). Init-time only.
class CouplingRegistry {
public:
    // Adds a produced name. Throws InitError if it is already produced.
    CouplingSlot add(const std::string& name, std::string_view producer);
    // The slot of a produced name. Throws InitError, naming `consumer`, if no
    // component produces it.
    CouplingSlot find(const std::string& name, std::string_view consumer) const;

    std::size_t size() const noexcept { return names_.size(); }

private:
    std::vector<std::string> names_;
    std::vector<std::string> producers_;
};

}  // namespace eyas
