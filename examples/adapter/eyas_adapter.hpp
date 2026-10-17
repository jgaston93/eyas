#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "eyas/model.hpp"
#include "iflight_model.hpp"

// The Eyas side of the host's flight-model interface (spec §6.10). It links
// libeyas and calls advance() on the host's thread, synchronously (Eyas is
// single-threaded; spec §6.10 "Threading"). All it does is translate: units,
// sign conventions and field names, plus mapping AdvanceStatus to the host's
// health flag.
class EyasAdapter final : public host::IFlightModel {
public:
    bool initialize(const std::string& aircraft, const host::InitialCondition& ic,
                    std::string& error) override;
    host::PilotInputs trim_inputs() const override { return trim_; }
    void update(const host::PilotInputs& in, std::int64_t dt_ns) override;
    host::Pose pose() const override;
    host::AirData air_data() const override;
    host::RotorData rotor() const override;
    bool healthy() const override { return healthy_; }

private:
    std::optional<eyas::Model> model_;  // empty until initialize() succeeds
    eyas::Environment env_;             // held constant here; a host would set wind etc.
    host::PilotInputs trim_;            // the trimmed controls, in host conventions
    bool healthy_ = false;
};
