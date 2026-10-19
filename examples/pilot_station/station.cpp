#include "station.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "common/frames.hpp"

namespace station {

namespace {

constexpr double kKnot = 1852.0 / 3600.0;  // [m/s]

eyas::Model build(const StationOptions& o) {
    return o.origin ? eyas::Model::from_file(o.config, *o.origin)
                    : eyas::Model::from_file(o.config);
}

}  // namespace

Station::Station(const StationOptions& options)
    : model_(build(options)), frame_ns_(examples::nominal_frame_ns(std::max(options.rate, 1))) {
    if (options.rate < 1) {
        throw std::runtime_error("the host rate must be positive");
    }
    if (!(options.hover_height > 0.0)) {
        throw std::runtime_error("--hover-height must be positive");
    }
    // A new model sits at the NED origin, so its pose is the origin.
    const eyas::AircraftState& o = model_.outputs();
    origin_ = {o.latitude, o.longitude, o.altitude};
    // A standard day, no wind, and level ground through the origin (D-032
    // item 9), where the pad is: what eyas_drive flies with --ground.
    env_.ground_height = origin_.altitude;

    // Airborne starts: cold trims, as eyas_drive makes them, so a recorded
    // run replays bit for bit.
    for (const double kt : {0.0, 60.0, 120.0}) {
        eyas::TrimCondition c;
        c.airspeed = kt * kKnot;
        c.altitude = origin_.altitude + options.hover_height;
        c.afcs = options.afcs;
        const eyas::TrimResult r = model_.trim(c);
        if (!r.converged) {
            throw std::runtime_error("trim at " + std::to_string(static_cast<int>(kt)) +
                                     " kt did not converge");
        }
        StartPoint p;
        p.name = kt == 0.0 ? "hover" : std::to_string(static_cast<int>(kt));
        p.snap = model_.make_snapshot();
        // The trim knows no ground (its outputs have none): one frame over
        // the real ground shows whether the gear clears it at this height.
        (void)model_.advance(r.controls, env_, frame_ns_);
        if (model_.outputs().weight_on_wheels || model_.outputs().altitude < env_.ground_height) {
            throw std::runtime_error("--hover-height " + std::to_string(options.hover_height) +
                                     " m puts the gear on the ground at the trimmed attitude");
        }
        p.controls = r.controls;
        p.replayable = true;
        p.trim_speed_kt = kt;
        p.altitude = c.altitude;
        starts_.push_back(std::move(p));
    }

    // The pad: as M8a's parked case (D-042 (d)), dropped from 2.2 m onto the
    // gear, brakes set, collective 0.2, and flown for the 40 s that case takes
    // to settle (|v| < 1e-3 m/s from then on). The cyclic and pedal are the
    // hover trim's, not the parked case's half right pedal: they are the
    // offsets the pilot's centred stick and twist fly from, and a lift-off
    // needs about the hover's (with half right pedal it spun at once, found
    // flying the X-56).
    eyas::Controls parked = starts_.front().controls;  // the hover trim
    parked.collective = 0.2;
    parked.brake_left = 1.0;
    parked.brake_right = 1.0;
    eyas::BodyState drop;
    drop.pos_n = eyas::Vec3(0.0, 0.0, -2.2);
    if (!model_.reset(drop)) {
        throw std::runtime_error("cannot place the aircraft on the pad");
    }
    for (int i = 0; i < 40 * options.rate; ++i) {
        const eyas::AdvanceStatus s = model_.advance(parked, env_, frame_ns_);
        if (s.non_finite || s.capped || s.rejected_dt || s.rejected_env) {
            throw std::runtime_error("the aircraft did not settle on the pad");
        }
    }
    StartPoint pad;
    pad.name = "pad";
    pad.snap = model_.make_snapshot();
    pad.controls = parked;
    pad.altitude = origin_.altitude;
    starts_.insert(starts_.begin(), std::move(pad));
    // A restore brings back the snapshot's sim time, so the pad starts at
    // 40 s; it doesn't replay in eyas_drive anyway.
    (void)start(0);
}

bool Station::start(std::size_t index) noexcept {
    if (index >= starts_.size() || !model_.restore(starts_[index].snap)) {
        return false;
    }
    current_ = index;
    start_time_ = model_.outputs().sim_time;
    worst_ns_ = 0;
    last_ns_ = 0;
    frames_ = 0;
    faulted_ = false;
    crashed_ = false;
    return true;
}

eyas::AdvanceStatus Station::frame(const eyas::Controls& controls) noexcept {
    if (crashed_) {
        return {};
    }
    using Clock = std::chrono::steady_clock;
    const auto t0 = Clock::now();
    const eyas::AdvanceStatus s = model_.advance(controls, env_, frame_ns_);
    last_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
    worst_ns_ = std::max(worst_ns_, last_ns_);
    ++frames_;
    faulted_ = faulted_ || s.non_finite || s.capped || s.rejected_dt || s.rejected_env;
    const eyas::AircraftState& o = model_.outputs();
    constexpr double kTilt = 30.0 * std::numbers::pi / 180.0;
    const bool tipped =
        o.weight_on_wheels && (std::abs(o.roll) > kTilt || std::abs(o.pitch) > kTilt);
    const bool overloaded = !(std::abs(o.load_factor) <= 10.0);  // NaN counts
    const bool buried = o.altitude < env_.ground_height;
    crashed_ = faulted_ || tipped || overloaded || buried;
    return s;
}

}  // namespace station
