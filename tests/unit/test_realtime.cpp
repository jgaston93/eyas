// M8a acceptance (spec §9 M8, D-042): host-rate decoupling on the UH-60A
// through Model. A host calling at 60 Hz, or with a jittered frame time, gets
// a trajectory bit-identical to calling at the 1 kHz physics rate with the
// same control on each physics sub-step; outputs included. advance() and
// outputs() allocate nothing in flight (spec §7 rule 1).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/trim.hpp"
#include "eyas/model.hpp"
#include "support/alloc_counter.hpp"
#include "support/bit_identical.hpp"
#include "support/m6d_fixtures.hpp"
#include "support/m8a_long_runs.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

using test::bit_identical;
using test::physical_outputs;
constexpr std::int64_t kStepNs = 1'000'000;  // the UH-60A config's physics step

// A 60 kt trim, flown with a longitudinal doublet: the control of host
// frame f, whose sub-steps start at sim time t0 [s].
struct Flight {
    Model model = Model::from_file(test::uh60a_config_path());
    TrimResult trim;
    Flight() {
        TrimCondition c;
        c.airspeed = 60.0 * units::kKnot;
        trim = model.trim(c);
    }
    Controls control(double t0) const {
        Controls c = trim.controls;
        if (t0 >= 1.0 && t0 < 1.5) {
            c.lon_cyclic += 0.03;
        } else if (t0 >= 1.5 && t0 < 2.0) {
            c.lon_cyclic -= 0.03;
        }
        return c;
    }
};

// Flies `a` with host frames `dts` [ns], and `b` at 1 kHz with each frame's
// control held over the sub-steps `a` ran in it. After every frame the state
// and the physical outputs must be bit-identical.
void expect_decoupled(Flight& a, Flight& b, const std::vector<std::int64_t>& dts) {
    ASSERT_TRUE(a.trim.converged);
    ASSERT_TRUE(b.trim.converged);
    const Environment env;  // the trim's: no ground, ISA, no wind
    std::int64_t total_ns = 0;
    std::int64_t total_steps = 0;
    for (std::size_t f = 0; f < dts.size(); ++f) {
        const Controls c = a.control(a.model.outputs().sim_time);
        const AdvanceStatus s = a.model.advance(c, env, dts[f]);
        ASSERT_FALSE(s.capped || s.non_finite || s.rejected_dt || s.rejected_env);
        for (std::int64_t k = 0; k < s.substeps; ++k) {
            ASSERT_EQ(b.model.advance(c, env, kStepNs).substeps, 1);
        }
        total_ns += dts[f];
        total_steps += s.substeps;
        ASSERT_EQ(total_steps, total_ns / kStepNs);  // D-006: floor(Σ dt / step)
        ASSERT_TRUE(bit_identical(a.model.state(), b.model.state())) << "frame " << f;
        ASSERT_TRUE(
            bit_identical(physical_outputs(a.model.outputs()), physical_outputs(b.model.outputs())))
            << "frame " << f;
    }
}

// Spec §9 M8: 60 Hz host frames against 1 kHz calls, 5 s with a doublet.
TEST(RealTime, HostRateDecoupling) {
    Flight a;
    Flight b;
    expect_decoupled(a, b, std::vector<std::int64_t>(300, 16'666'667));
    EXPECT_GT(a.model.outputs().sim_time, 4.99);
}

// The same for a jittered frame time with the same total: 1 to 40 ms frames,
// not whole steps, drawn from a fixed seed.
TEST(RealTime, JitteredHostRate) {
    std::mt19937_64 rng(20261017);
    std::uniform_int_distribution<std::int64_t> dist(1'000'000, 40'000'000);
    std::vector<std::int64_t> dts;
    std::int64_t total = 0;
    while (total < 5'000'000'000) {
        dts.push_back(dist(rng));
        total += dts.back();
    }
    Flight a;
    Flight b;
    expect_decoupled(a, b, dts);
}

// advance() and outputs(), including the Tier 2–4 evaluation, allocate
// nothing in flight (on the ground: Uh60aAircraft.RealTimeRules).
TEST(RealTime, ZeroAllocationInFlight) {
    Flight a;
    ASSERT_TRUE(a.trim.converged);
    const Environment env;
    (void)a.model.advance(a.trim.controls, env, 16'666'667);  // warm the table hints
    double sink = 0.0;
    {
        const test::AllocationCounter count;
        for (int f = 0; f < 120; ++f) {
            (void)a.model.advance(a.control(a.model.outputs().sim_time), env, 16'666'667);
            sink += a.model.outputs().calibrated_airspeed + a.model.outputs().main_rotor_thrust;
        }
        EXPECT_EQ(count.count(), 0U);
    }
    EXPECT_TRUE(std::isfinite(sink));
}

// ---- D-042: the long runs, 60 s gates (30 min: tests/validation/m8a_cases) ----

// (b) The held periodic orbit repeats: every component state comes back to
// its trim start within the trim's own periodicity tolerance (1e-8 in each
// state's units, TrimOptions::periodic_tol) for a minute of revolutions,
// and over the last 50 revolutions the deviation no longer moves (no
// secular trend: a stable orbit settles at residual / (1 − contraction)).
TEST(RealTime, HeldOrbitRepeatsFor60s) {
    for (const double kt : {0.0, 120.0}) {
        SCOPED_TRACE(kt);
        const double T = test::uh60a_period();
        const test::HeldOrbitDrift d = test::held_orbit_drift(kt, static_cast<int>(60.0 / T));
        ASSERT_TRUE(d.converged);
        EXPECT_EQ(d.unconverged, 0U);
        const double worst = *std::max_element(d.deviation.begin(), d.deviation.end());
        EXPECT_LT(worst, TrimOptions{}.periodic_tol);
        const auto n = d.deviation.size();
        EXPECT_LT(std::abs(d.deviation[n - 1] - d.deviation[n - 51]), 1e-12);
    }
}

// (d) The free UH-60A parked on its gear, brakes set: it lands, settles and
// stays. From 40 s (settled; VALIDATION.md M8a) to 60 s the CG moves less
// than 1 mm and the attitude less than 1e-4 rad; the residual motion is the
// rotor's 4/rev vibration. Every state stays finite, sim time is exact, and
// the quaternion norm error before renormalization stays below 1e-12 (M1).
TEST(RealTime, ParkedAircraftSettlesFor60s) {
    const std::vector<test::ParkedSample> p = test::parked_run(60);
    ASSERT_EQ(p.size(), 60U);
    const test::ParkedSample& a = p[39];
    const test::ParkedSample& b = p[59];
    EXPECT_EQ(b.t_ns, 60'000'000'000);
    for (const test::ParkedSample& s : p) {
        EXPECT_TRUE(s.finite);
        EXPECT_LT(s.quat_norm_error, 1e-12);
    }
    EXPECT_TRUE(b.weight_on_wheels);
    EXPECT_LT((b.pos_n - a.pos_n).norm(), 1e-3);
    EXPECT_LT(std::abs(b.roll - a.roll), 1e-4);
    EXPECT_LT(std::abs(b.pitch - a.pitch), 1e-4);
    EXPECT_LT(std::abs(b.heading - a.heading), 1e-4);
    EXPECT_LT(b.speed, 1e-3);
}

}  // namespace
}  // namespace eyas
