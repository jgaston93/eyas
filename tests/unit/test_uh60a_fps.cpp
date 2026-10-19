// M8c3 (D-045): the flying UH-60A with GenHel's FPS, against the FPS's jobs
// (GenHel CR-166309 p. 5.5-3): it holds the pitch and roll attitude, the
// airspeed above 30 kt and the heading, and with the turn switch on
// coordinates turns. No FPS-on flight data is in the reference library, so
// each test flies the nonlinear model hands-off (or with a simple pilot) and
// compares the FPS with the SAS and PBA alone, and the yaw integrator's two
// wirings with each other (DATA_GAPS.md, "UH-60A AFCS"). Gates are set from
// the runs with a margin, each stated beside its numbers.

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "analysis/linearize.hpp"
#include "support/m8c3_fps.hpp"
#include "util/error.hpp"

namespace eyas {
namespace {

namespace m8c3 = test::m8c3;
using units::kDegree;

std::size_t fcs_index(const Executive& ex) {
    for (std::size_t i = 0; i < ex.num_components(); ++i) {
        if (dynamic_cast<const GenHelFcs*>(&ex.component(i)) != nullptr) {
            return i;
        }
    }
    throw std::runtime_error("no genhel_fcs component");
}

// In trim the FPS's references hold the trimmed attitude, heading and
// airspeed, so it outputs nothing and the trims with it on equal those
// without, bit for bit (GenHel's IC mode, p. 5.5-2). With the turn switch on
// the lateral acceleration's proportional path (KLAYYO) is in, moving the
// pedal alone by its trim actuator's inches; its integral (KIAYYO) is held
// and reported as the trim's held drift rate, KIAYYO times the lagged Y.SENS.
TEST(Uh60aFps, TrimsMoveOnlyTheStick) {
    const Executive ex = m8c3::executive();
    const std::size_t off = m8c3::fcs_offset(ex) - body_layout::kSize;  // in component_states
    for (const double kt : {0.0, 60.0, 120.0}) {
        SCOPED_TRACE(kt);
        const m8c3::Trims t = m8c3::trims(ex, kt);
        EXPECT_EQ(t.fps.controls.collective, t.sas.controls.collective);
        EXPECT_EQ(t.fps.controls.lat_cyclic, t.sas.controls.lat_cyclic);
        EXPECT_EQ(t.fps.controls.lon_cyclic, t.sas.controls.lon_cyclic);
        EXPECT_EQ(t.fps.controls.pedal, t.sas.controls.pedal);
        EXPECT_EQ(t.fps.pitch, t.sas.pitch);
        EXPECT_EQ(t.fps.roll, t.sas.roll);
        EXPECT_EQ(t.fps.held_drift_rate, 0.0);
    }
    // 100 kt, turn switch on.
    const m8c3::Trims t = m8c3::trims(ex, 100.0);
    TrimCondition c = m8c3::level(100.0, m8c3::kFps);
    c.afcs.fps_turn = true;
    std::vector<double> x;
    const TrimResult on = solve_trim(ex, c, &t.sas, x);
    ASSERT_TRUE(on.converged);
    EXPECT_NEAR(on.pitch, t.sas.pitch, 1e-6);
    EXPECT_NEAR(on.roll, t.sas.roll, 1e-6);
    EXPECT_NEAR(on.controls.collective, t.sas.controls.collective, 1e-6);
    EXPECT_NEAR(on.controls.lat_cyclic, t.sas.controls.lat_cyclic, 1e-6);
    EXPECT_NEAR(on.controls.lon_cyclic, t.sas.controls.lon_cyclic, 1e-6);
    // YOTEMP = KLAYYO ay (p₁ = 0 in trim) [%], → THROL by KPCDGY [deg], → in
    // by 1/KXPTHR, which the pilot's pedal takes out.
    const double ay_ft = on.component_states[off + GenHelFcs::kFpsAccelLag] / 0.3048;
    const double inches = -2.38 * ay_ft * -0.298 / -5.539;
    const StickTravel& pedal =
        dynamic_cast<const GenHelFcs&>(ex.component(fcs_index(ex))).config().rigging.pedal;
    const double half = 0.5 * (pedal.hi - pedal.lo) / 0.0254;  // [in]
    EXPECT_NEAR((on.controls.pedal - t.sas.controls.pedal) * half, -inches, 1e-5);
    EXPECT_NEAR(on.held_drift_rate, std::abs(-0.656 * ay_ft) * 0.01, 1e-9);
    std::printf("100 kt, turn switch on: ay %.4f ft/s^2, pedal %+.5f, held drift %.3e /s\n", ay_ft,
                on.controls.pedal - t.sas.controls.pedal, on.held_drift_rate);
}

// Attitude hold: after a pitch-rate kick (0.05 rad/s) or roll-rate kick
// (0.1 rad/s), hands-off at 60 and 120 kt, the FPS brings θ and φ back to
// their trim: within 0.1° over 20–30 s (measured 0.008–0.080°, the most
// after the 60 kt pitch kick). With the SAS and PBA alone the 60 kt kicks
// leave 0.79° and 0.29° then (gate 0.2°, so the test fails without the FPS);
// at 120 kt the SAS returns them nearly as well (0.10°, 0.04°).
TEST(Uh60aFps, HoldsTheAttitude) {
    const Executive ex = m8c3::executive();
    for (const double kt : {60.0, 120.0}) {
        const m8c3::Trims t = m8c3::trims(ex, kt);
        for (const int axis : {1, 0}) {
            SCOPED_TRACE(std::to_string(kt) + (axis == 1 ? " kt pitch" : " kt roll"));
            const auto dev = [&](const std::vector<m8c3::Sample>& s, bool fps) {
                const TrimResult& r = fps ? t.fps : t.sas;
                return std::max(m8c3::largest(s, 20.0, 30.0,
                                              [&](const m8c3::Sample& q) {
                                                  return q.theta - r.pitch / kDegree;
                                              }),
                                m8c3::largest(s, 20.0, 30.0, [&](const m8c3::Sample& q) {
                                    return q.phi - r.roll / kDegree;
                                }));
            };
            const double fps = dev(m8c3::rate_kick(ex, t, kt, axis, true, 30.0), true);
            const double sas = dev(m8c3::rate_kick(ex, t, kt, axis, false, 30.0), false);
            std::printf(
                "%3.0f kt %s kick: largest attitude change over 20-30 s %.3f deg (FPS), "
                "%.3f deg (SAS)\n",
                kt, axis == 1 ? "pitch" : "roll", fps, sas);
            EXPECT_LT(fps, 0.1);
            if (kt == 60.0) {
                EXPECT_GT(sas, 0.2);
            }
        }
    }
}

// Airspeed hold above 30 kt (Fig. 5.1.5(a): KSVXPO and the integral KIVXPO):
// with the stick held 0.01 forward, the SAS-only aircraft settles 1.0 kt
// (60 kt) and 3.0 kt (100 kt) fast; with the FPS the integral brings it back,
// within 0.1 kt over 120–150 s (measured 0.01–0.04 kt).
TEST(Uh60aFps, HoldsTheAirspeedAbove30Knots) {
    const Executive ex = m8c3::executive();
    for (const double kt : {60.0, 100.0}) {
        SCOPED_TRACE(kt);
        const m8c3::Trims t = m8c3::trims(ex, kt);
        const auto dv = [&](const m8c3::Sample& q) { return q.airspeed - kt; };
        const auto fps = m8c3::stick_push(ex, t, kt, true, 150.0);
        const auto sas = m8c3::stick_push(ex, t, kt, false, 150.0);
        const double fps_dv = m8c3::largest(fps, 120.0, 150.0, dv);
        const double sas_dv = std::abs(dv(sas.back()));
        std::printf(
            "%3.0f kt, stick +0.01: airspeed change at 120-150 s %.3f kt (FPS), %.2f kt "
            "(SAS) at 150 s\n",
            kt, fps_dv, sas_dv);
        EXPECT_LT(fps_dv, 0.1);
        EXPECT_GT(sas_dv, 0.8);
    }
}

// Below 30 kt the airspeed synchronizer tracks (VG30SW), so the FPS holds the
// attitude but not the speed: at 20 kt a 5 kt gust leaves the aircraft
// 4.6–5.9 kt fast over 40 s (gate 3.5–7) with the pitch within 2°. The SAS
// alone lets the low-speed phugoid diverge: 38 kt fast and 17° nose down by
// 20 s.
TEST(Uh60aFps, DoesNotHoldTheAirspeedBelow30Knots) {
    const Executive ex = m8c3::executive();
    const m8c3::Trims t = m8c3::trims(ex, 20.0);
    const auto s = m8c3::gust(ex, t, 20.0, true, 40.0);
    for (const m8c3::Sample& q : m8c3::window(s, 5.0, 40.0)) {
        EXPECT_GT(q.airspeed - 20.0, 3.5) << q.t;
        EXPECT_LT(q.airspeed - 20.0, 7.0) << q.t;
        EXPECT_LT(std::abs(q.theta - t.fps.pitch / kDegree), 2.0) << q.t;
    }
    std::printf("20 kt, 5 kt gust (FPS): airspeed change %.2f kt at 40 s\n",
                s.back().airspeed - 20.0);
}

// Heading hold in hover (Fig. 5.1.5(c)) with a steady pedal offset: the
// physical wiring (D-045 M8c3) integrates the heading error beyond the ±1°
// deadband, so the heading settles at the deadband's edge: within 1.05° from
// 10 s to 90 s (measured 0.95°). As printed, the heading path only runs with
// the turn switch on, and the lateral acceleration's integral runs instead:
// the heading drifts 35° in 90 s (gate 10°).
TEST(Uh60aFps, HoldsTheHeadingInHoverWithThePhysicalWiring) {
    for (const char* wiring : {"physical", "as_printed"}) {
        SCOPED_TRACE(wiring);
        const Executive ex = m8c3::executive(wiring);
        const m8c3::Trims t = m8c3::trims(ex, 0.0);
        const auto s = m8c3::pedal_push(ex, t, 0.0, true, 0.05, 90.0);
        const auto psi = [](const m8c3::Sample& q) { return q.heading; };
        const double worst = m8c3::largest(s, 10.0, 90.0, psi);
        std::printf("hover, pedal +0.05 (%s): largest heading change over 10-90 s %.2f deg\n",
                    wiring, worst);
        if (std::string(wiring) == "physical") {
            EXPECT_LT(worst, 1.05);
        } else {
            EXPECT_GT(std::abs(s.back().heading), 10.0);
        }
    }
}

// Turn coordination (Fig. 5.1.5(c), turn switch on): rolled into a turn of
// about 10° of bank at 100 kt with the trim release held, then let go, the
// physical wiring integrates the lagged lateral acceleration and centres the
// ball: |Y.SENS| below 0.03 m/s² over 30–40 s (measured 0.011–0.015). As
// printed, nothing integrates it (the heading path is synchronized to zero):
// 0.11 m/s² (gate 0.06). With the turn switch off the heading hold yaws
// against the turn: 0.8 m/s² by 20 s (gate 0.3), and the aircraft departs
// after about 45 s, which the host's latch (GenHel p. 5.5-3) prevents.
TEST(Uh60aFps, CoordinatesTurnsWithTheTurnSwitch) {
    for (const char* wiring : {"physical", "as_printed"}) {
        SCOPED_TRACE(wiring);
        const Executive ex = m8c3::executive(wiring);
        const m8c3::Trims t = m8c3::trims(ex, 100.0);
        const auto on = m8c3::turn(ex, t, 100.0, true, 40.0);
        const auto ay = [](const m8c3::Sample& q) { return q.ay; };
        const double ball = m8c3::largest(on, 30.0, 40.0, ay);
        const double bank = m8c3::window(on, 30.0, 40.0).front().phi;
        std::printf(
            "100 kt turn, switch on (%s): bank %.1f deg, largest |Y.SENS| over 30-40 s "
            "%.3f m/s^2\n",
            wiring, bank, ball);
        EXPECT_GT(bank, 8.0);  // a turn, held by the roll attitude hold
        if (std::string(wiring) == "physical") {
            EXPECT_LT(ball, 0.03);
            const auto off = m8c3::turn(ex, t, 100.0, false, 20.0);
            const double away = std::abs(off.back().ay);
            std::printf("100 kt turn, switch off: |Y.SENS| %.3f m/s^2 at 20 s\n", away);
            EXPECT_GT(away, 0.3);
        } else {
            EXPECT_GT(ball, 0.06);
        }
    }
}

// The collective-to-yaw path (KSXCYO on the synchronized collective, faded
// out from 40 to 100 kt) with the heading hold: at 20 kt a 0.05 step of
// collective turns the SAS-only aircraft 3.7° within 10 s; with the FPS the
// heading stays within 1.1° (gate: half the SAS's). Not gated: the path on
// its own overcorrects in Eyas. Without it the FPS keeps the heading within
// 0.7°, and with it the first second's yaw rate reverses (up to +0.8 deg/s
// with the SAS alone, down to −0.6 with the FPS). VALIDATION.md M8c3
// reports it.
TEST(Uh60aFps, HoldsTheHeadingThroughACollectiveStep) {
    const Executive ex = m8c3::executive();
    const m8c3::Trims t = m8c3::trims(ex, 20.0);
    const auto psi = [](const m8c3::Sample& q) { return q.heading; };
    const double fps =
        m8c3::largest(m8c3::collective_step(ex, t, 20.0, true, 0.05, 10.0), 0.0, 10.0, psi);
    const double sas =
        m8c3::largest(m8c3::collective_step(ex, t, 20.0, false, 0.05, 10.0), 0.0, 10.0, psi);
    std::printf(
        "20 kt, collective +0.05: largest heading change in 10 s %.2f deg (FPS), %.2f deg "
        "(SAS)\n",
        fps, sas);
    EXPECT_LT(fps, 0.5 * sas);
}

// Linear models with the FPS engaged: its synchronizers' references don't
// move (a zero row), so they have no quasi-static value; condense() names one
// and says to keep them dynamic, which works (code review).
TEST(Uh60aFps, CondenseNamesTheEngagedReferences) {
    const Executive ex = m8c3::executive();
    const m8c3::Trims t = m8c3::trims(ex, 90.0);
    const LinearModel m = linearize(ex, m8c3::level(90.0, m8c3::kFps), t.fps);
    try {
        (void)condense(m);
        ADD_FAILURE() << "condense() accepted the engaged references";
    } catch (const InitError& e) {
        EXPECT_NE(std::string(e.what()).find("no dynamics of its own"), std::string::npos)
            << e.what();
    }
    std::vector<std::size_t> retained;
    for (std::size_t i = 0; i < m.states.size(); ++i) {
        if (m.states[i].rfind("fcs[", 0) == 0) {
            retained.push_back(i);
        }
    }
    EXPECT_NO_THROW((void)condense(m, retained));
}

}  // namespace
}  // namespace eyas
