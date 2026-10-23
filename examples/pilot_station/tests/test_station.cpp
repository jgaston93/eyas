// Pilot-station tests (D-044): the control mapping and force trim, the
// mapping files, the FlightGear line formats and UDP link, the start points,
// and the real-time rule (no allocation) on the frame path.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "common/afcs.hpp"
#include "common/control_file.hpp"
#include "common/frames.hpp"
#include "common/origin.hpp"
#include "controls_map.hpp"
#include "fg_link.hpp"
#include "mapping.hpp"
#include "recorder.hpp"
#include "station.hpp"
#include "support/alloc_counter.hpp"
#include "watchdog.hpp"

namespace station {
namespace {

AxisMap absolute(int axis, double gain = 1.0) {
    AxisMap m;
    m.axis = axis;
    m.deadband = 0.0;
    m.gain = gain;
    return m;
}

RawInput axes(double a0, double a1 = 0.0, double a2 = 0.0, double a3 = 0.0) {
    RawInput r;
    r.axes[0] = a0;
    r.axes[1] = a1;
    r.axes[2] = a2;
    r.axes[3] = a3;
    return r;
}

// ---- Shaping --------------------------------------------------------------------

TEST(StationControls, ShapeIsOddContinuousAndReachesTheEnds) {
    EXPECT_EQ(shape(0.04, 0.05, 0.0), 0.0);
    EXPECT_EQ(shape(0.05, 0.05, 0.0), 0.0);
    EXPECT_EQ(shape(1.0, 0.05, 0.0), 1.0);
    EXPECT_EQ(shape(-1.0, 0.05, 0.7), -1.0);
    EXPECT_EQ(shape(1.5, 0.05, 0.0), 1.0);  // beyond the device range
    EXPECT_DOUBLE_EQ(shape(0.525, 0.05, 0.0), 0.5);
    EXPECT_DOUBLE_EQ(shape(-0.3, 0.1, 0.4), -shape(0.3, 0.1, 0.4));
    // Expo: (1 − e) x + e x³.
    EXPECT_DOUBLE_EQ(shape(0.5, 0.0, 1.0), 0.125);
    EXPECT_DOUBLE_EQ(shape(0.5, 0.0, 0.5), 0.5 * 0.5 + 0.5 * 0.125);
    EXPECT_EQ(shape(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0), 0.0);
}

// ---- Mapping ----------------------------------------------------------------------

eyas::Controls start_controls() {
    eyas::Controls c;
    c.collective = 0.4;
    c.lat_cyclic = 0.1;
    c.lon_cyclic = -0.2;
    c.pedal = 0.3;
    return c;
}

TEST(StationControls, AbsoluteAxesMoveFromTheStart) {
    ControlsMapConfig cfg;
    cfg.lat_cyclic = absolute(0);
    cfg.lon_cyclic = absolute(1, 0.5);
    cfg.lon_cyclic.invert = true;
    ControlsMap m(cfg, start_controls());
    const eyas::Controls c = m.update(axes(0.5, 0.4), 1.0 / 60.0);
    EXPECT_DOUBLE_EQ(c.lat_cyclic, 0.6);
    EXPECT_DOUBLE_EQ(c.lon_cyclic, -0.2 - 0.5 * 0.4);
    // Unmapped controls hold the start.
    EXPECT_EQ(c.collective, 0.4);
    EXPECT_EQ(c.pedal, 0.3);
    // The applied controls are start + delta, as eyas_drive computes them.
    const eyas::Controls a = apply(m.start(), m.deltas());
    EXPECT_EQ(a.lat_cyclic, c.lat_cyclic);
    EXPECT_EQ(a.lon_cyclic, c.lon_cyclic);
}

TEST(StationControls, ControlsStayInTheirRanges) {
    ControlsMapConfig cfg;
    cfg.lat_cyclic = absolute(0, 2.0);
    cfg.pedal = absolute(1, 2.0);
    cfg.collective = absolute(2, 1.0);
    eyas::Controls s = start_controls();
    s.lat_cyclic = 0.8;
    ControlsMap m(cfg, s);
    const double ulp = 2.0 * std::numeric_limits<double>::epsilon();
    // Held (pickup pending): the collective is the start, in range whatever
    // the lever says (code review: the held path was untested).
    for (const double v : {-1.0, 1.0}) {
        const eyas::Controls c = m.update(axes(0.0, 0.0, v), 0.1);
        EXPECT_TRUE(m.collective_held());
        EXPECT_EQ(c.collective, 0.4);
    }
    (void)m.update(axes(0.0, 0.0, -0.2), 0.1);  // collective lever picked up at its start
    ASSERT_FALSE(m.collective_held());
    for (const double v : {-1.0, 1.0}) {
        const eyas::Controls c = m.update(axes(v, -v, v), 0.1);
        EXPECT_NEAR(c.lat_cyclic, v, ulp);
        EXPECT_NEAR(c.pedal, -v, ulp);
        EXPECT_NEAR(c.collective, v > 0.0 ? 1.0 : 0.0, ulp);
    }
}

// An absolute collective is a lever: its middle is mid travel, whatever the start.
// (After pickup: see LeverCollectiveWaitsForPickup.)
TEST(StationControls, AbsoluteCollectiveIsALever) {
    ControlsMapConfig cfg;
    cfg.collective = absolute(0);
    ControlsMap m(cfg, start_controls());
    (void)m.update(axes(-0.2), 0.1);  // lever at 0.4 = the start: picked up
    ASSERT_FALSE(m.collective_held());
    EXPECT_DOUBLE_EQ(m.update(axes(-1.0), 0.1).collective, 0.0);
    EXPECT_DOUBLE_EQ(m.update(axes(0.0), 0.1).collective, 0.5);
    EXPECT_DOUBLE_EQ(m.update(axes(1.0), 0.1).collective, 1.0);
}

// Pickup (found flying the X-56): after a start, a lever collective holds
// the start value whatever the lever says, until the lever is brought within
// the pickup band of it, or passes it in a small step (at most kPickupStep).
TEST(StationControls, LeverCollectiveWaitsForPickup) {
    ControlsMapConfig cfg;
    cfg.collective = absolute(0);
    ControlsMap m(cfg, start_controls());  // start collective 0.4
    EXPECT_TRUE(m.collective_held());
    EXPECT_FALSE(m.collective_lever_known());             // no sample yet: the HUD shows no tick
    EXPECT_EQ(m.update(axes(0.0), 0.1).collective, 0.4);  // lever at 0.5: held
    EXPECT_TRUE(m.collective_lever_known());
    EXPECT_DOUBLE_EQ(m.collective_lever(), 0.5);
    EXPECT_EQ(m.update(axes(0.1), 0.1).collective, 0.4);  // further away: held
    // A big step across the held value (0.55 -> 0.3) is a jump, not a pass.
    EXPECT_EQ(m.update(axes(-0.4), 0.1).collective, 0.4);
    EXPECT_TRUE(m.collective_held());
    // Small steps up towards it (0.3 -> 0.35 -> 0.39): picked up once within
    // the band, and follows from there.
    EXPECT_EQ(m.update(axes(-0.3), 0.1).collective, 0.4);
    EXPECT_DOUBLE_EQ(m.update(axes(-0.22), 0.1).collective, 0.39);  // within 2 %: taken
    EXPECT_FALSE(m.collective_held());
    EXPECT_DOUBLE_EQ(m.update(axes(-0.14), 0.1).collective, 0.43);
    // A small step across without landing in the band also takes over.
    m.restart(start_controls());
    (void)m.update(axes(-0.24), 0.1);                               // 0.38
    EXPECT_DOUBLE_EQ(m.update(axes(-0.16), 0.1).collective, 0.42);  // passed in 0.04
    EXPECT_FALSE(m.collective_held());
    // Within the pickup band (2 % of travel) also takes over.
    m.restart(start_controls());
    EXPECT_DOUBLE_EQ(m.update(axes(-0.17), 0.1).collective, 0.415);  // 1.5 % away: taken
    EXPECT_FALSE(m.collective_held());
    // Unrelated: a rate collective never waits.
    cfg.collective.mode = AxisMode::rate;
    ControlsMap r(cfg, start_controls());
    EXPECT_FALSE(r.collective_held());
}

// Rate mode integrates exactly for a constant input and holds when centred.
TEST(StationControls, RateCollectiveIntegratesAndHolds) {
    ControlsMapConfig cfg;
    cfg.collective = absolute(0, 0.5);
    cfg.collective.mode = AxisMode::rate;
    ControlsMap m(cfg, start_controls());
    eyas::Controls c;
    for (int i = 0; i < 10; ++i) {
        c = m.update(axes(0.4), 0.1);
    }
    EXPECT_NEAR(c.collective, 0.4 + 0.5 * 0.4 * 1.0, 1e-14);
    for (int i = 0; i < 10; ++i) {
        EXPECT_NEAR(m.update(axes(0.0), 0.1).collective, 0.6, 1e-14);
    }
    for (int i = 0; i < 100; ++i) {
        c = m.update(axes(1.0), 0.1);
    }
    EXPECT_NEAR(c.collective, 1.0, 3e-16);
}

// Force trim: on release of the trim button the control stays where it is,
// even as the spring-centred stick returns; once centred it flies again
// about the new trim point. Trim reset goes back to the start.
TEST(StationControls, ForceTrimHoldsTheReleasePoint) {
    ControlsMapConfig cfg;
    cfg.lat_cyclic = absolute(0);
    cfg.trim_button = 0;
    cfg.trim_reset_button = 1;
    ControlsMap m(cfg, start_controls());
    RawInput in = axes(0.5);
    in.buttons[0] = true;
    EXPECT_DOUBLE_EQ(m.update(in, 0.1).lat_cyclic, 0.6);  // flies normally while held
    EXPECT_TRUE(m.trim_held());
    in.buttons[0] = false;
    EXPECT_DOUBLE_EQ(m.update(in, 0.1).lat_cyclic, 0.6);           // released at 0.5
    EXPECT_DOUBLE_EQ(m.update(axes(0.3), 0.1).lat_cyclic, 0.6);    // returning: latched
    EXPECT_DOUBLE_EQ(m.update(axes(0.03), 0.1).lat_cyclic, 0.6);   // still outside recentre
    EXPECT_DOUBLE_EQ(m.update(axes(0.01), 0.1).lat_cyclic, 0.61);  // within: flies again
    EXPECT_DOUBLE_EQ(m.update(axes(0.0), 0.1).lat_cyclic, 0.6);
    EXPECT_DOUBLE_EQ(m.update(axes(-0.2), 0.1).lat_cyclic, 0.4);
    EXPECT_DOUBLE_EQ(m.trim_points().lat_cyclic, 0.5);
    RawInput reset = axes(0.0);
    reset.buttons[1] = true;
    EXPECT_DOUBLE_EQ(m.update(reset, 0.1).lat_cyclic, 0.1);
    EXPECT_EQ(m.trim_points().lat_cyclic, 0.0);
}

TEST(StationControls, BrakeButtonToggles) {
    ControlsMapConfig cfg;
    cfg.brake_button = 2;
    ControlsMap m(cfg, start_controls());
    RawInput in;
    EXPECT_EQ(m.update(in, 0.1).brake_left, 0.0);
    in.buttons[2] = true;
    EXPECT_EQ(m.update(in, 0.1).brake_left, 1.0);
    EXPECT_EQ(m.update(in, 0.1).brake_right, 1.0);  // held: no second toggle
    in.buttons[2] = false;
    (void)m.update(in, 0.1);
    in.buttons[2] = true;
    EXPECT_EQ(m.update(in, 0.1).brake_left, 0.0);
    eyas::Controls parked = start_controls();
    parked.brake_left = parked.brake_right = 1.0;
    m.restart(parked);
    EXPECT_TRUE(m.brakes_set());
}

// M8c2: the SAS button toggles both SAS channels together (off if either is
// on), the PBA button the pitch bias actuator; a restart takes the start's.
TEST(StationControls, AfcsButtonsToggle) {
    ControlsMapConfig cfg;
    cfg.sas_button = 3;
    cfg.pba_button = 4;
    eyas::Controls start = start_controls();
    start.afcs = {true, true, true};
    ControlsMap m(cfg, start);
    RawInput in;
    eyas::Controls c = m.update(in, 0.1);
    EXPECT_TRUE(c.afcs.sas_digital && c.afcs.sas_analog && c.afcs.pitch_bias);
    in.buttons[3] = true;
    c = m.update(in, 0.1);
    EXPECT_FALSE(c.afcs.sas_digital || c.afcs.sas_analog);
    EXPECT_TRUE(c.afcs.pitch_bias);
    c = m.update(in, 0.1);  // held: no second toggle
    EXPECT_FALSE(c.afcs.sas_digital);
    in.buttons[3] = false;
    in.buttons[4] = true;
    c = m.update(in, 0.1);
    EXPECT_FALSE(c.afcs.pitch_bias);
    in.buttons[4] = false;
    (void)m.update(in, 0.1);
    in.buttons[3] = true;
    c = m.update(in, 0.1);
    EXPECT_TRUE(c.afcs.sas_digital && c.afcs.sas_analog);
    // One channel on counts as on: the button turns the SAS off.
    start.afcs = {true, false, false};
    m.restart(start);
    EXPECT_TRUE(m.afcs().sas_digital && !m.afcs().sas_analog);
    in = RawInput{};
    (void)m.update(in, 0.1);
    in.buttons[3] = true;
    EXPECT_FALSE(m.update(in, 0.1).afcs.sas_digital);
}

// --afcs text: the station writes it into the replay command, eyas_drive
// reads it back; with '+' it is the control files' afcs column (M8c3).
TEST(StationControls, AfcsTextRoundTrips) {
    for (int bits = 0; bits < 64; ++bits) {
        const auto on = [bits](int k) { return (bits & (1 << k)) != 0; };
        const eyas::AfcsSwitches s{on(0), on(1), on(2), on(3), on(4), on(5)};
        for (const char sep : {',', '+'}) {
            const eyas::AfcsSwitches r = examples::parse_afcs(examples::afcs_text(s, sep), sep);
            EXPECT_EQ(r, s) << bits << sep;
        }
    }
    EXPECT_EQ(examples::afcs_text({true, true, true}), "sas,pba");
    EXPECT_EQ(examples::afcs_text({true, true, true, true, true, true}, '+'),
              "sas+pba+fps+fps_turn+trim_release");
    EXPECT_EQ(examples::afcs_text({}), "none");
    EXPECT_THROW((void)examples::parse_afcs("sas,autopilot"), std::runtime_error);
}

// M8c3: the FPS button toggles the FPS, and the trim button is the AFCS's
// trim release while held.
TEST(StationControls, FpsButtonAndTrimRelease) {
    ControlsMapConfig cfg;
    cfg.trim_button = 0;
    cfg.fps_button = 5;
    eyas::Controls start = start_controls();
    start.afcs = {true, true, true};
    ControlsMap m(cfg, start);
    RawInput in;
    EXPECT_FALSE(m.update(in, 0.1).afcs.fps);
    in.buttons[5] = true;
    EXPECT_TRUE(m.update(in, 0.1).afcs.fps);
    EXPECT_TRUE(m.update(in, 0.1).afcs.fps);  // held: no second toggle
    in.buttons[5] = false;
    EXPECT_FALSE(m.update(in, 0.1).afcs.trim_release);
    in.buttons[0] = true;
    EXPECT_TRUE(m.update(in, 0.1).afcs.trim_release);
    in.buttons[0] = false;
    const eyas::Controls c = m.update(in, 0.1);
    EXPECT_FALSE(c.afcs.trim_release);
    EXPECT_TRUE(c.afcs.fps && c.afcs.sas_digital && c.afcs.pitch_bias);
    m.restart(start);
    EXPECT_FALSE(m.afcs().fps);
}

// GenHel's turn switch latch (p. 5.5-3): on with the FPS on, VXBIKT above
// 60 kt, the trim release pressed and more than 2° of bank; off with the
// feet on the pedals and the trim release pressed, or the FPS off; held
// otherwise.
TEST(StationControls, TurnSwitchLatchesAsGenHel) {
    constexpr double kDeg = std::numbers::pi / 180.0;
    eyas::AfcsSwitches s;
    s.fps = true;
    s.trim_release = true;
    const TurnSwitchInputs turning{100.0, 10.0 * kDeg};
    EXPECT_TRUE(latch_turn_switch(false, s, false, turning));
    EXPECT_FALSE(latch_turn_switch(false, s, false, {59.0, 10.0 * kDeg}));  // too slow
    EXPECT_FALSE(latch_turn_switch(false, s, false, {100.0, 1.0 * kDeg}));  // wings level
    EXPECT_FALSE(latch_turn_switch(false, s, true, turning));               // feet on
    s.trim_release = false;
    EXPECT_FALSE(latch_turn_switch(false, s, false, turning));   // trim release needed
    EXPECT_TRUE(latch_turn_switch(true, s, true, turning));      // latched: feet alone
    EXPECT_TRUE(latch_turn_switch(true, s, false, {0.0, 0.0}));  // keep it
    s.trim_release = true;
    EXPECT_FALSE(latch_turn_switch(true, s, true, {0.0, 0.0}));  // feet and trim: off
    s.fps = false;
    s.trim_release = false;
    EXPECT_FALSE(latch_turn_switch(true, s, false, turning));  // FPS off

    // Through the map: trim held in a bank at 100 kt latches it; feet on
    // the pedals (beyond feet_on_pedals) with trim held unlatches it.
    ControlsMapConfig cfg;
    cfg.trim_button = 0;
    cfg.pedal = absolute(1);
    eyas::Controls start = start_controls();
    start.afcs = {true, true, true, true, false, false};
    ControlsMap m(cfg, start);
    RawInput in;
    EXPECT_FALSE(m.update(in, 0.1, turning).afcs.fps_turn);
    in.buttons[0] = true;
    EXPECT_TRUE(m.update(in, 0.1, turning).afcs.fps_turn);
    in.buttons[0] = false;
    EXPECT_TRUE(m.update(in, 0.1, {120.0, 0.0}).afcs.fps_turn);
    in.axes[1] = 0.5;  // feet on, trim up: stays
    EXPECT_TRUE(m.update(in, 0.1, turning).afcs.fps_turn);
    in.buttons[0] = true;  // and trim pressed: off
    EXPECT_FALSE(m.update(in, 0.1, turning).afcs.fps_turn);
}

// ---- Mapping files ----------------------------------------------------------------

TEST(StationMapping, FileOverridesTheDefaults) {
    const ControlsMapConfig base = default_map(InputSource::gamepad);
    const ControlsMapConfig m = parse_map(
        "# a comment\n"
        "collective.mode = absolute   # a lever\n"
        "collective.axis = 5\n"
        "pedal.invert = true\n"
        "lat_cyclic.expo = 0.3\n"
        "trim_button = 4\n"
        "\n",
        base, "test.map");
    EXPECT_EQ(m.collective.mode, AxisMode::absolute);
    EXPECT_EQ(m.collective.axis, 5);
    EXPECT_TRUE(m.pedal.invert);
    EXPECT_EQ(m.lat_cyclic.expo, 0.3);
    EXPECT_EQ(m.trim_button, 4);
    EXPECT_EQ(m.lon_cyclic.axis, base.lon_cyclic.axis);  // untouched
}

TEST(StationMapping, WrittenMapReadsBack) {
    for (const InputSource s : {InputSource::gamepad, InputSource::joystick,
                                InputSource::mouse_keyboard, InputSource::flightgear}) {
        ControlsMapConfig m = default_map(s);
        m.feet_on_pedals = 0.3;  // not the default, so the read-back shows
        m.power_lever = absolute(5);
        m.engine_select_button = 7;
        m.rpm_up_button = 8;
        m.rpm_down_button = 9;
        m.rpm_rate = 0.02;
        m.rpm_low = 0.95;
        m.rpm_high = 1.02;
        std::ostringstream out;
        write_map(out, m);
        const ControlsMapConfig r = parse_map(out.str(), ControlsMapConfig{}, "written");
        for (const auto& [a, b] : {std::pair{m.collective, r.collective},
                                   {m.lat_cyclic, r.lat_cyclic},
                                   {m.lon_cyclic, r.lon_cyclic},
                                   {m.pedal, r.pedal},
                                   {m.power_lever, r.power_lever}}) {
            EXPECT_EQ(a.axis, b.axis);
            EXPECT_EQ(a.invert, b.invert);
            EXPECT_EQ(a.deadband, b.deadband);
            EXPECT_EQ(a.expo, b.expo);
            EXPECT_EQ(a.mode, b.mode);
            EXPECT_EQ(a.gain, b.gain);
        }
        EXPECT_EQ(m.trim_button, r.trim_button);
        EXPECT_EQ(m.trim_reset_button, r.trim_reset_button);
        EXPECT_EQ(m.brake_button, r.brake_button);
        EXPECT_EQ(m.sas_button, r.sas_button);
        EXPECT_EQ(m.pba_button, r.pba_button);
        EXPECT_EQ(m.fps_button, r.fps_button);
        EXPECT_GE(m.fps_button, 0);  // every device source has one (M8c3)
        EXPECT_EQ(m.recentre, r.recentre);
        EXPECT_EQ(m.pickup, r.pickup);
        EXPECT_EQ(m.feet_on_pedals, r.feet_on_pedals);
        EXPECT_EQ(m.engine_select_button, r.engine_select_button);
        EXPECT_EQ(m.rpm_up_button, r.rpm_up_button);
        EXPECT_EQ(m.rpm_down_button, r.rpm_down_button);
        EXPECT_EQ(m.rpm_rate, r.rpm_rate);
        EXPECT_EQ(m.rpm_low, r.rpm_low);
        EXPECT_EQ(m.rpm_high, r.rpm_high);
    }
    EXPECT_THROW((void)parse_map("rpm_low = 1.2\n", ControlsMapConfig{}, "t"), std::runtime_error);
    EXPECT_THROW((void)parse_map("feet_on_pedals = 1.5\n", ControlsMapConfig{}, "t"),
                 std::runtime_error);
}

TEST(StationMapping, BadLinesNameFileAndLine) {
    const ControlsMapConfig base;
    for (const char* text :
         {"collective.axis = x\n", "nonsense\n", "yaw.axis = 1\n", "pedal.mode = sideways\n",
          "pedal.axis = 12\n", "pedal.axis = 4:0\n", "pedal.axis = 1:-2\n", "\ntrim_button = 32\n",
          "brake_button = 0:x\n", "pedal.invert = maybe\n"}) {
        try {
            (void)parse_map(text, base, "bad.map");
            ADD_FAILURE() << "accepted: " << text;
        } catch (const std::runtime_error& e) {
            EXPECT_EQ(std::string(e.what()).rfind("bad.map:", 0), 0U) << e.what();
        }
    }
    EXPECT_THROW((void)parse_source("wheel"), std::runtime_error);
    // device:index, a plain index on device 0, and -1 for none.
    const ControlsMapConfig m = parse_map(
        "collective.axis = 2:1\nlat_cyclic.axis = 5\npedal.axis = -1\ntrim_button = 1:31\n", base,
        "ok.map");
    EXPECT_EQ(m.collective.axis, axis_index(2, 1));
    EXPECT_EQ(m.lat_cyclic.axis, axis_index(0, 5));
    EXPECT_EQ(m.pedal.axis, -1);
    EXPECT_EQ(m.trim_button, button_index(1, 31));
    EXPECT_EQ(parse_source("fg"), InputSource::flightgear);
}

// ---- FlightGear line formats ------------------------------------------------------

TEST(StationFlightGear, ControlLineRoundTrips) {
    RawInput in;
    for (int i = 0; i < kMaxAxes; ++i) {
        in.axes[static_cast<std::size_t>(i)] = -1.0 + 0.25 * i;
    }
    in.buttons[0] = true;
    in.buttons[15] = true;
    std::array<char, 4096> buf{};
    const std::size_t n = encode_ctrl(in, buf);
    ASSERT_GT(n, 0U);
    EXPECT_EQ(buf[n - 1], '\n');
    RawInput out;
    ASSERT_TRUE(parse_ctrl(std::span<const char>(buf.data(), n), out));
    EXPECT_EQ(out.axes, in.axes);
    EXPECT_EQ(out.buttons, in.buttons);
}

TEST(StationFlightGear, MalformedControlLinesAreRejected) {
    RawInput out;
    out.axes[0] = 0.7;
    // A line as FlightGear writes it: per device, its axes then its buttons.
    std::string good;
    for (int d = 0; d < kMaxDevices; ++d) {
        for (int i = 0; i < kAxesPerDevice; ++i) {
            good += (good.empty() ? "" : ",") + std::to_string(0.01 * (d * 100 + i));
        }
        for (int i = 0; i < kButtonsPerDevice; ++i) {
            good += (d == 2 && i == 5) ? ",1" : ",0";
        }
    }
    ASSERT_TRUE(parse_ctrl(std::span<const char>(good.data(), good.size()), out));
    EXPECT_EQ(out.axes[static_cast<std::size_t>(axis_index(1, 3))], 1.03);
    EXPECT_EQ(out.axes[static_cast<std::size_t>(axis_index(2, 0))], 2.0);
    EXPECT_TRUE(out.buttons[static_cast<std::size_t>(button_index(2, 5))]);
    EXPECT_FALSE(out.buttons[static_cast<std::size_t>(button_index(1, 5))]);
    const RawInput before = out;
    for (const std::string& bad : {good.substr(0, good.size() - 2),  // short
                                   good + ",0",                      // long
                                   std::string("0.1,x") + good.substr(8), std::string(""),
                                   std::string("nan") + good.substr(8), good + " "}) {
        EXPECT_FALSE(parse_ctrl(std::span<const char>(bad.data(), bad.size()), out)) << bad;
        EXPECT_EQ(out.axes, before.axes);
    }
    const std::string crlf = good + "\r\n";
    EXPECT_TRUE(parse_ctrl(std::span<const char>(crlf.data(), crlf.size()), out));
}

std::vector<double> fields(const char* line, std::size_t n) {
    std::vector<double> out;
    std::stringstream s(std::string(line, n));
    std::string f;
    while (std::getline(s, f, ',')) {
        out.push_back(std::stod(f));
    }
    return out;
}

TEST(StationFlightGear, FdmLineHasTheProtocolFieldsInFgUnits) {
    eyas::AircraftState s;
    s.latitude = 0.5;
    s.longitude = -2.0;
    s.altitude = 100.0;
    s.heading = 1.0;
    s.calibrated_airspeed = 51.444444444444;  // 100 kt
    s.vertical_speed = 3.048;                 // 10 ft/s
    s.weight_on_wheels = true;
    std::array<char, 1024> buf{};
    const std::size_t n = encode_fdm(s, buf);
    ASSERT_GT(n, 0U);
    const std::vector<double> f = fields(buf.data(), n);
    ASSERT_EQ(f.size(), static_cast<std::size_t>(kFdmFields));
    EXPECT_NEAR(f[0], 0.5 * 180.0 / M_PI, 1e-9);
    EXPECT_NEAR(f[1], -2.0 * 180.0 / M_PI, 1e-9);
    EXPECT_NEAR(f[2], 100.0 / 0.3048, 1e-4);
    EXPECT_NEAR(f[5], 180.0 / M_PI, 1e-5);
    EXPECT_NEAR(f[9], 100.0, 1e-4);
    EXPECT_NEAR(f[10], 10.0, 1e-4);
    EXPECT_EQ(f[19], 0.0);  // NaN rotor azimuth (no rotor) goes as 0
    EXPECT_EQ(f[21], 1.0);
    std::array<char, 16> tiny{};
    EXPECT_EQ(encode_fdm(s, tiny), 0U);
}

// The protocol files FlightGear reads: well-formed comments (XML forbids
// "--" inside one, which FlightGear reports as "not well-formed") and as
// many chunks as the C++ writes or reads.
std::string read_file(const std::string& path) {
    std::ifstream in(path);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}

int count(const std::string& text, const std::string& what) {
    int n = 0;
    for (auto p = text.find(what); p != std::string::npos; p = text.find(what, p + 1)) {
        ++n;
    }
    return n;
}

TEST(StationFlightGear, ProtocolFilesMatchTheCode) {
    for (const auto& [file, chunks] : {std::pair{"eyas_fdm.xml", kFdmFields},
                                       {"eyas_ctrl.xml", kMaxAxes + kMaxButtons},
                                       {"eyas_joystick.xml", 0}}) {
        const std::string text = read_file(std::string(EYAS_STATION_FG_DIR) + "/" + file);
        ASSERT_FALSE(text.empty()) << file;
        EXPECT_EQ(count(text, "<chunk>"), chunks) << file;
        EXPECT_EQ(count(text, "</chunk>"), chunks) << file;
        if (std::string(file) == "eyas_joystick.xml") {
            // It publishes every axis and button eyas_ctrl.xml sends.
            EXPECT_EQ(count(text, "<js n="), kMaxDevices);
            EXPECT_EQ(count(text, "<axis n="), kMaxAxes);
            EXPECT_EQ(count(text, "<button n="), kMaxButtons);
            for (int d = 0; d < kMaxDevices; ++d) {
                EXPECT_EQ(count(text, "/devices/status/joysticks/joystick[" + std::to_string(d) +
                                          "]/button["),
                          2 * kButtonsPerDevice);  // press and release
            }
        }
        for (auto b = text.find("<!--"); b != std::string::npos; b = text.find("<!--", b + 4)) {
            const auto e = text.find("-->", b + 4);
            ASSERT_NE(e, std::string::npos) << file;
            EXPECT_EQ(text.substr(b + 4, e - b - 4).find("--"), std::string::npos)
                << file << ": \"--\" inside a comment";
        }
    }
}

// The link over loopback: an FdmLine reaches a listener, and a control line
// sent to the station's port is read back. Both ends take free ports from
// the system, so concurrent test runs can't collide (code review).
TEST(StationFlightGear, LinkSendsAndReceivesOverLoopback) {
    const int fg = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(fg, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    ASSERT_EQ(::bind(fg, reinterpret_cast<const sockaddr*>(&a), sizeof a), 0);
    socklen_t len = sizeof a;
    ASSERT_EQ(::getsockname(fg, reinterpret_cast<sockaddr*>(&a), &len), 0);
    const int kFgPort = ntohs(a.sin_port);
    timeval tv{1, 0};
    ASSERT_EQ(::setsockopt(fg, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv), 0);

    FgLink link;
    link.open("127.0.0.1:" + std::to_string(kFgPort), 0);
    const int kStationPort = link.bound_port();
    ASSERT_GT(kStationPort, 0);
    ASSERT_TRUE(link.sending());
    ASSERT_TRUE(link.receiving());
    eyas::AircraftState s;
    s.latitude = 0.6;
    link.send(s);
    std::array<char, 4096> buf{};
    const ssize_t got = ::recv(fg, buf.data(), buf.size(), 0);
    ASSERT_GT(got, 0);
    EXPECT_EQ(fields(buf.data(), static_cast<std::size_t>(got)).size(),
              static_cast<std::size_t>(kFdmFields));

    RawInput in;
    in.axes[3] = -0.5;
    in.buttons[1] = true;
    const std::size_t n = encode_ctrl(in, buf);
    a.sin_port = htons(static_cast<std::uint16_t>(kStationPort));
    for (int i = 0; i < 2; ++i) {  // the newest of two lines wins
        in.axes[4] = i;
        const std::size_t m = encode_ctrl(in, buf);
        ASSERT_EQ(::sendto(fg, buf.data(), m, 0, reinterpret_cast<const sockaddr*>(&a), sizeof a),
                  static_cast<ssize_t>(m));
    }
    (void)n;
    RawInput out;
    bool ok = false;
    for (int i = 0; i < 100 && !ok; ++i) {
        ok = link.poll(out) && out.axes[4] == 1.0;
        if (!ok) {
            ::usleep(10'000);
        }
    }
    EXPECT_TRUE(ok);
    EXPECT_EQ(out.axes[3], -0.5);
    EXPECT_TRUE(out.buttons[1]);
    EXPECT_EQ(link.lines_rejected(), 0);
    ::close(fg);
}

// ---- Start points and the frame path (the UH-60A) ---------------------------------

Station& uh60a() {
    static std::unique_ptr<Station> st = [] {
        StationOptions o;
        o.config = EYAS_UH60A_CONFIG;
        return std::make_unique<Station>(o);
    }();
    return *st;
}

TEST(StationStarts, PadAndTrimsAreWhereTheySay) {
    Station& st = uh60a();
    ASSERT_EQ(st.starts().size(), 4U);
    EXPECT_EQ(st.starts()[0].name, "pad");
    EXPECT_FALSE(st.starts()[0].replayable);
    ASSERT_TRUE(st.start(0));
    EXPECT_TRUE(st.outputs().weight_on_wheels);
    EXPECT_LT(st.outputs().ground_speed, 1e-3);
    const char* names[] = {"hover", "60", "120"};
    const double kt[] = {0.0, 60.0, 120.0};
    for (std::size_t i = 1; i < 4; ++i) {
        ASSERT_TRUE(st.start(i));
        const StartPoint& p = st.starts()[i];
        EXPECT_EQ(p.name, names[i - 1]);
        EXPECT_TRUE(p.replayable);
        EXPECT_FALSE(st.outputs().weight_on_wheels);
        EXPECT_NEAR(st.outputs().altitude - st.ground_height(), 30.48, 1e-9);
        // The trim holds V on average over the periodic orbit; its start
        // carries the rotor's vibration (measured 4e-4 m/s at 120 kt).
        EXPECT_NEAR(st.outputs().true_airspeed, kt[i - 1] * 1852.0 / 3600.0, 2e-3);
        EXPECT_EQ(st.outputs().sim_time_ns, 0);
    }
    EXPECT_FALSE(st.faulted());
}

// A restart replays bit for bit: restore() is the whole reposition.
TEST(StationStarts, RestartReplaysBitIdentically) {
    Station& st = uh60a();
    const auto fly = [&] {
        EXPECT_TRUE(st.start(2));
        eyas::Controls c = st.starts()[2].controls;
        for (int i = 0; i < 60; ++i) {
            c.lon_cyclic = st.starts()[2].controls.lon_cyclic + (i > 20 ? 0.02 : 0.0);
            (void)st.frame(c);
        }
        return st.outputs();
    };
    const eyas::AircraftState a = fly();
    const eyas::AircraftState b = fly();
    EXPECT_EQ(std::memcmp(a.pos_n.data(), b.pos_n.data(), sizeof(double) * 3), 0);
    EXPECT_EQ(a.q_nb.coeffs(), b.q_nb.coeffs());
    EXPECT_EQ(a.sim_time_ns, b.sim_time_ns);
}

// Collective fully down from the 100 ft hover: the aircraft hits the ground
// at about 20 m/s, past what the gear model covers (SPEC_REVIEW #34). The
// station calls it a crash, freezes the aircraft, and a start clears it.
TEST(StationStarts, CrashFreezesUntilRestart) {
    Station& st = uh60a();
    ASSERT_TRUE(st.start(1));
    eyas::Controls c = st.starts()[1].controls;
    c.collective = 0.0;
    int frames = 0;
    while (!st.crashed() && frames < 60 * 10) {
        (void)st.frame(c);
        ++frames;
    }
    ASSERT_TRUE(st.crashed());
    EXPECT_GT(frames, 60);  // a fall from 100 ft, not an instant fault
    const std::int64_t t = st.outputs().sim_time_ns;
    (void)st.frame(c);
    EXPECT_EQ(st.outputs().sim_time_ns, t);
    ASSERT_TRUE(st.start(1));
    EXPECT_FALSE(st.crashed());
    EXPECT_FALSE(st.faulted());
}

// Everything the main loop does per frame, apart from drawing: map the
// controls, advance, encode and send the FG line, poll for controls.
TEST(StationRealTime, FramePathDoesNotAllocate) {
    Station& st = uh60a();
    ASSERT_TRUE(st.start(1));
    ControlsMap map(default_map(InputSource::gamepad), st.starts()[1].controls);
    FgLink link;
    link.open("127.0.0.1:9", 0);  // the discard port; nothing listens there
    RawInput in = axes(0.1, -0.2, 0.05, 0.1);
    (void)st.frame(map.update(in, 1.0 / 60.0));  // warm up
    eyas::test::AllocationCounter count;
    for (int i = 0; i < 30; ++i) {
        in.axes[2] = 0.01 * i;
        (void)link.poll(in);
        (void)st.frame(map.update(in, 1.0 / 60.0));
        link.send(st.outputs());
    }
    EXPECT_EQ(count.count(), 0U);
    EXPECT_FALSE(st.faulted());
}

// ---- Code-review fixes --------------------------------------------------------------

TEST(StationFlightGear, ListenPortMustBeAUdpPort) {
    FgLink a;
    EXPECT_THROW(a.open("", 70000), std::runtime_error);  // would have wrapped to 4464
    FgLink b;
    EXPECT_THROW(b.open("", -5), std::runtime_error);
    FgLink c;
    c.open("", FgLink::kNoListen);
    EXPECT_FALSE(c.receiving());
    EXPECT_EQ(c.bound_port(), -1);
}

// Shared with eyas_drive, so both fly the same frames for any duration
// (the two formulas used to differ at 50 Hz and 100 Hz).
TEST(StationFrames, FrameArithmeticIsShared) {
    EXPECT_EQ(examples::nominal_frame_ns(60), 16'666'667);
    EXPECT_EQ(examples::nominal_frame_ns(50), 20'000'000);
    EXPECT_EQ(examples::frames_for(10.0, 16'666'667), 600);
    EXPECT_EQ(examples::frames_for(0.29, 20'000'000), 15);  // 14.5 frames: rounds up
}

// No whitespace in --origin: it goes into the replay command as one argument.
TEST(StationFrames, OriginTextHasNoSpaces) {
    EXPECT_NO_THROW((void)examples::parse_origin("51.47,-0.45,25"));
    EXPECT_THROW((void)examples::parse_origin("51.47, -0.45, 25"), std::runtime_error);
    EXPECT_THROW((void)examples::parse_origin("51.47,-0.45"), std::runtime_error);
}

TEST(StationRecorder, ShellQuoteOnlyWhenNeeded) {
    EXPECT_EQ(shell_quote("data/uh60a/uh60a.yaml"), "data/uh60a/uh60a.yaml");
    EXPECT_EQ(shell_quote("51.47,-0.45,25"), "51.47,-0.45,25");
    EXPECT_EQ(shell_quote("my flights/a.csv"), "'my flights/a.csv'");
    EXPECT_EQ(shell_quote("it's"), "'it'\\''s'");
    EXPECT_EQ(shell_quote(""), "''");
}

// Scripts and recordings run from the start point: the pad's snapshot
// carries 40 s of settling, the trims start at 0 (code review).
TEST(StationStarts, FlightTimeRunsFromTheStartPoint) {
    Station& st = uh60a();
    ASSERT_TRUE(st.start(0));
    EXPECT_GT(st.outputs().sim_time, 39.0);
    EXPECT_EQ(st.flight_time(), 0.0);
    (void)st.frame(st.starts()[0].controls);
    EXPECT_NEAR(st.flight_time(), 0.016, 1e-12);  // 16 whole 1 ms steps (D-006)
    ASSERT_TRUE(st.start(2));
    (void)st.frame(st.starts()[2].controls);
    EXPECT_EQ(st.flight_time(), st.outputs().sim_time);  // exactly, for replay
}

TEST(StationStarts, HoverHeightMustClearTheGear) {
    StationOptions o;
    o.config = EYAS_UH60A_CONFIG;
    o.hover_height = 0.0;
    EXPECT_THROW(Station{o}, std::runtime_error);
    o.hover_height = 0.5;  // the trim converges, but the gear is in the ground
    EXPECT_THROW(Station{o}, std::runtime_error);
}

// A change of brakes marks the rest of the recording as not replayable.
TEST(StationRecorder, BrakeChangeIsMarked) {
    Station& st = uh60a();
    ASSERT_TRUE(st.start(1));
    const std::string path =
        (std::filesystem::temp_directory_path() / "eyas_station_brakes.csv").string();
    {
        Recorder rec(path, "", "cfg.yaml", "", 60);
        rec.begin(st, st.outputs());
        eyas::Controls c = st.starts()[1].controls;
        rec.frame(0.0, {}, c, st.outputs());
        c.brake_left = c.brake_right = 1.0;
        rec.frame(0.5, {}, c, st.outputs());
        rec.frame(0.6, {}, c, st.outputs());
        rec.end();
    }
    const std::string text = read_file(path);
    EXPECT_EQ(count(text, "# Replay: eyas_drive "), 1);
    EXPECT_EQ(count(text, "# NOT REPLAYABLE from t = 0.5 s"), 1);
}

// eyas_drive has no engine input either (D-055): a lever or speed-trim
// change is marked, once.
TEST(StationRecorder, EngineControlChangeIsMarked) {
    Station& st = uh60a();
    ASSERT_TRUE(st.start(1));
    const std::string path =
        (std::filesystem::temp_directory_path() / "eyas_station_engines.csv").string();
    {
        Recorder rec(path, "", "cfg.yaml", "", 60);
        rec.begin(st, st.outputs());
        eyas::Controls c = st.starts()[1].controls;
        rec.frame(0.0, {}, c, st.outputs());
        c.engines[1].power_lever = 0.0;
        rec.frame(0.5, {}, c, st.outputs());
        c.rotor_speed_reference = 1.01;
        rec.frame(0.6, {}, c, st.outputs());
        rec.end();
    }
    const std::string text = read_file(path);
    EXPECT_EQ(count(text, "# NOT REPLAYABLE from t = 0.5 s: the engine controls"), 1);
    EXPECT_EQ(count(text, "# NOT REPLAYABLE"), 1);
}

// The start's AFCS switches go into the replay command (M8c2), and every
// row carries the switches, a new row where they change, so eyas_drive
// replays them (M8c3): no NOT REPLAYABLE mark.
TEST(StationRecorder, AfcsChangesAreRecorded) {
    Station& st = uh60a();
    ASSERT_TRUE(st.start(1));
    EXPECT_TRUE(st.starts()[1].controls.afcs.sas_digital);  // the station's default
    const std::string path =
        (std::filesystem::temp_directory_path() / "eyas_station_afcs.csv").string();
    {
        Recorder rec(path, "", "cfg.yaml", "", 60);
        rec.begin(st, st.outputs());
        eyas::Controls c = st.starts()[1].controls;
        rec.frame(0.0, {}, c, st.outputs());
        c.afcs.pitch_bias = false;
        rec.frame(0.25, {}, c, st.outputs());
        rec.frame(0.3, {}, c, st.outputs());
        rec.end();
    }
    const std::string text = read_file(path);
    EXPECT_EQ(count(text, " --afcs sas,pba "), 1);
    EXPECT_EQ(count(text, "NOT REPLAYABLE"), 0);
    EXPECT_EQ(count(text, "\n0,0,0,0,0,sas+pba\n"), 1);
    EXPECT_EQ(count(text, "\n0.25,0,0,0,0,sas\n"), 1);
    EXPECT_EQ(count(text, "\n0.29999999999999999,"), 0);  // unchanged: no row
    // And it reads back with the switches.
    const std::vector<examples::ControlRow> rows = examples::read_controls(path);
    ASSERT_EQ(rows.size(), 2U);
    EXPECT_TRUE(rows[0].has_afcs && rows[0].afcs.pitch_bias);
    EXPECT_TRUE(rows[1].has_afcs && !rows[1].afcs.pitch_bias && rows[1].afcs.sas_digital);
    eyas::Controls start;
    examples::ControlSchedule sched(rows);
    EXPECT_TRUE(sched.at(start, 0.1).afcs.pitch_bias);
    EXPECT_FALSE(sched.at(start, 0.3).afcs.pitch_bias);
}

// Control files: exactly one of the two headers, and nothing after the last
// column, so a misspelt afcs column can't be read as a file without switches
// (code review, M8c3).
TEST(StationRecorder, ControlFileReaderIsStrict) {
    const std::filesystem::path dir = std::filesystem::temp_directory_path();
    const auto read = [&](const std::string& name, const std::string& text) {
        const std::string path = (dir / name).string();
        std::ofstream(path) << text;
        return examples::read_controls(path);
    };
    const std::string old_header = "time_s,d_collective,d_lat_cyclic,d_lon_cyclic,d_pedal\n";
    EXPECT_THROW(read("eyas_cf_upper.csv",
                      "time_s,d_collective,d_lat_cyclic,d_lon_cyclic,"
                      "d_pedal,AFCS\n0,0,0,0,0,sas\n"),
                 std::runtime_error);
    EXPECT_THROW(read("eyas_cf_extra.csv", old_header + "0,0,0,0,0,sas+fps\n"), std::runtime_error);
    EXPECT_THROW(read("eyas_cf_bad.csv",
                      std::string(examples::kControlHeaderAfcs) + "\n0,0,0,0,0,sas+autopilot\n"),
                 std::runtime_error);
    const auto rows = read("eyas_cf_crlf.csv",
                           std::string(examples::kControlHeaderAfcs) + "\r\n0,0,0,0,0,sas+fps\r\n");
    ASSERT_EQ(rows.size(), 1U);
    EXPECT_TRUE(rows[0].has_afcs && rows[0].afcs.fps && rows[0].afcs.sas_analog);
    EXPECT_EQ(read("eyas_cf_old.csv", old_header + "0,0,0,0,0\n").size(), 1U);
}

// The exact FlightGear sequence (code review): the untouched throttle reads
// 0 (mid travel), then its first real sample is idle. From the 60 kt start
// (collective 0.403) that step passes the held value, but as a jump it must
// not pick up: the collective used to drop to 0 in one frame. Same from the
// pad. The X-56 map's collective: device 2, axis 0, inverted.
TEST(StationControls, FlightGearUntouchedThenIdleDoesNotDropTheCollective) {
    const ControlsMapConfig cfg = read_map_file(std::string(EYAS_STATION_MAPS_DIR) + "/x56_fg.map",
                                                default_map(InputSource::flightgear));
    RawInput untouched;
    RawInput idle;
    idle.axes[static_cast<std::size_t>(axis_index(2, 0))] = 1.0;  // reads +1 at idle
    for (const double start : {0.403, 0.2, 0.543}) {
        eyas::Controls s;
        s.collective = start;
        ControlsMap m(cfg, s);
        EXPECT_EQ(m.update(untouched, 1.0 / 60.0).collective, start);
        EXPECT_EQ(m.update(idle, 1.0 / 60.0).collective, start) << start;
        EXPECT_TRUE(m.collective_held());
    }
}

// After input was lost, the device may come back reading something else (a
// restarted FlightGear reads an untouched axis as 0): rearm_pickup() holds the
// collective where it is until the lever is brought to it (code review).
TEST(StationControls, RearmedPickupHoldsAfterInputLoss) {
    ControlsMapConfig cfg;
    cfg.collective = absolute(0);
    ControlsMap m(cfg, start_controls());
    (void)m.update(axes(0.086), 0.1);  // 0.543 ...
    (void)m.update(axes(-0.2), 0.1);   // ... a jump; then 0.4 = held: picked up
    ASSERT_FALSE(m.collective_held());
    EXPECT_DOUBLE_EQ(m.update(axes(0.086), 0.1).collective, 0.543);
    m.rearm_pickup();  // input lost, now back
    EXPECT_FALSE(m.collective_lever_known());
    EXPECT_DOUBLE_EQ(m.update(axes(0.0), 0.1).collective, 0.543);  // untouched: held
    EXPECT_TRUE(m.collective_held());
    EXPECT_DOUBLE_EQ(m.update(axes(0.08), 0.1).collective, 0.54);  // within 2 %: taken
    EXPECT_FALSE(m.collective_held());
}

// The power lever (D-055): full axis travel from the idle stop (−1) to fly
// (+1), for both engines; from the start (fly) it waits until brought to it
// (pickup), as the collective does.
TEST(StationControls, PowerLeverPicksUpThenMovesBothEngines) {
    ControlsMapConfig cfg;
    cfg.power_lever = absolute(1);
    ControlsMap m(cfg, start_controls());
    EXPECT_TRUE(m.power_lever_held(0));
    eyas::Controls c = m.update(axes(0.0, -1.0), 0.1);  // lever aft: held at fly
    EXPECT_EQ(c.engines[0].power_lever, 1.0);
    EXPECT_EQ(c.engines[1].power_lever, 1.0);
    c = m.update(axes(0.0, 0.99), 0.1);  // brought to fly: taken
    EXPECT_FALSE(m.power_lever_held(0));
    EXPECT_FALSE(m.power_lever_held(1));
    c = m.update(axes(0.0, -1.0), 0.1);
    EXPECT_EQ(c.engines[0].power_lever, 0.0);
    EXPECT_EQ(c.engines[1].power_lever, 0.0);
    c = m.update(axes(0.0, 0.0), 0.1);
    EXPECT_DOUBLE_EQ(c.engines[0].power_lever, 0.5);
    // Unmapped: the start's levers.
    ControlsMap u(ControlsMapConfig{}, start_controls());
    EXPECT_EQ(u.update(axes(0.0, -1.0), 0.1).engines[1].power_lever, 1.0);
}

// The engine-select button cycles which engines the lever moves (all,
// engine 1, engine 2, all); a newly selected engine waits for the lever.
TEST(StationControls, EngineSelectMovesOneLever) {
    ControlsMapConfig cfg;
    cfg.power_lever = absolute(1);
    cfg.engine_select_button = 2;
    ControlsMap m(cfg, start_controls());
    RawInput in = axes(0.0, 1.0);
    (void)m.update(in, 0.1);  // both picked up at fly
    in.buttons[2] = true;
    (void)m.update(in, 0.1);  // engine 1 only
    EXPECT_EQ(m.engine_selection(), 1);
    in.buttons[2] = false;
    (void)m.update(in, 0.1);  // at fly: engine 1 picked up again
    in.axes[1] = -1.0;
    eyas::Controls c = m.update(in, 0.1);
    EXPECT_EQ(c.engines[0].power_lever, 0.0);  // engine 1 to idle
    EXPECT_EQ(c.engines[1].power_lever, 1.0);  // engine 2 stays at fly
    in.buttons[2] = true;
    (void)m.update(in, 0.1);  // engine 2 only, waiting: the lever is at idle
    in.buttons[2] = false;
    c = m.update(in, 0.1);
    EXPECT_EQ(m.engine_selection(), 2);
    EXPECT_TRUE(m.power_lever_held(1));
    EXPECT_EQ(c.engines[1].power_lever, 1.0);
    EXPECT_EQ(c.engines[0].power_lever, 0.0);  // engine 1 holds where it was left
}

// The engine-speed trim buttons move the reference at rpm_rate within its
// range; the start's reference is where it begins.
TEST(StationControls, SpeedTrimButtonsBeepTheReference) {
    ControlsMapConfig cfg;
    cfg.rpm_up_button = 0;
    cfg.rpm_down_button = 1;
    ControlsMap m(cfg, start_controls());
    RawInput in;
    in.buttons[0] = true;
    eyas::Controls c;
    for (int i = 0; i < 10; ++i) {
        c = m.update(in, 0.1);
    }
    EXPECT_NEAR(c.rotor_speed_reference, 1.01, 1e-12);
    for (int i = 0; i < 100; ++i) {
        c = m.update(in, 0.1);
    }
    EXPECT_EQ(c.rotor_speed_reference, cfg.rpm_high);
    in.buttons[0] = false;
    in.buttons[1] = true;
    for (int i = 0; i < 200; ++i) {
        c = m.update(in, 0.1);
    }
    EXPECT_EQ(c.rotor_speed_reference, cfg.rpm_low);
    m.restart(start_controls());
    EXPECT_EQ(m.rotor_speed_reference(), 1.0);
}

TEST(StationMapping, PickupMustBeAFraction) {
    for (const char* v : {"-0.02", "nan", "0.6", "x"}) {
        EXPECT_THROW((void)parse_map(std::string("pickup = ") + v + "\n", {}, "p.map"),
                     std::runtime_error)
            << v;
    }
    EXPECT_EQ(parse_map("pickup = 0\n", {}, "p.map").pickup, 0.0);
    EXPECT_EQ(parse_map("pickup = 0.05\n", {}, "p.map").pickup, 0.05);
}

// The input watchdog: stale until the first sample, fresh within the
// timeout, stale again after a gap longer than it, fresh when samples return.
TEST(StationInput, WatchdogPausesOnAGap) {
    using namespace std::chrono_literals;
    const InputWatchdog::Clock::time_point t0{};
    InputWatchdog w(250ms);
    EXPECT_TRUE(w.stale(t0));
    EXPECT_FALSE(w.seen());
    w.fresh(t0 + 1s);
    EXPECT_FALSE(w.stale(t0 + 1s));
    EXPECT_FALSE(w.stale(t0 + 1s + 250ms));  // at the timeout: still fresh
    EXPECT_TRUE(w.stale(t0 + 1s + 251ms));
    EXPECT_TRUE(w.seen());
    w.fresh(t0 + 2s);
    EXPECT_FALSE(w.stale(t0 + 2s + 100ms));
}

}  // namespace
}  // namespace station
