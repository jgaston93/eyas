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
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "controls_map.hpp"
#include "fg_link.hpp"
#include "mapping.hpp"
#include "station.hpp"
#include "support/alloc_counter.hpp"

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
    for (const double v : {-1.0, 1.0}) {
        const eyas::Controls c = m.update(axes(v, -v, v), 0.1);
        EXPECT_NEAR(c.lat_cyclic, v, ulp);
        EXPECT_NEAR(c.pedal, -v, ulp);
        EXPECT_NEAR(c.collective, v > 0.0 ? 1.0 : 0.0, ulp);
    }
}

// An absolute collective is a lever: its middle is mid travel, whatever the start.
TEST(StationControls, AbsoluteCollectiveIsALever) {
    ControlsMapConfig cfg;
    cfg.collective = absolute(0);
    ControlsMap m(cfg, start_controls());
    EXPECT_DOUBLE_EQ(m.update(axes(-1.0), 0.1).collective, 0.0);
    EXPECT_DOUBLE_EQ(m.update(axes(0.0), 0.1).collective, 0.5);
    EXPECT_DOUBLE_EQ(m.update(axes(1.0), 0.1).collective, 1.0);
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
        const ControlsMapConfig m = default_map(s);
        std::ostringstream out;
        write_map(out, m);
        const ControlsMapConfig r = parse_map(out.str(), ControlsMapConfig{}, "written");
        for (const auto& [a, b] : {std::pair{m.collective, r.collective},
                                   {m.lat_cyclic, r.lat_cyclic},
                                   {m.lon_cyclic, r.lon_cyclic},
                                   {m.pedal, r.pedal}}) {
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
        EXPECT_EQ(m.recentre, r.recentre);
    }
}

TEST(StationMapping, BadLinesNameFileAndLine) {
    const ControlsMapConfig base;
    for (const char* text :
         {"collective.axis = x\n", "nonsense\n", "yaw.axis = 1\n", "pedal.mode = sideways\n",
          "pedal.axis = 8\n", "\ntrim_button = 16\n", "pedal.invert = maybe\n"}) {
        try {
            (void)parse_map(text, base, "bad.map");
            ADD_FAILURE() << "accepted: " << text;
        } catch (const std::runtime_error& e) {
            EXPECT_EQ(std::string(e.what()).rfind("bad.map:", 0), 0U) << e.what();
        }
    }
    EXPECT_THROW((void)parse_source("wheel"), std::runtime_error);
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
    std::array<char, 512> buf{};
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
    const std::string good = "0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1";
    ASSERT_TRUE(parse_ctrl(std::span<const char>(good.data(), good.size()), out));
    EXPECT_EQ(out.axes[7], 0.8);
    EXPECT_TRUE(out.buttons[15]);
    const RawInput before = out;
    for (const std::string& bad : {good.substr(0, good.size() - 2),  // short
                                   good + ",0",                      // long
                                   std::string("0.1,x") + good.substr(5), std::string(""),
                                   std::string("nan") + good.substr(3), good + " "}) {
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
    for (const auto& [file, chunks] :
         {std::pair{"eyas_fdm.xml", kFdmFields}, {"eyas_ctrl.xml", kMaxAxes + kMaxButtons}}) {
        const std::string text = read_file(std::string(EYAS_STATION_FG_DIR) + "/" + file);
        ASSERT_FALSE(text.empty()) << file;
        EXPECT_EQ(count(text, "<chunk>"), chunks) << file;
        EXPECT_EQ(count(text, "</chunk>"), chunks) << file;
        for (auto b = text.find("<!--"); b != std::string::npos; b = text.find("<!--", b + 4)) {
            const auto e = text.find("-->", b + 4);
            ASSERT_NE(e, std::string::npos) << file;
            EXPECT_EQ(text.substr(b + 4, e - b - 4).find("--"), std::string::npos)
                << file << ": \"--\" inside a comment";
        }
    }
}

// The link over loopback: an FdmLine reaches a listener, and a control line
// sent to the station's port is read back.
TEST(StationFlightGear, LinkSendsAndReceivesOverLoopback) {
    constexpr int kFgPort = 45601;
    constexpr int kStationPort = 45602;
    const int fg = ::socket(AF_INET, SOCK_DGRAM, 0);
    ASSERT_GE(fg, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = htons(kFgPort);
    ASSERT_EQ(::bind(fg, reinterpret_cast<const sockaddr*>(&a), sizeof a), 0);
    timeval tv{1, 0};
    ASSERT_EQ(::setsockopt(fg, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv), 0);

    FgLink link;
    link.open("127.0.0.1:" + std::to_string(kFgPort), kStationPort);
    ASSERT_TRUE(link.sending());
    ASSERT_TRUE(link.receiving());
    eyas::AircraftState s;
    s.latitude = 0.6;
    link.send(s);
    std::array<char, 2048> buf{};
    const ssize_t got = ::recv(fg, buf.data(), buf.size(), 0);
    ASSERT_GT(got, 0);
    EXPECT_EQ(fields(buf.data(), static_cast<std::size_t>(got)).size(),
              static_cast<std::size_t>(kFdmFields));

    RawInput in;
    in.axes[3] = -0.5;
    in.buttons[1] = true;
    const std::size_t n = encode_ctrl(in, buf);
    a.sin_port = htons(kStationPort);
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
    link.open("127.0.0.1:45603", 45604);
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

}  // namespace
}  // namespace station
