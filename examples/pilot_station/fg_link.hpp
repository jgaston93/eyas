#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

#include "controls_map.hpp"
#include "eyas/aircraft_state.hpp"

// The FlightGear link (D-044): FlightGear as an image generator, run with
// --fdm=null and two generic protocols whose files are in flightgear/:
//   eyas_fdm  (FG input)  one ASCII line per frame: the pose and the values
//                         FG's displays and animations read;
//   eyas_ctrl (FG output) one ASCII line per FG frame: the raw axes and
//                         buttons of joysticks 0..kMaxDevices-1, for --input
//                         fg. FlightGear doesn't publish raw joystick values
//                         itself: eyas_joystick.xml, loaded with --config,
//                         binds them to the properties eyas_ctrl sends.
// tools/build_fg_station_files.py writes the three files.
// The field order here and in those files must match; both are listed in
// encode_fdm() and parse_ctrl().
//
// Real time: encoding, parsing, sending and receiving don't allocate or throw.
namespace station {

// Writes one eyas_fdm line (newline-terminated) into `buf`. Returns its
// length, or 0 if it doesn't fit.
std::size_t encode_fdm(const eyas::AircraftState& s, std::span<char> buf) noexcept;

// The number of fields in an eyas_fdm line, for the tests.
inline constexpr int kFdmFields = 22;

// Parses one eyas_ctrl line: for each of kMaxDevices devices, its
// kAxesPerDevice axes (floats) then its kButtonsPerDevice buttons (0 or 1),
// comma separated, an optional trailing newline. Returns false, leaving
// `out` unchanged, on any malformed line.
bool parse_ctrl(std::span<const char> line, RawInput& out) noexcept;

// Writes one eyas_ctrl line, as FG would; for the tests and for a loopback.
std::size_t encode_ctrl(const RawInput& in, std::span<char> buf) noexcept;

// A UDP socket pair: sends eyas_fdm lines to FG and receives eyas_ctrl
// lines. Opening resolves and binds (init time; throws std::runtime_error).
class FgLink {
public:
    FgLink() = default;
    FgLink(const FgLink&) = delete;
    FgLink& operator=(const FgLink&) = delete;
    ~FgLink();

    // `host:port` FG listens on for eyas_fdm (empty: don't send). `listen_port`
    // receives eyas_ctrl: kNoListen doesn't receive, 0 takes any free port
    // (see bound_port()), 1–65535 that port; anything else throws.
    void open(const std::string& fg_address, int listen_port);
    static constexpr int kNoListen = -1;
    // The UDP port receiving eyas_ctrl, or −1 when not receiving.
    int bound_port() const noexcept { return bound_port_; }

    bool sending() const noexcept { return send_fd_ >= 0; }
    bool receiving() const noexcept { return recv_fd_ >= 0; }

    // Sends the current state. Never blocks; a full socket buffer drops the line.
    void send(const eyas::AircraftState& s) noexcept;
    // Reads every pending line and keeps the newest valid one in `out`.
    // Returns true if one arrived this call.
    bool poll(RawInput& out) noexcept;

    std::int64_t lines_received() const noexcept { return received_; }
    std::int64_t lines_rejected() const noexcept { return rejected_; }

private:
    int send_fd_ = -1;
    int recv_fd_ = -1;
    int bound_port_ = -1;
    std::int64_t received_ = 0;
    std::int64_t rejected_ = 0;
    std::array<char, 4096> buf_{};
};

}  // namespace station
