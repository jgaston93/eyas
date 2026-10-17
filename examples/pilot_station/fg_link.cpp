#include "fg_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <stdexcept>
#include <system_error>

namespace station {

namespace {

constexpr double kDeg = 180.0 / std::numbers::pi;  // [deg/rad]
constexpr double kFt = 1.0 / 0.3048;               // [ft/m]
constexpr double kKt = 3600.0 / 1852.0;            // [kt per m/s]

// FG reads NaN as garbage; components the config lacks report NaN (D-043).
double finite_or_zero(double v) noexcept {
    return std::isfinite(v) ? v : 0.0;
}

}  // namespace

std::size_t encode_fdm(const eyas::AircraftState& s, std::span<char> buf) noexcept {
    // Field order = flightgear/eyas_fdm.xml chunk order.
    const int n = std::snprintf(
        buf.data(), buf.size(),
        "%.10f,%.10f,%.4f,"  // latitude-deg, longitude-deg, altitude-ft
        "%.5f,%.5f,%.5f,"    // roll-deg, pitch-deg, heading-deg
        "%.5f,%.5f,%.5f,"    // roll, pitch, yaw rate [deg/s] (body rates)
        "%.4f,%.4f,"         // airspeed-kt (calibrated), vertical-speed-fps
        "%.4f,%.4f,%.4f,"    // uBody, vBody, wBody [ft/s]
        "%.4f,%.4f,%.4f,"    // speed-north, -east, -down [ft/s]
        "%.4f,%.4f,"         // side-slip-deg, alpha-deg
        "%.3f,%.3f,%d\n",    // main, tail rotor blade[0] azimuth [deg], wow
        s.latitude * kDeg, s.longitude * kDeg, s.altitude * kFt, s.roll * kDeg, s.pitch * kDeg,
        s.heading * kDeg, s.omega_b.x() * kDeg, s.omega_b.y() * kDeg, s.omega_b.z() * kDeg,
        s.calibrated_airspeed * kKt, s.vertical_speed * kFt, s.vel_b.x() * kFt, s.vel_b.y() * kFt,
        s.vel_b.z() * kFt, s.vel_n.x() * kFt, s.vel_n.y() * kFt, s.vel_n.z() * kFt,
        finite_or_zero(s.sideslip) * kDeg, finite_or_zero(s.angle_of_attack) * kDeg,
        finite_or_zero(s.main_rotor_azimuth) * kDeg, finite_or_zero(s.tail_rotor_azimuth) * kDeg,
        s.weight_on_wheels ? 1 : 0);
    if (n <= 0 || static_cast<std::size_t>(n) >= buf.size()) {
        return 0;
    }
    return static_cast<std::size_t>(n);
}

bool parse_ctrl(std::span<const char> line, RawInput& out) noexcept {
    const char* p = line.data();
    const char* end = line.data() + line.size();
    while (end > p && (end[-1] == '\n' || end[-1] == '\r' || end[-1] == '\0')) {
        --end;
    }
    RawInput r;
    // Device by device: its axes, then its buttons (eyas_ctrl.xml's order).
    constexpr int kPerDevice = kAxesPerDevice + kButtonsPerDevice;
    for (int i = 0; i < kMaxDevices * kPerDevice; ++i) {
        if (i > 0) {
            if (p == end || *p != ',') {
                return false;
            }
            ++p;
        }
        double v = 0.0;
        const auto [q, ec] = std::from_chars(p, end, v);
        if (ec != std::errc{} || !std::isfinite(v)) {
            return false;
        }
        p = q;
        const int device = i / kPerDevice;
        const int k = i % kPerDevice;
        if (k < kAxesPerDevice) {
            r.axes[static_cast<std::size_t>(axis_index(device, k))] = v;
        } else {
            r.buttons[static_cast<std::size_t>(button_index(device, k - kAxesPerDevice))] =
                v != 0.0;
        }
    }
    if (p != end) {
        return false;
    }
    out = r;
    return true;
}

std::size_t encode_ctrl(const RawInput& in, std::span<char> buf) noexcept {
    std::size_t used = 0;
    constexpr int kPerDevice = kAxesPerDevice + kButtonsPerDevice;
    for (int i = 0; i < kMaxDevices * kPerDevice; ++i) {
        const int device = i / kPerDevice;
        const int k = i % kPerDevice;
        const char* sep = i > 0 ? "," : "";
        const int n =
            k < kAxesPerDevice
                ? std::snprintf(buf.data() + used, buf.size() - used, "%s%.6f", sep,
                                in.axes[static_cast<std::size_t>(axis_index(device, k))])
                : std::snprintf(
                      buf.data() + used, buf.size() - used, "%s%d", sep,
                      in.buttons[static_cast<std::size_t>(button_index(device, k - kAxesPerDevice))]
                          ? 1
                          : 0);
        if (n <= 0 || used + static_cast<std::size_t>(n) >= buf.size()) {
            return 0;
        }
        used += static_cast<std::size_t>(n);
    }
    if (used + 1 >= buf.size()) {
        return 0;
    }
    buf[used++] = '\n';
    return used;
}

FgLink::~FgLink() {
    if (send_fd_ >= 0) {
        ::close(send_fd_);
    }
    if (recv_fd_ >= 0) {
        ::close(recv_fd_);
    }
}

void FgLink::open(const std::string& fg_address, int listen_port) {
    if (!fg_address.empty()) {
        const auto colon = fg_address.rfind(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("--fg: expected host:port, got " + fg_address);
        }
        const std::string host = fg_address.substr(0, colon);
        const std::string port = fg_address.substr(colon + 1);
        addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        addrinfo* res = nullptr;
        if (::getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || res == nullptr) {
            throw std::runtime_error("--fg: cannot resolve " + fg_address);
        }
        send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        const bool ok = send_fd_ >= 0 && ::connect(send_fd_, res->ai_addr, res->ai_addrlen) == 0 &&
                        ::fcntl(send_fd_, F_SETFL, O_NONBLOCK) == 0;
        const int err = errno;  // before freeaddrinfo() can change it
        ::freeaddrinfo(res);
        if (!ok) {
            throw std::runtime_error("--fg: cannot open a UDP socket to " + fg_address + ": " +
                                     std::generic_category().message(err));
        }
    }
    if (listen_port != kNoListen && (listen_port < 0 || listen_port > 65535)) {
        throw std::runtime_error("--listen: " + std::to_string(listen_port) +
                                 " is not a UDP port (1-65535)");
    }
    if (listen_port >= 0) {
        recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_ANY);
        a.sin_port = htons(static_cast<std::uint16_t>(listen_port));
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): the sockets API
        const auto* sa = reinterpret_cast<const sockaddr*>(&a);
        if (recv_fd_ < 0 || ::bind(recv_fd_, sa, sizeof a) != 0 ||
            ::fcntl(recv_fd_, F_SETFL, O_NONBLOCK) != 0) {
            throw std::runtime_error("--listen: cannot bind UDP port " +
                                     std::to_string(listen_port) + ": " +
                                     std::generic_category().message(errno));
        }
        sockaddr_in got{};
        socklen_t len = sizeof got;
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): the sockets API
        if (::getsockname(recv_fd_, reinterpret_cast<sockaddr*>(&got), &len) != 0) {
            throw std::runtime_error("--listen: cannot read the bound port: " +
                                     std::generic_category().message(errno));
        }
        bound_port_ = ntohs(got.sin_port);
    }
}

void FgLink::send(const eyas::AircraftState& s) noexcept {
    if (send_fd_ < 0) {
        return;
    }
    const std::size_t n = encode_fdm(s, buf_);
    if (n > 0) {
        // A refused or full send just drops this frame's line (FG may not be up).
        (void)::send(send_fd_, buf_.data(), n, MSG_DONTWAIT);
    }
}

bool FgLink::poll(RawInput& out) noexcept {
    if (recv_fd_ < 0) {
        return false;
    }
    bool got = false;
    for (;;) {
        const ssize_t n = ::recv(recv_fd_, buf_.data(), buf_.size(), MSG_DONTWAIT);
        if (n <= 0) {
            break;
        }
        if (parse_ctrl(std::span<const char>(buf_.data(), static_cast<std::size_t>(n)), out)) {
            got = true;
            ++received_;
        } else {
            ++rejected_;
        }
    }
    return got;
}

}  // namespace station
