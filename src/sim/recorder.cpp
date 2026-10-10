#include "sim/recorder.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <fstream>
#include <stdexcept>

#include "util/error.hpp"

namespace eyas {

Recorder::Recorder(std::vector<std::string> channels, std::size_t capacity)
    : channels_(std::move(channels)), capacity_(capacity) {
    if (channels_.empty()) {
        throw InitError("Recorder: at least one channel is required");
    }
    if (capacity_ == 0) {
        throw InitError("Recorder: capacity must be positive");
    }
    for (std::size_t i = 0; i < channels_.size(); ++i) {
        if (std::find(channels_.begin(), channels_.begin() + static_cast<std::ptrdiff_t>(i),
                      channels_[i]) != channels_.begin() + static_cast<std::ptrdiff_t>(i)) {
            throw InitError("Recorder: duplicate channel '" + channels_[i] + "'");
        }
    }
    times_.resize(capacity_);
    values_.resize(capacity_ * channels_.size());
}

bool Recorder::record(std::int64_t time_ns, std::span<const double> values) noexcept {
    if (values.size() != channels_.size()) {
        ++rejected_;
        return false;
    }
    times_[next_] = time_ns;
    std::copy(values.begin(), values.end(),
              values_.begin() + static_cast<std::ptrdiff_t>(next_ * channels_.size()));
    next_ = (next_ + 1) % capacity_;
    if (count_ < capacity_) {
        ++count_;
    } else {
        wrapped_ = true;
    }
    return true;
}

void Recorder::write_csv(const std::filesystem::path& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error(path.string() + ": cannot open for writing");
    }
    out << "time_ns";
    for (const std::string& name : channels_) {
        out << ',' << name;
    }
    out << '\n';

    // std::to_chars gives the shortest text that round-trips exactly.
    std::array<char, 32> buf{};
    for (std::size_t i = 0; i < count_; ++i) {
        auto res = std::to_chars(buf.data(), buf.data() + buf.size(), time_ns(i));
        out.write(buf.data(), res.ptr - buf.data());
        for (std::size_t c = 0; c < channels_.size(); ++c) {
            res = std::to_chars(buf.data(), buf.data() + buf.size(), value(i, c));
            out << ',';
            out.write(buf.data(), res.ptr - buf.data());
        }
        out << '\n';
    }
    if (!out) {
        throw std::runtime_error(path.string() + ": write failed");
    }
}

}  // namespace eyas
