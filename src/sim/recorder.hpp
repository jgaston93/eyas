#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace eyas {

// Fixed-capacity time-history recorder. All memory is allocated in the
// constructor; record() never allocates, throws or does I/O, so it may be
// called from advance() (spec §7). When full, the oldest sample is
// overwritten (ring buffer) and wrapped() becomes true.
class Recorder {
public:
    // Throws InitError if there are no channels, duplicate channel names, or
    // zero capacity.
    Recorder(std::vector<std::string> channels, std::size_t capacity);

    // Stores one sample. Returns false and counts a rejection, storing
    // nothing, if values.size() differs from the channel count.
    bool record(std::int64_t time_ns, std::span<const double> values) noexcept;

    std::size_t size() const noexcept { return count_; }
    std::size_t capacity() const noexcept { return capacity_; }
    bool wrapped() const noexcept { return wrapped_; }
    std::uint64_t rejected() const noexcept { return rejected_; }
    const std::vector<std::string>& channels() const noexcept { return channels_; }

    // Retained samples in chronological order; i = 0 is the oldest.
    std::int64_t time_ns(std::size_t i) const noexcept { return times_[slot(i)]; }
    double value(std::size_t i, std::size_t channel) const noexcept {
        return values_[slot(i) * channels_.size() + channel];
    }

    // Writes "time_ns,<channels...>" with round-trip-exact numbers. Not for
    // use in the real-time loop. Throws std::runtime_error on I/O failure.
    void write_csv(const std::filesystem::path& path) const;

private:
    std::size_t slot(std::size_t i) const noexcept {
        return wrapped_ ? (next_ + i) % capacity_ : i;
    }

    std::vector<std::string> channels_;
    std::size_t capacity_;
    std::vector<std::int64_t> times_;
    std::vector<double> values_;
    std::size_t next_ = 0;
    std::size_t count_ = 0;
    bool wrapped_ = false;
    std::uint64_t rejected_ = 0;
};

}  // namespace eyas
