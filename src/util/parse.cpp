#include "util/parse.hpp"

#include <charconv>
#include <cmath>
#include <system_error>

namespace eyas::detail {

std::string_view trim(std::string_view s) noexcept {
    constexpr std::string_view kSpace = " \t\r";
    const auto first = s.find_first_not_of(kSpace);
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(kSpace);
    return s.substr(first, last - first + 1);
}

std::optional<double> parse_finite_double(std::string_view s) noexcept {
    // from_chars rejects a leading '+'; accept it, as hand-entered data often has one.
    if (!s.empty() && s.front() == '+') {
        s.remove_prefix(1);
    }
    double value = 0.0;
    const char* end = s.data() + s.size();
    const auto [ptr, ec] = std::from_chars(s.data(), end, value);
    if (ec != std::errc() || ptr != end || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

}  // namespace eyas::detail
