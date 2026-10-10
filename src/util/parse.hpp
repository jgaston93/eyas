#pragma once

#include <optional>
#include <string_view>

// Locale-independent text parsing shared by the config and table loaders.
namespace eyas::detail {

// Strips leading and trailing spaces, tabs and carriage returns.
std::string_view trim(std::string_view s) noexcept;

// Parses the whole of `s` as a finite double (std::from_chars, so independent
// of the C locale). Returns nullopt on any leftover characters or non-finite
// result.
std::optional<double> parse_finite_double(std::string_view s) noexcept;

}  // namespace eyas::detail
