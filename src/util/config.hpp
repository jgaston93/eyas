#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace eyas {

// Read-only view of a YAML configuration document, used at init time only.
// Every accessor throws InitError naming the file and the full key path.
//
// - Physical quantities are strings with an explicit unit and are returned
//   in SI:   radius: "26.83 ft"   ->  quantity("radius") == 8.178...
// - Dimensionless values are plain YAML numbers, read with number(). A
//   quantity without a unit is an error, so a missing unit can never be
//   silently taken as SI.
// - Keys are dotted paths: quantity("main_rotor.radius").
// - A value tagged !placeholder marks a data gap (DATA_GAPS.md). It still
//   loads; placeholders() lists every one so init can report them.
//       blade_weight: !placeholder "256.9 lbf"
class Config {
public:
    static Config from_file(const std::filesystem::path& path);
    static Config from_string(std::string_view yaml, std::string_view origin = "<string>");

    double quantity(std::string_view key) const;
    double number(std::string_view key) const;
    std::int64_t integer(std::string_view key) const;
    bool boolean(std::string_view key) const;
    std::string string(std::string_view key) const;

    bool contains(std::string_view key) const;
    Config child(std::string_view key) const;

    // For a sequence node: element count, and element access.
    std::size_t size() const;
    Config operator[](std::size_t index) const;

    // Full key paths of every !placeholder value in the whole document.
    const std::vector<std::string>& placeholders() const { return *placeholders_; }

private:
    Config(const YAML::Node& node, std::shared_ptr<const std::string> origin, std::string path,
           std::shared_ptr<const std::vector<std::string>> placeholders);

    std::optional<YAML::Node> find(std::string_view key) const;
    std::string scalar(std::string_view key) const;
    std::string full_path(std::string_view key) const;
    [[noreturn]] void fail(std::string_view key, std::string_view what) const;

    YAML::Node node_;
    std::shared_ptr<const std::string> origin_;
    std::string path_;
    std::shared_ptr<const std::vector<std::string>> placeholders_;
};

}  // namespace eyas
