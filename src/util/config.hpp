#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "util/error.hpp"
#include "util/units.hpp"

namespace eyas {

struct CsvTable;

// Read-only view of a YAML configuration document, used at init time only.
// Every accessor throws InitError naming the file and the full key path.
//
// - Physical quantities are strings with an explicit unit and are returned
//   in SI:   radius: "26.83 ft"   ->  quantity("radius", length) == 8.178...
//   The caller names the dimension the value must have; a unit of another
//   dimension is an error.
// - Dimensionless values are plain YAML numbers, read with number(). A
//   quantity without a unit is an error, so a missing unit can never be
//   silently taken as SI.
// - A map key given twice is an error at load.
// - Keys are dotted paths: quantity("main_rotor.radius", Dimension::length).
// - allow_keys() rejects keys the reader doesn't know, so a misspelled
//   optional key is an error rather than silently ignored.
// - A value tagged !placeholder marks a data gap (DATA_GAPS.md). It still
//   loads; placeholders() lists every one so init can report them.
//       blade_weight: !placeholder "256.9 lbf"
//   A reader that loads a data file marked as a placeholder (a CSV table
//   with "# status: placeholder") adds it with note_placeholder() (D-025).
class Config {
public:
    static Config from_file(const std::filesystem::path& path);
    static Config from_string(std::string_view yaml, std::string_view origin = "<string>");

    double quantity(std::string_view key, units::Dimension dimension) const;
    double number(std::string_view key) const;
    std::int64_t integer(std::string_view key) const;
    bool boolean(std::string_view key) const;
    std::string string(std::string_view key) const;
    // A file path. A relative path is taken relative to the directory of the
    // config file (from_file) or to the current directory (from_string).
    std::filesystem::path path(std::string_view key) const;
    // The CSV table in the file `key` names (path()). A table marked
    // "# status: placeholder" is recorded with note_placeholder() (D-025).
    // Call it inside in_context(key), with the reader's own column checks, so
    // that file errors name the key.
    CsvTable csv_table(std::string_view key) const;

    bool contains(std::string_view key) const;
    // Throws unless this node is a map whose keys are all in `allowed`.
    void allow_keys(std::initializer_list<std::string_view> allowed) const;
    Config child(std::string_view key) const;

    // For a sequence node: element count, and element access.
    std::size_t size() const;
    Config operator[](std::size_t index) const;

    // Throws InitError "<file>: <full path of key>: <what>". An empty key names
    // this node itself. For checks made by the reader after a value is read.
    [[noreturn]] void fail(std::string_view key, std::string_view what) const;

    // Runs `f`, re-raising any InitError it throws against `key` via fail(),
    // so errors from a constructor or a data file loaded for `key` still name
    // the config file and key path (D-021).
    template <class F>
    auto in_context(std::string_view key, F&& f) const -> decltype(f()) {
        try {
            return std::forward<F>(f)();
        } catch (const InitError& e) {
            fail(key, e.what());
        }
    }

    // Every data gap in the whole document: the full key path of each
    // !placeholder value, then "<path>: <what>" for each note_placeholder().
    const std::vector<std::string>& placeholders() const { return *placeholders_; }
    // Records a data gap found by the reader of `key`, e.g. a placeholder
    // table in the file `key` names. The list is shared by every node of the
    // document, so placeholders() on any of them includes it.
    void note_placeholder(std::string_view key, std::string_view what) const;

private:
    Config(const YAML::Node& node, std::shared_ptr<const std::string> origin, std::string path,
           std::shared_ptr<std::vector<std::string>> placeholders,
           std::shared_ptr<const std::filesystem::path> base_dir);

    std::optional<YAML::Node> find(std::string_view key) const;
    std::string scalar(std::string_view key) const;
    std::string full_path(std::string_view key) const;

    YAML::Node node_;
    std::shared_ptr<const std::string> origin_;
    std::string path_;
    std::shared_ptr<std::vector<std::string>> placeholders_;  // shared, appendable
    std::shared_ptr<const std::filesystem::path> base_dir_;   // for relative paths
};

}  // namespace eyas
