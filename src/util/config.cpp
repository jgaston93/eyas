#include "util/config.hpp"

#include <charconv>
#include <fstream>
#include <set>
#include <sstream>
#include <system_error>

#include "util/error.hpp"
#include "util/parse.hpp"
#include "util/units.hpp"

namespace eyas {
namespace {

std::string join(std::string_view path, std::string_view key) {
    if (path.empty()) {
        return std::string(key);
    }
    return std::string(path) + "." + std::string(key);
}

// Walks the whole document once at load: lists every !placeholder value and
// rejects duplicate map keys, which yaml-cpp otherwise accepts silently
// (lookups return the first, so a later correction would be ignored).
void check_and_collect(const YAML::Node& node, const std::string& origin, const std::string& path,
                       std::vector<std::string>& placeholders) {
    if (node.Tag() == "!placeholder") {
        placeholders.push_back(path);
    }
    if (node.IsMap()) {
        std::set<std::string> seen;
        for (const auto& kv : node) {
            const std::string& key = kv.first.Scalar();
            const std::string child = join(path, key);
            if (!seen.insert(key).second) {
                std::string msg = origin;
                msg += ": " + child + ": key given twice";
                throw InitError(msg);
            }
            check_and_collect(kv.second, origin, child, placeholders);
        }
    } else if (node.IsSequence()) {
        for (std::size_t i = 0; i < node.size(); ++i) {
            check_and_collect(node[i], origin, path + "[" + std::to_string(i) + "]", placeholders);
        }
    }
}

}  // namespace

Config::Config(const YAML::Node& node, std::shared_ptr<const std::string> origin, std::string path,
               std::shared_ptr<const std::vector<std::string>> placeholders)
    : node_(node),
      origin_(std::move(origin)),
      path_(std::move(path)),
      placeholders_(std::move(placeholders)) {}

Config Config::from_string(std::string_view yaml, std::string_view origin) {
    YAML::Node root;
    try {
        root = YAML::Load(std::string(yaml));
    } catch (const YAML::Exception& e) {
        throw InitError(std::string(origin) + ": YAML error: " + e.what());
    }
    auto placeholders = std::make_shared<std::vector<std::string>>();
    check_and_collect(root, std::string(origin), "", *placeholders);
    return Config(root, std::make_shared<const std::string>(origin), "", std::move(placeholders));
}

Config Config::from_file(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw InitError(path.string() + ": cannot open config file");
    }
    std::ostringstream text;
    text << in.rdbuf();
    return from_string(text.str(), path.string());
}

std::string Config::full_path(std::string_view key) const {
    return join(path_, key);
}

void Config::fail(std::string_view key, std::string_view what) const {
    throw InitError(*origin_ + ": " + full_path(key) + ": " + std::string(what));
}

// Walks the dotted key using const access only, so lookups never insert
// nodes into the document.
std::optional<YAML::Node> Config::find(std::string_view key) const {
    YAML::Node node = node_;
    std::size_t start = 0;
    while (start <= key.size()) {
        const std::size_t dot = key.find('.', start);
        const std::string segment(key.substr(start, dot - start));
        if (segment.empty() || !node.IsMap()) {
            return std::nullopt;
        }
        const YAML::Node& parent = node;
        const YAML::Node child = parent[segment];
        if (!child.IsDefined()) {
            return std::nullopt;
        }
        node.reset(child);
        if (dot == std::string_view::npos) {
            break;
        }
        start = dot + 1;
    }
    return node;
}

std::string Config::scalar(std::string_view key) const {
    const auto node = find(key);
    if (!node) {
        fail(key, "missing");
    }
    if (!node->IsScalar()) {
        fail(key, "expected a single value");
    }
    return node->Scalar();
}

double Config::quantity(std::string_view key) const {
    const std::string text = scalar(key);
    const std::string_view s = detail::trim(text);
    const std::size_t space = s.find_first_of(" \t");
    if (space == std::string_view::npos) {
        fail(key, "'" + text +
                      "' has no unit; write e.g. \"26.83 ft\", or read dimensionless values "
                      "with number()");
    }
    const auto value = detail::parse_finite_double(s.substr(0, space));
    if (!value) {
        fail(key, "'" + text + "' does not start with a finite number");
    }
    const std::string_view unit = detail::trim(s.substr(space));
    const auto scale = units::si_scale(unit);
    if (!scale) {
        fail(key,
             "unknown unit '" + std::string(unit) + "' (see units.cpp; use lbf or lbm, not lb)");
    }
    return *value * *scale;
}

double Config::number(std::string_view key) const {
    const std::string text = scalar(key);
    const auto value = detail::parse_finite_double(detail::trim(text));
    if (!value) {
        fail(key, "'" + text +
                      "' is not a plain finite number (quantities with units are read "
                      "with quantity())");
    }
    return *value;
}

std::int64_t Config::integer(std::string_view key) const {
    const std::string text = scalar(key);
    const std::string_view s = detail::trim(text);
    std::int64_t value = 0;
    const char* end = s.data() + s.size();
    const auto [ptr, ec] = std::from_chars(s.data(), end, value);
    if (ec != std::errc() || ptr != end) {
        fail(key, "'" + text + "' is not an integer");
    }
    return value;
}

bool Config::boolean(std::string_view key) const {
    const std::string text = scalar(key);
    if (text == "true") {
        return true;
    }
    if (text == "false") {
        return false;
    }
    fail(key, "'" + text + "' is not true or false");
}

std::string Config::string(std::string_view key) const {
    return scalar(key);
}

bool Config::contains(std::string_view key) const {
    return find(key).has_value();
}

Config Config::child(std::string_view key) const {
    const auto node = find(key);
    if (!node) {
        fail(key, "missing");
    }
    return Config(*node, origin_, full_path(key), placeholders_);
}

std::size_t Config::size() const {
    if (!node_.IsSequence()) {
        throw InitError(*origin_ + ": " + path_ + ": expected a sequence");
    }
    return node_.size();
}

Config Config::operator[](std::size_t index) const {
    const std::size_t n = size();
    const std::string path = path_ + "[" + std::to_string(index) + "]";
    if (index >= n) {
        throw InitError(*origin_ + ": " + path + ": index out of range (size " + std::to_string(n) +
                        ")");
    }
    const YAML::Node& seq = node_;
    return Config(seq[index], origin_, path, placeholders_);
}

}  // namespace eyas
