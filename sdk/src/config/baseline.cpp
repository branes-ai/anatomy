#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/config/yaml.hpp>
#include <branes/anatomy/util/sha256.hpp>

#include <cmath>
#include <exception>
#include <string>

namespace branes::anatomy::config {

using namespace yaml;

namespace {
constexpr std::string_view kIdent = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_";
}  // namespace

// ── Baseline ────────────────────────────────────────────────────────

result<BaselineConfig, ConfigError>
parse_baseline(std::string_view yaml, std::string_view origin, std::span<const std::string_view> slots) {
    return guarded<BaselineConfig>(origin, [&]() -> BaselineConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"name", "class", "composition", "graph", "parameters"});
        BaselineConfig b;
        b.name = get_string(root, "", "name");
        require(!b.name.empty() &&
                    b.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-") == std::string::npos,
                "name",
                "must match [a-z0-9-]+");

        const YAML::Node comp = child(root, "", "composition");
        only_keys(comp, "composition", slots);
        for (const auto slot : slots) {
            const std::string key(slot);
            const std::string impl = get_string(comp, "composition", key);
            require(!impl.empty() && impl.find_first_not_of(kIdent) == std::string::npos,
                    "composition." + key,
                    "must match [A-Za-z0-9_]+");
            b.composition[key] = impl;
        }

        if (const YAML::Node params = root["parameters"]) {
            require(params.IsMap() || params.IsNull(), "parameters", "expected a mapping");
            if (params.IsMap()) {
                for (const auto& kv : params) {
                    const auto key = kv.first.as<std::string>();
                    require(kv.second.IsScalar(), "parameters." + key, "must be a scalar");
                    b.parameters[key] = kv.second.as<std::string>();
                }
            }
        }

        // The domain flow form (anatomy docs/adr/0004, step 2): a class and a
        // graph, together. A scoreboard graph names the body of each
        // composite; a research graph is the name of a graph registered in C++.
        const auto is_name = [](const std::string& s, std::string_view chars) {
            return !s.empty() && s.find_first_not_of(chars) == std::string::npos;
        };
        require(!root["graph"] || root["class"], "class", "G8: a baseline with a graph declares its class");
        if (root["class"]) {
            b.baseline_class = get_string(root, "", "class");
            require(b.baseline_class == "scoreboard" || b.baseline_class == "research",
                    "class",
                    "must be scoreboard or research");
            const YAML::Node graph = root["graph"];
            require(graph && !graph.IsNull(), "graph", "G8: a baseline with a class gives its graph");
            if (b.baseline_class == "scoreboard") {
                require(graph.IsMap(), "graph", "a scoreboard graph maps each composite to its body");
                for (const auto& kv : graph) {
                    const auto composite = kv.first.as<std::string>();
                    const std::string key = "graph." + composite;
                    require(is_name(composite, "abcdefghijklmnopqrstuvwxyz0123456789_") &&
                                composite.find_first_of("0123456789") != 0,
                            key,
                            "G1: a composite's name matches [a-z_][a-z0-9_]*");
                    require(kv.second.IsScalar(), key, "expected a body's name");
                    const auto body = kv.second.as<std::string>();
                    require(is_name(body, kIdent), key, "G1: a body's name matches [A-Za-z0-9_]+");
                    b.bodies[composite] = body;
                }
            } else {
                require(graph.IsScalar(), "graph", "a research graph is the name of a registered graph");
                b.graph = graph.as<std::string>();
                require(is_name(b.graph, kIdent), "graph", "G1: a graph's name matches [A-Za-z0-9_]+");
            }
        }
        return b;
    });
}

result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path,
                                                  std::span<const std::string_view> slots) {
    return load_file<BaselineConfig>(
        path, [&](std::string_view text, std::string_view origin) { return parse_baseline(text, origin, slots); });
}

std::string emit_baseline(const BaselineConfig& b) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << b.name;
    if (!b.baseline_class.empty()) {
        out << YAML::Key << "class" << YAML::Value << b.baseline_class;
    }
    out << YAML::Key << "composition" << YAML::Value << YAML::BeginMap;
    for (const auto& [slot, impl] : b.composition) {
        out << YAML::Key << slot << YAML::Value << impl;
    }
    out << YAML::EndMap;
    if (!b.bodies.empty()) {
        out << YAML::Key << "graph" << YAML::Value << YAML::BeginMap;
        for (const auto& [composite, body] : b.bodies) {
            out << YAML::Key << composite << YAML::Value << body;
        }
        out << YAML::EndMap;
    } else if (!b.graph.empty()) {
        out << YAML::Key << "graph" << YAML::Value << b.graph;
    }
    out << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
    for (const auto& [key, value] : b.parameters) {
        out << YAML::Key << key << YAML::Value << value;
    }
    out << YAML::EndMap;
    out << YAML::EndMap;
    return finish(out);
}

std::string composition_hash(const BaselineConfig& b, std::span<const std::string_view> slots) {
    std::string canonical;
    for (const auto slot : slots) {
        const auto it = b.composition.find(std::string(slot));
        canonical += std::string(slot) + "=" + (it == b.composition.end() ? std::string() : it->second) + "\n";
    }
    return sha256_hex(canonical);
}

std::optional<double> numeric_parameter(const BaselineConfig& b, std::string_view key) {
    const auto it = b.parameters.find(std::string(key));
    if (it == b.parameters.end()) {
        return std::nullopt;
    }
    try {
        std::size_t used = 0;
        const double v = std::stod(it->second, &used);
        if (used != it->second.size() || !std::isfinite(v)) {
            return std::nullopt;
        }
        return v;
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace branes::anatomy::config
