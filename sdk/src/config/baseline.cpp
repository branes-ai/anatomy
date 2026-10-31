#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/config/yaml.hpp>
#include <branes/anatomy/util/sha256.hpp>

#include <cmath>
#include <exception>
#include <string>

namespace branes::anatomy::config {

using namespace yaml;

// ── Baseline ────────────────────────────────────────────────────────

result<BaselineConfig, ConfigError>
parse_baseline(std::string_view yaml, std::string_view origin, std::span<const std::string_view> slots) {
    return guarded<BaselineConfig>(origin, [&]() -> BaselineConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"name", "composition", "parameters"});
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
            require(!impl.empty() &&
                        impl.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_") ==
                            std::string::npos,
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
    out << YAML::Key << "composition" << YAML::Value << YAML::BeginMap;
    for (const auto& [slot, impl] : b.composition) {
        out << YAML::Key << slot << YAML::Value << impl;
    }
    out << YAML::EndMap;
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
