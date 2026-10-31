#pragma once
/// @file yaml.hpp
/// @brief YAML parsing and emitting helpers for config loaders: anatomy's and a mission's.
///
/// The helpers every loader shares: a key path for each error, unknown keys
/// rejected, numbers checked finite, and doubles emitted to 17 significant
/// digits so a file round-trips. Anatomy's own loaders use them, and so does a
/// mission's loader for its world file (racing-drone: the course; scout-drone:
/// the field). They are a library of their own, branes::anatomy_yaml, so
/// yaml-cpp stays out of the core's and the profiles' public headers.
///
/// Parsing uses exceptions internally (yaml-cpp throws); every public entry
/// point catches them with guarded() and returns a ConfigError, so the API
/// never throws.

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>

namespace branes::anatomy::config::yaml {

struct Fail {
    std::string key;
    std::string message;
};

inline std::string join(std::string_view prefix, std::string_view key) {
    return prefix.empty() ? std::string(key) : std::string(prefix) + "." + std::string(key);
}

inline YAML::Node child(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = n[key];
    if (!c) {
        throw Fail{join(prefix, key), "missing"};
    }
    return c;
}

/// Reject keys outside `allowed`, so a typo is an error rather than a silently
/// ignored setting.
inline void only_keys(const YAML::Node& n, std::string_view prefix, std::span<const std::string_view> allowed) {
    if (!n.IsMap()) {
        throw Fail{std::string(prefix), "expected a mapping"};
    }
    for (const auto& kv : n) {
        const auto key = kv.first.as<std::string>();
        bool known = false;
        for (const auto a : allowed) {
            known = known || key == a;
        }
        if (!known) {
            throw Fail{join(prefix, key), "unknown key"};
        }
    }
}

inline void only_keys(const YAML::Node& n, std::string_view prefix, std::initializer_list<std::string_view> allowed) {
    only_keys(n, prefix, std::span<const std::string_view>(allowed.begin(), allowed.size()));
}

inline double as_double(const YAML::Node& n, const std::string& where) {
    if (!n.IsScalar()) {
        throw Fail{where, "expected a number"};
    }
    try {
        const double v = n.as<double>();
        if (!std::isfinite(v)) {
            throw Fail{where, "must be finite"};
        }
        return v;
    } catch (const YAML::Exception&) {
        throw Fail{where, "expected a number"};
    }
}

inline double get_double(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    return as_double(child(n, prefix, key), join(prefix, key));
}

/// An optional number: `fallback` if the key is absent.
inline double get_double_or(const YAML::Node& n, std::string_view prefix, const std::string& key, double fallback) {
    const YAML::Node c = n[key];
    return c ? as_double(c, join(prefix, key)) : fallback;
}

inline std::uint64_t get_uint(const YAML::Node& n, std::string_view prefix, const std::string& key, std::uint64_t max) {
    const YAML::Node c = child(n, prefix, key);
    const std::string where = join(prefix, key);
    if (!c.IsScalar()) {
        throw Fail{where, "expected a non-negative integer"};
    }
    const auto text = c.as<std::string>();
    if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
        throw Fail{where, "expected a non-negative integer"};
    }
    try {
        const std::uint64_t v = c.as<std::uint64_t>();
        if (v > max) {
            throw Fail{where, "out of range"};
        }
        return v;
    } catch (const YAML::Exception&) {
        throw Fail{where, "out of range"};
    }
}

inline std::int64_t get_int64(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = child(n, prefix, key);
    const std::string where = join(prefix, key);
    try {
        return c.as<std::int64_t>();
    } catch (const YAML::Exception&) {
        throw Fail{where, "expected an integer"};
    }
}

inline std::string get_string(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = child(n, prefix, key);
    if (!c.IsScalar()) {
        throw Fail{join(prefix, key), "expected a string"};
    }
    return c.as<std::string>();
}

template <std::size_t N>
std::array<double, N> get_array(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = child(n, prefix, key);
    const std::string where = join(prefix, key);
    if (!c.IsSequence() || c.size() != N) {
        throw Fail{where, "expected a list of " + std::to_string(N) + " numbers"};
    }
    std::array<double, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        out[i] = as_double(c[i], where + "[" + std::to_string(i) + "]");
    }
    return out;
}

inline Quat<double> get_quat(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const auto a = get_array<4>(n, prefix, key);
    const Quat<double> q{a[0], a[1], a[2], a[3]};
    if (!is_unit(q)) {
        throw Fail{join(prefix, key), "must be a unit quaternion (w, x, y, z)"};
    }
    return q;
}

inline std::string rate_message() {
    return "must be in [" + std::to_string(kMinRateHz) + ", " + std::to_string(kMaxRateHz) + "] Hz";
}

inline void require(bool condition, std::string key, std::string message) {
    if (!condition) {
        throw Fail{std::move(key), std::move(message)};
    }
}

inline YAML::Node parse_root(std::string_view yaml) {
    try {
        YAML::Node root = YAML::Load(std::string(yaml));
        if (!root.IsMap()) {
            throw Fail{"", "the file must be a YAML mapping"};
        }
        return root;
    } catch (const YAML::ParserException& e) {
        throw Fail{"", std::string("YAML syntax error: ") + e.what()};
    }
}

/// Run `body`, translating every failure into a ConfigError for `origin`.
template <typename T, typename F>
result<T, ConfigError> guarded(std::string_view origin, F&& body) {
    try {
        return body();
    } catch (const Fail& f) {
        return ConfigError{std::string(origin), f.key, f.message};
    } catch (const YAML::Exception& e) {
        return ConfigError{std::string(origin), "", e.what()};
    }
}

inline std::optional<std::string> read_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

template <typename T, typename Parse>
result<T, ConfigError> load_file(const std::filesystem::path& path, Parse&& parse) {
    const auto text = read_file(path);
    if (!text) {
        return ConfigError{path.string(), "", "cannot read the file"};
    }
    return parse(*text, path.string());
}

// ── emitting helpers ────────────────────────────────────────────────

inline YAML::Emitter& emit_seq(YAML::Emitter& out, std::span<const double> values) {
    out << YAML::Flow << YAML::BeginSeq;
    for (const double v : values) {
        out << v;
    }
    return out << YAML::EndSeq;
}

inline YAML::Emitter& emit_quat(YAML::Emitter& out, const Quat<double>& q) {
    const std::array<double, 4> a = {q.w, q.x, q.y, q.z};
    return emit_seq(out, a);
}

inline std::string finish(YAML::Emitter& out) {
    return std::string(out.c_str()) + "\n";
}

}  // namespace branes::anatomy::config::yaml
