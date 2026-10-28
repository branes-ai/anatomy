#pragma once
/// @file baseline.hpp
/// @brief The baseline file: type, loader, emitter, composition hash.
///
/// Every loader validates the
/// whole file before it returns a value, so a config that loads satisfies all
/// the invariants listed on its type. Loading never throws; it returns
/// result<X, ConfigError>. Every emitter writes a file that loads back to an
/// equal value (round trip), with doubles written to 17 significant digits.
///
/// The mission supplies its slot list (racing-drone: config.hpp, kSlotNames).

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/result.hpp>

#include <filesystem>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace branes::anatomy::config {

// ── Baseline ────────────────────────────────────────────────────────

/// baselines/<name>.yaml.
///
/// Invariants: name matches [a-z0-9-]+; composition names exactly the
/// mission's slots, each with an implementation name [A-Za-z0-9_]+;
/// parameters are scalar values.
struct BaselineConfig {
    std::string name;
    std::map<std::string, std::string> composition;  ///< slot → implementation
    std::map<std::string, std::string> parameters;   ///< key → scalar value

    friend bool operator==(const BaselineConfig&, const BaselineConfig&) = default;
};

/// `slots` is the mission's slot list (racing-drone: config.hpp, kSlotNames).
/// Pre: `slots` is sorted, without repeats, and every name matches [a-z_]+.
[[nodiscard]] result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path,
                                                                std::span<const std::string_view> slots);
[[nodiscard]] result<BaselineConfig, ConfigError>
parse_baseline(std::string_view yaml, std::string_view origin, std::span<const std::string_view> slots);
[[nodiscard]] std::string emit_baseline(const BaselineConfig& b);

/// SHA-256 (hex) of the canonical composition "slot=Impl\n", in the order of
/// `slots` (sorted) — identical to the hash cmake/baselines.cmake compiles into
/// each binary. Pre: as for parse_baseline.
[[nodiscard]] std::string composition_hash(const BaselineConfig& b, std::span<const std::string_view> slots);

/// A numeric parameter, or nullopt if absent or not a number.
[[nodiscard]] std::optional<double> numeric_parameter(const BaselineConfig& b, std::string_view key);

/// The control rate a baseline runs at when it gives no `control_rate_hz` [Hz]:
/// the apps tick the stack at it, and a slot that needs the period reads it.
inline constexpr double kDefaultControlRateHz = 100.0;

}  // namespace branes::anatomy::config
