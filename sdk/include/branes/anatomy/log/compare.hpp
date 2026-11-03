#pragma once
/// @file compare.hpp
/// @brief compare(): two replay logs, their headers field by field and their
/// records byte by byte (docs/plan/flow-graph, 2c).
///
/// Two builds of one baseline (its slot stack and its graph's CPU stack) write
/// logs whose records must be byte-identical, and whose headers differ only in
/// the stack's hash. compare() checks that: the header fields not left out are
/// equal, and the records are, in number, order, channel, time and payload.
/// It reads the records' framing only, so it takes any mission's channels.

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/result.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace branes::anatomy::log {

/// The names of LogHeader's fields, in the order the log stores them.
inline constexpr std::array<std::string_view, 8> kHeaderFields = {"baseline_name",
                                                                  "composition_hash",
                                                                  "world_hash",
                                                                  "vehicle_hash",
                                                                  "scenario_hash",
                                                                  "reference",
                                                                  "world_yaml",
                                                                  "vehicle_yaml"};

/// What compare() found: the number of records compared, and the first
/// difference, if there is one.
struct Comparison {
    std::uint64_t records{0};               ///< records equal in both, before any difference
    std::optional<std::string> difference;  ///< nullopt: the logs are equal
};

/// Compare the logs at `a` and `b`, leaving out the header fields `except`,
/// each one of kHeaderFields. An error is a log that can't be read (as
/// LogReader::open() reads it, or a truncated record), or a name in `except`
/// that isn't a header field; a difference is not an error.
[[nodiscard]] result<Comparison, LogError>
compare(const std::filesystem::path& a, const std::filesystem::path& b, std::span<const std::string_view> except);

}  // namespace branes::anatomy::log
