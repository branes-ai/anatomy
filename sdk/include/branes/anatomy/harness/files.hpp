#pragma once
/// @file files.hpp
/// @brief The baseline check of ADR 0001, and file contents with their hash for log headers.
///
/// The mission
/// supplies its slot list (racing-drone: harness.hpp).

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/util/sha256.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

namespace branes::anatomy::harness {

// ── baseline check ──────────────────────────────────────────────────

/// Load `<dir>/<name>.yaml` and confirm it matches the binary (ADR 0001):
/// same name, same composition hash over the mission's `slots`. The binary
/// refuses to start otherwise. Pre: as for config::parse_baseline.
[[nodiscard]] inline result<config::BaselineConfig, std::string>
check_baseline(const std::filesystem::path& dir,
               std::string_view name,
               std::string_view compiled_hash,
               std::span<const std::string_view> slots) {
    const auto path = dir / (std::string(name) + ".yaml");
    auto b = config::load_baseline(path, slots);
    if (!b) {
        return b.error().to_string();
    }
    if (b->name != name) {
        return std::string(path.string() + ": baseline name '" + b->name + "' does not match the binary ('" +
                           std::string(name) + "')");
    }
    if (const std::string h = config::composition_hash(*b, slots); h != compiled_hash) {
        return std::string(path.string() + ": composition changed since this binary was built (hash " + h +
                           ", built with " + std::string(compiled_hash) + "); rebuild");
    }
    return std::move(b).value();
}

/// A file's text and SHA-256, for embedding in a log header.
struct FileContent {
    std::string text;
    std::string hash;
};

[[nodiscard]] inline std::optional<FileContent> read_with_hash(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    std::string hash = sha256_hex(text);
    return FileContent{std::move(text), std::move(hash)};
}

}  // namespace branes::anatomy::harness
