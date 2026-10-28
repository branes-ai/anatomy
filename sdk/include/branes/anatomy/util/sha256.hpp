#pragma once
/// @file sha256.hpp
/// @brief SHA-256 (FIPS 180-4), for the composition and config-file hashes in the log header.

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace branes::anatomy {

using Sha256Digest = std::array<std::uint8_t, 32>;

[[nodiscard]] Sha256Digest sha256(std::span<const std::byte> data) noexcept;

/// Lowercase hex of a digest (64 characters).
[[nodiscard]] std::string to_hex(const Sha256Digest& d);

/// Hex SHA-256 of a string's bytes.
[[nodiscard]] std::string sha256_hex(std::string_view s);

/// Hex SHA-256 of a file's bytes, or nullopt if it cannot be read.
[[nodiscard]] std::optional<std::string> sha256_file_hex(const std::filesystem::path& path);

}  // namespace branes::anatomy
