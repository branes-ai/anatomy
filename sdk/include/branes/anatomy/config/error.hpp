#pragma once
/// @file error.hpp
/// @brief ConfigError: what a config loader reports. Mission-neutral.

#include <string>

namespace branes::anatomy::config {

/// What went wrong, and where.
struct ConfigError {
    std::string file;  ///< file or origin name
    std::string key;   ///< YAML key path, e.g. "camera.fx" (empty if not key-specific)
    std::string message;

    [[nodiscard]] std::string to_string() const {
        return file + (key.empty() ? "" : ": " + key) + ": " + message;
    }
};

}  // namespace branes::anatomy::config
