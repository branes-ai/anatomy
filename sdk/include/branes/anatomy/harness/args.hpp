#pragma once
/// @file args.hpp
/// @brief Args: `--key value` / `--flag` command-line parsing for the harnesses.
///
///

#include <branes/anatomy/result.hpp>

#include <charconv>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <system_error>

namespace branes::anatomy::harness {

// ── command line ────────────────────────────────────────────────────

/// Parsed `--key value` options and `--flag` switches.
class Args {
public:
    /// `flags` are the switches that take no value; every other `--name`
    /// takes the next argument as its value. Unknown names are accepted here
    /// and rejected by only().
    [[nodiscard]] static result<Args, std::string>
    parse(int argc, const char* const* argv, const std::set<std::string>& flags) {
        Args a;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg.rfind("--", 0) != 0 || arg.size() < 3) {
                return std::string("unexpected argument '" + arg + "'");
            }
            const std::string name = arg.substr(2);
            if (flags.count(name) != 0) {
                a.flags_.insert(name);
            } else if (i + 1 < argc) {
                a.values_[name] = argv[++i];
            } else {
                return std::string("option --" + name + " needs a value");
            }
        }
        return a;
    }

    /// Error text naming the first option outside `allowed`, if any.
    [[nodiscard]] std::optional<std::string> only(const std::set<std::string>& allowed) const {
        for (const auto& [k, v] : values_) {
            if (allowed.count(k) == 0) {
                return "option --" + k + " is not accepted here";
            }
        }
        for (const auto& f : flags_) {
            if (allowed.count(f) == 0) {
                return "option --" + f + " is not accepted here";
            }
        }
        return std::nullopt;
    }

    [[nodiscard]] std::optional<std::string> value(const std::string& name) const {
        const auto it = values_.find(name);
        return it == values_.end() ? std::nullopt : std::optional<std::string>(it->second);
    }
    [[nodiscard]] bool has(const std::string& name) const {
        return values_.count(name) != 0 || flags_.count(name) != 0;
    }

private:
    std::map<std::string, std::string> values_;
    std::set<std::string> flags_;
};

/// The whole of `s` as a number of type N (std::from_chars), or nullopt if it
/// is not one: "10x" and an empty string are rejected.
template <typename N>
[[nodiscard]] std::optional<N> parse_number(const std::string& s) {
    N v{};
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || end != s.data() + s.size()) {
        return std::nullopt;
    }
    return v;
}

}  // namespace branes::anatomy::harness
