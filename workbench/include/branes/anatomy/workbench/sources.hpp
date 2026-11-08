#pragma once
/// @file sources.hpp
/// @brief The workbench's random numbers, and its sources: an input's
/// messages at a fixed period (docs/plan/workbench, "The workbench").
///
/// The randomness is std::mt19937_64's raw output, whose sequence the standard
/// fixes, mapped to ranges here: the standard distributions differ between
/// GCC, Clang and MSVC, and a seed must give one run on all three. A normal
/// deviate uses std::log, std::sqrt and std::cos, so it is the same to within
/// each library's accuracy, not bit for bit, across compilers (W3).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <cmath>
#include <cstdint>
#include <functional>
#include <numbers>
#include <random>
#include <string>
#include <utility>

namespace branes::anatomy::workbench {

/// A seeded generator of uniform and normal deviates.
class rng {
public:
    explicit rng(std::uint64_t seed) : e_(seed) {}

    [[nodiscard]] std::uint64_t next() {
        return e_();
    }
    /// Uniform in [0, 1), from the top 53 bits.
    [[nodiscard]] double unit() {
        return static_cast<double>(e_() >> 11) * 0x1.0p-53;
    }
    /// Uniform in [lo, hi).
    [[nodiscard]] double uniform(double lo, double hi) {
        return lo + (hi - lo) * unit();
    }
    /// Standard normal (Box–Muller, one deviate per two draws).
    [[nodiscard]] double normal() {
        const double u = 1.0 - unit();  // (0, 1], so the log is finite
        const double v = unit();
        return std::sqrt(-2.0 * std::log(u)) * std::cos(2.0 * std::numbers::pi * v);
    }

private:
    std::mt19937_64 e_;
};

/// A seed for stream `k` of seed `seed`: so the k-th message of a source can
/// be drawn without drawing the k before it.
[[nodiscard]] constexpr std::uint64_t substream(std::uint64_t seed, std::uint64_t k) noexcept {
    std::uint64_t z = seed + 0x9E3779B97F4A7C15ULL * (k + 1);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

/// An input's messages, one every `period_ns` from `start_ns`: the k-th is
/// handed to the stack at start_ns + k·period_ns.
///
/// Invariant: period_ns > 0, and `hand` hands the stack one message of its input.
struct source {
    std::string name;  ///< the input's
    std::int64_t period_ns = 0;
    std::int64_t start_ns = 0;
    std::function<void(flow::stack&, std::uint64_t k, Timestamp t)> hand;
};

/// Input `in`'s k-th message is make(k, t).
template <class M, class F>
[[nodiscard]] source
sequence(const flow::input_port<M>& in, std::int64_t period_ns, F make, std::int64_t start_ns = 0) {
    return source{
        in.ref.port, period_ns, start_ns, [in, make = std::move(make)](flow::stack& s, std::uint64_t k, Timestamp t) {
            s.input(in, make(k, t));
        }};
}

/// A tick input, at its own times.
[[nodiscard]] inline source
clock(const flow::input_port<Timestamp>& in, std::int64_t period_ns, std::int64_t start_ns = 0) {
    return sequence(in, period_ns, [](std::uint64_t, Timestamp t) { return t; }, start_ns);
}

}  // namespace branes::anatomy::workbench
