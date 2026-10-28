#pragma once
/// @file splitmix64.hpp
/// @brief SplitMix64: the deterministic random source of the SITL harness.
///
/// std::*_distribution output is implementation-defined, so it differs between
/// standard libraries; scenario randomization uses this generator and explicit
/// transforms instead, so a seed means the same thing on every platform.
/// Algorithm: Steele, Lea, Flood, "Fast splittable pseudorandom number
/// generators" (OOPSLA 2014), the public-domain reference constants.

#include <cstdint>

namespace branes::anatomy {

class SplitMix64 {
public:
    explicit constexpr SplitMix64(std::uint64_t seed) noexcept : state_(seed) {}

    constexpr std::uint64_t next() noexcept {
        std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    /// Uniform in [0, 1), from the top 53 bits.
    constexpr double uniform01() noexcept {
        return static_cast<double>(next() >> 11) * 0x1.0p-53;
    }

    /// Uniform in [−a, a).
    constexpr double symmetric(double a) noexcept {
        return a * (2.0 * uniform01() - 1.0);
    }

private:
    std::uint64_t state_;
};

}  // namespace branes::anatomy
