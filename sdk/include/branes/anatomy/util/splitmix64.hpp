#pragma once
/// @file splitmix64.hpp
/// @brief SplitMix64: the deterministic random source of the SITL harness.
///
/// std::*_distribution output is implementation-defined, so it differs between
/// standard libraries; scenario randomization uses this generator and explicit
/// transforms instead, so a seed means the same thing on every platform.
/// Algorithm: Steele, Lea, Flood, "Fast splittable pseudorandom number
/// generators" (OOPSLA 2014), the public-domain reference constants.
///
/// A run draws from several streams, each seeded by stream_seed() from the
/// scenario's seed and the stream's own constant, so adding draws to one stream
/// never shifts another. The simulator's streams are the IMU, the
/// randomization and the camera (sim/basic_simulator.hpp). A mission that
/// draws too uses a stream with a constant of its own (scout-drone: the
/// obstacles of a generated field).

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

/// The seed of the random stream `stream` of a run seeded `seed`. Streams with
/// different constants are independent of each other.
[[nodiscard]] constexpr std::uint64_t stream_seed(std::uint64_t seed, std::uint64_t stream) noexcept {
    SplitMix64 mix(seed ^ (stream * 0x9E3779B97F4A7C15ull));
    return mix.next();
}

}  // namespace branes::anatomy
