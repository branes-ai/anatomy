#pragma once
// A made-up watch post with its own stack profile (anatomy
// docs/plan/generalize-core): its truth is two targets, its one input is a
// range measurement on a channel of its own, and each tick produces an
// assessment. Shared by tests/sdk/profile.cpp, which drives it through the
// harness, and tests/mobility/mobility_harness.cpp, which checks that the
// mobility-only loops refuse it.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace watch {

using branes::anatomy::Output;
using branes::anatomy::Timestamp;
using branes::anatomy::Vec3;
namespace log = branes::anatomy::log;

constexpr log::Channel kRange = static_cast<log::Channel>(8);        ///< input: one range measurement
constexpr log::Channel kAssessment = static_cast<log::Channel>(19);  ///< output: one assessment per tick

/// The scene's truth: two targets.
template <typename T>
struct SceneTruth {
    Timestamp t;
    std::array<Vec3<T>, 2> targets{};

    friend constexpr bool operator==(const SceneTruth&, const SceneTruth&) = default;
};

template <typename T>
[[nodiscard]] bool valid(const SceneTruth<T>& x) noexcept {
    for (const auto& p : x.targets) {
        for (const T v : p) {
            if (!std::isfinite(v)) {
                return false;
            }
        }
    }
    return true;
}

template <typename To, typename From>
[[nodiscard]] SceneTruth<To> cast(const SceneTruth<From>& x) noexcept {
    return SceneTruth<To>{x.t, {branes::anatomy::cast<To>(x.targets[0]), branes::anatomy::cast<To>(x.targets[1])}};
}

/// A range measurement [m]. Invariant: finite and ≥ 0.
template <typename T>
struct Range {
    T r{};
};

template <typename T>
[[nodiscard]] bool valid(const Range<T>& m) noexcept {
    return std::isfinite(m.r) && m.r >= T(0);
}

/// What one tick assesses: how many ranges it saw, the last one, and how far
/// apart the true targets were (SITL).
template <typename T>
struct Assessment {
    std::uint32_t measurements{0};
    T last_range{};
    T spread{};

    friend constexpr bool operator==(const Assessment&, const Assessment&) = default;
};

std::vector<std::byte> encode(const SceneTruth<double>& x) {
    branes::anatomy::ByteWriter w;
    w.i64(x.t.ns);
    for (const auto& p : x.targets) {
        for (const double v : p) {
            w.f64(v);
        }
    }
    return w.take();
}

std::vector<std::byte> encode(const Range<double>& m) {
    branes::anatomy::ByteWriter w;
    w.f64(m.r);
    return w.take();
}

std::optional<Assessment<double>> decode_assessment(std::span<const std::byte> b) {
    branes::anatomy::ByteReader r(b);
    Assessment<double> a;
    a.measurements = r.u32();
    a.last_range = r.f64();
    a.spread = r.f64();
    if (r.failed() || !r.done()) {
        return std::nullopt;
    }
    return a;
}

/// The watch post's profile.
struct WatchProfile {
    template <typename T>
    using truth_type = SceneTruth<T>;
    template <typename T>
    using output_type = Assessment<T>;

    static constexpr std::array<log::Channel, 1> inputs = {kRange};
    static constexpr std::array<log::Channel, 1> outputs = {kAssessment};

    template <class S>
    static constexpr bool drives = requires(S s, const Range<typename S::scalar_type>& m) { s.on_range(m); };

    [[nodiscard]] static std::optional<SceneTruth<double>> decode_truth(std::span<const std::byte> b) {
        branes::anatomy::ByteReader r(b);
        SceneTruth<double> x;
        x.t = Timestamp{r.i64()};
        for (auto& p : x.targets) {
            for (double& v : p) {
                v = r.f64();
            }
        }
        if (r.failed() || !r.done()) {
            return std::nullopt;
        }
        return x;
    }

    template <class S>
    [[nodiscard]] static branes::anatomy::result<branes::anatomy::ok_t, std::string> feed(S& s,
                                                                                          const log::LogRecord& rec) {
        using T = typename S::scalar_type;
        branes::anatomy::ByteReader r(rec.payload);
        const double v = r.f64();
        if (r.failed() || !r.done()) {
            return std::string("undecodable range record at t=" + std::to_string(rec.t.ns));
        }
        const Range<T> m{static_cast<T>(v)};
        if (!valid(m)) {
            return std::string("range record at t=" + std::to_string(rec.t.ns) +
                               " is not valid in the stack's scalar type");
        }
        s.on_range(m);
        return branes::anatomy::ok;
    }

    template <typename T>
    static void encode(const Assessment<T>& y, Timestamp t, std::vector<Output>& out) {
        branes::anatomy::ByteWriter w;
        w.u32(y.measurements);
        w.f64(static_cast<double>(y.last_range));
        w.f64(static_cast<double>(y.spread));
        out.push_back(Output{kAssessment, t, w.take()});
    }
};

/// The watch post's stack: it counts ranges and reads the truth.
template <typename T>
class WatchStack {
public:
    using scalar_type = T;
    using profile = WatchProfile;

    void on_range(const Range<T>& m) {
        ++seen_.measurements;
        seen_.last_range = m.r;
    }
    void on_truth(const SceneTruth<T>& x) {
        T d2 = T(0);
        for (std::size_t i = 0; i < 3; ++i) {
            const T d = x.targets[0][i] - x.targets[1][i];
            d2 += d * d;
        }
        seen_.spread = std::sqrt(d2);
    }
    Assessment<T> on_tick(Timestamp) {
        return seen_;
    }

private:
    Assessment<T> seen_{};
};

}  // namespace watch
