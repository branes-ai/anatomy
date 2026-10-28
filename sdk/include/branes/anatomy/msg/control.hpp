#pragma once
/// @file control.hpp
/// @brief Reference (planner → controller) and ThrustRates (controller → FC).

#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <array>
#include <cstdint>

namespace branes::anatomy {

inline constexpr std::uint32_t kMaxReferenceSamples = 64;

/// One sample of a planned trajectory, world frame.
template <typename T>
struct ReferenceSample {
    Timestamp t;
    Vec3<T> position{};
    Vec3<T> velocity{};
    Vec3<T> acceleration{};
    T yaw{0};       ///< [rad]
    T progress{0};  ///< the reference's path parameter (MPCC progress variable) [m]

    friend constexpr bool operator==(const ReferenceSample&, const ReferenceSample&) = default;
};

/// A planned trajectory segment.
///
/// Invariants:
///   I1: count ≤ kMaxReferenceSamples
///   I2: sample times strictly increase
///   I3: progress never decreases
template <typename T>
struct Reference {
    std::uint32_t count{0};
    std::array<ReferenceSample<T>, kMaxReferenceSamples> samples{};

    friend constexpr bool operator==(const Reference&, const Reference&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const Reference<T>& r) noexcept {
    if (r.count > kMaxReferenceSamples) {
        return false;  // I1
    }
    for (std::uint32_t i = 1; i < r.count; ++i) {
        if (!(r.samples[i - 1].t < r.samples[i].t)) {
            return false;  // I2
        }
        if (!(r.samples[i - 1].progress <= r.samples[i].progress)) {
            return false;  // I3
        }
    }
    return true;
}

/// A reference in another scalar type, element by element (all samples, used or not).
template <typename To, typename From>
[[nodiscard]] constexpr Reference<To> cast(const Reference<From>& r) noexcept {
    Reference<To> out{};
    out.count = r.count;
    for (std::uint32_t i = 0; i < kMaxReferenceSamples; ++i) {
        const ReferenceSample<From>& s = r.samples[i];
        out.samples[i] = ReferenceSample<To>{
            s.t, cast<To>(s.position), cast<To>(s.velocity), cast<To>(s.acceleration), To(s.yaw), To(s.progress)};
    }
    return out;
}

/// Which fields of a ThrustRates command are in use (ADR 0002).
enum class CommandKind : std::uint8_t {
    thrust_rates = 0,  ///< collective thrust + body rates, through the FC INDI / rate loop
    motors = 1,        ///< per-motor commands, accepted by the FC only in a configured mode
};

/// The command the companion sends to the FC.
///
/// Invariants:
///   I1: thrust ≥ 0
///   I2: every motor command ∈ [0, 1]
template <typename T>
struct ThrustRates {
    Timestamp t;
    CommandKind kind{CommandKind::thrust_rates};
    T thrust{0};                ///< collective thrust [N]
    Vec3<T> body_rates{};       ///< [rad/s]
    std::array<T, 4> motors{};  ///< normalized motor commands (kind == motors)

    friend constexpr bool operator==(const ThrustRates&, const ThrustRates&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const ThrustRates<T>& u) noexcept {
    if (!(u.thrust >= T(0))) {
        return false;  // I1
    }
    for (const T m : u.motors) {
        if (!(m >= T(0) && m <= T(1))) {
            return false;  // I2
        }
    }
    return true;
}

/// A command in another scalar type, element by element.
template <typename To, typename From>
[[nodiscard]] constexpr ThrustRates<To> cast(const ThrustRates<From>& u) noexcept {
    return ThrustRates<To>{u.t, u.kind, To(u.thrust), cast<To>(u.body_rates), cast<To>(u.motors)};
}

}  // namespace branes::anatomy
