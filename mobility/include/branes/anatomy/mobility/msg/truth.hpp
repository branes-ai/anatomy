#pragma once
/// @file truth.hpp
/// @brief TruthState: the simulated vehicle's true state (SITL only).
///
/// The SITL harness logs it on the `truth` channel for scoring. It is never a
/// flight input (SYS-02): only SITL-only slots, such as the ground-truth
/// estimator of baseline B0, may read it.

#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <array>

namespace branes::anatomy {

/// The true vehicle state at `t`, in the world frame (z up).
///
/// Invariants: I1 attitude is a unit quaternion (is_unit); I2 rotor speeds ≥ 0.
template <typename T>
struct TruthState {
    Timestamp t;
    Vec3<T> position{};               ///< world frame [m]
    Vec3<T> velocity{};               ///< world frame [m/s]
    Quat<T> attitude{};               ///< body → world
    Vec3<T> body_rates{};             ///< body frame [rad/s]
    std::array<T, 4> rotor_speeds{};  ///< [rad/s]
    bool on_ground{false};

    friend constexpr bool operator==(const TruthState&, const TruthState&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const TruthState<T>& x) noexcept {
    if (!is_unit(x.attitude)) {
        return false;  // I1
    }
    for (const T w : x.rotor_speeds) {
        if (!(w >= T(0))) {
            return false;  // I2
        }
    }
    return true;
}

/// A true state in another scalar type, element by element.
template <typename To, typename From>
[[nodiscard]] constexpr TruthState<To> cast(const TruthState<From>& x) noexcept {
    return TruthState<To>{x.t,
                          cast<To>(x.position),
                          cast<To>(x.velocity),
                          cast<To>(x.attitude),
                          cast<To>(x.body_rates),
                          cast<To>(x.rotor_speeds),
                          x.on_ground};
}

}  // namespace branes::anatomy
