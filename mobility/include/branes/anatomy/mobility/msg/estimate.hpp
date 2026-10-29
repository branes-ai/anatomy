#pragma once
/// @file estimate.hpp
/// @brief StateEstimate and EstimatorHealth: the estimator slot's outputs.

#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace branes::anatomy {

/// Size of the error state: δp, δv, δθ, δb_a, δb_g (3 each).
inline constexpr std::size_t kErrorStateDim = 15;

/// The vehicle state in the world frame, valid at `t_valid` (EST-01, EST-06).
///
/// Invariants:
///   I1: q is a unit quaternion (is_unit)
///   I2: covariance is symmetric with a non-negative diagonal
template <typename T>
struct StateEstimate {
    Timestamp t_valid;
    Vec3<T> position{};                                           ///< p, world frame [m]
    Vec3<T> velocity{};                                           ///< v, world frame [m/s]
    Quat<T> attitude{};                                           ///< q, body → world
    Vec3<T> body_rates{};                                         ///< ω, body frame [rad/s]
    Vec3<T> accel_bias{};                                         ///< b_a [m/s²]
    Vec3<T> gyro_bias{};                                          ///< b_g [rad/s]
    std::array<T, kErrorStateDim * kErrorStateDim> covariance{};  ///< row-major

    friend constexpr bool operator==(const StateEstimate&, const StateEstimate&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const StateEstimate<T>& x) noexcept {
    if (!is_unit(x.attitude)) {
        return false;  // I1
    }
    for (std::size_t r = 0; r < kErrorStateDim; ++r) {
        if (!(x.covariance[r * kErrorStateDim + r] >= T(0))) {
            return false;  // I2: diagonal
        }
        for (std::size_t c = r + 1; c < kErrorStateDim; ++c) {
            if (x.covariance[r * kErrorStateDim + c] != x.covariance[c * kErrorStateDim + r]) {
                return false;  // I2: symmetry
            }
        }
    }
    return true;
}

/// A state estimate in another scalar type, element by element.
template <typename To, typename From>
[[nodiscard]] constexpr StateEstimate<To> cast(const StateEstimate<From>& x) noexcept {
    return StateEstimate<To>{x.t_valid,
                             cast<To>(x.position),
                             cast<To>(x.velocity),
                             cast<To>(x.attitude),
                             cast<To>(x.body_rates),
                             cast<To>(x.accel_bias),
                             cast<To>(x.gyro_bias),
                             cast<To>(x.covariance)};
}

/// Estimator self-assessment, consumed by the health ladder (SAF-03).
template <typename T>
struct EstimatorHealth {
    T nis{0};                      ///< normalized innovation squared of the last update
    std::int64_t staleness_ns{0};  ///< time since the last accepted vision update
    bool ok{false};                ///< estimator's own verdict

    friend constexpr bool operator==(const EstimatorHealth&, const EstimatorHealth&) = default;
};

}  // namespace branes::anatomy
