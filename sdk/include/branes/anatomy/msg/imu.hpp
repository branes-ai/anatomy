#pragma once
/// @file imu.hpp
/// @brief ImuSample: one IMU sample in the body frame.

#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <cstdint>

namespace branes::anatomy {

/// Saturation flags of an ImuSample: one bit per axis per sensor.
enum ImuSaturation : std::uint8_t {
    gyro_x_saturated = 1u << 0,
    gyro_y_saturated = 1u << 1,
    gyro_z_saturated = 1u << 2,
    accel_x_saturated = 1u << 3,
    accel_y_saturated = 1u << 4,
    accel_z_saturated = 1u << 5,
};
inline constexpr std::uint8_t imu_saturation_mask = 0x3F;

/// One IMU sample in the body frame, timestamped on the companion clock.
///
/// Invariants: I1 saturated ⊆ imu_saturation_mask.
template <typename T>
struct ImuSample {
    Timestamp t;
    Vec3<T> gyro{};             ///< angular rate [rad/s]
    Vec3<T> accel{};            ///< specific force [m/s²]
    std::uint8_t saturated{0};  ///< ImuSaturation bits
    T temperature{0};           ///< sensor temperature [°C]

    friend constexpr bool operator==(const ImuSample&, const ImuSample&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const ImuSample<T>& s) noexcept {
    return (s.saturated & ~imu_saturation_mask) == 0;  // I1
}

/// An IMU sample in another scalar type, element by element.
template <typename To, typename From>
[[nodiscard]] constexpr ImuSample<To> cast(const ImuSample<From>& s) noexcept {
    return ImuSample<To>{s.t, cast<To>(s.gyro), cast<To>(s.accel), s.saturated, To(s.temperature)};
}

}  // namespace branes::anatomy
