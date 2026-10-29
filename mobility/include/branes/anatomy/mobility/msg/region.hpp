#pragma once
/// @file region.hpp
/// @brief Geofence and StartPose: where a vehicle may fly, and where it starts.
///
/// every mission has
/// a flight volume and a start pose, whatever its task.

#include <branes/anatomy/msg/geometry.hpp>

#include <cstddef>

namespace branes::anatomy {

/// Axis-aligned flight volume in the world frame.
template <typename T>
struct Geofence {
    Vec3<T> min{};
    Vec3<T> max{};

    friend constexpr bool operator==(const Geofence&, const Geofence&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool contains(const Geofence<T>& f, const Vec3<T>& p) noexcept {
    for (std::size_t i = 0; i < 3; ++i) {
        if (!(p[i] >= f.min[i] && p[i] <= f.max[i])) {
            return false;
        }
    }
    return true;
}

/// Start pose on the pad.
template <typename T>
struct StartPose {
    Vec3<T> position{};
    T yaw{0};  ///< [rad]

    friend constexpr bool operator==(const StartPose&, const StartPose&) = default;
};

}  // namespace branes::anatomy
