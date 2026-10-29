#pragma once
/// @file intruders.hpp
/// @brief IntruderDetections: what perception hands the DAA tracker.
///
/// Fixed capacity, so
/// a cycle never allocates (SW-01).

#include <branes/anatomy/msg/geometry.hpp>

#include <array>
#include <cstdint>

namespace branes::anatomy {

inline constexpr std::uint32_t kMaxDetections = 8;

/// One intruder (another drone or obstacle) seen in one image.
template <typename T>
struct IntruderDetection {
    Vec3<T> bearing{};  ///< unit direction in the camera frame
    T angular_size{0};  ///< apparent size [rad]
    T range{0};         ///< range estimate [m], from the known intruder size
    T range_sigma{0};   ///< 1σ range uncertainty [m]

    friend constexpr bool operator==(const IntruderDetection&, const IntruderDetection&) = default;
};

/// The intruders seen in one image. Invariants: I1 count ≤ kMaxDetections.
template <typename T>
struct IntruderDetections {
    std::uint32_t count{0};
    std::array<IntruderDetection<T>, kMaxDetections> detections{};

    friend constexpr bool operator==(const IntruderDetections&, const IntruderDetections&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const IntruderDetections<T>& d) noexcept {
    return d.count <= kMaxDetections;  // I1
}

}  // namespace branes::anatomy
