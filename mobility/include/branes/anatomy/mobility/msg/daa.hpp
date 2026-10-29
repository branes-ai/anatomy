#pragma once
/// @file daa.hpp
/// @brief ConflictSet: the DAA tracker's predictions, consumed by the controller and the safety filter.
///
/// racing-drone docs/arch/framework-architecture.md §8 defines how the safety filter treats
/// a set by its age and `tracker_ok`.

#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <array>
#include <cstdint>

namespace branes::anatomy {

inline constexpr std::uint32_t kMaxConflicts = 8;

/// One tracked intruder and its predicted conflict with the ownship.
template <typename T>
struct Conflict {
    std::int32_t intruder_id{-1};
    Vec3<T> position{};    ///< world frame [m], at the set's valid_at
    Vec3<T> velocity{};    ///< world frame [m/s]
    T tube_radius{0};      ///< prediction uncertainty radius [m]
    T time_to_closest{0};  ///< [s]
    T min_separation{0};   ///< predicted minimum separation [m]

    friend constexpr bool operator==(const Conflict&, const Conflict&) = default;
};

/// Invariants: I1 count ≤ kMaxConflicts.
template <typename T>
struct ConflictSet {
    Timestamp valid_at;
    bool tracker_ok{false};
    std::uint32_t count{0};
    std::array<Conflict<T>, kMaxConflicts> conflicts{};

    friend constexpr bool operator==(const ConflictSet&, const ConflictSet&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const ConflictSet<T>& c) noexcept {
    return c.count <= kMaxConflicts;  // I1
}

}  // namespace branes::anatomy
