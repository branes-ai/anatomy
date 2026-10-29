#pragma once
/// @file neutral_nulls.hpp
/// @brief Null implementations of the mission-neutral slots.
///
/// Stateless and
/// deterministic: each returns a valid, fixed output derived only from its
/// inputs, so a mission's null baseline exercises the framework end to end.

#include <branes/anatomy/mobility/slots/neutral.hpp>

#include <optional>

namespace branes::anatomy {

/// Commands zero thrust and zero body rates (motors stay idle).
template <typename T>
class NullController {
public:
    [[nodiscard]] ThrustRates<T>
    step(const StateEstimate<T>& x, const Reference<T>&, const ConflictSet<T>&) const noexcept {
        ThrustRates<T> u{};
        u.t = x.t_valid;
        return u;
    }
};

/// Passes the command through unchanged.
template <typename T>
class NullSafetyFilter {
public:
    [[nodiscard]] ThrustRates<T>
    filter(const StateEstimate<T>&, const ThrustRates<T>& u, const ConflictSet<T>&, const Geofence<T>&) const noexcept {
        return u;
    }
};

/// No DAA (DaaTracker "None"): never reports a conflict. A fresh, empty set
/// with tracker_ok, so the safety filter sees "no known intruders".
template <typename T>
class NullDaaTracker {
public:
    void update(const IntruderDetections<T>&, const StateEstimate<T>&) noexcept {}
    [[nodiscard]] ConflictSet<T> conflicts(const StateEstimate<T>& x) const noexcept {
        ConflictSet<T> c{};
        c.valid_at = x.t_valid;
        c.tracker_ok = true;
        return c;
    }
};

/// Discards commands and relays no IMU samples; reports a disarmed FC.
template <typename T>
class NullFcBridge {
public:
    void send(const ThrustRates<T>&, Timestamp) noexcept {}
    [[nodiscard]] std::optional<ImuSample<T>> poll_imu() noexcept {
        return std::nullopt;
    }
    [[nodiscard]] FcStatus fc_status() const noexcept {
        return FcStatus{};
    }
};

}  // namespace branes::anatomy
