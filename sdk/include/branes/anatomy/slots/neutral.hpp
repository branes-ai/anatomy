#pragma once
/// @file neutral.hpp
/// @brief The mission-neutral slot concepts: Controller, SafetyFilter,
/// DaaTracker, FcBridge.
///
/// none of them sees
/// the mission's task. A mission adds the slots that do (racing-drone: slots.hpp).
///
/// Every slot is templated on the scalar type T (cortex ADR 0015).

#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/daa.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/fc.hpp>
#include <branes/anatomy/msg/intruders.hpp>
#include <branes/anatomy/msg/region.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <concepts>
#include <optional>

namespace branes::anatomy {

/// Tracks the reference: MPCC, a learned policy, or PID.
template <class C, class T>
concept Controller =
    requires(C c, const StateEstimate<T>& x, const Reference<T>& ref, const ConflictSet<T>& conflicts) {
        { c.step(x, ref, conflicts) } -> std::same_as<ThrustRates<T>>;
    };

/// Last-resort command filter: CBF separation, geofence, envelope.
template <class F, class T>
concept SafetyFilter = requires(F f,
                                const StateEstimate<T>& x,
                                const ThrustRates<T>& u,
                                const ConflictSet<T>& conflicts,
                                const Geofence<T>& fence) {
    { f.filter(x, u, conflicts, fence) } -> std::same_as<ThrustRates<T>>;
};

/// Tracks intruders and predicts conflicts over the control horizon.
template <class A, class T>
concept DaaTracker = requires(A a, const A& ca, const IntruderDetections<T>& det, const StateEstimate<T>& x) {
    a.update(det, x);
    { ca.conflicts(x) } -> std::same_as<ConflictSet<T>>;
};

/// The companion's link to the flight controller.
template <class B, class T>
concept FcBridge = requires(B b, const B& cb, const ThrustRates<T>& u, Timestamp now) {
    b.send(u, now);                                                 // also serves as the companion heartbeat (SAF-01)
    { b.poll_imu() } -> std::same_as<std::optional<ImuSample<T>>>;  // FC-relayed, FC-timestamped
    { cb.fc_status() } -> std::same_as<FcStatus>;
};

}  // namespace branes::anatomy
