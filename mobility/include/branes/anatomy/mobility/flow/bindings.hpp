#pragma once
/// @file bindings.hpp
/// @brief The bindings of the mission-neutral composites: the CPU calling
/// convention of their operators (anatomy docs/adr/0004, step 2;
/// docs/plan/flow-graph, 2b).
///
/// One binding per slot concept of slots/neutral.hpp, over its signature
/// (flow/signatures.hpp). Each handler calls the method the concept names, so
/// every implementation of the slot is an operator of the composite,
/// unchanged. A binding declares the handlers a graph fires, and every
/// composite's handler must fire (G6), so a port no graph reads yet has none:
/// the FC bridge's poll_imu() and fc_status().
///
/// The controller, the safety filter and the DAA tracker's update may keep
/// state, so their handlers declare an update: a body of the composite may
/// then update its operators' state there (G5).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/mobility/flow/signatures.hpp>
#include <branes/anatomy/msg/time.hpp>

namespace branes::anatomy::flow {

/// Controller: step(estimate, reference, conflicts) → command.
template <class T>
struct ControllerBinding {
    using signature = controller_signature<T>;

    struct step : handler<"step", ins<"estimate", "reference", "conflicts">, outs<"command">, state_role::update> {
        static ThrustRates<T>
        call(auto& ctl, const StateEstimate<T>& x, const Reference<T>& ref, const ConflictSet<T>& conflicts) {
            return ctl.step(x, ref, conflicts);
        }
    };

    using handlers = flow::handlers<step>;
};

/// SafetyFilter: filter(estimate, command, conflicts, geofence) → safe_command.
template <class T>
struct SafetyFilterBinding {
    using signature = safety_filter_signature<T>;

    struct filter : handler<"filter",
                            ins<"estimate", "command", "conflicts", "geofence">,
                            outs<"safe_command">,
                            state_role::update> {
        static ThrustRates<T> call(auto& safe,
                                   const StateEstimate<T>& x,
                                   const ThrustRates<T>& u,
                                   const ConflictSet<T>& conflicts,
                                   const Geofence<T>& fence) {
            return safe.filter(x, u, conflicts, fence);
        }
    };

    using handlers = flow::handlers<filter>;
};

/// DaaTracker: update(detections, estimate); conflicts(estimate) → conflicts.
template <class T>
struct DaaTrackerBinding {
    using signature = daa_tracker_signature<T>;

    struct update : handler<"update", ins<"detections", "estimate">, outs<>, state_role::update> {
        static void call(auto& daa, const IntruderDetections<T>& det, const StateEstimate<T>& x) {
            daa.update(det, x);
        }
    };
    struct conflicts : handler<"conflicts", ins<"estimate">, outs<"conflicts">, state_role::read> {
        static ConflictSet<T> call(const auto& daa, const StateEstimate<T>& x) {
            return daa.conflicts(x);
        }
    };

    using handlers = flow::handlers<update, conflicts>;
};

/// FcBridge: send(command) at the tick's time, which is also the companion's
/// heartbeat (SAF-01).
template <class T>
struct FcBridgeBinding {
    using signature = fc_bridge_signature<T>;

    struct send : handler<"send", ins<"command">, outs<>, state_role::update, takes_time::yes> {
        static void call(auto& fc, Timestamp now, const ThrustRates<T>& u) {
            fc.send(u, now);
        }
    };

    using handlers = flow::handlers<send>;
};

}  // namespace branes::anatomy::flow
