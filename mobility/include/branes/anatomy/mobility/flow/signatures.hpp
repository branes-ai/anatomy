#pragma once
/// @file signatures.hpp
/// @brief The signatures of the mission-neutral slots, as composites (anatomy
/// docs/adr/0004, decision 6 and step 1; docs/plan/flow-domains).
///
/// Each port carries the type its slot concept (slots/neutral.hpp) takes or
/// returns, in the concept's order, and holds that type's domain_of. A composite
/// declares no state: that belongs to whichever graph fills it.

#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/mobility/flow/domains.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/daa.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/fc.hpp>
#include <branes/anatomy/mobility/msg/imu.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>

namespace branes::anatomy::flow {

/// Controller: step(estimate, reference, conflicts) → command.
template <class T>
using controller_signature = signature<ports<port<"estimate", StateEstimate<T>, direction::in>,
                                             port<"reference", Reference<T>, direction::in>,
                                             port<"conflicts", ConflictSet<T>, direction::in>,
                                             port<"command", ThrustRates<T>, direction::out>>>;

/// SafetyFilter: filter(estimate, command, conflicts, geofence) → safe_command.
template <class T>
using safety_filter_signature = signature<ports<port<"estimate", StateEstimate<T>, direction::in>,
                                                port<"command", ThrustRates<T>, direction::in>,
                                                port<"conflicts", ConflictSet<T>, direction::in>,
                                                port<"geofence", Geofence<T>, direction::in>,
                                                port<"safe_command", ThrustRates<T>, direction::out>>>;

/// DaaTracker: update(detections, estimate); conflicts(estimate) → conflicts.
template <class T>
using daa_tracker_signature = signature<ports<port<"detections", IntruderDetections<T>, direction::in>,
                                              port<"estimate", StateEstimate<T>, direction::in>,
                                              port<"conflicts", ConflictSet<T>, direction::out>>>;

/// FcBridge: send(command); poll_imu() → imu; fc_status() → fc_status.
template <class T>
using fc_bridge_signature = signature<ports<port<"command", ThrustRates<T>, direction::in>,
                                            port<"imu", ImuSample<T>, direction::out>,
                                            port<"fc_status", FcStatus, direction::out>>>;

}  // namespace branes::anatomy::flow
