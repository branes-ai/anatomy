#pragma once
/// @file signatures.hpp
/// @brief The signatures of the mission-neutral slots, as composites (anatomy
/// docs/adr/0004, decision 6 and step 1; docs/plan/flow-domains).
///
/// Each port's domain is the domain_of of the type its slot concept
/// (slots/neutral.hpp) takes or returns, in the concept's order. A composite
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
using controller_signature = signature<ports<port<"estimate", domain_of_t<StateEstimate<T>>, direction::in>,
                                             port<"reference", domain_of_t<Reference<T>>, direction::in>,
                                             port<"conflicts", domain_of_t<ConflictSet<T>>, direction::in>,
                                             port<"command", domain_of_t<ThrustRates<T>>, direction::out>>>;

/// SafetyFilter: filter(estimate, command, conflicts, geofence) → safe_command.
template <class T>
using safety_filter_signature = signature<ports<port<"estimate", domain_of_t<StateEstimate<T>>, direction::in>,
                                                port<"command", domain_of_t<ThrustRates<T>>, direction::in>,
                                                port<"conflicts", domain_of_t<ConflictSet<T>>, direction::in>,
                                                port<"geofence", domain_of_t<Geofence<T>>, direction::in>,
                                                port<"safe_command", domain_of_t<ThrustRates<T>>, direction::out>>>;

/// DaaTracker: update(detections, estimate); conflicts(estimate) → conflicts.
template <class T>
using daa_tracker_signature = signature<ports<port<"detections", domain_of_t<IntruderDetections<T>>, direction::in>,
                                              port<"estimate", domain_of_t<StateEstimate<T>>, direction::in>,
                                              port<"conflicts", domain_of_t<ConflictSet<T>>, direction::out>>>;

/// FcBridge: send(command); poll_imu() → imu; fc_status() → fc_status.
template <class T>
using fc_bridge_signature = signature<ports<port<"command", domain_of_t<ThrustRates<T>>, direction::in>,
                                            port<"imu", domain_of_t<ImuSample<T>>, direction::out>,
                                            port<"fc_status", domain_of_t<FcStatus>, direction::out>>>;

}  // namespace branes::anatomy::flow
