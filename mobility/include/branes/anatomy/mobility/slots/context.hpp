#pragma once
/// @file context.hpp
/// @brief BasicSlotContext: what a roaming vehicle's slot may be built from;
/// MissionWorld; TruthConsumer.
///
/// The mission names its world type (racing-drone: SlotContext, over the
/// course; context.hpp). The context holds load-time configs (double): the
/// vehicle file, the baseline file (its `parameters:` block), and the
/// mission's world. Slots are built from it by make_slot (slots/slot_context.hpp).
///
/// A slot that declares `process_truth(const TruthState<T>&)` (TruthConsumer)
/// receives the simulated true state from the stack's on_truth(). Only SITL
/// logs carry truth (SYS-02), so in flight such a slot never receives any.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/slots/slot_context.hpp>

#include <concepts>

namespace branes::anatomy {

/// A mission's world, as slots may read it: wherever the task is, every world
/// has the vehicle's start pose (racing-drone: CourseMap<double>::start).
template <class W>
concept MissionWorld = requires(const W& w) {
    { w.start } -> std::convertible_to<const StartPose<double>&>;
};

/// The configs a slot may be built from. Non-owning: valid for the duration of
/// the from_context() call only.
template <MissionWorld World>
struct BasicSlotContext {
    const config::VehicleConfig& vehicle;
    const config::BaselineConfig& baseline;
    const World& world;
};

/// A slot that reads a roaming vehicle's simulated true state, in scalar type
/// T (SITL only). Another profile's truth slots are ConsumesTruth<S, Truth>
/// (compose/profile.hpp).
template <class S, class T>
concept TruthConsumer = ConsumesTruth<S, TruthState<T>>;

}  // namespace branes::anatomy
