#pragma once
/// @file slot_context.hpp
/// @brief BasicSlotContext: what a slot may be built from, and the optional slot hooks.
///
/// the mission names
/// its world type (racing-drone: SlotContext, over the course; context.hpp).
///
/// A slot type with a factory
///
///   static result<S, std::string> from_context(const BasicSlotContext<World>&)
///
/// is built from the context by the mission's stack; any other slot is
/// default-constructed. The choice is made per slot at compile time, so the
/// null slots stay as they are. The context holds load-time configs (double):
/// the vehicle file, the baseline file (its `parameters:` block), and the
/// mission's world.
///
/// A slot that declares `process_truth(const TruthState<T>&)` (TruthConsumer)
/// receives the simulated true state from the stack's on_truth(). Only SITL
/// logs carry truth (SYS-02), so in flight such a slot never receives any.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/config/vehicle.hpp>
#include <branes/anatomy/msg/region.hpp>
#include <branes/anatomy/msg/truth.hpp>
#include <branes/anatomy/result.hpp>

#include <concepts>
#include <string>
#include <string_view>

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

/// A slot type built by `S::from_context(ctx)` from a context `Ctx`.
template <class S, class Ctx>
concept ContextBuilt = requires(const Ctx& ctx) {
    { S::from_context(ctx) } -> std::same_as<result<S, std::string>>;
};

/// A slot that reads a roaming vehicle's simulated true state, in scalar type
/// T (SITL only). Another profile's truth slots are ConsumesTruth<S, Truth>
/// (compose/profile.hpp).
template <class S, class T>
concept TruthConsumer = ConsumesTruth<S, TruthState<T>>;

/// Build slot S from `ctx` if it is ContextBuilt, else default-construct it.
template <class S, class World>
[[nodiscard]] result<S, std::string> make_slot(const BasicSlotContext<World>& ctx) {
    if constexpr (ContextBuilt<S, BasicSlotContext<World>>) {
        return S::from_context(ctx);
    } else {
        (void)ctx;
        return S{};
    }
}

/// A baseline parameter as a number: `fallback` if absent, an error if present
/// but not a number or outside [lo, hi].
[[nodiscard]] inline result<double, std::string>
parameter_in(const config::BaselineConfig& b, std::string_view key, double fallback, double lo, double hi) {
    double v = fallback;
    if (b.parameters.count(std::string(key)) != 0) {
        const auto n = config::numeric_parameter(b, key);
        if (!n) {
            return std::string("baseline parameter " + std::string(key) + " is not a number");
        }
        v = *n;
    }
    if (!(v >= lo && v <= hi)) {
        return std::string("baseline parameter " + std::string(key) + " must be in [" + std::to_string(lo) + ", " +
                           std::to_string(hi) + "]");
    }
    return v;
}

}  // namespace branes::anatomy
