#pragma once
/// @file tick.hpp
/// @brief TickOutput, and RunnableStack: what a harness needs from a stack.
///
/// A mission composes
/// its slots into its own stack type (racing-drone: Stack, stack.hpp); the harness
/// (harness/runner.hpp) drives any stack that satisfies RunnableStack, with the
/// same events, so a log of the events replays to the same outputs (SYS-08,
/// SW-03).

#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/msg/truth.hpp>

#include <concepts>

namespace branes::anatomy {

/// What one control tick produces.
template <typename T>
struct TickOutput {
    StateEstimate<T> state;
    Reference<T> reference;  ///< the planner's output, the controller's input
    ThrustRates<T> command;

    friend constexpr bool operator==(const TickOutput&, const TickOutput&) = default;
};

/// A stack the harness can drive: it names its scalar type, takes the four
/// input events, and returns a TickOutput per tick. Preconditions are the
/// stack's (valid inputs; ticks in non-decreasing time); the Runner checks the
/// ones a log could violate before it calls.
template <class S>
concept RunnableStack = requires(S s,
                                 const ImuSample<typename S::scalar_type>& imu,
                                 const CameraFrame& frame,
                                 const TruthState<typename S::scalar_type>& truth,
                                 Timestamp now) {
    s.on_imu(imu);
    s.on_frame(frame);
    s.on_truth(truth);
    { s.on_tick(now) } -> std::same_as<TickOutput<typename S::scalar_type>>;
};

}  // namespace branes::anatomy
