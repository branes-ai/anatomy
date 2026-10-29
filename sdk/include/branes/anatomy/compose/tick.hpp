#pragma once
/// @file tick.hpp
/// @brief TickOutput, and MobilityProfile: the stack profile of a roaming vehicle.
///
/// A mission composes
/// its slots into its own stack type (racing-drone: Stack, stack.hpp); the harness
/// (harness/runner.hpp) drives any stack that satisfies RunnableStack
/// (compose/profile.hpp), with the same events, so a log of the events replays
/// to the same outputs (SYS-08, SW-03). A mobile mission's stack names
/// MobilityProfile: it takes IMU samples and camera frames, reads the vehicle's
/// own true state, and produces a state estimate, a reference and a command
/// per tick.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/msg/truth.hpp>
#include <branes/anatomy/result.hpp>

#include <array>
#include <cassert>
#include <concepts>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace branes::anatomy {

/// What one control tick produces.
template <typename T>
struct TickOutput {
    StateEstimate<T> state;
    Reference<T> reference;  ///< the planner's output, the controller's input
    ThrustRates<T> command;

    friend constexpr bool operator==(const TickOutput&, const TickOutput&) = default;
};

/// The profile of a roaming vehicle's stack (StackProfile): its truth is the
/// vehicle's own TruthState, its inputs are the IMU and the camera, and each
/// tick produces a TickOutput, logged as the estimate, the reference and the
/// command, in that order.
struct MobilityProfile {
    template <typename T>
    using truth_type = TruthState<T>;
    template <typename T>
    using output_type = TickOutput<T>;

    static constexpr std::array<log::Channel, 2> inputs = {log::Channel::imu, log::Channel::camera};
    static constexpr std::array<log::Channel, 3> outputs = {
        log::Channel::state, log::Channel::reference, log::Channel::command};

    /// The stack's input hooks: on_imu for an IMU sample, on_frame for a camera frame.
    template <class S>
    static constexpr bool drives = requires(S s, const ImuSample<typename S::scalar_type>& imu, const CameraFrame& f) {
        s.on_imu(imu);
        s.on_frame(f);
    };

    [[nodiscard]] static std::optional<TruthState<double>> decode_truth(std::span<const std::byte> b) {
        return codec::decode_truth(b);
    }

    /// Hand one IMU or camera record to `s`. An IMU sample is cast to the
    /// stack's scalar type and must be valid() in it; a camera frame is passed
    /// as decoded. Pre: rec.channel is imu or camera.
    template <class S>
    [[nodiscard]] static result<ok_t, std::string> feed(S& s, const log::LogRecord& rec) {
        using T = typename S::scalar_type;
        if (rec.channel == log::Channel::imu) {
            const auto x = codec::decode_imu(rec.payload);
            if (!x) {
                return std::string("undecodable IMU record at t=" + std::to_string(rec.t.ns));
            }
            const ImuSample<T> st = cast<T>(*x);
            if (!valid(st)) {
                return std::string("IMU record at t=" + std::to_string(rec.t.ns) +
                                   " is not valid in the stack's scalar type");
            }
            s.on_imu(st);
            return ok;
        }
        assert(rec.channel == log::Channel::camera);  // Pre
        const auto f = codec::decode_frame(rec.payload);
        if (!f) {
            return std::string("undecodable camera record at t=" + std::to_string(rec.t.ns));
        }
        s.on_frame(view_of(*f));
        return ok;
    }

    /// Append the tick's estimate, reference and command, in pipeline order,
    /// each cast to double.
    template <typename T>
    static void encode(const TickOutput<T>& y, Timestamp t, std::vector<Output>& out) {
        out.push_back(Output{log::Channel::state, t, codec::encode(cast<double>(y.state))});
        out.push_back(Output{log::Channel::reference, t, codec::encode(cast<double>(y.reference))});
        out.push_back(Output{log::Channel::command, t, codec::encode(cast<double>(y.command))});
    }
};
static_assert(StackProfile<MobilityProfile>);

/// A stack of a roaming vehicle: a RunnableStack whose profile is MobilityProfile.
template <class S>
concept MobileStack = RunnableStack<S> && std::same_as<typename S::profile, MobilityProfile>;

}  // namespace branes::anatomy
