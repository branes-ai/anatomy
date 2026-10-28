#pragma once
/// @file runner.hpp
/// @brief Runner: feeds log records to a stack and encodes its outputs.
///
/// SITL and replay
/// both go through it, so a replay of a SITL log reproduces the SITL outputs
/// record for record.

#include <branes/anatomy/compose/tick.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/msg/truth.hpp>
#include <branes/anatomy/result.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace branes::anatomy::harness {

// ── runner ──────────────────────────────────────────────────────────

/// One record the stack produced.
struct Output {
    log::Channel channel;
    Timestamp t;
    std::vector<std::byte> payload;
};

/// Feeds input records to a stack and returns the encoded outputs.
///
/// Records are double. Each decoded input is cast to the stack's scalar type T
/// and each output back to double. Inputs that do not decode, or are not
/// valid() once cast, are reported, never silently skipped: a log that does
/// not replay cleanly is not replay-complete (ADR 0006).
///
/// Model
/// -----
///   stack_        : StackT& [non-owning] — the stack it drives
///   pass_through_ : the mission's channels that are recorded but not consumed
///                   by the stack (racing-drone: log::kRacingChannels)
///   last_tick_    : std::optional<Timestamp> — the previous tick's time
///
/// Invariants
/// ----------
///   I1: last_tick_ never decreases                       — constrains last_tick_
///   I2: pass_through_ holds no framework channel         — constrains pass_through_
template <RunnableStack StackT>
class Runner {
    using T = typename StackT::scalar_type;

public:
    /// Pre: no channel of `pass_through` is a framework channel (I2).
    Runner(StackT& stack, std::span<const log::Channel> pass_through)
        : stack_(stack), pass_through_(pass_through.begin(), pass_through.end()) {
        for ([[maybe_unused]] const log::Channel c : pass_through_) {
            assert(!log::is_framework(static_cast<std::uint16_t>(c)));  // Pre / I2
        }
    }

    /// Process one input record. Pre: !log::is_output(rec.channel).
    [[nodiscard]] result<std::vector<Output>, std::string> feed(const log::LogRecord& rec) {
        assert(!log::is_output(rec.channel));  // Pre
        std::vector<Output> out;
        if (std::find(pass_through_.begin(), pass_through_.end(), rec.channel) != pass_through_.end()) {
            return out;  // a mission channel: recorded, not consumed by the stack
        }
        switch (rec.channel) {
        case log::Channel::imu: {
            const auto s = codec::decode_imu(rec.payload);
            if (!s) {
                return std::string("undecodable IMU record at t=" + std::to_string(rec.t.ns));
            }
            const ImuSample<T> st = cast<T>(*s);
            if (!valid(st)) {
                return std::string("IMU record at t=" + std::to_string(rec.t.ns) +
                                   " is not valid in the stack's scalar type");
            }
            stack_.on_imu(st);
            break;
        }
        case log::Channel::camera: {
            const auto f = codec::decode_frame(rec.payload);
            if (!f) {
                return std::string("undecodable camera record at t=" + std::to_string(rec.t.ns));
            }
            stack_.on_frame(view_of(*f));
            break;
        }
        case log::Channel::tick: {
            // The log is untrusted input: check the stack's I2 precondition here
            // and report a violation, rather than relying on its assert.
            if (last_tick_ && rec.t < *last_tick_) {
                return std::string("tick at t=" + std::to_string(rec.t.ns) +
                                   " is earlier than the previous tick (t=" + std::to_string(last_tick_->ns) + ")");
            }
            last_tick_ = rec.t;
            const TickOutput<T> y = stack_.on_tick(rec.t);
            // Pipeline order: estimate, reference, command.
            out.push_back(Output{log::Channel::state, rec.t, codec::encode(cast<double>(y.state))});
            out.push_back(Output{log::Channel::reference, rec.t, codec::encode(cast<double>(y.reference))});
            out.push_back(Output{log::Channel::command, rec.t, codec::encode(cast<double>(y.command))});
            break;
        }
        case log::Channel::truth: {
            // SITL only: the stack hands it to TruthConsumer slots (B0's
            // ground-truth estimator), so a replay reproduces their outputs.
            const auto x = codec::decode_truth(rec.payload);
            if (!x || !valid(*x)) {
                return std::string("undecodable truth record at t=" + std::to_string(rec.t.ns));
            }
            const TruthState<T> xt = cast<T>(*x);
            if (!valid(xt)) {
                return std::string("truth record at t=" + std::to_string(rec.t.ns) +
                                   " is not valid in the stack's scalar type");
            }
            stack_.on_truth(xt);
            break;
        }
        case log::Channel::sim_config:
            break;  // sim channels: recorded, not consumed by the stack
        default:
            return std::string("not an input channel");
        }
        return out;
    }

private:
    StackT& stack_;
    std::vector<log::Channel> pass_through_;
    std::optional<Timestamp> last_tick_;
};

}  // namespace branes::anatomy::harness
