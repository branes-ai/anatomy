#pragma once
/// @file runner.hpp
/// @brief Runner: feeds log records to a stack and encodes its outputs.
///
/// SITL and replay
/// both go through it, so a replay of a SITL log reproduces the SITL outputs
/// record for record.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/time.hpp>
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

/// One record the stack produced (compose/profile.hpp).
using Output = anatomy::Output;

/// Feeds input records to a stack and returns the encoded outputs.
///
/// Records are double. Each decoded input is cast to the stack's scalar type T
/// and each output back to double. Inputs that do not decode, or are not
/// valid() once cast, are reported, never silently skipped: a log that does
/// not replay cleanly is not replay-complete (ADR 0006).
///
/// The Runner handles the events every mission has — the tick, the truth
/// (SITL) and sim_config — and hands the channels of the stack's profile P to
/// P::feed; each tick's output is encoded by P::encode.
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
///   I2: pass_through_ holds no framework channel, and no
///       input channel of the profile                     — constrains pass_through_
template <RunnableStack StackT>
class Runner {
    using T = typename StackT::scalar_type;
    using P = typename StackT::profile;

public:
    /// Pre: no channel of `pass_through` is a framework channel or one of the
    /// profile's inputs (I2).
    Runner(StackT& stack, std::span<const log::Channel> pass_through)
        : stack_(stack), pass_through_(pass_through.begin(), pass_through.end()) {
        for ([[maybe_unused]] const log::Channel c : pass_through_) {
            assert(!log::is_framework(static_cast<std::uint16_t>(c)));  // Pre / I2
            assert(!is_profile_input<P>(c));                            // Pre / I2
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
        case log::Channel::tick: {
            // The log is untrusted input: check the stack's I2 precondition here
            // and report a violation, rather than relying on its assert.
            if (last_tick_ && rec.t < *last_tick_) {
                return std::string("tick at t=" + std::to_string(rec.t.ns) +
                                   " is earlier than the previous tick (t=" + std::to_string(last_tick_->ns) + ")");
            }
            last_tick_ = rec.t;
            P::encode(stack_.on_tick(rec.t), rec.t, out);
            assert(encoded_by_profile(out, rec.t));  // the profile's encode postcondition
            break;
        }
        case log::Channel::truth: {
            // SITL only: the stack hands it to its truth-consuming slots (B0's
            // ground-truth estimator), so a replay reproduces their outputs.
            const auto x = P::decode_truth(rec.payload);
            if (!x || !valid(*x)) {
                return std::string("undecodable truth record at t=" + std::to_string(rec.t.ns));
            }
            const auto xt = cast<T>(*x);
            if (!valid(xt)) {
                return std::string("truth record at t=" + std::to_string(rec.t.ns) +
                                   " is not valid in the stack's scalar type");
            }
            stack_.on_truth(xt);
            break;
        }
        case log::Channel::sim_config:
            break;  // sim channels: recorded, not consumed by the stack
        default: {
            if (!is_profile_input<P>(rec.channel)) {
                return std::string("not an input channel");
            }
            const auto fed = P::feed(stack_, rec);
            if (!fed) {
                return fed.error();
            }
            break;
        }
        }
        return out;
    }

private:
    /// True if `out` holds one record per output channel of the profile, in
    /// its order, at time t.
    [[nodiscard]] static bool encoded_by_profile(const std::vector<Output>& out, Timestamp t) noexcept {
        if (out.size() != P::outputs.size()) {
            return false;
        }
        for (std::size_t k = 0; k < out.size(); ++k) {
            if (out[k].channel != P::outputs[k] || !(out[k].t == t)) {
                return false;
            }
        }
        return true;
    }

    StackT& stack_;
    std::vector<log::Channel> pass_through_;
    std::optional<Timestamp> last_tick_;
};

}  // namespace branes::anatomy::harness
