#pragma once
/// @file profile.hpp
/// @brief StackProfile and RunnableStack: what a harness needs from a stack,
/// for any kind of mission.
///
/// A profile says what a stack of one kind of mission takes and produces: its
/// truth type, its output type, and its own input and output channels, with
/// the codecs between them and the log (anatomy docs/adr/0001, Decision 2).
/// The harness (harness/runner.hpp) handles the events every mission has —
/// the tick, the truth (SITL), sim_config — and hands the profile's own
/// channels to the profile. The mobility profile is MobilityProfile
/// (mobility/compose/tick.hpp).
///
/// Model: a profile P is stateless; every member is static.
///   truth_type<T>     : the simulated truth, in scalar type T
///   output_type<T>    : what one tick produces
///   inputs            : the profile's input channels
///   outputs           : its output channels, in pipeline order
///   decode_truth(b)   : the truth record's payload, decoded; empty if malformed
///   feed(s, rec)      : decode, check and cast one input record, and hand it to s
///   encode(y, t, out) : append one Output per channel of `outputs`, in that order
///   drives<S>         : S has the profile's input hooks
///
/// Invariants (StackProfile checks them at compile time):
///   P1: every channel of `inputs` is an input (!is_output), and none is the
///       harness's own: tick, truth, sim_config
///   P2: every channel of `outputs` is an output (is_output)
///   P3: `inputs` and `outputs` hold no duplicates
///
/// Contracts:
///   feed(s, rec) -> result<ok_t, std::string>
///     Pre:  rec.channel is one of `inputs`
///     Post: success ⇒ one event was handed to s, decoded and cast to its
///           scalar type, with the checks the profile makes; error ⇒ s is
///           unchanged, and the message names the record's kind and time
///   encode(y, t, out)
///     Post: appended exactly outputs.size() records, the k-th on outputs[k]
///           at time t

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace branes::anatomy {

/// One record a stack produced.
struct Output {
    log::Channel channel;
    Timestamp t;
    std::vector<std::byte> payload;
};

namespace detail {

[[nodiscard]] constexpr bool is_harness_channel(log::Channel c) noexcept {
    return c == log::Channel::tick || c == log::Channel::truth || c == log::Channel::sim_config;
}

[[nodiscard]] constexpr bool distinct(std::span<const log::Channel> cs) noexcept {
    for (std::size_t i = 0; i < cs.size(); ++i) {
        for (std::size_t j = i + 1; j < cs.size(); ++j) {
            if (cs[i] == cs[j]) {
                return false;
            }
        }
    }
    return true;
}

/// P1 and P3 for the input channels.
[[nodiscard]] constexpr bool valid_inputs(std::span<const log::Channel> cs) noexcept {
    return distinct(cs) && std::none_of(cs.begin(), cs.end(), [](log::Channel c) {
               return log::is_output(c) || is_harness_channel(c);
           });
}

/// P2 and P3 for the output channels.
[[nodiscard]] constexpr bool valid_outputs(std::span<const log::Channel> cs) noexcept {
    return distinct(cs) && std::all_of(cs.begin(), cs.end(), [](log::Channel c) { return log::is_output(c); });
}

}  // namespace detail

/// A profile: its types, its channels (P1–P3), and the truth's codec and
/// check. feed, encode, drives and the casts are checked against a stack, by
/// RunnableStack.
template <class P>
concept StackProfile = requires(std::span<const std::byte> b, const typename P::template truth_type<double>& x) {
    typename P::template output_type<double>;
    { P::decode_truth(b) } -> std::same_as<std::optional<typename P::template truth_type<double>>>;
    { valid(x) } -> std::convertible_to<bool>;
} && detail::valid_inputs(P::inputs) && detail::valid_outputs(P::outputs);

/// True if `c` is one of profile P's input channels.
template <StackProfile P>
[[nodiscard]] constexpr bool is_profile_input(log::Channel c) noexcept {
    return std::find(P::inputs.begin(), P::inputs.end(), c) != P::inputs.end();
}

/// A stack the harness can drive: it names its scalar type and its profile,
/// takes the truth and the tick, returns the profile's output per tick, and
/// has the profile's input hooks. A stack names its profile explicitly
/// (`using profile = ...;`); there is no default. Preconditions are the
/// stack's (valid inputs; ticks in non-decreasing time); the Runner checks the
/// ones a log could violate before it calls.
template <class S>
concept RunnableStack = StackProfile<typename S::profile> &&
                        requires(S s,
                                 const typename S::profile::template truth_type<typename S::scalar_type>& truth,
                                 const typename S::profile::template output_type<typename S::scalar_type>& y,
                                 const typename S::profile::template truth_type<double>& recorded,
                                 const log::LogRecord& rec,
                                 std::vector<Output>& out,
                                 Timestamp now) {
                            {
                                cast<typename S::scalar_type>(recorded)
                            } -> std::same_as<typename S::profile::template truth_type<typename S::scalar_type>>;
                            { valid(truth) } -> std::convertible_to<bool>;
                            s.on_truth(truth);
                            {
                                s.on_tick(now)
                            } -> std::same_as<typename S::profile::template output_type<typename S::scalar_type>>;
                            { S::profile::feed(s, rec) } -> std::same_as<result<ok_t, std::string>>;
                            S::profile::encode(y, now, out);
                        } &&
                        S::profile::template drives<S>;

/// A slot that reads the simulated truth of type `Truth` (SITL only).
template <class S, class Truth>
concept ConsumesTruth = requires(S s, const Truth& x) { s.process_truth(x); };

}  // namespace branes::anatomy
