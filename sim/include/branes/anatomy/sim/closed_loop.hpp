#pragma once
/// @file closed_loop.hpp
/// @brief One scored closed-loop SITL run without a log, and the summaries over many.
///
/// A run is the SITL
/// closed loop: the simulator's events go through the Runner into the stack,
/// each command goes back to the simulator, and the mission's scorer watches
/// the truth and the estimates, until the run is complete or has failed. The
/// mission supplies the simulator, the stack, the scorer and its channels
/// (racing-drone: batch.hpp, run_one).

#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::sim {

/// A mission's scorer: it watches the truth and the estimates, says when the
/// run is complete or has failed, and gives the score.
template <class S>
concept RunScorer = requires(S s, const S& cs, const TruthState<double>& x, const StateEstimate<double>& e) {
    s.on_truth(x);
    s.on_estimate(e);
    { cs.complete() } -> std::convertible_to<bool>;
    { cs.failed() } -> std::convertible_to<bool>;
    cs.score();
};

/// One seed's run, with the mission's score.
template <class ScoreT>
struct BasicRun {
    std::uint64_t seed{0};
    ScoreT score;
    std::string error;  ///< set if the run could not be set up or fed, or its controller held ticks on a rejected dt
};

/// Run the closed loop of `sim` and `stack`, scored by `scorer`, and return the
/// score, or the error that stopped the run. `mission` is the mission's
/// pass-through channels (harness::Runner). The loop decodes the stack's
/// command and estimate, so it takes a mobile stack (MobileStack) only.
template <class ScoreT, class Oracle, MobileStack StackT, RunScorer ScorerT>
[[nodiscard]] BasicRun<ScoreT> run_closed_loop(BasicSimulator<Oracle>& sim,
                                               StackT& stack,
                                               std::span<const log::Channel> mission,
                                               ScorerT& scorer,
                                               std::uint64_t seed) {
    BasicRun<ScoreT> out;
    out.seed = seed;
    harness::Runner runner(stack, mission);
    while (auto ev = sim.next()) {
        if (ev->channel == log::Channel::truth) {
            const auto x = codec::decode_truth(ev->payload);
            if (!x) {
                out.error = "undecodable truth record";
                return out;
            }
            scorer.on_truth(*x);
        }
        auto outputs = runner.feed(log::LogRecord{ev->channel, ev->t, std::move(ev->payload)});
        if (!outputs) {
            out.error = outputs.error();
            return out;
        }
        for (const auto& o : *outputs) {
            if (o.channel == log::Channel::command) {
                const auto u = codec::decode_command(o.payload);
                if (!u) {
                    out.error = "the stack produced a command that does not decode";
                    return out;
                }
                sim.command(*u);
            } else if (o.channel == log::Channel::state) {
                if (const auto x = codec::decode_state(o.payload)) {
                    scorer.on_estimate(*x);
                }
            }
        }
        if (scorer.complete() || scorer.failed()) {
            break;
        }
    }
    // A controller that counts ticks held on a dt its TimeBase rejected
    // (CascadedPid, racing-drone docs/plan/time-base/) fails the run, so the hold shows.
    if constexpr (requires { stack.controller().timing_faults(); }) {
        if (const auto n = stack.controller().timing_faults(); n != 0) {
            out.error = "the controller held " + std::to_string(n) + " ticks whose dt its TimeBase rejected";
            return out;
        }
    }
    out.score = scorer.score();
    return out;
}

/// A string as a JSON string literal's body: quotes and backslashes escaped,
/// as in to_json(Score).
[[nodiscard]] inline std::string json_escape(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '"' || c == '\\') {
            out += '\\';
        }
        out += c;
    }
    return out;
}

/// A run's JSON line, without the newline: {"seed":…,"score":{…}}, or
/// {"seed":…,"run_error":"…"} for a run that could not be set up or fed, or
/// whose controller held ticks on a rejected dt. The score's JSON is the
/// mission's to_json(ScoreT).
template <class ScoreT>
[[nodiscard]] std::string to_json(const BasicRun<ScoreT>& r) {
    std::string out = "{\"seed\":" + std::to_string(r.seed);
    if (!r.error.empty()) {
        return out + ",\"run_error\":\"" + json_escape(r.error) + "\"}";
    }
    return out + ",\"score\":" + to_json(r.score) + "}";
}

/// Nearest-rank percentile of `v` (sorted in place); NaN if empty.
[[nodiscard]] inline double percentile(std::vector<double>& v, double p) {
    if (v.empty()) {
        return std::nan("");
    }
    std::sort(v.begin(), v.end());
    const auto rank = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())));
    return v[rank == 0 ? 0 : rank - 1];
}

/// A failure message without its time or count, for grouping.
[[nodiscard]] inline std::string failure_reason(const std::string& failure) {
    for (const char* cut : {" at t=", ":"}) {
        if (const auto i = failure.find(cut); i != std::string::npos) {
            return failure.substr(0, i);
        }
    }
    return failure;
}

}  // namespace branes::anatomy::sim
