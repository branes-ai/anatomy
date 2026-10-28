#pragma once
/// @file replay.hpp
/// @brief replay_log() and verify_outputs(): the replay loop of replay_<baseline>.
///
/// A replay feeds a
/// log's inputs to the stack through the Runner and writes them, with the
/// stack's outputs, to a new log; the input log's outputs are replaced. With
/// verification, the outputs produced and the outputs recorded are kept, so
/// verify_outputs() can compare them record for record once the new log is
/// closed. The mission supplies the stack and its pass-through channels
/// (racing-drone: apps/replay/main.cpp).

#include <branes/anatomy/compose/tick.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/result.hpp>

#include <cstddef>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::harness {

/// What a replay kept for verification: the outputs it produced and the ones
/// the input log recorded, each in log order. Empty without verification.
struct ReplayOutputs {
    std::vector<log::LogRecord> produced;
    std::vector<log::LogRecord> recorded;
};

/// Replay every record of `reader` through `stack` into `writer`.
/// Returns the outputs kept for verification (if `verify`), or the error that
/// stopped the replay. Pre: writer.is_open().
template <RunnableStack StackT>
[[nodiscard]] result<ReplayOutputs, std::string> replay_log(
    log::LogReader& reader, StackT& stack, std::span<const log::Channel> mission, log::LogWriter& writer, bool verify) {
    Runner runner(stack, mission);
    ReplayOutputs kept;
    for (;;) {
        auto rec = reader.next();
        if (!rec) {
            return rec.error().path + ": " + rec.error().message;
        }
        if (!rec->has_value()) {
            break;  // end of log
        }
        log::LogRecord& r = **rec;
        if (log::is_output(r.channel)) {
            if (verify) {
                kept.recorded.push_back(std::move(r));
            }
            continue;  // the input log's outputs are replaced by ours
        }
        writer.write(r.channel, r.t, r.payload);
        auto outputs = runner.feed(r);
        if (!outputs) {
            return outputs.error();
        }
        for (auto& o : *outputs) {
            writer.write(o.channel, o.t, o.payload);
            if (verify) {
                kept.produced.push_back(log::LogRecord{o.channel, o.t, std::move(o.payload)});
            }
        }
    }
    return kept;
}

/// Compare the outputs produced with those recorded, record for record.
/// Returns how many matched, or the first difference.
[[nodiscard]] inline result<std::size_t, std::string> verify_outputs(const ReplayOutputs& kept) {
    if (kept.produced.size() != kept.recorded.size()) {
        return std::string("verify: " + std::to_string(kept.produced.size()) + " outputs produced, " +
                           std::to_string(kept.recorded.size()) + " recorded");
    }
    for (std::size_t i = 0; i < kept.produced.size(); ++i) {
        if (!(kept.produced[i] == kept.recorded[i])) {
            return std::string("verify: output " + std::to_string(i) + " (t=" + std::to_string(kept.produced[i].t.ns) +
                               ") differs from the recording");
        }
    }
    return kept.produced.size();
}

}  // namespace branes::anatomy::harness
