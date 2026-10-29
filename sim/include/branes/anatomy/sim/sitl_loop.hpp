#pragma once
/// @file sitl_loop.hpp
/// @brief run_sitl(): the SITL closed loop that writes the log, for sitl_<baseline>.
///
/// Every simulator
/// event is logged, fed to the stack through the Runner, and every output is
/// logged; each command goes back to the simulated FC. The mission supplies
/// the simulator, the stack, and its pass-through channels (racing-drone:
/// apps/sitl/main.cpp).

#include <branes/anatomy/compose/tick.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>

#include <cstdint>
#include <span>
#include <string>
#include <utility>

namespace branes::anatomy::sim {

/// Run `sim` and `stack` to the end, logging every input and output to
/// `writer`. Returns the number of ticks, or the error that stopped the run.
/// The loop sends the stack's command to the FC, so it takes a mobile stack
/// (MobileStack) only.
/// Pre: writer.is_open().
template <class Oracle, MobileStack StackT>
[[nodiscard]] result<std::uint64_t, std::string>
run_sitl(BasicSimulator<Oracle>& sim, StackT& stack, std::span<const log::Channel> mission, log::LogWriter& writer) {
    harness::Runner runner(stack, mission);
    std::uint64_t ticks = 0;
    while (auto ev = sim.next()) {
        writer.write(ev->channel, ev->t, ev->payload);
        auto outputs = runner.feed(log::LogRecord{ev->channel, ev->t, std::move(ev->payload)});
        if (!outputs) {
            return outputs.error();
        }
        for (const auto& o : *outputs) {
            writer.write(o.channel, o.t, o.payload);
            if (o.channel == log::Channel::command) {
                // The command the stack sent, back to the simulated FC.
                // decode(encode(u)) == u: the codec writes every double exactly.
                const auto u = codec::decode_command(o.payload);
                if (!u) {
                    return std::string("the stack produced a command that does not decode");
                }
                sim.command(*u);
            }
        }
        if (ev->channel == log::Channel::tick) {
            ++ticks;
        }
    }
    return ticks;
}

}  // namespace branes::anatomy::sim
