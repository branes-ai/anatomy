#pragma once
/// @file flight.hpp
/// @brief The flight stack's tick loop (flight_stack_<baseline>).
///
/// Phase 0's flight
/// stack runs the composed stack open loop on the control tick; the HIL and
/// flight I/O replace it in later phases.

#include <branes/anatomy/compose/tick.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <cstdint>
#include <limits>

namespace branes::anatomy::harness {

/// True if ticks 0 … cycles − 1 at `period_ns` all have times inside int64
/// (no signed overflow). Pre: cycles ≥ 0, period_ns > 0.
[[nodiscard]] constexpr bool ticks_fit(long long cycles, std::int64_t period_ns) noexcept {
    return cycles == 0 || cycles - 1 <= std::numeric_limits<std::int64_t>::max() / period_ns;
}

/// Run `cycles` control ticks at times i · period_ns.
/// Pre: ticks_fit(cycles, period_ns).
template <RunnableStack StackT>
void run_ticks(StackT& stack, long long cycles, std::int64_t period_ns) {
    for (long long i = 0; i < cycles; ++i) {
        (void)stack.on_tick(Timestamp{i * period_ns});
    }
}

}  // namespace branes::anatomy::harness
