#pragma once
/// @file driver.hpp
/// @brief The driver: a pattern's sources merged into one stream in time
/// order, handed to its stack (docs/plan/workbench, "The workbench").
///
/// Two inputs at the same time go in the order the pattern declares its
/// sources: the fixed order between simultaneous triggers that the model
/// requires (section 8).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <vector>

namespace branes::anatomy::workbench {

/// What the driver says after each input: its index in the stream, the
/// source it came from, and its time.
using after_fn = std::function<void(std::size_t i, std::size_t source, Timestamp t)>;

/// Hand `s` the first `inputs` messages of the merged stream, calling `after`
/// once each has run. Pre: every source's period is positive, and its input is s's.
inline void drive(flow::stack& s, std::span<const source> sources, std::size_t inputs, const after_fn& after = {}) {
    if (sources.empty()) {
        return;
    }
    std::vector<std::uint64_t> k(sources.size(), 0);
    for (std::size_t i = 0; i < inputs; ++i) {
        // The earliest next message; the first declared, of two at one time.
        std::size_t next = sources.size();
        std::int64_t at = 0;
        std::size_t j = 0;
        for (const source& src : sources) {
            const std::int64_t t = src.start_ns + static_cast<std::int64_t>(k[j]) * src.period_ns;
            if (next == sources.size() || t < at) {
                next = j;
                at = t;
            }
            ++j;
        }
        sources[next].hand(s, k[next], Timestamp{at});
        ++k[next];
        if (after) {
            after(i, next, Timestamp{at});
        }
    }
}

}  // namespace branes::anatomy::workbench
