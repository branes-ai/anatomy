#pragma once
/// @file parallel.hpp
/// @brief for_each_index(): independent runs over a thread pool, for the batch runners.
///
/// The batch runners
/// run one closed loop per seed; the runs are independent, so the pool changes
/// the wall time, never the results.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

namespace branes::anatomy::harness {

/// Call `body(i)` once for every i in [0, count), on min(jobs, count) threads.
/// Each index is claimed by exactly one thread; the call returns when all are
/// done. Pre: jobs ≥ 1; `body` is safe to call concurrently for distinct i.
template <class Body>
void for_each_index(std::uint64_t count, unsigned jobs, Body&& body) {
    std::atomic<std::uint64_t> next{0};
    const auto worker = [&] {
        for (std::uint64_t i = next++; i < count; i = next++) {
            body(i);
        }
    };
    std::vector<std::jthread> pool;
    for (unsigned k = 0; k < std::min<std::uint64_t>(jobs, count); ++k) {
        pool.emplace_back(worker);
    }
}

}  // namespace branes::anatomy::harness
