// The harness pieces: replay verification, the flight tick bound, the batch
// pool, and number parsing. The log dump is tested in
// tests/mobility/mobility_harness.cpp.

#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/harness/flight.hpp>
#include <branes/anatomy/harness/parallel.hpp>
#include <branes/anatomy/harness/replay.hpp>
#include <branes/anatomy/log/log.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <system_error>
#include <vector>

using namespace branes::anatomy;

TEST_CASE("neutral: verify_outputs counts equal outputs and names the first difference", "[neutral]") {
    harness::ReplayOutputs kept;
    kept.produced = {log::LogRecord{log::Channel::state, Timestamp{1}, {}},
                     log::LogRecord{log::Channel::command, Timestamp{1}, {}}};
    kept.recorded = kept.produced;
    const auto same = harness::verify_outputs(kept);
    REQUIRE(same);
    CHECK(*same == 2u);

    kept.recorded[1].t = Timestamp{2};
    const auto differs = harness::verify_outputs(kept);
    REQUIRE_FALSE(differs);
    CHECK(differs.error() == "verify: output 1 (t=1) differs from the recording");

    kept.recorded.pop_back();
    const auto fewer = harness::verify_outputs(kept);
    REQUIRE_FALSE(fewer);
    CHECK(fewer.error() == "verify: 2 outputs produced, 1 recorded");
}

TEST_CASE("neutral: ticks_fit keeps the last tick time inside int64", "[neutral]") {
    constexpr std::int64_t max = std::numeric_limits<std::int64_t>::max();
    CHECK(harness::ticks_fit(0, max));
    CHECK(harness::ticks_fit(1, max));  // one tick, at t = 0
    CHECK_FALSE(harness::ticks_fit(3, max));
    CHECK(harness::ticks_fit(1'000, 10'000'000));
    CHECK(harness::ticks_fit(max / 10'000'000 + 1, 10'000'000));  // the last tick at the largest multiple
    CHECK_FALSE(harness::ticks_fit(max / 10'000'000 + 2, 10'000'000));
}

TEST_CASE("neutral: for_each_index visits every index once, on any number of jobs", "[neutral]") {
    for (const unsigned jobs : {1u, 3u, 64u}) {
        std::vector<std::atomic<int>> seen(100);
        harness::for_each_index(seen.size(), jobs, [&](std::uint64_t i) { ++seen[i]; });
        for (const auto& s : seen) {
            CHECK(s == 1);
        }
    }
    bool called = false;
    harness::for_each_index(0, 4, [&](std::uint64_t) { called = true; });
    CHECK_FALSE(called);
}

TEST_CASE("neutral: parse_number takes the whole string or nothing", "[neutral]") {
    CHECK(harness::parse_number<std::uint64_t>("42") == 42u);
    CHECK(harness::parse_number<double>("0.5") == 0.5);
    CHECK_FALSE(harness::parse_number<std::uint64_t>("10x"));
    CHECK_FALSE(harness::parse_number<std::uint64_t>(""));
    CHECK_FALSE(harness::parse_number<unsigned>("-1"));
}
