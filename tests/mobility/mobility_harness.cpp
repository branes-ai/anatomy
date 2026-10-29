// The mobility profile's harness pieces: the log dump with a mission's own
// channel, and the checks that a stack is a mobile one (MobileStack), which
// the SITL and closed loops require.

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/log/dump.hpp>

#include "../sdk/watch_profile.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <random>
#include <string>
#include <system_error>
#include <vector>

using namespace branes::anatomy;

namespace {

/// A path in this process's own temporary directory, so concurrent runs of the
/// test never share files.
std::filesystem::path temp_path(const std::string& name) {
    static const std::filesystem::path dir = [] {
        std::random_device rd;
        for (;;) {
            const auto d = std::filesystem::temp_directory_path() /
                           ("anatomy_neutral_harness_" + std::to_string(rd()) + "_" + std::to_string(rd()));
            std::error_code ec;
            if (std::filesystem::create_directory(d, ec)) {
                return d;  // created by this process: no other run uses it
            }
            REQUIRE_FALSE(ec);  // existed already: draw another name
        }
    }();
    return dir / name;
}

std::string read_all(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// A mission's channel: a counter, one byte per record.
constexpr log::Channel kCounter = static_cast<log::Channel>(9);

std::optional<std::string> counter_rows(const log::LogRecord& r, std::uint64_t& count) {
    if (r.payload.size() != 1) {
        return std::nullopt;
    }
    count = 1;
    std::string row = log::csv::start(r.t);
    log::csv::put(row, std::uint64_t{std::to_integer<std::uint8_t>(r.payload[0])});
    return row + '\n';
}

}  // namespace

namespace {

static_assert(!MobileStack<watch::WatchStack<double>>);  // so the SITL and closed loops refuse it

/// A stack that names the mobility profile without its input hooks is not runnable.
struct WrongProfileStack {
    using scalar_type = double;
    using profile = MobilityProfile;
    void on_range(const watch::Range<double>&) {}
    void on_truth(const TruthState<double>&) {}
    TickOutput<double> on_tick(Timestamp) {
        return {};
    }
};
static_assert(!RunnableStack<WrongProfileStack>);

}  // namespace

TEST_CASE("neutral: dump_records writes a mission's channel in its own format", "[neutral]") {
    const auto log_path = temp_path("log.brlog");
    const auto out_dir = temp_path("dump");
    std::error_code ec;
    std::filesystem::remove_all(out_dir, ec);
    std::filesystem::create_directories(out_dir);
    {
        auto w = log::LogWriter::open(log_path, log::LogHeader{});
        REQUIRE(w);
        w->write(log::Channel::tick, Timestamp{1'000'000'000}, {});
        w->write(kCounter, Timestamp{1'000'000'000}, std::vector<std::byte>{std::byte{7}});
        w->write(kCounter, Timestamp{2'000'000'000}, std::vector<std::byte>{std::byte{8}});
        REQUIRE(w->close());
    }
    const std::array<log::Channel, 1> mission = {kCounter};
    const std::array<log::ChannelFormat, 1> formats = {
        log::ChannelFormat{kCounter, "counter.csv", "t_ns,t_s,value", &counter_rows}};
    {
        auto reader = log::LogReader::open(log_path, mission);  // scoped: closed before cleanup
        REQUIRE(reader);
        const auto dumped = log::dump_records(*reader, out_dir, formats);
        REQUIRE(dumped);
        CHECK(dumped->records == 3u);
        REQUIRE(dumped->files.size() == 2u);  // tick (3), then the mission's channel (9)
        CHECK(dumped->files[0].file_name == "tick.csv");
        CHECK(dumped->files[1].file_name == "counter.csv");
        CHECK(dumped->files[1].rows == 2u);
    }
    CHECK(read_all(out_dir / "counter.csv") == "t_ns,t_s,value\n1000000000,1,7\n2000000000,2,8\n");
    CHECK(read_all(out_dir / "tick.csv") == "t_ns,t_s\n1000000000,1\n");

    // A mission record its format cannot read is an error that names the file.
    {
        auto w = log::LogWriter::open(log_path, log::LogHeader{});
        REQUIRE(w);
        w->write(kCounter, Timestamp{5}, {});
        REQUIRE(w->close());
    }
    {
        auto reader = log::LogReader::open(log_path, mission);
        REQUIRE(reader);
        const auto bad = log::dump_records(*reader, out_dir, formats);
        REQUIRE_FALSE(bad);
        CHECK(bad.error() == "undecodable counter.csv record at t=5");
    }
    std::filesystem::remove(log_path, ec);
    std::filesystem::remove_all(out_dir, ec);
    std::filesystem::remove(log_path.parent_path(), ec);  // this process's directory, now empty
}
