// Unit tests for the replay log (log/log.hpp): records round-trip, identical
// writes give identical files, and damaged files are reported, not misread.

#include <branes/anatomy/log/log.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

using namespace branes::anatomy;
using namespace branes::anatomy::log;

namespace {

/// A mission's own channel, at the id a mission might use (racing-drone: 5).
constexpr Channel kMissionChannel = static_cast<Channel>(5);
/// The test mission's channels, for LogReader::open.
constexpr std::array<Channel, 1> kMission = {kMissionChannel};

std::filesystem::path temp_file(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("anatomy_log_test_" + name + ".brlog");
}

LogHeader sample_header() {
    LogHeader h;
    h.baseline_name = "null";
    h.composition_hash = "abc";
    h.world_hash = "c";
    h.vehicle_hash = "v";
    h.world_yaml = "name: course\n";
    h.vehicle_yaml = "name: vehicle\n";
    return h;
}

void write_sample(const std::filesystem::path& p) {
    auto w = LogWriter::open(p, sample_header());
    REQUIRE(w);
    const std::vector<std::byte> payload = {std::byte{1}, std::byte{2}, std::byte{3}};
    w->write(Channel::imu, Timestamp{10}, payload);
    w->write(Channel::tick, Timestamp{10}, {});
    w->write(Channel::imu, Timestamp{20}, payload);
    REQUIRE(w->record_count() == 3u);
    REQUIRE(w->close());
    REQUIRE_FALSE(w->is_open());
}

/// Remove a test file without throwing (on Windows a still-open file cannot be removed).
void cleanup(const std::filesystem::path& p) {
    std::error_code ec;
    std::filesystem::remove(p, ec);
}

std::vector<char> read_all(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST_CASE("log: header and records round-trip", "[log]") {
    const auto p = temp_file("roundtrip");
    write_sample(p);

    std::vector<LogRecord> recs;
    {
        // Scoped: the reader closes its file before cleanup() (Windows cannot
        // remove an open file).
        auto r = LogReader::open(p, kMission);
        REQUIRE(r);
        REQUIRE(r->header() == sample_header());
        for (;;) {
            auto rec = r->next();
            REQUIRE(rec);
            if (!rec->has_value()) {
                break;
            }
            recs.push_back(**rec);
        }
    }
    REQUIRE(recs.size() == 3u);
    CHECK(recs[0].channel == Channel::imu);
    CHECK(recs[0].t == Timestamp{10});
    CHECK(recs[0].payload.size() == 3u);
    CHECK(recs[1].channel == Channel::tick);
    CHECK(recs[1].payload.empty());
    CHECK(recs[2].t == Timestamp{20});
    cleanup(p);
}

TEST_CASE("log: identical writes give byte-identical files", "[log]") {
    const auto a = temp_file("same_a");
    const auto b = temp_file("same_b");
    write_sample(a);
    write_sample(b);
    REQUIRE(read_all(a) == read_all(b));
    cleanup(a);
    cleanup(b);
}

TEST_CASE("log: a moved-from writer no longer owns the file", "[log]") {
    const auto p = temp_file("moved");
    auto w = LogWriter::open(p, sample_header());
    REQUIRE(w);
    LogWriter moved = std::move(*w);
    REQUIRE(moved.is_open());
    REQUIRE_FALSE(w->is_open());
    REQUIRE(moved.close());
    cleanup(p);
}

TEST_CASE("log: bad magic, truncation, and missing files are errors", "[log]") {
    CHECK_FALSE(LogReader::open(temp_file("does_not_exist"), kMission));

    const auto bad = temp_file("bad_magic");
    {
        std::ofstream out(bad, std::ios::binary);
        out << "NOTALOG.................";
    }
    const auto r = LogReader::open(bad, kMission);
    REQUIRE_FALSE(r);
    CHECK(r.error().message == "not a log (bad magic)");
    cleanup(bad);

    const auto cut = temp_file("truncated");
    write_sample(cut);
    const auto size = std::filesystem::file_size(cut);
    std::filesystem::resize_file(cut, size - 2);  // cut into the last payload
    {
        auto reader = LogReader::open(cut, kMission);  // scoped: closed before cleanup()
        REQUIRE(reader);
        REQUIRE(*reader->next());  // records 1 and 2 are intact
        REQUIRE(*reader->next());
        const auto last = reader->next();
        REQUIRE_FALSE(last);
        CHECK(last.error().message == "truncated record payload");
    }
    cleanup(cut);
}

TEST_CASE("log: a version-1 log (before the reference channel) is rejected", "[log]") {
    const auto p = temp_file("version1");
    write_sample(p);
    {
        std::fstream f(p, std::ios::binary | std::ios::in | std::ios::out);
        f.seekp(8);  // the version follows the 8-byte magic
        const char v1[4] = {1, 0, 0, 0};
        f.write(v1, 4);
    }
    {
        const auto r = LogReader::open(p, kMission);  // scoped: closed before cleanup()
        REQUIRE_FALSE(r);
        CHECK(r.error().message == "unsupported log version 1");
    }
    REQUIRE(kLogVersion == 2u);
    cleanup(p);
}

TEST_CASE("log: a mission channel reads only when the mission names it", "[log]") {
    const auto p = temp_file("mission_channel");
    {
        auto w = LogWriter::open(p, sample_header());
        REQUIRE(w);
        w->write(kMissionChannel, Timestamp{10}, {});
        REQUIRE(w->close());
    }
    {
        auto r = LogReader::open(p, kMission);  // scoped: closed before cleanup()
        REQUIRE(r);
        const auto rec = r->next();
        REQUIRE(rec);
        REQUIRE(rec->has_value());
        CHECK((*rec)->channel == kMissionChannel);
    }
    {
        auto r = LogReader::open(p, {});  // no mission channels: 5 is unknown
        REQUIRE(r);
        const auto rec = r->next();
        REQUIRE_FALSE(rec);
        CHECK(rec.error().message == "unknown channel 5");
    }
    cleanup(p);
}

TEST_CASE("log: mission channels must not reuse a framework id or repeat (I2)", "[log]") {
    const auto p = temp_file("mission_i2");
    write_sample(p);
    const std::array<Channel, 1> framework = {Channel::imu};
    const std::array<Channel, 2> twice = {kMissionChannel, kMissionChannel};
    CHECK_FALSE(LogReader::open(p, framework));
    CHECK_FALSE(LogReader::open(p, twice));
    CHECK(LogReader::open(p, kMission));
    cleanup(p);
}
