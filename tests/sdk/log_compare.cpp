// Unit tests for log::compare() (log/compare.hpp): two logs are equal when
// their headers are, but for the fields left out, and their records are byte
// for byte; the first difference is named, and an unreadable log is an error.

#include <branes/anatomy/log/compare.hpp>
#include <branes/anatomy/log/log.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

using namespace branes::anatomy;
using namespace branes::anatomy::log;

namespace {

/// A mission's own channel, which compare() reads without being told of it.
constexpr Channel kMissionChannel = static_cast<Channel>(5);

std::filesystem::path temp_file(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("anatomy_log_compare_test_" + name + ".brlog");
}

LogHeader sample_header() {
    LogHeader h;
    h.baseline_name = "null";
    h.composition_hash = "slots";
    h.world_hash = "w";
    h.vehicle_hash = "v";
    h.world_yaml = "name: world\n";
    h.vehicle_yaml = "name: vehicle\n";
    return h;
}

struct Rec {
    Channel channel;
    std::int64_t t;
    std::vector<std::byte> payload;
};

std::vector<Rec> sample_records() {
    return {{Channel::imu, 10, {std::byte{1}, std::byte{2}}},
            {kMissionChannel, 10, {std::byte{3}}},
            {Channel::tick, 10, {}},
            {Channel::command, 10, {std::byte{4}, std::byte{5}}}};
}

void write_log(const std::filesystem::path& p, const LogHeader& h, const std::vector<Rec>& records) {
    auto w = LogWriter::open(p, h);
    REQUIRE(w);
    for (const Rec& r : records) {
        w->write(r.channel, Timestamp{r.t}, r.payload);
    }
    REQUIRE(w->close());
}

void cleanup(const std::filesystem::path& p) {
    std::error_code ec;
    std::filesystem::remove(p, ec);
}

constexpr std::array<std::string_view, 1> kExceptHash = {"composition_hash"};

}  // namespace

TEST_CASE("log compare: equal logs are equal, with every record counted", "[log][compare]") {
    const auto a = temp_file("equal_a");
    const auto b = temp_file("equal_b");
    write_log(a, sample_header(), sample_records());
    write_log(b, sample_header(), sample_records());

    const auto c = compare(a, b, {});
    REQUIRE(c);
    CHECK_FALSE(c->difference);
    CHECK(c->records == 4u);
    cleanup(a);
    cleanup(b);
}

TEST_CASE("log compare: a header field left out may differ, and no other", "[log][compare]") {
    const auto a = temp_file("hash_a");
    const auto b = temp_file("hash_b");
    LogHeader h = sample_header();
    write_log(a, h, sample_records());
    h.composition_hash = "graph";
    write_log(b, h, sample_records());

    const auto all = compare(a, b, {});
    REQUIRE(all);
    REQUIRE(all->difference);
    CHECK(*all->difference == "the header's composition_hash differs: 'slots' and 'graph'");

    const auto except = compare(a, b, kExceptHash);
    REQUIRE(except);
    CHECK_FALSE(except->difference);
    CHECK(except->records == 4u);

    h.world_yaml = "name: another world\n";
    write_log(b, h, sample_records());
    const auto yaml = compare(a, b, kExceptHash);
    REQUIRE(yaml);
    REQUIRE(yaml->difference);
    CHECK(*yaml->difference == "the header's world_yaml differs");
    cleanup(a);
    cleanup(b);
}

TEST_CASE("log compare: the first record that differs is named", "[log][compare]") {
    const auto a = temp_file("records_a");
    const auto b = temp_file("records_b");
    write_log(a, sample_header(), sample_records());

    SECTION("a payload") {
        auto r = sample_records();
        r[3].payload[1] = std::byte{6};
        write_log(b, sample_header(), r);
        const auto c = compare(a, b, {});
        REQUIRE(c);
        REQUIRE(c->difference);
        CHECK(*c->difference == "record 3 (channel 17 at t=10 ns) has another payload");
        CHECK(c->records == 3u);
    }
    SECTION("a channel") {
        auto r = sample_records();
        r[1].channel = Channel::camera;
        write_log(b, sample_header(), r);
        const auto c = compare(a, b, {});
        REQUIRE(c);
        REQUIRE(c->difference);
        CHECK(*c->difference == "record 1 is channel 5 at t=10 ns in one and channel 2 at t=10 ns in the other");
    }
    SECTION("a time") {
        auto r = sample_records();
        r[0].t = 9;
        write_log(b, sample_header(), r);
        const auto c = compare(a, b, {});
        REQUIRE(c);
        REQUIRE(c->difference);
        CHECK(*c->difference == "record 0 is channel 1 at t=10 ns in one and channel 1 at t=9 ns in the other");
    }
    SECTION("a log with more records") {
        auto r = sample_records();
        r.pop_back();
        write_log(b, sample_header(), r);
        const auto c = compare(a, b, {});
        REQUIRE(c);
        REQUIRE(c->difference);
        CHECK(*c->difference == a.string() + " has more records: record 3 is channel 17 at t=10 ns");
        CHECK(c->records == 3u);
    }
    cleanup(a);
    cleanup(b);
}

TEST_CASE("log compare: an unreadable log and an unknown field are errors", "[log][compare]") {
    const auto a = temp_file("errors_a");
    const auto b = temp_file("errors_b");
    write_log(a, sample_header(), sample_records());

    constexpr std::array<std::string_view, 1> unknown = {"compositon_hash"};
    const auto field = compare(a, a, unknown);
    REQUIRE_FALSE(field);
    CHECK(field.error().message == "'compositon_hash' is not a header field");

    cleanup(b);
    const auto missing = compare(a, b, {});
    REQUIRE_FALSE(missing);
    CHECK(missing.error().path == b.string());

    // b: a's bytes, its last record cut short.
    {
        std::ifstream in(a, std::ios::binary);
        std::vector<char> bytes{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
        bytes.pop_back();
        std::ofstream out(b, std::ios::binary);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    const auto truncated = compare(a, b, {});
    REQUIRE_FALSE(truncated);
    CHECK(truncated.error().path == b.string());
    CHECK(truncated.error().message == "truncated record payload");
    cleanup(a);
    cleanup(b);
}
