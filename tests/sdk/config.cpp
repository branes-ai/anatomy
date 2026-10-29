// Unit tests for the baseline loader (config/baseline.hpp): the repo's files load,
// emit → parse round-trips to an equal value, and each invariant violation is
// reported with the offending key.

#include <branes/anatomy/config/baseline.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <string_view>

using namespace branes::anatomy;
using namespace branes::anatomy::config;

namespace {

const std::filesystem::path src = ANATOMY_SOURCE_DIR;

/// Replace the first occurrence of `from` in `s` with `to`. Pre: `from` occurs in `s`.
std::string replace(std::string s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    REQUIRE(at != std::string::npos);
    return s.replace(at, from.size(), to);
}

// A test mission's three slots. These one-argument loaders are what a mission
// writes over the slot-list-taking ones (racing-drone: config.hpp).
constexpr std::array<std::string_view, 3> kTestSlots = {"controller", "estimator", "safety_filter"};

result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path) {
    return config::load_baseline(path, kTestSlots);
}
result<BaselineConfig, ConfigError> parse_baseline(std::string_view yaml, std::string_view origin) {
    return config::parse_baseline(yaml, origin, kTestSlots);
}
std::string composition_hash(const BaselineConfig& b) {
    return config::composition_hash(b, kTestSlots);
}

}  // namespace

TEST_CASE("baseline: a baseline loads; composition hash is canonical", "[config]") {
    const auto b = load_baseline(src / "tests/fixtures/baseline.yaml");
    REQUIRE(b);
    CHECK(b->name == "test");
    CHECK(b->composition.size() == kTestSlots.size());
    CHECK(numeric_parameter(*b, "control_rate_hz") == 100.0);
    CHECK_FALSE(numeric_parameter(*b, "absent"));

    const auto back = parse_baseline(emit_baseline(*b), "emitted");
    REQUIRE(back);
    REQUIRE(*back == *b);
    CHECK(composition_hash(*back) == composition_hash(*b));

    auto other = *b;
    other.composition["controller"] = "Mpcc";
    CHECK(composition_hash(other) != composition_hash(*b));  // the hash covers the composition
    other = *b;
    other.parameters["control_rate_hz"] = "200";
    CHECK(composition_hash(other) == composition_hash(*b));  // ... and not the parameters
}

TEST_CASE("baseline: name rule, missing slot, unknown slot", "[config]") {
    const std::string slots = "composition:\n  estimator: A\n  controller: A\n  safety_filter: A\n";
    CHECK(parse_baseline("name: ok-1\n" + slots, "case"));
    CHECK(parse_baseline("name: Bad_Name\n" + slots, "case").error().key == "name");
    CHECK(parse_baseline("name: x\n" + replace(slots, "  safety_filter: A\n", ""), "case").error().key ==
          "composition.safety_filter");
    CHECK(parse_baseline("name: x\n" + slots + "  pilot_arbiter: A\n", "case").error().key ==
          "composition.pilot_arbiter");
}

TEST_CASE("baseline: the loader takes another mission's slot list", "[config]") {
    const std::array<std::string_view, 2> slots = {"controller", "planner"};
    const std::string yaml = "name: two\ncomposition:\n  controller: A\n  planner: B\n";
    const auto b = parse_baseline(yaml, "case", slots);
    REQUIRE(b);
    CHECK(b->composition.size() == 2u);
    CHECK(composition_hash(*b, slots) != composition_hash(*b));  // the test mission's slots hash differently
    CHECK(parse_baseline(yaml, "case").error().key == "composition.planner");  // not one of the test mission's slots
    CHECK(parse_baseline(yaml + "  estimator: C\n", "case", slots).error().key == "composition.estimator");
}
