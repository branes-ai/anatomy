// The graphs' canonical forms, against goldens recorded before the
// `accumulate` rate rule (docs/plan/workbench, B2, property W9): the hash of
// the canonical form of each of the test mission's graphs, and of 200
// generated graphs drawn without `accumulate`, equals the hash main had. A
// change to canonical() that touches a graph without an `accumulate` edge
// changes one of them.
//
// The goldens are tests/fixtures/flow/goldens/canonical.txt. To record them
// again, which only a change meant to alter every graph's hash should do, run
// this test with ANATOMY_WRITE_GOLDENS=1.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>

#include "../fixtures/flow/generate.hpp"
#include "../fixtures/flow/test_mission.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>

namespace {

namespace fl = branes::anatomy::flow;

const std::filesystem::path kGoldens =
    std::filesystem::path(ANATOMY_SOURCE_DIR) / "tests" / "fixtures" / "flow" / "goldens" / "canonical.txt";

constexpr std::uint64_t kSeeds = 200;

/// ANATOMY_WRITE_GOLDENS is set. MSVC's getenv is deprecated (C4996), so it reads a copy.
bool write_goldens() {
#ifdef _MSC_VER
    char* s = nullptr;
    std::size_t len = 0;
    const bool set = _dupenv_s(&s, &len, "ANATOMY_WRITE_GOLDENS") == 0 && s != nullptr;
    std::free(s);
    return set;
#else
    return std::getenv("ANATOMY_WRITE_GOLDENS") != nullptr;
#endif
}

std::string hash_of(fl::baseline_class kind, const std::function<void(fl::builder&)>& describe) {
    fl::builder g{"test", kind};
    describe(g);
    const auto built = g.build();
    if (!built) {
        FAIL(built.error().to_string());
    }
    return fl::graph_hash(*built);
}

/// One line per graph: its name and the hash of its canonical form.
std::string goldens() {
    using Several = test::Bodies<test::TestEstimatorOp, test::NullMonitorOp, test::GainTrack<>>;
    std::ostringstream o;
    o << "# The hash of each graph's canonical form (tests/sdk/flow_goldens.cpp).\n";
    o << "test.scoreboard " << hash_of(fl::baseline_class::scoreboard, [](fl::builder& g) { test::scoreboard(g); })
      << "\n";
    o << "test.body "
      << hash_of(fl::baseline_class::scoreboard, [](fl::builder& g) { test::scoreboard<Several>(g); }) << "\n";
    o << "test.research "
      << hash_of(fl::baseline_class::research, [](fl::builder& g) { (void)test::research(g); }) << "\n";
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        const auto b = gen::build(gen::generate(seed));
        if (!b) {
            FAIL("seed " << seed << " doesn't build: " << b.error().to_string());
        }
        o << "seed." << seed << " " << b->stack.hash() << "\n";
    }
    return o.str();
}

}  // namespace

TEST_CASE("flow goldens: every graph without accumulate keeps main's canonical form (W9)", "[flow][goldens]") {
    const std::string now = goldens();
    if (write_goldens()) {
        std::filesystem::create_directories(kGoldens.parent_path());
        std::ofstream(kGoldens, std::ios::binary) << now;
        WARN("wrote " << kGoldens.string());
    }
    std::ifstream in(kGoldens, std::ios::binary);
    REQUIRE(in);
    std::stringstream recorded;
    recorded << in.rdbuf();

    // Line by line, so a failure names the graph.
    std::istringstream want(recorded.str());
    std::istringstream got(now);
    std::string w;
    std::string g;
    std::size_t lines = 0;
    while (std::getline(want, w)) {
        if (!w.empty() && w.back() == '\r') {
            w.pop_back();  // a checkout that converts line endings
        }
        REQUIRE(std::getline(got, g));
        CHECK(g == w);
        ++lines;
    }
    CHECK_FALSE(std::getline(got, g));
    CHECK(lines == 4 + kSeeds);
}
