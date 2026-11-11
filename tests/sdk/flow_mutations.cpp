// The flow graph's mutation tests (docs/plan/flow-testing, T2): every valid
// random recipe, mutated to break one invariant, is refused by the builder,
// whose error names the invariant and the node, firing or output the
// mutation changed (P6). Its baseline file, mutated, is refused by
// check_baseline(), naming G8 and the key it broke.

#include <branes/anatomy/flow/graph.hpp>

#include "../fixtures/flow/generate.hpp"
#include "../fixtures/flow/mutate.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

namespace {

constexpr std::uint64_t kSeeds = 200;

}  // namespace

TEST_CASE("flow mutations: each breaks one invariant, which the builder names", "[flow][mutation]") {
    std::map<std::string, std::size_t> applied;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        const gen::Recipe valid = gen::generate(seed);
        if (!gen::build(valid)) {
            INFO(gen::to_string(valid));
            FAIL("the valid recipe doesn't build");
        }
        for (const mutate::Mutation& m : mutate::mutations(valid)) {
            ++applied[m.what];
            const auto b = gen::build(m.recipe);
            if (b) {
                INFO(gen::to_string(m.recipe));
                FAIL(m.what + ": the builder accepts it");
            }
            const auto& e = b.error();
            if (e.message.rfind(m.invariant + ":", 0) != 0 || e.key != m.key) {
                INFO(gen::to_string(m.recipe));
                FAIL(m.what + ": expected " + m.invariant + " at '" + m.key + "', got '" + e.key + "': " + e.message);
            }
        }
    }
    // Every mutation applied to every seed, but two nodes of one name, to the seeds with two nodes.
    CHECK(applied.size() == 20);
    for (const auto& [what, n] : applied) {
        INFO(what);
        if (what == "G1 two nodes of one name") {
            CHECK(n > 0);
        } else {
            CHECK(n == kSeeds);
        }
    }
}

TEST_CASE("flow mutations: each mutation of the baseline file is a G8 check_baseline() names",
          "[flow][mutation][config]") {
    std::size_t scoreboard = 0;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        const gen::Recipe valid = gen::generate(seed);
        const auto b = gen::build(valid);
        REQUIRE(b);
        const branes::anatomy::flow::Graph& g = b->stack.graph();
        scoreboard += g.kind == branes::anatomy::flow::baseline_class::scoreboard ? 1 : 0;
        INFO(gen::to_string(valid));
        REQUIRE(branes::anatomy::flow::check_baseline(mutate::baseline_of(g), "toys.yaml", g));
        for (const mutate::FileMutation& m : mutate::file_mutations(g)) {
            const auto checked = branes::anatomy::flow::check_baseline(m.file, "toys.yaml", g);
            if (checked) {
                FAIL(m.what + ": check_baseline() accepts it");
            }
            const auto& e = checked.error();
            if (e.message.rfind("G8:", 0) != 0 || e.key != m.key || e.file != "toys.yaml") {
                FAIL(m.what + ": expected G8 at '" + m.key + "', got '" + e.key + "': " + e.message);
            }
        }
    }
    CHECK(scoreboard > 0);
    CHECK(scoreboard < kSeeds);
}
