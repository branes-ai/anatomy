// The flow graph's metamorphic tests (docs/plan/flow-testing, T3): changes
// to a graph that preserve its meaning preserve its records, and change its
// hash (P7). A scoreboard graph against the research graph of the same
// operators and wiring; two independent firings in the other order; every
// name changed. And the swap the other way: a read of a state and its
// update, in the other order, which the builder accepts, and whose records
// the interpreter still gives (P5).

#include "../fixtures/flow/drive.hpp"
#include "../fixtures/flow/generate.hpp"
#include "../fixtures/flow/metamorphic.hpp"
#include "../fixtures/flow/toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace {

using drive::hand;
using drive::Step;
using toys::Num;

constexpr std::uint64_t kSeeds = 300;
constexpr std::size_t kSteps = 500;

/// How `a` and its twin differ: in their hashes, which must, or in an
/// output after an input, which must not; an empty string if as they must.
std::string difference(const gen::Recipe& a, const metamorphic::Twin& twin) {
    auto ba = gen::build(a);
    auto bb = gen::build(twin.recipe);
    if (!ba || !bb) {
        return "a recipe doesn't build: " + (ba ? bb.error() : ba.error()).to_string();
    }
    if (ba->stack.hash() == bb->stack.hash()) {
        return "the hashes are equal";
    }
    // Each output of a, and its twin's.
    std::vector<std::size_t> pair;
    for (std::size_t o = 0; o < ba->origins.size(); ++o) {
        const gen::Origin& from = ba->origins[o];
        const gen::Origin want{twin.moved[from.firing], from.port, from.field};
        std::optional<std::size_t> found;
        for (std::size_t p = 0; p < bb->origins.size() && !found; ++p) {
            if (bb->origins[p] == want) {
                found = p;
            }
        }
        if (!found) {
            return "output " + std::to_string(o) + " has no twin";
        }
        pair.push_back(*found);
    }
    const auto steps = drive::stream(a, kSteps, a.seed ^ 0x7417ULL);
    for (std::size_t i = 0; i < steps.size(); ++i) {
        hand(*ba, steps[i]);
        hand(*bb, steps[i]);
        for (std::size_t o = 0; o < pair.size(); ++o) {
            const Num& x = ba->stack.output(ba->outputs[o]);
            const Num& y = bb->stack.output(bb->outputs[pair[o]]);
            if (x.t != y.t || x.v != y.v) {
                return "after input " + std::to_string(i) + ", output " + std::to_string(o) + ": (" +
                       std::to_string(x.t.ns) + ", " + std::to_string(x.v) + ") against its twin's (" +
                       std::to_string(y.t.ns) + ", " + std::to_string(y.v) + ")";
            }
        }
    }
    return {};
}

void require_same_records(const gen::Recipe& a, const metamorphic::Twin& twin) {
    const std::string diff = difference(a, twin);
    if (!diff.empty()) {
        INFO(gen::to_string(a));
        INFO(gen::to_string(twin.recipe));
        FAIL(diff);
    }
}

}  // namespace

TEST_CASE("flow metamorphic: a scoreboard graph's records are its research graph's", "[flow][metamorphic]") {
    std::size_t graphs = 0;
    std::size_t bodies = 0;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        if (r.kind != branes::anatomy::flow::baseline_class::scoreboard) {
            continue;
        }
        ++graphs;
        for (const gen::NodeSpec& n : r.nodes) {
            bodies += n.body ? 1 : 0;
        }
        require_same_records(r, metamorphic::as_research(r));
    }
    INFO(graphs << " scoreboard graphs, " << bodies << " bodies");
    CHECK(graphs > 0);
    CHECK(bodies > 0);
}

TEST_CASE("flow metamorphic: independent firings in the other order give the same records", "[flow][metamorphic]") {
    std::size_t swaps = 0;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        if (const auto twin = metamorphic::swapped(r)) {
            ++swaps;
            require_same_records(r, *twin);
        }
    }
    INFO(swaps << " swaps");
    CHECK(swaps > 0);
}

TEST_CASE("flow metamorphic: renaming changes the hash, not the records", "[flow][metamorphic]") {
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        require_same_records(r, metamorphic::renamed(r));
    }
}

TEST_CASE("flow metamorphic: a read and an update in the other order still run as the graph says",
          "[flow][metamorphic]") {
    // Rarer than the other swaps, so more seeds: only those that have one are run.
    std::size_t swaps = 0;
    for (std::uint64_t seed = 1; seed <= 4000; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        if (const auto twin = metamorphic::read_update_swapped(r)) {
            ++swaps;
            for (const gen::Recipe* x : {&r, &twin->recipe}) {
                const std::string diff = drive::from_interpreter(*x, kSteps);
                if (!diff.empty()) {
                    INFO(gen::to_string(*x));
                    FAIL(diff);
                }
            }
        }
    }
    INFO(swaps << " swaps");
    CHECK(swaps > 0);
}

TEST_CASE("flow metamorphic: a read before an update and one after it differ", "[flow][metamorphic]") {
    // in0's region: Counter1.read at in0 and Counter1.add(in0), in both orders,
    // then Counter1.peek.
    gen::Recipe before;
    before.inputs = {{"in0", false, true}};
    before.nodes = {{"n0", gen::Kind::counter, 1, false}};
    before.firings = {
        {0, 1, 0, {}}, {0, 0, 0, {gen::SourceSpec{gen::SourceSpec::From::input, 0, 0, -1, false}}}, {0, 2, 0, {}}};
    const metamorphic::Twin after = metamorphic::swap_at(before, 0);

    auto b = gen::build(before);
    auto a = gen::build(after.recipe);
    REQUIRE(b);
    REQUIRE(a);
    CHECK(b->stack.hash() != a->stack.hash());
    hand(*b, Step{0, Num{branes::anatomy::Timestamp{1000}, 7}});
    hand(*a, Step{0, Num{branes::anatomy::Timestamp{1000}, 7}});
    // o0 is the read: of the state before the update, then after it.
    CHECK(b->stack.output(b->outputs[0]).v == 0);
    CHECK(a->stack.output(a->outputs[0]).v == toys::mix(7, 1));
    for (const gen::Recipe* x : {static_cast<const gen::Recipe*>(&before), &after.recipe}) {
        INFO(gen::to_string(*x));
        CHECK(drive::from_interpreter(*x, kSteps).empty());
    }
}
