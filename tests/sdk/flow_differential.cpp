// The flow graph's differential tests (docs/plan/flow-testing, T1): random
// graphs of toy operators, run by the CPU stack (flow::stack) and by the
// reference interpreter, which must agree on every output after every input
// (P1, P2), accumulate edges among them (docs/plan/workbench, W6); a hold
// before its first write (P4); a read of state before and after its update
// (P5); and a running stack that allocates nothing (P9, W8).
// tests/sdk/flow_accumulate.cpp tests the windows themselves.
//
// ANATOMY_FLOW_SEEDS sets how many seeds the differential test runs: 500 by
// default, more for a longer run, which CI doesn't make.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/msg/time.hpp>

#include "../fixtures/flow/drive.hpp"
#include "../fixtures/flow/generate.hpp"
#include "../fixtures/flow/interpreter.hpp"
#include "../fixtures/flow/toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <new>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

// Every allocation in this test, for P9. The replacements pair malloc with
// free; GCC, inlining them into the library's new and delete, can't see that
// they pair, and warns.
namespace {
std::atomic<std::size_t> g_allocations{0};
}  // namespace

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmismatched-new-delete"
#endif
void* operator new(std::size_t n) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n == 0 ? 1 : n)) {
        return p;
    }
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept {
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif

namespace {

using branes::anatomy::Timestamp;
using drive::hand;
using drive::interpreter_of;
using drive::Step;
using drive::stream;
using toys::Num;

/// ANATOMY_FLOW_SEEDS, or 500. MSVC's getenv is deprecated (C4996), so it reads a copy.
std::size_t seeds() {
    std::size_t n = 500;
#ifdef _MSC_VER
    char* s = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&s, &len, "ANATOMY_FLOW_SEEDS") == 0 && s != nullptr) {
        n = static_cast<std::size_t>(std::strtoull(s, nullptr, 10));
    }
    std::free(s);
#else
    if (const char* s = std::getenv("ANATOMY_FLOW_SEEDS")) {
        n = static_cast<std::size_t>(std::strtoull(s, nullptr, 10));
    }
#endif
    return n;
}

using gen::SourceSpec;
using From = SourceSpec::From;

}  // namespace

TEST_CASE("flow differential: the stack runs what the graph says", "[flow][differential]") {
    const std::size_t n = seeds();
    std::size_t scoreboard = 0;
    std::size_t bodies = 0;
    std::size_t holds = 0;
    std::size_t projections = 0;
    std::size_t accumulates = 0;
    std::vector<std::size_t> kinds(gen::kAllKinds, 0);
    for (std::uint64_t seed = 1; seed <= n; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        const std::string diff = drive::from_interpreter(r, 1000);
        if (!diff.empty()) {
            INFO(gen::to_string(r));
            FAIL(diff);
        }
        scoreboard += r.kind == branes::anatomy::flow::baseline_class::scoreboard ? 1 : 0;
        for (const gen::NodeSpec& node : r.nodes) {
            ++kinds[static_cast<std::size_t>(node.kind)];
            bodies += node.body ? 1 : 0;
        }
        for (const gen::FiringSpec& f : r.firings) {
            for (const SourceSpec& s : f.in) {
                holds += s.hold ? 1 : 0;
                projections += s.field >= 0 ? 1 : 0;
                accumulates += s.accumulate > 0 ? 1 : 0;
            }
        }
    }
    // What the seeds covered: both classes, every kind, bodies, holds, projections and accumulates.
    INFO("seeds " << n << ": " << scoreboard << " scoreboard, " << bodies << " bodies, " << holds << " holds, "
                  << projections << " projections, " << accumulates << " accumulates");
    CHECK(scoreboard > 0);
    CHECK(scoreboard < n);
    CHECK(bodies > 0);
    CHECK(holds > 0);
    CHECK(projections > 0);
    CHECK(accumulates > 0);
    for (std::size_t k = 0; k < gen::kAllKinds; ++k) {
        INFO(gen::kind_name(static_cast<gen::Kind>(k)));
        // Acc is the mutations' only.
        CHECK((static_cast<gen::Kind>(k) == gen::Kind::acc) == (kinds[k] == 0));
    }
}

TEST_CASE("flow differential: a seed gives one recipe", "[flow][differential]") {
    CHECK(gen::to_string(gen::generate(7)) == gen::to_string(gen::generate(7)));
    CHECK(gen::to_string(gen::generate(7)) != gen::to_string(gen::generate(8)));
}

TEST_CASE("flow differential: P4 a hold reads its message value-initialized before its first write",
          "[flow][differential]") {
    // in1's region reads hold(#0.y), which in0's region writes, and hold(in0),
    // before in0 has been handed anything.
    gen::Recipe r;
    r.inputs = {{"in0", false, true}, {"in1", false, true}};
    r.nodes = {
        {"n0", gen::Kind::unary, 1, false}, {"n1", gen::Kind::unary, 2, false}, {"n2", gen::Kind::unary, 3, false}};
    r.firings = {{0, 0, 0, {SourceSpec{From::input, 0, 0, -1, false}}},
                 {1, 0, 1, {SourceSpec{From::firing, 0, 0, -1, true}}},
                 {2, 0, 1, {SourceSpec{From::input, 0, 0, -1, true}}}};
    auto b = gen::build(r);
    REQUIRE(b);
    interp::Interpreter in = interpreter_of(r, *b);

    const Step first{1, Num{Timestamp{5}, 42}};
    hand(*b, first);
    in.input("in1", interp::Val{5, 42});
    // o1 is n1's: Unary2 of a value-initialized Num; o2 is n2's: Unary3 of one.
    CHECK(b->stack.output(b->outputs[1]).t.ns == 0);
    CHECK(b->stack.output(b->outputs[1]).v == toys::mix(0, 2));
    CHECK(b->stack.output(b->outputs[2]).v == toys::mix(0, 3));
    CHECK(in.output(1) == interp::Val{0, toys::mix(0, 2)});
    CHECK(in.output(2) == interp::Val{0, toys::mix(0, 3)});
    // And o0, whose region hasn't run, is value-initialized itself.
    CHECK(b->stack.output(b->outputs[0]).v == 0);
    CHECK(in.output(0) == interp::Val{});
}

TEST_CASE("flow differential: P5 a read before an update sees the last state, one after it the new",
          "[flow][differential]") {
    // in0's region: Counter1.read at in0, Counter1.add(in0), Counter1.peek.
    gen::Recipe r;
    r.inputs = {{"in0", false, true}};
    r.nodes = {{"n0", gen::Kind::counter, 1, false}};
    r.firings = {{0, 1, 0, {}}, {0, 0, 0, {SourceSpec{From::input, 0, 0, -1, false}}}, {0, 2, 0, {}}};
    auto b = gen::build(r);
    REQUIRE(b);
    interp::Interpreter in = interpreter_of(r, *b);

    std::uint64_t sum = 0;
    for (const std::uint64_t x : {7ULL, 11ULL, 13ULL}) {
        const auto t = static_cast<std::int64_t>(x * 1000);
        hand(*b, Step{0, Num{Timestamp{t}, x}});
        in.input("in0", interp::Val{t, x});
        const Num before = b->stack.output(b->outputs[0]);
        const Num after = b->stack.output(b->outputs[1]);
        CHECK(before.t.ns == t);
        CHECK(before.v == sum);
        sum = toys::mix(sum + x, 1);
        CHECK(after.v == sum);
        CHECK(in.output(0) == interp::Val{t, before.v});
        CHECK(in.output(1) == interp::Val{0, after.v});
    }
}

TEST_CASE("flow differential: P9 a running stack allocates nothing", "[flow][differential]") {
    // Seeds whose graphs are of several nodes; every toy message is of fixed
    // size, and so is every window (W8).
    std::size_t accumulates = 0;
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        for (const gen::FiringSpec& f : r.firings) {
            for (const SourceSpec& s : f.in) {
                accumulates += s.accumulate > 0 ? 1 : 0;
            }
        }
        auto b = gen::build(r);
        REQUIRE(b);
        const auto steps = stream(r, 1000, seed);
        for (std::size_t i = 0; i < 10; ++i) {
            hand(*b, steps[i]);
        }
        const std::size_t before = g_allocations.load();
        for (const Step& s : steps) {
            hand(*b, s);
        }
        const std::size_t made = g_allocations.load() - before;
        INFO(gen::to_string(r));
        CHECK(made == 0);
    }
    CHECK(accumulates > 0);
}
