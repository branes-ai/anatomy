// The MLP pattern (workbench/patterns/mlp.hpp; docs/plan/workbench, B3):
// its stack writes, bit for bit, what its operators write by hand (W1); its
// outputs meet its oracle (W2); a seed gives byte-identical traces run to run
// (W3); and its weights, constants, reach every firing by reference (W5).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/mlp.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
namespace mlp = branes::anatomy::workbench::mlp;
using branes::anatomy::Timestamp;

std::filesystem::path temp(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("anatomy_workbench_mlp_" + name);
}

std::vector<char> bytes_of(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

TEST_CASE("mlp: the graph is one region of six firings, its weights constants", "[workbench][mlp]") {
    const auto g = wb::describe<mlp::pattern>(mlp::config{});
    REQUIRE(g);
    REQUIRE(g->regions.size() == 1);
    CHECK(g->regions[0].trigger == "x");
    REQUIRE(g->regions[0].firings.size() == 6);
    CHECK(g->instances[0].op == "Linear_f32_32x16");
    CHECK(g->instances[3].op == "Cast_f32_f64_32");
    CHECK(g->instances[4].op == "Linear_f64_8x32");
    CHECK(g->regions[0].firings[0].in[0].from == fl::Ref::kind::constant);
    CHECK(g->regions[0].firings[0].in[0].port == "w1");
    // The canonical form is a function of the configuration's seed: its constants' values aren't in it.
    mlp::config other;
    other.seed = 2;
    CHECK(fl::canonical(*g) == fl::canonical(*wb::describe<mlp::pattern>(other)));
    STATIC_REQUIRE(mlp::multiply_adds == 32 * 16 + 8 * 32);
}

TEST_CASE("mlp: W1 the stack writes what the operators write by hand", "[workbench][mlp]") {
    const mlp::config cfg;
    const auto samples = wb::run<mlp::pattern>(cfg, wb::run_options{200, {}});
    REQUIRE(samples);
    REQUIRE(samples->size() == 200);
    const mlp::weights w = mlp::weights_of(cfg);
    for (std::size_t i = 0; i < samples->size(); ++i) {
        const mlp::sample want = mlp::hand_run(w, mlp::input_of(cfg, i));
        INFO("input " << i);
        CHECK((*samples)[i].hidden == want.hidden);
        CHECK((*samples)[i].y == want.y);
    }
}

TEST_CASE("mlp: W2 the outputs meet the oracle, for several seeds", "[workbench][mlp]") {
    for (std::uint64_t seed = 1; seed <= 5; ++seed) {
        mlp::config cfg;
        cfg.seed = seed;
        const auto samples = wb::run<mlp::pattern>(cfg, wb::run_options{100, {}});
        REQUIRE(samples);
        const wb::report r = mlp::pattern::oracle(cfg, *samples);
        INFO(r.to_string());
        CHECK(r.passed());
        REQUIRE(r.checks.size() == 3);
        // Layer 1's f32 sets the error: it is well above layer 2's own.
        CHECK(r.checks[1].value > r.checks[2].value);
    }
}

TEST_CASE("mlp: the oracle fails a wrong output", "[workbench][mlp]") {
    const mlp::config cfg;
    auto samples = wb::run<mlp::pattern>(cfg, wb::run_options{10, {}});
    REQUIRE(samples);
    (*samples)[3].y.elements[2] += 1e-3;
    const wb::report r = mlp::pattern::oracle(cfg, *samples);
    CHECK_FALSE(r.passed());
    CHECK_FALSE(r.checks[1].passed);
    CHECK_FALSE(r.checks[2].passed);
    CHECK(r.to_string().find("FAIL") != std::string::npos);
    // And with no inputs, it passes nothing.
    CHECK_FALSE(mlp::pattern::oracle(cfg, {}).passed());
}

TEST_CASE("mlp: W3 a seed's traces are byte-identical run to run", "[workbench][mlp]") {
    const auto a = temp("a.brtrace");
    const auto b = temp("b.brtrace");
    const auto c = temp("c.brtrace");
    mlp::config cfg;
    REQUIRE(wb::run<mlp::pattern>(cfg, wb::run_options{50, a}));
    REQUIRE(wb::run<mlp::pattern>(cfg, wb::run_options{50, b}));
    cfg.seed = 9;
    REQUIRE(wb::run<mlp::pattern>(cfg, wb::run_options{50, c}));

    const auto ab = fl::compare_traces(a, b);
    REQUIRE(ab);
    CHECK_FALSE(ab->difference);
    CHECK(ab->records == 50 * 6);
    CHECK(bytes_of(a) == bytes_of(b));
    // Every edge has a codec: a payload in each record. The reader closes the
    // file at the end of its block, before Windows will remove it.
    {
        auto reader = fl::trace_reader::open(a);
        REQUIRE(reader);
        for (auto rec = reader->next(); rec && *rec; rec = reader->next()) {
            CHECK((*rec)->payload.has_value());
        }
    }
    // Another seed: other weights, so the first firing already differs.
    const auto ac = fl::compare_traces(a, c);
    REQUIRE(ac);
    REQUIRE(ac->difference);
    CHECK(ac->difference->find("x[0].y (linear1.apply)") != std::string::npos);
    for (const auto& p : {a, b, c}) {
        std::error_code ec;
        std::filesystem::remove(p, ec);
    }
}

namespace {

/// Reads the MLP's first weights, and records where it read them and what.
struct Probe {
    std::vector<const void*>* seen = nullptr;
    std::vector<float>* first = nullptr;
};
struct ProbeContext {
    mutable std::vector<const void*> seen;
    mutable std::vector<float> first;
};
struct ProbeBinding {
    using signature = fl::signature<fl::ports<fl::port<"w", mlp::W1, fl::direction::in>,
                                              fl::port<"x", mlp::X, fl::direction::in>,
                                              fl::port<"y", mlp::X, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"w", "x">, fl::outs<"y">> {
        static mlp::X call(const Probe& op, const mlp::W1& w, const mlp::X& x) {
            op.seen->push_back(&w);
            op.first->push_back(w.elements[0]);
            return x;
        }
    };
    using handlers = fl::handlers<apply>;
};

/// Built from the context, so every firing writes to the test's lists.
struct Made : Probe {
    static branes::anatomy::result<Made, std::string> from_context(const ProbeContext& c) {
        Made m;
        m.seen = &c.seen;
        m.first = &c.first;
        return m;
    }
};
using ProbeOp = fl::registered<"Probe", Made, ProbeBinding>;

}  // namespace

TEST_CASE("mlp: W5 a constant reaches every firing by reference, unchanged", "[workbench][mlp]") {
    // Two regions read the MLP's w1: each firing sees the stack's one copy of it.
    using P = ProbeOp;
    const ProbeContext ctx;
    const mlp::weights w = mlp::weights_of(mlp::config{});
    fl::stack_builder<ProbeContext> g("w5", fl::baseline_class::research, ctx);
    const auto a = fl::input<mlp::X>(g, "a");
    const auto b = fl::input<mlp::X>(g, "b");
    const auto w1 = fl::constant<mlp::W1>(g, "w1", w.w1);
    const auto oa = fl::output<mlp::X>(g, "oa", a);
    const auto ob = fl::output<mlp::X>(g, "ob", b);
    const auto pa = fl::instance<P>(g, "pa");
    const auto pb = fl::instance<P>(g, "pb");
    fl::scope ra = g.region(a);
    auto [ya] = fl::fire<"apply">(ra, pa, w1, a);
    g.produce(oa, ya);
    fl::scope rb = g.region(b);
    auto [yb] = fl::fire<"apply">(rb, pb, w1, b);
    g.produce(ob, yb);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    fl::stack s = std::move(built).value();

    for (std::uint64_t k = 0; k < 20; ++k) {
        s.input(k % 2 == 0 ? a : b, mlp::input_of(mlp::config{}, k));
    }
    REQUIRE(ctx.seen.size() == 20);
    for (std::size_t i = 0; i < ctx.seen.size(); ++i) {
        CHECK(ctx.seen[i] == ctx.seen[0]);
        CHECK(ctx.first[i] == w.w1.elements[0]);
    }
    // Not the builder's value, which the stack copied once, when it was built.
    CHECK(ctx.seen[0] != static_cast<const void*>(&w.w1));
}
