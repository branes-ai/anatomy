// The carried rate rule (flow/builder.hpp; docs/plan/computational-domain,
// C4): a reader of a carried edge sees the value its producer wrote in the
// region's previous run, or the message value-initialized before the first.
// The producer may be a later firing or the reader's own, which closes a
// loop. Also the edge's canonical line and flowchart, and G7's refusals. The
// stack and the reference interpreter agree on random graphs with carried
// edges in flow_differential.cpp (W14); flow_mutations.cpp refuses carried
// edges from an input and from another region.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>

#include "../fixtures/flow/toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <string>

namespace {

namespace fl = branes::anatomy::flow;
using branes::anatomy::Timestamp;
using toys::mix;
using toys::Num;
using toys::Two;

Num num(std::int64_t t, std::uint64_t v) {
    return Num{Timestamp{t}, v};
}

/// One region of p: u = Binary1(p, carried(y)), y = Unary2(u), and outputs u and y.
struct Loop {
    fl::stack stack;
    fl::input_port<Num> p;
    fl::output_port<Num> u;
    fl::output_port<Num> y;
};

Loop loop() {
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("loop", fl::baseline_class::research, ctx);
    const auto p = fl::input<Num>(g, "p");
    const auto ou = fl::output<Num>(g, "u", p);
    const auto oy = fl::output<Num>(g, "y", p);
    const auto binary = fl::instance<toys::Binary1>(g, "binary");
    const auto unary = fl::instance<toys::Unary2>(g, "unary");
    fl::scope r = g.region(p);
    const auto y_last = fl::carried<Num>(r);
    auto [u] = fl::fire<"apply">(r, binary, p, y_last);
    auto [y] = fl::fire<"apply">(r, unary, u);
    fl::close(r, y_last, y);
    g.produce(ou, u);
    g.produce(oy, y);
    auto st = std::move(g).build_stack();
    if (!st) {
        FAIL(st.error().to_string());
    }
    return Loop{std::move(st).value(), p, ou, oy};
}

/// Binary's toy: z = (max of the times, mix(3x + y)).
Num binary(const Num& x, const Num& y, std::uint64_t salt) {
    return Num{x.t.ns > y.t.ns ? x.t : y.t, mix(x.v * 3 + y.v, salt)};
}

/// The error a builder's description stops at.
template <class F>
fl::Graph built(F describe) {
    fl::builder g("carried", fl::baseline_class::research);
    describe(g);
    auto b = g.build();
    if (!b) {
        FAIL(b.error().to_string());
    }
    return std::move(b).value();
}

template <class F>
branes::anatomy::config::ConfigError refused(F describe) {
    fl::builder g("carried", fl::baseline_class::research);
    describe(g);
    auto b = g.build();
    REQUIRE_FALSE(b);
    return b.error();
}

}  // namespace

TEST_CASE("carried: a loop reads the last run's value, and zero before the first", "[flow][carried]") {
    Loop l = loop();
    Num y{};  // the carried edge, value-initialized
    for (const std::uint64_t v : {5ULL, 7ULL, 11ULL}) {
        const auto t = static_cast<std::int64_t>(v * 10);
        l.stack.input(l.p, num(t, v));
        const Num u = binary(num(t, v), y, 1);
        CHECK(l.stack.output(l.u).v == u.v);
        CHECK(l.stack.output(l.u).t.ns == t);
        y = Num{u.t, mix(u.v, 2)};
        CHECK(l.stack.output(l.y).v == y.v);
    }
}

TEST_CASE("carried: a firing's own out port, and an earlier firing's, are the last run's", "[flow][carried]") {
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("loop", fl::baseline_class::research, ctx);
    const auto p = fl::input<Num>(g, "p");
    const auto oself = fl::output<Num>(g, "self", p);
    const auto odelay = fl::output<Num>(g, "delay", p);
    const auto self = fl::instance<toys::Unary1>(g, "self_loop");
    const auto first = fl::instance<toys::Unary2>(g, "first");
    const auto delay = fl::instance<toys::Unary3>(g, "delay_line");
    fl::scope r = g.region(p);
    // self_loop reads its own y; delay_line reads first's y, one run late.
    const auto own = fl::carried<Num>(r);
    auto [ys] = fl::fire<"apply">(r, self, own);
    fl::close(r, own, ys);
    auto [yf] = fl::fire<"apply">(r, first, p);
    const auto late = fl::carried<Num>(r);
    fl::close(r, late, yf);  // closed before it is read: its reader names the producer at once
    auto [yd] = fl::fire<"apply">(r, delay, late);
    g.produce(oself, ys);
    g.produce(odelay, yd);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    fl::stack s = std::move(built).value();

    std::uint64_t own_v = 0;
    std::uint64_t first_v = 0;  // first's y, as the last run wrote it
    for (const std::uint64_t v : {2ULL, 3ULL, 4ULL}) {
        s.input(p, num(static_cast<std::int64_t>(v), v));
        own_v = mix(own_v, 1);
        CHECK(s.output(oself).v == own_v);
        CHECK(s.output(oself).t.ns == 0);
        CHECK(s.output(odelay).v == mix(first_v, 3));
        first_v = mix(v, 2);
    }
    const fl::Region& region = s.graph().regions[0];
    CHECK(region.firings[0].in[0] == fl::Ref{fl::Ref::kind::firing, "p", 0, "y", {}, fl::rate_rule::carried, 0});
    CHECK(region.firings[2].in[0] == fl::Ref{fl::Ref::kind::firing, "p", 1, "y", {}, fl::rate_rule::carried, 0});
}

TEST_CASE("carried: of a field of a record", "[flow][carried]") {
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("loop", fl::baseline_class::research, ctx);
    const auto p = fl::input<Num>(g, "p");
    const auto out = fl::output<Num>(g, "y", p);
    const auto unary = fl::instance<toys::Unary1>(g, "unary");
    const auto rec = fl::instance<toys::Rec1>(g, "rec");
    fl::scope r = g.region(p);
    const auto b_last = fl::carried<Num>(r);
    auto [y] = fl::fire<"apply">(r, unary, b_last);
    auto [two] = fl::fire<"apply">(r, rec, p);
    fl::close(r, b_last, fl::project<"b", &Two::b>(two));
    g.produce(out, y);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    fl::stack s = std::move(built).value();
    s.input(p, num(1, 7));
    CHECK(s.output(out).v == mix(0, 1));
    s.input(p, num(2, 8));
    CHECK(s.output(out).v == mix(mix(7, 301), 1));
    CHECK(s.output(out).t.ns == 1);
    CHECK(s.graph().regions[0].firings[0].in[0].path == std::vector<std::string>{"b"});
}

TEST_CASE("carried: the canonical line and the flowchart name the rule", "[flow][carried]") {
    const Loop l = loop();
    const fl::Graph& g = l.stack.graph();
    const std::string text = fl::canonical(g);
    CHECK(text.find("\nin firing p#1 y carried\n") != std::string::npos);
    const std::string chart = fl::mermaid(g);
    CHECK(chart.find(R"x(r0f1 -.->|"y (carried)"| r0f0)x") != std::string::npos);

    // Without the rule, the line is an edge within the region, and the hash differs.
    fl::Graph plain = g;
    plain.regions[0].firings[0].in[1].rate = fl::rate_rule::none;
    CHECK(fl::canonical(plain).find("\nin firing p#1 y\n") != std::string::npos);
    CHECK(fl::graph_hash(plain) != fl::graph_hash(g));
}

TEST_CASE("carried: G7 refuses an edge that is never closed, or closed twice", "[flow][carried]") {
    const auto unclosed = refused([](fl::builder& g) {
        const auto p = fl::input<Num>(g, "p");
        const auto unary = fl::instance<toys::Unary1>(g, "unary");
        fl::scope r = g.region(p);
        const auto c = fl::carried<Num>(r);
        (void)fl::fire<"apply">(r, unary, c);
    });
    CHECK(unclosed.message.rfind("G7: every carried edge is closed", 0) == 0);
    CHECK(unclosed.key == "p[0][0]");

    const auto twice = refused([](fl::builder& g) {
        const auto p = fl::input<Num>(g, "p");
        const auto unary = fl::instance<toys::Unary1>(g, "unary");
        fl::scope r = g.region(p);
        const auto c = fl::carried<Num>(r);
        auto [y] = fl::fire<"apply">(r, unary, c);
        fl::close(r, c, y);
        fl::close(r, c, y);
    });
    CHECK(twice.message.rfind("G7: a carried edge has one producer", 0) == 0);
    CHECK(twice.key == "p[0][0]");
}

TEST_CASE("carried: G7 refuses an edge between regions, either way", "[flow][carried]") {
    // Read in another region than its own.
    const auto read = refused([](fl::builder& g) {
        const auto a = fl::input<Num>(g, "a");
        const auto b = fl::input<Num>(g, "b");
        const auto ua = fl::instance<toys::Unary1>(g, "ua");
        const auto ub = fl::instance<toys::Unary2>(g, "ub");
        fl::scope ra = g.region(a);
        fl::scope rb = g.region(b);
        const auto c = fl::carried<Num>(ra);
        auto [y] = fl::fire<"apply">(ra, ua, a);
        fl::close(ra, c, y);
        (void)fl::fire<"apply">(rb, ub, c);
    });
    CHECK(read.message.rfind("G7: a carried edge is read in its own region", 0) == 0);
    CHECK(read.key == "b[0][0]");

    // Closed with another region's out port.
    const auto closed = refused([](fl::builder& g) {
        const auto a = fl::input<Num>(g, "a");
        const auto b = fl::input<Num>(g, "b");
        const auto ua = fl::instance<toys::Unary1>(g, "ua");
        const auto ub = fl::instance<toys::Unary2>(g, "ub");
        fl::scope ra = g.region(a);
        fl::scope rb = g.region(b);
        auto [y] = fl::fire<"apply">(ra, ua, a);
        const auto c = fl::carried<Num>(rb);
        (void)fl::fire<"apply">(rb, ub, c);
        fl::close(rb, c, y);
    });
    CHECK(closed.message.rfind("G7: a carried edge carries an out port of a firing of its own region", 0) == 0);
    CHECK(closed.key == "b[0][0]");
}

TEST_CASE("carried: a graph that only describes carries a domain", "[flow][carried]") {
    using D = fl::scalar<double>;
    struct Op {};
    struct Binding {
        using signature =
            fl::signature<fl::ports<fl::port<"x", D, fl::direction::in>, fl::port<"y", D, fl::direction::out>>>;
        struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
            static double call(const Op&, double x) {
                return x;
            }
        };
        using handlers = fl::handlers<apply>;
    };
    using Reg = fl::registered<"Integrator", Op, Binding>;
    const fl::Graph g = built([](fl::builder& b) {
        const auto t = b.tick("tick");
        const auto op = fl::instance<Reg>(b, "integrator");
        fl::scope r = b.region(t);
        const auto c = fl::carried<D>(r);
        auto [y] = fl::fire<"apply">(r, op, c);
        fl::close(r, c, y);
    });
    CHECK(g.regions[0].firings[0].in[0] ==
          fl::Ref{fl::Ref::kind::firing, "tick", 0, "y", {}, fl::rate_rule::carried, 0});
}
