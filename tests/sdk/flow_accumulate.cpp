// The accumulate rate rule (flow/window.hpp, flow/builder.hpp;
// docs/plan/workbench, B2, property W6): when its region begins, a reader
// sees, oldest first, the values its producer wrote since that region last
// began, the latest N of them, and how many were dropped; before the
// producer first writes, an empty window. Also the window's domain, and the
// edge's canonical line and flowchart. The stack and the reference
// interpreter agree on random graphs with accumulate edges in
// flow_differential.cpp; G7's refusals are in flow_mutations.cpp.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/window.hpp>

#include "../fixtures/flow/toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace {

namespace fl = branes::anatomy::flow;
using branes::anatomy::Timestamp;
using toys::Num;
using toys::Two;

using W3 = fl::window<Num, 3>;

/// Writes what it reads: the window, for a test to look at.
struct Seen {};
struct SeenBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"x", W3, fl::direction::in>, fl::port<"w", W3, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"w">> {
        static W3 call(const Seen&, const W3& x) {
            return x;
        }
    };
    using handlers = fl::handlers<apply>;
};
using SeenOp = fl::registered<"Seen", Seen, SeenBinding>;

Num num(std::int64_t t, std::uint64_t v) {
    return Num{Timestamp{t}, v};
}

/// Two windows of the same values, in order, with the same count and number dropped (Num has no ==).
bool same(const W3& x, const W3& y) {
    for (std::size_t k = 0; k < 3; ++k) {
        if (x.items[k].t != y.items[k].t || x.items[k].v != y.items[k].v) {
            return false;
        }
    }
    return x.count == y.count && x.dropped == y.dropped;
}

/// `a` writes values; `b`'s region reads accumulate<3>(a) into output `w`.
struct OneEdge {
    fl::stack stack;
    fl::input_port<Num> a;
    fl::input_port<Num> b;
    fl::output_port<W3> w;
};

OneEdge one_edge() {
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("acc", fl::baseline_class::research, ctx);
    const auto a = fl::input<Num>(g, "a");
    const auto b = fl::input<Num>(g, "b");
    const auto w = fl::output<W3>(g, "w", b);
    const auto seen = fl::instance<SeenOp>(g, "seen");
    fl::scope r = g.region(b);
    auto [x] = fl::fire<"apply">(r, seen, fl::accumulate<3>(a));
    g.produce(w, x);
    auto st = std::move(g).build_stack();
    if (!st) {
        FAIL(st.error().to_string());
    }
    return OneEdge{std::move(st).value(), a, b, w};
}

}  // namespace

TEST_CASE("accumulate: a window's domain is its items and the number dropped", "[flow][accumulate]") {
    using D = fl::domain_of_t<W3>;
    STATIC_REQUIRE(std::is_same_v<D,
                                  fl::record<fl::field<"items", fl::vector<std::uint64_t, fl::bounded<3>>>,
                                             fl::field<"dropped", fl::scalar<std::uint64_t>>>>);
    // A record's window holds records; a graph that only describes reads the domain.
    STATIC_REQUIRE(std::is_same_v<fl::field_t<fl::domain_of_t<fl::window<Two, 2>>, "items">,
                                  fl::vector<fl::domain_of_t<Two>, fl::bounded<2>>>);
    STATIC_REQUIRE(std::is_same_v<fl::window_t<fl::scalar<double>, 4>, fl::window_domain_t<fl::scalar<double>, 4>>);
    STATIC_REQUIRE(std::is_same_v<fl::window_t<Num, 4>, fl::window<Num, 4>>);
    // Two capacities are two domains, so G4 refuses an edge between them.
    STATIC_REQUIRE_FALSE(std::is_same_v<fl::domain_of_t<W3>, fl::domain_of_t<fl::window<Num, 4>>>);
    // A window compares equal when its message type does.
    STATIC_REQUIRE(std::equality_comparable<fl::window<int, 2>>);
    STATIC_REQUIRE_FALSE(std::equality_comparable<W3>);
}

TEST_CASE("accumulate: before the producer first writes, the window is empty (W6)", "[flow][accumulate]") {
    OneEdge e = one_edge();
    e.stack.input(e.b, num(1, 0));
    const W3& w = e.stack.output(e.w);
    CHECK(w.count == 0);
    CHECK(w.dropped == 0);
    CHECK(same(w, W3{}));
}

TEST_CASE("accumulate: a short window, oldest first, and then an empty one (W6)", "[flow][accumulate]") {
    OneEdge e = one_edge();
    e.stack.input(e.a, num(1, 10));
    e.stack.input(e.a, num(2, 20));
    e.stack.input(e.b, num(3, 0));
    {
        const W3& w = e.stack.output(e.w);
        CHECK(w.count == 2);
        CHECK(w.dropped == 0);
        CHECK(w.items[0].v == 10);
        CHECK(w.items[1].v == 20);
        CHECK(w.items[1].t.ns == 2);
        CHECK(w.items[2].v == 0);  // W2: past the count, value-initialized
        CHECK(w.items[2].t.ns == 0);
    }
    // Nothing written since the region last began: an empty window.
    e.stack.input(e.b, num(4, 0));
    CHECK(same(e.stack.output(e.w), W3{}));
}

TEST_CASE("accumulate: an overfull window keeps the latest N, and counts the rest (W6)", "[flow][accumulate]") {
    OneEdge e = one_edge();
    for (std::uint64_t v = 1; v <= 5; ++v) {
        e.stack.input(e.a, num(static_cast<std::int64_t>(v), v));
    }
    e.stack.input(e.b, num(6, 0));
    const W3& w = e.stack.output(e.w);
    CHECK(w.count == 3);
    CHECK(w.dropped == 2);
    CHECK(w.items[0].v == 3);
    CHECK(w.items[1].v == 4);
    CHECK(w.items[2].v == 5);

    // The next window counts only what it dropped itself.
    for (std::uint64_t v = 6; v <= 10; ++v) {
        e.stack.input(e.a, num(static_cast<std::int64_t>(v), v));
    }
    e.stack.input(e.b, num(11, 0));
    CHECK(e.stack.output(e.w).dropped == 2);
    CHECK(e.stack.output(e.w).items[0].v == 8);
}

TEST_CASE("accumulate: two readers of one out port fill two windows (W6)", "[flow][accumulate]") {
    // p's region writes y = Unary1(p); b's and c's regions each accumulate
    // y, and b's region reads it twice, in two firings, which see one window.
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("acc", fl::baseline_class::research, ctx);
    const auto p = fl::input<Num>(g, "p");
    const auto b = fl::input<Num>(g, "b");
    const auto c = fl::input<Num>(g, "c");
    const auto wb = fl::output<W3>(g, "wb", b);
    const auto wb2 = fl::output<W3>(g, "wb2", b);
    const auto wc = fl::output<W3>(g, "wc", c);
    const auto unary = fl::instance<toys::Unary1>(g, "unary");
    const auto seen_b = fl::instance<SeenOp>(g, "seen_b");
    const auto seen_b2 = fl::instance<SeenOp>(g, "seen_b2");
    const auto seen_c = fl::instance<SeenOp>(g, "seen_c");
    fl::scope rp = g.region(p);
    auto [y] = fl::fire<"apply">(rp, unary, p);
    fl::scope rb = g.region(b);
    auto [xb] = fl::fire<"apply">(rb, seen_b, fl::accumulate<3>(y));
    auto [xb2] = fl::fire<"apply">(rb, seen_b2, fl::accumulate<3>(y));
    g.produce(wb, xb);
    g.produce(wb2, xb2);
    fl::scope rc = g.region(c);
    auto [xc] = fl::fire<"apply">(rc, seen_c, fl::accumulate<3>(y));
    g.produce(wc, xc);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    fl::stack s = std::move(built).value();

    s.input(p, num(1, 1));
    s.input(p, num(2, 2));
    s.input(b, num(3, 0));
    s.input(p, num(4, 4));
    s.input(c, num(5, 0));
    CHECK(s.output(wb).count == 2);
    CHECK(same(s.output(wb), s.output(wb2)));
    CHECK(s.output(wb).items[0].v == toys::mix(1, 1));
    // c's region hadn't begun: it reads all three.
    CHECK(s.output(wc).count == 3);
    CHECK(s.output(wc).items[2].v == toys::mix(4, 1));
    CHECK(s.output(wc).items[2].t.ns == 4);
    s.input(b, num(6, 0));
    CHECK(s.output(wb).count == 1);
    CHECK(s.output(wb).items[0].v == toys::mix(4, 1));
}

TEST_CASE("accumulate: of a field of a record", "[flow][accumulate]") {
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("acc", fl::baseline_class::research, ctx);
    const auto p = fl::input<Num>(g, "p");
    const auto b = fl::input<Num>(g, "b");
    const auto w = fl::output<W3>(g, "w", b);
    const auto rec = fl::instance<toys::Rec1>(g, "rec");
    const auto seen = fl::instance<SeenOp>(g, "seen");
    fl::scope rp = g.region(p);
    auto [r] = fl::fire<"apply">(rp, rec, p);
    fl::scope rb = g.region(b);
    auto [x] = fl::fire<"apply">(rb, seen, fl::accumulate<3>(fl::project<"b", &Two::b>(r)));
    g.produce(w, x);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    fl::stack s = std::move(built).value();
    s.input(p, num(1, 7));
    s.input(b, num(2, 0));
    CHECK(s.output(w).count == 1);
    CHECK(s.output(w).items[0].v == toys::mix(7, 301));
}

TEST_CASE("accumulate: the canonical line and the flowchart name the capacity", "[flow][accumulate]") {
    const OneEdge e = one_edge();
    const fl::Graph& g = e.stack.graph();
    REQUIRE(g.regions.size() == 1);
    const fl::Ref& ref = g.regions[0].firings[0].in[0];
    CHECK(ref.rate == fl::rate_rule::accumulate);
    CHECK(ref.capacity == 3);

    const std::string text = fl::canonical(g);
    CHECK(text.find("\nin input a accumulate 3\n") != std::string::npos);
    const std::string chart = fl::mermaid(g);
    CHECK(chart.find(R"(in0 -.->|"accumulate 3"| r0f0)") != std::string::npos);

    // Of a firing's out port, the label names the port too.
    fl::Graph other = g;
    other.regions[0].firings[0].in[0].from = fl::Ref::kind::firing;
    other.regions[0].firings[0].in[0].region = "p";
    other.regions[0].firings[0].in[0].port = "y";
    CHECK(fl::mermaid(other).find(R"x(-.->|"y (accumulate 3)"| r0f0)x") != std::string::npos);
    CHECK(fl::canonical(other).find("\nin firing p#0 y accumulate 3\n") != std::string::npos);
    CHECK(fl::graph_hash(other) != fl::graph_hash(g));
}
