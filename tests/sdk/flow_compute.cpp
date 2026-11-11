// Unit tests for the computational domain of a handler (flow/compute.hpp;
// docs/plan/computational-domain, C1, property W10 on the toys): each toy's
// declaration passes K1–K7, K8 classifies it as expected, its operation
// counts and depth are their closed forms, and a mutation of a valid
// declaration that breaks one of K3–K7 is refused with that id. K1 and K2 at
// compile time are the compile-fail tests (tests/compile_fail/flow_k*).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>

#include "../fixtures/flow/compute_toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <string>

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

namespace {

std::string problems_of(const cd::report& r) {
    std::string out;
    for (const cd::problem& p : r.problems) {
        out += p.id + ": " + p.what + "\n";
    }
    return out;
}

template <class Op, fl::fixed_string H>
cd::report checked() {
    cd::report r = cd::check<Op, H>();
    INFO(problems_of(r));
    CHECK(r.ok());
    return r;
}

/// A table for analyzing a mutation of handler H of binding B.
template <class B, fl::fixed_string H>
cd::table table_for() {
    return cd::table_of<B, fl::handler_t<B, H>>();
}

}  // namespace

TEST_CASE("compute: a reduction is uniform, with R*C multiply-adds", "[flow][compute][W10][W13]") {
    const cd::report r = checked<toys::Linear34, "apply">();
    REQUIRE(r.stages.size() == 1);
    CHECK(r.stages[0] == cd::shape::uniform);
    CHECK(r.count(cd::op::mul, cd::etype::f64) == 3 * 4);
    CHECK(r.count(cd::op::add, cd::etype::f64) == 3 * 4);
    // The init, the C points of each row, and the write: C + 2 definitions deep.
    CHECK(r.depth == 4 + 2);
    CHECK(r.definitions == 3 * (1 + 4 + 1));
    REQUIRE(r.width.size() == 6);
    CHECK(r.width[0] == 3);
}

TEST_CASE("compute: an elementwise map is one definition deep", "[flow][compute][W10][W13]") {
    const cd::report r = checked<toys::Scale5, "apply">();
    CHECK(r.stages[0] == cd::shape::uniform);
    CHECK(r.count(cd::op::mul) == 5);
    CHECK(r.depth == 1);
    CHECK(r.width == std::vector<std::uint64_t>{5});
}

TEST_CASE("compute: a triangular space has n(n+1)/2 points", "[flow][compute][W10][W13]") {
    const cd::report r = checked<toys::TriMul4, "apply">();
    CHECK(r.stages[0] == cd::shape::uniform);
    CHECK(r.count(cd::op::mul) == 4 * 5 / 2);
    CHECK(r.count(cd::op::add) == 4 * 5 / 2);
}

TEST_CASE("compute: a program's stages are joined by a local", "[flow][compute][W10][W13]") {
    const cd::report r = checked<toys::Affine23, "apply">();
    REQUIRE(r.stages.size() == 2);
    CHECK(r.stages[0] == cd::shape::uniform);
    CHECK(r.stages[1] == cd::shape::uniform);
    CHECK(r.count(cd::op::mul) == 2 * 3);
    CHECK(r.count(cd::op::add) == 2 * 3 + 2);
    // Stage 1 reads the local stage 0 writes: init, 3 points, the write, then z.
    CHECK(r.depth == 1 + 3 + 1 + 1);
}

TEST_CASE("compute: a state is read before the firing and written after it", "[flow][compute][W10][W13]") {
    const cd::report r = checked<toys::Delay4, "filter">();
    CHECK(r.count(cd::op::mul) == 4);
    CHECK(r.count(cd::op::add) == 4);
}

TEST_CASE("compute: a butterfly's partner read is non-affine", "[flow][compute][W10]") {
    const cd::report r = checked<toys::Butterfly8, "apply">();
    CHECK(r.stages[0] == cd::shape::non_affine);
    CHECK(r.count(cd::op::sel_lt) == 8);
}

TEST_CASE("compute: a bounded extent is checked at every count", "[flow][compute][W10]") {
    const cd::report r = checked<toys::CountScale4, "apply">();
    CHECK(r.stages[0] == cd::shape::uniform);
    CHECK(r.count(cd::op::mul) == 4);  // at the bound
}

TEST_CASE("compute: the text form names the space, the definitions and the counts", "[flow][compute]") {
    const std::string linear = cd::text_of<toys::Linear34, "apply">();
    INFO(linear);
    CHECK(linear.find("param w[0] = 3\nparam w[1] = 4\n") != std::string::npos);
    CHECK(linear.find("stage 0: (i, j) | 0 <= i < w[0], 0 <= j < w[1]") != std::string::npos);
    CHECK(linear.find("acc(i, j) = acc(i, j - 1) + (w(i, j) * x(j))") != std::string::npos);
    CHECK(linear.find("init acc = 0  | j == -1") != std::string::npos);
    CHECK(linear.find("y(i) <- acc(i, j)  | j == w[1] - 1") != std::string::npos);
    const std::string bounded = cd::text_of<toys::CountScale4, "apply">();
    INFO(bounded);
    CHECK(bounded.find("param count(x) <= 4") != std::string::npos);
    CHECK(bounded.find("count y = count(x)") != std::string::npos);
}

// ── Mutations: each breaks one invariant, which is reported ─────────

namespace {
using LB = toys::LinearBinding<3, 4>;
constexpr auto f64 = cd::etype::f64;
}  // namespace

TEST_CASE("compute: K3 refuses a read of a variable nothing defines", "[flow][compute][W10]") {
    static constexpr cd::system no_init = [] {
        cd::system s;
        const auto cols = s.extent("w", 1);
        const auto i = s.index("i", 0, s.extent("w", 0));
        const auto j = s.index("j", 0, cols);
        const auto acc = s.var("acc", f64);
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
        s.write(s.out("y").at(i), acc(i, j), s.where(j == cols - 1));
        return s;
    }();
    const cd::report r = cd::analyze(no_init, table_for<LB, "apply">());
    INFO(problems_of(r));
    CHECK(r.has("K3"));
}

TEST_CASE("compute: K4 refuses a read outside a port, and an unguarded bounded read", "[flow][compute][W10]") {
    static constexpr cd::system past_x = [] {
        cd::system s;
        const auto cols = s.extent("w", 1);
        const auto i = s.index("i", 0, s.extent("w", 0));
        const auto j = s.index("j", 0, cols);
        const auto acc = s.var("acc", f64);
        s.init(acc, s.literal(0, f64), s.face(j == -1));
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j + 1));
        s.write(s.out("y").at(i), acc(i, j), s.where(j == cols - 1));
        return s;
    }();
    const cd::report r = cd::analyze(past_x, table_for<LB, "apply">());
    INFO(problems_of(r));
    CHECK(r.has("K4"));

    // Reads x over its bound rather than its count: refused for counts below the bound.
    using CB = toys::CountScaleBinding<4>;
    static constexpr cd::system over_bound = [] {
        cd::system s;
        const auto n = s.count("x");
        const auto i = s.index("i", 0, s.extent("x", 0));
        s.write(s.out("y").at(i), s.in("x")(i) * 2.0, s.where(i < n));
        s.write(s.out("y").at(i), s.in("x")(i) * 0.0, s.where(i >= n));
        s.set_count(s.out("y"), n);
        return s;
    }();
    const cd::report b = cd::analyze(over_bound, table_for<CB, "apply">());
    INFO(problems_of(b));
    CHECK(b.has("K4"));
}

TEST_CASE("compute: K5 refuses an output written twice, outside it, or not at all", "[flow][compute][W10]") {
    static constexpr cd::system every_point = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.extent("w", 0));
        const auto j = s.index("j", 0, s.extent("w", 1));
        const auto acc = s.var("acc", f64);
        s.init(acc, s.literal(0, f64), s.face(j == -1));
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
        s.write(s.out("y").at(i), acc(i, j));  // from every j, not only the last
        return s;
    }();
    CHECK(cd::analyze(every_point, table_for<LB, "apply">()).has("K5"));

    static constexpr cd::system shifted = [] {
        cd::system s;
        const auto cols = s.extent("w", 1);
        const auto i = s.index("i", 0, s.extent("w", 0));
        const auto j = s.index("j", 0, cols);
        const auto acc = s.var("acc", f64);
        s.init(acc, s.literal(0, f64), s.face(j == -1));
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
        s.write(s.out("y").at(i + 1), acc(i, j), s.where(j == cols - 1));  // y(R) is outside, y(0) unwritten
        return s;
    }();
    const cd::report r = cd::analyze(shifted, table_for<LB, "apply">());
    INFO(problems_of(r));
    CHECK(r.has("K5"));

    // A declared count past the bound, and one below zero.
    using CB = toys::CountScaleBinding<4>;
    static constexpr cd::system count_over = [] {
        cd::system s;
        const auto n = s.count("x");
        const auto i = s.index("i", 0, n);
        s.write(s.out("y").at(i), s.in("x")(i) * 2.0);
        s.set_count(s.out("y"), n + 1);
        return s;
    }();
    CHECK(cd::analyze(count_over, table_for<CB, "apply">()).has("K5"));
    static constexpr cd::system count_under = [] {
        cd::system s;
        const auto n = s.count("x");
        const auto i = s.index("i", 0, n - 1);
        s.write(s.out("y").at(i), s.in("x")(i) * 2.0);
        s.set_count(s.out("y"), n - 1);
        return s;
    }();
    CHECK(cd::analyze(count_under, table_for<CB, "apply">()).has("K5"));
    static constexpr cd::system no_count = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.count("x"));
        s.write(s.out("y").at(i), s.in("x")(i) * 2.0);
        return s;
    }();
    CHECK(cd::analyze(no_count, table_for<CB, "apply">()).has("K5"));
}

TEST_CASE("compute: K6 refuses a cycle of dependences", "[flow][compute][W10]") {
    using SB = toys::ScaleBinding<5>;
    static constexpr cd::system cycle = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.extent("x", 0));
        const auto a = s.var("a", f64);
        const auto b = s.var("b", f64);
        s.eq(a, b(i) + s.in("x")(i));
        s.eq(b, a(i) * 2.0);
        s.write(s.out("y").at(i), a(i));
        return s;
    }();
    const cd::report r = cd::analyze(cycle, table_for<SB, "apply">());
    INFO(problems_of(r));
    CHECK(r.has("K6"));
}

TEST_CASE("compute: K7 refuses state that the handler's role doesn't allow", "[flow][compute][W10]") {
    // The delay line's declaration under a table whose role is none, then read.
    using DB = toys::DelayBinding<4>;
    cd::table t = table_for<DB, "filter">();
    const cd::system& s = fl::handler_t<DB, "filter">::compute;
    t.role = fl::state_role::none;
    CHECK(cd::analyze(s, t).has("K7"));
    t.role = fl::state_role::read;
    CHECK(cd::analyze(s, t).has("K7"));
    // An update that reads the line and never writes it.
    static constexpr cd::system no_write = [] {
        cd::system s2;
        const auto n = s2.extent("line", 0);
        const auto k = s2.index("k", 0, n);
        const auto acc = s2.var("acc", f64);
        s2.init(acc, s2.literal(0, f64), s2.face(k == -1));
        s2.eq(acc, acc(k - 1) + s2.in("taps")(k)*s2.state("line")(k));
        s2.write(s2.out("y").at(), acc(k), s2.where(k == n - 1));
        return s2;
    }();
    const cd::report r = cd::analyze(no_write, table_for<DB, "filter">());
    INFO(problems_of(r));
    CHECK(r.has("K7"));
}

TEST_CASE("compute: K1 and K2 are reported at run time too", "[flow][compute][W10]") {
    // Reads the out port y: K1.
    static constexpr cd::system reads_out = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.extent("w", 0));
        s.write(s.out("y").at(i), s.in("y")(i));
        return s;
    }();
    CHECK(cd::k1(reads_out, table_for<LB, "apply">()).k == 1);
    CHECK(cd::analyze(reads_out, table_for<LB, "apply">()).has("K1"));
    // Binds a parameter to x's second extent, which a vector hasn't: K2.
    static constexpr cd::system no_dim = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.extent("x", 1));
        s.write(s.out("y").at(i), s.in("x")(i));
        return s;
    }();
    CHECK(cd::k1(no_dim, table_for<LB, "apply">()).k == 0);
    CHECK(cd::k2(no_dim, table_for<LB, "apply">()).k == 2);
}

TEST_CASE("compute: sin and cos take a real, and are counted by kind", "[flow][compute][W10][W13]") {
    // y(i) = sin(x(i)) * cos(x(i)), over x's 4 elements: K1 types it.
    static constexpr cd::system trig = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.extent("x", 0));
        s.write(s.out("y").at(i), sin(s.in("x")(i)) * cos(s.in("x")(i)));
        return s;
    }();
    CHECK(cd::k1(trig, table_for<LB, "apply">()).k == 0);
    CHECK(cd::to_string(cd::op::sin) == "sin");
    CHECK(cd::to_string(cd::op::cos) == "cos");
    CHECK(cd::is_counted(cd::op::sin));
    CHECK(cd::is_counted(cd::op::cos));
    // Of a complex: refused by K1, as a mismatched type.
    static constexpr cd::system of_complex = [] {
        cd::system s;
        const auto i = s.index("i", 0, s.extent("x", 0));
        s.write(s.out("y").at(i), s.cast(cos(s.literal(1, cd::etype::c128)), cd::etype::f64));
        return s;
    }();
    const cd::finding f = cd::k1(of_complex, table_for<LB, "apply">());
    CHECK(f.k == 1);
    CHECK(f.what == static_cast<std::uint8_t>(cd::reason::type));
}
