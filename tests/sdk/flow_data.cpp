// Unit tests for flow::data<D> (branes/anatomy/flow/data.hpp) and complex
// element types (docs/plan/workbench, B1, property W4): each structure's
// storage size and index mapping, a symmetric domain's one element for (i, j)
// and (j, i), a bounded extent's count, and data<D> run as messages through
// the CPU stack. Edges between data of different domains are the compile-fail
// tests (tests/compile_fail/flow_w4_*).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

#include <catch2/catch_test_macros.hpp>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <set>
#include <type_traits>

using namespace branes::anatomy::flow;

namespace {

using X4 = dim<4>;
using Dense = matrix<double, dim<3>, dim<5>>;
using Sym = matrix<double, X4, X4, structure::symmetric>;
using Upper = matrix<double, X4, X4, structure::upper_triangular>;
using Lower = matrix<double, X4, X4, structure::lower_triangular>;
using Diag = matrix<double, X4, X4, structure::diagonal>;
using Fixes = matrix<double, bounded<4, "m">, dim<2>>;
using Samples = vector<float, bounded<8>>;

/// Every (i, j) inside D's structure maps to a distinct position, and together
/// they cover [0, stored): the layout is a bijection onto the storage.
template <class D>
bool layout_is_a_bijection() {
    using V = data<D>;
    std::set<std::size_t> seen;
    std::size_t inside = 0;
    for (std::size_t i = 0; i < V::max_rows; ++i) {
        for (std::size_t j = 0; j < V::cols; ++j) {
            if (!V::inside(i, j) || (D::kind == structure::symmetric && i > j)) {
                continue;
            }
            ++inside;
            const std::size_t k = V::index(i, j);
            if (k >= V::stored || !seen.insert(k).second) {
                return false;
            }
        }
    }
    return inside == V::stored && seen.size() == V::stored;
}

}  // namespace

TEST_CASE("data: the storage holds the elements the structure needs", "[flow][data]") {
    STATIC_REQUIRE(data<scalar<double>>::stored == 1);
    STATIC_REQUIRE(data<vector<double, dim<16>>>::stored == 16);
    STATIC_REQUIRE(data<Dense>::stored == 15);
    STATIC_REQUIRE(data<Sym>::stored == 10);
    STATIC_REQUIRE(data<Upper>::stored == 10);
    STATIC_REQUIRE(data<Lower>::stored == 10);
    STATIC_REQUIRE(data<Diag>::stored == 4);
    STATIC_REQUIRE(data<Fixes>::stored == 8);
    STATIC_REQUIRE(sizeof(data<Sym>::elements) == 10 * sizeof(double));
    STATIC_REQUIRE(std::is_same_v<domain_of_t<data<Sym>>, Sym>);
}

TEST_CASE("data: each structure's layout is a bijection onto its storage", "[flow][data]") {
    CHECK(layout_is_a_bijection<Dense>());
    CHECK(layout_is_a_bijection<Sym>());
    CHECK(layout_is_a_bijection<Upper>());
    CHECK(layout_is_a_bijection<Lower>());
    CHECK(layout_is_a_bijection<Diag>());
    CHECK(layout_is_a_bijection<Fixes>());
}

TEST_CASE("data: dense is row-major, and the triangles are packed row by row", "[flow][data]") {
    CHECK(data<Dense>::index(0, 0) == 0);
    CHECK(data<Dense>::index(0, 4) == 4);
    CHECK(data<Dense>::index(1, 0) == 5);
    CHECK(data<Dense>::index(2, 4) == 14);

    // The upper triangle of 4 x 4, row by row: 4, 3, 2 and 1 elements.
    CHECK(data<Upper>::index(0, 0) == 0);
    CHECK(data<Upper>::index(0, 3) == 3);
    CHECK(data<Upper>::index(1, 1) == 4);
    CHECK(data<Upper>::index(2, 2) == 7);
    CHECK(data<Upper>::index(3, 3) == 9);

    // The lower triangle, row by row: 1, 2, 3 and 4 elements.
    CHECK(data<Lower>::index(0, 0) == 0);
    CHECK(data<Lower>::index(1, 0) == 1);
    CHECK(data<Lower>::index(2, 0) == 3);
    CHECK(data<Lower>::index(3, 3) == 9);

    CHECK(data<Diag>::index(2, 2) == 2);
}

TEST_CASE("data: a symmetric matrix's (i, j) and (j, i) are one element (A3)", "[flow][data]") {
    data<Sym> p;
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = i; j < 4; ++j) {
            p(i, j) = static_cast<double>(10 * i + j);
        }
    }
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            CHECK(&p(i, j) == &p(j, i));
            CHECK(p.at(i, j) == p.at(j, i));
        }
    }
    p(3, 1) = -1.0;
    CHECK(p(1, 3) == -1.0);
}

TEST_CASE("data: a triangle or a diagonal reads zero outside its structure", "[flow][data]") {
    data<Upper> u;
    data<Lower> l;
    data<Diag> d;
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            if (data<Upper>::inside(i, j)) {
                u(i, j) = 1.0;
            }
            if (data<Lower>::inside(i, j)) {
                l(i, j) = 1.0;
            }
            if (data<Diag>::inside(i, j)) {
                d(i, j) = 1.0;
            }
        }
    }
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            CHECK(u.at(i, j) == (i <= j ? 1.0 : 0.0));
            CHECK(l.at(i, j) == (j <= i ? 1.0 : 0.0));
            CHECK(d.at(i, j) == (i == j ? 1.0 : 0.0));
        }
    }
}

TEST_CASE("data: a bounded extent's count stays within its bound (A1, A2)", "[flow][data]") {
    data<Fixes> z;
    CHECK(z.count() == 0);
    REQUIRE(z.resize(3));
    CHECK(z.count() == 3);
    z(2, 1) = 7.0;

    // Above the bound: refused, and nothing changes.
    CHECK_FALSE(z.resize(5));
    CHECK(z.count() == 3);
    CHECK(z(2, 1) == 7.0);

    // Shrinking resets the rows it drops, so equal rows in use compare equal.
    REQUIRE(z.resize(2));
    data<Fixes> fresh;
    REQUIRE(fresh.resize(2));
    CHECK(z == fresh);
    REQUIRE(z.resize(3));
    CHECK(z(2, 1) == 0.0);

    REQUIRE(z.resize(4));
    CHECK(z.count() == 4);
    REQUIRE(z.resize(0));
    CHECK(z == data<Fixes>{});

    data<Samples> s;
    CHECK(s.count() == 0);
    CHECK(s.resize(8));
    CHECK_FALSE(s.resize(9));
}

TEST_CASE("data: a static extent's count is its size, and only that", "[flow][data]") {
    data<Dense> m;
    CHECK(m.count() == 3);
    CHECK(m.resize(3));
    CHECK_FALSE(m.resize(2));
    CHECK(m.count() == 3);

    data<scalar<int>> s;
    CHECK(s.count() == 1);
    s() = 4;
    CHECK(s.elements[0] == 4);
}

TEST_CASE("data: complex elements", "[flow][data]") {
    STATIC_REQUIRE(is_element_v<std::complex<float>>);
    STATIC_REQUIRE(is_element_v<std::complex<double>>);
    STATIC_REQUIRE(is_element_v<std::complex<long double>>);
    STATIC_REQUIRE_FALSE(is_element_v<std::complex<int>>);

    using Spectrum = vector<std::complex<double>, dim<4>>;
    data<Spectrum> x;
    x(1) = {1.0, -2.0};
    CHECK(x(1).imag() == -2.0);
    CHECK(x(0) == std::complex<double>{});
}

// ── data<D> as messages through the CPU stack ───────────────────────

namespace {

using Vec3 = data<vector<double, dim<3>>>;
using Cov3 = data<matrix<double, dim<3>, dim<3>, structure::symmetric>>;

/// y = a·x, and P' = a²·P: a vector and a symmetric matrix, through one handler.
struct Scale {
    double a = 2.0;
};

struct ScaleBinding {
    using signature = branes::anatomy::flow::signature<ports<port<"x", Vec3, direction::in>,
                                                             port<"p", Cov3, direction::in>,
                                                             port<"y", Vec3, direction::out>,
                                                             port<"q", Cov3, direction::out>>>;
    struct apply : handler<"apply", ins<"x", "p">, outs<"y", "q">> {
        static std::tuple<Vec3, Cov3> call(const Scale& op, const Vec3& x, const Cov3& p) {
            Vec3 y;
            for (std::size_t i = 0; i < 3; ++i) {
                y(i) = op.a * x(i);
            }
            Cov3 q;
            for (std::size_t k = 0; k < Cov3::stored; ++k) {
                q.elements[k] = op.a * op.a * p.elements[k];
            }
            return {y, q};
        }
    };
    using handlers = branes::anatomy::flow::handlers<apply>;
};

using ScaleOp = registered<"Scale", Scale, ScaleBinding>;

struct Context {};

}  // namespace

TEST_CASE("data: vectors and symmetric matrices run through the CPU stack", "[flow][data]") {
    Context ctx;
    stack_builder<Context> g("data", baseline_class::research, ctx);
    const auto x = input<Vec3>(g, "x");
    Cov3 p0;
    p0(0, 2) = 3.0;
    p0(1, 1) = 5.0;
    const auto p = constant<Cov3>(g, "p", p0);
    const auto y = output<Vec3>(g, "y", x);
    const auto q = output<Cov3>(g, "q", x);
    const auto scale = instance<ScaleOp>(g, "scale");
    scope r = g.region(x);
    auto [ys, qs] = fire<"apply">(r, scale, x, p);
    g.produce(y, ys);
    g.produce(q, qs);

    auto built = std::move(g).build_stack();
    REQUIRE(built);
    stack s = std::move(built).value();

    Vec3 in;
    in(0) = 1.0;
    in(1) = -2.0;
    in(2) = 0.5;
    s.input(x, in);
    CHECK(s.output(y)(0) == 2.0);
    CHECK(s.output(y)(1) == -4.0);
    CHECK(s.output(y)(2) == 1.0);
    CHECK(s.output(q)(2, 0) == 12.0);
    CHECK(s.output(q)(1, 1) == 20.0);
    CHECK(s.output(q)(0, 1) == 0.0);
}
