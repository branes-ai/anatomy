// Unit tests for flow::data<D> (branes/anatomy/flow/data.hpp) and complex
// element types (docs/plan/workbench, B1, property W4): each structure's
// storage size and index mapping, a symmetric domain's one element for (i, j)
// and (j, i), a bounded extent's count, rank 3 and up stored row-major
// (docs/plan/computational-domain, W17), and data<D> run as messages through
// the CPU stack. Edges between data of different domains are the compile-fail
// tests (tests/compile_fail/flow_w4_*).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
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

TEST_CASE("data: a unit-norm vector stores what dense does, and starts as e0 (W17, A4)", "[flow][data]") {
    using Quat = vector<double, dim<4>, structure::unit_norm>;
    using Dense4 = vector<double, dim<4>>;
    STATIC_REQUIRE(stored_elements_v<Quat> == 4);
    STATIC_REQUIRE(data<Quat>::stored == data<Dense4>::stored);
    STATIC_REQUIRE(sizeof(data<Quat>) == sizeof(data<Dense4>));
    STATIC_REQUIRE(data<Quat>{}.structure_holds());
    STATIC_REQUIRE(data<Dense4>{}.structure_holds());  // dense has no norm to hold

    data<Quat> q;
    CHECK(q.count() == 4);
    CHECK(q.elements == std::array<double, 4>{1.0, 0.0, 0.0, 0.0});
    // Stored in order, as a dense vector is.
    q(0) = 0.5;
    q(1) = -0.5;
    q(2) = 0.5;
    q(3) = 0.5;
    CHECK(&q(2) == &q.elements[2]);
    CHECK(q.structure_holds());

    // A4's tolerance is 256 ulps of 1: a normalized vector holds it, and a
    // norm off by more does not.
    STATIC_REQUIRE(unit_norm_tolerance<double> == 256 * std::numeric_limits<double>::epsilon());
    q(3) = 0.5 + 1e-12;
    CHECK_FALSE(q.structure_holds());
    q(3) = 0.5 + 1e-15;
    CHECK(q.structure_holds());
    q.elements = {0.0, 0.0, 0.0, 0.0};
    CHECK_FALSE(q.structure_holds());
    q(3) = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(q.structure_holds());

    data<vector<float, dim<3>, structure::unit_norm>> f;
    f(0) = 0.6f;
    f(1) = 0.8f;
    CHECK(f.structure_holds());
    f(1) = 0.8001f;
    CHECK_FALSE(f.structure_holds());
}

TEST_CASE("data: a rank-3 or rank-4 domain is stored row-major, the last index fastest (W17)", "[flow][data]") {
    using Cube = tensor<double, dim<2>, dim<3>, dim<4>>;
    using Kernels = tensor<float, dim<2>, dim<3>, dim<2>, dim<2>>;
    STATIC_REQUIRE(data<Cube>::rank == 3);
    STATIC_REQUIRE(data<Cube>::stored == 24);
    STATIC_REQUIRE(data<Cube>::row_size == 12);
    STATIC_REQUIRE(data<Cube>::sizes == std::array<std::size_t, 3>{2, 3, 4});
    STATIC_REQUIRE(sizeof(data<Cube>::elements) == 24 * sizeof(double));
    STATIC_REQUIRE(data<Kernels>::stored == 24);
    STATIC_REQUIRE(data<Kernels>::row_size == 12);
    STATIC_REQUIRE(std::is_same_v<domain_of_t<data<Cube>>, Cube>);

    // Row-major: (i, j, k) at (i·3 + j)·4 + k, a bijection onto the storage.
    std::set<std::size_t> seen;
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t k = 0; k < 4; ++k) {
                const std::size_t e = data<Cube>::index(i, j, k);
                CHECK(e == (i * 3 + j) * 4 + k);
                seen.insert(e);
            }
        }
    }
    CHECK(seen.size() == 24);
    CHECK(*seen.rbegin() == 23);
    CHECK(data<Kernels>::index(1, 2, 1, 0) == ((1 * 3 + 2) * 2 + 1) * 2);

    data<Cube> c;
    CHECK(c.count() == 2);
    c(1, 2, 3) = 5.0;
    CHECK(c.elements[23] == 5.0);
    CHECK(&c(0, 1, 2) == &c.elements[6]);
    const data<Cube>& cc = c;
    CHECK(cc(1, 2, 3) == 5.0);
    CHECK_FALSE(c.resize(1));  // a static leading extent's count is its size

    data<Kernels> w;
    w(1, 2, 1, 0) = 2.5f;
    CHECK(w.elements[22] == 2.5f);
}

TEST_CASE("data: a bounded leading extent of rank 3 counts rows of every trailing element (A1, A2, W17)",
          "[flow][data]") {
    using Frames = tensor<double, bounded<3>, dim<2>, dim<2>>;
    STATIC_REQUIRE(data<Frames>::stored == 12);
    STATIC_REQUIRE(data<Frames>::row_size == 4);
    data<Frames> f;
    CHECK(f.count() == 0);
    REQUIRE(f.resize(3));
    for (double& e : f.elements) {
        e = 1.0;
    }
    // Shrinking to one row resets the last two, all four elements of each.
    REQUIRE(f.resize(1));
    for (std::size_t k = 0; k < 12; ++k) {
        INFO("element " << k);
        CHECK(f.elements[k] == (k < 4 ? 1.0 : 0.0));
    }
    CHECK_FALSE(f.resize(4));
    CHECK(f.count() == 1);
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

namespace {

using Cube = data<tensor<double, dim<2>, dim<2>, dim<3>>>;

/// y(i, j, k) = x(i, j, k) + i: a rank-3 message, through one handler.
struct Lift {};

struct LiftBinding {
    using signature =
        branes::anatomy::flow::signature<ports<port<"x", Cube, direction::in>, port<"y", Cube, direction::out>>>;
    struct apply : handler<"apply", ins<"x">, outs<"y">> {
        static Cube call(const Lift&, const Cube& x) {
            Cube y;
            for (std::size_t i = 0; i < 2; ++i) {
                for (std::size_t j = 0; j < 2; ++j) {
                    for (std::size_t k = 0; k < 3; ++k) {
                        y(i, j, k) = x(i, j, k) + static_cast<double>(i);
                    }
                }
            }
            return y;
        }
    };
    using handlers = branes::anatomy::flow::handlers<apply>;
};

using LiftOp = registered<"Lift", Lift, LiftBinding>;

}  // namespace

TEST_CASE("data: a rank-3 message runs through the CPU stack (W17)", "[flow][data]") {
    Context ctx;
    stack_builder<Context> g("cube", baseline_class::research, ctx);
    const auto x = input<Cube>(g, "x");
    const auto y = output<Cube>(g, "y", x);
    const auto lift = instance<LiftOp>(g, "lift");
    scope r = g.region(x);
    auto [ys] = fire<"apply">(r, lift, x);
    g.produce(y, ys);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    stack s = std::move(built).value();

    Cube in;
    for (std::size_t e = 0; e < Cube::stored; ++e) {
        in.elements[e] = static_cast<double>(e);
    }
    s.input(x, in);
    CHECK(s.output(y)(0, 1, 2) == 5.0);
    CHECK(s.output(y)(1, 0, 0) == 6.0 + 1.0);
    CHECK(s.output(y)(1, 1, 2) == 11.0 + 1.0);
}
