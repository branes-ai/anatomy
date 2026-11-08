// The MTL5 adapter (workbench/mtl5.hpp; docs/plan/workbench, B4): a view
// reads and writes a data<D>'s elements in place; assign and add of an
// expression, and of one view to another, write the data<D> behind it; an
// expression of other dimensions returns false and leaves it unchanged; a
// view can't be copied; pack(unpack(d)) == d for every structure; a product
// through the views equals the loops; and nothing allocates.

#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/workbench/mtl5.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <new>
#include <type_traits>

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

namespace fl = branes::anatomy::flow;
namespace m5 = branes::anatomy::workbench::mtl5;

using V3 = fl::data<fl::vector<double, fl::dim<3>>>;
using M3 = fl::data<fl::matrix<double, fl::dim<3>, fl::dim<3>>>;
using M23 = fl::data<fl::matrix<double, fl::dim<2>, fl::dim<3>>>;
using Bounded = fl::data<fl::vector<double, fl::bounded<4>>>;
template <fl::structure S>
using Square = fl::data<fl::matrix<double, fl::dim<4>, fl::dim<4>, S>>;

}  // namespace

TEST_CASE("mtl5: a view reads and writes the elements in place", "[workbench][mtl5]") {
    M23 a;
    a(1, 2) = 7.0;
    auto v = m5::view(a);
    CHECK(v.rows() == 2);
    CHECK(v.cols() == 3);
    CHECK(v(1, 2) == 7.0);
    CHECK(&v.get()(1, 2) == &a.elements[5]);  // row-major, over the data's own elements
    v(0, 1) = 3.0;
    CHECK(a(0, 1) == 3.0);

    const V3 c = [] {
        V3 x;
        x(2) = 5.0;
        return x;
    }();
    const auto r = m5::view(c);
    CHECK(r(2) == 5.0);

    // A bounded value's view has its count of rows.
    Bounded b;
    REQUIRE(b.resize(2));
    CHECK(m5::view(b).rows() == 2);

    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<decltype(v)>);
    STATIC_REQUIRE_FALSE(std::is_copy_assignable_v<decltype(v)>);
    STATIC_REQUIRE_FALSE(std::is_move_constructible_v<decltype(v)>);
}

TEST_CASE("mtl5: assign and add write the data behind a view", "[workbench][mtl5]") {
    M3 a;
    M3 b;
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            a(i, j) = static_cast<double>(i * 3 + j);
            b(i, j) = 1.0;
        }
    }
    M3 c;
    auto va = m5::view(a);
    auto vb = m5::view(b);
    auto vc = m5::view(c);
    REQUIRE(m5::assign(vc, va.get() + vb.get()));  // an expression
    CHECK(c(2, 1) == 8.0);
    REQUIRE(m5::add(vc, va));  // another view
    CHECK(c(2, 1) == 15.0);
    REQUIRE(m5::assign(vc, mtl::trans(va.get())));
    CHECK(c(0, 2) == 6.0);
    REQUIRE(m5::assign(vc, vb));
    CHECK(c == b);
}

TEST_CASE("mtl5: an expression of other dimensions is refused, and nothing is written", "[workbench][mtl5]") {
    // A 3-vector into the view of a bounded value whose count is 0.
    Bounded b;
    V3 x;
    x(0) = 1.0;
    {
        auto vb = m5::view(b);
        auto vx = m5::view(x);
        CHECK_FALSE(m5::assign(vb, vx));
        CHECK_FALSE(m5::add(vb, vx));
    }
    CHECK(b == Bounded{});
    // A product whose shapes don't agree.
    M23 a;
    M3 c;
    c(0, 0) = 4.0;
    const M3 before = c;
    auto va = m5::view(a);
    auto vc = m5::view(c);
    CHECK_FALSE(m5::mult(va, va, vc));
    CHECK(c == before);
}

TEST_CASE("mtl5: a product through the views equals the loops", "[workbench][mtl5]") {
    M23 a;
    M3 b;
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            a(i, j) = 0.5 * static_cast<double>(i + 1) - static_cast<double>(j);
        }
    }
    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            b(i, j) = static_cast<double>(i) - 0.25 * static_cast<double>(j * j);
        }
    }
    M23 c;
    auto vc = m5::view(c);
    REQUIRE(m5::mult(m5::view(a), m5::view(b), vc));
    for (std::size_t i = 0; i < 2; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            double s = 0;
            for (std::size_t k = 0; k < 3; ++k) {
                s += a(i, k) * b(k, j);
            }
            CHECK(c(i, j) == s);
        }
    }
    // Matrix times vector, into a stack temporary.
    V3 x;
    x(0) = 1.0;
    x(1) = 2.0;
    x(2) = -1.0;
    m5::fixed_vector<double, 2> y;
    REQUIRE(m5::mult(m5::view(a), m5::view(x), y));
    CHECK(y(1) == a(1, 0) + 2.0 * a(1, 1) - a(1, 2));
}

namespace {

template <fl::structure S>
Square<S> filled() {
    Square<S> d;
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            if (Square<S>::inside(i, j) && (S != fl::structure::symmetric || i <= j)) {
                d(i, j) = 1.0 + static_cast<double>(i * 4 + j);
            }
        }
    }
    return d;
}

}  // namespace

TEST_CASE("mtl5: pack(unpack(d)) is d, for every structure", "[workbench][mtl5]") {
    const auto sym = filled<fl::structure::symmetric>();
    const auto m = m5::unpack(sym);
    CHECK(m(3, 1) == m(1, 3));
    CHECK(m(3, 1) == sym.at(1, 3));
    Square<fl::structure::symmetric> s2;
    m5::pack(m, s2);
    CHECK(s2 == sym);

    const auto up = filled<fl::structure::upper_triangular>();
    Square<fl::structure::upper_triangular> u2;
    m5::pack(m5::unpack(up), u2);
    CHECK(u2 == up);
    CHECK(m5::unpack(up)(3, 0) == 0.0);

    const auto lo = filled<fl::structure::lower_triangular>();
    Square<fl::structure::lower_triangular> l2;
    m5::pack(m5::unpack(lo), l2);
    CHECK(l2 == lo);

    const auto dg = filled<fl::structure::diagonal>();
    Square<fl::structure::diagonal> d2;
    m5::pack(m5::unpack(dg), d2);
    CHECK(d2 == dg);
}

TEST_CASE("mtl5: the adapter allocates nothing", "[workbench][mtl5]") {
    M3 a;
    M3 b;
    M3 c;
    V3 x;
    m5::fixed_vector<double, 3> y;
    auto sym = filled<fl::structure::symmetric>();
    {
        // MTL5 makes its thread pool on the first product; make it here.
        auto vc = m5::view(c);
        REQUIRE(m5::mult(m5::view(a), m5::view(b), vc));
        REQUIRE(m5::mult(m5::view(a), m5::view(x), y));
    }
    const std::size_t before = g_allocations.load();
    for (int pass = 0; pass < 10; ++pass) {
        auto va = m5::view(a);
        auto vb = m5::view(b);
        auto vc = m5::view(c);
        REQUIRE(m5::mult(va, vb, vc));
        REQUIRE(m5::mult(va, m5::view(x), y));
        REQUIRE(m5::assign(vc, va.get() + vb.get()));
        REQUIRE(m5::add(vc, vb));
        const auto m = m5::unpack(sym);
        m5::pack(m, sym);
    }
    CHECK(g_allocations.load() - before == 0);
}
