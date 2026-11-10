// Tests of the evaluator of a computational domain (workbench/evaluate.hpp;
// docs/plan/computational-domain, C1): on the toys of
// tests/fixtures/flow/compute_toys.hpp, a declaration evaluated on a firing's
// values gives what its handler gives (W11), in its deterministic order and
// in 20 random orders, bit for bit (W12). A declaration with a problem is
// reported, not evaluated.

#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/workbench/evaluate.hpp>

#include "../fixtures/flow/compute_toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <random>
#include <string>

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;
namespace ev = branes::anatomy::workbench::eval;
using branes::anatomy::Timestamp;

namespace {

/// A double in [-1, 1) from the generator's raw output.
double draw(std::mt19937_64& g) {
    return static_cast<double>(g() >> 11) * 0x1.0p-52 - 1.0;
}

template <class D>
fl::data<D> random_value(std::mt19937_64& g, std::size_t count = static_cast<std::size_t>(-1)) {
    fl::data<D> d;
    if constexpr (fl::data<D>::bounded) {
        (void)d.resize(count == static_cast<std::size_t>(-1) ? fl::data<D>::max_rows : count);
        for (std::size_t k = 0; k < d.count() * fl::data<D>::cols; ++k) {
            d.elements[k] = draw(g);
        }
    } else {
        for (auto& e : d.elements) {
            e = draw(g);
        }
    }
    return d;
}

std::string problems_of(const std::vector<cd::problem>& ps) {
    std::string out;
    for (const cd::problem& p : ps) {
        out += p.id + ": " + p.what + "\n";
    }
    return out;
}

/// W11: every out port and state the handler gives, the declaration gives,
/// to `tolerance` relative to its largest element.
void require_agree(const ev::firing& f, long double tolerance = 0) {
    INFO(problems_of(f.problems));
    REQUIRE(f.problems.empty());
    REQUIRE(!f.handler.empty());
    for (const auto& [name, h] : f.handler) {
        INFO("output " << name);
        const auto it = f.declared.find(name);
        REQUIRE(it != f.declared.end());
        CHECK(ev::difference(it->second, h) <= tolerance);
    }
}

/// W12: the same inputs in 20 random orders give the deterministic order's outputs, bit for bit.
void require_orders_agree(const cd::system& s, const cd::table& t, const ev::inputs& in) {
    const ev::outputs ref = ev::evaluate(s, t, in, 0);
    INFO(problems_of(ref.problems));
    REQUIRE(ref.problems.empty());
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        const ev::outputs o = ev::evaluate(s, t, in, seed);
        CHECK(o.ports == ref.ports);
        CHECK(o.states == ref.states);
    }
}

template <class B, fl::fixed_string H>
const cd::system& compute_of() {
    return fl::handler_t<B, H>::compute;
}
template <class B, fl::fixed_string H>
cd::table table_of() {
    return cd::table_of<B, fl::handler_t<B, H>>();
}

}  // namespace

TEST_CASE("evaluate: a reduction gives what its handler gives, in any order", "[workbench][evaluate][W11][W12]") {
    std::mt19937_64 g(1);
    for (int run = 0; run < 10; ++run) {
        const auto w = random_value<fl::matrix<double, fl::dim<3>, fl::dim<4>>>(g);
        const auto x = random_value<fl::vector<double, fl::dim<4>>>(g);
        // The declaration fixes the order of each row's sum, which the handler keeps.
        require_agree(ev::fire_both<toys::Linear34, "apply">({}, Timestamp{0}, w, x));
        ev::inputs in;
        in.ports["w"] = ev::to_tensor(w);
        in.ports["x"] = ev::to_tensor(x);
        using B = toys::LinearBinding<3, 4>;
        require_orders_agree(compute_of<B, "apply">(), table_of<B, "apply">(), in);
    }
}

TEST_CASE("evaluate: a map, a triangle and a program give what their handlers give",
          "[workbench][evaluate][W11][W12]") {
    std::mt19937_64 g(2);
    for (int run = 0; run < 10; ++run) {
        const auto x5 = random_value<fl::vector<double, fl::dim<5>>>(g);
        require_agree(ev::fire_both<toys::Scale5, "apply">({}, Timestamp{0}, x5));

        const auto l = random_value<fl::matrix<double, fl::dim<4>, fl::dim<4>, fl::structure::lower_triangular>>(g);
        const auto x4 = random_value<fl::vector<double, fl::dim<4>>>(g);
        require_agree(ev::fire_both<toys::TriMul4, "apply">({}, Timestamp{0}, l, x4));

        const auto a = random_value<fl::matrix<double, fl::dim<2>, fl::dim<3>>>(g);
        const auto x3 = random_value<fl::vector<double, fl::dim<3>>>(g);
        const auto b = random_value<fl::vector<double, fl::dim<2>>>(g);
        require_agree(ev::fire_both<toys::Affine23, "apply">({}, Timestamp{0}, a, x3, b));
        ev::inputs in;
        in.ports["a"] = ev::to_tensor(a);
        in.ports["x"] = ev::to_tensor(x3);
        in.ports["b"] = ev::to_tensor(b);
        using B = toys::AffineBinding<2, 3>;
        require_orders_agree(compute_of<B, "apply">(), table_of<B, "apply">(), in);
    }
}

TEST_CASE("evaluate: a state is read before the firing and written after it", "[workbench][evaluate][W11][W12]") {
    std::mt19937_64 g(3);
    toys::Delay<4> op;
    op.line = random_value<fl::vector<double, fl::dim<4>>>(g);
    const auto taps = random_value<fl::vector<double, fl::dim<4>>>(g);
    for (int run = 0; run < 10; ++run) {
        toys::Scalar x;
        x() = draw(g);
        const ev::firing f = ev::fire_both<toys::Delay4, "filter">(op, Timestamp{0}, x, taps);
        require_agree(f);
        CHECK(f.handler.count("state line") == 1);
        // The next firing starts from the line the handler left.
        (void)toys::DelayBinding<4>::filter::call(op, x, taps);
    }
    ev::inputs in;
    in.ports["x"] = ev::to_tensor(toys::Scalar{});
    in.ports["taps"] = ev::to_tensor(taps);
    in.states["line"] = ev::to_tensor(op.line);
    using B = toys::DelayBinding<4>;
    require_orders_agree(compute_of<B, "filter">(), table_of<B, "filter">(), in);
}

TEST_CASE("evaluate: a non-affine read gives what its handler gives", "[workbench][evaluate][W11][W12]") {
    std::mt19937_64 g(4);
    const auto x = random_value<fl::vector<double, fl::dim<8>>>(g);
    require_agree(ev::fire_both<toys::Butterfly8, "apply">({}, Timestamp{0}, x));
    ev::inputs in;
    in.ports["x"] = ev::to_tensor(x);
    using B = toys::ButterflyBinding<8>;
    require_orders_agree(compute_of<B, "apply">(), table_of<B, "apply">(), in);
}

TEST_CASE("evaluate: a bounded output takes its declared count, at every count", "[workbench][evaluate][W11]") {
    std::mt19937_64 g(5);
    for (std::size_t count = 0; count <= 4; ++count) {
        INFO("count " << count);
        const auto x = random_value<fl::vector<double, fl::bounded<4>>>(g, count);
        const ev::firing f = ev::fire_both<toys::CountScale4, "apply">({}, Timestamp{0}, x);
        require_agree(f);
        CHECK(f.declared.at("y").count == static_cast<std::int64_t>(count));
    }
}

TEST_CASE("evaluate: a declaration with a problem is reported, not evaluated", "[workbench][evaluate]") {
    static constexpr cd::system no_init = [] {
        cd::system s;
        const auto cols = s.extent("w", 1);
        const auto i = s.index("i", 0, s.extent("w", 0));
        const auto j = s.index("j", 0, cols);
        const auto acc = s.var("acc", cd::etype::f64);
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
        s.write(s.out("y").at(i), acc(i, j), s.where(j == cols - 1));
        return s;
    }();
    std::mt19937_64 g(6);
    ev::inputs in;
    in.ports["w"] = ev::to_tensor(random_value<fl::matrix<double, fl::dim<3>, fl::dim<4>>>(g));
    in.ports["x"] = ev::to_tensor(random_value<fl::vector<double, fl::dim<4>>>(g));
    const ev::outputs o = ev::evaluate(no_init, table_of<toys::LinearBinding<3, 4>, "apply">(), in);
    REQUIRE(!o.problems.empty());
    CHECK(o.problems.front().id == "K3");
    CHECK(o.ports.empty());
}
