// The workbench's driver, sources and trace codecs (workbench/driver.hpp,
// sources.hpp, codecs.hpp; docs/plan/workbench, B3): sources merged in time
// order, ties in the order they are declared; a seed's deviates; and the
// encodings of data<D> and window<M, N>.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/window.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/workbench/codecs.hpp>
#include <branes/anatomy/workbench/driver.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <catch2/catch_test_macros.hpp>

#include <complex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
using branes::anatomy::Timestamp;

using S = fl::data<fl::scalar<double>>;

/// Passes its input on.
struct Echo {};
struct EchoBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"x", S, fl::direction::in>, fl::port<"y", S, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static S call(const Echo&, const S& x) {
            return x;
        }
    };
    using handlers = fl::handlers<apply>;
};
using EchoOp = fl::registered<"Echo", Echo, EchoBinding>;
struct Ctx {};

}  // namespace

TEST_CASE("driver: sources merge in time order, ties in declared order", "[workbench][driver]") {
    const Ctx ctx;
    fl::stack_builder<Ctx> g("d", fl::baseline_class::research, ctx);
    const auto a = fl::input<S>(g, "a");
    const auto b = fl::input<S>(g, "b");
    const auto oa = fl::output<S>(g, "oa", a);
    const auto ob = fl::output<S>(g, "ob", b);
    const auto ea = fl::instance<EchoOp>(g, "ea");
    const auto eb = fl::instance<EchoOp>(g, "eb");
    fl::scope ra = g.region(a);
    auto [ya] = fl::fire<"apply">(ra, ea, a);
    g.produce(oa, ya);
    fl::scope rb = g.region(b);
    auto [yb] = fl::fire<"apply">(rb, eb, b);
    g.produce(ob, yb);
    auto built = std::move(g).build_stack();
    REQUIRE(built);
    fl::stack st = std::move(built).value();

    // a every 3 ns from 0, b every 2 ns from 0, and a's twin every 6 ns from 1.
    const auto value = [](std::uint64_t k, Timestamp t) {
        S m;
        m() = static_cast<double>(t.ns) + 0.001 * static_cast<double>(k);
        return m;
    };
    const std::vector<wb::source> sources = {
        wb::sequence(a, 3, value), wb::sequence(b, 2, value), wb::sequence(a, 6, value, 1)};
    std::vector<std::pair<std::size_t, std::int64_t>> order;
    wb::drive(st, sources, 9, [&](std::size_t i, std::size_t src, Timestamp t) {
        CHECK(i == order.size());
        order.emplace_back(src, t.ns);
        // The stack has run the input's region before the driver says so.
        const S& out = src == 1 ? st.output(ob) : st.output(oa);
        CHECK(static_cast<std::int64_t>(out()) == t.ns);
    });
    const std::vector<std::pair<std::size_t, std::int64_t>> want = {
        {0, 0}, {1, 0}, {2, 1}, {1, 2}, {0, 3}, {1, 4}, {0, 6}, {1, 6}, {2, 7}};
    CHECK(order == want);
    CHECK(sources[0].name == "a");
}

TEST_CASE("driver: a seed's deviates are the same each time, and in range", "[workbench][driver]") {
    wb::rng a(7);
    wb::rng b(7);
    for (int i = 0; i < 1000; ++i) {
        const double u = a.unit();
        CHECK(u == b.unit());
        CHECK(u >= 0.0);
        CHECK(u < 1.0);
    }
    CHECK(wb::rng(7).next() != wb::rng(8).next());
    CHECK(wb::substream(1, 0) != wb::substream(1, 1));
    CHECK(wb::substream(1, 0) != wb::substream(2, 0));
    // A normal deviate's mean and variance, over many.
    wb::rng n(11);
    double sum = 0;
    double sq = 0;
    const int count = 200000;
    for (int i = 0; i < count; ++i) {
        const double x = n.normal();
        sum += x;
        sq += x * x;
    }
    CHECK(sum / count < 0.01);
    CHECK(sum / count > -0.01);
    CHECK(sq / count < 1.01);
    CHECK(sq / count > 0.99);
}

TEST_CASE("codecs: data<D> as its count and its elements' bit patterns", "[workbench][codecs]") {
    fl::data<fl::vector<float, fl::dim<2>>> v;
    v(0) = 1.0f;
    v(1) = -2.0f;
    const auto b = wb::encode(v);
    REQUIRE(b.size() == 8 + 2 * 4);
    CHECK(b[0] == std::byte{2});         // the count, little-endian
    CHECK(b[8 + 3] == std::byte{0x3f});  // 1.0f is 0x3f800000

    // A bounded value: the count, then every stored element, so equal values encode alike.
    fl::data<fl::vector<double, fl::bounded<3>>> x;
    REQUIRE(x.resize(1));
    x(0) = 5.0;
    fl::data<fl::vector<double, fl::bounded<3>>> y;
    REQUIRE(y.resize(3));
    y(1) = 9.0;
    REQUIRE(y.resize(1));
    y(0) = 5.0;
    CHECK(wb::encode(x) == wb::encode(y));
    CHECK(wb::encode(x).size() == 8 + 3 * 8);

    fl::data<fl::vector<std::complex<double>, fl::dim<1>>> c;
    c(0) = {1.0, 2.0};
    CHECK(wb::encode(c).size() == 8 + 16);

    fl::trace_codecs codecs;
    wb::add_data<fl::vector<float, fl::dim<2>>>(codecs);
    const auto* f = codecs.find(typeid(decltype(v)));
    REQUIRE(f != nullptr);
    CHECK((*f)(&v) == b);
}

TEST_CASE("codecs: a window as its count, its dropped, and its items", "[workbench][codecs]") {
    using M = fl::data<fl::scalar<double>>;
    fl::window<M, 4> w;
    w.count = 2;
    w.dropped = 7;
    w.items[0]() = 1.0;
    w.items[1]() = 2.0;
    w.items[2]() = 3.0;  // past the count: not encoded
    const auto item = [](const M& m) { return wb::encode(m); };
    const auto b = wb::encode(w, item);
    CHECK(b.size() == 8 + 8 + 2 * (4 + 16));
    CHECK(b[8] == std::byte{7});
    fl::trace_codecs codecs;
    wb::add_window<M, 4>(codecs, item);
    REQUIRE(codecs.find(typeid(fl::window<M, 4>)) != nullptr);
}
