#pragma once
// Toy operators that declare their computational domains (flow/compute.hpp;
// docs/plan/computational-domain, C1): a reduction, an elementwise map, a
// triangular space, a program of two stages joined by a local, a state, a
// non-affine read, and a bounded extent with its count. Each has a handler
// that computes what its declaration says, for the evaluator's tests (W11).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

#include <cstddef>
#include <tuple>

namespace toys {

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

template <std::size_t N>
using Vec = fl::data<fl::vector<double, fl::dim<N>>>;
template <std::size_t R, std::size_t C>
using Mat = fl::data<fl::matrix<double, fl::dim<R>, fl::dim<C>>>;
template <std::size_t N>
using Lower = fl::data<fl::matrix<double, fl::dim<N>, fl::dim<N>, fl::structure::lower_triangular>>;
using Scalar = fl::data<fl::scalar<double>>;
template <std::size_t N>
using Bounded = fl::data<fl::vector<double, fl::bounded<N>>>;

constexpr auto f64 = cd::etype::f64;

// ── A reduction: y = W·x ────────────────────────────────────────────

template <std::size_t R, std::size_t C>
struct Linear {};

template <std::size_t R, std::size_t C>
struct LinearBinding {
    using signature = fl::signature<fl::ports<fl::port<"w", Mat<R, C>, fl::direction::in>,
                                              fl::port<"x", Vec<C>, fl::direction::in>,
                                              fl::port<"y", Vec<R>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"w", "x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto rows = s.extent("w", 0);
            const auto cols = s.extent("w", 1);
            const auto i = s.index("i", 0, rows);
            const auto j = s.index("j", 0, cols);
            const auto acc = s.var("acc", f64);
            s.init(acc, s.literal(0, f64), s.face(j == -1));
            s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
            s.write(s.out("y").at(i), acc(i, j), s.where(j == cols - 1));
            return s;
        }();
        static Vec<R> call(const Linear<R, C>&, const Mat<R, C>& w, const Vec<C>& x) {
            Vec<R> y;
            for (std::size_t i = 0; i < R; ++i) {
                double acc = 0;
                for (std::size_t j = 0; j < C; ++j) {
                    acc = acc + w(i, j) * x(j);
                }
                y(i) = acc;
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

using Linear34 = fl::registered<"Toy_Linear_3x4", Linear<3, 4>, LinearBinding<3, 4>>;

// ── An elementwise map: y = 2·x ─────────────────────────────────────

template <std::size_t N>
struct Scale {};

template <std::size_t N>
struct ScaleBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"x", Vec<N>, fl::direction::in>, fl::port<"y", Vec<N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto i = s.index("i", 0, s.extent("x", 0));
            s.write(s.out("y").at(i), s.in("x")(i) * 2.0);
            return s;
        }();
        static Vec<N> call(const Scale<N>&, const Vec<N>& x) {
            Vec<N> y;
            for (std::size_t i = 0; i < N; ++i) {
                y(i) = x(i) * 2.0;
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

using Scale5 = fl::registered<"Toy_Scale_5", Scale<5>, ScaleBinding<5>>;

// ── A triangular space: y = L·x, L lower triangular ──────────────────

template <std::size_t N>
struct TriMul {};

template <std::size_t N>
struct TriMulBinding {
    using signature = fl::signature<fl::ports<fl::port<"l", Lower<N>, fl::direction::in>,
                                              fl::port<"x", Vec<N>, fl::direction::in>,
                                              fl::port<"y", Vec<N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"l", "x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto n = s.extent("x", 0);
            const auto i = s.index("i", 0, n);
            const auto j = s.index("j", 0, i + 1);  // 0 <= j <= i
            const auto acc = s.var("acc", f64);
            s.init(acc, s.literal(0, f64), s.face(j == -1));
            s.eq(acc, acc(i, j - 1) + s.in("l")(i, j) * s.in("x")(j));
            s.write(s.out("y").at(i), acc(i, j), s.where(j == i));
            return s;
        }();
        static Vec<N> call(const TriMul<N>&, const Lower<N>& l, const Vec<N>& x) {
            Vec<N> y;
            for (std::size_t i = 0; i < N; ++i) {
                double acc = 0;
                for (std::size_t j = 0; j <= i; ++j) {
                    acc = acc + l(i, j) * x(j);
                }
                y(i) = acc;
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

using TriMul4 = fl::registered<"Toy_TriMul_4", TriMul<4>, TriMulBinding<4>>;

// ── A program of two stages: t = A·x, then z = t + b ────────────────

template <std::size_t R, std::size_t C>
struct Affine {};

template <std::size_t R, std::size_t C>
struct AffineBinding {
    using signature = fl::signature<fl::ports<fl::port<"a", Mat<R, C>, fl::direction::in>,
                                              fl::port<"x", Vec<C>, fl::direction::in>,
                                              fl::port<"b", Vec<R>, fl::direction::in>,
                                              fl::port<"z", Vec<R>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"a", "x", "b">, fl::outs<"z">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto rows = s.extent("a", 0);
            const auto cols = s.extent("a", 1);
            const auto t = s.local("t", f64, rows);
            const auto i = s.index("i", 0, rows);
            const auto j = s.index("j", 0, cols);
            const auto acc = s.var("acc", f64);
            s.init(acc, s.literal(0, f64), s.face(j == -1));
            s.eq(acc, acc(i, j - 1) + s.in("a")(i, j) * s.in("x")(j));
            s.write(t.at(i), acc(i, j), s.where(j == cols - 1));
            s.stage();
            const auto k = s.index("k", 0, rows);
            s.write(s.out("z").at(k), t(k) + s.in("b")(k));
            return s;
        }();
        static Vec<R> call(const Affine<R, C>&, const Mat<R, C>& a, const Vec<C>& x, const Vec<R>& b) {
            Vec<R> z;
            for (std::size_t i = 0; i < R; ++i) {
                double acc = 0;
                for (std::size_t j = 0; j < C; ++j) {
                    acc = acc + a(i, j) * x(j);
                }
                z(i) = acc + b(i);
            }
            return z;
        }
    };
    using handlers = fl::handlers<apply>;
};

using Affine23 = fl::registered<"Toy_Affine_2x3", Affine<2, 3>, AffineBinding<2, 3>>;

// ── A state: a delay line of L taps, y = Σ taps(k)·d(k) ─────────────

template <std::size_t L>
struct Delay {
    Vec<L> line;
};

template <std::size_t L>
struct DelayBinding {
    using signature = fl::signature<fl::ports<fl::port<"x", Scalar, fl::direction::in>,
                                              fl::port<"taps", Vec<L>, fl::direction::in>,
                                              fl::port<"y", Scalar, fl::direction::out>>,
                                    fl::states<fl::state<"line", fl::vector<double, fl::dim<L>>>>>;
    struct filter : fl::handler<"filter", fl::ins<"x", "taps">, fl::outs<"y">, fl::state_role::update> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto n = s.extent("line", 0);
            const auto k = s.index("k", 0, n);
            const auto d = s.var("d", f64);  // the line after the shift
            const auto acc = s.var("acc", f64);
            s.eq(d, s.in("x")(), s.where(k == 0));
            s.eq(d, s.state("line")(k - 1), s.where(k >= 1));
            s.init(acc, s.literal(0, f64), s.face(k == -1));
            s.eq(acc, acc(k - 1) + s.in("taps")(k)*d(k));
            s.write(s.state("line").at(k), d(k));
            s.write(s.out("y").at(), acc(k), s.where(k == n - 1));
            return s;
        }();
        static Scalar call(Delay<L>& op, const Scalar& x, const Vec<L>& taps) {
            for (std::size_t k = L - 1; k >= 1; --k) {
                op.line(k) = op.line(k - 1);
            }
            op.line(0) = x();
            double acc = 0;
            for (std::size_t k = 0; k < L; ++k) {
                acc = acc + taps(k) * op.line(k);
            }
            Scalar y;
            y() = acc;
            return y;
        }
    };
    using handlers = fl::handlers<filter>;
    using cd_state = cd::members<cd::member<"line", &Delay<L>::line>>;
};

using Delay4 = fl::registered<"Toy_Delay_4", Delay<4>, DelayBinding<4>>;

// ── A non-affine read: one radix-2 butterfly stage, y(p) = x(p) ± x(p xor 1) ──

template <std::size_t N>
struct Butterfly {};

template <std::size_t N>
struct ButterflyBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"x", Vec<N>, fl::direction::in>, fl::port<"y", Vec<N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto p = s.index("p", 0, s.extent("x", 0));
            const auto x = s.in("x");
            // The even point of a pair adds its partner, the odd one subtracts it from it.
            const auto even = s.value(p % 2, f64);
            s.write(s.out("y").at(p), cd::select_lt(even, s.literal(0.5, f64), x(p) + x(p ^ 1), x(p ^ 1) - x(p)));
            return s;
        }();
        static Vec<N> call(const Butterfly<N>&, const Vec<N>& x) {
            Vec<N> y;
            for (std::size_t p = 0; p < N; ++p) {
                y(p) = p % 2 == 0 ? x(p) + x(p ^ 1) : x(p ^ 1) - x(p);
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

using Butterfly8 = fl::registered<"Toy_Butterfly_8", Butterfly<8>, ButterflyBinding<8>>;

// ── A bounded extent: y = 2·x over x's count, and y's count is x's ──

template <std::size_t N>
struct CountScale {};

template <std::size_t N>
struct CountScaleBinding {
    using signature = fl::signature<
        fl::ports<fl::port<"x", Bounded<N>, fl::direction::in>, fl::port<"y", Bounded<N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto n = s.count("x");
            const auto i = s.index("i", 0, n);
            s.write(s.out("y").at(i), s.in("x")(i) * 2.0);
            s.set_count(s.out("y"), n);
            return s;
        }();
        static Bounded<N> call(const CountScale<N>&, const Bounded<N>& x) {
            Bounded<N> y;
            (void)y.resize(x.count());
            for (std::size_t i = 0; i < x.count(); ++i) {
                y(i) = x(i) * 2.0;
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

using CountScale4 = fl::registered<"Toy_CountScale_4", CountScale<4>, CountScaleBinding<4>>;

}  // namespace toys
