#pragma once
/// @file mlp.hpp
/// @brief The MLP pattern: y = W₂·tanh(W₁·x + b₁) + b₂ (docs/plan/workbench,
/// "The patterns", B3).
///
/// x is a 16-vector, the hidden layer has 32 units, and the output 8. Layer 1
/// is f32 and layer 2 f64, joined by a Cast, so the element type is per edge.
/// The weights and biases are constants with values, drawn from the seed. One
/// region, triggered by each input x.
///
/// Its operators are Linear (y = W·x), BiasAdd, Tanh and Cast, each a template
/// over its element type and shape, registered under names that carry them
/// (`Linear_f32_32x16`).
///
/// Oracle (W2): a direct evaluation in long double, written without the
/// operators. The output agrees with it to a relative 1e-5, which layer 1's
/// f32 sets; layer 2, given layer 1's output, to 1e-12. The relative error is
/// in the max norm over a vector, and the largest over the run.
///
/// The operation count, for step 5's cost record to check against:
/// multiply_adds = 32·16 + 8·32 = 768, plus the epilogues: 32 + 8 bias adds,
/// 32 tanh and 32 casts.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/workbench/codecs.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace branes::anatomy::workbench::mlp {

namespace fl = branes::anatomy::flow;

template <class E, std::size_t N>
using vec = fl::data<fl::vector<E, fl::dim<N>>>;
template <class E, std::size_t R, std::size_t C>
using mat = fl::data<fl::matrix<E, fl::dim<R>, fl::dim<C>>>;

inline constexpr std::size_t kIn = 16;
inline constexpr std::size_t kHidden = 32;
inline constexpr std::size_t kOut = 8;
inline constexpr std::size_t multiply_adds = kHidden * kIn + kOut * kHidden;

using X = vec<float, kIn>;
using Hidden = vec<float, kHidden>;
using Y = vec<double, kOut>;
using W1 = mat<float, kHidden, kIn>;
using B1 = vec<float, kHidden>;
using W2 = mat<double, kOut, kHidden>;
using B2 = vec<double, kOut>;

// ── Operators ───────────────────────────────────────────────────────

/// y = W·x, each row summed in E, in column order.
template <class E, std::size_t R, std::size_t C>
struct Linear {
    [[nodiscard]] static vec<E, R> apply(const mat<E, R, C>& w, const vec<E, C>& x) noexcept {
        vec<E, R> y;
        for (std::size_t i = 0; i < R; ++i) {
            E sum{};
            for (std::size_t j = 0; j < C; ++j) {
                sum += w.elements[i * C + j] * x.elements[j];
            }
            y.elements[i] = sum;
        }
        return y;
    }
};

template <class E, std::size_t N>
struct BiasAdd {
    [[nodiscard]] static vec<E, N> apply(const vec<E, N>& x, const vec<E, N>& b) noexcept {
        vec<E, N> y;
        for (std::size_t i = 0; i < N; ++i) {
            y.elements[i] = x.elements[i] + b.elements[i];
        }
        return y;
    }
};

template <class E, std::size_t N>
struct Tanh {
    [[nodiscard]] static vec<E, N> apply(const vec<E, N>& x) noexcept {
        vec<E, N> y;
        for (std::size_t i = 0; i < N; ++i) {
            y.elements[i] = std::tanh(x.elements[i]);
        }
        return y;
    }
};

template <class From, class To, std::size_t N>
struct Cast {
    [[nodiscard]] static vec<To, N> apply(const vec<From, N>& x) noexcept {
        vec<To, N> y;
        for (std::size_t i = 0; i < N; ++i) {
            y.elements[i] = static_cast<To>(x.elements[i]);
        }
        return y;
    }
};

// ── Bindings ────────────────────────────────────────────────────────

template <class E, std::size_t R, std::size_t C>
struct LinearBinding {
    using signature = fl::signature<fl::ports<fl::port<"w", mat<E, R, C>, fl::direction::in>,
                                              fl::port<"x", vec<E, C>, fl::direction::in>,
                                              fl::port<"y", vec<E, R>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"w", "x">, fl::outs<"y">> {
        static vec<E, R> call(const Linear<E, R, C>&, const mat<E, R, C>& w, const vec<E, C>& x) {
            return Linear<E, R, C>::apply(w, x);
        }
    };
    using handlers = fl::handlers<apply>;
};

template <class E, std::size_t N>
struct BiasAddBinding {
    using signature = fl::signature<fl::ports<fl::port<"x", vec<E, N>, fl::direction::in>,
                                              fl::port<"b", vec<E, N>, fl::direction::in>,
                                              fl::port<"y", vec<E, N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x", "b">, fl::outs<"y">> {
        static vec<E, N> call(const BiasAdd<E, N>&, const vec<E, N>& x, const vec<E, N>& b) {
            return BiasAdd<E, N>::apply(x, b);
        }
    };
    using handlers = fl::handlers<apply>;
};

template <class E, std::size_t N>
struct TanhBinding {
    using signature = fl::signature<
        fl::ports<fl::port<"x", vec<E, N>, fl::direction::in>, fl::port<"y", vec<E, N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static vec<E, N> call(const Tanh<E, N>&, const vec<E, N>& x) {
            return Tanh<E, N>::apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

template <class From, class To, std::size_t N>
struct CastBinding {
    using signature = fl::signature<
        fl::ports<fl::port<"x", vec<From, N>, fl::direction::in>, fl::port<"y", vec<To, N>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static vec<To, N> call(const Cast<From, To, N>&, const vec<From, N>& x) {
            return Cast<From, To, N>::apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

using Linear1 = fl::registered<"Linear_f32_32x16", Linear<float, kHidden, kIn>, LinearBinding<float, kHidden, kIn>>;
using BiasAdd1 = fl::registered<"BiasAdd_f32_32", BiasAdd<float, kHidden>, BiasAddBinding<float, kHidden>>;
using Tanh1 = fl::registered<"Tanh_f32_32", Tanh<float, kHidden>, TanhBinding<float, kHidden>>;
using Cast1 = fl::registered<"Cast_f32_f64_32", Cast<float, double, kHidden>, CastBinding<float, double, kHidden>>;
using Linear2 = fl::registered<"Linear_f64_8x32", Linear<double, kOut, kHidden>, LinearBinding<double, kOut, kHidden>>;
using BiasAdd2 = fl::registered<"BiasAdd_f64_8", BiasAdd<double, kOut>, BiasAddBinding<double, kOut>>;

// ── Configuration, weights and inputs ───────────────────────────────

struct config {
    std::uint64_t seed = 1;
    std::int64_t period_ns = 10'000'000;  ///< an input every 10 ms
};

/// What the operators are built from. They have no parameters of their own.
struct context {
    config cfg;
};

struct weights {
    W1 w1;
    B1 b1;
    W2 w2;
    B2 b2;
};

/// The weights and biases, drawn from the seed: uniform in ±1/√(fan in).
[[nodiscard]] inline weights weights_of(const config& c) {
    rng r(substream(c.seed, 0));
    weights w;
    const double a1 = 1.0 / std::sqrt(static_cast<double>(kIn));
    const double a2 = 1.0 / std::sqrt(static_cast<double>(kHidden));
    for (float& e : w.w1.elements) {
        e = static_cast<float>(r.uniform(-a1, a1));
    }
    for (float& e : w.b1.elements) {
        e = static_cast<float>(r.uniform(-a1, a1));
    }
    for (double& e : w.w2.elements) {
        e = r.uniform(-a2, a2);
    }
    for (double& e : w.b2.elements) {
        e = r.uniform(-a2, a2);
    }
    return w;
}

/// The k-th input: uniform in [-1, 1).
[[nodiscard]] inline X input_of(const config& c, std::uint64_t k) {
    rng r(substream(c.seed, k + 1));
    X x;
    for (float& e : x.elements) {
        e = static_cast<float>(r.uniform(-1.0, 1.0));
    }
    return x;
}

// ── The graph ───────────────────────────────────────────────────────

struct ports {
    fl::input_port<X> x;
    fl::output_port<Hidden> hidden;  ///< layer 1's output, tanh(W₁·x + b₁)
    fl::output_port<Y> y;
};

template <class G>
[[nodiscard]] ports graph(G& g, const config& c) {
    const weights w = weights_of(c);
    const auto x = fl::input<X>(g, "x");
    const auto w1 = fl::constant<W1>(g, "w1", w.w1);
    const auto b1 = fl::constant<B1>(g, "b1", w.b1);
    const auto w2 = fl::constant<W2>(g, "w2", w.w2);
    const auto b2 = fl::constant<B2>(g, "b2", w.b2);
    const auto hidden = fl::output<Hidden>(g, "hidden", x);
    const auto y = fl::output<Y>(g, "y", x);
    const auto linear1 = fl::instance<Linear1>(g, "linear1");
    const auto bias1 = fl::instance<BiasAdd1>(g, "bias1");
    const auto tanh1 = fl::instance<Tanh1>(g, "tanh1");
    const auto cast = fl::instance<Cast1>(g, "cast");
    const auto linear2 = fl::instance<Linear2>(g, "linear2");
    const auto bias2 = fl::instance<BiasAdd2>(g, "bias2");

    fl::scope r = g.region(x);
    auto [h1] = fl::fire<"apply">(r, linear1, w1, x);
    auto [a1] = fl::fire<"apply">(r, bias1, h1, b1);
    auto [t1] = fl::fire<"apply">(r, tanh1, a1);
    auto [c1] = fl::fire<"apply">(r, cast, t1);
    auto [h2] = fl::fire<"apply">(r, linear2, w2, c1);
    auto [out] = fl::fire<"apply">(r, bias2, h2, b2);
    g.produce(hidden, t1);
    g.produce(y, out);
    return ports{x, hidden, y};
}

// ── The run, by hand (W1), and the oracle (W2) ──────────────────────

/// What the oracle reads after each input.
struct sample {
    Hidden hidden;
    Y y;
};

/// The operators called by hand, in the graph's order, on its constants.
[[nodiscard]] inline sample hand_run(const weights& w, const X& x) {
    const auto h1 = LinearBinding<float, kHidden, kIn>::apply::call({}, w.w1, x);
    const auto a1 = BiasAddBinding<float, kHidden>::apply::call({}, h1, w.b1);
    const auto t1 = TanhBinding<float, kHidden>::apply::call({}, a1);
    const auto c1 = CastBinding<float, double, kHidden>::apply::call({}, t1);
    const auto h2 = LinearBinding<double, kOut, kHidden>::apply::call({}, w.w2, c1);
    return sample{t1, BiasAddBinding<double, kOut>::apply::call({}, h2, w.b2)};
}

/// W₂·h + b₂ in long double.
[[nodiscard]] inline std::vector<long double> layer2(const weights& w, const std::vector<long double>& h) {
    std::vector<long double> y(kOut);
    for (std::size_t i = 0; i < kOut; ++i) {
        long double s = w.b2.elements[i];
        for (std::size_t j = 0; j < kHidden; ++j) {
            s += static_cast<long double>(w.w2.elements[i * kHidden + j]) * h[j];
        }
        y[i] = s;
    }
    return y;
}

/// The MLP in long double, from x.
[[nodiscard]] inline std::vector<long double> evaluate(const weights& w, const X& x) {
    std::vector<long double> h(kHidden);
    for (std::size_t i = 0; i < kHidden; ++i) {
        long double s = w.b1.elements[i];
        for (std::size_t j = 0; j < kIn; ++j) {
            s += static_cast<long double>(w.w1.elements[i * kIn + j]) * static_cast<long double>(x.elements[j]);
        }
        h[i] = std::tanh(s);
    }
    return layer2(w, h);
}

struct pattern {
    static constexpr std::string_view name = "mlp";
    static constexpr std::size_t default_inputs = 100;
    using config = mlp::config;
    using context = mlp::context;
    using ports = mlp::ports;
    using sample = mlp::sample;

    template <class G>
    [[nodiscard]] static ports graph(G& g, const config& c) {
        return mlp::graph(g, c);
    }

    [[nodiscard]] static std::vector<source> sources(const ports& p, const config& c) {
        return {sequence(p.x, c.period_ns, [c](std::uint64_t k, Timestamp) { return input_of(c, k); })};
    }

    [[nodiscard]] static sample collect(const fl::stack& s, const ports& p, std::size_t, std::size_t, Timestamp) {
        return sample{s.output(p.hidden), s.output(p.y)};
    }

    /// Sample i is the i-th input's: the MLP has one source.
    [[nodiscard]] static report oracle(const config& c, std::span<const sample> samples) {
        const weights w = weights_of(c);
        long double whole = 0;
        long double second = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const sample& s = samples[i];
            const std::vector<long double> want = evaluate(w, input_of(c, i));
            whole = std::fmax(whole, relative_max(s.y.elements, want, kOut));
            std::vector<long double> h(kHidden);
            for (std::size_t j = 0; j < kHidden; ++j) {
                h[j] = s.hidden.elements[j];
            }
            second = std::fmax(second, relative_max(s.y.elements, layer2(w, h), kOut));
        }
        report r{std::string(name), {}};
        r.checks.push_back(at_least("inputs", static_cast<double>(samples.size()), 1));
        r.checks.push_back(at_most("y against long double, relative", static_cast<double>(whole), 1e-5));
        r.checks.push_back(at_most("layer 2 given layer 1's output, relative", static_cast<double>(second), 1e-12));
        return r;
    }

    [[nodiscard]] static fl::trace_codecs codecs() {
        fl::trace_codecs c;
        add_data<fl::vector<float, fl::dim<kIn>>>(c);
        add_data<fl::vector<float, fl::dim<kHidden>>>(c);
        add_data<fl::vector<double, fl::dim<kHidden>>>(c);
        add_data<fl::vector<double, fl::dim<kOut>>>(c);
        return c;
    }
};

}  // namespace branes::anatomy::workbench::mlp
