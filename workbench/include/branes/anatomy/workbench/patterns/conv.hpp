#pragma once
/// @file conv.hpp
/// @brief The convolution pattern: a 2D convolution of a 3 × 32 × 32 image
/// with 8 kernels of 3 × 3, then a bias and a ReLU, in one region
/// (docs/plan/computational-domain, C6).
///
/// Its messages are rank-3 data<D>, row-major (flow/data.hpp), and its
/// kernels a rank-4 one: the image x(c, i, j), the kernels w(o, c, ky, kx),
/// the bias b(o) and the feature maps y(o, i, j). Four operators, each a
/// template over its element type and shape:
///   Pad1     x padded with one zero on each side of each channel: 3 × 34 × 34
///   Conv2d   conv(o, i, j) = Σ_c Σ_ky Σ_kx w(o, c, ky, kx)·xp(c, i + ky, j + kx),
///            summed from 0 in the order c, ky, kx: a cross-correlation, as a
///            neural network's convolution is. The output has the input's size
///   BiasAdd  conv(o, i, j) + b(o)
///   Relu     y = x ≤ 0 ? 0 : x
/// The convolution's computational domain is (o, i, j, c, ky, kx): a sliding
/// window, whose read of the padded input at (c, i + ky, j + kx) is affine.
/// Its input face is a window into the padded input, so it needs no guard.
///
/// The kernels and the bias are constants, drawn from the seed. The first
/// three inputs are impulses, a 1 in one channel at (kImpulseRow,
/// kImpulseCol); the rest are uniform in [-1, 1).
///
/// Oracle (W2):
///   the convolution's output, and y, against a direct summation in long
///     double, to 1e-12;
///   an impulse in channel c returns channel c of each kernel, reversed about
///     the impulse, exactly, and zero everywhere else.
///
/// The operation count, for step 5's cost record to check against:
/// multiply_adds = 8·32·32·3·9 = 221 184, plus 8·32·32 bias adds and
/// 8·32·32 selects.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
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
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace branes::anatomy::workbench::conv {

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

template <class E, std::size_t A, std::size_t B, std::size_t C>
using tensor3 = fl::data<fl::tensor<E, fl::dim<A>, fl::dim<B>, fl::dim<C>>>;
template <class E, std::size_t O, std::size_t C, std::size_t K>
using kernels = fl::data<fl::tensor<E, fl::dim<O>, fl::dim<C>, fl::dim<K>, fl::dim<K>>>;
template <class E, std::size_t N>
using vec = fl::data<fl::vector<E, fl::dim<N>>>;

inline constexpr std::size_t kChannels = 3;
inline constexpr std::size_t kSize = 32;  ///< the image's rows and columns
inline constexpr std::size_t kKernels = 8;
inline constexpr std::size_t kTaps = 3;  ///< a kernel's rows and columns
inline constexpr std::size_t kPadded = kSize + kTaps - 1;
inline constexpr std::size_t multiply_adds = kKernels * kSize * kSize * kChannels * kTaps * kTaps;
/// Where the first three inputs' impulses are.
inline constexpr std::size_t kImpulseRow = 9;
inline constexpr std::size_t kImpulseCol = 20;

using Image = tensor3<double, kChannels, kSize, kSize>;
using Padded = tensor3<double, kChannels, kPadded, kPadded>;
using Weights = kernels<double, kKernels, kChannels, kTaps>;
using Bias = vec<double, kKernels>;
using Features = tensor3<double, kKernels, kSize, kSize>;

// ── Operators ───────────────────────────────────────────────────────

/// x with one zero on each side of each channel's rows and columns.
template <class E, std::size_t C, std::size_t H, std::size_t W>
struct Pad1 {
    [[nodiscard]] static tensor3<E, C, H + 2, W + 2> apply(const tensor3<E, C, H, W>& x) noexcept {
        tensor3<E, C, H + 2, W + 2> y;  // zero
        for (std::size_t c = 0; c < C; ++c) {
            for (std::size_t i = 0; i < H; ++i) {
                for (std::size_t j = 0; j < W; ++j) {
                    y(c, i + 1, j + 1) = x(c, i, j);
                }
            }
        }
        return y;
    }
};

/// conv(o, i, j) = Σ w(o, c, ky, kx)·x(c, i + ky, j + kx) over a padded x,
/// summed in E from 0 in the order c, ky, kx.
template <class E, std::size_t O, std::size_t C, std::size_t K, std::size_t H, std::size_t W>
struct Conv2d {
    [[nodiscard]] static tensor3<E, O, H, W> apply(const kernels<E, O, C, K>& w,
                                                   const tensor3<E, C, H + K - 1, W + K - 1>& x) noexcept {
        tensor3<E, O, H, W> y;
        for (std::size_t o = 0; o < O; ++o) {
            for (std::size_t i = 0; i < H; ++i) {
                for (std::size_t j = 0; j < W; ++j) {
                    E sum{};
                    for (std::size_t c = 0; c < C; ++c) {
                        for (std::size_t ky = 0; ky < K; ++ky) {
                            for (std::size_t kx = 0; kx < K; ++kx) {
                                sum += w(o, c, ky, kx) * x(c, i + ky, j + kx);
                            }
                        }
                    }
                    y(o, i, j) = sum;
                }
            }
        }
        return y;
    }
};

/// x(o, i, j) + b(o).
template <class E, std::size_t O, std::size_t H, std::size_t W>
struct BiasAdd {
    [[nodiscard]] static tensor3<E, O, H, W> apply(const tensor3<E, O, H, W>& x, const vec<E, O>& b) noexcept {
        tensor3<E, O, H, W> y;
        for (std::size_t o = 0; o < O; ++o) {
            for (std::size_t i = 0; i < H; ++i) {
                for (std::size_t j = 0; j < W; ++j) {
                    y(o, i, j) = x(o, i, j) + b(o);
                }
            }
        }
        return y;
    }
};

/// x ≤ 0 ? 0 : x, elementwise.
template <class E, std::size_t O, std::size_t H, std::size_t W>
struct Relu {
    [[nodiscard]] static tensor3<E, O, H, W> apply(const tensor3<E, O, H, W>& x) noexcept {
        tensor3<E, O, H, W> y;
        for (std::size_t k = 0; k < x.elements.size(); ++k) {
            y.elements[k] = x.elements[k] <= E{0} ? E{0} : x.elements[k];
        }
        return y;
    }
};

// ── Bindings ────────────────────────────────────────────────────────

template <class E, std::size_t C, std::size_t H, std::size_t W>
struct Pad1Binding {
    using signature = fl::signature<fl::ports<fl::port<"x", tensor3<E, C, H, W>, fl::direction::in>,
                                              fl::port<"y", tensor3<E, C, H + 2, W + 2>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        /// The image inside, and zeros on the four borders: each element written once (K5).
        static constexpr cd::system compute = [] {
            constexpr cd::etype e = cd::etype_of<E>;
            cd::system s;
            const auto rows = s.extent("x", 1);
            const auto cols = s.extent("x", 2);
            const auto c = s.index("c", 0, s.extent("x", 0));
            const auto i = s.index("i", 0, rows + 2);
            const auto j = s.index("j", 0, cols + 2);
            const auto y = s.out("y").at(c, i, j);
            s.write(y, s.in("x")(c, i - 1, j - 1), s.where(i >= 1, i <= rows, j >= 1, j <= cols));
            s.write(y, s.literal(0, e), s.where(i == 0));
            s.write(y, s.literal(0, e), s.where(i == rows + 1));
            s.write(y, s.literal(0, e), s.where(i >= 1, i <= rows, j == 0));
            s.write(y, s.literal(0, e), s.where(i >= 1, i <= rows, j == cols + 1));
            return s;
        }();
        static tensor3<E, C, H + 2, W + 2> call(const Pad1<E, C, H, W>&, const tensor3<E, C, H, W>& x) {
            return Pad1<E, C, H, W>::apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

template <class E, std::size_t O, std::size_t C, std::size_t K, std::size_t H, std::size_t W>
struct Conv2dBinding {
    using signature = fl::signature<fl::ports<fl::port<"w", kernels<E, O, C, K>, fl::direction::in>,
                                              fl::port<"x", tensor3<E, C, H + K - 1, W + K - 1>, fl::direction::in>,
                                              fl::port<"y", tensor3<E, O, H, W>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"w", "x">, fl::outs<"y">> {
        /// A sliding window: a reduction along (c, ky, kx), in that order,
        /// chained through acc. Each point reads the term before it: the
        /// one at kx − 1 along a kernel's row, the last of the row before at
        /// kx = 0, and the last of the channel before at the row's start.
        /// O·H·W·C·K² multiply-adds.
        static constexpr cd::system compute = [] {
            constexpr cd::etype e = cd::etype_of<E>;
            cd::system s;
            const auto chans = s.extent("w", 1);
            const auto krows = s.extent("w", 2);
            const auto kcols = s.extent("w", 3);
            const auto o = s.index("o", 0, s.extent("w", 0));
            const auto i = s.index("i", 0, s.extent("y", 1));
            const auto j = s.index("j", 0, s.extent("y", 2));
            const auto c = s.index("c", 0, chans);
            const auto ky = s.index("ky", 0, krows);
            const auto kx = s.index("kx", 0, kcols);
            const auto acc = s.var("acc", e);
            const auto term = s.in("w")(o, c, ky, kx) * s.in("x")(c, i + ky, j + kx);
            s.init(acc, s.literal(0, e), s.face(c == 0, ky == 0, kx == -1));
            s.eq(acc, acc(o, i, j, c, ky, kx - 1) + term, s.where(kx >= 1));
            s.eq(acc, acc(o, i, j, c, ky, kx - 1) + term, s.where(c == 0, ky == 0, kx == 0));
            s.eq(acc, acc(o, i, j, c, ky - 1, kcols - 1) + term, s.where(ky >= 1, kx == 0));
            s.eq(acc, acc(o, i, j, c - 1, krows - 1, kcols - 1) + term, s.where(c >= 1, ky == 0, kx == 0));
            s.write(s.out("y").at(o, i, j),
                    acc(o, i, j, c, ky, kx),
                    s.where(c == chans - 1, ky == krows - 1, kx == kcols - 1));
            return s;
        }();
        static tensor3<E, O, H, W> call(const Conv2d<E, O, C, K, H, W>&,
                                        const kernels<E, O, C, K>& w,
                                        const tensor3<E, C, H + K - 1, W + K - 1>& x) {
            return Conv2d<E, O, C, K, H, W>::apply(w, x);
        }
    };
    using handlers = fl::handlers<apply>;
};

template <class E, std::size_t O, std::size_t H, std::size_t W>
struct BiasAddBinding {
    using signature = fl::signature<fl::ports<fl::port<"x", tensor3<E, O, H, W>, fl::direction::in>,
                                              fl::port<"b", vec<E, O>, fl::direction::in>,
                                              fl::port<"y", tensor3<E, O, H, W>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x", "b">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto o = s.index("o", 0, s.extent("x", 0));
            const auto i = s.index("i", 0, s.extent("x", 1));
            const auto j = s.index("j", 0, s.extent("x", 2));
            s.write(s.out("y").at(o, i, j), s.in("x")(o, i, j) + s.in("b")(o));
            return s;
        }();
        static tensor3<E, O, H, W> call(const BiasAdd<E, O, H, W>&, const tensor3<E, O, H, W>& x, const vec<E, O>& b) {
            return BiasAdd<E, O, H, W>::apply(x, b);
        }
    };
    using handlers = fl::handlers<apply>;
};

template <class E, std::size_t O, std::size_t H, std::size_t W>
struct ReluBinding {
    using signature = fl::signature<fl::ports<fl::port<"x", tensor3<E, O, H, W>, fl::direction::in>,
                                              fl::port<"y", tensor3<E, O, H, W>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            constexpr cd::etype e = cd::etype_of<E>;
            cd::system s;
            const auto o = s.index("o", 0, s.extent("x", 0));
            const auto i = s.index("i", 0, s.extent("x", 1));
            const auto j = s.index("j", 0, s.extent("x", 2));
            const auto x = s.in("x")(o, i, j);
            s.write(s.out("y").at(o, i, j), cd::select_le(x, s.literal(0, e), s.literal(0, e), x));
            return s;
        }();
        static tensor3<E, O, H, W> call(const Relu<E, O, H, W>&, const tensor3<E, O, H, W>& x) {
            return Relu<E, O, H, W>::apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

using Pad64 = fl::
    registered<"Pad1_f64_3x32x32", Pad1<double, kChannels, kSize, kSize>, Pad1Binding<double, kChannels, kSize, kSize>>;
using Conv64 = fl::registered<"Conv2d_f64_8x3x3x3_32x32",
                              Conv2d<double, kKernels, kChannels, kTaps, kSize, kSize>,
                              Conv2dBinding<double, kKernels, kChannels, kTaps, kSize, kSize>>;
using BiasAdd64 = fl::registered<"BiasAdd_f64_8x32x32",
                                 BiasAdd<double, kKernels, kSize, kSize>,
                                 BiasAddBinding<double, kKernels, kSize, kSize>>;
using Relu64 = fl::
    registered<"Relu_f64_8x32x32", Relu<double, kKernels, kSize, kSize>, ReluBinding<double, kKernels, kSize, kSize>>;

// ── Configuration, weights and inputs ───────────────────────────────

struct config {
    std::uint64_t seed = 1;
    std::int64_t period_ns = 10'000'000;  ///< an image every 10 ms
};

/// What the operators are built from. They have no parameters of their own.
struct context {
    config cfg;
};

struct weights {
    Weights w;
    Bias b;
};

/// The kernels and the bias, drawn from the seed: uniform in ±1/√(fan in).
[[nodiscard]] inline weights weights_of(const config& c) {
    rng r(substream(c.seed, 0));
    weights w;
    const double a = 1.0 / std::sqrt(static_cast<double>(kChannels * kTaps * kTaps));
    for (double& e : w.w.elements) {
        e = r.uniform(-a, a);
    }
    for (double& e : w.b.elements) {
        e = r.uniform(-a, a);
    }
    return w;
}

/// The k-th input: for k < 3, an impulse in channel k; then uniform in [-1, 1).
[[nodiscard]] inline Image input_of(const config& c, std::uint64_t k) {
    Image x;
    if (k < kChannels) {
        x(static_cast<std::size_t>(k), kImpulseRow, kImpulseCol) = 1.0;
        return x;
    }
    rng r(substream(c.seed, k + 1));
    for (double& e : x.elements) {
        e = r.uniform(-1.0, 1.0);
    }
    return x;
}

// ── The graph ───────────────────────────────────────────────────────

struct ports {
    fl::input_port<Image> x;
    fl::output_port<Features> conv;  ///< the convolution's output, before the bias
    fl::output_port<Features> y;
};

template <class G>
[[nodiscard]] ports graph(G& g, const config& c) {
    const weights w = weights_of(c);
    const auto x = fl::input<Image>(g, "x");
    const auto kw = fl::constant<Weights>(g, "w", w.w);
    const auto kb = fl::constant<Bias>(g, "b", w.b);
    const auto conv = fl::output<Features>(g, "conv", x);
    const auto y = fl::output<Features>(g, "y", x);
    const auto pad = fl::instance<Pad64>(g, "pad");
    const auto conv2d = fl::instance<Conv64>(g, "conv2d");
    const auto bias = fl::instance<BiasAdd64>(g, "bias");
    const auto relu = fl::instance<Relu64>(g, "relu");

    fl::scope r = g.region(x);
    auto [xp] = fl::fire<"apply">(r, pad, x);
    auto [z] = fl::fire<"apply">(r, conv2d, kw, xp);
    auto [zb] = fl::fire<"apply">(r, bias, z, kb);
    auto [out] = fl::fire<"apply">(r, relu, zb);
    g.produce(conv, z);
    g.produce(y, out);
    return ports{x, conv, y};
}

// ── The run, by hand (W1), and the oracle (W2) ──────────────────────

/// What the oracle reads after each input.
struct sample {
    Features conv;
    Features y;

    friend bool operator==(const sample&, const sample&) = default;
};

/// The four operators called by hand, in the graph's order, on kernels w and
/// bias b: the convolution's output, and y. Any instantiation of them, so a
/// test can run the declarations on a small one. The hook sees each
/// handler's values before it runs (pattern.hpp).
template <class PadOp, class ConvOp, class BiasOp, class ReluOp, class W, class B, class X, class Hook>
[[nodiscard]] auto chain(const W& w, const B& b, const X& x, Hook&& hook) {
    const Timestamp t{};
    hook.template before<PadOp, "apply">({}, t, x);
    const auto xp = PadOp::binding::apply::call({}, x);
    hook.template before<ConvOp, "apply">({}, t, w, xp);
    const auto z = ConvOp::binding::apply::call({}, w, xp);
    hook.template before<BiasOp, "apply">({}, t, z, b);
    const auto zb = BiasOp::binding::apply::call({}, z, b);
    hook.template before<ReluOp, "apply">({}, t, zb);
    return std::pair{z, ReluOp::binding::apply::call({}, zb)};
}

/// The pattern's operators called by hand, on its constants.
template <class Hook = no_hook>
[[nodiscard]] sample hand_run(const weights& w, const Image& x, Hook&& hook = {}) {
    auto [z, y] = chain<Pad64, Conv64, BiasAdd64, Relu64>(w.w, w.b, x, hook);
    return sample{z, y};
}

namespace oracle_detail {

/// The convolution of x, by direct summation in long double, with zeros
/// outside x: element (o, i, j) at o·kSize² + i·kSize + j.
[[nodiscard]] inline std::vector<long double> convolve(const weights& w, const Image& x) {
    std::vector<long double> out(Features::stored);
    for (std::size_t o = 0; o < kKernels; ++o) {
        for (std::size_t i = 0; i < kSize; ++i) {
            for (std::size_t j = 0; j < kSize; ++j) {
                long double sum = 0;
                for (std::size_t c = 0; c < kChannels; ++c) {
                    for (std::size_t ky = 0; ky < kTaps; ++ky) {
                        for (std::size_t kx = 0; kx < kTaps; ++kx) {
                            // The input row and column this tap reads, if inside it.
                            const std::size_t r = i + ky;
                            const std::size_t q = j + kx;
                            if (r < 1 || r > kSize || q < 1 || q > kSize) {
                                continue;
                            }
                            sum += static_cast<long double>(w.w(o, c, ky, kx)) * x(c, r - 1, q - 1);
                        }
                    }
                }
                out[Features::index(o, i, j)] = sum;
            }
        }
    }
    return out;
}

/// The elements of an impulse's convolution that aren't channel c of each
/// kernel, reversed about the impulse: (o, kImpulseRow + 1 − ky,
/// kImpulseCol + 1 − kx) holds w(o, c, ky, kx), exactly, and every other
/// element is zero.
[[nodiscard]] inline std::size_t impulse_mismatches(const weights& w, const Features& conv, std::size_t c) {
    std::size_t bad = 0;
    for (std::size_t o = 0; o < kKernels; ++o) {
        for (std::size_t i = 0; i < kSize; ++i) {
            for (std::size_t j = 0; j < kSize; ++j) {
                const std::size_t ky = kImpulseRow + 1 - i;  // wraps when i > kImpulseRow + 1
                const std::size_t kx = kImpulseCol + 1 - j;
                const double want = ky < kTaps && kx < kTaps ? w.w(o, c, ky, kx) : 0.0;
                bad += conv(o, i, j) == want ? 0 : 1;
            }
        }
    }
    return bad;
}

}  // namespace oracle_detail

struct pattern {
    static constexpr std::string_view name = "conv";
    /// Three impulses, then 13 images.
    static constexpr std::size_t default_inputs = 16;
    using config = conv::config;
    using context = conv::context;
    using ports = conv::ports;
    using sample = conv::sample;
    using leaves =
        workbench::leaves<leaf<Pad64, "apply">, leaf<Conv64, "apply">, leaf<BiasAdd64, "apply">, leaf<Relu64, "apply">>;

    template <class G>
    [[nodiscard]] static ports graph(G& g, const config& c) {
        return conv::graph(g, c);
    }

    [[nodiscard]] static std::vector<source> sources(const ports& p, const config& c) {
        return {sequence(p.x, c.period_ns, [c](std::uint64_t k, Timestamp) { return input_of(c, k); })};
    }

    [[nodiscard]] static sample collect(const fl::stack& s, const ports& p, std::size_t, std::size_t, Timestamp) {
        return sample{s.output(p.conv), s.output(p.y)};
    }

    /// Sample i is the i-th input's: the pattern has one source.
    [[nodiscard]] static report oracle(const config& c, std::span<const sample> samples) {
        using namespace oracle_detail;
        const weights w = weights_of(c);
        long double conv_err = 0;
        long double y_err = 0;
        std::size_t images = 0;
        std::size_t impulses = 0;
        std::size_t mismatches = 0;
        for (std::size_t k = 0; k < samples.size(); ++k) {
            const sample& s = samples[k];
            const std::vector<long double> want = convolve(w, input_of(c, k));
            for (std::size_t e = 0; e < want.size(); ++e) {
                const long double z = want[e] + w.b(e / (kSize * kSize));
                conv_err = std::fmax(conv_err, std::fabs(s.conv.elements[e] - want[e]));
                y_err = std::fmax(y_err, std::fabs(s.y.elements[e] - (z <= 0 ? 0 : z)));
            }
            if (k < kChannels) {
                mismatches += impulse_mismatches(w, s.conv, k);
                ++impulses;
            } else {
                ++images;
            }
        }
        report r{std::string(name), {}};
        r.checks.push_back(at_least("impulses, one per channel", static_cast<double>(impulses), kChannels));
        r.checks.push_back(at_least("images after the impulses", static_cast<double>(images), 1));
        r.checks.push_back(at_most("convolution against long double", static_cast<double>(conv_err), 1e-12));
        r.checks.push_back(at_most("y against long double", static_cast<double>(y_err), 1e-12));
        r.checks.push_back(
            at_most("impulses: elements other than each kernel, reversed", static_cast<double>(mismatches), 0));
        return r;
    }

    [[nodiscard]] static fl::trace_codecs codecs() {
        fl::trace_codecs c;
        add_data<fl::domain_of_t<Image>>(c);
        add_data<fl::domain_of_t<Padded>>(c);
        add_data<fl::domain_of_t<Weights>>(c);
        add_data<fl::domain_of_t<Bias>>(c);
        add_data<fl::domain_of_t<Features>>(c);
        return c;
    }
};

}  // namespace branes::anatomy::workbench::conv
