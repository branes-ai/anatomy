#pragma once
/// @file dsp.hpp
/// @brief The DSP spectral pipeline pattern: a band-pass FIR, frames by
/// `accumulate`, a Hann window, an FFT and a magnitude (docs/plan/workbench,
/// "The patterns", B5).
///
/// Samples at 8192 Hz, frames of N = 256 (bins 32 Hz apart). Two regions:
///   the sample region  fired by each sample x: Fir, a band-pass FIR of 63 taps
///                      whose delay line is its state, and whose taps are a
///                      constant, a Hamming-windowed sinc designed from the
///                      configuration when the graph is built
///   the frame region   fired by a frame tick at every 256th sample's time,
///                      after the sample (the driver's declared order): Framer
///                      reads accumulate<256> of the filtered samples, then
///                      Hann (its window a constant), ToComplex, Fft (radix 2)
///                      and Magnitude
/// Outputs: the filtered sample, and per frame the filtered frame, the values
/// its window dropped, the spectrum (256 complex bins) and the magnitude of
/// bins 0 to 128.
///
/// The source is a sum of tones at bin centers, two in the pass band and one
/// in the stop band, plus seeded noise. 1/8192 s isn't a whole number of
/// nanoseconds, so the sample period is 122 070 ns; the operators count
/// samples, not time.
///
/// Oracle (W2), in long double and without the operators:
///   every frame's window holds 256 samples and dropped none;
///   the filtered frame agrees with the FIR, evaluated from the samples and
///     the taps, to a relative 1e-12;
///   the spectrum and the magnitude agree with a direct DFT of the windowed
///     frame to a relative 1e-12, and Parseval holds between them to 1e-12;
///   from the second frame (the first holds the filter's transient), the two
///     largest magnitudes are at the pass-band tones' bins, and each tone's
///     magnitude is within 1 % of A·|H(f)|·Σw/2, H evaluated from the taps;
///   the design puts the stop-band tone at least 40 dB down.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/flow/window.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/workbench/codecs.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace branes::anatomy::workbench::dsp {

namespace fl = branes::anatomy::flow;

inline constexpr std::size_t kFrame = 256;
inline constexpr std::size_t kTaps = 63;
inline constexpr std::size_t kBins = kFrame / 2 + 1;

using SampleD = fl::scalar<double>;
using TapsD = fl::vector<double, fl::dim<kTaps>>;
using FrameD = fl::vector<double, fl::bounded<kFrame>>;
using DroppedD = fl::scalar<std::uint64_t>;
using RealD = fl::vector<double, fl::dim<kFrame>>;
using SpectrumD = fl::vector<std::complex<double>, fl::dim<kFrame>>;
using MagnitudeD = fl::vector<double, fl::dim<kBins>>;

using Sample = fl::data<SampleD>;
using Taps = fl::data<TapsD>;
using Frame = fl::data<FrameD>;
using Dropped = fl::data<DroppedD>;
using Real = fl::data<RealD>;
using Spectrum = fl::data<SpectrumD>;
using Magnitude = fl::data<MagnitudeD>;
using Window = fl::window<Sample, kFrame>;

// ── Configuration ───────────────────────────────────────────────────

/// A tone at a bin's center frequency, bin·fs/N.
struct tone {
    std::size_t bin = 0;
    double amplitude = 0;
};

struct config {
    std::uint64_t seed = 1;
    double sample_rate_hz = 8192;
    double low_hz = 1024;  ///< the pass band's edges, where the FIR is 6 dB down
    double high_hz = 2048;
    std::array<tone, 2> pass{{{40, 1.0}, {52, 0.5}}};  ///< 1280 and 1664 Hz
    tone stop{100, 1.0};                               ///< 3200 Hz
    double noise = 1e-3;                               ///< the noise's standard deviation

    /// The sample period, to the nanosecond below.
    [[nodiscard]] std::int64_t period_ns() const noexcept {
        return static_cast<std::int64_t>(1e9 / sample_rate_hz);
    }
};

struct context {
    config cfg;
};

/// The band-pass taps: the difference of two low-pass windowed sincs, with a
/// Hamming window, centered on tap 31.
[[nodiscard]] inline Taps taps_of(const config& c) {
    const double f1 = c.low_hz / c.sample_rate_hz;  // cycles per sample
    const double f2 = c.high_hz / c.sample_rate_hz;
    const auto sinc = [](double x) { return x == 0.0 ? 1.0 : std::sin(std::numbers::pi * x) / (std::numbers::pi * x); };
    constexpr double mid = static_cast<double>(kTaps - 1) / 2;
    Taps h;
    for (std::size_t n = 0; n < kTaps; ++n) {
        const double m = static_cast<double>(n) - mid;
        const double w = 0.54 - 0.46 * std::cos(2.0 * std::numbers::pi * static_cast<double>(n) / (kTaps - 1));
        h(n) = w * (2.0 * f2 * sinc(2.0 * f2 * m) - 2.0 * f1 * sinc(2.0 * f1 * m));
    }
    return h;
}

/// The periodic Hann window: a tone at a bin's center leaks into its two neighbours only.
[[nodiscard]] inline Real hann_of() {
    Real w;
    for (std::size_t n = 0; n < kFrame; ++n) {
        w(n) = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(n) / kFrame);
    }
    return w;
}

/// The k-th sample: the tones, with phases drawn from the seed, and noise.
[[nodiscard]] inline Sample sample_of(const config& c, std::uint64_t k) {
    rng phases(substream(c.seed, 0));
    double v = 0;
    const auto add = [&](const tone& t) {
        // bin·k mod N, exactly, so the phase doesn't lose precision as k grows.
        const auto m = static_cast<double>((t.bin * k) % kFrame);
        v += t.amplitude * std::sin(2.0 * std::numbers::pi * m / kFrame + phases.uniform(0.0, 2.0 * std::numbers::pi));
    };
    add(c.pass[0]);
    add(c.pass[1]);
    add(c.stop);
    rng noise(substream(c.seed, k + 1));
    Sample s;
    s() = v + c.noise * noise.normal();
    return s;
}

// ── The operators ───────────────────────────────────────────────────

/// A FIR of L taps: its delay line is its state, newest first.
template <class E, std::size_t L>
struct Fir {
    fl::data<fl::vector<E, fl::dim<L>>> delay;

    /// y = Σ h(i)·x[n − i], summed in tap order.
    [[nodiscard]] E filter(const fl::data<fl::vector<E, fl::dim<L>>>& h, E x) noexcept {
        for (std::size_t i = L - 1; i > 0; --i) {
            delay(i) = delay(i - 1);
        }
        delay(0) = x;
        E y{};
        for (std::size_t i = 0; i < L; ++i) {
            y += h(i) * delay(i);
        }
        return y;
    }
};

struct FirBinding {
    using signature = fl::signature<fl::ports<fl::port<"x", Sample, fl::direction::in>,
                                              fl::port<"taps", Taps, fl::direction::in>,
                                              fl::port<"y", Sample, fl::direction::out>>,
                                    fl::states<fl::state<"delay", TapsD>>>;
    struct filter : fl::handler<"filter", fl::ins<"x", "taps">, fl::outs<"y">, fl::state_role::update> {
        static Sample call(Fir<double, kTaps>& op, const Sample& x, const Taps& h) {
            Sample y;
            y() = op.filter(h, x());
            return y;
        }
    };
    using handlers = fl::handlers<filter>;
};

/// A window of samples as a frame, its count kept, and the number it dropped.
struct Framer {};

struct FramerBinding {
    using signature = fl::signature<fl::ports<fl::port<"w", Window, fl::direction::in>,
                                              fl::port<"frame", Frame, fl::direction::out>,
                                              fl::port<"dropped", Dropped, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"w">, fl::outs<"frame", "dropped">> {
        static std::tuple<Frame, Dropped> call(const Framer&, const Window& w) {
            Frame f;
            (void)f.resize(w.count);
            for (std::size_t k = 0; k < w.count; ++k) {
                f(k) = w.items[k]();
            }
            Dropped d;
            d() = w.dropped;
            return {f, d};
        }
    };
    using handlers = fl::handlers<apply>;
};

/// The frame times the window; a short frame is zero past its count.
struct Hann {};

struct HannBinding {
    using signature = fl::signature<fl::ports<fl::port<"x", Frame, fl::direction::in>,
                                              fl::port<"w", Real, fl::direction::in>,
                                              fl::port<"y", Real, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x", "w">, fl::outs<"y">> {
        static Real call(const Hann&, const Frame& x, const Real& w) {
            Real y;
            for (std::size_t n = 0; n < x.count(); ++n) {
                y(n) = x(n) * w(n);
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

struct ToComplex {};

struct ToComplexBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"x", Real, fl::direction::in>, fl::port<"y", Spectrum, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static Spectrum call(const ToComplex&, const Real& x) {
            Spectrum y;
            for (std::size_t n = 0; n < kFrame; ++n) {
                y(n) = std::complex<double>(x(n), 0.0);
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

/// A radix-2 FFT of N points, decimation in time: X_k = Σ x_n·e^(−2πi·kn/N).
/// Its twiddle factors are computed once, from_context.
template <class E, std::size_t N>
struct Fft {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "Fft: N is a power of two");
    std::array<std::complex<E>, N / 2> twiddle{};

    [[nodiscard]] static result<Fft, std::string> from_context(const context&) {
        Fft f;
        for (std::size_t k = 0; k < N / 2; ++k) {
            const E a = static_cast<E>(-2.0 * std::numbers::pi * static_cast<double>(k) / static_cast<double>(N));
            f.twiddle[k] = std::complex<E>(std::cos(a), std::sin(a));
        }
        return f;
    }

    template <class X>
    void transform(const X& x, X& y) const noexcept {
        // Bit-reversed order, then log2(N) passes of butterflies.
        std::size_t bits = 0;
        while ((std::size_t{1} << bits) < N) {
            ++bits;
        }
        for (std::size_t n = 0; n < N; ++n) {
            std::size_t r = 0;
            for (std::size_t b = 0; b < bits; ++b) {
                r |= ((n >> b) & 1U) << (bits - 1 - b);
            }
            y(r) = x(n);
        }
        for (std::size_t len = 2; len <= N; len <<= 1) {
            const std::size_t half = len / 2;
            const std::size_t step = N / len;
            for (std::size_t start = 0; start < N; start += len) {
                for (std::size_t j = 0; j < half; ++j) {
                    const std::complex<E> t = twiddle[j * step] * y(start + j + half);
                    const std::complex<E> u = y(start + j);
                    y(start + j) = u + t;
                    y(start + j + half) = u - t;
                }
            }
        }
    }
};

struct FftBinding {
    using signature = fl::signature<
        fl::ports<fl::port<"x", Spectrum, fl::direction::in>, fl::port<"y", Spectrum, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static Spectrum call(const Fft<double, kFrame>& op, const Spectrum& x) {
            Spectrum y;
            op.transform(x, y);
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

/// |X_k| for k = 0 to N/2: a real frame's spectrum is symmetric about N/2.
struct MagnitudeOp {};

struct MagnitudeBinding {
    using signature = fl::signature<
        fl::ports<fl::port<"x", Spectrum, fl::direction::in>, fl::port<"y", Magnitude, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static Magnitude call(const MagnitudeOp&, const Spectrum& x) {
            Magnitude y;
            for (std::size_t k = 0; k < kBins; ++k) {
                y(k) = std::abs(x(k));
            }
            return y;
        }
    };
    using handlers = fl::handlers<apply>;
};

using Fir64 = fl::registered<"Fir_f64_63", Fir<double, kTaps>, FirBinding>;
using Framer64 = fl::registered<"Framer_f64_256", Framer, FramerBinding>;
using Hann64 = fl::registered<"Hann_f64_256", Hann, HannBinding>;
using ToComplex64 = fl::registered<"Cast_f64_c128_256", ToComplex, ToComplexBinding>;
using Fft128 = fl::registered<"Fft_c128_256", Fft<double, kFrame>, FftBinding>;
using Magnitude128 = fl::registered<"Magnitude_c128_129", MagnitudeOp, MagnitudeBinding>;

// ── The graph ───────────────────────────────────────────────────────

struct ports {
    fl::input_port<Sample> x;
    fl::input_port<Timestamp> frame;
    fl::output_port<Sample> filtered;
    fl::output_port<Frame> frame_out;  ///< the filtered frame
    fl::output_port<Dropped> dropped;
    fl::output_port<Spectrum> spectrum;
    fl::output_port<Magnitude> magnitude;
};

template <class G>
[[nodiscard]] ports graph(G& g, const config& c) {
    const auto x = fl::input<Sample>(g, "x");
    const auto tick = g.tick("frame");
    const auto taps = fl::constant<Taps>(g, "taps", taps_of(c));
    const auto hann = fl::constant<Real>(g, "hann", hann_of());
    ports p{x,
            tick,
            fl::output<Sample>(g, "filtered", x),
            fl::output<Frame>(g, "frame_out", tick),
            fl::output<Dropped>(g, "dropped", tick),
            fl::output<Spectrum>(g, "spectrum", tick),
            fl::output<Magnitude>(g, "magnitude", tick)};
    const auto fir = fl::instance<Fir64>(g, "fir");
    const auto framer = fl::instance<Framer64>(g, "framer");
    const auto window = fl::instance<Hann64>(g, "window");
    const auto to_complex = fl::instance<ToComplex64>(g, "to_complex");
    const auto fft = fl::instance<Fft128>(g, "fft");
    const auto magnitude = fl::instance<Magnitude128>(g, "magnitude_op");

    fl::scope rs = g.region(x);
    auto [y] = fl::fire<"filter">(rs, fir, x, taps);
    g.produce(p.filtered, y);

    fl::scope rf = g.region(tick);
    auto [f, d] = fl::fire<"apply">(rf, framer, fl::accumulate<kFrame>(y));
    auto [wf] = fl::fire<"apply">(rf, window, f, hann);
    auto [z] = fl::fire<"apply">(rf, to_complex, wf);
    auto [spectrum] = fl::fire<"apply">(rf, fft, z);
    auto [mag] = fl::fire<"apply">(rf, magnitude, spectrum);
    g.produce(p.frame_out, f);
    g.produce(p.dropped, d);
    g.produce(p.spectrum, spectrum);
    g.produce(p.magnitude, mag);
    return p;
}

// ── Samples, the run by hand (W1), and the oracle (W2) ──────────────

/// A frame tick's outputs.
struct frame_outputs {
    Frame frame;
    Dropped dropped;
    Spectrum spectrum;
    Magnitude magnitude;

    friend bool operator==(const frame_outputs&, const frame_outputs&) = default;
};

/// After each input: which source it came from (0 a sample, 1 a frame tick),
/// its time, the latest filtered sample, and a frame tick's outputs (none
/// after a sample, so a run's samples stay small).
struct sample {
    std::size_t source = 0;
    Timestamp t;
    Sample filtered;
    std::vector<frame_outputs> frame;

    friend bool operator==(const sample&, const sample&) = default;
};

/// The frame tick's first time: the 256th sample's, after it.
[[nodiscard]] inline std::int64_t frame_start_ns(const config& c) noexcept {
    return static_cast<std::int64_t>(kFrame - 1) * c.period_ns();
}

/// The operators called by hand on the same stream, in the graph's order:
/// the samples and the frame ticks merged as the driver merges them, and the
/// window kept as an accumulator keeps it.
[[nodiscard]] inline std::vector<sample> hand_run(const config& c, std::size_t inputs) {
    const context ctx{c};
    Fir<double, kTaps> fir;
    const Fft<double, kFrame> fft = *Fft<double, kFrame>::from_context(ctx);
    const Taps taps = taps_of(c);
    const Real hann = hann_of();
    Window w;
    std::vector<sample> out;
    sample s;
    std::uint64_t k = 0;
    std::uint64_t frames = 0;
    const std::int64_t period = c.period_ns();
    for (std::size_t i = 0; i < inputs; ++i) {
        const std::int64_t ts = static_cast<std::int64_t>(k) * period;
        const std::int64_t tf = frame_start_ns(c) + static_cast<std::int64_t>(frames * kFrame) * period;
        s.frame.clear();
        if (ts <= tf) {  // the sample first, at a shared time
            s.filtered = FirBinding::filter::call(fir, sample_of(c, k), taps);
            if (w.count < kFrame) {
                w.items[w.count++] = s.filtered;
            } else {  // the latest N, oldest first
                for (std::size_t j = 1; j < kFrame; ++j) {
                    w.items[j - 1] = w.items[j];
                }
                w.items[kFrame - 1] = s.filtered;
                ++w.dropped;
            }
            s.source = 0;
            s.t = Timestamp{ts};
            ++k;
        } else {
            frame_outputs o;
            std::tie(o.frame, o.dropped) = FramerBinding::apply::call({}, w);
            w = Window{};
            const Real wf = HannBinding::apply::call({}, o.frame, hann);
            const Spectrum z = ToComplexBinding::apply::call({}, wf);
            o.spectrum = FftBinding::apply::call(fft, z);
            o.magnitude = MagnitudeBinding::apply::call({}, o.spectrum);
            s.frame.push_back(o);
            s.source = 1;
            s.t = Timestamp{tf};
            ++frames;
        }
        out.push_back(s);
    }
    return out;
}

namespace oracle_detail {

using cld = std::complex<long double>;

/// H(f) at a bin's center, from the taps.
[[nodiscard]] inline long double response(const Taps& h, std::size_t bin) {
    cld sum{};
    for (std::size_t n = 0; n < kTaps; ++n) {
        const long double a = -2.0L * std::numbers::pi_v<long double> * static_cast<long double>((bin * n) % kFrame) /
                              static_cast<long double>(kFrame);
        sum += static_cast<long double>(h(n)) * cld(std::cos(a), std::sin(a));
    }
    return std::abs(sum);
}

}  // namespace oracle_detail

struct pattern {
    static constexpr std::string_view name = "dsp";
    /// 32 frames: each 256 samples and its tick.
    static constexpr std::size_t default_inputs = 32 * (kFrame + 1);
    using config = dsp::config;
    using context = dsp::context;
    using ports = dsp::ports;
    using sample = dsp::sample;

    template <class G>
    [[nodiscard]] static ports graph(G& g, const config& c) {
        return dsp::graph(g, c);
    }

    /// The samples, then the frame ticks: at a shared time, the sample goes first.
    [[nodiscard]] static std::vector<source> sources(const ports& p, const config& c) {
        return {sequence(p.x, c.period_ns(), [c](std::uint64_t k, Timestamp) { return sample_of(c, k); }),
                clock(p.frame, static_cast<std::int64_t>(kFrame) * c.period_ns(), frame_start_ns(c))};
    }

    [[nodiscard]] static sample
    collect(const fl::stack& s, const ports& p, std::size_t, std::size_t source, Timestamp t) {
        sample out{source, t, s.output(p.filtered), {}};
        if (source == 1) {
            out.frame.push_back(
                frame_outputs{s.output(p.frame_out), s.output(p.dropped), s.output(p.spectrum), s.output(p.magnitude)});
        }
        return out;
    }

    [[nodiscard]] static report oracle(const config& c, std::span<const sample> samples) {
        using namespace oracle_detail;
        report rep{std::string(name), {}};
        const Taps h = taps_of(c);
        const Real hann = hann_of();
        long double sum_w = 0;
        for (const double v : hann.elements) {
            sum_w += v;
        }
        // The DFT's twiddles, e^(−2πi·m/N), in long double.
        std::array<cld, kFrame> tw{};
        for (std::size_t m = 0; m < kFrame; ++m) {
            const long double a = -2.0L * std::numbers::pi_v<long double> * static_cast<long double>(m) /
                                  static_cast<long double>(kFrame);
            tw[m] = cld(std::cos(a), std::sin(a));
        }
        const std::array<long double, 3> gain{
            response(h, c.pass[0].bin), response(h, c.pass[1].bin), response(h, c.stop.bin)};
        const std::array<tone, 3> tones{c.pass[0], c.pass[1], c.stop};

        std::vector<double> x;  // every sample so far
        std::size_t frames = 0;
        std::size_t short_or_long = 0;
        std::uint64_t dropped = 0;
        long double fir_err = 0;
        long double dft_err = 0;
        long double mag_err = 0;
        long double parseval = 0;
        std::size_t misplaced = 0;
        std::array<long double, 3> tone_err{};
        for (const sample& s : samples) {
            if (s.source == 0) {
                x.push_back(sample_of(c, x.size())());
                continue;
            }
            if (s.frame.empty()) {
                continue;
            }
            const frame_outputs& o = s.frame.front();
            ++frames;
            short_or_long += o.frame.count() == kFrame ? 0 : 1;
            dropped += o.dropped();
            if (o.frame.count() != kFrame || x.size() < kFrame) {
                continue;
            }
            // The filtered frame, from the samples and the taps.
            const std::size_t first = x.size() - kFrame;
            std::array<long double, kFrame> y{};
            for (std::size_t n = 0; n < kFrame; ++n) {
                const std::size_t at = first + n;
                long double v = 0;
                for (std::size_t i = 0; i < kTaps && i <= at; ++i) {
                    v += static_cast<long double>(h(i)) * static_cast<long double>(x[at - i]);
                }
                y[n] = v;
            }
            fir_err = std::fmax(fir_err, relative_max(o.frame.elements, y, kFrame));
            // A direct DFT of the windowed frame, as the pipeline windowed it.
            std::array<long double, kFrame> wx{};
            long double energy = 0;
            for (std::size_t n = 0; n < kFrame; ++n) {
                wx[n] = static_cast<long double>(o.frame(n)) * static_cast<long double>(hann(n));
                energy += wx[n] * wx[n];
            }
            long double diff = 0;
            long double norm = 0;
            long double mdiff = 0;
            long double spectral = 0;
            for (std::size_t k = 0; k < kFrame; ++k) {
                cld d{};
                for (std::size_t n = 0; n < kFrame; ++n) {
                    d += wx[n] * tw[(k * n) % kFrame];
                }
                const cld got(o.spectrum(k).real(), o.spectrum(k).imag());
                diff = std::fmax(diff, std::abs(got - d));
                norm = std::fmax(norm, std::abs(d));
                spectral += std::norm(got);
                if (k < kBins) {
                    mdiff = std::fmax(mdiff, std::fabs(static_cast<long double>(o.magnitude(k)) - std::abs(d)));
                }
            }
            dft_err = std::fmax(dft_err, norm > 0 ? diff / norm : diff);
            mag_err = std::fmax(mag_err, norm > 0 ? mdiff / norm : mdiff);
            const long double want = static_cast<long double>(kFrame) * energy;
            parseval = std::fmax(parseval, want > 0 ? std::fabs(spectral - want) / want : std::fabs(spectral));
            if (frames == 1) {
                continue;  // the filter's transient
            }
            // The two largest magnitudes, and each tone's against its prediction.
            std::size_t top = 0;
            std::size_t second = 1;
            if (o.magnitude(second) > o.magnitude(top)) {
                std::swap(top, second);
            }
            for (std::size_t k = 2; k < kBins; ++k) {
                if (o.magnitude(k) > o.magnitude(top)) {
                    second = top;
                    top = k;
                } else if (o.magnitude(k) > o.magnitude(second)) {
                    second = k;
                }
            }
            const bool placed =
                (top == c.pass[0].bin && second == c.pass[1].bin) || (top == c.pass[1].bin && second == c.pass[0].bin);
            misplaced += placed ? 0 : 1;
            for (std::size_t t = 0; t < 3; ++t) {
                const long double predicted = static_cast<long double>(tones[t].amplitude) * gain[t] * sum_w / 2;
                const long double got = o.magnitude(tones[t].bin);
                tone_err[t] = std::fmax(tone_err[t], std::fabs(got - predicted) / predicted);
            }
        }
        rep.checks.push_back(at_least("frames", static_cast<double>(frames), 2));
        rep.checks.push_back(at_most("frames whose window isn't 256 samples", static_cast<double>(short_or_long), 0));
        rep.checks.push_back(at_most("samples dropped", static_cast<double>(dropped), 0));
        rep.checks.push_back(at_most("filtered frame against the FIR, relative", static_cast<double>(fir_err), 1e-12));
        rep.checks.push_back(at_most("spectrum against a DFT, relative", static_cast<double>(dft_err), 1e-12));
        rep.checks.push_back(at_most("magnitude against the DFT's, relative", static_cast<double>(mag_err), 1e-12));
        rep.checks.push_back(at_most("Parseval, relative", static_cast<double>(parseval), 1e-12));
        rep.checks.push_back(
            at_most("frames whose two largest bins aren't the tones'", static_cast<double>(misplaced), 0));
        rep.checks.push_back(
            at_most("pass-band tone 1 against its prediction, relative", static_cast<double>(tone_err[0]), 0.01));
        rep.checks.push_back(
            at_most("pass-band tone 2 against its prediction, relative", static_cast<double>(tone_err[1]), 0.01));
        rep.checks.push_back(
            at_most("stop-band tone against its prediction, relative", static_cast<double>(tone_err[2]), 0.01));
        rep.checks.push_back(
            at_least("stop-band tone's attenuation [dB]", -20.0 * std::log10(static_cast<double>(gain[2])), 40.0));
        return rep;
    }

    [[nodiscard]] static fl::trace_codecs codecs() {
        fl::trace_codecs c;
        add_data<SampleD>(c);
        add_data<FrameD>(c);
        add_data<DroppedD>(c);
        add_data<RealD>(c);
        add_data<SpectrumD>(c);
        add_data<MagnitudeD>(c);
        return c;
    }
};

}  // namespace branes::anatomy::workbench::dsp
