#pragma once
/// @file attitude.hpp
/// @brief The attitude filter pattern: a unit quaternion, integrated from a
/// gyro and corrected toward gravity from an accelerometer
/// (docs/plan/computational-domain, C5).
///
/// The attitude q is a unit quaternion (w, x, y, z), Hamilton's convention,
/// that rotates the body frame into the world frame, whose z axis is up. Its
/// domain is `vector<double, dim<4>, structure::unit_norm>`: stored as a dense
/// vector, with its norm 1 (flow/data.hpp, A4). It is the filter's state.
///
/// A complementary filter of gain g, on two inputs:
///   propagate, at each gyro sample ω (400 Hz): the rotation over one period
///     h at the constant rate ω, exactly, q ← q ⊗ exp(½·ω·h), with
///     exp(½·ω·h) = (cos(|ω|·h/2), sin(|ω|·h/2)·ω/|ω|), and (1, 0, 0, 0) at
///     ω = 0; then q ← q/|q|.
///   correct, at each accelerometer sample a (50 Hz): the up direction the
///     attitude predicts, in the body frame, is v = (2(xz − wy), 2(yz + wx),
///     w² − x² − y² + z²); the target is u = (1 − g)·v + g·a/|a|, and q
///     turns so that it predicts û: q ← q ⊗ r, with r = (|u| + u·v, u × v),
///     the rotation of u onto v; then q ← q/|q|. With |a| = 0 the target is v,
///     and q only renormalizes.
///
/// So a correction leaves the tilt φ between v and the measured up â at
///   tan φ' = (1 − g)·sin φ / ((1 − g)·cos φ + g),
/// and, for a small tilt, φ' ≈ (1 − g)·φ: the gain is the fraction of the
/// tilt each correction removes.
///
/// Two filters run on their own inputs, in four regions: `spinning`, from
/// level, on a body turning at a constant rate, its accelerometer measuring
/// gravity in its own frame; and `resting`, from a tilt about x, on a body at
/// rest and level.
///
/// Oracle (W2), in long double:
///   the spinning filter's attitude after 1 s is exp(½·ω·t), to 1e-9;
///   the resting filter's tilt after each correction is its recurrence above,
///     from the configured tilt, to 1e-12, and its last correction removes the
///     fraction g of the tilt, to a relative 1e-6;
///   every attitude a firing writes has norm 1, to 1e-12.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/workbench/codecs.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::anatomy::workbench::attitude {

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

using QuatD = fl::vector<double, fl::dim<4>, fl::structure::unit_norm>;
using Quat = fl::data<QuatD>;
using Vec3D = fl::vector<double, fl::dim<3>>;
using Vec3 = fl::data<Vec3D>;

// ── Configuration ───────────────────────────────────────────────────

struct config {
    std::uint64_t seed = 1;                      ///< the pattern draws nothing: every seed runs the same motions
    std::int64_t gyro_period_ns = 2'500'000;     ///< 400 Hz
    std::int64_t accel_period_ns = 20'000'000;   ///< 50 Hz
    double gain = 0.05;                          ///< g, in (0, 1): the fraction of a small tilt a correction removes
    std::array<double, 3> rate{0.3, -0.5, 0.8};  ///< the spinning body's rate, in its frame [rad/s]
    double tilt = 0.5;                           ///< the resting filter's first tilt, about x [rad]
    double gravity = 9.80665;                    ///< what an accelerometer at rest measures [m/s²]

    [[nodiscard]] double h() const noexcept {
        return static_cast<double>(gyro_period_ns) * 1e-9;
    }
};

struct context {
    config cfg;
};

[[nodiscard]] inline Vec3 vec3(double x, double y, double z) noexcept {
    Vec3 v;
    v(0) = x;
    v(1) = y;
    v(2) = z;
    return v;
}

// ── The operator ────────────────────────────────────────────────────

/// Where a filter's attitude starts: level, or tilted about x by the configured tilt.
enum class start : std::uint8_t { level, tilted };

/// The filter: its attitude, and the constants its handlers take from the configuration.
template <start S>
struct Filter {
    Quat q;              ///< (1, 0, 0, 0) until from_context sets it
    double half_dt = 0;  ///< h/2
    double g = 0;        ///< the gain
    double keep = 0;     ///< 1 − g

    [[nodiscard]] static result<Filter, std::string> from_context(const context& c) {
        if (!(c.cfg.gyro_period_ns > 0)) {
            return std::string("attitude: the gyro's period is positive");
        }
        if (!(c.cfg.gain > 0 && c.cfg.gain < 1)) {
            return std::string("attitude: the gain is in (0, 1)");
        }
        Filter f;
        if constexpr (S == start::tilted) {
            f.q(0) = std::cos(c.cfg.tilt / 2);
            f.q(1) = std::sin(c.cfg.tilt / 2);
        }
        f.half_dt = c.cfg.h() / 2;
        f.g = c.cfg.gain;
        f.keep = 1.0 - c.cfg.gain;
        return f;
    }

    /// q ← (q ⊗ (c, d))/|q ⊗ (c, d)|; returns q.
    Quat turn(double c, double dx, double dy, double dz) {
        const double q0 = q(0);
        const double q1 = q(1);
        const double q2 = q(2);
        const double q3 = q(3);
        const double p0 = q0 * c - q1 * dx - q2 * dy - q3 * dz;
        const double p1 = q0 * dx + q1 * c + q2 * dz - q3 * dy;
        const double p2 = q0 * dy - q1 * dz + q2 * c + q3 * dx;
        const double p3 = q0 * dz + q1 * dy - q2 * dx + q3 * c;
        const double m = std::sqrt(p0 * p0 + p1 * p1 + p2 * p2 + p3 * p3);
        q(0) = p0 / m;
        q(1) = p1 / m;
        q(2) = p2 / m;
        q(3) = p3 / m;
        return q;
    }

    [[nodiscard]] Quat propagate(const Vec3& w) {
        const double wx = w(0);
        const double wy = w(1);
        const double wz = w(2);
        const double n = std::sqrt(wx * wx + wy * wy + wz * wz);
        const double half = n * half_dt;
        const double c = std::cos(half);
        const double k = n <= 0.0 ? half_dt : std::sin(half) / n;  // sin(|ω|·h/2)/|ω|, h/2 at ω = 0
        return turn(c, k * wx, k * wy, k * wz);
    }

    [[nodiscard]] Quat correct(const Vec3& a) {
        const double q0 = q(0);
        const double q1 = q(1);
        const double q2 = q(2);
        const double q3 = q(3);
        // The up direction the attitude predicts, in the body frame.
        const double vx = 2.0 * (q1 * q3 - q0 * q2);
        const double vy = 2.0 * (q2 * q3 + q0 * q1);
        const double vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        const double ax = a(0);
        const double ay = a(1);
        const double az = a(2);
        const double an = std::sqrt(ax * ax + ay * ay + az * az);
        const double k = an <= 0.0 ? 0.0 : g / an;
        // The target u = (1 − g)·v + g·â, and r, the rotation of u onto v.
        const double ux = keep * vx + k * ax;
        const double uy = keep * vy + k * ay;
        const double uz = keep * vz + k * az;
        const double un = std::sqrt(ux * ux + uy * uy + uz * uz);
        const double r0 = un + (ux * vx + uy * vy + uz * vz);
        const double r1 = uy * vz - uz * vy;
        const double r2 = uz * vx - ux * vz;
        const double r3 = ux * vy - uy * vx;
        return turn(r0, r1, r2, r3);
    }
};

namespace declare {

/// The variables of q ⊗ (c, d), normalized, written to the state q and the
/// out port: each line of Filter::turn, in its order.
constexpr void turn(cd::system& s, cd::target q, cd::val c, cd::val dx, cd::val dy, cd::val dz) {
    constexpr cd::etype e = cd::etype::f64;
    const auto p0 = s.var("p0", e);
    const auto p1 = s.var("p1", e);
    const auto p2 = s.var("p2", e);
    const auto p3 = s.var("p3", e);
    const auto m = s.var("m", e);
    s.eq(p0, q(0) * c - q(1) * dx - q(2) * dy - q(3) * dz);
    s.eq(p1, q(0) * dx + q(1) * c + q(2) * dz - q(3) * dy);
    s.eq(p2, q(0) * dy - q(1) * dz + q(2) * c + q(3) * dx);
    s.eq(p3, q(0) * dz + q(1) * dy - q(2) * dx + q(3) * c);
    s.eq(m, sqrt(p0() * p0() + p1() * p1() + p2() * p2() + p3() * p3()));
    const std::array<cd::target, 4> p{p0, p1, p2, p3};
    constexpr std::array<std::string_view, 4> names{"q0", "q1", "q2", "q3"};
    for (std::size_t i = 0; i < 4; ++i) {
        const auto qi = s.var(names[i], e);
        s.eq(qi, p[i]() / m());
        s.write(q.at(i), qi());
        s.write(s.out("attitude").at(i), qi());
    }
}

/// propagate: one point, with no index.
constexpr cd::system propagate() {
    constexpr cd::etype e = cd::etype::f64;
    cd::system s;
    const auto q = s.state("q");
    const auto w = s.in("w");
    const auto half_dt = s.param("half_dt", e);
    const auto n = s.var("n", e);
    const auto half = s.var("half", e);
    const auto c = s.var("c", e);
    const auto k = s.var("k", e);
    s.eq(n, sqrt(w(0) * w(0) + w(1) * w(1) + w(2) * w(2)));
    s.eq(half, n() * half_dt());
    s.eq(c, cos(half()));
    s.eq(k, select_le(n(), s.literal(0, e), half_dt(), sin(half()) / n()));
    // Variables, so each product is counted once, as the handler computes it once.
    const auto dx = s.var("dx", e);
    const auto dy = s.var("dy", e);
    const auto dz = s.var("dz", e);
    s.eq(dx, k() * w(0));
    s.eq(dy, k() * w(1));
    s.eq(dz, k() * w(2));
    turn(s, q, c(), dx(), dy(), dz());
    return s;
}

/// correct: one point, with no index.
constexpr cd::system correct() {
    constexpr cd::etype e = cd::etype::f64;
    cd::system s;
    const auto q = s.state("q");
    const auto a = s.in("a");
    const auto vx = s.var("vx", e);
    const auto vy = s.var("vy", e);
    const auto vz = s.var("vz", e);
    const auto an = s.var("an", e);
    const auto k = s.var("k", e);
    const auto ux = s.var("ux", e);
    const auto uy = s.var("uy", e);
    const auto uz = s.var("uz", e);
    const auto un = s.var("un", e);
    const auto r0 = s.var("r0", e);
    const auto r1 = s.var("r1", e);
    const auto r2 = s.var("r2", e);
    const auto r3 = s.var("r3", e);
    s.eq(vx, 2.0 * (q(1) * q(3) - q(0) * q(2)));
    s.eq(vy, 2.0 * (q(2) * q(3) + q(0) * q(1)));
    s.eq(vz, q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3));
    s.eq(an, sqrt(a(0) * a(0) + a(1) * a(1) + a(2) * a(2)));
    s.eq(k, select_le(an(), s.literal(0, e), s.literal(0, e), s.param("g", e)() / an()));
    const auto keep = s.param("keep", e);
    s.eq(ux, keep() * vx() + k() * a(0));
    s.eq(uy, keep() * vy() + k() * a(1));
    s.eq(uz, keep() * vz() + k() * a(2));
    s.eq(un, sqrt(ux() * ux() + uy() * uy() + uz() * uz()));
    s.eq(r0, un() + (ux() * vx() + uy() * vy() + uz() * vz()));
    s.eq(r1, uy() * vz() - uz() * vy());
    s.eq(r2, uz() * vx() - ux() * vz());
    s.eq(r3, ux() * vy() - uy() * vx());
    turn(s, q, r0(), r1(), r2(), r3());
    return s;
}

}  // namespace declare

template <start S>
struct FilterBinding {
    using signature = fl::signature<fl::ports<fl::port<"w", Vec3, fl::direction::in>,
                                              fl::port<"a", Vec3, fl::direction::in>,
                                              fl::port<"attitude", Quat, fl::direction::out>>,
                                    fl::states<fl::state<"q", QuatD>>>;
    struct propagate : fl::handler<"propagate", fl::ins<"w">, fl::outs<"attitude">, fl::state_role::update> {
        static constexpr cd::system compute = declare::propagate();
        static Quat call(Filter<S>& op, const Vec3& w) {
            return op.propagate(w);
        }
    };
    struct correct : fl::handler<"correct", fl::ins<"a">, fl::outs<"attitude">, fl::state_role::update> {
        static constexpr cd::system compute = declare::correct();
        static Quat call(Filter<S>& op, const Vec3& a) {
            return op.correct(a);
        }
    };
    using handlers = fl::handlers<propagate, correct>;
    using cd_state = cd::members<cd::member<"q", &Filter<S>::q>>;
    using cd_params = cd::members<cd::member<"half_dt", &Filter<S>::half_dt>,
                                  cd::member<"g", &Filter<S>::g>,
                                  cd::member<"keep", &Filter<S>::keep>>;
};

using Level64 = fl::registered<"Attitude_f64", Filter<start::level>, FilterBinding<start::level>>;
using Tilted64 = fl::registered<"AttitudeTilted_f64", Filter<start::tilted>, FilterBinding<start::tilted>>;

// ── The graph ───────────────────────────────────────────────────────

/// The filters, and each one's outputs: after a propagation, after a correction.
inline constexpr std::size_t kFilters = 2;
inline constexpr std::array<std::string_view, kFilters> kFilterNames{"spinning", "resting"};

/// The sources, in the order the driver breaks ties by: at a shared time, both
/// gyros, then both accelerometers, so a correction sees the propagation to its time.
enum src : std::size_t { gyro_spinning, gyro_resting, accel_spinning, accel_resting, kSources };

struct ports {
    std::array<fl::input_port<Vec3>, kFilters> gyro;
    std::array<fl::input_port<Vec3>, kFilters> accel;
    std::array<fl::output_port<Quat>, kFilters> propagated;
    std::array<fl::output_port<Quat>, kFilters> corrected;
};

namespace detail {
/// One filter: a region on its gyro that propagates, and one on its accelerometer that corrects.
template <class G, class B>
void filter(G& g, const fl::node<B>& f, ports& p, std::size_t i) {
    fl::scope rg = g.region(p.gyro[i]);
    auto [qg] = fl::fire<"propagate">(rg, f, p.gyro[i]);
    g.produce(p.propagated[i], qg);
    fl::scope ra = g.region(p.accel[i]);
    auto [qa] = fl::fire<"correct">(ra, f, p.accel[i]);
    g.produce(p.corrected[i], qa);
}
}  // namespace detail

template <class G>
[[nodiscard]] ports graph(G& g, const config&) {
    ports p;
    for (std::size_t i = 0; i < kFilters; ++i) {
        const std::string n(kFilterNames[i]);
        p.gyro[i] = fl::input<Vec3>(g, "gyro_" + n);
        p.accel[i] = fl::input<Vec3>(g, "accel_" + n);
    }
    for (std::size_t i = 0; i < kFilters; ++i) {
        const std::string n(kFilterNames[i]);
        p.propagated[i] = fl::output<Quat>(g, n + "_propagated", p.gyro[i]);
        p.corrected[i] = fl::output<Quat>(g, n + "_corrected", p.accel[i]);
    }
    const auto spinning = fl::instance<Level64>(g, "spinning");
    const auto resting = fl::instance<Tilted64>(g, "resting");
    detail::filter(g, spinning, p, 0);
    detail::filter(g, resting, p, 1);
    return p;
}

// ── The motions, the samples, the run by hand (W1) and the oracle (W2) ──

/// The spinning body's attitude at time t [s]: exp(½·ω·t), from level.
[[nodiscard]] inline std::array<long double, 4> spin_truth(const config& c, long double t) {
    const long double wx = c.rate[0];
    const long double wy = c.rate[1];
    const long double wz = c.rate[2];
    const long double n = std::sqrt(wx * wx + wy * wy + wz * wz);
    const long double half = n * t / 2;
    const long double k = n > 0 ? std::sin(half) / n : 0;
    return {std::cos(half), k * wx, k * wy, k * wz};
}

/// The k-th message of source `s`, at time t: the gyros measure the bodies'
/// rates, and the accelerometers gravity, in each body's frame.
[[nodiscard]] inline Vec3 measured(const config& c, std::size_t s, Timestamp t) {
    switch (s) {
    case gyro_spinning:
        return vec3(c.rate[0], c.rate[1], c.rate[2]);
    case accel_spinning: {
        // Up in the body frame: the third row of the attitude's rotation.
        const auto [w, x, y, z] = spin_truth(c, static_cast<long double>(t.ns) * 1e-9L);
        const long double g = c.gravity;
        return vec3(static_cast<double>(g * 2 * (x * z - w * y)),
                    static_cast<double>(g * 2 * (y * z + w * x)),
                    static_cast<double>(g * (w * w - x * x - y * y + z * z)));
    }
    case accel_resting:
        return vec3(0.0, 0.0, c.gravity);
    default:
        return vec3(0.0, 0.0, 0.0);
    }
}

/// Each source's period and first time: a gyro sample integrates the period
/// before it, so the first is at one period.
[[nodiscard]] inline std::int64_t period_of(const config& c, std::size_t s) noexcept {
    return s == gyro_spinning || s == gyro_resting ? c.gyro_period_ns : c.accel_period_ns;
}

/// After each input: its source, its time, and every output.
struct sample {
    std::size_t source = 0;
    Timestamp t;
    std::array<Quat, kFilters> propagated;
    std::array<Quat, kFilters> corrected;

    friend bool operator==(const sample&, const sample&) = default;
};

/// The operators called by hand on the same stream, merged as the driver
/// merges the sources.
template <class Hook = no_hook>
[[nodiscard]] std::vector<sample> hand_run(const config& c, std::size_t inputs, Hook&& hook = {}) {
    const context ctx{c};
    auto spinning = *Filter<start::level>::from_context(ctx);
    auto resting = *Filter<start::tilted>::from_context(ctx);
    std::array<std::uint64_t, kSources> k{};
    std::vector<sample> out;
    out.reserve(inputs);
    sample s;
    for (std::size_t i = 0; i < inputs; ++i) {
        std::size_t next = kSources;
        std::int64_t at = 0;
        for (std::size_t j = 0; j < kSources; ++j) {
            const std::int64_t t = period_of(c, j) * static_cast<std::int64_t>(k[j] + 1);
            if (next == kSources || t < at) {
                next = j;
                at = t;
            }
        }
        ++k[next];
        const Timestamp t{at};
        const Vec3 m = measured(c, next, t);
        const auto one = [&]<class Op>(typename Op::type& f, std::size_t filter) {
            using B = typename Op::binding;
            if (next < accel_spinning) {
                hook.template before<Op, "propagate">(f, t, m);
                s.propagated[filter] = B::propagate::call(f, m);
            } else {
                hook.template before<Op, "correct">(f, t, m);
                s.corrected[filter] = B::correct::call(f, m);
            }
        };
        if (next % kFilters == 0) {
            one.template operator()<Level64>(spinning, 0);
        } else {
            one.template operator()<Tilted64>(resting, 1);
        }
        s.source = next;
        s.t = t;
        out.push_back(s);
    }
    return out;
}

namespace oracle_detail {

/// The tilt of attitude q: the angle between the up it predicts and the world's.
[[nodiscard]] inline long double tilt_of(const Quat& q) {
    const long double w = q(0);
    const long double x = q(1);
    const long double y = q(2);
    const long double z = q(3);
    const long double vx = 2 * (x * z - w * y);
    const long double vy = 2 * (y * z + w * x);
    const long double vz = w * w - x * x - y * y + z * z;
    return std::atan2(std::sqrt(vx * vx + vy * vy), vz);
}

/// The tilt after a correction of gain g from tilt φ.
[[nodiscard]] inline long double corrected_tilt(long double phi, long double g) {
    return std::atan2((1 - g) * std::sin(phi), (1 - g) * std::cos(phi) + g);
}

[[nodiscard]] inline long double norm_error(const Quat& q) {
    long double sq = 0;
    for (const double e : q.elements) {
        sq += static_cast<long double>(e) * e;
    }
    return std::fabs(std::sqrt(sq) - 1);
}

}  // namespace oracle_detail

struct pattern {
    static constexpr std::string_view name = "attitude";
    /// 4 s: 1600 samples of each gyro and 200 of each accelerometer.
    static constexpr std::size_t default_inputs = 3600;
    using config = attitude::config;
    using context = attitude::context;
    using ports = attitude::ports;
    using sample = attitude::sample;
    using leaves = workbench::leaves<leaf<Level64, "propagate">,
                                     leaf<Level64, "correct">,
                                     leaf<Tilted64, "propagate">,
                                     leaf<Tilted64, "correct">>;

    template <class G>
    [[nodiscard]] static ports graph(G& g, const config& c) {
        return attitude::graph(g, c);
    }

    [[nodiscard]] static std::vector<source> sources(const ports& p, const config& c) {
        std::vector<source> out;
        for (std::size_t s = 0; s < kSources; ++s) {
            const auto& in = s < accel_spinning ? p.gyro[s % kFilters] : p.accel[s % kFilters];
            const std::int64_t period = period_of(c, s);
            out.push_back(
                sequence(in, period, [c, s](std::uint64_t, Timestamp t) { return measured(c, s, t); }, period));
        }
        return out;
    }

    [[nodiscard]] static sample
    collect(const fl::stack& st, const ports& p, std::size_t, std::size_t source, Timestamp t) {
        sample out{source, t, {}, {}};
        for (std::size_t i = 0; i < kFilters; ++i) {
            out.propagated[i] = st.output(p.propagated[i]);
            out.corrected[i] = st.output(p.corrected[i]);
        }
        return out;
    }

    [[nodiscard]] static report oracle(const config& c, std::span<const sample> samples) {
        using namespace oracle_detail;
        report rep{std::string(name), {}};
        const long double g = c.gain;
        long double spin_err = 1;  // until a sample at 1 s
        long double tilt_err = 0;
        long double norm_err = 0;
        long double phi = c.tilt;                 // the resting filter's tilt, as its recurrence predicts
        std::array<long double, 2> last{-1, -1};  // its last two measured tilts
        std::size_t corrections = 0;
        for (const sample& s : samples) {
            const std::size_t f = s.source % kFilters;
            const Quat& q = s.source < accel_spinning ? s.propagated[f] : s.corrected[f];
            norm_err = std::fmax(norm_err, norm_error(q));
            if (s.source == accel_spinning && s.t.ns == 1'000'000'000) {
                const auto want = spin_truth(c, 1.0L);
                spin_err = 0;
                for (std::size_t i = 0; i < 4; ++i) {
                    spin_err = std::fmax(spin_err, std::fabs(static_cast<long double>(q(i)) - want[i]));
                }
            }
            if (s.source == accel_resting) {
                phi = corrected_tilt(phi, g);
                const long double measured_tilt = tilt_of(q);
                tilt_err = std::fmax(tilt_err, std::fabs(measured_tilt - phi));
                last = {last[1], measured_tilt};
                ++corrections;
            }
        }
        rep.checks.push_back(at_least("corrections of the resting filter", static_cast<double>(corrections), 2));
        rep.checks.push_back(
            at_most("spinning: attitude against exp(w t / 2) at 1 s", static_cast<double>(spin_err), 1e-9));
        rep.checks.push_back(
            at_most("resting: tilt against its recurrence", static_cast<double>(corrections ? tilt_err : 1), 1e-12));
        const long double ratio = last[0] > 0 ? last[1] / last[0] : 0;
        rep.checks.push_back(at_most("resting: last correction's decay against 1 - g, relative",
                                     static_cast<double>(std::fabs(ratio - (1 - g)) / (1 - g)),
                                     1e-6));
        rep.checks.push_back(at_most("norm of every attitude written", static_cast<double>(norm_err), 1e-12));
        return rep;
    }

    [[nodiscard]] static fl::trace_codecs codecs() {
        fl::trace_codecs c;
        add_data<Vec3D>(c);
        add_data<QuatD>(c);
        return c;
    }
};

}  // namespace branes::anatomy::workbench::attitude
