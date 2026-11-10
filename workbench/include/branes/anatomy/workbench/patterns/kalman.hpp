#pragma once
/// @file kalman.hpp
/// @brief The Kalman filter pattern: constant velocity in the plane
/// (docs/plan/workbench, "The patterns", B4).
///
/// The state is x = (px, py, vx, vy), and its covariance P a symmetric 4 × 4
/// data. Kalman is one operator with four handlers:
///   predict    a time handler, fired by a 100 Hz clock: x = F·x, P = F·P·Fᵀ + Q
///   update     fired by `fixes`, up to 4 position fixes with a count, at 10 Hz
///              on clock ticks, sometimes none: one update per fix, in Joseph form
///   prior      reads x and P, in the clock region after predict
///   posterior  reads x and P, in the fixes region after update
/// Two of them update its state, in two regions fired by two triggers at
/// shared times; the clock goes first (the driver's declared order), so an
/// update applies to the predicted state. The signature declares x, P and
/// t_last, the time of the last predict, as its state domains. Its arithmetic is MTL5's, through the adapter
/// (workbench/mtl5.hpp): P is kept packed between firings, and a handler
/// unpacks it, computes, and packs the result.
///
/// The graph runs the filter twice, as Kalman_f64 and Kalman_f32, the f32 one
/// fed the fixes through a Cast, so the oracle can compare the two.
///
/// Oracle (W2), in long double and without MTL5:
///   P is positive definite after every firing: its Cholesky factorization succeeds;
///   with one fix per update, P after the 500th update agrees with the
///     steady-state solution of the discrete Riccati equation to a relative 1e-9;
///   the normalized estimation error squared (NEES) of the first 2000 updates,
///     taken at every 25th, averages within its 95 % chi-square bounds. A
///     filter's errors are correlated from one update to the next, so the
///     mean over every update varies several times more than the bounds of
///     independent samples allow; 25 updates apart (2.5 s), they are nearly
///     independent;
///   the f32 and f64 filters' posteriors agree to a relative 1e-4.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/workbench/codecs.hpp>
#include <branes/anatomy/workbench/mtl5.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace branes::anatomy::workbench::kalman {

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

template <class E>
using StateD = fl::vector<E, fl::dim<4>>;
template <class E>
using CovD = fl::matrix<E, fl::dim<4>, fl::dim<4>, fl::structure::symmetric>;
template <class E>
using FixesD = fl::matrix<E, fl::bounded<4>, fl::dim<2>>;
using TimeD = fl::scalar<std::int64_t>;

template <class E>
using Vec4 = fl::data<StateD<E>>;
template <class E>
using Cov4 = fl::data<CovD<E>>;
template <class E>
using Fixes = fl::data<FixesD<E>>;

// ── Configuration ───────────────────────────────────────────────────

struct config {
    std::uint64_t seed = 1;
    std::int64_t clock_period_ns = 10'000'000;  ///< 100 Hz
    std::int64_t fix_period_ns = 100'000'000;   ///< 10 Hz, on clock ticks
    double q = 0.5;                             ///< white acceleration's spectral density per axis [m²/s³]
    double r = 0.25;                            ///< a fix's variance per axis [m²]
    std::size_t max_fixes = 1;                  ///< fixes per update: 1 to max_fixes
    double p_none = 0.0;                        ///< the chance that an update has none
    std::array<double, 4> x0{0.0, 0.0, 1.0, 0.5};
    std::array<double, 4> p0{4.0, 4.0, 1.0, 1.0};  ///< P's diagonal at the start

    /// One fix in every update: the Riccati check applies.
    [[nodiscard]] bool one_fix() const noexcept {
        return max_fixes == 1 && p_none == 0.0;
    }
};

struct context {
    config cfg;
};

// ── The operator ────────────────────────────────────────────────────

/// Constant velocity in the plane, in element type E.
template <class E>
struct Kalman {
    Vec4<E> x;
    Cov4<E> P;
    fl::data<TimeD> t_last;  ///< the last predict's time [ns]
    E q{};
    E r{};

    /// A symmetric result's two triangles agree to this, relative, before it is packed.
    static constexpr double kSymmetry = 1e3 * static_cast<double>(std::numeric_limits<E>::epsilon());

    [[nodiscard]] static result<Kalman, std::string> from_context(const context& c) {
        Kalman k;
        for (std::size_t i = 0; i < 4; ++i) {
            k.x(i) = static_cast<E>(c.cfg.x0[i]);
            k.P(i, i) = static_cast<E>(c.cfg.p0[i]);
        }
        k.q = static_cast<E>(c.cfg.q);
        k.r = static_cast<E>(c.cfg.r);
        return k;
    }

    void predict(Timestamp t) {
        const E dt = static_cast<E>(static_cast<double>(t.ns - t_last()) * 1e-9);
        t_last() = t.ns;
        if (!(dt > E{0})) {
            return;
        }
        mtl5::fixed_matrix<E, 4, 4> F;
        for (std::size_t i = 0; i < 4; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                F(i, j) = i == j ? E{1} : E{0};
            }
        }
        F(0, 2) = dt;
        F(1, 3) = dt;

        auto xv = mtl5::view(x);
        mtl5::fixed_vector<E, 4> fx;
        (void)mtl5::mult(F, xv, fx);
        (void)mtl5::assign(xv, fx);

        const auto p = mtl5::unpack(P);
        mtl5::fixed_matrix<E, 4, 4> fp;
        mtl5::fixed_matrix<E, 4, 4> fpf;
        (void)mtl5::mult(F, p, fp);
        (void)mtl5::mult(fp, mtl::trans(F), fpf);
        // Q of white acceleration, per axis: q·[dt³/3, dt²/2; dt²/2, dt].
        const E q11 = q * dt * dt * dt / E{3};
        const E q12 = q * dt * dt / E{2};
        const E q22 = q * dt;
        for (std::size_t a = 0; a < 2; ++a) {
            fpf(a, a) += q11;
            fpf(a, a + 2) += q12;
            fpf(a + 2, a) += q12;
            fpf(a + 2, a + 2) += q22;
        }
        mtl5::pack(fpf, P, kSymmetry);
    }

    /// One fix z = (px, py) + noise of variance r per axis, in Joseph form.
    void update_one(E zx, E zy) {
        const auto p = mtl5::unpack(P);
        // S = H·P·Hᵀ + R, H = [I₂ 0].
        const E s00 = p(0, 0) + r;
        const E s01 = p(0, 1);
        const E s11 = p(1, 1) + r;
        const E det = s00 * s11 - s01 * s01;
        mtl5::fixed_matrix<E, 2, 2> si;
        si(0, 0) = s11 / det;
        si(0, 1) = -s01 / det;
        si(1, 0) = -s01 / det;
        si(1, 1) = s00 / det;
        // K = P·Hᵀ·S⁻¹.
        mtl5::fixed_matrix<E, 4, 2> pht;
        for (std::size_t i = 0; i < 4; ++i) {
            pht(i, 0) = p(i, 0);
            pht(i, 1) = p(i, 1);
        }
        mtl5::fixed_matrix<E, 4, 2> K;
        (void)mtl5::mult(pht, si, K);
        // x += K·(z − H·x).
        auto xv = mtl5::view(x);
        mtl5::fixed_vector<E, 2> nu;
        nu(0) = zx - xv(0);
        nu(1) = zy - xv(1);
        mtl5::fixed_vector<E, 4> dx;
        (void)mtl5::mult(K, nu, dx);
        (void)mtl5::add(xv, dx);
        // P = (I − K·H)·P·(I − K·H)ᵀ + K·R·Kᵀ.
        mtl5::fixed_matrix<E, 2, 4> H;
        for (std::size_t i = 0; i < 2; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                H(i, j) = i == j ? E{1} : E{0};
            }
        }
        mtl5::fixed_matrix<E, 4, 4> kh;
        (void)mtl5::mult(K, H, kh);
        mtl5::fixed_matrix<E, 4, 4> a;
        for (std::size_t i = 0; i < 4; ++i) {
            for (std::size_t j = 0; j < 4; ++j) {
                a(i, j) = (i == j ? E{1} : E{0}) - kh(i, j);
            }
        }
        mtl5::fixed_matrix<E, 4, 4> ap;
        mtl5::fixed_matrix<E, 4, 4> apa;
        (void)mtl5::mult(a, p, ap);
        (void)mtl5::mult(ap, mtl::trans(a), apa);
        mtl5::fixed_matrix<E, 4, 4> kk;
        (void)mtl5::mult(K, mtl::trans(K), kk);
        (void)mtl5::add(apa, r * kk);
        mtl5::pack(apa, P, kSymmetry);
    }

    void update(const Fixes<E>& z) {
        for (std::size_t k = 0; k < z.count(); ++k) {
            update_one(z(k, 0), z(k, 1));
        }
    }
};

namespace declare {

/// predict: one space (i, j, k) over 4 × 4 × 4. dt, Q's terms and F live at
/// the points the regions give them; x ← F·x along k at j = 0, F·P along k,
/// then (F·P)·Fᵀ along k, whose last face writes P's upper triangle with Q
/// added. If dt <= 0, x and P keep their values; t_last always takes the time.
template <class E>
constexpr cd::system predict() {
    constexpr cd::etype e = cd::etype_of<E>;
    cd::system s;
    const auto n = s.extent("x", 0);
    const auto i = s.index("i", 0, n);
    const auto j = s.index("j", 0, n);
    const auto k = s.index("k", 0, n);
    const auto x = s.state("x");
    const auto P = s.state("P");
    const auto q = s.param("q", e);
    const auto at0 = [&] { return s.where(i == 0, j == 0, k == 0); };

    // dt in E, from the time since the last predict, in f64 as the handler computes it.
    const auto dt = s.var("dt", e);
    s.eq(dt, s.cast(s.cast(s.time() - s.state("t_last")(), cd::etype::f64) * 1e-9, e), at0());
    s.write(s.state("t_last").at(), s.time(), at0());
    const auto d = [&] { return dt(0, 0, 0); };
    const auto q11 = s.var("q11", e);
    const auto q12 = s.var("q12", e);
    const auto q22 = s.var("q22", e);
    s.eq(q11, q() * d() * d() * d() / 3.0, at0());
    s.eq(q12, q() * d() * d() / 2.0, at0());
    s.eq(q22, q() * d(), at0());

    // F = I, with dt at (0, 2) and (1, 3), at k = 0.
    const auto F = s.var("F", e);
    s.eq(F, s.literal(1, e), s.where(k == 0, i == j));
    s.eq(F, d(), s.where(k == 0, j == i + 2));
    s.eq(F, s.literal(0, e), s.where(k == 0, j < i));
    s.eq(F, s.literal(0, e), s.where(k == 0, j == i + 1));
    s.eq(F, s.literal(0, e), s.where(k == 0, j == i + 3));

    // x ← F·x, at j = 0.
    const auto fx = s.var("fx", e);
    s.init(fx, s.literal(0, e), s.face(k == -1, j == 0));
    s.eq(fx, fx(i, j, k - 1) + F(i, k, 0) * x(k), s.where(j == 0));
    s.write(x.at(i), cd::select_le(d(), s.literal(0, e), x(i), fx(i, 0, n - 1)), s.where(j == 0, k == n - 1));

    // F·P, then (F·P)·Fᵀ.
    const auto fp = s.var("fp", e);
    s.init(fp, s.literal(0, e), s.face(k == -1));
    s.eq(fp, fp(i, j, k - 1) + F(i, k, 0) * P(k, j));
    const auto fpf = s.var("fpf", e);
    s.init(fpf, s.literal(0, e), s.face(k == -1));
    s.eq(fpf, fpf(i, j, k - 1) + fp(i, k, n - 1) * F(j, k, 0));

    // P ← F·P·Fᵀ + Q, its upper triangle, on the last face.
    const auto last = n - 1;
    const auto keep = [&](cd::val v) { return cd::select_le(d(), s.literal(0, e), P(i, j), v); };
    s.write(P.at(i, j), keep(fpf(i, j, last) + q11(0, 0, 0)), s.where(k == last, i == j, i < 2));
    s.write(P.at(i, j), keep(fpf(i, j, last) + q22(0, 0, 0)), s.where(k == last, i == j, i >= 2));
    s.write(P.at(i, j), keep(fpf(i, j, last) + q12(0, 0, 0)), s.where(k == last, j == i + 2));
    s.write(P.at(i, j), keep(fpf(i, j, last)), s.where(k == last, j == i + 1));
    s.write(P.at(i, j), keep(fpf(i, j, last)), s.where(k == last, j == i + 3));
    return s;
}

/// update: one fix at a time, f over the fixes' count, in Joseph form, in
/// one space (f, i, j, k) over count × 4 × 4 × 4. Each step of the handler
/// is a variable on the region its result occupies; a product is a
/// reduction along k. The state before fix f is pc (P) and xc (x), and after
/// it pn and xn; the last fix's are written to the state.
template <class E>
constexpr cd::system update() {
    constexpr cd::etype e = cd::etype_of<E>;
    cd::system s;
    const auto n = s.extent("x", 0);
    const auto m = s.count("fixes");
    const auto f = s.index("f", 0, m);
    const auto i = s.index("i", 0, n);
    const auto j = s.index("j", 0, n);
    const auto k = s.index("k", 0, n);
    const auto r = s.param("r", e);
    const auto z = s.in("fixes");
    const auto P = s.state("P");
    const auto x = s.state("x");
    const auto zero = [&] { return s.literal(0, e); };
    const auto one = [&] { return s.literal(1, e); };

    // The state before fix f.
    const auto pc = s.var("pc", e);
    const auto pn = s.var("pn", e);
    s.eq(pc, P(i, j), s.where(k == 0, f == 0));
    s.eq(pc, pn(f - 1, i, j, 0), s.where(k == 0, f >= 1, i <= j));
    s.eq(pc, pn(f - 1, j, i, 0), s.where(k == 0, f >= 1, j < i));
    const auto xc = s.var("xc", e);
    const auto xn = s.var("xn", e);
    s.eq(xc, x(i), s.where(j == 0, k == 0, f == 0));
    s.eq(xc, xn(f - 1, i, 0, 0), s.where(j == 0, k == 0, f >= 1));

    // S = H·P·Hᵀ + R, and its inverse.
    const auto at0 = [&] { return s.where(i == 0, j == 0, k == 0); };
    const auto s00 = s.var("s00", e);
    const auto s01 = s.var("s01", e);
    const auto s11 = s.var("s11", e);
    const auto det = s.var("det", e);
    s.eq(s00, pc(f, 0, 0, 0) + r(), at0());
    s.eq(s01, pc(f, 0, 1, 0), at0());
    s.eq(s11, pc(f, 1, 1, 0) + r(), at0());
    s.eq(det, s00(f, 0, 0, 0) * s11(f, 0, 0, 0) - s01(f, 0, 0, 0) * s01(f, 0, 0, 0), at0());
    const auto si = s.var("si", e);
    s.eq(si, s11(f, 0, 0, 0) / det(f, 0, 0, 0), s.where(k == 0, i == 0, j == 0));
    s.eq(si, -s01(f, 0, 0, 0) / det(f, 0, 0, 0), s.where(k == 0, i == 0, j == 1));
    s.eq(si, -s01(f, 0, 0, 0) / det(f, 0, 0, 0), s.where(k == 0, i == 1, j == 0));
    s.eq(si, s00(f, 0, 0, 0) / det(f, 0, 0, 0), s.where(k == 0, i == 1, j == 1));

    // K = P·Hᵀ·S⁻¹, a 4 × 2 product along k < 2; K(i, j) is gain(f, i, j, 1).
    const auto gain = s.var("gain", e);
    s.init(gain, zero(), s.face(k == -1, j < 2));
    s.eq(gain, gain(f, i, j, k - 1) + pc(f, i, k, 0) * si(f, k, j, 0), s.where(j < 2, k < 2));

    // x += K·(z − H·x).
    const auto nu = s.var("nu", e);
    s.eq(nu, z(f, i) - xc(f, i, 0, 0), s.where(i < 2, j == 0, k == 0));
    const auto dx = s.var("dx", e);
    s.init(dx, zero(), s.face(k == -1, j == 0));
    s.eq(dx, dx(f, i, j, k - 1) + gain(f, i, k, 1) * nu(f, k, 0, 0), s.where(j == 0, k < 2));
    s.eq(xn, xc(f, i, 0, 0) + dx(f, i, 0, 1), s.where(j == 0, k == 0));

    // A = I − K·H, with H = [I₂ 0]: K·H along k < 2.
    const auto kh = s.var("kh", e);
    s.init(kh, zero(), s.face(k == -1));
    s.eq(kh, kh(f, i, j, k - 1) + gain(f, i, k, 1) * one(), s.where(k < 2, k == j));
    s.eq(kh, kh(f, i, j, k - 1) + gain(f, i, k, 1) * zero(), s.where(k < 2, k < j));
    s.eq(kh, kh(f, i, j, k - 1) + gain(f, i, k, 1) * zero(), s.where(k < 2, j < k));
    const auto a = s.var("a", e);
    s.eq(a, one() - kh(f, i, j, 1), s.where(k == 0, i == j));
    s.eq(a, zero() - kh(f, i, j, 1), s.where(k == 0, j < i));
    s.eq(a, zero() - kh(f, i, j, 1), s.where(k == 0, i < j));

    // P = A·P·Aᵀ + K·R·Kᵀ: A·P, then (A·P)·Aᵀ, and K·Kᵀ along k < 2.
    const auto ap = s.var("ap", e);
    s.init(ap, zero(), s.face(k == -1));
    s.eq(ap, ap(f, i, j, k - 1) + a(f, i, k, 0) * pc(f, k, j, 0));
    const auto apa = s.var("apa", e);
    s.init(apa, zero(), s.face(k == -1));
    s.eq(apa, apa(f, i, j, k - 1) + ap(f, i, k, n - 1) * a(f, j, k, 0));
    const auto kk = s.var("kk", e);
    s.init(kk, zero(), s.face(k == -1));
    s.eq(kk, kk(f, i, j, k - 1) + gain(f, i, k, 1) * gain(f, j, k, 1), s.where(k < 2));
    s.eq(pn, apa(f, i, j, n - 1) + r() * kk(f, i, j, 1), s.where(k == 0, i <= j));

    // The last fix's state.
    s.write(P.at(i, j), pn(f, i, j, 0), s.where(f == m - 1, k == 0, i <= j));
    s.write(x.at(i), xn(f, i, 0, 0), s.where(f == m - 1, j == 0, k == 0));
    return s;
}

/// prior and posterior: x and P, copied out.
constexpr cd::system read_out() {
    cd::system s;
    const auto n = s.extent("x", 0);
    const auto i = s.index("i", 0, n);
    const auto j = s.index("j", 0, n);
    s.write(s.out("mean").at(i), s.state("x")(i), s.where(j == 0));
    s.write(s.out("cov").at(i, j), s.state("P")(i, j), s.where(i <= j));
    return s;
}

}  // namespace declare

template <class E>
struct KalmanBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"fixes", Fixes<E>, fl::direction::in>,
                                fl::port<"mean", Vec4<E>, fl::direction::out>,
                                fl::port<"cov", Cov4<E>, fl::direction::out>>,
                      fl::states<fl::state<"x", StateD<E>>, fl::state<"P", CovD<E>>, fl::state<"t_last", TimeD>>>;
    struct predict : fl::handler<"predict", fl::ins<>, fl::outs<>, fl::state_role::update, fl::takes_time::yes> {
        static constexpr cd::system compute = declare::predict<E>();
        static void call(Kalman<E>& op, Timestamp t) {
            op.predict(t);
        }
    };
    struct update : fl::handler<"update", fl::ins<"fixes">, fl::outs<>, fl::state_role::update> {
        static constexpr cd::system compute = declare::update<E>();
        static void call(Kalman<E>& op, const Fixes<E>& z) {
            op.update(z);
        }
    };
    struct prior : fl::handler<"prior", fl::ins<>, fl::outs<"mean", "cov">, fl::state_role::read> {
        static constexpr cd::system compute = declare::read_out();
        static std::tuple<Vec4<E>, Cov4<E>> call(const Kalman<E>& op) {
            return {op.x, op.P};
        }
    };
    struct posterior : fl::handler<"posterior", fl::ins<>, fl::outs<"mean", "cov">, fl::state_role::read> {
        static constexpr cd::system compute = declare::read_out();
        static std::tuple<Vec4<E>, Cov4<E>> call(const Kalman<E>& op) {
            return {op.x, op.P};
        }
    };
    using handlers = fl::handlers<predict, update, prior, posterior>;
    using cd_state = cd::members<cd::member<"x", &Kalman<E>::x>,
                                 cd::member<"P", &Kalman<E>::P>,
                                 cd::member<"t_last", &Kalman<E>::t_last>>;
    using cd_params = cd::members<cd::member<"q", &Kalman<E>::q>, cd::member<"r", &Kalman<E>::r>>;
};

/// The fixes, in another element type; the count is kept.
template <class From, class To>
struct CastFixes {
    [[nodiscard]] static Fixes<To> apply(const Fixes<From>& z) noexcept {
        Fixes<To> out;
        (void)out.resize(z.count());
        for (std::size_t k = 0; k < z.count(); ++k) {
            out(k, 0) = static_cast<To>(z(k, 0));
            out(k, 1) = static_cast<To>(z(k, 1));
        }
        return out;
    }
};

template <class From, class To>
struct CastFixesBinding {
    using signature = fl::signature<
        fl::ports<fl::port<"x", Fixes<From>, fl::direction::in>, fl::port<"y", Fixes<To>, fl::direction::out>>>;
    struct apply : fl::handler<"apply", fl::ins<"x">, fl::outs<"y">> {
        static constexpr cd::system compute = [] {
            cd::system s;
            const auto m = s.count("x");
            const auto k = s.index("k", 0, m);
            const auto c = s.index("c", 0, s.extent("x", 1));
            s.write(s.out("y").at(k, c), s.cast(s.in("x")(k, c), cd::etype_of<To>));
            s.set_count(s.out("y"), m);
            return s;
        }();
        static Fixes<To> call(const CastFixes<From, To>&, const Fixes<From>& z) {
            return CastFixes<From, To>::apply(z);
        }
    };
    using handlers = fl::handlers<apply>;
};

using Kalman64 = fl::registered<"Kalman_f64", Kalman<double>, KalmanBinding<double>>;
using Kalman32 = fl::registered<"Kalman_f32", Kalman<float>, KalmanBinding<float>>;
using CastFixes64 = fl::registered<"Cast_f64_f32_fixes4x2", CastFixes<double, float>, CastFixesBinding<double, float>>;

// ── The truth, and the fixes ────────────────────────────────────────

/// The noisy constant-velocity trajectory: the truth at every clock tick,
/// drawn from the seed as the filter's model says (x₀ ~ N(x̂₀, P₀), white
/// acceleration of density q), extended as it is read. Ticks are read in order.
class trajectory {
public:
    explicit trajectory(const config& c) : c_(c), r_(substream(c.seed, 1)) {
        std::array<double, 4> s{};
        for (std::size_t i = 0; i < 4; ++i) {
            s[i] = c.x0[i] + std::sqrt(c.p0[i]) * r_.normal();
        }
        states_.push_back(s);
    }

    /// The truth at clock tick n (time n·clock_period).
    [[nodiscard]] const std::array<double, 4>& at(std::size_t n) {
        const double dt = static_cast<double>(c_.clock_period_ns) * 1e-9;
        // Q's Cholesky factor per axis: [√a 0; b/√a √(c − b²/a)].
        const double a = c_.q * dt * dt * dt / 3.0;
        const double b = c_.q * dt * dt / 2.0;
        const double cc = c_.q * dt;
        const double l11 = std::sqrt(a);
        const double l21 = b / l11;
        const double l22 = std::sqrt(cc - l21 * l21);
        while (states_.size() <= n) {
            std::array<double, 4> s = states_.back();
            s[0] += dt * s[2];
            s[1] += dt * s[3];
            for (std::size_t axis = 0; axis < 2; ++axis) {
                const double u = r_.normal();
                const double v = r_.normal();
                s[axis] += l11 * u;
                s[axis + 2] += l21 * u + l22 * v;
            }
            states_.push_back(s);
        }
        return states_[n];
    }

private:
    config c_;
    rng r_;
    std::vector<std::array<double, 4>> states_;
};

/// The k-th fixes: the truth's position at their time, plus noise of variance r.
[[nodiscard]] inline Fixes<double> fixes_of(const config& c, trajectory& truth, std::uint64_t k) {
    rng r(substream(c.seed, 2'000'000 + k));
    std::size_t count = 1;
    if (!c.one_fix()) {
        count = r.unit() < c.p_none ? 0 : 1 + static_cast<std::size_t>(r.next() % c.max_fixes);
    }
    const auto tick = static_cast<std::size_t>(static_cast<std::int64_t>(k) * c.fix_period_ns / c.clock_period_ns);
    const std::array<double, 4>& s = truth.at(tick);
    Fixes<double> z;
    (void)z.resize(count);
    const double sigma = std::sqrt(c.r);
    for (std::size_t i = 0; i < count; ++i) {
        z(i, 0) = s[0] + sigma * r.normal();
        z(i, 1) = s[1] + sigma * r.normal();
    }
    return z;
}

// ── The graph ───────────────────────────────────────────────────────

struct ports {
    fl::input_port<Timestamp> clock;
    fl::input_port<Fixes<double>> fixes;
    fl::output_port<Vec4<double>> prior_mean;
    fl::output_port<Cov4<double>> prior_cov;
    fl::output_port<Vec4<double>> posterior_mean;
    fl::output_port<Cov4<double>> posterior_cov;
    fl::output_port<Vec4<float>> posterior_mean_f32;
    fl::output_port<Cov4<float>> posterior_cov_f32;
};

template <class G>
[[nodiscard]] ports graph(G& g, const config&) {
    const auto tick = g.tick("clock");
    const auto fixes = fl::input<Fixes<double>>(g, "fixes");
    ports p{tick,
            fixes,
            fl::output<Vec4<double>>(g, "prior_mean", tick),
            fl::output<Cov4<double>>(g, "prior_cov", tick),
            fl::output<Vec4<double>>(g, "posterior_mean", fixes),
            fl::output<Cov4<double>>(g, "posterior_cov", fixes),
            fl::output<Vec4<float>>(g, "posterior_mean_f32", fixes),
            fl::output<Cov4<float>>(g, "posterior_cov_f32", fixes)};
    const auto kf = fl::instance<Kalman64>(g, "kalman");
    const auto kf32 = fl::instance<Kalman32>(g, "kalman_f32");
    const auto cast = fl::instance<CastFixes64>(g, "cast");

    fl::scope rc = g.region(tick);
    fl::fire<"predict">(rc, kf, fl::at(tick));
    auto [pm, pc] = fl::fire<"prior">(rc, kf);
    fl::fire<"predict">(rc, kf32, fl::at(tick));
    (void)fl::fire<"prior">(rc, kf32);
    g.produce(p.prior_mean, pm);
    g.produce(p.prior_cov, pc);

    fl::scope rf = g.region(fixes);
    fl::fire<"update">(rf, kf, fixes);
    auto [qm, qc] = fl::fire<"posterior">(rf, kf);
    auto [z32] = fl::fire<"apply">(rf, cast, fixes);
    fl::fire<"update">(rf, kf32, z32);
    auto [qm32, qc32] = fl::fire<"posterior">(rf, kf32);
    g.produce(p.posterior_mean, qm);
    g.produce(p.posterior_cov, qc);
    g.produce(p.posterior_mean_f32, qm32);
    g.produce(p.posterior_cov_f32, qc32);
    return p;
}

// ── Samples, the run by hand (W1), and the oracle (W2) ──────────────

/// After each input: which source it came from (0 the clock, 1 the fixes),
/// its time, and the outputs.
struct sample {
    std::size_t source = 0;
    Timestamp t;
    Vec4<double> prior_mean;
    Cov4<double> prior_cov;
    Vec4<double> posterior_mean;
    Cov4<double> posterior_cov;
    Vec4<float> posterior_mean_f32;
    Cov4<float> posterior_cov_f32;

    friend bool operator==(const sample&, const sample&) = default;
};

/// The operators called by hand on the same stream, in the graph's order:
/// the clock and the fixes merged as the driver merges them.
template <class Hook = no_hook>
[[nodiscard]] std::vector<sample> hand_run(const config& c, std::size_t inputs, Hook&& hook = {}) {
    const context ctx{c};
    auto kf = *Kalman<double>::from_context(ctx);
    auto kf32 = *Kalman<float>::from_context(ctx);
    trajectory truth(c);
    using B = KalmanBinding<double>;
    using B32 = KalmanBinding<float>;
    std::vector<sample> out;
    sample s;
    std::uint64_t ticks = 0;
    std::uint64_t fixes = 0;
    for (std::size_t i = 0; i < inputs; ++i) {
        const std::int64_t tc = static_cast<std::int64_t>(ticks) * c.clock_period_ns;
        const std::int64_t tf = static_cast<std::int64_t>(fixes) * c.fix_period_ns;
        if (tc <= tf) {  // the clock first, at a shared time
            hook.template before<Kalman64, "predict">(kf, Timestamp{tc});
            B::predict::call(kf, Timestamp{tc});
            hook.template before<Kalman64, "prior">(kf, Timestamp{tc});
            std::tie(s.prior_mean, s.prior_cov) = B::prior::call(kf);
            hook.template before<Kalman32, "predict">(kf32, Timestamp{tc});
            B32::predict::call(kf32, Timestamp{tc});
            hook.template before<Kalman32, "prior">(kf32, Timestamp{tc});
            (void)B32::prior::call(kf32);
            s.source = 0;
            s.t = Timestamp{tc};
            ++ticks;
        } else {
            const Fixes<double> z = fixes_of(c, truth, fixes);
            const Timestamp t{tf};
            hook.template before<Kalman64, "update">(kf, t, z);
            B::update::call(kf, z);
            hook.template before<Kalman64, "posterior">(kf, t);
            std::tie(s.posterior_mean, s.posterior_cov) = B::posterior::call(kf);
            hook.template before<CastFixes64, "apply">({}, t, z);
            const Fixes<float> z32 = CastFixesBinding<double, float>::apply::call({}, z);
            hook.template before<Kalman32, "update">(kf32, t, z32);
            B32::update::call(kf32, z32);
            hook.template before<Kalman32, "posterior">(kf32, t);
            std::tie(s.posterior_mean_f32, s.posterior_cov_f32) = B32::posterior::call(kf32);
            s.source = 1;
            s.t = Timestamp{tf};
            ++fixes;
        }
        out.push_back(s);
    }
    return out;
}

namespace oracle_detail {

using M4 = std::array<std::array<long double, 4>, 4>;

[[nodiscard]] inline M4 mul(const M4& a, const M4& b) {
    M4 c{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            for (std::size_t k = 0; k < 4; ++k) {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    return c;
}
[[nodiscard]] inline M4 transpose(const M4& a) {
    M4 t{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            t[i][j] = a[j][i];
        }
    }
    return t;
}

/// P's lower Cholesky factor, or false if P isn't positive definite.
[[nodiscard]] inline bool cholesky(const M4& p, M4& l) {
    l = M4{};
    for (std::size_t j = 0; j < 4; ++j) {
        long double d = p[j][j];
        for (std::size_t k = 0; k < j; ++k) {
            d -= l[j][k] * l[j][k];
        }
        if (!(d > 0)) {
            return false;
        }
        l[j][j] = std::sqrt(d);
        for (std::size_t i = j + 1; i < 4; ++i) {
            long double s = p[i][j];
            for (std::size_t k = 0; k < j; ++k) {
                s -= l[i][k] * l[j][k];
            }
            l[i][j] = s / l[j][j];
        }
    }
    return true;
}

template <class E>
[[nodiscard]] M4 full(const Cov4<E>& p) {
    M4 m{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            m[i][j] = p.at(i, j);
        }
    }
    return m;
}

[[nodiscard]] inline M4 predicted(const M4& p, long double dt, long double q) {
    M4 f{};
    for (std::size_t i = 0; i < 4; ++i) {
        f[i][i] = 1;
    }
    f[0][2] = dt;
    f[1][3] = dt;
    M4 out = mul(mul(f, p), transpose(f));
    for (std::size_t a = 0; a < 2; ++a) {
        out[a][a] += q * dt * dt * dt / 3;
        out[a][a + 2] += q * dt * dt / 2;
        out[a + 2][a] += q * dt * dt / 2;
        out[a + 2][a + 2] += q * dt;
    }
    return out;
}

/// The covariance after one fix of variance r per axis.
[[nodiscard]] inline M4 updated(const M4& p, long double r) {
    const long double s00 = p[0][0] + r;
    const long double s01 = p[0][1];
    const long double s11 = p[1][1] + r;
    const long double det = s00 * s11 - s01 * s01;
    std::array<std::array<long double, 2>, 4> k{};
    for (std::size_t i = 0; i < 4; ++i) {
        k[i][0] = (p[i][0] * s11 - p[i][1] * s01) / det;
        k[i][1] = (p[i][1] * s00 - p[i][0] * s01) / det;
    }
    M4 out{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            out[i][j] = p[i][j] - k[i][0] * p[0][j] - k[i][1] * p[1][j];  // (I − KH)·P
        }
    }
    // Symmetrize: the optimal update's (I − KH)P is symmetric in exact arithmetic.
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            out[i][j] = out[j][i] = (out[i][j] + out[j][i]) / 2;
        }
    }
    return out;
}

/// χ²'s p-quantile for k degrees of freedom, by Wilson and Hilferty.
[[nodiscard]] inline double chi2_quantile(double k, double z) {
    const double a = 2.0 / (9.0 * k);
    const double t = 1.0 - a + z * std::sqrt(a);
    return k * t * t * t;
}

}  // namespace oracle_detail

/// The updates between two NEES samples, so the samples are nearly independent.
inline constexpr std::size_t kNeesStride = 25;

struct pattern {
    static constexpr std::string_view name = "kalman";
    /// 2000 updates at 10 Hz, and the clock's 20000 ticks between them.
    static constexpr std::size_t default_inputs = 22'000;
    using config = kalman::config;
    using context = kalman::context;
    using ports = kalman::ports;
    using sample = kalman::sample;
    using leaves = workbench::leaves<leaf<Kalman64, "predict">,
                                     leaf<Kalman64, "update">,
                                     leaf<Kalman64, "prior">,
                                     leaf<Kalman64, "posterior">,
                                     leaf<Kalman32, "predict">,
                                     leaf<Kalman32, "update">,
                                     leaf<Kalman32, "prior">,
                                     leaf<Kalman32, "posterior">,
                                     leaf<CastFixes64, "apply">>;

    template <class G>
    [[nodiscard]] static ports graph(G& g, const config& c) {
        return kalman::graph(g, c);
    }

    /// The clock, then the fixes: at a shared time, the clock goes first.
    [[nodiscard]] static std::vector<source> sources(const ports& p, const config& c) {
        auto truth = std::make_shared<trajectory>(c);
        return {clock(p.clock, c.clock_period_ns),
                sequence(p.fixes, c.fix_period_ns, [c, truth](std::uint64_t k, Timestamp) {
                    return fixes_of(c, *truth, k);
                })};
    }

    [[nodiscard]] static sample
    collect(const fl::stack& s, const ports& p, std::size_t, std::size_t source, Timestamp t) {
        return sample{source,
                      t,
                      s.output(p.prior_mean),
                      s.output(p.prior_cov),
                      s.output(p.posterior_mean),
                      s.output(p.posterior_cov),
                      s.output(p.posterior_mean_f32),
                      s.output(p.posterior_cov_f32)};
    }

    [[nodiscard]] static report oracle(const config& c, std::span<const sample> samples) {
        using namespace oracle_detail;
        report rep{std::string(name), {}};
        trajectory truth(c);
        std::size_t not_pd = 0;
        std::size_t updates = 0;
        long double nees_sum = 0;
        std::size_t nees_n = 0;
        long double f32_x = 0;
        long double f32_p = 0;
        M4 at500{};
        bool reached = false;
        std::uint64_t k = 0;
        M4 l{};
        for (const sample& s : samples) {
            if (s.source == 0) {
                not_pd += cholesky(full(s.prior_cov), l) ? 0 : 1;
                continue;
            }
            const M4 p = full(s.posterior_cov);
            const bool pd = cholesky(p, l);
            not_pd += pd ? 0 : 1;
            M4 l32{};  // its own: the NEES below solves with the f64 filter's factor, l
            not_pd += cholesky(full(s.posterior_cov_f32), l32) ? 0 : 1;
            const std::size_t count = fixes_of(c, truth, k).count();
            ++k;
            ++updates;
            if (updates == 500) {
                at500 = p;
                reached = true;
            }
            f32_x = std::fmax(f32_x, relative_max(s.posterior_mean_f32.elements, s.posterior_mean.elements, 4));
            f32_p = std::fmax(
                f32_p, relative_max(s.posterior_cov_f32.elements, s.posterior_cov.elements, Cov4<double>::stored));
            // NEES: e = truth − x̂, eᵀ·P⁻¹·e, by the Cholesky factor.
            if (count > 0 && pd && updates <= 2000 && (updates - 1) % kNeesStride == 0) {
                const auto tick = static_cast<std::size_t>(s.t.ns / c.clock_period_ns);
                const std::array<double, 4>& x = truth.at(tick);
                std::array<long double, 4> y{};
                for (std::size_t i = 0; i < 4; ++i) {
                    long double v = static_cast<long double>(x[i]) - s.posterior_mean.elements[i];
                    for (std::size_t j = 0; j < i; ++j) {
                        v -= l[i][j] * y[j];
                    }
                    y[i] = v / l[i][i];
                }
                nees_sum += y[0] * y[0] + y[1] * y[1] + y[2] * y[2] + y[3] * y[3];
                ++nees_n;
            }
        }
        rep.checks.push_back(at_least("updates", static_cast<double>(updates), 1));
        rep.checks.push_back(at_most("firings whose P isn't positive definite", static_cast<double>(not_pd), 0));
        if (c.one_fix()) {
            // The steady state of P ← update(predict¹⁰(P)), iterated in long double.
            const long double dt = static_cast<long double>(c.clock_period_ns) * 1e-9L;
            const auto per = static_cast<std::size_t>(c.fix_period_ns / c.clock_period_ns);
            M4 ss{};
            for (std::size_t i = 0; i < 4; ++i) {
                ss[i][i] = c.p0[i];
            }
            ss = updated(ss, c.r);
            for (std::size_t it = 0; it < 20000; ++it) {
                for (std::size_t j = 0; j < per; ++j) {
                    ss = predicted(ss, dt, c.q);
                }
                ss = updated(ss, c.r);
            }
            long double err = 0;
            long double norm = 0;
            for (std::size_t i = 0; i < 4; ++i) {
                for (std::size_t j = 0; j < 4; ++j) {
                    err = std::fmax(err, std::fabs(at500[i][j] - ss[i][j]));
                    norm = std::fmax(norm, std::fabs(ss[i][j]));
                }
            }
            rep.checks.push_back(at_most("P after 500 updates against Riccati, relative",
                                         reached ? static_cast<double>(err / norm) : 1.0,
                                         1e-9));
        }
        if (nees_n > 0) {
            const double n = static_cast<double>(nees_n);
            const double mean = static_cast<double>(nees_sum) / n;
            rep.checks.push_back(
                at_least("NEES mean, above its 95 % bound", mean, chi2_quantile(4 * n, -1.959964) / n));
            rep.checks.push_back(at_most("NEES mean, below its 95 % bound", mean, chi2_quantile(4 * n, 1.959964) / n));
        }
        rep.checks.push_back(at_most("f32 posterior mean against f64, relative", static_cast<double>(f32_x), 1e-4));
        rep.checks.push_back(
            at_most("f32 posterior covariance against f64, relative", static_cast<double>(f32_p), 1e-4));
        return rep;
    }

    [[nodiscard]] static fl::trace_codecs codecs() {
        fl::trace_codecs c;
        add_data<StateD<double>>(c);
        add_data<CovD<double>>(c);
        add_data<StateD<float>>(c);
        add_data<CovD<float>>(c);
        add_data<FixesD<float>>(c);
        return c;
    }
};

}  // namespace branes::anatomy::workbench::kalman
