// The computational domains of the MLP's, the Kalman filter's, the DSP
// pipeline's and PID control's handlers (docs/plan/computational-domain, C2,
// C3 and C4):
//   W10 every declared handler passes K1–K7, and K8 classifies it as stated;
//   W11 each declaration, evaluated on the values of each of its firings in
//       the run by hand, gives what its handler gives;
//   W12 a firing's declaration gives the same results in 20 random orders;
//   W13 the operation counts are their closed forms.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/workbench/evaluate.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/dsp.hpp>
#include <branes/anatomy/workbench/patterns/kalman.hpp>
#include <branes/anatomy/workbench/patterns/mlp.hpp>
#include <branes/anatomy/workbench/patterns/pid.hpp>

#include <catch2/catch_test_macros.hpp>

#include <bit>
#include <cstdint>
#include <map>
#include <string>

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;
namespace ev = branes::anatomy::workbench::eval;
namespace wb = branes::anatomy::workbench;
using branes::anatomy::Timestamp;

namespace {

std::string problems_of(const std::vector<cd::problem>& ps) {
    std::string out;
    for (const cd::problem& p : ps) {
        out += p.id + ": " + p.what + "\n";
    }
    return out;
}

/// W10 on every leaf of pattern P: the checks pass, and each stage has the class given.
template <class P>
std::map<std::string, cd::report> check_all() {
    std::map<std::string, cd::report> out;
    wb::for_each_leaf<typename P::leaves>([&]<class Op, fl::fixed_string H>() {
        const std::string who = std::string(Op::name) + "." + std::string(H.view());
        cd::report r = cd::check<Op, H>();
        INFO(who << "\n" << problems_of(r.problems));
        CHECK(r.ok());
        out[who] = std::move(r);
    });
    return out;
}

/// The run by hand's hook for W11 and W12: each firing's declaration against
/// its handler, the largest difference per leaf, and, for the first `orders`
/// firings of each leaf and every seventh, 20 random orders against the
/// deterministic one.
struct recorder {
    std::map<std::string, long double> worst;
    std::map<std::string, std::size_t> firings;
    std::vector<std::string> problems;
    std::size_t orders = 2;
    std::size_t order_mismatches = 0;

    template <class Op, fl::fixed_string H, class... In>
    void before(const typename Op::type& op, Timestamp t, const In&... in) {
        const std::string who = std::string(Op::name) + "." + std::string(H.view());
        const ev::firing f = ev::fire_both<Op, H>(op, t, in...);
        if (!f.problems.empty()) {
            problems.push_back(who + ": " + problems_of(f.problems));
            return;
        }
        long double& w = worst[who];
        for (const auto& [name, h] : f.handler) {
            const auto it = f.declared.find(name);
            if (it == f.declared.end()) {
                problems.push_back(who + ": the declaration gives no " + name);
                continue;
            }
            w = std::max(w, ev::difference(it->second, h));
        }
        // The first firings, and every seventh, so later ones (several fixes) are sampled too.
        const std::size_t nth = firings[who]++;
        if (nth < orders || nth % 7 == 0) {
            using B = typename Op::binding;
            using Hd = fl::handler_t<B, H>;
            const ev::inputs x = ev::inputs_of<Op, H>(op, t, in...);
            const cd::table tb = cd::table_of<B, Hd>();
            const ev::outputs ref = ev::evaluate(Hd::compute, tb, x, 0);
            for (std::uint64_t seed = 1; seed <= 20; ++seed) {
                const ev::outputs o = ev::evaluate(Hd::compute, tb, x, seed);
                order_mismatches += (o.ports != ref.ports || o.states != ref.states) ? 1 : 0;
            }
        }
    }
};

void require_clean(const recorder& r) {
    std::string all;
    for (const std::string& p : r.problems) {
        all += p;
    }
    INFO(all);
    CHECK(r.problems.empty());
    CHECK(r.order_mismatches == 0);
}

}  // namespace

// ── The MLP ─────────────────────────────────────────────────────────

TEST_CASE("compute: the MLP's handlers pass the checks, and are uniform", "[workbench][compute][mlp][W10]") {
    const auto reports = check_all<wb::mlp::pattern>();
    REQUIRE(reports.size() == 6);
    for (const auto& [who, r] : reports) {
        INFO(who);
        REQUIRE(r.stages.size() == 1);
        CHECK(r.stages[0] == cd::shape::uniform);
    }
    // The epilogues are elementwise: one definition deep.
    CHECK(reports.at("BiasAdd_f32_32.apply").depth == 1);
    CHECK(reports.at("Tanh_f32_32.apply").depth == 1);
    CHECK(reports.at("Cast_f32_f64_32.apply").depth == 1);
}

TEST_CASE("compute: the MLP's counts are R*C multiply-adds per layer, and its epilogues",
          "[workbench][compute][mlp][W13]") {
    using cd::etype;
    using cd::op;
    namespace m = wb::mlp;
    const auto r = check_all<m::pattern>();
    const cd::report& l1 = r.at("Linear_f32_32x16.apply");
    const cd::report& l2 = r.at("Linear_f64_8x32.apply");
    CHECK(l1.count(op::mul, etype::f32) == m::kHidden * m::kIn);
    CHECK(l1.count(op::add, etype::f32) == m::kHidden * m::kIn);
    CHECK(l2.count(op::mul, etype::f64) == m::kOut * m::kHidden);
    CHECK(l2.count(op::add, etype::f64) == m::kOut * m::kHidden);
    // The number step 1 recorded beside the pattern, derived here.
    CHECK(l1.count(op::mul) + l2.count(op::mul) == m::multiply_adds);
    CHECK(r.at("BiasAdd_f32_32.apply").count(op::add, etype::f32) == m::kHidden);
    CHECK(r.at("Tanh_f32_32.apply").count(op::tanh, etype::f32) == m::kHidden);
    CHECK(r.at("Cast_f32_f64_32.apply").count(op::cast) == m::kHidden);
    CHECK(r.at("BiasAdd_f64_8.apply").count(op::add, etype::f64) == m::kOut);
    // A reduction of C terms is C + 2 definitions deep: the seed, the terms, the write.
    CHECK(l1.depth == m::kIn + 2);
    CHECK(l2.depth == m::kHidden + 2);
}

TEST_CASE("compute: the MLP's declarations give what its handlers give, exactly",
          "[workbench][compute][mlp][W11][W12]") {
    const wb::mlp::config c;
    const auto w = wb::mlp::weights_of(c);
    recorder rec;
    for (std::uint64_t i = 0; i < 20; ++i) {
        (void)wb::mlp::hand_run(w, wb::mlp::input_of(c, i), rec);
    }
    require_clean(rec);
    REQUIRE(rec.worst.size() == 6);
    for (const auto& [who, d] : rec.worst) {
        INFO(who << ": " << static_cast<double>(d));
        // Each declaration fixes the order of every operation its handler does.
        CHECK(d == 0);
        CHECK(rec.firings.at(who) == 20);
    }
}

// ── The Kalman filter ───────────────────────────────────────────────

TEST_CASE("compute: the Kalman filter's handlers pass the checks", "[workbench][compute][kalman][W10]") {
    const auto r = check_all<wb::kalman::pattern>();
    REQUIRE(r.size() == 9);
    for (const char* e : {"Kalman_f64", "Kalman_f32"}) {
        const std::string k = e;
        // predict and update read their results at fixed points and across the diagonal: affine.
        CHECK(r.at(k + ".predict").stages[0] == cd::shape::affine);
        CHECK(r.at(k + ".update").stages[0] == cd::shape::affine);
        CHECK(r.at(k + ".prior").stages[0] == cd::shape::uniform);
        CHECK(r.at(k + ".posterior").stages[0] == cd::shape::uniform);
    }
    CHECK(r.at("Cast_f64_f32_fixes4x2.apply").stages[0] == cd::shape::uniform);
}

TEST_CASE("compute: the Kalman filter's counts are their closed forms", "[workbench][compute][kalman][W13]") {
    using cd::etype;
    using cd::op;
    const auto r = check_all<wb::kalman::pattern>();
    const cd::report& p = r.at("Kalman_f64.predict");
    // dt: the time's difference (i64), two casts and a scaling (f64). Q: q·dt³/3,
    // q·dt²/2, q·dt. F·x: 4·4 multiply-adds; F·P and (F·P)·Fᵀ: 4³ each; Q added
    // to the 6 elements of P's upper triangle that it touches; and a select
    // for each element of x and of P's upper triangle (dt <= 0 keeps them).
    CHECK(p.count(op::sub, etype::i64) == 1);
    CHECK(p.count(op::cast) == 2);
    CHECK(p.count(op::mul, etype::f64) == 1 + (3 + 2 + 1) + 16 + 64 + 64);
    CHECK(p.count(op::add, etype::f64) == 16 + 64 + 64 + 6);
    CHECK(p.count(op::div, etype::f64) == 2);
    CHECK(p.count(op::sel_le) == 4 + 10);

    // One fix, in Joseph form, and the counts are at the bound of 4 fixes:
    //   S: 2 adds, and det = s00·s11 − s01²: 2 multiplies, 1 subtraction;
    //   S⁻¹: 4 divisions, 2 negations;
    //   K = P·Hᵀ·S⁻¹: 4·2·2 multiply-adds;
    //   ν = z − H·x: 2 subtractions; K·ν: 4·2 multiply-adds; x + K·ν: 4 adds;
    //   K·H: 4·4·2 multiply-adds; A = I − K·H: 16 subtractions;
    //   A·P and (A·P)·Aᵀ: 4³ multiply-adds each; K·Kᵀ: 4·4·2;
    //   A·P·Aᵀ + r·K·Kᵀ on P's upper triangle: 10 multiplies and 10 adds.
    const std::uint64_t fixes = 4;
    const cd::report& u = r.at("Kalman_f64.update");
    CHECK(u.count(op::mul, etype::f64) == fixes * (2 + 16 + 8 + 32 + 64 + 64 + 32 + 10));
    CHECK(u.count(op::add, etype::f64) == fixes * (2 + 16 + 8 + 4 + 32 + 64 + 64 + 32 + 10));
    CHECK(u.count(op::sub, etype::f64) == fixes * (1 + 2 + 16));
    CHECK(u.count(op::div, etype::f64) == fixes * 4);
    CHECK(u.count(op::neg, etype::f64) == fixes * 2);
    // The f32 filter computes the same, in f32, but for dt's scaling in f64.
    const cd::report& p32 = r.at("Kalman_f32.predict");
    CHECK(p32.count(op::mul, etype::f32) == (3 + 2 + 1) + 16 + 64 + 64);
    CHECK(p32.count(op::mul, etype::f64) == 1);
    CHECK(r.at("Kalman_f32.update").count(op::mul, etype::f32) == u.count(op::mul, etype::f64));
    CHECK(r.at("Cast_f64_f32_fixes4x2.apply").count(op::cast) == 4 * 2);
}

TEST_CASE("compute: the Kalman filter's declarations give what its handlers give",
          "[workbench][compute][kalman][W11][W12]") {
    // Some updates with several fixes, and some with none.
    wb::kalman::config c;
    c.max_fixes = 4;
    c.p_none = 0.2;
    recorder rec;
    (void)wb::kalman::hand_run(c, 400, rec);
    require_clean(rec);
    REQUIRE(rec.worst.size() == 9);
    for (const auto& [who, d] : rec.worst) {
        INFO(who << ": " << static_cast<double>(d));
        // The products' sums run in the order the handler's MTL5 products run them.
        CHECK(d == 0);
    }
    CHECK(rec.firings.at("Kalman_f64.update") > 30);
}

// ── The DSP pipeline ────────────────────────────────────────────────

TEST_CASE("compute: the DSP pipeline's handlers pass the checks, and the FFT isn't affine",
          "[workbench][compute][dsp][W10]") {
    const auto r = check_all<wb::dsp::pattern>();
    REQUIRE(r.size() == 6);
    // The butterfly's partner, p + 2^(s-1), and the bit-reversed input.
    CHECK(r.at("Fft_c128_256.apply").stages[0] == cd::shape::non_affine);
    // The FIR reads the old line one tap back, a state's read: its variables' reads are uniform.
    CHECK(r.at("Fir_f64_63.filter").stages[0] == cd::shape::uniform);
    REQUIRE(r.at("Framer_f64_256.apply").stages.size() == 2);
    for (const char* who : {"Hann_f64_256.apply", "Cast_f64_c128_256.apply", "Magnitude_c128_129.apply"}) {
        INFO(who);
        CHECK(r.at(who).stages[0] == cd::shape::uniform);
        CHECK(r.at(who).depth == 1);
    }
}

TEST_CASE("compute: the DSP pipeline's counts are their closed forms", "[workbench][compute][dsp][W13]") {
    using cd::etype;
    using cd::op;
    namespace d = wb::dsp;
    const auto r = check_all<d::pattern>();
    // 63 multiply-adds per sample, and a reduction of L terms is L + 2 deep.
    const cd::report& fir = r.at("Fir_f64_63.filter");
    CHECK(fir.count(op::mul, etype::f64) == d::kTaps);
    CHECK(fir.count(op::add, etype::f64) == d::kTaps);
    CHECK(fir.depth == d::kTaps + 2);
    // (N/2)·log2 N butterflies: a complex multiply, an add and a subtraction each.
    constexpr std::uint64_t log2n = std::countr_zero(d::kFrame);
    constexpr std::uint64_t butterflies = d::kFrame / 2 * log2n;
    const cd::report& fft = r.at("Fft_c128_256.apply");
    CHECK(fft.count(op::mul, etype::c128) == butterflies);
    CHECK(fft.count(op::add, etype::c128) == butterflies);
    CHECK(fft.count(op::sub, etype::c128) == butterflies);
    CHECK(fft.count(op::mul) + fft.count(op::add) + fft.count(op::sub) == 3 * butterflies);
    // The input, then two definitions per stage (t, then y), then the write.
    CHECK(fft.depth == 2 * log2n + 2);
    CHECK(r.at("Hann_f64_256.apply").count(op::mul, etype::f64) == d::kFrame);
    CHECK(r.at("Cast_f64_c128_256.apply").count(op::cast) == d::kFrame);
    CHECK(r.at("Magnitude_c128_129.apply").count(op::cabs) == d::kBins);
    CHECK(r.at("Framer_f64_256.apply").counts.empty());
}

TEST_CASE("compute: the DSP pipeline's declarations give what its handlers give, exactly",
          "[workbench][compute][dsp][W11][W12]") {
    const wb::dsp::config c;
    recorder rec;
    (void)wb::dsp::hand_run(c, 3 * (wb::dsp::kFrame + 1), rec);
    require_clean(rec);
    REQUIRE(rec.worst.size() == 6);
    for (const auto& [who, d] : rec.worst) {
        INFO(who << ": " << static_cast<double>(d));
        CHECK(d == 0);
    }
    CHECK(rec.firings.at("Fir_f64_63.filter") == 3 * wb::dsp::kFrame);
    CHECK(rec.firings.at("Fft_c128_256.apply") == 3);
}

TEST_CASE("compute: a short window's frame, and its drops, are declared too", "[workbench][compute][dsp][W11]") {
    namespace d = wb::dsp;
    d::Window w;
    for (std::size_t k = 0; k < 100; ++k) {
        w.items[k]() = static_cast<double>(k) - 49.5;
    }
    w.count = 100;
    w.dropped = 3;
    const d::Real hann = d::hann_of();
    for (const std::size_t count : {std::size_t{0}, std::size_t{1}, std::size_t{100}}) {
        w.count = count;
        INFO("count " << count);
        const ev::firing framed = ev::fire_both<d::Framer64, "apply">({}, Timestamp{}, w);
        REQUIRE(framed.problems.empty());
        for (const auto& [name, h] : framed.handler) {
            INFO(name);
            CHECK(ev::difference(framed.declared.at(name), h) == 0);
        }
        const auto [frame, dropped] = d::FramerBinding::apply::call({}, w);
        CHECK(dropped() == 3);
        CHECK(frame.count() == count);
        const ev::firing windowed = ev::fire_both<d::Hann64, "apply">({}, Timestamp{}, frame, hann);
        REQUIRE(windowed.problems.empty());
        CHECK(ev::difference(windowed.declared.at("y"), windowed.handler.at("y")) == 0);
    }
}

TEST_CASE("compute: a declaration that computes something else is caught", "[workbench][compute][W11]") {
    // Linear_f64_8x32's declaration, but each row's sum seeded with 1, not 0.
    using B = wb::mlp::LinearBinding<double, wb::mlp::kOut, wb::mlp::kHidden>;
    static constexpr cd::system seeded_one = [] {
        cd::system s;
        const auto cols = s.extent("w", 1);
        const auto i = s.index("i", 0, s.extent("w", 0));
        const auto j = s.index("j", 0, cols);
        const auto acc = s.var("acc", cd::etype::f64);
        s.init(acc, s.literal(1, cd::etype::f64), s.face(j == -1));
        s.eq(acc, acc(i, j - 1) + s.in("w")(i, j) * s.in("x")(j));
        s.write(s.out("y").at(i), acc(i, j), s.where(j == cols - 1));
        return s;
    }();
    const wb::mlp::config c;
    const auto w = wb::mlp::weights_of(c);
    const auto h = wb::mlp::hand_run(w, wb::mlp::input_of(c, 0)).hidden;
    const auto x = wb::mlp::CastBinding<float, double, wb::mlp::kHidden>::apply::call({}, h);
    const ev::inputs in = ev::inputs_of<wb::mlp::Linear2, "apply">({}, Timestamp{}, w.w2, x);
    const ev::outputs o = ev::evaluate(seeded_one, cd::table_of<B, B::apply>(), in);
    REQUIRE(o.problems.empty());
    const auto want = ev::to_tensor(B::apply::call({}, w.w2, x));
    CHECK(ev::difference(o.ports.at("y"), want) > 0.01);
}

// ── PID control ─────────────────────────────────────────────────────

TEST_CASE("compute: PID control's handlers pass the checks, at one point each", "[workbench][compute][pid][W10]") {
    const auto r = check_all<wb::pid::pattern>();
    REQUIRE(r.size() == 4);
    for (const auto& [who, rep] : r) {
        INFO(who);
        REQUIRE(rep.stages.size() == 1);
        CHECK(rep.stages[0] == cd::shape::uniform);
    }
    // e, then the integral and the difference, then the command.
    CHECK(r.at("PidPID_f64.step").depth == 3);
    // The plant's next value, then its two writes.
    CHECK(r.at("Plant_f64.step").depth == 2);
}

TEST_CASE("compute: PID control's counts are its handlers' operations", "[workbench][compute][pid][W13]") {
    using cd::etype;
    using cd::op;
    const auto r = check_all<wb::pid::pattern>();
    // e = r − y; I + h·e; (e − e_last)/h; kp·e + ki·I + kd·D. Every controller
    // computes all three terms: the P and PI controllers' other gains are zero.
    for (const char* who : {"PidP_f64.step", "PidPI_f64.step", "PidPID_f64.step"}) {
        INFO(who);
        const cd::report& c = r.at(who);
        CHECK(c.count(op::sub, etype::f64) == 2);
        CHECK(c.count(op::mul, etype::f64) == 4);
        CHECK(c.count(op::add, etype::f64) == 3);
        CHECK(c.count(op::div, etype::f64) == 1);
    }
    // a·x + b·u.
    const cd::report& p = r.at("Plant_f64.step");
    CHECK(p.count(op::mul, etype::f64) == 2);
    CHECK(p.count(op::add, etype::f64) == 1);
}

TEST_CASE("compute: PID control's declarations give what its handlers give, exactly",
          "[workbench][compute][pid][W11][W12]") {
    const wb::pid::config c;
    recorder rec;
    (void)wb::pid::hand_run(c, 300, rec);
    require_clean(rec);
    REQUIRE(rec.worst.size() == 4);
    for (const auto& [who, d] : rec.worst) {
        INFO(who << ": " << static_cast<double>(d));
        CHECK(d == 0);
    }
    CHECK(rec.firings.at("PidPID_f64.step") == 300);
    CHECK(rec.firings.at("Plant_f64.step") == 3 * 300);
}
