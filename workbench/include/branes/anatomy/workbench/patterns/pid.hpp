#pragma once
/// @file pid.hpp
/// @brief The PID control pattern: three loops closed through carried edges
/// (docs/plan/computational-domain, C4).
///
/// The plant is first order, y' = (K·u − y)/τ, discretized exactly for a
/// zero-order hold of period h: x ← a·x + b·u, with a = exp(−h/τ) and
/// b = K·(1 − a). Its state x is its output y. The controller is a PID on
/// e = r − y, with a rectangle-rule integral and a backward difference:
///   e = r − y,  I ← I + h·e,  D = (e − e_last)/h,  e_last ← e,
///   u = kp·e + ki·I + kd·D.
/// Its integral I and its last error e_last are its declared state.
///
/// One region, on a 100 Hz clock, runs three loops: P (ki = kd = 0), PI
/// (kd = 0) and PID. In each, the controller reads the plant's output from
/// the previous tick, through a carried edge, and the plant reads the
/// controller's command from this one:
///
///   const auto y_last = fl::carried<Scalar>(region);
///   auto [u] = fl::fire<"step">(region, controller, setpoint, y_last);
///   auto [y] = fl::fire<"step">(region, plant, u);
///   fl::close(region, y_last, y);
///
/// At tick k the controller reads y_k, the plant's output at tick k − 1, or
/// 0 at k = 0 (the carried edge's message value-initialized), and the plant
/// writes y_{k+1} = a·y_k + b·u_k. The setpoint r is a constant step.
///
/// Oracle (W2), in long double, from the gains alone:
///   the P loop is y_{k+1} = λ·y_k + b·kp·r with λ = a − b·kp, so
///     y_k = y∞·(1 − λᵏ) with y∞ = b·kp·r/(1 − λ): its outputs agree to 1e-12;
///   the PI and PID loops' steady-state errors, after 20 plant time
///     constants, are below 1e-9;
///   the PI loop's poles, the eigenvalues of its 2 × 2 closed-loop matrix,
///     give its step response in modal form, whose peak predicts the loop's
///     overshoot to a relative 1 %; and the loop is underdamped (at least 5 %).

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
#include <complex>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::anatomy::workbench::pid {

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

using ScalarD = fl::scalar<double>;
using Scalar = fl::data<ScalarD>;

// ── Configuration ───────────────────────────────────────────────────

struct config {
    std::uint64_t seed = 1;                     ///< the pattern draws nothing: every seed runs the same loops
    std::int64_t clock_period_ns = 10'000'000;  ///< 100 Hz
    double setpoint = 1.0;                      ///< r, a step at t = 0
    double gain = 2.0;                          ///< the plant's K
    double tau = 0.5;                           ///< the plant's time constant τ [s]
    double kp = 1.2;
    double ki = 20.0;  ///< [1/s]
    double kd = 0.05;  ///< [s]

    [[nodiscard]] double h() const noexcept {
        return static_cast<double>(clock_period_ns) * 1e-9;
    }
};

struct context {
    config cfg;
};

/// A scalar message of value v.
[[nodiscard]] inline Scalar scalar(double v) noexcept {
    Scalar s;
    s() = v;
    return s;
}

// ── The operators ───────────────────────────────────────────────────

/// Which of the controller's terms are on: the others' gains are zero.
enum class terms : std::uint8_t { p, pi, pid };

/// A PID controller of period h, with the gains its terms take from the configuration.
template <terms T>
struct Pid {
    fl::data<ScalarD> integral;  ///< I
    fl::data<ScalarD> e_last;    ///< the last error, 0 before the first
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double h = 0;

    [[nodiscard]] static result<Pid, std::string> from_context(const context& c) {
        if (!(c.cfg.clock_period_ns > 0)) {
            return std::string("pid: the clock's period is positive");
        }
        Pid p;
        p.kp = c.cfg.kp;
        p.ki = T == terms::p ? 0.0 : c.cfg.ki;
        p.kd = T == terms::pid ? c.cfg.kd : 0.0;
        p.h = c.cfg.h();
        return p;
    }

    [[nodiscard]] Scalar step(const Scalar& r, const Scalar& y) {
        const double e = r() - y();
        integral() = integral() + h * e;
        const double d = (e - e_last()) / h;
        e_last() = e;
        return scalar(kp * e + ki * integral() + kd * d);
    }
};

/// The first-order plant, discretized exactly: x ← a·x + b·u, and y = x.
struct Plant {
    fl::data<ScalarD> x;
    double a = 0;
    double b = 0;

    [[nodiscard]] static result<Plant, std::string> from_context(const context& c) {
        if (!(c.cfg.tau > 0)) {
            return std::string("pid: the plant's time constant is positive");
        }
        Plant p;
#ifdef _MSC_VER
        // tau > 0 here, but MSVC's optimizer, given a test's tau of 0, warns of the division anyway.
#pragma warning(suppress : 4723)
#endif
        p.a = std::exp(-c.cfg.h() / c.cfg.tau);
        p.b = c.cfg.gain * (1.0 - p.a);
        return p;
    }

    [[nodiscard]] Scalar step(const Scalar& u) {
        x() = a * x() + b * u();
        return x;
    }
};

namespace declare {

/// The controller's step: one point, with no index. Each line of the handler
/// is a variable, so the declaration computes in the handler's order.
constexpr cd::system controller() {
    constexpr cd::etype e = cd::etype::f64;
    cd::system s;
    const auto integral = s.state("integral");
    const auto e_last = s.state("e_last");
    const auto h = s.param("h", e);
    const auto err = s.var("e", e);
    const auto i_new = s.var("i_new", e);
    const auto d = s.var("d", e);
    s.eq(err, s.in("r")() - s.in("y")());
    s.eq(i_new, integral() + h() * err());
    s.eq(d, (err() - e_last()) / h());
    s.write(integral.at(), i_new());
    s.write(e_last.at(), err());
    s.write(s.out("u").at(), s.param("kp", e)() * err() + s.param("ki", e)() * i_new() + s.param("kd", e)() * d());
    return s;
}

/// The plant's step: x ← a·x + b·u, written to the state and to y.
constexpr cd::system plant() {
    constexpr cd::etype e = cd::etype::f64;
    cd::system s;
    const auto x = s.state("x");
    const auto next = s.var("next", e);
    s.eq(next, s.param("a", e)() * x() + s.param("b", e)() * s.in("u")());
    s.write(x.at(), next());
    s.write(s.out("y").at(), next());
    return s;
}

}  // namespace declare

template <terms T>
struct PidBinding {
    using signature = fl::signature<fl::ports<fl::port<"r", Scalar, fl::direction::in>,
                                              fl::port<"y", Scalar, fl::direction::in>,
                                              fl::port<"u", Scalar, fl::direction::out>>,
                                    fl::states<fl::state<"integral", ScalarD>, fl::state<"e_last", ScalarD>>>;
    struct step : fl::handler<"step", fl::ins<"r", "y">, fl::outs<"u">, fl::state_role::update> {
        static constexpr cd::system compute = declare::controller();
        static Scalar call(Pid<T>& op, const Scalar& r, const Scalar& y) {
            return op.step(r, y);
        }
    };
    using handlers = fl::handlers<step>;
    using cd_state = cd::members<cd::member<"integral", &Pid<T>::integral>, cd::member<"e_last", &Pid<T>::e_last>>;
    using cd_params = cd::members<cd::member<"kp", &Pid<T>::kp>,
                                  cd::member<"ki", &Pid<T>::ki>,
                                  cd::member<"kd", &Pid<T>::kd>,
                                  cd::member<"h", &Pid<T>::h>>;
};

struct PlantBinding {
    using signature =
        fl::signature<fl::ports<fl::port<"u", Scalar, fl::direction::in>, fl::port<"y", Scalar, fl::direction::out>>,
                      fl::states<fl::state<"x", ScalarD>>>;
    struct step : fl::handler<"step", fl::ins<"u">, fl::outs<"y">, fl::state_role::update> {
        static constexpr cd::system compute = declare::plant();
        static Scalar call(Plant& op, const Scalar& u) {
            return op.step(u);
        }
    };
    using handlers = fl::handlers<step>;
    using cd_state = cd::members<cd::member<"x", &Plant::x>>;
    using cd_params = cd::members<cd::member<"a", &Plant::a>, cd::member<"b", &Plant::b>>;
};

using P64 = fl::registered<"PidP_f64", Pid<terms::p>, PidBinding<terms::p>>;
using PI64 = fl::registered<"PidPI_f64", Pid<terms::pi>, PidBinding<terms::pi>>;
using PID64 = fl::registered<"PidPID_f64", Pid<terms::pid>, PidBinding<terms::pid>>;
using Plant64 = fl::registered<"Plant_f64", Plant, PlantBinding>;

// ── The graph ───────────────────────────────────────────────────────

/// The loops, in the order the region runs them.
inline constexpr std::size_t kLoops = 3;
inline constexpr std::array<std::string_view, kLoops> kLoopNames{"p", "pi", "pid"};

struct ports {
    fl::input_port<Timestamp> clock;
    std::array<fl::output_port<Scalar>, kLoops> u;  ///< each controller's command
    std::array<fl::output_port<Scalar>, kLoops> y;  ///< each plant's output
};

namespace detail {
/// One loop in region `r`: the controller reads the setpoint and, through a
/// carried edge, the plant's last output; the plant reads the command.
template <class G, class C>
void loop(G& g,
          fl::scope& r,
          const fl::constant_port<Scalar>& setpoint,
          const fl::node<C>& controller,
          const fl::node<PlantBinding>& plant,
          const fl::output_port<Scalar>& out_u,
          const fl::output_port<Scalar>& out_y) {
    const auto y_last = fl::carried<Scalar>(r);
    auto [u] = fl::fire<"step">(r, controller, setpoint, y_last);
    auto [y] = fl::fire<"step">(r, plant, u);
    fl::close(r, y_last, y);
    g.produce(out_u, u);
    g.produce(out_y, y);
}
}  // namespace detail

template <class G>
[[nodiscard]] ports graph(G& g, const config& c) {
    const auto tick = g.tick("clock");
    ports p{tick, {}, {}};
    for (std::size_t l = 0; l < kLoops; ++l) {
        p.u[l] = fl::output<Scalar>(g, "u_" + std::string(kLoopNames[l]), tick);
        p.y[l] = fl::output<Scalar>(g, "y_" + std::string(kLoopNames[l]), tick);
    }
    const auto setpoint = fl::constant<Scalar>(g, "setpoint", scalar(c.setpoint));
    const auto cp = fl::instance<P64>(g, "controller_p");
    const auto plant_p = fl::instance<Plant64>(g, "plant_p");
    const auto cpi = fl::instance<PI64>(g, "controller_pi");
    const auto plant_pi = fl::instance<Plant64>(g, "plant_pi");
    const auto cpid = fl::instance<PID64>(g, "controller_pid");
    const auto plant_pid = fl::instance<Plant64>(g, "plant_pid");

    fl::scope r = g.region(tick);
    detail::loop(g, r, setpoint, cp, plant_p, p.u[0], p.y[0]);
    detail::loop(g, r, setpoint, cpi, plant_pi, p.u[1], p.y[1]);
    detail::loop(g, r, setpoint, cpid, plant_pid, p.u[2], p.y[2]);
    return p;
}

// ── Samples, the run by hand (W1), and the oracle (W2) ──────────────

/// After each tick: its time, and each loop's command and output.
struct sample {
    Timestamp t;
    std::array<double, kLoops> u{};
    std::array<double, kLoops> y{};

    friend bool operator==(const sample&, const sample&) = default;
};

/// The operators called by hand, tick by tick, in the graph's order; each
/// loop's carried edge is the plant's output from the tick before, 0 at first.
template <class Hook = no_hook>
[[nodiscard]] std::vector<sample> hand_run(const config& c, std::size_t inputs, Hook&& hook = {}) {
    const context ctx{c};
    auto cp = *Pid<terms::p>::from_context(ctx);
    auto cpi = *Pid<terms::pi>::from_context(ctx);
    auto cpid = *Pid<terms::pid>::from_context(ctx);
    std::array<Plant, kLoops> plants{*Plant::from_context(ctx), *Plant::from_context(ctx), *Plant::from_context(ctx)};
    std::array<Scalar, kLoops> y_last{};
    const Scalar r = scalar(c.setpoint);
    std::vector<sample> out;
    out.reserve(inputs);
    for (std::size_t k = 0; k < inputs; ++k) {
        const Timestamp t{static_cast<std::int64_t>(k) * c.clock_period_ns};
        sample s{t, {}, {}};
        const auto one = [&]<class Op>(std::size_t l, typename Op::type& controller) {
            hook.template before<Op, "step">(controller, t, r, y_last[l]);
            const Scalar u = Op::binding::step::call(controller, r, y_last[l]);
            hook.template before<Plant64, "step">(plants[l], t, u);
            const Scalar y = PlantBinding::step::call(plants[l], u);
            s.u[l] = u();
            s.y[l] = y();
            y_last[l] = y;
        };
        one.template operator()<P64>(0, cp);
        one.template operator()<PI64>(1, cpi);
        one.template operator()<PID64>(2, cpid);
        out.push_back(s);
    }
    return out;
}

namespace oracle_detail {

using cl = std::complex<long double>;

/// The plant's a and b, in long double.
struct discrete {
    long double a = 0;
    long double b = 0;
    long double h = 0;
};

[[nodiscard]] inline discrete discretized(const config& c) {
    const long double h = static_cast<long double>(c.clock_period_ns) * 1e-9L;
    const long double a = std::exp(-h / static_cast<long double>(c.tau));
    return discrete{a, static_cast<long double>(c.gain) * (1 - a), h};
}

/// The PI loop's output y_k, k ≥ 0, in modal form from its poles. Its state
/// is (y_k, I_{k−1}), and
///   y_{k+1} = (a − b·(kp + ki·h))·y_k + b·ki·I_{k−1} + b·(kp + ki·h)·r
///   I_k     = −h·y_k + I_{k−1} + h·r,
/// so z_k = (y_k − r, I_{k−1} − I∞) is A^k·z_0, with I∞ = (1 − a)·r/(b·ki).
/// A = V·diag(p)·V⁻¹ by its eigenvalues p and eigenvectors (A₀₁, p − A₀₀).
class pi_modes {
public:
    pi_modes(const config& c) {
        const discrete d = discretized(c);
        const long double kp = c.kp;
        const long double ki = c.ki;
        r_ = c.setpoint;
        const long double a00 = d.a - d.b * (kp + ki * d.h);
        const long double a01 = d.b * ki;
        const long double a10 = -d.h;
        const long double a11 = 1;
        const cl tr = a00 + a11;
        const cl det = a00 * a11 - a01 * a10;
        const cl disc = std::sqrt(tr * tr - 4.0L * det);
        p_ = {(tr + disc) / 2.0L, (tr - disc) / 2.0L};
        // V's columns (a01, p − a00); z_0 = V·w.
        const cl z0y = -r_;
        const cl z0i = -(1 - d.a) * r_ / (d.b * ki);
        const cl v10 = p_[0] - a00;
        const cl v11 = p_[1] - a00;
        const cl vdet = a01 * v11 - a01 * v10;
        w_ = {(v11 * z0y - a01 * z0i) / vdet, (-v10 * z0y + a01 * z0i) / vdet};
        a01_ = a01;
    }

    [[nodiscard]] const std::array<cl, 2>& poles() const noexcept {
        return p_;
    }

    /// y_k, from the poles' powers.
    [[nodiscard]] long double y(std::size_t k) const {
        const auto n = static_cast<long double>(k);
        const cl dev = a01_ * (w_[0] * std::pow(p_[0], n) + w_[1] * std::pow(p_[1], n));
        return r_ + dev.real();
    }

private:
    long double r_ = 0;
    std::array<cl, 2> p_{};
    std::array<cl, 2> w_{};
    long double a01_ = 0;
};

}  // namespace oracle_detail

/// The plant time constants after which the steady-state error is checked.
inline constexpr double kSettle = 20.0;

struct pattern {
    static constexpr std::string_view name = "pid";
    /// 15 s at 100 Hz: 30 plant time constants.
    static constexpr std::size_t default_inputs = 1500;
    using config = pid::config;
    using context = pid::context;
    using ports = pid::ports;
    using sample = pid::sample;
    using leaves = workbench::leaves<leaf<P64, "step">, leaf<PI64, "step">, leaf<PID64, "step">, leaf<Plant64, "step">>;

    template <class G>
    [[nodiscard]] static ports graph(G& g, const config& c) {
        return pid::graph(g, c);
    }

    [[nodiscard]] static std::vector<source> sources(const ports& p, const config& c) {
        return {clock(p.clock, c.clock_period_ns)};
    }

    [[nodiscard]] static sample collect(const fl::stack& s, const ports& p, std::size_t, std::size_t, Timestamp t) {
        sample out{t, {}, {}};
        for (std::size_t l = 0; l < kLoops; ++l) {
            out.u[l] = s.output(p.u[l])();
            out.y[l] = s.output(p.y[l])();
        }
        return out;
    }

    [[nodiscard]] static report oracle(const config& c, std::span<const sample> samples) {
        using namespace oracle_detail;
        report rep{std::string(name), {}};
        const discrete d = discretized(c);
        const long double r = c.setpoint;

        // The P loop: y_{k+1} = y∞·(1 − λ^{k+1}); sample k holds y_{k+1}.
        const long double lambda = d.a - d.b * static_cast<long double>(c.kp);
        const long double y_inf = d.b * static_cast<long double>(c.kp) * r / (1 - lambda);
        long double p_err = 0;
        for (std::size_t k = 0; k < samples.size(); ++k) {
            const long double want = y_inf * (1 - std::pow(lambda, static_cast<long double>(k + 1)));
            p_err = std::fmax(p_err, std::fabs(static_cast<long double>(samples[k].y[0]) - want));
        }
        rep.checks.push_back(at_least("ticks", static_cast<double>(samples.size()), 1));
        rep.checks.push_back(at_most("P loop against its closed form", static_cast<double>(p_err), 1e-12));

        // The steady-state errors after 20 time constants.
        const auto settled = static_cast<std::size_t>(kSettle * c.tau / c.h());
        for (std::size_t l = 1; l < kLoops; ++l) {
            long double worst = samples.size() > settled ? 0 : 1;
            for (std::size_t k = settled; k < samples.size(); ++k) {
                worst = std::fmax(worst, std::fabs(r - static_cast<long double>(samples[k].y[l])));
            }
            rep.checks.push_back(
                at_most(std::string(kLoopNames[l] == "pi" ? "PI" : "PID") + " loop's error after 20 time constants",
                        static_cast<double>(worst),
                        1e-9));
        }

        // The PI loop's overshoot: its peak against the modal form's.
        const pi_modes modes(c);
        long double peak = 0;
        long double predicted = 0;
        for (std::size_t k = 0; k < samples.size(); ++k) {
            peak = std::fmax(peak, static_cast<long double>(samples[k].y[1]));
            predicted = std::fmax(predicted, modes.y(k + 1));
        }
        const long double overshoot = (peak - r) / r;
        const long double expected = (predicted - r) / r;
        rep.checks.push_back(at_least("PI loop's overshoot", static_cast<double>(overshoot), 0.05));
        rep.checks.push_back(at_most("PI loop's overshoot against its poles', relative",
                                     static_cast<double>(std::fabs(overshoot - expected) / expected),
                                     0.01));
        return rep;
    }

    [[nodiscard]] static fl::trace_codecs codecs() {
        fl::trace_codecs c;
        add_data<ScalarD>(c);
        return c;
    }
};

}  // namespace branes::anatomy::workbench::pid
