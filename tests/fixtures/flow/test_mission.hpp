#pragma once
// A test mission (docs/plan/flow-graph), for the graph builder's tests:
// inputs at three rates, a SITL-only truth, a constant, the tick, and four
// composites. The frame region reads the estimate before the estimator takes
// the frame; the tick region holds the marks, which fire a trigger of their
// own. Its messages and operators are empty types: the builder reads only
// their domains and their bindings.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/domains.hpp>

#include <tuple>

namespace test {

namespace fl = branes::anatomy::flow;
using branes::anatomy::CameraFrame;
using branes::anatomy::Timestamp;

// ── Messages ────────────────────────────────────────────────────────

struct Imu {};
struct Marks {};
struct Truth {};
struct Estimate {};
struct Limits {};
struct Alerts {};
struct Command {};

}  // namespace test

namespace branes::anatomy::flow {

template <>
struct domain_of<test::Imu> {
    using type = vector<double, dim<6>>;
};
template <>
struct domain_of<test::Marks> {
    using type = vector<double, bounded<8>>;
};
template <>
struct domain_of<test::Truth> {
    using type = vector<double, dim<9>>;
};
template <>
struct domain_of<test::Estimate> {
    using type = record<field<"p", vector<double, dim<3>>>, field<"v", vector<double, dim<3>>>>;
};
template <>
struct domain_of<test::Limits> {
    using type = record<field<"ceiling", scalar<double>>, field<"rate", scalar<double>>>;
};
template <>
struct domain_of<test::Alerts> {
    using type = vector<int, bounded<4>>;
};
template <>
struct domain_of<test::Command> {
    using type = vector<double, dim<4>>;
};

}  // namespace branes::anatomy::flow

namespace test {

using fl::direction;
using fl::domain_of_t;
using fl::ins;
using fl::outs;
using fl::port;
using fl::state_role;

using Setpoint = fl::vector<double, fl::dim<2>>;

// ── Bindings ────────────────────────────────────────────────────────

struct EstimatorBinding {
    using signature = fl::signature<fl::ports<port<"imu", domain_of_t<Imu>, direction::in>,
                                              port<"frame", domain_of_t<CameraFrame>, direction::in>,
                                              port<"estimate", domain_of_t<Estimate>, direction::out>>>;
    using handlers =
        fl::handlers<fl::handler<"imu", ins<"imu">, outs<>, state_role::update>,
                     fl::handler<"frame", ins<"frame">, outs<>, state_role::update>,
                     fl::handler<"estimate", ins<>, outs<"estimate">, state_role::read, fl::takes_time::yes>>;
};

struct MonitorBinding {
    using signature = fl::signature<fl::ports<port<"estimate", domain_of_t<Estimate>, direction::in>,
                                              port<"alerts", domain_of_t<Alerts>, direction::out>>>;
    using handlers = fl::handlers<fl::handler<"update", ins<"estimate">, outs<>, state_role::update>,
                                  fl::handler<"alerts", ins<"estimate">, outs<"alerts">, state_role::read>>;
};

struct ControllerBinding {
    using signature = fl::signature<fl::ports<port<"estimate", domain_of_t<Estimate>, direction::in>,
                                              port<"limits", domain_of_t<Limits>, direction::in>,
                                              port<"alerts", domain_of_t<Alerts>, direction::in>,
                                              port<"marks", domain_of_t<Marks>, direction::in>,
                                              port<"command", domain_of_t<Command>, direction::out>>>;
    using handlers = fl::handlers<fl::handler<"step", ins<"estimate", "limits", "alerts", "marks">, outs<"command">>>;
};

struct SafetyFilterBinding {
    using signature = fl::signature<fl::ports<port<"estimate", domain_of_t<Estimate>, direction::in>,
                                              port<"command", domain_of_t<Command>, direction::in>,
                                              port<"ceiling", fl::scalar<double>, direction::in>,
                                              port<"safe_command", domain_of_t<Command>, direction::out>>>;
    using handlers = fl::handlers<fl::handler<"filter", ins<"estimate", "command", "ceiling">, outs<"safe_command">>>;
};

// Operators of their own, for a body of several operators and a research graph.

struct GainBinding {
    using signature = fl::signature<fl::ports<port<"estimate", domain_of_t<Estimate>, direction::in>,
                                              port<"limits", domain_of_t<Limits>, direction::in>,
                                              port<"setpoint", Setpoint, direction::out>>>;
    using handlers = fl::handlers<fl::handler<"apply", ins<"estimate", "limits">, outs<"setpoint">>>;
};

/// Gain, but its handler updates its state: a body may not, where its
/// composite handler doesn't (G5).
struct StatefulGainBinding {
    using signature = GainBinding::signature;
    using handlers =
        fl::handlers<fl::handler<"apply", ins<"estimate", "limits">, outs<"setpoint">, state_role::update>>;
};

struct TrackBinding {
    using signature = fl::signature<fl::ports<port<"estimate", domain_of_t<Estimate>, direction::in>,
                                              port<"setpoint", Setpoint, direction::in>,
                                              port<"alerts", domain_of_t<Alerts>, direction::in>,
                                              port<"marks", domain_of_t<Marks>, direction::in>,
                                              port<"command", domain_of_t<Command>, direction::out>>>;
    using handlers = fl::handlers<fl::handler<"step", ins<"estimate", "setpoint", "alerts", "marks">, outs<"command">>>;
};

/// An estimator's filter: what a body of the estimator fires, its read
/// untimed, so the body's timed estimate fires it at each of its inputs.
struct FilterBinding {
    using signature = EstimatorBinding::signature;
    using handlers = fl::handlers<fl::handler<"imu", ins<"imu">, outs<>, state_role::update>,
                                  fl::handler<"frame", ins<"frame">, outs<>, state_role::update>,
                                  fl::handler<"read", ins<>, outs<"estimate">, state_role::read>>;
};

struct TruthCheckBinding {
    using signature = fl::signature<fl::ports<port<"truth", domain_of_t<Truth>, direction::in>,
                                              port<"alerts", domain_of_t<Alerts>, direction::out>>>;
    using handlers = fl::handlers<fl::handler<"check", ins<"truth">, outs<"alerts">>>;
};

// ── Operators ───────────────────────────────────────────────────────

struct TestEstimator {};
struct NullMonitor {};
struct PassController {};
struct NullSafetyFilter {};
struct Gain {};
struct Track {};
struct TruthCheck {};
struct Filter {};

using TestEstimatorOp = fl::registered<"TestEstimator", TestEstimator, EstimatorBinding>;
using NullMonitorOp = fl::registered<"NullMonitor", NullMonitor, MonitorBinding>;
using PassControllerOp = fl::registered<"PassController", PassController, ControllerBinding>;
using NullSafetyFilterOp = fl::registered<"NullSafetyFilter", NullSafetyFilter, SafetyFilterBinding>;
using GainOp = fl::registered<"Gain", Gain, GainBinding>;
using StatefulGainOp = fl::registered<"StatefulGain", Gain, StatefulGainBinding>;
using TrackOp = fl::registered<"Track", Track, TrackBinding>;
using TruthCheckOp = fl::registered<"TruthCheck", TruthCheck, TruthCheckBinding>;
using FilterOp = fl::registered<"Filter", Filter, FilterBinding>;

/// A body of the controller: Gain makes a setpoint, which Track follows.
template <class G = GainOp>
struct GainTrack {
    static constexpr std::string_view name = "GainTrack";
    using binding = ControllerBinding;

    static void build(fl::body<ControllerBinding>& b) {
        const auto gain = fl::instance<G>(b, "gain");
        const auto track = fl::instance<TrackOp>(b, "track");
        fl::on<"step">(b, [=](fl::scope& s, auto estimate, auto limits, auto alerts, auto marks) {
            auto [setpoint] = fl::fire<"apply">(s, gain, estimate, limits);
            auto [command] = fl::fire<"step">(s, track, estimate, setpoint, alerts, marks);
            return std::tuple{command};
        });
    }
};

/// A body of the estimator, whose timed estimate fires an untimed read: the
/// mission fires the estimate at the frame and at the tick, so the body's
/// read fires once in each.
struct FilteredEstimator {
    static constexpr std::string_view name = "FilteredEstimator";
    using binding = EstimatorBinding;

    static void build(fl::body<EstimatorBinding>& b) {
        const auto filter = fl::instance<FilterOp>(b, "filter");
        fl::on<"imu">(b, [=](fl::scope& s, auto imu) {
            fl::fire<"imu">(s, filter, imu);
            return std::tuple<>{};
        });
        fl::on<"frame">(b, [=](fl::scope& s, auto frame) {
            fl::fire<"frame">(s, filter, frame);
            return std::tuple<>{};
        });
        fl::on<"estimate">(b, [=](fl::scope& s, const fl::time_of&) { return fl::fire<"read">(s, filter); });
    }
};

// ── The mission ─────────────────────────────────────────────────────

struct Ports {
    fl::input_port<Imu> imu;
    fl::input_port<CameraFrame> frame;
    fl::input_port<Marks> marks;
    fl::input_port<Truth> truth;
    fl::input_port<Timestamp> tick;
    fl::constant_port<Limits> limits;
    fl::output_port<Estimate> estimate;
    fl::output_port<Command> command;
};

/// The external ports.
inline Ports ports(fl::builder& g) {
    Ports p;
    p.imu = fl::input<Imu>(g, "imu");
    p.frame = fl::input<CameraFrame>(g, "frame", {.trigger = "frame", .time = "header.t_mid", .sitl_only = false});
    p.marks = fl::input<Marks>(g, "marks");
    p.truth = fl::input<Truth>(g, "truth", {.trigger = "truth", .time = "", .sitl_only = true});
    p.tick = g.tick("tick");
    p.limits = fl::constant<Limits>(g, "limits", {.from = "limits"});
    p.estimate = fl::output<Estimate>(g, "estimate", p.tick);
    p.command = fl::output<Command>(g, "command", p.tick);
    return p;
}

/// A body for each composite: a registered operator, or a body of several.
template <class Estimator = TestEstimatorOp,
          class Monitor = NullMonitorOp,
          class Controller = PassControllerOp,
          class SafetyFilter = NullSafetyFilterOp>
struct Bodies {
    using estimator = Estimator;
    using monitor = Monitor;
    using controller = Controller;
    using safety_filter = SafetyFilter;
};

struct Composites {
    fl::node<EstimatorBinding> estimator;
    fl::node<MonitorBinding> monitor;
    fl::node<ControllerBinding> controller;
    fl::node<SafetyFilterBinding> safety_filter;
};

template <class B>
Composites composites(fl::builder& g) {
    return Composites{fl::composite<EstimatorBinding, typename B::estimator>(g, "estimator"),
                      fl::composite<MonitorBinding, typename B::monitor>(g, "monitor"),
                      fl::composite<ControllerBinding, typename B::controller>(g, "controller"),
                      fl::composite<SafetyFilterBinding, typename B::safety_filter>(g, "safety_filter")};
}

/// The mission's regions, over its composites.
inline void regions(fl::builder& g, const Ports& p, const Composites& c) {
    fl::scope imu = g.region(p.imu);
    fl::fire<"imu">(imu, c.estimator, p.imu);

    fl::scope frame = g.region(p.frame);
    auto [prior] = fl::fire<"estimate">(frame, c.estimator, fl::at(p.frame));
    fl::fire<"frame">(frame, c.estimator, p.frame);
    fl::fire<"update">(frame, c.monitor, prior);

    fl::scope tick = g.region(p.tick);
    auto [estimate] = fl::fire<"estimate">(tick, c.estimator, fl::at(p.tick));
    auto [alerts] = fl::fire<"alerts">(tick, c.monitor, estimate);
    auto [command] = fl::fire<"step">(tick, c.controller, estimate, p.limits, alerts, fl::hold(p.marks));
    auto [safe] = fl::fire<"filter">(tick, c.safety_filter, estimate, command, fl::project<"ceiling">(p.limits));
    g.produce(p.estimate, estimate);
    g.produce(p.command, safe);
}

/// A scoreboard graph of the test mission.
template <class B = Bodies<>>
void scoreboard(fl::builder& g) {
    const Ports p = ports(g);
    regions(g, p, composites<B>(g));
}

/// A research graph of the test mission: operators of its own between the
/// mission's external ports, which it returns.
inline Ports research(fl::builder& g) {
    const Ports p = ports(g);
    const auto estimator = fl::instance<TestEstimatorOp>(g, "estimator");
    const auto monitor = fl::instance<NullMonitorOp>(g, "monitor");
    const auto gain = fl::instance<GainOp>(g, "gain");
    const auto track = fl::instance<TrackOp>(g, "track");

    fl::scope imu = g.region(p.imu);
    fl::fire<"imu">(imu, estimator, p.imu);

    fl::scope frame = g.region(p.frame);
    auto [prior] = fl::fire<"estimate">(frame, estimator, fl::at(p.frame));
    fl::fire<"frame">(frame, estimator, p.frame);
    fl::fire<"update">(frame, monitor, prior);

    fl::scope tick = g.region(p.tick);
    auto [estimate] = fl::fire<"estimate">(tick, estimator, fl::at(p.tick));
    auto [alerts] = fl::fire<"alerts">(tick, monitor, estimate);
    auto [setpoint] = fl::fire<"apply">(tick, gain, estimate, p.limits);
    auto [command] = fl::fire<"step">(tick, track, estimate, setpoint, alerts, fl::hold(p.marks));
    g.produce(p.estimate, estimate);
    g.produce(p.command, command);
    return p;
}

}  // namespace test
