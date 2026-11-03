#pragma once
// A neutral "field" mission as a graph (docs/plan/flow-graph, 2b), for the
// CPU stack's tests: a world with a goal and a geofence, the simulated truth
// as the estimate, a planner that aims at the goal, and the neutral
// composites. Its baselines are in this directory, and the generated headers
// name their bodies (cmake/flow.cmake). FieldSlots is the same stack wired by
// hand, as a slot stack is, for the logs to be compared with.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/glue/cascaded_pid.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/flow/bindings.hpp>
#include <branes/anatomy/mobility/flow/domains.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/mobility/slots/context.hpp>
#include <branes/anatomy/mobility/slots/ground_truth_core.hpp>
#include <branes/anatomy/mobility/slots/neutral_nulls.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/slots/slot_context.hpp>

#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace field {

namespace an = branes::anatomy;
namespace fl = branes::anatomy::flow;

/// The mission's world: the start pose, the goal and the geofence.
struct World {
    an::StartPose<double> start{{0.0, 0.0, 0.1}, 0.0};
    an::Vec3<double> goal{40.0, 0.0, 2.0};
    an::Geofence<double> geofence{{-50.0, -50.0, 0.0}, {50.0, 50.0, 20.0}};
};
static_assert(an::MissionWorld<World>);

using Context = an::BasicSlotContext<World>;

}  // namespace field

template <>
struct branes::anatomy::flow::domain_of<field::World> {
    using type = record<field<"start", domain_of_t<StartPose<double>>>,
                        field<"goal", vector<double, dim<3>>>,
                        field<"geofence", domain_of_t<Geofence<double>>>>;
};

namespace field {

using fl::direction;
using fl::ins;
using fl::outs;
using fl::port;
using fl::state_role;

// ── The mission's composites ────────────────────────────────────────

/// The estimator: the IMU and the truth in, the estimate at a time out.
template <class T>
struct EstimatorBinding {
    using signature = fl::signature<fl::ports<port<"imu", an::ImuSample<T>, direction::in>,
                                              port<"truth", an::TruthState<T>, direction::in>,
                                              port<"estimate", an::StateEstimate<T>, direction::out>>>;

    struct imu : fl::handler<"imu", ins<"imu">, outs<>, state_role::update> {
        static void call(auto& est, const an::ImuSample<T>& s) {
            est.process_imu(s);
        }
    };
    struct truth : fl::handler<"truth", ins<"truth">, outs<>, state_role::update> {
        static void call(auto& est, const an::TruthState<T>& x) {
            est.process_truth(x);
        }
    };
    struct estimate : fl::handler<"estimate", ins<>, outs<"estimate">, state_role::read, fl::takes_time::yes> {
        static an::StateEstimate<T> call(const auto& est, an::Timestamp t) {
            return est.state_at(t);
        }
    };

    using handlers = fl::handlers<imu, truth, estimate>;
};

/// The planner: the world and the estimate in, the reference out.
template <class T>
struct PlannerBinding {
    using signature = fl::signature<fl::ports<port<"field", World, direction::in>,
                                              port<"estimate", an::StateEstimate<T>, direction::in>,
                                              port<"reference", an::Reference<T>, direction::out>>>;

    struct plan : fl::handler<"plan", ins<"field", "estimate">, outs<"reference">> {
        static an::Reference<T> call(const auto& p, const World& w, const an::StateEstimate<T>& x) {
            return p.plan(w, x);
        }
    };

    using handlers = fl::handlers<plan>;
};

/// Aims at the goal, at the estimate's time.
template <class T>
class GoalPlanner {
public:
    [[nodiscard]] an::Reference<T> plan(const World& w, const an::StateEstimate<T>& x) const {
        an::Reference<T> r{};
        r.count = 1;
        r.samples[0].t = x.t_valid;
        r.samples[0].position = an::Vec3<T>{T(w.goal[0]), T(w.goal[1]), T(w.goal[2])};
        return r;
    }
};

// ── A body of several operators: the planner, in two steps ──────────

/// Smooth: a reference in, the same reference out. With GoalPlanner before
/// it, the body plans what GoalPlanner alone does.
template <class T>
struct SmoothBinding {
    using signature = fl::signature<fl::ports<port<"reference", an::Reference<T>, direction::in>,
                                              port<"estimate", an::StateEstimate<T>, direction::in>,
                                              port<"smoothed", an::Reference<T>, direction::out>>>;

    struct smooth : fl::handler<"smooth", ins<"reference", "estimate">, outs<"smoothed">> {
        static an::Reference<T> call(const auto& s, const an::Reference<T>& r, const an::StateEstimate<T>& x) {
            return s.smooth(r, x);
        }
    };

    using handlers = fl::handlers<smooth>;
};

template <class T>
class Smooth {
public:
    [[nodiscard]] an::Reference<T> smooth(const an::Reference<T>& r, const an::StateEstimate<T>&) const {
        return r;
    }
};

template <class T>
struct GoalSmooth {
    static constexpr std::string_view name = "GoalSmooth";
    using binding = PlannerBinding<T>;

    static void build(auto& b) {
        const auto goal = fl::instance<fl::registered<"GoalPlanner", GoalPlanner<T>, PlannerBinding<T>>>(b, "goal");
        const auto smooth = fl::instance<fl::registered<"Smooth", Smooth<T>, SmoothBinding<T>>>(b, "smooth");
        fl::on<"plan">(b, [=](fl::scope& s, auto world, auto estimate) {
            auto [reference] = fl::fire<"plan">(s, goal, world, estimate);
            auto [smoothed] = fl::fire<"smooth">(s, smooth, reference, estimate);
            return std::tuple{smoothed};
        });
    }
};

// ── The mission ─────────────────────────────────────────────────────

template <class T>
struct Ports {
    fl::input_port<an::ImuSample<T>> imu;
    fl::input_port<an::CameraFrame> camera;
    fl::input_port<an::TruthState<T>> truth;
    fl::input_port<an::Timestamp> tick;
    fl::constant_port<World> field;
    fl::constant_port<an::IntruderDetections<T>> detections;  ///< no producer yet: none
    fl::output_port<an::StateEstimate<T>> estimate;
    fl::output_port<an::Reference<T>> reference;
    fl::output_port<an::ThrustRates<T>> command;
};

template <class T>
Ports<T> ports(fl::builder& g, const World& world) {
    Ports<T> p;
    p.imu = fl::input<an::ImuSample<T>>(g, "imu");
    p.camera = fl::input<an::CameraFrame>(
        g,
        "camera",
        {.trigger = "frame", .time = [](const an::CameraFrame& f) { return f.header.t_mid; }, .sitl_only = false});
    p.truth = fl::input<an::TruthState<T>>(g, "truth", {.trigger = "truth", .time = nullptr, .sitl_only = true});
    p.tick = g.tick("tick");
    p.field = fl::constant<World>(g, "field", world);
    p.detections = fl::constant<an::IntruderDetections<T>>(g, "detections");
    p.estimate = fl::output<an::StateEstimate<T>>(g, "estimate", p.tick);
    p.reference = fl::output<an::Reference<T>>(g, "reference", p.tick);
    p.command = fl::output<an::ThrustRates<T>>(g, "command", p.tick);
    return p;
}

/// The scoreboard graph: the mission's composites, filled by Bodies.
template <class T, class Bodies, class G>
Ports<T> scoreboard(G& g, const World& world) {
    const Ports<T> p = ports<T>(g, world);
    const auto est = fl::composite<EstimatorBinding<T>, typename Bodies::estimator>(g, "estimator");
    const auto planner = fl::composite<PlannerBinding<T>, typename Bodies::planner>(g, "planner");
    const auto daa = fl::composite<fl::DaaTrackerBinding<T>, typename Bodies::daa_tracker>(g, "daa_tracker");
    const auto ctl = fl::composite<fl::ControllerBinding<T>, typename Bodies::controller>(g, "controller");
    const auto safe = fl::composite<fl::SafetyFilterBinding<T>, typename Bodies::safety_filter>(g, "safety_filter");
    const auto fc = fl::composite<fl::FcBridgeBinding<T>, typename Bodies::fc_bridge>(g, "fc_bridge");

    fl::scope imu = g.region(p.imu);
    fl::fire<"imu">(imu, est, p.imu);

    fl::scope truth = g.region(p.truth);
    fl::fire<"truth">(truth, est, p.truth);

    fl::scope frame = g.region(p.camera);
    auto [prior] = fl::fire<"estimate">(frame, est, fl::at(p.camera));
    fl::fire<"update">(frame, daa, p.detections, prior);

    fl::scope tick = g.region(p.tick);
    auto [x] = fl::fire<"estimate">(tick, est, fl::at(p.tick));
    auto [reference] = fl::fire<"plan">(tick, planner, p.field, x);
    auto [conflicts] = fl::fire<"conflicts">(tick, daa, x);
    auto [command] = fl::fire<"step">(tick, ctl, x, reference, conflicts);
    auto [safe_command] =
        fl::fire<"filter">(tick, safe, x, command, conflicts, fl::project<"geofence", &World::geofence>(p.field));
    fl::fire<"send">(tick, fc, fl::at(p.tick), safe_command);
    g.produce(p.estimate, x);
    g.produce(p.reference, reference);
    g.produce(p.command, safe_command);
    return p;
}

// ── A research graph ────────────────────────────────────────────────

/// Direct: plans and commands in one step, its own signature, with two outs.
template <class T>
struct DirectBinding {
    using signature = fl::signature<fl::ports<port<"field", World, direction::in>,
                                              port<"estimate", an::StateEstimate<T>, direction::in>,
                                              port<"reference", an::Reference<T>, direction::out>,
                                              port<"command", an::ThrustRates<T>, direction::out>>>;

    struct step : fl::handler<"step", ins<"field", "estimate">, outs<"reference", "command">> {
        static std::tuple<an::Reference<T>, an::ThrustRates<T>>
        call(const auto& d, const World& w, const an::StateEstimate<T>& x) {
            return d.step(w, x);
        }
    };

    using handlers = fl::handlers<step>;
};

/// GoalPlanner's reference and NullController's command, in one operator.
template <class T>
class Direct {
public:
    [[nodiscard]] std::tuple<an::Reference<T>, an::ThrustRates<T>> step(const World& w,
                                                                        const an::StateEstimate<T>& x) const {
        an::ThrustRates<T> u{};
        u.t = x.t_valid;
        return {GoalPlanner<T>{}.plan(w, x), u};
    }
};

template <class T>
struct DirectGraph {
    template <class G>
    static Ports<T> describe(G& g, const World& world) {
        const Ports<T> p = ports<T>(g, world);
        const auto est = fl::instance<fl::registered<"GroundTruthCore", an::GroundTruthCore<T>, EstimatorBinding<T>>>(
            g, "estimator");
        const auto direct = fl::instance<fl::registered<"Direct", Direct<T>, DirectBinding<T>>>(g, "direct");

        fl::scope imu = g.region(p.imu);
        fl::fire<"imu">(imu, est, p.imu);
        fl::scope truth = g.region(p.truth);
        fl::fire<"truth">(truth, est, p.truth);
        fl::scope tick = g.region(p.tick);
        auto [x] = fl::fire<"estimate">(tick, est, fl::at(p.tick));
        auto [reference, command] = fl::fire<"step">(tick, direct, p.field, x);
        g.produce(p.estimate, x);
        g.produce(p.reference, reference);
        g.produce(p.command, command);
        return p;
    }
};

// ── The stack ───────────────────────────────────────────────────────

/// A baseline's graph: the scoreboard graph over its bodies, or its research graph.
template <class Baseline, class G>
Ports<double> describe(G& g, const World& world) {
    if constexpr (Baseline::kind == fl::baseline_class::scoreboard) {
        return scoreboard<double, typename Baseline::bodies>(g, world);
    } else {
        return Baseline::graph::describe(g, world);
    }
}

/// A baseline's stack: the CPU stack of its graph, driven by the harness
/// (RunnableStack). Each hook hands the stack its input, which runs that
/// input's region; the tick returns the tick region's outputs.
template <class Baseline>
class FieldGraph {
public:
    using scalar_type = double;
    using profile = an::MobilityProfile;

    /// Build the graph and its operators from `ctx`, and check the baseline
    /// file against the graph (G8).
    [[nodiscard]] static an::result<FieldGraph, std::string> create(const Context& ctx) {
        fl::stack_builder<Context> g{"field", Baseline::kind, ctx};
        const Ports<double> p = describe<Baseline>(g, ctx.world);
        auto st = g.build_stack();
        if (!st) {
            return st.error().to_string();
        }
        if (const auto c = fl::check_baseline(ctx.baseline, Baseline::name, st->graph()); !c) {
            return c.error().to_string();
        }
        return FieldGraph(std::move(st).value(), p);
    }

    void on_imu(const an::ImuSample<double>& s) {
        stack_.input(ports_.imu, s);
    }
    void on_frame(const an::CameraFrame& f) {
        stack_.input(ports_.camera, f);
    }
    void on_truth(const an::TruthState<double>& x) {
        stack_.input(ports_.truth, x);
    }
    an::TickOutput<double> on_tick(an::Timestamp now) {
        stack_.input(ports_.tick, now);
        return an::TickOutput<double>{
            stack_.output(ports_.estimate), stack_.output(ports_.reference), stack_.output(ports_.command)};
    }

    [[nodiscard]] const fl::stack& stack() const noexcept {
        return stack_;
    }

private:
    FieldGraph(fl::stack s, Ports<double> p) : stack_(std::move(s)), ports_(std::move(p)) {}

    fl::stack stack_;
    Ports<double> ports_;
};

/// The same stack as a scoreboard graph's, wired by hand, as a slot stack is.
template <class Est, class Plan, class Ctl, class Safe, class Daa, class Fc>
class FieldSlots {
public:
    using scalar_type = double;
    using profile = an::MobilityProfile;

    [[nodiscard]] static an::result<FieldSlots, std::string> create(const Context& ctx) {
        auto ctl = an::make_slot<Ctl>(ctx);
        if (!ctl) {
            return ctl.error();
        }
        return FieldSlots(ctx.world, std::move(ctl).value());
    }

    void on_imu(const an::ImuSample<double>& s) {
        est_.process_imu(s);
    }
    void on_frame(const an::CameraFrame& f) {
        daa_.update(an::IntruderDetections<double>{}, est_.state_at(f.header.t_mid));
    }
    void on_truth(const an::TruthState<double>& x) {
        est_.process_truth(x);
    }
    an::TickOutput<double> on_tick(an::Timestamp now) {
        const an::StateEstimate<double> x = est_.state_at(now);
        const an::Reference<double> ref = plan_.plan(world_, x);
        const an::ConflictSet<double> conflicts = daa_.conflicts(x);
        const an::ThrustRates<double> u = safe_.filter(x, ctl_.step(x, ref, conflicts), conflicts, world_.geofence);
        fc_.send(u, now);
        return an::TickOutput<double>{x, ref, u};
    }

private:
    FieldSlots(const World& w, Ctl ctl) : world_(w), ctl_(std::move(ctl)) {}

    World world_;
    Est est_{};
    Plan plan_{};
    Ctl ctl_;
    Safe safe_{};
    Daa daa_{};
    Fc fc_{};
};

}  // namespace field
