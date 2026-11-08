// Unit tests for the graph builder (flow/builder.hpp, flow/graph.hpp;
// docs/plan/flow-graph, 2a): the test mission's scoreboard, body and
// research graphs build and flatten to the expected firings; the canonical
// form and the hash are a function of the graph; a baseline file names what
// fills the graph; and a graph that breaks each invariant checked as it is
// built is refused with its message. The invariants the compiler checks are
// tested by tests/compile_fail/flow_g*.cpp.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/harness/graph.hpp>
#include <branes/anatomy/msg/camera.hpp>

#include "../fixtures/flow/test_mission.hpp"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <functional>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

using namespace branes::anatomy;
using namespace branes::anatomy::flow;

namespace {

const std::filesystem::path src = ANATOMY_SOURCE_DIR;

// The test mission's slots, for the slot form the baselines keep.
constexpr std::array<std::string_view, 3> kTestSlots = {"controller", "estimator", "safety_filter"};

using BodyOfSeveral = test::Bodies<test::TestEstimatorOp, test::NullMonitorOp, test::GainTrack<>>;

result<Graph, config::ConfigError> built(baseline_class kind, const std::function<void(builder&)>& describe) {
    builder g{"test", kind};
    describe(g);
    return g.build();
}

Graph graph(baseline_class kind, const std::function<void(builder&)>& describe) {
    const auto g = built(kind, describe);
    if (!g) {
        FAIL(g.error().to_string());
    }
    return *g;
}

Graph scoreboard_graph() {
    return graph(baseline_class::scoreboard, [](builder& g) { test::scoreboard(g); });
}

Graph body_graph() {
    return graph(baseline_class::scoreboard, [](builder& g) { test::scoreboard<BodyOfSeveral>(g); });
}

Graph research_graph() {
    return graph(baseline_class::research, [](builder& g) { (void)test::research(g); });
}

/// The error a graph is refused with.
config::ConfigError error(baseline_class kind, const std::function<void(builder&)>& describe) {
    const auto g = built(kind, describe);
    REQUIRE_FALSE(g);
    CHECK(g.error().file == "test");
    return g.error();
}

config::ConfigError scoreboard_error(const std::function<void(builder&)>& describe) {
    return error(baseline_class::scoreboard, describe);
}

Ref firing(std::string region, std::size_t i, std::string port, std::vector<std::string> path = {}) {
    Ref r;
    r.from = Ref::kind::firing;
    r.region = std::move(region);
    r.firing = i;
    r.port = std::move(port);
    r.path = std::move(path);
    return r;
}

Ref input(std::string port, rate_rule rate = rate_rule::none) {
    Ref r;
    r.from = Ref::kind::input;
    r.port = std::move(port);
    r.rate = rate;
    return r;
}

Ref constant(std::string port, std::vector<std::string> path = {}) {
    Ref r;
    r.from = Ref::kind::constant;
    r.port = std::move(port);
    r.path = std::move(path);
    return r;
}

config::BaselineConfig baseline(const std::string& yaml) {
    const auto b = config::parse_baseline(yaml, "baseline", kTestSlots);
    if (!b) {
        FAIL(b.error().to_string());
    }
    return *b;
}

config::ConfigError baseline_error(const std::string& yaml) {
    const auto b = config::parse_baseline(yaml, "baseline", kTestSlots);
    REQUIRE_FALSE(b);
    return b.error();
}

/// Replace the first occurrence of `from` in `s` with `to`. Pre: `from` occurs in `s`.
std::string replace(std::string s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    REQUIRE(at != std::string::npos);
    return s.replace(at, from.size(), to);
}

const std::string kSlots =
    "composition: {controller: PassController, estimator: TestEstimator, safety_filter: NullSafetyFilter}\n";
const std::string kScoreboard = "name: test-scoreboard\nclass: scoreboard\n" + kSlots +
                                "graph:\n  estimator: TestEstimator\n  monitor: NullMonitor\n  controller: "
                                "PassController\n  safety_filter: NullSafetyFilter\n";
const std::string kResearch = "name: test-research\nclass: research\n" + kSlots + "graph: Research\n";

}  // namespace

// ── The graphs build ────────────────────────────────────────────────

TEST_CASE("flow graph: a scoreboard graph of single-operator bodies", "[flow][graph]") {
    const Graph g = scoreboard_graph();
    CHECK(g.mission == "test");
    CHECK(g.kind == baseline_class::scoreboard);
    REQUIRE(g.composites.size() == 4u);
    CHECK(g.composites[2] == Composite{"controller", "PassController"});
    REQUIRE(g.instances.size() == 4u);
    CHECK(g.instances[2] == Instance{"controller", "PassController", "controller"});
    REQUIRE(g.regions.size() == 3u);
    CHECK(g.regions[0].trigger == "imu");

    // The frame region reads the estimate before the estimator takes the
    // frame, and the monitor gets that prior.
    const Region& frame = g.regions[1];
    REQUIRE(frame.firings.size() == 3u);
    CHECK(frame.firings[0] == Firing{"estimator", "estimate", "frame", {}, "estimator.estimate", state_role::read});
    CHECK(frame.firings[1].in == std::vector<Ref>{input("frame")});
    CHECK(frame.firings[2].in == std::vector<Ref>{firing("frame", 0, "estimate")});

    const Region& tick = g.regions[2];
    REQUIRE(tick.firings.size() == 4u);
    CHECK(tick.firings[2].composite == "controller.step");
    CHECK(tick.firings[2].in == std::vector<Ref>{firing("tick", 0, "estimate"),
                                                 constant("limits"),
                                                 firing("tick", 1, "alerts"),
                                                 input("marks", rate_rule::hold)});
    CHECK(tick.firings[3].in[2] == constant("limits", {"ceiling"}));

    REQUIRE(g.outputs.size() == 2u);
    CHECK(g.outputs[0] == Output{"estimate", "tick", firing("tick", 0, "estimate")});
    CHECK(g.outputs[1] == Output{"command", "tick", firing("tick", 3, "safe_command")});
    CHECK_FALSE(g.sitl_only);
    CHECK(graph_hash(g).size() == 64u);
}

TEST_CASE("flow graph: a scoreboard body of several operators", "[flow][graph]") {
    const Graph g = body_graph();
    CHECK(g.composites[2] == Composite{"controller", "GainTrack"});
    REQUIRE(g.instances.size() == 5u);
    CHECK(g.instances[2] == Instance{"controller.gain", "Gain", "controller"});
    CHECK(g.instances[3] == Instance{"controller.track", "Track", "controller"});

    // controller.step becomes two firings, fed from the composite's sources.
    const Region& tick = g.regions[2];
    REQUIRE(tick.firings.size() == 5u);
    CHECK(tick.firings[2].op == "controller.gain");
    CHECK(tick.firings[2].composite == "controller.step");
    CHECK(tick.firings[2].in == std::vector<Ref>{firing("tick", 0, "estimate"), constant("limits")});
    CHECK(tick.firings[3].op == "controller.track");
    CHECK(tick.firings[3].in == std::vector<Ref>{firing("tick", 0, "estimate"),
                                                 firing("tick", 2, "setpoint"),
                                                 firing("tick", 1, "alerts"),
                                                 input("marks", rate_rule::hold)});
    // The safety filter reads the composite's command, which is Track's.
    CHECK(tick.firings[4].in[1] == firing("tick", 3, "command"));
    CHECK(graph_hash(g) != graph_hash(scoreboard_graph()));
}

TEST_CASE("flow graph: a body's firing fires once per firing of its composite", "[flow][graph]") {
    // The estimator's estimate fires at the frame and at the tick, and the
    // body's untimed read once in each.
    const Graph g = graph(baseline_class::scoreboard,
                          [](builder& b) { test::scoreboard<test::Bodies<test::FilteredEstimator>>(b); });
    CHECK(g.instances[0] == Instance{"estimator.filter", "Filter", "estimator"});
    CHECK(g.regions[1].firings[0] ==
          Firing{"estimator.filter", "read", "", {}, "estimator.estimate", state_role::read});
    CHECK(g.regions[2].firings[0] == g.regions[1].firings[0]);
    CHECK(g.regions[2].firings[1].in == std::vector<Ref>{firing("tick", 0, "estimate")});
}

TEST_CASE("flow graph: a research graph with operators of their own", "[flow][graph]") {
    const Graph g = research_graph();
    CHECK(g.kind == baseline_class::research);
    CHECK(g.composites.empty());
    REQUIRE(g.instances.size() == 4u);
    CHECK(g.instances[2] == Instance{"gain", "Gain", ""});
    const Region& tick = g.regions[2];
    REQUIRE(tick.firings.size() == 4u);
    CHECK(tick.firings[2].composite.empty());
    CHECK(tick.firings[3].in[1] == firing("tick", 2, "setpoint"));
    CHECK(g.outputs[1] == Output{"command", "tick", firing("tick", 3, "command")});
}

// ── The canonical form and the hash ─────────────────────────────────

TEST_CASE("flow graph: the canonical form is a function of the graph", "[flow][graph]") {
    // Two builds of one description give one graph, one form and one hash.
    CHECK(scoreboard_graph() == scoreboard_graph());
    CHECK(canonical(scoreboard_graph()) == canonical(scoreboard_graph()));
    CHECK(canonical(body_graph()) == canonical(body_graph()));
    CHECK(canonical(research_graph()) == canonical(research_graph()));

    // A different operator, body or order of firings is a different graph.
    const Graph g = scoreboard_graph();
    Graph other = g;
    other.instances[1].op = "OtherMonitor";
    CHECK(graph_hash(other) != graph_hash(g));
    other = g;
    std::swap(other.regions[2].firings[0], other.regions[2].firings[1]);
    CHECK(canonical(other) != canonical(g));
    other = g;
    other.composites[2].body = "GainTrack";
    CHECK(graph_hash(other) != graph_hash(g));
    CHECK(graph_hash(research_graph()) != graph_hash(g));
}

// ── The baseline file ───────────────────────────────────────────────

TEST_CASE("flow graph: a baseline file names what fills its graph", "[flow][graph][config]") {
    const config::BaselineConfig b = baseline(kScoreboard);
    CHECK(b.baseline_class == "scoreboard");
    CHECK(b.bodies.size() == 4u);
    CHECK(b.bodies.at("controller") == "PassController");
    CHECK(b.graph.empty());
    CHECK(check_baseline(b, "baseline", scoreboard_graph()));
    CHECK(config::parse_baseline(config::emit_baseline(b), "emitted", kTestSlots).value() == b);

    const config::BaselineConfig r = baseline(kResearch);
    CHECK(r.graph == "Research");
    CHECK(r.bodies.empty());
    CHECK(check_baseline(r, "baseline", research_graph()));
    CHECK(config::parse_baseline(config::emit_baseline(r), "emitted", kTestSlots).value() == r);

    // Every existing baseline file still loads, unchanged.
    const auto old = config::load_baseline(src / "tests/fixtures/baseline.yaml", kTestSlots);
    REQUIRE(old);
    CHECK(old->baseline_class.empty());
    CHECK(old->bodies.empty());
    CHECK(old->graph.empty());
    CHECK(config::parse_baseline(config::emit_baseline(*old), "emitted", kTestSlots).value() == *old);
}

TEST_CASE("flow graph: G8 a baseline's class and its bodies", "[flow][graph][invariant][config]") {
    auto e = baseline_error("name: test\n" + kSlots + "graph: {estimator: TestEstimator}\n");
    CHECK(e.key == "class");
    CHECK(e.message == "G8: a baseline with a graph declares its class");
    e = baseline_error("name: test\nclass: scoreboard\n" + kSlots);
    CHECK(e.key == "graph");
    CHECK(e.message == "G8: a baseline with a class gives its graph");
    e = baseline_error(replace(kResearch, "class: research", "class: other"));
    CHECK(e.key == "class");
    CHECK(e.message == "must be scoreboard or research");
    e = baseline_error(replace(kResearch, "graph: Research", "graph: {estimator: TestEstimator}"));
    CHECK(e.key == "graph");
    CHECK(e.message == "a research graph is the name of a registered graph");
    e = baseline_error(replace(kScoreboard, "  estimator:", "  Estimator:"));
    CHECK(e.key == "graph.Estimator");
    CHECK(e.message == "G1: a composite's name matches [a-z_][a-z0-9_]*");

    // The file and the graph built for it agree.
    const Graph g = scoreboard_graph();
    auto c = check_baseline(baseline(kResearch), "baseline", g);
    REQUIRE_FALSE(c);
    CHECK(c.error().key == "class");
    CHECK(c.error().message == "G8: the baseline's class is its graph's, scoreboard, not 'research'");
    c = check_baseline(baseline(replace(kScoreboard, "  monitor: NullMonitor\n", "")), "baseline", g);
    REQUIRE_FALSE(c);
    CHECK(c.error().key == "graph");
    CHECK(c.error().message == "G8: a scoreboard baseline gives every composite a body: 'monitor' has none");
    c = check_baseline(baseline(kScoreboard + "  planner: NullMonitor\n"), "baseline", g);
    REQUIRE_FALSE(c);
    CHECK(c.error().key == "graph.planner");
    CHECK(c.error().message == "G8: a scoreboard baseline keeps the mission's composites: 'planner' is not one");
    c = check_baseline(
        baseline(replace(kScoreboard, "  controller: PassController", "  controller: GainTrack")), "baseline", g);
    REQUIRE_FALSE(c);
    CHECK(c.error().key == "graph.controller");
    CHECK(c.error().message ==
          "G8: the body the file names fills the composite: the file names 'GainTrack', and the graph has "
          "'PassController'");
    CHECK(check_baseline(baseline(replace(kScoreboard, "  controller: PassController", "  controller: GainTrack")),
                         "baseline",
                         body_graph()));
}

// ── The invariants checked as the graph is built ────────────────────

TEST_CASE("flow graph: G1 names", "[flow][graph][invariant]") {
    auto e = scoreboard_error([](builder& g) {
        (void)test::ports(g);
        (void)composite<test::MonitorBinding, test::NullMonitorOp>(g, "Monitor");
    });
    CHECK(e.key == "Monitor");
    CHECK(e.message == "G1: a composite's names match [a-z_][a-z0-9_]*");
    e = scoreboard_error([](builder& g) {
        (void)test::ports(g);
        (void)composite<test::MonitorBinding, test::NullMonitorOp>(g, "imu");
    });
    CHECK(e.key == "imu");
    CHECK(e.message == "G1: nodes and external ports share one namespace, and 'imu' is taken");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        (void)output<test::Command>(g, "command", p.tick);
    });
    CHECK(e.key == "out.command");
    CHECK(e.message == "G1: an output's names are distinct");
}

namespace {

/// A body that implements none of its composite's handlers.
struct Silent {
    static constexpr std::string_view name = "Silent";
    using binding = test::ControllerBinding;
    static void build(body<test::ControllerBinding>& b) {
        (void)instance<test::TrackOp>(b, "track");
    }
};

/// A body that implements its composite's handler twice.
struct Twice {
    static constexpr std::string_view name = "Twice";
    using binding = test::ControllerBinding;
    static void build(body<test::ControllerBinding>& b) {
        const auto track = instance<test::TrackOp>(b, "track");
        for (int i = 0; i < 2; ++i) {
            on<"step">(b, [=](scope& s, auto estimate, auto /*limits*/, auto alerts, auto marks) {
                return fire<"step">(s, track, estimate, value<test::Setpoint>{}, alerts, marks);
            });
        }
    }
};

}  // namespace

TEST_CASE("flow graph: G2 a body implements its composite's handlers", "[flow][graph][invariant]") {
    auto e = scoreboard_error([](builder& g) { (void)composite<test::ControllerBinding, Silent>(g, "controller"); });
    CHECK(e.key == "controller");
    CHECK(e.message == "G2: a body implements every handler of its composite: Silent has no 'step'");
    e = scoreboard_error([](builder& g) { (void)composite<test::ControllerBinding, Twice>(g, "controller"); });
    CHECK(e.key == "controller.step");
    CHECK(e.message == "G2: a body implements each handler of its composite once");
}

TEST_CASE("flow graph: G3 every external out port has one producer", "[flow][graph][invariant]") {
    auto e = error(baseline_class::research, [](builder& g) {
        const test::Ports p = test::ports(g);
        const auto estimator = instance<test::TestEstimatorOp>(g, "estimator");
        scope imu = g.region(p.imu);
        fire<"imu">(imu, estimator, p.imu);
        scope frame = g.region(p.frame);
        fire<"frame">(frame, estimator, p.frame);
        scope tick = g.region(p.tick);
        auto [estimate] = fire<"estimate">(tick, estimator, at(p.tick));
        g.produce(p.estimate, estimate);
    });
    CHECK(e.key == "out.command");
    CHECK(e.message == "G3: every external out port has one producer: 'command' has none");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope tick = g.region(p.tick);
        auto [estimate] = fire<"estimate">(tick, c.estimator, at(p.tick));
        g.produce(p.estimate, estimate);
        g.produce(p.estimate, estimate);
    });
    CHECK(e.key == "out.estimate");
    CHECK(e.message == "G3: every external out port has one producer: 'estimate' has two");
}

TEST_CASE("flow graph: G5 a region updates state once, and a body only where its composite does",
          "[flow][graph][invariant]") {
    auto e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        auto [prior] = fire<"estimate">(frame, c.estimator, at(p.frame));
        fire<"update">(frame, c.monitor, prior);
        fire<"frame">(frame, c.estimator, p.frame);
        fire<"imu">(frame, c.estimator, hold(p.imu));
    });
    CHECK(e.key == "frame[3]");
    CHECK(e.message == "G5: a region updates a node's state once, so each read is before or after it");

    // A body updates state only where its composite handler does, so the
    // mission's order of reads and updates holds for the body's operators.
    e = scoreboard_error([](builder& g) {
        test::scoreboard<
            test::Bodies<test::TestEstimatorOp, test::NullMonitorOp, test::GainTrack<test::StatefulGainOp>>>(g);
    });
    CHECK(e.key == "tick[2]");
    CHECK(e.message == "G5: controller.gain.apply updates its state, and controller.step declares no update");
}

TEST_CASE("flow graph: G6 regions, triggers and firings", "[flow][graph][invariant]") {
    auto e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        (void)g.region(p.tick);
        (void)g.region(p.tick);
    });
    CHECK(e.key == "tick");
    CHECK(e.message == "G6: a trigger has one region");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        (void)fire<"estimate">(frame, c.estimator, at(p.tick));
    });
    CHECK(e.key == "frame[0]");
    CHECK(e.message == "G6: a firing takes its time from an input of its region's trigger");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        fire<"frame">(frame, c.estimator, p.frame);
        fire<"frame">(frame, c.estimator, p.frame);
    });
    CHECK(e.key == "frame[1]");
    CHECK(e.message == "G6: every firing belongs to exactly one region");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        auto [prior] = fire<"estimate">(frame, c.estimator, at(p.frame));
        g.produce(p.estimate, prior);
    });
    CHECK(e.key == "out.estimate");
    CHECK(e.message == "G6: an output is written by a firing of its region, 'tick'");

    // Every handler of every composite fires: the mission without its imu region.
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        auto [prior] = fire<"estimate">(frame, c.estimator, at(p.frame));
        fire<"frame">(frame, c.estimator, p.frame);
        fire<"update">(frame, c.monitor, prior);
        scope tick = g.region(p.tick);
        auto [estimate] = fire<"estimate">(tick, c.estimator, at(p.tick));
        auto [alerts] = fire<"alerts">(tick, c.monitor, estimate);
        auto [command] = fire<"step">(tick, c.controller, estimate, p.limits, alerts, hold(p.marks));
        auto [safe] = fire<"filter">(
            tick, c.safety_filter, estimate, command, project<"ceiling", &test::Limits::ceiling>(p.limits));
        g.produce(p.estimate, estimate);
        g.produce(p.command, safe);
    });
    CHECK(e.key == "estimator");
    CHECK(e.message == "G6: every handler fires in a region: estimator.imu fires in none");
}

TEST_CASE("flow graph: G7 an edge between regions holds, and no other", "[flow][graph][invariant]") {
    auto e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope tick = g.region(p.tick);
        auto [estimate] = fire<"estimate">(tick, c.estimator, at(p.tick));
        auto [alerts] = fire<"alerts">(tick, c.monitor, estimate);
        (void)fire<"step">(tick, c.controller, estimate, p.limits, alerts, p.marks);
    });
    CHECK(e.key == "tick[2][3]");
    CHECK(e.message == "G7: an edge between regions has a rate rule: 'marks' fires 'marks', and needs hold(marks) or "
                        "accumulate<N>(marks)");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        auto [prior] = fire<"estimate">(frame, c.estimator, at(p.frame));
        scope tick = g.region(p.tick);
        fire<"update">(tick, c.monitor, prior);
    });
    CHECK(e.key == "tick[0][0]");
    CHECK(e.message == "G7: an edge between regions has a rate rule: estimate is written in region 'frame', and needs "
                       "hold(estimate) or accumulate<N>(estimate)");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope frame = g.region(p.frame);
        auto [prior] = fire<"estimate">(frame, c.estimator, at(p.frame));
        fire<"update">(frame, c.monitor, hold(prior));
    });
    CHECK(e.key == "frame[1][0]");
    CHECK(e.message == "G7: an edge within a region has no rate rule");
    e = scoreboard_error([](builder& g) {
        const test::Ports p = test::ports(g);
        const test::Composites c = test::composites<test::Bodies<>>(g);
        scope tick = g.region(p.tick);
        auto [estimate] = fire<"estimate">(tick, c.estimator, at(p.tick));
        auto [alerts] = fire<"alerts">(tick, c.monitor, estimate);
        (void)fire<"step">(tick, c.controller, estimate, hold(p.limits), alerts, hold(p.marks));
    });
    CHECK(e.key == "tick[2][1]");
    CHECK(e.message == "G7: a constant takes no rate rule");
}

TEST_CASE("flow graph: G8 a graph's class fixes what it holds", "[flow][graph][invariant]") {
    auto e = error(baseline_class::research,
                   [](builder& g) { (void)composite<test::MonitorBinding, test::NullMonitorOp>(g, "monitor"); });
    CHECK(e.key == "monitor");
    CHECK(e.message == "G8: a research graph has no composites: its operators are instances of its own");
    e = scoreboard_error([](builder& g) { (void)instance<test::GainOp>(g, "gain"); });
    CHECK(e.key == "gain");
    CHECK(e.message == "G8: a scoreboard graph's operators are in its composites' bodies");
    e = error(baseline_class::research, [](builder& g) { (void)test::ports(g); });
    CHECK(e.key.empty());
    CHECK(e.message == "G8: a research graph has at least one operator");
}

TEST_CASE("flow graph: G9 a graph that reads a SITL-only input is SITL-only", "[flow][graph][invariant]") {
    // G9 is derived, not refused: the stack says it (2b).
    CHECK_FALSE(research_graph().sitl_only);
    const Graph g = graph(baseline_class::research, [](builder& b) {
        const test::Ports p = test::research(b);
        const auto check = instance<test::TruthCheckOp>(b, "truth_check");
        scope truth = b.region(p.truth);
        (void)fire<"check">(truth, check, p.truth);
    });
    CHECK(g.sitl_only);
    CHECK(graph_hash(g) != graph_hash(research_graph()));
}

// ── The Mermaid flowchart ───────────────────────────────────────────

TEST_CASE("flow graph: the mermaid flowchart", "[flow][graph]") {
    // A subgraph per region, its firings in order; a node per input, constant
    // and output; an edge per in port, dotted where it holds.
    CHECK(mermaid(scoreboard_graph()) == R"(flowchart LR
    %% flow-graph test, scoreboard
    in0(["imu"])
    in1(["frame"])
    in2(["tick"])
    in3(["marks"])
    k0[("limits")]
    subgraph r0["region imu"]
        r0f0["0: estimator.imu<br/>update"]
    end
    subgraph r1["region frame"]
        r1f0["0: estimator.estimate@frame<br/>read"]
        r1f1["1: estimator.frame<br/>update"]
        r1f2["2: monitor.update<br/>update"]
    end
    subgraph r2["region tick"]
        r2f0["0: estimator.estimate@tick<br/>read"]
        r2f1["1: monitor.alerts<br/>read"]
        r2f2["2: controller.step"]
        r2f3["3: safety_filter.filter"]
    end
    out0[/"estimate"/]
    out1[/"command"/]
    in0 --> r0f0
    in1 -->|"time"| r1f0
    in1 --> r1f1
    r1f0 -->|"estimate"| r1f2
    in2 -->|"time"| r2f0
    r2f0 -->|"estimate"| r2f1
    r2f0 -->|"estimate"| r2f2
    k0 --> r2f2
    r2f1 -->|"alerts"| r2f2
    in3 -.->|"hold"| r2f2
    r2f0 -->|"estimate"| r2f3
    r2f2 -->|"command"| r2f3
    k0 -->|".ceiling"| r2f3
    r2f0 -->|"estimate"| out0
    r2f3 -->|"safe_command"| out1
)");

    // A body's firings name the composite handler they implement.
    const std::string body = mermaid(body_graph());
    CHECK(body.find(R"(r2f2["2: controller.gain.apply<br/>controller.step"])") != std::string::npos);
    CHECK(body.find(R"(r2f3["3: controller.track.step<br/>controller.step"])") != std::string::npos);

    // A research graph has no composites to name; a SITL-only graph says so;
    // a name is escaped, `#` too, and the mission's stays on its comment line.
    CHECK(mermaid(research_graph()).find("<br/>controller") == std::string::npos);
    Graph g = scoreboard_graph();
    g.sitl_only = true;
    g.mission = "two\nlines";
    g.outputs[0].name = "a\"b<c>";
    g.outputs[1].name = "a#quot;b";
    const std::string sitl = mermaid(g);
    CHECK(sitl.find("%% flow-graph two lines, scoreboard, SITL-only\n") != std::string::npos);
    CHECK(sitl.find(R"(out0[/"a#quot;b#lt;c#gt;"/])") != std::string::npos);
    CHECK(sitl.find(R"(out1[/"a#35;quot;b"/])") != std::string::npos);
}

TEST_CASE("flow graph: --print-graph prints the text or the flowchart", "[flow][graph]") {
    const Graph g = scoreboard_graph();
    std::ostringstream text;
    CHECK_FALSE(harness::print_graph(g, "text", text));
    CHECK(text.str() == canonical(g));
    std::ostringstream chart;
    CHECK_FALSE(harness::print_graph(g, "mermaid", chart));
    CHECK(chart.str() == mermaid(g));
    std::ostringstream none;
    const auto e = harness::print_graph(g, "dot", none);
    REQUIRE(e);
    CHECK(*e == "--print-graph takes text or mermaid, not 'dot'");
    CHECK(none.str().empty());
}

// ── The camera frame's domain ───────────────────────────────────────

TEST_CASE("flow graph: the camera frame is an opaque scalar", "[flow][graph]") {
    using D = domain_of_t<CameraFrame>;
    STATIC_REQUIRE(is_domain_v<D>);
    STATIC_REQUIRE(rank_v<D> == 0);
    STATIC_REQUIRE(std::is_same_v<D::element_type, CameraFrame>);
}
