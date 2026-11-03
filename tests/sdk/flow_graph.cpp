// Unit tests for the graph description (flow/graph.hpp; docs/plan/flow-graph,
// 2a): the test mission's interface, registry and baselines load and flatten
// to the expected firings; the canonical form and the hash survive a round
// trip; and a file that breaks each invariant is refused with its message.
// G4 (an edge joins two ports of one domain) is the compiler's, in the
// generated stack, and is tested with the generator (2b).

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/msg/camera.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <type_traits>

using namespace branes::anatomy;
using namespace branes::anatomy::flow;

namespace {

const std::filesystem::path src = ANATOMY_SOURCE_DIR;
const std::filesystem::path fixtures = src / "tests/fixtures/flow";

std::string read(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    REQUIRE(in);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

/// Replace the first occurrence of `from` in `s` with `to`. Pre: `from` occurs in `s`.
std::string replace(std::string s, const std::string& from, const std::string& to) {
    const auto at = s.find(from);
    REQUIRE(at != std::string::npos);
    return s.replace(at, from.size(), to);
}

// The test mission's slots, for the slot form the baselines keep.
constexpr std::array<std::string_view, 3> kTestSlots = {"controller", "estimator", "safety_filter"};

Interface mission() {
    const auto m = load_interface(fixtures / "interface.yaml");
    REQUIRE(m);
    return *m;
}

Registry registry() {
    const auto r = load_registry(fixtures / "operators.yaml");
    REQUIRE(r);
    return *r;
}

config::BaselineConfig baseline(const std::string& yaml) {
    const auto b = config::parse_baseline(yaml, "baseline", kTestSlots);
    REQUIRE(b);
    return *b;
}

result<Graph, config::ConfigError> graph_of(const std::string& yaml) {
    return load_graph(baseline(yaml), "baseline", mission(), registry());
}

Graph graph(const std::string& yaml) {
    const auto g = graph_of(yaml);
    if (!g) {
        FAIL(g.error().to_string());
    }
    return *g;
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

/// The error a broken interface is refused with.
config::ConfigError interface_error(const std::string& yaml) {
    const auto m = parse_interface(yaml, "interface");
    REQUIRE_FALSE(m);
    return m.error();
}

/// The error a broken baseline graph is refused with.
config::ConfigError graph_error(const std::string& yaml) {
    const auto g = graph_of(yaml);
    REQUIRE_FALSE(g);
    return g.error();
}

bool starts_with(const std::string& s, std::string_view prefix) {
    return s.rfind(prefix, 0) == 0;
}

}  // namespace

// ── The files load ──────────────────────────────────────────────────

TEST_CASE("flow graph: the interface and the registry load", "[flow][graph]") {
    const Interface m = mission();
    CHECK(m.mission == "test");
    CHECK(m.inputs.size() == 5u);
    CHECK(m.inputs[1].time == "header.t_mid");
    CHECK(m.inputs[3].sitl_only);
    CHECK(m.inputs[4].message.empty());  // the tick carries only its time
    CHECK(m.constants.size() == 1u);
    REQUIRE(m.composites.size() == 4u);
    CHECK(m.composites[0].handlers[2] == HandlerDecl{"estimate", {}, {"estimate"}, true, state_role::read});
    REQUIRE(m.regions.size() == 3u);
    CHECK(m.regions[2].firings[2].sources[3] == SourceDecl{"marks", rate_rule::hold});

    const Registry r = registry();
    REQUIRE(r.operators.size() == 6u);
    CHECK(r.operators[0].composite == "estimator");
    CHECK(r.operators[4].handlers[0].out == std::vector<std::string>{"setpoint"});
}

TEST_CASE("flow graph: a scoreboard baseline of single-operator bodies", "[flow][graph]") {
    const Graph g = graph(read(fixtures / "scoreboard.yaml"));
    CHECK(g.kind == baseline_class::scoreboard);
    REQUIRE(g.instances.size() == 4u);
    CHECK(g.instances[2] == Instance{"controller", "PassController", "controller"});
    REQUIRE(g.regions.size() == 3u);

    // The frame region reads the estimate before the estimator takes the
    // frame, and the monitor gets that prior.
    const Region& frame = g.regions[1];
    REQUIRE(frame.firings.size() == 3u);
    CHECK(frame.firings[0].handler == "estimate");
    CHECK(frame.firings[0].at == "frame");
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
    const Graph g = graph(read(fixtures / "body.yaml"));
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
    CHECK(graph_hash(g) != graph_hash(graph(read(fixtures / "scoreboard.yaml"))));
}

TEST_CASE("flow graph: a research baseline with operators of their own", "[flow][graph]") {
    const Graph g = graph(read(fixtures / "research.yaml"));
    CHECK(g.kind == baseline_class::research);
    REQUIRE(g.instances.size() == 4u);
    CHECK(g.instances[2] == Instance{"gain", "Gain", ""});
    const Region& tick = g.regions[2];
    REQUIRE(tick.firings.size() == 4u);
    CHECK(tick.firings[3].in[1] == firing("tick", 2, "setpoint"));
    CHECK(g.outputs[1] == Output{"command", "tick", firing("tick", 3, "command")});
}

TEST_CASE("flow graph: existing baseline files load unchanged", "[flow][graph][config]") {
    const auto b = config::load_baseline(src / "tests/fixtures/baseline.yaml", kTestSlots);
    REQUIRE(b);
    CHECK(b->baseline_class.empty());
    CHECK(b->graph.empty());
    CHECK(config::parse_baseline(config::emit_baseline(*b), "emitted", kTestSlots).value() == *b);
    // A graph without a class, or a class of neither kind, is refused.
    const auto none = load_graph(*b, "baseline", mission(), registry());
    REQUIRE_FALSE(none);
    CHECK(starts_with(none.error().message, "G8:"));
    CHECK(config::parse_baseline(
              replace(read(fixtures / "scoreboard.yaml"), "class: scoreboard", "class: other"), "case", kTestSlots)
              .error()
              .key == "class");
}

// ── The canonical form and the hash ─────────────────────────────────

TEST_CASE("flow graph: the canonical form survives a round trip", "[flow][graph]") {
    for (const char* file : {"scoreboard.yaml", "body.yaml", "research.yaml"}) {
        const std::string yaml = read(fixtures / file);
        const config::BaselineConfig b = baseline(yaml);
        const Graph g = graph(yaml);

        // emit → parse gives the same baseline, and the same graph.
        const auto back = config::parse_baseline(config::emit_baseline(b), "emitted", kTestSlots);
        REQUIRE(back);
        CHECK(*back == b);
        const auto again = load_graph(*back, "emitted", mission(), registry());
        REQUIRE(again);
        CHECK(*again == g);
        CHECK(canonical(*again) == canonical(g));
        CHECK(graph_hash(*again) == graph_hash(g));
    }

    // The layout of the file isn't part of the graph: flow style, and the
    // order of the composites' bodies, give one canonical form.
    const std::string yaml = read(fixtures / "scoreboard.yaml");
    const std::string flow_style =
        replace(yaml,
                "graph:\n  estimator: TestEstimator\n  monitor: NullMonitor\n  controller: PassController\n  "
                "safety_filter: NullSafetyFilter\n",
                "graph: {safety_filter: NullSafetyFilter, controller: PassController, monitor: NullMonitor, "
                "estimator: TestEstimator}\n");
    CHECK(canonical(graph(flow_style)) == canonical(graph(yaml)));

    // A different operator, or order of firings, is a different graph.
    const Graph g = graph(yaml);
    Graph other = g;
    other.instances[1].op = "OtherMonitor";
    CHECK(graph_hash(other) != graph_hash(g));
    std::swap(other.regions[2].firings[0], other.regions[2].firings[1]);
    CHECK(canonical(other) != canonical(g));
}

// ── The invariants ──────────────────────────────────────────────────

TEST_CASE("flow graph: G1 names", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "interface.yaml");
    auto e = interface_error(replace(yaml, "  monitor:\n    signature", "  Monitor:\n    signature"));
    CHECK(e.key == "composites.Monitor");
    CHECK(e.message == "G1: a composite's names match [a-z_][a-z0-9_]*");
    e = interface_error(replace(yaml, "      update: { in: [estimate]", "      update: { in: [estimate, estimate]"));
    CHECK(e.key == "composites.monitor.handlers.update.in[1]");
    CHECK(e.message == "G1: a handler's port names are distinct");
    e = interface_error(replace(yaml, "  monitor:\n    signature", "  imu:\n    signature"));
    CHECK(e.key == "composites.imu");
    CHECK(starts_with(e.message, "G1: nodes and external ports share one namespace"));
}

TEST_CASE("flow graph: G2 operators are registered, and fill their composite", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "scoreboard.yaml");
    auto e = graph_error(replace(yaml, "  monitor: NullMonitor", "  monitor: Absent"));
    CHECK(e.key == "graph.monitor");
    CHECK(e.message == "G2: every operator is registered: 'Absent' is not");
    e = graph_error(replace(yaml, "  monitor: NullMonitor", "  monitor: PassController"));
    CHECK(e.key == "graph.monitor");
    CHECK(starts_with(e.message, "G2: a body of one operator fills its composite"));
    e = interface_error(replace(read(fixtures / "interface.yaml"), "monitor.alerts: [", "monitor.alarm: ["));
    CHECK(e.key == "regions.tick[1]");
    CHECK(e.message == "G2: 'monitor' has no handler 'alarm'");
}

TEST_CASE("flow graph: G3 every in port has one source, which exists", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "interface.yaml");
    auto e = interface_error(
        replace(yaml, "[estimator.estimate, limits, monitor.alerts", "[estimator.estimate, monitor.alerts"));
    CHECK(e.key == "regions.tick[2]");
    CHECK(starts_with(e.message, "G3: every in port has one source"));
    e = interface_error(replace(yaml, "limits.ceiling]", "limit.ceiling]"));
    CHECK(e.key == "regions.tick[3][2]");
    CHECK(starts_with(e.message, "G3: every source exists"));
    e = interface_error(replace(yaml, "    - estimator.estimate@tick\n", "    - estimator.estimate\n"));
    CHECK(e.key == "regions.tick[0]");
    CHECK(starts_with(e.message, "G3: estimator.estimate takes a time"));
    e = graph_error(replace(read(fixtures / "research.yaml"), "    command: track.command\n", ""));
    CHECK(e.key == "graph.out");
    CHECK(e.message == "G3: every external out port has one producer: 'command' has none");
}

TEST_CASE("flow graph: G5 the declared order is a topological order", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "interface.yaml");
    auto e = interface_error(replace(replace(yaml, "    - monitor.alerts: [estimator.estimate]\n", ""),
                                     "    - safety_filter.filter",
                                     "    - monitor.alerts: [estimator.estimate]\n    - safety_filter.filter"));
    CHECK(e.key == "regions.tick[1][2]");
    CHECK(starts_with(e.message, "G5: the declared order is a topological order"));
    e = interface_error(replace(replace(yaml, "  imu:\n    - estimator.imu: [imu]\n", ""),
                                "    - estimator.frame: [frame]\n",
                                "    - estimator.frame: [frame]\n    - estimator.imu: [{from: imu, rate: hold}]\n"));
    CHECK(e.key == "regions.frame[2]");
    CHECK(e.message == "G5: a region updates a node's state once, so each read is before or after it");
    e = graph_error(replace(read(fixtures / "body.yaml"),
                            "          - gain.apply: [estimate, limits]\n          - track.step: [estimate, "
                            "gain.setpoint, alerts, marks]\n",
                            "          - track.step: [estimate, gain.setpoint, alerts, marks]\n          - "
                            "gain.apply: [estimate, limits]\n"));
    CHECK(e.key == "graph.controller.handlers.step.fire[0][1]");
    CHECK(starts_with(e.message, "G5: the declared order is a topological order"));
}

TEST_CASE("flow graph: G6 regions, triggers and firings", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "interface.yaml");
    auto e = interface_error(
        replace(yaml, "  tick:\n    - estimator.estimate@tick", "  clock:\n    - estimator.estimate@tick"));
    CHECK(e.key == "regions.clock");
    CHECK(e.message == "G6: a region is a trigger an input declares");
    e = interface_error(replace(yaml, "  imu:\n    - estimator.imu: [imu]\n", ""));
    CHECK(e.key == "regions");
    CHECK(e.message == "G6: every handler fires in a region: estimator.imu fires in none");
    e = interface_error(replace(yaml, "    - estimator.estimate@frame\n", "    - estimator.estimate@tick\n"));
    CHECK(e.key == "regions.frame[0]");
    CHECK(e.message == "G6: a firing takes its time from an input of its region's trigger");
    e = interface_error(replace(yaml,
                                "    - monitor.update: [estimator.estimate]\n",
                                "    - monitor.update: [estimator.estimate]\n    - estimator.frame: [frame]\n"));
    CHECK(e.key == "regions.frame[3]");
    CHECK(e.message == "G6: every firing belongs to exactly one region");
}

TEST_CASE("flow graph: G7 an edge between regions holds", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "interface.yaml");
    auto e = interface_error(replace(yaml, "{from: marks, rate: hold}", "marks"));
    CHECK(e.key == "regions.tick[2][3]");
    CHECK(starts_with(e.message, "G7: an edge between regions has a rate rule"));
    e = interface_error(replace(yaml, "{from: marks, rate: hold}", "{from: marks, rate: accumulate}"));
    CHECK(e.key == "regions.tick[2][3].rate");
    CHECK(e.message == "G7: step 2 accepts the rate rule hold only");
    e = interface_error(replace(yaml,
                                "    - monitor.update: [estimator.estimate]\n",
                                "    - monitor.update: [{from: estimator.estimate, rate: hold}]\n"));
    CHECK(e.key == "regions.frame[2][0]");
    CHECK(e.message == "G7: an edge within a region has no rate rule");
}

TEST_CASE("flow graph: G8 a baseline's class, and a scoreboard's composites", "[flow][graph][invariant]") {
    const std::string yaml = read(fixtures / "scoreboard.yaml");
    auto e = graph_error(replace(yaml, "class: scoreboard\n", ""));
    CHECK(e.key == "class");
    CHECK(e.message == "G8: a baseline with a graph declares its class, scoreboard or research");
    e = graph_error(replace(yaml, "  monitor: NullMonitor\n", ""));
    CHECK(e.key == "graph");
    CHECK(e.message == "G8: a scoreboard baseline gives every composite a body: 'monitor' has none");
    e = graph_error(replace(yaml, "  monitor: NullMonitor\n", "  monitor: NullMonitor\n  planner: NullMonitor\n"));
    CHECK(e.key == "graph.planner");
    CHECK(starts_with(e.message, "G8: a scoreboard baseline keeps the interface's composites"));
    e = graph_error(
        replace(read(fixtures / "body.yaml"), "      step:\n        fire:", "      stride:\n        fire:"));
    CHECK(e.key == "graph.controller.handlers.stride");
    CHECK(e.message == "G8: controller has no handler 'stride'");
}

TEST_CASE("flow graph: G9 a graph that reads a SITL-only input is SITL-only", "[flow][graph][invariant]") {
    // G9 is derived, not refused: the generated header says it (2b).
    CHECK_FALSE(graph(read(fixtures / "scoreboard.yaml")).sitl_only);
    const std::string truth =
        replace(replace(read(fixtures / "interface.yaml"),
                        "      estimate: { out: [estimate]",
                        "      truth:    { in: [truth], state: update }\n      estimate: { out: [estimate]"),
                "  frame:\n    - estimator.estimate@frame",
                "  truth:\n    - estimator.truth: [truth]\n  frame:\n    - estimator.estimate@frame");
    const auto m = parse_interface(truth, "interface");
    REQUIRE(m);
    const auto g = load_graph(baseline(read(fixtures / "scoreboard.yaml")), "baseline", *m, registry());
    REQUIRE(g);
    CHECK(g->sitl_only);
    CHECK(graph_hash(*g) != graph_hash(graph(read(fixtures / "scoreboard.yaml"))));
}

// ── The camera frame's domain ───────────────────────────────────────

TEST_CASE("flow graph: the camera frame is an opaque scalar", "[flow][graph]") {
    using D = domain_of_t<CameraFrame>;
    STATIC_REQUIRE(is_domain_v<D>);
    STATIC_REQUIRE(rank_v<D> == 0);
    STATIC_REQUIRE(std::is_same_v<D::element_type, CameraFrame>);
}
