// Tests of the CPU stack (flow::stack; docs/plan/flow-graph, 2b), on the
// field mission (fixtures/flow/field): each baseline's stack builds from its
// file and its graph, through the generated headers (cmake/flow.cmake); the
// stack runs the graph its description builds, with its hash; and its SITL
// log is record for record the log of the same operators wired by hand, as
// a slot stack is.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/glue/cascaded_pid.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/slots/ground_truth_core.hpp>
#include <branes/anatomy/mobility/slots/neutral_nulls.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>
#include <branes/anatomy/sim/sitl_loop.hpp>

#include <catch2/catch_test_macros.hpp>
#include <flow/field/field-body.hpp>
#include <flow/field/field-null.hpp>
#include <flow/field/field-pid.hpp>
#include <flow/field/field-research.hpp>
#include <flow/field/field_graph.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <random>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

using namespace branes::anatomy;

namespace {

const std::filesystem::path src = ANATOMY_SOURCE_DIR;
const std::filesystem::path field_dir = src / "tests/fixtures/flow/field";

// The slots of the field mission's slot form.
constexpr std::array<std::string_view, 3> kFieldSlots = {"controller", "estimator", "planner"};

template <class Ctl>
using Slots = field::FieldSlots<GroundTruthCore<double>,
                                field::GoalPlanner<double>,
                                Ctl,
                                NullSafetyFilter<double>,
                                NullDaaTracker<double>,
                                NullFcBridge<double>>;

/// A baseline's file, a vehicle and the world: what a stack is built from.
struct Setup {
    config::VehicleConfig vehicle;
    config::BaselineConfig baseline;
    field::World world;

    [[nodiscard]] field::Context context() const {
        return field::Context{vehicle, baseline, world};
    }
};

Setup setup(std::string_view baseline) {
    const auto vehicle = config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(vehicle);
    const auto b = config::load_baseline(field_dir / (std::string(baseline) + ".yaml"), kFieldSlots);
    if (!b) {
        FAIL(b.error().to_string());
    }
    return Setup{*vehicle, *b, field::World{}};
}

template <class Baseline>
field::FieldGraph<Baseline> graph_stack(const Setup& s) {
    auto st = field::FieldGraph<Baseline>::create(s.context());
    if (!st) {
        FAIL(st.error());
    }
    return std::move(st).value();
}

/// A path in this process's own temporary directory.
std::filesystem::path temp_path(const std::string& name) {
    static const std::filesystem::path dir = [] {
        std::random_device rd;
        for (;;) {
            const auto d = std::filesystem::temp_directory_path() /
                           ("anatomy_flow_stack_" + std::to_string(rd()) + "_" + std::to_string(rd()));
            std::error_code ec;
            if (std::filesystem::create_directory(d, ec)) {
                return d;
            }
            REQUIRE_FALSE(ec);
        }
    }();
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);  // each section's cleanup removes it
    return dir / name;
}

config::ScenarioConfig scenario() {
    config::ScenarioConfig s;
    s.name = "field";
    s.duration_s = 2.0;
    s.seed = 7;
    return s;
}

/// The records of a SITL run of `stack`, as logged.
template <class S>
std::vector<log::LogRecord> sitl_records(S& stack, const Setup& s, const std::string& name) {
    auto sim = sim::BasicSimulator<sim::NoOracle>::create(s.vehicle, s.world.start, scenario(), 100.0, {});
    REQUIRE(sim);
    const auto path = temp_path(name + ".brlog");
    {
        auto w = log::LogWriter::open(path, log::LogHeader{});
        REQUIRE(w);
        const auto run = sim::run_sitl(*sim, stack, {}, *w);
        REQUIRE(run);
        CHECK(*run > 0u);
        REQUIRE(w->close());
    }
    std::vector<log::LogRecord> records;
    auto reader = log::LogReader::open(path, {});
    REQUIRE(reader);
    for (;;) {
        auto rec = reader->next();
        REQUIRE(rec);
        if (!*rec) {
            break;
        }
        records.push_back(**rec);
    }
    return records;
}

/// The records are equal, one by one.
void check_equal(const std::vector<log::LogRecord>& graph, const std::vector<log::LogRecord>& slots) {
    REQUIRE(graph.size() == slots.size());
    std::size_t commands = 0;
    for (std::size_t i = 0; i < graph.size(); ++i) {
        INFO("record " << i);
        CHECK(graph[i].channel == slots[i].channel);
        CHECK(graph[i].t == slots[i].t);
        CHECK(graph[i].payload == slots[i].payload);
        commands += graph[i].channel == log::Channel::command ? 1u : 0u;
    }
    CHECK(commands > 0u);
}

/// The graph a baseline's description builds, with no operators: what the stack must run.
template <class Baseline>
flow::Graph described() {
    flow::builder g{"field", Baseline::kind};
    (void)field::describe<Baseline>(g, field::World{});
    const auto built = g.build();
    if (!built) {
        FAIL(built.error().to_string());
    }
    return *built;
}

}  // namespace

static_assert(RunnableStack<field::FieldGraph<field::baselines::field_null>>);
static_assert(MobileStack<field::FieldGraph<field::baselines::field_null>>);
static_assert(MobileStack<Slots<NullController<double>>>);

TEST_CASE("flow stack: each baseline's stack runs the graph its description builds", "[flow][stack]") {
    const auto null = graph_stack<field::baselines::field_null>(setup("field-null"));
    CHECK(null.stack().graph() == described<field::baselines::field_null>());
    CHECK(null.stack().hash() == flow::graph_hash(described<field::baselines::field_null>()));
    CHECK(null.stack().graph().sitl_only);  // it reads the truth (G9)
    CHECK(null.stack().graph().composites[3] == flow::Composite{"controller", "NullController"});

    const auto pid = graph_stack<field::baselines::field_pid>(setup("field-pid"));
    CHECK(pid.stack().graph() == described<field::baselines::field_pid>());
    CHECK(pid.stack().hash() != null.stack().hash());

    const auto body = graph_stack<field::baselines::field_body>(setup("field-body"));
    CHECK(body.stack().graph() == described<field::baselines::field_body>());
    CHECK(body.stack().graph().instances[1] == flow::Instance{"planner.goal", "GoalPlanner", "planner"});

    const auto research = graph_stack<field::baselines::field_research>(setup("field-research"));
    CHECK(research.stack().graph() == described<field::baselines::field_research>());
    CHECK(research.stack().graph().kind == flow::baseline_class::research);
}

TEST_CASE("flow stack: a graph build's SITL log is the slot build's, record for record", "[flow][stack]") {
    SECTION("single-operator bodies") {
        const Setup s = setup("field-null");
        auto graph = graph_stack<field::baselines::field_null>(s);
        auto slots = Slots<NullController<double>>::create(s.context()).value();
        check_equal(sitl_records(graph, s, "null_graph"), sitl_records(slots, s, "null_slots"));
    }
    SECTION("a controller with state") {
        const Setup s = setup("field-pid");
        auto graph = graph_stack<field::baselines::field_pid>(s);
        auto slots = Slots<glue::CascadedPid<double>>::create(s.context()).value();
        check_equal(sitl_records(graph, s, "pid_graph"), sitl_records(slots, s, "pid_slots"));
    }
    SECTION("a body of several operators") {
        const Setup s = setup("field-body");
        auto graph = graph_stack<field::baselines::field_body>(s);
        auto slots = Slots<NullController<double>>::create(s.context()).value();
        check_equal(sitl_records(graph, s, "body_graph"), sitl_records(slots, s, "body_slots"));
    }
    SECTION("a research graph with an operator of its own, of two outs") {
        const Setup s = setup("field-research");
        auto graph = graph_stack<field::baselines::field_research>(s);
        auto slots = Slots<NullController<double>>::create(s.context()).value();
        check_equal(sitl_records(graph, s, "research_graph"), sitl_records(slots, s, "research_slots"));
    }
    std::error_code ec;
    std::filesystem::remove_all(temp_path("x").parent_path(), ec);
}

TEST_CASE("flow stack: a baseline file that doesn't name the graph's bodies is refused", "[flow][stack]") {
    Setup s = setup("field-null");
    s.baseline.bodies["controller"] = "CascadedPid";
    const auto st = field::FieldGraph<field::baselines::field_null>::create(s.context());
    REQUIRE_FALSE(st);
    CHECK(st.error() == "field-null: graph.controller: G8: the body the file names fills the composite: the file names "
                        "'CascadedPid', and the graph has 'NullController'");
}

TEST_CASE("flow stack: an operator that can't be built is refused, with its error", "[flow][stack]") {
    Setup s = setup("field-pid");
    s.baseline.parameters["control_rate_hz"] = "not a number";
    const auto st = field::FieldGraph<field::baselines::field_pid>::create(s.context());
    REQUIRE_FALSE(st);
    CHECK(st.error().rfind("field: controller: the operator could not be built: ", 0) == 0);
}

TEST_CASE("flow stack: a builder builds one stack, which owns its operators", "[flow][stack]") {
    const Setup s = setup("field-null");
    const field::Context ctx = s.context();
    flow::stack_builder<field::Context> g{"field", flow::baseline_class::scoreboard, ctx};
    (void)field::describe<field::baselines::field_null>(g, s.world);
    auto& builder = static_cast<flow::builder&>(g);
    REQUIRE(std::move(builder).build_stack());
    const auto again = std::move(builder).build_stack();
    REQUIRE_FALSE(again);
    CHECK(again.error().message == "a builder builds one stack, which owns its operators");
}

TEST_CASE("flow stack: a source that is no builder's is refused, not run", "[flow][stack]") {
    namespace fl = branes::anatomy::flow;
    const Setup s = setup("field-research");
    const field::Context ctx = s.context();
    fl::stack_builder<field::Context> g{"field", fl::baseline_class::research, ctx};
    const field::Ports<double> p = field::ports<double>(g, s.world);
    const auto est =
        fl::instance<fl::registered<"GroundTruthCore", GroundTruthCore<double>, field::EstimatorBinding<double>>>(
            g, "estimator");
    const auto direct =
        fl::instance<fl::registered<"Direct", field::Direct<double>, field::DirectBinding<double>>>(g, "direct");
    fl::scope imu = g.region(p.imu);
    fl::fire<"imu">(imu, est, fl::hold(fl::value<ImuSample<double>>{}));  // made by hand: no slot
    fl::scope truth = g.region(p.truth);
    fl::fire<"truth">(truth, est, p.truth);
    fl::scope tick = g.region(p.tick);
    auto [x] = fl::fire<"estimate">(tick, est, fl::at(p.tick));
    auto [reference, command] = fl::fire<"step">(tick, direct, p.field, x);
    g.produce(p.estimate, x);
    g.produce(p.reference, reference);
    g.produce(p.command, command);
    const auto st = std::move(g).build_stack();
    REQUIRE_FALSE(st);
    CHECK(st.error().key == "imu[0][0]");
    CHECK(st.error().message == "the CPU stack has no value for the source: it is no builder's");
}
