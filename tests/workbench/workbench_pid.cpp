// The PID control pattern (workbench/patterns/pid.hpp;
// docs/plan/computational-domain, C4): its graph closes three loops through
// carried edges; its stack writes, bit for bit, what its operators write by
// hand (W1); its outputs meet its oracle (W2); and a seed's traces are
// byte-identical run to run (W3).

#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/pid.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
namespace pid = branes::anatomy::workbench::pid;

const wb::check* find(const wb::report& r, const std::string& name) {
    for (const wb::check& c : r.checks) {
        if (c.name.find(name) != std::string::npos) {
            return &c;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("pid: each loop is closed through a carried edge in one region", "[workbench][pid]") {
    const auto g = wb::describe<pid::pattern>(pid::config{});
    REQUIRE(g);
    REQUIRE(g->regions.size() == 1);
    const fl::Region& r = g->regions[0];
    CHECK(r.trigger == "clock");
    REQUIRE(r.firings.size() == 2 * pid::kLoops);
    for (std::size_t l = 0; l < pid::kLoops; ++l) {
        INFO("loop " << pid::kLoopNames[l]);
        const fl::Firing& controller = r.firings[2 * l];
        const fl::Firing& plant = r.firings[2 * l + 1];
        CHECK(controller.op == "controller_" + std::string(pid::kLoopNames[l]));
        CHECK(controller.state == fl::state_role::update);
        CHECK(controller.in[0] == fl::Ref{fl::Ref::kind::constant, "", 0, "setpoint", {}, fl::rate_rule::none, 0});
        // The controller reads the plant's output from the last tick: a firing after its own.
        CHECK(controller.in[1] ==
              fl::Ref{fl::Ref::kind::firing, "clock", 2 * l + 1, "y", {}, fl::rate_rule::carried, 0});
        CHECK(plant.in[0] == fl::Ref{fl::Ref::kind::firing, "clock", 2 * l, "u", {}, fl::rate_rule::none, 0});
    }
    CHECK(fl::canonical(*g).find("\nin firing clock#1 y carried\n") != std::string::npos);
    CHECK(fl::mermaid(*g).find(R"x(r0f1 -.->|"y (carried)"| r0f0)x") != std::string::npos);
}

TEST_CASE("pid: W1 the stack writes what the operators write by hand", "[workbench][pid]") {
    const pid::config c;
    const auto samples = wb::run<pid::pattern>(c, wb::run_options{600, {}});
    REQUIRE(samples);
    const auto want = pid::hand_run(c, 600);
    REQUIRE(samples->size() == want.size());
    for (std::size_t i = 0; i < want.size(); ++i) {
        INFO("tick " << i);
        REQUIRE((*samples)[i] == want[i]);
    }
    // At the first tick each controller reads the carried edge value-initialized: e = r.
    const double r = c.setpoint;
    const double h = c.h();
    CHECK(want[0].u[0] == c.kp * r);
    CHECK(want[0].u[1] == c.kp * r + c.ki * (h * r));
    CHECK(want[0].u[2] == c.kp * r + c.ki * (h * r) + c.kd * (r / h));
}

TEST_CASE("pid: W2 the outputs meet the oracle", "[workbench][pid]") {
    const pid::config c;
    const auto samples = wb::run<pid::pattern>(c, wb::run_options{pid::pattern::default_inputs, {}});
    REQUIRE(samples);
    const wb::report r = pid::pattern::oracle(c, *samples);
    INFO(r.to_string());
    CHECK(r.passed());
    CHECK(r.checks.size() == 6);
    CHECK(find(r, "overshoot")->value > 0.2);
}

TEST_CASE("pid: the oracle fails a loop that's wrong", "[workbench][pid]") {
    const pid::config c;
    auto samples = wb::run<pid::pattern>(c, wb::run_options{pid::pattern::default_inputs, {}});
    REQUIRE(samples);
    {
        // A loop that reads this tick's output, not the last: it is another recurrence.
        auto wrong = *samples;
        for (std::size_t k = 1; k < wrong.size(); ++k) {
            wrong[k].y[0] = (*samples)[k - 1].y[0];
        }
        const wb::report r = pid::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "P loop")->passed);
    }
    {
        // A PI loop that settles 1e-6 off, and peaks 2 % higher.
        auto wrong = *samples;
        for (pid::sample& s : wrong) {
            s.y[1] = c.setpoint + (s.y[1] - c.setpoint) * 1.02 + 1e-6;
        }
        const wb::report r = pid::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "PI loop's error")->passed);
        CHECK_FALSE(find(r, "against its poles")->passed);
    }
    // Too short a run to settle.
    const wb::report r = pid::pattern::oracle(c, std::span(*samples).first(500));
    CHECK_FALSE(r.passed());
}

TEST_CASE("pid: W3 a seed's traces are byte-identical run to run", "[workbench][pid]") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto a = dir / "anatomy_workbench_pid_a.brtrace";
    const auto b = dir / "anatomy_workbench_pid_b.brtrace";
    const pid::config c;
    REQUIRE(wb::run<pid::pattern>(c, wb::run_options{500, a}));
    REQUIRE(wb::run<pid::pattern>(c, wb::run_options{500, b}));
    const auto cmp = fl::compare_traces(a, b);
    REQUIRE(cmp);
    CHECK_FALSE(cmp->difference);
    CHECK(cmp->records == 500 * 2 * pid::kLoops);
    std::error_code ec;
    std::filesystem::remove(a, ec);
    std::filesystem::remove(b, ec);
}

TEST_CASE("pid: the operators refuse a clock or a plant they can't discretize", "[workbench][pid]") {
    // The bad values are read through a volatile: given a constant 0, MSVC's
    // optimizer warns of a division the check before it never reaches (C4723).
    volatile double zero = 0.0;
    pid::config c;
    c.clock_period_ns = static_cast<std::int64_t>(zero);
    CHECK_FALSE(pid::Pid<pid::terms::pid>::from_context(pid::context{c}));
    // So does the graph's stack, which builds its operators from the context.
    CHECK_FALSE(wb::run<pid::pattern>(c, wb::run_options{10, {}}));
    c = pid::config{};
    c.tau = zero;
    CHECK_FALSE(pid::Plant::from_context(pid::context{c}));
    c.tau = zero - 0.5;
    CHECK_FALSE(pid::Plant::from_context(pid::context{c}));
    CHECK(pid::Plant::from_context(pid::context{pid::config{}}));
}
