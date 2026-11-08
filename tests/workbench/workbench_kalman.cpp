// The Kalman filter pattern (workbench/patterns/kalman.hpp;
// docs/plan/workbench, B4): its stack writes, bit for bit, what its operators
// write by hand (W1); its outputs meet its oracle, with one fix per update and
// with up to four or none (W2); and a seed's traces are byte-identical run to
// run (W3). Its graph is the first with an operator that updates its state in
// two regions, fired by two triggers at shared times.

#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/kalman.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
namespace kf = branes::anatomy::workbench::kalman;

/// Up to four fixes per update, and a fifth of the updates with none.
kf::config varied(std::uint64_t seed) {
    kf::config c;
    c.seed = seed;
    c.max_fixes = 4;
    c.p_none = 0.2;
    return c;
}

const wb::check* find(const wb::report& r, const std::string& name) {
    for (const wb::check& c : r.checks) {
        if (c.name.find(name) != std::string::npos) {
            return &c;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("kalman: predict and update update the state in two regions", "[workbench][kalman]") {
    const auto g = wb::describe<kf::pattern>(kf::config{});
    REQUIRE(g);
    REQUIRE(g->regions.size() == 2);
    const fl::Region& clock = g->regions[0];
    const fl::Region& fixes = g->regions[1];
    CHECK(clock.trigger == "clock");
    CHECK(fixes.trigger == "fixes");
    CHECK(clock.firings[0].handler == "predict");
    CHECK(clock.firings[0].at == "clock");
    CHECK(clock.firings[0].state == fl::state_role::update);
    CHECK(clock.firings[1].handler == "prior");
    CHECK(clock.firings[1].state == fl::state_role::read);
    CHECK(fixes.firings[0].op == "kalman");
    CHECK(fixes.firings[0].handler == "update");
    CHECK(fixes.firings[0].state == fl::state_role::update);
    CHECK(fixes.firings[3].op == "kalman_f32");
    CHECK(fixes.firings[3].in[0] == fl::Ref{fl::Ref::kind::firing, "fixes", 2, "y", {}, fl::rate_rule::none, 0});
    CHECK(g->instances[2].op == "Cast_f64_f32_fixes4x2");
}

TEST_CASE("kalman: W1 the stack writes what the operators write by hand", "[workbench][kalman]") {
    for (const kf::config& c : {kf::config{}, varied(3)}) {
        const auto samples = wb::run<kf::pattern>(c, wb::run_options{3000, {}});
        REQUIRE(samples);
        const auto want = kf::hand_run(c, 3000);
        REQUIRE(samples->size() == want.size());
        std::size_t updates = 0;
        for (std::size_t i = 0; i < want.size(); ++i) {
            INFO("input " << i);
            REQUIRE((*samples)[i] == want[i]);
            updates += want[i].source;
        }
        // The clock's ticks and the fixes, merged: ten ticks per update, the tick first.
        CHECK(updates == 3000 / 11 + 1);
        CHECK(want[0].source == 0);
        CHECK(want[1].source == 1);
        CHECK(want[1].t == want[0].t);
    }
}

TEST_CASE("kalman: W2 the outputs meet the oracle, one fix per update", "[workbench][kalman]") {
    // NEES is a 95 % test, so about one seed in twenty fails it: of seeds 1 to
    // 100, 2, 12, 42, 67, 75 and 93 do. These fixed seeds pass, each with its
    // mean well inside the bounds.
    for (const std::uint64_t seed : {1ULL, 3ULL, 4ULL}) {
        kf::config c;
        c.seed = seed;
        const auto samples = wb::run<kf::pattern>(c, wb::run_options{kf::pattern::default_inputs, {}});
        REQUIRE(samples);
        const wb::report r = kf::pattern::oracle(c, *samples);
        INFO(r.to_string());
        CHECK(r.passed());
        REQUIRE(find(r, "Riccati") != nullptr);
        REQUIRE(find(r, "NEES mean, above") != nullptr);
        CHECK(find(r, "updates")->value == 2000);
    }
}

TEST_CASE("kalman: W2 the outputs meet the oracle, with up to four fixes or none", "[workbench][kalman]") {
    // Fixed seeds, each well inside the NEES bounds (of seeds 1 to 10, 2 falls just below them).
    for (const std::uint64_t seed : {1ULL, 3ULL}) {
        const kf::config c = varied(seed);
        const auto samples = wb::run<kf::pattern>(c, wb::run_options{kf::pattern::default_inputs, {}});
        REQUIRE(samples);
        const wb::report r = kf::pattern::oracle(c, *samples);
        INFO(r.to_string());
        CHECK(r.passed());
        CHECK(find(r, "Riccati") == nullptr);  // it needs one fix per update
    }
    // Some updates have none, and some have four.
    const kf::config c = varied(1);
    kf::trajectory truth(c);
    std::size_t none = 0;
    std::size_t four = 0;
    for (std::uint64_t k = 0; k < 200; ++k) {
        const auto z = kf::fixes_of(c, truth, k);
        none += z.count() == 0 ? 1 : 0;
        four += z.count() == 4 ? 1 : 0;
    }
    CHECK(none > 0);
    CHECK(four > 0);
}

TEST_CASE("kalman: the oracle fails a filter that's wrong", "[workbench][kalman]") {
    const kf::config c;
    auto samples = wb::run<kf::pattern>(c, wb::run_options{6000, {}});
    REQUIRE(samples);
    // The 500th update's covariance, off by a part in 10⁶; and an f32 mean, off by 1 %.
    std::size_t updates = 0;
    for (kf::sample& s : *samples) {
        if (s.source == 1 && ++updates == 500) {
            s.posterior_cov(0, 0) *= 1.000001;
            s.posterior_mean_f32(0) *= 1.01f;
        }
    }
    const wb::report r = kf::pattern::oracle(c, *samples);
    CHECK_FALSE(r.passed());
    CHECK_FALSE(find(r, "Riccati")->passed);
    CHECK_FALSE(find(r, "f32 posterior mean")->passed);
}

TEST_CASE("kalman: W3 a seed's traces are byte-identical run to run", "[workbench][kalman]") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto a = dir / "anatomy_workbench_kalman_a.brtrace";
    const auto b = dir / "anatomy_workbench_kalman_b.brtrace";
    const kf::config c = varied(5);
    REQUIRE(wb::run<kf::pattern>(c, wb::run_options{2000, a}));
    REQUIRE(wb::run<kf::pattern>(c, wb::run_options{2000, b}));
    const auto cmp = fl::compare_traces(a, b);
    REQUIRE(cmp);
    CHECK_FALSE(cmp->difference);
    CHECK(cmp->records > 2000);
    std::error_code ec;
    std::filesystem::remove(a, ec);
    std::filesystem::remove(b, ec);
}
