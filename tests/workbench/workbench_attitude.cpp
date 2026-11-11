// The attitude filter pattern (workbench/patterns/attitude.hpp;
// docs/plan/computational-domain, C5): its graph runs two filters on four
// inputs, each writing a unit-norm quaternion (W17); its stack writes, bit for
// bit, what its operators write by hand (W1); its outputs meet its oracle
// (W2); and a seed's traces are byte-identical run to run (W3).

#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/attitude.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>
#include <type_traits>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
namespace at = branes::anatomy::workbench::attitude;

const wb::check* find(const wb::report& r, const std::string& name) {
    for (const wb::check& c : r.checks) {
        if (c.name.find(name) != std::string::npos) {
            return &c;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("attitude: the attitude is a unit-norm quaternion, stored as dense (W17)", "[workbench][attitude]") {
    STATIC_REQUIRE(at::QuatD::kind == fl::structure::unit_norm);
    STATIC_REQUIRE(at::Quat::stored == 4);
    STATIC_REQUIRE(std::is_same_v<fl::domain_of_t<at::Quat>, at::QuatD>);
    const auto level = at::Filter<at::start::level>::from_context(at::context{at::config{}});
    REQUIRE(level);
    CHECK(level->q == at::Quat{});  // (1, 0, 0, 0): level
    const at::config c;
    const auto tilted = at::Filter<at::start::tilted>::from_context(at::context{c});
    REQUIRE(tilted);
    CHECK(tilted->q.structure_holds());
    CHECK(std::fabs(at::oracle_detail::tilt_of(tilted->q) - c.tilt) < 1e-15);
}

TEST_CASE("attitude: each filter runs in a region on each of its inputs", "[workbench][attitude]") {
    const auto g = wb::describe<at::pattern>(at::config{});
    REQUIRE(g);
    REQUIRE(g->regions.size() == 2 * at::kFilters);
    for (std::size_t f = 0; f < at::kFilters; ++f) {
        INFO("filter " << at::kFilterNames[f]);
        const std::string n(at::kFilterNames[f]);
        const fl::Region& gyro = g->regions[2 * f];
        const fl::Region& accel = g->regions[2 * f + 1];
        CHECK(gyro.trigger == "gyro_" + n);
        CHECK(accel.trigger == "accel_" + n);
        REQUIRE(gyro.firings.size() == 1);
        REQUIRE(accel.firings.size() == 1);
        CHECK(gyro.firings[0].handler == "propagate");
        CHECK(accel.firings[0].handler == "correct");
        CHECK(gyro.firings[0].state == fl::state_role::update);
        CHECK(accel.firings[0].state == fl::state_role::update);
    }
    CHECK(fl::canonical(*g).find("instance resting AttitudeTilted_f64") != std::string::npos);
}

TEST_CASE("attitude: W1 the stack writes what the operators write by hand", "[workbench][attitude]") {
    const at::config c;
    const auto samples = wb::run<at::pattern>(c, wb::run_options{1000, {}});
    REQUIRE(samples);
    const auto want = at::hand_run(c, 1000);
    REQUIRE(samples->size() == want.size());
    for (std::size_t i = 0; i < want.size(); ++i) {
        INFO("input " << i);
        REQUIRE((*samples)[i] == want[i]);
    }
    // The merged stream: both gyros at 2.5 ms, and at 20 ms both gyros before both accelerometers.
    CHECK(want[0].source == at::gyro_spinning);
    CHECK(want[1].source == at::gyro_resting);
    CHECK(want[0].t.ns == c.gyro_period_ns);
    CHECK(want[16].source == at::accel_spinning);
    CHECK(want[17].source == at::accel_resting);
    CHECK(want[17].t.ns == c.accel_period_ns);
    // At rest, a propagation only renormalizes: the resting filter is still at its tilt.
    CHECK(std::fabs(at::oracle_detail::tilt_of(want[15].propagated[1]) - c.tilt) < 1e-15);
}

TEST_CASE("attitude: W2 the outputs meet the oracle", "[workbench][attitude]") {
    const at::config c;
    const auto samples = wb::run<at::pattern>(c, wb::run_options{at::pattern::default_inputs, {}});
    REQUIRE(samples);
    const wb::report r = at::pattern::oracle(c, *samples);
    INFO(r.to_string());
    CHECK(r.passed());
    CHECK(r.checks.size() == 5);
    CHECK(find(r, "corrections")->value == 200);
}

TEST_CASE("attitude: the oracle fails a filter that's wrong", "[workbench][attitude]") {
    const at::config c;
    const auto samples = wb::run<at::pattern>(c, wb::run_options{at::pattern::default_inputs, {}});
    REQUIRE(samples);
    {
        // A spinning filter 2e-9 off in z at 1 s.
        auto wrong = *samples;
        for (at::sample& s : wrong) {
            if (s.source == at::accel_spinning && s.t.ns == 1'000'000'000) {
                s.corrected[0](3) += 2e-9;
            }
        }
        const wb::report r = at::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "spinning")->passed);
        CHECK(find(r, "resting: tilt")->passed);
    }
    {
        // A resting filter of another gain: its tilts follow another recurrence.
        at::config other = c;
        other.gain = 0.051;
        const auto off = wb::run<at::pattern>(other, wb::run_options{at::pattern::default_inputs, {}});
        REQUIRE(off);
        const wb::report r = at::pattern::oracle(c, *off);
        CHECK_FALSE(find(r, "resting: tilt")->passed);
        CHECK_FALSE(find(r, "decay")->passed);
    }
    {
        // An attitude off norm 1 by 1e-10.
        auto wrong = *samples;
        wrong[100].propagated[0].elements[0] *= 1 + 1e-10;
        const wb::report r = at::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "norm")->passed);
    }
    // Too short a run to reach 1 s.
    const wb::report r = at::pattern::oracle(c, std::span(*samples).first(500));
    CHECK_FALSE(r.passed());
}

TEST_CASE("attitude: W3 a seed's traces are byte-identical run to run", "[workbench][attitude]") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto a = dir / "anatomy_workbench_attitude_a.brtrace";
    const auto b = dir / "anatomy_workbench_attitude_b.brtrace";
    const at::config c;
    REQUIRE(wb::run<at::pattern>(c, wb::run_options{900, a}));
    REQUIRE(wb::run<at::pattern>(c, wb::run_options{900, b}));
    const auto cmp = fl::compare_traces(a, b);
    REQUIRE(cmp);
    CHECK_FALSE(cmp->difference);
    CHECK(cmp->records == 900);
    std::error_code ec;
    std::filesystem::remove(a, ec);
    std::filesystem::remove(b, ec);
}

TEST_CASE("attitude: the operator refuses a gyro period or a gain it can't use", "[workbench][attitude]") {
    using F = at::Filter<at::start::tilted>;
    // The bad values are read through a volatile, so no optimizer sees a constant.
    volatile double zero = 0.0;
    at::config c;
    c.gyro_period_ns = static_cast<std::int64_t>(zero);
    CHECK_FALSE(F::from_context(at::context{c}));
    // So does the graph's stack, which builds its operators from the context.
    CHECK_FALSE(wb::run<at::pattern>(c, wb::run_options{10, {}}));
    c = at::config{};
    c.gain = zero;
    CHECK_FALSE(F::from_context(at::context{c}));
    c.gain = zero + 1.0;
    CHECK_FALSE(F::from_context(at::context{c}));
    CHECK(F::from_context(at::context{at::config{}}));
}
