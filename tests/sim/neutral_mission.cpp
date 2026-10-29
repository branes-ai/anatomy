// The framework, driven by a made-up scouting mission: a world with a goal and
// a start pose, a stack of the framework's own slots, a mission channel of its
// own, and its own scorer. It came from racing-drone's framework split
// (racing-drone docs/plan/framework-split), where it showed that the framework
// pieces work for a mission that is not racing.

#include <branes/anatomy/compose/tick.hpp>
#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/config/scenario.hpp>
#include <branes/anatomy/config/vehicle.hpp>
#include <branes/anatomy/glue/cascaded_pid.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/msg/region.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>
#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/anatomy/slots/ground_truth_core.hpp>
#include <branes/anatomy/slots/neutral.hpp>
#include <branes/anatomy/slots/neutral_nulls.hpp>
#include <branes/anatomy/slots/slot_context.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>

using namespace branes::anatomy;

namespace {

const std::filesystem::path src = ANATOMY_SOURCE_DIR;

/// A scouting mission's world: a goal and a start pose, no gates.
struct Field {
    StartPose<double> start{{0.0, 0.0, 0.1}, 0.0};
    Vec3<double> goal{40.0, 0.0, 2.0};
};
static_assert(MissionWorld<Field>);

/// A depth-image channel of that mission (unused by the framework).
constexpr log::Channel kDepth = static_cast<log::Channel>(7);

/// A stack of the mission: the truth as the estimate, a fixed reference, and
/// the neutral null controller. It satisfies RunnableStack.
class FieldStack {
public:
    using scalar_type = double;
    using profile = MobilityProfile;

    void on_imu(const ImuSample<double>& s) {
        est_.process_imu(s);
    }
    void on_frame(const CameraFrame&) {}
    void on_truth(const TruthState<double>& x) {
        est_.process_truth(x);
    }
    TickOutput<double> on_tick(Timestamp now) {
        const StateEstimate<double> x = est_.state_at(now);
        Reference<double> r{};
        r.count = 1;
        r.samples[0].t = now;
        r.samples[0].position = Vec3<double>{40.0, 0.0, 2.0};
        return TickOutput<double>{x, r, ctl_.step(x, r, ConflictSet<double>{})};
    }

private:
    GroundTruthCore<double> est_;
    NullController<double> ctl_;
};
static_assert(RunnableStack<FieldStack>);
static_assert(MobileStack<FieldStack>);

/// The mission's score: how many truth records the run saw.
struct FieldScore {
    std::uint64_t truths{0};
};

/// The mission's scorer: complete after `limit` truth records.
class FieldScorer {
public:
    explicit FieldScorer(std::uint64_t limit) : limit_(limit) {}
    void on_truth(const TruthState<double>&) {
        ++score_.truths;
    }
    void on_estimate(const StateEstimate<double>&) {}
    [[nodiscard]] bool complete() const noexcept {
        return score_.truths >= limit_;
    }
    [[nodiscard]] bool failed() const noexcept {
        return false;
    }
    [[nodiscard]] FieldScore score() const {
        return score_;
    }

private:
    std::uint64_t limit_;
    FieldScore score_;
};
static_assert(sim::RunScorer<FieldScorer>);

/// A scenario of the mission, built in code: no world file, no mission sim parameters.
config::ScenarioConfig field_scenario(double duration_s) {
    config::ScenarioConfig s;
    s.name = "field";
    s.duration_s = duration_s;
    s.seed = 7;
    return s;
}

}  // namespace

TEST_CASE("neutral: the runner drives another mission's stack and passes its channel through", "[neutral]") {
    FieldStack stack;
    const std::array<log::Channel, 1> mission = {kDepth};
    harness::Runner runner(stack, mission);

    const auto tick = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{1'000'000}, {}});
    REQUIRE(tick);
    CHECK(tick->size() == 3u);

    const auto depth = runner.feed(log::LogRecord{kDepth, Timestamp{2'000'000}, {}});
    REQUIRE(depth);
    CHECK(depth->empty());  // recorded, not consumed

    // Another mission's channel (5) is not this mission's: it is not an input here.
    const auto other = runner.feed(log::LogRecord{static_cast<log::Channel>(5), Timestamp{3'000'000}, {}});
    REQUIRE_FALSE(other);
    CHECK(other.error() == "not an input channel");
}

TEST_CASE("neutral: the cascaded PID builds from another mission's slot context", "[neutral]") {
    const auto vehicle = config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(vehicle);
    const config::BaselineConfig baseline{"field", {}, {}};
    const Field field{};
    const BasicSlotContext<Field> ctx{*vehicle, baseline, field};
    static_assert(ContextBuilt<glue::CascadedPid<double>, BasicSlotContext<Field>>);
    const auto pid = make_slot<glue::CascadedPid<double>>(ctx);
    REQUIRE(pid);
}

TEST_CASE("neutral: the simulator runs another mission's start pose without an oracle", "[neutral]") {
    const auto vehicle = config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(vehicle);
    const Field field{};
    auto sim = sim::BasicSimulator<sim::NoOracle>::create(*vehicle, field.start, field_scenario(0.1), 100.0, {});
    REQUIRE(sim);
    const auto first = sim->next();
    REQUIRE(first);
    CHECK(first->channel == log::Channel::sim_config);
    std::uint64_t truths = 0;
    while (auto ev = sim->next()) {
        CHECK(log::is_framework(static_cast<std::uint16_t>(ev->channel)));  // no oracle, so no mission channel
        if (ev->channel == log::Channel::truth) {
            const auto x = codec::decode_truth(ev->payload);
            REQUIRE(x);
            if (truths == 0) {
                CHECK(x->position[0] == field.start.position[0]);
                CHECK(x->position[1] == field.start.position[1]);
            }
            ++truths;
        }
    }
    CHECK(truths > 0u);
}

TEST_CASE("neutral: the closed loop runs another mission's stack and scorer", "[neutral]") {
    const auto vehicle = config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(vehicle);
    const Field field{};
    auto sim = sim::BasicSimulator<sim::NoOracle>::create(*vehicle, field.start, field_scenario(1.0), 100.0, {});
    REQUIRE(sim);
    FieldStack stack;
    FieldScorer scorer(50);
    const std::array<log::Channel, 1> mission = {kDepth};
    const auto run = sim::run_closed_loop<FieldScore>(*sim, stack, mission, scorer, 7);
    CHECK(run.error.empty());
    CHECK(run.seed == 7u);
    CHECK(run.score.truths == 50u);  // stopped once the scorer was complete
}
