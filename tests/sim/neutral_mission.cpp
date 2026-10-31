// The framework, driven by a made-up scouting mission: a world with a goal and
// a start pose, a stack of the framework's own slots, a mission channel of its
// own, and its own scorer. It came from racing-drone's framework split
// (racing-drone docs/plan/framework-split), where it showed that the framework
// pieces work for a mission that is not racing. A second stack takes that
// channel as an input, through a profile that extends MobilityProfile with
// it, and the SITL and closed loops run it.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/glue/cascaded_pid.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/mobility/slots/context.hpp>
#include <branes/anatomy/mobility/slots/ground_truth_core.hpp>
#include <branes/anatomy/mobility/slots/neutral.hpp>
#include <branes/anatomy/mobility/slots/neutral_nulls.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>
#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/anatomy/sim/sitl_loop.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <random>
#include <string>
#include <system_error>
#include <vector>

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

/// The mission's depth sensor, as an oracle: at each camera frame, one byte on
/// kDepth, the frame's count (mod 256).
class DepthOracle {
public:
    [[nodiscard]] std::optional<sim::SimEvent> observe(const TruthState<double>&, Timestamp t) {
        return sim::SimEvent{kDepth, t, std::vector<std::byte>{std::byte{frames_++}}};
    }

private:
    std::uint8_t frames_{0};
};
static_assert(sim::OracleSensor<DepthOracle>);

/// A profile that takes the depth channel as well: MobilityProfile with one
/// more input, handed to the stack's on_depth().
struct DepthProfile : MobilityProfile {
    static constexpr std::array<log::Channel, 3> inputs = {log::Channel::imu, log::Channel::camera, kDepth};

    template <class S>
    static constexpr bool drives = MobilityProfile::drives<S> && requires(S s, std::uint8_t d) { s.on_depth(d); };

    template <class S>
    [[nodiscard]] static result<ok_t, std::string> feed(S& s, const log::LogRecord& rec) {
        if (rec.channel != kDepth) {
            return MobilityProfile::feed(s, rec);
        }
        if (rec.payload.size() != 1) {
            return std::string("undecodable depth record at t=" + std::to_string(rec.t.ns));
        }
        s.on_depth(std::to_integer<std::uint8_t>(rec.payload[0]));
        return ok;
    }
};
static_assert(StackProfile<DepthProfile>);
static_assert(MobileProfile<DepthProfile, double>);

/// FieldStack, which takes the depth channel and counts its records and the frames.
class DepthStack : public FieldStack {
public:
    using profile = DepthProfile;

    void on_frame(const CameraFrame&) {
        ++frames_;
    }
    void on_depth(std::uint8_t d) {
        last_ = d;
        ++depths_;
    }
    [[nodiscard]] std::uint64_t frames() const noexcept {
        return frames_;
    }
    [[nodiscard]] std::uint64_t depths() const noexcept {
        return depths_;
    }
    [[nodiscard]] std::uint8_t last_depth() const noexcept {
        return last_;
    }

private:
    std::uint64_t frames_{0};
    std::uint64_t depths_{0};
    std::uint8_t last_{0};
};
static_assert(RunnableStack<DepthStack>);
static_assert(MobileStack<DepthStack>);  // so the SITL and closed loops take it

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

/// A path in this process's own temporary directory, so concurrent runs of the
/// test never share files.
std::filesystem::path temp_path(const std::string& name) {
    static const std::filesystem::path dir = [] {
        std::random_device rd;
        for (;;) {
            const auto d = std::filesystem::temp_directory_path() /
                           ("anatomy_neutral_mission_" + std::to_string(rd()) + "_" + std::to_string(rd()));
            std::error_code ec;
            if (std::filesystem::create_directory(d, ec)) {
                return d;  // created by this process: no other run uses it
            }
            REQUIRE_FALSE(ec);  // existed already: draw another name
        }
    }();
    return dir / name;
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

TEST_CASE("neutral: the closed loop hands a stack the mission's own input", "[neutral]") {
    const auto vehicle = config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(vehicle);
    const Field field{};
    auto sim = sim::BasicSimulator<DepthOracle>::create(*vehicle, field.start, field_scenario(1.0), 100.0, {});
    REQUIRE(sim);
    DepthStack stack;
    FieldScorer scorer(50);
    // No pass-through channels: kDepth is the stack's input.
    const auto run = sim::run_closed_loop<FieldScore>(*sim, stack, {}, scorer, 7);
    CHECK(run.error.empty());
    CHECK(run.score.truths == 50u);
    CHECK(stack.frames() > 0u);
    CHECK(stack.depths() == stack.frames());  // one depth record per frame, each consumed
    CHECK(stack.last_depth() == static_cast<std::uint8_t>(stack.frames() - 1));
}

TEST_CASE("neutral: the SITL loop logs the mission's own input and hands it to the stack", "[neutral]") {
    const auto vehicle = config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(vehicle);
    const Field field{};
    auto sim = sim::BasicSimulator<DepthOracle>::create(*vehicle, field.start, field_scenario(0.5), 100.0, {});
    REQUIRE(sim);
    DepthStack stack;
    const auto log_path = temp_path("depth.brlog");
    std::uint64_t ticks = 0;
    {
        auto w = log::LogWriter::open(log_path, log::LogHeader{});
        REQUIRE(w);
        const auto run = sim::run_sitl(*sim, stack, {}, *w);  // kDepth: the stack's input
        REQUIRE(run);
        ticks = *run;
        REQUIRE(w->close());
    }
    CHECK(ticks > 0u);
    CHECK(stack.frames() > 0u);
    CHECK(stack.depths() == stack.frames());

    // The log holds every depth record the stack took, and a command per tick.
    const std::array<log::Channel, 1> mission = {kDepth};
    std::uint64_t depths = 0;
    std::uint64_t commands = 0;
    {
        auto reader = log::LogReader::open(log_path, mission);  // scoped: closed before cleanup
        REQUIRE(reader);
        for (;;) {
            auto rec = reader->next();
            REQUIRE(rec);
            if (!*rec) {
                break;
            }
            depths += (*rec)->channel == kDepth ? 1u : 0u;
            commands += (*rec)->channel == log::Channel::command ? 1u : 0u;
        }
    }
    CHECK(depths == stack.depths());
    CHECK(commands == ticks);
    std::error_code ec;
    std::filesystem::remove_all(log_path.parent_path(), ec);
}
