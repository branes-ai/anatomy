// The harness, driven by a stack that is not a vehicle's: a made-up watch
// post with its own profile (anatomy docs/plan/generalize-core). Its truth is
// two targets, its one input is a range measurement on a channel of its own,
// and each tick produces an assessment, not a state estimate and a command.
// The Runner, replay_log and verify_outputs drive it unchanged, which shows
// that the core's outputs, truth and events take the mission's types.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/compose/tick.hpp>
#include <branes/anatomy/harness/replay.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <system_error>
#include <vector>

using namespace branes::anatomy;

namespace watch {

using branes::anatomy::Output;
using branes::anatomy::Timestamp;
using branes::anatomy::Vec3;
namespace log = branes::anatomy::log;

constexpr log::Channel kRange = static_cast<log::Channel>(8);        ///< input: one range measurement
constexpr log::Channel kAssessment = static_cast<log::Channel>(19);  ///< output: one assessment per tick

/// The scene's truth: two targets.
template <typename T>
struct SceneTruth {
    Timestamp t;
    std::array<Vec3<T>, 2> targets{};

    friend constexpr bool operator==(const SceneTruth&, const SceneTruth&) = default;
};

template <typename T>
[[nodiscard]] bool valid(const SceneTruth<T>& x) noexcept {
    for (const auto& p : x.targets) {
        for (const T v : p) {
            if (!std::isfinite(v)) {
                return false;
            }
        }
    }
    return true;
}

template <typename To, typename From>
[[nodiscard]] SceneTruth<To> cast(const SceneTruth<From>& x) noexcept {
    return SceneTruth<To>{x.t, {branes::anatomy::cast<To>(x.targets[0]), branes::anatomy::cast<To>(x.targets[1])}};
}

/// A range measurement [m]. Invariant: finite and ≥ 0.
template <typename T>
struct Range {
    T r{};
};

template <typename T>
[[nodiscard]] bool valid(const Range<T>& m) noexcept {
    return std::isfinite(m.r) && m.r >= T(0);
}

/// What one tick assesses: how many ranges it saw, the last one, and how far
/// apart the true targets were (SITL).
template <typename T>
struct Assessment {
    std::uint32_t measurements{0};
    T last_range{};
    T spread{};

    friend constexpr bool operator==(const Assessment&, const Assessment&) = default;
};

std::vector<std::byte> encode(const SceneTruth<double>& x) {
    branes::anatomy::ByteWriter w;
    w.i64(x.t.ns);
    for (const auto& p : x.targets) {
        for (const double v : p) {
            w.f64(v);
        }
    }
    return w.take();
}

std::vector<std::byte> encode(const Range<double>& m) {
    branes::anatomy::ByteWriter w;
    w.f64(m.r);
    return w.take();
}

std::optional<Assessment<double>> decode_assessment(std::span<const std::byte> b) {
    branes::anatomy::ByteReader r(b);
    Assessment<double> a;
    a.measurements = r.u32();
    a.last_range = r.f64();
    a.spread = r.f64();
    if (r.failed() || !r.done()) {
        return std::nullopt;
    }
    return a;
}

/// The watch post's profile.
struct WatchProfile {
    template <typename T>
    using truth_type = SceneTruth<T>;
    template <typename T>
    using output_type = Assessment<T>;

    static constexpr std::array<log::Channel, 1> inputs = {kRange};
    static constexpr std::array<log::Channel, 1> outputs = {kAssessment};

    template <class S>
    static constexpr bool drives = requires(S s, const Range<typename S::scalar_type>& m) { s.on_range(m); };

    [[nodiscard]] static std::optional<SceneTruth<double>> decode_truth(std::span<const std::byte> b) {
        branes::anatomy::ByteReader r(b);
        SceneTruth<double> x;
        x.t = Timestamp{r.i64()};
        for (auto& p : x.targets) {
            for (double& v : p) {
                v = r.f64();
            }
        }
        if (r.failed() || !r.done()) {
            return std::nullopt;
        }
        return x;
    }

    template <class S>
    [[nodiscard]] static branes::anatomy::result<branes::anatomy::ok_t, std::string> feed(S& s,
                                                                                          const log::LogRecord& rec) {
        using T = typename S::scalar_type;
        branes::anatomy::ByteReader r(rec.payload);
        const double v = r.f64();
        if (r.failed() || !r.done()) {
            return std::string("undecodable range record at t=" + std::to_string(rec.t.ns));
        }
        const Range<T> m{static_cast<T>(v)};
        if (!valid(m)) {
            return std::string("range record at t=" + std::to_string(rec.t.ns) +
                               " is not valid in the stack's scalar type");
        }
        s.on_range(m);
        return branes::anatomy::ok;
    }

    template <typename T>
    static void encode(const Assessment<T>& y, Timestamp t, std::vector<Output>& out) {
        branes::anatomy::ByteWriter w;
        w.u32(y.measurements);
        w.f64(static_cast<double>(y.last_range));
        w.f64(static_cast<double>(y.spread));
        out.push_back(Output{kAssessment, t, w.take()});
    }
};

/// The watch post's stack: it counts ranges and reads the truth.
template <typename T>
class WatchStack {
public:
    using scalar_type = T;
    using profile = WatchProfile;

    void on_range(const Range<T>& m) {
        ++seen_.measurements;
        seen_.last_range = m.r;
    }
    void on_truth(const SceneTruth<T>& x) {
        T d2 = T(0);
        for (std::size_t i = 0; i < 3; ++i) {
            const T d = x.targets[0][i] - x.targets[1][i];
            d2 += d * d;
        }
        seen_.spread = std::sqrt(d2);
    }
    Assessment<T> on_tick(Timestamp) {
        return seen_;
    }

private:
    Assessment<T> seen_{};
};

}  // namespace watch

namespace {

using watch::WatchProfile;
using watch::WatchStack;

// ── compile-time checks ─────────────────────────────────────────────────

static_assert(StackProfile<WatchProfile>);
static_assert(RunnableStack<WatchStack<double>>);
static_assert(RunnableStack<WatchStack<float>>);
static_assert(!MobileStack<WatchStack<double>>);  // so the SITL and closed loops refuse it

/// A stack that names no profile is not runnable: there is no default.
struct NoProfileStack {
    using scalar_type = double;
    void on_range(const watch::Range<double>&) {}
    void on_truth(const watch::SceneTruth<double>&) {}
    watch::Assessment<double> on_tick(Timestamp) {
        return {};
    }
};
static_assert(!RunnableStack<NoProfileStack>);

/// A stack that names the mobility profile without its input hooks is not runnable.
struct WrongProfileStack {
    using scalar_type = double;
    using profile = MobilityProfile;
    void on_range(const watch::Range<double>&) {}
    void on_truth(const TruthState<double>&) {}
    TickOutput<double> on_tick(Timestamp) {
        return {};
    }
};
static_assert(!RunnableStack<WrongProfileStack>);

/// The watch profile with other channels, to check P1–P3.
template <auto In, auto Out>
struct Channels : WatchProfile {
    static constexpr auto inputs = In;
    static constexpr auto outputs = Out;
};
constexpr auto kOut = std::array{watch::kAssessment};
static_assert(StackProfile<Channels<std::array{watch::kRange}, kOut>>);
static_assert(!StackProfile<Channels<std::array{log::Channel::command}, kOut>>);               // P1: an output as input
static_assert(!StackProfile<Channels<std::array{log::Channel::tick}, kOut>>);                  // P1: the harness's own
static_assert(!StackProfile<Channels<std::array{log::Channel::truth}, kOut>>);                 // P1: the harness's own
static_assert(!StackProfile<Channels<std::array{watch::kRange}, std::array{watch::kRange}>>);  // P2
static_assert(!StackProfile<Channels<std::array{watch::kRange, watch::kRange}, kOut>>);        // P3
static_assert(!StackProfile<Channels<std::array{watch::kRange}, std::array{watch::kAssessment, watch::kAssessment}>>);

// ── helpers ─────────────────────────────────────────────────────────────

constexpr std::array<log::Channel, 2> kWatchChannels = {watch::kRange, watch::kAssessment};

watch::SceneTruth<double> scene(std::int64_t ns, double apart) {
    return watch::SceneTruth<double>{Timestamp{ns}, {Vec3<double>{0.0, 0.0, 0.0}, Vec3<double>{apart, 0.0, 0.0}}};
}

/// A path in this process's own temporary directory.
std::filesystem::path temp_path(const std::string& name) {
    static const std::filesystem::path dir = [] {
        std::random_device rd;
        for (;;) {
            const auto d = std::filesystem::temp_directory_path() /
                           ("anatomy_profile_" + std::to_string(rd()) + "_" + std::to_string(rd()));
            std::error_code ec;
            if (std::filesystem::create_directory(d, ec)) {
                return d;
            }
            REQUIRE_FALSE(ec);
        }
    }();
    return dir / name;
}

}  // namespace

TEST_CASE("profile: the runner feeds another profile's events and encodes its output", "[profile]") {
    WatchStack<double> stack;
    harness::Runner runner(stack, std::span<const log::Channel>{});

    REQUIRE(runner.feed(log::LogRecord{log::Channel::truth, Timestamp{1}, watch::encode(scene(1, 3.0))}));
    REQUIRE(runner.feed(log::LogRecord{watch::kRange, Timestamp{2}, watch::encode(watch::Range<double>{12.5})}));
    REQUIRE(runner.feed(log::LogRecord{watch::kRange, Timestamp{3}, watch::encode(watch::Range<double>{7.0})}));
    REQUIRE(runner.feed(log::LogRecord{log::Channel::sim_config, Timestamp{0}, {}}));

    const auto out = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{10}, {}});
    REQUIRE(out);
    REQUIRE(out->size() == 1u);
    CHECK((*out)[0].channel == watch::kAssessment);
    CHECK((*out)[0].t == Timestamp{10});
    const auto a = watch::decode_assessment((*out)[0].payload);
    REQUIRE(a);
    CHECK(*a == watch::Assessment<double>{2, 7.0, 3.0});
}

TEST_CASE("profile: the runner rejects malformed records of another profile", "[profile]") {
    WatchStack<double> stack;
    harness::Runner runner(stack, std::span<const log::Channel>{});

    const auto short_range = runner.feed(log::LogRecord{watch::kRange, Timestamp{5}, {}});
    REQUIRE_FALSE(short_range);
    CHECK(short_range.error() == "undecodable range record at t=5");

    const auto negative =
        runner.feed(log::LogRecord{watch::kRange, Timestamp{6}, watch::encode(watch::Range<double>{-1.0})});
    REQUIRE_FALSE(negative);
    CHECK(negative.error() == "range record at t=6 is not valid in the stack's scalar type");

    const auto short_truth = runner.feed(log::LogRecord{log::Channel::truth, Timestamp{7}, {}});
    REQUIRE_FALSE(short_truth);
    CHECK(short_truth.error() == "undecodable truth record at t=7");

    const auto nan_truth = runner.feed(log::LogRecord{
        log::Channel::truth, Timestamp{8}, watch::encode(scene(8, std::numeric_limits<double>::quiet_NaN()))});
    REQUIRE_FALSE(nan_truth);
    CHECK(nan_truth.error() == "undecodable truth record at t=8");

    // A mobility channel is not this profile's input.
    const auto imu = runner.feed(log::LogRecord{log::Channel::imu, Timestamp{9}, {}});
    REQUIRE_FALSE(imu);
    CHECK(imu.error() == "not an input channel");

    REQUIRE(runner.feed(log::LogRecord{log::Channel::tick, Timestamp{20}, {}}));
    const auto early = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{19}, {}});
    REQUIRE_FALSE(early);
    CHECK(early.error() == "tick at t=19 is earlier than the previous tick (t=20)");

    // Nothing rejected reached the stack.
    const auto out = runner.feed(log::LogRecord{log::Channel::tick, Timestamp{21}, {}});
    REQUIRE(out);
    CHECK(watch::decode_assessment((*out)[0].payload) == watch::Assessment<double>{});
}

TEST_CASE("profile: records valid in double but not in the stack's scalar type are rejected", "[profile]") {
    WatchStack<float> stack;
    harness::Runner runner(stack, std::span<const log::Channel>{});

    const auto range =
        runner.feed(log::LogRecord{watch::kRange, Timestamp{1}, watch::encode(watch::Range<double>{1e300})});
    REQUIRE_FALSE(range);
    CHECK(range.error() == "range record at t=1 is not valid in the stack's scalar type");

    const auto truth = runner.feed(log::LogRecord{log::Channel::truth, Timestamp{2}, watch::encode(scene(2, 1e300))});
    REQUIRE_FALSE(truth);
    CHECK(truth.error() == "truth record at t=2 is not valid in the stack's scalar type");
}

TEST_CASE("profile: a replay of another profile's log reproduces its outputs", "[profile]") {
    const auto recorded = temp_path("watch.brlog");
    const auto replayed = temp_path("watch_replay.brlog");
    const log::LogHeader header{"watch", "c", "w", "v", "none", true, "world: {}\n", "vehicle: {}\n"};

    // Record a run: truth, ranges and ticks, through the Runner.
    {
        auto writer = log::LogWriter::open(recorded, header);
        REQUIRE(writer);
        WatchStack<double> stack;
        harness::Runner runner(stack, std::span<const log::Channel>{});
        for (std::int64_t k = 0; k < 20; ++k) {
            std::vector<log::LogRecord> in = {
                {log::Channel::truth, Timestamp{k * 10}, watch::encode(scene(k * 10, 1.0 + static_cast<double>(k)))},
                {watch::kRange,
                 Timestamp{k * 10 + 1},
                 watch::encode(watch::Range<double>{0.5 * static_cast<double>(k)})},
                {log::Channel::tick, Timestamp{k * 10 + 2}, {}}};
            for (const auto& r : in) {
                writer->write(r.channel, r.t, r.payload);
                const auto out = runner.feed(r);
                REQUIRE(out);
                for (const auto& o : *out) {
                    writer->write(o.channel, o.t, o.payload);
                }
            }
        }
        REQUIRE(writer->close());
    }

    // Replay it into a fresh stack, and compare.
    auto reader = log::LogReader::open(recorded, kWatchChannels);
    REQUIRE(reader);
    auto writer = log::LogWriter::open(replayed, header);
    REQUIRE(writer);
    WatchStack<double> stack;
    const auto kept = harness::replay_log(*reader, stack, std::span<const log::Channel>{}, *writer, true);
    REQUIRE(kept);
    REQUIRE(writer->close());
    const auto matched = harness::verify_outputs(*kept);
    REQUIRE(matched);
    CHECK(*matched == 20u);

    const auto last = watch::decode_assessment(kept->produced.back().payload);
    REQUIRE(last);
    CHECK(*last == watch::Assessment<double>{20, 9.5, 20.0});
}
