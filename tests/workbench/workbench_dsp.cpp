// The DSP spectral pipeline pattern (workbench/patterns/dsp.hpp;
// docs/plan/workbench, B5): its stack writes, bit for bit, what its operators
// write by hand (W1); its outputs meet its oracle (W2); and a seed's traces
// are byte-identical run to run (W3). It is the first pattern with an
// `accumulate` edge, and the first with complex elements.

#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/dsp.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
namespace dsp = branes::anatomy::workbench::dsp;

const wb::check* find(const wb::report& r, const std::string& name) {
    for (const wb::check& c : r.checks) {
        if (c.name.find(name) != std::string::npos) {
            return &c;
        }
    }
    return nullptr;
}

/// The i-th frame tick's outputs.
dsp::frame_outputs& frame(std::vector<dsp::sample>& samples, std::size_t i) {
    dsp::frame_outputs* found = nullptr;
    std::size_t n = 0;
    for (dsp::sample& s : samples) {
        if (s.source == 1 && !s.frame.empty() && n++ == i) {
            found = &s.frame.front();
            break;
        }
    }
    REQUIRE(found != nullptr);
    return *found;
}

}  // namespace

TEST_CASE("dsp: the frame region reads accumulate<256> of the filtered samples", "[workbench][dsp]") {
    const auto g = wb::describe<dsp::pattern>(dsp::config{});
    REQUIRE(g);
    REQUIRE(g->regions.size() == 2);
    const fl::Region& samples = g->regions[0];
    const fl::Region& frames = g->regions[1];
    CHECK(samples.trigger == "x");
    CHECK(frames.trigger == "frame");
    CHECK(samples.firings[0].handler == "filter");
    CHECK(samples.firings[0].state == fl::state_role::update);
    REQUIRE(frames.firings.size() == 5);
    CHECK(frames.firings[0].op == "framer");
    CHECK(frames.firings[0].in[0] ==
          fl::Ref{fl::Ref::kind::firing, "x", 0, "y", {}, fl::rate_rule::accumulate, dsp::kFrame});
    CHECK(frames.firings[3].op == "fft");
    const std::string text = fl::canonical(*g);
    CHECK(text.find(" accumulate 256\n") != std::string::npos);
    CHECK(text.find("instance fft Fft_c128_256 -") != std::string::npos);
}

TEST_CASE("dsp: W1 the stack writes what the operators write by hand", "[workbench][dsp]") {
    const dsp::config c;
    constexpr std::size_t inputs = 4 * (dsp::kFrame + 1) + 10;
    const auto samples = wb::run<dsp::pattern>(c, wb::run_options{inputs, {}});
    REQUIRE(samples);
    const auto want = dsp::hand_run(c, inputs);
    REQUIRE(samples->size() == want.size());
    std::size_t frames = 0;
    for (std::size_t i = 0; i < want.size(); ++i) {
        INFO("input " << i);
        REQUIRE((*samples)[i] == want[i]);
        frames += want[i].source;
    }
    // A frame tick after every 256th sample, at its time.
    CHECK(frames == 4);
    CHECK(want[dsp::kFrame].source == 1);
    CHECK(want[dsp::kFrame].t == want[dsp::kFrame - 1].t);
    CHECK(want[dsp::kFrame].frame.front().frame.count() == dsp::kFrame);
}

TEST_CASE("dsp: W2 the outputs meet the oracle", "[workbench][dsp]") {
    for (const std::uint64_t seed : {1ULL, 2ULL, 3ULL, 4ULL, 5ULL}) {
        dsp::config c;
        c.seed = seed;
        const auto samples = wb::run<dsp::pattern>(c, wb::run_options{dsp::pattern::default_inputs, {}});
        REQUIRE(samples);
        const wb::report r = dsp::pattern::oracle(c, *samples);
        INFO(r.to_string());
        CHECK(r.passed());
        CHECK(find(r, "frames")->value == 32);
    }
}

TEST_CASE("dsp: the oracle fails a pipeline that's wrong", "[workbench][dsp]") {
    const dsp::config c;
    constexpr std::size_t inputs = 4 * (dsp::kFrame + 1);
    auto samples = wb::run<dsp::pattern>(c, wb::run_options{inputs, {}});
    REQUIRE(samples);
    REQUIRE(dsp::pattern::oracle(c, *samples).passed());

    SECTION("a filtered sample, off by a part in 10⁹") {
        frame(*samples, 1).frame(100) *= 1.000000001;
        const wb::report r = dsp::pattern::oracle(c, *samples);
        CHECK_FALSE(find(r, "filtered frame")->passed);
    }
    SECTION("a tone's bin of the spectrum, off by a part in 10⁹") {
        frame(*samples, 2).spectrum(c.pass[0].bin) *= 1.000000001;
        const wb::report r = dsp::pattern::oracle(c, *samples);
        CHECK_FALSE(find(r, "spectrum against")->passed);
        CHECK(find(r, "magnitude against")->passed);
    }
    SECTION("a pass-band tone 2 % low") {
        frame(*samples, 3).magnitude(c.pass[0].bin) *= 0.98;
        const wb::report r = dsp::pattern::oracle(c, *samples);
        CHECK_FALSE(find(r, "pass-band tone 1")->passed);
        CHECK_FALSE(find(r, "magnitude against")->passed);
    }
    SECTION("a short window, and one that dropped") {
        (void)frame(*samples, 1).frame.resize(200);
        frame(*samples, 2).dropped() = 3;
        const wb::report r = dsp::pattern::oracle(c, *samples);
        CHECK(find(r, "isn't 256")->value == 1);
        CHECK(find(r, "dropped")->value == 3);
        CHECK_FALSE(r.passed());
    }
}

TEST_CASE("dsp: a narrower pass band leaves a pass-band tone outside it", "[workbench][dsp]") {
    dsp::config c;
    c.high_hz = 1500;  // tone 2, at 1664 Hz, is now in the transition band
    const auto samples = wb::run<dsp::pattern>(c, wb::run_options{4 * (dsp::kFrame + 1), {}});
    REQUIRE(samples);
    const wb::report r = dsp::pattern::oracle(c, *samples);
    // The prediction follows H, so the tones still agree with it; the largest bins don't.
    CHECK(find(r, "pass-band tone 2")->passed);
    CHECK_FALSE(find(r, "largest bins")->passed);
}

TEST_CASE("dsp: W3 a seed's traces are byte-identical run to run", "[workbench][dsp]") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto a = dir / "anatomy_workbench_dsp_a.brtrace";
    const auto b = dir / "anatomy_workbench_dsp_b.brtrace";
    dsp::config c;
    c.seed = 7;
    constexpr std::size_t inputs = 3 * (dsp::kFrame + 1);
    REQUIRE(wb::run<dsp::pattern>(c, wb::run_options{inputs, a}));
    REQUIRE(wb::run<dsp::pattern>(c, wb::run_options{inputs, b}));
    const auto cmp = fl::compare_traces(a, b);
    REQUIRE(cmp);
    CHECK_FALSE(cmp->difference);
    CHECK(cmp->records > inputs);
    std::error_code ec;
    std::filesystem::remove(a, ec);
    std::filesystem::remove(b, ec);
}
