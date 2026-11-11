// The convolution pattern (workbench/patterns/conv.hpp;
// docs/plan/computational-domain, C6): its messages are rank-3 data<D>, and
// its kernels rank-4, stored row-major (W17); its stack writes, bit for bit,
// what its operators write by hand (W1); its outputs meet its oracle (W2);
// and a seed's traces are byte-identical run to run (W3).

#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/workbench/pattern.hpp>
#include <branes/anatomy/workbench/patterns/conv.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <filesystem>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fl = branes::anatomy::flow;
namespace wb = branes::anatomy::workbench;
namespace cv = branes::anatomy::workbench::conv;

const wb::check* find(const wb::report& r, const std::string& name) {
    for (const wb::check& c : r.checks) {
        if (c.name.find(name) != std::string::npos) {
            return &c;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("conv: the image, the feature maps and the kernels are rank 3 and 4, row-major (W17)", "[workbench][conv]") {
    STATIC_REQUIRE(cv::Image::rank == 3);
    STATIC_REQUIRE(cv::Weights::rank == 4);
    STATIC_REQUIRE(cv::Image::stored == 3 * 32 * 32);
    STATIC_REQUIRE(cv::Weights::stored == 8 * 3 * 3 * 3);
    STATIC_REQUIRE(cv::Features::index(1, 2, 3) == 1 * 32 * 32 + 2 * 32 + 3);
    STATIC_REQUIRE(cv::Weights::index(7, 2, 1, 0) == ((7 * 3 + 2) * 3 + 1) * 3);
    // The first three inputs are impulses, one per channel.
    const cv::config c;
    for (std::size_t k = 0; k < cv::kChannels; ++k) {
        const cv::Image x = cv::input_of(c, k);
        double sum = 0;
        for (const double e : x.elements) {
            sum += e;
        }
        CHECK(sum == 1.0);
        CHECK(x(k, cv::kImpulseRow, cv::kImpulseCol) == 1.0);
    }
}

TEST_CASE("conv: the four operators run in one region on each image", "[workbench][conv]") {
    const auto g = wb::describe<cv::pattern>(cv::config{});
    REQUIRE(g);
    REQUIRE(g->regions.size() == 1);
    const fl::Region& r = g->regions[0];
    CHECK(r.trigger == "x");
    REQUIRE(r.firings.size() == 4);
    CHECK(r.firings[0].op == "pad");
    CHECK(r.firings[1].op == "conv2d");
    CHECK(r.firings[2].op == "bias");
    CHECK(r.firings[3].op == "relu");
    CHECK(fl::canonical(*g).find("instance conv2d Conv2d_f64_8x3x3x3_32x32") != std::string::npos);
}

TEST_CASE("conv: W1 the stack writes what the operators write by hand", "[workbench][conv]") {
    const cv::config c;
    const auto samples = wb::run<cv::pattern>(c, wb::run_options{6, {}});
    REQUIRE(samples);
    REQUIRE(samples->size() == 6);
    const cv::weights w = cv::weights_of(c);
    for (std::size_t i = 0; i < samples->size(); ++i) {
        INFO("input " << i);
        REQUIRE((*samples)[i] == cv::hand_run(w, cv::input_of(c, i)));
    }
}

TEST_CASE("conv: W2 the outputs meet the oracle", "[workbench][conv]") {
    const cv::config c;
    const auto samples = wb::run<cv::pattern>(c, wb::run_options{cv::pattern::default_inputs, {}});
    REQUIRE(samples);
    const wb::report r = cv::pattern::oracle(c, *samples);
    INFO(r.to_string());
    CHECK(r.passed());
    CHECK(r.checks.size() == 5);
    CHECK(find(r, "convolution against")->value > 0);  // rounding, so the check measures something
}

TEST_CASE("conv: the oracle fails a convolution that's wrong", "[workbench][conv]") {
    const cv::config c;
    const auto samples = wb::run<cv::pattern>(c, wb::run_options{cv::pattern::default_inputs, {}});
    REQUIRE(samples);
    {
        // One element 1e-11 off.
        auto wrong = *samples;
        wrong[5].conv(3, 31, 0) += 1e-11;
        const wb::report r = cv::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "convolution against")->passed);
        CHECK(find(r, "y against")->passed);
    }
    {
        // A convolution, not a cross-correlation: the kernels flipped.
        cv::weights flipped = cv::weights_of(c);
        const cv::weights w = flipped;
        for (std::size_t o = 0; o < cv::kKernels; ++o) {
            for (std::size_t ch = 0; ch < cv::kChannels; ++ch) {
                for (std::size_t ky = 0; ky < cv::kTaps; ++ky) {
                    for (std::size_t kx = 0; kx < cv::kTaps; ++kx) {
                        flipped.w(o, ch, ky, kx) = w.w(o, ch, cv::kTaps - 1 - ky, cv::kTaps - 1 - kx);
                    }
                }
            }
        }
        std::vector<cv::sample> wrong;
        for (std::size_t k = 0; k < cv::pattern::default_inputs; ++k) {
            wrong.push_back(cv::hand_run(flipped, cv::input_of(c, k)));
        }
        const wb::report r = cv::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "impulses: elements")->passed);
        CHECK_FALSE(find(r, "convolution against")->passed);
    }
    {
        // A ReLU that passes negatives.
        auto wrong = *samples;
        wrong[4].y = wrong[4].conv;
        const wb::report r = cv::pattern::oracle(c, wrong);
        CHECK_FALSE(find(r, "y against")->passed);
        CHECK(find(r, "convolution against")->passed);
    }
    // Only the impulses: no image.
    const wb::report r = cv::pattern::oracle(c, std::span(*samples).first(cv::kChannels));
    CHECK_FALSE(r.passed());
}

TEST_CASE("conv: W3 a seed's traces are byte-identical run to run", "[workbench][conv]") {
    const auto dir = std::filesystem::temp_directory_path();
    const auto a = dir / "anatomy_workbench_conv_a.brtrace";
    const auto b = dir / "anatomy_workbench_conv_b.brtrace";
    const cv::config c;
    REQUIRE(wb::run<cv::pattern>(c, wb::run_options{5, a}));
    REQUIRE(wb::run<cv::pattern>(c, wb::run_options{5, b}));
    const auto cmp = fl::compare_traces(a, b);
    REQUIRE(cmp);
    CHECK_FALSE(cmp->difference);
    CHECK(cmp->records == 5 * 4);
    std::error_code ec;
    std::filesystem::remove(a, ec);
    std::filesystem::remove(b, ec);
}
