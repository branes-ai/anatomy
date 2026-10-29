// Unit tests for the data contracts' invariants (branes/anatomy/msg/): each
// valid() accepts the default or a hand-built good value and rejects one
// violation per invariant.

#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/msg/geometry.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <vector>

using namespace branes::anatomy;

TEST_CASE("Quat: identity is unit, a scaled quaternion is not", "[msg]") {
    REQUIRE(is_unit(Quat<double>{}));
    REQUIRE(is_unit(Quat<double>{0.5, -0.5, 0.5, -0.5}));
    REQUIRE_FALSE(is_unit(Quat<double>{2.0, 0.0, 0.0, 0.0}));
    REQUIRE(is_unit(Quat<float>{}));
}

TEST_CASE("CameraFrame: I1 data present iff pixels, I2 stride covers a row", "[msg]") {
    std::vector<std::byte> pixels(4 * 3);
    CameraFrame f{FrameHeader{Timestamp{1}, 0, 0, 4, 3, 4, PixelFormat::mono8, false}, pixels.data()};
    REQUIRE(valid(f));

    CameraFrame empty{};
    REQUIRE(valid(empty));  // no pixels, no data

    CameraFrame no_data = f;
    no_data.data = nullptr;
    REQUIRE_FALSE(valid(no_data));  // I1

    CameraFrame narrow = f;
    narrow.header.format = PixelFormat::rgb8;  // needs stride 12
    REQUIRE_FALSE(valid(narrow));              // I2

    // I2 in 64-bit: 0x55555556 · 3 wraps to 2 in 32 bits, which would pass a 32-bit check.
    const FrameHeader wrap{Timestamp{0}, 0, 0, 0x55555556u, 1, 2, PixelFormat::rgb8, false};
    REQUIRE_FALSE(valid(wrap));
}

TEST_CASE("CameraFrameRecord: to_record and view_of round-trip", "[msg]") {
    std::vector<std::byte> pixels(6 * 2 * 3);
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        pixels[i] = static_cast<std::byte>(i);
    }
    const CameraFrame f{FrameHeader{Timestamp{42}, 1000, 20, 6, 2, 18, PixelFormat::rgb8, true}, pixels.data()};
    const CameraFrameRecord r = to_record(f);
    REQUIRE(valid(r));
    REQUIRE(r.header == f.header);
    const CameraFrame v = view_of(r);
    REQUIRE(valid(v));
    REQUIRE(v.header == f.header);
    REQUIRE(std::vector<std::byte>(v.bytes().begin(), v.bytes().end()) == pixels);
}
