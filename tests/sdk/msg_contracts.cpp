// Unit tests for the data contracts' invariants (branes/anatomy/msg/): each
// valid() accepts the default or a hand-built good value and rejects one
// violation per invariant.

#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/daa.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/intruders.hpp>
#include <branes/anatomy/msg/sensors.hpp>

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

TEST_CASE("ImuSample: I1 saturation bits stay inside the mask", "[msg]") {
    ImuSample<double> s{};
    REQUIRE(valid(s));
    s.saturated = accel_z_saturated | gyro_x_saturated;
    REQUIRE(valid(s));
    s.saturated = 0x40;
    REQUIRE_FALSE(valid(s));
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

TEST_CASE("StateEstimate: I1 unit attitude, I2 symmetric non-negative covariance", "[msg]") {
    StateEstimate<double> x{};
    REQUIRE(valid(x));
    x.covariance[0 * kErrorStateDim + 1] = 0.1;
    REQUIRE_FALSE(valid(x));  // I2: asymmetric
    x.covariance[1 * kErrorStateDim + 0] = 0.1;
    REQUIRE(valid(x));
    x.covariance[2 * kErrorStateDim + 2] = -1.0;
    REQUIRE_FALSE(valid(x));  // I2: negative variance
    x.covariance[2 * kErrorStateDim + 2] = 1.0;
    x.attitude = Quat<double>{1.0, 1.0, 0.0, 0.0};
    REQUIRE_FALSE(valid(x));  // I1
}

TEST_CASE("Reference: I2 strictly increasing time, I3 non-decreasing progress", "[msg]") {
    Reference<double> r{};
    r.count = 3;
    for (std::uint32_t i = 0; i < 3; ++i) {
        r.samples[i].t = Timestamp{static_cast<std::int64_t>(i) * 10};
        r.samples[i].progress = static_cast<double>(i);
    }
    REQUIRE(valid(r));
    r.samples[2].t = r.samples[1].t;
    REQUIRE_FALSE(valid(r));  // I2
    r.samples[2].t = Timestamp{20};
    r.samples[2].progress = 0.5;
    REQUIRE_FALSE(valid(r));  // I3
    r.count = kMaxReferenceSamples + 1;
    REQUIRE_FALSE(valid(r));  // I1
}

// Test names stay ASCII: CTest passes them back to Catch2 as filters, and
// Windows mangles non-ASCII command-line characters (≥ became =).
TEST_CASE("ThrustRates: I1 thrust >= 0, I2 motors in [0, 1]", "[msg]") {
    ThrustRates<double> u{};
    REQUIRE(valid(u));
    u.thrust = -1.0;
    REQUIRE_FALSE(valid(u));
    u.thrust = 5.0;
    u.motors[3] = 1.2;
    REQUIRE_FALSE(valid(u));
}

TEST_CASE("ConflictSet and IntruderDetections: count bounds", "[msg]") {
    ConflictSet<double> c{};
    REQUIRE(valid(c));
    c.count = kMaxConflicts + 1;
    REQUIRE_FALSE(valid(c));
    IntruderDetections<double> d{};
    REQUIRE(valid(d));
    d.count = kMaxDetections + 1;
    REQUIRE_FALSE(valid(d));
}
