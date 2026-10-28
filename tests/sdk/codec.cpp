// Unit tests for the log payload codec (msg/codec.hpp) and the byte encoding
// beneath it (util/bytes.hpp): round trips are exact, and malformed payloads
// are rejected rather than half-decoded.

#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

using namespace branes::anatomy;

TEST_CASE("ByteWriter writes little-endian; ByteReader reads it back", "[codec]") {
    ByteWriter w;
    w.u16(0x0102);
    w.i64(-2);
    w.f64(-0.0);
    w.str("gate");
    const auto& b = w.data();
    REQUIRE(b.size() == 2 + 8 + 8 + 4 + 4);
    REQUIRE(b[0] == std::byte{0x02});  // little-endian
    REQUIRE(b[1] == std::byte{0x01});

    ByteReader r(b);
    REQUIRE(r.u16() == 0x0102);
    REQUIRE(r.i64() == -2);
    const double z = r.f64();
    REQUIRE(z == 0.0);
    REQUIRE(std::signbit(z));  // the bit pattern survives, sign included
    REQUIRE(r.str() == "gate");
    REQUIRE(r.done());
}

TEST_CASE("ByteReader: reading past the end fails instead of overrunning", "[codec]") {
    const std::vector<std::byte> b(3);
    ByteReader r(b);
    REQUIRE(r.u32() == 0u);
    REQUIRE(r.failed());
    REQUIRE_FALSE(r.done());
}

TEST_CASE("codec: ImuSample round trip", "[codec]") {
    ImuSample<double> s{};
    s.t = Timestamp{123456789};
    s.gyro = {0.1, -0.2, 0.3};
    s.accel = {1.0, 2.0, 9.80665};
    s.saturated = accel_z_saturated;
    s.temperature = 31.5;
    const auto bytes = codec::encode(s);
    const auto back = codec::decode_imu(bytes);
    REQUIRE(back);
    REQUIRE(*back == s);
    REQUIRE(codec::encode(*back) == bytes);  // equal values, equal bytes
}

TEST_CASE("codec: StateEstimate and ThrustRates round trip", "[codec]") {
    StateEstimate<double> x{};
    x.t_valid = Timestamp{-5};
    x.position = {1.0, 2.0, 3.0};
    x.attitude = Quat<double>{0.5, -0.5, 0.5, -0.5};
    x.covariance[0] = 0.25;
    const auto xb = codec::decode_state(codec::encode(x));
    REQUIRE(xb);
    REQUIRE(*xb == x);

    ThrustRates<double> u{};
    u.t = Timestamp{99};
    u.kind = CommandKind::motors;
    u.thrust = 4.5;
    u.motors = {0.1, 0.2, 0.3, 0.4};
    const auto ub = codec::decode_command(codec::encode(u));
    REQUIRE(ub);
    REQUIRE(*ub == u);
}

TEST_CASE("codec: CameraFrameRecord round trip", "[codec]") {
    CameraFrameRecord f{};
    f.header = FrameHeader{Timestamp{7}, 2000, 18, 3, 2, 3, PixelFormat::mono8, false};
    f.bytes = {std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}, std::byte{5}, std::byte{6}};
    const auto back = codec::decode_frame(codec::encode(f));
    REQUIRE(back);
    REQUIRE(*back == f);
}

TEST_CASE("codec: wrong size and invalid values are rejected", "[codec]") {
    ImuSample<double> s{};
    auto bytes = codec::encode(s);
    bytes.push_back(std::byte{0});
    REQUIRE_FALSE(codec::decode_imu(bytes));  // trailing byte
    bytes.resize(bytes.size() - 2);
    REQUIRE_FALSE(codec::decode_imu(bytes));  // truncated

    StateEstimate<double> x{};
    x.attitude = Quat<double>{2.0, 0.0, 0.0, 0.0};  // not unit: invalid
    REQUIRE_FALSE(codec::decode_state(codec::encode(x)));

    CameraFrameRecord f{};
    f.header = FrameHeader{Timestamp{0}, 0, 0, 2, 2, 2, PixelFormat::mono8, false};
    f.bytes.resize(3);  // needs 4
    REQUIRE_FALSE(codec::decode_frame(codec::encode(f)));
}

TEST_CASE("codec: NaN and Inf are rejected in every decoder", "[codec]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    ImuSample<double> s{};
    s.accel[1] = nan;
    REQUIRE_FALSE(codec::decode_imu(codec::encode(s)));
    s = ImuSample<double>{};
    s.temperature = inf;
    REQUIRE_FALSE(codec::decode_imu(codec::encode(s)));

    StateEstimate<double> x{};
    x.velocity[0] = inf;
    REQUIRE_FALSE(codec::decode_state(codec::encode(x)));
    x = StateEstimate<double>{};
    x.gyro_bias[2] = nan;
    REQUIRE_FALSE(codec::decode_state(codec::encode(x)));

    ThrustRates<double> u{};
    u.body_rates[0] = -inf;
    REQUIRE_FALSE(codec::decode_command(codec::encode(u)));
}

TEST_CASE("codec: TruthState round trip", "[codec]") {
    TruthState<double> x{};
    x.t = Timestamp{123};
    x.position = {1.5, -2.25, 3.0};
    x.velocity = {0.1, 0.2, -0.3};
    x.attitude = Quat<double>{0.6, 0.8, 0.0, 0.0};
    x.body_rates = {0.01, -0.02, 0.03};
    x.rotor_speeds = {1600.0, 1601.5, 1599.25, 1600.125};
    x.on_ground = true;
    const auto xb = codec::decode_truth(codec::encode(x));
    REQUIRE(xb);
    REQUIRE(*xb == x);
}

TEST_CASE("codec: invalid truth records are rejected", "[codec]") {
    TruthState<double> x{};
    x.attitude = Quat<double>{2.0, 0.0, 0.0, 0.0};  // not unit
    REQUIRE_FALSE(codec::decode_truth(codec::encode(x)));
    x = TruthState<double>{};
    x.rotor_speeds[3] = -1.0;  // I2
    REQUIRE_FALSE(codec::decode_truth(codec::encode(x)));
    x = TruthState<double>{};
    x.position[1] = std::numeric_limits<double>::quiet_NaN();
    REQUIRE_FALSE(codec::decode_truth(codec::encode(x)));
}

namespace {

/// A valid reference with `n` samples: times strictly increase, progress never decreases.
Reference<double> reference_of(std::uint32_t n) {
    Reference<double> ref{};
    ref.count = n;
    for (std::uint32_t i = 0; i < n; ++i) {
        ReferenceSample<double>& s = ref.samples[i];
        const double k = static_cast<double>(i);
        s.t = Timestamp{1'000'000 + 4'000'000 * static_cast<std::int64_t>(i)};
        s.position = {1.0 + k, -2.0 + 0.5 * k, 3.0};
        s.velocity = {4.0, 0.25 * k, -0.125};
        s.acceleration = {0.0, -1.5, 0.5 * k};
        s.yaw = 0.01 * k - 0.3;
        s.progress = 0.5 * k;
    }
    return ref;
}

}  // namespace

TEST_CASE("codec: Reference round trip encodes only the used samples", "[codec]") {
    for (const std::uint32_t n : {0u, 1u, kMaxReferenceSamples}) {
        const Reference<double> ref = reference_of(n);
        const auto bytes = codec::encode(ref);
        CHECK(bytes.size() == 4 + n * (8 + 11 * 8));  // count, then t + 11 doubles per sample
        const auto back = codec::decode_reference(bytes);
        REQUIRE(back);
        CHECK(*back == ref);
        CHECK(codec::encode(*back) == bytes);
    }
}

TEST_CASE("codec: invalid reference records are rejected", "[codec]") {
    ByteWriter w;
    w.u32(kMaxReferenceSamples + 1);  // I1: too many samples
    CHECK_FALSE(codec::decode_reference(w.take()));

    // A NaN in each of the 11 doubles of a sample.
    for (int field = 0; field < 11; ++field) {
        Reference<double> ref = reference_of(2);
        ReferenceSample<double>& s = ref.samples[1];
        const double nan = std::numeric_limits<double>::quiet_NaN();
        if (field < 3) {
            s.position[static_cast<std::size_t>(field)] = nan;
        } else if (field < 6) {
            s.velocity[static_cast<std::size_t>(field - 3)] = nan;
        } else if (field < 9) {
            s.acceleration[static_cast<std::size_t>(field - 6)] = nan;
        } else if (field == 9) {
            s.yaw = nan;
        } else {
            s.progress = nan;
        }
        CAPTURE(field);
        CHECK_FALSE(codec::decode_reference(codec::encode(ref)));
    }

    Reference<double> ref = reference_of(3);
    ref.samples[2].t = ref.samples[1].t;  // I2: times must strictly increase
    CHECK_FALSE(codec::decode_reference(codec::encode(ref)));
    ref = reference_of(3);
    ref.samples[2].progress = ref.samples[1].progress - 0.1;  // I3: progress never decreases
    CHECK_FALSE(codec::decode_reference(codec::encode(ref)));

    auto bytes = codec::encode(reference_of(2));
    bytes.push_back(std::byte{0});  // one trailing byte
    CHECK_FALSE(codec::decode_reference(bytes));
    bytes.resize(bytes.size() - 2);  // truncated by one byte
    CHECK_FALSE(codec::decode_reference(bytes));
}
