// Unit tests for the byte encoding (util/bytes.hpp) and the camera frame codec
// (msg/camera_codec.hpp): round trips are exact, and malformed payloads are
// rejected rather than half-decoded. The mobility codecs are tested in
// tests/mobility/mobility_codec.cpp.

#include <branes/anatomy/msg/camera_codec.hpp>
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

TEST_CASE("codec: CameraFrameRecord round trip", "[codec]") {
    CameraFrameRecord f{};
    f.header = FrameHeader{Timestamp{7}, 2000, 18, 3, 2, 3, PixelFormat::mono8, false};
    f.bytes = {std::byte{1}, std::byte{2}, std::byte{3}, std::byte{4}, std::byte{5}, std::byte{6}};
    const auto back = codec::decode_frame(codec::encode(f));
    REQUIRE(back);
    REQUIRE(*back == f);
}
