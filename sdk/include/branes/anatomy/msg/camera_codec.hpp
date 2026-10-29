#pragma once
/// @file camera_codec.hpp
/// @brief Log payload encoding of CameraFrameRecord.
///
/// As for every codec (msg/codec.hpp): logs are double precision, each
/// encode() writes every field explicitly (util/bytes.hpp), and decode_frame()
/// returns nullopt unless the payload has exactly the expected size and the
/// decoded record is valid().

#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace branes::anatomy::codec {

namespace detail {

inline void put(ByteWriter& w, const FrameHeader& h) {
    w.i64(h.t_mid.ns);
    w.i64(h.exposure_ns);
    w.i64(h.row_readout_ns);
    w.u32(h.width);
    w.u32(h.height);
    w.u32(h.stride);
    w.u8(static_cast<std::uint8_t>(h.format));
    w.boolean(h.corrupted);
}
inline std::optional<FrameHeader> get_frame_header(ByteReader& r) {
    FrameHeader h{};
    h.t_mid.ns = r.i64();
    h.exposure_ns = r.i64();
    h.row_readout_ns = r.i64();
    h.width = r.u32();
    h.height = r.u32();
    h.stride = r.u32();
    const std::uint8_t f = r.u8();
    if (f > static_cast<std::uint8_t>(PixelFormat::rgb8)) {
        return std::nullopt;
    }
    h.format = static_cast<PixelFormat>(f);
    h.corrupted = r.boolean();
    return h;
}

}  // namespace detail

// ── CameraFrameRecord ───────────────────────────────────────────────

[[nodiscard]] inline std::vector<std::byte> encode(const CameraFrameRecord& f) {
    ByteWriter w;
    detail::put(w, f.header);
    w.u32(static_cast<std::uint32_t>(f.bytes.size()));
    w.bytes(f.bytes);
    return w.take();
}

[[nodiscard]] inline std::optional<CameraFrameRecord> decode_frame(std::span<const std::byte> b) {
    ByteReader r(b);
    auto h = detail::get_frame_header(r);
    if (!h) {
        return std::nullopt;
    }
    const std::uint32_t n = r.u32();
    const auto pixels = r.bytes(n);
    CameraFrameRecord f{*h, std::vector<std::byte>(pixels.begin(), pixels.end())};
    if (!r.done() || !valid(f)) {
        return std::nullopt;
    }
    return f;
}

}  // namespace branes::anatomy::codec
