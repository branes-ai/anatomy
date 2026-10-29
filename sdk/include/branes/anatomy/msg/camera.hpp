#pragma once
/// @file camera.hpp
/// @brief CameraFrame (in-process view), and CameraFrameRecord (owning wire form).
///
/// racing-drone docs/arch/framework-architecture.md §4: CameraFrame never crosses a process
/// boundary, Zenoh, or a log. Its owning wire and log form is
/// CameraFrameRecord; to_record() and view_of() convert explicitly. Nothing in
/// a frame names the body that carries the camera, so the frame contracts are
/// the same for a vehicle's camera and a fixed one.

#include <branes/anatomy/msg/time.hpp>

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace branes::anatomy {

enum class PixelFormat : std::uint8_t {
    mono8 = 0,
    rgb8 = 1,
};

[[nodiscard]] constexpr std::uint32_t bytes_per_pixel(PixelFormat f) noexcept {
    return f == PixelFormat::rgb8 ? 3u : 1u;
}

/// Timing and geometry of one camera frame, shared by the view and the record.
struct FrameHeader {
    Timestamp t_mid;                 ///< mid-exposure time
    std::int64_t exposure_ns{0};     ///< exposure duration
    std::int64_t row_readout_ns{0};  ///< rolling-shutter time per row (0 = global shutter)
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t stride{0};  ///< bytes per row
    PixelFormat format{PixelFormat::mono8};
    bool corrupted{false};  ///< set by the driver or a corruption detector (EST-08)

    friend constexpr bool operator==(const FrameHeader&, const FrameHeader&) = default;
};

/// Size in bytes of the image a header describes.
[[nodiscard]] constexpr std::size_t image_bytes(const FrameHeader& h) noexcept {
    return static_cast<std::size_t>(h.stride) * h.height;
}

/// A camera frame as seen inside the flight-stack process. Non-owning: `data`
/// points into a driver or Resource Manager buffer that outlives the frame.
///
/// Invariants:
///   I1: data != nullptr  ⇔  width·height > 0
///   I2: stride ≥ width·bytes_per_pixel(format)
struct CameraFrame {
    FrameHeader header;
    const std::byte* data{nullptr};  ///< [non-owning] image_bytes(header) bytes

    [[nodiscard]] std::span<const std::byte> bytes() const noexcept {
        return {data, data ? image_bytes(header) : 0};
    }
};

[[nodiscard]] constexpr bool valid(const FrameHeader& h) noexcept {
    // 64-bit product: width · bpp can wrap in 32 bits for a corrupt header.
    return std::uint64_t{h.stride} >= std::uint64_t{h.width} * bytes_per_pixel(h.format) && h.exposure_ns >= 0 &&
           h.row_readout_ns >= 0;
}

[[nodiscard]] constexpr bool valid(const CameraFrame& f) noexcept {
    const bool has_pixels = f.header.width > 0 && f.header.height > 0;
    return (f.data != nullptr) == has_pixels  // I1
           && valid(f.header);                // I2
}

/// The owning wire and log form of a CameraFrame. Phase 0 stores raw pixels;
/// compressed encodings are declared in the log header when they are added.
///
/// Invariants: I1 bytes.size() == image_bytes(header); I2 valid(header).
struct CameraFrameRecord {
    FrameHeader header;
    std::vector<std::byte> bytes;  ///< [owning]

    friend bool operator==(const CameraFrameRecord&, const CameraFrameRecord&) = default;
};

[[nodiscard]] inline bool valid(const CameraFrameRecord& r) noexcept {
    return r.bytes.size() == image_bytes(r.header) && valid(r.header);
}

/// Copy a frame into its owning record. Pre: valid(f). Post: valid(result).
[[nodiscard]] inline CameraFrameRecord to_record(const CameraFrame& f) {
    const auto src = f.bytes();
    return CameraFrameRecord{f.header, std::vector<std::byte>(src.begin(), src.end())};
}

/// A non-owning view of a record, valid while `r` is alive and unchanged.
/// Pre: valid(r). Post: valid(result).
[[nodiscard]] inline CameraFrame view_of(const CameraFrameRecord& r) noexcept {
    return CameraFrame{r.header, r.bytes.empty() ? nullptr : r.bytes.data()};
}

}  // namespace branes::anatomy
