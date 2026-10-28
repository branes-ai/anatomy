#pragma once
/// @file codec.hpp
/// @brief Log payload encoding of the data contracts that cross the log.
///
/// Logs are always double precision, whatever scalar type the stack runs in.
/// Each encode() writes every field explicitly (util/bytes.hpp), so equal
/// values encode to equal bytes. Each decode_*() returns nullopt unless the
/// payload has exactly the expected size and the decoded value is valid().

#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/truth.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace branes::anatomy::codec {

namespace detail {

inline void put(ByteWriter& w, const Vec3<double>& v) {
    for (const double c : v) {
        w.f64(c);
    }
}
inline Vec3<double> get_vec3(ByteReader& r) {
    Vec3<double> v{};
    for (double& c : v) {
        c = r.f64();
    }
    return v;
}
inline bool finite(const Vec3<double>& v) noexcept {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
inline bool finite(const Quat<double>& q) noexcept {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}
template <std::size_t N>
inline bool finite(const std::array<double, N>& a) noexcept {
    for (const double v : a) {
        if (!std::isfinite(v)) {
            return false;
        }
    }
    return true;
}

inline void put(ByteWriter& w, const Quat<double>& q) {
    w.f64(q.w);
    w.f64(q.x);
    w.f64(q.y);
    w.f64(q.z);
}
inline Quat<double> get_quat(ByteReader& r) {
    Quat<double> q{};
    q.w = r.f64();
    q.x = r.f64();
    q.y = r.f64();
    q.z = r.f64();
    return q;
}
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

// ── ImuSample ───────────────────────────────────────────────────────

[[nodiscard]] inline std::vector<std::byte> encode(const ImuSample<double>& s) {
    ByteWriter w;
    w.i64(s.t.ns);
    detail::put(w, s.gyro);
    detail::put(w, s.accel);
    w.u8(s.saturated);
    w.f64(s.temperature);
    return w.take();
}

[[nodiscard]] inline std::optional<ImuSample<double>> decode_imu(std::span<const std::byte> b) {
    ByteReader r(b);
    ImuSample<double> s{};
    s.t.ns = r.i64();
    s.gyro = detail::get_vec3(r);
    s.accel = detail::get_vec3(r);
    s.saturated = r.u8();
    s.temperature = r.f64();
    // Every decoded double must be finite: a corrupt log must not feed NaN / Inf into the stack.
    if (!r.done() || !detail::finite(s.gyro) || !detail::finite(s.accel) || !std::isfinite(s.temperature) ||
        !valid(s)) {
        return std::nullopt;
    }
    return s;
}

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

// ── StateEstimate ───────────────────────────────────────────────────

[[nodiscard]] inline std::vector<std::byte> encode(const StateEstimate<double>& x) {
    ByteWriter w;
    w.i64(x.t_valid.ns);
    detail::put(w, x.position);
    detail::put(w, x.velocity);
    detail::put(w, x.attitude);
    detail::put(w, x.body_rates);
    detail::put(w, x.accel_bias);
    detail::put(w, x.gyro_bias);
    for (const double c : x.covariance) {
        w.f64(c);
    }
    return w.take();
}

[[nodiscard]] inline std::optional<StateEstimate<double>> decode_state(std::span<const std::byte> b) {
    ByteReader r(b);
    StateEstimate<double> x{};
    x.t_valid.ns = r.i64();
    x.position = detail::get_vec3(r);
    x.velocity = detail::get_vec3(r);
    x.attitude = detail::get_quat(r);
    x.body_rates = detail::get_vec3(r);
    x.accel_bias = detail::get_vec3(r);
    x.gyro_bias = detail::get_vec3(r);
    for (double& c : x.covariance) {
        c = r.f64();
    }
    if (!r.done() || !detail::finite(x.position) || !detail::finite(x.velocity) || !detail::finite(x.attitude) ||
        !detail::finite(x.body_rates) || !detail::finite(x.accel_bias) || !detail::finite(x.gyro_bias) ||
        !detail::finite(x.covariance) || !valid(x)) {
        return std::nullopt;
    }
    return x;
}

// ── ThrustRates ─────────────────────────────────────────────────────

[[nodiscard]] inline std::vector<std::byte> encode(const ThrustRates<double>& u) {
    ByteWriter w;
    w.i64(u.t.ns);
    w.u8(static_cast<std::uint8_t>(u.kind));
    w.f64(u.thrust);
    detail::put(w, u.body_rates);
    for (const double m : u.motors) {
        w.f64(m);
    }
    return w.take();
}

[[nodiscard]] inline std::optional<ThrustRates<double>> decode_command(std::span<const std::byte> b) {
    ByteReader r(b);
    ThrustRates<double> u{};
    u.t.ns = r.i64();
    const std::uint8_t kind = r.u8();
    if (kind > static_cast<std::uint8_t>(CommandKind::motors)) {
        return std::nullopt;
    }
    u.kind = static_cast<CommandKind>(kind);
    u.thrust = r.f64();
    u.body_rates = detail::get_vec3(r);
    for (double& m : u.motors) {
        m = r.f64();
    }
    if (!r.done() || !std::isfinite(u.thrust) || !detail::finite(u.body_rates) || !detail::finite(u.motors) ||
        !valid(u)) {
        return std::nullopt;
    }
    return u;
}

// ── Reference ───────────────────────────────────────────────────────

/// Encodes the `count` used samples only.
[[nodiscard]] inline std::vector<std::byte> encode(const Reference<double>& ref) {
    ByteWriter w;
    w.u32(ref.count);
    for (std::uint32_t i = 0; i < ref.count && i < kMaxReferenceSamples; ++i) {
        const ReferenceSample<double>& s = ref.samples[i];
        w.i64(s.t.ns);
        detail::put(w, s.position);
        detail::put(w, s.velocity);
        detail::put(w, s.acceleration);
        w.f64(s.yaw);
        w.f64(s.progress);
    }
    return w.take();
}

/// The unused samples of the result are value-initialized.
[[nodiscard]] inline std::optional<Reference<double>> decode_reference(std::span<const std::byte> b) {
    ByteReader r(b);
    Reference<double> ref{};
    ref.count = r.u32();
    if (ref.count > kMaxReferenceSamples) {
        return std::nullopt;
    }
    for (std::uint32_t i = 0; i < ref.count; ++i) {
        ReferenceSample<double>& s = ref.samples[i];
        s.t.ns = r.i64();
        s.position = detail::get_vec3(r);
        s.velocity = detail::get_vec3(r);
        s.acceleration = detail::get_vec3(r);
        s.yaw = r.f64();
        s.progress = r.f64();
        if (!detail::finite(s.position) || !detail::finite(s.velocity) || !detail::finite(s.acceleration) ||
            !std::isfinite(s.yaw) || !std::isfinite(s.progress)) {
            return std::nullopt;
        }
    }
    if (!r.done() || !valid(ref)) {
        return std::nullopt;
    }
    return ref;
}

// ── TruthState (sim channel) ────────────────────────────────────────

[[nodiscard]] inline std::vector<std::byte> encode(const TruthState<double>& x) {
    ByteWriter w;
    w.i64(x.t.ns);
    detail::put(w, x.position);
    detail::put(w, x.velocity);
    detail::put(w, x.attitude);
    detail::put(w, x.body_rates);
    for (const double v : x.rotor_speeds) {
        w.f64(v);
    }
    w.boolean(x.on_ground);
    return w.take();
}

[[nodiscard]] inline std::optional<TruthState<double>> decode_truth(std::span<const std::byte> b) {
    ByteReader r(b);
    TruthState<double> x{};
    x.t.ns = r.i64();
    x.position = detail::get_vec3(r);
    x.velocity = detail::get_vec3(r);
    x.attitude = detail::get_quat(r);
    x.body_rates = detail::get_vec3(r);
    for (double& v : x.rotor_speeds) {
        v = r.f64();
    }
    x.on_ground = r.boolean();
    if (!r.done() || !detail::finite(x.position) || !detail::finite(x.velocity) || !detail::finite(x.attitude) ||
        !detail::finite(x.body_rates) || !detail::finite(x.rotor_speeds) || !valid(x)) {
        return std::nullopt;
    }
    return x;
}

}  // namespace branes::anatomy::codec
