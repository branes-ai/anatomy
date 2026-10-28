#pragma once
/// @file vehicle.hpp
/// @brief The vehicle file: types, loader, emitter.
///
/// Every loader validates the
/// whole file before it returns a value, so a config that loads satisfies all
/// the invariants listed on its type. Loading never throws; it returns
/// result<X, ConfigError>. Every emitter writes a file that loads back to an
/// equal value (round trip), with doubles written to 17 significant digits.

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/result.hpp>

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

namespace branes::anatomy::config {

// ── Vehicle ─────────────────────────────────────────────────────────

enum class CameraModel : std::uint8_t {
    pinhole_radtan = 0,
    equidistant = 1,  ///< Kannala-Brandt
};

enum class Shutter : std::uint8_t {
    global = 0,
    rolling = 1,
};

struct RotorParams {
    double thrust_coefficient{0};  ///< thrust = k_f·ω² [N / (rad/s)²]
    double torque_coefficient{0};  ///< torque = k_m·thrust [m]
    double max_thrust{0};          ///< per rotor [N]
    double time_constant{0};       ///< first-order motor time constant [s]

    friend bool operator==(const RotorParams&, const RotorParams&) = default;
};

struct DragParams {
    Vec3<double> linear{};     ///< body-frame linear drag [N / (m/s)]
    Vec3<double> quadratic{};  ///< body-frame quadratic drag, the v|v| term [N / (m/s)²]

    friend bool operator==(const DragParams&, const DragParams&) = default;
};

/// Largest IMU latency a vehicle file may give [s].
inline constexpr double kMaxImuLatencyS = 0.1;

struct ImuParams {
    double rate_hz{0};
    double gyro_range{0};           ///< per axis [rad/s]
    double accel_range{0};          ///< per axis [m/s²]
    double gyro_noise_density{0};   ///< [rad/s/√Hz]
    double accel_noise_density{0};  ///< [m/s²/√Hz]
    // Optional (default 0), used by the SITL IMU model (sim/imu.hpp).
    double gyro_bias_random_walk{0};     ///< [rad/s²/√Hz]
    double accel_bias_random_walk{0};    ///< [m/s³/√Hz]
    double initial_gyro_bias_sigma{0};   ///< turn-on bias, 1σ per axis [rad/s]
    double initial_accel_bias_sigma{0};  ///< turn-on bias, 1σ per axis [m/s²]
    double latency_s{0};                 ///< measurement → delivery [s], ≤ kMaxImuLatencyS

    friend bool operator==(const ImuParams&, const ImuParams&) = default;
};

/// Camera frame convention: z forward, x right, y down.
struct CameraParams {
    CameraModel model{CameraModel::equidistant};
    std::uint32_t width{0};
    std::uint32_t height{0};
    PixelFormat format{PixelFormat::mono8};
    double fx{0};
    double fy{0};
    double cx{0};
    double cy{0};
    std::array<double, 4> distortion{};  ///< k1..k4 (equidistant) or k1, k2, p1, p2 (radtan)
    double rate_hz{0};
    Shutter shutter{Shutter::rolling};
    std::int64_t row_readout_ns{0};
    Vec3<double> translation_body{};  ///< camera origin in the body frame [m]
    Quat<double> rotation_body{};     ///< camera → body

    friend bool operator==(const CameraParams&, const CameraParams&) = default;
};

/// vehicles/<name>.yaml.
///
/// Invariants: mass > 0; inertia diagonal > 0; arm_length > 0; rotor_count == 4;
/// rotor parameters > 0; drag ≥ 0; IMU rate, ranges > 0 and noise ≥ 0; camera
/// width, height, fx, fy, rate > 0; row_readout_ns ≥ 0; rotation_body is unit.
struct VehicleConfig {
    std::string name;
    double mass{0};          ///< [kg]
    Vec3<double> inertia{};  ///< diagonal [kg·m²]
    double arm_length{0};    ///< center to rotor axis [m]
    std::uint32_t rotor_count{0};
    RotorParams rotor;
    DragParams drag;
    ImuParams imu;
    CameraParams camera;

    friend bool operator==(const VehicleConfig&, const VehicleConfig&) = default;
};

[[nodiscard]] result<VehicleConfig, ConfigError> load_vehicle(const std::filesystem::path& path);
[[nodiscard]] result<VehicleConfig, ConfigError> parse_vehicle(std::string_view yaml, std::string_view origin);
[[nodiscard]] std::string emit_vehicle(const VehicleConfig& v);

}  // namespace branes::anatomy::config
