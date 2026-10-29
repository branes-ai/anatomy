#pragma once
/// @file synthetic.hpp
/// @brief SyntheticSensors: the null SITL's deterministic IMU / camera / tick event source.
///
///

#include <branes/anatomy/config/vehicle.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/msg/imu.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/util/splitmix64.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace branes::anatomy::harness {

// ── synthetic sensors (null SITL) ───────────────────────────────────

/// One input event of the synthetic sensor stream.
struct SensorEvent {
    log::Channel channel;
    Timestamp t;
    std::vector<std::byte> payload;
};

/// Deterministic IMU / camera / tick stream for a vehicle at rest: gravity on
/// the accelerometer, white noise from the vehicle's noise densities, and a
/// test-pattern image. The same seed gives the same bytes on every platform
/// that rounds IEEE-754 the same way.
///
/// Model
/// -----
///   rng_          : SplitMix64 — the only random source
///   next_imu_, next_frame_, next_tick_ : int64 ns — next event time per stream
///   end_          : int64 ns — events at t ≥ end_ are not produced
///   frame_index_  : uint32 — frames produced so far
///
/// Invariants
/// ----------
///   I1: events come out in non-decreasing time; at equal times the order is
///       IMU, then camera, then tick                  — constrains next_*
///   I2: every period is > 0 (create() validates; next() also checks)
///                                                    — constrains *_period_
class SyntheticSensors {
public:
    /// Validate the rates and duration, then build the stream. Rejects any rate
    /// outside [kMinRateHz, kMaxRateHz] and any duration outside
    /// (0, kMaxDurationS], so every period is a positive, representable number
    /// of nanoseconds (I2) and the end time fits int64.
    [[nodiscard]] static result<SyntheticSensors, std::string>
    create(const config::VehicleConfig& vehicle, double duration_s, std::uint64_t seed, double control_hz) {
        const auto imu = checked_period_ns(vehicle.imu.rate_hz);
        const auto frame = checked_period_ns(vehicle.camera.rate_hz);
        const auto tick = checked_period_ns(control_hz);
        if (!imu || !frame || !tick) {
            return std::string("IMU, camera, and control rates must be in [" + std::to_string(kMinRateHz) + ", " +
                               std::to_string(kMaxRateHz) + "] Hz");
        }
        if (!(duration_s > 0 && duration_s <= kMaxDurationS)) {
            return std::string("duration must be in (0, " + std::to_string(kMaxDurationS) + "] s");
        }
        return SyntheticSensors(vehicle, static_cast<std::int64_t>(duration_s * 1e9), seed, *imu, *frame, *tick);
    }

    /// The next event, or nullopt once every stream has passed the end.
    [[nodiscard]] std::optional<SensorEvent> next() {
        if (imu_period_ <= 0 || frame_period_ <= 0 || tick_period_ <= 0) {
            return std::nullopt;  // I2 runtime defense: never loop on a zero period
        }
        const std::int64_t t = std::min({next_imu_, next_frame_, next_tick_});
        if (t >= end_) {
            return std::nullopt;
        }
        if (next_imu_ == t) {  // I1: IMU first at equal times
            next_imu_ += imu_period_;
            return SensorEvent{log::Channel::imu, Timestamp{t}, codec::encode(imu_at(t))};
        }
        if (next_frame_ == t) {
            next_frame_ += frame_period_;
            return SensorEvent{log::Channel::camera, Timestamp{t}, codec::encode(frame_at(t))};
        }
        next_tick_ += tick_period_;
        return SensorEvent{log::Channel::tick, Timestamp{t}, {}};
    }

private:
    SyntheticSensors(const config::VehicleConfig& vehicle,
                     std::int64_t end_ns,
                     std::uint64_t seed,
                     std::int64_t imu_period,
                     std::int64_t frame_period,
                     std::int64_t tick_period)
        : vehicle_(vehicle), rng_(seed), imu_period_(imu_period), frame_period_(frame_period),
          tick_period_(tick_period), end_(end_ns) {
        assert(imu_period_ > 0 && frame_period_ > 0 && tick_period_ > 0);  // I2
        // White noise of density d sampled at rate f has σ = d·√f; a uniform
        // variable on [−a, a) has σ = a/√3.
        gyro_amp_ = vehicle.imu.gyro_noise_density * std::sqrt(vehicle.imu.rate_hz) * std::sqrt(3.0);
        accel_amp_ = vehicle.imu.accel_noise_density * std::sqrt(vehicle.imu.rate_hz) * std::sqrt(3.0);
    }

    ImuSample<double> imu_at(std::int64_t t) {
        constexpr double g = 9.80665;
        ImuSample<double> s{};
        s.t = Timestamp{t};
        for (double& w : s.gyro) {
            w = rng_.symmetric(gyro_amp_);
        }
        s.accel = {rng_.symmetric(accel_amp_), rng_.symmetric(accel_amp_), g + rng_.symmetric(accel_amp_)};
        s.temperature = 25.0;
        return s;
    }

    CameraFrameRecord frame_at(std::int64_t t) {
        const auto& cam = vehicle_.camera;
        CameraFrameRecord f{};
        f.header.t_mid = Timestamp{t};
        f.header.exposure_ns = 2'000'000;
        f.header.row_readout_ns = cam.row_readout_ns;
        f.header.width = cam.width;
        f.header.height = cam.height;
        f.header.format = cam.format;
        f.header.stride = cam.width * bytes_per_pixel(cam.format);
        f.bytes.resize(image_bytes(f.header));
        // A moving diagonal gradient: cheap, deterministic, and different per frame.
        for (std::uint32_t y = 0; y < f.header.height; ++y) {
            for (std::uint32_t x = 0; x < f.header.stride; ++x) {
                f.bytes[std::size_t{y} * f.header.stride + x] = static_cast<std::byte>((x + y + frame_index_) & 0xFFu);
            }
        }
        ++frame_index_;
        return f;
    }

    config::VehicleConfig vehicle_;
    SplitMix64 rng_;
    std::int64_t imu_period_;
    std::int64_t frame_period_;
    std::int64_t tick_period_;
    std::int64_t end_;
    std::int64_t next_imu_{0};
    std::int64_t next_frame_{0};
    std::int64_t next_tick_{0};
    std::uint32_t frame_index_{0};
    double gyro_amp_{0};
    double accel_amp_{0};
};

}  // namespace branes::anatomy::harness
