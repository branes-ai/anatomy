#pragma once
/// @file imu.hpp
/// @brief ImuModel: the simulated IMU (noise, bias, saturation, latency).
///
/// measure() turns the true body rates and specific force at time t into one
/// sample:
///
///   gyro  = ω + b_g + n_g,   accel = f + b_a + n_a
///
/// then clips each axis at ±range and sets its ImuSaturation bit. White noise
/// of density d sampled at rate f has σ = d·√f; it is drawn uniform on
/// [−σ√3, σ√3), which has that σ and needs no libm call, so the samples are
/// bit-reproducible wherever IEEE-754 rounds the same way. The biases start at
/// a uniform draw of the turn-on σ and random-walk per sample with
/// σ_step = d_rw·√(1/f).
///
/// The sample carries its measurement time t but is delivered at
/// t + latency (pop_due), as an FC-timestamped sample reaches the companion.
///
/// Model
/// -----
///   p_        : config::ImuParams [owned, immutable]
///   rng_      : SplitMix64 — the model's only random source
///   b_g_, b_a_: Vec3 — current biases
///   pending_  : ring of (delivery time, sample), capacity fixed at construction
///
/// Invariants
/// ----------
///   I1: the biases are finite                               — b_g_, b_a_
///   I2: pending samples are in non-decreasing delivery time — pending_
///   I3: every delivered axis has |value| ≤ range, and its saturation bit is
///       set iff the unclipped value was beyond the range     — (per sample)

#include <branes/anatomy/config/vehicle.hpp>
#include <branes/anatomy/msg/imu.hpp>
#include <branes/anatomy/util/splitmix64.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace branes::anatomy::sim {

class ImuModel {
public:
    /// Pre: `p` passed the vehicle loader (rate in range, latency in [0, 0.1] s).
    /// Post: biases drawn from the turn-on σ; nothing pending. I1, I2.
    ImuModel(const config::ImuParams& p, std::uint64_t seed) : p_(p), rng_(seed) {
        const double unit = std::sqrt(3.0);  // uniform on [−√3, √3) has σ = 1
        for (std::size_t i = 0; i < 3; ++i) {
            b_g_[i] = rng_.symmetric(unit) * p_.initial_gyro_bias_sigma;
            b_a_[i] = rng_.symmetric(unit) * p_.initial_accel_bias_sigma;
        }
        const double sqrt_rate = std::sqrt(p_.rate_hz);
        gyro_amp_ = p_.gyro_noise_density * sqrt_rate * unit;
        accel_amp_ = p_.accel_noise_density * sqrt_rate * unit;
        const double sqrt_dt = 1.0 / sqrt_rate;
        gyro_walk_amp_ = p_.gyro_bias_random_walk * sqrt_dt * unit;
        accel_walk_amp_ = p_.accel_bias_random_walk * sqrt_dt * unit;
        latency_ns_ = static_cast<std::int64_t>(std::llround(p_.latency_s * 1e9));
        // Samples in flight at once: latency · rate, plus slack for rounding.
        pending_.resize(static_cast<std::size_t>(p_.latency_s * p_.rate_hz) + 4);
        assert(check_invariant());
    }

    /// Measure at `t` and queue the sample for delivery at t + latency.
    /// Pre: t ≥ the previous measurement's t (I2); inputs finite.
    /// Returns the sample (the FC reads it at once, without the latency).
    ImuSample<double> measure(Timestamp t, const Vec3<double>& body_rates, const Vec3<double>& specific_force) {
        assert(!last_t_ || *last_t_ <= t);  // Pre / I2
        last_t_ = t;
        ImuSample<double> s{};
        s.t = t;
        s.temperature = 25.0;
        for (std::size_t i = 0; i < 3; ++i) {
            const double g = body_rates[i] + b_g_[i] + rng_.symmetric(gyro_amp_);
            const double a = specific_force[i] + b_a_[i] + rng_.symmetric(accel_amp_);
            s.gyro[i] = clip(g, p_.gyro_range, s.saturated, static_cast<std::uint8_t>(gyro_x_saturated << i));
            s.accel[i] = clip(a, p_.accel_range, s.saturated, static_cast<std::uint8_t>(accel_x_saturated << i));
        }
        for (std::size_t i = 0; i < 3; ++i) {
            b_g_[i] += rng_.symmetric(gyro_walk_amp_);
            b_a_[i] += rng_.symmetric(accel_walk_amp_);
        }
        push(Timestamp{t.ns + latency_ns_}, s);
        assert(check_invariant());
        return s;
    }

    /// Delivery time of the oldest pending sample, if any.
    [[nodiscard]] std::optional<Timestamp> next_delivery() const noexcept {
        if (size_ == 0) {
            return std::nullopt;
        }
        return pending_[head_].first;
    }

    /// The oldest pending sample if it is due at `now`.
    [[nodiscard]] std::optional<ImuSample<double>> pop_due(Timestamp now) noexcept {
        if (size_ == 0 || now < pending_[head_].first) {
            return std::nullopt;
        }
        ImuSample<double> s = pending_[head_].second;
        head_ = (head_ + 1) % pending_.size();
        --size_;
        return s;
    }

    /// Add a step to the biases (fault injection). Pre: finite.
    void add_bias(const Vec3<double>& gyro, const Vec3<double>& accel) noexcept {
        for (std::size_t i = 0; i < 3; ++i) {
            b_g_[i] += gyro[i];
            b_a_[i] += accel[i];
        }
        assert(check_invariant());
    }

    [[nodiscard]] const Vec3<double>& gyro_bias() const noexcept {
        return b_g_;
    }
    [[nodiscard]] const Vec3<double>& accel_bias() const noexcept {
        return b_a_;
    }
    [[nodiscard]] const config::ImuParams& params() const noexcept {
        return p_;
    }

private:
    static double clip(double v, double range, std::uint8_t& flags, std::uint8_t bit) noexcept {
        if (v > range) {
            flags = static_cast<std::uint8_t>(flags | bit);
            return range;
        }
        if (v < -range) {
            flags = static_cast<std::uint8_t>(flags | bit);
            return -range;
        }
        return v;  // I3
    }

    void push(Timestamp at, const ImuSample<double>& s) {
        if (size_ == pending_.size()) {
            // Cannot happen for a loader-checked rate and latency; grow rather
            // than drop, so the stream stays complete.
            std::vector<std::pair<Timestamp, ImuSample<double>>> grown(pending_.size() * 2);
            for (std::size_t i = 0; i < size_; ++i) {
                grown[i] = pending_[(head_ + i) % pending_.size()];
            }
            pending_ = std::move(grown);
            head_ = 0;
        }
        pending_[(head_ + size_) % pending_.size()] = {at, s};
        ++size_;
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        bool good = true;
        for (std::size_t i = 0; i < 3; ++i) {
            good = good && std::isfinite(b_g_[i]) && std::isfinite(b_a_[i]);  // I1
        }
        for (std::size_t k = 1; k < size_; ++k) {
            const auto& a = pending_[(head_ + k - 1) % pending_.size()].first;
            const auto& b = pending_[(head_ + k) % pending_.size()].first;
            good = good && !(b < a);  // I2
        }
        return good;
    }

    config::ImuParams p_;
    SplitMix64 rng_;
    Vec3<double> b_g_{};
    Vec3<double> b_a_{};
    double gyro_amp_{0};
    double accel_amp_{0};
    double gyro_walk_amp_{0};
    double accel_walk_amp_{0};
    std::int64_t latency_ns_{0};
    std::vector<std::pair<Timestamp, ImuSample<double>>> pending_;
    std::size_t head_{0};
    std::size_t size_{0};
    std::optional<Timestamp> last_t_;
};

}  // namespace branes::anatomy::sim
