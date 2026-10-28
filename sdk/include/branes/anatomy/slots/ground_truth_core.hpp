#pragma once
/// @file ground_truth_core.hpp
/// @brief GroundTruthCore: the simulated true state as the estimate (SITL only).
///
/// it never reads the
/// mission's perception. A mission's ground-truth estimator adds the frame
/// input its Estimator slot takes (racing-drone: GroundTruthEstimator,
/// ground_truth.hpp). It is a TruthConsumer: the stack
/// hands it each truth record, and state_at() reports the latest one, with
/// zero biases and zero covariance. It ignores the IMU and camera. It never
/// receives truth in flight (SYS-02), so there it reports the vehicle at rest
/// and health().ok stays false.
///
/// Model
/// -----
///   last_   : std::optional<TruthState<T>> — the latest truth record
///   latest_ : Timestamp — the latest input time seen (truth or IMU)
///
/// Invariants
/// ----------
///   I1: last_, if set, passes valid(TruthState)   — constrains last_
///   I2: last_, if set, has last_->t ≤ latest_     — constrains last_, latest_

#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/msg/truth.hpp>

#include <cassert>
#include <optional>

namespace branes::anatomy {

template <typename T>
class GroundTruthCore {
public:
    /// Pre: valid(x). Post: last_ == x; I1, I2.
    void process_truth(const TruthState<T>& x) {
        assert(valid(x));  // Pre / I1
        last_ = x;
        if (latest_ < x.t) {
            latest_ = x.t;
        }
        assert(check_invariant());
    }

    /// The IMU only advances the input clock that staleness is measured on.
    void process_imu(const ImuSample<T>& s) noexcept {
        if (latest_ < s.t) {
            latest_ = s.t;
        }
    }

    /// The latest truth (t_valid = its time), or at rest at `now` before any.
    /// Post: valid(result).
    [[nodiscard]] StateEstimate<T> state_at(Timestamp now) const noexcept {
        StateEstimate<T> x{};  // zero biases, zero covariance
        if (!last_) {
            x.t_valid = now;
            return x;
        }
        x.t_valid = last_->t;
        x.position = last_->position;
        x.velocity = last_->velocity;
        x.attitude = last_->attitude;
        x.body_rates = last_->body_rates;
        return x;
    }

    /// ok iff truth has arrived; staleness is the input-clock time since it.
    [[nodiscard]] EstimatorHealth<T> health() const noexcept {
        if (!last_) {
            return EstimatorHealth<T>{T(0), 0, false};
        }
        return EstimatorHealth<T>{T(0), latest_.ns - last_->t.ns, true};
    }

private:
    [[nodiscard]] bool check_invariant() const noexcept {
        return !last_ || (valid(*last_) && !(latest_ < last_->t));  // I1, I2
    }

    std::optional<TruthState<T>> last_;
    Timestamp latest_{};
};

}  // namespace branes::anatomy
