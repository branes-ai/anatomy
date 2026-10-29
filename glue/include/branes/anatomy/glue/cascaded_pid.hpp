#pragma once
/// @file cascaded_pid.hpp
/// @brief CascadedPid: the reflex position and attitude loops in the Controller slot.
///
/// Baseline B0's controller (racing-drone docs/plan/b0/). Each tick runs the reflex
/// PositionController on the reference's first sample (position, velocity
/// and acceleration feed-forward, yaw), then the AttitudeController on its
/// attitude target. The output is the collective thrust and the body-rate
/// setpoint: the rate loop belongs to the flight controller (SimFc in SITL),
/// so the slot sends exactly the thrust-and-rates command it defines.
///
/// The gains are reflex's cascade_config_from() for the vehicle file; the
/// baseline parameter max_speed_m_s caps the position loop's velocity
/// setpoint (horizontal, up, and down). Conflicts are not used (B0 has no DAA).
///
/// dt is the time between the estimates of consecutive ticks, converted to T
/// by a TimeBase (msg/time.hpp) in the named type `time_wide_type`. On the
/// first tick, when dt ≤ 0, or with an empty reference, the last command is
/// held, re-stamped with the estimate's time. So is a tick whose dt the
/// TimeBase rejects (longer than two periods, or not resolved in T to the
/// accuracy below); timing_faults() counts those.
///
/// The dt accuracy (racing-drone docs/plan/time-base/). dt enters only the velocity PIDs'
/// integral, k_i·e·dt (their k_d is 0; the attitude loop has no dt), so a
/// relative dt error ε is a relative k_i error ε. reflex's design puts the
/// velocity crossover at ω_c ≈ k_v and the integral zero at r·ω_c, r = k_i/k_v²;
/// the zero costs phase atan((1+ε)·r) there, so ε costs ε·r/(1+r²) rad to first
/// order. A dt error may cost no more phase margin than one control period of
/// delay already does, ω_c·T_s:
///
///   ε_max = ω_c·T_s·(1 + r²)/r,   the smallest over the three axes
///
/// (≈ 0.144 for RP-1 at 250 Hz). First order in ε; ω_c ≈ k_v treats the inner
/// loops as ideal.
///
/// Model
/// -----
///   position_      : reflex PositionController [owned]
///   attitude_      : reflex AttitudeController [owned]
///   time_          : TimeBase<T, time_wide_type> — tick intervals to dt [owned]
///   last_t_        : optional<Timestamp> — t_valid of the previous tick's estimate
///   last_          : ThrustRates — the last command (hover thrust, zero rates at first)
///   timing_faults_ : uint64 — ticks held because time_ rejected their dt
///
/// Invariants
/// ----------
///   I1: the position and attitude configs are valid (reflex validators); the
///       vehicle's thrust-to-weight exceeds 1.25 (cascade_config_from)
///                                            — constrains position_, attitude_
///   I2: T_min ≤ last_.thrust ≤ T_max; last_.kind == thrust_rates
///                                            — constrains last_
///   I3: last_t_ never decreases               — constrains last_t_

#include <branes/anatomy/glue/reflex_params.hpp>
#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/daa.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/slots/context.hpp>
#include <branes/reflex/control/attitude.hpp>
#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/position.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>

#include <cassert>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

namespace branes::anatomy::glue {

namespace rx = branes::reflex;

template <typename T>
class CascadedPid {
public:
    /// The type tick intervals are converted to dt in. Named here, not hidden:
    /// TimeBase rejects, at compile time, a T with more digits than it.
    using time_wide_type = double;

    /// Pre: position_config_valid(pos), attitude_config_valid(att).
    /// Post: I1–I3; no tick yet; last_ = hover thrust, zero rates.
    CascadedPid(const rx::control::PositionConfig<T>& pos,
                const rx::control::AttitudeConfig<T>& att,
                const TimeBase<T, time_wide_type>& time)
        : position_(pos), attitude_(att), time_(time) {
        last_.thrust = position_.target().thrust;
        assert(check_invariant());
    }

    /// From the vehicle file (cascade_config_from) and the baseline parameters
    /// max_speed_m_s (default 4.5) and control_rate_hz (the apps' default).
    /// Fails if the vehicle cannot hover with margin (thrust-to-weight ≤ 1.25),
    /// a parameter is out of range, or T can't convert the control period to
    /// the dt accuracy ε_max. Mission-neutral: it reads only the vehicle, the
    /// baseline, and the world's start pose.
    template <MissionWorld World>
    [[nodiscard]] static result<CascadedPid, std::string> from_context(const BasicSlotContext<World>& ctx) {
        const auto p = reflex_params_from<T>(ctx.vehicle, ctx.world.start.position[2]);
        if (!p) {
            return p.error();
        }
        const auto max_speed = parameter_in(ctx.baseline, "max_speed_m_s", 4.5, 0.1, 50.0);
        if (!max_speed) {
            return max_speed.error();
        }
        const auto rate_hz =
            parameter_in(ctx.baseline, "control_rate_hz", config::kDefaultControlRateHz, kMinRateHz, kMaxRateHz);
        if (!rate_hz) {
            return rate_hz.error();
        }
        rx::control::CascadeConfig<T> c = rx::control::cascade_config_from(*p);
        const T v = T(*max_speed);
        c.position.max_speed_xy = v;
        c.position.max_speed_up = v < c.position.max_speed_up ? v : c.position.max_speed_up;
        c.position.max_speed_down = v < c.position.max_speed_down ? v : c.position.max_speed_down;
        if (!rx::control::position_config_valid(c.position) || !rx::control::attitude_config_valid(c.attitude)) {
            return std::string("CascadedPid: vehicle '" + ctx.vehicle.name +
                               "' cannot hover inside the cascade's thrust limit (thrust-to-weight must exceed 1.25)");
        }
        const std::int64_t period = period_ns(*rate_hz);
        const auto tol = dt_tolerance(c.position, static_cast<time_wide_type>(period) * time_wide_type(1e-9));
        if (!tol) {
            return tol.error();
        }
        const auto time = make_time_base<T, time_wide_type>(period, *tol);
        if (!time) {
            return "CascadedPid: " + time.error() + " (the controller's dt accuracy ε_max)";
        }
        return CascadedPid(c.position, c.attitude, *time);
    }

    /// ε_max for the velocity loops of `c` at control period `period_s` (see
    /// the file comment). Fails unless every axis is a PI loop (k_p, k_i > 0,
    /// k_d = 0), the loops the derivation covers.
    [[nodiscard]] static result<time_wide_type, std::string> dt_tolerance(const rx::control::PositionConfig<T>& c,
                                                                          time_wide_type period_s) {
        using W = time_wide_type;
        W tol = std::numeric_limits<W>::infinity();
        for (const auto& v : c.velocity) {
            const W kv = static_cast<W>(v.kp);
            const W ki = static_cast<W>(v.ki);
            if (!(kv > W(0) && ki > W(0) && static_cast<W>(v.kd) == W(0))) {
                return std::string(
                    "CascadedPid: the dt accuracy is derived for PI velocity loops (k_p, k_i > 0, k_d = 0)");
            }
            const W r = ki / (kv * kv);
            const W e = kv * period_s * (W(1) + r * r) / r;
            tol = e < tol ? e : tol;
        }
        return tol;
    }

    /// One control tick. Pre: valid(x); x and ref finite.
    /// Post: I1–I3; valid(result), result.kind == thrust_rates, result.t == x.t_valid.
    [[nodiscard]] ThrustRates<T> step(const StateEstimate<T>& x, const Reference<T>& ref, const ConflictSet<T>&) {
        assert(valid(x));  // Pre
        const std::optional<Timestamp> prev = last_t_;
        if (!prev || *prev < x.t_valid) {
            last_t_ = x.t_valid;  // I3
        }
        const std::optional<T> dt = prev && *prev < x.t_valid ? time_.seconds(x.t_valid.ns - prev->ns) : std::nullopt;
        if (prev && *prev < x.t_valid && !dt) {
            ++timing_faults_;
        }
        if (!dt || ref.count == 0) {
            last_.t = x.t_valid;  // hold
            assert(check_invariant());
            return last_;
        }
        const ReferenceSample<T>& r = ref.samples[0];
        rx::control::PositionSetpoint<T> sp;
        sp.position = v3(r.position);
        sp.velocity = v3(r.velocity);
        sp.acceleration = v3(r.acceleration);
        sp.yaw = r.yaw;
        const rx::Quat<T> q{x.attitude.w, x.attitude.x, x.attitude.y, x.attitude.z};
        const rx::control::AttitudeTarget<T> target = position_.update(v3(x.position), v3(x.velocity), q, sp, *dt);
        const rx::Vec3<T> rates = attitude_.update(q, target.attitude);

        last_.t = x.t_valid;
        last_.thrust = target.thrust;
        last_.body_rates = {rates(0), rates(1), rates(2)};
        assert(check_invariant());
        return last_;
    }

    [[nodiscard]] const rx::control::PositionController<T>& position_loop() const noexcept {
        return position_;
    }

    [[nodiscard]] const TimeBase<T, time_wide_type>& time_base() const noexcept {
        return time_;
    }

    /// Ticks held because the TimeBase rejected their dt.
    [[nodiscard]] std::uint64_t timing_faults() const noexcept {
        return timing_faults_;
    }

private:
    [[nodiscard]] static rx::Vec3<T> v3(const Vec3<T>& a) {
        return rx::vec3(a[0], a[1], a[2]);
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        const auto& c = position_.config();
        bool good = rx::control::position_config_valid(c) && rx::control::attitude_config_valid(attitude_.config());
        good = good && !(last_.thrust < c.min_thrust) && !(c.max_thrust < last_.thrust);  // I2
        return good && last_.kind == CommandKind::thrust_rates && valid(last_);           // I2
    }

    rx::control::PositionController<T> position_;
    rx::control::AttitudeController<T> attitude_;
    TimeBase<T, time_wide_type> time_;
    std::optional<Timestamp> last_t_;
    ThrustRates<T> last_{};
    std::uint64_t timing_faults_{0};
};

}  // namespace branes::anatomy::glue
