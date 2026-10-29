#pragma once
/// @file fc.hpp
/// @brief SimFc: the simulated flight controller on the far side of the companion link.
///
/// The SITL harness hands SimFc the command each control tick produced
/// (receive) and steps it at the FC rate (step), which turns the last accepted
/// command into rotor speed commands for the plant:
///
///  - `thrust_rates`: the reflex rate loop on the FC's gyro, through a
///    Butterworth low-pass, gives a body torque; with the commanded collective
///    thrust it is mixed (reflex Quad X mixer) into rotor thrusts and speeds.
///    The mixer's saturation report feeds the rate loop's anti-windup on the
///    next step.
///  - `motors`: normalized rotor thrusts m_i ∈ [0, 1] bypass the rate loop:
///    ω_i = ω_max·√m_i, so k_f·ω_i² = m_i·T_max.
///
/// Failsafe: with no command yet, or none received for longer than the
/// command timeout, every rotor is commanded to zero and the rate loop is
/// reset (a stand-in for the reflex mode manager, Phase 4). An invalid command
/// is rejected and does not refresh the timeout.
///
/// Model
/// -----
///   rate_    : reflex RateController [owned] — gains from cascade_config_from
///   mixer_   : reflex Mixer [owned]
///   lpf_[3]  : reflex dsp::Biquad — the FC's gyro filter
///   cmd_     : optional<ThrustRates> — the last accepted command
///   rx_      : Timestamp — when cmd_ was received
///   sat_     : reflex Saturation — the last mix's report
///   out_     : RotorCommand — the last step's output
///
/// Invariants
/// ----------
///   I1: the configuration is valid (timeout > 0, dt > 0, ω_max > 0)
///   I2: cmd_, if any, passes valid(ThrustRates)
///   I3: every rotor command in out_ is in [0, ω_max]

#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/reflex/control/cascade.hpp>
#include <branes/reflex/control/mixer.hpp>
#include <branes/reflex/control/rate.hpp>
#include <branes/reflex/dsp/biquad.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace branes::anatomy::sim {

namespace rx = branes::reflex;

class SimFc {
public:
    /// Pre: quadrotor_params_valid(p); `s` passed the scenario loader.
    /// Post: no command (failsafe), rotors at zero. I1–I3.
    SimFc(const rx::model::QuadrotorParams<double>& p, const config::SimParams& s)
        : rate_(rx::control::cascade_config_from(p).rate), mixer_(rx::control::mixer_params_from(p)),
          omega_max_(rx::model::QuadrotorModel<double>(p).max_rotor_speed()),
          timeout_ns_(static_cast<std::int64_t>(std::llround(s.command_timeout_s * 1e9))),
          dt_(1.0 / s.physics_rate_hz) {
        const auto c = rx::dsp::butterworth_lowpass(s.fc_gyro_lowpass_hz, s.physics_rate_hz);
        for (auto& f : lpf_) {
            f = rx::dsp::Biquad<double>(c);
        }
        assert(check_invariant());
    }

    /// Accept `u`, received at `now`. Returns false (and keeps the previous
    /// command) if `u` is invalid or not finite.
    bool receive(const ThrustRates<double>& u, Timestamp now) {
        if (!usable(u)) {
            ++rejected_;
            return false;
        }
        cmd_ = u;
        rx_ = now;
        assert(check_invariant());
        return true;
    }

    /// One FC step at `now`, with the FC's gyro sample `gyro` [rad/s].
    /// Post: every rotor command in [0, ω_max] (I3).
    rx::model::RotorCommand<double> step(Timestamp now, const Vec3<double>& gyro) {
        std::array<double, 3> filtered{};
        for (std::size_t i = 0; i < 3; ++i) {
            filtered[i] = lpf_[i].update(gyro[i]);
        }
        rx::model::RotorCommand<double> out{};
        if (!cmd_ || now.ns - rx_.ns > timeout_ns_) {
            if (!failsafe_) {
                rate_.reset();
                sat_ = {};
                failsafe_ = true;
            }
            out_ = out;  // all zero
            assert(check_invariant());
            return out_;
        }
        failsafe_ = false;
        if (cmd_->kind == CommandKind::motors) {
            for (std::size_t i = 0; i < 4; ++i) {
                out.speeds[i] = omega_max_ * std::sqrt(cmd_->motors[i]);
            }
        } else {
            const auto rate_sp = rx::vec3(cmd_->body_rates[0], cmd_->body_rates[1], cmd_->body_rates[2]);
            const auto rate = rx::vec3(filtered[0], filtered[1], filtered[2]);
            const auto torque = rate_.update(rate_sp, rate, rx::zero3<double>(), dt_, sat_.torque);
            const rx::control::Wrench<double> demand{cmd_->thrust, torque};
            const rx::control::MixerOutput<double> mixed = mixer_.mix(demand);
            sat_ = rx::control::saturation_from(demand, mixed);
            out = mixer_.to_rotor_command(mixed.thrusts);
        }
        for (double& w : out.speeds) {
            w = w < 0 ? 0 : (w > omega_max_ ? omega_max_ : w);  // I3 against rounding
        }
        out_ = out;
        assert(check_invariant());
        return out_;
    }

    /// True while the FC commands zero thrust for lack of a fresh command.
    [[nodiscard]] bool failsafe() const noexcept {
        return failsafe_;
    }
    [[nodiscard]] std::uint64_t rejected() const noexcept {
        return rejected_;
    }
    [[nodiscard]] double max_rotor_speed() const noexcept {
        return omega_max_;
    }
    [[nodiscard]] const rx::model::RotorCommand<double>& output() const noexcept {
        return out_;
    }

private:
    [[nodiscard]] static bool usable(const ThrustRates<double>& u) noexcept {
        bool finite = std::isfinite(u.thrust);
        for (std::size_t i = 0; i < 3; ++i) {
            finite = finite && std::isfinite(u.body_rates[i]);
        }
        return finite && valid(u);  // valid() checks thrust ≥ 0 and motors ∈ [0, 1]
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        bool good = timeout_ns_ > 0 && dt_ > 0 && omega_max_ > 0;  // I1
        good = good && (!cmd_ || valid(*cmd_));                    // I2
        for (const double w : out_.speeds) {
            good = good && w >= 0 && w <= omega_max_;  // I3
        }
        return good;
    }

    rx::control::RateController<double> rate_;
    rx::control::Mixer<double> mixer_;
    std::array<rx::dsp::Biquad<double>, 3> lpf_{};
    double omega_max_;
    std::int64_t timeout_ns_;
    double dt_;
    std::optional<ThrustRates<double>> cmd_;
    Timestamp rx_{};
    rx::control::Saturation sat_{};
    rx::model::RotorCommand<double> out_{};
    bool failsafe_{true};
    std::uint64_t rejected_{0};
};

}  // namespace branes::anatomy::sim
