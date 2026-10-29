#pragma once
/// @file basic_simulator.hpp
/// @brief BasicSimulator: the closed-loop SITL event source on the reflex dynamics.
///
/// The mission
/// supplies the vehicle's start pose and an oracle sensor (OracleSensor): what
/// it perceives of its task at each camera frame, computed from the truth
/// (racing-drone: the gate projector, GateOracle in simulator.hpp).
///
/// It replaces Phase 0's SyntheticSensors in sitl_<baseline>. The harness
/// pulls events with next() and feeds them to the stack exactly as before; it
/// hands each tick's command back with command(), and the simulated FC
/// (SimFc) applies it to the plant (reflex QuadrotorModel).
///
/// Time is integer nanoseconds. The plant advances on a fixed grid of
/// `physics_rate_hz`; before any event at time t is produced, the plant has
/// been stepped up to the last grid point ≤ t, so every event at t sees the
/// state at that point. Each plant step from t first delivers the commands
/// due by t to the FC, then runs one FC step on the FC's latest gyro sample.
/// At equal times the events come out in this order:
///
///   sim_config (t = 0 only) → IMU deliveries → camera → oracle → truth → tick
///
/// The oracle event is the one the oracle returns for the frame's time, on the
/// oracle's channel (racing-drone: oracle_gates); an oracle may return none.
///
/// An IMU measurement at t is taken before the deliveries at t, so with zero
/// latency a sample is delivered at the time it was measured. A command from
/// the tick at t takes effect in the plant step from t (plus any injected
/// delay).
///
/// Random streams (all SplitMix64, each seeded from the scenario seed and its
/// own constant, so adding a fault does not shift the IMU noise): the IMU,
/// the randomization draws, and the camera-corruption draws.
///
/// Model
/// -----
///   model_, x_     : reflex QuadrotorModel and its state at now_
///   fc_, imu_      : SimFc, ImuModel [owned]
///   oracle_, cam_  : the mission's oracle sensor and the camera [owned]
///   pending_       : the oracle event of the last frame, until it is emitted
///   next_*         : next time per stream [ns]; end_ the run's end
///   commands_      : (apply time, command), sorted by apply time
///   faults_        : the scenario's faults, with a fired flag for one-shots
///
/// Invariants
/// ----------
///   I1: events come out in non-decreasing time, in the order above
///   I2: x_ passes quadrotor_state_valid
///   I3: every period is > 0, and the physics period ≤ min(10 ms, τ)
///   I4: the same inputs (configs, seed, commands) give the same events

#include <branes/anatomy/config/scenario.hpp>
#include <branes/anatomy/config/vehicle.hpp>
#include <branes/anatomy/harness/synthetic.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/msg/codec.hpp>
#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/imu.hpp>
#include <branes/anatomy/msg/region.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/msg/truth.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/sim/camera_model.hpp>
#include <branes/anatomy/sim/fc.hpp>
#include <branes/anatomy/sim/imu.hpp>
#include <branes/anatomy/sim/vehicle.hpp>
#include <branes/anatomy/util/splitmix64.hpp>
#include <branes/reflex/math/quat.hpp>
#include <branes/reflex/math/vec3.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::sim {

namespace rx = branes::reflex;

using SimEvent = harness::SensorEvent;
using PlantState = rx::model::QuadrotorState<double>;

/// Test and tooling hooks; a normal SITL run uses the defaults.
struct SimOptions {
    std::optional<PlantState> initial_state;  ///< start here instead of at rest on the pad
};

/// A mission's oracle sensor: at each camera frame, its perception of the
/// task computed from the true state at the frame's time, as the log event to
/// emit (on the mission's own channel), or none. It must be deterministic in
/// its inputs (I4).
template <class O>
concept OracleSensor = requires(O& o, const TruthState<double>& x, Timestamp t) {
    { o.observe(x, t) } -> std::same_as<std::optional<SimEvent>>;
};

/// An oracle that perceives nothing: no oracle events.
struct NoOracle {
    [[nodiscard]] std::optional<SimEvent> observe(const TruthState<double>&, Timestamp) const noexcept {
        return std::nullopt;
    }
};

template <OracleSensor Oracle>
class BasicSimulator {
public:
    /// Validate the inputs and build the run, starting at `start`.
    [[nodiscard]] static result<BasicSimulator, std::string> create(const config::VehicleConfig& vehicle,
                                                                    const StartPose<double>& start,
                                                                    const config::ScenarioConfig& scenario,
                                                                    double control_hz,
                                                                    Oracle oracle,
                                                                    const SimOptions& options = {}) {
        const auto phys = checked_period_ns(scenario.sim.physics_rate_hz);
        const auto imu = checked_period_ns(vehicle.imu.rate_hz);
        const auto frame = checked_period_ns(vehicle.camera.rate_hz);
        const auto tick = checked_period_ns(control_hz);
        if (!phys || !imu || !frame || !tick) {
            return std::string("physics, IMU, camera, and control rates must be in [" + std::to_string(kMinRateHz) +
                               ", " + std::to_string(kMaxRateHz) + "] Hz");
        }
        if (!(scenario.duration_s > 0 && scenario.duration_s <= kMaxDurationS)) {
            return std::string("duration must be in (0, " + std::to_string(kMaxDurationS) + "] s");
        }
        for (std::size_t i = 0; i < scenario.faults.size(); ++i) {
            if (!config::fault_event_valid(scenario.faults[i])) {
                return std::string("fault " + std::to_string(i) + " is out of range (see config::FaultEvent)");
            }
        }
        auto base = reflex_params_from(vehicle, start.position[2]);
        if (!base) {
            return base.error();
        }
        SplitMix64 draw_rng(stream_seed(scenario.seed, kDrawStream));
        const Draws d = draw(scenario.randomization, draw_rng);
        const PlantParams p = apply(*base, d);
        const double dt = static_cast<double>(*phys) * 1e-9;
        if (!rx::model::quadrotor_params_valid(p)) {
            return std::string("the randomized vehicle parameters are not valid");
        }
        if (!(dt <= 0.01 && dt <= p.motor_time_constant)) {
            return std::string("the physics step must be ≤ 10 ms and ≤ the motor time constant");  // I3
        }
        if (options.initial_state &&
            !rx::model::QuadrotorModel<double>(p).quadrotor_state_valid(*options.initial_state)) {
            return std::string("the initial state is not a valid plant state (see reflex quadrotor_state_violation)");
        }
        return BasicSimulator(vehicle, start, scenario, std::move(oracle), p, d, *phys, *imu, *frame, *tick, options);
    }

    /// The next event, or nullopt once every stream has passed the end.
    [[nodiscard]] std::optional<SimEvent> next() {
        if (!config_sent_) {
            config_sent_ = true;
            const std::string text = to_yaml(draws_);
            std::vector<std::byte> payload(text.size());
            std::transform(text.begin(), text.end(), payload.begin(), [](char c) { return static_cast<std::byte>(c); });
            return SimEvent{log::Channel::sim_config, Timestamp{0}, std::move(payload)};
        }
        for (;;) {
            std::int64_t t = std::min({next_measure_, next_frame_, next_truth_, next_tick_});
            if (pending_) {
                t = std::min(t, pending_->t.ns);
            }
            if (const auto d = imu_.next_delivery()) {
                t = std::min(t, d->ns);
            }
            if (t >= end_) {
                return std::nullopt;
            }
            while (now_ + phys_ns_ <= t) {
                advance();
            }
            if (next_measure_ == t) {  // internal: the FC's gyro and the companion's sample
                measure(Timestamp{t});
                continue;
            }
            if (auto s = imu_.pop_due(Timestamp{t})) {
                return SimEvent{log::Channel::imu, Timestamp{t}, codec::encode(*s)};
            }
            if (next_frame_ == t) {
                next_frame_ += frame_ns_;
                if (active(config::FaultType::camera_blackout, t)) {
                    (void)camera_rng_.uniform01();  // keep the corruption stream aligned
                    continue;
                }
                pending_ = oracle_.observe(truth(Timestamp{t}), Timestamp{t});
                assert(!pending_ || pending_->t.ns == t);  // the oracle reports at the frame's time (I1)
                return SimEvent{log::Channel::camera, Timestamp{t}, codec::encode(frame_at(t))};
            }
            if (pending_ && pending_->t.ns == t) {
                SimEvent e = std::move(*pending_);
                pending_.reset();
                return e;
            }
            if (next_truth_ == t) {
                next_truth_ += phys_ns_;
                return SimEvent{log::Channel::truth, Timestamp{t}, codec::encode(truth(Timestamp{t}))};
            }
            assert(next_tick_ == t);
            next_tick_ += tick_ns_;
            last_tick_ = Timestamp{t};
            return SimEvent{log::Channel::tick, Timestamp{t}, {}};
        }
    }

    /// The command the tick just produced. It reaches the FC after any
    /// injected delay, or never during a link loss.
    void command(const ThrustRates<double>& u) {
        const std::int64_t t = last_tick_.ns;
        if (active(config::FaultType::link_loss, t)) {
            ++dropped_;
            return;
        }
        std::int64_t delay = 0;
        for (const Fault& f : faults_) {
            if (f.event.type == config::FaultType::command_delay && in_window(f.event, t)) {
                delay = std::max(delay, static_cast<std::int64_t>(std::llround(f.event.delay_s * 1e9)));
            }
        }
        const std::pair<std::int64_t, ThrustRates<double>> entry{t + delay, u};
        const auto at = std::upper_bound(
            commands_.begin(), commands_.end(), entry, [](const auto& a, const auto& b) { return a.first < b.first; });
        commands_.insert(at, entry);
    }

    /// The plant state at now().
    [[nodiscard]] const PlantState& state() const noexcept {
        return x_;
    }
    /// Time of the plant state.
    [[nodiscard]] Timestamp now() const noexcept {
        return Timestamp{now_};
    }
    [[nodiscard]] TruthState<double> truth(Timestamp t) const {
        TruthState<double> s{};
        s.t = t;
        s.position = {x_.position(0), x_.position(1), x_.position(2)};
        s.velocity = {x_.velocity(0), x_.velocity(1), x_.velocity(2)};
        s.attitude = {x_.attitude.w, x_.attitude.x, x_.attitude.y, x_.attitude.z};
        s.body_rates = {x_.body_rates(0), x_.body_rates(1), x_.body_rates(2)};
        s.rotor_speeds = x_.rotor_speeds;
        s.on_ground = x_.on_ground;
        return s;
    }
    [[nodiscard]] const rx::model::QuadrotorModel<double>& model() const noexcept {
        return model_;
    }
    [[nodiscard]] const SimFc& fc() const noexcept {
        return fc_;
    }
    [[nodiscard]] const ImuModel& imu() const noexcept {
        return imu_;
    }
    [[nodiscard]] const Draws& draws() const noexcept {
        return draws_;
    }
    [[nodiscard]] std::uint64_t dropped_commands() const noexcept {
        return dropped_;
    }
    /// The rotor command of the last plant step (after motor faults).
    [[nodiscard]] const rx::model::RotorCommand<double>& rotor_command() const noexcept {
        return applied_;
    }

    static constexpr std::uint64_t kImuStream = 0x494D55;     // "IMU"
    static constexpr std::uint64_t kDrawStream = 0x52414E44;  // "RAND"
    static constexpr std::uint64_t kCameraStream = 0x43414D;  // "CAM"

    /// The seed of one of the run's random streams.
    [[nodiscard]] static constexpr std::uint64_t stream_seed(std::uint64_t seed, std::uint64_t stream) noexcept {
        SplitMix64 mix(seed ^ (stream * 0x9E3779B97F4A7C15ull));
        return mix.next();
    }

    /// The mission's oracle sensor.
    [[nodiscard]] const Oracle& oracle() const noexcept {
        return oracle_;
    }

private:
    BasicSimulator(const config::VehicleConfig& vehicle,
                   const StartPose<double>& start_pose,
                   const config::ScenarioConfig& scenario,
                   Oracle oracle,
                   const PlantParams& p,
                   const Draws& d,
                   std::int64_t phys_ns,
                   std::int64_t imu_ns,
                   std::int64_t frame_ns,
                   std::int64_t tick_ns,
                   const SimOptions& options)
        : model_(p), fc_(p, scenario.sim), imu_(vehicle.imu, stream_seed(scenario.seed, kImuStream)),
          oracle_(std::move(oracle)), cam_(vehicle.camera), draws_(d),
          camera_rng_(stream_seed(scenario.seed, kCameraStream)), test_pattern_(scenario.sim.test_pattern_frames),
          phys_ns_(phys_ns), imu_ns_(imu_ns), frame_ns_(frame_ns), tick_ns_(tick_ns),
          dt_(static_cast<double>(phys_ns) * 1e-9), end_(static_cast<std::int64_t>(scenario.duration_s * 1e9)) {
        for (const config::FaultEvent& f : scenario.faults) {
            faults_.push_back(Fault{f, false});
        }
        if (options.initial_state) {
            x_ = *options.initial_state;
        } else {
            const StartPose<double> start = apply(start_pose, d);
            x_.position = rx::vec3(start.position[0], start.position[1], start_pose.position[2]);
            x_.attitude = rx::from_rotation_vector(rx::vec3(0.0, 0.0, start.yaw));
            x_.on_ground = true;
        }
        assert(model_.quadrotor_state_valid(x_));  // I2
    }

    struct Fault {
        config::FaultEvent event;
        bool fired;
    };

    /// The fault's start or end time in ns (seconds rounded to the nearest ns,
    /// so window edges fall exactly on the event grid).
    [[nodiscard]] static std::int64_t ns_of(double seconds) noexcept {
        return static_cast<std::int64_t>(std::llround(seconds * 1e9));
    }

    [[nodiscard]] static bool in_window(const config::FaultEvent& f, std::int64_t t) noexcept {
        return t >= ns_of(f.t_s) && t < ns_of(f.until_s);
    }

    [[nodiscard]] bool active(config::FaultType type, std::int64_t t) const noexcept {
        for (const Fault& f : faults_) {
            if (f.event.type == type && in_window(f.event, t)) {
                return true;
            }
        }
        return false;
    }

    /// One plant step from now_: deliver due commands, step the FC, step the plant.
    void advance() {
        while (!commands_.empty() && commands_.front().first <= now_) {
            (void)fc_.receive(commands_.front().second, Timestamp{now_});
            commands_.erase(commands_.begin());
        }
        rx::model::RotorCommand<double> u = fc_.step(Timestamp{now_}, fc_gyro_);
        for (const Fault& f : faults_) {
            if (f.event.type == config::FaultType::motor_scale && in_window(f.event, now_)) {
                // Thrust is k_f·ω², so a thrust fraction s is a speed fraction √s.
                u.speeds[f.event.rotor] *= std::sqrt(f.event.scale);
            }
        }
        applied_ = u;
        x_ = model_.step(x_, u, dt_);
        now_ += phys_ns_;
        assert(model_.quadrotor_state_valid(x_));  // I2
    }

    /// The IMU measurement at t: true body rates and specific force at the plant state.
    void measure(Timestamp t) {
        next_measure_ += imu_ns_;
        for (Fault& f : faults_) {
            if (f.event.type == config::FaultType::imu_bias_step && !f.fired && t.ns >= ns_of(f.event.t_s)) {
                imu_.add_bias(f.event.gyro, f.event.accel);
                f.fired = true;
            }
        }
        // Specific force: what the accelerometer reads, Rᵀ(v̇ + g e₃) in the
        // body frame; on the ground the contact cancels the motion, Rᵀ g e₃.
        Vec3<double> a_world{0, 0, model_.params().gravity};
        if (!x_.on_ground) {
            const auto d = model_.derivative(x_, applied_);
            a_world = {d.velocity(0), d.velocity(1), d.velocity(2) + model_.params().gravity};
        }
        const Quat<double> q{x_.attitude.w, x_.attitude.x, x_.attitude.y, x_.attitude.z};
        const Vec3<double> f_body = detail::rotate(detail::conjugate(q), a_world);
        const Vec3<double> rates{x_.body_rates(0), x_.body_rates(1), x_.body_rates(2)};
        const ImuSample<double> s = imu_.measure(t, rates, f_body);
        fc_gyro_ = s.gyro;
    }

    [[nodiscard]] CameraFrameRecord frame_at(std::int64_t t) {
        CameraFrameRecord f{};
        f.header.t_mid = Timestamp{t};
        f.header.exposure_ns = 2'000'000;
        f.header.row_readout_ns = cam_.row_readout_ns;
        f.header.format = cam_.format;
        f.header.corrupted = false;
        const double draw_u = camera_rng_.uniform01();
        for (const Fault& fault : faults_) {
            if (fault.event.type == config::FaultType::camera_corrupt && in_window(fault.event, t) &&
                draw_u < fault.event.probability) {
                f.header.corrupted = true;
            }
        }
        if (test_pattern_) {
            f.header.width = cam_.width;
            f.header.height = cam_.height;
            f.header.stride = cam_.width * bytes_per_pixel(cam_.format);
            f.bytes.resize(image_bytes(f.header));
            for (std::uint32_t y = 0; y < f.header.height; ++y) {
                for (std::uint32_t x = 0; x < f.header.stride; ++x) {
                    f.bytes[std::size_t{y} * f.header.stride + x] =
                        static_cast<std::byte>((x + y + frame_index_) & 0xFFu);
                }
            }
        }
        ++frame_index_;
        return f;
    }

    rx::model::QuadrotorModel<double> model_;
    PlantState x_{};
    SimFc fc_;
    ImuModel imu_;
    Oracle oracle_;
    config::CameraParams cam_;
    Draws draws_;
    SplitMix64 camera_rng_;
    bool test_pattern_;
    std::int64_t phys_ns_;
    std::int64_t imu_ns_;
    std::int64_t frame_ns_;
    std::int64_t tick_ns_;
    double dt_;
    std::int64_t end_;
    std::int64_t now_{0};
    std::int64_t next_measure_{0};
    std::int64_t next_frame_{0};
    std::int64_t next_truth_{0};
    std::int64_t next_tick_{0};
    Timestamp last_tick_{};
    bool config_sent_{false};
    std::optional<SimEvent> pending_;
    std::vector<std::pair<std::int64_t, ThrustRates<double>>> commands_;
    std::vector<Fault> faults_;
    Vec3<double> fc_gyro_{};
    rx::model::RotorCommand<double> applied_{};
    std::uint32_t frame_index_{0};
    std::uint64_t dropped_{0};
};

}  // namespace branes::anatomy::sim
