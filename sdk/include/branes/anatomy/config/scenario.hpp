#pragma once
/// @file scenario.hpp
/// @brief The scenario file: types, loader, emitter.
///
/// Every loader validates the
/// whole file before it returns a value, so a config that loads satisfies all
/// the invariants listed on its type. Loading never throws; it returns
/// result<X, ConfigError>. Every emitter writes a file that loads back to an
/// equal value (round trip), with doubles written to 17 significant digits.
///
/// The mission supplies a ScenarioSchema: the key of its world file and its own
/// `sim:` parameters (racing-drone: config.hpp, kRacingScenario).

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace branes::anatomy::config {

// ── Scenario ────────────────────────────────────────────────────────

/// SITL settings of a scenario (`sim:`), all optional.
struct SimParams {
    double physics_rate_hz{1000};     ///< plant and simulated-FC step rate
    double command_timeout_s{0.1};    ///< simulated FC: no command for longer → rotors off
    double fc_gyro_lowpass_hz{100};   ///< simulated FC's gyro filter cutoff
    bool test_pattern_frames{false};  ///< camera records carry a test-pattern image (else header only)

    friend bool operator==(const SimParams&, const SimParams&) = default;
};

/// Per-run randomization (`randomization:`), drawn from the seed. Each field
/// is a half-width: a parameter p becomes p·(1 + u), u uniform in [−f, f].
struct Randomization {
    double mass{0};
    double inertia{0};
    double drag{0};
    double motor_time_constant{0};
    double start_xy_m{0};     ///< start position jitter per axis [m]
    double start_yaw_rad{0};  ///< start yaw jitter [rad]

    friend bool operator==(const Randomization&, const Randomization&) = default;
};

enum class FaultType : std::uint8_t {
    imu_bias_step,    ///< add `gyro` / `accel` to the IMU biases at t_s
    camera_corrupt,   ///< mark each frame corrupted with `probability`
    camera_blackout,  ///< deliver no frames
    command_delay,    ///< delay commands to the simulated FC by `delay_s`
    link_loss,        ///< drop commands to the simulated FC
    motor_scale,      ///< rotor `rotor` delivers `scale` of its commanded thrust
};

/// One timed fault (`faults:` entry). Active on [t_s, until_s); until_s
/// defaults to the end of the run. imu_bias_step acts once, at t_s.
///
/// Invariants (fault_event_valid): every field finite;
/// 0 ≤ t_s < until_s ≤ kMaxDurationS; probability, delay_s, scale ∈ [0, 1];
/// rotor < 4.
struct FaultEvent {
    FaultType type{FaultType::link_loss};
    double t_s{0};
    double until_s{kMaxDurationS};
    Vec3<double> gyro{};     ///< imu_bias_step [rad/s]
    Vec3<double> accel{};    ///< imu_bias_step [m/s²]
    double probability{0};   ///< camera_corrupt, ∈ [0, 1]
    double delay_s{0};       ///< command_delay, ∈ [0, 1] s
    std::uint32_t rotor{0};  ///< motor_scale, < 4
    double scale{1};         ///< motor_scale, ∈ [0, 1]

    friend bool operator==(const FaultEvent&, const FaultEvent&) = default;
};

[[nodiscard]] std::string_view name_of(FaultType t) noexcept;

/// True if `f` satisfies FaultEvent's invariants. The scenario loader enforces
/// them field by field; a Simulator built from a programmatic scenario checks
/// them with this.
[[nodiscard]] bool fault_event_valid(const FaultEvent& f) noexcept;

/// A mission's own `sim:` parameter: a number ≥ `min`, `fallback` when absent.
struct WorldSimParam {
    std::string_view key;
    double fallback{0};
    double min{0};
};

/// What a mission adds to the scenario file: the key that names its world file
/// (racing-drone: `course`), and its own `sim:` parameters (racing-drone: the oracle gate
/// corner noise).
///
/// Invariants: world_key is not one of the framework's keys; the world_sim
/// keys are distinct and not framework `sim:` keys; every fallback ≥ its min.
struct ScenarioSchema {
    std::string_view world_key;
    std::span<const WorldSimParam> world_sim;
};

/// The mission's `sim:` parameters at their fallbacks, in schema order.
[[nodiscard]] inline std::vector<double> default_world_sim(const ScenarioSchema& schema) {
    std::vector<double> v;
    for (const WorldSimParam& p : schema.world_sim) {
        v.push_back(p.fallback);
    }
    return v;
}

/// scenarios/<name>.yaml. World and vehicle paths are relative to the scenario
/// file's directory.
///
/// Invariants: the world and vehicle files exist; duration_s > 0; seed is given;
/// speed_limit_m_s, if given, > 0; sim rates and limits are in range; every
/// world_sim value is ≥ its schema min, one per schema parameter, in schema
/// order; randomization fractions ∈ [0, 1); every fault has 0 ≤ t_s < until_s
/// and its own fields in range.
struct ScenarioConfig {
    std::string name;
    std::string world_ref;               ///< as written in the file, under the schema's world_key
    std::string vehicle_ref;             ///< as written in the file
    std::filesystem::path world_path;    ///< resolved
    std::filesystem::path vehicle_path;  ///< resolved
    double duration_s{0};
    std::uint64_t seed{0};
    std::optional<double> speed_limit_m_s;  ///< optional: the scoreboard fails a run that exceeds it [m/s]
    SimParams sim;                          ///< optional `sim:` block, the framework's keys
    std::vector<double> world_sim;          ///< the mission's `sim:` keys, in schema order
    Randomization randomization;            ///< optional `randomization:` block
    std::vector<FaultEvent> faults;         ///< optional `faults:` list, in file order

    friend bool operator==(const ScenarioConfig&, const ScenarioConfig&) = default;
};

[[nodiscard]] result<ScenarioConfig, ConfigError> load_scenario(const std::filesystem::path& path,
                                                                const ScenarioSchema& schema);
/// `base_dir` resolves the world and vehicle references.
[[nodiscard]] result<ScenarioConfig, ConfigError> parse_scenario(std::string_view yaml,
                                                                 std::string_view origin,
                                                                 const std::filesystem::path& base_dir,
                                                                 const ScenarioSchema& schema);
/// Pre: s.world_sim has one value per schema parameter.
[[nodiscard]] std::string emit_scenario(const ScenarioConfig& s, const ScenarioSchema& schema);

}  // namespace branes::anatomy::config
