// Shared fixtures for the sim/ tests: the test vehicle (a copy of racing-drone's
// RP-1), a start pose, a test oracle, and a quiet scenario (no randomization,
// no faults) to edit per test.
#pragma once

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <functional>
#include <optional>
#include <vector>

namespace sim_fixtures {

namespace anatomy = branes::anatomy;

inline const std::filesystem::path src = ANATOMY_SOURCE_DIR;

inline anatomy::config::VehicleConfig rp1() {
    auto v = anatomy::config::load_vehicle(src / "tests/fixtures/vehicle.yaml");
    REQUIRE(v);
    return *v;
}

/// RP-1 with no sensor noise, bias, or latency: measurements equal the truth.
inline anatomy::config::VehicleConfig rp1_ideal_imu() {
    auto v = rp1();
    v.imu.gyro_noise_density = 0;
    v.imu.accel_noise_density = 0;
    v.imu.gyro_bias_random_walk = 0;
    v.imu.accel_bias_random_walk = 0;
    v.imu.initial_gyro_bias_sigma = 0;
    v.imu.initial_accel_bias_sigma = 0;
    v.imu.latency_s = 0;
    return v;
}

/// The vehicle's start pose: on the pad, 0.1 m up (racing-drone's course start).
inline anatomy::StartPose<double> start() {
    return anatomy::StartPose<double>{{3.0, 4.0, 0.1}, 0.0};
}

/// A test mission's channel, and an oracle that marks every frame on it: the
/// simulator's oracle events without any mission's perception.
inline constexpr anatomy::log::Channel kMark = static_cast<anatomy::log::Channel>(5);

struct MarkOracle {
    [[nodiscard]] std::optional<anatomy::sim::SimEvent> observe(const anatomy::TruthState<double>&,
                                                                anatomy::Timestamp t) const {
        return anatomy::sim::SimEvent{kMark, t, {}};
    }
};
static_assert(anatomy::sim::OracleSensor<MarkOracle>);

/// The simulator of the tests.
using TestSimulator = anatomy::sim::BasicSimulator<MarkOracle>;

inline anatomy::config::ScenarioConfig scenario(double duration_s, std::uint64_t seed = 20261026) {
    anatomy::config::ScenarioConfig s;
    s.name = "test";
    s.duration_s = duration_s;
    s.seed = seed;
    return s;
}

/// A command for tick time t.
using Script = std::function<std::optional<anatomy::ThrustRates<double>>(anatomy::Timestamp)>;

/// Run `sim` to the end, answering every tick with `script` (nullopt: no
/// command), and return every event.
inline std::vector<anatomy::sim::SimEvent> run(TestSimulator& sim, const Script& script) {
    std::vector<anatomy::sim::SimEvent> out;
    while (auto e = sim.next()) {
        if (e->channel == anatomy::log::Channel::tick) {
            if (const auto u = script(e->t)) {
                sim.command(*u);
            }
        }
        out.push_back(std::move(*e));
    }
    return out;
}

inline anatomy::ThrustRates<double> motors(double m0, double m1, double m2, double m3) {
    anatomy::ThrustRates<double> u{};
    u.kind = anatomy::CommandKind::motors;
    u.motors = {m0, m1, m2, m3};
    return u;
}

inline anatomy::ThrustRates<double> thrust_rates(double thrust, double p = 0, double q = 0, double r = 0) {
    anatomy::ThrustRates<double> u{};
    u.thrust = thrust;
    u.body_rates = {p, q, r};
    return u;
}

}  // namespace sim_fixtures
