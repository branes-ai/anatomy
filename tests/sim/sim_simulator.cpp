// BasicSimulator and the plan's acceptance criteria (racing-drone docs/plan/sitl/):
//  - event order and determinism;
//  - scripted motor commands reproduce reflex QuadrotorModel bit for bit, and
//    the analytic cases (free fall, hover, ground contact) hold to reflex's
//    own tolerances;
//  - a thrust-and-rates script through the simulated FC's rate loop takes off
//    and settles the body rates, and the command timeout cuts the rotors;
//  - every fault type and the randomization act as specified.

#include <branes/anatomy/sim/basic_simulator.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace branes::anatomy;
using namespace branes::anatomy::sim;
using Catch::Matchers::WithinAbs;
using sim_fixtures::kMark;
using sim_fixtures::MarkOracle;
using sim_fixtures::motors;
using sim_fixtures::run;
using sim_fixtures::scenario;
using sim_fixtures::start;
using sim_fixtures::TestSimulator;
using sim_fixtures::thrust_rates;

namespace {

TestSimulator make(const config::VehicleConfig& v, const config::ScenarioConfig& s, const SimOptions& o = {}) {
    auto sim = TestSimulator::create(v, start(), s, 100.0, MarkOracle{}, o);
    REQUIRE(sim);
    return std::move(sim).value();
}

std::vector<TruthState<double>> truths(const std::vector<SimEvent>& events) {
    std::vector<TruthState<double>> out;
    for (const auto& e : events) {
        if (e.channel == log::Channel::truth) {
            const auto x = codec::decode_truth(e.payload);
            REQUIRE(x);
            out.push_back(*x);
        }
    }
    return out;
}

/// An airborne state at height z, level, with every rotor at speed w.
PlantState airborne(double z, double w) {
    PlantState x{};
    x.position = rx::vec3(10.0, 4.0, z);
    x.rotor_speeds = {w, w, w, w};
    return x;
}

double weight(const config::VehicleConfig& v) {
    return v.mass * 9.80665;
}

}  // namespace

// ── event stream ────────────────────────────────────────────────────

TEST_CASE("simulator: events come out in time order, in the documented order at equal times", "[sim][simulator]") {
    const auto events = [] {
        TestSimulator sim = make(sim_fixtures::rp1(), scenario(0.3));
        return run(sim, [](Timestamp) { return thrust_rates(5.0); });
    }();
    const std::map<log::Channel, int> rank = {{log::Channel::sim_config, 0},
                                              {log::Channel::imu, 1},
                                              {log::Channel::camera, 2},
                                              {kMark, 3},  // the oracle event
                                              {log::Channel::truth, 4},
                                              {log::Channel::tick, 5}};
    REQUIRE(events.front().channel == log::Channel::sim_config);
    std::map<log::Channel, int> count;
    for (std::size_t i = 1; i < events.size(); ++i) {
        const auto& a = events[i - 1];
        const auto& b = events[i];
        INFO("event " << i);
        REQUIRE(a.t <= b.t);
        if (a.t == b.t && a.channel != log::Channel::imu) {
            REQUIRE(rank.at(a.channel) <= rank.at(b.channel));
        }
        ++count[b.channel];
    }
    CHECK(count[log::Channel::truth] == 300);  // 1 kHz for 0.3 s
    CHECK(count[log::Channel::tick] == 30);    // 100 Hz
    CHECK(count[log::Channel::camera] == 28);  // 90 Hz (11 111 111 ns): k = 0 … 27, the last at 299.99997 ms
    CHECK(count[kMark] == count[log::Channel::camera]);
    CHECK(count[log::Channel::imu] == 299);  // 1 kHz, the last one still in flight (1 ms latency)
}

TEST_CASE("simulator: the same seed gives the same events; another seed changes the noise", "[sim][simulator]") {
    auto s = scenario(0.2);
    s.randomization.mass = 0.1;
    s.faults.push_back(config::FaultEvent{config::FaultType::camera_corrupt, 0, 1e6, {}, {}, 0.5, 0, 0, 1});
    const auto script = [](Timestamp) { return thrust_rates(11.0, 0.1); };
    TestSimulator a = make(sim_fixtures::rp1(), s);
    TestSimulator b = make(sim_fixtures::rp1(), s);
    const auto ea = run(a, script);
    const auto eb = run(b, script);
    REQUIRE(ea.size() == eb.size());
    for (std::size_t i = 0; i < ea.size(); ++i) {
        REQUIRE(ea[i].channel == eb[i].channel);
        REQUIRE(ea[i].t == eb[i].t);
        REQUIRE(ea[i].payload == eb[i].payload);
    }
    s.seed += 1;
    TestSimulator c = make(sim_fixtures::rp1(), s);
    const auto ec = run(c, script);
    bool imu_differs = false;
    for (std::size_t i = 0; i < ec.size() && i < ea.size(); ++i) {
        imu_differs = imu_differs || (ea[i].channel == log::Channel::imu && ea[i].payload != ec[i].payload);
    }
    CHECK(imu_differs);
    CHECK(c.draws().mass_scale != a.draws().mass_scale);
}

TEST_CASE("simulator: create rejects out-of-range faults from a programmatic scenario", "[sim][simulator]") {
    auto s = scenario(1.0);
    config::FaultEvent f;
    f.type = config::FaultType::motor_scale;
    f.rotor = 7;  // only rotors 0-3 exist
    s.faults.push_back(f);
    CHECK_FALSE(TestSimulator::create(sim_fixtures::rp1(), start(), s, 100.0, MarkOracle{}));
    s.faults[0].rotor = 1;
    s.faults[0].scale = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(TestSimulator::create(sim_fixtures::rp1(), start(), s, 100.0, MarkOracle{}));
    s.faults[0].scale = 0.5;
    CHECK(TestSimulator::create(sim_fixtures::rp1(), start(), s, 100.0, MarkOracle{}));
}

TEST_CASE("simulator: create rejects an invalid initial state", "[sim][simulator]") {
    SimOptions o;
    o.initial_state = airborne(2.0, 0.0);
    CHECK(TestSimulator::create(sim_fixtures::rp1(), start(), scenario(1.0), 100.0, MarkOracle{}, o));
    o.initial_state->position = rx::vec3(10.0, 4.0, -1.0);  // below the ground (the pad at 0.1 m)
    CHECK_FALSE(TestSimulator::create(sim_fixtures::rp1(), start(), scenario(1.0), 100.0, MarkOracle{}, o));
    o.initial_state = airborne(2.0, 1e6);  // a rotor beyond its maximum speed
    CHECK_FALSE(TestSimulator::create(sim_fixtures::rp1(), start(), scenario(1.0), 100.0, MarkOracle{}, o));
    o.initial_state = airborne(2.0, 0.0);
    o.initial_state->attitude = rx::Quat<double>{2.0, 0.0, 0.0, 0.0};  // not unit
    CHECK_FALSE(TestSimulator::create(sim_fixtures::rp1(), start(), scenario(1.0), 100.0, MarkOracle{}, o));
}

TEST_CASE("simulator: the sim_config record holds the drawn randomization", "[sim][simulator]") {
    auto s = scenario(0.01);
    s.randomization.mass = 0.2;
    TestSimulator sim = make(sim_fixtures::rp1(), s);
    const auto first = sim.next();
    REQUIRE(first);
    REQUIRE(first->channel == log::Channel::sim_config);
    const std::string text(reinterpret_cast<const char*>(first->payload.data()), first->payload.size());
    CHECK(text == to_yaml(sim.draws()));
    CHECK(sim.model().params().mass == sim_fixtures::rp1().mass * sim.draws().mass_scale);
}

// ── acceptance 2: scripted trajectories ─────────────────────────────

TEST_CASE("acceptance: scripted motor commands reproduce QuadrotorModel bit for bit", "[sim][simulator][acceptance]") {
    const auto v = sim_fixtures::rp1();
    const auto s = scenario(2.0);
    // Per-rotor thrust fractions that change every 0.25 s, enough to lift off,
    // roll, pitch, and yaw.
    const auto fractions = [](Timestamp t) {
        const int k = static_cast<int>(t.ns / 250'000'000);
        const double base = 0.3 + 0.02 * (k % 3);
        return std::array<double, 4>{base + 0.01 * (k % 2), base, base - 0.005 * (k % 4), base + 0.003};
    };
    TestSimulator sim = make(v, s);
    const auto events = run(sim, [&](Timestamp t) {
        const auto m = fractions(t);
        return motors(m[0], m[1], m[2], m[3]);
    });
    const auto got = truths(events);

    // The reference: the same model, stepped directly. The tick at t (every
    // 10 ms) sets the command for the plant steps from t on.
    const auto p = *reflex_params_from(v, start().position[2]);
    const rx::model::QuadrotorModel<double> m(p);
    PlantState x{};
    x.position = rx::vec3(start().position[0], start().position[1], start().position[2]);
    x.attitude = rx::from_rotation_vector(rx::vec3(0.0, 0.0, start().yaw));
    x.on_ground = true;
    rx::model::RotorCommand<double> u{};
    bool lifted = false;
    for (std::size_t k = 0; k < got.size(); ++k) {
        const Timestamp t{static_cast<std::int64_t>(k) * 1'000'000};
        INFO("step " << k);
        REQUIRE(got[k].t == t);
        REQUIRE(got[k].position[0] == x.position(0));
        REQUIRE(got[k].position[1] == x.position(1));
        REQUIRE(got[k].position[2] == x.position(2));
        REQUIRE(got[k].velocity[2] == x.velocity(2));
        REQUIRE(got[k].attitude.w == x.attitude.w);
        REQUIRE(got[k].attitude.z == x.attitude.z);
        REQUIRE(got[k].body_rates[0] == x.body_rates(0));
        REQUIRE(got[k].rotor_speeds == x.rotor_speeds);
        REQUIRE(got[k].on_ground == x.on_ground);
        lifted = lifted || !x.on_ground;
        if (t.ns % 10'000'000 == 0) {
            const auto f = fractions(t);
            for (std::size_t i = 0; i < 4; ++i) {
                u.speeds[i] = m.max_rotor_speed() * std::sqrt(f[i]);
            }
        }
        x = m.step(x, u, 1e-3);
    }
    CHECK(got.size() == 2000);
    CHECK(lifted);
}

TEST_CASE("acceptance: free fall follows z0 - g t^2 / 2", "[sim][simulator][acceptance]") {
    auto v = sim_fixtures::rp1();
    v.drag.linear = {0, 0, 0};
    v.drag.quadratic = {0, 0, 0};
    SimOptions o;
    o.initial_state = airborne(20.0, 0.0);
    TestSimulator sim = make(v, scenario(0.5), o);
    const auto got = truths(run(sim, [](Timestamp) { return std::nullopt; }));  // no command: rotors off
    const double g = 9.80665;
    const auto& last = got.back();
    const double t = static_cast<double>(last.t.ns) * 1e-9;
    CHECK_THAT(last.position[2], WithinAbs(20.0 - 0.5 * g * t * t, 1e-9));  // reflex's tolerance in double
    CHECK_THAT(last.velocity[2], WithinAbs(-g * t, 1e-9));
}

TEST_CASE("acceptance: rotors at the hover speed hold the vehicle in place", "[sim][simulator][acceptance]") {
    const auto v = sim_fixtures::rp1();
    const auto p = *reflex_params_from(v, start().position[2]);
    const rx::model::QuadrotorModel<double> m(p);
    const double hover = weight(v) / (4 * v.rotor.max_thrust);  // thrust fraction per rotor
    SimOptions o;
    o.initial_state = airborne(2.0, m.hover_rotor_speed());
    TestSimulator sim = make(v, scenario(1.0), o);
    const auto got = truths(run(sim, [&](Timestamp) { return motors(hover, hover, hover, hover); }));
    const auto& last = got.back();
    CHECK_THAT(last.position[2], WithinAbs(2.0, 1e-9));
    CHECK_THAT(last.velocity[2], WithinAbs(0.0, 1e-9));
    CHECK_THAT(last.body_rates[2], WithinAbs(0.0, 1e-9));
}

TEST_CASE("acceptance: below hover thrust the vehicle stays on the pad", "[sim][simulator][acceptance]") {
    const auto v = sim_fixtures::rp1();
    const double m = 0.9 * weight(v) / (4 * v.rotor.max_thrust);
    TestSimulator sim = make(v, scenario(1.0));
    const auto got = truths(run(sim, [&](Timestamp) { return motors(m, m, m, m); }));
    for (const auto& x : got) {
        REQUIRE(x.on_ground);
        REQUIRE(x.position == got.front().position);
    }
    CHECK(got.back().rotor_speeds[0] > 0);
}

// ── acceptance 3: closed loop through the rate loop ─────────────────

TEST_CASE("acceptance: a thrust-and-rates script takes off and settles the body rates",
          "[sim][simulator][acceptance]") {
    const auto v = sim_fixtures::rp1();
    const double w = weight(v);
    TestSimulator sim = make(v, scenario(2.0));
    const auto got = truths(run(sim, [&](Timestamp t) { return thrust_rates(t.ns < 1'000'000'000 ? 1.3 * w : w); }));
    const double pad = start().position[2];
    double worst = 0;
    for (const auto& x : got) {
        if (x.t.ns >= 1'500'000'000) {
            for (const double r : x.body_rates) {
                worst = std::max(worst, std::abs(r));
            }
        }
    }
    INFO("worst body rate over [1.5, 2] s: " << worst << " rad/s; final height " << got.back().position[2]);
    CHECK_FALSE(got.back().on_ground);
    CHECK(got.back().position[2] > pad + 0.2);
    CHECK(worst < 0.05);
}

TEST_CASE("acceptance: the command timeout cuts the rotors one FC step after it expires",
          "[sim][simulator][acceptance]") {
    const auto v = sim_fixtures::rp1();
    TestSimulator sim = make(v, scenario(0.8));
    // Commands on every tick up to 0.49 s, then silence. The FC's last
    // command is from 0.49 s; with the 0.1 s timeout the step from 0.59 s
    // still uses it, and the step from 0.591 s is the first in failsafe.
    std::map<std::int64_t, double> rotor_at;  // truth time → rotor command of the step that led there
    while (auto e = sim.next()) {
        if (e->channel == log::Channel::tick && e->t.ns < 500'000'000) {
            sim.command(thrust_rates(weight(v)));
        }
        if (e->channel == log::Channel::truth) {
            rotor_at[e->t.ns] = sim.rotor_command().speeds[0];
        }
    }
    CHECK(rotor_at.at(591'000'000) > 0);   // the step from 0.590 s
    CHECK(rotor_at.at(592'000'000) == 0);  // the step from 0.591 s
    CHECK(sim.fc().failsafe());
}

// ── faults ──────────────────────────────────────────────────────────

namespace {

config::FaultEvent fault(config::FaultType type, double t0, double t1) {
    config::FaultEvent f;
    f.type = type;
    f.t_s = t0;
    f.until_s = t1;
    return f;
}

}  // namespace

TEST_CASE("faults: link loss drops commands, and a long one ends in failsafe", "[sim][faults]") {
    auto s = scenario(1.0);
    s.faults.push_back(fault(config::FaultType::link_loss, 0.3, 0.6));
    TestSimulator sim = make(sim_fixtures::rp1(), s);
    bool failsafe_in_loss = false;
    while (auto e = sim.next()) {
        if (e->channel == log::Channel::tick) {
            sim.command(thrust_rates(5.0));
        }
        if (e->channel == log::Channel::truth && e->t.ns > 450'000'000 && e->t.ns < 600'000'000) {
            failsafe_in_loss = failsafe_in_loss || sim.fc().failsafe();
        }
    }
    CHECK(sim.dropped_commands() == 30);  // ticks at 0.30 … 0.59 s
    CHECK(failsafe_in_loss);
    CHECK_FALSE(sim.fc().failsafe());  // commands resumed
}

TEST_CASE("faults: a command delay postpones every command by the delay", "[sim][faults]") {
    const auto first_spin = [](double delay) {
        auto s = scenario(0.2);
        if (delay > 0) {
            auto f = fault(config::FaultType::command_delay, 0, 1e6);
            f.delay_s = delay;
            s.faults.push_back(f);
        }
        TestSimulator sim = make(sim_fixtures::rp1(), s);
        while (auto e = sim.next()) {
            if (e->channel == log::Channel::tick && e->t.ns >= 50'000'000) {
                sim.command(thrust_rates(5.0));
            }
            if (e->channel == log::Channel::truth && sim.rotor_command().speeds[0] > 0) {
                return e->t.ns;
            }
        }
        return std::int64_t{-1};
    };
    const std::int64_t base = first_spin(0);
    REQUIRE(base == 51'000'000);  // the tick at 50 ms drives the step from 50 ms
    CHECK(first_spin(0.02) == base + 20'000'000);
}

TEST_CASE("faults: motor_scale delivers a thrust fraction on one rotor", "[sim][faults]") {
    auto s = scenario(0.1);
    auto f = fault(config::FaultType::motor_scale, 0.05, 1e6);
    f.rotor = 2;
    f.scale = 0.49;
    s.faults.push_back(f);
    TestSimulator sim = make(sim_fixtures::rp1(), s);
    std::array<double, 4> before{}, after{};
    while (auto e = sim.next()) {
        if (e->channel == log::Channel::tick) {
            sim.command(motors(0.4, 0.4, 0.4, 0.4));
        }
        if (e->channel == log::Channel::truth && e->t.ns == 40'000'000) {
            before = sim.rotor_command().speeds;
        }
        if (e->channel == log::Channel::truth && e->t.ns == 80'000'000) {
            after = sim.rotor_command().speeds;
        }
    }
    CHECK(before[2] == before[0]);
    CHECK(after[2] == after[0] * std::sqrt(0.49));
    CHECK(after[1] == after[0]);
}

TEST_CASE("faults: a camera blackout delivers no frames; corruption marks a fraction", "[sim][faults]") {
    auto s = scenario(4.0);
    s.faults.push_back(fault(config::FaultType::camera_blackout, 1.0, 2.0));
    auto c = fault(config::FaultType::camera_corrupt, 2.0, 1e6);
    c.probability = 0.3;
    s.faults.push_back(c);
    TestSimulator sim = make(sim_fixtures::rp1(), s);
    int in_blackout = 0, after = 0, corrupted = 0, before_corrupt = 0;
    while (auto e = sim.next()) {
        if (e->channel != log::Channel::camera) {
            continue;
        }
        const auto f = codec::decode_frame(e->payload);
        REQUIRE(f);
        if (e->t.ns >= 1'000'000'000 && e->t.ns < 2'000'000'000) {
            ++in_blackout;
        } else if (e->t.ns >= 2'000'000'000) {
            ++after;
            corrupted += f->header.corrupted ? 1 : 0;
        } else {
            before_corrupt += f->header.corrupted ? 1 : 0;
        }
    }
    CHECK(in_blackout == 0);
    CHECK(before_corrupt == 0);
    // Binomial(n, 0.3): five standard deviations either side.
    const double n = after;
    INFO(corrupted << " of " << after << " corrupted");
    CHECK(std::abs(corrupted - 0.3 * n) <= 5 * std::sqrt(n * 0.3 * 0.7));
}

TEST_CASE("faults: an IMU bias step shifts the biases once, at its time", "[sim][faults]") {
    auto s = scenario(0.3);
    auto f = fault(config::FaultType::imu_bias_step, 0.1, 1e6);
    f.gyro = {0.2, 0, 0};
    s.faults.push_back(f);
    TestSimulator sim = make(sim_fixtures::rp1_ideal_imu(), s);
    double at_50 = 0, at_150 = 0, at_250 = 0;
    while (auto e = sim.next()) {
        if (e->channel != log::Channel::imu) {
            continue;
        }
        const auto x = codec::decode_imu(e->payload);
        REQUIRE(x);
        if (x->t.ns == 50'000'000) {
            at_50 = x->gyro[0];
        } else if (x->t.ns == 150'000'000) {
            at_150 = x->gyro[0];
        } else if (x->t.ns == 250'000'000) {
            at_250 = x->gyro[0];
        }
    }
    CHECK(at_50 == 0.0);  // ideal IMU, vehicle at rest
    CHECK(at_150 == 0.2);
    CHECK(at_250 == 0.2);
}
