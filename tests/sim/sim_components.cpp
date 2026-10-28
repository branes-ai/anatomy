// Unit and contract tests for the sim/ components: the vehicle mapping and
// randomization, the IMU model, the simulated FC, and the camera model.

#include <branes/anatomy/sim/camera_model.hpp>
#include <branes/anatomy/sim/fc.hpp>
#include <branes/anatomy/sim/imu.hpp>
#include <branes/anatomy/sim/vehicle.hpp>

#include "sim_fixtures.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <numbers>

using namespace branes::anatomy;
using namespace branes::anatomy::sim;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using sim_fixtures::rp1;

// ── vehicle ─────────────────────────────────────────────────────────

TEST_CASE("vehicle: reflex parameters map from the vehicle file", "[sim][vehicle]") {
    const auto v = rp1();
    const auto p = reflex_params_from(v, 0.1);
    REQUIRE(p);
    CHECK(p->mass == v.mass);
    CHECK(p->inertia[2] == v.inertia[2]);
    CHECK(p->thrust_coefficient == v.rotor.thrust_coefficient);
    CHECK(p->max_thrust == v.rotor.max_thrust);
    CHECK(p->motor_time_constant == v.rotor.time_constant);
    CHECK(p->drag_quadratic[1] == v.drag.quadratic[1]);
    CHECK(p->ground_z == 0.1);

    auto six = v;
    six.rotor_count = 6;
    CHECK_FALSE(reflex_params_from(six, 0.0));
    auto bad = v;
    bad.mass = -1;
    CHECK_FALSE(reflex_params_from(bad, 0.0));
}

TEST_CASE("vehicle: randomization draws stay within their widths and repeat per seed", "[sim][vehicle]") {
    config::Randomization r;
    r.mass = 0.1;
    r.inertia = 0.2;
    r.start_xy_m = 0.5;
    r.start_yaw_rad = 0.3;
    for (std::uint64_t seed = 1; seed < 200; ++seed) {
        SplitMix64 a(seed);
        SplitMix64 b(seed);
        const Draws d = draw(r, a);
        const Draws e = draw(r, b);
        CHECK(d.mass_scale == e.mass_scale);
        CHECK(std::abs(d.mass_scale - 1) <= 0.1);
        CHECK(std::abs(d.inertia_scale - 1) <= 0.2);
        CHECK(d.drag_scale == 1);  // zero width
        CHECK(std::abs(d.start_dx) <= 0.5);
        CHECK(std::abs(d.start_dyaw) <= 0.3);
    }
    // No randomization: the identity, and the parameters are unchanged.
    SplitMix64 rng(7);
    const Draws none = draw(config::Randomization{}, rng);
    const auto p = *reflex_params_from(rp1(), 0.0);
    const auto q = apply(p, none);
    CHECK(q.mass == p.mass);
    CHECK(q.inertia == p.inertia);
}

// ── IMU ─────────────────────────────────────────────────────────────

TEST_CASE("imu: white noise has sigma = density * sqrt(rate) and zero mean", "[sim][imu]") {
    auto p = sim_fixtures::rp1_ideal_imu().imu;
    p.gyro_noise_density = 1.7e-4;
    p.accel_noise_density = 2.0e-3;
    ImuModel imu(p, 42);
    constexpr int kN = 20000;
    double sum[2]{}, sum2[2]{};
    for (int k = 0; k < kN; ++k) {
        const auto s = imu.measure(Timestamp{k * 1'000'000LL}, {0, 0, 0}, {0, 0, 0});
        sum[0] += s.gyro[0];
        sum2[0] += s.gyro[0] * s.gyro[0];
        sum[1] += s.accel[2];
        sum2[1] += s.accel[2] * s.accel[2];
        (void)imu.pop_due(Timestamp{k * 1'000'000LL});
    }
    // Uniform noise has kurtosis 1.8, so the sample variance has relative
    // standard error √(0.8/N) and the sample σ half that; the mean has σ/√N.
    // Both bounds are five standard errors.
    const double sigma[2] = {p.gyro_noise_density * std::sqrt(p.rate_hz), p.accel_noise_density * std::sqrt(p.rate_hz)};
    for (int i = 0; i < 2; ++i) {
        const double mean = sum[i] / kN;
        const double sd = std::sqrt(sum2[i] / kN - mean * mean);
        INFO("axis " << i << ": sigma " << sd << " expected " << sigma[i]);
        CHECK(std::abs(mean) <= 5 * sigma[i] / std::sqrt(kN));
        CHECK(std::abs(sd / sigma[i] - 1) <= 5 * 0.5 * std::sqrt(0.8 / kN));
    }
}

TEST_CASE("imu: each axis clips at its range and sets its saturation bit", "[sim][imu]") {
    const auto p = sim_fixtures::rp1_ideal_imu().imu;
    ImuModel imu(p, 1);
    const auto s = imu.measure(Timestamp{0}, {40.0, -1.0, -50.0}, {0.0, 200.0, 9.8});
    CHECK(s.gyro[0] == p.gyro_range);
    CHECK(s.gyro[1] == -1.0);
    CHECK(s.gyro[2] == -p.gyro_range);
    CHECK(s.accel[1] == p.accel_range);
    CHECK(s.accel[2] == 9.8);
    CHECK(s.saturated == (gyro_x_saturated | gyro_z_saturated | accel_y_saturated));
    CHECK(valid(s));
}

TEST_CASE("imu: a sample is delivered latency after it was measured", "[sim][imu]") {
    auto p = sim_fixtures::rp1_ideal_imu().imu;
    p.latency_s = 0.0025;
    ImuModel imu(p, 1);
    (void)imu.measure(Timestamp{0}, {0.1, 0, 0}, {0, 0, 9.8});
    (void)imu.measure(Timestamp{1'000'000}, {0.2, 0, 0}, {0, 0, 9.8});
    REQUIRE(imu.next_delivery() == Timestamp{2'500'000});
    CHECK_FALSE(imu.pop_due(Timestamp{2'499'999}));
    const auto first = imu.pop_due(Timestamp{2'500'000});
    REQUIRE(first);
    CHECK(first->t == Timestamp{0});  // carries its measurement time
    CHECK(first->gyro[0] == 0.1);
    CHECK(imu.next_delivery() == Timestamp{3'500'000});
}

TEST_CASE("imu: turn-on bias, bias steps, and seeds", "[sim][imu]") {
    auto p = sim_fixtures::rp1_ideal_imu().imu;
    p.initial_gyro_bias_sigma = 0.01;
    ImuModel a(p, 5);
    ImuModel b(p, 5);
    ImuModel c(p, 6);
    CHECK(a.gyro_bias() == b.gyro_bias());
    CHECK(a.gyro_bias() != c.gyro_bias());
    for (const double g : a.gyro_bias()) {
        CHECK(std::abs(g) <= 0.01 * std::sqrt(3.0));  // uniform with σ = 0.01
    }
    const auto before = a.gyro_bias();
    a.add_bias({0.5, 0, 0}, {0, 0, 1});
    CHECK(a.gyro_bias()[0] == before[0] + 0.5);
    CHECK(a.accel_bias()[2] == 1.0);
    const auto s = a.measure(Timestamp{0}, {0, 0, 0}, {0, 0, 0});
    CHECK(s.gyro[0] == a.gyro_bias()[0]);  // no noise or walk: the sample is the bias
}

// ── simulated FC ────────────────────────────────────────────────────

namespace {

SimFc make_fc(config::SimParams s = {}) {
    return SimFc(*reflex_params_from(rp1(), 0.0), s);
}

}  // namespace

TEST_CASE("fc: without a command the FC commands zero rotor speed", "[sim][fc]") {
    SimFc fc = make_fc();
    const auto u = fc.step(Timestamp{0}, {0, 0, 0});
    CHECK(fc.failsafe());
    for (const double w : u.speeds) {
        CHECK(w == 0.0);
    }
}

TEST_CASE("fc: motors commands map to omega_max * sqrt(m) exactly", "[sim][fc]") {
    SimFc fc = make_fc();
    REQUIRE(fc.receive(sim_fixtures::motors(0.0, 0.25, 0.5, 1.0), Timestamp{0}));
    const auto u = fc.step(Timestamp{0}, {0, 0, 0});
    const double w_max = fc.max_rotor_speed();
    CHECK(u.speeds[0] == 0.0);
    CHECK(u.speeds[1] == w_max * std::sqrt(0.25));
    CHECK(u.speeds[2] == w_max * std::sqrt(0.5));
    CHECK(u.speeds[3] == w_max);
    CHECK_FALSE(fc.failsafe());
}

TEST_CASE("fc: invalid commands are rejected and keep the previous one", "[sim][fc]") {
    SimFc fc = make_fc();
    REQUIRE(fc.receive(sim_fixtures::motors(0.5, 0.5, 0.5, 0.5), Timestamp{0}));
    CHECK_FALSE(fc.receive(sim_fixtures::motors(0.5, 1.5, 0.5, 0.5), Timestamp{1}));
    CHECK_FALSE(fc.receive(sim_fixtures::thrust_rates(-1.0), Timestamp{2}));
    CHECK_FALSE(fc.receive(sim_fixtures::thrust_rates(5.0, std::numeric_limits<double>::quiet_NaN()), Timestamp{3}));
    CHECK(fc.rejected() == 3);
    const auto u = fc.step(Timestamp{4}, {0, 0, 0});
    CHECK(u.speeds[1] == fc.max_rotor_speed() * std::sqrt(0.5));
}

TEST_CASE("fc: a command older than the timeout puts the FC in failsafe", "[sim][fc]") {
    config::SimParams s;
    s.command_timeout_s = 0.1;
    SimFc fc = make_fc(s);
    REQUIRE(fc.receive(sim_fixtures::motors(0.5, 0.5, 0.5, 0.5), Timestamp{0}));
    CHECK(fc.step(Timestamp{100'000'000}, {0, 0, 0}).speeds[0] > 0);  // exactly the timeout: still fresh
    CHECK(fc.step(Timestamp{100'000'001}, {0, 0, 0}).speeds[0] == 0);
    CHECK(fc.failsafe());
}

TEST_CASE("fc: a thrust command at zero rates splits the thrust evenly", "[sim][fc]") {
    SimFc fc = make_fc();
    const auto p = *reflex_params_from(rp1(), 0.0);
    const double thrust = 10.0;
    REQUIRE(fc.receive(sim_fixtures::thrust_rates(thrust), Timestamp{0}));
    const auto u = fc.step(Timestamp{0}, {0, 0, 0});
    for (const double w : u.speeds) {
        CHECK_THAT(p.thrust_coefficient * w * w, WithinRel(thrust / 4, 1e-12));
    }
}

TEST_CASE("fc: the rate loop opposes a measured body rate", "[sim][fc]") {
    SimFc fc = make_fc();
    REQUIRE(fc.receive(sim_fixtures::thrust_rates(10.0), Timestamp{0}));
    // A positive yaw rate on the gyro: the FC must command a negative yaw
    // torque, which in Quad X speeds up the rotors that spin the other way.
    rx::model::RotorCommand<double> u{};
    for (int k = 0; k < 20; ++k) {
        u = fc.step(Timestamp{k * 1'000'000LL}, {0, 0, 1.0});
    }
    const auto p = *reflex_params_from(rp1(), 0.0);
    const rx::model::QuadrotorModel<double> m(p);
    double yaw = 0;
    for (std::size_t i = 0; i < 4; ++i) {
        yaw += -m.rotor_spin(i) * p.torque_coefficient * p.thrust_coefficient * u.speeds[i] * u.speeds[i];
    }
    CHECK(yaw < 0);
}

// ── camera model ────────────────────────────────────────────────────

namespace {

/// The vehicle level at `p`, yawed by `yaw`, at rest.
TruthState<double> pose(double x, double y, double z, double yaw) {
    TruthState<double> s{};
    s.position = {x, y, z};
    s.attitude = Quat<double>{std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
    return s;
}

}  // namespace

TEST_CASE("camera: the RP-1 camera looks along body +x", "[sim][camera]") {
    const auto cam = rp1().camera;
    const auto c = to_camera(cam, pose(0, 0, 0, 0), {5.0 + cam.translation_body[0], 0.0, cam.translation_body[2]});
    CHECK_THAT(c[0], WithinAbs(0.0, 1e-12));
    CHECK_THAT(c[1], WithinAbs(0.0, 1e-12));
    CHECK_THAT(c[2], WithinAbs(5.0, 1e-12));
}
