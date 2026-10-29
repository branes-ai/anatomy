// Unit tests for the data contracts' invariants (branes/anatomy/mobility/msg/): each
// valid() accepts the default or a hand-built good value and rejects one
// violation per invariant.

#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/daa.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/imu.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <vector>

using namespace branes::anatomy;

TEST_CASE("ImuSample: I1 saturation bits stay inside the mask", "[msg]") {
    ImuSample<double> s{};
    REQUIRE(valid(s));
    s.saturated = accel_z_saturated | gyro_x_saturated;
    REQUIRE(valid(s));
    s.saturated = 0x40;
    REQUIRE_FALSE(valid(s));
}

TEST_CASE("StateEstimate: I1 unit attitude, I2 symmetric non-negative covariance", "[msg]") {
    StateEstimate<double> x{};
    REQUIRE(valid(x));
    x.covariance[0 * kErrorStateDim + 1] = 0.1;
    REQUIRE_FALSE(valid(x));  // I2: asymmetric
    x.covariance[1 * kErrorStateDim + 0] = 0.1;
    REQUIRE(valid(x));
    x.covariance[2 * kErrorStateDim + 2] = -1.0;
    REQUIRE_FALSE(valid(x));  // I2: negative variance
    x.covariance[2 * kErrorStateDim + 2] = 1.0;
    x.attitude = Quat<double>{1.0, 1.0, 0.0, 0.0};
    REQUIRE_FALSE(valid(x));  // I1
}

TEST_CASE("Reference: I2 strictly increasing time, I3 non-decreasing progress", "[msg]") {
    Reference<double> r{};
    r.count = 3;
    for (std::uint32_t i = 0; i < 3; ++i) {
        r.samples[i].t = Timestamp{static_cast<std::int64_t>(i) * 10};
        r.samples[i].progress = static_cast<double>(i);
    }
    REQUIRE(valid(r));
    r.samples[2].t = r.samples[1].t;
    REQUIRE_FALSE(valid(r));  // I2
    r.samples[2].t = Timestamp{20};
    r.samples[2].progress = 0.5;
    REQUIRE_FALSE(valid(r));  // I3
    r.count = kMaxReferenceSamples + 1;
    REQUIRE_FALSE(valid(r));  // I1
}

// Test names stay ASCII: CTest passes them back to Catch2 as filters, and
// Windows mangles non-ASCII command-line characters (≥ became =).

TEST_CASE("ThrustRates: I1 thrust >= 0, I2 motors in [0, 1]", "[msg]") {
    ThrustRates<double> u{};
    REQUIRE(valid(u));
    u.thrust = -1.0;
    REQUIRE_FALSE(valid(u));
    u.thrust = 5.0;
    u.motors[3] = 1.2;
    REQUIRE_FALSE(valid(u));
}

TEST_CASE("ConflictSet and IntruderDetections: count bounds", "[msg]") {
    ConflictSet<double> c{};
    REQUIRE(valid(c));
    c.count = kMaxConflicts + 1;
    REQUIRE_FALSE(valid(c));
    IntruderDetections<double> d{};
    REQUIRE(valid(d));
    d.count = kMaxDetections + 1;
    REQUIRE_FALSE(valid(d));
}
