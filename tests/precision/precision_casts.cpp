// The harness-boundary casts in every sweep type (docs/plan/precision-sweep/,
// acceptance criterion 2):
//  - a double converts to T with round to nearest: no representable neighbor
//    of the result is closer to the value (checked against the actual
//    neighbors, so no tolerance is chosen). Posits never round a nonzero value
//    to zero (the posit standard): below minpos the result is ±minpos, so zero
//    is not a candidate for them;
//  - each message cast is that scalar conversion, field by field, with the
//    non-scalar fields copied; for T = double it is the identity.

#include <branes/anatomy/msg/control.hpp>
#include <branes/anatomy/msg/estimate.hpp>
#include <branes/anatomy/msg/sensors.hpp>
#include <branes/anatomy/msg/truth.hpp>
#include <branes/anatomy/sim/sweep_types.hpp>
#include <branes/anatomy/util/splitmix64.hpp>

#include "precision_fixtures.hpp"

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

using namespace branes::anatomy;
using precision_fixtures::neighbors;

namespace {

template <typename T>
inline constexpr bool is_posit = std::is_same_v<T, sim::posit32> || std::is_same_v<T, sim::posit16>;

/// Values at the stack's magnitudes (the assessment's Table 2 and RP-1's
/// parameters), then log-uniform values over [1e-8, 1e4], both signs.
std::vector<double> representative_values() {
    std::vector<double> v{
        0.5, 4.0, 3.2, 32.0, 64.0, 76.0, 8.9e-7, 0.016, 12.0, 0.966, 9.80665, 1e-10, 0.7071067811865476};
    SplitMix64 rng(30);
    for (int i = 0; i < 2000; ++i) {
        v.push_back(std::pow(10.0, -8.0 + 12.0 * rng.uniform01()));
    }
    const std::size_t n = v.size();
    for (std::size_t i = 0; i < n; ++i) {
        v.push_back(-v[i]);
    }
    return v;
}

/// A value in [-50, 50) from the stream, for message fields.
double draw(SplitMix64& rng) {
    return rng.symmetric(50.0);
}

// Each message's scalar fields, in a fixed order, for filling and comparing.
template <class A, class F>
void each_of(A& a, F& f) {
    for (auto& x : a) {
        f(x);
    }
}
template <class Q, class F>
void each_of_quat(Q& q, F& f) {
    f(q.w);
    f(q.x);
    f(q.y);
    f(q.z);
}
template <class M, class F>
void each_imu(M& s, F&& f) {
    each_of(s.gyro, f);
    each_of(s.accel, f);
    f(s.temperature);
}
template <class M, class F>
void each_truth(M& s, F&& f) {
    each_of(s.position, f);
    each_of(s.velocity, f);
    each_of_quat(s.attitude, f);
    each_of(s.body_rates, f);
    each_of(s.rotor_speeds, f);
}
template <class M, class F>
void each_estimate(M& s, F&& f) {
    each_of(s.position, f);
    each_of(s.velocity, f);
    each_of_quat(s.attitude, f);
    each_of(s.body_rates, f);
    each_of(s.accel_bias, f);
    each_of(s.gyro_bias, f);
    each_of(s.covariance, f);
}
template <class M, class F>
void each_reference(M& r, F&& f) {
    for (auto& s : r.samples) {
        each_of(s.position, f);
        each_of(s.velocity, f);
        each_of(s.acceleration, f);
        f(s.yaw);
        f(s.progress);
    }
}
template <class M, class F>
void each_command(M& u, F&& f) {
    f(u.thrust);
    each_of(u.body_rates, f);
    each_of(u.motors, f);
}

/// Fill every scalar field of a double message with a distinct value.
template <class M, class Each>
M filled(Each each) {
    M m{};
    SplitMix64 rng(1700);
    each(m, [&](double& x) { x = draw(rng); });
    return m;
}

/// The scalar fields of a message as doubles.
template <class M, class Each>
std::vector<double> fields(const M& m, Each each) {
    std::vector<double> out;
    each(m, [&](const auto& x) { out.push_back(double(x)); });
    return out;
}

/// Pre: the message's scalar fields. Post: each field of cast<T>(m) is T(field).
template <typename T, class M, class Each>
void check_fields(const M& m, Each each) {
    const auto c = cast<T>(m);
    const std::vector<double> in = fields(m, each);
    const std::vector<double> out = fields(c, each);
    REQUIRE(in.size() == out.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        INFO("field " << i << " = " << in[i]);
        CHECK(out[i] == double(T(in[i])));
    }
    CHECK(cast<double>(cast<double>(m)) == m);  // identity for double
}

}  // namespace

TEMPLATE_LIST_TEST_CASE("cast: double to T rounds to nearest", "[precision][cast]", sim::SweepTypes) {
    using T = TestType;
    const double largest = double(std::numeric_limits<T>::max());
    for (const double v : representative_values()) {
        if (!(std::fabs(v) <= largest)) {
            continue;  // outside T's range; there is no nearest value to round to
        }
        const T t = T(v);
        const double r = double(t);
        const auto [below, above] = neighbors(t);
        INFO("v = " << v << ", T(v) = " << r << ", neighbors " << below << ", " << above);
        // No neighbor is closer. Zero is no candidate for a nonzero value in a
        // posit; chosen at compile time (MSVC C4127 rejects a constant `if`).
        const auto no_closer = [&](double neighbor) {
            if constexpr (is_posit<T>) {
                if (v != 0 && neighbor == 0) {
                    return true;
                }
            }
            return std::fabs(v - r) <= std::fabs(v - neighbor);
        };
        CHECK(no_closer(below));
        CHECK(no_closer(above));
        if constexpr (is_posit<T>) {
            if (v != 0) {
                CHECK(r != 0);  // posits: a nonzero value stays nonzero
            }
        }
    }
    CHECK(double(T(0.0)) == 0.0);
}

TEMPLATE_LIST_TEST_CASE("cast: each message cast is the scalar conversion of each field",
                        "[precision][cast]",
                        sim::SweepTypes) {
    using T = TestType;
    const auto imu = [](auto& m, auto&& f) { each_imu(m, f); };
    const auto truth = [](auto& m, auto&& f) { each_truth(m, f); };
    const auto estimate = [](auto& m, auto&& f) { each_estimate(m, f); };
    const auto reference = [](auto& m, auto&& f) { each_reference(m, f); };
    const auto command = [](auto& m, auto&& f) { each_command(m, f); };

    auto s = filled<ImuSample<double>>(imu);
    s.t = Timestamp{11};
    s.saturated = gyro_y_saturated | accel_z_saturated;
    check_fields<T>(s, imu);
    CHECK(cast<T>(s).t == s.t);
    CHECK(cast<T>(s).saturated == s.saturated);

    auto x = filled<TruthState<double>>(truth);
    x.t = Timestamp{12};
    x.on_ground = true;
    check_fields<T>(x, truth);
    CHECK(cast<T>(x).t == x.t);
    CHECK(cast<T>(x).on_ground);

    auto e = filled<StateEstimate<double>>(estimate);
    e.t_valid = Timestamp{13};
    check_fields<T>(e, estimate);
    CHECK(cast<T>(e).t_valid == e.t_valid);

    auto r = filled<Reference<double>>(reference);
    r.count = 3;
    for (std::uint32_t i = 0; i < kMaxReferenceSamples; ++i) {
        r.samples[i].t = Timestamp{100 + std::int64_t(i)};
    }
    check_fields<T>(r, reference);
    CHECK(cast<T>(r).count == r.count);
    for (std::uint32_t i = 0; i < kMaxReferenceSamples; ++i) {
        CHECK(cast<T>(r).samples[i].t == r.samples[i].t);
    }

    auto u = filled<ThrustRates<double>>(command);
    u.t = Timestamp{14};
    u.kind = CommandKind::motors;
    check_fields<T>(u, command);
    CHECK(cast<T>(u).t == u.t);
    CHECK(cast<T>(u).kind == u.kind);
}
