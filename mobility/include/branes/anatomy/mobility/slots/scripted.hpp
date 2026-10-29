#pragma once
/// @file scripted.hpp
/// @brief ScriptedHopController: an open-loop test controller for SITL tests.
///
/// Not a mission controller: it ignores the state and reference and commands a
/// fixed hop by time, so a SITL run moves the vehicle without any closed-loop
/// component. The `scripted-hop` baseline uses it for the SITL determinism
/// gate (racing-drone docs/plan/sitl/). The thrusts suit RP-1 (weight ≈ 9.47 N).
///
///   [0, 0.5) s   12 N, zero rates            (lift off and climb)
///   [0.5, 1) s   9.4 N, yaw rate 0.3 rad/s   (coast up while yawing)
///   ≥ 1 s        9.0 N, zero rates           (settle down)

#include <branes/anatomy/mobility/slots/neutral.hpp>

#include <cstdint>

namespace branes::anatomy {

template <typename T>
class ScriptedHopController {
public:
    [[nodiscard]] ThrustRates<T>
    step(const StateEstimate<T>& x, const Reference<T>&, const ConflictSet<T>&) const noexcept {
        ThrustRates<T> u{};
        u.t = x.t_valid;
        const std::int64_t t = x.t_valid.ns;
        if (t < 500'000'000) {
            u.thrust = T(12);
        } else if (t < 1'000'000'000) {
            u.thrust = T(9.4);
            u.body_rates[2] = T(0.3);
        } else {
            u.thrust = T(9.0);
        }
        return u;
    }
};

}  // namespace branes::anatomy
