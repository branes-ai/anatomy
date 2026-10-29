#pragma once
/// @file reflex_params.hpp
/// @brief reflex QuadrotorParams for a vehicle file.
///
/// The one mapping from a VehicleConfig onto reflex's QuadrotorParams, shared
/// by the SITL plant (sim/vehicle.hpp) and the controllers built from the
/// vehicle file (CascadedPid). The vehicle file has no battery model yet, so
/// the reflex defaults (a large battery, no sag) stay.

#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>

namespace branes::anatomy::glue {

namespace rx = branes::reflex;

/// "; <name> = <value> is <why> in this scalar type" if the nonzero vehicle
/// value `x` converts to zero, a subnormal, or a non-finite value in T; else
/// empty. It names the parameter when reflex's validation rejects the set.
template <typename T>
[[nodiscard]] std::string unrepresentable(std::string_view name, double x) {
    if (x == 0) {
        return {};
    }
    const double back = double(T(x));
    const char* why = nullptr;
    if (!std::isfinite(back)) {
        why = "not finite";
    } else if (back == 0) {
        why = "zero";
    } else if (std::fabs(back) < double(std::numeric_limits<T>::min())) {
        why = "subnormal";
    }
    if (why == nullptr) {
        return {};
    }
    char buf[32];
    const auto [end, ec] = std::to_chars(buf, buf + sizeof buf, x);
    const std::string value = ec == std::errc{} ? std::string(buf, end) : std::string("?");
    return "; " + std::string(name) + " = " + value + " is " + why + " in this scalar type";
}

/// reflex QuadrotorParams for `v` in scalar type T, with the ground at
/// `ground_z` [m]. Fails unless the result passes quadrotor_params_valid.
template <typename T = double>
[[nodiscard]] result<rx::model::QuadrotorParams<T>, std::string> reflex_params_from(const config::VehicleConfig& v,
                                                                                    double ground_z) {
    if (v.rotor_count != rx::model::kRotors) {
        return std::string("the reflex model simulates quadrotors only (rotor_count 4)");
    }
    rx::model::QuadrotorParams<T> p;
    p.mass = T(v.mass);
    p.inertia = {T(v.inertia[0]), T(v.inertia[1]), T(v.inertia[2])};
    p.arm_length = T(v.arm_length);
    p.thrust_coefficient = T(v.rotor.thrust_coefficient);
    p.torque_coefficient = T(v.rotor.torque_coefficient);
    p.max_thrust = T(v.rotor.max_thrust);
    p.motor_time_constant = T(v.rotor.time_constant);
    p.drag_linear = {T(v.drag.linear[0]), T(v.drag.linear[1]), T(v.drag.linear[2])};
    p.drag_quadratic = {T(v.drag.quadratic[0]), T(v.drag.quadratic[1]), T(v.drag.quadratic[2])};
    p.ground_z = T(ground_z);
    if (!rx::model::quadrotor_params_valid(p)) {
        std::string why = unrepresentable<T>("mass", v.mass);
        for (std::size_t i = 0; i < 3; ++i) {
            const std::string axis(1, "xyz"[i]);
            why += unrepresentable<T>("inertia." + axis, v.inertia[i]);
            why += unrepresentable<T>("drag.linear." + axis, v.drag.linear[i]);
            why += unrepresentable<T>("drag.quadratic." + axis, v.drag.quadratic[i]);
        }
        why += unrepresentable<T>("arm_length", v.arm_length);
        why += unrepresentable<T>("rotor.thrust_coefficient", v.rotor.thrust_coefficient);
        why += unrepresentable<T>("rotor.torque_coefficient", v.rotor.torque_coefficient);
        why += unrepresentable<T>("rotor.max_thrust", v.rotor.max_thrust);
        why += unrepresentable<T>("rotor.time_constant", v.rotor.time_constant);
        why += unrepresentable<T>("ground_z", ground_z);
        return std::string("vehicle '" + v.name + "' does not give valid reflex model parameters" + why);
    }
    return p;
}

}  // namespace branes::anatomy::glue
