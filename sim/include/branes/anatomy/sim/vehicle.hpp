#pragma once
/// @file vehicle.hpp
/// @brief The reflex plant parameters for a vehicle file, and the per-run randomization.
///
/// reflex_params_from() maps a VehicleConfig onto reflex's QuadrotorParams
/// (glue/reflex_params.hpp). The ground is a plane at the start pad's height, so
/// a vehicle starts at rest on the pad.
///
/// randomize() applies a scenario's `randomization:` block: each parameter p
/// becomes p·(1 + u) with u uniform in [−f, f], drawn from the run's
/// randomization stream, and the start pose is jittered. The draws are
/// returned so the harness can log them.

#include <branes/anatomy/glue/reflex_params.hpp>
#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/util/splitmix64.hpp>
#include <branes/reflex/model/quadrotor.hpp>

#include <cstdio>
#include <string>

namespace branes::anatomy::sim {

namespace rx = branes::reflex;

using PlantParams = rx::model::QuadrotorParams<double>;

/// reflex QuadrotorParams for `v`, with the ground at `ground_z` [m]
/// (the mapping is shared with the controllers, glue/reflex_params.hpp).
[[nodiscard]] inline result<PlantParams, std::string> reflex_params_from(const config::VehicleConfig& v,
                                                                         double ground_z) {
    return glue::reflex_params_from<double>(v, ground_z);
}

/// The values one run drew from its scenario's randomization block.
struct Draws {
    double mass_scale{1};
    double inertia_scale{1};
    double drag_scale{1};
    double motor_time_constant_scale{1};
    double start_dx{0};    ///< [m]
    double start_dy{0};    ///< [m]
    double start_dyaw{0};  ///< [rad]
};

/// The draws as YAML text, for the log's sim_config record.
[[nodiscard]] inline std::string to_yaml(const Draws& d) {
    char buf[512];
    std::snprintf(buf,
                  sizeof buf,
                  "mass_scale: %.17g\ninertia_scale: %.17g\ndrag_scale: %.17g\nmotor_time_constant_scale: %.17g\n"
                  "start_dx_m: %.17g\nstart_dy_m: %.17g\nstart_dyaw_rad: %.17g\n",
                  d.mass_scale,
                  d.inertia_scale,
                  d.drag_scale,
                  d.motor_time_constant_scale,
                  d.start_dx,
                  d.start_dy,
                  d.start_dyaw);
    return buf;
}

/// Draw a run's randomization from `rng`, always in the same order (so a
/// zero-width field still consumes its draw and the other draws do not shift).
[[nodiscard]] inline Draws draw(const config::Randomization& r, SplitMix64& rng) {
    Draws d;
    d.mass_scale = 1 + rng.symmetric(r.mass);
    d.inertia_scale = 1 + rng.symmetric(r.inertia);
    d.drag_scale = 1 + rng.symmetric(r.drag);
    d.motor_time_constant_scale = 1 + rng.symmetric(r.motor_time_constant);
    d.start_dx = rng.symmetric(r.start_xy_m);
    d.start_dy = rng.symmetric(r.start_xy_m);
    d.start_dyaw = rng.symmetric(r.start_yaw_rad);
    return d;
}

/// `p` with the draws applied. Randomization fractions are < 1 (the scenario
/// loader checks), so every scaled parameter stays positive.
[[nodiscard]] inline PlantParams apply(PlantParams p, const Draws& d) {
    p.mass *= d.mass_scale;
    for (double& j : p.inertia) {
        j *= d.inertia_scale;
    }
    for (std::size_t i = 0; i < 3; ++i) {
        p.drag_linear[i] *= d.drag_scale;
        p.drag_quadratic[i] *= d.drag_scale;
    }
    p.motor_time_constant *= d.motor_time_constant_scale;
    return p;
}

/// The start pose with the draws applied.
[[nodiscard]] inline StartPose<double> apply(StartPose<double> s, const Draws& d) {
    s.position[0] += d.start_dx;
    s.position[1] += d.start_dy;
    s.yaw += d.start_dyaw;
    return s;
}

}  // namespace branes::anatomy::sim
