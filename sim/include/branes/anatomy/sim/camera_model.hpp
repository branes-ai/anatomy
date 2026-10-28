#pragma once
/// @file camera_model.hpp
/// @brief The camera model: world point → camera frame → pixels, no rendering.
///
/// what an oracle
/// sensor uses to see the world through the vehicle's camera (racing-drone: the gate
/// projector, gates.hpp).
///
/// Conventions:
///  - Camera frame: z forward, x right, y down (the vehicle file's convention);
///    rotation_body maps camera to body, translation_body is the camera origin
///    in the body frame.
///  - Equidistant (Kannala–Brandt): θ = atan2(r, z), r = √(x² + y²),
///    θ_d = θ(1 + k1θ² + k2θ⁴ + k3θ⁶ + k4θ⁸), (u, v) = (fx θ_d x/r + cx, fy θ_d y/r + cy).
///    Pinhole–radtan: (x', y') = (x/z, y/z) with radial k1, k2 and tangential
///    p1, p2 distortion.

#include <branes/anatomy/config/vehicle.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/truth.hpp>

#include <array>
#include <cmath>
#include <optional>

namespace branes::anatomy::sim {

namespace detail {

/// v rotated by unit quaternion q (Hamilton, scalar first).
[[nodiscard]] inline Vec3<double> rotate(const Quat<double>& q, const Vec3<double>& v) noexcept {
    // t = 2 q_v × v;  v' = v + w t + q_v × t
    const Vec3<double> u{q.x, q.y, q.z};
    const auto cross = [](const Vec3<double>& a, const Vec3<double>& b) {
        return Vec3<double>{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    };
    Vec3<double> t = cross(u, v);
    for (double& c : t) {
        c *= 2;
    }
    const Vec3<double> ut = cross(u, t);
    return {v[0] + q.w * t[0] + ut[0], v[1] + q.w * t[1] + ut[1], v[2] + q.w * t[2] + ut[2]};
}

[[nodiscard]] inline Quat<double> conjugate(const Quat<double>& q) noexcept {
    return {q.w, -q.x, -q.y, -q.z};
}

/// Pixel coordinates of camera-frame point `p`, or nullopt behind the camera.
[[nodiscard]] inline std::optional<std::array<double, 2>> project(const config::CameraParams& cam,
                                                                  const Vec3<double>& p) noexcept {
    if (!(p[2] > 0)) {
        return std::nullopt;
    }
    const auto& k = cam.distortion;
    if (cam.model == config::CameraModel::equidistant) {
        const double r = std::sqrt(p[0] * p[0] + p[1] * p[1]);
        if (r == 0) {
            return std::array<double, 2>{cam.cx, cam.cy};
        }
        const double th = std::atan2(r, p[2]);
        const double th2 = th * th;
        const double thd = th * (1 + th2 * (k[0] + th2 * (k[1] + th2 * (k[2] + th2 * k[3]))));
        return std::array<double, 2>{cam.fx * thd * p[0] / r + cam.cx, cam.fy * thd * p[1] / r + cam.cy};
    }
    const double x = p[0] / p[2];
    const double y = p[1] / p[2];
    const double r2 = x * x + y * y;
    const double radial = 1 + r2 * (k[0] + r2 * k[1]);
    const double xd = x * radial + 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x);
    const double yd = y * radial + k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y;
    return std::array<double, 2>{cam.fx * xd + cam.cx, cam.fy * yd + cam.cy};
}

}  // namespace detail

/// World-frame point → camera frame, for the vehicle at `x`.
[[nodiscard]] inline Vec3<double>
to_camera(const config::CameraParams& cam, const TruthState<double>& x, const Vec3<double>& p) noexcept {
    const Vec3<double> d{p[0] - x.position[0], p[1] - x.position[1], p[2] - x.position[2]};
    const Vec3<double> b = detail::rotate(detail::conjugate(x.attitude), d);
    const Vec3<double> bc{
        b[0] - cam.translation_body[0], b[1] - cam.translation_body[1], b[2] - cam.translation_body[2]};
    return detail::rotate(detail::conjugate(cam.rotation_body), bc);
}

}  // namespace branes::anatomy::sim
