#pragma once
/// @file geometry.hpp
/// @brief Vec3 and Quat: the geometric value types of the data contracts.
///
/// Both are plain aggregates over std::array / named scalars, so every contract
/// that holds them stays a repr(C)-compatible value type (cortex ADR 0010).
/// The arithmetic lives in the components (MTL5 from Phase 1); the contracts
/// only carry values.
///
/// cast<To>() converts a value to another scalar type, element by element; the
/// message types build their casts on these.

#include <array>
#include <cstddef>
#include <limits>

namespace branes::anatomy {

/// A 3-vector (x, y, z).
template <typename T>
using Vec3 = std::array<T, 3>;

/// A rotation quaternion, Hamilton convention, scalar first (w, x, y, z).
/// The default is the identity rotation.
template <typename T>
struct Quat {
    T w{1};
    T x{0};
    T y{0};
    T z{0};

    friend constexpr bool operator==(const Quat&, const Quat&) = default;
};

/// Squared norm of a quaternion.
template <typename T>
[[nodiscard]] constexpr T norm_squared(const Quat<T>& q) noexcept {
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

/// The contracts' unit-norm tolerance in T: max(1e-6, 4ε_T).
///
/// A quaternion that is unit in double, cast to T and checked in T, has
/// |‖q‖² − 1| ≤ 6u + O(u²) < 8u = 4ε_T, with u = ε_T/2 the unit roundoff
/// (racing-drone docs/plan/precision-sweep/ §2 derives it, posits included). For double and
/// float 4ε_T is below 1e-6, so their tolerance is 1e-6.
template <typename T>
[[nodiscard]] constexpr T unit_tolerance() noexcept {
    const T fixed = T(1e-6);
    const T rounding = T(4) * std::numeric_limits<T>::epsilon();
    return fixed < rounding ? rounding : fixed;
}

/// True if q is a unit quaternion within `tol` on the squared norm
/// (|‖q‖² − 1| ≤ tol). The contracts use unit_tolerance<T>().
template <typename T>
[[nodiscard]] constexpr bool is_unit(const Quat<T>& q, T tol = unit_tolerance<T>()) noexcept {
    const T d = norm_squared(q) - T(1);
    return d <= tol && -d <= tol;
}

/// An array (a Vec3 among them) in another scalar type.
template <typename To, typename From, std::size_t N>
[[nodiscard]] constexpr std::array<To, N> cast(const std::array<From, N>& a) noexcept {
    std::array<To, N> out{};
    for (std::size_t i = 0; i < N; ++i) {
        out[i] = To(a[i]);
    }
    return out;
}

/// A quaternion in another scalar type.
template <typename To, typename From>
[[nodiscard]] constexpr Quat<To> cast(const Quat<From>& q) noexcept {
    return Quat<To>{To(q.w), To(q.x), To(q.y), To(q.z)};
}

}  // namespace branes::anatomy
