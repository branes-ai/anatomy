#pragma once
// Helpers shared by the precision tests.

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>

namespace precision_fixtures {

/// The two representable neighbors of t, as doubles.
template <typename T>
std::pair<double, double> neighbors(T t) {
    if (t == T(0)) {
        // ±0 sits between the smallest positive and negative values; a cfloat's
        // encoding decrement from +0 doesn't land on −denorm_min.
        const double d = double(std::numeric_limits<T>::denorm_min());
        return {-d, d};
    }
    if constexpr (std::is_floating_point_v<T>) {
        return {double(std::nextafter(t, -std::numeric_limits<T>::infinity())),
                double(std::nextafter(t, std::numeric_limits<T>::infinity()))};
    } else {
        T below = t;
        T above = t;
        --below;  // the adjacent encodings, which Universal orders by value
        ++above;
        return {double(below), double(above)};
    }
}

}  // namespace precision_fixtures
