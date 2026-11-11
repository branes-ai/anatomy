#pragma once
/// @file data.hpp
/// @brief The value of a domain: its elements, stored by its structure, and
/// the count of a bounded leading extent (anatomy docs/adr/0005, step 1;
/// docs/plan/workbench, B1).
///
/// A message of type data<D> is its own domain (domain_of<data<D>> is D), so
/// a port that carries it joins only a port of the same shape, structure and
/// element type (G4). The storage is a std::array of fixed size: a data<D>
/// holds nothing on the heap.
///
/// Model:
///   D                 : a domain of rank 0, 1 or 2 whose element type E is
///                       arithmetic, complex, an enumeration or Timestamp
///   elements          : stored_elements_v<D> values of E, laid out by D's structure:
///                         dense            row-major
///                         symmetric        the upper triangle, row by row
///                         upper_triangular the upper triangle, row by row
///                         lower_triangular the lower triangle, row by row
///                         diagonal         the diagonal
///                         unit_norm        as dense: a vector, in order
///   count()           : the rows in use. A bounded leading extent's count, in
///                       [0, its bound]; a static leading extent's size; 1 for a scalar
///
/// Invariants:
///   A1: 0 <= count() <= the leading extent's size or bound; a static leading
///       extent's count is its size, and a scalar's is 1
///   A2: every element of a row at or beyond count() is value-initialized, so
///       two values with the same rows in use compare equal and encode alike
///   A3: a symmetric domain's (i, j) and (j, i) are one stored element
///   A4: a unit-norm vector's norm is 1, to within unit_norm_tolerance<E>. Its
///       writer keeps it so: a value starts as the first basis vector, and the
///       CPU stack asserts it of each one a firing writes, in a debug build
///       (structure_holds; docs/plan/computational-domain, C5)
///
/// Contracts (preconditions are checked by assert in debug builds):
///   data()                — Post: a bounded leading extent's count is 0; every element is E{},
///                           but a unit-norm vector's first, which is 1 (A4).
///   resize(n)             — Post: true and count() == n if n is a count A1 allows; else
///                           false and nothing changes. Rows beyond n are reset to E{} (A2).
///   operator()(i[, j])    — Pre: i < count(), j < the column extent; for a triangular or
///                           diagonal domain, (i, j) is inside its structure. Post: the element.
///   at(i[, j]) const      — Pre: i < count(), j < the column extent. Post: the element, or
///                           E{} outside a triangle or a diagonal.
///   structure_holds()     — Post: for a unit-norm vector, whether A4 holds; else true.

#include <branes/anatomy/flow/domain.hpp>

#include <array>
#include <cassert>
#include <cstddef>
#include <limits>
#include <tuple>
#include <type_traits>

namespace branes::anatomy::flow {

namespace detail {
template <class Tuple, std::size_t K>
struct extent_or_none {
    using type = void;
};
template <class Tuple, std::size_t K>
    requires(K < std::tuple_size_v<Tuple>)
struct extent_or_none<Tuple, K> {
    using type = std::tuple_element_t<K, Tuple>;
};

template <class X>
inline constexpr std::size_t size_of_extent = [] {
    if constexpr (std::is_void_v<X>) {
        return std::size_t{1};
    } else {
        return X::size;
    }
}();

template <class X>
inline constexpr bool extent_is_bounded = [] {
    if constexpr (std::is_void_v<X>) {
        return false;
    } else {
        return X::is_bounded;
    }
}();
}  // namespace detail

/// How far a unit-norm vector of element type E may be from norm 1 (A4): 256
/// ulps of 1, room for the rounding of a normalization and of the arithmetic
/// that follows it, and far below any error that isn't rounding.
template <class E>
inline constexpr E unit_norm_tolerance = E{256} * std::numeric_limits<E>::epsilon();

/// The value of domain D (rank 0, 1 or 2).
template <class D>
class data {
    static_assert(is_domain_v<D>, "data: D is a domain");
    static_assert(D::rank <= 2, "data: a domain of rank 0, 1 or 2");
    static_assert(!is_record_v<typename D::element_type>,
                  "data: a record's element type has no C++ type to store; a record message stays a struct");

    using row_extent = typename detail::extent_or_none<typename D::extents, 0>::type;
    using col_extent = typename detail::extent_or_none<typename D::extents, 1>::type;
    static_assert(!detail::extent_is_bounded<col_extent>, "data: only the leading extent may be bounded");
    static_assert(D::kind == structure::dense || !detail::extent_is_bounded<row_extent>,
                  "data: a structured domain's extents are static");

public:
    using domain = D;
    using element_type = typename D::element_type;

    static constexpr std::size_t rank = D::rank;
    /// The leading extent's size or bound: the most rows a value has.
    static constexpr std::size_t max_rows = detail::size_of_extent<row_extent>;
    /// The column extent's size; 1 below rank 2.
    static constexpr std::size_t cols = detail::size_of_extent<col_extent>;
    static constexpr bool bounded = detail::extent_is_bounded<row_extent>;
    static constexpr std::size_t stored = stored_elements_v<D>;

    std::array<element_type, stored> elements = initial();

    /// The rows in use (A1).
    [[nodiscard]] constexpr std::size_t count() const noexcept {
        return count_;
    }

    /// Set the rows in use, if `n` is a count A1 allows: up to the bound of a
    /// bounded leading extent, and only its size for a static one. Rows at or
    /// beyond `n` are reset to value-initialized elements (A2).
    [[nodiscard]] constexpr bool resize(std::size_t n) noexcept {
        if constexpr (bounded) {
            if (n > max_rows) {
                return false;
            }
            // Every row past n, which A2 already holds past count_: a bound
            // the compiler can see, which a loop to count_ * cols is not.
            for (std::size_t k = n * cols; k < stored; ++k) {
                elements[k] = element_type{};
            }
            count_ = n;
            return true;
        } else {
            return n == max_rows;
        }
    }

    /// A scalar's element.
    [[nodiscard]] constexpr element_type& operator()() noexcept
        requires(rank == 0)
    {
        return elements[0];
    }
    [[nodiscard]] constexpr const element_type& operator()() const noexcept
        requires(rank == 0)
    {
        return elements[0];
    }

    /// Element i of a vector. Pre: i < count().
    [[nodiscard]] constexpr element_type& operator()(std::size_t i) noexcept
        requires(rank == 1)
    {
        assert(i < count_);
        return elements[i];
    }
    [[nodiscard]] constexpr const element_type& operator()(std::size_t i) const noexcept
        requires(rank == 1)
    {
        assert(i < count_);
        return elements[i];
    }

    /// Element (i, j) of a matrix, stored by its structure (A3). Pre: i <
    /// count(), j < cols, and (i, j) is inside a triangle or a diagonal.
    [[nodiscard]] constexpr element_type& operator()(std::size_t i, std::size_t j) noexcept
        requires(rank == 2)
    {
        assert(i < count_ && j < cols);
        assert(inside(i, j));
        return elements[index(i, j)];
    }
    [[nodiscard]] constexpr const element_type& operator()(std::size_t i, std::size_t j) const noexcept
        requires(rank == 2)
    {
        assert(i < count_ && j < cols);
        assert(inside(i, j));
        return elements[index(i, j)];
    }

    /// Element (i, j), or a value-initialized element outside a triangle or a
    /// diagonal. Pre: i < count(), j < cols.
    [[nodiscard]] constexpr element_type at(std::size_t i, std::size_t j) const noexcept
        requires(rank == 2)
    {
        assert(i < count_ && j < cols);
        return inside(i, j) ? elements[index(i, j)] : element_type{};
    }

    /// Whether (i, j) has a stored element: always, but outside a triangle
    /// or off a diagonal.
    [[nodiscard]] static constexpr bool inside(std::size_t i, std::size_t j) noexcept
        requires(rank == 2)
    {
        if constexpr (D::kind == structure::upper_triangular) {
            return i <= j;
        } else if constexpr (D::kind == structure::lower_triangular) {
            return j <= i;
        } else if constexpr (D::kind == structure::diagonal) {
            return i == j;
        } else {
            return true;
        }
    }

    /// The position of (i, j) in `elements`. Pre: inside(i, j).
    [[nodiscard]] static constexpr std::size_t index(std::size_t i, std::size_t j) noexcept
        requires(rank == 2)
    {
        if constexpr (D::kind == structure::dense) {
            return i * cols + j;
        } else if constexpr (D::kind == structure::lower_triangular) {
            return i * (i + 1) / 2 + j;  // rows 0..i-1 hold 1 + 2 + ... + i elements
        } else if constexpr (D::kind == structure::diagonal) {
            return i;
        } else {
            // Symmetric or upper triangular: rows 0..r-1 of the upper triangle
            // hold n + (n - 1) + ... + (n - r + 1) = r(2n - r + 1)/2 elements.
            const std::size_t r = i < j ? i : j;
            const std::size_t c = i < j ? j : i;
            return r * (2 * cols - r + 1) / 2 + (c - r);
        }
    }

    /// A4: a unit-norm vector's squared norm is within twice the tolerance of
    /// 1, which is its norm's within the tolerance to first order. True for
    /// any other structure.
    [[nodiscard]] constexpr bool structure_holds() const noexcept {
        if constexpr (D::kind == structure::unit_norm) {
            element_type sq{};
            for (const element_type& e : elements) {
                sq += e * e;
            }
            const element_type off = sq - element_type{1};
            return (off < element_type{0} ? -off : off) <= element_type{2} * unit_norm_tolerance<element_type>;
        } else {
            return true;
        }
    }

    friend constexpr bool operator==(const data&, const data&) = default;

private:
    /// Every element value-initialized, but a unit-norm vector's first (A4).
    [[nodiscard]] static constexpr std::array<element_type, stored> initial() noexcept {
        std::array<element_type, stored> e{};
        if constexpr (D::kind == structure::unit_norm) {
            e[0] = element_type{1};
        }
        return e;
    }

    std::size_t count_ = bounded ? 0 : max_rows;
};

template <class D>
struct domain_of<data<D>> {
    using type = D;
};

}  // namespace branes::anatomy::flow
