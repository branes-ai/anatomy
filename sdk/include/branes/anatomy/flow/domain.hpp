#pragma once
/// @file domain.hpp
/// @brief Data domains: extents, structures, domains and records, as types
/// (anatomy docs/adr/0004, step 1; docs/plan/flow-domains).
///
/// A domain is an element type, a structure and a list of extents, whose
/// number is its rank (docs/arch/domain-flow-model.md §3). A record is a list
/// of named fields, each a domain or a record; a record is also an element
/// type, so an array of records is a domain. Everything here is a type, and
/// two domains match when they are the same type.
///
/// Model:
///   dim<N, Name>              : a static extent of size N, optionally named
///   bounded<Max, Name>        : an extent of at most Max, with a count in the data
///   domain<E, S, Extents...>  : element type E, structure S, the extents
///   field<Name, D>            : a named domain or record
///   record<Fields...>         : the fields, in order
///   domain_of<M>              : the domain or record a message type M is
///
/// Invariants (static_asserts, checked when a type is instantiated: by a
/// query, or by a record, port or signature that holds it, which instantiates
/// what it holds):
///   D1: every extent has a size or a bound of at least 1
///   D2: a symmetric, triangular or diagonal structure needs rank 2 and the
///       same extent twice; a unit-norm one needs rank 1, a static extent, and
///       a floating-point element type
///   D3: the element type is an element type (is_element_v): arithmetic, an
///       enumeration, Timestamp, std::complex of a floating-point type, or a record
///   R1: a record has at least one field, its field names are non-empty and
///       distinct, and each field holds a domain or a record
///
/// Contracts:
///   field_t<R, Name> — Pre: R has a field Name, or a static_assert. Post: its type.
///   domain_of_t<M>   — Pre: domain_of<M> is specialized. Post: a domain or a record.

#include <branes/anatomy/msg/time.hpp>

#include <algorithm>
#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace branes::anatomy::flow {

/// A string as a template argument (a name).
template <std::size_t N>
struct fixed_string {
    char value[N]{};

    // Implicit, so a literal converts in a template argument.
    constexpr fixed_string(const char (&s)[N]) noexcept {
        std::copy_n(s, N, value);
    }
    [[nodiscard]] constexpr std::string_view view() const noexcept {
        return {value, N - 1};
    }
};

namespace detail {
struct extent_tag {};
struct domain_tag {};
struct record_tag {};
struct field_tag {};

/// True if every name is non-empty and no two are equal.
template <std::size_t K>
[[nodiscard]] constexpr bool distinct_nonempty(const std::array<std::string_view, K>& names) noexcept {
    for (std::size_t i = 0; i < K; ++i) {
        if (names[i].empty()) {
            return false;
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (names[i] == names[j]) {
                return false;
            }
        }
    }
    return true;
}

/// The position of `name` in `names`, or K if it is not there.
template <std::size_t K>
[[nodiscard]] constexpr std::size_t index_of(const std::array<std::string_view, K>& names,
                                             std::string_view name) noexcept {
    for (std::size_t i = 0; i < K; ++i) {
        if (names[i] == name) {
            return i;
        }
    }
    return K;
}
}  // namespace detail

// ── Extents ─────────────────────────────────────────────────────────

/// A static extent of size N. A name (`n`) is carried, for binding from config in step 2.
template <std::size_t N, fixed_string Name = "">
struct dim : detail::extent_tag {
    static_assert(N >= 1, "D1: every extent has a size or a bound of at least 1");
    static constexpr std::size_t size = N;
    static constexpr bool is_bounded = false;
    static constexpr std::string_view name = Name.view();
};

/// An extent of at most Max; the data carries the count in use.
template <std::size_t Max, fixed_string Name = "">
struct bounded : detail::extent_tag {
    static_assert(Max >= 1, "D1: every extent has a size or a bound of at least 1");
    static constexpr std::size_t size = Max;
    static constexpr bool is_bounded = true;
    static constexpr std::string_view name = Name.view();
};

template <class X>
inline constexpr bool is_extent_v = std::is_base_of_v<detail::extent_tag, X>;

// ── Element types ───────────────────────────────────────────────────

/// Specialize with `static constexpr bool is_element = true;` to admit a type
/// that is not arithmetic, an enumeration, Timestamp, a complex or a record.
template <class T>
struct element_traits {
    static constexpr bool is_element = false;
};

template <>
struct element_traits<Timestamp> {
    static constexpr bool is_element = true;
};

/// std::complex of float, double or long double (docs/plan/workbench, B1).
template <class T>
struct element_traits<std::complex<T>> {
    static constexpr bool is_element = std::is_floating_point_v<T>;
};

template <class R>
inline constexpr bool is_record_v = std::is_base_of_v<detail::record_tag, R>;

template <class T>
inline constexpr bool is_element_v =
    std::is_same_v<T, std::remove_cvref_t<T>> &&
    (std::is_arithmetic_v<T> || std::is_enum_v<T> || is_record_v<T> || element_traits<T>::is_element);

// ── Domains ─────────────────────────────────────────────────────────

/// How a domain's elements are constrained, and so stored (flow/data.hpp). A
/// unit-norm vector is stored as dense is; its norm is 1 (docs/plan/computational-domain, C5).
enum class structure : std::uint8_t { dense, symmetric, lower_triangular, upper_triangular, diagonal, unit_norm };

namespace detail {
template <class... Extents>
[[nodiscard]] constexpr bool square() noexcept {
    if constexpr (sizeof...(Extents) == 2) {
        return std::is_same_v<std::tuple_element_t<0, std::tuple<Extents...>>,
                              std::tuple_element_t<1, std::tuple<Extents...>>>;
    } else {
        return false;
    }
}

/// A unit-norm domain's shape: one static extent, and a floating-point element.
template <class E, class... Extents>
[[nodiscard]] constexpr bool unit_norm_shape() noexcept {
    if constexpr (sizeof...(Extents) == 1) {
        return std::is_floating_point_v<E> && !(Extents::is_bounded || ...);
    } else {
        return false;
    }
}
}  // namespace detail

/// Element type E, structure S, and the extents (rank = their number).
template <class E, structure S, class... Extents>
struct domain : detail::domain_tag {
    static_assert((is_extent_v<Extents> && ...), "D1: a domain's extents are dim or bounded");
    static_assert(S == structure::dense || S == structure::unit_norm || detail::square<Extents...>(),
                  "D2: a symmetric, triangular or diagonal structure needs a square domain of rank 2");
    static_assert(S != structure::unit_norm || detail::unit_norm_shape<E, Extents...>(),
                  "D2: a unit-norm structure needs a vector of static extent with a floating-point element type");
    static_assert(is_element_v<E>, "D3: the element type is an element type");

    using element_type = E;
    using extents = std::tuple<Extents...>;
    static constexpr structure kind = S;
    static constexpr std::size_t rank = sizeof...(Extents);
};

template <class E>
using scalar = domain<E, structure::dense>;
template <class E, class X, structure S = structure::dense>
using vector = domain<E, S, X>;
template <class E, class R, class C, structure S = structure::dense>
using matrix = domain<E, S, R, C>;

template <class D>
inline constexpr bool is_domain_v = std::is_base_of_v<detail::domain_tag, D>;

/// What a field, a port, a state or a message carries: a domain or a record.
template <class D>
inline constexpr bool is_data_v = is_domain_v<D> || is_record_v<D>;

// ── Records ─────────────────────────────────────────────────────────

/// A named domain or record.
template <fixed_string Name, class D>
struct field : detail::field_tag {
    static_assert(is_data_v<D>, "R1: each field holds a domain or a record");
    using type = D;
    static constexpr std::string_view name = Name.view();
};

template <class F>
inline constexpr bool is_field_v = std::is_base_of_v<detail::field_tag, F>;

/// Named fields, in order.
template <class... Fields>
struct record : detail::record_tag {
    static_assert(sizeof...(Fields) >= 1, "R1: a record has at least one field");
    static_assert((is_field_v<Fields> && ...), "R1: a record's members are fields");

    static constexpr std::size_t size = sizeof...(Fields);
    static constexpr std::array<std::string_view, sizeof...(Fields)> names{Fields::name...};
    static_assert(detail::distinct_nonempty(names), "R1: a record's field names are non-empty and distinct");

    using field_types = std::tuple<typename Fields::type...>;
};

namespace detail {
template <class R, fixed_string Name>
struct find_field {
    static_assert(is_record_v<R>, "field_t: R is a record");
    static constexpr std::size_t index = index_of(R::names, Name.view());
    static_assert(index < R::size, "field_t: the record has no field of that name");
    using type = std::tuple_element_t<(index < R::size ? index : 0), typename R::field_types>;
};
}  // namespace detail

template <class R, fixed_string Name>
using field_t = typename detail::find_field<R, Name>::type;

// ── Messages ────────────────────────────────────────────────────────

/// The domain or record a message type is. Specialized per message, with `using type = ...;`.
template <class M>
struct domain_of;

template <class M>
using domain_of_t = typename domain_of<M>::type;

// ── Queries ─────────────────────────────────────────────────────────

namespace detail {
template <class D>
struct require_domain {
    static_assert(is_domain_v<D>, "a domain query takes a domain");
    using type = D;
};

template <class Tuple>
struct extent_product;
template <class... Extents>
struct extent_product<std::tuple<Extents...>> {
    static constexpr std::size_t value = (std::size_t{1} * ... * Extents::size);
};

template <class Tuple>
struct any_bounded;
template <class... Extents>
struct any_bounded<std::tuple<Extents...>> {
    static constexpr bool value = (Extents::is_bounded || ...);
};
}  // namespace detail

/// The number of extents.
template <class D>
inline constexpr std::size_t rank_v = detail::require_domain<D>::type::rank;

/// The product of the extents' sizes or bounds.
template <class D>
inline constexpr std::size_t max_elements_v =
    detail::extent_product<typename detail::require_domain<D>::type::extents>::value;

/// The elements the structure needs: n(n+1)/2 for symmetric and triangular, n
/// for diagonal, and every element for dense and unit-norm.
template <class D>
inline constexpr std::size_t stored_elements_v = [] {
    using Dom = typename detail::require_domain<D>::type;
    if constexpr (Dom::kind == structure::dense || Dom::kind == structure::unit_norm) {
        return max_elements_v<Dom>;
    } else {
        constexpr std::size_t n = std::tuple_element_t<0, typename Dom::extents>::size;
        if constexpr (Dom::kind == structure::diagonal) {
            return n;
        } else {
            return n * (n + 1) / 2;
        }
    }
}();

/// Some extent is bounded.
template <class D>
inline constexpr bool is_bounded_v = detail::any_bounded<typename detail::require_domain<D>::type::extents>::value;

}  // namespace branes::anatomy::flow
