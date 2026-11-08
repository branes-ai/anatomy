#pragma once
/// @file window.hpp
/// @brief What an `accumulate` edge's reader reads: the values its producer
/// wrote since the reader's region last began, oldest first (anatomy
/// docs/plan/workbench, B2; the model's accumulator domain, section 8).
///
/// Model:
///   window<M, N>  : items, the values in order, oldest first; count, how many
///                   of items are in use; dropped, how many older values were
///                   written but didn't fit
///   its domain    : record<field<"items", vector<E, bounded<N>>>,
///                          field<"dropped", scalar<std::uint64_t>>>, where E is
///                   the element type of M's domain if that is a scalar, or M's
///                   record. A message whose domain has a shape isn't
///                   accumulated, until a pattern needs one.
///
/// Invariants:
///   W1: 0 <= count <= N
///   W2: every item at or beyond count is value-initialized, so two windows
///       of the same values compare equal and encode alike
///
/// The stack writes a window when the reader's region begins (flow/builder.hpp);
/// a reader takes it by const reference.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/signature.hpp>

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace branes::anatomy::flow {

/// The values an `accumulate<N>` edge's producer wrote since its reader's
/// region last began: the latest N of them, oldest first, and how many older
/// ones were dropped.
template <class M, std::size_t N>
struct window {
    static_assert(N >= 1, "G7: an accumulator holds at least one value");

    static constexpr std::size_t capacity = N;

    std::array<M, N> items{};
    std::size_t count = 0;      ///< W1
    std::uint64_t dropped = 0;  ///< written since the region last began, and not in items

    friend constexpr bool operator==(const window&, const window&)
        requires std::equality_comparable<M>
    = default;
};

namespace detail {
/// E for a window's items: a record, or a scalar domain's element type.
template <class D, bool = is_record_v<D>>
struct window_element {
    using type = D;
};
template <class D>
struct window_element<D, false> {
    static_assert(D::rank == 0, "accumulate: a message whose domain has a shape isn't accumulated");
    using type = typename D::element_type;
};
}  // namespace detail

/// The domain of a window of N values of X (a message type, a domain or a record).
template <class X, std::size_t N>
using window_domain_t = record<field<"items", vector<typename detail::window_element<data_t<X>>::type, bounded<N>>>,
                               field<"dropped", scalar<std::uint64_t>>>;

template <class M, std::size_t N>
struct domain_of<window<M, N>> {
    using type = window_domain_t<M, N>;
};

/// What an `accumulate<N>` edge from a value of X carries: window<X, N> for a
/// message type, and its domain for a graph that only describes (X a domain
/// or a record).
template <class X, std::size_t N>
using window_t = std::conditional_t<is_data_v<X>, window_domain_t<X, N>, window<X, N>>;

}  // namespace branes::anatomy::flow
