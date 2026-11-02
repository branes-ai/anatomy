#pragma once
/// @file signature.hpp
/// @brief Operator signatures: ports, state domains and a computational
/// domain, as types (anatomy docs/adr/0004, step 1; docs/plan/flow-domains).
///
/// An operator's signature names what it reads and writes (its ports), what
/// it keeps between firings (its state domains), and, for a leaf, the index
/// space it computes over (docs/arch/domain-flow-model.md §5). Two ports
/// connect when their domains are the same type.
///
/// Model:
///   port<Name, D, Dir>          : a named domain or record, read, written, or both
///   state<Name, D>              : a named domain or record kept between firings
///   index_space<Extents...>     : a leaf's computational domain; no_index_space otherwise
///   signature<Ports, States, S> : ports<...>, states<...>, and the index space
///   signature_of<Op>            : Op::signature, or a specialization
///
/// Invariants (static_asserts, checked when a signature is instantiated,
/// which instantiates its ports, states, index space and their domains, so
/// D1–R1 hold too):
///   S1: the port and state names are non-empty and distinct, across ports
///       and state together, and each port and state holds a domain or a record
///   S2: a signature has at least one out or inout port, or a state domain
///
/// Contracts:
///   port_t<Sig, Name>, direction_v<Sig, Name> — Pre: Sig has a port Name,
///       or a static_assert. Post: its domain, its direction.
///   signature_of_t<Op> — Pre: Op::signature exists, or signature_of<Op> is
///       specialized. Post: a signature (S1, S2).

#include <branes/anatomy/flow/domain.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <tuple>
#include <type_traits>

namespace branes::anatomy::flow {

namespace detail {
struct port_tag {};
struct state_tag {};
struct signature_tag {};
struct index_space_tag {};
}  // namespace detail

enum class direction : std::uint8_t { in, out, inout };

/// A named domain or record that an operator reads (in), writes (out), or both.
template <fixed_string Name, class D, direction Dir>
struct port : detail::port_tag {
    static_assert(is_data_v<D>, "S1: each port holds a domain or a record");
    using type = D;
    static constexpr std::string_view name = Name.view();
    static constexpr direction dir = Dir;
};

/// A named domain or record that an operator keeps between firings.
template <fixed_string Name, class D>
struct state : detail::state_tag {
    static_assert(is_data_v<D>, "S1: each state holds a domain or a record");
    using type = D;
    static constexpr std::string_view name = Name.view();
};

template <class P>
inline constexpr bool is_port_v = std::is_base_of_v<detail::port_tag, P>;
template <class S>
inline constexpr bool is_state_v = std::is_base_of_v<detail::state_tag, S>;

/// A leaf operator's computational domain.
template <class... Extents>
struct index_space : detail::index_space_tag {
    static_assert((is_extent_v<Extents> && ...), "D1: an index space's extents are dim or bounded");
    static constexpr std::size_t rank = sizeof...(Extents);
};

/// A composite's, or an operator's whose index space is not declared.
struct no_index_space {};

template <class... P>
struct ports {
    static_assert((is_port_v<P> && ...), "S1: ports<> holds ports");
    static constexpr std::size_t size = sizeof...(P);
    static constexpr std::array<std::string_view, sizeof...(P)> names{P::name...};
    static constexpr bool writes = ((P::dir != direction::in) || ...);
    using types = std::tuple<P...>;
};

template <class... S>
struct states {
    static_assert((is_state_v<S> && ...), "S1: states<> holds states");
    static constexpr std::size_t size = sizeof...(S);
    static constexpr std::array<std::string_view, sizeof...(S)> names{S::name...};
    using types = std::tuple<S...>;
};

namespace detail {
template <class T>
struct is_ports : std::false_type {};
template <class... P>
struct is_ports<ports<P...>> : std::true_type {};

template <class T>
struct is_states : std::false_type {};
template <class... S>
struct is_states<states<S...>> : std::true_type {};

// is_base_of instantiates an index_space, so a signature checks its extents (D1).
template <class T>
struct is_index_space : std::bool_constant<std::is_same_v<T, no_index_space> || std::is_base_of_v<index_space_tag, T>> {
};

template <std::size_t A, std::size_t B>
[[nodiscard]] constexpr std::array<std::string_view, A + B> concat(const std::array<std::string_view, A>& a,
                                                                   const std::array<std::string_view, B>& b) noexcept {
    std::array<std::string_view, A + B> joined{};
    for (std::size_t i = 0; i < A; ++i) {
        joined[i] = a[i];
    }
    for (std::size_t i = 0; i < B; ++i) {
        joined[A + i] = b[i];
    }
    return joined;
}
}  // namespace detail

/// An operator's ports, state domains and index space.
template <class Ports, class States = states<>, class Space = no_index_space>
struct signature : detail::signature_tag {
    static_assert(detail::is_ports<Ports>::value, "signature: Ports is ports<...>");
    static_assert(detail::is_states<States>::value, "signature: States is states<...>");
    static_assert(detail::is_index_space<Space>::value, "signature: Space is index_space<...> or no_index_space");
    static_assert(detail::distinct_nonempty(detail::concat(Ports::names, States::names)),
                  "S1: the port and state names are non-empty and distinct");
    static_assert(Ports::writes || States::size > 0,
                  "S2: a signature has at least one out or inout port, or a state domain");

    using port_list = Ports;
    using state_list = States;
    using space = Space;
};

template <class Sig>
inline constexpr bool is_signature_v = std::is_base_of_v<detail::signature_tag, Sig>;

namespace detail {
template <class Sig, fixed_string Name>
struct find_port {
    static_assert(is_signature_v<Sig>, "port_t: Sig is a signature");
    using list = typename Sig::port_list;
    static constexpr std::size_t index = index_of(list::names, Name.view());
    static_assert(index < list::size, "port_t: the signature has no port of that name");
    using type = std::tuple_element_t<(index < list::size ? index : 0), typename list::types>;
};

template <class Op>
concept has_member_signature = requires { typename Op::signature; };

template <class Sig>
struct checked_signature {
    static_assert(is_signature_v<Sig>, "signature_of: the operator's signature is a signature<...>");
    using type = Sig;
};
}  // namespace detail

/// A port's domain or record.
template <class Sig, fixed_string Name>
using port_t = typename detail::find_port<Sig, Name>::type::type;

/// A port's direction.
template <class Sig, fixed_string Name>
inline constexpr direction direction_v = detail::find_port<Sig, Name>::type::dir;

/// An operator's signature: Op::signature, unless this is specialized for Op.
template <class Op>
struct signature_of {
    static_assert(detail::has_member_signature<Op>, "signature_of: Op has no signature; give it one, or specialize");
    using type = typename Op::signature;
};

template <class Op>
using signature_of_t = typename detail::checked_signature<typename signature_of<Op>::type>::type;

}  // namespace branes::anatomy::flow
