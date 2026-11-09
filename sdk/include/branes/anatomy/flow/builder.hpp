#pragma once
/// @file builder.hpp
/// @brief The graph builder: a mission's graph described in C++, and checked
/// as it is built (anatomy docs/adr/0004, step 2; docs/plan/flow-graph).
///
/// A mission declares its external ports and its composites, and fires their
/// handlers in regions, one per trigger. A scoreboard baseline fills each
/// composite with a body; a research baseline fires operators of its own
/// between the same external ports. What build() returns is the checked
/// Graph (flow/graph.hpp).
///
/// Model:
///   handler<Name, ins<...>, outs<...>, State, Time> : one function of a binding;
///                            to run, a struct deriving from it with a static call()
///   a binding B            : B::signature (step 1) and B::handlers
///   registered<Name, T, B> : an operator, its C++ type and its binding
///   a body                 : Body::name, Body::binding, Body::build(auto& body)
///   value<X>               : a value of message type X (or of domain X, to describe
///                            only): an input, a constant, or an out port of a firing
///   node<B>                : a composite or an operator instance, of binding B
///   scope                  : the region a firing goes into
///   stack_builder<Ctx>     : a builder that also builds the operators, from the
///                            mission's context, and returns the CPU stack (stack)
///
/// A mission's description is a template over its builder (template <class G>
/// void graph(G& g)): built with a builder, it is checked and returns the
/// Graph; built with a stack_builder, it also builds the stack that runs it.
///
/// The checks split by where the facts live. What the types know is checked
/// by the compiler, with a static_assert that names the invariant: a firing
/// names a handler of its node (G2), gives one source per in port (G3), and
/// joins ports of the same domain (G4); a body fills its composite (G2). The
/// rest is checked as the graph is built, and build() returns the first
/// invariant broken, as a ConfigError naming the node or the firing.
///
/// A call that takes a type or a name as a template argument is a free
/// function, so that it reads the same inside a mission's templates:
/// flow::fire<"step">(tick, controller, ...), never tick.template fire<...>.
///
/// The order of a region's firings is the order of the calls. A firing's out
/// ports are values only once it is fired, so a region's flow edges are
/// acyclic and its order a topological order of them, by construction (G5).

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/flow/window.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/slots/slot_context.hpp>

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <typeinfo>
#include <utility>
#include <vector>

namespace branes::anatomy::flow {

// ── Handlers and bindings ───────────────────────────────────────────

/// Whether a handler takes the time of the input that fires it.
enum class takes_time : std::uint8_t { no, yes };

/// A handler's in ports, by name, in the binding's order.
template <fixed_string... Names>
struct ins {
    static constexpr std::size_t size = sizeof...(Names);
    static constexpr std::array<std::string_view, sizeof...(Names)> names{Names.view()...};
};

/// A handler's out ports, by name, in the binding's order.
template <fixed_string... Names>
struct outs {
    static constexpr std::size_t size = sizeof...(Names);
    static constexpr std::array<std::string_view, sizeof...(Names)> names{Names.view()...};
};

namespace detail {
struct handler_tag {};
struct registered_tag {};

/// [A-Za-z0-9_]+: an operator's, a body's or a graph's name.
[[nodiscard]] constexpr bool is_operator_name(std::string_view s) noexcept {
    if (s.empty()) {
        return false;
    }
    for (const char c : s) {
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) {
            return false;
        }
    }
    return true;
}

template <class T>
struct is_ins : std::false_type {};
template <fixed_string... N>
struct is_ins<ins<N...>> : std::true_type {};
template <class T>
struct is_outs : std::false_type {};
template <fixed_string... N>
struct is_outs<outs<N...>> : std::true_type {};
}  // namespace detail

/// One function of a binding: its ports, what it does with its operator's
/// state (ADR 0004, decision 4), and whether it takes a time.
template <fixed_string Name,
          class In,
          class Out = outs<>,
          state_role State = state_role::none,
          takes_time Time = takes_time::no>
struct handler : detail::handler_tag {
    static_assert(detail::is_ins<In>::value, "handler: In is ins<...>");
    static_assert(detail::is_outs<Out>::value, "handler: Out is outs<...>");
    static_assert(detail::distinct_nonempty(In::names) && detail::distinct_nonempty(Out::names),
                  "G1: a handler's port names are non-empty and distinct");

    static constexpr std::string_view name = Name.view();
    using in = In;
    using out = Out;
    static constexpr state_role state = State;
    static constexpr bool timed = Time == takes_time::yes;
};

/// To run on the CPU, a handler is a struct deriving from handler<>, whose
/// static call() fires it on its operator:
///
///   struct frame : flow::handler<"frame", flow::ins<"camera">, flow::outs<>, flow::state_role::update> {
///       static void call(auto& est, const CameraFrame& f) { est.process_frame(f); }
///   };
///
/// call(op, [time,] in...) takes the operator, the time if the handler takes
/// one, and the in ports' messages, in order; it returns nothing, the out
/// port's message, or a std::tuple of the out ports' messages.

template <class... H>
struct handlers {
    static_assert((std::is_base_of_v<detail::handler_tag, H> && ...), "handlers<> holds handlers");
    static_assert(sizeof...(H) >= 1, "G2: a binding has at least one handler");
    static_assert(detail::distinct_nonempty(std::array<std::string_view, sizeof...(H)>{H::name...}),
                  "G1: a binding's handler names are non-empty and distinct");

    static constexpr std::size_t size = sizeof...(H);
    static constexpr std::array<std::string_view, sizeof...(H)> names{H::name...};
    using types = std::tuple<H...>;
};

namespace detail {
template <class Ports>
struct port_directions;
template <class... P>
struct port_directions<ports<P...>> {
    static constexpr std::array<direction, sizeof...(P)> value{P::dir...};
};

/// Every name is a port of `Sig` that a handler may read (`in`) or write.
template <class Sig, std::size_t K>
[[nodiscard]] constexpr bool ports_of(const std::array<std::string_view, K>& names, bool in) noexcept {
    using list = typename Sig::port_list;
    constexpr auto dirs = port_directions<list>::value;
    for (std::size_t i = 0; i < K; ++i) {
        const std::size_t at = index_of(list::names, names[i]);
        if (at == list::size || dirs[at] == (in ? direction::out : direction::in)) {
            return false;
        }
    }
    return true;
}

template <class Sig, class... H>
[[nodiscard]] constexpr bool handlers_fit(std::tuple<H...>*) noexcept {
    return ((ports_of<Sig>(H::in::names, true) && ports_of<Sig>(H::out::names, false)) && ...);
}
}  // namespace detail

/// A binding: the CPU calling convention of a composite's operators, or of
/// an operator of its own. It gives its signature (step 1) and its handlers,
/// each with its call() to run.
template <class B>
concept binding = requires {
    typename B::signature;
    typename B::handlers;
} && is_signature_v<typename B::signature>;

namespace detail {
/// A handler that declares its computational domain (flow/compute.hpp).
template <class H>
concept has_compute = requires { H::compute; };

/// K1 (k == 1) or K2 holds for handler H of B, if it declares a computational domain.
template <class B, class H>
[[nodiscard]] constexpr bool compute_holds(int k) noexcept {
    if constexpr (has_compute<H>) {
        const cd::table t = cd::table_of<B, H>();
        return (k == 1 ? cd::k1(H::compute, t) : cd::k2(H::compute, t)).k == 0;
    } else {
        (void)k;
        return true;
    }
}

/// K1 or K2 holds for every handler of B that declares a computational domain.
template <class B, class... H>
[[nodiscard]] constexpr bool computes_hold(std::tuple<H...>*, int k) noexcept {
    return (compute_holds<B, H>(k) && ...);
}

template <binding B>
struct checked_binding {
    static_assert(handlers_fit<typename B::signature>(static_cast<typename B::handlers::types*>(nullptr)),
                  "G2: a handler's in ports are in or inout ports of its signature, and its out ports out or inout");
    static_assert(computes_hold<B>(static_cast<typename B::handlers::types*>(nullptr), 1),
                  "K1: a computational domain names its binding's ports and states, reads and writes only what "
                  "its handler may, gives each read and write its target's rank, keeps one element type per "
                  "operation, and keeps each index in its stage");
    static_assert(computes_hold<B>(static_cast<typename B::handlers::types*>(nullptr), 2),
                  "K2: a computational domain binds each parameter to an extent, a bound or a count its target has");
    using type = B;
};

template <class B, fixed_string Name>
struct find_handler {
    using list = typename checked_binding<B>::type::handlers;
    static constexpr std::size_t index = index_of(list::names, Name.view());
    static_assert(index < list::size, "G2: a firing names a handler its node declares");
    using type = std::tuple_element_t<(index < list::size ? index : 0), typename list::types>;
};

/// A port's message type, or its domain if it carries none.
template <class Sig, fixed_string N>
using port_value_t = std::conditional_t<std::is_void_v<message_t<Sig, N>>, port_t<Sig, N>, message_t<Sig, N>>;

/// A handler's ports' message types (or domains), in order.
template <class Sig, class Names>
struct port_values;
template <class Sig, fixed_string... N>
struct port_values<Sig, ins<N...>> {
    using type = std::tuple<port_value_t<Sig, N>...>;
};
template <class Sig, fixed_string... N>
struct port_values<Sig, outs<N...>> {
    using type = std::tuple<port_value_t<Sig, N>...>;
};
}  // namespace detail

/// A binding's handler of that name.
template <class B, fixed_string Name>
using handler_t = typename detail::find_handler<B, Name>::type;

namespace cd {
/// K1–K8, the counts and the depth of the computational domain of handler H
/// of registered operator Op (flow/compute.hpp).
template <class Op, fixed_string H>
[[nodiscard]] report check() {
    using B = typename Op::binding;
    using Hd = handler_t<B, H>;
    static_assert(flow::detail::has_compute<Hd>, "cd::check: the handler declares no computational domain");
    return analyze(Hd::compute, table_of<B, Hd>());
}

/// The computational domain of handler H of Op, as text.
template <class Op, fixed_string H>
[[nodiscard]] std::string text_of() {
    using B = typename Op::binding;
    using Hd = handler_t<B, H>;
    static_assert(flow::detail::has_compute<Hd>, "cd::text_of: the handler declares no computational domain");
    return text(Hd::compute, table_of<B, Hd>());
}
}  // namespace cd

/// An operator, as the registry names it: its name, its C++ type, and its
/// binding (its composite's, if it fills one).
template <fixed_string Name, class Type, class Binding>
struct registered : detail::registered_tag {
    static_assert(detail::is_operator_name(Name.view()), "G1: an operator's name matches [A-Za-z0-9_]+");
    static constexpr std::string_view name = Name.view();
    using type = Type;
    using binding = typename detail::checked_binding<Binding>::type;
};

template <class Op>
inline constexpr bool is_registered_v = std::is_base_of_v<detail::registered_tag, Op>;

// ── Values ──────────────────────────────────────────────────────────

namespace detail {
/// One projection of a record: the member's address in the record's.
using access_fn = const void* (*)(const void*);

inline constexpr std::size_t no_slot = std::numeric_limits<std::size_t>::max();

/// An accumulate edge's accumulator, untyped, as the stack runs it.
struct accumulator_ops {
    std::shared_ptr<void> (*make)();         ///< an empty accumulator
    std::shared_ptr<void> (*make_window)();  ///< an empty window, which the reader reads
    void (*push)(void* acc, const void* m);  ///< the producer wrote m
    void (*latch)(void* acc, void* w);       ///< the reader's region begins: fill w, and empty the accumulator
};

/// The latest N values of M written since the last latch, in a ring, and how
/// many older ones were dropped. Pre, for push and latch: `acc` is a ring<M,
/// N>, `m` an M, `w` a window<M, N>.
template <class M, std::size_t N>
struct ring {
    std::array<M, N> items{};
    std::size_t head = 0;  ///< the oldest value's position
    std::size_t size = 0;
    std::uint64_t dropped = 0;

    [[nodiscard]] static std::shared_ptr<void> make() {
        return std::make_shared<ring>();
    }
    [[nodiscard]] static std::shared_ptr<void> make_window() {
        return std::make_shared<window<M, N>>();
    }
    static void push(void* acc, const void* m) {
        ring& r = *static_cast<ring*>(acc);
        const M& x = *static_cast<const M*>(m);
        if (r.size < N) {
            r.items[(r.head + r.size) % N] = x;
            ++r.size;
        } else {
            r.items[r.head] = x;  // over the oldest, which is dropped
            r.head = (r.head + 1) % N;
            ++r.dropped;
        }
    }
    static void latch(void* acc, void* w) {
        ring& r = *static_cast<ring*>(acc);
        window<M, N>& out = *static_cast<window<M, N>*>(w);
        for (std::size_t k = 0; k < N; ++k) {
            out.items[k] = k < r.size ? r.items[(r.head + k) % N] : M{};  // W2
        }
        out.count = r.size;
        out.dropped = r.dropped;
        r.head = 0;
        r.size = 0;
        r.dropped = 0;
    }
};

template <class M, std::size_t N>
inline constexpr accumulator_ops accumulator_ops_v{
    &ring<M, N>::make, &ring<M, N>::make_window, &ring<M, N>::push, &ring<M, N>::latch};

/// What a value is, untyped: where the graph reads it from (Ref), where the
/// stack keeps it (a slot), and the projections taken from it.
struct source {
    Ref ref;
    std::size_t slot = no_slot;
    std::vector<access_fn> access;  ///< outermost first, one per ref.path entry
    /// An accumulate edge's accumulator, if it carries a message type; else null.
    const accumulator_ops* accumulator = nullptr;
};
}  // namespace detail

/// A value of message type X: an input, a constant, or an out port of a
/// firing, with the record fields projected from it. X may be a domain, to
/// describe a graph that doesn't run.
template <class X>
struct value : detail::source {
    static_assert(is_port_data_v<X>, "a value holds a message type with a domain, a domain or a record");
    using type = X;
    using domain = data_t<X>;
};

/// An external input: a message M the stack is handed, which fires its trigger.
template <class M>
struct input_port : value<M> {
    std::size_t index = 0;  ///< the builder's
};

/// A constant: a message M fixed when the stack is built.
template <class M>
struct constant_port : value<M> {};

/// An external output: a message M the stack returns from a region.
template <class M>
struct output_port {
    using message = M;
    std::size_t index = 0;
};

/// A value read in another region than its own: the latest value written,
/// or the message value-initialized, before its region first writes it.
template <class X>
struct held : detail::source {
    using type = X;
};

/// A value read in another region than its own through an accumulator of
/// capacity N: the window of the values written since the reader's region
/// last began (flow/window.hpp).
template <class X, std::size_t N>
struct accumulated : detail::source {
    using type = window_t<X, N>;
};

/// The input whose time a time handler takes.
struct time_of {
    std::string input;
};

/// A composite or an operator instance, of binding B.
template <class B>
struct node {
    using binding = B;
    std::size_t index = 0;
};

template <class X>
[[nodiscard]] held<X> hold(const value<X>& v) {
    held<X> h{static_cast<const detail::source&>(v)};
    h.ref.rate = rate_rule::hold;
    h.ref.capacity = 0;
    h.accumulator = nullptr;
    return h;
}

/// `v`, read in another region than its own as the values written since that
/// region last began: the latest N of them, oldest first, and how many older
/// ones didn't fit. Each accumulate edge has an accumulator of its own.
template <std::size_t N, class X>
[[nodiscard]] accumulated<X, N> accumulate(const value<X>& v) {
    static_assert(N >= 1, "G7: an accumulator holds at least one value");
    accumulated<X, N> a{static_cast<const detail::source&>(v)};
    a.ref.rate = rate_rule::accumulate;
    a.ref.capacity = N;
    if constexpr (!is_data_v<X>) {
        a.accumulator = &detail::accumulator_ops_v<X, N>;
    }
    return a;
}

template <class M>
[[nodiscard]] time_of at(const input_port<M>& in) {
    return time_of{in.ref.port};
}

/// The field `F` of a record value that only describes (X is a domain).
template <fixed_string F, class X>
[[nodiscard]] auto project(const value<X>& v) {
    static_assert(is_data_v<X>, "a projection of a message names its member: project<\"f\", &M::f>(x)");
    if constexpr (is_data_v<X>) {
        value<field_t<X, F>> out{static_cast<const detail::source&>(v)};
        out.ref.path.emplace_back(F.view());
        return out;
    }
}

/// The field `F` of a message, as its member `Member` (G4: the member's
/// domain is field_t of the message's record).
template <fixed_string F, auto Member, class M>
[[nodiscard]] auto project(const value<M>& v) {
    static_assert(std::is_member_object_pointer_v<decltype(Member)>, "project<\"f\", &M::f>: a member of M");
    using C = std::remove_cvref_t<decltype(std::declval<const M&>().*Member)>;
    static_assert(std::is_same_v<data_t<C>, field_t<data_t<M>, F>>,
                  "G4: a projection x.f is field_t of x's record, and the member's domain is it");
    value<C> out{static_cast<const detail::source&>(v)};
    out.ref.path.emplace_back(F.view());
    out.access.push_back(+[](const void* p) -> const void* { return &(static_cast<const M*>(p)->*Member); });
    return out;
}

namespace detail {
template <class X>
X source_type(const value<X>&);
template <class X>
X source_type(const held<X>&);
template <class X, std::size_t N>
window_t<X, N> source_type(const accumulated<X, N>&);

template <class S>
concept source_like = requires(const S& s) { source_type(s); };

template <class S>
using source_type_t = decltype(source_type(std::declval<const S&>()));

template <class S>
[[nodiscard]] source source_of(const S& s) {
    return static_cast<const source&>(s);
}

/// A handler, as the builder records it.
struct handler_info {
    std::string name;
    std::vector<std::string> in;
    std::vector<std::string> out;
    bool timed = false;
    state_role state = state_role::none;
};

template <class H>
[[nodiscard]] handler_info info_of() {
    return handler_info{std::string(H::name),
                        std::vector<std::string>(H::in::names.begin(), H::in::names.end()),
                        std::vector<std::string>(H::out::names.begin(), H::out::names.end()),
                        H::timed,
                        H::state};
}

template <class B>
[[nodiscard]] std::vector<handler_info> handlers_of() {
    using list = typename checked_binding<B>::type::handlers::types;
    return [&]<std::size_t... I>(std::index_sequence<I...>) {
        return std::vector<handler_info>{info_of<std::tuple_element_t<I, list>>()...};
    }(std::make_index_sequence<std::tuple_size_v<list>>{});
}

template <class Types>
[[nodiscard]] auto values_of(std::span<const source> sources) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) {
        return std::tuple<value<std::tuple_element_t<I, Types>>...>{
            value<std::tuple_element_t<I, Types>>{sources[I]}...};
    }(std::make_index_sequence<std::tuple_size_v<Types>>{});
}

template <class... X>
[[nodiscard]] std::vector<source> sources_of(const std::tuple<value<X>...>& values) {
    return std::apply([](const auto&... v) { return std::vector<source>{source_of(v)...}; }, values);
}

/// G4: each source has its port's domain and, when both have one, its message type.
template <class Ports, class... S>
[[nodiscard]] constexpr bool edges_match() noexcept {
    if constexpr (std::tuple_size_v<Ports> != sizeof...(S)) {
        return true;  // G3 reports it
    } else {
        return []<std::size_t... I>(std::index_sequence<I...>) {
            return (std::is_same_v<data_t<std::tuple_element_t<I, Ports>>,
                                   data_t<source_type_t<std::tuple_element_t<I, std::tuple<S...>>>>> &&
                    ...);
        }(std::index_sequence_for<S...>{});
    }
}

template <class P, class S>
inline constexpr bool same_message_v = is_data_v<P> || is_data_v<S> || std::is_same_v<P, S>;

template <class Ports, class... S>
[[nodiscard]] constexpr bool messages_match() noexcept {
    if constexpr (std::tuple_size_v<Ports> != sizeof...(S)) {
        return true;
    } else {
        return []<std::size_t... I>(std::index_sequence<I...>) {
            return (same_message_v<std::tuple_element_t<I, Ports>,
                                   source_type_t<std::tuple_element_t<I, std::tuple<S...>>>> &&
                    ...);
        }(std::index_sequence_for<S...>{});
    }
}

// ── What the stack runs ─────────────────────────────────────────────

/// A firing, as the stack runs it: its operator, its in ports' messages, its
/// out ports' messages, and the time it takes.
using thunk_fn = void (*)(void* op, const void* const* in, void* const* out, Timestamp at);

/// A new slot for a message; null for a port that carries none.
using slot_factory = std::shared_ptr<void> (*)();

template <class X>
[[nodiscard]] std::shared_ptr<void> new_slot() {
    return std::make_shared<X>();
}

template <class X>
[[nodiscard]] constexpr slot_factory factory_of() noexcept {
    if constexpr (is_data_v<X>) {
        return nullptr;
    } else {
        return &new_slot<X>;
    }
}

template <class Types>
[[nodiscard]] std::vector<slot_factory> factories_of() {
    return []<std::size_t... I>(std::index_sequence<I...>) {
        return std::vector<slot_factory>{factory_of<std::tuple_element_t<I, Types>>()...};
    }(std::make_index_sequence<std::tuple_size_v<Types>>{});
}

/// Each port's message type (or domain), for an observer to encode it by.
template <class Types>
[[nodiscard]] std::vector<std::type_index> types_of() {
    return []<std::size_t... I>(std::index_sequence<I...>) {
        return std::vector<std::type_index>{std::type_index(typeid(std::tuple_element_t<I, Types>))...};
    }(std::make_index_sequence<std::tuple_size_v<Types>>{});
}

template <class Op, class H, class Ins>
struct callable;
template <class Op, class H, class... In>
struct callable<Op, H, std::tuple<In...>> {
    static constexpr bool value = H::timed ? requires(Op& o, Timestamp t, const In&... in) { H::call(o, t, in...); }
                                           : requires(Op& o, const In&... in) { H::call(o, in...); };
};

/// The thunk that fires handler H of an operator of type Op.
template <class Op, class Sig, class H>
[[nodiscard]] thunk_fn thunk_of() {
    using in_types = typename port_values<Sig, typename H::in>::type;
    using out_types = typename port_values<Sig, typename H::out>::type;
    static_assert(
        []<std::size_t... I>(std::index_sequence<I...>) {
            return (!is_data_v<std::tuple_element_t<I, in_types>> && ...);
        }(std::make_index_sequence<std::tuple_size_v<in_types>>{}) &&
            []<std::size_t... I>(std::index_sequence<I...>) {
                return (!is_data_v<std::tuple_element_t<I, out_types>> && ...);
            }(std::make_index_sequence<std::tuple_size_v<out_types>>{}),
        "the CPU stack runs a handler whose ports carry message types");
    static_assert(callable<Op, H, in_types>::value,
                  "G2: a handler's call() takes its operator, its time if it takes one, and its in ports' messages");
    if constexpr (callable<Op, H, in_types>::value) {
        return [](void* op,
                  [[maybe_unused]] const void* const* in,
                  [[maybe_unused]] void* const* out,
                  [[maybe_unused]] Timestamp at) {
            Op& o = *static_cast<Op*>(op);
            [&]<std::size_t... I>(std::index_sequence<I...>) {
                const auto call = [&]() -> decltype(auto) {
                    if constexpr (H::timed) {
                        return H::call(o, at, *static_cast<const std::tuple_element_t<I, in_types>*>(in[I])...);
                    } else {
                        return H::call(o, *static_cast<const std::tuple_element_t<I, in_types>*>(in[I])...);
                    }
                };
                constexpr std::size_t n = std::tuple_size_v<out_types>;
                if constexpr (n == 0) {
                    call();
                } else if constexpr (n == 1) {
                    *static_cast<std::tuple_element_t<0, out_types>*>(out[0]) = call();
                } else {
                    auto r = call();
                    [&]<std::size_t... J>(std::index_sequence<J...>) {
                        ((*static_cast<std::tuple_element_t<J, out_types>*>(out[J]) = std::get<J>(std::move(r))), ...);
                    }(std::make_index_sequence<n>{});
                }
            }(std::make_index_sequence<std::tuple_size_v<in_types>>{});
        };
    } else {
        return nullptr;
    }
}

template <class Op, class B>
[[nodiscard]] std::vector<thunk_fn> thunks_of() {
    using list = typename B::handlers::types;
    return [&]<std::size_t... I>(std::index_sequence<I...>) {
        return std::vector<thunk_fn>{thunk_of<Op, typename B::signature, std::tuple_element_t<I, list>>()...};
    }(std::make_index_sequence<std::tuple_size_v<list>>{});
}

/// An input's time, from its message: the accessor given, the tick's own
/// time, or the message's member `t`.
template <class M>
[[nodiscard]] std::function<Timestamp(const void*)> time_fn_of(Timestamp (*time)(const M&)) {
    if (time != nullptr) {
        return [time](const void* m) { return time(*static_cast<const M*>(m)); };
    }
    if constexpr (std::is_same_v<M, Timestamp>) {
        return [](const void* m) { return *static_cast<const Timestamp*>(m); };
    } else if constexpr (requires(const M& m) {
                             { m.t } -> std::convertible_to<Timestamp>;
                         }) {
        return [](const void* m) { return Timestamp{static_cast<const M*>(m)->t}; };
    } else {
        return nullptr;
    }
}
}  // namespace detail

// ── The builder ─────────────────────────────────────────────────────

class builder;
class stack;

/// One firing, as the stack hands it to its observer, once the firing has
/// written its out ports: where it is in the graph, the time of the input
/// that ran its region, and each out port's name, message type and message.
/// The spans are valid during the call only.
struct fired {
    std::size_t region = 0;  ///< its region's index in graph().regions
    std::size_t firing = 0;  ///< its index in the region's firings
    Timestamp t;             ///< the region's input's time, or zero if the input has none
    std::span<const std::string> ports;
    std::span<const std::type_index> types;
    std::span<void* const> messages;
};

/// What a stack calls after each firing, if it is given one (stack::observe).
using observer = std::function<void(const fired&)>;

/// The region a firing goes into; in a body, also the composite handler the
/// firing implements.
class scope {
public:
    [[nodiscard]] builder& owner() const noexcept {
        return *owner_;
    }

private:
    friend class builder;
    scope(builder& owner, std::size_t region) : owner_(&owner), region_(region) {}

    builder* owner_;
    std::size_t region_;
    std::string composite_;  ///< `c.h`, in a body; else empty
    std::string outer_;      ///< the composite firing a body's firing expands, in a body; else empty
    state_role state_ = state_role::none;
};

/// What a body's handler is, once its types are checked: given its scope,
/// the time it takes (or null), and its composite handler's sources, it fires
/// the body's operators and returns the composite handler's out ports.
using body_handler =
    std::function<std::vector<detail::source>(scope&, const time_of*, std::span<const detail::source>)>;

template <class M>
struct input_options {
    std::string trigger;                    ///< empty: the input's own name
    Timestamp (*time)(const M&) = nullptr;  ///< empty: the tick's own time, or the message's member `t`
    bool sitl_only = false;
};

/// Records a graph as it is described, checking it as it goes.
///
/// Invariant: the first invariant broken, if any, is kept, and build()
/// returns it; every call after it still returns a handle, so a description
/// runs to its end.
///
/// Pre, for every call: the handles it is given were returned by this builder.
class builder {
public:
    builder(std::string mission, baseline_class kind) : builder(std::move(mission), kind, false) {}
    builder(const builder&) = delete;
    builder& operator=(const builder&) = delete;

    /// The tick: an input that carries only its time, and fires its own trigger.
    [[nodiscard]] input_port<Timestamp> tick(std::string name);

    /// The region of `trigger`'s trigger: one per trigger (G6).
    template <class M>
    [[nodiscard]] scope region(const input_port<M>& trigger) {
        return region_of(trigger.ref.port);
    }

    /// Output `out` is written by `v`, a firing of the output's region (G3, G6).
    template <class M, class X>
    void produce(const output_port<M>& out, const value<X>& v) {
        static_assert(std::is_same_v<data_t<M>, data_t<X>> && detail::same_message_v<M, X>,
                      "G4: every edge joins two ports of the same domain, and of one message type");
        produce_source(out.index, v);
    }

    /// The checked graph, or the first invariant it breaks.
    [[nodiscard]] result<Graph, config::ConfigError> build() const;

    /// The CPU stack that runs the checked graph, which takes the builder's
    /// operators and slots: a builder builds one stack. Pre: a stack_builder.
    [[nodiscard]] result<stack, config::ConfigError> build_stack() &&;

    // ── The typed functions' entry points (below); not for missions ──

    struct input_ref {
        detail::source source;
        std::size_t index = 0;
    };
    [[nodiscard]] input_ref add_input(std::string name,
                                      bool message,
                                      std::string trigger,
                                      bool sitl_only,
                                      std::shared_ptr<void> slot,
                                      std::function<Timestamp(const void*)> time);
    [[nodiscard]] detail::source add_constant(std::string name, std::shared_ptr<void> slot);
    [[nodiscard]] std::size_t add_output(std::string name, const Ref& trigger);
    [[nodiscard]] std::size_t add_composite(std::string name,
                                            std::string_view body,
                                            std::vector<detail::handler_info> handlers,
                                            bool of_one_operator);
    [[nodiscard]] std::size_t
    add_instance(std::string name, std::string_view op, std::vector<detail::handler_info> handlers);
    [[nodiscard]] std::size_t add_body_instance(std::size_t composite,
                                                std::string local,
                                                std::string_view op,
                                                std::vector<detail::handler_info> handlers);
    /// The operator node `node` fires, and the thunk of each of its handlers.
    void
    set_operator(std::size_t node, result<std::shared_ptr<void>, std::string> op, std::vector<detail::thunk_fn> thunks);
    void add_body_handler(std::size_t composite, std::string_view handler, body_handler fn);
    void end_body(std::size_t composite);
    [[nodiscard]] std::vector<detail::source> fire(scope& s,
                                                   std::size_t node,
                                                   std::string_view handler,
                                                   const time_of* at,
                                                   std::vector<detail::source> in,
                                                   std::span<const detail::slot_factory> outs,
                                                   std::span<const std::type_index> types);

protected:
    builder(std::string mission, baseline_class kind, bool runs);

private:
    struct input_rec {
        std::string name;
        bool message = true;
        std::string trigger;
        bool sitl_only = false;
        std::size_t slot = 0;
        std::function<Timestamp(const void*)> time;
    };
    struct output_rec {
        std::string name;
        std::string region;
        bool produced = false;
        detail::source from;
    };
    struct node_rec {
        std::string name;
        std::vector<detail::handler_info> handlers;
        std::vector<bool> fired;   ///< per handler
        bool composite = false;    ///< a composite: its firings are its body's
        bool must_fire = false;    ///< every handler fires in a region (G6)
        std::size_t instance = 0;  ///< the instance it fires, unless its body has several operators
        bool of_one_operator = true;
        std::vector<body_handler> body;  ///< per handler, a body of several operators
        std::string body_name;
        std::vector<detail::thunk_fn> thunks;  ///< per handler, to run
    };
    /// A firing, as the stack will run it.
    struct firing_rec {
        std::size_t instance = 0;
        detail::thunk_fn thunk = nullptr;
        std::vector<detail::source> in;
        std::vector<std::size_t> out;      ///< slots
        std::size_t at = detail::no_slot;  ///< the input whose time it takes
        std::vector<std::string> ports;    ///< the out ports' names
        std::vector<std::type_index> types;
    };

    [[nodiscard]] scope region_of(const std::string& input);
    void produce_source(std::size_t out, const detail::source& from);
    void fail(std::string key, std::string message);
    void check_name(const std::string& key, const std::string& name, std::string_view what);
    void check_source(const Ref& ref, const std::string& trigger, const std::string& key);
    [[nodiscard]] const input_rec* find_input(std::string_view name) const;
    [[nodiscard]] std::size_t add_slot(std::shared_ptr<void> slot);

    /// An accumulate edge, as the stack will run it.
    struct accumulator_rec {
        detail::source from;     ///< the producer's value, as the edge reads it
        std::size_t region = 0;  ///< the reader's region
        std::size_t acc = 0;     ///< slots: the accumulator, and the window the reader reads
        std::size_t window = 0;
        const detail::accumulator_ops* ops = nullptr;
    };

    Graph graph_;
    bool runs_ = false;     ///< a stack_builder's: the graph is also run
    bool stacked_ = false;  ///< build_stack() has taken the operators and slots
    std::vector<input_rec> inputs_;
    std::vector<std::string> constants_;
    std::vector<output_rec> outputs_;
    std::vector<node_rec> nodes_;
    std::vector<std::string> names_;                 ///< nodes and external ports: one namespace (G1)
    std::vector<std::string> fired_;                 ///< `instance.handler@at`, once each (G6)
    std::vector<std::vector<std::size_t>> updated_;  ///< per region, the instances it updates (G5)
    std::vector<std::shared_ptr<void>> slots_;       ///< every input's, constant's and out port's message
    std::vector<std::shared_ptr<void>> operators_;   ///< per instance
    std::vector<std::vector<firing_rec>> runs_of_;   ///< per region, its firings, to run
    std::vector<accumulator_rec> accumulators_;      ///< one per accumulate edge
    std::optional<config::ConfigError> error_;
};

/// A builder that also builds each operator from the mission's context Ctx
/// (make_slot), so build_stack() returns the stack that runs the graph.
template <class Ctx>
class stack_builder : public builder {
public:
    stack_builder(std::string mission, baseline_class kind, const Ctx& ctx)
        : builder(std::move(mission), kind, true), ctx_(&ctx) {}

    [[nodiscard]] const Ctx& context() const noexcept {
        return *ctx_;
    }

private:
    const Ctx* ctx_;
};

namespace detail {
template <class G>
concept describes = std::derived_from<G, builder>;

template <class G>
concept runs = describes<G> && requires(const G& g) { g.context(); };

/// Build operator Op of node `n` from g's context, with its binding's thunks.
template <class Op, class B, class G>
void build_operator(G& g, std::size_t n) {
    if constexpr (runs<G>) {
        auto made = make_slot<Op>(g.context());
        result<std::shared_ptr<void>, std::string> op =
            made ? result<std::shared_ptr<void>, std::string>(
                       std::shared_ptr<void>(std::make_shared<Op>(std::move(made).value())))
                 : result<std::shared_ptr<void>, std::string>(made.error());
        g.set_operator(n, std::move(op), thunks_of<Op, B>());
    } else {
        (void)g;
        (void)n;
    }
}
}  // namespace detail

/// The CPU stack: the checked graph's operators and the values they read and
/// write, fired region by region in the checked order. Built by
/// stack_builder::build_stack(); it allocates nothing and throws nothing once
/// built, and calls no virtual function: each firing is one indirect call.
///
/// A message that holds a pointer it doesn't own (a camera frame) is valid
/// only while its region runs. Every input's and out port's message is
/// value-initialized when the stack is built, so a hold, or an output, read
/// before its region first runs reads that.
///
/// An accumulate edge's accumulator takes a copy of each value its producer
/// writes: an input's when it is handed, a firing's out port's after the
/// firing. As the reader's region begins, the stack writes the window the
/// reader reads, and empties the accumulator, so every firing of the region
/// reads one window. Before the producer first writes, the window is empty.
///
/// An observer, if it is given one, is called after each firing with what the
/// firing wrote (docs/plan/flow-testing, T4: flow/trace.hpp writes it to a
/// trace). With none, the stack runs as it does without: one test per firing.
class stack {
public:
    stack(stack&&) noexcept = default;
    stack& operator=(stack&&) noexcept = default;
    stack(const stack&) = delete;
    stack& operator=(const stack&) = delete;

    /// Hand the stack input `in`'s message, and run its trigger's region, if it has one.
    template <class M>
    void input(const input_port<M>& in, const M& m) {
        const input_rec& r = inputs_[in.index];
        *static_cast<M*>(r.slot) = m;
        for (const push& p : r.pushes) {
            p.fn(p.acc, p.from);
        }
        if (r.region != detail::no_slot) {
            run(r.region, observer_ && r.time ? r.time(r.slot) : Timestamp{});
        }
    }

    /// Call `o` after each firing from now on; an empty `o` stops observing.
    void observe(observer o) {
        observer_ = std::move(o);
    }

    /// Output `out`'s message, as its region last wrote it.
    template <class M>
    [[nodiscard]] const M& output(const output_port<M>& out) const noexcept {
        return *static_cast<const M*>(outputs_[out.index]);
    }

    [[nodiscard]] const Graph& graph() const noexcept {
        return graph_;
    }
    /// graph_hash(graph()).
    [[nodiscard]] const std::string& hash() const noexcept {
        return hash_;
    }

private:
    friend class builder;
    stack() = default;

    /// A value written, into an accumulator that an accumulate edge reads.
    struct push {
        void (*fn)(void*, const void*) = nullptr;
        void* acc = nullptr;
        const void* from = nullptr;
    };
    /// An accumulator's window, written as its reader's region begins.
    struct latch {
        void (*fn)(void*, void*) = nullptr;
        void* acc = nullptr;
        void* window = nullptr;
    };
    struct firing {
        detail::thunk_fn thunk = nullptr;
        void* op = nullptr;
        std::vector<const void*> in;
        std::vector<void*> out;
        std::size_t at = detail::no_slot;  ///< the input whose time it takes
        std::vector<std::string> ports;    ///< the out ports' names
        std::vector<std::type_index> types;
        std::vector<push> pushes;  ///< the accumulators its out ports feed
    };
    struct input_rec {
        void* slot = nullptr;
        std::size_t region = detail::no_slot;
        std::function<Timestamp(const void*)> time;
        std::vector<push> pushes;  ///< the accumulators it feeds
    };

    /// Run a region; `now` is its input's time, for the observer.
    void run(std::size_t region, Timestamp now) {
        for (const latch& l : latches_[region]) {
            l.fn(l.acc, l.window);
        }
        const std::vector<firing>& fs = regions_[region];
        for (std::size_t i = 0; i < fs.size(); ++i) {
            const firing& f = fs[i];
            const Timestamp t = f.at == detail::no_slot ? Timestamp{} : inputs_[f.at].time(inputs_[f.at].slot);
            f.thunk(f.op, f.in.data(), f.out.data(), t);
            for (const push& p : f.pushes) {
                p.fn(p.acc, p.from);
            }
            if (observer_) {
                observer_(fired{region, i, now, f.ports, f.types, f.out});
            }
        }
    }

    std::vector<std::shared_ptr<void>> slots_;
    std::vector<std::shared_ptr<void>> operators_;
    std::vector<std::vector<firing>> regions_;
    std::vector<std::vector<latch>> latches_;  ///< per region, the windows it reads
    std::vector<input_rec> inputs_;
    std::vector<const void*> outputs_;
    Graph graph_;
    std::string hash_;
    observer observer_;
};

// ── The typed functions ─────────────────────────────────────────────

template <class M>
[[nodiscard]] input_port<M> input(builder& g, std::string name, input_options<M> options = {}) {
    std::string trigger = options.trigger.empty() ? name : std::move(options.trigger);
    auto in = g.add_input(std::move(name),
                          true,
                          std::move(trigger),
                          options.sitl_only,
                          std::make_shared<M>(),
                          detail::time_fn_of<M>(options.time));
    return input_port<M>{{in.source}, in.index};
}

template <class M>
[[nodiscard]] constant_port<M> constant(builder& g, std::string name, M value = {}) {
    return constant_port<M>{{g.add_constant(std::move(name), std::make_shared<M>(std::move(value)))}};
}

/// An output written in `trigger`'s region.
template <class M, class N>
[[nodiscard]] output_port<M> output(builder& g, std::string name, const input_port<N>& trigger) {
    return output_port<M>{g.add_output(std::move(name), trigger.ref)};
}

/// What a body of several operators is given to describe itself.
template <class B, class G = builder>
class body {
public:
    using binding = B;
    body(G& g, std::size_t composite) : owner_(&g), composite_(composite) {}

    [[nodiscard]] G& owner() const noexcept {
        return *owner_;
    }
    [[nodiscard]] std::size_t composite() const noexcept {
        return composite_;
    }

private:
    G* owner_;
    std::size_t composite_;
};

namespace detail {
template <class Body, class G>
concept body_type = requires(body<typename Body::binding, G>& b) {
    { Body::name } -> std::convertible_to<std::string_view>;
    Body::build(b);
};
}  // namespace detail

/// A composite of binding B, filled by Body: a registered operator of B, or
/// a body of several operators. In a scoreboard graph only (G8).
template <binding B, class Body, detail::describes G>
[[nodiscard]] node<B> composite(G& g, std::string name) {
    static_assert(is_registered_v<Body> || detail::body_type<Body, G>,
                  "G2: a composite's body is a registered operator, or a body of several");
    static_assert(std::is_same_v<typename Body::binding, B>, "G2: a body fills its composite: it has its binding");
    if constexpr (is_registered_v<Body>) {
        const std::size_t c = g.add_composite(std::move(name), Body::name, detail::handlers_of<B>(), true);
        detail::build_operator<typename Body::type, B>(g, c);
        return node<B>{c};
    } else {
        static_assert(detail::is_operator_name(Body::name), "G1: a body's name matches [A-Za-z0-9_]+");
        const std::size_t c = g.add_composite(std::move(name), Body::name, detail::handlers_of<B>(), false);
        body<B, G> b{g, c};
        Body::build(b);
        g.end_body(c);
        return node<B>{c};
    }
}

/// An operator instance of a research graph (G8).
template <class Op, detail::describes G>
[[nodiscard]] node<typename Op::binding> instance(G& g, std::string name) {
    static_assert(is_registered_v<Op>, "G2: an instance is of a registered operator");
    using B = typename Op::binding;
    const std::size_t n = g.add_instance(std::move(name), Op::name, detail::handlers_of<B>());
    detail::build_operator<typename Op::type, B>(g, n);
    return node<B>{n};
}

/// An operator instance of a body, named `<composite>.<name>`.
template <class Op, class B, class G>
[[nodiscard]] node<typename Op::binding> instance(body<B, G>& b, std::string name) {
    static_assert(is_registered_v<Op>, "G2: an instance is of a registered operator");
    using OB = typename Op::binding;
    const std::size_t n =
        b.owner().add_body_instance(b.composite(), std::move(name), Op::name, detail::handlers_of<OB>());
    detail::build_operator<typename Op::type, OB>(b.owner(), n);
    return node<OB>{n};
}

namespace detail {
template <class H, class B, class... S>
[[nodiscard]] auto fire(scope& s, std::size_t node, const time_of* at, const S&... sources) {
    using sig = typename B::signature;
    using in_types = typename port_values<sig, typename H::in>::type;
    using out_types = typename port_values<sig, typename H::out>::type;
    static_assert((source_like<S> && ...),
                  "G3: a source is a value, an input, a constant, or hold(...) or accumulate<N>(...) of one");
    static_assert(sizeof...(S) == H::in::size, "G3: every in port of the handler has one source");
    static_assert(edges_match<in_types, S...>(), "G4: every edge joins two ports of the same domain");
    static_assert(messages_match<in_types, S...>(),
                  "G4: an edge joins two ports of one message type, when both have one");
    static const std::vector<slot_factory> factories = factories_of<out_types>();
    static const std::vector<std::type_index> types = types_of<out_types>();
    const std::vector<source> outs =
        s.owner().fire(s, node, H::name, at, std::vector<source>{source_of(sources)...}, factories, types);
    return values_of<out_types>(outs);
}
}  // namespace detail

/// Fire handler H of node `n` in `s`, one source per in port, in its order.
/// Returns the handler's out ports, in its order: auto [command] = fire<...>(...).
template <fixed_string H, class B, class... S>
auto fire(scope& s, const node<B>& n, const S&... sources) {
    using h = handler_t<B, H>;
    static_assert(!h::timed, "G3: a handler that takes a time is fired at(<input>), its first argument");
    return detail::fire<h, B>(s, n.index, nullptr, sources...);
}

/// Fire time handler H of node `n` in `s`, at the time of input `t`.
template <fixed_string H, class B, class... S>
auto fire(scope& s, const node<B>& n, const time_of& t, const S&... sources) {
    using h = handler_t<B, H>;
    static_assert(h::timed, "G3: a handler that takes no time is not fired at an input");
    return detail::fire<h, B>(s, n.index, &t, sources...);
}

/// A body's implementation of its composite's handler H: `f(scope&, [time_of,]
/// value<X>...)`, one value per in port, returns std::tuple of the handler's
/// out ports' values, in its order.
template <fixed_string H, class B, class G, class F>
void on(body<B, G>& b, F f) {
    using h = handler_t<B, H>;
    using sig = typename B::signature;
    using in_types = typename detail::port_values<sig, typename h::in>::type;
    using out_types = typename detail::port_values<sig, typename h::out>::type;
    using expected = decltype(detail::values_of<out_types>(std::span<const detail::source>{}));
    const auto call =
        [f = std::move(f)](scope& s, [[maybe_unused]] const time_of* t, std::span<const detail::source> in) {
            auto args = detail::values_of<in_types>(in);
            if constexpr (h::timed) {
                return std::apply([&](const auto&... v) { return f(s, *t, v...); }, args);
            } else {
                return std::apply([&](const auto&... v) { return f(s, v...); }, args);
            }
        };
    static_assert(
        std::is_same_v<std::invoke_result_t<decltype(call), scope&, const time_of*, std::span<const detail::source>>,
                       expected>,
        "G4: a body's handler returns its composite handler's out ports, of their domains and message types");
    b.owner().add_body_handler(
        b.composite(), h::name, [call](scope& s, const time_of* t, std::span<const detail::source> in) {
            return detail::sources_of(call(s, t, in));
        });
}

}  // namespace branes::anatomy::flow
