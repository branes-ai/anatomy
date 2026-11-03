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
///   handler<Name, ins<...>, outs<...>, State, Time> : one function of a binding
///   a binding B            : B::signature (step 1) and B::handlers
///   registered<Name, T, B> : an operator, its C++ type and its binding
///   a body                 : Body::name, Body::binding, Body::build(body<B>&)
///   value<D>               : a value of domain D: an input, a constant, or an out port of a firing
///   node<B>                : a composite or an operator instance, of binding B
///   scope                  : the region a firing goes into
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
#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/signature.hpp>
#include <branes/anatomy/result.hpp>

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
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
/// an operator of its own. It gives its signature (step 1) and its handlers;
/// the CPU stack (2b) adds one static function per handler.
template <class B>
concept binding = requires {
    typename B::signature;
    typename B::handlers;
} && is_signature_v<typename B::signature>;

namespace detail {
template <binding B>
struct checked_binding {
    static_assert(handlers_fit<typename B::signature>(static_cast<typename B::handlers::types*>(nullptr)),
                  "G2: a handler's in ports are in or inout ports of its signature, and its out ports out or inout");
    using type = B;
};

template <class B, fixed_string Name>
struct find_handler {
    using list = typename checked_binding<B>::type::handlers;
    static constexpr std::size_t index = index_of(list::names, Name.view());
    static_assert(index < list::size, "G2: a firing names a handler its node declares");
    using type = std::tuple_element_t<(index < list::size ? index : 0), typename list::types>;
};

template <class Sig, class Names>
struct port_domains;
template <class Sig, fixed_string... N>
struct port_domains<Sig, ins<N...>> {
    using type = std::tuple<port_t<Sig, N>...>;
};
template <class Sig, fixed_string... N>
struct port_domains<Sig, outs<N...>> {
    using type = std::tuple<port_t<Sig, N>...>;
};
}  // namespace detail

/// A binding's handler of that name.
template <class B, fixed_string Name>
using handler_t = typename detail::find_handler<B, Name>::type;

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

/// A value of domain D: an input, a constant, or an out port of a firing,
/// with the record fields projected from it.
template <class D>
struct value {
    static_assert(is_data_v<D>, "a value holds a domain or a record");
    using domain = D;
    Ref ref;
};

/// An external input: a message M the stack is handed, which fires its trigger.
template <class M>
struct input_port : value<domain_of_t<M>> {
    using message = M;
};

/// A constant: a message M fixed when the stack is built.
template <class M>
struct constant_port : value<domain_of_t<M>> {
    using message = M;
};

/// An external output: a message M the stack returns from a region.
template <class M>
struct output_port {
    using message = M;
    std::size_t index = 0;
};

/// A value read in another region than its own: the latest value written.
template <class D>
struct held {
    using domain = D;
    Ref ref;
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

template <class D>
[[nodiscard]] held<D> hold(const value<D>& v) {
    held<D> h{v.ref};
    h.ref.rate = rate_rule::hold;
    return h;
}

template <class M>
[[nodiscard]] time_of at(const input_port<M>& in) {
    return time_of{in.ref.port};
}

/// The field `F` of a record value (G4: its domain is field_t of the record).
template <fixed_string F, class D>
[[nodiscard]] value<field_t<D, F>> project(const value<D>& v) {
    value<field_t<D, F>> out{v.ref};
    out.ref.path.emplace_back(F.view());
    return out;
}

namespace detail {
template <class D>
D source_domain(const value<D>&);
template <class D>
D source_domain(const held<D>&);

template <class S>
concept source = requires(const S& s) { source_domain(s); };

template <class S>
using source_domain_t = decltype(source_domain(std::declval<const S&>()));

template <class D>
[[nodiscard]] Ref ref_of(const value<D>& v) {
    return v.ref;
}
template <class D>
[[nodiscard]] Ref ref_of(const held<D>& h) {
    return h.ref;
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

template <class Domains>
[[nodiscard]] auto values_of(std::span<const Ref> refs) {
    return [&]<std::size_t... I>(std::index_sequence<I...>) {
        return std::tuple<value<std::tuple_element_t<I, Domains>>...>{
            value<std::tuple_element_t<I, Domains>>{refs[I]}...};
    }(std::make_index_sequence<std::tuple_size_v<Domains>>{});
}

template <class... D>
[[nodiscard]] std::vector<Ref> refs_of(const std::tuple<value<D>...>& values) {
    return std::apply([](const auto&... v) { return std::vector<Ref>{v.ref...}; }, values);
}

template <class Domains, class... S>
[[nodiscard]] constexpr bool domains_match() noexcept {
    if constexpr (std::tuple_size_v<Domains> != sizeof...(S)) {
        return true;  // G3 reports it
    } else {
        return []<std::size_t... I>(std::index_sequence<I...>) {
            return (std::is_same_v<std::tuple_element_t<I, Domains>,
                                   source_domain_t<std::tuple_element_t<I, std::tuple<S...>>>> &&
                    ...);
        }(std::index_sequence_for<S...>{});
    }
}
}  // namespace detail

// ── The builder ─────────────────────────────────────────────────────

class builder;

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
    state_role state_ = state_role::none;
};

/// What a body's handler is, once its types are checked: given its scope,
/// the time it takes (or null), and its composite handler's sources, it fires
/// the body's operators and returns the composite handler's out ports.
using body_handler = std::function<std::vector<Ref>(scope&, const time_of*, std::span<const Ref>)>;

struct input_options {
    std::string trigger;  ///< empty: the input's own name
    std::string time;     ///< the message member holding its time; empty: `t`
    bool sitl_only = false;
};

struct constant_options {
    std::string from;  ///< what the stack is built from; empty: a default value
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
    builder(std::string mission, baseline_class kind);
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
    template <class M, class D>
    void produce(const output_port<M>& out, const value<D>& v) {
        static_assert(std::is_same_v<domain_of_t<M>, D>, "G4: every edge joins two ports of the same domain");
        produce_ref(out.index, v.ref);
    }

    /// The checked graph, or the first invariant it breaks.
    [[nodiscard]] result<Graph, config::ConfigError> build() const;

    // ── The typed functions' entry points (below); not for missions ──

    [[nodiscard]] Ref add_input(std::string name, bool message, input_options options);
    [[nodiscard]] Ref add_constant(std::string name, constant_options options);
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
    void add_body_handler(std::size_t composite, std::string_view handler, body_handler fn);
    void end_body(std::size_t composite);
    [[nodiscard]] std::vector<Ref>
    fire(scope& s, std::size_t node, std::string_view handler, const time_of* at, std::vector<Ref> in);

private:
    struct input_rec {
        std::string name;
        bool message = true;
        std::string trigger;
        std::string time;  ///< for the CPU stack (2b)
        bool sitl_only = false;
    };
    struct constant_rec {
        std::string name;
        std::string from;  ///< for the CPU stack (2b)
    };
    struct output_rec {
        std::string name;
        std::string region;
        bool produced = false;
        Ref from;
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
    };

    [[nodiscard]] scope region_of(const std::string& input);
    void produce_ref(std::size_t out, const Ref& from);
    void fail(std::string key, std::string message);
    void check_name(const std::string& key, const std::string& name, std::string_view what);
    void check_source(const Ref& ref, const std::string& trigger, const std::string& key);
    [[nodiscard]] const input_rec* find_input(std::string_view name) const;
    [[nodiscard]] static std::vector<Ref> no_refs(std::size_t n);

    Graph graph_;
    std::vector<input_rec> inputs_;
    std::vector<constant_rec> constants_;
    std::vector<output_rec> outputs_;
    std::vector<node_rec> nodes_;
    std::vector<std::string> names_;                 ///< nodes and external ports: one namespace (G1)
    std::vector<std::string> fired_;                 ///< `instance.handler@at`, once each (G6)
    std::vector<std::vector<std::size_t>> updated_;  ///< per region, the instances it updates (G5)
    std::optional<config::ConfigError> error_;
};

// ── The typed functions ─────────────────────────────────────────────

template <class M>
[[nodiscard]] input_port<M> input(builder& g, std::string name, input_options options = {}) {
    return input_port<M>{{g.add_input(std::move(name), true, std::move(options))}};
}

template <class M>
[[nodiscard]] constant_port<M> constant(builder& g, std::string name, constant_options options = {}) {
    return constant_port<M>{{g.add_constant(std::move(name), std::move(options))}};
}

/// An output written in `trigger`'s region.
template <class M, class N>
[[nodiscard]] output_port<M> output(builder& g, std::string name, const input_port<N>& trigger) {
    return output_port<M>{g.add_output(std::move(name), trigger.ref)};
}

/// What a body of several operators is given to describe itself.
template <class B>
class body {
public:
    using binding = B;
    body(builder& g, std::size_t composite) : owner_(&g), composite_(composite) {}

    [[nodiscard]] builder& owner() const noexcept {
        return *owner_;
    }
    [[nodiscard]] std::size_t composite() const noexcept {
        return composite_;
    }

private:
    builder* owner_;
    std::size_t composite_;
};

namespace detail {
template <class Body>
concept body_type = requires(body<typename Body::binding>& b) {
    { Body::name } -> std::convertible_to<std::string_view>;
    Body::build(b);
};
}  // namespace detail

/// A composite of binding B, filled by Body: a registered operator of B, or
/// a body of several operators. In a scoreboard graph only (G8).
template <binding B, class Body>
[[nodiscard]] node<B> composite(builder& g, std::string name) {
    static_assert(is_registered_v<Body> || detail::body_type<Body>,
                  "G2: a composite's body is a registered operator, or a body of several");
    static_assert(std::is_same_v<typename Body::binding, B>, "G2: a body fills its composite: it has its binding");
    if constexpr (is_registered_v<Body>) {
        return node<B>{g.add_composite(std::move(name), Body::name, detail::handlers_of<B>(), true)};
    } else {
        static_assert(detail::is_operator_name(Body::name), "G1: a body's name matches [A-Za-z0-9_]+");
        const std::size_t c = g.add_composite(std::move(name), Body::name, detail::handlers_of<B>(), false);
        body<B> b{g, c};
        Body::build(b);
        g.end_body(c);
        return node<B>{c};
    }
}

/// An operator instance of a research graph (G8).
template <class Op>
[[nodiscard]] node<typename Op::binding> instance(builder& g, std::string name) {
    static_assert(is_registered_v<Op>, "G2: an instance is of a registered operator");
    return node<typename Op::binding>{
        g.add_instance(std::move(name), Op::name, detail::handlers_of<typename Op::binding>())};
}

/// An operator instance of a body, named `<composite>.<name>`.
template <class Op, class B>
[[nodiscard]] node<typename Op::binding> instance(body<B>& b, std::string name) {
    static_assert(is_registered_v<Op>, "G2: an instance is of a registered operator");
    return node<typename Op::binding>{b.owner().add_body_instance(
        b.composite(), std::move(name), Op::name, detail::handlers_of<typename Op::binding>())};
}

namespace detail {
template <class H, class B, class... S>
[[nodiscard]] auto fire(scope& s, std::size_t node, const time_of* at, const S&... sources) {
    using sig = typename B::signature;
    using in_domains = typename port_domains<sig, typename H::in>::type;
    using out_domains = typename port_domains<sig, typename H::out>::type;
    static_assert((source<S> && ...), "G3: a source is a value, an input, a constant, or hold(...) of one");
    static_assert(sizeof...(S) == H::in::size, "G3: every in port of the handler has one source");
    static_assert(domains_match<in_domains, S...>(), "G4: every edge joins two ports of the same domain");
    const std::vector<Ref> outs = s.owner().fire(s, node, H::name, at, std::vector<Ref>{ref_of(sources)...});
    return values_of<out_domains>(outs);
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
/// value<D>...)`, one value per in port, returns std::tuple of the handler's
/// out ports' values, in its order.
template <fixed_string H, class B, class F>
void on(body<B>& b, F f) {
    using h = handler_t<B, H>;
    using sig = typename B::signature;
    using in_domains = typename detail::port_domains<sig, typename h::in>::type;
    using out_domains = typename detail::port_domains<sig, typename h::out>::type;
    using expected = decltype(detail::values_of<out_domains>(std::span<const Ref>{}));
    const auto call = [f = std::move(f)](scope& s, const time_of* t, std::span<const Ref> in) {
        auto args = detail::values_of<in_domains>(in);
        if constexpr (h::timed) {
            return std::apply([&](const auto&... v) { return f(s, *t, v...); }, args);
        } else {
            return std::apply([&](const auto&... v) { return f(s, v...); }, args);
        }
    };
    static_assert(
        std::is_same_v<std::invoke_result_t<decltype(call), scope&, const time_of*, std::span<const Ref>>, expected>,
        "G4: a body's handler returns its composite handler's out ports, of their domains");
    b.owner().add_body_handler(b.composite(), h::name, [call](scope& s, const time_of* t, std::span<const Ref> in) {
        return detail::refs_of(call(s, t, in));
    });
}

}  // namespace branes::anatomy::flow
