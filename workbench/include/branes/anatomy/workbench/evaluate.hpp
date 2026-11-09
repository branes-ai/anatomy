#pragma once
/// @file evaluate.hpp
/// @brief The evaluator of a computational domain: a declaration run point
/// by point, in an order its dependences allow, on a firing's values
/// (docs/plan/computational-domain, C1; properties W11 and W12).
///
/// It is an oracle of the declaration, not of the operator: W11 holds it to
/// the handler, and W12 to itself across orders. It is a reference
/// implementation, slow by design, and lives in the workbench, which no
/// mission links.
///
/// Model:
///   tensor   : a port's, a state's, a field's or a parameter's value: its
///              shape, its count, and its stored elements, laid out as
///              data<D> lays them out (flow/data.hpp)
///   inputs   : the in and inout ports and fields, the state before the
///              firing, the operator parameters, and the time
///   outputs  : the out and inout ports written, the state after the firing,
///              and the problems found (K3–K6 for this firing)
///
/// Semantics (flow/compute.hpp): a read of a port or a state sees its value
/// before the firing; the state after the firing starts as a copy of it, and
/// writes change it. Each definition at a point is evaluated once, in its
/// element type, children before parents (cd::postorder).
///
/// Orders: seed 0 is the deterministic order, the least stage, then the
/// lexicographically least point, then the definition declared first, among
/// those whose reads are all written. Any other seed picks among them at
/// random, from std::mt19937_64's raw output.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/compute.hpp>
#include <branes/anatomy/flow/data.hpp>
#include <branes/anatomy/flow/window.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <random>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace branes::anatomy::workbench::eval {

namespace fl = branes::anatomy::flow;
namespace cd = branes::anatomy::flow::cd;

/// One element, of type t: an integer in i, a real in re, a complex in re and im.
/// A float's or a double's value is exact in a long double.
struct value {
    cd::etype t = cd::etype::none;
    long double re = 0;
    long double im = 0;
    std::int64_t i = 0;

    friend bool operator==(const value&, const value&) = default;
};

struct tensor {
    cd::port_info shape;
    std::int64_t count = 0;  ///< a bounded leading extent's count; its size otherwise
    std::vector<value> v;    ///< the stored elements

    friend bool operator==(const tensor& a, const tensor& b) {
        return a.count == b.count && a.v == b.v;
    }
};

struct inputs {
    std::map<std::string, tensor> ports;   ///< by path: "x", "w.items"
    std::map<std::string, tensor> states;  ///< before the firing
    std::map<std::string, tensor> params;  ///< operator parameters
    std::int64_t time = 0;                 ///< ns
};

struct outputs {
    std::map<std::string, tensor> ports;   ///< the out and inout ports written
    std::map<std::string, tensor> states;  ///< after the firing
    std::vector<cd::problem> problems;
};

// ── Values ──────────────────────────────────────────────────────────

namespace detail {

template <class T>
struct is_complex : std::false_type {};
template <class T>
struct is_complex<std::complex<T>> : std::true_type {};

/// z as a T: a complex's real part for a real T, a real as a complex.
template <class T, class Z>
[[nodiscard]] T to(Z z) {
    if constexpr (is_complex<T>::value) {
        if constexpr (is_complex<Z>::value) {
            return T(z);
        } else {
            return T(static_cast<typename T::value_type>(z));
        }
    } else if constexpr (is_complex<Z>::value) {
        return static_cast<T>(z.real());
    } else {
        return static_cast<T>(z);
    }
}

template <class T>
[[nodiscard]] value make(cd::etype t, T x) {
    value v;
    v.t = t;
    if constexpr (std::is_integral_v<T>) {
        v.i = static_cast<std::int64_t>(x);
    } else if constexpr (std::is_floating_point_v<T>) {
        v.re = static_cast<long double>(x);
    } else {
        v.re = static_cast<long double>(x.real());
        v.im = static_cast<long double>(x.imag());
    }
    return v;
}

/// Calls f with v as its C++ type.
template <class F>
decltype(auto) visit(const value& v, F&& f) {
    switch (v.t) {
    case cd::etype::i64:
        return f(v.i);
    case cd::etype::u64:
        return f(static_cast<std::uint64_t>(v.i));
    case cd::etype::f32:
        return f(static_cast<float>(v.re));
    case cd::etype::f64:
        return f(static_cast<double>(v.re));
    case cd::etype::f80:
        return f(v.re);
    case cd::etype::c64:
        return f(std::complex<float>(static_cast<float>(v.re), static_cast<float>(v.im)));
    case cd::etype::c128:
        return f(std::complex<double>(static_cast<double>(v.re), static_cast<double>(v.im)));
    case cd::etype::c160:
        return f(std::complex<long double>(v.re, v.im));
    default:
        return f(v.re);
    }
}

/// x converted to type t.
template <class T>
[[nodiscard]] value convert(cd::etype t, T x) {
    auto real_part = [&] {
        if constexpr (is_complex<T>::value) {
            return x.real();
        } else {
            return x;
        }
    };
    switch (t) {
    case cd::etype::i64:
        return make(t, static_cast<std::int64_t>(real_part()));
    case cd::etype::u64:
        return make(t, static_cast<std::uint64_t>(real_part()));
    case cd::etype::f32:
        return make(t, static_cast<float>(real_part()));
    case cd::etype::f64:
        return make(t, static_cast<double>(real_part()));
    case cd::etype::f80:
        return make(t, static_cast<long double>(real_part()));
    case cd::etype::c64:
        if constexpr (is_complex<T>::value) {
            return make(t, std::complex<float>(x));
        } else {
            return make(t, std::complex<float>(static_cast<float>(x)));
        }
    case cd::etype::c128:
        if constexpr (is_complex<T>::value) {
            return make(t, std::complex<double>(x));
        } else {
            return make(t, std::complex<double>(static_cast<double>(x)));
        }
    case cd::etype::c160:
        if constexpr (is_complex<T>::value) {
            return make(t, std::complex<long double>(x));
        } else {
            return make(t, std::complex<long double>(static_cast<long double>(x)));
        }
    default:
        return {};
    }
}

[[nodiscard]] inline value cast(cd::etype t, const value& v) {
    return visit(v, [&](auto x) { return convert(t, x); });
}

[[nodiscard]] inline value binary(cd::op o, const value& a, const value& b) {
    return visit(a, [&](auto x) -> value {
        using T = decltype(x);
        const T y = visit(b, [](auto z) { return to<T>(z); });
        switch (o) {
        case cd::op::add:
            return make(a.t, static_cast<T>(x + y));
        case cd::op::sub:
            return make(a.t, static_cast<T>(x - y));
        case cd::op::mul:
            return make(a.t, static_cast<T>(x * y));
        case cd::op::div:
            if constexpr (std::is_integral_v<T>) {
                return make(a.t, y == 0 ? T{0} : static_cast<T>(x / y));
            } else {
                return make(a.t, static_cast<T>(x / y));
            }
        default:
            return {};
        }
    });
}

[[nodiscard]] inline value unary(cd::op o, cd::etype result, const value& a) {
    return visit(a, [&](auto x) -> value {
        using T = decltype(x);
        if constexpr (is_complex<T>::value) {
            switch (o) {
            case cd::op::neg:
                return make(a.t, -x);
            case cd::op::conj:
                return make(a.t, std::conj(x));
            case cd::op::real:
                return make(result, x.real());
            case cd::op::imag:
                return make(result, x.imag());
            case cd::op::cabs:
                return make(result, std::abs(x));
            default:
                return {};
            }
        } else if constexpr (std::is_floating_point_v<T>) {
            switch (o) {
            case cd::op::neg:
                return make(a.t, -x);
            case cd::op::sqrt:
                return make(a.t, std::sqrt(x));
            case cd::op::tanh:
                return make(a.t, std::tanh(x));
            case cd::op::exp:
                return make(a.t, std::exp(x));
            case cd::op::abs:
                return make(a.t, std::abs(x));
            default:
                return {};
            }
        } else {
            if (o == cd::op::neg) {
                return make(a.t, static_cast<T>(T{0} - x));
            }
            return {};
        }
    });
}

[[nodiscard]] inline bool less(const value& a, const value& b, bool or_equal) {
    return visit(a, [&](auto x) -> bool {
        using T = decltype(x);
        if constexpr (is_complex<T>::value) {
            return false;
        } else {
            const T y = visit(b, [](auto z) { return to<T>(z); });
            return or_equal ? x <= y : x < y;
        }
    });
}

}  // namespace detail

// ── The evaluation ──────────────────────────────────────────────────

/// Runs s on the inputs, in the order seed picks (0: the deterministic order).
[[nodiscard]] inline outputs
evaluate(const cd::system& s, const cd::table& t, const inputs& in, std::uint64_t seed = 0) {
    outputs out;
    auto path_of = [](const cd::name_t& name, const cd::name_t& field) {
        std::string p{name.view()};
        if (!field.empty()) {
            p += ".";
            p += field.view();
        }
        return p;
    };
    auto find_value = [&](const cd::name_t& name, const cd::name_t& field) -> const tensor* {
        const std::string p = path_of(name, field);
        if (const auto it = in.ports.find(p); it != in.ports.end()) {
            return &it->second;
        }
        if (const auto it = in.states.find(p); it != in.states.end()) {
            return &it->second;
        }
        return nullptr;
    };

    // The counts, from the inputs.
    std::vector<std::int64_t> counts;
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const cd::param_decl& p = s.params[i];
        if (p.kind == cd::pkind::count) {
            const tensor* x = find_value(p.target, p.field);
            counts.push_back(x ? x->count : 0);
        }
    }
    const cd::dgraph g = cd::build_graph(s, t, counts);
    if (!g.problems.empty()) {
        out.problems = g.problems;
        return out;
    }

    std::array<cd::etype, cd::max_nodes> ty{};
    (void)cd::detail::type_nodes(s, t, ty);
    std::vector<std::vector<std::uint16_t>> order(s.n_defs);
    for (std::uint16_t d = 0; d < s.n_defs; ++d) {
        order[d] = cd::postorder(s, s.defs[d].value);
    }

    // The outputs: each written port or state, starting from its old value
    // (inout, state) or empty (out); every state is carried over.
    for (const auto& [name, x] : in.states) {
        out.states[name] = x;
    }
    std::vector<tensor*> target_of(s.n_operands, nullptr);
    std::vector<tensor> locals(s.n_operands);
    for (std::uint16_t i = 0; i < s.n_operands; ++i) {
        const cd::operand& o = s.operands[i];
        bool written = false;
        for (std::uint16_t d = 0; d < s.n_defs; ++d) {
            written = written || (s.defs[d].kind == cd::dkind::write && s.defs[d].target == i);
        }
        if (!written) {
            continue;
        }
        if (o.kind == cd::okind::local) {
            tensor& l = locals[i];
            l.shape.rank = o.rank;
            l.shape.t = o.t;
            for (std::size_t k = 0; k < o.rank; ++k) {
                l.shape.extent[k] = cd::eval_ix(s, o.extent[k], nullptr, g.params);
            }
            l.count = l.shape.rank ? l.shape.extent[0] : 1;
            l.v.assign(static_cast<std::size_t>(cd::stored_of(l.shape)), value{o.t});
            target_of[i] = &l;
            continue;
        }
        const cd::port_info* p = t.find(o.name, o.field);
        if (p == nullptr) {
            continue;
        }
        const std::string path = path_of(o.name, o.field);
        tensor* dst = nullptr;
        if (o.kind == cd::okind::state) {
            dst = &out.states[path];
        } else {
            dst = &out.ports[path];
            if (p->dir == cd::access::inout) {
                if (const tensor* old = find_value(o.name, o.field)) {
                    *dst = *old;
                }
            }
        }
        if (dst->v.empty()) {
            dst->shape = *p;
            dst->count = p->rank ? p->extent[0] : 1;
            dst->v.assign(static_cast<std::size_t>(cd::stored_of(*p)), value{p->t});
        }
        for (std::size_t c = 0; c < s.n_counts; ++c) {
            if (s.counts[c].target == i) {
                dst->count = g.counts[c];
                // Rows past the count are value-initialized (A2).
                const std::int64_t cols =
                    p->rank >= 2 ? cd::stored_of(*p) / std::max<std::int64_t>(p->extent[0], 1) : 1;
                for (std::size_t e = static_cast<std::size_t>(dst->count * cols); e < dst->v.size(); ++e) {
                    dst->v[e] = value{p->t};
                }
            }
        }
        target_of[i] = dst;
    }

    // The order.
    const std::size_t n = g.nodes.size();
    std::vector<std::uint32_t> indegree(n, 0);
    std::vector<std::vector<std::uint32_t>> succ(n);
    for (std::uint32_t k = 0; k < n; ++k) {
        for (std::uint32_t r = 0; r < g.nodes[k].nsrc; ++r) {
            succ[g.srcs[g.nodes[k].src + r]].push_back(k);
            ++indegree[k];
        }
    }
    auto before = [&](std::uint32_t a, std::uint32_t b) {
        const cd::dgraph::dnode& x = g.nodes[a];
        const cd::dgraph::dnode& y = g.nodes[b];
        const int sa = s.defs[x.def].stage;
        const int sb = s.defs[y.def].stage;
        if (sa != sb) {
            return sa < sb;
        }
        const int rank = cd::stage_rank(s, sa);
        for (int k = 0; k < rank; ++k) {
            const std::int64_t pa = g.coords[x.point + static_cast<std::uint32_t>(k)];
            const std::int64_t pb = g.coords[y.point + static_cast<std::uint32_t>(k)];
            if (pa != pb) {
                return pa < pb;
            }
        }
        return x.def < y.def;
    };
    std::vector<std::uint32_t> ready;
    for (std::uint32_t k = 0; k < n; ++k) {
        if (indegree[k] == 0) {
            ready.push_back(k);
        }
    }
    std::mt19937_64 rng(seed);

    std::vector<value> result(n);
    std::vector<value> memo(cd::max_nodes);
    std::size_t done = 0;
    while (!ready.empty()) {
        std::size_t pick = 0;
        if (seed == 0) {
            for (std::size_t k = 1; k < ready.size(); ++k) {
                if (before(ready[k], ready[pick])) {
                    pick = k;
                }
            }
        } else {
            pick = static_cast<std::size_t>(rng() % ready.size());
        }
        const std::uint32_t id = ready[pick];
        ready[pick] = ready.back();
        ready.pop_back();
        ++done;

        // Evaluate the definition at its point.
        const cd::dgraph::dnode& nd = g.nodes[id];
        const cd::def& d = s.defs[nd.def];
        const std::int64_t* point = g.coords.data() + nd.point;
        std::uint32_t next_src = nd.src;
        for (const std::uint16_t m : order[nd.def]) {
            const cd::node& x = s.nodes[m];
            value v;
            switch (x.o) {
            case cd::op::lit:
                v = detail::convert(ty[m], static_cast<long double>(x.v));
                break;
            case cd::op::time:
                v = detail::make(cd::etype::i64, in.time);
                break;
            case cd::op::ival: {
                const std::int64_t k = cd::eval_ix(s, x.a, point, g.params);
                v = cd::is_integer(x.t) ? detail::make(x.t, k) : detail::convert(x.t, static_cast<long double>(k));
                break;
            }
            case cd::op::read: {
                const cd::operand& o = s.operands[x.a];
                std::array<std::int64_t, cd::max_rank> idx{};
                for (std::uint16_t k = 0; k < x.c; ++k) {
                    idx[k] = cd::eval_ix(s, s.args[x.b + k], point, g.params);
                }
                if (o.kind == cd::okind::var || o.kind == cd::okind::local) {
                    v = result[g.srcs[next_src++]];
                } else if (o.kind == cd::okind::param) {
                    const auto it = in.params.find(std::string{o.name.view()});
                    if (it == in.params.end() || it->second.v.empty()) {
                        out.problems.push_back(
                            {"K4", "operator parameter " + std::string{o.name.view()} + " isn't given"});
                        return out;
                    }
                    const std::int64_t e = x.c ? idx[0] : 0;
                    if (e < 0 || e >= static_cast<std::int64_t>(it->second.v.size())) {
                        out.problems.push_back(
                            {"K4", "operator parameter " + std::string{o.name.view()} + " is read outside it"});
                        return out;
                    }
                    v = it->second.v[static_cast<std::size_t>(e)];
                } else {
                    const tensor* src = find_value(o.name, o.field);
                    if (src == nullptr) {
                        out.problems.push_back({"K4", path_of(o.name, o.field) + " isn't given"});
                        return out;
                    }
                    const std::int64_t e = cd::element_of(src->shape, src->count, idx.data());
                    if (e < 0) {
                        out.problems.push_back({"K4", path_of(o.name, o.field) + " is read outside it"});
                        return out;
                    }
                    v = src->v[static_cast<std::size_t>(e)];
                }
                break;
            }
            case cd::op::cast:
                v = detail::cast(x.t, memo[x.a]);
                break;
            case cd::op::add:
            case cd::op::sub:
            case cd::op::mul:
            case cd::op::div: {
                value a = memo[x.a];
                value b = memo[x.b];
                a = a.t == ty[m] ? a : detail::cast(ty[m], a);
                b = b.t == ty[m] ? b : detail::cast(ty[m], b);
                v = detail::binary(x.o, a, b);
                break;
            }
            case cd::op::sel_lt:
            case cd::op::sel_le:
                v = detail::less(memo[x.a], memo[x.b], x.o == cd::op::sel_le) ? memo[x.c] : memo[x.d];
                break;
            default:
                v = detail::unary(x.o, ty[m], memo[x.a]);
                break;
            }
            memo[m] = v;
        }
        value v = memo[d.value];
        const cd::etype want = cd::detail::type_of_operand(s, t, d.target);
        if (v.t != want) {
            v = detail::cast(want, v);
        }
        result[id] = v;
        if (d.kind == cd::dkind::write) {
            if (tensor* dst = target_of[d.target]) {
                dst->v[static_cast<std::size_t>(nd.element)] = v;
            }
        }
        for (const std::uint32_t m : succ[id]) {
            if (--indegree[m] == 0) {
                ready.push_back(m);
            }
        }
    }
    if (done != n) {
        out.problems.push_back({"K6", "the dependences have a cycle"});
    }
    return out;
}

// ── Messages to tensors and back ────────────────────────────────────

template <class D>
[[nodiscard]] tensor to_tensor(const fl::data<D>& d) {
    tensor x;
    x.shape = cd::detail::info_of_domain<D>("", "", cd::access::in);
    x.count = static_cast<std::int64_t>(d.count());
    const cd::etype t = cd::etype_of<typename D::element_type>;
    x.v.reserve(d.elements.size());
    for (const auto& e : d.elements) {
        x.v.push_back(detail::convert(t, e));
    }
    return x;
}

template <class D>
void from_tensor(const tensor& x, fl::data<D>& d) {
    using E = typename D::element_type;
    (void)d.resize(static_cast<std::size_t>(x.count));
    for (std::size_t k = 0; k < d.elements.size() && k < x.v.size(); ++k) {
        d.elements[k] = detail::visit(x.v[k], [](auto z) { return detail::to<E>(z); });
    }
}

namespace detail {
template <class M>
struct item_element {
    using type = M;
};
template <class D>
struct item_element<fl::data<D>> {
    using type = typename D::element_type;
};

template <class M>
[[nodiscard]] auto item_value(const M& m) {
    if constexpr (requires { m.elements; }) {
        return m.elements[0];
    } else {
        return m;
    }
}

template <class X>
void put(std::map<std::string, tensor>& out, const std::string& name, const X& m) {
    if constexpr (requires {
                      typename X::domain;
                      m.elements;
                  }) {
        out[name] = to_tensor(m);
    } else if constexpr (requires {
                             m.items;
                             m.dropped;
                             X::capacity;
                         }) {
        using E = typename item_element<std::remove_cvref_t<decltype(m.items[0])>>::type;
        fl::data<fl::vector<E, fl::bounded<X::capacity>>> items;
        (void)items.resize(m.count);
        for (std::size_t k = 0; k < m.count; ++k) {
            items(k) = item_value(m.items[k]);
        }
        out[name + ".items"] = to_tensor(items);
        fl::data<fl::scalar<std::uint64_t>> dropped;
        dropped() = m.dropped;
        out[name + ".dropped"] = to_tensor(dropped);
    } else {
        static_assert(sizeof(X) == 0, "eval: a message the evaluator can't read");
    }
}

template <class Sig, class Names>
struct in_types;
template <class Sig, fl::fixed_string... N>
struct in_types<Sig, fl::ins<N...>> {
    using type = std::tuple<fl::detail::port_value_t<Sig, N>...>;
};
template <class Sig, class Names>
struct out_types;
template <class Sig, fl::fixed_string... N>
struct out_types<Sig, fl::outs<N...>> {
    using type = std::tuple<fl::detail::port_value_t<Sig, N>...>;
};

template <class B, class Op>
void put_members(std::map<std::string, tensor>& states, std::map<std::string, tensor>& params, const Op& op) {
    if constexpr (requires { typename B::cd_state; }) {
        std::apply([&](auto... m) { (put(states, std::string{decltype(m)::name}, op.*(decltype(m)::ptr)), ...); },
                   typename B::cd_state::types{});
    }
    if constexpr (requires { typename B::cd_params; }) {
        std::apply(
            [&](auto... m) {
                (
                    [&] {
                        const auto& x = op.*(decltype(m)::ptr);
                        using X = std::remove_cvref_t<decltype(x)>;
                        tensor tx;
                        if constexpr (requires {
                                          x.size();
                                          x[0];
                                      }) {
                            using E = std::remove_cvref_t<decltype(x[0])>;
                            tx.shape.t = cd::etype_of<E>;
                            tx.shape.rank = 1;
                            tx.shape.extent[0] = static_cast<std::int64_t>(x.size());
                            tx.count = tx.shape.extent[0];
                            for (const auto& e : x) {
                                tx.v.push_back(convert(cd::etype_of<E>, e));
                            }
                        } else {
                            tx.shape.t = cd::etype_of<X>;
                            tx.count = 1;
                            tx.v.push_back(convert(cd::etype_of<X>, x));
                        }
                        params[std::string{decltype(m)::name}] = tx;
                    }(),
                    ...);
            },
            typename B::cd_params::types{});
    }
}
}  // namespace detail

/// What the handler and its declaration gave on one firing: each out port
/// and each state after it, as tensors.
struct firing {
    std::map<std::string, tensor> handler;
    std::map<std::string, tensor> declared;
    std::vector<cd::problem> problems;
};

/// Fires handler H of Op on a copy of `op`, and evaluates its declaration on
/// the same values, in the order `seed` picks.
template <class Op, fl::fixed_string H, class... In>
[[nodiscard]] firing fire_both(const typename Op::type& op, Timestamp time, const In&... in) {
    using B = typename Op::binding;
    using Hd = fl::handler_t<B, H>;
    using Sig = typename B::signature;
    firing f;

    // The declaration's inputs.
    inputs x;
    x.time = time.ns;
    {
        std::size_t k = 0;
        (detail::put(x.ports, std::string{Hd::in::names[k++]}, in), ...);
    }
    detail::put_members<B>(x.states, x.params, op);
    const outputs o = evaluate(Hd::compute, cd::table_of<B, Hd>(), x, 0);
    f.problems = o.problems;
    for (const auto& [name, v] : o.ports) {
        f.declared[name] = v;
    }
    for (const auto& [name, v] : o.states) {
        f.declared["state " + name] = v;
    }

    // The handler, on a copy of the operator.
    typename Op::type copy = op;
    auto call = [&]() -> decltype(auto) {
        if constexpr (Hd::timed) {
            return Hd::call(copy, time, in...);
        } else {
            return Hd::call(copy, in...);
        }
    };
    using R = decltype(call());
    if constexpr (std::is_void_v<R>) {
        call();
    } else {
        const R r = call();
        if constexpr (requires { std::tuple_size<R>::value; }) {
            std::size_t k = 0;
            std::apply([&](const auto&... e) { (detail::put(f.handler, std::string{Hd::out::names[k++]}, e), ...); },
                       r);
        } else {
            detail::put(f.handler, std::string{Hd::out::names[0]}, r);
        }
    }
    std::map<std::string, tensor> after_states;
    std::map<std::string, tensor> ignored;
    detail::put_members<B>(after_states, ignored, copy);
    for (const auto& [name, v] : after_states) {
        f.handler["state " + name] = v;
    }
    (void)sizeof(Sig);
    return f;
}

/// The largest difference between two tensors' elements, relative to the
/// largest magnitude in `ref` (or absolute, if that is 0); infinity if their
/// shapes or counts differ.
[[nodiscard]] inline long double difference(const tensor& a, const tensor& ref) {
    if (a.count != ref.count || a.v.size() != ref.v.size()) {
        return std::numeric_limits<long double>::infinity();
    }
    long double scale = 0;
    long double diff = 0;
    for (std::size_t k = 0; k < a.v.size(); ++k) {
        const value& x = a.v[k];
        const value& y = ref.v[k];
        if (cd::is_integer(y.t)) {
            diff = std::max(diff, static_cast<long double>(x.i == y.i ? 0 : 1));
            continue;
        }
        scale = std::max(scale, std::hypot(y.re, y.im));
        diff = std::max(diff, std::hypot(x.re - y.re, x.im - y.im));
    }
    return scale > 0 ? diff / scale : diff;
}

}  // namespace branes::anatomy::workbench::eval
