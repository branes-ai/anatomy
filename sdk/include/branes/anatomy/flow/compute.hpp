#pragma once
/// @file compute.hpp
/// @brief The computational domain of a handler: what each point of its
/// index space computes, which points it reads, and where its ports and its
/// state enter and leave (anatomy docs/adr/0005, decision 2;
/// docs/plan/computational-domain, C1).
///
/// A handler declares its computational domain as a `static constexpr
/// cd::system compute` member of its handler struct, in its operator's
/// binding. A handler without one is opaque. The description is a system of
/// affine recurrence equations, as in domain_flow's `.sure` specifications,
/// before uniformization: a body may read a port at an affine function of the
/// point. Uniformizing, projecting and scheduling it are the lowering's.
///
/// Model:
///   system          : one or more stages, each an index space, and what is
///                     defined over it; stages are joined by locals
///   parameter       : an integer, or an extent, a bound or a count of a port,
///                     a state or a field; bound when the system is checked
///   index           : a named index of a stage, lo <= i < hi, with lo and hi
///                     affine in the parameters and the stage's earlier indices
///   operand         : a port, a state, a field of either (`"w.items"`), a
///                     local (an intermediate of the handler), a variable of a
///                     stage, or an operator parameter
///   definition      : an equation (a variable at every point of a region), an
///                     init (a variable at the points of a face, outside its
///                     region), or a write (an element of a port, a state or a
///                     local, from the points of a region)
///   region          : the stage's space, narrowed by constraints; a face pins
///                     indices (`j == -1`) instead of ranging over them
///   expression      : literals, reads, the firing's time, an index as a value,
///                     and the operations of `op`
///
/// Semantics: a read of a port or a state sees its value before the firing,
/// and a write sets its value after it. An `inout` port or a state is read
/// and written that way, so its old value is never overwritten by the
/// description; a lowering that updates it in place orders its own writes
/// after the reads it can see here.
///
/// Invariants, checked at compile time when a binding is registered
/// (flow::registered, flow/builder.hpp):
///   K1: the indices and the variables, locals and operator parameters have
///       distinct, non-empty names; every port and state named is the
///       binding's; an in port is read only if the handler reads it, an out
///       port written only if it writes it, and in ports are never written or
///       out ports read; each read and write has its target's rank; every
///       operation's operands have one element type, the one its result needs;
///       every index belongs to the stage that uses it
///   K2: each parameter is bound to an integer, or to an extent, a bound or a
///       count that its target has
/// The rest (K3–K8) enumerate the bound index space: cd::analyze, at run time.
///
/// Capacities: a declaration that exceeds one (max_nodes and the rest) doesn't
/// compile; it calls capacity_exceeded, which isn't constexpr.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/signature.hpp>

#include <array>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <vector>

namespace branes::anatomy::flow::cd {

// ── Element types ───────────────────────────────────────────────────

/// The element types a computational domain computes in.
enum class etype : std::uint8_t { none, i64, u64, f32, f64, f80, c64, c128, c160 };

namespace detail {
template <class E>
struct etype_of_impl : std::integral_constant<etype, etype::none> {};
template <>
struct etype_of_impl<std::int64_t> : std::integral_constant<etype, etype::i64> {};
template <>
struct etype_of_impl<std::uint64_t> : std::integral_constant<etype, etype::u64> {};
template <>
struct etype_of_impl<float> : std::integral_constant<etype, etype::f32> {};
template <>
struct etype_of_impl<double> : std::integral_constant<etype, etype::f64> {};
template <>
struct etype_of_impl<long double> : std::integral_constant<etype, etype::f80> {};
template <>
struct etype_of_impl<std::complex<float>> : std::integral_constant<etype, etype::c64> {};
template <>
struct etype_of_impl<std::complex<double>> : std::integral_constant<etype, etype::c128> {};
template <>
struct etype_of_impl<std::complex<long double>> : std::integral_constant<etype, etype::c160> {};
}  // namespace detail

/// E's element type, or none if a computational domain can't compute in E.
template <class E>
inline constexpr etype etype_of = detail::etype_of_impl<E>::value;

[[nodiscard]] constexpr bool is_integer(etype t) noexcept {
    return t == etype::i64 || t == etype::u64;
}
[[nodiscard]] constexpr bool is_real(etype t) noexcept {
    return t == etype::f32 || t == etype::f64 || t == etype::f80;
}
[[nodiscard]] constexpr bool is_complex(etype t) noexcept {
    return t == etype::c64 || t == etype::c128 || t == etype::c160;
}
/// The real type of a complex one; t itself otherwise.
[[nodiscard]] constexpr etype real_of(etype t) noexcept {
    switch (t) {
    case etype::c64:
        return etype::f32;
    case etype::c128:
        return etype::f64;
    case etype::c160:
        return etype::f80;
    default:
        return t;
    }
}
[[nodiscard]] constexpr std::string_view to_string(etype t) noexcept {
    switch (t) {
    case etype::i64:
        return "i64";
    case etype::u64:
        return "u64";
    case etype::f32:
        return "f32";
    case etype::f64:
        return "f64";
    case etype::f80:
        return "f80";
    case etype::c64:
        return "c64";
    case etype::c128:
        return "c128";
    case etype::c160:
        return "c160";
    default:
        return "none";
    }
}

// ── Capacities ──────────────────────────────────────────────────────

inline constexpr std::size_t max_name = 24;
inline constexpr std::size_t max_nodes = 1024;
inline constexpr std::size_t max_args = 1024;
inline constexpr std::size_t max_constraints = 384;
inline constexpr std::size_t max_indices = 24;
inline constexpr std::size_t max_params = 24;
inline constexpr std::size_t max_operands = 48;
inline constexpr std::size_t max_defs = 96;
inline constexpr std::size_t max_counts = 8;
inline constexpr std::size_t max_stages = 8;
inline constexpr std::size_t max_region = 6;
inline constexpr std::size_t max_rank = 6;
inline constexpr std::size_t max_ports = 24;

/// Not constexpr: a declaration that reaches it doesn't compile.
[[noreturn]] inline void capacity_exceeded(const char* what) noexcept {
    (void)what;
    std::abort();
}

// ── Names ───────────────────────────────────────────────────────────

/// A name of at most max_name characters, stored in place.
struct name_t {
    std::array<char, max_name> c{};
    std::uint8_t n = 0;

    constexpr name_t() = default;
    constexpr explicit name_t(std::string_view s) noexcept {
        if (s.size() > max_name) {
            capacity_exceeded("cd: a name is longer than max_name");
        }
        for (std::size_t i = 0; i < s.size(); ++i) {
            c[i] = s[i];
        }
        n = static_cast<std::uint8_t>(s.size());
    }
    [[nodiscard]] constexpr std::string_view view() const noexcept {
        return {c.data(), n};
    }
    [[nodiscard]] constexpr bool empty() const noexcept {
        return n == 0;
    }
    friend constexpr bool operator==(const name_t& a, const name_t& b) noexcept {
        return a.view() == b.view();
    }
};

// ── The description ─────────────────────────────────────────────────

/// An expression's operation. The integer ones build index expressions; the
/// others build values.
enum class op : std::uint8_t {
    // integers: an index function, a bound, a count
    iconst,  ///< k
    index,   ///< index a
    param,   ///< parameter a
    iadd,
    isub,
    imul,
    idiv,
    imod,
    ixor,
    ishl,
    ishr,
    // values
    lit,   ///< v, of the type its context gives it
    read,  ///< operand a at the c index expressions from args[b]
    time,  ///< the firing's time, in ns (i64)
    ival,  ///< index expression a as a value of type t
    cast,  ///< a, to type t
    add,
    sub,
    mul,
    div,
    neg,
    sqrt,
    tanh,
    exp,
    abs,  ///< of a real
    conj,
    real,
    imag,
    cabs,    ///< the magnitude of a complex
    sel_lt,  ///< a < b ? c : d
    sel_le,  ///< a <= b ? c : d
};

[[nodiscard]] constexpr bool is_index_op(op o) noexcept {
    return o <= op::ishr;
}
/// Counted by cd::count: an operation, not a literal, a read or the time.
[[nodiscard]] constexpr bool is_counted(op o) noexcept {
    return o >= op::cast;
}
[[nodiscard]] std::string_view to_string(op o) noexcept;

struct node {
    op o = op::iconst;
    etype t = etype::none;
    std::uint16_t a = 0;
    std::uint16_t b = 0;
    std::uint16_t c = 0;
    std::uint16_t d = 0;
    std::int64_t k = 0;
    double v = 0;
};

enum class cmp : std::uint8_t { eq, lt, le };

/// lhs cmp rhs, both index expressions.
struct constraint {
    cmp c = cmp::eq;
    std::uint16_t lhs = 0;
    std::uint16_t rhs = 0;
};

/// A stage's space narrowed by constraints. In a face, an equality whose left
/// side is an index pins it.
struct region {
    std::array<std::uint16_t, max_region> ids{};
    std::uint8_t n = 0;
    bool face = false;
};

struct index_decl {
    name_t name;
    std::uint16_t lo = 0;  ///< an index expression
    std::uint16_t hi = 0;  ///< an index expression, exclusive
    std::uint8_t stage = 0;
};

/// What a parameter is bound to.
enum class pkind : std::uint8_t {
    integer,  ///< value
    extent,   ///< extent `dim` of an operand: a static extent's size, a bounded one's bound
    count,    ///< the count of an operand's bounded leading extent
};

/// A parameter. Its target is a path, resolved when the system is checked:
/// a local of that name, or a port's or a state's (field) in the table.
struct param_decl {
    name_t name;
    pkind kind = pkind::integer;
    name_t target;
    name_t field;
    std::uint8_t dim = 0;
    std::int64_t value = 0;
};

enum class okind : std::uint8_t {
    port,   ///< a port of the signature, or a field of one
    state,  ///< a state domain of the signature, or a field of one
    local,  ///< an intermediate of the handler, written by one stage and read by later ones
    var,    ///< a variable of a stage, defined at its points
    param,  ///< an operator parameter: a scalar (rank 0) or a table (rank 1)
};

struct operand {
    okind kind = okind::port;
    name_t name;
    name_t field;                                  ///< a port's or a state's field; empty for the whole
    etype t = etype::none;                         ///< declared for a local, a variable or a parameter
    std::uint8_t rank = 0;                         ///< declared for a local or a parameter; a variable's is its stage's
    std::uint8_t stage = 0;                        ///< a variable's
    std::array<std::uint16_t, max_rank> extent{};  ///< a local's, as index expressions
};

enum class dkind : std::uint8_t {
    eq,     ///< target(p) = value at every point p of the region
    init,   ///< target(q) = value at every point q of the face
    write,  ///< target(args(p)) = value at every point p of the region
};

struct def {
    dkind kind = dkind::eq;
    std::uint16_t target = 0;
    std::uint16_t args = 0;  ///< a write's index expressions, from args[args]
    std::uint16_t nargs = 0;
    std::uint16_t value = 0;
    region where;
    std::uint8_t stage = 0;
};

/// The count of a bounded out port's or state's leading extent, after the firing.
struct count_decl {
    std::uint16_t target = 0;
    std::uint16_t expr = 0;
};

class system;

/// An index expression, while a system is declared.
struct ix {
    system* s = nullptr;
    std::uint16_t n = 0;
};

/// A value expression, while a system is declared.
struct val {
    system* s = nullptr;
    std::uint16_t n = 0;
};

/// A constraint, while a system is declared.
struct cond {
    system* s = nullptr;
    std::uint16_t n = 0;
};

/// An element of a port, a state or a local, to write.
struct place {
    system* s = nullptr;
    std::uint16_t target = 0;
    std::uint16_t args = 0;
    std::uint16_t nargs = 0;
};

template <class X>
concept index_like = std::is_same_v<X, ix> || std::integral<X>;

/// An operand, while a system is declared: read it with (), write it with at().
struct target {
    system* s = nullptr;
    std::uint16_t id = 0;

    template <index_like... A>
    constexpr val operator()(A... a) const noexcept;
    template <index_like... A>
    constexpr place at(A... a) const noexcept;
};

/// The computational domain of a handler.
class system {
public:
    std::array<node, max_nodes> nodes{};
    std::array<std::uint16_t, max_args> args{};
    std::array<constraint, max_constraints> constraints{};
    std::array<index_decl, max_indices> indices{};
    std::array<param_decl, max_params> params{};
    std::array<operand, max_operands> operands{};
    std::array<def, max_defs> defs{};
    std::array<count_decl, max_counts> counts{};
    std::array<region, max_stages> spaces{};  ///< each stage's constraints, beside its bounds
    std::uint16_t n_nodes = 0;
    std::uint16_t n_args = 0;
    std::uint16_t n_constraints = 0;
    std::uint8_t n_indices = 0;
    std::uint8_t n_params = 0;
    std::uint8_t n_operands = 0;
    std::uint8_t n_defs = 0;
    std::uint8_t n_counts = 0;
    std::uint8_t n_stages = 1;

    // ── Declaring ──

    /// An index of the current stage, lo <= i < hi.
    template <index_like L, index_like H>
    constexpr ix index(std::string_view name, L lo, H hi) noexcept {
        if (n_indices == max_indices) {
            capacity_exceeded("cd: more than max_indices indices");
        }
        index_decl d;
        d.name = name_t{name};
        d.lo = to_ix(lo).n;
        d.hi = to_ix(hi).n;
        d.stage = static_cast<std::uint8_t>(n_stages - 1);
        indices[n_indices] = d;
        node x;
        x.o = op::index;
        x.a = n_indices;
        ++n_indices;
        return {this, add_node(x)};
    }

    /// A further constraint on the current stage's space.
    constexpr void constrain(cond c) noexcept {
        region& r = spaces[n_stages - 1];
        if (r.n == max_region) {
            capacity_exceeded("cd: more than max_region constraints on a space");
        }
        r.ids[r.n++] = c.n;
    }

    /// Begin the next stage.
    constexpr void stage() noexcept {
        if (n_stages == max_stages) {
            capacity_exceeded("cd: more than max_stages stages");
        }
        ++n_stages;
    }

    /// Extent `dim` of a port, a state, a field (`"w.items"`) or a local: a
    /// static extent's size, a bounded one's bound.
    constexpr ix extent(std::string_view path, std::size_t dim) noexcept {
        return add_param(path, pkind::extent, dim);
    }
    /// The count of the bounded leading extent of a port, a state or a field,
    /// at run time.
    constexpr ix count(std::string_view path) noexcept {
        return add_param(path, pkind::count, 0);
    }

    /// A port or a field of one, to read (in ports) or write (out ports); an
    /// inout port is both.
    constexpr target in(std::string_view path) noexcept {
        return {this, find_or_add(okind::port, path)};
    }
    constexpr target out(std::string_view path) noexcept {
        return {this, find_or_add(okind::port, path)};
    }
    /// A state domain or a field of one.
    constexpr target state(std::string_view path) noexcept {
        return {this, find_or_add(okind::state, path)};
    }
    /// A variable of the current stage, of element type t.
    constexpr target var(std::string_view name, etype t) noexcept {
        operand o;
        o.kind = okind::var;
        o.name = name_t{name};
        o.t = t;
        o.stage = static_cast<std::uint8_t>(n_stages - 1);
        return {this, add_operand(o)};
    }
    /// A local of element type t and the extents given, as index expressions of parameters.
    template <index_like... E>
    constexpr target local(std::string_view name, etype t, E... extents) noexcept {
        static_assert(sizeof...(E) <= max_rank, "cd: a local's rank is at most max_rank");
        operand o;
        o.kind = okind::local;
        o.name = name_t{name};
        o.t = t;
        o.rank = static_cast<std::uint8_t>(sizeof...(E));
        std::size_t k = 0;
        ((o.extent[k++] = to_ix(extents).n), ...);
        (void)k;  // unused for a scalar local
        return {this, add_operand(o)};
    }
    /// A local declared earlier, by name.
    constexpr target local(std::string_view name) noexcept {
        for (std::uint16_t i = 0; i < n_operands; ++i) {
            if (operands[i].kind == okind::local && operands[i].name.view() == name) {
                return {this, i};
            }
        }
        capacity_exceeded("cd: no local of that name is declared");
    }
    /// The local of that name, or nullptr.
    [[nodiscard]] constexpr const operand* local_named(const name_t& name) const noexcept {
        for (std::uint16_t i = 0; i < n_operands; ++i) {
            if (operands[i].kind == okind::local && operands[i].name == name) {
                return &operands[i];
            }
        }
        return nullptr;
    }

    /// An operator parameter of element type t: a scalar (rank 0) or a table (rank 1).
    constexpr target param(std::string_view name, etype t, std::uint8_t rank = 0) noexcept {
        operand o;
        o.kind = okind::param;
        o.name = name_t{name};
        o.t = t;
        o.rank = rank;
        return {this, add_operand(o)};
    }

    /// The firing's time, in ns.
    constexpr val time() noexcept {
        node x;
        x.o = op::time;
        x.t = etype::i64;
        return {this, add_node(x)};
    }
    /// An index expression as a value of type t.
    constexpr val value(ix e, etype t) noexcept {
        node x;
        x.o = op::ival;
        x.t = t;
        x.a = e.n;
        return {this, add_node(x)};
    }
    /// e, converted to type t.
    constexpr val cast(val e, etype t) noexcept {
        node x;
        x.o = op::cast;
        x.t = t;
        x.a = e.n;
        return {this, add_node(x)};
    }
    /// A literal of type t.
    constexpr val literal(double v, etype t) noexcept {
        node x;
        x.o = op::lit;
        x.t = t;
        x.v = v;
        return {this, add_node(x)};
    }

    // ── Regions ──

    /// The current stage's space.
    constexpr region all() const noexcept {
        return {};
    }
    /// The current stage's space, narrowed.
    template <class... C>
    constexpr region where(C... c) const noexcept {
        static_assert(sizeof...(C) <= max_region, "cd: a region has at most max_region constraints");
        region r;
        ((r.ids[r.n++] = c.n), ...);
        return r;
    }
    /// A face: an equality `i == e` pins index i to e; other constraints narrow.
    template <class... C>
    constexpr region face(C... c) const noexcept {
        region r = where(c...);
        r.face = true;
        return r;
    }

    // ── Definitions ──

    /// v(p) = e at every point p of the region.
    constexpr void eq(target v, val e, region r = {}) noexcept {
        def d;
        d.kind = dkind::eq;
        d.target = v.id;
        d.value = e.n;
        d.where = r;
        d.stage = static_cast<std::uint8_t>(n_stages - 1);
        add_def(d);
    }
    /// v(q) = e at every point q of the face.
    constexpr void init(target v, val e, region r) noexcept {
        def d;
        d.kind = dkind::init;
        d.target = v.id;
        d.value = e.n;
        d.where = r;
        d.where.face = true;
        d.stage = static_cast<std::uint8_t>(n_stages - 1);
        add_def(d);
    }
    /// The element p names = e, at every point of the region.
    constexpr void write(place p, val e, region r = {}) noexcept {
        def d;
        d.kind = dkind::write;
        d.target = p.target;
        d.args = p.args;
        d.nargs = p.nargs;
        d.value = e.n;
        d.where = r;
        d.stage = static_cast<std::uint8_t>(n_stages - 1);
        add_def(d);
    }
    /// The count of a bounded out port's or state's leading extent, after the firing.
    constexpr void set_count(target p, ix e) noexcept {
        if (n_counts == max_counts) {
            capacity_exceeded("cd: more than max_counts counts");
        }
        counts[n_counts++] = count_decl{p.id, e.n};
    }

    // ── Building blocks, for the handles ──

    constexpr std::uint16_t add_node(const node& x) noexcept {
        if (n_nodes == max_nodes) {
            capacity_exceeded("cd: more than max_nodes expression nodes");
        }
        nodes[n_nodes] = x;
        return n_nodes++;
    }
    constexpr std::uint16_t add_arg(std::uint16_t a) noexcept {
        if (n_args == max_args) {
            capacity_exceeded("cd: more than max_args index arguments");
        }
        args[n_args] = a;
        return n_args++;
    }
    constexpr std::uint16_t add_constraint(cmp c, std::uint16_t lhs, std::uint16_t rhs) noexcept {
        if (n_constraints == max_constraints) {
            capacity_exceeded("cd: more than max_constraints constraints");
        }
        constraints[n_constraints] = constraint{c, lhs, rhs};
        return n_constraints++;
    }
    constexpr ix to_ix(ix e) noexcept {
        return e;
    }
    template <std::integral I>
    constexpr ix to_ix(I k) noexcept {
        node x;
        x.o = op::iconst;
        x.k = static_cast<std::int64_t>(k);
        return {this, add_node(x)};
    }

private:
    constexpr std::uint16_t add_operand(const operand& o) noexcept {
        if (n_operands == max_operands) {
            capacity_exceeded("cd: more than max_operands operands");
        }
        operands[n_operands] = o;
        return n_operands++;
    }
    constexpr void add_def(const def& d) noexcept {
        if (n_defs == max_defs) {
            capacity_exceeded("cd: more than max_defs definitions");
        }
        defs[n_defs++] = d;
    }
    /// A port's or a state's operand, `name` or `name.field`; a local's, by name.
    constexpr std::uint16_t find_or_add(okind k, std::string_view path) noexcept {
        const std::size_t dot = path.find('.');
        const name_t name{path.substr(0, dot)};
        const name_t field{dot == std::string_view::npos ? std::string_view{} : path.substr(dot + 1)};
        for (std::uint16_t i = 0; i < n_operands; ++i) {
            const operand& o = operands[i];
            if (o.kind == k && o.name == name && o.field == field) {
                return i;
            }
        }
        operand o;
        o.kind = k;
        o.name = name;
        o.field = field;
        return add_operand(o);
    }
    constexpr ix add_param(std::string_view path, pkind k, std::size_t dim) noexcept {
        if (n_params == max_params) {
            capacity_exceeded("cd: more than max_params parameters");
        }
        param_decl p;
        // Named for reading: `w[1]` for an extent, `count(x)` for a count.
        std::array<char, max_name> buf{};
        std::size_t len = 0;
        auto put = [&](std::string_view part) {
            for (const char ch : part) {
                if (len == max_name) {
                    capacity_exceeded("cd: a parameter's name is longer than max_name");
                }
                buf[len++] = ch;
            }
        };
        if (k == pkind::count) {
            put("count(");
            put(path);
            put(")");
        } else {
            put(path);
            put("[");
            const char digit[2] = {static_cast<char>('0' + dim), '\0'};
            put(std::string_view{digit, 1});
            put("]");
        }
        p.name = name_t{std::string_view{buf.data(), len}};
        p.kind = k;
        const std::size_t dot = path.find('.');
        p.target = name_t{path.substr(0, dot)};
        p.field = name_t{dot == std::string_view::npos ? std::string_view{} : path.substr(dot + 1)};
        p.dim = static_cast<std::uint8_t>(dim);
        params[n_params] = p;
        node x;
        x.o = op::param;
        x.a = n_params;
        ++n_params;
        return {this, add_node(x)};
    }
};

// ── Handles: index expressions ─────────────────────────────────────

namespace detail {
constexpr ix ibin(op o, ix a, ix b) noexcept {
    node x;
    x.o = o;
    x.a = a.n;
    x.b = b.n;
    return {a.s, a.s->add_node(x)};
}
constexpr val vbin(op o, val a, val b) noexcept {
    node x;
    x.o = o;
    x.a = a.n;
    x.b = b.n;
    return {a.s, a.s->add_node(x)};
}
constexpr val vun(op o, val a) noexcept {
    node x;
    x.o = o;
    x.a = a.n;
    return {a.s, a.s->add_node(x)};
}
constexpr val vlit(system* s, double v) noexcept {
    node x;
    x.o = op::lit;
    x.v = v;
    return {s, s->add_node(x)};
}
}  // namespace detail

constexpr ix operator+(ix a, ix b) noexcept {
    return detail::ibin(op::iadd, a, b);
}
constexpr ix operator-(ix a, ix b) noexcept {
    return detail::ibin(op::isub, a, b);
}
constexpr ix operator*(ix a, ix b) noexcept {
    return detail::ibin(op::imul, a, b);
}
constexpr ix operator/(ix a, ix b) noexcept {
    return detail::ibin(op::idiv, a, b);
}
constexpr ix operator%(ix a, ix b) noexcept {
    return detail::ibin(op::imod, a, b);
}
constexpr ix operator^(ix a, ix b) noexcept {
    return detail::ibin(op::ixor, a, b);
}
constexpr ix operator<<(ix a, ix b) noexcept {
    return detail::ibin(op::ishl, a, b);
}
constexpr ix operator>>(ix a, ix b) noexcept {
    return detail::ibin(op::ishr, a, b);
}
template <std::integral I>
constexpr ix operator+(ix a, I k) noexcept {
    return a + a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator+(I k, ix a) noexcept {
    return a.s->to_ix(k) + a;
}
template <std::integral I>
constexpr ix operator-(ix a, I k) noexcept {
    return a - a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator-(I k, ix a) noexcept {
    return a.s->to_ix(k) - a;
}
template <std::integral I>
constexpr ix operator*(ix a, I k) noexcept {
    return a * a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator*(I k, ix a) noexcept {
    return a.s->to_ix(k) * a;
}
template <std::integral I>
constexpr ix operator/(ix a, I k) noexcept {
    return a / a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator%(ix a, I k) noexcept {
    return a % a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator^(ix a, I k) noexcept {
    return a ^ a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator<<(ix a, I k) noexcept {
    return a << a.s->to_ix(k);
}
template <std::integral I>
constexpr ix operator<<(I k, ix a) noexcept {
    return a.s->to_ix(k) << a;
}
template <std::integral I>
constexpr ix operator>>(ix a, I k) noexcept {
    return a >> a.s->to_ix(k);
}

// Constraints. Write the index first: `j == C - 1`, `i < R`, `0 <= i` as `i >= 0`.
constexpr cond operator==(ix a, ix b) noexcept {
    return {a.s, a.s->add_constraint(cmp::eq, a.n, b.n)};
}
constexpr cond operator<(ix a, ix b) noexcept {
    return {a.s, a.s->add_constraint(cmp::lt, a.n, b.n)};
}
constexpr cond operator<=(ix a, ix b) noexcept {
    return {a.s, a.s->add_constraint(cmp::le, a.n, b.n)};
}
constexpr cond operator>(ix a, ix b) noexcept {
    return {a.s, a.s->add_constraint(cmp::lt, b.n, a.n)};
}
constexpr cond operator>=(ix a, ix b) noexcept {
    return {a.s, a.s->add_constraint(cmp::le, b.n, a.n)};
}
template <std::integral I>
constexpr cond operator==(ix a, I k) noexcept {
    return a == a.s->to_ix(k);
}
template <std::integral I>
constexpr cond operator<(ix a, I k) noexcept {
    return a < a.s->to_ix(k);
}
template <std::integral I>
constexpr cond operator<=(ix a, I k) noexcept {
    return a <= a.s->to_ix(k);
}
template <std::integral I>
constexpr cond operator>(ix a, I k) noexcept {
    return a > a.s->to_ix(k);
}
template <std::integral I>
constexpr cond operator>=(ix a, I k) noexcept {
    return a >= a.s->to_ix(k);
}

// ── Handles: values ─────────────────────────────────────────────────

constexpr val operator+(val a, val b) noexcept {
    return detail::vbin(op::add, a, b);
}
constexpr val operator-(val a, val b) noexcept {
    return detail::vbin(op::sub, a, b);
}
constexpr val operator*(val a, val b) noexcept {
    return detail::vbin(op::mul, a, b);
}
constexpr val operator/(val a, val b) noexcept {
    return detail::vbin(op::div, a, b);
}
constexpr val operator-(val a) noexcept {
    return detail::vun(op::neg, a);
}
constexpr val operator+(val a, double k) noexcept {
    return a + detail::vlit(a.s, k);
}
constexpr val operator+(double k, val a) noexcept {
    return detail::vlit(a.s, k) + a;
}
constexpr val operator-(val a, double k) noexcept {
    return a - detail::vlit(a.s, k);
}
constexpr val operator-(double k, val a) noexcept {
    return detail::vlit(a.s, k) - a;
}
constexpr val operator*(val a, double k) noexcept {
    return a * detail::vlit(a.s, k);
}
constexpr val operator*(double k, val a) noexcept {
    return detail::vlit(a.s, k) * a;
}
constexpr val operator/(val a, double k) noexcept {
    return a / detail::vlit(a.s, k);
}
constexpr val operator/(double k, val a) noexcept {
    return detail::vlit(a.s, k) / a;
}

constexpr val sqrt(val a) noexcept {
    return detail::vun(op::sqrt, a);
}
constexpr val tanh(val a) noexcept {
    return detail::vun(op::tanh, a);
}
constexpr val exp(val a) noexcept {
    return detail::vun(op::exp, a);
}
constexpr val abs(val a) noexcept {
    return detail::vun(op::abs, a);
}
constexpr val conj(val a) noexcept {
    return detail::vun(op::conj, a);
}
constexpr val real(val a) noexcept {
    return detail::vun(op::real, a);
}
constexpr val imag(val a) noexcept {
    return detail::vun(op::imag, a);
}
/// The magnitude of a complex value.
constexpr val cabs(val a) noexcept {
    return detail::vun(op::cabs, a);
}
/// a < b ? x : y
constexpr val select_lt(val a, val b, val x, val y) noexcept {
    node n;
    n.o = op::sel_lt;
    n.a = a.n;
    n.b = b.n;
    n.c = x.n;
    n.d = y.n;
    return {a.s, a.s->add_node(n)};
}
/// a <= b ? x : y
constexpr val select_le(val a, val b, val x, val y) noexcept {
    node n;
    n.o = op::sel_le;
    n.a = a.n;
    n.b = b.n;
    n.c = x.n;
    n.d = y.n;
    return {a.s, a.s->add_node(n)};
}

template <index_like... A>
constexpr val target::operator()(A... a) const noexcept {
    static_assert(sizeof...(A) <= max_rank, "cd: a read has at most max_rank indices");
    std::array<std::uint16_t, sizeof...(A) + 1> ids{s->to_ix(a).n..., 0};
    node x;
    x.o = op::read;
    x.a = id;
    x.b = s->n_args;
    x.c = static_cast<std::uint16_t>(sizeof...(A));
    for (std::size_t i = 0; i < sizeof...(A); ++i) {
        s->add_arg(ids[i]);
    }
    return {s, s->add_node(x)};
}

template <index_like... A>
constexpr place target::at(A... a) const noexcept {
    static_assert(sizeof...(A) <= max_rank, "cd: a write has at most max_rank indices");
    std::array<std::uint16_t, sizeof...(A) + 1> ids{s->to_ix(a).n..., 0};
    place p;
    p.s = s;
    p.target = id;
    p.args = s->n_args;
    p.nargs = static_cast<std::uint16_t>(sizeof...(A));
    for (std::size_t i = 0; i < sizeof...(A); ++i) {
        s->add_arg(ids[i]);
    }
    return p;
}

// ── The ports a system is checked against ───────────────────────────

/// How a port or a state may be used.
enum class access : std::uint8_t { in, out, inout, state };

/// A port, a state, or a field of either, as the binding declares it, and
/// whether the handler reads (its ins) and writes (its outs) it.
struct port_info {
    name_t name;
    name_t field;
    access dir = access::in;
    etype t = etype::none;
    std::uint8_t rank = 0;
    std::array<std::int64_t, max_rank> extent{};
    bool bounded = false;  ///< the leading extent is bounded
    structure s = structure::dense;
    bool read = false;
    bool written = false;
};

/// A handler's ports and states, its state role, and whether it takes a time.
struct table {
    std::array<port_info, max_ports> p{};
    std::uint8_t n = 0;
    state_role role = state_role::none;
    bool timed = false;

    [[nodiscard]] constexpr const port_info* find(const name_t& name, const name_t& field) const noexcept {
        for (std::size_t i = 0; i < n; ++i) {
            if (p[i].name == name && p[i].field == field) {
                return &p[i];
            }
        }
        return nullptr;
    }
};

namespace detail {
template <class Tuple>
struct extents_info;
template <class... X>
struct extents_info<std::tuple<X...>> {
    static constexpr std::array<std::int64_t, max_rank> sizes = [] {
        std::array<std::int64_t, max_rank> a{};
        std::size_t k = 0;
        ((a[k++] = static_cast<std::int64_t>(X::size)), ...);
        (void)k;  // unused for a scalar, which has no extents
        return a;
    }();
    static constexpr bool leading_bounded = [] {
        constexpr std::array<bool, sizeof...(X) + 1> b{X::is_bounded..., false};
        return b[0];
    }();
};

template <class D>
constexpr port_info info_of_domain(std::string_view name, std::string_view field, access dir) noexcept {
    port_info p;
    p.name = name_t{name};
    p.field = name_t{field};
    p.dir = dir;
    p.t = etype_of<typename D::element_type>;
    p.rank = static_cast<std::uint8_t>(D::rank);
    p.extent = extents_info<typename D::extents>::sizes;
    p.bounded = extents_info<typename D::extents>::leading_bounded;
    p.s = D::kind;
    return p;
}

constexpr void push(table& t, const port_info& p) noexcept {
    if (t.n == max_ports) {
        capacity_exceeded("cd: more than max_ports ports and fields");
    }
    t.p[t.n++] = p;
}

template <class R, std::size_t... I>
constexpr void push_fields(table& t, std::string_view name, access dir, std::index_sequence<I...>) noexcept {
    (
        [&] {
            using F = std::tuple_element_t<I, typename R::field_types>;
            if constexpr (is_domain_v<F>) {
                push(t, info_of_domain<F>(name, R::names[I], dir));
            }
        }(),
        ...);
}

template <class X>
constexpr void push_data(table& t, std::string_view name, access dir) noexcept {
    if constexpr (is_domain_v<X>) {
        push(t, info_of_domain<X>(name, {}, dir));
    } else {
        push_fields<X>(t, name, dir, std::make_index_sequence<X::size>{});
    }
}

constexpr access access_of(direction d) noexcept {
    return d == direction::in ? access::in : (d == direction::out ? access::out : access::inout);
}

template <std::size_t K>
constexpr bool contains(const std::array<std::string_view, K>& names, std::string_view n) noexcept {
    for (const auto& x : names) {
        if (x == n) {
            return true;
        }
    }
    return false;
}

template <class H, class... P>
constexpr void push_ports(table& t, std::tuple<P...>*) noexcept {
    (
        [&] {
            const std::size_t first = t.n;
            push_data<typename P::type>(t, P::name, access_of(P::dir));
            for (std::size_t i = first; i < t.n; ++i) {
                t.p[i].read = contains(H::in::names, P::name);
                t.p[i].written = contains(H::out::names, P::name);
            }
        }(),
        ...);
}

template <class... S>
constexpr void push_states(table& t, std::tuple<S...>*) noexcept {
    (
        [&] {
            const std::size_t first = t.n;
            push_data<typename S::type>(t, S::name, access::state);
            for (std::size_t i = first; i < t.n; ++i) {
                t.p[i].read = true;
                t.p[i].written = true;
            }
        }(),
        ...);
}
}  // namespace detail

/// The table of binding B's ports and states, as handler H uses them.
template <class B, class H>
constexpr table table_of() noexcept {
    using Sig = typename B::signature;
    table t;
    detail::push_ports<H>(t, static_cast<typename Sig::port_list::types*>(nullptr));
    detail::push_states(t, static_cast<typename Sig::state_list::types*>(nullptr));
    t.role = H::state;
    t.timed = H::timed;
    return t;
}

// ── K1 and K2 ───────────────────────────────────────────────────────

/// What a check found: K, 0 if nothing; what, a reason within it; at, the
/// node, operand, parameter, index or definition it is about.
struct finding {
    std::uint8_t k = 0;
    std::uint8_t what = 0;
    std::uint16_t at = 0;
};

/// K1's and K2's reasons.
enum class reason : std::uint8_t {
    none,
    empty_name,      ///< at: an operand, or an index (+ max_operands)
    duplicate_name,  ///< at: an operand, or an index (+ max_operands)
    unknown_port,    ///< at: an operand
    not_readable,    ///< at: a read node
    not_writable,    ///< at: a definition
    arity,           ///< at: a read node, or a definition
    type,            ///< at: a node, or a definition
    stage,           ///< at: a node
    param_target,    ///< at: a parameter
    param_dim,       ///< at: a parameter
    param_count,     ///< at: a parameter
};

[[nodiscard]] std::string_view to_string(reason r) noexcept;

namespace detail {
/// The rank a read or a write of operand o needs.
constexpr int rank_of(const system& s, const table& t, std::uint16_t o) noexcept {
    const operand& x = s.operands[o];
    switch (x.kind) {
    case okind::var: {
        int r = 0;
        for (std::size_t i = 0; i < s.n_indices; ++i) {
            r += s.indices[i].stage == x.stage ? 1 : 0;
        }
        return r;
    }
    case okind::local:
    case okind::param:
        return x.rank;
    default: {
        const port_info* p = t.find(x.name, x.field);
        return p ? p->rank : -1;
    }
    }
}

constexpr etype type_of_operand(const system& s, const table& t, std::uint16_t o) noexcept {
    const operand& x = s.operands[o];
    if (x.kind == okind::port || x.kind == okind::state) {
        const port_info* p = t.find(x.name, x.field);
        return p ? p->t : etype::none;
    }
    return x.t;
}

/// The stage an index expression's indices belong to: -1 for none, -2 for two.
constexpr int stage_of_ix(const system& s, std::uint16_t n) noexcept {
    const node& x = s.nodes[n];
    if (x.o == op::index) {
        return s.indices[x.a].stage;
    }
    if (x.o == op::iconst || x.o == op::param) {
        return -1;
    }
    const int a = stage_of_ix(s, x.a);
    const int b = stage_of_ix(s, x.b);
    if (a == -2 || b == -2 || (a >= 0 && b >= 0 && a != b)) {
        return -2;
    }
    return a >= 0 ? a : b;
}

/// Types of the value nodes, bottom up; a literal takes its sibling's type.
/// Returns the first node whose operands disagree, or max_nodes.
constexpr std::uint16_t type_nodes(const system& s, const table& t, std::array<etype, max_nodes>& ty) noexcept {
    for (std::uint16_t i = 0; i < s.n_nodes; ++i) {
        const node& x = s.nodes[i];
        if (is_index_op(x.o)) {
            ty[i] = etype::i64;
            continue;
        }
        switch (x.o) {
        case op::lit:
            ty[i] = x.t;
            break;
        case op::read:
            ty[i] = type_of_operand(s, t, x.a);
            if (ty[i] == etype::none) {
                return i;
            }
            break;
        case op::time:
            ty[i] = etype::i64;
            break;
        case op::ival:
        case op::cast:
            ty[i] = x.t;
            if (x.t == etype::none || (x.o == op::cast && ty[x.a] == etype::none)) {
                return i;
            }
            break;
        case op::add:
        case op::sub:
        case op::mul:
        case op::div: {
            etype a = ty[x.a];
            etype b = ty[x.b];
            // A literal of no type takes its sibling's.
            if (a == etype::none && s.nodes[x.a].o == op::lit) {
                a = b;
                ty[x.a] = b;
            }
            if (b == etype::none && s.nodes[x.b].o == op::lit) {
                b = a;
                ty[x.b] = a;
            }
            if (a == etype::none || a != b) {
                return i;
            }
            ty[i] = a;
            break;
        }
        case op::neg:
            ty[i] = ty[x.a];
            if (ty[i] == etype::none) {
                return i;
            }
            break;
        case op::sqrt:
        case op::tanh:
        case op::exp:
        case op::abs:
            if (!is_real(ty[x.a])) {
                return i;
            }
            ty[i] = ty[x.a];
            break;
        case op::conj:
            if (!is_complex(ty[x.a])) {
                return i;
            }
            ty[i] = ty[x.a];
            break;
        case op::real:
        case op::imag:
        case op::cabs:
            if (!is_complex(ty[x.a])) {
                return i;
            }
            ty[i] = real_of(ty[x.a]);
            break;
        case op::sel_lt:
        case op::sel_le: {
            etype a = ty[x.a];
            etype b = ty[x.b];
            if (a == etype::none && s.nodes[x.a].o == op::lit) {
                a = b;
                ty[x.a] = b;
            }
            if (b == etype::none && s.nodes[x.b].o == op::lit) {
                b = a;
                ty[x.b] = a;
            }
            etype c = ty[x.c];
            etype d = ty[x.d];
            if (c == etype::none && s.nodes[x.c].o == op::lit) {
                c = d;
                ty[x.c] = d;
            }
            if (d == etype::none && s.nodes[x.d].o == op::lit) {
                d = c;
                ty[x.d] = c;
            }
            if (a == etype::none || a != b || is_complex(a) || c == etype::none || c != d) {
                return i;
            }
            ty[i] = c;
            break;
        }
        default:
            return i;
        }
    }
    return static_cast<std::uint16_t>(max_nodes);
}

/// Every index node under value node n belongs to `stage`; returns the first
/// that doesn't, or max_nodes.
constexpr std::uint16_t check_stage(const system& s, std::uint16_t n, int stage) noexcept {
    const node& x = s.nodes[n];
    if (is_index_op(x.o)) {
        const int st = stage_of_ix(s, n);
        return (st == -2 || (st >= 0 && st != stage)) ? n : static_cast<std::uint16_t>(max_nodes);
    }
    std::uint16_t bad = static_cast<std::uint16_t>(max_nodes);
    auto sub = [&](std::uint16_t c) {
        if (bad == max_nodes) {
            bad = check_stage(s, c, stage);
        }
    };
    switch (x.o) {
    case op::read:
        for (std::uint16_t k = 0; k < x.c; ++k) {
            sub(s.args[x.b + k]);
        }
        break;
    case op::ival:
    case op::cast:
    case op::neg:
    case op::sqrt:
    case op::tanh:
    case op::exp:
    case op::abs:
    case op::conj:
    case op::real:
    case op::imag:
    case op::cabs:
        sub(x.a);
        break;
    case op::add:
    case op::sub:
    case op::mul:
    case op::div:
        sub(x.a);
        sub(x.b);
        break;
    case op::sel_lt:
    case op::sel_le:
        sub(x.a);
        sub(x.b);
        sub(x.c);
        sub(x.d);
        break;
    default:
        break;
    }
    return bad;
}

constexpr bool is_port_or_state(okind k) noexcept {
    return k == okind::port || k == okind::state;
}

/// The first read of a port or a state under value node n that the table
/// refuses, or max_nodes.
constexpr std::uint16_t check_reads(const system& s, const table& t, std::uint16_t n, int stage, reason& why) noexcept {
    const node& x = s.nodes[n];
    if (is_index_op(x.o)) {
        return static_cast<std::uint16_t>(max_nodes);
    }
    if (x.o == op::read) {
        const operand& o = s.operands[x.a];
        if (is_port_or_state(o.kind)) {
            const port_info* p = t.find(o.name, o.field);
            if (p == nullptr || p->dir == access::out || !p->read) {
                why = reason::not_readable;
                return n;
            }
        }
        if (o.kind == okind::var && o.stage != stage) {
            why = reason::stage;
            return n;
        }
        if (o.kind == okind::param && x.c != o.rank) {
            why = reason::arity;
            return n;
        }
        if (rank_of(s, t, x.a) != x.c) {
            why = reason::arity;
            return n;
        }
        return static_cast<std::uint16_t>(max_nodes);
    }
    std::uint16_t bad = static_cast<std::uint16_t>(max_nodes);
    auto sub = [&](std::uint16_t c) {
        if (bad == max_nodes) {
            bad = check_reads(s, t, c, stage, why);
        }
    };
    switch (x.o) {
    case op::cast:
    case op::neg:
    case op::sqrt:
    case op::tanh:
    case op::exp:
    case op::abs:
    case op::conj:
    case op::real:
    case op::imag:
    case op::cabs:
        sub(x.a);
        break;
    case op::add:
    case op::sub:
    case op::mul:
    case op::div:
        sub(x.a);
        sub(x.b);
        break;
    case op::sel_lt:
    case op::sel_le:
        sub(x.a);
        sub(x.b);
        sub(x.c);
        sub(x.d);
        break;
    default:
        break;
    }
    return bad;
}
}  // namespace detail

/// K1 (see the file comment).
[[nodiscard]] constexpr finding k1(const system& s, const table& t) noexcept {
    constexpr std::uint8_t K = 1;
    // Names: indices, and the variables, locals and parameters among themselves.
    for (std::uint16_t i = 0; i < s.n_indices; ++i) {
        if (s.indices[i].name.empty()) {
            return {K, static_cast<std::uint8_t>(reason::empty_name), static_cast<std::uint16_t>(max_operands + i)};
        }
        for (std::uint16_t j = 0; j < i; ++j) {
            if (s.indices[j].name == s.indices[i].name) {
                return {
                    K, static_cast<std::uint8_t>(reason::duplicate_name), static_cast<std::uint16_t>(max_operands + i)};
            }
        }
    }
    for (std::uint16_t i = 0; i < s.n_operands; ++i) {
        const operand& o = s.operands[i];
        if (o.name.empty()) {
            return {K, static_cast<std::uint8_t>(reason::empty_name), i};
        }
        if (!detail::is_port_or_state(o.kind)) {
            for (std::uint16_t j = 0; j < i; ++j) {
                const operand& q = s.operands[j];
                if (!detail::is_port_or_state(q.kind) && q.name == o.name) {
                    return {K, static_cast<std::uint8_t>(reason::duplicate_name), i};
                }
            }
        } else {
            const port_info* p = t.find(o.name, o.field);
            const bool state = p != nullptr && p->dir == access::state;
            if (p == nullptr || state != (o.kind == okind::state)) {
                return {K, static_cast<std::uint8_t>(reason::unknown_port), i};
            }
        }
    }
    // Types.
    std::array<etype, max_nodes> ty{};
    if (const std::uint16_t bad = detail::type_nodes(s, t, ty); bad != max_nodes) {
        return {K, static_cast<std::uint8_t>(reason::type), bad};
    }
    // Definitions: their targets, ranks, types, stages, and reads.
    for (std::uint16_t i = 0; i < s.n_defs; ++i) {
        const def& d = s.defs[i];
        const operand& o = s.operands[d.target];
        if (d.kind == dkind::write) {
            if (o.kind == okind::var || o.kind == okind::param) {
                return {K, static_cast<std::uint8_t>(reason::not_writable), i};
            }
            if (detail::is_port_or_state(o.kind)) {
                const port_info* p = t.find(o.name, o.field);
                if (p == nullptr || p->dir == access::in || !p->written) {
                    return {K, static_cast<std::uint8_t>(reason::not_writable), i};
                }
            }
            if (detail::rank_of(s, t, d.target) != d.nargs) {
                return {K, static_cast<std::uint8_t>(reason::arity), i};
            }
            for (std::uint16_t k = 0; k < d.nargs; ++k) {
                const int st = detail::stage_of_ix(s, s.args[d.args + k]);
                if (st == -2 || (st >= 0 && st != d.stage)) {
                    return {K, static_cast<std::uint8_t>(reason::stage), s.args[d.args + k]};
                }
            }
        } else if (o.kind != okind::var || o.stage != d.stage) {
            return {K, static_cast<std::uint8_t>(reason::not_writable), i};
        }
        // The value's type is the target's; a literal takes it.
        const etype want = detail::type_of_operand(s, t, d.target);
        etype got = ty[d.value];
        if (got == etype::none && s.nodes[d.value].o == op::lit) {
            got = want;
        }
        if (got != want) {
            return {K, static_cast<std::uint8_t>(reason::type), static_cast<std::uint16_t>(i)};
        }
        if (const std::uint16_t bad = detail::check_stage(s, d.value, d.stage); bad != max_nodes) {
            return {K, static_cast<std::uint8_t>(reason::stage), bad};
        }
        for (std::uint8_t k = 0; k < d.where.n; ++k) {
            const constraint& c = s.constraints[d.where.ids[k]];
            for (const std::uint16_t side : {c.lhs, c.rhs}) {
                const int st = detail::stage_of_ix(s, side);
                if (st == -2 || (st >= 0 && st != d.stage)) {
                    return {K, static_cast<std::uint8_t>(reason::stage), side};
                }
            }
        }
        reason why = reason::none;
        if (const std::uint16_t bad = detail::check_reads(s, t, d.value, d.stage, why); bad != max_nodes) {
            return {K, static_cast<std::uint8_t>(why), bad};
        }
    }
    // The time, only in a handler that takes one.
    for (std::uint16_t i = 0; i < s.n_nodes; ++i) {
        if (s.nodes[i].o == op::time && !t.timed) {
            return {K, static_cast<std::uint8_t>(reason::not_readable), i};
        }
    }
    // Index bounds: the stage's own earlier indices, and parameters.
    for (std::uint16_t i = 0; i < s.n_indices; ++i) {
        for (const std::uint16_t side : {s.indices[i].lo, s.indices[i].hi}) {
            const int st = detail::stage_of_ix(s, side);
            if (st == -2 || (st >= 0 && st != s.indices[i].stage)) {
                return {K, static_cast<std::uint8_t>(reason::stage), side};
            }
        }
    }
    return {};
}

/// K2 (see the file comment).
[[nodiscard]] constexpr finding k2(const system& s, const table& t) noexcept {
    constexpr std::uint8_t K = 2;
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const param_decl& p = s.params[i];
        if (p.kind == pkind::integer) {
            continue;
        }
        if (const operand* o = s.local_named(p.target); o != nullptr && p.field.empty()) {
            if (p.kind == pkind::count || p.dim >= o->rank) {
                return {
                    K, static_cast<std::uint8_t>(p.kind == pkind::count ? reason::param_count : reason::param_dim), i};
            }
            continue;
        }
        const port_info* q = t.find(p.target, p.field);
        if (q == nullptr) {
            return {K, static_cast<std::uint8_t>(reason::param_target), i};
        }
        if (p.dim >= q->rank) {
            return {K, static_cast<std::uint8_t>(reason::param_dim), i};
        }
        if (p.kind == pkind::count && !q->bounded) {
            return {K, static_cast<std::uint8_t>(reason::param_count), i};
        }
    }
    return {};
}

// ── The operator's members, for the evaluator ───────────────────────

/// A member of an operator that holds a state domain or an operator
/// parameter, named as the computational domain names it. A binding lists
/// them, so the evaluator (workbench/evaluate.hpp) can read the operator:
///   using cd_state  = cd::members<cd::member<"line", &Delay::line>>;
///   using cd_params = cd::members<cd::member<"twiddle", &Fft::twiddle>>;
template <fixed_string Name, auto Ptr>
struct member {
    static constexpr std::string_view name = Name.view();
    static constexpr auto ptr = Ptr;
};

template <class... M>
struct members {
    using types = std::tuple<M...>;
};

// ── K3–K8, counts and depth: enumerated, at run time ────────────────

/// How a read's index function depends on the point (K8), and so a stage:
/// uniform (a SURE) if every variable read is, affine if every read is.
enum class shape : std::uint8_t { uniform, affine, non_affine };

[[nodiscard]] std::string_view to_string(shape s) noexcept;

/// An invariant broken: its id (K1–K8) and where.
struct problem {
    std::string id;
    std::string what;
};

/// The operations of one kind and element type, summed over the space.
struct op_count {
    op o = op::add;
    etype t = etype::none;
    std::uint64_t n = 0;
};

/// What cd::analyze found.
struct report {
    std::vector<problem> problems;     ///< empty: K1–K7 hold
    std::vector<shape> stages;         ///< K8: each stage's class
    std::vector<shape> reads;          ///< K8: each read node's class, by node (max_nodes for none)
    std::vector<op_count> counts;      ///< at the bound of every count
    std::uint64_t definitions = 0;     ///< the definitions at points, at the bounds
    std::uint64_t depth = 0;           ///< the longest chain of dependences, in definitions
    std::vector<std::uint64_t> width;  ///< the definitions at each depth

    [[nodiscard]] bool ok() const noexcept {
        return problems.empty();
    }
    /// The operations of kind o, of every element type, or of type t.
    [[nodiscard]] std::uint64_t count(op o) const noexcept;
    [[nodiscard]] std::uint64_t count(op o, etype t) const noexcept;
    /// Whether some problem has this id.
    [[nodiscard]] bool has(std::string_view id) const noexcept;
};

/// K1–K8, the counts and the depth of a system against a table. K3–K7 are
/// checked for every count of the bounded extents in [0, bound] (or, past
/// 4096 combinations, for 0, 1 and the bound of each); the counts and the
/// depth are at the bounds.
[[nodiscard]] report analyze(const system& s, const table& t);

/// The system as text, for reading: its parameters bound, each stage's
/// space, its definitions, its counts and its classes.
[[nodiscard]] std::string text(const system& s, const table& t);

// ── The dependence graph, for the evaluator ─────────────────────────

/// A system's definitions at points, for one binding of its counts.
struct dgraph {
    struct dnode {
        std::uint16_t def = 0;
        std::uint32_t point = 0;    ///< its coordinates, from coords[point], the stage's rank of them
        std::int64_t element = -1;  ///< a write's stored element
        std::uint32_t src = 0;      ///< the definitions its reads of variables and locals read, from srcs[src]
        std::uint32_t nsrc = 0;
    };
    std::vector<dnode> nodes;
    std::vector<std::int64_t> coords;
    std::vector<std::uint32_t> srcs;
    std::vector<std::int64_t> params;  ///< each parameter's value
    std::vector<std::int64_t> counts;  ///< each count_decl's value
    std::vector<problem> problems;     ///< K3–K6 for this binding
};

/// The graph of s for the counts given: one per parameter of kind count, in
/// order (others ignored). Reads of variables and locals are resolved in
/// cd::postorder's order of each definition's value.
[[nodiscard]] dgraph build_graph(const system& s, const table& t, const std::vector<std::int64_t>& counts);

/// The value nodes under `root`, each once, children before parents (a, b,
/// c, d in that order): the order in which build_graph resolves a
/// definition's reads, and the evaluator evaluates it.
[[nodiscard]] std::vector<std::uint16_t> postorder(const system& s, std::uint16_t root);

/// The rank of a stage: its number of indices.
[[nodiscard]] int stage_rank(const system& s, int stage) noexcept;

/// An integer expression's value at a point (the stage's indices, in order) for the parameters given.
[[nodiscard]] std::int64_t
eval_ix(const system& s, std::uint16_t n, const std::int64_t* point, const std::vector<std::int64_t>& params) noexcept;

/// The stored element (i, j, ...) of a domain names, or -1 if it is outside
/// it: past an extent, past a bounded leading extent's count, or outside a
/// triangle or a diagonal. A symmetric domain's (i, j) and (j, i) are one.
[[nodiscard]] std::int64_t element_of(const port_info& p, std::int64_t count, const std::int64_t* idx) noexcept;

/// The elements a domain stores.
[[nodiscard]] std::int64_t stored_of(const port_info& p) noexcept;

}  // namespace branes::anatomy::flow::cd
