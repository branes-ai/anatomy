#pragma once
// The toy operators of the flow graph's differential tests
// (docs/plan/flow-testing): operators of their own over a time and an
// unsigned integer, so that every comparison is exact. Each kind is a
// template over a salt, registered once per salt (Unary1, Unary2, ...), so
// two operators of one kind compute different functions, and the salt is in
// the name the Graph holds. The reference interpreter (interpreter.hpp)
// implements them again, from the plan's table, without this header.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/util/bytes.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <vector>

namespace toys {

namespace fl = branes::anatomy::flow;
using branes::anatomy::Timestamp;

// ── Messages ────────────────────────────────────────────────────────

/// A value: its time, and its number.
struct Num {
    Timestamp t;
    std::uint64_t v = 0;
};

/// A record of two values, which a source projects.
struct Two {
    Num a;
    Num b;
};

/// The toys' codec, for their traces: each field, little-endian (util/bytes.hpp).
[[nodiscard]] inline std::vector<std::byte> encode(const Num& x) {
    branes::anatomy::ByteWriter w;
    w.i64(x.t.ns);
    w.u64(x.v);
    return w.take();
}
[[nodiscard]] inline std::vector<std::byte> encode(const Two& x) {
    branes::anatomy::ByteWriter w;
    w.i64(x.a.t.ns);
    w.u64(x.a.v);
    w.i64(x.b.t.ns);
    w.u64(x.b.v);
    return w.take();
}

/// The mixing function every toy operator computes with: splitmix64 of x,
/// keyed by the salt.
[[nodiscard]] constexpr std::uint64_t mix(std::uint64_t x, std::uint64_t salt) noexcept {
    std::uint64_t z = x ^ (salt * 0x9E3779B97F4A7C15ULL);
    z += 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

}  // namespace toys

namespace branes::anatomy::flow {

template <>
struct domain_of<toys::Num> {
    using type = scalar<std::uint64_t>;
};
template <>
struct domain_of<toys::Two> {
    using type = record<field<"a", scalar<std::uint64_t>>, field<"b", scalar<std::uint64_t>>>;
};

}  // namespace branes::anatomy::flow

namespace toys {

using fl::direction;
using fl::ins;
using fl::outs;
using fl::port;
using fl::state_role;

// ── Operators ───────────────────────────────────────────────────────

template <std::uint64_t S>
struct Unary {
    [[nodiscard]] Num apply(const Num& x) const {
        return Num{x.t, mix(x.v, S)};
    }
};

/// Its arguments' order matters, so a swapped edge shows.
template <std::uint64_t S>
struct Binary {
    [[nodiscard]] Num apply(const Num& x, const Num& y) const {
        return Num{std::max(x.t, y.t), mix(x.v * 3 + y.v, S)};
    }
};

/// The only toy with state: add() updates its sum, and read() and peek()
/// read it, at a time and at none.
template <std::uint64_t S>
struct Counter {
    std::uint64_t sum = 0;

    void add(const Num& x) {
        sum = mix(sum + x.v, S);
    }
    [[nodiscard]] Num read(Timestamp t) const {
        return Num{t, sum};
    }
    [[nodiscard]] Num peek() const {
        return Num{Timestamp{}, sum};
    }
};

template <std::uint64_t S>
struct Clock {
    [[nodiscard]] Num at(Timestamp t) const {
        return Num{t, mix(static_cast<std::uint64_t>(t.ns), S)};
    }
};

template <std::uint64_t S>
struct Pair {
    [[nodiscard]] std::tuple<Num, Num> apply(const Num& x) const {
        return {Num{x.t, mix(x.v, S)}, Num{x.t, mix(x.v, S + 100)}};
    }
};

template <std::uint64_t S>
struct Rec {
    [[nodiscard]] Two apply(const Num& x) const {
        return Two{Num{x.t, mix(x.v, S + 200)}, Num{x.t, mix(x.v, S + 300)}};
    }
};

/// Two updates of one state, for the mutations only (T2): a region that
/// fires both breaks G5. The generator never draws it, and it never runs.
template <std::uint64_t S>
struct Acc {
    std::uint64_t sum = 0;

    void add(const Num& x) {
        sum = mix(sum + x.v, S);
    }
    void scale(const Num& x) {
        sum = mix(sum * x.v, S);
    }
};

// ── Bindings ────────────────────────────────────────────────────────

struct UnaryBinding {
    using signature = fl::signature<fl::ports<port<"x", Num, direction::in>, port<"y", Num, direction::out>>>;
    struct apply : fl::handler<"apply", ins<"x">, outs<"y">> {
        static Num call(const auto& op, const Num& x) {
            return op.apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

struct BinaryBinding {
    using signature = fl::signature<
        fl::ports<port<"x", Num, direction::in>, port<"y", Num, direction::in>, port<"z", Num, direction::out>>>;
    struct apply : fl::handler<"apply", ins<"x", "y">, outs<"z">> {
        static Num call(const auto& op, const Num& x, const Num& y) {
            return op.apply(x, y);
        }
    };
    using handlers = fl::handlers<apply>;
};

struct CounterBinding {
    using signature = fl::signature<fl::ports<port<"x", Num, direction::in>, port<"y", Num, direction::out>>>;
    struct add : fl::handler<"add", ins<"x">, outs<>, state_role::update> {
        static void call(auto& op, const Num& x) {
            op.add(x);
        }
    };
    struct read : fl::handler<"read", ins<>, outs<"y">, state_role::read, fl::takes_time::yes> {
        static Num call(const auto& op, Timestamp t) {
            return op.read(t);
        }
    };
    struct peek : fl::handler<"peek", ins<>, outs<"y">, state_role::read> {
        static Num call(const auto& op) {
            return op.peek();
        }
    };
    using handlers = fl::handlers<add, read, peek>;
};

struct ClockBinding {
    using signature = fl::signature<fl::ports<port<"y", Num, direction::out>>>;
    struct at : fl::handler<"at", ins<>, outs<"y">, state_role::none, fl::takes_time::yes> {
        static Num call(const auto& op, Timestamp t) {
            return op.at(t);
        }
    };
    using handlers = fl::handlers<at>;
};

struct PairBinding {
    using signature = fl::signature<
        fl::ports<port<"x", Num, direction::in>, port<"y", Num, direction::out>, port<"z", Num, direction::out>>>;
    struct apply : fl::handler<"apply", ins<"x">, outs<"y", "z">> {
        static std::tuple<Num, Num> call(const auto& op, const Num& x) {
            return op.apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

struct RecBinding {
    using signature = fl::signature<fl::ports<port<"x", Num, direction::in>, port<"r", Two, direction::out>>>;
    struct apply : fl::handler<"apply", ins<"x">, outs<"r">> {
        static Two call(const auto& op, const Num& x) {
            return op.apply(x);
        }
    };
    using handlers = fl::handlers<apply>;
};

/// Its out port is unused: a signature has one (S2).
struct AccBinding {
    using signature = fl::signature<fl::ports<port<"x", Num, direction::in>, port<"y", Num, direction::out>>>;
    struct add : fl::handler<"add", ins<"x">, outs<>, state_role::update> {
        static void call(auto& op, const Num& x) {
            op.add(x);
        }
    };
    struct scale : fl::handler<"scale", ins<"x">, outs<>, state_role::update> {
        static void call(auto& op, const Num& x) {
            op.scale(x);
        }
    };
    using handlers = fl::handlers<add, scale>;
};

// ── The registry: each kind, once per salt ──────────────────────────

using Unary1 = fl::registered<"Unary1", Unary<1>, UnaryBinding>;
using Unary2 = fl::registered<"Unary2", Unary<2>, UnaryBinding>;
using Unary3 = fl::registered<"Unary3", Unary<3>, UnaryBinding>;
using Binary1 = fl::registered<"Binary1", Binary<1>, BinaryBinding>;
using Binary2 = fl::registered<"Binary2", Binary<2>, BinaryBinding>;
using Binary3 = fl::registered<"Binary3", Binary<3>, BinaryBinding>;
using Counter1 = fl::registered<"Counter1", Counter<1>, CounterBinding>;
using Counter2 = fl::registered<"Counter2", Counter<2>, CounterBinding>;
using Counter3 = fl::registered<"Counter3", Counter<3>, CounterBinding>;
using Clock1 = fl::registered<"Clock1", Clock<1>, ClockBinding>;
using Clock2 = fl::registered<"Clock2", Clock<2>, ClockBinding>;
using Clock3 = fl::registered<"Clock3", Clock<3>, ClockBinding>;
using Pair1 = fl::registered<"Pair1", Pair<1>, PairBinding>;
using Pair2 = fl::registered<"Pair2", Pair<2>, PairBinding>;
using Pair3 = fl::registered<"Pair3", Pair<3>, PairBinding>;
using Rec1 = fl::registered<"Rec1", Rec<1>, RecBinding>;
using Rec2 = fl::registered<"Rec2", Rec<2>, RecBinding>;
using Rec3 = fl::registered<"Rec3", Rec<3>, RecBinding>;
using Acc1 = fl::registered<"Acc1", Acc<1>, AccBinding>;

// ── Bodies of several operators ─────────────────────────────────────

/// A unary composite: Unary1, then Unary2 of its result.
struct UnaryChain {
    static constexpr std::string_view name = "UnaryChain";
    using binding = UnaryBinding;

    static void build(auto& b) {
        const auto first = fl::instance<Unary1>(b, "first");
        const auto second = fl::instance<Unary2>(b, "second");
        fl::on<"apply">(b, [=](fl::scope& s, auto x) {
            auto [y] = fl::fire<"apply">(s, first, x);
            return fl::fire<"apply">(s, second, y);
        });
    }
};

/// A binary composite: Unary3 of its first argument, then Binary1.
struct BinaryPre {
    static constexpr std::string_view name = "BinaryPre";
    using binding = BinaryBinding;

    static void build(auto& b) {
        const auto pre = fl::instance<Unary3>(b, "pre");
        const auto join = fl::instance<Binary1>(b, "join");
        fl::on<"apply">(b, [=](fl::scope& s, auto x, auto y) {
            auto [px] = fl::fire<"apply">(s, pre, x);
            return fl::fire<"apply">(s, join, px, y);
        });
    }
};

/// What the toy operators are built from: nothing.
struct Context {};

}  // namespace toys
