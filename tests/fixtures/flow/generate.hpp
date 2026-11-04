#pragma once
// Random graphs of toy operators (toys.hpp), for the flow graph's
// differential tests (docs/plan/flow-testing, T1). A graph is generated as a
// recipe: a plain description of its inputs, constants, nodes and firings,
// and of each in port's source, which prints as text, so a failing case can
// be read and rerun from its seed. build() describes the recipe to a
// stack_builder, the API missions use, and returns the stack with its
// external ports. A recipe can also say what makes it invalid, for the
// mutations (mutate.hpp, T2): a node placed against its graph's class, a
// time taken from another region's input, a second region for a trigger, and
// an output with no producer, or with two.
//
// The randomness is std::mt19937_64's raw output, whose sequence the
// standard fixes, mapped to ranges here: the standard distributions differ
// between GCC, Clang and MSVC, and a seed must give one graph on all three.

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/result.hpp>

#include "toys.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <random>
#include <span>
#include <sstream>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

namespace gen {

namespace fl = branes::anatomy::flow;
using branes::anatomy::Timestamp;
using toys::Num;
using toys::Two;

// ── Recipes ─────────────────────────────────────────────────────────

/// The kinds the generator draws, then Acc, which only the mutations use.
enum class Kind : std::uint8_t { unary, binary, counter, clock, pair, rec, acc };
inline constexpr std::size_t kKinds = 6;

/// A handler of a toy kind, as the generator needs it.
struct HandlerSpec {
    const char* name;
    std::size_t ins;
    bool timed;
    std::vector<bool> outs;  ///< per out port: true for a record (Two), false for a value (Num)
};

[[nodiscard]] inline const std::vector<HandlerSpec>& handlers_of(Kind k) {
    static const std::vector<std::vector<HandlerSpec>> table = {
        {{"apply", 1, false, {false}}},
        {{"apply", 2, false, {false}}},
        {{"add", 1, false, {}}, {"read", 0, true, {false}}, {"peek", 0, false, {false}}},
        {{"at", 0, true, {false}}},
        {{"apply", 1, false, {false, false}}},
        {{"apply", 1, false, {true}}},
        {{"add", 1, false, {}}, {"scale", 1, false, {}}},
    };
    return table[static_cast<std::size_t>(k)];
}

[[nodiscard]] inline const char* kind_name(Kind k) {
    static constexpr const char* names[] = {"Unary", "Binary", "Counter", "Clock", "Pair", "Rec", "Acc"};
    return names[static_cast<std::size_t>(k)];
}

struct InputSpec {
    std::string name;
    bool tick = false;    ///< carries only its time
    bool region = false;  ///< fires a region of its own
};

struct ConstantSpec {
    std::string name;
    Num value;
};

struct NodeSpec {
    std::string name;
    Kind kind = Kind::unary;
    unsigned salt = 1;  ///< 1 to 3; Acc has only 1
    bool body = false;  ///< a body of several operators (UnaryChain, BinaryPre): scoreboard only
};

/// Where a node goes against its graph's class: as an instance, or as a
/// composite. A node not placed goes where its class has it.
enum class Place : std::uint8_t { instance, composite };

struct SourceSpec {
    enum class From : std::uint8_t { input, constant, firing };
    From from = From::input;
    std::size_t index = 0;  ///< the input, the constant, or the firing (in the recipe's order)
    std::size_t port = 0;   ///< the firing's out port
    int field = -1;         ///< the field projected from a record: 0 for "a", 1 for "b"; else -1
    bool hold = false;
};

struct FiringSpec {
    std::size_t node = 0;
    std::size_t handler = 0;
    std::size_t region = 0;  ///< the input whose region it fires in
    std::vector<SourceSpec> in;
};

/// A graph, as data. Its firings are in the order they are described, which
/// interleaves the regions; each region runs its own in that order.
struct Recipe {
    std::uint64_t seed = 0;
    fl::baseline_class kind = fl::baseline_class::research;
    std::vector<InputSpec> inputs;
    std::vector<ConstantSpec> constants;
    std::vector<NodeSpec> nodes;
    std::vector<FiringSpec> firings;
    std::map<std::size_t, Place> placed;       ///< node → where it goes against the class
    std::map<std::size_t, std::size_t> at;     ///< firing → the input whose time it takes, not its region's
    std::optional<std::size_t> second_region;  ///< an input whose trigger is given a second region
    bool unproduced = false;                   ///< an output, "ox", that nothing produces
    bool produced_twice = false;               ///< the first output produced again
};

[[nodiscard]] inline std::string to_string(const Recipe& r) {
    std::ostringstream o;
    o << "recipe seed " << r.seed << ", " << (r.kind == fl::baseline_class::research ? "research" : "scoreboard")
      << "\n";
    for (const InputSpec& in : r.inputs) {
        o << "  input " << in.name << (in.tick ? " tick" : "") << (in.region ? "" : " (no region)") << "\n";
    }
    for (const ConstantSpec& c : r.constants) {
        o << "  constant " << c.name << " = (" << c.value.t.ns << ", " << c.value.v << ")\n";
    }
    for (std::size_t i = 0; i < r.nodes.size(); ++i) {
        const NodeSpec& n = r.nodes[i];
        const auto placed = r.placed.find(i);
        o << "  node " << n.name << ": "
          << (n.body ? (n.kind == Kind::unary ? "UnaryChain" : "BinaryPre")
                     : std::string(kind_name(n.kind)) + std::to_string(n.salt))
          << (placed == r.placed.end()            ? ""
              : placed->second == Place::instance ? ", as an instance"
                                                  : ", as a composite")
          << "\n";
    }
    for (std::size_t i = 0; i < r.firings.size(); ++i) {
        const FiringSpec& f = r.firings[i];
        const HandlerSpec& h = handlers_of(r.nodes[f.node].kind)[f.handler];
        o << "  #" << i << " in " << r.inputs[f.region].name << ": " << r.nodes[f.node].name << "." << h.name << "(";
        if (h.timed) {
            const auto at = r.at.find(i);
            o << "at " << r.inputs[at == r.at.end() ? f.region : at->second].name << (f.in.empty() ? "" : ", ");
        }
        for (std::size_t k = 0; k < f.in.size(); ++k) {
            const SourceSpec& s = f.in[k];
            std::string what;
            switch (s.from) {
            case SourceSpec::From::input:
                what = r.inputs[s.index].name;
                break;
            case SourceSpec::From::constant:
                what = r.constants[s.index].name;
                break;
            case SourceSpec::From::firing:
                what = "#" + std::to_string(s.index) + "." + std::to_string(s.port) +
                       (s.field < 0 ? "" : (s.field == 0 ? ".a" : ".b"));
                break;
            }
            o << (k > 0 ? ", " : "") << (s.hold ? "hold(" + what + ")" : what);
        }
        o << ")\n";
    }
    if (r.second_region) {
        o << "  a second region for " << r.inputs[*r.second_region].name << "\n";
    }
    if (r.unproduced) {
        o << "  an output ox that nothing produces\n";
    }
    if (r.produced_twice) {
        o << "  the first output produced twice\n";
    }
    return o.str();
}

// ── The generator ───────────────────────────────────────────────────

class Rng {
public:
    explicit Rng(std::uint64_t seed) : e_(seed) {}

    [[nodiscard]] std::uint64_t next() {
        return e_();
    }
    /// A number in [0, n). Pre: n > 0.
    [[nodiscard]] std::size_t below(std::size_t n) {
        return static_cast<std::size_t>(e_() % n);
    }
    /// True with probability num / den.
    [[nodiscard]] bool chance(std::size_t num, std::size_t den) {
        return below(den) < num;
    }

private:
    std::mt19937_64 e_;
};

/// A random valid graph. Each untimed handler fires once, in one region,
/// and each time handler in some regions, once in each (G6); Counter.add is
/// the only update, so no region updates a state twice (G5). Every in port's
/// source is drawn from what the region can read: its input, the constants,
/// and its earlier firings' out ports, directly; and hold() of every other
/// input, and of every firing described before it in another region (G7).
[[nodiscard]] inline Recipe generate(std::uint64_t seed) {
    Rng rng(seed);
    Recipe r;
    r.seed = seed;
    r.kind = rng.chance(1, 2) ? fl::baseline_class::research : fl::baseline_class::scoreboard;

    // The inputs: the first a value with a region, so every in port has a source.
    const std::size_t n_inputs = 1 + rng.below(4);
    for (std::size_t i = 0; i < n_inputs; ++i) {
        InputSpec in{"in" + std::to_string(i), false, true};
        if (i > 0) {
            in.tick = rng.chance(1, 3);
            in.region = in.tick || rng.chance(3, 4);
        }
        r.inputs.push_back(in);
    }
    std::vector<std::size_t> regions;
    for (std::size_t i = 0; i < r.inputs.size(); ++i) {
        if (r.inputs[i].region) {
            regions.push_back(i);
        }
    }
    const std::size_t n_constants = rng.below(3);
    for (std::size_t i = 0; i < n_constants; ++i) {
        const auto t = static_cast<std::int64_t>(rng.below(1000));
        r.constants.push_back(ConstantSpec{"c" + std::to_string(i), Num{Timestamp{t}, rng.next()}});
    }

    // The nodes, and where each handler fires.
    const std::size_t n_nodes = 1 + rng.below(6);
    for (std::size_t i = 0; i < n_nodes; ++i) {
        NodeSpec n;
        n.name = "n" + std::to_string(i);
        n.kind = static_cast<Kind>(rng.below(kKinds));
        n.salt = static_cast<unsigned>(1 + rng.below(3));
        n.body = r.kind == fl::baseline_class::scoreboard && (n.kind == Kind::unary || n.kind == Kind::binary) &&
                 rng.chance(1, 3);
        r.nodes.push_back(n);
        const auto& hs = handlers_of(n.kind);
        for (std::size_t h = 0; h < hs.size(); ++h) {
            if (!hs[h].timed) {
                r.firings.push_back(FiringSpec{i, h, regions[rng.below(regions.size())], {}});
                continue;
            }
            std::vector<std::size_t> at;
            for (const std::size_t region : regions) {
                if (rng.chance(1, 2)) {
                    at.push_back(region);
                }
            }
            if (at.empty()) {
                at.push_back(regions[rng.below(regions.size())]);
            }
            for (const std::size_t region : at) {
                r.firings.push_back(FiringSpec{i, h, region, {}});
            }
        }
    }
    for (std::size_t i = r.firings.size(); i > 1; --i) {
        std::swap(r.firings[i - 1], r.firings[rng.below(i)]);
    }

    // Each in port's source.
    for (std::size_t i = 0; i < r.firings.size(); ++i) {
        FiringSpec& f = r.firings[i];
        std::vector<SourceSpec> can;
        using From = SourceSpec::From;
        for (std::size_t k = 0; k < r.inputs.size(); ++k) {
            if (!r.inputs[k].tick) {
                can.push_back(SourceSpec{From::input, k, 0, -1, k != f.region});
            }
        }
        for (std::size_t k = 0; k < r.constants.size(); ++k) {
            can.push_back(SourceSpec{From::constant, k, 0, -1, false});
        }
        for (std::size_t d = 0; d < i; ++d) {
            const FiringSpec& e = r.firings[d];
            const auto& outs = handlers_of(r.nodes[e.node].kind)[e.handler].outs;
            for (std::size_t p = 0; p < outs.size(); ++p) {
                if (outs[p]) {
                    can.push_back(SourceSpec{From::firing, d, p, 0, e.region != f.region});
                    can.push_back(SourceSpec{From::firing, d, p, 1, e.region != f.region});
                } else {
                    can.push_back(SourceSpec{From::firing, d, p, -1, e.region != f.region});
                }
            }
        }
        for (std::size_t k = 0; k < handlers_of(r.nodes[f.node].kind)[f.handler].ins; ++k) {
            f.in.push_back(can[rng.below(can.size())]);
        }
    }
    return r;
}

// ── The build ───────────────────────────────────────────────────────

using Input = std::variant<fl::input_port<Num>, fl::input_port<Timestamp>>;

/// A recipe's stack, its inputs in the recipe's order, and its outputs: one
/// per value each firing writes, a record's two fields each.
struct Built {
    fl::stack stack;
    std::vector<Input> inputs;
    std::vector<fl::output_port<Num>> outputs;
};

namespace detail {

using Node = std::variant<fl::node<toys::UnaryBinding>,
                          fl::node<toys::BinaryBinding>,
                          fl::node<toys::CounterBinding>,
                          fl::node<toys::ClockBinding>,
                          fl::node<toys::PairBinding>,
                          fl::node<toys::RecBinding>,
                          fl::node<toys::AccBinding>>;
using Out = std::variant<fl::value<Num>, fl::value<Two>>;

/// A source, as a firing takes it: a value, held or not.
struct Src {
    fl::value<Num> v;
    bool held = false;
};

template <class F>
auto with(const Src& s, F f) {
    return s.held ? f(fl::hold(s.v)) : f(s.v);
}

template <class... X>
std::vector<Out> outs_of(const std::tuple<fl::value<X>...>& t) {
    return std::apply([](const auto&... v) { return std::vector<Out>{Out{v}...}; }, t);
}

/// f(std::type_identity<Op>) for the registered operator of a kind and a salt.
template <class F>
void with_op(Kind k, unsigned salt, F f) {
    using namespace toys;
    const auto pick = [&]<class A, class B, class C>(
                          std::type_identity<A>, std::type_identity<B>, std::type_identity<C>) {
        salt == 1 ? f(std::type_identity<A>{}) : salt == 2 ? f(std::type_identity<B>{}) : f(std::type_identity<C>{});
    };
    switch (k) {
    case Kind::unary:
        return pick(std::type_identity<Unary1>{}, std::type_identity<Unary2>{}, std::type_identity<Unary3>{});
    case Kind::binary:
        return pick(std::type_identity<Binary1>{}, std::type_identity<Binary2>{}, std::type_identity<Binary3>{});
    case Kind::counter:
        return pick(std::type_identity<Counter1>{}, std::type_identity<Counter2>{}, std::type_identity<Counter3>{});
    case Kind::clock:
        return pick(std::type_identity<Clock1>{}, std::type_identity<Clock2>{}, std::type_identity<Clock3>{});
    case Kind::pair:
        return pick(std::type_identity<Pair1>{}, std::type_identity<Pair2>{}, std::type_identity<Pair3>{});
    case Kind::rec:
        return pick(std::type_identity<Rec1>{}, std::type_identity<Rec2>{}, std::type_identity<Rec3>{});
    case Kind::acc:
        return f(std::type_identity<Acc1>{});
    }
}

template <class G>
Node make_node(G& g, fl::baseline_class kind, const NodeSpec& n, std::optional<Place> placed) {
    if (n.body) {
        if (n.kind == Kind::unary) {
            return fl::composite<toys::UnaryBinding, toys::UnaryChain>(g, n.name);
        }
        return fl::composite<toys::BinaryBinding, toys::BinaryPre>(g, n.name);
    }
    std::optional<Node> made;
    const bool instance = placed ? *placed == Place::instance : kind == fl::baseline_class::research;
    with_op(n.kind, n.salt, [&]<class Op>(std::type_identity<Op>) {
        if (instance) {
            made.emplace(fl::instance<Op>(g, n.name));
        } else {
            made.emplace(fl::composite<typename Op::binding, Op>(g, n.name));
        }
    });
    return *made;
}

/// Fire handler h of node n, at input `at`'s time if it takes one.
inline std::vector<Out> fire(fl::scope& s, const Node& n, std::size_t h, const Input& at, std::span<const Src> in) {
    const fl::time_of t = std::visit([](const auto& p) { return fl::at(p); }, at);
    switch (n.index()) {
    case 0:
        return with(in[0], [&](const auto& x) { return outs_of(fl::fire<"apply">(s, std::get<0>(n), x)); });
    case 1:
        return with(in[0], [&](const auto& x) {
            return with(in[1], [&](const auto& y) { return outs_of(fl::fire<"apply">(s, std::get<1>(n), x, y)); });
        });
    case 2:
        if (h == 0) {
            return with(in[0], [&](const auto& x) { return outs_of(fl::fire<"add">(s, std::get<2>(n), x)); });
        }
        if (h == 1) {
            return outs_of(fl::fire<"read">(s, std::get<2>(n), t));
        }
        return outs_of(fl::fire<"peek">(s, std::get<2>(n)));
    case 3:
        return outs_of(fl::fire<"at">(s, std::get<3>(n), t));
    case 4:
        return with(in[0], [&](const auto& x) { return outs_of(fl::fire<"apply">(s, std::get<4>(n), x)); });
    case 5:
        return with(in[0], [&](const auto& x) { return outs_of(fl::fire<"apply">(s, std::get<5>(n), x)); });
    default:
        if (h == 0) {
            return with(in[0], [&](const auto& x) { return outs_of(fl::fire<"add">(s, std::get<6>(n), x)); });
        }
        return with(in[0], [&](const auto& x) { return outs_of(fl::fire<"scale">(s, std::get<6>(n), x)); });
    }
}

}  // namespace detail

/// The recipe's stack, built by a stack_builder, or the invariant it breaks.
[[nodiscard]] inline branes::anatomy::result<Built, branes::anatomy::config::ConfigError> build(const Recipe& r) {
    static const toys::Context ctx;
    fl::stack_builder<toys::Context> g("toys", r.kind, ctx);

    std::vector<Input> inputs;
    for (const InputSpec& in : r.inputs) {
        if (in.tick) {
            inputs.emplace_back(g.tick(in.name));
        } else {
            inputs.emplace_back(fl::input<Num>(g, in.name));
        }
    }
    std::vector<fl::constant_port<Num>> constants;
    for (const ConstantSpec& c : r.constants) {
        constants.push_back(fl::constant<Num>(g, c.name, c.value));
    }
    std::vector<detail::Node> nodes;
    for (std::size_t i = 0; i < r.nodes.size(); ++i) {
        const auto placed = r.placed.find(i);
        nodes.push_back(detail::make_node(
            g, r.kind, r.nodes[i], placed == r.placed.end() ? std::nullopt : std::optional<Place>(placed->second)));
    }
    std::vector<std::optional<fl::scope>> scopes(r.inputs.size());
    for (std::size_t i = 0; i < r.inputs.size(); ++i) {
        if (r.inputs[i].region) {
            scopes[i].emplace(std::visit([&](const auto& p) { return g.region(p); }, inputs[i]));
        }
    }
    if (r.second_region) {
        (void)std::visit([&](const auto& p) { return g.region(p); }, inputs[*r.second_region]);
    }

    std::vector<std::vector<detail::Out>> written;
    std::vector<fl::output_port<Num>> outputs;
    std::optional<fl::value<Num>> first;  ///< what the first output is produced from
    for (std::size_t i = 0; i < r.firings.size(); ++i) {
        const FiringSpec& f = r.firings[i];
        const auto at = r.at.find(i);
        std::vector<detail::Src> in;
        for (const SourceSpec& s : f.in) {
            fl::value<Num> v;
            switch (s.from) {
            case SourceSpec::From::input:
                v = std::get<fl::input_port<Num>>(inputs[s.index]);
                break;
            case SourceSpec::From::constant:
                v = constants[s.index];
                break;
            case SourceSpec::From::firing: {
                const detail::Out& o = written[s.index][s.port];
                if (s.field < 0) {
                    v = std::get<fl::value<Num>>(o);
                } else if (s.field == 0) {
                    v = fl::project<"a", &Two::a>(std::get<fl::value<Two>>(o));
                } else {
                    v = fl::project<"b", &Two::b>(std::get<fl::value<Two>>(o));
                }
                break;
            }
            }
            in.push_back(detail::Src{v, s.hold});
        }
        written.push_back(detail::fire(
            *scopes[f.region], nodes[f.node], f.handler, inputs[at == r.at.end() ? f.region : at->second], in));

        // An output for every value the firing writes.
        const auto output = [&](const fl::value<Num>& v) {
            const std::string name = "o" + std::to_string(outputs.size());
            outputs.push_back(std::visit([&](const auto& p) { return fl::output<Num>(g, name, p); }, inputs[f.region]));
            g.produce(outputs.back(), v);
            if (!first) {
                first = v;
            }
        };
        for (const detail::Out& o : written.back()) {
            if (const auto* v = std::get_if<fl::value<Num>>(&o)) {
                output(*v);
            } else {
                const auto& rec = std::get<fl::value<Two>>(o);
                output(fl::project<"a", &Two::a>(rec));
                output(fl::project<"b", &Two::b>(rec));
            }
        }
    }

    if (r.unproduced) {
        (void)std::visit([&](const auto& p) { return fl::output<Num>(g, "ox", p); }, inputs[0]);
    }
    if (r.produced_twice && first) {
        g.produce(outputs.front(), *first);
    }

    auto st = std::move(g).build_stack();
    if (!st) {
        return st.error();
    }
    return Built{std::move(st).value(), std::move(inputs), std::move(outputs)};
}

}  // namespace gen
