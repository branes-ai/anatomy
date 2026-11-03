#pragma once
/// @file graph.hpp
/// @brief The graph description: a mission's interface, the operator registry,
/// a baseline's graph, their loaders and checks, and the graph hash (anatomy
/// docs/adr/0004, step 2; docs/plan/flow-graph).
///
/// Three files describe a graph. The mission interface (interface.yaml) gives
/// the external ports, the composites and their handlers, and the regions
/// that fire them. The operator registry gives each operator's header, type,
/// and the composite it fills or its own handlers. A baseline's `class:` and
/// `graph:` give a body for each composite (scoreboard) or a whole graph of
/// operators (research). load_graph() checks the three together and returns
/// the graph flattened to operator firings, in the order the regions run them.
///
/// The files name C++ types as strings, which nothing here compiles. Domains
/// are C++ types, so G4 (an edge joins two ports of one domain) is checked by
/// the compiler, in the stack the generator emits. Every other invariant is
/// checked here, and a file that breaks one is refused with a ConfigError
/// naming the file, the key, and the invariant (G1–G9, docs/plan/flow-graph).
///
/// Loading never throws.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/result.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace branes::anatomy::flow {

// ── Declarations: what the files say ────────────────────────────────

/// What a handler does with its operator's state (ADR 0004, decision 4).
enum class state_role : std::uint8_t { none, read, update };

/// The rule on an edge between regions that run at different rates. Step 2
/// implements `hold` only: the reader takes the latest value written.
enum class rate_rule : std::uint8_t { none, hold };

/// A baseline's class (ADR 0004, decision 6).
enum class baseline_class : std::uint8_t { scoreboard, research };

/// One handler of a composite or an operator: a function of its binding.
/// `in` and `out` are port names of its signature, in the binding's order.
struct HandlerDecl {
    std::string name;
    std::vector<std::string> in;
    std::vector<std::string> out;
    bool time = false;  ///< takes the time of the input that fires it (`h@input`)
    state_role state = state_role::none;

    friend bool operator==(const HandlerDecl&, const HandlerDecl&) = default;
};

/// An external input: a message the stack is handed, and the trigger it fires.
struct InputDecl {
    std::string name;
    std::string message;  ///< empty: the input carries only its time (the tick)
    std::string trigger;
    std::string time;  ///< the message member holding its time; empty: `t`
    bool sitl_only = false;

    friend bool operator==(const InputDecl&, const InputDecl&) = default;
};

/// A constant: fixed when the stack is built.
struct ConstantDecl {
    std::string name;
    std::string message;
    std::string from;  ///< what the stack is built from; empty: a default value

    friend bool operator==(const ConstantDecl&, const ConstantDecl&) = default;
};

/// A source, as written: `ref`, or `{from: ref, rate: hold}`. A ref is
/// `<port>[.<field>...]` or `<node>.<port>[.<field>...]`, where a node is a
/// composite, an operator instance, or (in a body) nothing.
struct SourceDecl {
    std::string ref;
    rate_rule rate = rate_rule::none;

    friend bool operator==(const SourceDecl&, const SourceDecl&) = default;
};

/// An external output: what the stack returns from a region.
struct OutputDecl {
    std::string name;
    std::string message;
    std::string region;
    std::string from;  ///< the producer, in the interface's composites

    friend bool operator==(const OutputDecl&, const OutputDecl&) = default;
};

/// A firing, as written: `<node>.<handler>[@<input>]`, with one source per in port.
struct FiringDecl {
    std::string node;
    std::string handler;
    std::string at;
    std::vector<SourceDecl> sources;

    friend bool operator==(const FiringDecl&, const FiringDecl&) = default;
};

/// A region: the firings one trigger runs, in the declared order.
struct RegionDecl {
    std::string trigger;
    std::vector<FiringDecl> firings;

    friend bool operator==(const RegionDecl&, const RegionDecl&) = default;
};

struct CompositeDecl {
    std::string name;
    std::string signature;  ///< a C++ alias template, on the scalar type
    std::string binding;    ///< a C++ class template, on the scalar type
    std::vector<HandlerDecl> handlers;

    friend bool operator==(const CompositeDecl&, const CompositeDecl&) = default;
};

/// mission/interface.yaml.
///
/// Invariants: G1 (names, and composites and external ports are one
/// namespace); every composite handler fires in a region (G6); every region
/// is a declared trigger (G6), and its firings and every output's producer
/// resolve (G3, G5, G7).
struct Interface {
    std::string mission;
    std::string profile;
    std::vector<InputDecl> inputs;
    std::vector<ConstantDecl> constants;
    std::vector<OutputDecl> outputs;
    std::vector<CompositeDecl> composites;  ///< in the order the stack builds them
    std::vector<RegionDecl> regions;

    friend bool operator==(const Interface&, const Interface&) = default;
};

/// One registered operator. It fills `composite`, taking that composite's
/// signature, binding and handlers; or, if `composite` is empty, it has its own.
struct OperatorDecl {
    std::string name;
    std::string header;
    std::string type;
    std::string composite;
    std::string signature;
    std::string binding;
    std::vector<HandlerDecl> handlers;

    friend bool operator==(const OperatorDecl&, const OperatorDecl&) = default;
};

/// The operator registry: what flow_register_operator() records.
struct Registry {
    std::vector<OperatorDecl> operators;

    friend bool operator==(const Registry&, const Registry&) = default;
};

// ── The checked graph ───────────────────────────────────────────────

/// Where a firing's in port, or an output, takes its value from.
struct Ref {
    enum class kind : std::uint8_t { input, constant, firing };
    kind from = kind::input;
    std::string region;             ///< the producing firing's region (kind::firing)
    std::size_t firing = 0;         ///< its index in that region (kind::firing)
    std::string port;               ///< the input, the constant, or the firing's out port
    std::vector<std::string> path;  ///< the record fields projected, outermost first
    rate_rule rate = rate_rule::none;

    friend bool operator==(const Ref&, const Ref&) = default;
};

/// One call of an operator's handler.
struct Firing {
    std::string op;  ///< the operator instance
    std::string handler;
    std::string at;         ///< the input whose time a time handler takes; else empty
    std::vector<Ref> in;    ///< one per in port of the handler, in its order
    std::string composite;  ///< the composite handler it implements, `c.h`; empty in research
    state_role state = state_role::none;

    friend bool operator==(const Firing&, const Firing&) = default;
};

struct Region {
    std::string trigger;
    std::vector<Firing> firings;  ///< in the order they run

    friend bool operator==(const Region&, const Region&) = default;
};

/// An operator instance. A scoreboard body of one operator is named after its
/// composite; one of several, `<composite>.<instance>`.
struct Instance {
    std::string name;
    std::string op;         ///< the registered operator
    std::string composite;  ///< the composite whose body holds it; empty in research

    friend bool operator==(const Instance&, const Instance&) = default;
};

struct Output {
    std::string name;
    std::string region;
    Ref from;

    friend bool operator==(const Output&, const Output&) = default;
};

/// A baseline's graph, checked and flattened to operator firings.
///
/// Invariants: G1–G3 and G5–G9 hold; every Ref of kind firing names an
/// earlier firing of its own region, or one of another region with rate hold.
struct Graph {
    std::string mission;
    baseline_class kind = baseline_class::scoreboard;
    std::vector<Instance> instances;  ///< in the order the stack builds them
    std::vector<Region> regions;      ///< in the interface's order, then the baseline's
    std::vector<Output> outputs;      ///< in the interface's order
    bool sitl_only = false;           ///< G9: a firing reads a sitl_only input

    friend bool operator==(const Graph&, const Graph&) = default;
};

// ── Loaders ─────────────────────────────────────────────────────────

[[nodiscard]] result<Interface, config::ConfigError> parse_interface(std::string_view yaml, std::string_view origin);
[[nodiscard]] result<Interface, config::ConfigError> load_interface(const std::filesystem::path& path);

[[nodiscard]] result<Registry, config::ConfigError> parse_registry(std::string_view yaml, std::string_view origin);
[[nodiscard]] result<Registry, config::ConfigError> load_registry(const std::filesystem::path& path);

/// The graph of baseline `b`, loaded from `origin`, checked against the
/// interface and the registry. A baseline with no graph, or a graph with no
/// class, is refused (G8).
[[nodiscard]] result<Graph, config::ConfigError> load_graph(const config::BaselineConfig& b,
                                                            std::string_view origin,
                                                            const Interface& mission,
                                                            const Registry& registry);

/// The graph's canonical form: one line per instance, region, firing, edge
/// and output, in the graph's order. Two files that describe one graph,
/// whatever their layout, have one canonical form.
[[nodiscard]] std::string canonical(const Graph& g);

/// SHA-256 (hex) of canonical(g): the composition hash of a graph build.
[[nodiscard]] std::string graph_hash(const Graph& g);

}  // namespace branes::anatomy::flow
