#pragma once
/// @file graph.hpp
/// @brief The checked graph: a baseline's operators, flattened to their
/// firings in the order the regions run them, with its canonical form and
/// its hash (anatomy docs/adr/0004, step 2; docs/plan/flow-graph).
///
/// A graph is described in C++ and checked as it is built (flow/builder.hpp).
/// What the builder returns is a Graph: a value with no types in it, which
/// the CPU stack runs (2b) and the domain flow compiler reads (step 4).

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/result.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace branes::anatomy::flow {

/// What a handler does with its operator's state (ADR 0004, decision 4).
enum class state_role : std::uint8_t { none, read, update };

/// The rule on an edge between regions that run at different rates. Step 2
/// implements `hold` only: the reader takes the latest value written.
enum class rate_rule : std::uint8_t { none, hold };

/// A baseline's class (ADR 0004, decision 6).
enum class baseline_class : std::uint8_t { scoreboard, research };

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

/// A composite of a scoreboard graph, and the body that fills it: a
/// registered operator, or a body of several.
struct Composite {
    std::string name;
    std::string body;

    friend bool operator==(const Composite&, const Composite&) = default;
};

/// An operator instance. A composite's body of one operator is named after
/// its composite; one of several, `<composite>.<instance>`.
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
/// Invariants: G1–G9 hold (docs/plan/flow-graph); every Ref of kind firing
/// names an earlier firing of its own region, or one of another region with
/// rate hold.
struct Graph {
    std::string mission;
    baseline_class kind = baseline_class::scoreboard;
    std::vector<Composite> composites;  ///< in the order the mission declares them; empty in research
    std::vector<Instance> instances;    ///< in the order the stack builds them
    std::vector<Region> regions;        ///< in the order the graph declares them
    std::vector<Output> outputs;        ///< in the order the mission declares them
    bool sitl_only = false;             ///< G9: a firing reads a sitl_only input

    friend bool operator==(const Graph&, const Graph&) = default;
};

/// The graph's canonical form: one line per composite, instance, region,
/// firing, edge and output, in the graph's order. Two graphs are equal when
/// their canonical forms are.
[[nodiscard]] std::string canonical(const Graph& g);

/// SHA-256 (hex) of canonical(g): the composition hash of a graph build.
[[nodiscard]] std::string graph_hash(const Graph& g);

/// The graph as a Mermaid flowchart, for reading: a subgraph per region with
/// its firings in order, each labelled with its index, a node per input, constant and output, and an edge
/// per in port, labelled with the port and the fields projected. An edge
/// that holds across regions is dotted. It is not hashed; canonical() is.
[[nodiscard]] std::string mermaid(const Graph& g);

/// G8, between a baseline file and the graph built for it: the file's class
/// is the graph's, and a scoreboard file names, for each of the graph's
/// composites and no other, the body that fills it.
[[nodiscard]] result<ok_t, config::ConfigError>
check_baseline(const config::BaselineConfig& b, std::string_view origin, const Graph& g);

}  // namespace branes::anatomy::flow
