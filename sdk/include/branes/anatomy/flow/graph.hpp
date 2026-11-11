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

/// The rule on an edge. Between regions that run at different rates: `hold`,
/// the reader takes the latest value written; `accumulate`, the reader takes
/// the values written since its region last began, up to a capacity
/// (flow/window.hpp; docs/plan/workbench, B2). `queue` is the model's third,
/// for when a pattern or a mission needs it. Within a region: `carried`, the
/// reader takes the value its producer wrote in the region's previous run,
/// which closes a loop (docs/plan/computational-domain, C4).
enum class rate_rule : std::uint8_t { none, hold, accumulate, carried };

/// A baseline's class (ADR 0004, decision 6).
enum class baseline_class : std::uint8_t { scoreboard, research };

/// Where a firing's in port, or an output, takes its value from.
struct Ref {
    enum class kind : std::uint8_t { input, constant, firing };
    kind from = kind::input;
    std::string region;                ///< the producing firing's region (kind::firing)
    std::size_t firing = 0;            ///< its index in that region (kind::firing)
    std::string port;                  ///< the input, the constant, or the firing's out port
    std::vector<std::string> path;     ///< the record fields projected, outermost first
    rate_rule rate = rate_rule::none;  ///< carried: the producer's value from the region's previous run
    std::size_t capacity = 0;          ///< the values an accumulate edge keeps; 0 for any other

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
/// names an earlier firing of its own region with rate none, any firing of
/// its own region with rate carried, or one of another region with rate hold
/// or accumulate; a Ref's capacity is at least 1 if its rate is accumulate,
/// and 0 otherwise. So every cycle of firings crosses a carried edge.
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
/// their canonical forms are. An edge's rate rule ends its line: ` hold`,
/// ` accumulate N` with N its capacity, or ` carried`; an edge with none adds
/// nothing, so a graph without an accumulate or a carried edge has the form
/// it had before there was one.
[[nodiscard]] std::string canonical(const Graph& g);

/// SHA-256 (hex) of canonical(g): the composition hash of a graph build.
[[nodiscard]] std::string graph_hash(const Graph& g);

/// The graph as a Mermaid flowchart, for reading: a subgraph per region with
/// its firings in order, each labelled with its index, a node per input, constant and output, and an edge
/// per in port, labelled with the port and the fields projected. An edge
/// between regions is dotted, and labelled `hold` or `accumulate N`; so is a
/// carried edge, labelled `carried`, which runs back to an earlier firing or
/// to its own. It is not hashed; canonical() is.
[[nodiscard]] std::string mermaid(const Graph& g);

/// G8, between a baseline file and the graph built for it: the file's class
/// is the graph's, and a scoreboard file names, for each of the graph's
/// composites and no other, the body that fills it.
[[nodiscard]] result<ok_t, config::ConfigError>
check_baseline(const config::BaselineConfig& b, std::string_view origin, const Graph& g);

}  // namespace branes::anatomy::flow
