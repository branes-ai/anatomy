#pragma once
// Mutations of valid recipes (generate.hpp), for the builder's checks
// (docs/plan/flow-testing, T2). Each mutation changes one thing in a valid
// recipe so that it breaks one invariant, and says which invariant, and the
// key of the error that must name it: the node, the firing (its region's
// trigger and its index there, `trigger[i]`), the firing's in port
// (`trigger[i][k]`), or the output (`out.<name>`).
//
// A mutation that needs something the recipe lacks adds it first, validly:
// a node and its firings, an input with no region, or a constant. So every
// recipe gets every mutation, but G1's two nodes of one name, which needs
// two nodes.
//
// G8's other half, the baseline file's, is check_baseline()'s, against the
// valid graph: file_mutations() changes the file, not the recipe.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/flow/graph.hpp>

#include "generate.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mutate {

namespace fl = branes::anatomy::flow;
using gen::FiringSpec;
using gen::Kind;
using gen::NodeSpec;
using gen::Recipe;
using gen::SourceSpec;
using From = SourceSpec::From;

struct Mutation {
    std::string what;
    Recipe recipe;
    std::string invariant;  ///< "G1" to "G8": what the error's message starts with
    std::string key;        ///< the error's key
};

/// The firings a firing of node `n` is flattened to: a body's two, else one.
[[nodiscard]] inline std::size_t width(const Recipe& r, std::size_t n) {
    return r.nodes[n].body ? 2 : 1;
}

/// The key of the builder's error at firing i: `trigger[index in its region]`.
[[nodiscard]] inline std::string firing_key(const Recipe& r, std::size_t i) {
    const std::size_t region = r.firings[i].region;
    std::size_t at = 0;
    for (std::size_t j = 0; j < i; ++j) {
        if (r.firings[j].region == region) {
            at += width(r, r.firings[j].node);
        }
    }
    return r.inputs[region].name + "[" + std::to_string(at) + "]";
}

/// Add a node of `kind`, and return its index.
inline std::size_t add_node(Recipe& r, std::string name, Kind kind) {
    r.nodes.push_back(NodeSpec{std::move(name), kind, 1, false});
    return r.nodes.size() - 1;
}

/// Add a firing of node n's handler h in input 0's region, and return its index.
inline std::size_t add_firing(Recipe& r, std::size_t n, std::size_t h, std::vector<SourceSpec> in) {
    r.firings.push_back(FiringSpec{n, h, 0, std::move(in)});
    return r.firings.size() - 1;
}

/// An input other than input 0, adding one with no region if there is none.
inline std::size_t other_input(Recipe& r) {
    if (r.inputs.size() == 1) {
        r.inputs.push_back(gen::InputSpec{"inx", false, false});
    }
    return 1;
}

/// A value input other than input 0, adding one with no region if there is none.
inline std::size_t other_value_input(Recipe& r) {
    for (std::size_t k = 1; k < r.inputs.size(); ++k) {
        if (!r.inputs[k].tick) {
            return k;
        }
    }
    r.inputs.push_back(gen::InputSpec{"inx", false, false});
    return r.inputs.size() - 1;
}

/// The first in port whose source `pick` accepts, as (firing, port).
template <class F>
[[nodiscard]] std::optional<std::pair<std::size_t, std::size_t>> find_source(const Recipe& r, F pick) {
    for (std::size_t i = 0; i < r.firings.size(); ++i) {
        for (std::size_t k = 0; k < r.firings[i].in.size(); ++k) {
            if (pick(r.firings[i].in[k])) {
                return std::pair{i, k};
            }
        }
    }
    return std::nullopt;
}

[[nodiscard]] inline std::string port_key(const Recipe& r, std::pair<std::size_t, std::size_t> at) {
    return firing_key(r, at.first) + "[" + std::to_string(at.second) + "]";
}

/// Every mutation of a valid recipe. Input 0 is a value with a region (generate()).
[[nodiscard]] inline std::vector<Mutation> mutations(const Recipe& valid) {
    std::vector<Mutation> out;
    const auto add = [&](std::string what, Recipe r, std::string invariant, std::string key) {
        out.push_back(Mutation{std::move(what), std::move(r), std::move(invariant), std::move(key)});
    };

    // G1: names.
    {
        Recipe r = valid;
        r.nodes[0].name = "";
        add("G1 an empty name", r, "G1", "");
    }
    {
        Recipe r = valid;
        r.nodes[0].name = "N0";
        add("G1 a name that isn't [a-z_][a-z0-9_]*", r, "G1", "N0");
    }
    {
        Recipe r = valid;
        r.nodes[0].name = r.inputs[0].name;
        add("G1 a node and an external port of one name", r, "G1", r.inputs[0].name);
    }
    if (valid.nodes.size() >= 2) {
        Recipe r = valid;
        r.nodes[1].name = r.nodes[0].name;
        add("G1 two nodes of one name", r, "G1", r.nodes[0].name);
    }

    // G3: an external output's producer.
    {
        Recipe r = valid;
        r.unproduced = true;
        add("G3 an output with no producer", r, "G3", "out.ox");
    }
    {
        Recipe r = valid;
        r.produced_twice = true;
        add("G3 an output with two producers", r, "G3", "out.o0");
    }

    // G5: a second update of an instance's state in one region.
    {
        Recipe r = valid;
        const std::size_t n = add_node(r, "nacc", Kind::acc);
        const SourceSpec in0{From::input, 0, 0, -1, false};
        add_firing(r, n, 0, {in0});
        const std::size_t second = add_firing(r, n, 1, {in0});
        add("G5 a second update in one region", r, "G5", firing_key(r, second));
    }

    // G6: regions, triggers and firings.
    {
        Recipe r = valid;
        r.second_region = 0;
        add("G6 a second region for a trigger", r, "G6", r.inputs[0].name);
    }
    {
        Recipe r = valid;
        std::optional<std::size_t> timed;
        for (std::size_t i = 0; i < r.firings.size() && !timed; ++i) {
            if (gen::handlers_of(r.nodes[r.firings[i].node].kind)[r.firings[i].handler].timed) {
                timed = i;
            }
        }
        if (!timed) {
            timed = add_firing(r, add_node(r, "nt", Kind::clock), 0, {});
        }
        r.at[*timed] = r.firings[*timed].region == 0 ? other_input(r) : 0;
        add("G6 a time taken from an input of another region", r, "G6", firing_key(r, *timed));
    }
    {
        Recipe r = valid;
        add_node(r, "nx", Kind::unary);
        add("G6 a handler that never fires", r, "G6", "nx");
    }
    {
        Recipe r = valid;
        r.firings.push_back(r.firings[0]);
        add("G6 a handler fired twice at one input", r, "G6", firing_key(r, r.firings.size() - 1));
    }

    // G7: rate rules.
    {
        Recipe r = valid;
        auto at = find_source(r, [](const SourceSpec& s) { return s.hold; });
        if (!at) {
            const std::size_t other = other_value_input(r);
            at = std::pair{add_firing(r, add_node(r, "nh", Kind::unary), 0, {{From::input, other, 0, -1, true}}),
                           std::size_t{0}};
        }
        r.firings[at->first].in[at->second].hold = false;
        add("G7 an edge between regions with no rate rule", r, "G7", port_key(r, *at));
    }
    {
        Recipe r = valid;
        auto at = find_source(r, [](const SourceSpec& s) {
            return !s.hold && s.accumulate == 0 && !s.carried && s.from != From::constant;
        });
        if (!at) {
            at = std::pair{add_firing(r, add_node(r, "nw", Kind::unary), 0, {{From::input, 0, 0, -1, false}}),
                           std::size_t{0}};
        }
        r.firings[at->first].in[at->second].hold = true;
        add("G7 a rate rule on an edge within a region", r, "G7", port_key(r, *at));
    }
    {
        Recipe r = valid;
        auto at = find_source(r, [](const SourceSpec& s) { return s.from == From::constant; });
        if (!at) {
            r.constants.push_back(gen::ConstantSpec{"cx", gen::Num{}});
            at = std::pair{
                add_firing(
                    r, add_node(r, "nk", Kind::unary), 0, {{From::constant, r.constants.size() - 1, 0, -1, false}}),
                std::size_t{0}};
        }
        r.firings[at->first].in[at->second].hold = true;
        add("G7 a rate rule on an edge from a constant", r, "G7", port_key(r, *at));
    }
    {
        // A Fold in input 0's region, accumulating input 0 (W7).
        Recipe r = valid;
        SourceSpec in0{From::input, 0, 0, -1, false};
        in0.accumulate = toys::kFoldCapacity[1];
        const std::size_t i = add_firing(r, add_node(r, "nfw", Kind::fold), 0, {in0});
        add("G7 an accumulate edge within a region", r, "G7", port_key(r, {i, 0}));
    }
    {
        // A Fold accumulating a constant (W7).
        Recipe r = valid;
        r.constants.push_back(gen::ConstantSpec{"cfx", gen::Num{}});
        SourceSpec k{From::constant, r.constants.size() - 1, 0, -1, false};
        k.accumulate = toys::kFoldCapacity[1];
        const std::size_t i = add_firing(r, add_node(r, "nfk", Kind::fold), 0, {k});
        add("G7 an accumulate edge from a constant", r, "G7", port_key(r, {i, 0}));
    }
    {
        // A carried edge from input 0, in input 0's region (C4).
        Recipe r = valid;
        SourceSpec in0{From::input, 0, 0, -1, false};
        in0.carried = true;
        const std::size_t i = add_firing(r, add_node(r, "nci", Kind::unary), 0, {in0});
        add("G7 a carried edge from an input", r, "G7", port_key(r, {i, 0}));
    }
    {
        // A carried edge in input 0's region, from a firing of a region of its own (C4).
        Recipe r = valid;
        r.inputs.push_back(gen::InputSpec{"inr", false, true});
        const std::size_t other = r.inputs.size() - 1;
        r.firings.push_back(
            FiringSpec{add_node(r, "ncp", Kind::unary), 0, other, {{From::input, other, 0, -1, false}}});
        SourceSpec from{From::firing, r.firings.size() - 1, 0, -1, false};
        from.carried = true;
        const std::size_t i = add_firing(r, add_node(r, "ncr", Kind::unary), 0, {from});
        add("G7 a carried edge from another region", r, "G7", port_key(r, {i, 0}));
    }

    // G8: what a graph's class holds.
    {
        Recipe r = valid;
        r.kind = fl::baseline_class::scoreboard;
        r.nodes[0].body = false;
        r.placed[0] = gen::Place::instance;
        add("G8 an instance outside a body in a scoreboard graph", r, "G8", r.nodes[0].name);
    }
    {
        Recipe r = valid;
        r.kind = fl::baseline_class::research;
        for (NodeSpec& n : r.nodes) {
            n.body = false;
        }
        r.placed[0] = gen::Place::composite;
        add("G8 a composite in a research graph", r, "G8", r.nodes[0].name);
    }
    return out;
}

// ── The baseline file ───────────────────────────────────────────────

struct FileMutation {
    std::string what;
    branes::anatomy::config::BaselineConfig file;
    std::string key;
};

/// The baseline file of a checked graph: its class, and each composite's body.
[[nodiscard]] inline branes::anatomy::config::BaselineConfig baseline_of(const fl::Graph& g) {
    branes::anatomy::config::BaselineConfig b;
    b.name = "toys";
    b.baseline_class = g.kind == fl::baseline_class::scoreboard ? "scoreboard" : "research";
    for (const fl::Composite& c : g.composites) {
        b.bodies[c.name] = c.body;
    }
    return b;
}

/// Every mutation of a checked graph's baseline file, each a G8 that
/// check_baseline() refuses.
[[nodiscard]] inline std::vector<FileMutation> file_mutations(const fl::Graph& g) {
    const auto valid = baseline_of(g);
    std::vector<FileMutation> out;
    {
        auto b = valid;
        b.baseline_class = g.kind == fl::baseline_class::scoreboard ? "research" : "scoreboard";
        out.push_back(FileMutation{"the wrong class", b, "class"});
    }
    if (g.kind == fl::baseline_class::scoreboard) {
        const std::string& c = g.composites.front().name;
        {
            auto b = valid;
            b.bodies["nx"] = "Unary1";
            out.push_back(FileMutation{"a body for a composite the graph doesn't have", b, "graph.nx"});
        }
        {
            auto b = valid;
            b.bodies.erase(c);
            out.push_back(FileMutation{"no body for a composite the graph has", b, "graph"});
        }
        {
            auto b = valid;
            b.bodies[c] = b.bodies[c] == "Rec1" ? "Rec2" : "Rec1";
            out.push_back(FileMutation{"another body than the graph's", b, "graph." + c});
        }
    }
    return out;
}

}  // namespace mutate
