#pragma once
// Changes to a recipe (generate.hpp) that must not change its records, for
// the flow graph's metamorphic tests (docs/plan/flow-testing, T3, P7). Each
// returns its twin recipe, and where each of the first recipe's firings went
// in it, so their outputs can be paired: an output of the first is
// (firing, port, field), and its twin's is (moved[firing], port, field).
//
//   as_research(): a scoreboard recipe's composites as instances of their
//                  own, a body's two operators as two firings in its place;
//   swapped():     two adjacent firings of a region, which share no value
//                  and no node, in the other order;
//   renamed():     every node, input, constant and output renamed.
//
// read_update_swapped() is the swap the other way: a Counter's update and a
// read of it, adjacent in a region, in the other order. The builder accepts
// both orders (G5), and the records may differ.

#include "generate.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace metamorphic {

namespace fl = branes::anatomy::flow;
using gen::FiringSpec;
using gen::Kind;
using gen::NodeSpec;
using gen::Recipe;
using gen::SourceSpec;
using From = SourceSpec::From;

struct Twin {
    Recipe recipe;
    std::vector<std::size_t> moved;  ///< the first recipe's firing i is the twin's moved[i]
};

/// The firings' sources, as `moved` renumbers the firings.
inline void renumber(std::vector<SourceSpec>& in, const std::vector<std::size_t>& moved) {
    for (SourceSpec& s : in) {
        if (s.from == From::firing) {
            s.index = moved[s.index];
        }
    }
}

/// The research graph of a scoreboard recipe: each composite of one operator
/// an instance of it, and each body (UnaryChain, BinaryPre) its operators as
/// instances, fired in its firing's place, with the same wiring. Its records
/// are the scoreboard's; its hash differs (its class, and its instances).
/// Pre: r is a scoreboard recipe, with no mutation.
[[nodiscard]] inline Twin as_research(const Recipe& r) {
    Twin t{r, {}};
    Recipe& out = t.recipe;
    out.kind = fl::baseline_class::research;
    out.nodes.clear();
    out.firings.clear();

    // Each node's instances: one, or a body's two.
    std::vector<std::pair<std::size_t, std::size_t>> of(r.nodes.size());
    for (std::size_t n = 0; n < r.nodes.size(); ++n) {
        const NodeSpec& node = r.nodes[n];
        if (!node.body) {
            out.nodes.push_back(node);
            of[n] = {out.nodes.size() - 1, out.nodes.size() - 1};
        } else if (node.kind == Kind::unary) {
            out.nodes.push_back(NodeSpec{node.name + "_first", Kind::unary, 1, false});
            out.nodes.push_back(NodeSpec{node.name + "_second", Kind::unary, 2, false});
            of[n] = {out.nodes.size() - 2, out.nodes.size() - 1};
        } else {
            out.nodes.push_back(NodeSpec{node.name + "_pre", Kind::unary, 3, false});
            out.nodes.push_back(NodeSpec{node.name + "_join", Kind::binary, 1, false});
            of[n] = {out.nodes.size() - 2, out.nodes.size() - 1};
        }
    }
    for (const FiringSpec& f : r.firings) {
        std::vector<SourceSpec> in = f.in;
        renumber(in, t.moved);
        const auto [first, last] = of[f.node];
        if (!r.nodes[f.node].body) {
            out.firings.push_back(FiringSpec{first, f.handler, f.region, in});
        } else {
            // The body's first operator takes the composite's first argument;
            // the second its result and, in BinaryPre, the second argument.
            out.firings.push_back(FiringSpec{first, 0, f.region, {in[0]}});
            const SourceSpec inner{From::firing, out.firings.size() - 1, 0, -1, false};
            if (r.nodes[f.node].kind == Kind::unary) {
                out.firings.push_back(FiringSpec{last, 0, f.region, {inner}});
            } else {
                out.firings.push_back(FiringSpec{last, 0, f.region, {inner, in[1]}});
            }
        }
        t.moved.push_back(out.firings.size() - 1);
    }
    return t;
}

/// Firings i and i + 1 of r, in the other order, sources renumbered.
[[nodiscard]] inline Twin swap_at(const Recipe& r, std::size_t i) {
    Twin t{r, {}};
    for (std::size_t k = 0; k < r.firings.size(); ++k) {
        t.moved.push_back(k == i ? i + 1 : k == i + 1 ? i : k);
    }
    std::swap(t.recipe.firings[i], t.recipe.firings[i + 1]);
    for (FiringSpec& f : t.recipe.firings) {
        renumber(f.in, t.moved);
    }
    return t;
}

/// Whether firing j reads any out port of firing i.
[[nodiscard]] inline bool reads(const Recipe& r, std::size_t j, std::size_t i) {
    for (const SourceSpec& s : r.firings[j].in) {
        if (s.from == From::firing && s.index == i) {
            return true;
        }
    }
    return false;
}

/// The first two adjacent firings of one region, of different nodes, the
/// second not reading the first, in the other order; or none. Adjacent in the
/// recipe's order, so every source is still described before it is read.
[[nodiscard]] inline std::optional<Twin> swapped(const Recipe& r) {
    for (std::size_t i = 0; i + 1 < r.firings.size(); ++i) {
        const FiringSpec& a = r.firings[i];
        const FiringSpec& b = r.firings[i + 1];
        if (a.region == b.region && a.node != b.node && !reads(r, i + 1, i)) {
            return swap_at(r, i);
        }
    }
    return std::nullopt;
}

/// The first Counter update and read of it (read or peek), adjacent in a
/// region, the second not reading the first, in the other order; or none.
[[nodiscard]] inline std::optional<Twin> read_update_swapped(const Recipe& r) {
    for (std::size_t i = 0; i + 1 < r.firings.size(); ++i) {
        const FiringSpec& a = r.firings[i];
        const FiringSpec& b = r.firings[i + 1];
        if (a.region == b.region && a.node == b.node && r.nodes[a.node].kind == Kind::counter &&
            ((a.handler == 0) != (b.handler == 0)) && !reads(r, i + 1, i)) {
            return swap_at(r, i);
        }
    }
    return std::nullopt;
}

/// r with every node, input, constant and output renamed.
[[nodiscard]] inline Twin renamed(const Recipe& r) {
    Twin t{r, {}};
    for (std::size_t k = 0; k < r.firings.size(); ++k) {
        t.moved.push_back(k);
    }
    for (NodeSpec& n : t.recipe.nodes) {
        n.name = "m_" + n.name;
    }
    for (gen::InputSpec& in : t.recipe.inputs) {
        in.name = "x_" + in.name;
    }
    for (gen::ConstantSpec& c : t.recipe.constants) {
        c.name = "k_" + c.name;
    }
    t.recipe.output_prefix = "q";
    return t;
}

}  // namespace metamorphic
