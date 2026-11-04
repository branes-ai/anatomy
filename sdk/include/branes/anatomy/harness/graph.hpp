#pragma once
/// @file graph.hpp
/// @brief `--print-graph <form>`: a graph build's harness prints the graph it
/// built, and exits.
///
/// Every mission's graph builds take the option, with the same forms, so a
/// graph can be read from any of them (anatomy docs/guides/reading-a-graph.md).

#include <branes/anatomy/flow/graph.hpp>

#include <optional>
#include <ostream>
#include <string>
#include <string_view>

namespace branes::anatomy::harness {

/// The option's name, as Args takes it.
inline constexpr std::string_view print_graph_option = "print-graph";

/// Write the graph to `out` in the form `--print-graph` names: `text`, its
/// canonical form, which its hash is of; or `mermaid`, a flowchart
/// (flow::mermaid). Error text if `form` is neither, and nothing written.
[[nodiscard]] inline std::optional<std::string>
print_graph(const flow::Graph& g, std::string_view form, std::ostream& out) {
    if (form == "text") {
        out << flow::canonical(g);
    } else if (form == "mermaid") {
        out << flow::mermaid(g);
    } else {
        return "--print-graph takes text or mermaid, not '" + std::string(form) + "'";
    }
    return std::nullopt;
}

}  // namespace branes::anatomy::harness
