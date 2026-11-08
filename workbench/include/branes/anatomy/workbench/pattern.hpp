#pragma once
/// @file pattern.hpp
/// @brief What a pattern provides, its oracle's report, and its run
/// (docs/plan/workbench, "The workbench").
///
/// A pattern P is a type with:
///   P::name                         its name (`mlp`)
///   P::default_inputs               how many inputs workbench_run hands it, unless told
///   P::config                       its configuration, a struct with defaults: rates, parameters, a seed
///   P::context                      what its operators are built from (from_context): holds the configuration
///   P::ports                        its graph's external ports
///   P::graph(G&, const config&)     its graph, a template over its builder, returning its ports
///   P::sources(ports, config)       its sources, in the order the driver breaks ties by
///   P::sample, P::collect(stack, ports, i, source)
///                                   what the oracle reads, collected after each input
///   P::oracle(config, samples)      its report (W2)
///   P::codecs()                     the trace codecs of its edges' messages
/// and its hand-wired run (W1), which its tests compare with the stack.

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/workbench/driver.hpp>
#include <branes/anatomy/workbench/sources.hpp>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::workbench {

/// One check of an oracle: its value, against its tolerance.
struct check {
    std::string name;
    double value = 0;
    double tolerance = 0;  ///< the most the value may be, or, for a lower bound, the least
    bool passed = false;
    bool upper = true;  ///< tolerance is an upper bound (at_most), not a lower one (at_least)
};

/// A check that passes if `value` is at most `tolerance` (and is a number).
[[nodiscard]] inline check at_most(std::string name, double value, double tolerance) {
    return check{std::move(name), value, tolerance, value <= tolerance};
}

/// A check that passes if `value` is at least `bound` (and is a number).
[[nodiscard]] inline check at_least(std::string name, double value, double bound) {
    return check{std::move(name), value, bound, value >= bound, false};
}

/// An oracle's report: every check, each with its value and tolerance.
struct report {
    std::string pattern;
    std::vector<check> checks;

    [[nodiscard]] bool passed() const noexcept {
        for (const check& c : checks) {
            if (!c.passed) {
                return false;
            }
        }
        return !checks.empty();
    }

    /// One line per check: PASS or FAIL, the value, and the tolerance.
    [[nodiscard]] std::string to_string() const {
        std::string s = pattern + ": " + std::to_string(checks.size()) + " checks, " +
                        (passed() ? "all passed" : "some failed") + "\n";
        for (const check& c : checks) {
            char line[256];
            std::snprintf(line,
                          sizeof line,
                          "  %s  %-48s %.6g (%s %.6g)\n",
                          c.passed ? "PASS" : "FAIL",
                          c.name.c_str(),
                          c.value,
                          c.upper ? "at most" : "at least",
                          c.tolerance);
            s += line;
        }
        return s;
    }
};

/// Scale every upper bound by `k`, so an at_most check passes if value <= k·tolerance; a
/// lower bound is unchanged. For workbench_run's --tolerance-scale, which shows how much
/// margin a pattern's tolerances have.
[[nodiscard]] inline report scaled(report r, double k) {
    for (check& c : r.checks) {
        if (c.upper) {
            c.tolerance *= k;
            c.passed = c.value <= c.tolerance;
        }
    }
    return r;
}

/// What a run writes besides its report, if asked.
struct run_options {
    std::size_t inputs = 100;
    std::optional<std::filesystem::path> trace;
};

/// Pattern P's stack, built from its context, and its ports.
template <class P>
struct built {
    flow::stack stack;
    typename P::ports ports;
};

/// P's graph, run by the CPU stack.
template <class P>
[[nodiscard]] result<built<P>, config::ConfigError> build(const typename P::context& ctx) {
    flow::stack_builder<typename P::context> g(std::string(P::name), flow::baseline_class::research, ctx);
    typename P::ports ports = P::graph(g, ctx.cfg);
    auto st = std::move(g).build_stack();
    if (!st) {
        return st.error();
    }
    return built<P>{std::move(st).value(), std::move(ports)};
}

/// P's graph, checked only: for its canonical form and its flowchart.
template <class P>
[[nodiscard]] result<flow::Graph, config::ConfigError> describe(const typename P::config& cfg) {
    flow::builder g(std::string(P::name), flow::baseline_class::research);
    (void)P::graph(g, cfg);
    return g.build();
}

/// Run P with `cfg` for `opt.inputs` inputs, writing a trace if asked, and
/// return its samples. An error is a graph that doesn't build, or a trace
/// that can't be written.
template <class P>
[[nodiscard]] result<std::vector<typename P::sample>, std::string> run(const typename P::config& cfg,
                                                                       const run_options& opt) {
    const typename P::context ctx{cfg};
    auto b = build<P>(ctx);
    if (!b) {
        return b.error().to_string();
    }
    std::optional<flow::trace_writer> writer;
    if (opt.trace) {
        auto w = flow::trace_writer::open(*opt.trace, b->stack.graph(), P::codecs());
        if (!w) {
            return w.error().path + ": " + w.error().message;
        }
        writer.emplace(std::move(w).value());
        b->stack.observe([&writer](const flow::fired& f) { writer->write(f); });
    }
    const std::vector<source> sources = P::sources(b->ports, cfg);
    std::vector<typename P::sample> samples;
    samples.reserve(opt.inputs);
    drive(b->stack, sources, opt.inputs, [&](std::size_t i, std::size_t s, Timestamp t) {
        samples.push_back(P::collect(b->stack, b->ports, i, s, t));
    });
    b->stack.observe({});
    if (writer) {
        if (auto closed = writer->close(); !closed) {
            return closed.error().path + ": " + closed.error().message;
        }
    }
    return samples;
}

/// The relative difference of two vectors, in the max norm: max |a - b| / max |b|,
/// or max |a - b| if b is zero.
template <class A, class B>
[[nodiscard]] long double relative_max(const A& a, const B& b, std::size_t n) {
    long double diff = 0;
    long double norm = 0;
    for (std::size_t i = 0; i < n; ++i) {
        diff = std::fmax(diff, std::fabs(static_cast<long double>(a[i]) - static_cast<long double>(b[i])));
        norm = std::fmax(norm, std::fabs(static_cast<long double>(b[i])));
    }
    return norm > 0 ? diff / norm : diff;
}

}  // namespace branes::anatomy::workbench
