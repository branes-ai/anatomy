#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/util/sha256.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::flow {

namespace {

/// [a-z_][a-z0-9_]*
bool is_name(std::string_view s) {
    constexpr std::string_view lower = "abcdefghijklmnopqrstuvwxyz_";
    constexpr std::string_view lower_digits = "abcdefghijklmnopqrstuvwxyz0123456789_";
    return !s.empty() && lower.find(s.front()) != std::string_view::npos &&
           s.find_first_not_of(lower_digits) == std::string_view::npos;
}

std::string in_quotes(std::string_view s) {
    return "'" + std::string(s) + "'";
}

bool contains(const std::vector<std::string>& v, std::string_view s) {
    return std::find(v.begin(), v.end(), s) != v.end();
}

std::string text(const Ref& ref) {
    std::string s;
    switch (ref.from) {
    case Ref::kind::input:
        s = "input " + ref.port;
        break;
    case Ref::kind::constant:
        s = "constant " + ref.port;
        break;
    case Ref::kind::firing:
        s = "firing " + ref.region + "#" + std::to_string(ref.firing) + " " + ref.port;
        break;
    }
    for (const auto& f : ref.path) {
        s += "." + f;
    }
    return s + (ref.rate == rate_rule::hold ? " hold" : "");
}

/// A source as a mission writes it: `x`, or `x.f` for project<"f">(x).
std::string written(const Ref& ref) {
    std::string s = ref.port;
    for (const auto& f : ref.path) {
        s += "." + f;
    }
    return s;
}

std::string text(state_role s) {
    return s == state_role::read ? "read" : s == state_role::update ? "update" : "none";
}

}  // namespace

// ── The builder ─────────────────────────────────────────────────────

builder::builder(std::string mission, baseline_class kind) {
    graph_.mission = std::move(mission);
    graph_.kind = kind;
}

void builder::fail(std::string key, std::string message) {
    if (!error_) {
        error_ = config::ConfigError{graph_.mission, std::move(key), std::move(message)};
    }
}

void builder::check_name(const std::string& key, const std::string& name, std::string_view what) {
    if (!is_name(name)) {
        fail(key, "G1: " + std::string(what) + " names match [a-z_][a-z0-9_]*");
    } else if (contains(names_, name)) {
        fail(key, "G1: nodes and external ports share one namespace, and " + in_quotes(name) + " is taken");
    } else {
        names_.push_back(name);
    }
}

const builder::input_rec* builder::find_input(std::string_view name) const {
    const auto it = std::find_if(inputs_.begin(), inputs_.end(), [&](const input_rec& in) { return in.name == name; });
    return it == inputs_.end() ? nullptr : &*it;
}

std::vector<Ref> builder::no_refs(std::size_t n) {
    return std::vector<Ref>(n);
}

input_port<Timestamp> builder::tick(std::string name) {
    return input_port<Timestamp>{{add_input(std::move(name), false, {})}};
}

Ref builder::add_input(std::string name, bool message, input_options options) {
    check_name(name, name, "an input's");
    std::string trigger = options.trigger.empty() ? name : std::move(options.trigger);
    if (!is_name(trigger)) {
        fail(name, "G1: a trigger's names match [a-z_][a-z0-9_]*");
    }
    inputs_.push_back(input_rec{name, message, std::move(trigger), std::move(options.time), options.sitl_only});
    Ref r;
    r.from = Ref::kind::input;
    r.port = std::move(name);
    return r;
}

Ref builder::add_constant(std::string name, constant_options options) {
    check_name(name, name, "a constant's");
    constants_.push_back(constant_rec{name, std::move(options.from)});
    Ref r;
    r.from = Ref::kind::constant;
    r.port = std::move(name);
    return r;
}

std::size_t builder::add_output(std::string name, const Ref& trigger) {
    const std::string key = "out." + name;
    if (!is_name(name)) {
        fail(key, "G1: an output's names match [a-z_][a-z0-9_]*");
    } else if (std::any_of(outputs_.begin(), outputs_.end(), [&](const output_rec& o) { return o.name == name; })) {
        fail(key, "G1: an output's names are distinct");
    }
    const input_rec* in = find_input(trigger.port);
    outputs_.push_back(output_rec{std::move(name), in != nullptr ? in->trigger : trigger.port, false, {}});
    return outputs_.size() - 1;
}

std::size_t builder::add_composite(std::string name,
                                   std::string_view body,
                                   std::vector<detail::handler_info> handlers,
                                   bool of_one_operator) {
    check_name(name, name, "a composite's");
    if (graph_.kind != baseline_class::scoreboard) {
        fail(name, "G8: a research graph has no composites: its operators are instances of its own");
    }
    graph_.composites.push_back(Composite{name, std::string(body)});
    node_rec n;
    n.name = name;
    n.fired.assign(handlers.size(), false);
    n.body.resize(handlers.size());
    n.handlers = std::move(handlers);
    n.composite = true;
    n.must_fire = true;
    n.of_one_operator = of_one_operator;
    n.body_name = std::string(body);
    if (of_one_operator) {
        n.instance = graph_.instances.size();
        graph_.instances.push_back(Instance{name, std::string(body), name});
    }
    nodes_.push_back(std::move(n));
    return nodes_.size() - 1;
}

std::size_t builder::add_instance(std::string name, std::string_view op, std::vector<detail::handler_info> handlers) {
    check_name(name, name, "an instance's");
    if (graph_.kind != baseline_class::research) {
        fail(name, "G8: a scoreboard graph's operators are in its composites' bodies");
    }
    node_rec n;
    n.name = name;
    n.fired.assign(handlers.size(), false);
    n.handlers = std::move(handlers);
    n.must_fire = true;
    n.instance = graph_.instances.size();
    graph_.instances.push_back(Instance{std::move(name), std::string(op), ""});
    nodes_.push_back(std::move(n));
    return nodes_.size() - 1;
}

std::size_t builder::add_body_instance(std::size_t composite,
                                       std::string local,
                                       std::string_view op,
                                       std::vector<detail::handler_info> handlers) {
    const std::string name = nodes_[composite].name + "." + local;
    if (!is_name(local)) {
        fail(name, "G1: an instance's names match [a-z_][a-z0-9_]*");
    } else if (std::any_of(nodes_.begin(), nodes_.end(), [&](const node_rec& n) { return n.name == name; })) {
        fail(name, "G1: a body's instance names are distinct");
    }
    node_rec n;
    n.name = name;
    n.fired.assign(handlers.size(), false);
    n.handlers = std::move(handlers);
    n.instance = graph_.instances.size();
    graph_.instances.push_back(Instance{name, std::string(op), nodes_[composite].name});
    nodes_.push_back(std::move(n));
    return nodes_.size() - 1;
}

void builder::add_body_handler(std::size_t composite, std::string_view handler, body_handler fn) {
    node_rec& c = nodes_[composite];
    const auto it = std::find_if(
        c.handlers.begin(), c.handlers.end(), [&](const detail::handler_info& h) { return h.name == handler; });
    const auto i = static_cast<std::size_t>(it - c.handlers.begin());
    if (c.body[i]) {
        fail(c.name + "." + std::string(handler), "G2: a body implements each handler of its composite once");
        return;
    }
    c.body[i] = std::move(fn);
}

void builder::end_body(std::size_t composite) {
    const node_rec& c = nodes_[composite];
    for (std::size_t i = 0; i < c.handlers.size(); ++i) {
        if (!c.body[i]) {
            fail(c.name,
                 "G2: a body implements every handler of its composite: " + c.body_name + " has no " +
                     in_quotes(c.handlers[i].name));
        }
    }
}

scope builder::region_of(const std::string& input) {
    const input_rec* in = find_input(input);
    const std::string trigger = in != nullptr ? in->trigger : input;
    for (std::size_t r = 0; r < graph_.regions.size(); ++r) {
        if (graph_.regions[r].trigger == trigger) {
            fail(trigger, "G6: a trigger has one region");
            return scope(*this, r);
        }
    }
    graph_.regions.push_back(Region{trigger, {}});
    updated_.emplace_back();
    return scope(*this, graph_.regions.size() - 1);
}

void builder::check_source(const Ref& ref, const std::string& trigger, const std::string& key) {
    const bool hold = ref.rate == rate_rule::hold;
    switch (ref.from) {
    case Ref::kind::constant:
        if (hold) {
            fail(key, "G7: a constant takes no rate rule");
        }
        return;
    case Ref::kind::input: {
        const input_rec* in = find_input(ref.port);
        if (in != nullptr && !in->message && !ref.path.empty()) {
            fail(key, "G3: " + in_quotes(in->name) + " carries only its time");
        }
        const std::string from = in != nullptr ? in->trigger : std::string();
        if (from == trigger && hold) {
            fail(key, "G7: an edge within a region has no rate rule");
        } else if (from != trigger && !hold) {
            fail(key,
                 "G7: an edge between regions has a rate rule: " + in_quotes(ref.port) + " fires " + in_quotes(from) +
                     ", and needs hold(" + written(ref) + ")");
        }
        return;
    }
    case Ref::kind::firing:
        if (ref.region == trigger && hold) {
            fail(key, "G7: an edge within a region has no rate rule");
        } else if (ref.region != trigger && !hold) {
            fail(key,
                 "G7: an edge between regions has a rate rule: " + written(ref) + " is written in region " +
                     in_quotes(ref.region) + ", and needs hold(" + written(ref) + ")");
        }
        return;
    }
}

std::vector<Ref>
builder::fire(scope& s, std::size_t node, std::string_view handler, const time_of* at, std::vector<Ref> in) {
    node_rec& n = nodes_[node];
    const auto it = std::find_if(
        n.handlers.begin(), n.handlers.end(), [&](const detail::handler_info& h) { return h.name == handler; });
    const auto hi = static_cast<std::size_t>(it - n.handlers.begin());
    const detail::handler_info h = *it;
    const std::string trigger = graph_.regions[s.region_].trigger;
    const std::string key = trigger + "[" + std::to_string(graph_.regions[s.region_].firings.size()) + "]";
    const std::string at_input = at != nullptr ? at->input : std::string();

    if (at != nullptr) {
        const input_rec* t = find_input(at_input);
        if (t == nullptr || t->trigger != trigger) {
            fail(key, "G6: a firing takes its time from an input of its region's trigger");
        }
    }
    for (std::size_t k = 0; k < in.size(); ++k) {
        check_source(in[k], trigger, key + "[" + std::to_string(k) + "]");
    }
    // A body's firing is one per expansion of its composite's firing, which
    // is itself once per region and input.
    const std::string fired = n.name + "." + h.name + "@" + at_input + (s.outer_.empty() ? "" : " in " + s.outer_);
    if (contains(fired_, fired)) {
        fail(key, "G6: every firing belongs to exactly one region");
        return no_refs(h.out.size());
    }
    fired_.push_back(fired);
    n.fired[hi] = true;

    // A composite whose body has several operators fires them, in place of itself.
    if (n.composite && !n.of_one_operator) {
        if (!n.body[hi]) {
            return no_refs(h.out.size());  // end_body() reported it
        }
        scope inner = s;
        inner.composite_ = n.name + "." + h.name;
        inner.state_ = h.state;
        inner.outer_ = fired;
        return n.body[hi](inner, at, in);
    }

    const std::string op = graph_.instances[n.instance].name;
    if (!s.composite_.empty() && h.state == state_role::update && s.state_ != state_role::update) {
        fail(key, "G5: " + op + "." + h.name + " updates its state, and " + s.composite_ + " declares no update");
    }
    if (h.state == state_role::update) {
        auto& updated = updated_[s.region_];
        if (std::find(updated.begin(), updated.end(), n.instance) != updated.end()) {
            fail(key, "G5: a region updates a node's state once, so each read is before or after it");
        }
        updated.push_back(n.instance);
    }
    std::string composite = s.composite_;
    if (composite.empty() && n.composite) {
        composite = n.name + "." + h.name;
    }
    Region& region = graph_.regions[s.region_];
    region.firings.push_back(Firing{op, h.name, at_input, std::move(in), std::move(composite), h.state});

    std::vector<Ref> outs;
    for (const std::string& port : h.out) {
        Ref r;
        r.from = Ref::kind::firing;
        r.region = trigger;
        r.firing = region.firings.size() - 1;
        r.port = port;
        outs.push_back(std::move(r));
    }
    return outs;
}

void builder::produce_ref(std::size_t out, const Ref& from) {
    output_rec& o = outputs_[out];
    const std::string key = "out." + o.name;
    if (o.produced) {
        fail(key, "G3: every external out port has one producer: " + in_quotes(o.name) + " has two");
        return;
    }
    if (from.from != Ref::kind::firing || from.region != o.region || from.rate != rate_rule::none) {
        fail(key, "G6: an output is written by a firing of its region, " + in_quotes(o.region));
        return;
    }
    o.produced = true;
    o.from = from;
}

result<Graph, config::ConfigError> builder::build() const {
    if (error_) {
        return *error_;
    }
    for (const node_rec& n : nodes_) {
        for (std::size_t i = 0; n.must_fire && i < n.handlers.size(); ++i) {
            if (!n.fired[i]) {
                return config::ConfigError{graph_.mission,
                                           n.name,
                                           "G6: every handler fires in a region: " + n.name + "." + n.handlers[i].name +
                                               " fires in none"};
            }
        }
    }
    if (graph_.kind == baseline_class::scoreboard && graph_.composites.empty()) {
        return config::ConfigError{graph_.mission, "", "G8: a scoreboard graph has at least one composite"};
    }
    if (graph_.kind == baseline_class::research && graph_.instances.empty()) {
        return config::ConfigError{graph_.mission, "", "G8: a research graph has at least one operator"};
    }
    Graph g = graph_;
    for (const output_rec& o : outputs_) {
        if (!o.produced) {
            return config::ConfigError{graph_.mission,
                                       "out." + o.name,
                                       "G3: every external out port has one producer: " + in_quotes(o.name) +
                                           " has none"};
        }
        g.outputs.push_back(Output{o.name, o.region, o.from});
    }

    // G9: a graph that reads a SITL-only input, or takes its time, is SITL-only.
    const auto sitl = [&](const Ref& ref) {
        const input_rec* in = ref.from == Ref::kind::input ? find_input(ref.port) : nullptr;
        return in != nullptr && in->sitl_only;
    };
    for (const Region& r : g.regions) {
        for (const Firing& f : r.firings) {
            const input_rec* at = f.at.empty() ? nullptr : find_input(f.at);
            g.sitl_only =
                g.sitl_only || (at != nullptr && at->sitl_only) || std::any_of(f.in.begin(), f.in.end(), sitl);
        }
    }
    return g;
}

// ── The graph ───────────────────────────────────────────────────────

std::string canonical(const Graph& g) {
    std::string s = "flow-graph 1\nmission " + g.mission + "\nclass " +
                    (g.kind == baseline_class::scoreboard ? "scoreboard" : "research") + "\n";
    for (const Composite& c : g.composites) {
        s += "composite " + c.name + " " + c.body + "\n";
    }
    for (const Instance& i : g.instances) {
        s += "instance " + i.name + " " + i.op + " " + (i.composite.empty() ? "-" : i.composite) + "\n";
    }
    for (const Region& r : g.regions) {
        s += "region " + r.trigger + "\n";
        for (std::size_t i = 0; i < r.firings.size(); ++i) {
            const Firing& f = r.firings[i];
            s += "firing " + std::to_string(i) + " " + f.op + "." + f.handler + (f.at.empty() ? "" : "@" + f.at) + " " +
                 (f.composite.empty() ? "-" : f.composite) + " " + text(f.state) + "\n";
            for (const Ref& ref : f.in) {
                s += "in " + text(ref) + "\n";
            }
        }
    }
    for (const Output& o : g.outputs) {
        s += "output " + o.name + " " + o.region + " " + text(o.from) + "\n";
    }
    return s + "sitl_only " + (g.sitl_only ? "true" : "false") + "\n";
}

std::string graph_hash(const Graph& g) {
    return sha256_hex(canonical(g));
}

result<ok_t, config::ConfigError>
check_baseline(const config::BaselineConfig& b, std::string_view origin, const Graph& g) {
    const std::string file(origin);
    const std::string kind = g.kind == baseline_class::scoreboard ? "scoreboard" : "research";
    if (b.baseline_class != kind) {
        return config::ConfigError{
            file, "class", "G8: the baseline's class is its graph's, " + kind + ", not " + in_quotes(b.baseline_class)};
    }
    for (const Composite& c : g.composites) {
        const auto it = b.bodies.find(c.name);
        if (it == b.bodies.end()) {
            return config::ConfigError{file,
                                       "graph",
                                       "G8: a scoreboard baseline gives every composite a body: " + in_quotes(c.name) +
                                           " has none"};
        }
        if (it->second != c.body) {
            return config::ConfigError{file,
                                       "graph." + c.name,
                                       "G8: the body the file names fills the composite: the file names " +
                                           in_quotes(it->second) + ", and the graph has " + in_quotes(c.body)};
        }
    }
    for (const auto& [composite, body] : b.bodies) {
        if (std::none_of(
                g.composites.begin(), g.composites.end(), [&](const Composite& c) { return c.name == composite; })) {
            return config::ConfigError{
                file,
                "graph." + composite,
                "G8: a scoreboard baseline keeps the mission's composites: " + in_quotes(composite) + " is not one"};
        }
    }
    return ok;
}

}  // namespace branes::anatomy::flow
