#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/util/sha256.hpp>

#include <algorithm>
#include <memory>
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

builder::builder(std::string mission, baseline_class kind, bool runs) : runs_(runs) {
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

std::size_t builder::add_slot(std::shared_ptr<void> slot) {
    slots_.push_back(std::move(slot));
    return slots_.size() - 1;
}

input_port<Timestamp> builder::tick(std::string name) {
    const std::string trigger = name;
    auto in = add_input(
        std::move(name), false, trigger, false, std::make_shared<Timestamp>(), detail::time_fn_of<Timestamp>(nullptr));
    return input_port<Timestamp>{{in.source}, in.index};
}

builder::input_ref builder::add_input(std::string name,
                                      bool message,
                                      std::string trigger,
                                      bool sitl_only,
                                      std::shared_ptr<void> slot,
                                      std::function<Timestamp(const void*)> time) {
    check_name(name, name, "an input's");
    if (!is_name(trigger)) {
        fail(name, "G1: a trigger's names match [a-z_][a-z0-9_]*");
    }
    const std::size_t id = add_slot(std::move(slot));
    inputs_.push_back(input_rec{name, message, std::move(trigger), sitl_only, id, std::move(time)});
    input_ref r;
    r.source.ref.from = Ref::kind::input;
    r.source.ref.port = std::move(name);
    r.source.slot = id;
    r.index = inputs_.size() - 1;
    return r;
}

detail::source builder::add_constant(std::string name, std::shared_ptr<void> slot) {
    check_name(name, name, "a constant's");
    constants_.push_back(name);
    detail::source r;
    r.ref.from = Ref::kind::constant;
    r.ref.port = std::move(name);
    r.slot = add_slot(std::move(slot));
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

void builder::set_operator(std::size_t node,
                           result<std::shared_ptr<void>, std::string> op,
                           std::vector<detail::thunk_fn> thunks) {
    node_rec& n = nodes_[node];
    if (!op) {
        fail(n.name, "the operator could not be built: " + op.error());
        return;
    }
    if (operators_.size() <= n.instance) {
        operators_.resize(n.instance + 1);
    }
    operators_[n.instance] = std::move(op).value();
    n.thunks = std::move(thunks);
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
    runs_of_.emplace_back();
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

std::vector<detail::source> builder::fire(scope& s,
                                          std::size_t node,
                                          std::string_view handler,
                                          const time_of* at,
                                          std::vector<detail::source> in,
                                          std::span<const detail::slot_factory> outs,
                                          std::span<const std::type_index> types) {
    node_rec& n = nodes_[node];
    const auto it = std::find_if(
        n.handlers.begin(), n.handlers.end(), [&](const detail::handler_info& h) { return h.name == handler; });
    const auto hi = static_cast<std::size_t>(it - n.handlers.begin());
    const detail::handler_info h = *it;
    const std::string trigger = graph_.regions[s.region_].trigger;
    const std::string key = trigger + "[" + std::to_string(graph_.regions[s.region_].firings.size()) + "]";
    const std::string at_input = at != nullptr ? at->input : std::string();
    const std::vector<detail::source> none(h.out.size());

    std::size_t at_index = detail::no_slot;
    if (at != nullptr) {
        const input_rec* t = find_input(at_input);
        if (t == nullptr || t->trigger != trigger) {
            fail(key, "G6: a firing takes its time from an input of its region's trigger");
        } else {
            at_index = static_cast<std::size_t>(t - inputs_.data());
            if (runs_ && !t->time) {
                fail(key,
                     "the CPU stack takes a time from an input that has one: give " + in_quotes(t->name) +
                         " input_options::time");
            }
        }
    }
    for (std::size_t k = 0; k < in.size(); ++k) {
        check_source(in[k].ref, trigger, key + "[" + std::to_string(k) + "]");
    }
    // A body's firing is one per expansion of its composite's firing, which
    // is itself once per region and input.
    const std::string fired = n.name + "." + h.name + "@" + at_input + (s.outer_.empty() ? "" : " in " + s.outer_);
    if (contains(fired_, fired)) {
        fail(key, "G6: every firing belongs to exactly one region");
        return none;
    }
    fired_.push_back(fired);
    n.fired[hi] = true;

    // A composite whose body has several operators fires them, in place of itself.
    if (n.composite && !n.of_one_operator) {
        if (!n.body[hi]) {
            return none;  // end_body() reported it
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
    std::vector<Ref> refs;
    for (const detail::source& src : in) {
        refs.push_back(src.ref);
    }
    region.firings.push_back(Firing{op, h.name, at_input, std::move(refs), std::move(composite), h.state});

    firing_rec run{n.instance,
                   n.thunks.empty() ? nullptr : n.thunks[hi],
                   std::move(in),
                   {},
                   at_index,
                   h.out,
                   std::vector<std::type_index>(types.begin(), types.end())};
    std::vector<detail::source> result;
    for (std::size_t k = 0; k < h.out.size(); ++k) {
        detail::source out;
        out.ref.from = Ref::kind::firing;
        out.ref.region = trigger;
        out.ref.firing = region.firings.size() - 1;
        out.ref.port = h.out[k];
        if (outs[k] != nullptr) {
            out.slot = add_slot(outs[k]());
        } else if (runs_) {
            fail(key,
                 "the CPU stack runs ports that carry message types: " + op + "." + h.name + "'s " +
                     in_quotes(h.out[k]) + " carries none");
        }
        run.out.push_back(out.slot);
        result.push_back(std::move(out));
    }
    runs_of_[s.region_].push_back(std::move(run));
    return result;
}

void builder::produce_source(std::size_t out, const detail::source& from) {
    output_rec& o = outputs_[out];
    const std::string key = "out." + o.name;
    if (o.produced) {
        fail(key, "G3: every external out port has one producer: " + in_quotes(o.name) + " has two");
        return;
    }
    if (from.ref.from != Ref::kind::firing || from.ref.region != o.region || from.ref.rate != rate_rule::none) {
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
        g.outputs.push_back(Output{o.name, o.region, o.from.ref});
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

result<stack, config::ConfigError> builder::build_stack() && {
    auto g = build();
    if (!g) {
        return g.error();
    }
    if (!runs_) {
        return config::ConfigError{
            graph_.mission, "", "a stack is built by a stack_builder, which builds the operators from a context"};
    }
    if (stacked_) {
        return config::ConfigError{graph_.mission, "", "a builder builds one stack, which owns its operators"};
    }
    const auto resolve = [&](const detail::source& src) -> const void* {
        if (src.slot == detail::no_slot) {
            return nullptr;
        }
        const void* p = slots_[src.slot].get();
        for (const detail::access_fn f : src.access) {
            p = f(p);
        }
        return p;
    };
    stack st;
    for (const input_rec& in : inputs_) {
        std::size_t region = detail::no_slot;
        for (std::size_t r = 0; r < g->regions.size(); ++r) {
            if (g->regions[r].trigger == in.trigger) {
                region = r;
            }
        }
        st.inputs_.push_back(stack::input_rec{slots_[in.slot].get(), region, in.time});
    }
    for (std::size_t r = 0; r < runs_of_.size(); ++r) {
        std::vector<stack::firing> region;
        for (std::size_t i = 0; i < runs_of_[r].size(); ++i) {
            const firing_rec& f = runs_of_[r][i];
            const std::string key = g->regions[r].trigger + "[" + std::to_string(i) + "]";
            void* op = f.instance < operators_.size() ? operators_[f.instance].get() : nullptr;
            if (op == nullptr || f.thunk == nullptr) {
                return config::ConfigError{graph_.mission, key, "the CPU stack has no operator to fire"};
            }
            stack::firing run{f.thunk, op, {}, {}, f.at, f.ports, f.types};
            for (std::size_t k = 0; k < f.in.size(); ++k) {
                const void* in = resolve(f.in[k]);
                if (in == nullptr) {
                    return config::ConfigError{graph_.mission,
                                               key + "[" + std::to_string(k) + "]",
                                               "the CPU stack has no value for the source: it is no builder's"};
                }
                run.in.push_back(in);
            }
            for (const std::size_t slot : f.out) {
                run.out.push_back(slots_[slot].get());
            }
            region.push_back(std::move(run));
        }
        st.regions_.push_back(std::move(region));
    }
    for (const output_rec& o : outputs_) {
        const void* out = resolve(o.from);
        if (out == nullptr) {
            return config::ConfigError{
                graph_.mission, "out." + o.name, "the CPU stack has no value for the output: it is no builder's"};
        }
        st.outputs_.push_back(out);
    }
    // The stack owns the operators and the slots now; the builder keeps its graph.
    st.slots_ = std::move(slots_);
    st.operators_ = std::move(operators_);
    stacked_ = true;
    st.hash_ = graph_hash(*g);
    st.graph_ = std::move(g).value();
    return st;
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

namespace {

/// `s` as Mermaid text inside double quotes. `#` starts an entity, so it is
/// one too.
std::string mermaid_text(std::string_view s) {
    std::string t;
    for (const char c : s) {
        switch (c) {
        case '#':
            t += "#35;";
            break;
        case '"':
            t += "#quot;";
            break;
        case '<':
            t += "#lt;";
            break;
        case '>':
            t += "#gt;";
            break;
        default:
            t += c;
        }
    }
    return t;
}

/// The index of `name` in `names`, appended if it is not there.
std::size_t index_of(std::vector<std::string>& names, const std::string& name) {
    const auto it = std::find(names.begin(), names.end(), name);
    if (it != names.end()) {
        return static_cast<std::size_t>(it - names.begin());
    }
    names.push_back(name);
    return names.size() - 1;
}

}  // namespace

std::string mermaid(const Graph& g) {
    // The inputs and constants the graph reads, in the order it first reads them.
    std::vector<std::string> inputs;
    std::vector<std::string> constants;
    const auto source = [&](const Ref& ref) -> std::string {
        switch (ref.from) {
        case Ref::kind::input:
            return "in" + std::to_string(index_of(inputs, ref.port));
        case Ref::kind::constant:
            return "k" + std::to_string(index_of(constants, ref.port));
        case Ref::kind::firing:
            break;
        }
        for (std::size_t r = 0; r < g.regions.size(); ++r) {
            if (g.regions[r].trigger == ref.region) {
                return "r" + std::to_string(r) + "f" + std::to_string(ref.firing);
            }
        }
        return "r_" + ref.region + "f" + std::to_string(ref.firing);
    };
    // An edge names the out port it reads, and an input or a constant only
    // the fields it projects: its node names it.
    const auto edge = [&](const Ref& ref, const std::string& to) {
        std::string label = ref.from == Ref::kind::firing ? ref.port : "";
        for (const auto& f : ref.path) {
            label += "." + f;
        }
        if (ref.rate == rate_rule::hold) {
            label += label.empty() ? "hold" : " (hold)";
        }
        const std::string arrow = ref.rate == rate_rule::hold ? " -.->" : " -->";
        return "    " + source(ref) + arrow + (label.empty() ? "" : "|\"" + mermaid_text(label) + "\"|") + " " + to +
               "\n";
    };

    std::string nodes;
    std::string edges;
    for (std::size_t r = 0; r < g.regions.size(); ++r) {
        const Region& region = g.regions[r];
        nodes += "    subgraph r" + std::to_string(r) + "[\"region " + mermaid_text(region.trigger) + "\"]\n";
        for (std::size_t i = 0; i < region.firings.size(); ++i) {
            const Firing& f = region.firings[i];
            const std::string id = "r" + std::to_string(r) + "f" + std::to_string(i);
            const std::string call = f.op + "." + f.handler;
            std::string notes = f.composite.empty() || f.composite == call ? "" : f.composite;
            if (f.state != state_role::none) {
                notes += (notes.empty() ? "" : ", ") + text(f.state);
            }
            // The label leads with the firing's place in its region's order. The
            // <br/> is Mermaid's line break; the names are escaped.
            nodes += "        " + id + "[\"" + std::to_string(i) + ": " + mermaid_text(call) +
                     (f.at.empty() ? "" : "@" + mermaid_text(f.at)) +
                     (notes.empty() ? "" : "<br/>" + mermaid_text(notes)) + "\"]\n";
            if (!f.at.empty()) {
                Ref time;
                time.port = f.at;
                edges += "    " + source(time) + " -->|\"time\"| " + id + "\n";
            }
            for (const Ref& ref : f.in) {
                edges += edge(ref, id);
            }
        }
        nodes += "    end\n";
    }
    for (std::size_t o = 0; o < g.outputs.size(); ++o) {
        const std::string id = "out" + std::to_string(o);
        nodes += "    " + id + "[/\"" + mermaid_text(g.outputs[o].name) + "\"/]\n";
        edges += edge(g.outputs[o].from, id);
    }

    // A line break in the mission's name would end the comment.
    std::string mission = g.mission;
    std::replace(mission.begin(), mission.end(), '\n', ' ');
    std::replace(mission.begin(), mission.end(), '\r', ' ');
    std::string s = "flowchart LR\n    %% flow-graph " + mission + ", " +
                    (g.kind == baseline_class::scoreboard ? "scoreboard" : "research") +
                    (g.sitl_only ? ", SITL-only" : "") + "\n";
    for (std::size_t i = 0; i < inputs.size(); ++i) {
        s += "    in" + std::to_string(i) + "([\"" + mermaid_text(inputs[i]) + "\"])\n";
    }
    for (std::size_t k = 0; k < constants.size(); ++k) {
        s += "    k" + std::to_string(k) + "[(\"" + mermaid_text(constants[k]) + "\")]\n";
    }
    return s + nodes + edges;
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
