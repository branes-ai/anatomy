#include <branes/anatomy/config/yaml.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/util/sha256.hpp>

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace branes::anatomy::flow {

using namespace config::yaml;

namespace {

// ── Names ───────────────────────────────────────────────────────────

constexpr std::string_view kLower = "abcdefghijklmnopqrstuvwxyz_";
constexpr std::string_view kLowerDigits = "abcdefghijklmnopqrstuvwxyz0123456789_";
constexpr std::string_view kIdentStart = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_";
constexpr std::string_view kIdent = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_";

/// [a-z_][a-z0-9_]*
bool is_name(std::string_view s) {
    return !s.empty() && kLower.find(s.front()) != std::string_view::npos &&
           s.find_first_not_of(kLowerDigits) == std::string_view::npos;
}

/// A C++ identifier.
bool is_ident(std::string_view s) {
    return !s.empty() && kIdentStart.find(s.front()) != std::string_view::npos &&
           s.find_first_not_of(kIdent) == std::string_view::npos;
}

/// A C++ qualified name: identifiers joined by `::`.
bool is_cpp_name(std::string_view s) {
    while (true) {
        const auto at = s.find("::");
        if (!is_ident(s.substr(0, at))) {
            return false;
        }
        if (at == std::string_view::npos) {
            return true;
        }
        s.remove_prefix(at + 2);
    }
}

std::vector<std::string> split(std::string_view s, char sep) {
    std::vector<std::string> out;
    while (true) {
        const auto at = s.find(sep);
        out.emplace_back(s.substr(0, at));
        if (at == std::string_view::npos) {
            return out;
        }
        s.remove_prefix(at + 1);
    }
}

std::string in_quotes(std::string_view s) {
    return "'" + std::string(s) + "'";
}

/// G1: `name` is a name, and the first of its kind in `seen`.
void check_name(const std::string& key, const std::string& name, std::string_view what, std::set<std::string>& seen) {
    require(is_name(name), key, "G1: " + std::string(what) + " names match [a-z_][a-z0-9_]*");
    require(seen.insert(name).second, key, "G1: " + std::string(what) + " names are distinct");
}

// ── YAML shapes ─────────────────────────────────────────────────────

/// A mapping's entries, in the file's order; an absent or null node has none.
std::vector<std::pair<std::string, YAML::Node>> entries(const YAML::Node& n, const std::string& key) {
    std::vector<std::pair<std::string, YAML::Node>> out;
    if (!n || n.IsNull()) {
        return out;
    }
    require(n.IsMap(), key, "expected a mapping");
    for (const auto& kv : n) {
        out.emplace_back(kv.first.as<std::string>(), kv.second);
    }
    return out;
}

/// A mapping that may be written as null (`name:` or `name: {}`).
YAML::Node mapping_or_null(const YAML::Node& n, const std::string& key) {
    if (!n || n.IsNull()) {
        return YAML::Node(YAML::NodeType::Map);
    }
    require(n.IsMap(), key, "expected a mapping");
    return n;
}

std::string cpp_name(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const std::string s = get_string(n, prefix, key);
    require(is_cpp_name(s), join(prefix, key), "must be a C++ qualified name");
    return s;
}

std::string optional_string(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    return n[key] ? get_string(n, prefix, key) : std::string();
}

bool optional_bool(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const YAML::Node c = n[key];
    if (!c) {
        return false;
    }
    const std::string where = join(prefix, key);
    require(c.IsScalar(), where, "expected true or false");
    try {
        return c.as<bool>();
    } catch (const YAML::Exception&) {
        throw Fail{where, "expected true or false"};
    }
}

/// A list of port names, distinct (G1).
std::vector<std::string> port_list(const YAML::Node& n, const std::string& key) {
    std::vector<std::string> out;
    if (!n) {
        return out;
    }
    require(n.IsSequence(), key, "expected a list of port names");
    std::set<std::string> seen;
    for (std::size_t i = 0; i < n.size(); ++i) {
        const std::string where = key + "[" + std::to_string(i) + "]";
        require(n[i].IsScalar(), where, "expected a port name");
        out.push_back(n[i].as<std::string>());
        check_name(where, out.back(), "a handler's port", seen);
    }
    return out;
}

/// The `handlers` of `parent`, at `prefix`.
std::vector<HandlerDecl> parse_handlers(const YAML::Node& parent, const std::string& prefix) {
    const std::string key = join(prefix, "handlers");
    std::vector<HandlerDecl> out;
    std::set<std::string> seen;
    for (const auto& [name, body] : entries(child(parent, prefix, "handlers"), key)) {
        const std::string where = join(key, name);
        check_name(where, name, "a handler's", seen);
        const YAML::Node h = mapping_or_null(body, where);
        only_keys(h, where, {"in", "out", "time", "state"});
        HandlerDecl d;
        d.name = name;
        d.in = port_list(h["in"], join(where, "in"));
        d.out = port_list(h["out"], join(where, "out"));
        d.time = optional_bool(h, where, "time");
        const std::string state = optional_string(h, where, "state");
        require(state.empty() || state == "read" || state == "update", join(where, "state"), "must be read or update");
        d.state = state == "read" ? state_role::read : state == "update" ? state_role::update : state_role::none;
        out.push_back(std::move(d));
    }
    require(!out.empty(), key, "declares no handler");
    return out;
}

SourceDecl parse_source(const YAML::Node& n, const std::string& key) {
    SourceDecl s;
    if (n.IsScalar()) {
        s.ref = n.as<std::string>();
        return s;
    }
    require(n.IsMap(), key, "expected a source: <ref> or {from: <ref>, rate: hold}");
    only_keys(n, key, {"from", "rate"});
    s.ref = get_string(n, key, "from");
    const std::string rate = get_string(n, key, "rate");
    require(rate == "hold", join(key, "rate"), "G7: step 2 accepts the rate rule hold only");
    s.rate = rate_rule::hold;
    return s;
}

/// `<node>.<handler>[@<input>]`, or `<node>.<handler>: <sources>`.
FiringDecl parse_firing(const YAML::Node& n, const std::string& key) {
    std::string target;
    YAML::Node sources;
    if (n.IsScalar()) {
        target = n.as<std::string>();
    } else {
        require(n.IsMap() && n.size() == 1, key, "expected <node>.<handler>, or <node>.<handler>: [sources]");
        const auto kv = *n.begin();
        target = kv.first.as<std::string>();
        sources = kv.second;
    }
    FiringDecl f;
    const auto at = target.find('@');
    if (at != std::string::npos) {
        f.at = target.substr(at + 1);
        target.resize(at);
        require(is_name(f.at), key, "G1: the input after @ is a name");
    }
    const auto parts = split(target, '.');
    require(parts.size() == 2 && is_name(parts[0]) && is_name(parts[1]), key, "expected <node>.<handler>[@<input>]");
    f.node = parts[0];
    f.handler = parts[1];
    if (sources && !sources.IsNull()) {
        if (sources.IsSequence()) {
            for (std::size_t i = 0; i < sources.size(); ++i) {
                f.sources.push_back(parse_source(sources[i], key + "[" + std::to_string(i) + "]"));
            }
        } else {
            f.sources.push_back(parse_source(sources, key));
        }
    }
    return f;
}

std::vector<FiringDecl> parse_firings(const YAML::Node& n, const std::string& key) {
    require(n.IsSequence(), key, "expected a list of firings");
    std::vector<FiringDecl> out;
    for (std::size_t i = 0; i < n.size(); ++i) {
        out.push_back(parse_firing(n[i], key + "[" + std::to_string(i) + "]"));
    }
    return out;
}

std::vector<RegionDecl> parse_regions(const YAML::Node& n, const std::string& key) {
    std::vector<RegionDecl> out;
    std::set<std::string> seen;
    for (const auto& [trigger, list] : entries(n, key)) {
        const std::string where = join(key, trigger);
        check_name(where, trigger, "a region's", seen);
        out.push_back(RegionDecl{trigger, parse_firings(list, where)});
    }
    return out;
}

// ── Resolving sources ───────────────────────────────────────────────

/// A composite or an operator instance, as a region's firings see it.
struct Node {
    std::string name;
    const std::vector<HandlerDecl>* handlers = nullptr;
};

const HandlerDecl* find_handler(const std::vector<HandlerDecl>& hs, std::string_view name) {
    const auto it = std::find_if(hs.begin(), hs.end(), [&](const HandlerDecl& h) { return h.name == name; });
    return it == hs.end() ? nullptr : &*it;
}

bool has(const std::vector<std::string>& v, std::string_view s) {
    return std::find(v.begin(), v.end(), s) != v.end();
}

const InputDecl* find_input(const Interface& m, std::string_view name) {
    const auto it = std::find_if(m.inputs.begin(), m.inputs.end(), [&](const InputDecl& d) { return d.name == name; });
    return it == m.inputs.end() ? nullptr : &*it;
}

const ConstantDecl* find_constant(const Interface& m, std::string_view name) {
    const auto it =
        std::find_if(m.constants.begin(), m.constants.end(), [&](const ConstantDecl& d) { return d.name == name; });
    return it == m.constants.end() ? nullptr : &*it;
}

/// A region's firing, its node and handler found.
struct Placed {
    std::size_t node = 0;
    const HandlerDecl* handler = nullptr;
    const FiringDecl* decl = nullptr;
};

/// A set of regions over a set of nodes: the interface's over its composites,
/// or a research baseline's over its instances.
class Regions {
public:
    /// Check each firing names a node's handler (G2), takes a source per in
    /// port and its time from an input of its region (G3, G6), and appears
    /// once (G6), with at most one update of a node's state per region (G5).
    Regions(const Interface& mission, std::vector<Node> nodes, const std::vector<RegionDecl>& regions, std::string key)
        : mission_(mission), nodes_(std::move(nodes)), regions_(regions), key_(std::move(key)) {
        std::set<std::string> fired;
        placed_.resize(regions_.size());
        for (std::size_t r = 0; r < regions_.size(); ++r) {
            const RegionDecl& region = regions_[r];
            const std::string rkey = join(key_, region.trigger);
            require(std::any_of(mission_.inputs.begin(),
                                mission_.inputs.end(),
                                [&](const InputDecl& d) { return d.trigger == region.trigger; }),
                    rkey,
                    "G6: a region is a trigger an input declares");
            std::set<std::size_t> updated;
            for (std::size_t i = 0; i < region.firings.size(); ++i) {
                const FiringDecl& f = region.firings[i];
                const std::string fkey = rkey + "[" + std::to_string(i) + "]";
                const auto n = node_index(f.node);
                require(n.has_value(), fkey, "G2: " + in_quotes(f.node) + " is not a node of this graph");
                const HandlerDecl* h = find_handler(*nodes_[*n].handlers, f.handler);
                require(h != nullptr, fkey, "G2: " + in_quotes(f.node) + " has no handler " + in_quotes(f.handler));
                require(f.sources.size() == h->in.size(),
                        fkey,
                        "G3: every in port has one source: " + f.node + "." + f.handler + " has " +
                            std::to_string(h->in.size()) + " in ports, and " + std::to_string(f.sources.size()) +
                            " sources");
                require(!h->time || !f.at.empty(),
                        fkey,
                        "G3: " + f.node + "." + f.handler + " takes a time: name its input, " + f.node + "." +
                            f.handler + "@<input>");
                require(h->time || f.at.empty(), fkey, "G3: " + f.node + "." + f.handler + " takes no time");
                if (!f.at.empty()) {
                    const InputDecl* in = find_input(mission_, f.at);
                    require(in != nullptr && in->trigger == region.trigger,
                            fkey,
                            "G6: a firing takes its time from an input of its region's trigger");
                }
                require(fired.insert(f.node + "." + f.handler + "@" + f.at).second,
                        fkey,
                        "G6: every firing belongs to exactly one region");
                require(h->state != state_role::update || updated.insert(*n).second,
                        fkey,
                        "G5: a region updates a node's state once, so each read is before or after it");
                placed_[r].push_back(Placed{*n, h, &f});
            }
        }
    }

    [[nodiscard]] const std::vector<Node>& nodes() const noexcept {
        return nodes_;
    }
    [[nodiscard]] const std::vector<RegionDecl>& regions() const noexcept {
        return regions_;
    }
    [[nodiscard]] const std::vector<std::vector<Placed>>& placed() const noexcept {
        return placed_;
    }

    /// Every handler of every node fires in some region (G6).
    void check_all_fired() const {
        for (const Node& n : nodes_) {
            for (const HandlerDecl& h : *n.handlers) {
                const bool fired = std::any_of(placed_.begin(), placed_.end(), [&](const std::vector<Placed>& ps) {
                    return std::any_of(ps.begin(), ps.end(), [&](const Placed& p) {
                        return nodes_[p.node].name == n.name && p.handler == &h;
                    });
                });
                require(
                    fired, key_, "G6: every handler fires in a region: " + n.name + "." + h.name + " fires in none");
            }
        }
    }

    /// The source `s` of a firing at index `pos` of region `r` (pos = the
    /// region's size for an output). Refs of kind firing index this object's
    /// regions. G3: it exists, with one producer; G5: a producer in the region
    /// comes first; G7: an edge between regions holds, one within doesn't.
    [[nodiscard]] Ref resolve(const SourceDecl& s, std::size_t r, std::size_t pos, const std::string& key) const {
        const std::string& trigger = regions_[r].trigger;
        const auto segs = split(s.ref, '.');
        for (const auto& seg : segs) {
            require(is_name(seg), key, "G3: a source is <port>[.<field>...] or <node>.<port>[.<field>...]");
        }
        Ref ref;
        ref.rate = s.rate;
        if (const auto n = node_index(segs[0])) {
            require(segs.size() >= 2, key, "G3: a source names a node's out port, " + segs[0] + ".<port>");
            ref.from = Ref::kind::firing;
            ref.port = segs[1];
            ref.path.assign(segs.begin() + 2, segs.end());
            std::vector<std::pair<std::size_t, std::size_t>> here;
            std::vector<std::pair<std::size_t, std::size_t>> there;
            for (std::size_t q = 0; q < placed_.size(); ++q) {
                for (std::size_t i = 0; i < placed_[q].size(); ++i) {
                    const Placed& p = placed_[q][i];
                    if (p.node == *n && has(p.handler->out, ref.port)) {
                        (q == r ? here : there).emplace_back(q, i);
                    }
                }
            }
            const auto& producers = here.empty() ? there : here;
            require(!producers.empty(),
                    key,
                    "G3: every source exists: no firing of " + in_quotes(segs[0]) + " writes " + in_quotes(ref.port));
            require(producers.size() == 1,
                    key,
                    "G3: a source has one producer: more than one firing of " + in_quotes(segs[0]) + " writes " +
                        in_quotes(ref.port));
            ref.region = regions_[producers[0].first].trigger;
            ref.firing = producers[0].second;
            if (!here.empty()) {
                require(ref.firing < pos,
                        key,
                        "G5: the declared order is a topological order: " + s.ref + " is written after it is read");
                require(s.rate == rate_rule::none, key, "G7: an edge within a region has no rate rule");
            } else {
                require(s.rate != rate_rule::none,
                        key,
                        "G7: an edge between regions has a rate rule: " + s.ref + " is written in region " +
                            in_quotes(ref.region) + ", and needs {from: " + s.ref + ", rate: hold}");
            }
            return ref;
        }
        ref.port = segs[0];
        ref.path.assign(segs.begin() + 1, segs.end());
        if (const InputDecl* in = find_input(mission_, ref.port)) {
            ref.from = Ref::kind::input;
            require(
                !in->message.empty() || ref.path.empty(), key, "G3: " + in_quotes(in->name) + " carries only its time");
            if (in->trigger == trigger) {
                require(s.rate == rate_rule::none, key, "G7: an edge within a region has no rate rule");
            } else {
                require(s.rate != rate_rule::none,
                        key,
                        "G7: an edge between regions has a rate rule: " + in_quotes(in->name) + " fires " +
                            in_quotes(in->trigger) + ", and needs {from: " + s.ref + ", rate: hold}");
            }
            return ref;
        }
        require(find_constant(mission_, ref.port) != nullptr,
                key,
                "G3: every source exists: " + in_quotes(s.ref) + " names no node, input or constant");
        require(s.rate == rate_rule::none, key, "G7: a constant takes no rate rule");
        ref.from = Ref::kind::constant;
        return ref;
    }

    /// Every firing's sources, resolved: [region][firing][in port].
    [[nodiscard]] std::vector<std::vector<std::vector<Ref>>> resolve_all() const {
        std::vector<std::vector<std::vector<Ref>>> out(regions_.size());
        for (std::size_t r = 0; r < regions_.size(); ++r) {
            for (std::size_t i = 0; i < placed_[r].size(); ++i) {
                const FiringDecl& f = *placed_[r][i].decl;
                std::vector<Ref> in;
                for (std::size_t k = 0; k < f.sources.size(); ++k) {
                    const std::string skey =
                        join(key_, regions_[r].trigger) + "[" + std::to_string(i) + "][" + std::to_string(k) + "]";
                    in.push_back(resolve(f.sources[k], r, i, skey));
                }
                out[r].push_back(std::move(in));
            }
        }
        return out;
    }

    /// The region index of `trigger`, if a region has it.
    [[nodiscard]] std::optional<std::size_t> region_index(std::string_view trigger) const {
        for (std::size_t r = 0; r < regions_.size(); ++r) {
            if (regions_[r].trigger == trigger) {
                return r;
            }
        }
        return std::nullopt;
    }

private:
    [[nodiscard]] std::optional<std::size_t> node_index(std::string_view name) const {
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            if (nodes_[i].name == name) {
                return i;
            }
        }
        return std::nullopt;
    }

    const Interface& mission_;
    std::vector<Node> nodes_;
    const std::vector<RegionDecl>& regions_;
    std::string key_;
    std::vector<std::vector<Placed>> placed_;
};

std::vector<Node> composite_nodes(const Interface& m) {
    std::vector<Node> out;
    for (const CompositeDecl& c : m.composites) {
        out.push_back(Node{c.name, &c.handlers});
    }
    return out;
}

/// The output `o`, resolved at the end of its region (G3, G6).
Ref resolve_output(const Regions& regions, const OutputDecl& o, const std::string& from, const std::string& key) {
    const auto r = regions.region_index(o.region);
    require(r.has_value(), key, "G6: an output's region is a region of the graph");
    return regions.resolve(SourceDecl{from, rate_rule::none}, *r, regions.placed()[*r].size(), key);
}

// ── Operators ───────────────────────────────────────────────────────

const OperatorDecl* find_operator(const Registry& reg, std::string_view name) {
    const auto it =
        std::find_if(reg.operators.begin(), reg.operators.end(), [&](const OperatorDecl& d) { return d.name == name; });
    return it == reg.operators.end() ? nullptr : &*it;
}

const CompositeDecl* find_composite(const Interface& m, std::string_view name) {
    const auto it =
        std::find_if(m.composites.begin(), m.composites.end(), [&](const CompositeDecl& c) { return c.name == name; });
    return it == m.composites.end() ? nullptr : &*it;
}

/// A registered operator `name`, with its handlers: its composite's or its own (G2).
const std::vector<HandlerDecl>&
operator_handlers(const Interface& m, const Registry& reg, const std::string& name, const std::string& key) {
    const OperatorDecl* op = find_operator(reg, name);
    require(op != nullptr, key, "G2: every operator is registered: " + in_quotes(name) + " is not");
    if (op->composite.empty()) {
        return op->handlers;
    }
    const CompositeDecl* c = find_composite(m, op->composite);
    require(c != nullptr,
            key,
            "G2: " + in_quotes(name) + " fills composite " + in_quotes(op->composite) +
                ", which the interface doesn't declare");
    return c->handlers;
}

/// G1: instance names, and that no name of the graph's nodes is also an
/// external port's, so a source names one thing.
void check_node_names(const Interface& m,
                      const std::string& key,
                      const std::string& name,
                      std::set<std::string>& seen) {
    check_name(key, name, "a node's", seen);
    require(find_input(m, name) == nullptr && find_constant(m, name) == nullptr,
            key,
            "G1: nodes and external ports share one namespace: " + in_quotes(name) + " is both");
}

// ── Flattening ──────────────────────────────────────────────────────

/// A flat firing's source while the graph is laid out: a Ref to a flat
/// firing, or, declared, a Ref to a composite's firing in the interface's
/// regions, whose out ports are known once every region is laid out.
struct Pending {
    Ref ref;
    bool declared = false;
};

/// The flat firings, region by region, and what each composite firing's out
/// ports became.
struct Layout {
    std::vector<std::string> triggers;
    std::vector<std::vector<std::pair<Firing, std::vector<Pending>>>> firings;  ///< [region]
    std::vector<std::vector<std::map<std::string, Pending>>> outs;  ///< [region][composite firing][out port]

    [[nodiscard]] std::size_t region(std::string_view trigger) const {
        return static_cast<std::size_t>(std::find(triggers.begin(), triggers.end(), trigger) - triggers.begin());
    }

    /// Append a firing to region r; returns its index there.
    std::size_t add(std::size_t r, Firing f, std::vector<Pending> in) {
        firings[r].emplace_back(std::move(f), std::move(in));
        return firings[r].size() - 1;
    }

    /// `p`, rewritten to the flat firings. A body may pass a composite's in
    /// port through to an out port, so this follows a chain of them.
    [[nodiscard]] Ref flat(Pending p, const std::string& key) const {
        std::size_t composite_firings = 0;
        for (const auto& r : outs) {
            composite_firings += r.size();
        }
        for (std::size_t hops = 0; p.declared; ++hops) {
            require(hops <= composite_firings, key, "G5: the flow edges are acyclic: a cycle passes through bodies");
            Pending next = outs[region(p.ref.region)][p.ref.firing].at(p.ref.port);
            next.ref.path.insert(next.ref.path.end(), p.ref.path.begin(), p.ref.path.end());
            if (p.ref.rate != rate_rule::none) {
                next.ref.rate = p.ref.rate;
            }
            p = std::move(next);
        }
        return p.ref;
    }
};

/// A scoreboard body of several operators, for one composite.
struct Body {
    std::vector<std::pair<std::string, std::string>> instances;     ///< local name → operator
    std::map<std::string, std::vector<FiringDecl>> fire;            ///< composite handler → its firings
    std::map<std::string, std::map<std::string, std::string>> out;  ///< handler → out port → source
};

Body parse_body(
    const YAML::Node& n, const std::string& key, const Interface& m, const Registry& reg, const CompositeDecl& c) {
    only_keys(n, key, {"operators", "handlers"});
    Body b;
    std::set<std::string> in_ports;
    for (const auto& h : c.handlers) {
        in_ports.insert(h.in.begin(), h.in.end());
    }
    std::set<std::string> seen;
    for (const auto& [local, op] : entries(child(n, key, "operators"), join(key, "operators"))) {
        const std::string where = join(join(key, "operators"), local);
        check_name(where, local, "a node's", seen);
        require(!in_ports.contains(local),
                where,
                "G1: a body's operators and its composite's in ports share one namespace: " + in_quotes(local) +
                    " is both");
        require(op.IsScalar(), where, "expected an operator name");
        (void)operator_handlers(m, reg, op.as<std::string>(), where);
        b.instances.emplace_back(local, op.as<std::string>());
    }
    require(!b.instances.empty(), join(key, "operators"), "G8: a body has at least one operator");
    for (const auto& [name, body] : entries(child(n, key, "handlers"), join(key, "handlers"))) {
        const std::string where = join(join(key, "handlers"), name);
        const HandlerDecl* h = find_handler(c.handlers, name);
        require(h != nullptr, where, "G8: " + c.name + " has no handler " + in_quotes(name));
        require(!b.fire.contains(name), where, "G1: a body's handler names are distinct");
        const YAML::Node hb = mapping_or_null(body, where);
        only_keys(hb, where, {"fire", "out"});
        b.fire[name] = parse_firings(child(hb, where, "fire"), join(where, "fire"));
        auto& outs = b.out[name];
        for (const auto& [port, ref] : entries(hb["out"], join(where, "out"))) {
            const std::string pkey = join(join(where, "out"), port);
            require(has(h->out, port), pkey, "G3: " + c.name + "." + name + " has no out port " + in_quotes(port));
            require(ref.IsScalar(), pkey, "expected a source");
            outs[port] = ref.as<std::string>();
        }
        for (const auto& port : h->out) {
            require(outs.contains(port),
                    join(where, "out"),
                    "G3: every out port has one producer: " + c.name + "." + name + " writes " + in_quotes(port));
        }
    }
    for (const auto& h : c.handlers) {
        require(b.fire.contains(h.name),
                join(key, "handlers"),
                "G8: a body implements every handler of its composite: " + c.name + "." + h.name + " has none");
    }
    return b;
}

/// Lay out, in region r, the firings of composite handler `ch`'s body, fired
/// with sources `in` and the time of input `at`. Returns what ch's out ports
/// become.
std::map<std::string, Pending> expand_body(Layout& lay,
                                           std::size_t r,
                                           const Interface& m,
                                           const Registry& reg,
                                           const CompositeDecl& c,
                                           const Body& body,
                                           const HandlerDecl& ch,
                                           const std::string& at,
                                           const std::vector<Pending>& in,
                                           const std::string& key) {
    std::vector<Node> locals;
    for (const auto& [name, op] : body.instances) {
        locals.push_back(Node{name, &operator_handlers(m, reg, op, key)});
    }
    const auto local_of = [&](std::string_view name) {
        return std::find_if(locals.begin(), locals.end(), [&](const Node& l) { return l.name == name; });
    };
    const auto& fires = body.fire.at(ch.name);
    std::vector<std::size_t> index;  // each body firing's flat index
    // A source: one of ch's in ports, or an earlier body firing's out port.
    const auto resolve = [&](const std::string& text, std::size_t pos, const std::string& skey) -> Pending {
        const auto segs = split(text, '.');
        for (const auto& seg : segs) {
            require(is_name(seg), skey, "G3: a source is <port>[.<field>...] or <node>.<port>[.<field>...]");
        }
        if (const auto port = std::find(ch.in.begin(), ch.in.end(), segs[0]); port != ch.in.end()) {
            Pending p = in[static_cast<std::size_t>(port - ch.in.begin())];
            p.ref.path.insert(p.ref.path.end(), segs.begin() + 1, segs.end());
            return p;
        }
        const auto local = local_of(segs[0]);
        require(local != locals.end(),
                skey,
                "G3: every source exists: " + in_quotes(text) + " names no operator, or in port of " + c.name + "." +
                    ch.name);
        require(segs.size() >= 2, skey, "G3: a source names an operator's out port, " + segs[0] + ".<port>");
        std::size_t producers = 0;
        std::size_t producer = 0;
        for (std::size_t i = 0; i < fires.size(); ++i) {
            const HandlerDecl* h =
                fires[i].node == segs[0] ? find_handler(*local->handlers, fires[i].handler) : nullptr;
            if (h != nullptr && has(h->out, segs[1])) {
                ++producers;
                producer = i;
            }
        }
        require(producers > 0,
                skey,
                "G3: every source exists: no firing of " + in_quotes(segs[0]) + " writes " + in_quotes(segs[1]));
        require(producers == 1,
                skey,
                "G3: a source has one producer: more than one firing of " + in_quotes(segs[0]) + " writes " +
                    in_quotes(segs[1]));
        require(producer < pos,
                skey,
                "G5: the declared order is a topological order: " + text + " is written after it is read");
        Ref ref;
        ref.from = Ref::kind::firing;
        ref.region = lay.triggers[r];
        ref.firing = index[producer];
        ref.port = segs[1];
        ref.path.assign(segs.begin() + 2, segs.end());
        return Pending{ref, false};
    };
    std::set<std::string> updated;
    for (std::size_t i = 0; i < fires.size(); ++i) {
        const FiringDecl& f = fires[i];
        const std::string fkey = join(key, "fire") + "[" + std::to_string(i) + "]";
        const auto local = local_of(f.node);
        require(local != locals.end(), fkey, "G2: " + in_quotes(f.node) + " is not an operator of this body");
        const HandlerDecl* h = find_handler(*local->handlers, f.handler);
        require(h != nullptr, fkey, "G2: " + in_quotes(f.node) + " has no handler " + in_quotes(f.handler));
        require(f.at.empty(), fkey, "G3: a body's firing takes its composite handler's time, and names no input");
        require(!h->time || ch.time,
                fkey,
                "G3: " + f.node + "." + f.handler + " takes a time, and " + c.name + "." + ch.name + " has none");
        require(f.sources.size() == h->in.size(),
                fkey,
                "G3: every in port has one source: " + f.node + "." + f.handler + " has " +
                    std::to_string(h->in.size()) + " in ports, and " + std::to_string(f.sources.size()) + " sources");
        require(h->state != state_role::update || updated.insert(f.node).second,
                fkey,
                "G5: a region updates a node's state once, so each read is before or after it");
        std::vector<Pending> sources;
        for (std::size_t k = 0; k < f.sources.size(); ++k) {
            const std::string skey = fkey + "[" + std::to_string(k) + "]";
            require(f.sources[k].rate == rate_rule::none, skey, "G7: an edge within a body has no rate rule");
            sources.push_back(resolve(f.sources[k].ref, i, skey));
        }
        index.push_back(lay.add(
            r,
            Firing{
                c.name + "." + f.node, f.handler, h->time ? at : std::string(), {}, c.name + "." + ch.name, h->state},
            std::move(sources)));
    }
    std::map<std::string, Pending> outs;
    for (const auto& [port, text] : body.out.at(ch.name)) {
        outs[port] = resolve(text, fires.size(), join(join(key, "out"), port));
    }
    return outs;
}

void mark_sitl(Graph& g, const Interface& m) {
    const auto sitl = [&](const Ref& ref) {
        if (ref.from != Ref::kind::input) {
            return false;
        }
        const InputDecl* in = find_input(m, ref.port);
        return in != nullptr && in->sitl_only;
    };
    for (const Region& r : g.regions) {
        for (const Firing& f : r.firings) {
            const InputDecl* at = f.at.empty() ? nullptr : find_input(m, f.at);
            g.sitl_only =
                g.sitl_only || (at != nullptr && at->sitl_only) || std::any_of(f.in.begin(), f.in.end(), sitl);
        }
    }
    for (const Output& o : g.outputs) {
        g.sitl_only = g.sitl_only || sitl(o.from);
    }
}

Graph scoreboard_graph(const YAML::Node& graph, const Interface& m, const Registry& reg) {
    Graph g;
    g.mission = m.mission;
    g.kind = baseline_class::scoreboard;
    std::map<std::string, std::string> single;  // composite → its one operator
    std::map<std::string, Body> bodies;
    for (const auto& [name, body] : entries(graph, "graph")) {
        const std::string key = join("graph", name);
        const CompositeDecl* c = find_composite(m, name);
        require(c != nullptr,
                key,
                "G8: a scoreboard baseline keeps the interface's composites: " + in_quotes(name) + " is not one");
        require(!single.contains(name) && !bodies.contains(name), key, "G1: composite names are distinct");
        if (body.IsScalar()) {
            const std::string op = body.as<std::string>();
            (void)operator_handlers(m, reg, op, key);
            require(find_operator(reg, op)->composite == name,
                    key,
                    "G2: a body of one operator fills its composite: " + in_quotes(op) + " doesn't fill " +
                        in_quotes(name));
            single[name] = op;
        } else {
            require(body.IsMap(), key, "expected an operator name, or {operators, handlers}");
            bodies[name] = parse_body(body, key, m, reg, *c);
        }
    }
    for (const CompositeDecl& c : m.composites) {
        if (const auto it = single.find(c.name); it != single.end()) {
            g.instances.push_back(Instance{c.name, it->second, c.name});
        } else {
            const auto body = bodies.find(c.name);
            require(body != bodies.end(),
                    "graph",
                    "G8: a scoreboard baseline gives every composite a body: " + in_quotes(c.name) + " has none");
            for (const auto& [local, op] : body->second.instances) {
                g.instances.push_back(Instance{c.name + "." + local, op, c.name});
            }
        }
    }

    // The interface's regions, checked over the composites, with each
    // composite firing laid out as its operator's firing or its body's.
    const Regions regions(m, composite_nodes(m), m.regions, "regions");
    const auto sources = regions.resolve_all();
    Layout lay;
    for (std::size_t r = 0; r < m.regions.size(); ++r) {
        lay.triggers.push_back(m.regions[r].trigger);
        lay.outs.emplace_back(regions.placed()[r].size());
    }
    lay.firings.resize(m.regions.size());
    for (std::size_t r = 0; r < m.regions.size(); ++r) {
        for (std::size_t i = 0; i < regions.placed()[r].size(); ++i) {
            const Placed& p = regions.placed()[r][i];
            const CompositeDecl& c = m.composites[p.node];
            std::vector<Pending> in;
            for (const Ref& ref : sources[r][i]) {
                in.push_back(Pending{ref, ref.from == Ref::kind::firing});
            }
            if (const auto body = bodies.find(c.name); body != bodies.end()) {
                lay.outs[r][i] = expand_body(lay,
                                             r,
                                             m,
                                             reg,
                                             c,
                                             body->second,
                                             *p.handler,
                                             p.decl->at,
                                             in,
                                             join(join(join("graph", c.name), "handlers"), p.handler->name));
            } else {
                const std::size_t at = lay.add(
                    r,
                    Firing{c.name, p.handler->name, p.decl->at, {}, c.name + "." + p.handler->name, p.handler->state},
                    std::move(in));
                for (const auto& port : p.handler->out) {
                    Ref ref;
                    ref.from = Ref::kind::firing;
                    ref.region = lay.triggers[r];
                    ref.firing = at;
                    ref.port = port;
                    lay.outs[r][i][port] = Pending{ref, false};
                }
            }
        }
    }
    for (std::size_t r = 0; r < m.regions.size(); ++r) {
        Region region{lay.triggers[r], {}};
        for (auto& [f, in] : lay.firings[r]) {
            for (const Pending& p : in) {
                f.in.push_back(lay.flat(p, "graph"));
            }
            region.firings.push_back(std::move(f));
        }
        g.regions.push_back(std::move(region));
    }
    for (const OutputDecl& o : m.outputs) {
        const Ref from = resolve_output(regions, o, o.from, join(join("external.out", o.name), "from"));
        g.outputs.push_back(Output{o.name, o.region, lay.flat(Pending{from, from.from == Ref::kind::firing}, "graph")});
    }
    mark_sitl(g, m);
    return g;
}

Graph research_graph(const YAML::Node& graph, const Interface& m, const Registry& reg) {
    only_keys(graph, "graph", {"operators", "regions", "out"});
    Graph g;
    g.mission = m.mission;
    g.kind = baseline_class::research;
    std::vector<Node> nodes;
    std::set<std::string> seen;
    for (const auto& [name, op] : entries(child(graph, "graph", "operators"), "graph.operators")) {
        const std::string key = join("graph.operators", name);
        check_node_names(m, key, name, seen);
        require(op.IsScalar(), key, "expected an operator name");
        nodes.push_back(Node{name, &operator_handlers(m, reg, op.as<std::string>(), key)});
        g.instances.push_back(Instance{name, op.as<std::string>(), ""});
    }
    require(!nodes.empty(), "graph.operators", "G8: a research graph has at least one operator");
    const std::vector<RegionDecl> decls = parse_regions(child(graph, "graph", "regions"), "graph.regions");
    const Regions regions(m, std::move(nodes), decls, "graph.regions");
    const auto sources = regions.resolve_all();
    for (std::size_t r = 0; r < decls.size(); ++r) {
        Region region{decls[r].trigger, {}};
        for (std::size_t i = 0; i < regions.placed()[r].size(); ++i) {
            const Placed& p = regions.placed()[r][i];
            region.firings.push_back(
                Firing{regions.nodes()[p.node].name, p.handler->name, p.decl->at, sources[r][i], "", p.handler->state});
        }
        g.regions.push_back(std::move(region));
    }
    const auto outs = entries(child(graph, "graph", "out"), "graph.out");
    for (const auto& [name, ref] : outs) {
        const std::string key = join("graph.out", name);
        require(std::any_of(m.outputs.begin(), m.outputs.end(), [&](const OutputDecl& o) { return o.name == name; }),
                key,
                "G3: " + in_quotes(name) + " is not an output of the interface");
        require(ref.IsScalar(), key, "expected a source");
    }
    for (const OutputDecl& o : m.outputs) {
        const auto it = std::find_if(outs.begin(), outs.end(), [&](const auto& e) { return e.first == o.name; });
        require(it != outs.end(),
                "graph.out",
                "G3: every external out port has one producer: " + in_quotes(o.name) + " has none");
        g.outputs.push_back(Output{
            o.name, o.region, resolve_output(regions, o, it->second.as<std::string>(), join("graph.out", o.name))});
    }
    mark_sitl(g, m);
    return g;
}

// ── Canonical form ──────────────────────────────────────────────────

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

std::string text(state_role s) {
    return s == state_role::read ? "read" : s == state_role::update ? "update" : "none";
}

}  // namespace

// ── Public entry points ─────────────────────────────────────────────

result<Interface, config::ConfigError> parse_interface(std::string_view yaml, std::string_view origin) {
    return guarded<Interface>(origin, [&]() -> Interface {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"mission", "profile", "external", "composites", "regions"});
        Interface m;
        m.mission = get_string(root, "", "mission");
        require(is_name(m.mission), "mission", "G1: a mission's name matches [a-z_][a-z0-9_]*");
        m.profile = cpp_name(root, "", "profile");

        const YAML::Node external = child(root, "", "external");
        only_keys(external, "external", {"in", "constant", "out"});
        std::set<std::string> ports;
        for (const auto& [name, body] : entries(child(external, "external", "in"), "external.in")) {
            const std::string key = join("external.in", name);
            check_name(key, name, "an external port's", ports);
            only_keys(body, key, {"message", "trigger", "time", "sitl_only"});
            InputDecl d;
            d.name = name;
            d.message = body["message"] ? cpp_name(body, key, "message") : std::string();
            d.trigger = get_string(body, key, "trigger");
            require(is_name(d.trigger), join(key, "trigger"), "G1: a trigger's name matches [a-z_][a-z0-9_]*");
            d.time = optional_string(body, key, "time");
            require(d.time.empty() || !d.message.empty(), join(key, "time"), "an input with no message is its time");
            for (const auto& seg : d.time.empty() ? std::vector<std::string>{} : split(d.time, '.')) {
                require(is_name(seg), join(key, "time"), "must be a member path, <field>[.<field>...]");
            }
            d.sitl_only = optional_bool(body, key, "sitl_only");
            m.inputs.push_back(std::move(d));
        }
        require(!m.inputs.empty(), "external.in", "the stack has at least one input");
        for (const auto& [name, body] : entries(external["constant"], "external.constant")) {
            const std::string key = join("external.constant", name);
            check_name(key, name, "an external port's", ports);
            only_keys(body, key, {"message", "from"});
            m.constants.push_back(
                ConstantDecl{name, cpp_name(body, key, "message"), optional_string(body, key, "from")});
        }
        for (const auto& [name, body] : entries(child(external, "external", "out"), "external.out")) {
            const std::string key = join("external.out", name);
            check_name(key, name, "an external port's", ports);
            only_keys(body, key, {"message", "region", "from"});
            m.outputs.push_back(OutputDecl{
                name, cpp_name(body, key, "message"), get_string(body, key, "region"), get_string(body, key, "from")});
        }
        require(!m.outputs.empty(), "external.out", "the stack has at least one output");

        std::set<std::string> composites;
        for (const auto& [name, body] : entries(child(root, "", "composites"), "composites")) {
            const std::string key = join("composites", name);
            check_name(key, name, "a composite's", composites);
            require(find_input(m, name) == nullptr && find_constant(m, name) == nullptr,
                    key,
                    "G1: nodes and external ports share one namespace: " + in_quotes(name) + " is both");
            only_keys(body, key, {"signature", "binding", "handlers"});
            m.composites.push_back(CompositeDecl{
                name, cpp_name(body, key, "signature"), cpp_name(body, key, "binding"), parse_handlers(body, key)});
        }
        require(!m.composites.empty(), "composites", "the stack has at least one composite");

        m.regions = parse_regions(child(root, "", "regions"), "regions");
        const Regions regions(m, composite_nodes(m), m.regions, "regions");
        regions.check_all_fired();
        (void)regions.resolve_all();
        for (const OutputDecl& o : m.outputs) {
            (void)resolve_output(regions, o, o.from, join(join("external.out", o.name), "from"));
        }
        return m;
    });
}

result<Interface, config::ConfigError> load_interface(const std::filesystem::path& path) {
    return load_file<Interface>(
        path, [](std::string_view text, std::string_view origin) { return parse_interface(text, origin); });
}

result<Registry, config::ConfigError> parse_registry(std::string_view yaml, std::string_view origin) {
    return guarded<Registry>(origin, [&]() -> Registry {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"operators"});
        Registry reg;
        std::set<std::string> seen;
        for (const auto& [name, body] : entries(child(root, "", "operators"), "operators")) {
            const std::string key = join("operators", name);
            require(is_ident(name), key, "G1: an operator's name is a C++ identifier");
            require(seen.insert(name).second, key, "G1: operator names are distinct");
            require(body.IsMap(), key, "expected a mapping");
            OperatorDecl d;
            d.name = name;
            d.header = get_string(body, key, "header");
            d.type = cpp_name(body, key, "type");
            if (body["composite"]) {
                only_keys(body, key, {"header", "type", "composite"});
                d.composite = get_string(body, key, "composite");
                require(is_name(d.composite), join(key, "composite"), "G1: a composite's names match [a-z_][a-z0-9_]*");
            } else {
                only_keys(body, key, {"header", "type", "signature", "binding", "handlers"});
                d.signature = cpp_name(body, key, "signature");
                d.binding = cpp_name(body, key, "binding");
                d.handlers = parse_handlers(body, key);
            }
            reg.operators.push_back(std::move(d));
        }
        return reg;
    });
}

result<Registry, config::ConfigError> load_registry(const std::filesystem::path& path) {
    return load_file<Registry>(
        path, [](std::string_view text, std::string_view origin) { return parse_registry(text, origin); });
}

result<Graph, config::ConfigError> load_graph(const config::BaselineConfig& b,
                                              std::string_view origin,
                                              const Interface& mission,
                                              const Registry& registry) {
    return guarded<Graph>(origin, [&]() -> Graph {
        require(!b.graph.empty(), "graph", "G8: the baseline has no graph");
        require(b.baseline_class == "scoreboard" || b.baseline_class == "research",
                "class",
                "G8: a baseline with a graph declares its class, scoreboard or research");
        const YAML::Node graph = YAML::Load(b.graph);
        require(graph.IsMap(), "graph", "expected a mapping");
        return b.baseline_class == "scoreboard" ? scoreboard_graph(graph, mission, registry)
                                                : research_graph(graph, mission, registry);
    });
}

std::string canonical(const Graph& g) {
    std::string s = "flow-graph 1\nmission " + g.mission + "\nclass " +
                    (g.kind == baseline_class::scoreboard ? "scoreboard" : "research") + "\n";
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

}  // namespace branes::anatomy::flow
