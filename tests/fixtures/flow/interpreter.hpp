#pragma once
// The reference interpreter of the flow graph's differential tests
// (docs/plan/flow-testing, T1): it runs a checked flow::Graph directly,
// for the toy operators only. It is the oracle the CPU stack is compared
// with, so it shares none of the stack's machinery: it keeps every value in
// a map, finds each operator by the name the Graph holds, and implements the
// toy operators again from the plan's table, without toys.hpp.
//
// What it is given is property P2: the Graph, and the constants' values,
// which the Graph doesn't hold. A region's trigger is its input's name, as
// it is for an input declared with no trigger of its own: the Graph names
// its regions' triggers, not its inputs.

#include <branes/anatomy/flow/graph.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

namespace interp {

namespace fl = branes::anatomy::flow;

/// A value: its time in nanoseconds, and its number.
struct Val {
    std::int64_t t = 0;
    std::uint64_t v = 0;

    friend bool operator==(const Val&, const Val&) = default;
};

/// A record of two values, fields "a" and "b".
struct Rec {
    Val a;
    Val b;
};

using Value = std::variant<Val, Rec>;

[[nodiscard]] constexpr std::uint64_t mix(std::uint64_t x, std::uint64_t salt) noexcept {
    std::uint64_t z = x ^ (salt * 0x9E3779B97F4A7C15ULL);
    z += 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

class Interpreter {
public:
    /// Pre: g is a checked graph of toy operators; `constants` holds the value of each of its constants.
    Interpreter(fl::Graph g, std::map<std::string, Val> constants)
        : g_(std::move(g)), constants_(std::move(constants)) {}

    /// Input `name`'s message, and the run of its region, if it has one.
    void input(const std::string& name, Val m) {
        inputs_[name] = m;
        for (std::size_t r = 0; r < g_.regions.size(); ++r) {
            if (g_.regions[r].trigger == name) {
                run(g_.regions[r]);
            }
        }
    }

    /// Output i's value, as its region last wrote it.
    [[nodiscard]] Val output(std::size_t i) const {
        return as_val(resolve(g_.outputs.at(i).from));
    }

    [[nodiscard]] const fl::Graph& graph() const noexcept {
        return g_;
    }

    /// The value firing `firing` of region `region` last wrote to `port`, or none if it hasn't.
    [[nodiscard]] std::optional<Value>
    edge(const std::string& region, std::size_t firing, const std::string& port) const {
        const auto it = values_.find(Key{region, firing, port});
        return it == values_.end() ? std::nullopt : std::optional<Value>(it->second);
    }

private:
    using Key = std::tuple<std::string, std::size_t, std::string>;  // region, firing, port

    /// An operator, from its registered name: its kind, then its salt.
    struct Op {
        std::string kind;
        std::uint64_t salt = 0;
    };

    [[nodiscard]] static Op op_of(const std::string& name) {
        std::size_t i = name.size();
        while (i > 0 && name[i - 1] >= '0' && name[i - 1] <= '9') {
            --i;
        }
        if (i == name.size()) {
            throw std::logic_error("interp: no salt in operator " + name);
        }
        return Op{name.substr(0, i), std::stoull(name.substr(i))};
    }

    [[nodiscard]] const std::string& registered(const std::string& instance) const {
        for (const fl::Instance& in : g_.instances) {
            if (in.name == instance) {
                return in.op;
            }
        }
        throw std::logic_error("interp: no instance " + instance);
    }

    [[nodiscard]] static Val as_val(const Value& v) {
        if (const Val* x = std::get_if<Val>(&v)) {
            return *x;
        }
        throw std::logic_error("interp: a record where a value is read");
    }

    /// A source's value: an input's or a constant's latest, or a firing's
    /// out port's latest, value-initialized before its first write (P4).
    [[nodiscard]] Value resolve(const fl::Ref& ref) const {
        Value v;
        switch (ref.from) {
        case fl::Ref::kind::input: {
            const auto it = inputs_.find(ref.port);
            v = it == inputs_.end() ? Val{} : it->second;
            break;
        }
        case fl::Ref::kind::constant:
            v = constants_.at(ref.port);
            break;
        case fl::Ref::kind::firing: {
            const auto it = values_.find(Key{ref.region, ref.firing, ref.port});
            v = it == values_.end() ? default_of(ref) : it->second;
            break;
        }
        }
        for (const std::string& field : ref.path) {
            const Rec r = std::get<Rec>(v);
            v = field == "a" ? r.a : r.b;
        }
        return v;
    }

    /// The value-initialized message of a firing's out port: a record for Rec's.
    [[nodiscard]] Value default_of(const fl::Ref& ref) const {
        for (const fl::Region& r : g_.regions) {
            if (r.trigger == ref.region) {
                return op_of(registered(r.firings.at(ref.firing).op)).kind == "Rec" ? Value{Rec{}} : Value{Val{}};
            }
        }
        throw std::logic_error("interp: no region " + ref.region);
    }

    void run(const fl::Region& region) {
        for (std::size_t i = 0; i < region.firings.size(); ++i) {
            const fl::Firing& f = region.firings[i];
            std::vector<Val> in;
            for (const fl::Ref& ref : f.in) {
                in.push_back(as_val(resolve(ref)));
            }
            const std::int64_t t = f.at.empty() ? 0 : inputs_.at(f.at).t;
            for (auto& [port, value] : fire(f, in, t)) {
                values_[Key{region.trigger, i, port}] = value;
            }
        }
    }

    /// The toy operators, as the plan's table defines them.
    std::vector<std::pair<std::string, Value>> fire(const fl::Firing& f, const std::vector<Val>& in, std::int64_t t) {
        const Op op = op_of(registered(f.op));
        const std::uint64_t s = op.salt;
        if (op.kind == "Unary" && f.handler == "apply") {
            return {{"y", Val{in[0].t, mix(in[0].v, s)}}};
        }
        if (op.kind == "Binary" && f.handler == "apply") {
            return {{"z", Val{std::max(in[0].t, in[1].t), mix(in[0].v * 3 + in[1].v, s)}}};
        }
        if (op.kind == "Counter") {
            std::uint64_t& sum = sums_[f.op];
            if (f.handler == "add") {
                sum = mix(sum + in[0].v, s);
                return {};
            }
            if (f.handler == "read") {
                return {{"y", Val{t, sum}}};
            }
            if (f.handler == "peek") {
                return {{"y", Val{0, sum}}};
            }
        }
        if (op.kind == "Clock" && f.handler == "at") {
            return {{"y", Val{t, mix(static_cast<std::uint64_t>(t), s)}}};
        }
        if (op.kind == "Pair" && f.handler == "apply") {
            return {{"y", Val{in[0].t, mix(in[0].v, s)}}, {"z", Val{in[0].t, mix(in[0].v, s + 100)}}};
        }
        if (op.kind == "Rec" && f.handler == "apply") {
            return {{"r", Rec{Val{in[0].t, mix(in[0].v, s + 200)}, Val{in[0].t, mix(in[0].v, s + 300)}}}};
        }
        throw std::logic_error("interp: no toy " + op.kind + "." + f.handler);
    }

    fl::Graph g_;
    std::map<std::string, Val> constants_;
    std::map<std::string, Val> inputs_;
    std::map<Key, Value> values_;
    std::map<std::string, std::uint64_t> sums_;  ///< per Counter instance
};

}  // namespace interp
