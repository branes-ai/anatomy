// The run-time half of the computational domain (flow/compute.hpp): the
// enumeration of a system's points, its dependence graph, the checks K3–K8,
// the operation counts, the depth, and the text form (docs/plan/computational-domain, C1).

#include <branes/anatomy/flow/compute.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace branes::anatomy::flow::cd {

std::string_view to_string(op o) noexcept {
    switch (o) {
    case op::iconst:
        return "const";
    case op::index:
        return "index";
    case op::param:
        return "param";
    case op::iadd:
        return "+";
    case op::isub:
        return "-";
    case op::imul:
        return "*";
    case op::idiv:
        return "/";
    case op::imod:
        return "%";
    case op::ixor:
        return "xor";
    case op::ishl:
        return "<<";
    case op::ishr:
        return ">>";
    case op::lit:
        return "literal";
    case op::read:
        return "read";
    case op::time:
        return "time";
    case op::ival:
        return "value";
    case op::cast:
        return "cast";
    case op::add:
        return "add";
    case op::sub:
        return "sub";
    case op::mul:
        return "mul";
    case op::div:
        return "div";
    case op::neg:
        return "neg";
    case op::sqrt:
        return "sqrt";
    case op::tanh:
        return "tanh";
    case op::exp:
        return "exp";
    case op::sin:
        return "sin";
    case op::cos:
        return "cos";
    case op::abs:
        return "abs";
    case op::conj:
        return "conj";
    case op::real:
        return "real";
    case op::imag:
        return "imag";
    case op::cabs:
        return "cabs";
    case op::sel_lt:
        return "select_lt";
    case op::sel_le:
        return "select_le";
    }
    return "?";
}

std::string_view to_string(reason r) noexcept {
    switch (r) {
    case reason::none:
        return "none";
    case reason::empty_name:
        return "a name is empty";
    case reason::duplicate_name:
        return "a name is declared twice";
    case reason::unknown_port:
        return "names no port or state of the binding";
    case reason::not_readable:
        return "reads what the handler can't read";
    case reason::not_writable:
        return "writes what the handler can't write";
    case reason::arity:
        return "has a number of indices other than its target's rank";
    case reason::type:
        return "mixes element types, or gives a target a value of another type";
    case reason::stage:
        return "uses an index or a variable of another stage";
    case reason::param_target:
        return "binds a parameter to no port or state";
    case reason::param_dim:
        return "binds a parameter to an extent its target hasn't";
    case reason::param_count:
        return "binds a count to an extent that isn't bounded";
    }
    return "?";
}

std::string_view to_string(shape s) noexcept {
    switch (s) {
    case shape::uniform:
        return "uniform";
    case shape::affine:
        return "affine";
    case shape::non_affine:
        return "non-affine";
    }
    return "?";
}

std::uint64_t report::count(op o) const noexcept {
    std::uint64_t n = 0;
    for (const op_count& c : counts) {
        n += c.o == o ? c.n : 0;
    }
    return n;
}

std::uint64_t report::count(op o, etype t) const noexcept {
    std::uint64_t n = 0;
    for (const op_count& c : counts) {
        n += (c.o == o && c.t == t) ? c.n : 0;
    }
    return n;
}

bool report::has(std::string_view id) const noexcept {
    return std::any_of(problems.begin(), problems.end(), [&](const problem& p) { return p.id == id; });
}

namespace {

/// The position of index i among its stage's indices.
int position_of(const system& s, std::uint16_t i) {
    int k = 0;
    for (std::uint16_t j = 0; j < i; ++j) {
        k += s.indices[j].stage == s.indices[i].stage ? 1 : 0;
    }
    return k;
}

/// The indices of a stage, in order.
std::vector<std::uint16_t> indices_of(const system& s, int stage) {
    std::vector<std::uint16_t> out;
    for (std::uint16_t i = 0; i < s.n_indices; ++i) {
        if (s.indices[i].stage == stage) {
            out.push_back(i);
        }
    }
    return out;
}

bool holds(const system& s, std::uint16_t c, const std::int64_t* point, const std::vector<std::int64_t>& params) {
    const constraint& k = s.constraints[c];
    const std::int64_t a = eval_ix(s, k.lhs, point, params);
    const std::int64_t b = eval_ix(s, k.rhs, point, params);
    switch (k.c) {
    case cmp::eq:
        return a == b;
    case cmp::lt:
        return a < b;
    case cmp::le:
        return a <= b;
    }
    return false;
}

/// The index a pin constraint pins, or -1.
int pinned_index(const system& s, std::uint16_t c) {
    const constraint& k = s.constraints[c];
    if (k.c == cmp::eq && s.nodes[k.lhs].o == op::index) {
        return s.nodes[k.lhs].a;
    }
    return -1;
}

/// Calls f(point) for every point of def d's region, in lexicographic order.
template <class F>
void for_points(const system& s, const def& d, const std::vector<std::int64_t>& params, F&& f) {
    const std::vector<std::uint16_t> ids = indices_of(s, d.stage);
    std::vector<std::int64_t> point(ids.size(), 0);
    const bool init = d.kind == dkind::init;
    const region& space = s.spaces[d.stage];
    // A pin of an init's face, by index, or -1.
    std::vector<int> pin(ids.size(), -1);
    if (init) {
        for (std::uint8_t k = 0; k < d.where.n; ++k) {
            const int p = pinned_index(s, d.where.ids[k]);
            for (std::size_t q = 0; q < ids.size(); ++q) {
                if (p == ids[q]) {
                    pin[q] = d.where.ids[k];
                }
            }
        }
    }
    std::function<void(std::size_t)> rec = [&](std::size_t level) {
        if (level == ids.size()) {
            for (std::uint8_t k = 0; k < d.where.n; ++k) {
                if (!holds(s, d.where.ids[k], point.data(), params)) {
                    return;
                }
            }
            if (!init) {
                for (std::uint8_t k = 0; k < space.n; ++k) {
                    if (!holds(s, space.ids[k], point.data(), params)) {
                        return;
                    }
                }
            }
            f(point);
            return;
        }
        if (pin[level] >= 0) {
            point[level] = eval_ix(s, s.constraints[static_cast<std::size_t>(pin[level])].rhs, point.data(), params);
            rec(level + 1);
            return;
        }
        const index_decl& x = s.indices[ids[level]];
        const std::int64_t lo = eval_ix(s, x.lo, point.data(), params);
        const std::int64_t hi = eval_ix(s, x.hi, point.data(), params);
        for (std::int64_t v = lo; v < hi; ++v) {
            point[level] = v;
            rec(level + 1);
        }
    };
    rec(0);
}

std::string point_text(const std::int64_t* p, int rank) {
    std::string out = "(";
    for (int k = 0; k < rank; ++k) {
        out += (k ? ", " : "") + std::to_string(p[k]);
    }
    return out + ")";
}

std::string operand_text(const operand& o) {
    std::string out{o.name.view()};
    if (!o.field.empty()) {
        out += ".";
        out += o.field.view();
    }
    return out;
}

/// A local's shape, as a port_info.
port_info local_shape(const system& s, const operand& o, const std::vector<std::int64_t>& params) {
    port_info p;
    p.name = o.name;
    p.t = o.t;
    p.rank = o.rank;
    for (std::size_t k = 0; k < o.rank; ++k) {
        p.extent[k] = eval_ix(s, o.extent[k], nullptr, params);
    }
    return p;
}

/// Packs a point as a key.
struct key_hash {
    std::size_t operator()(const std::vector<std::int64_t>& v) const noexcept {
        std::size_t h = 1469598103934665603ULL;
        for (const std::int64_t x : v) {
            h ^= static_cast<std::size_t>(x) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        }
        return h;
    }
};

std::vector<std::int64_t>
bind_params(const system& s, const table& t, const std::vector<std::int64_t>& counts, std::vector<problem>& problems) {
    std::vector<std::int64_t> v(s.n_params, 0);
    std::size_t next_count = 0;
    // First the integers, the ports' extents and the counts; then the locals'
    // extents, which are expressions of those.
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const param_decl& p = s.params[i];
        const bool local = p.field.empty() && s.local_named(p.target) != nullptr;
        switch (p.kind) {
        case pkind::integer:
            v[i] = p.value;
            break;
        case pkind::extent:
            if (!local) {
                const port_info* q = t.find(p.target, p.field);
                v[i] = q ? q->extent[p.dim] : 0;
            }
            break;
        case pkind::count:
            if (next_count < counts.size()) {
                v[i] = counts[next_count];
            } else {
                problems.push_back({"K2", "no count is given for " + std::string{p.name.view()}});
            }
            ++next_count;
            break;
        }
    }
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const param_decl& p = s.params[i];
        const operand* o = p.field.empty() ? s.local_named(p.target) : nullptr;
        if (p.kind == pkind::extent && o != nullptr) {
            v[i] = eval_ix(s, o->extent[p.dim], nullptr, v);
        }
    }
    return v;
}

/// The count of a port's or state's bounded leading extent, as the system
/// knows it: its count parameter's value, or -1 if it names none.
std::int64_t count_param(const system& s, std::uint16_t operand_id, const std::vector<std::int64_t>& params) {
    const operand& o = s.operands[operand_id];
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const param_decl& p = s.params[i];
        if (p.kind == pkind::count && p.target == o.name && p.field == o.field) {
            return params[i];
        }
    }
    return -1;
}

/// Whether node n is affine in the indices: constants, indices, parameters,
/// sums and differences, and products by a constant (a product of an index by
/// a parameter is not).
bool is_affine(const system& s, std::uint16_t n, bool& has_index) {
    const node& x = s.nodes[n];
    switch (x.o) {
    case op::iconst:
    case op::param:
        return true;
    case op::index:
        has_index = true;
        return true;
    case op::iadd:
    case op::isub:
        return is_affine(s, x.a, has_index) && is_affine(s, x.b, has_index);
    case op::imul: {
        bool ia = false;
        bool ib = false;
        const bool a = is_affine(s, x.a, ia);
        const bool b = is_affine(s, x.b, ib);
        const bool const_a = s.nodes[x.a].o == op::iconst;
        const bool const_b = s.nodes[x.b].o == op::iconst;
        has_index = has_index || ia || ib;
        return a && b && (!(ia && ib)) && ((!ia || const_b) && (!ib || const_a));
    }
    default: {
        bool ia = false;
        bool ib = false;
        const bool a = is_affine(s, x.a, ia);
        const bool b = is_affine(s, x.b, ib);
        // A function of parameters and constants alone is a constant.
        if (!ia && !ib) {
            return a && b;
        }
        has_index = true;
        return false;
    }
    }
}

/// Whether node n is `index + c` for index `idx`, with c constant (no parameter).
bool is_offset(const system& s, std::uint16_t n, std::uint16_t idx) {
    const node& x = s.nodes[n];
    if (x.o == op::index) {
        return x.a == idx;
    }
    auto is_const = [&](std::uint16_t m) {
        const op o = s.nodes[m].o;
        return o == op::iconst;
    };
    if (x.o == op::iadd) {
        return (is_offset(s, x.a, idx) && is_const(x.b)) || (is_const(x.a) && is_offset(s, x.b, idx));
    }
    if (x.o == op::isub) {
        return is_offset(s, x.a, idx) && is_const(x.b);
    }
    return false;
}

shape classify_read(const system& s, const node& r) {
    bool any = true;
    for (std::uint16_t k = 0; k < r.c; ++k) {
        bool has = false;
        any = any && is_affine(s, s.args[r.b + k], has);
    }
    if (!any) {
        return shape::non_affine;
    }
    const operand& o = s.operands[r.a];
    if (o.kind != okind::var) {
        return shape::affine;
    }
    const std::vector<std::uint16_t> ids = indices_of(s, o.stage);
    if (ids.size() != r.c) {
        return shape::affine;
    }
    for (std::uint16_t k = 0; k < r.c; ++k) {
        if (!is_offset(s, s.args[r.b + k], ids[k])) {
            return shape::affine;
        }
    }
    return shape::uniform;
}

void add_problem(std::vector<problem>& out, std::string id, std::string what) {
    // Keep the first few of each id: one is enough to fix, a flood is noise.
    std::size_t same = 0;
    for (const problem& p : out) {
        if (p.id == id) {
            if (p.what == what) {
                return;
            }
            ++same;
        }
    }
    if (same < 8) {
        out.push_back({std::move(id), std::move(what)});
    }
}

}  // namespace

int stage_rank(const system& s, int stage) noexcept {
    int r = 0;
    for (std::size_t i = 0; i < s.n_indices; ++i) {
        r += s.indices[i].stage == stage ? 1 : 0;
    }
    return r;
}

std::int64_t
eval_ix(const system& s, std::uint16_t n, const std::int64_t* point, const std::vector<std::int64_t>& params) noexcept {
    const node& x = s.nodes[n];
    switch (x.o) {
    case op::iconst:
        return x.k;
    case op::index:
        return point ? point[position_of(s, x.a)] : 0;
    case op::param:
        return x.a < params.size() ? params[x.a] : 0;
    default:
        break;
    }
    const std::int64_t a = eval_ix(s, x.a, point, params);
    const std::int64_t b = eval_ix(s, x.b, point, params);
    switch (x.o) {
    case op::iadd:
        return a + b;
    case op::isub:
        return a - b;
    case op::imul:
        return a * b;
    case op::idiv:
        return b == 0 ? 0 : a / b;
    case op::imod:
        return b == 0 ? 0 : a % b;
    case op::ixor:
        return a ^ b;
    case op::ishl:
        return (b < 0 || b > 62) ? 0 : a * (std::int64_t{1} << b);
    case op::ishr:
        return (b < 0 || b > 62) ? 0 : a >> b;
    default:
        return 0;
    }
}

std::int64_t element_of(const port_info& p, std::int64_t count, const std::int64_t* idx) noexcept {
    for (std::size_t k = 0; k < p.rank; ++k) {
        if (idx[k] < 0 || idx[k] >= p.extent[k]) {
            return -1;
        }
    }
    if (p.rank >= 1 && p.bounded && idx[0] >= count) {
        return -1;
    }
    if (p.rank == 2 && p.s != structure::dense) {
        const std::int64_t i = idx[0];
        const std::int64_t j = idx[1];
        const std::int64_t n = p.extent[1];
        switch (p.s) {
        case structure::lower_triangular:
            return j <= i ? i * (i + 1) / 2 + j : -1;
        case structure::upper_triangular:
            if (i > j) {
                return -1;
            }
            [[fallthrough]];
        case structure::symmetric: {
            const std::int64_t r = i < j ? i : j;
            const std::int64_t c = i < j ? j : i;
            return r * (2 * n - r + 1) / 2 + (c - r);
        }
        case structure::diagonal:
            return i == j ? i : -1;
        default:
            break;
        }
    }
    std::int64_t e = 0;
    for (std::size_t k = 0; k < p.rank; ++k) {
        e = e * p.extent[k] + idx[k];
    }
    return e;
}

std::int64_t stored_of(const port_info& p) noexcept {
    if (p.rank == 2 && p.s != structure::dense) {
        const std::int64_t n = p.extent[0];
        return p.s == structure::diagonal ? n : n * (n + 1) / 2;
    }
    std::int64_t e = 1;
    for (std::size_t k = 0; k < p.rank; ++k) {
        e *= p.extent[k];
    }
    return e;
}

std::vector<std::uint16_t> postorder(const system& s, std::uint16_t root) {
    std::vector<std::uint16_t> out;
    std::vector<bool> seen(s.n_nodes, false);
    std::function<void(std::uint16_t)> rec = [&](std::uint16_t n) {
        if (seen[n] || is_index_op(s.nodes[n].o)) {
            return;
        }
        seen[n] = true;
        const node& x = s.nodes[n];
        switch (x.o) {
        case op::lit:
        case op::read:
        case op::time:
        case op::ival:
            break;
        case op::cast:
        case op::neg:
        case op::sqrt:
        case op::tanh:
        case op::exp:
        case op::sin:
        case op::cos:
        case op::abs:
        case op::conj:
        case op::real:
        case op::imag:
        case op::cabs:
            rec(x.a);
            break;
        case op::add:
        case op::sub:
        case op::mul:
        case op::div:
            rec(x.a);
            rec(x.b);
            break;
        case op::sel_lt:
        case op::sel_le:
            rec(x.a);
            rec(x.b);
            rec(x.c);
            rec(x.d);
            break;
        default:
            break;
        }
        out.push_back(n);
    };
    rec(root);
    return out;
}

dgraph build_graph(const system& s, const table& t, const std::vector<std::int64_t>& counts) {
    dgraph g;
    g.params = bind_params(s, t, counts, g.problems);
    const std::vector<std::int64_t>& params = g.params;

    // The count each count_decl sets, and each written operand's shape.
    g.counts.assign(s.n_counts, 0);
    for (std::size_t i = 0; i < s.n_counts; ++i) {
        g.counts[i] = eval_ix(s, s.counts[i].expr, nullptr, params);
    }
    auto declared_count = [&](std::uint16_t operand_id) -> std::int64_t {
        for (std::size_t i = 0; i < s.n_counts; ++i) {
            if (s.counts[i].target == operand_id) {
                return g.counts[i];
            }
        }
        return -1;
    };
    std::vector<port_info> shape(s.n_operands);
    std::vector<bool> has_shape(s.n_operands, false);
    for (std::uint16_t i = 0; i < s.n_operands; ++i) {
        const operand& o = s.operands[i];
        if (o.kind == okind::local) {
            shape[i] = local_shape(s, o, params);
            has_shape[i] = true;
        } else if (o.kind == okind::port || o.kind == okind::state) {
            if (const port_info* p = t.find(o.name, o.field)) {
                shape[i] = *p;
                has_shape[i] = true;
            }
        }
    }

    // Pass 1: the definitions at points.
    std::vector<std::unordered_map<std::vector<std::int64_t>, std::uint32_t, key_hash>> defined(s.n_operands);
    std::map<std::pair<std::uint16_t, std::int64_t>, std::uint32_t> written;
    std::vector<int> rank_of_def(s.n_defs, 0);
    for (std::uint16_t di = 0; di < s.n_defs; ++di) {
        const def& d = s.defs[di];
        const operand& o = s.operands[d.target];
        const int rank = stage_rank(s, d.stage);
        rank_of_def[di] = rank;
        for_points(s, d, params, [&](const std::vector<std::int64_t>& point) {
            dgraph::dnode n;
            n.def = di;
            n.point = static_cast<std::uint32_t>(g.coords.size());
            g.coords.insert(g.coords.end(), point.begin(), point.end());
            const auto id = static_cast<std::uint32_t>(g.nodes.size());
            if (d.kind == dkind::write) {
                std::array<std::int64_t, max_rank> idx{};
                for (std::uint16_t k = 0; k < d.nargs; ++k) {
                    idx[k] = eval_ix(s, s.args[d.args + k], point.data(), params);
                }
                if (has_shape[d.target]) {
                    const port_info& p = shape[d.target];
                    std::int64_t count = p.extent[0];
                    if (p.bounded) {
                        const std::int64_t c = declared_count(d.target);
                        count = c >= 0 ? c : p.extent[0];
                    }
                    n.element = element_of(p, count, idx.data());
                }
                if (n.element < 0) {
                    add_problem(g.problems,
                                "K5",
                                operand_text(o) + " is written outside it, at " + point_text(point.data(), rank));
                } else {
                    auto [it, fresh] = written.try_emplace({d.target, n.element}, id);
                    if (!fresh) {
                        add_problem(g.problems,
                                    "K5",
                                    operand_text(o) + "'s element " + std::to_string(n.element) +
                                        " is written twice, the second time at " + point_text(point.data(), rank));
                    }
                }
            } else {
                auto [it, fresh] = defined[d.target].try_emplace(point, id);
                if (!fresh) {
                    add_problem(
                        g.problems, "K5", operand_text(o) + " is defined twice at " + point_text(point.data(), rank));
                }
            }
            g.nodes.push_back(n);
        });
    }

    // Pass 2: each definition's reads.
    std::vector<std::vector<std::uint16_t>> order(s.n_defs);
    for (std::uint16_t di = 0; di < s.n_defs; ++di) {
        order[di] = postorder(s, s.defs[di].value);
    }
    for (std::uint32_t ni = 0; ni < g.nodes.size(); ++ni) {
        dgraph::dnode& n = g.nodes[ni];
        const def& d = s.defs[n.def];
        const std::int64_t* point = g.coords.data() + n.point;
        const int rank = rank_of_def[n.def];
        n.src = static_cast<std::uint32_t>(g.srcs.size());
        for (const std::uint16_t r : order[n.def]) {
            const node& x = s.nodes[r];
            if (x.o != op::read) {
                continue;
            }
            const operand& o = s.operands[x.a];
            std::vector<std::int64_t> idx(x.c);
            for (std::uint16_t k = 0; k < x.c; ++k) {
                idx[k] = eval_ix(s, s.args[x.b + k], point, params);
            }
            const std::string at = " at " + point_text(point, rank);
            switch (o.kind) {
            case okind::var: {
                const auto it = defined[x.a].find(idx);
                if (it == defined[x.a].end()) {
                    add_problem(g.problems,
                                "K3",
                                operand_text(o) + point_text(idx.data(), static_cast<int>(idx.size())) +
                                    " is read but not defined," + at);
                    g.srcs.push_back(ni);  // a placeholder; the graph isn't evaluated with problems
                } else {
                    g.srcs.push_back(it->second);
                }
                break;
            }
            case okind::local: {
                const std::int64_t e = element_of(shape[x.a], shape[x.a].extent[0], idx.data());
                if (e < 0) {
                    add_problem(g.problems, "K4", operand_text(o) + " is read outside it," + at);
                    g.srcs.push_back(ni);
                    break;
                }
                const auto it = written.find({x.a, e});
                if (it == written.end()) {
                    add_problem(g.problems,
                                "K3",
                                "local " + operand_text(o) + "'s element " + std::to_string(e) +
                                    " is read but not written," + at);
                    g.srcs.push_back(ni);
                } else {
                    g.srcs.push_back(it->second);
                }
                break;
            }
            case okind::port:
            case okind::state: {
                if (!has_shape[x.a]) {
                    break;
                }
                const port_info& p = shape[x.a];
                std::int64_t count = p.extent[0];
                if (p.bounded) {
                    count = count_param(s, x.a, params);
                    if (count < 0) {
                        add_problem(g.problems,
                                    "K4",
                                    operand_text(o) +
                                        " has a bounded extent whose count the system doesn't name, so its "
                                        "reads can't be guarded");
                        break;
                    }
                }
                if (element_of(p, count, idx.data()) < 0) {
                    add_problem(g.problems,
                                "K4",
                                operand_text(o) + point_text(idx.data(), static_cast<int>(idx.size())) +
                                    " is read outside it," + at);
                }
                break;
            }
            case okind::param:
                break;
            }
        }
        n.nsrc = static_cast<std::uint32_t>(g.srcs.size()) - n.src;
        (void)d;
    }

    // K5: the outputs' coverage, and their declared counts.
    for (std::uint16_t i = 0; i < s.n_operands; ++i) {
        const operand& o = s.operands[i];
        if (!has_shape[i] || o.kind == okind::var || o.kind == okind::param) {
            continue;
        }
        const port_info& p = shape[i];
        bool writes = false;
        for (std::uint16_t di = 0; di < s.n_defs; ++di) {
            writes = writes || (s.defs[di].kind == dkind::write && s.defs[di].target == i);
        }
        const bool must = o.kind == okind::local || p.dir == access::out;
        if (!writes && !(must && (o.kind == okind::local || p.written))) {
            continue;
        }
        std::int64_t count = p.extent[0];
        if (p.bounded && p.rank >= 1) {
            const std::int64_t c = declared_count(i);
            if (c < 0) {
                add_problem(g.problems, "K5", operand_text(o) + " is bounded and written, but declares no count");
                continue;
            }
            if (c > p.extent[0]) {
                add_problem(g.problems,
                            "K5",
                            operand_text(o) + "'s declared count " + std::to_string(c) + " is outside [0, " +
                                std::to_string(p.extent[0]) + "]");
                continue;
            }
            count = c;
        }
        // Negative counts: refused before any row is checked.
        if (count < 0) {
            add_problem(g.problems,
                        "K5",
                        operand_text(o) + "'s declared count " + std::to_string(count) + " is outside [0, " +
                            std::to_string(p.extent[0]) + "]");
            continue;
        }
        if (!must) {
            continue;  // an inout port or a state: at most once, which pass 1 checked
        }
        // Every stored element below the count is written once.
        std::vector<std::int64_t> idx(p.rank, 0);
        std::vector<bool> seen(static_cast<std::size_t>(std::max<std::int64_t>(stored_of(p), 1)), false);
        std::function<void(std::size_t)> rec = [&](std::size_t level) {
            if (level == p.rank) {
                const std::int64_t e = element_of(p, count, idx.data());
                if (e < 0 || seen[static_cast<std::size_t>(e)]) {
                    return;
                }
                seen[static_cast<std::size_t>(e)] = true;
                if (written.find({i, e}) == written.end()) {
                    add_problem(g.problems,
                                "K5",
                                operand_text(o) + point_text(idx.data(), static_cast<int>(p.rank)) +
                                    " is never written");
                }
                return;
            }
            const std::int64_t hi = (level == 0 && p.bounded) ? count : p.extent[level];
            for (std::int64_t v = 0; v < hi; ++v) {
                idx[level] = v;
                rec(level + 1);
            }
        };
        rec(0);
    }

    // K6: the dependences are acyclic.
    {
        std::vector<std::uint32_t> indegree(g.nodes.size(), 0);
        std::vector<std::vector<std::uint32_t>> succ(g.nodes.size());
        for (std::uint32_t ni = 0; ni < g.nodes.size(); ++ni) {
            const dgraph::dnode& n = g.nodes[ni];
            for (std::uint32_t k = 0; k < n.nsrc; ++k) {
                const std::uint32_t from = g.srcs[n.src + k];
                succ[from].push_back(ni);
                ++indegree[ni];
            }
        }
        std::vector<std::uint32_t> ready;
        for (std::uint32_t ni = 0; ni < g.nodes.size(); ++ni) {
            if (indegree[ni] == 0) {
                ready.push_back(ni);
            }
        }
        std::size_t done = 0;
        while (!ready.empty()) {
            const std::uint32_t n = ready.back();
            ready.pop_back();
            ++done;
            for (const std::uint32_t m : succ[n]) {
                if (--indegree[m] == 0) {
                    ready.push_back(m);
                }
            }
        }
        if (done != g.nodes.size()) {
            for (std::uint32_t ni = 0; ni < g.nodes.size(); ++ni) {
                if (indegree[ni] > 0) {
                    const dgraph::dnode& n = g.nodes[ni];
                    add_problem(g.problems,
                                "K6",
                                "the dependences have a cycle through " +
                                    operand_text(s.operands[s.defs[n.def].target]) + " at " +
                                    point_text(g.coords.data() + n.point, rank_of_def[n.def]));
                    break;
                }
            }
        }
    }
    return g;
}

report analyze(const system& s, const table& t) {
    report r;
    if (const finding f = k1(s, t); f.k != 0) {
        r.problems.push_back(
            {"K1", std::string{to_string(static_cast<reason>(f.what))} + " (at " + std::to_string(f.at) + ")"});
        return r;
    }
    if (const finding f = k2(s, t); f.k != 0) {
        r.problems.push_back(
            {"K2",
             std::string{to_string(static_cast<reason>(f.what))} + ": " + std::string{s.params[f.at].name.view()}});
        return r;
    }

    // The counts' bounds, in the order of the count parameters.
    std::vector<std::int64_t> bounds;
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const param_decl& p = s.params[i];
        if (p.kind == pkind::count) {
            const port_info* q = t.find(p.target, p.field);
            bounds.push_back(q ? q->extent[0] : 0);
        }
    }
    // Every combination of counts, or 0, 1 and the bound of each past 4096.
    std::vector<std::vector<std::int64_t>> choices(bounds.size());
    std::uint64_t combos = 1;
    for (const std::int64_t b : bounds) {
        combos *= static_cast<std::uint64_t>(b + 1);
    }
    for (std::size_t k = 0; k < bounds.size(); ++k) {
        if (combos <= 4096) {
            for (std::int64_t c = 0; c <= bounds[k]; ++c) {
                choices[k].push_back(c);
            }
        } else {
            choices[k] = {0, std::min<std::int64_t>(1, bounds[k]), bounds[k]};
        }
    }
    std::vector<std::int64_t> counts(bounds.size(), 0);
    std::function<void(std::size_t)> each = [&](std::size_t k) {
        if (k == bounds.size()) {
            dgraph g = build_graph(s, t, counts);
            for (problem& p : g.problems) {
                std::string what = p.what;
                if (!counts.empty()) {
                    what += " (counts";
                    for (const std::int64_t c : counts) {
                        what += " " + std::to_string(c);
                    }
                    what += ")";
                }
                add_problem(r.problems, p.id, std::move(what));
            }
            return;
        }
        for (const std::int64_t c : choices[k]) {
            counts[k] = c;
            each(k + 1);
        }
    };
    each(0);

    // K7: state against the state role.
    bool reads_state = false;
    bool writes_state = false;
    for (std::uint16_t i = 0; i < s.n_operands; ++i) {
        const operand& o = s.operands[i];
        if (o.kind != okind::state) {
            continue;
        }
        bool written = false;
        for (std::uint16_t di = 0; di < s.n_defs; ++di) {
            written = written || (s.defs[di].kind == dkind::write && s.defs[di].target == i);
        }
        bool read = false;
        for (std::uint16_t n = 0; n < s.n_nodes; ++n) {
            read = read || (s.nodes[n].o == op::read && s.nodes[n].a == i);
        }
        reads_state = reads_state || read;
        writes_state = writes_state || written;
        if (t.role == state_role::update && read && !written) {
            // A field read and another field of the same state written counts as written.
            bool other = false;
            for (std::uint16_t j = 0; j < s.n_operands; ++j) {
                const operand& q = s.operands[j];
                if (q.kind == okind::state && q.name == o.name) {
                    for (std::uint16_t di = 0; di < s.n_defs; ++di) {
                        other = other || (s.defs[di].kind == dkind::write && s.defs[di].target == j);
                    }
                }
            }
            if (!other) {
                add_problem(
                    r.problems, "K7", "an update handler reads state " + operand_text(o) + " but doesn't write it");
            }
        }
    }
    if (t.role == state_role::none && (reads_state || writes_state)) {
        add_problem(r.problems, "K7", "a handler whose state role is none names a state");
    }
    if (t.role == state_role::read && writes_state) {
        add_problem(r.problems, "K7", "a handler whose state role is read writes a state");
    }

    // K8: each read's class, and each stage's.
    r.reads.assign(s.n_nodes, shape::uniform);
    r.stages.assign(s.n_stages, shape::uniform);
    std::vector<bool> any_var_affine(s.n_stages, false);
    for (std::uint16_t di = 0; di < s.n_defs; ++di) {
        const def& d = s.defs[di];
        for (const std::uint16_t n : postorder(s, d.value)) {
            const node& x = s.nodes[n];
            if (x.o != op::read) {
                continue;
            }
            const shape c = classify_read(s, x);
            r.reads[n] = c;
            if (c == shape::non_affine) {
                r.stages[d.stage] = shape::non_affine;
            } else if (c == shape::affine && s.operands[x.a].kind == okind::var) {
                any_var_affine[d.stage] = true;
            }
        }
        // A write's index function is classified too: a scatter that isn't affine.
        for (std::uint16_t k = 0; k < d.nargs; ++k) {
            bool has = false;
            if (!is_affine(s, s.args[d.args + k], has)) {
                r.stages[d.stage] = shape::non_affine;
            }
        }
    }
    for (std::size_t st = 0; st < s.n_stages; ++st) {
        if (r.stages[st] != shape::non_affine && any_var_affine[st]) {
            r.stages[st] = shape::affine;
        }
    }

    // Counts and depth, at the bounds.
    if (r.problems.empty()) {
        const dgraph g = build_graph(s, t, bounds);
        r.definitions = g.nodes.size();
        std::vector<std::vector<std::uint16_t>> order(s.n_defs);
        for (std::uint16_t di = 0; di < s.n_defs; ++di) {
            order[di] = postorder(s, s.defs[di].value);
        }
        std::map<std::pair<op, etype>, std::uint64_t> tally;
        std::array<etype, max_nodes> ty{};
        (void)detail::type_nodes(s, t, ty);
        for (const dgraph::dnode& n : g.nodes) {
            for (const std::uint16_t m : order[n.def]) {
                const node& x = s.nodes[m];
                if (is_counted(x.o)) {
                    // An operation's type: its operand's, for one whose result is real.
                    etype et = ty[m];
                    if (x.o == op::real || x.o == op::imag || x.o == op::cabs) {
                        et = ty[x.a];
                    }
                    ++tally[{x.o, et}];
                }
            }
        }
        for (const auto& [k, v] : tally) {
            r.counts.push_back({k.first, k.second, v});
        }
        // Depth: the longest chain, by levels in a topological order.
        std::vector<std::uint64_t> level(g.nodes.size(), 0);
        std::vector<std::uint32_t> indegree(g.nodes.size(), 0);
        std::vector<std::vector<std::uint32_t>> succ(g.nodes.size());
        for (std::uint32_t ni = 0; ni < g.nodes.size(); ++ni) {
            for (std::uint32_t k = 0; k < g.nodes[ni].nsrc; ++k) {
                succ[g.srcs[g.nodes[ni].src + k]].push_back(ni);
                ++indegree[ni];
            }
        }
        std::vector<std::uint32_t> ready;
        for (std::uint32_t ni = 0; ni < g.nodes.size(); ++ni) {
            if (indegree[ni] == 0) {
                ready.push_back(ni);
                level[ni] = 1;
            }
        }
        while (!ready.empty()) {
            const std::uint32_t n = ready.back();
            ready.pop_back();
            r.depth = std::max(r.depth, level[n]);
            for (const std::uint32_t m : succ[n]) {
                level[m] = std::max(level[m], level[n] + 1);
                if (--indegree[m] == 0) {
                    ready.push_back(m);
                }
            }
        }
        r.width.assign(r.depth, 0);
        for (const std::uint64_t l : level) {
            if (l > 0) {
                ++r.width[l - 1];
            }
        }
    }
    return r;
}

// ── Text ────────────────────────────────────────────────────────────

namespace {

std::string ix_text(const system& s, std::uint16_t n, bool top = true) {
    const node& x = s.nodes[n];
    switch (x.o) {
    case op::iconst:
        return std::to_string(x.k);
    case op::index:
        return std::string{s.indices[x.a].name.view()};
    case op::param:
        return std::string{s.params[x.a].name.view()};
    default:
        break;
    }
    std::string out = ix_text(s, x.a, false) + " " + std::string{to_string(x.o)} + " " + ix_text(s, x.b, false);
    return top ? out : "(" + out + ")";
}

std::string cond_text(const system& s, std::uint16_t c) {
    const constraint& k = s.constraints[c];
    const char* rel = k.c == cmp::eq ? " == " : (k.c == cmp::lt ? " < " : " <= ");
    return ix_text(s, k.lhs) + rel + ix_text(s, k.rhs);
}

std::string args_text(const system& s, std::uint16_t first, std::uint16_t n) {
    std::string out = "(";
    for (std::uint16_t k = 0; k < n; ++k) {
        out += (k ? ", " : "") + ix_text(s, s.args[first + k]);
    }
    return out + ")";
}

std::string val_text(const system& s, std::uint16_t n, bool top = true) {
    const node& x = s.nodes[n];
    auto wrap = [&](const std::string& e) { return top ? e : "(" + e + ")"; };
    switch (x.o) {
    case op::lit: {
        std::ostringstream o;
        o << x.v;
        return o.str();
    }
    case op::read:
        return operand_text(s.operands[x.a]) + args_text(s, x.b, x.c);
    case op::time:
        return "time";
    case op::ival:
        return std::string{to_string(x.t)} + "(" + ix_text(s, x.a) + ")";
    case op::cast:
        return std::string{to_string(x.t)} + "(" + val_text(s, x.a) + ")";
    case op::add:
        return wrap(val_text(s, x.a, false) + " + " + val_text(s, x.b, false));
    case op::sub:
        return wrap(val_text(s, x.a, false) + " - " + val_text(s, x.b, false));
    case op::mul:
        return wrap(val_text(s, x.a, false) + " * " + val_text(s, x.b, false));
    case op::div:
        return wrap(val_text(s, x.a, false) + " / " + val_text(s, x.b, false));
    case op::neg:
        return "-" + val_text(s, x.a, false);
    case op::sel_lt:
    case op::sel_le:
        return wrap(val_text(s, x.a, false) + (x.o == op::sel_lt ? " < " : " <= ") + val_text(s, x.b, false) + " ? " +
                    val_text(s, x.c, false) + " : " + val_text(s, x.d, false));
    default:
        return std::string{to_string(x.o)} + "(" + val_text(s, x.a) + ")";
    }
}

std::string region_text(const system& s, const region& r) {
    std::string out;
    for (std::uint8_t k = 0; k < r.n; ++k) {
        out += (k ? ", " : "") + cond_text(s, r.ids[k]);
    }
    return out;
}

}  // namespace

std::string text(const system& s, const table& t) {
    std::ostringstream out;
    std::vector<problem> ignored;
    std::vector<std::int64_t> bounds;
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        if (s.params[i].kind == pkind::count) {
            const port_info* q = t.find(s.params[i].target, s.params[i].field);
            bounds.push_back(q ? q->extent[0] : 0);
        }
    }
    const std::vector<std::int64_t> params = bind_params(s, t, bounds, ignored);
    for (std::uint16_t i = 0; i < s.n_params; ++i) {
        const param_decl& p = s.params[i];
        out << "param " << p.name.view() << (p.kind == pkind::count ? " <= " : " = ") << params[i] << "\n";
    }
    for (std::uint16_t i = 0; i < s.n_operands; ++i) {
        const operand& o = s.operands[i];
        if (o.kind == okind::local) {
            out << "local " << o.name.view() << ": " << to_string(o.t) << "[";
            for (std::size_t k = 0; k < o.rank; ++k) {
                out << (k ? ", " : "") << ix_text(s, o.extent[k]);
            }
            out << "]\n";
        } else if (o.kind == okind::param) {
            out << "operator parameter " << o.name.view() << ": " << to_string(o.t) << (o.rank ? "[]" : "") << "\n";
        }
    }
    for (int st = 0; st < s.n_stages; ++st) {
        out << "stage " << st << ": (";
        bool first = true;
        for (std::uint16_t i = 0; i < s.n_indices; ++i) {
            if (s.indices[i].stage == st) {
                out << (first ? "" : ", ") << s.indices[i].name.view();
                first = false;
            }
        }
        out << ") |";
        first = true;
        for (std::uint16_t i = 0; i < s.n_indices; ++i) {
            if (s.indices[i].stage == st) {
                out << (first ? " " : ", ") << ix_text(s, s.indices[i].lo) << " <= " << s.indices[i].name.view()
                    << " < " << ix_text(s, s.indices[i].hi);
                first = false;
            }
        }
        const region& sp = s.spaces[static_cast<std::size_t>(st)];
        if (sp.n) {
            out << ", " << region_text(s, sp);
        }
        out << "\n";
        for (std::uint16_t i = 0; i < s.n_operands; ++i) {
            const operand& o = s.operands[i];
            if (o.kind == okind::var && o.stage == st) {
                out << "  var " << o.name.view() << ": " << to_string(o.t) << "\n";
            }
        }
        for (std::uint16_t di = 0; di < s.n_defs; ++di) {
            const def& d = s.defs[di];
            if (d.stage != st) {
                continue;
            }
            const operand& o = s.operands[d.target];
            out << "  ";
            switch (d.kind) {
            case dkind::eq: {
                out << o.name.view() << "(";
                bool f = true;
                for (std::uint16_t i = 0; i < s.n_indices; ++i) {
                    if (s.indices[i].stage == st) {
                        out << (f ? "" : ", ") << s.indices[i].name.view();
                        f = false;
                    }
                }
                out << ") = " << val_text(s, d.value);
                break;
            }
            case dkind::init:
                out << "init " << o.name.view() << " = " << val_text(s, d.value);
                break;
            case dkind::write:
                out << operand_text(o) << args_text(s, d.args, d.nargs) << " <- " << val_text(s, d.value);
                break;
            }
            if (d.where.n) {
                out << "  | " << region_text(s, d.where);
            }
            out << "\n";
        }
    }
    for (std::size_t i = 0; i < s.n_counts; ++i) {
        out << "count " << operand_text(s.operands[s.counts[i].target]) << " = " << ix_text(s, s.counts[i].expr)
            << "\n";
    }
    return out.str();
}

}  // namespace branes::anatomy::flow::cd
