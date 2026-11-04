// The edge trace's tests (flow/trace.hpp; docs/plan/flow-testing, T4): a
// stack's observer is handed every firing, with the value the reference
// interpreter gives each of its edges, and observing doesn't change what the
// stack writes; a trace file holds every edge's value, or, for a type with no
// codec, the edge without it; and compare_traces() names the first edge two
// traces differ on, or the trace it can't read.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/trace.hpp>

#include "../fixtures/flow/drive.hpp"
#include "../fixtures/flow/generate.hpp"
#include "../fixtures/flow/interpreter.hpp"
#include "../fixtures/flow/toys.hpp"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <system_error>
#include <typeindex>
#include <variant>
#include <vector>

namespace fl = branes::anatomy::flow;
using branes::anatomy::Timestamp;
using toys::Num;
using toys::Two;

namespace {

/// A temporary directory, removed when it goes.
class TempDir {
public:
    TempDir() {
        std::random_device rd;
        path_ = std::filesystem::temp_directory_path() /
                ("anatomy_flow_trace_" + std::to_string(rd()) + "_" + std::to_string(rd()));
        std::filesystem::create_directories(path_);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    std::filesystem::path path_;
};

fl::trace_codecs toy_codecs(bool records) {
    fl::trace_codecs c;
    c.add<Num>([](const Num& x) { return toys::encode(x); });
    if (records) {
        c.add<Two>([](const Two& x) { return toys::encode(x); });
    }
    return c;
}

/// The bytes of an interpreter's value, as the toys' codec encodes the message.
std::vector<std::byte> encoded(const interp::Value& v) {
    if (const auto* x = std::get_if<interp::Val>(&v)) {
        return toys::encode(Num{Timestamp{x->t}, x->v});
    }
    const auto& r = std::get<interp::Rec>(v);
    return toys::encode(Two{Num{Timestamp{r.a.t}, r.a.v}, Num{Timestamp{r.b.t}, r.b.v}});
}

/// An edge an observer was handed: where, when, and the bytes of its message.
struct Seen {
    std::string region;
    std::size_t firing = 0;
    std::string port;
    Timestamp t;
    std::vector<std::byte> bytes;
};

std::vector<std::byte> bytes_of(std::type_index type, const void* m) {
    if (type == std::type_index(typeid(Num))) {
        return toys::encode(*static_cast<const Num*>(m));
    }
    return toys::encode(*static_cast<const Two*>(m));
}

/// Run recipe r, writing a trace at `path` (if given) and checking each edge
/// the observer is handed against the interpreter, and its outputs against a
/// stack that isn't observed. Returns the edges seen, or fails.
std::vector<Seen> run_observed(const gen::Recipe& r, std::size_t n, const std::filesystem::path* path, bool records) {
    auto observed = gen::build(r);
    auto plain = gen::build(r);
    REQUIRE(observed);
    REQUIRE(plain);
    interp::Interpreter in = drive::interpreter_of(r, *observed);
    std::optional<fl::trace_writer> w;
    if (path != nullptr) {
        auto opened = fl::trace_writer::open(*path, observed->stack.graph(), toy_codecs(records));
        REQUIRE(opened);
        w.emplace(std::move(opened).value());
    }
    std::vector<Seen> seen;
    std::vector<Seen> step;
    const fl::Graph& g = observed->stack.graph();
    observed->stack.observe([&](const fl::fired& f) {
        for (std::size_t k = 0; k < f.ports.size(); ++k) {
            step.push_back(
                Seen{g.regions[f.region].trigger, f.firing, f.ports[k], f.t, bytes_of(f.types[k], f.messages[k])});
        }
        if (w) {
            w->write(f);
        }
    });
    const auto steps = drive::stream(r, n, r.seed ^ 0x7ACEULL);
    for (const drive::Step& s : steps) {
        step.clear();
        drive::hand(*observed, s);
        drive::hand(*plain, s);
        in.input(r.inputs[s.input].name, interp::Val{s.message.t.ns, s.message.v});
        // Every edge the region wrote, with the interpreter's value of it, and the input's time.
        for (const Seen& e : step) {
            const auto want = in.edge(e.region, e.firing, e.port);
            if (!want || encoded(*want) != e.bytes || e.t != s.message.t) {
                INFO(gen::to_string(r));
                FAIL("the observer's " + e.region + "[" + std::to_string(e.firing) + "]." + e.port +
                     " isn't the interpreter's, at t=" + std::to_string(s.message.t.ns));
            }
        }
        for (std::size_t o = 0; o < observed->outputs.size(); ++o) {
            const Num& x = observed->stack.output(observed->outputs[o]);
            const Num& y = plain->stack.output(plain->outputs[o]);
            if (x.t != y.t || x.v != y.v) {
                INFO(gen::to_string(r));
                FAIL("observing changes output " + std::to_string(o));
            }
        }
        seen.insert(seen.end(), step.begin(), step.end());
    }
    if (w) {
        REQUIRE(w->close());
    }
    return seen;
}

/// Write recipe r's trace of n inputs at `path`.
void write_trace(const gen::Recipe& r, std::size_t n, const std::filesystem::path& path) {
    (void)run_observed(r, n, &path, true);
}

}  // namespace

TEST_CASE("flow trace: the observer is handed every edge, with the interpreter's value", "[flow][trace]") {
    std::size_t edges = 0;
    for (std::uint64_t seed = 1; seed <= 200; ++seed) {
        edges += run_observed(gen::generate(seed), 300, nullptr, true).size();
    }
    INFO(edges << " edges");
    CHECK(edges > 0);
}

TEST_CASE("flow trace: a trace file holds every edge and its value", "[flow][trace]") {
    TempDir dir;
    std::size_t uncoded = 0;
    for (std::uint64_t seed = 1; seed <= 40; ++seed) {
        const gen::Recipe r = gen::generate(seed);
        for (const bool records : {true, false}) {
            const auto path = dir.path() / ("t" + std::to_string(seed) + (records ? "" : "_num") + ".brtrace");
            const std::vector<Seen> seen = run_observed(r, 200, &path, records);
            auto reader = fl::trace_reader::open(path);
            REQUIRE(reader);
            CHECK(reader->header().mission == "toys");
            CHECK(reader->header().graph_hash == gen::build(r)->stack.hash());
            for (std::size_t i = 0;; ++i) {
                auto next = reader->next();
                REQUIRE(next);
                if (!*next) {
                    CHECK(i == seen.size());
                    break;
                }
                REQUIRE(i < seen.size());
                const fl::trace_record& rec = **next;
                CHECK(rec.region == seen[i].region);
                CHECK(rec.firing == seen[i].firing);
                CHECK(rec.port == seen[i].port);
                CHECK(rec.t == seen[i].t);
                // Without a codec for records, Rec's port keeps its edge and its time, not its value.
                const bool coded = records || seen[i].bytes.size() == 16;
                if (coded) {
                    CHECK(rec.payload == seen[i].bytes);
                } else {
                    CHECK(!rec.payload);
                    ++uncoded;
                }
            }
        }
    }
    CHECK(uncoded > 0);
}

/// One unary firing at in0, of constant c0 = value: the traces compare_traces() is tested on.
gen::Recipe one_firing(std::uint64_t value) {
    gen::Recipe r;
    r.seed = 1;
    r.inputs = {{"in0", false, true}};
    r.nodes = {{"n0", gen::Kind::unary, 1, false}};
    r.constants = {{"c0", Num{Timestamp{5}, value}}};
    r.firings = {{0, 0, 0, {gen::SourceSpec{gen::SourceSpec::From::constant, 0, 0, -1, false}}}};
    return r;
}

TEST_CASE("flow trace: compare_traces names the first edge two traces differ on", "[flow][trace]") {
    TempDir dir;
    const auto at = [&](const char* name) { return dir.path() / name; };
    write_trace(one_firing(9), 10, at("a.brtrace"));
    write_trace(one_firing(9), 10, at("same.brtrace"));
    write_trace(one_firing(9), 6, at("short.brtrace"));
    write_trace(one_firing(8), 10, at("value.brtrace"));
    gen::Recipe renamed = one_firing(9);
    renamed.nodes[0].name = "renamed";
    write_trace(renamed, 10, at("edge.brtrace"));

    const auto equal = fl::compare_traces(at("a.brtrace"), at("same.brtrace"));
    REQUIRE(equal);
    CHECK(!equal->difference);
    CHECK(equal->records == 10);

    const auto value = fl::compare_traces(at("a.brtrace"), at("value.brtrace"));
    REQUIRE(value);
    REQUIRE(value->difference);
    CHECK(value->difference->find("record 0: in0[0].y (n0.apply) at t=") == 0);
    CHECK(value->difference->find(" ns has another value") != std::string::npos);

    const auto edge = fl::compare_traces(at("a.brtrace"), at("edge.brtrace"));
    REQUIRE(edge);
    REQUIRE(edge->difference);
    CHECK(*edge->difference == "record 0: a's is in0[0].y (n0.apply), b's in0[0].y (renamed.apply)");

    const auto longer = fl::compare_traces(at("a.brtrace"), at("short.brtrace"));
    REQUIRE(longer);
    REQUIRE(longer->difference);
    CHECK(longer->records == 6);
    CHECK(*longer->difference == "record 6: a has more records, from in0[0].y (n0.apply)");

    const auto none = fl::compare_traces(at("a.brtrace"), at("none.brtrace"));
    REQUIRE(!none);
    CHECK(none.error().message == "cannot open the trace file");

    {
        std::ofstream junk(at("junk.brtrace"), std::ios::binary);
        junk << "BRLOG and then some more bytes";
    }
    const auto junk = fl::compare_traces(at("junk.brtrace"), at("a.brtrace"));
    REQUIRE(!junk);
    CHECK(junk.error().message == "not a trace (bad magic)");

    // A trace cut inside its last record.
    std::filesystem::copy_file(at("a.brtrace"), at("cut.brtrace"));
    std::filesystem::resize_file(at("cut.brtrace"), std::filesystem::file_size(at("cut.brtrace")) - 3);
    const auto cut = fl::compare_traces(at("cut.brtrace"), at("a.brtrace"));
    REQUIRE(!cut);
    CHECK(cut.error().message == "truncated record");
}

TEST_CASE("flow trace: a record's time differs when its input's does", "[flow][trace]") {
    // The second trace's input comes a nanosecond later.
    const gen::Recipe r = one_firing(9);
    TempDir dir;
    for (const std::int64_t shift : {0, 1}) {
        auto b = gen::build(r);
        REQUIRE(b);
        auto w = fl::trace_writer::open(dir.path() / ("t" + std::to_string(shift)), b->stack.graph(), toy_codecs(true));
        REQUIRE(w);
        b->stack.observe([&](const fl::fired& f) { w->write(f); });
        drive::hand(*b, drive::Step{0, Num{Timestamp{100 + shift}, 1}});
        REQUIRE(w->close());
    }
    const auto c = fl::compare_traces(dir.path() / "t0", dir.path() / "t1");
    REQUIRE(c);
    REQUIRE(c->difference);
    CHECK(*c->difference == "record 0: in0[0].y (n0.apply) is at t=100 ns in a, at t=101 ns in b");
}
