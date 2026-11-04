// write_traces <dir>: the traces trace_compare_test.cmake compares, each of
// one unary firing of a constant, at in0, for 4 inputs: a.brtrace; value.brtrace,
// the constant changed; short.brtrace, 3 inputs.

#include <branes/anatomy/flow/trace.hpp>

#include "../fixtures/flow/generate.hpp"
#include "../fixtures/flow/toys.hpp"

#include <cstdint>
#include <cstdio>
#include <filesystem>

namespace fl = branes::anatomy::flow;
using branes::anatomy::Timestamp;
using toys::Num;

namespace {

bool write(const std::filesystem::path& p, std::uint64_t value, int inputs) {
    gen::Recipe r;
    r.inputs = {{"in0", false, true}};
    r.nodes = {{"n0", gen::Kind::unary, 1, false}};
    r.constants = {{"c0", Num{Timestamp{5}, value}}};
    r.firings = {{0, 0, 0, {gen::SourceSpec{gen::SourceSpec::From::constant, 0, 0, -1, false}}}};
    auto b = gen::build(r);
    if (!b) {
        return false;
    }
    fl::trace_codecs codecs;
    codecs.add<Num>([](const Num& x) { return toys::encode(x); });
    auto w = fl::trace_writer::open(p, b->stack.graph(), codecs);
    if (!w) {
        return false;
    }
    b->stack.observe([&](const fl::fired& f) { w->write(f); });
    const auto& in0 = std::get<fl::input_port<Num>>(b->inputs[0]);
    for (int i = 0; i < inputs; ++i) {
        b->stack.input(in0, Num{Timestamp{10 * (i + 1)}, 1});
    }
    return static_cast<bool>(w->close());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: write_traces <dir>\n");
        return 2;
    }
    const std::filesystem::path dir = argv[1];
    if (!write(dir / "a.brtrace", 9, 4) || !write(dir / "value.brtrace", 8, 4) || !write(dir / "short.brtrace", 9, 3)) {
        std::fprintf(stderr, "write_traces: cannot write the traces in %s\n", dir.string().c_str());
        return 1;
    }
    return 0;
}
