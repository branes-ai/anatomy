// trace_compare: two edge traces, record by record (flow/trace.hpp;
// docs/plan/flow-testing, T4).
//
//   trace_compare --a <a.brtrace> --b <b.brtrace>
//
// Exits 0 when the traces are equal, 1 when they differ, naming the first
// edge they differ on, and 2 when one can't be read or the arguments are
// wrong. The traces' graphs may differ: two baselines' traces show the first
// edge on which their runs part.

#include <branes/anatomy/flow/trace.hpp>
#include <branes/anatomy/harness/args.hpp>

#include <cstdio>
#include <string>

namespace an = branes::anatomy;

namespace {

int usage(const std::string& message) {
    std::fprintf(stderr, "trace_compare: %s\n", message.c_str());
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return usage(args.error());
    }
    if (auto bad = args->only({"a", "b"})) {
        return usage(*bad);
    }
    const auto a = args->value("a");
    const auto b = args->value("b");
    if (!a || !b) {
        return usage("usage: --a <a.brtrace> --b <b.brtrace>");
    }
    const auto c = an::flow::compare_traces(*a, *b);
    if (!c) {
        return usage(c.error().path + ": " + c.error().message);
    }
    if (c->difference) {
        std::printf("trace_compare: %s and %s differ: %s\n", a->c_str(), b->c_str(), c->difference->c_str());
        return 1;
    }
    std::printf("trace_compare: %s and %s are equal, %llu records\n",
                a->c_str(),
                b->c_str(),
                static_cast<unsigned long long>(c->records));
    return 0;
}
