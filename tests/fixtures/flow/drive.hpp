#pragma once
// Driving a recipe's stack (generate.hpp) with a stream of inputs, and
// comparing it with the reference interpreter (interpreter.hpp), for the
// flow graph's tests (docs/plan/flow-testing, T1 and T3).

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/msg/time.hpp>

#include "generate.hpp"
#include "interpreter.hpp"
#include "toys.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace drive {

using branes::anatomy::Timestamp;
using toys::Num;

/// One input of a stream: which input, and its message.
struct Step {
    std::size_t input = 0;
    Num message;
};

/// A stream of `n` inputs to a recipe's inputs, in increasing time.
inline std::vector<Step> stream(const gen::Recipe& r, std::size_t n, std::uint64_t seed) {
    gen::Rng rng(seed);
    std::vector<Step> steps;
    std::int64_t t = 0;
    for (std::size_t i = 0; i < n; ++i) {
        t += static_cast<std::int64_t>(1 + rng.below(1'000'000));
        const std::size_t in = rng.below(r.inputs.size());
        steps.push_back(Step{in, Num{Timestamp{t}, r.inputs[in].tick ? 0 : rng.next()}});
    }
    return steps;
}

/// Hand the stack one input.
inline void hand(gen::Built& b, const Step& s) {
    std::visit(
        [&]<class M>(const branes::anatomy::flow::input_port<M>& p) {
            if constexpr (std::is_same_v<M, Timestamp>) {
                b.stack.input(p, s.message.t);
            } else {
                b.stack.input(p, s.message);
            }
        },
        b.inputs[s.input]);
}

inline interp::Interpreter interpreter_of(const gen::Recipe& r, const gen::Built& b) {
    std::map<std::string, interp::Val> constants;
    for (const gen::ConstantSpec& c : r.constants) {
        constants[c.name] = interp::Val{c.value.t.ns, c.value.v};
    }
    return interp::Interpreter(b.stack.graph(), constants);
}

/// The first output, after the first input, on which the stack and the
/// interpreter differ, or an empty string.
inline std::string from_interpreter(const gen::Recipe& r, std::size_t n) {
    auto built = gen::build(r);
    if (!built) {
        return "the recipe doesn't build: " + built.error().to_string();
    }
    interp::Interpreter in = interpreter_of(r, *built);
    const auto steps = stream(r, n, r.seed ^ 0x5EEDULL);
    for (std::size_t i = 0; i < steps.size(); ++i) {
        hand(*built, steps[i]);
        in.input(r.inputs[steps[i].input].name, interp::Val{steps[i].message.t.ns, steps[i].message.v});
        for (std::size_t o = 0; o < built->outputs.size(); ++o) {
            const Num& got = built->stack.output(built->outputs[o]);
            const interp::Val want = in.output(o);
            if (got.t.ns != want.t || got.v != want.v) {
                return "after input " + std::to_string(i) + " (" + r.inputs[steps[i].input].name + "), output " +
                       in.graph().outputs[o].name + ": the stack has (" + std::to_string(got.t.ns) + ", " +
                       std::to_string(got.v) + "), the interpreter (" + std::to_string(want.t) + ", " +
                       std::to_string(want.v) + ")";
            }
        }
    }
    return {};
}

}  // namespace drive
