// Must not compile (docs/plan/workbench, B2): an accumulate edge into a
// window port of another capacity. Two capacities are two domains, so ctest
// builds this target and expects the G4 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>

namespace fl = branes::anatomy::flow;
using namespace branes::anatomy::flow;

struct M {
    double v = 0;
};

template <>
struct branes::anatomy::flow::domain_of<M> {
    using type = scalar<double>;
};

struct Sink {};
struct SinkBinding {
    using signature = fl::signature<ports<port<"x", window<M, 4>, direction::in>, port<"y", M, direction::out>>>;
    struct apply : handler<"apply", ins<"x">, outs<"y">> {
        static M call(const Sink&, const window<M, 4>&) {
            return {};
        }
    };
    using handlers = fl::handlers<apply>;
};
using SinkOp = registered<"Sink", Sink, SinkBinding>;

void describe(builder& g) {
    const auto x = input<M>(g, "x");
    const auto t = input<M>(g, "t");
    const auto sink = instance<SinkOp>(g, "sink");
    scope r = g.region(t);
    (void)fire<"apply">(r, sink, accumulate<3>(x));
}
