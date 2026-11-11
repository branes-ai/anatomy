// Must not compile (docs/plan/computational-domain, C6, W17 and W4): an edge
// between two rank-3 data of the same extents in another order. ctest builds
// this target and expects the G4 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>

namespace fl = branes::anatomy::flow;
using namespace branes::anatomy::flow;

using From = data<tensor<double, dim<3>, dim<4>, dim<5>>>;
using To = data<tensor<double, dim<3>, dim<5>, dim<4>>>;

struct Sink {};
struct SinkBinding {
    using signature = fl::signature<ports<port<"x", To, direction::in>, port<"y", To, direction::out>>>;
    struct apply : handler<"apply", ins<"x">, outs<"y">> {
        static To call(const Sink&, const To& x) {
            return x;
        }
    };
    using handlers = fl::handlers<apply>;
};
using SinkOp = registered<"Sink", Sink, SinkBinding>;

void describe(builder& g) {
    const auto x = input<From>(g, "x");
    const auto sink = instance<SinkOp>(g, "sink");
    scope r = g.region(x);
    (void)fire<"apply">(r, sink, x);
}
