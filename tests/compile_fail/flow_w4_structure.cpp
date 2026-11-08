// Must not compile (docs/plan/workbench, B1, W4): an edge between data of
// a dense and a symmetric matrix. ctest builds this target and expects the G4 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>

namespace fl = branes::anatomy::flow;
using namespace branes::anatomy::flow;

using From = data<matrix<double, dim<4>, dim<4>>>;
using To = data<matrix<double, dim<4>, dim<4>, structure::symmetric>>;

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
