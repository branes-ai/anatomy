// Must not compile (docs/plan/workbench, B1, W4): an edge between data of
// an f32 vector into an f64 port. ctest builds this target and expects the G4 static_assert's message.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/data.hpp>

namespace fl = branes::anatomy::flow;
using namespace branes::anatomy::flow;

using From = data<vector<float, dim<16>>>;
using To = data<vector<double, dim<16>>>;

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
