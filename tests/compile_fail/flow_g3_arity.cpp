// Must not compile (docs/plan/flow-graph, 2a): a firing with a source missing.
// ctest builds this target and expects the G3 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

void describe(fl::builder& g) {
    const test::Ports p = test::ports(g);
    const test::Composites c = test::composites<test::Bodies<>>(g);
    fl::scope tick = g.region(p.tick);
    auto [estimate] = fl::fire<"estimate">(tick, c.estimator, fl::at(p.tick));
    (void)fl::fire<"step">(tick, c.controller, estimate, p.limits, fl::hold(p.marks));
}
