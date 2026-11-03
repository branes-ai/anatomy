// Must not compile (docs/plan/flow-graph, 2a): an output written by a value of another domain.
// ctest builds this target and expects the G4 static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

void describe(fl::builder& g) {
    const test::Ports p = test::ports(g);
    const test::Composites c = test::composites<test::Bodies<>>(g);
    fl::scope tick = g.region(p.tick);
    auto [estimate] = fl::fire<"estimate">(tick, c.estimator, fl::at(p.tick));
    g.produce(p.command, estimate);
}
