// Must not compile (docs/plan/flow-graph, 2b): a projection of a message that
// names no member, which the stack could not read.
// ctest builds this target and expects the static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

void describe(fl::builder& g) {
    const test::Ports p = test::ports(g);
    (void)fl::project<"ceiling">(p.limits);
}
