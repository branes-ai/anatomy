// Must not compile (docs/plan/flow-graph, 2a): a projection of a value that isn't a record.
// ctest builds this target and expects field_t's static_assert's message.

#include "../fixtures/flow/test_mission.hpp"

namespace fl = branes::anatomy::flow;

void describe(fl::builder& g) {
    const test::Ports p = test::ports(g);
    (void)fl::project<"ceiling">(p.marks);
}
